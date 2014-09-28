/*
 * vizzini.c
 *
 * Copyright (c) 2013 Exar Corporation, Inc.
 *
 * ChangeLog:
 *            v.1.0 - Combined Kerenl support from 2.6.+ to 3.2
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* This version of the Linux driver source contains a number of
   abominable conditional compilation sections to manage the API
   changes between kernel versions 2.6.18, 2.6.25, and the latest
   (currently 2.6.27).  At some point we'll hand a version of this
   driver off to the mainline Linux source tree, and we'll strip all
   these sections out.  For now it makes it much easier to keep it all
   in sync while the driver is being developed. */


#define DRIVER_VERSION "v.1.1"
#define DRIVER_AUTHOR "Rob Duncan <rob.duncan@exar.com>"
#define DRIVER_DESC "USB Driver for Vizzini USB serial port"

#undef VIZZINI_IWA

#define ELITE_KERNEL


#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/errno.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <asm/uaccess.h>

#include <linux/usb/cdc.h>
#ifndef CDC_DATA_INTERFACE_TYPE
#define CDC_DATA_INTERFACE_TYPE 0x0a
#endif
#ifndef USB_RT_ACM
#define USB_RT_ACM      (USB_TYPE_CLASS | USB_RECIP_INTERFACE)
#define ACM_CTRL_DTR            0x01
#define ACM_CTRL_RTS            0x02
#define ACM_CTRL_DCD            0x01
#define ACM_CTRL_DSR            0x02
#define ACM_CTRL_BRK            0x04
#define ACM_CTRL_RI             0x08
#define ACM_CTRL_FRAMING        0x10
#define ACM_CTRL_PARITY         0x20
#define ACM_CTRL_OVERRUN        0x40
#endif

#include "linux/version.h"

#include "vizzini.h"


#define N_IN_URB    4
#define N_OUT_URB   4
#define IN_BUFLEN   4096

static int debug;


/* -------------------------------------------------------------------------- */

#if defined(RHEL_RELEASE_CODE)
#if RHEL_RELEASE_CODE < RHEL_RELEASE_VERSION(5, 2)
#define true 1

static inline int usb_endpoint_dir_in(const struct usb_endpoint_descriptor *epd)
{
        return ((epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN);
}

static inline int usb_endpoint_dir_out(const struct usb_endpoint_descriptor *epd)
{
        return ((epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT);
}

static inline int usb_endpoint_xfer_bulk(const struct usb_endpoint_descriptor *epd)
{
        return ((epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
                USB_ENDPOINT_XFER_BULK);
}

static inline int usb_endpoint_is_bulk_in(const struct usb_endpoint_descriptor *epd)
{
        return (usb_endpoint_xfer_bulk(epd) && usb_endpoint_dir_in(epd));
}

static inline int usb_endpoint_is_bulk_out(const struct usb_endpoint_descriptor *epd)
{
        return (usb_endpoint_xfer_bulk(epd) && usb_endpoint_dir_out(epd));
}
#endif
#endif

/* -------------------------------------------------------------------------- */

#include "vzioctl.h"

/* -------------------------------------------------------------------------- */

static struct usb_device_id id_table [] = {
        { USB_DEVICE(0x04e2, 0x1410) },
        { USB_DEVICE(0x04e2, 0x1412) },
        { USB_DEVICE(0x04e2, 0x1414) },
        { }
};
MODULE_DEVICE_TABLE(usb, id_table);


static void vizzini_disconnect(struct usb_interface *interface);

static struct usb_driver vizzini_driver = {
        .name          = "vizzini",
        .probe         = usb_serial_probe,
        .disconnect    = vizzini_disconnect,
        .id_table      = id_table,
#ifndef	ELITE_KERNEL
        .no_dynamic_id = 1,
#endif
};


/* -------------------------------------------------------------------------- */

struct vizzini_serial_private
{
        struct usb_interface *data_interface;
};


struct vizzini_port_private {
        spinlock_t    lock;     /* lock the structure */
        int           outstanding_urbs; /* number of out urbs in flight */

        struct urb   *in_urbs[N_IN_URB];
        char         *in_buffer[N_IN_URB];

        int           ctrlin;
        int           ctrlout;
        int           clocal;

        int           block;
        int           preciseflags; /* USB: wide mode, TTY: flags per character */
        int           trans9;   /* USB: wide mode, serial 9N1 */
        unsigned int  baud_base; /* setserial: used to hack in non-standard baud rates */
        int           have_extra_byte;
        int           extra_byte;

	int           bcd_device;

#ifdef VIZZINI_IWA
        int           iwa;
#endif
};


/* -------------------------------------------------------------------------- */

static int vizzini_rev_a(struct usb_serial_port *port)
{
        struct vizzini_port_private *portdata = usb_get_serial_port_data(port);
	return portdata->bcd_device == 0;
}




/* -------------------------------------------------------------------------- */

static int vizzini_test_mode(struct usb_serial_port *port,
                             int selector)
{
        struct usb_serial *serial = port->serial;
        int retval = usb_control_msg(serial->dev, usb_sndctrlpipe(serial->dev, 0),
                                     USB_REQ_SET_FEATURE,
                                     USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
                                     USB_DEVICE_TEST_MODE,
                                     selector << 8,
                                     NULL, 0, 5000);
        if (debug) dev_dbg(&port->dev, "vz_test_mode: selector=0x%02x\n", selector);
        return retval < 0 ? retval : 0;
}


static int vizzini_set_reg(struct usb_serial_port *port,
                           int block, int regnum, int value)
{
        struct usb_serial *serial = port->serial;
        int result;


        if (debug) dev_dbg(&port->dev, "%s 0x%02x:0x%02x = 0x%02x\n", __func__, block, regnum, value);

        result = usb_control_msg(serial->dev,                     /* usb device */
                                 usb_sndctrlpipe(serial->dev, 0), /* endpoint pipe */
                                 XR_SET_REG,                      /* request */
                                 USB_DIR_OUT | USB_TYPE_VENDOR,   /* request_type */
                                 value,                           /* request value */
                                 regnum | (block << 8),           /* index */
                                 NULL,                            /* data */
                                 0,                               /* size */
                                 5000);                           /* timeout */

        return result;
}


static int vizzini_get_reg(struct usb_serial_port *port,
                           int block, int reg, char *value)
{
        struct usb_serial *serial = port->serial;
        int result;

        result = usb_control_msg(serial->dev,                     /* usb device */
                                 usb_rcvctrlpipe(serial->dev, 0), /* endpoint pipe */
                                 XR_GETN_REG,                     /* request */
                                 USB_DIR_IN | USB_TYPE_VENDOR,    /* request_type */
                                 0,                               /* request value */
                                 reg | (block << 8),              /* index */
                                 value,                           /* data */
                                 1,                               /* size */
                                 5000);                           /* timeout */

        return result;
}


#if 0
static int vizzini_getn_reg(struct usb_serial_port *port,
                            int block, char *data, size_t size)
{
        struct usb_serial *serial = port->serial;
        int result;

        result = usb_control_msg(serial->dev,           /* usb device */
                                 usb_rcvctrlpipe(serial->dev, 0),/* endpoint pipe */
                                 XR_GETN_REG,           /* request */
                                 USB_DIR_IN | USB_TYPE_VENDOR,  /* request_type */
                                 0,             /* request value */
                                 0 | (block << 8),      /* index */
                                 data,              /* data */
                                 size,              /* size */
                                 5000);             /* timeout */

        return result;
}
#endif


static void vizzini_disable(struct usb_serial_port *port)
{
        struct vizzini_port_private *portdata = usb_get_serial_port_data(port);
        int block = portdata->block;

        vizzini_set_reg(port, block, UART_ENABLE, 0);
        vizzini_set_reg(port, URM_REG_BLOCK, URM_ENABLE_BASE + block, 0);
}


static void vizzini_enable(struct usb_serial_port *port)
{
        struct vizzini_port_private *portdata = usb_get_serial_port_data(port);
        int block = portdata->block;

        vizzini_set_reg(port, URM_REG_BLOCK, URM_ENABLE_BASE + block, URM_ENABLE_0_TX);
        vizzini_set_reg(port, block, UART_ENABLE, UART_ENABLE_TX | UART_ENABLE_RX);
        vizzini_set_reg(port, URM_REG_BLOCK, URM_ENABLE_BASE + block, URM_ENABLE_0_TX | URM_ENABLE_0_RX);
}


static void vizzini_loopback(struct usb_serial_port *port, int from)
{
	struct vizzini_port_private *portdata = usb_get_serial_port_data(port);
	int block = portdata->block;
	int lb;

	if (vizzini_rev_a(port)) {
		switch (from)
		{
		case 0: lb = UART_LOOPBACK_CTL_RX_UART3; break;
		case 1: lb = UART_LOOPBACK_CTL_RX_UART2; break;
		case 2: lb = UART_LOOPBACK_CTL_RX_UART1; break;
		case 3: lb = UART_LOOPBACK_CTL_RX_UART0; break;
		default: return;
		}
	} else {
		switch (from)
		{
		case 0: lb = UART_LOOPBACK_CTL_RX_UART0; break;
		case 1: lb = UART_LOOPBACK_CTL_RX_UART1; break;
		case 2: lb = UART_LOOPBACK_CTL_RX_UART2; break;
		case 3: lb = UART_LOOPBACK_CTL_RX_UART3; break;
		default: return;
		}
	}
	
	dev_info(&port->dev, "Internal loopback from %d\n", from);

	vizzini_disable(port);
	vizzini_set_reg(port, block, UART_LOOPBACK_CTL, UART_LOOPBACK_CTL_ENABLE | lb);
	vizzini_enable(port);
}


struct vizzini_baud_rate
{
	unsigned int tx;
	unsigned int rx0;
	unsigned int rx1;
};

static struct vizzini_baud_rate vizzini_baud_rates[] = {
	{ 0x000, 0x000, 0x000 },
	{ 0x000, 0x000, 0x000 },
	{ 0x100, 0x000, 0x100 },
	{ 0x020, 0x400, 0x020 },
	{ 0x010, 0x100, 0x010 },
	{ 0x208, 0x040, 0x208 },
	{ 0x104, 0x820, 0x108 },
	{ 0x844, 0x210, 0x884 },
	{ 0x444, 0x110, 0x444 },
	{ 0x122, 0x888, 0x224 },
	{ 0x912, 0x448, 0x924 },
	{ 0x492, 0x248, 0x492 },
	{ 0x252, 0x928, 0x292 },
	{ 0X94A, 0X4A4, 0XA52 },
	{ 0X52A, 0XAA4, 0X54A },
	{ 0XAAA, 0x954, 0X4AA },
	{ 0XAAA, 0x554, 0XAAA },
	{ 0x555, 0XAD4, 0X5AA },
	{ 0XB55, 0XAB4, 0X55A },
	{ 0X6B5, 0X5AC, 0XB56 },
	{ 0X5B5, 0XD6C, 0X6D6 },
	{ 0XB6D, 0XB6A, 0XDB6 },
	{ 0X76D, 0X6DA, 0XBB6 },
	{ 0XEDD, 0XDDA, 0X76E },
	{ 0XDDD, 0XBBA, 0XEEE },
	{ 0X7BB, 0XF7A, 0XDDE },
	{ 0XF7B, 0XEF6, 0X7DE },
	{ 0XDF7, 0XBF6, 0XF7E },
	{ 0X7F7, 0XFEE, 0XEFE },
	{ 0XFDF, 0XFBE, 0X7FE },
	{ 0XF7F, 0XEFE, 0XFFE },
	{ 0XFFF, 0XFFE, 0XFFD },
};

static int vizzini_set_baud_rate(struct usb_serial_port *port, unsigned int rate)
{
	struct vizzini_port_private *portdata = usb_get_serial_port_data(port);
	int 		block 	= portdata->block;
	unsigned int 	divisor = 48000000 / rate;
	unsigned int 	i 	= ((32 * 48000000) / rate) & 0x1f;
	unsigned int 	tx_mask = vizzini_baud_rates[i].tx;
	unsigned int 	rx_mask = (divisor & 1) ? vizzini_baud_rates[i].rx1 : vizzini_baud_rates[i].rx0;

	dev_dbg(&port->dev, "Setting baud rate to %d: i=%u div=%u tx=%03x rx=%03x\n", rate, i, divisor, tx_mask, rx_mask);

	vizzini_set_reg(port, block, UART_CLOCK_DIVISOR_0, (divisor >>  0) & 0xff);
	vizzini_set_reg(port, block, UART_CLOCK_DIVISOR_1, (divisor >>  8) & 0xff);
	vizzini_set_reg(port, block, UART_CLOCK_DIVISOR_2, (divisor >> 16) & 0xff);
	vizzini_set_reg(port, block, UART_TX_CLOCK_MASK_0, (tx_mask >>  0) & 0xff);
	vizzini_set_reg(port, block, UART_TX_CLOCK_MASK_1, (tx_mask >>  8) & 0xff);
	vizzini_set_reg(port, block, UART_RX_CLOCK_MASK_0, (rx_mask >>  0) & 0xff);
	vizzini_set_reg(port, block, UART_RX_CLOCK_MASK_1, (rx_mask >>  8) & 0xff);
	
	return -EINVAL;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
static void vizzini_set_termios(struct usb_serial_port *port,
                                struct termios *old_termios)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static void vizzini_set_termios(struct usb_serial_port *port,
                                struct ktermios *old_termios)
#else
static void vizzini_set_termios(struct tty_struct *tty_param,
                                struct usb_serial_port *port,
                                struct ktermios *old_termios)
#endif
{
        struct vizzini_port_private *portdata = usb_get_serial_port_data(port);

        unsigned int             cflag, block;
        speed_t                  rate;
        unsigned int             format_size, format_parity, format_stop, flow, gpio_mode;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
        struct tty_struct       *tty = port->tty;
#else
        struct tty_struct       *tty = port->port.tty;
#endif

        if (debug) dev_dbg(&port->dev, "%s\n", __func__);

/*  mutex_lock(&config_mutex); */

        cflag = tty->termios->c_cflag;

        portdata->clocal = ((cflag & CLOCAL) != 0);

        block = portdata->block;

        vizzini_disable(port);

        if ((cflag & CSIZE) == CS7) {
                format_size = UART_FORMAT_SIZE_7;
        } else if ((cflag & CSIZE) == CS5) {
                /* Enabling 5-bit mode is really 9-bit mode! */
                format_size = UART_FORMAT_SIZE_9;
        } else {
                format_size = UART_FORMAT_SIZE_8;
        }
        portdata->trans9 = (format_size == UART_FORMAT_SIZE_9);

        if (cflag & PARENB) {
                if (cflag & PARODD) {
                        if (cflag & CMSPAR) {
                                format_parity = UART_FORMAT_PARITY_1;
                        } else {
                                format_parity = UART_FORMAT_PARITY_ODD;
                        }
                } else {
                        if (cflag & CMSPAR) {
                                format_parity = UART_FORMAT_PARITY_0;
                        } else {
                                format_parity = UART_FORMAT_PARITY_EVEN;
                        }
                }
        } else {
                format_parity = UART_FORMAT_PARITY_NONE;
        }

        if (cflag & CSTOPB) {
                format_stop = UART_FORMAT_STOP_2;
        } else {
                format_stop = UART_FORMAT_STOP_1;
        }

#ifdef VIZZINI_IWA
        if (format_size == UART_FORMAT_SIZE_8) {
                portdata->iwa = format_parity;
                if (portdata->iwa != UART_FORMAT_PARITY_NONE) {
                        format_size = UART_FORMAT_SIZE_9;
                        format_parity = UART_FORMAT_PARITY_NONE;
                }
        } else {
                portdata->iwa = UART_FORMAT_PARITY_NONE;
        }
#endif
        vizzini_set_reg(port, block, UART_FORMAT, format_size | format_parity | format_stop);

        if (cflag & CRTSCTS) {
                flow      = UART_FLOW_MODE_HW;
                gpio_mode = UART_GPIO_MODE_SEL_RTS_CTS;
        } else if (I_IXOFF(tty) || I_IXON(tty)) {
                unsigned char   start_char = START_CHAR(tty);
                unsigned char   stop_char  = STOP_CHAR(tty);

                flow      = UART_FLOW_MODE_SW;
                gpio_mode = UART_GPIO_MODE_SEL_GPIO;

                vizzini_set_reg(port, block, UART_XON_CHAR, start_char);
                vizzini_set_reg(port, block, UART_XOFF_CHAR, stop_char);
        } else {
                flow      = UART_FLOW_MODE_NONE;
                gpio_mode = UART_GPIO_MODE_SEL_GPIO;
        }

        vizzini_set_reg(port, block, UART_FLOW, flow);
        vizzini_set_reg(port, block, UART_GPIO_MODE, gpio_mode);

        if (portdata->trans9) {
                /* Turn on wide mode if we're 9-bit transparent. */
                vizzini_set_reg(port, EPLOCALS_REG_BLOCK, (block * MEM_EP_LOCALS_SIZE) + EP_WIDE_MODE, 1);
#ifdef VIZZINI_IWA
        } else if (portdata->iwa != UART_FORMAT_PARITY_NONE) {
                vizzini_set_reg(port, EPLOCALS_REG_BLOCK, (block * MEM_EP_LOCALS_SIZE) + EP_WIDE_MODE, 1);
#endif
        } else if (!portdata->preciseflags) {
                /* Turn off wide mode unless we have precise flags. */
                vizzini_set_reg(port, EPLOCALS_REG_BLOCK, (block * MEM_EP_LOCALS_SIZE) + EP_WIDE_MODE, 0);
        }

        rate = tty_get_baud_rate(tty);
	if(rate)
		vizzini_set_baud_rate(port, rate);

        vizzini_enable(port);

/*  mutex_unlock(&config_mutex); */
}

#define UART_TX_BREAK	0x14
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static void vizzini_break_ctl(struct usb_serial_port *port, int break_state)
#else
static void vizzini_break_ctl(struct tty_struct *tty, int break_state)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
#else
        struct usb_serial_port *port = tty->driver_data;
#endif
		struct vizzini_port_private *portdata = usb_get_serial_port_data(port);

        if (debug) dev_dbg(&port->dev, "BREAK %d\n", break_state);
        
		if (break_state)
			vizzini_set_reg(port, portdata->block, UART_TX_BREAK, 0xFF);
        else
			vizzini_set_reg(port, portdata->block, UART_TX_BREAK, 0x0);
}



#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static int vizzini_tiocmget(struct usb_serial_port *port, struct file *file)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
static int vizzini_tiocmget(struct tty_struct *tty, struct file *file)
#else
static int vizzini_tiocmget(struct tty_struct *tty)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
#else
        struct usb_serial_port *port = tty->driver_data;
#endif
		char data;
		int result;
		struct vizzini_port_private *portdata = usb_get_serial_port_data(port);

		result = vizzini_get_reg(port, portdata->block, UART_GPIO_STATUS, &data); 
		if (result == 1)
			return ((data & 0x8) ? TIOCM_DTR : 0) | ((data & 0x20) ? TIOCM_RTS : 0) | ((data & 0x4) ? TIOCM_DSR : 0) | ((data & 0x1) ? TIOCM_RI : 0) | ((data & 0x2) ? TIOCM_CD : 0) | ((data & 0x10) ? TIOCM_CTS : 0); 
		else
			return -EFAULT;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static int vizzini_tiocmset(struct usb_serial_port *port, struct file *file,
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
static int vizzini_tiocmset(struct tty_struct *tty, struct file *file,
#else
static int vizzini_tiocmset(struct tty_struct *tty,
#endif
                            unsigned int set, unsigned int clear)

{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
#else
        struct usb_serial_port *port = tty->driver_data;
#endif
        struct vizzini_port_private *portdata = usb_get_serial_port_data(port);
        unsigned int newctrl;

/*  if (!VIZZINI_READY(vizzini)) */
/*      return -EINVAL; */

        newctrl = portdata->ctrlout;
        set = (set & TIOCM_DTR ? ACM_CTRL_DTR : 0) | (set & TIOCM_RTS ? ACM_CTRL_RTS : 0);
        clear = (clear & TIOCM_DTR ? ACM_CTRL_DTR : 0) | (clear & TIOCM_RTS ? ACM_CTRL_RTS : 0);

        newctrl = (newctrl & ~clear) | set;

        if (portdata->ctrlout == newctrl)
                return 0;

		portdata->ctrlout = newctrl;
		if (newctrl & ACM_CTRL_DTR) 
			vizzini_set_reg(port, portdata->block, UART_GPIO_CLR, 0x08);
		else
			vizzini_set_reg(port, portdata->block, UART_GPIO_SET, 0x08);

		if (newctrl & ACM_CTRL_RTS) 
			vizzini_set_reg(port, portdata->block, UART_GPIO_CLR, 0x20);
		else
			vizzini_set_reg(port, portdata->block, UART_GPIO_SET, 0x20);

        return 0;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static int vizzini_ioctl(struct usb_serial_port *port, struct file *file, unsigned int cmd, unsigned long arg)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
static int vizzini_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg)
#else
static int vizzini_ioctl(struct tty_struct *tty, unsigned int cmd, unsigned long arg)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
#else
        struct usb_serial_port *port = tty->driver_data;
#endif
        struct vizzini_port_private *portdata = usb_get_serial_port_data(port);

        unsigned int             block, reg, val, match, preciseflags, unicast, broadcast, flow, selector;
        char                    *data;
        int                      result;
        struct serial_struct     ss;

        if (debug) dev_dbg(&port->dev, "%s %08x\n", __func__, cmd);

/*  if (!VIZZINI_READY(vizzini)) */
/*      return -EINVAL; */

        switch (cmd) {
        case TIOCGSERIAL:
                if (!arg)
                        return -EFAULT;
                memset(&ss, 0, sizeof(ss));
                ss.baud_base = portdata->baud_base;
                if (copy_to_user((void __user *)arg, &ss, sizeof(ss)))
                        return -EFAULT;
                break;

        case TIOCSSERIAL:
                if (!arg)
                        return -EFAULT;
                if (copy_from_user(&ss, (void __user *)arg, sizeof(ss)))
                        return -EFAULT;
                portdata->baud_base = ss.baud_base;
                if (debug) dev_dbg(&port->dev, "baud_base=%d\n", portdata->baud_base);

/*      mutex_lock(&config_mutex); */
                vizzini_disable(port);
		if(portdata->baud_base)
			vizzini_set_baud_rate(port, portdata->baud_base);
                vizzini_enable(port);
/*      mutex_unlock(&config_mutex); */
                break;

        case VZIOC_GET_REG:
                if (get_user(block, (int __user *)arg))
                        return -EFAULT;
                if (get_user(reg, (int __user *)(arg + sizeof(int))))
                        return -EFAULT;

                data = kmalloc(1, GFP_KERNEL);
                if (data == NULL) {
                        dev_err(&port->dev, "%s - Cannot allocate USB buffer.\n", __func__);
                        return -ENOMEM;
				}

				if (block == -1)
					block = portdata->block;

                result = vizzini_get_reg(port, block, reg, data);
                if (result != 1) {
                        dev_err(&port->dev, "Cannot get register (%d)\n", result);
                        kfree(data);
                        return -EFAULT;
                }

                if (put_user(data[0], (int __user *)(arg + 2 * sizeof(int)))) {
                        dev_err(&port->dev, "Cannot put user result\n");
                        kfree(data);
                        return -EFAULT;
                }

                kfree(data);
                break;

        case VZIOC_SET_REG:
                if (get_user(block, (int __user *)arg))
                        return -EFAULT;
                if (get_user(reg, (int __user *)(arg + sizeof(int))))
                        return -EFAULT;
                if (get_user(val, (int __user *)(arg + 2 * sizeof(int))))
                        return -EFAULT;

				if (block == -1)
					block = portdata->block;

                result = vizzini_set_reg(port, block, reg, val);
                if (result < 0)
                        return -EFAULT;
                break;

        case VZIOC_SET_ADDRESS_MATCH:
                match = arg;

                if (debug) dev_dbg(&port->dev, "%s VIOC_SET_ADDRESS_MATCH %d\n", __func__, match);

                vizzini_disable(port);

                if (match & VZ_ADDRESS_MATCH_DISABLE) {
                        flow      = UART_FLOW_MODE_NONE;
                } else {
                        flow      = UART_FLOW_MODE_ADDR_MATCH_TX;
                        unicast   = (match >> VZ_ADDRESS_UNICAST_S) & 0xff;
                        broadcast = (match >> VZ_ADDRESS_BROADCAST_S) & 0xff;
                }

                if (debug) dev_dbg(&port->dev, "address match: flow=%d ucast=%d bcast=%u\n",
                                   flow, unicast, broadcast);
                vizzini_set_reg(port, portdata->block, UART_FLOW, flow);
                vizzini_set_reg(port, portdata->block, UART_XON_CHAR, unicast);
                vizzini_set_reg(port, portdata->block, UART_XOFF_CHAR, broadcast);

                vizzini_enable(port);
                break;

        case VZIOC_SET_PRECISE_FLAGS:
                preciseflags = arg;

                if (debug) dev_dbg(&port->dev, "%s VIOC_SET_PRECISE_FLAGS %d\n", __func__, preciseflags);

                vizzini_disable(port);

                if (preciseflags) {
                        portdata->preciseflags = 1;
                } else {
                        portdata->preciseflags = 0;
                }

                vizzini_set_reg(port, EPLOCALS_REG_BLOCK,
                                (portdata->block * MEM_EP_LOCALS_SIZE) + EP_WIDE_MODE,
                                portdata->preciseflags);

                vizzini_enable(port);
                break;

        case VZIOC_TEST_MODE:
                selector = arg;
                if (debug) dev_dbg(&port->dev, "%s VIOC_TEST_MODE 0x%02x\n", __func__, selector);
                vizzini_test_mode(port, selector);
                break;

	case VZIOC_LOOPBACK:
		selector = arg;
		dev_dbg(&port->dev, "VIOC_LOOPBACK 0x%02x\n", selector);
		vizzini_loopback(port, selector);
		break;

        default:
                return -ENOIOCTLCMD;
        }

        return 0;
}


/* -------------------------------------------------------------------------- */


#ifdef VIZZINI_IWA
static const int vizzini_parity[] =
{
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0
};
#endif



#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 18)
static void vizzini_out_callback(struct urb *urb, struct pt_regs *regs)
#else
static void vizzini_out_callback(struct urb *urb)
#endif
{
        struct usb_serial_port          *port     = urb->context;
        struct vizzini_port_private     *portdata = usb_get_serial_port_data(port);
        int                              status   = urb->status;
        unsigned long                    flags;

        if (debug) dev_dbg(&port->dev, "%s - port %d\n", __func__, port->number);

        /* free up the transfer buffer, as usb_free_urb() does not do this */
        kfree(urb->transfer_buffer);

        if (status)
                if (debug) dev_dbg(&port->dev, "%s - nonzero write bulk status received: %d\n",
                                   __func__, status);

        spin_lock_irqsave(&portdata->lock, flags);
        --portdata->outstanding_urbs;
        spin_unlock_irqrestore(&portdata->lock, flags);

        usb_serial_port_softint(port);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static int vizzini_write_room(struct usb_serial_port *port)
#else
static int vizzini_write_room(struct tty_struct *tty)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
#else
        struct usb_serial_port *port = tty->driver_data;
#endif
		struct vizzini_port_private     *portdata = usb_get_serial_port_data(port);
        unsigned long                    flags;

        if (debug) dev_dbg(&port->dev, "%s - port %d\n", __func__, port->number);

        /* try to give a good number back based on if we have any free urbs at
         * this point in time */
        spin_lock_irqsave(&portdata->lock, flags);
        if (portdata->outstanding_urbs > N_OUT_URB * 2 / 3) {
                spin_unlock_irqrestore(&portdata->lock, flags);
                if (debug) dev_dbg(&port->dev, "%s - write limit hit\n", __func__);
                return 0;
        }
        spin_unlock_irqrestore(&portdata->lock, flags);

        return 2048;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static int vizzini_write(struct usb_serial_port *port,
#else
static int vizzini_write(struct tty_struct *tty, struct usb_serial_port *port,
#endif
                         const unsigned char *buf, int count)
{
        struct vizzini_port_private     *portdata = usb_get_serial_port_data(port);
        struct usb_serial               *serial   = port->serial;
        int                              bufsize  = count;
        unsigned long                    flags;
        unsigned char                   *buffer;
        struct urb                      *urb;
        int                              status;

        portdata = usb_get_serial_port_data(port);

        if (debug) dev_dbg(&port->dev, "%s: write (%d chars)\n", __func__, count);

        spin_lock_irqsave(&portdata->lock, flags);
        if (portdata->outstanding_urbs > N_OUT_URB) {
                spin_unlock_irqrestore(&portdata->lock, flags);
                if (debug) dev_dbg(&port->dev, "%s - write limit hit\n", __func__);
                return 0;
        }
        portdata->outstanding_urbs++;
        spin_unlock_irqrestore(&portdata->lock, flags);

#ifdef VIZZINI_IWA
        if (portdata->iwa != UART_FORMAT_PARITY_NONE)
                bufsize = count * 2;
#endif
        buffer = kmalloc(bufsize, GFP_ATOMIC);

        if (!buffer) {
                dev_err(&port->dev, "out of memory\n");
                count = -ENOMEM;
                goto error_no_buffer;
        }

        urb = usb_alloc_urb(0, GFP_ATOMIC);
        if (!urb) {
                dev_err(&port->dev, "no more free urbs\n");
                count = -ENOMEM;
                goto error_no_urb;
        }

#ifdef VIZZINI_IWA
        if (portdata->iwa != UART_FORMAT_PARITY_NONE) {
                int i;
                char *b = buffer;
                for (i = 0; i < count; ++i) {
                        int c, p = 0;
                        c = buf[i];
                        switch (portdata->iwa) {
                        case UART_FORMAT_PARITY_ODD:    p = !vizzini_parity[c]; break;
                        case UART_FORMAT_PARITY_EVEN:   p = vizzini_parity[c];  break;
                        case UART_FORMAT_PARITY_1:  p = 1;          break;
                        case UART_FORMAT_PARITY_0:  p = 0;          break;
                        }
                        *b++ = c;
                        *b++ = p;
                }
        } else
#endif
                memcpy(buffer, buf, count);

/*         usb_serial_debug_data(debug, &port->dev, __func__, bufsize, buffer); */

        usb_fill_bulk_urb(urb, serial->dev,
                          usb_sndbulkpipe(serial->dev,
                                          port->bulk_out_endpointAddress),
                          buffer, bufsize, vizzini_out_callback, port);

        /* send it down the pipe */
        status = usb_submit_urb(urb, GFP_ATOMIC);
        if (status) {
                dev_err(&port->dev, "%s - usb_submit_urb(write bulk) failed "
                        "with status = %d\n", __func__, status);
                count = status;
                goto error;
        }

        /* we are done with this urb, so let the host driver
         * really free it when it is finished with it */
        usb_free_urb(urb);

        return count;
error:
        usb_free_urb(urb);
error_no_urb:
        kfree(buffer);
error_no_buffer:
        spin_lock_irqsave(&portdata->lock, flags);
        --portdata->outstanding_urbs;
        spin_unlock_irqrestore(&portdata->lock, flags);
        return count;
}



/* -------------------------------------------------------------------------- */

#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 18)
static void vizzini_in_callback(struct urb *urb, struct pt_regs *regs)
#else
static void vizzini_in_callback(struct urb *urb)
#endif
{
        int                              endpoint        = usb_pipeendpoint(urb->pipe);
        struct usb_serial_port          *port            = urb->context;
        struct vizzini_port_private     *portdata        = usb_get_serial_port_data(port);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
        struct tty_struct               *tty             = port->tty;
#else
        struct tty_struct               *tty             = port->port.tty;
#endif
		int                              preciseflags    = portdata->preciseflags;
        char                            *transfer_buffer = urb->transfer_buffer;
        int                              length, room, have_extra_byte;
        int                              err;

        if (debug) dev_dbg(&port->dev, "%s: %p\n", __func__, urb);

        if (urb->status) {
                if (debug) dev_dbg(&port->dev, "%s: nonzero status: %d on endpoint %02x.\n",
                                   __func__, urb->status, endpoint);
                return;
        }

#ifdef VIZZINI_IWA
        if (portdata->iwa != UART_FORMAT_PARITY_NONE) {
                preciseflags = true;
        }
#endif

        length = urb->actual_length;
        if (length == 0) {
                if (debug) dev_dbg(&port->dev, "%s: empty read urb received\n", __func__);
                err = usb_submit_urb(urb, GFP_ATOMIC);
                if (err)
                        dev_err(&port->dev, "resubmit read urb failed. (%d)\n", err);
                return;
        }

        length      = length + (portdata->have_extra_byte ? 1 : 0);
        have_extra_byte = (preciseflags && (length & 1));
        length      = (preciseflags) ? (length / 2) : length;

        room = tty_buffer_request_room(tty, length);
        if (room != length) {
                if (debug) dev_dbg(&port->dev, "Not enough room in TTY buf, dropped %d chars.\n", length - room);
        }

        if (room) {
                if (preciseflags) {
                        char *dp = transfer_buffer;
                        int i, ch, ch_flags;

                        for (i = 0; i < room; ++i) {
                                char tty_flag;

                                if (i == 0) {
                                        if (portdata->have_extra_byte) {
                                                ch = portdata->extra_byte;
                                        } else {
                                                ch = *dp++;
                                        }
                                } else {
                                        ch = *dp++;
                                }
                                ch_flags = *dp++;

#ifdef VIZZINI_IWA
                                {
                                        int p;
                                        switch (portdata->iwa) {
                                        case UART_FORMAT_PARITY_ODD:    p = !vizzini_parity[ch]; break;
                                        case UART_FORMAT_PARITY_EVEN:   p = vizzini_parity[ch];  break;
                                        case UART_FORMAT_PARITY_1:  p = 1;           break;
                                        case UART_FORMAT_PARITY_0:  p = 0;           break;
                                        default:                        p = 0;           break;
                                        }
                                        ch_flags ^= p;
                                }
#endif
                                if (ch_flags & RAMCTL_BUFFER_PARITY)
                                        tty_flag = TTY_PARITY;
                                else if (ch_flags & RAMCTL_BUFFER_BREAK)
                                        tty_flag = TTY_BREAK;
                                else if (ch_flags & RAMCTL_BUFFER_FRAME)
                                        tty_flag = TTY_FRAME;
                                else if (ch_flags & RAMCTL_BUFFER_OVERRUN)
                                        tty_flag = TTY_OVERRUN;
                                else
                                        tty_flag = TTY_NORMAL;

                                tty_insert_flip_char(tty, ch, tty_flag);
                        }
                } else {
                        tty_insert_flip_string(tty, transfer_buffer, room);
                }

                tty_flip_buffer_push(tty);
        }

        portdata->have_extra_byte = have_extra_byte;
        if (have_extra_byte) {
                portdata->extra_byte = transfer_buffer[urb->actual_length - 1];
        }

        err = usb_submit_urb(urb, GFP_ATOMIC);
        if (err)
                dev_err(&port->dev, "resubmit read urb failed. (%d)\n", err);
}


/* -------------------------------------------------------------------------- */
#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 18)
static void vizzini_int_callback(struct urb *urb, struct pt_regs *regs)
#else
static void vizzini_int_callback(struct urb *urb)
#endif
{
        struct usb_serial_port          *port     = urb->context;
        struct vizzini_port_private     *portdata = usb_get_serial_port_data(port);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
        struct tty_struct               *tty      = port->tty;
#else
        struct tty_struct               *tty      = port->port.tty;
#endif
        struct usb_cdc_notification     *dr       = urb->transfer_buffer;
        unsigned char                   *data;
        int                              newctrl;
        int                              status;

        switch (urb->status) {
        case 0:
                /* success */
                break;
        case -ECONNRESET:
        case -ENOENT:
        case -ESHUTDOWN:
                /* this urb is terminated, clean up */
                if (debug) dev_dbg(&port->dev, "urb shutting down with status: %d\n", urb->status);
                return;
        default:
                if (debug) dev_dbg(&port->dev, "nonzero urb status received: %d\n", urb->status);
                goto exit;
        }

/*  if (!VIZZINI_READY(vizzini)) */
/*      goto exit; */

        data = (unsigned char *)(dr + 1);
        switch (dr->bNotificationType) {

        case USB_CDC_NOTIFY_NETWORK_CONNECTION:
                if (debug) dev_dbg(&port->dev, "%s network\n", dr->wValue ? "connected to" : "disconnected from");
                break;

        case USB_CDC_NOTIFY_SERIAL_STATE:
                newctrl = le16_to_cpu(get_unaligned((__le16 *)data));

                if (!portdata->clocal && (portdata->ctrlin & ~newctrl & ACM_CTRL_DCD)) {
                        if (debug) dev_dbg(&port->dev, "calling hangup\n");
                        tty_hangup(tty);
                }

                portdata->ctrlin = newctrl;

                if (debug) dev_dbg(&port->dev, "input control lines: dcd%c dsr%c break%c ring%c framing%c parity%c overrun%c\n",
                                   portdata->ctrlin & ACM_CTRL_DCD ? '+' : '-',
                                   portdata->ctrlin & ACM_CTRL_DSR ? '+' : '-',
                                   portdata->ctrlin & ACM_CTRL_BRK ? '+' : '-',
                                   portdata->ctrlin & ACM_CTRL_RI  ? '+' : '-',
                                   portdata->ctrlin & ACM_CTRL_FRAMING ? '+' : '-',
                                   portdata->ctrlin & ACM_CTRL_PARITY ? '+' : '-',
                                   portdata->ctrlin & ACM_CTRL_OVERRUN ? '+' : '-');
                break;

        default:
                if (debug) dev_dbg(&port->dev, "unknown notification %d received: index %d len %d data0 %d data1 %d\n",
                                   dr->bNotificationType, dr->wIndex,
                                   dr->wLength, data[0], data[1]);
                break;
        }
exit:
        if (debug) dev_dbg(&port->dev, "Resubmitting interrupt IN urb %p\n", urb);
        status = usb_submit_urb(urb, GFP_ATOMIC);
        if (status)
                dev_err(&port->dev, "usb_submit_urb failed with result %d", status);
}


/* -------------------------------------------------------------------------- */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static int vizzini_open(struct usb_serial_port *port, struct file *filp)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
static int vizzini_open(struct tty_struct *tty_param,
                        struct usb_serial_port *port, struct file *filp)
#else
static int vizzini_open(struct tty_struct *tty_param, struct usb_serial_port *port)
#endif
{
        struct vizzini_port_private     *portdata;
        struct usb_serial               *serial = port->serial;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
        struct tty_struct               *tty    = port->tty;
#else
        struct tty_struct               *tty    = port->port.tty;
#endif
        int                              i;
        struct urb                      *urb;
        int                              result;

        portdata = usb_get_serial_port_data(port);

        if (debug) dev_dbg(&port->dev, "%s\n", __func__);

		portdata->ctrlout = ACM_CTRL_DTR | ACM_CTRL_RTS;
		if (portdata->ctrlout & ACM_CTRL_DTR) 
			vizzini_set_reg(port, portdata->block, UART_GPIO_CLR, 0x08);
		else
			vizzini_set_reg(port, portdata->block, UART_GPIO_SET, 0x08);

		if (portdata->ctrlout & ACM_CTRL_RTS) 
			vizzini_set_reg(port, portdata->block, UART_GPIO_CLR, 0x20);
		else
			vizzini_set_reg(port, portdata->block, UART_GPIO_SET, 0x20);

        /* Reset low level data toggle and start reading from endpoints */
        for (i = 0; i < N_IN_URB; i++) {
                if (debug) dev_dbg(&port->dev, "%s urb %d\n", __func__, i);

                urb = portdata->in_urbs[i];
                if (!urb)
                        continue;
                if (urb->dev != serial->dev) {
                        if (debug) dev_dbg(&port->dev, "%s: dev %p != %p\n", __func__,
                                           urb->dev, serial->dev);
                        continue;
                }

                /*
                 * make sure endpoint data toggle is synchronized with the
                 * device
                 */
/*      if (debug) dev_dbg(&port->dev, "%s clearing halt on %x\n", __func__, urb->pipe); */
/*      usb_clear_halt(urb->dev, urb->pipe); */

                if (debug) dev_dbg(&port->dev, "%s submitting urb %p\n", __func__, urb);
                result = usb_submit_urb(urb, GFP_KERNEL);
                if (result) {
                        dev_err(&port->dev, "submit urb %d failed (%d) %d\n",
                                i, result, urb->transfer_buffer_length);
                }
        }

        tty->low_latency = 1;

        /* start up the interrupt endpoint if we have one */
        if (port->interrupt_in_urb) {
                result = usb_submit_urb(port->interrupt_in_urb, GFP_KERNEL);
                if (result)
                        dev_err(&port->dev, "submit irq_in urb failed %d\n",
                                result);
        }
        return 0;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static void vizzini_close(struct usb_serial_port *port, struct file *filp)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
static void vizzini_close(struct tty_struct *tty_param,
                          struct usb_serial_port *port, struct file *filp)
#else
static void vizzini_close(struct usb_serial_port *port)
#endif
{
        int                              i;
        struct usb_serial               *serial = port->serial;
        struct vizzini_port_private     *portdata;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
        struct tty_struct               *tty    = port->tty;
#else
        struct tty_struct               *tty    = port->port.tty;
#endif
        if (debug) dev_dbg(&port->dev, "%s\n", __func__);
        portdata = usb_get_serial_port_data(port);

		portdata->ctrlout = 0;
		if (portdata->ctrlout & ACM_CTRL_DTR) 
			vizzini_set_reg(port, portdata->block, UART_GPIO_CLR, 0x08);
		else
			vizzini_set_reg(port, portdata->block, UART_GPIO_SET, 0x08);

		if (portdata->ctrlout & ACM_CTRL_RTS) 
			vizzini_set_reg(port, portdata->block, UART_GPIO_CLR, 0x20);
		else
			vizzini_set_reg(port, portdata->block, UART_GPIO_SET, 0x20);

        if (serial->dev) {
                /* Stop reading/writing urbs */
                for (i = 0; i < N_IN_URB; i++)
                        usb_kill_urb(portdata->in_urbs[i]);
        }

        usb_kill_urb(port->interrupt_in_urb);

        tty = NULL; /* FIXME */
}


static int vizzini_attach(struct usb_serial *serial)
{
        struct vizzini_serial_private   *serial_priv       = usb_get_serial_data(serial);
        struct usb_interface            *interface         = serial_priv->data_interface;
        struct usb_host_interface       *iface_desc;
        struct usb_endpoint_descriptor  *endpoint;
        struct usb_endpoint_descriptor  *bulk_in_endpoint  = NULL;
        struct usb_endpoint_descriptor  *bulk_out_endpoint = NULL;

        struct usb_serial_port          *port;
        struct vizzini_port_private     *portdata;
        struct urb                      *urb;
        int                              i, j;

        /* Assume that there's exactly one serial port. */
        port = serial->port[0];

        if (debug) dev_dbg(&port->dev, "%s\n", __func__);

        /* The usb_serial is now fully set up, but we want to make a
         * couple of modifications.  Namely, it was configured based
         * upon the control interface and not the data interface, so
         * it has no notion of the bulk in and out endpoints.  So we
         * essentially do some of the same allocations and
         * configurations that the usb-serial core would have done if
         * it had not made any faulty assumptions about the
         * endpoints. */

        iface_desc = interface->cur_altsetting;
        for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
                endpoint = &iface_desc->endpoint[i].desc;

                if (usb_endpoint_is_bulk_in(endpoint)) {
                        bulk_in_endpoint = endpoint;
                }

                if (usb_endpoint_is_bulk_out(endpoint)) {
                        bulk_out_endpoint = endpoint;
                }
        }

        if (!bulk_out_endpoint || !bulk_in_endpoint) {
                if (debug) dev_dbg(&port->dev, "Missing endpoint!\n");
                return -EINVAL;
        }

        port->bulk_out_endpointAddress = bulk_out_endpoint->bEndpointAddress;
        port->bulk_in_endpointAddress = bulk_in_endpoint->bEndpointAddress;

        portdata = kzalloc(sizeof(*portdata), GFP_KERNEL);
        if (!portdata) {
                if (debug) dev_dbg(&port->dev, "%s: kmalloc for vizzini_port_private (%d) failed!.\n",
                                   __func__, i);
                return -ENOMEM;
        }
        spin_lock_init(&portdata->lock);
        for (j = 0; j < N_IN_URB; j++) {
                portdata->in_buffer[j] = kmalloc(IN_BUFLEN, GFP_KERNEL);
                if (!portdata->in_buffer[j]) {
                        for (--j; j >= 0; j--)
                                kfree(portdata->in_buffer[j]);
                        kfree(portdata);
                        return -ENOMEM;
                }
        }

        /* Bulk OUT endpoints 0x1..0x4 map to register blocks 0..3 */
        portdata->block = port->bulk_out_endpointAddress - 1;

        usb_set_serial_port_data(port, portdata);

	portdata->bcd_device = le16_to_cpu(serial->dev->descriptor.bcdDevice);
	if (vizzini_rev_a(port))
		dev_info(&port->dev, "Adapting to revA silicon\n");

        /* initialize the in urbs */
        for (j = 0; j < N_IN_URB; ++j) {
                urb = usb_alloc_urb(0, GFP_KERNEL);
                if (urb == NULL) {
                        if (debug) dev_dbg(&port->dev, "%s: alloc for in port failed.\n",
                                           __func__);
                        continue;
                }
                /* Fill URB using supplied data. */
                if (debug) dev_dbg(&port->dev, "Filling URB %p, EP=%d buf=%p len=%d\n", urb, port->bulk_in_endpointAddress, portdata->in_buffer[j], IN_BUFLEN);
                usb_fill_bulk_urb(urb, serial->dev,
                                  usb_rcvbulkpipe(serial->dev,
                                                  port->bulk_in_endpointAddress),
                                  portdata->in_buffer[j], IN_BUFLEN,
                                  vizzini_in_callback, port);
                portdata->in_urbs[j] = urb;
        }

        return 0;
}


static void vizzini_serial_disconnect(struct usb_serial *serial)
{
        struct usb_serial_port          *port;
        struct vizzini_port_private     *portdata;
        int                              i, j;

        if (debug) dev_dbg(&serial->dev->dev, "%s %p\n", __func__, serial);

        for (i = 0; i < serial->num_ports; ++i) {
                port = serial->port[i];
                if (!port)
                        continue;
                portdata = usb_get_serial_port_data(port);
                if (!portdata)
                        continue;

                for (j = 0; j < N_IN_URB; j++) {
                        usb_kill_urb(portdata->in_urbs[j]);
                        usb_free_urb(portdata->in_urbs[j]);
                }
        }
}


static void vizzini_serial_release(struct usb_serial *serial)
{
        struct usb_serial_port          *port;
        struct vizzini_port_private     *portdata;
        int                              i, j;

        if (debug) dev_dbg(&serial->dev->dev, "%s %p\n", __func__, serial);

        for (i = 0; i < serial->num_ports; ++i) {
                port = serial->port[i];
                if (!port)
                        continue;
                portdata = usb_get_serial_port_data(port);
                if (!portdata)
                        continue;

                for (j = 0; j < N_IN_URB; j++) {
                        kfree(portdata->in_buffer[j]);
                }
                kfree(portdata);
                usb_set_serial_port_data(port, NULL);
        }
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
static void vizzini_shutdown(struct usb_serial *serial)
{
        vizzini_serial_disconnect(serial);
        vizzini_serial_release(serial);
}
#endif


/* -------------------------------------------------------------------------- */

static int vizzini_calc_num_ports(struct usb_serial *serial)
{
        return 1;
}


static int vizzini_probe(struct usb_serial *serial,
                         const struct usb_device_id *id)
{
        struct usb_interface                    *intf                     = serial->interface;
        unsigned char                           *buffer                   = intf->altsetting->extra;
        int                                      buflen                   = intf->altsetting->extralen;
        struct usb_device                       *usb_dev                  = interface_to_usbdev(intf);
        struct usb_cdc_union_desc               *union_header             = NULL;
        struct usb_cdc_country_functional_desc  *cfd                      = NULL;
        u8                                       ac_management_function   = 0;
        u8                                       call_management_function = 0;
        int                                      call_interface_num       = -1;
        int                                      data_interface_num;
        struct usb_interface                    *control_interface;
        struct usb_interface                    *data_interface;
        struct usb_endpoint_descriptor          *epctrl;
        struct usb_endpoint_descriptor          *epread;
        struct usb_endpoint_descriptor          *epwrite;
        struct vizzini_serial_private           *serial_priv;

        if (!buffer) {
                dev_err(&intf->dev, "Weird descriptor references\n");
                return -EINVAL;
        }

        if (!buflen) {
                if (intf->cur_altsetting->endpoint->extralen && intf->cur_altsetting->endpoint->extra) {
                        if (debug) dev_dbg(&intf->dev, "Seeking extra descriptors on endpoint\n");
                        buflen = intf->cur_altsetting->endpoint->extralen;
                        buffer = intf->cur_altsetting->endpoint->extra;
                } else {
                        dev_err(&intf->dev, "Zero length descriptor references\n");
                        return -EINVAL;
                }
        }

        while (buflen > 0) {
                if (buffer[1] != USB_DT_CS_INTERFACE) {
                        dev_err(&intf->dev, "skipping garbage\n");
                        goto next_desc;
                }

                switch (buffer[2]) {
                case USB_CDC_UNION_TYPE: /* we've found it */
                        if (union_header) {
                                dev_err(&intf->dev, "More than one union descriptor, skipping ...\n");
                                goto next_desc;
                        }
                        union_header = (struct usb_cdc_union_desc *)buffer;
                        break;
                case USB_CDC_COUNTRY_TYPE: /* export through sysfs */
                        cfd = (struct usb_cdc_country_functional_desc *)buffer;
                        break;
                case USB_CDC_HEADER_TYPE: /* maybe check version */
                        break; /* for now we ignore it */
                case USB_CDC_ACM_TYPE:
                        ac_management_function = buffer[3];
                        break;
                case USB_CDC_CALL_MANAGEMENT_TYPE:
                        call_management_function = buffer[3];
                        call_interface_num = buffer[4];
                        if ((call_management_function & 3) != 3) {
/*              dev_err(&intf->dev, "This device cannot do calls on its own. It is no modem.\n"); */
                        }
                        break;
                default:
                        /* there are LOTS more CDC descriptors that
                         * could legitimately be found here.
                         */
                        if (debug) dev_dbg(&intf->dev, "Ignoring descriptor: "
                                           "type %02x, length %d\n",
                                           buffer[2], buffer[0]);
                        break;
                }
        next_desc:
                buflen -= buffer[0];
                buffer += buffer[0];
        }

        if (!union_header) {
                if (call_interface_num > 0) {
                        if (debug) dev_dbg(&intf->dev, "No union descriptor, using call management descriptor\n");
                        data_interface = usb_ifnum_to_if(usb_dev, (data_interface_num = call_interface_num));
                        control_interface = intf;
                } else {
                        if (debug) dev_dbg(&intf->dev, "No union descriptor, giving up\n");
                        return -ENODEV;
                }
        } else {
                control_interface = usb_ifnum_to_if(usb_dev, union_header->bMasterInterface0);
                data_interface    = usb_ifnum_to_if(usb_dev, (data_interface_num = union_header->bSlaveInterface0));
                if (!control_interface || !data_interface) {
                        if (debug) dev_dbg(&intf->dev, "no interfaces\n");
                        return -ENODEV;
                }
        }

        if (data_interface_num != call_interface_num)
                if (debug) dev_dbg(&intf->dev, "Separate call control interface. That is not fully supported.\n");

        /* workaround for switched interfaces */
        if (data_interface->cur_altsetting->desc.bInterfaceClass != CDC_DATA_INTERFACE_TYPE) {
                if (control_interface->cur_altsetting->desc.bInterfaceClass == CDC_DATA_INTERFACE_TYPE) {
                        struct usb_interface *t;
/*          if (debug) dev_dbg(&intf->dev, "Your device has switched interfaces.\n"); */

                        t = control_interface;
                        control_interface = data_interface;
                        data_interface = t;
                } else {
                        return -EINVAL;
                }
        }

        /* Accept probe requests only for the control interface */
        if (intf != control_interface) {
/*      if (debug) dev_dbg(&intf->dev, "Skipping data interface %p\n", intf); */
                return -ENODEV;
        }
/*  if (debug) dev_dbg(&intf->dev, "Grabbing control interface %p\n", intf); */

        if (usb_interface_claimed(data_interface)) { /* valid in this context */
                if (debug) dev_dbg(&intf->dev, "The data interface isn't available\n");
                return -EBUSY;
        }

        if (data_interface->cur_altsetting->desc.bNumEndpoints < 2)
                return -EINVAL;

        epctrl  = &control_interface->cur_altsetting->endpoint[0].desc;
        epread  = &data_interface->cur_altsetting->endpoint[0].desc;
        epwrite = &data_interface->cur_altsetting->endpoint[1].desc;
        if (!usb_endpoint_dir_in(epread)) {
                struct usb_endpoint_descriptor *t;
                t   = epread;
                epread  = epwrite;
                epwrite = t;
        }

        /* The documentation suggests that we allocate private storage
         * with the attach() entry point, but we can't allow the data
         * interface to remain unclaimed until then; so we need
         * somewhere to save the claimed interface now. */
        if (!(serial_priv = kzalloc(sizeof(struct vizzini_serial_private), GFP_KERNEL))) {
                if (debug) dev_dbg(&intf->dev, "out of memory\n");
                goto alloc_fail;
        }
        usb_set_serial_data(serial, serial_priv);

/*  if (debug) dev_dbg(&intf->dev, "Claiming data interface %p\n", data_interface); */
        usb_driver_claim_interface(&vizzini_driver, data_interface, NULL);

        /* Don't set the data interface private data.  When we
         * disconnect we test this field against NULL to discover
         * whether we're dealing with the control or data
         * interface. */
        serial_priv->data_interface = data_interface;

        return 0;

alloc_fail:
        return -ENOMEM;
}


static void vizzini_disconnect(struct usb_interface *interface)
{
        struct usb_serial               *serial = usb_get_intfdata(interface);
        struct vizzini_serial_private   *serial_priv;

        if (debug) dev_dbg(&interface->dev, "%s %p\n", __func__, interface);

        if (!serial) {
                /* NULL interface private data means that we're
                 * dealing with the data interface and not the control
                 * interface.  So we just bail and let the real clean
                 * up happen later when the control interface is
                 * disconnected. */
                return;
        }

        serial_priv = usb_get_serial_data(serial);

/*  if (debug) dev_dbg(&interface->dev, "Releasing data interface %p.\n", serial_priv->data_interface); */
        usb_driver_release_interface(&vizzini_driver, serial_priv->data_interface);

        kfree(serial_priv);
        usb_set_serial_data(serial, NULL);

/*  if (debug) dev_dbg(&interface->dev, "Disconnecting control interface\n"); */
        usb_serial_disconnect(interface);
}



static struct usb_serial_driver vizzini_device = {
        .driver = {
                .owner =    THIS_MODULE,
                .name =     "vizzini",
        },
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 37)
#ifndef	ELITE_KERNEL
		.usb_driver	    = &vizzini_driver,
#endif
#endif
        .description        = "Vizzini USB serial port",
        .id_table           = id_table,
        .calc_num_ports     = vizzini_calc_num_ports,
        .probe              = vizzini_probe,
        .open               = vizzini_open,
        .close              = vizzini_close,
        .write              = vizzini_write,
        .write_room         = vizzini_write_room,
        .ioctl              = vizzini_ioctl,
        .set_termios        = vizzini_set_termios,
        .break_ctl          = vizzini_break_ctl,
        .tiocmget           = vizzini_tiocmget,
        .tiocmset           = vizzini_tiocmset,
        .attach             = vizzini_attach,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
        .shutdown          = vizzini_shutdown,
#else
        .disconnect        = vizzini_serial_disconnect,
        .release           = vizzini_serial_release,
#endif
        .read_int_callback  = vizzini_int_callback,
};


#ifdef	ELITE_KERNEL

static struct usb_serial_driver * const serial_drivers[] = {
	&vizzini_device, NULL
};

#endif


/* Functions used by new usb-serial code. */
static int __init vizzini_init(void)
{
        int retval;

#ifdef	ELITE_KERNEL

	retval = usb_serial_register_drivers(&vizzini_driver, serial_drivers);
        if (retval)
                goto failed_driver_register;

        printk(KERN_INFO DRIVER_DESC ": " DRIVER_VERSION "\n");

        return 0;

failed_driver_register:
        return retval;

#else

        retval = usb_serial_register(&vizzini_device);
        if (retval)
                goto failed_device_register;


        retval = usb_register(&vizzini_driver);
        if (retval)
                goto failed_driver_register;

        printk(KERN_INFO DRIVER_DESC ": " DRIVER_VERSION "\n");

        return 0;

failed_driver_register:
        usb_serial_deregister(&vizzini_device);
failed_device_register:
        return retval;

#endif
}


static void __exit vizzini_exit(void)
{
#ifdef	ELITE_KERNEL

	usb_serial_deregister_drivers(&vizzini_driver, serial_drivers);

#else

        usb_deregister(&vizzini_driver);
        usb_serial_deregister(&vizzini_device);

#endif
}

module_init(vizzini_init);
module_exit(vizzini_exit);


MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

#ifndef	ELITE_KERNEL
module_param(debug, bool, S_IRUGO | S_IWUSR);
#endif

MODULE_PARM_DESC(debug, "Debug messages");
