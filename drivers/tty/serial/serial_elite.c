/*
	linux/drivers/serial/serial_elite.c

	Copyright (c) 2012  S3Graphics Technologies, Inc.

	This program is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software Foundation,
	either version 2 of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful, but WITHOUT
	ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
	PARTICULAR PURPOSE.  See the GNU General Public License for more details.
	You should have received a copy of the GNU General Public License along with
	this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/module.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <mach/io.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <linux/serial_core.h>

#include <linux/dma-mapping.h>
#include <linux/uaccess.h>

#define BIT0	0x00000001
#define BIT1	0x00000002
#define BIT2	0x00000004
#define BIT3	0x00000008
#define BIT4	0x00000010
#define BIT5	0x00000020
#define BIT6	0x00000040
#define BIT7	0x00000080
#define BIT8	0x00000100
#define BIT9	0x00000200
#define BIT10	0x00000400
#define BIT11	0x00000800
#define BIT12	0x00001000
#define BIT13	0x00002000

/*
 * UART Line Control Register Bit Definitions
 */
#define URLCR_TXEN      BIT0	/* Transmit operation enabled           */
#define URLCR_RXEN      BIT1	/* Receive operation enabled            */
#define URLCR_DLEN      BIT2	/* Data length 0:7-bit 1:8-bit          */
#define URLCR_STBLEN    BIT3	/* Stop bit length 0:1-bit 1:2-bit      */
#define URLCR_PTYEN     BIT4	/* Parity bit  0:inactive 1:active      */
#define URLCR_PTYMODE   BIT5	/* Parity mode 0:evev 1:odd             */
/* Request to send. A software controlled RTS modem signal, used when IrDA is disableda */
#define URLCR_RTS       BIT6
#define URLCR_LPBEN     BIT7	/* Loopback mode 0:inactive 1:active    */
#define URLCR_DMAEN     BIT8	/* DMA enable. 0:inactive 1:active      */
#define URLCR_BKINIT    BIT9	/* Bluetooth break signal initiation.   */
/* Bit[10:31] are reserved. */

/*
 * UART Status Register Bit Definitions
 */
#define URUSR_TXON      BIT0	/* Transmission is active               */
#define URUSR_TXDBSY    BIT1	/* TX data is being loaded to TX port from either URTDR or TX FIFO */
#define URUSR_RXON      BIT2	/* Reception is active                  */
#define URUSR_RXDRDY    BIT3	/* RX data is ready in either URRDR or RX FIFO */
#define URUSR_CTS       BIT4	/* Status of CTS signal                 */
#define URUSR_MASK      ((1 << 5) - 1)	/* Mask for useful bits         */
/* Bit[5:31] are reserved. */

/*
 * UART Interrupt Enable Register Bit Definitions
 */
#define URIER_ETXDE     BIT0	/* Enable for TX data register empty    */
#define URIER_ERXDF     BIT1	/* Enable for RX data register full     */
#define URIER_ETXFAE    BIT2	/* Enable for TX FIFO almost full       */
#define URIER_ETXFE     BIT3	/* Enable for TX FIFO full              */
#define URIER_ERXFAF    BIT4	/* Enable for RX FIFO almost full       */
#define URIER_ERXFF     BIT5	/* Enable for RX FIFO full              */
#define URIER_ETXDUDR   BIT6	/* Enable for TX underrun               */
#define URIER_ERXDOVR   BIT7	/* Enable for RX overrun                */
#define URIER_EPER      BIT8	/* Enable for parity error              */
#define URIER_EFER      BIT9	/* Enable for frame error               */
#define URIER_EMODM     BIT10	/* Enable for modem control signal      */
#define URIER_ERXTOUT   BIT11	/* Enable for receive time out          */
#define URIER_EBK       BIT12	/* Enable for break signal done         */
/* Bit[13:31] are reserved. */

/*
 * UART Interrupt Status Register Bit Definitions
 */
#define URISR_TXDE      BIT0	/* TX data register empty               */
#define URISR_RXDF      BIT1	/* RX data register full                */
#define URISR_TXFAE     BIT2	/* TX FIFO almost empty                 */
#define URISR_TXFE      BIT3	/* TX FIFO empty                        */
#define URISR_RXFAF     BIT4	/* RX FIFO almost empty                 */
#define URISR_RXFF      BIT5	/* RX FIFO empty                        */
#define URISR_TXDUDR    BIT6	/* TX underrun                          */
#define URISR_RXDOVR    BIT7	/* RX overrun                           */
#define URISR_PER       BIT8	/* Parity error                         */
#define URISR_FER       BIT9	/* Frame error                          */

/* Toggle clear to send modem control signal. Used when IrDA is disabled*/
#define URISR_TCTS      BIT10
#define URISR_RXTOUT    BIT11	/* Receive time out                     */
#define URISR_BKDONE    BIT12	/* Break signal done                    */
#define URISR_MASK      ((1 << 13) - 1)	/* Mask for useful bits         */
/* Bit[13:31] are reserved. */

/*
 * IrDA Mode Control Register Description
 */
#define URICR_IREN      BIT0	/* Set "1" to enable IrDA               */
/* Bit[1:31] are reserved. */

/*
 * UART FIFO Control Register Description
 */
#define URFCR_FIFOEN            BIT0
/* Bit[1:3] are reserved. */

/*
 * Macros for setting threshold value to TX or RX FIFO level setting.
 */
#define URFCR_FLVMASK           0xf                           /* FIFO threshold Level Mask */
#define URFCR_TXFLV(x)          (((x) & URFCR_FLVMASK) << 4)  /* TX FIFO threshold */
#define URFCR_RXFLV(x)          (((x) & URFCR_FLVMASK) << 8)  /* RX FIFO threshold */
/* Bit[12:31] are reserved. */

/*
 * UART Baud Rate Divisor Register Description.
 */
#define URBRD_BRDMASK           0x3ff                         /* Bit[0:9] are baud rate divisor */
#define URBRD_BRD(x)            ((x) & URBRD_BRDMASK)
/* Bit[10:31] are reserved. */

/*
 * UART FIFO Index Register Description.
 */
#define URFIDX_IDXMASK          0x1f
/*
 * Macros for getting URFIDX value to TX or RX FIFO index.
 */                           /* FIFO index Mask */
#define URFIDX_TXFIDX(x)        ((x) & URFIDX_IDXMASK)        /* Get TX FIFO remaing entries */
/* Bit[5:7] are reserved. */

#define URFIDX_RXFIDX(x)        (((x) >> 8) & URFIDX_IDXMASK) /* Get RX FIFO remaing entries */
/* Bit[13:31] are reserved. */

/*
 * UART Break Counter Value Register Description.
 */
#define URBKR_BCVMASK           0x0fff                        /* Bit[0:11] are break counter value */
#define URBKR_BCV(x)            ((x) & URBKR_BCVMASK)
/* Bit[12:31] are reserved. */

#define URFCR_TXFRST            0x4	/* TX Fifo Reset */
#define URFCR_RXFRST            0x8	/* Rx Fifo Reset */

/*
 * UART clock divisor Register Description.
 */
#define URDIV_DIVMASK           0xf0000	/* Bit[16:19] are UART clock divisor */
#define URDIV_DIV(x)            (((x) >> 16) & URDIV_DIVMASK)
/* Bit[4:31] are reserved. */

/*
 * UART module registers offset, add by Harry temporary.
 */
#define URTDR                   0x0000
#define URRDR                   0x0004
#define URDIV                   0x0008
#define URLCR                   0x000C
#define URICR                   0x0010
#define URIER                   0x0014
#define URISR                   0x0018
#define URUSR                   0x001C
#define URFCR                   0x0020
#define URFIDX                  0x0024
#define URBKR                   0x0028
#define URTOD                   0x002C
#define URTXF                   0x01000
#define URRXF                   0x01020

/*
 * URBRD_BRD value simple examples.
 */
#define BRD_921600BPS           0x10000
#define BRD_460800BPS           0x10001
#define BRD_230400BPS           0x10003
#define BRD_115200BPS           0x10007
#define BRD_76800BPS            0x1000B
#define BRD_57600BPS            0x1000F
#define BRD_38400BPS            0x10017
#define BRD_28800BPS            0x1001F

/*
 * URBKR_BCV value simple examples.
 *
 * Simply calculated by (baud_rate * 0.004096)
 * then take the integer.
 */
#define BCV_921600BPS           3775
#define BCV_460800BPS           1887
#define BCV_230400BPS           944
#define BCV_115200BPS           472
#define BCV_76800BPS            315
#define BCV_57600BPS            236
#define BCV_38400BPS            157
#define BCV_28800BPS            118

/*
 * URDIV_DIV value simple examples.
 *
 * Followings generate UCLK = 12MHZ
 */
#define DIV_192MHZ              15
#define DIV_180MHZ              14
#define DIV_168MHZ              13
#define DIV_156MHZ              12
#define DIV_144MHZ              11
#define DIV_132MHZ              10
#define DIV_120MHZ              9
#define DIV_108MHZ              8
#define DIV_96MHZ               7
#define DIV_84MHZ               6
#define DIV_72MHZ               5
#define DIV_60MHZ               4
#define DIV_48MHZ               3
#define DIV_36MHZ               2
#define DIV_24MHZ               1
#define DIV_12MHZ               0

/*
 * Data mask used in RX FIFO or URRDR.
 */
#define RX_DATAMASK             0xff	/* Bit[0:7] are reception data */
#define RX_PERMASK              0x01ff	/* Bit[0:8] */
#define RX_FERMASK              0x03ff	/* Bit[0:9] */



#define PORT_ELITE 54
#if defined(CONFIG_SERIAL_ELITE_UART_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif



#define UART_BR_115K2 115200

/*
 * This is for saving useful I/O registers
 */

#ifdef CONFIG_SERIAL_ELITE_TTYVT
#define SERIAL_ELITE_MAJOR     204
#define MINOR_START             90      /* Start from ttyVT0 */
#define CALLOUT_ELITE_MAJOR    205     /* for callout device */
#else
#define SERIAL_ELITE_MAJOR     4
#define MINOR_START             64      /* Start from ttyS0 */
#define CALLOUT_ELITE_MAJOR    5       /* for callout device */
#endif

//;elite1k-520016c-JSS-01 #define NR_PORTS             2     /* UART0*/
#define NR_PORTS	3	//elite1k-520016c-JSS-01
#define ELITE_ISR_PASS_LIMIT   256

#define ELITE_UART_IOCTL_BASE       'S'
#define ELITE_UART_DISABLE_OUTPUT      _IOW(ELITE_UART_IOCTL_BASE, 14,  unsigned long)


struct elite_port {
	struct uart_port	port;
	struct timer_list	timer;
	unsigned int		old_status;
	unsigned long		disable_output;
	struct clk			*clk;
};

struct baud_info_s {
	unsigned int baud;		/* baud rate */
	unsigned int brd;		/* baud rate divisor */
	unsigned int bcv;		/* break counter value at this baud rate
							 * simply be calculated by baud * 0.004096
							 */
};

static struct baud_info_s baud_table[] =
{
	{   3600,  0x100FF,    15 },
	{   7600,  0x1007F,    30 },
	{   9600,  0x2003F,    39 },
	{  14400,  0x1003F,    59 },
	{  19200,  0x2001F,    79 },
	{  28800,  0x1001F,   118 },
	{  38400,  0x2000F,   157 },
	{  57600,  0x1000F,   236 },
	{ 115200,   0x10007,   472 },
	{ 230400,   0x10003,   944 },
	{ 460800,   0x10001,  1920 },
	{ 921600,   0x10000,  3775 },
};

#define BAUD_TABLE_SIZE                 ARRAY_SIZE(baud_table)

/*
 * Macros to put URISR and URUSR into a 32-bit status variable
 * URISR in bit[ 0:15]
 * URUSR in bit[16:31]
 */
#define URISR_TO_SM(x)                  ((x) & URISR_MASK)
#define URUSR_TO_SM(x)                  (((x) & URUSR_MASK) << 16)
#define SM_TO_URISR(x)                  ((x) & 0xffff)
#define SM_TO_URUSR(x)                  ((x) >> 16)

/*
 * Following is a trick if we're interesting to listen break signal,
 * but due to ELITE UART doesn't suppout this interrupt status.
 * So I make a fake interrupt status and use URISR_FER event to implement
 * break signal detect.
 */
#ifdef CONFIG_SERIAL_ELITE_BKSIG
#define SW_BKSIG                        (BIT31 | URISR_FER)
#endif
/*
 * This is the size of our serial port register set.
 */
#define UART_PORT_SIZE  0x1040

/*
 * This determines how often we check the modem status signals
 * for any change.  They generally aren't connected to an IRQ
 * so we have to poll them.  We also check immediately before
 * filling the TX fifo incase CTS has been dropped.
 */
#define MCTRL_TIMEOUT   (250*HZ/1000)

static inline unsigned char uart_readb(struct elite_port *t, unsigned long reg)
{
	return readb(t->port.membase + reg);
}

static inline void uart_writeb(struct elite_port *t, unsigned char val, unsigned long reg)
{
	writeb(val, t->port.membase + reg);
}

static inline unsigned short uart_readw(struct elite_port *t, unsigned long reg)
{
	return readw(t->port.membase + reg);
}

static inline void uart_writew(struct elite_port *t, unsigned short val, unsigned long reg)
{
	return writew(val, t->port.membase + reg);
}

static inline unsigned int uart_readl(struct elite_port *t, unsigned long reg)
{
	return readl(t->port.membase + reg);
}

static inline void uart_writel(struct elite_port *t, unsigned int val, unsigned long reg)
{
	writel(val, t->port.membase + reg);
} 



static void elite_tx_chars(struct elite_port *t);

static void elite_mctrl_check(struct elite_port *t)
{
	unsigned int status, changed;

	status = t->port.ops->get_mctrl(&t->port);
	changed = status ^ t->old_status;

	if (changed == 0)
		return;

	t->old_status = status;

	if (changed & TIOCM_RI)
		t->port.icount.rng++;
	if (changed & TIOCM_DSR)
		t->port.icount.dsr++;
	if (changed & TIOCM_CAR)
		uart_handle_dcd_change(&t->port, status & TIOCM_CAR);
	if (changed & TIOCM_CTS)
		uart_handle_cts_change(&t->port, status & TIOCM_CTS);

	wake_up_interruptible(&t->port.state->port.delta_msr_wait);
}



/*
 * This is our per-port timeout handler, for checking the
 * modem status signals.
 */
static void elite_timeout(unsigned long data)
{
	struct elite_port *t = (struct elite_port *)data;
	unsigned long flags;

	if (t->port.state) {
		spin_lock_irqsave(&t->port.lock, flags);
		elite_mctrl_check(t);
		spin_unlock_irqrestore(&t->port.lock, flags);
		mod_timer(&t->timer, jiffies + MCTRL_TIMEOUT);
	}
}

/*
 * Interrupts should be disabled on entry.
 */


static void elite_stop_tx(struct uart_port *port)
{
	struct elite_port *t = (struct elite_port *)port;

	unsigned int urier;

	urier = uart_readl(t, URIER);

	urier &= ~(URIER_ETXFAE | URIER_ETXFE);

	t->port.read_status_mask &= ~URISR_TO_SM(URISR_TXFAE | URISR_TXFE);

	uart_writel(t, urier, URIER);
}

/*
 * Interrupts may not be disabled on entry.
 */

static void elite_start_tx(struct uart_port *port)
{
	struct elite_port *t = (struct elite_port *)port;
	unsigned int urlcr;
	unsigned int urier;
	
	urlcr = uart_readl(t, URLCR);
	urier = uart_readl(t, URIER);

#ifdef CONFIG_SERIAL_ELITE_DUAL_DMA
    	urlcr |= URLCR_DMAEN;
	uart_writel(t, urlcr, URLCR);
#endif
	urier &= ~(URIER_ETXFAE | URIER_ETXFE);
	uart_writel(t, urier, URIER);
	
	elite_tx_chars(t);

	t->port.read_status_mask |= URISR_TO_SM(URISR_TXFAE | URISR_TXFE);

	urier = uart_readl(t, URIER);

	urier |= URIER_ETXFAE | URIER_ETXFE;

	uart_writel(t, urier, URIER);
}


static void elite_stop_rx(struct uart_port *port)
{
	struct elite_port *t = (struct elite_port *)port;
	unsigned int urier;

	urier = uart_readl(t, URIER);
	urier &= ~URIER_ERXFAF;
	uart_writel(t, urier, URIER);
}

static void elite_enable_ms(struct uart_port *port)
{
	struct elite_port *t = (struct elite_port *)port;
	
	mod_timer(&t->timer, jiffies);
}

/*
 * Inside the UART interrupt service routine dut to following
 * reason:
 *
 * URISR_RXFAF:  RX FIFO almost full (FIFO mode)
 * URISR_RXDF:   RX data register full (Register mode)
 * URISR_RXTOUT: RX timeout
 */

#define URRXFIFO 0x1020
static void elite_rx_chars(struct elite_port *t,  unsigned int status)
{
	struct tty_struct *tty = t->port.state->port.tty;
	unsigned int  flg, urfidx, rxfidx, ignored = 0;
	int ch;
	
	urfidx = uart_readl(t, URFIDX);
	rxfidx = URFIDX_RXFIDX(uart_readl(t, URFIDX));

	/*
	 * Check if there is data ready to be read.
	 *
	 * Note: We only receive characters.
	 */
	while ((status & URUSR_TO_SM(URUSR_RXDRDY)) && (rxfidx)) {
		ch = (unsigned int)(uart_readw(t, URRXFIFO) & 0x3FF);

		t->port.icount.rx++;

		flg = TTY_NORMAL;

		/*
		 * Check interrupt status information using status[URISR_bits].
		 *
		 * Notice that the error handling code is out of
		 * the main execution path and the URISR has already
		 * been read by ISR.
		 */
		if (status & URISR_TO_SM(URISR_PER | URISR_FER | URISR_RXDOVR)) {
			if (rxfidx > 1) {
				if (uart_handle_sysrq_char(&t->port, ch))
					goto ignore_char;
				goto error_return;
			} else {
				goto handle_error;
			}
		}
 
		if (uart_handle_sysrq_char(&t->port, ch))
			goto ignore_char;

error_return:

		uart_insert_char(&t->port, (status & 0xFFFF), URISR_TO_SM(URISR_RXDOVR) , ch, flg);

ignore_char:
		status &= 0xffff;       /* Keep URISR field*/
		status |= URUSR_TO_SM(uart_readl(t, URUSR));
		rxfidx = URFIDX_RXFIDX(uart_readl(t, URFIDX));
	}
out:
	tty_flip_buffer_push(tty);

	return;

handle_error:
	/*
	 * Update error counters.
	 */
	if (status & URISR_TO_SM(URISR_PER))
		t->port.icount.parity++;
	else if (status & URISR_TO_SM(URISR_FER)) {

#ifdef CONFIG_SERIAL_ELITE_BKSIG
	/*
	 * Experimental software patch for break signal detection.
	 *
	 * When I got there is a frame error in next frame data,
	 * I check the next data to judge if it is a break signal.
	 *
	 * FIXME: Open these if Bluetooth or IrDA need this patch.
	 *        Dec.29.2004 by Harry.
	 */
		if ((ch & RX_PERMASK) == 0) {
			t->port.icount.brk++;
			uart_handle_break(&t->port);
		} else
			t->port.icount.frame++;

#else   /* Don't support break sinal detection */

		t->port.icount.frame++;

#endif

	}

	/*
	 * RX Over Run event
	 */
	if (status & URISR_TO_SM(URISR_RXDOVR))
		t->port.icount.overrun++;

	if (status & t->port.ignore_status_mask) {
		if (++ignored > 100)
			goto out;
		goto ignore_char;
	}

	/*
	 * Second, handle the events which we're interesting to listen.
	 */
	status &= t->port.read_status_mask;

	if (status & URISR_TO_SM(URISR_PER))
		flg = TTY_PARITY;
	else if (status & URISR_TO_SM(URISR_FER)) {

#ifdef CONFIG_SERIAL_ELITE_BKSIG
	/* Software patch for break signal detection.
	 *
	 * When I got there is a frame error in next frame data,
	 * I check the next data to judge if it is a break signal.
	 *
	 * FIXME: Open these if Bluetooth or IrDA need this patch.
	 *        Dec.29.2004 by Harry.
	 */
		if (t->port.read_status_mask & SW_BKSIG) {
			if ((ch & RX_PERMASK) == 0) {
				DEBUG_INTR("handling break....");
				flg = TTY_BREAK;
				/*goto error_return;*/
			} else {
				flg = TTY_FRAME;
				/*goto error_return;*/
			}
		} else {
			flg = TTY_FRAME;
			/*goto error_return;*/
		}

#else   /* Don't support break sinal detection */

		flg = TTY_FRAME;

#endif
	}

	if (status & URISR_TO_SM(URISR_RXDOVR)) {
		/*
		 * Overrun is special, since it's reported
		 * immediately, and doesn't affect the current
		 * character.
		 */

		ch = 0;
		flg	= TTY_OVERRUN;
	}

#ifdef SUPPORT_SYSRQ
	t->port.sysrq = 0;
#endif
	goto error_return;
}


/*
 * Inside the UART interrupt service routine dut to following
 * reason:
 *
 * URISR_TXFAE: TX FIFO almost empty (FIFO mode)
 * URISR_TXFE:  TX FIFO empty(FIFO mode)
 */
#define URTXFIFO 0x1000
static void elite_tx_chars(struct elite_port *t)
{
	struct circ_buf *xmit = &t->port.state->xmit;

	if (t->port.x_char) {
		uart_writeb(t, t->port.x_char, URRXFIFO);
		t->port.icount.tx++;
		t->port.x_char = 0;
		return;
	}

	elite_mctrl_check(t);

	if (uart_circ_empty(xmit) || uart_tx_stopped(&t->port)) {
		elite_stop_tx(&t->port);
		return;
	}

	while ((uart_readl(t, URFIDX) & 0x1F) < 16) {
		if (uart_circ_empty(xmit))
			break;

		if (uart_readl(t, URUSR) & URUSR_TXDBSY)
			continue;
		uart_writeb(t, xmit->buf[xmit->tail], URTXFIFO);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		t->port.icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&t->port);

	if (uart_circ_empty(xmit))
		elite_stop_tx(&t->port);
}


static irqreturn_t elite_int(int irq, void *dev_id)
{
	struct elite_port *t = (struct elite_port*)dev_id;
	unsigned int status, pass_counter = 0;
	unsigned int urisr, urusr;

	spin_lock(&t->port.lock);
	/*
	 * Put interrupts status information to status bit[0:15]
	 * Put UART status register to status bit[16:31].
	 */
	urisr = uart_readl(t, URISR);
	urusr = uart_readl(t, URUSR);

	status = URISR_TO_SM(urisr) | URUSR_TO_SM(urusr);
	urisr |= SM_TO_URISR(status);
	uart_writel(t, urisr, URISR);
	
	do {
		/*
		 * First, we handle RX events.
		 *
		 * RX FIFO Almost Full.         (URUSR_RXFAF)
		 * RX Timeout.                  (URISR_RXTOUT)
		 * Frame error                  (URISR_FER)
		 *
		 * Note that also allow URISR_FER and URISR_PER event to do rx.
		 */

		if (status & URISR_TO_SM(URISR_RXFAF | URISR_RXFF | URISR_RXTOUT | URISR_PER | URISR_FER)) {
			elite_rx_chars(t,  status);
		}
		/*
		 * Second, we handle TX events.
		 *
		 * If there comes a TX FIFO Almost event, try to fill TX FIFO.
		 */
		 
		elite_tx_chars(t);
	
		if (pass_counter++ > ELITE_ISR_PASS_LIMIT)
			break;

		/*
		 * Update UART interrupt status and general status information.
		 */
		urisr = uart_readl(t, URISR);
		urusr = uart_readl(t, URUSR);
		status = (URISR_TO_SM(urisr) | URUSR_TO_SM(urusr));
		urisr |= SM_TO_URISR(status);
		uart_writel(t, urisr, URISR);

		/*
		 * Inside the loop, we handle events that we're interesting.
		 */
		status &= t->port.read_status_mask;

		/*
		 * Continue loop while following condition:
		 *
		 * TX FIFO Almost Empty.        (URISR_TXFAE)
		 * RX FIFO Almost Full.         (URISR_RXFAF)
		 * RX Receive Time Out.         (URISR_RXTOUT)
		 */
	} while (status & (URISR_TXFE | URISR_TXFAE | URISR_RXFAF | URISR_RXFF | URISR_RXTOUT));

	spin_unlock(&t->port.lock);
	
	return IRQ_HANDLED;
}


/* elite_tx_empty()
 *
 * Return TIOCSER_TEMT when transmitter is not busy.
 */


static unsigned int elite_tx_empty(struct uart_port *port)
{
	struct elite_port *t = (struct elite_port *)port;

	return (uart_readl(t, URUSR) & URUSR_TXDBSY) ? 0 : TIOCSER_TEMT;
}

/* elite_get_mctrl()
 *
 * Returns the current state of modem control inputs.
 *
 * Note: Only support CTS now.
 */
static unsigned int elite_get_mctrl(struct uart_port *port)
{
	unsigned int ret = TIOCM_DSR | TIOCM_CAR;
	struct elite_port *t = (struct elite_port *)port;
	unsigned int urusr;

	urusr = uart_readl(t, URUSR);
	ret |= (urusr & URUSR_CTS) ? TIOCM_CTS : 0;
	
	return ret;
}



/* elite_set_mctrl()
 *
 * This function sets the modem control lines for port described
 * by 'port' to the state described by mctrl. More detail please
 * refer to Documentation/serial/driver.
 *
 * Note: Only support RTS now.
 */

static void elite_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct elite_port *t = (struct elite_port *)port;
	
	unsigned int urlcr;

	urlcr = uart_readl(t, URLCR);

	if (mctrl & TIOCM_RTS)
		urlcr |= URLCR_RTS;
	else
		urlcr &= ~URLCR_RTS;

	uart_writel(t, urlcr, URLCR);
}


/*
 * Interrupts always disabled.
 */


static void elite_break_ctl(struct uart_port *port, int break_state)
{
	struct elite_port *t = (struct elite_port *)port;
	unsigned long flags;
	
	spin_lock_irqsave(&t->port.lock, flags);

	if (break_state == -1) {
		int i;
		unsigned int urdiv;
		unsigned int urbrd;
		urdiv = uart_readl(t, URDIV);
		urbrd = URBRD_BRD(urdiv);

		/*
		 * This looks something tricky.
		 * Anyway, we need to get current baud rate divisor,
		 * search bcv in baud_table[], program it into
		 * URBKR, then generate break signal.
		 */
		for (i = 0; i < BAUD_TABLE_SIZE; i++) {
			if ((baud_table[i].brd & URBRD_BRDMASK) == urbrd)
				break;
		}

		if (i < BAUD_TABLE_SIZE) {
			unsigned int urlcr;
			uart_writel(t, URBKR_BCV(baud_table[i].bcv), URBKR);
			urlcr = uart_readl(t, URLCR);
			urlcr |= URLCR_BKINIT;
			uart_writel(t, urlcr, URLCR);
		}
	}

	spin_unlock_irqrestore(&t->port.lock, flags);
}


static char *elite_uartname[] = {
	"uart0",
	"uart1",
	"uart2",
	"uart3"
};

static int elite_startup(struct uart_port *port)
{
	struct elite_port *t = (struct elite_port *)port;
	
	unsigned int urfcr;
	unsigned int urlcr;
	
	char *uartname = NULL;
	int retval;
	/*unsigned int tmp_irisr;*/
	int i;


	switch (t->port.irq) {

	case IRQ_UART0:
		uartname = elite_uartname[0];
		break;
	default:
		break;
	}
	
	/*
	 * Setup the UART clock divisor
	 */
	for (i = 0; i < BAUD_TABLE_SIZE; i++) {
		if (baud_table[i].baud == 115200)
			break;
	}

	uart_writel(t,  baud_table[i].brd, URDIV);
	
	/* Disable TX,RX*/
	uart_writel(t, 0, URLCR);
	/* Disable all interrupt*/
	uart_writel(t, 0, URIER);

	/*Reset TX,RX Fifo*/
	uart_writel(t, URFCR_TXFRST | URFCR_RXFRST, URFCR);
	
	/* Disable Fifo*/
	urfcr = uart_readl(t, URFCR);
	urfcr &= ~(URFCR_FIFOEN);
	uart_writel(t, urfcr, URFCR);

	urlcr = uart_readl(t, URLCR);
	urlcr |=  (URLCR_DLEN & ~URLCR_STBLEN & ~URLCR_PTYEN);
	uart_writel(t, urlcr, URLCR);

	/* Enable Fifo, Tx 16 , Rx 16*/
	urfcr = URFCR_FIFOEN | URFCR_TXFLV(8) | URFCR_RXFLV(8);

	uart_writel(t, urfcr, URFCR);

	/* Enable Fifo, Tx 8 , Rx 8*/
	urlcr = uart_readl(t, URLCR);
	urlcr |= URLCR_RXEN | URLCR_TXEN;
	uart_writel(t, urlcr, URLCR);
	/*
	 * Enable RX FIFO almost full, timeout, and overrun interrupts.
	 */
	uart_writel(t, URIER_ERXFAF | URIER_ERXFF | URIER_ERXTOUT | URIER_EPER | URIER_EFER | URIER_ERXDOVR, URIER);

	/*
	 * Enable modem status interrupts
	 */
	spin_lock_irq(&t->port.lock);
	elite_enable_ms(&t->port);
	spin_unlock_irq(&t->port.lock);
	retval = request_irq(t->port.irq, elite_int, 0, uartname, t);
	if (retval)
		return retval;
	return 0;
}

static void elite_shutdown(struct uart_port *port)
{
	struct elite_port *t = (struct elite_port *)port;
	
	unsigned int urier;

	del_timer_sync(&t->timer);

	free_irq(t->port.irq, t);
	urier = uart_readl(t, URIER);
	urier &= ~(URIER_ETXFE | URIER_ETXFAE | URIER_ERXFF | URIER_ERXFAF);
	uart_writel(t, urier, URIER);
}



/* elite_uart_pm()
 *
 * Switch on/off uart in powersave mode.
 *
 * Hint: Identify port by irq number.
 */
static void elite_uart_pm(struct uart_port *port, u_int state, u_int oldstate)
{
}

static void elite_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
	struct elite_port *t = (struct elite_port *)port;
	unsigned long flags;
	unsigned int new_urlcr, old_urlcr, old_urier, tmp_urisr, baud;
	unsigned int old_csize = old ? old->c_cflag & CSIZE : CS8;
	int i;
	unsigned int urlcr;
	
	/*
	 * If we don't support modem control lines, don't allow
	 * these to be set.
	 */
	if (0) {
		termios->c_cflag &= ~(HUPCL | CRTSCTS | CMSPAR);
		termios->c_cflag |= CLOCAL;
	}
	/*
	 * Only support CS7 and CS8.
	 */
	while ((termios->c_cflag & CSIZE) != CS7 && (termios->c_cflag & CSIZE) != CS8) {
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= old_csize;
		old_csize = CS8;
	}

	if ((termios->c_cflag & CSIZE) == CS8)
		new_urlcr = URLCR_DLEN;
	else
		new_urlcr = 0;

	if (termios->c_cflag & CRTSCTS)
		new_urlcr |= URLCR_RTS;
	else
		new_urlcr &= ~URLCR_RTS;

	if (termios->c_cflag & CSTOPB)
		new_urlcr |= URLCR_STBLEN;

	if (termios->c_cflag & PARENB) {
		/*
		 * Enable parity.
		 */
		new_urlcr |= URLCR_PTYEN;

		/*
		 * Parity mode select.
		 */
		if (termios->c_cflag & PARODD)
			new_urlcr |= URLCR_PTYMODE;
	}

	/*
	 * Ask the core to get baud rate, but we need to
	 * calculate quot by ourself.
	 */
	baud = uart_get_baud_rate(port, termios, old, 9600, 921000);

	/*
	 * We need to calculate quot by ourself.
	 *
	 * FIXME: Be careful, following result is not an
	 *        interger quotient, fix it if need.
	 */
	/*quot = port->uartclk / (13 * baud);*/

	spin_lock_irqsave(&t->port.lock, flags);

	/*
	 * Mask out other interesting to listen expect TX FIFO almost empty event.
	 */
	t->port.read_status_mask &= URISR_TO_SM(URISR_TXFAE | URISR_TXFE);

	/*
	 * We're also interested in receiving RX FIFO events.
	 */
	t->port.read_status_mask |= URISR_TO_SM(URISR_RXDOVR | URISR_RXFAF | URISR_RXFF);

	/*
	 * Check if we need to enable frame and parity error events
	 * to be passed to the TTY layer.
	 */
	if (termios->c_iflag & INPCK)
		t->port.read_status_mask |= URISR_TO_SM(URISR_FER | URISR_PER);

#ifdef CONFIG_SERIAL_ELITE_BKSIG
	/*
	 * check if we need to enable break events to be passed to the TTY layer.
	 */
	if (termios->c_iflag & (BRKINT | PARMRK))
		/*
		 * ELITE UART doesn't support break signal detection interrupt.
		 *
		 * I try to implement this using URISR_FER.
		 */
		t->port.read_status_mask |= SW_BKSIG;
#endif
	/*
	 * Characters to ignore
	 */
	t->port.ignore_status_mask = 0;

	if (termios->c_iflag & IGNPAR)
		t->port.ignore_status_mask |= URISR_TO_SM(URISR_FER | URISR_PER);

	if (termios->c_iflag & IGNBRK) {
#ifdef CONFIG_SERIAL_ELITE_BKSIG
		/*
		 * ELITE UART doesn't support break signal detection interrupt.
		 *
		 * I try to implement this using URISR_FER.
		 */
		t->port.ignore_status_mask |= BIT31;/*FIXME*/
#endif

		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			t->port.ignore_status_mask |= URISR_TO_SM(URISR_RXDOVR);
	}

	//del_timer_sync(&t->timer);

	/*
	 * Update the per-port timeout.
	 */
	 uart_update_timeout(port, termios->c_cflag, baud);

	/*
	 * Disable FIFO request interrupts and drain transmitter
	 */
	old_urlcr = uart_readl(t, URLCR);
	old_urier = uart_readl(t, URIER);
	uart_writel(t, old_urier & ~(URIER_ETXFAE | URIER_ERXFAF), URIER);

	/*
	 * Two step polling, first step polling the remaining
	 * entries in TX FIFO. This step make it safe to drain
	 * out all of remaining data in FIFO.
	 */
	while (URFIDX_TXFIDX(uart_readl(t, URFIDX)))
		barrier();

	/*
	 * Second step to make sure the last one data has been sent.
	 */
	while (uart_readl(t, URUSR) & URUSR_TXDBSY)
		barrier();

	/*
	 * Disable this UART port.
	 */
	uart_writel(t, 0, URIER);

	/*
	 * Set the parity, stop bits and data size
	 */
	uart_writel(t, new_urlcr, URLCR);

	/*
	 * Set baud rate
	 */
	/*quot -= 1;*/
	/*uart->urdiv = quot & 0xff;*/
	for (i = 0; i < BAUD_TABLE_SIZE; i++) {
		if (baud_table[i].baud == baud)
			break;
	}
	uart_writel(t, baud_table[i].brd, URDIV);
	/*
	 * Read to clean any pending pulse interrupts.
	 */
	tmp_urisr = uart_readl(t, URISR);

	/*
	 * Restore FIFO interrupt, TXEN bit, RXEN bit settings.
	 */
	uart_writel(t, old_urier, URIER);

	urlcr = uart_readl(t, URLCR);

#ifdef CONFIG_SERIAL_ELITE_DUAL_DMA
	urlcr |= old_urlcr & (URLCR_TXEN | URLCR_RXEN | URLCR_DMAEN);
#else
	urlcr |= old_urlcr & (URLCR_TXEN | URLCR_RXEN);
#endif	

	uart_writel(t, urlcr, URLCR);

	if (UART_ENABLE_MS(&t->port, termios->c_cflag))
		elite_enable_ms(&t->port);

	spin_unlock_irqrestore(&t->port.lock, flags);

}


static const char *elite_type(struct uart_port *port)
{
	struct elite_port *t = (struct elite_port *)port;
	
	return (t->port.type == PORT_ELITE) ? "elite serial" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void elite_release_port(struct uart_port *port)
{
	struct elite_port *t = (struct elite_port *)port;

	release_mem_region(t->port.mapbase, UART_PORT_SIZE);
}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int elite_request_port(struct uart_port *port)
{
	struct elite_port *t = (struct elite_port *)port;

	return request_mem_region(t->port.mapbase, UART_PORT_SIZE, "uart") != NULL ? 0 : -EBUSY;
}

/*
 * Configure/autoconfigure the port.
 */
static void elite_config_port(struct uart_port *port, int flags)
{
	struct elite_port *sport = (struct elite_port *)port;

	if (flags & UART_CONFIG_TYPE && elite_request_port(&sport->port) == 0)
		sport->port.type = PORT_ELITE;
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 * The only change we allow are to the flags and type, and
 * even then only between PORT_ELITE and PORT_UNKNOWN
 */
static int elite_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct elite_port *sport = (struct elite_port *)port;
	int ret = 0;

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_ELITE)
		ret = -EINVAL;
	if (sport->port.irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != SERIAL_IO_MEM)
		ret = -EINVAL;
	if (sport->port.uartclk / 16 != ser->baud_base)
		ret = -EINVAL;

	if ((void *)sport->port.mapbase != ser->iomem_base)
		ret = -EINVAL;
	if (sport->port.iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}

#ifdef CONFIG_CONSOLE_POLL
static void elite_poll_put_char(struct uart_port *port, unsigned char c)
{
	struct elite_uart_port *t = (struct elite_uart_port *)port;
	unsigned int old_urier;
	
	old_urier = uart_readl(t, URIER);
	uart_writel(t, 0, URIER);

	while (uart_readl(t, URUSR) & URUSR_TXDBSY);

	if (c == '\n') {
		uart_writeb(t, '\r', URTDR);
	}

	while (uart_readl(t, URUSR) & URUSR_TXDBSY);

	uart_writel(t, old_urier, URIER);
}

static int elite_poll_get_char(struct uart_port *port)
{
	struct elite_uart_port *t = (struct elite_uart_port *)port;

	if (!(uart_readl(t, URUSR) & URUSR_RXDRDY)) {
		return NO_POLL_CHAR;
	}

	return uart_readl(t, URRDR)& 0xff;
}

#endif

static int	elite_uart_ioctl(struct uart_port * port , unsigned int cmd, unsigned long data)
{
	struct elite_port *sport = (struct elite_port *)port;
	void __user *argp = (void __user *)data;

	switch (cmd) {
		case ELITE_UART_DISABLE_OUTPUT: {
			unsigned long disable_output = 0;
			unsigned long ret = 0;			
  			

			disable_output=data;
			printk("enter elite_uart_ioctl disable_output:%d\n",disable_output);
			sport->disable_output = disable_output;
			return  0;
		}
		break;
		default:
			break;
	}

	return -ENOIOCTLCMD;

}
static struct uart_ops elite_pops = {
	.tx_empty 		= elite_tx_empty,
	.set_mctrl		= elite_set_mctrl,
	.get_mctrl		= elite_get_mctrl,
	.stop_tx		= elite_stop_tx,
	.start_tx		= elite_start_tx,
	.stop_rx		= elite_stop_rx,
	.enable_ms		= elite_enable_ms,
	.break_ctl		= elite_break_ctl,
	.startup		= elite_startup,
	.shutdown		= elite_shutdown,
	.pm				= elite_uart_pm,
	.set_termios	= elite_set_termios,
	.type			= elite_type,
	.release_port	= elite_release_port,
	.request_port	= elite_request_port,
	.config_port	= elite_config_port,
	.verify_port	= elite_verify_port,
	.ioctl          = elite_uart_ioctl,
#ifdef CONFIG_CONSOLE_POLL
	.poll_put_char = elite_poll_put_char,
	.poll_get_char = elite_poll_get_char,
#endif
};

static struct elite_port elite_ports[NR_PORTS];

/* Setup the ELITE serial ports.  Note that we don't include the IrDA
 * port here since we have our own SIR/FIR driver (see drivers/net/irda)
 *
 * Note also that we support "console=ttyVTx" where "x" is either 0 to 2.
 * Which serial port this ends up being depends on the machine you're
 * running this kernel on.
 */
static void elite_init_ports(void)
{
	static int first = 1;
	int i;

	if (!first)
		return;

	first = 0;

	for (i = 0; i < NR_PORTS; i++) {
		elite_ports[i].port.uartclk    = 24000000;
		elite_ports[i].port.ops        = &elite_pops;
		elite_ports[i].port.fifosize   = 16;
		elite_ports[i].port.line       = i;
		elite_ports[i].port.iotype     = SERIAL_IO_MEM;
		//init_timer(&elite_ports[i].timer);
		//elite_ports[i].timer.function  = elite_timeout;
		//elite_ports[i].timer.data      = (unsigned long)&elite_ports[i];

/* elite1k-520016c-JSS-01 --*/
/*
        #ifdef CONFIG_ARCH_ELITE2000
		elite_ports[i].port.membase = (void *)(IO_ADDRESS(0xd8200000));
		elite_ports[i].port.mapbase = 0xd8200000;
        #else
		elite_ports[i].port.membase = (void *)(IO_ADDRESS(0xd82b0000));
		elite_ports[i].port.mapbase = 0xd82b0000;
        #endif
		elite_ports[i].port.irq     = 107;
*/
/* elite1k-520016c-JSS-01 --*/

//elite1k-520016c-JSS-01 ++S
            if ( i == 0 ) {
                    elite_ports[i].port.membase = (void *)(IO_ADDRESS(0xd8200000));
                    elite_ports[i].port.mapbase = 0xd8200000;
                    elite_ports[i].port.irq     = 64;
            }
            if ( i == 1 ) {
                    elite_ports[i].port.membase = (void *)(IO_ADDRESS(0xd82b0000));
                    elite_ports[i].port.mapbase = 0xd82b0000;
                    elite_ports[i].port.irq     = 65;
            }
            if ( i == 2 ) {
                    elite_ports[i].port.membase = (void *)(IO_ADDRESS(0xd8210000));
                    elite_ports[i].port.mapbase = 0xd8210000;
                    elite_ports[i].port.irq     = 79;
            }
//elite1k-520016c-JSS-01 ++E
		elite_ports[i].port.flags   = ASYNC_BOOT_AUTOCONF;
	}

	 
}

#ifdef CONFIG_SERIAL_ELITE_UART_CONSOLE

/*
 * Interrupts are disabled on entering
 *
 * Note: We do console writing with UART register mode.
 */
static void elite_console_write(struct console *co, const char *s, u_int count)
{
	struct elite_port *t = &elite_ports[co->index];
	unsigned int i, old_urlcr, old_urier;
	unsigned int old_urfcr;
	unsigned int urier, urlcr, urfcr;
	if(t->disable_output)
	{
	
		return;
	}
	old_urlcr = uart_readl(t, URLCR);
	old_urier = uart_readl(t, URIER);
	old_urfcr = uart_readl(t, URFCR);

	/*
	 * Second, switch to register mode with follows method:
	 *
	 * Disable FIFO threshold interrupts, and enable transmitter.
	 */

	urier = uart_readl(t, URIER);
	urlcr = uart_readl(t, URLCR);
	urfcr = uart_readl(t, URFCR);

	urier &= ~(URIER_ETXFAE | URIER_ERXFAF);
	urlcr |= URLCR_TXEN;
	urfcr &= ~URFCR_FIFOEN;

	uart_writel(t, urier, URIER);
	uart_writel(t, urlcr, URLCR);
	uart_writel(t, urfcr, URFCR);	

	/*
	 * Now, do each character
	 */
	for (i = 0; i < count; i++) {
		/*
		 * Polling until free for transmitting.
		 */
		while (uart_readl(t, URUSR) & URUSR_TXDBSY);

		uart_writel(t, (unsigned int)s[i], URTDR);

		/*
		 * Do CR if there comes a LF.
		 */
		if (s[i] == '\n') {
			/*
			 * Polling until free for transmitting.
			 */
			while (uart_readl(t, URUSR) & URUSR_TXDBSY);

			uart_writel(t, (unsigned int)'\r', URTDR);
		}
	}

	/*
	 * Finally, wait for transmitting done and restore URLCR and URIER.
	 */
	while (uart_readl(t, URUSR) & URUSR_TXDBSY);

	uart_writel(t, old_urlcr, URLCR);
	uart_writel(t, old_urier, URIER);
	uart_writel(t, old_urfcr, URFCR);
}

/*
 * If the port was already initialised (eg, by a boot loader), try to determine
 * the current setup.
 */

static void __init elite_console_get_options(struct elite_port *t, int *baud, int *parity, int *bits)
{
	int i;

	if ((uart_readl(t, URLCR) & (URLCR_RXEN | URLCR_TXEN)) == (URLCR_RXEN | URLCR_TXEN)) {
		/*
		 * Port was enabled.
		 */
		unsigned quot;

		*parity = 'n';
		/*
		 * Check parity mode, 0:evev 1:odd
		 */
		if (uart_readl(t, URLCR) & URLCR_PTYEN) {
			if (uart_readl(t, URLCR) & URLCR_PTYMODE)
				*parity = 'o';
			else
				*parity = 'e';
		}

		/*
		 * Check data length, 0:7-bit 1:8-bit
		 */
		if (uart_readl(t, URLCR) & URLCR_DLEN)
			*bits = 8;
		else
			*bits = 7;

		/*
		 * Get baud rate divisor.
		 */
		quot = (uart_readl(t, URDIV) & URBRD_BRDMASK);
		/*
		 * FIXME: I didn't trace the console driver want me
		 * report baud rate whether actual baud rate or ideal
		 * target baud rate, current I report baud as actual
		 * one, if it need value as target baud rate, just
		 * creat an array to fix it, Dec.23 by Harry.
		 */

		/*printk("%s: quot=%d\n", __FUNCTION__, quot);  // harry0*/
		for (i = 0; i < BAUD_TABLE_SIZE; i++) {
			if ((baud_table[i].brd & URBRD_BRDMASK) == quot) {
				*baud = baud_table[i].baud;
				break;
			}
		}

		/*
		 * If this condition is true, something might be wrong.
		 * I reprot the actual baud rate temporary.
		 * Check the printk information then fix it.
		 */
		if (i >= BAUD_TABLE_SIZE)
			*baud = t->port.uartclk / (13 * (quot + 1));
	}
}



#ifndef CONFIG_ELITE_DEFAULT_BAUDRATE
#define CONFIG_ELITE_DEFAULT_BAUDRATE  115200
#endif

static int __init
elite_console_setup(struct console *co, char *options)
{
	struct elite_port *t;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int baud = CONFIG_ELITE_DEFAULT_BAUDRATE;
	
	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	*/
	if (co->index == -1 || co->index >= NR_PORTS)
		co->index = 0;

	t = &elite_ports[co->index];
	
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		elite_console_get_options(t, &baud, &parity, &bits);
	
	return uart_set_options(&t->port, co, baud, parity, bits, flow);
}

static struct uart_driver elite_uart_driver;

static struct console elite_console = {

#ifdef CONFIG_SERIAL_ELITE_TTYVT
	.name	= "ttyVT",
#else
	.name	= "ttyS",
#endif

	.write 	= elite_console_write,
	.device	= uart_console_device,
	.setup	= elite_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &elite_uart_driver,
};

static int __init elite_rs_console_init(void)
{
	elite_init_ports();
	register_console(&elite_console);
	return 0;
}

console_initcall(elite_rs_console_init);

#define ELITE_CONSOLE  (&elite_console)

#else   /* CONFIG_SERIAL_ELITE_CONSOLE */

#define ELITE_CONSOLE  NULL

#endif

static struct uart_driver elite_uart_driver = {
	.owner          = THIS_MODULE,

#ifdef CONFIG_SERIAL_ELITE_TTYVT
	.driver_name    = "ttyVT",
	.dev_name       = "ttyVT",
#else
	.driver_name    = "ttyS",
	.dev_name       = "ttyS",
#endif
	.major          = SERIAL_ELITE_MAJOR,
	.minor          = MINOR_START,
	.nr             = NR_PORTS,
	.cons           = ELITE_CONSOLE,
};


#ifdef CONFIG_PM_SLEEP
static int elite_serial_suspend(struct device *dev)
{
	struct elite_port *t = dev_get_drvdata(dev);

	if(!t)
		return 0;

	uart_suspend_port(&elite_uart_driver, &t->port);

	return 0;
}

static int elite_serial_resume(struct device *dev)
{
	struct elite_port *t = dev_get_drvdata(dev);

	 if(!t)
		return 0;

	
	uart_resume_port(&elite_uart_driver, &t->port);


	return 0;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int elite_serial_runtime_suspend(struct device *dev)
{
	return 0;
}
static int elite_serial_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static int elite_serial_probe(struct platform_device *pdev)
{
	struct resource *resource;
	struct elite_port *t;
	struct uart_port *u;
	int ret;

	if (pdev->dev.of_node)
		ret = of_alias_get_id(pdev->dev.of_node, "serial");
	else {
        	if (pdev->id < 0 || pdev->id > NR_PORTS) {
                	return -ENODEV;
        	}
		ret = pdev->id;
	}
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get alias/pdev id, errno %d\n",
								ret);
		return -ENODEV;
	}
	t = &elite_ports[ret];

#ifndef CONFIG_ARCH_ELITE2000 
	t->clk = clk_get(&pdev->dev, "uart");
	if (IS_ERR(t->clk)) {
		ret = PTR_ERR(t->clk);
		t->clk = NULL;
		dev_err(&pdev->dev, "failed to get uart clock\n");
		return ret;
	}
	/* Enable the peripheral clock */
	clk_prepare_enable(t->clk);	
#endif
	u = &t->port;
	u->dev = &pdev->dev;
	platform_set_drvdata(pdev, u);
	u->line = ret;
	u->ops = &elite_pops;
	u->fifosize = 16;
	u->uartclk = 24000000;//clk_get_rate(t->clk);
	u->iotype = SERIAL_IO_MEM;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!resource)) {
		ret = -ENXIO;
		goto fr_clk;
	}


	u->mapbase = resource->start;
	u->membase = IO_ADDRESS(u->mapbase);
	if (unlikely(!u->membase)) {
		ret = -ENOMEM;
		goto fr_clk;
	}

	u->irq = platform_get_irq(pdev, 0);
	if (unlikely(u->irq < 0)) {
		ret = -ENXIO;
		goto fr_clk;
	}

	ret = uart_add_one_port(&elite_uart_driver, u);
	if (ret) {
		printk("failed to add uart port %d\n", u->line);
		platform_set_drvdata(pdev, NULL);
		goto fr_clk;
	}

	init_timer(&t->timer);
	t->timer.function = elite_timeout;
	t->timer.data = (unsigned long)t;
	return 0;

fr_clk:
	clk_put(t->clk);
	return ret;
}



static int elite_serial_remove(struct platform_device *pdev)
{
	struct elite_port *t = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if (t)
		uart_remove_one_port(&elite_uart_driver, &t->port);

	return 0;
}

static const struct dev_pm_ops elite_serial_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(elite_serial_suspend, elite_serial_resume)
	SET_RUNTIME_PM_OPS(elite_serial_runtime_suspend,
				elite_serial_runtime_resume, NULL)
};

#if defined(CONFIG_OF)
static const struct of_device_id elite_serial_of_match[] = {
	{ .compatible = "s3graphics,elite1000-uart", .data = NULL },
	{ .compatible = "s3graphics,elite2000-uart", .data = NULL },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, elite_serial_of_match);
#endif

static struct platform_device_id elite_serial_driver_ids[] = {
	{
		.name		= "elite-uart",
		.driver_data	= (kernel_ulong_t)NULL,
	}, {
		.name		= "elite-uart.0",
		.driver_data	= (kernel_ulong_t)NULL,
	}, {
		.name		= "elite-uart.1",
		.driver_data	= (kernel_ulong_t)NULL,
	
	},
	{ },
};
MODULE_DEVICE_TABLE(platform, elite_serial_driver_ids);

static struct platform_driver elite_serial_driver = {
	.driver = {
		.name    = "s3graphics-elite-uart",
		.owner = THIS_MODULE,
		.pm	= &elite_serial_dev_pm_ops,
		.of_match_table = of_match_ptr(elite_serial_of_match),
	},
	.id_table		= elite_serial_driver_ids,
	.probe          = elite_serial_probe,
	.remove         = elite_serial_remove,
};

static int __init elite_serial_init(void)
{
	int ret;

	ret = uart_register_driver(&elite_uart_driver);

	if (unlikely(ret)) {
		printk("could not register elite uart driver!\n");
		return ret;
	}

	ret = platform_driver_register(&elite_serial_driver);

	if (unlikely(ret)) {
		printk("could not register elite serial platform driver!\n");
		uart_unregister_driver(&elite_uart_driver);
		return ret;
	}

	return 0;
}

static void __exit elite_serial_exit(void)
{
	platform_driver_unregister(&elite_serial_driver);

	uart_unregister_driver(&elite_uart_driver);
}


module_init(elite_serial_init);
module_exit(elite_serial_exit);

MODULE_ALIAS("platform:s3graphics-uart");
MODULE_AUTHOR("S3 Graphics, Inc.");
MODULE_DESCRIPTION("ELITE [generic serial port] driver");
MODULE_LICENSE("GPL");

