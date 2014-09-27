/*++
drivers/spi/elite-spi.c

Some descriptions of such software. Copyright (c) 2008  WonderMedia Technologies, Inc.
This program is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software Foundation,
either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.

History:
--*/

#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/cache.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/bug.h>
#include <linux/ptrace.h>
#include <asm/io.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/ioport.h>
#include <linux/semaphore.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <asm/dma.h>

#include "spi-elite.h"

#define SPI_DEBUG

#ifdef SPI_DEBUG
#define SPI_PRINTF(fmt, args...)   printk(KERN_NOTICE"%s: " fmt, __func__ , ## args)      
#define ENTER() printk(KERN_NOTICE"Enter %s, file:%s line:%d\n", __func__, __FILE__, __LINE__)    
#define LEAVE() printk(KERN_NOTICE"Exit %s, file:%s line:%d\n", __func__, __FILE__, __LINE__)   
#else
#define SPI_PRINTF(fmt, args...)   
#define ENTER() 
#define LEAVE()   
#endif

/*dma allocate size*/
#define SPI_MAX_XFER_SIZE	  512

/*dma allocate size: 4KB*/
#define SPI_DMA_SIZE  4096	
#define SPI_DMA_CHUNK_SIZE  32

/*appear in /proc/devices & /proc/elite_spi*/
#define DEVICE_NAME         "elite_spi" 

#define SPI0_CLK_DIVISOR_VAL    0x06

#define SPI0_CLK_LOW_DIVISOR_VAL  0

/* user space versions of kernel symbols for spi clocking modes, matching <linux/spi/spi.h>*/
#define SPI_CPHA		0x01
#define SPI_CPOL		0x02

#define SPI_MODE_0		(0|0)
#define SPI_MODE_1		(0|SPI_CPHA)
#define SPI_MODE_2		(SPI_CPOL|0)
#define SPI_MODE_3		(SPI_CPOL|SPI_CPHA)

#define SPI_CS_HIGH		0x04
#define SPI_LSB_FIRST	0x08
#define SPI_3WIRE		0x10
#define SPI_LOOP		0x20
#define SPI_NO_CS		0x40

#define SUSPND    (1<<0)
#define SPIBUSY   (1<<1)
#define RXBUSY    (1<<2)
#define TXBUSY    (1<<3)

#define ETIMEOUT                1
#define EUNKNOWNMODE 2
#define EWRONGSIZE           3

#define USE_TX_DRIVE         0

#define SPI_MAX_RATE   0x500000
#define SPI_MIN_RATE    0x100000
#define MAX_FREQ_DIV  2047

static void spi_user_reset(struct elite_spi_device *device)
{
	ENTER();
	/* check user valid*/
	if (!device) 
	{
		printk(KERN_ERR "[SPI]: null user enter %s.\n", __func__);
		LEAVE();
		return;
	}
	memset(device, 0x0, sizeof(struct elite_spi_device));
	/* setting default frequence is 12Mhz*/
	device->freq = 12000;	
	/* work with polling mode*/
	device->op_mode = SPI_POLLING_MODE;	
	/* SSN Port Mode = Point to point*/
	device->ssn_port_mode = SPI_SSN_PORT_PTP;	
	/* setting TX Drive Output Value = 0xFF*/
	device->tx_nodata_value = 1;	
	/* master mode */
	device->arbiter = SPI_ARBITER_MASTER; 
#if USE_TX_DRIVE
	/* disable tx driver */
	device->tx_drive_enable = 0;          
#endif
	device->tx_drive_count = 0;
	/* clk mode 0 */
	device->clk_mode = 0;                 	
	LEAVE();
	return;
}


static void spi_set_freq(struct elite_spi_device *device, int freq)
{
	struct elite_spi_controller *controller;
	unsigned long cur_rate = 0;

	ENTER();
	if(freq > SPI_MAX_RATE || freq < SPI_MIN_RATE)
	{
		printk(KERN_ERR "this freq %d can't not be set\n", freq);
		LEAVE();
		return;
	}
	controller = device->controller;
	device->freq = freq; 

	LEAVE();
	return;
}

static void spi_set_clk_mode(struct elite_spi_device *device, u8 mode)
{
	enum spi_clk_mode_type clk_mode;
	ENTER();

	if(mode & SPI_CPHA && mode & SPI_CPOL)
	{
		clk_mode = SPI_CLK_MODE_3;
	}
	else if(mode & SPI_CPHA)
	{
		clk_mode = SPI_CLK_MODE_2;
	}
	else if(mode & SPI_CPOL)
	{
		clk_mode = SPI_CLK_MODE_1;
	}
	else 
	{
		clk_mode = SPI_CLK_MODE_0;
	}

	/* check user valid*/
	if (!device) 
	{
		printk(KERN_ERR "[SPI]: null user enter %s.\n", __func__);
		LEAVE();
		return;
	}
	/* set bus clock mode for the use*/
	device->clk_mode = clk_mode; 

	LEAVE();
	return;
}


void spi_set_chipselect(struct elite_spi_device *device, enum spi_ssn_type spi_ssn)
{
	ENTER();

	/* check user valid*/
	if (!device) 
	{
		printk(KERN_ERR "[SPI]: null user enter %s.\n", __func__);
		LEAVE();
		return;
	}
	if ((spi_ssn != SPI_SS0) && (spi_ssn != SPI_SS1) && (spi_ssn != SPI_SS2) && (spi_ssn != SPI_SS3)) 
	{
		printk(KERN_WARNING "[spi_set_chipselect] spi_ssn error, force spi_ssn to SPI_SS0\n");
		device->slave_select = SPI_SS0;
	} 
	else
	{
		/* set bus clock mode for the use*/
		device->slave_select  = spi_ssn; 
	}

	LEAVE();
	return;
}


void spi_set_op_mode(struct elite_spi_device *device, enum spi_operation_mode_type op_mode)
{
	ENTER();

	/* check user valid*/
	if (!device) 
	{
		printk(KERN_ERR "[SPI]: null user enter %s.\n", __func__);
		LEAVE();
		return;
	}
	/* set operation mode for the use*/
	if (op_mode == SPI_INTERRUPT_MODE)
	{
		SPI_PRINTF("[spi_set_op_mode] SPI INTERRUPT MODE\n");
	}
	else if (op_mode == SPI_DMA_MODE)
	{
		SPI_PRINTF("[spi_set_op_mode] SPI DMA MODE\n");
	}
	else if (op_mode == SPI_POLLING_MODE)
	{
		SPI_PRINTF("[spi_set_op_mode] SPI POLLING MODE\n");
	}
	else 
	{
		printk(KERN_ERR"[spi_set_op_mode] SPI operation mode error, force to polling mode\n");
	}
	device->op_mode = op_mode;

	LEAVE();
	return;
}

 void spi_set_arbiter(struct elite_spi_device *device, enum spi_arbiter_type arbiter )
{
	ENTER();

	/* check user valid*/
	if (!device) 
	{
		printk(KERN_ERR "[SPI]: null user enter %s.\n", __func__);
		LEAVE();
		return;
	}
	if ((arbiter != SPI_ARBITER_MASTER) && (arbiter != SPI_ARBITER_SLAVE)) 
	{
		printk(KERN_WARNING "[spi_set_arbiter] arbiter error, force arbiter to master\n");
		device->arbiter = SPI_ARBITER_MASTER;
	}
	/* set bus clock mode for the use*/
	device->arbiter = arbiter; 

	LEAVE();
	return;
}

 void spi_set_port_mode(struct elite_spi_device *device, enum spi_ssn_port_mode_type ssn_port_mode)
{
	ENTER();

	/* check user valid*/
	if (!device) 
	{
		printk(KERN_ERR "[SPI]: null user enter %s.\n", __func__);
		LEAVE();
		return;
	}

	if ((ssn_port_mode != SPI_SSN_PORT_MM) && (ssn_port_mode != SPI_SSN_PORT_PTP)) 
	{
		printk(KERN_WARNING "[spi_set_port_mode] port mode error, force port mode to point to point mode\n");
		device->ssn_port_mode = SPI_SSN_PORT_PTP;
	}
	/* set ssn port mode*/
	device->ssn_port_mode = ssn_port_mode; 

	LEAVE();
	return;
}

int spi_set_reg(struct elite_spi_device *device)
{
	/*ssn default is pull high*/
	unsigned int dataformat =0x0;
	unsigned int control = 0x0;
	unsigned int divisor = 0x0;
	struct elite_spi_controller *controller = device->controller;
	unsigned int spi_input_freq;

	ENTER();

	/* check user and spi port valid*/
	if (!device || !controller) 
	{
		printk(KERN_ERR "[SPI]: invaild user enter %s.\n", __func__);
		LEAVE();
		return -1;
	}

	/* multi-master use, do not care */
	dataformat |= SPI_DFCR_RESET_MASK;

	/* clear tx/fifo and tx/fifo*/
	//*(controller->regs.cr) = SPI_CR_RFR_MASK | SPI_CR_TFR_MASK;
	SPI_PRINTF("reg cr=0x%x\n",(controller->regs.cr));

	/* clear ssr */
	*(controller->regs.sr) |= (SPI_SR_RFTPI_MASK | SPI_SR_TFTPI_MASK | SPI_SR_TFUI_MASK  \
						| SPI_SR_TFEI_MASK | SPI_SR_RFOI_MASK | SPI_SR_RFFI_MASK   \
						|SPI_SR_RFEI_MASK | SPI_SR_MFEI_MASK);

	/* set clk divider */
	//control |= ((divisor << SPI_CR_TCD_SHIFT) & SPI_CR_TCD_MASK);
	control |= ((1 << SPI_CR_TCD_SHIFT) & SPI_CR_TCD_MASK);

	/* setting output ssn signal---chip select*/
	control |= ((device->slave_select << SPI_CR_SS_SHIFT) & SPI_CR_SS_MASK);

	if (device->op_mode == SPI_DMA_MODE) 
	{
		/* dma setting*/
		/* enable threshold requst, under/over-run, Empty, full irq request.*/
		/* threshold is set to 8 bytes => dma burst size must less than 8 bytes*/
		if (device->tx_cnt) 
		{
			control |= SPI_CR_DRC_MASK | SPI_CR_TFUI_MASK | SPI_CR_TFEI_MASK;
			if (device->tx_fifo_threshold == SPI_THRESHOLD_8BYTES)
				control |= SPI_CR_TFTS_MASK;
		}
		if (device->rx_cnt) 
		{
			control |= SPI_CR_DRC_MASK | SPI_CR_RFOI_MASK | SPI_CR_RFFI_MASK | SPI_CR_RFEI_MASK;
			if (device->rx_fifo_threshold == SPI_THRESHOLD_8BYTES)
				control |= SPI_CR_RFTS_MASK;
		}
		dataformat |= BIT24;
	} 
	else if (device->op_mode == SPI_INTERRUPT_MODE) 
	{
		/* when tx/fifo changed from not empty to empty, a interrupt occurs */
		control |= SPI_CR_TFEI_MASK;
		/* enable irq*/
		control |= SPI_CR_IE_MASK;
	}

	/* setting master/slave model*/
	if (device->arbiter == SPI_ARBITER_SLAVE)  /* slave mode*/
		control |= SPI_CR_MSMS_MASK;

	/* regular drive configuration as slave select signal */
	if (device->ssn_port_mode == SPI_SSN_PORT_PTP)
		dataformat |= SPI_DFCR_SPM_MASK;

	/* set clock mode*/
	if ((device->clk_mode == SPI_MODE_2) || (device->clk_mode == SPI_MODE_3))
		control |= SPI_CR_CPS_MASK;
	if ((device->clk_mode == SPI_MODE_1) || (device->clk_mode == SPI_MODE_3))
		control |= SPI_CR_CPHS_MASK;
	/*
	* use for output setting value when fifo goes empty
	* if enable tx driver feature, setting related register.
	* else don't care
	*/
	if (device->tx_drive_enable) 
	{
		/*wbuf is null, we can use tx_drive_enable to drive the clock
		*  instead of transmit dummy data
		*/
		if(device->wbuf == NULL)
		{
			dataformat |= SPI_DFCR_TDE_MASK;
			/* setting driver value 0xff*/
			dataformat |= ((device->tx_nodata_value << SPI_DFCR_TNDV_SHIFT) & SPI_DFCR_TNDV_MASK);
			/* setting driver connt*/
			*(controller->regs.srcv_cnt) = device->tx_drive_count;
		}
	}

	if (device->ssn_port_mode == SPI_SSN_PORT_MM) 
	{	
		/* multi-master mode*/
		/* enable passed mode fault error interrupt request if interrupt is enabled*/
		control |= SPI_CR_MFEI_MASK | SPI_CR_MFEF_MASK;
	}
	else 
	{	 
		/* otherwise, point-to-point mode*/
		dataformat |= SPI_DFCR_SPM_MASK;
	}

	/* set mapping configuration to register*/
	*(controller->regs.dfcr) = dataformat;
	*(controller->regs.cr) = control;

	SPI_PRINTF("SPI%d Register Setting:\n", controller->bus_num);
	SPI_PRINTF("Control Register:0x%08x\n", *(controller->regs.cr));
	SPI_PRINTF("Status Register:0x%08x\n", *(controller->regs.sr));
	SPI_PRINTF("Data Format Register:0x%08x\n", *(controller->regs.dfcr));

	LEAVE();
	return 0;
}


int spi_enable(struct elite_spi_device *device) 
{
	struct elite_spi_controller *controller = device->controller;
	ENTER();
	/* check user and spi port valid*/
	if (!device || !controller) 
	{
		printk(KERN_ERR "[SPI]: invaild user enter %s.\n", __func__);
		LEAVE();
		return -1;
	}
	/* take port semaphore for pending other user use*/
	down(&controller->port_sem); 
	SPI_PRINTF("(%s) Acquired Semaphore@ %lu\n", controller->curdev->name, jiffies);
	SPI_PRINTF("SPI Enable, CR:%08x SR:%08x DFR:%08x\n",*(controller->regs.cr), 
				*(controller->regs.sr), *(controller->regs.dfcr));

	/* module enable*/
	*(controller->regs.cr) |= SPI_CR_ME_MASK;

	LEAVE();
	return 0;
}

int spi_disable(struct elite_spi_device *device)
{
	struct elite_spi_controller *controller = device->controller;
	ENTER();
	/* check user and spi port valid*/
	if (!device || !controller) 
	{
		printk(KERN_ERR "[SPI]: invaild user enter %s.\n", __func__);
		LEAVE();
		return -1;
	}
	/* module disable*/
	*(controller->regs.cr) &= ~(SPI_CR_ME_MASK);
	/* release port semaphore for other user use*/
	up(&controller->port_sem); 
	SPI_PRINTF("(%s) Released Semaphore@ %lu\n", controller->curdev->name, jiffies);
	SPI_PRINTF("SPI Disable, CR:%08x SR:%08x DFR:%08x\n",*(controller->regs.cr),
				*(controller->regs.sr), *(controller->regs.dfcr));

	LEAVE();
	return 0;
}

static irqreturn_t spi_isr(int irq, void *dev)
{
	unsigned int timecnt = 0x30000;
	unsigned int rx_count = 0;
	unsigned int i = 0;
	struct elite_spi_controller *controller = (struct elite_spi_controller *)dev;
	unsigned int spi_status = *(controller->regs.sr);
	ENTER();
	disable_irq_nosync(controller->irq);

	if (spi_status  & SPI_CR_RFFI_MASK)
		printk(KERN_WARNING "RX FIFO Full interrupt\n");
	if (spi_status  & SPI_CR_RFOI_MASK)
		printk(KERN_WARNING "RX FIFO over-run interrupt\n");
	if (spi_status & SPI_SR_TFES_MASK) 
	{
		while (spi_status & SPI_SR_BUSY_MASK) 
		{
			if (timecnt <= 0)
				break;
			spi_status = *(controller->regs.sr);
			--timecnt;
		}
		rx_count = spi_status >> SPI_SR_RFCNT_SHIFT;
		SPI_PRINTF("rx_count:%d",rx_count);
		SPI_PRINTF("rx_buf:%p\n",controller->curdev->rbuf);	
	}
	if (controller->curdev->callback)
		controller->curdev->callback(controller->curdev->rbuf);
	else
		printk("no callback func\n");

	/* clear flags */
	*(controller->regs.sr) |= (BIT14 | BIT13 | BIT12 | BIT11 | BIT10 | BIT9  | BIT8 | BIT4);
	controller->tx_event_ack = 1;
	controller->curdev->rx_cnt = rx_count;
	wake_up_interruptible(&(controller->tx_event_queue));
	enable_irq(controller->irq);

	LEAVE();
	return IRQ_HANDLED;
};

static void spi_dma_read_isr(void *data)
{
	struct elite_spi_controller *controller = (struct elite_spi_controller *)data;
	struct elite_spi_device *device = controller->curdev;
	unsigned int remain_size;
	unsigned long flags;
	int i;
	ENTER();
	SPI_PRINTF("enter spi_dma_read_isr cur_rx_cnt:%d\n",controller->cur_rx_cnt);
	/*check user and spi port valid*/
	if (!controller || !device)
	{
		LEAVE();
		return;
	}
	/*stall interrupt trigger*/
	spin_lock_irqsave(&controller->rdma.spinlock, flags); 	

	/*copy data from physical memory to virtual memory for dma moving*/
	memcpy(device->rbuf + controller->cur_rx_cnt, device->ior, controller->rdma.xfer_cnt);
	/*increase read in data counter*/
	controller->cur_rx_cnt += controller->rdma.xfer_cnt; 
	remain_size = device->rx_cnt - controller->cur_rx_cnt;
	SPI_PRINTF("[spi_dsr_r] remain_size =%u\n", remain_size);

	/*hardware limitation:*/
	/*tx/fifo size is less then threshold size*/
	/*it will not trigger dma request and data will always stay in fifo*/
	/*patch:*/
	/*moving by software*/
	/*threshold size: 8 bytes*/
	if (remain_size > 8) 
	{	
		/*bigger than threshold, it can be move by dma*/
		/*maximum transfer bytes is SPI_DMA_CHUNK_SIZE each time*/
		if (remain_size > SPI_DMA_CHUNK_SIZE)
			controller->rdma.xfer_cnt = SPI_DMA_CHUNK_SIZE;
		else
			/*minus (% 8) is because last remain bytes(less than 8) have to be sent by polling mode*/
			/*must be times of 8*/
			controller->rdma.xfer_cnt = remain_size - (remain_size % 8);

		/* enable next time dma*/
		START_DMA(controller->rdma.config.channel, device->phys_addr_r, controller->rdma.xfer_cnt);
	} 
	else 
	{
		/* moving by polling method*/
		/* software patch*/
		if (remain_size) 
		{
			for (i = 0; i < remain_size; i++) 
			{
				/*read in ramind data of rx fifo*/
				*(device->rbuf + controller->cur_rx_cnt) = *(controller->regs.rfifo);

				/*increase read in counter*/
				controller->cur_rx_cnt++; 
			}
		}
		/*create event for notifing thet transfer is complete*/
		controller->rdma.event_ack = 1;
		wake_up_interruptible(&(controller->rdma.event_queue));
	}
	LEAVE();

	/* restore interrupt trigger*/
	spin_unlock_irqrestore(&controller->rdma.spinlock, flags); 
	return;
};

static void spi_dma_write_isr(void *data)
{
	struct elite_spi_controller *controller = (struct elite_spi_controller *)data;
	struct elite_spi_device *device = controller->curdev;
	unsigned int remain_size;
	unsigned long flags;
	ENTER();
	SPI_PRINTF("spi_dma_write_isr cur_tx_cnt:%d,xfer_cnt:%d,tx_cnt:%d\n"
		                ,controller->cur_tx_cnt,controller->wdma.xfer_cnt,device->tx_cnt);
	/*check user and spi port valid*/
	if (!controller || !device)
	{
		LEAVE();
		return;
	}
	//*((volatile int *)controller->regs.wfifo)=0x9f;
	//*((volatile int *)controller->regs.wfifo)=0x9f;

	/*stall interrupt trigger*/
	spin_lock_irqsave(&controller->wdma.spinlock, flags); 
	/*increase read in data counter*/
	controller->cur_tx_cnt += controller->wdma.xfer_cnt; 
	remain_size = device->tx_cnt - controller->cur_tx_cnt;
	SPI_PRINTF("remain_size = %d\n", remain_size);
	
	if (remain_size) 
	{
		/*maximum transfer bytes is SPI_DMA_CHUNK_SIZE each time*/
		if (remain_size > SPI_DMA_CHUNK_SIZE)
			controller->wdma.xfer_cnt = SPI_DMA_CHUNK_SIZE;
		else
			controller->wdma.xfer_cnt = remain_size;

		if (device->wbuf)
			/* else case: write-for-read. so, it does't not need to moving source data*/
			/* copy source data from virtual memory to physical memory for dma moving*/
			memcpy(device->iow, device->wbuf + controller->cur_tx_cnt, controller->wdma.xfer_cnt);

		/*enable next time dma*/
		START_DMA(controller->wdma.config.channel, device->phys_addr_w, controller->wdma.xfer_cnt);
	}
	else 
	{
		controller->wdma.event_ack = 1;
		wake_up_interruptible(&(controller->wdma.event_queue));
	}
	
	/*restore interrupt trigger*/
	spin_unlock_irqrestore(&controller->wdma.spinlock, flags); 
	LEAVE();
	return;
};

static void spi_controller_reset(struct elite_spi_controller *controller)
{
	ENTER();
	/* check user valid*/
	if (!controller) 
	{
		printk(KERN_ERR "[SPI]: null port enter %s.\n", __func__);
		LEAVE();
		return;
	}
	/* restore spi port number*/
	controller->bus_num = controller->pdata->bus_num; 
	
	/* reset semaphore*/
	sema_init(&controller->port_sem,1);	

	controller->rdma.config.device_id =  "spi_rx";
	controller->rdma.config.chunk_size =  4;
	controller->rdma.config.callback =  spi_dma_read_isr,
	
	controller->rdma.config.addr_wrp_bnd = ELITE_DMA_1_BURST;
	controller->rdma.config.bst_len = ELITE_DMA_BURST_LEN_SINGLE;
	controller->rdma.config.trans_size = ELITE_DMA_RANS_SIZE_32BIT;
	controller->rdma.config.if12_addr_mode = ELITE_DMA_ADDR_MODE_WRAP;
	controller->rdma.config.sw_req = ELITE_DMA_REQ_HW;
	controller->rdma.config.trans_dir = ELITE_DMA_TRANS_IF12_TO_IF0;
	controller->rdma.config.device=1;  //SPI0_DMA_RX_REQ;
	controller->rdma.config.fifo_addr= SPI0_RX_FIFO;
	controller->rdma.config.callback_data=controller;

	controller->rdma.xfer_cnt = 0x0;
	controller->rdma.event_ack = 0x0;
	spin_lock_init(&controller->rdma.spinlock);

	/* initial read in dma*/
	init_waitqueue_head(&controller->rdma.event_queue);

	controller->wdma.config.device_id =  "spi_tx";
	controller->wdma.config.chunk_size =  4;
	controller->wdma.config.callback =  spi_dma_write_isr,
	controller->wdma.config.addr_wrp_bnd = ELITE_DMA_1_BURST;
	controller->wdma.config.bst_len = ELITE_DMA_BURST_LEN_SINGLE;
	controller->wdma.config.trans_size = ELITE_DMA_RANS_SIZE_32BIT;
	controller->wdma.config.if12_addr_mode = ELITE_DMA_ADDR_MODE_WRAP;
	controller->wdma.config.sw_req = ELITE_DMA_REQ_HW;
	controller->wdma.config.trans_dir = ELITE_DMA_TRANS_IF0_TO_IF12;
	controller->wdma.config.device=0; // SPI0_DMA_TX_REQ;
	controller->wdma.config.fifo_addr= SPI0_TX_FIFO;
	controller->wdma.config.callback_data=controller;
	/*initial write out dma*/
	init_waitqueue_head(&controller->wdma.event_queue);
	controller->wdma.xfer_cnt = 0x0;
	controller->wdma.event_ack = 0x0;
	spin_lock_init(&controller->wdma.spinlock);

	/*set cr register*/
	*(controller->regs.cr) |= (0x1<<SPI_CR_TCD_SHIFT);
	*(controller->regs.cr) &=(~(0x3<<SPI_CR_SS_SHIFT));
	*(controller->regs.cr) |= (0x1<<SPI_CR_RFR_SHIFT);
	*(controller->regs.cr) |= (0x1<<SPI_CR_TFR_SHIFT);
	*(controller->regs.cr) &=(~(0x1<<SPI_CR_DRC_SHIFT));
	*(controller->regs.cr) &=(~(0x1<<SPI_CR_TFUI_SHIFT));
	*(controller->regs.cr) &=(~(0x1<<SPI_CR_TFEI_SHIFT));
	*(controller->regs.cr) &=(~(0x1<<SPI_CR_RFOI_SHIFT));
	*(controller->regs.cr) &=(~(0x1<<SPI_CR_RFFI_SHIFT));
	*(controller->regs.cr) &=(~(0x1<<SPI_CR_TIDS_SHIFT));
	*(controller->regs.cr) &=(~(0x1<<SPI_CR_IE_SHIFT));
	*(controller->regs.cr) &=(~(0x1<<SPI_CR_ME_SHIFT));
	*(controller->regs.cr) &=(~(0x1<<SPI_CR_MFEI_SHIFT));
	*(controller->regs.cr) &=(~(0x1<<SPI_CR_MSMS_SHIFT));
	*(controller->regs.cr) &=(~(0x1<<SPI_CR_RFEI_SHIFT));
	*(controller->regs.cr) |= (0x1<<SPI_CR_CPS_SHIFT);
	*(controller->regs.cr) |= (0x1<<SPI_CR_CPHS_SHIFT);
	*(controller->regs.cr) &=(~(0x1<<SPI_CR_MFEF_SHIFT));

	/*set sr register*/
	*(controller->regs.sr) |= (0x1<<SPI_SR_RFTPI_SHIFT);
	*(controller->regs.sr) |= (0x1<<SPI_SR_TFTPI_SHIFT);
	*(controller->regs.sr) |= (0x1<<SPI_SR_TFUI_SHIFT);
	*(controller->regs.sr) |= (0x1<<SPI_SR_TFEI_SHIFT);
	*(controller->regs.sr) |= (0x1<<SPI_SR_RFOI_SHIFT);
	*(controller->regs.sr) |= (0x1<<SPI_SR_RFFI_SHIFT);
	*(controller->regs.sr) |= (0x1<<SPI_SR_RFEI_SHIFT);
	*(controller->regs.sr) |= (0x1<<SPI_SR_MFEI_SHIFT);

	/*set dfcr register*/
	*(controller->regs.dfcr) |= (0x1<<SPI_DFCR_DSE_SHIFT);
	*(controller->regs.dfcr) |= (0x1<<SPI_DFCR_DSV_SHIFT);
	*(controller->regs.dfcr) |= (0x1<<SPI_DFCR_SPM_SHIFT);
	
	LEAVE();
	return;
}


static int spi_polling_write_and_read_32bytes(struct elite_spi_device *device,
				unsigned char *rbuf,const unsigned char *wbuf,unsigned int size)
{
	struct elite_spi_controller *controller = device->controller;
	unsigned int status, i;
	int timeout_cnt;
	int retval = size;
	ENTER();
	if (!device || !controller) 
	{
		printk(KERN_ERR "[SPI]: spi user null\n");
		LEAVE();
		return -1;
	}
	if (size > 0x20) 
	{
		printk(KERN_ERR "[spi_polling_write_and_read_32bytes] Error: Transfer size must less than 32 bytes.\n");
		LEAVE();
		return -1;
	}
	if (wbuf == NULL) 
	{
		/* if tx_drive is not enable, we need to write dummy data to drive clock */
		if(!device->tx_drive_enable)
		{
			for (i = 0; i <  (size+3)/4; i++)
			{
				SPI_PRINTF("polling_write 0xFFFFFFFF\n");
				/* write in garbage data to read */
				*((volatile unsigned int*)controller->regs.wfifo) = 0xFFFFFFFF;
			}
		}
	} 
	else 
	{
		for (i = 0; i < (size+3)/4; i++)
		{
			SPI_PRINTF("polling_write 0x%x\n",*((unsigned int*)wbuf + i));
			/* write in source data*/
			*((volatile unsigned int*)(controller->regs.wfifo)) = *((unsigned int*)wbuf + i);	
		}	
	}
	timeout_cnt = 0x200000;
	while (timeout_cnt) 
	{
		status = *(controller->regs.sr);
		if (status & SPI_SR_TFES_MASK)
			break;
		timeout_cnt = timeout_cnt - 1;
	}

	if (!timeout_cnt) 
	{
		printk(KERN_ERR "[spi_polling_write_and_read_32bytes] transmit time out\n");
		LEAVE();
		return -1;
	}
	/* read from device */
	if (rbuf != NULL) 
	{          
	       SPI_PRINTF("tx_cnt 0x%x\n",controller->curdev->tx_cnt);
		for (i = 0; i <  (size+3)/4; i++) 
		{
			timeout_cnt = 0x20000;
			while (timeout_cnt) 
			{
				status = *(controller->regs.sr);
				if (status & SPI_SR_RFCNT_MASK)   
					break;
				timeout_cnt = timeout_cnt - 1;
			}

			if (timeout_cnt)
			{
				*((unsigned int*)rbuf + i) = *((volatile unsigned int*)controller->regs.rfifo+i);
				SPI_PRINTF("polling_read 0x%x\n",*((unsigned int*)rbuf + i));

			}
			else
			{                  
				/* there is no read in data , should not happen*/
				printk(KERN_ERR "[spi_polling_write_and_read_32bytes] receive time out\n");
				*((unsigned int*)rbuf + i) = *((volatile unsigned int*)controller->regs.rfifo+i);
				retval = -ETIMEOUT;             
			}
		}
	}
	LEAVE();
	return retval;
}

static int spi_interrupt_write_and_read_32bytes(struct elite_spi_device *device,
				const unsigned char *wbuf,unsigned char *rbuf,unsigned int size)
{
	struct elite_spi_controller *controller = device->controller;
	unsigned int i;
	int retval = size;
	ENTER();
	if (!device || !controller) 
	{
		printk(KERN_ERR "[SPI]: spi user null\n");
		LEAVE();
		return -1;
	}
	if (size > 0x20) 
	{
		printk(KERN_ERR "[spi_interrupt_write_and_read_32bytes] Error: Transfer size must less than 32 bytes.\n");
		LEAVE();
		return -1;
	}

	if (wbuf == NULL) 
	{
		/* if tx_drive is not enable, we need to write dummy data to drive clock */
		if(!device->tx_drive_enable)
		{
			for (i = 0; i <  (size+3)/4; i++)
			{
			       SPI_PRINTF("interrupt_write 0xFFFFFFFF\n");
				/* write in garbage data to read */
				*((volatile unsigned int*)controller->regs.wfifo) = 0xFFFFFFFF;
			}
		}
	} 
	else
	{
		for (i = 0; i < (size+3)/4; i++)
		{
			SPI_PRINTF("interrupt_write 0x%x\n",*((unsigned int*)wbuf + i));
			/* write in source data*/
			*((volatile unsigned int*)(controller->regs.wfifo)) = *((unsigned int*)wbuf + i);	
		}
	}
	if(rbuf)
	{
		for (i = 0; i < (controller->curdev->rx_cnt+3)/4; ++i)
		{
			*((unsigned int*)rbuf + i) = *((volatile unsigned int*)controller->regs.rfifo+i);
			SPI_PRINTF("rx_buf:0x%x\n",*((unsigned int*)rbuf + i));	
		}
	}
	LEAVE();
	return retval;
}

static int spi_polling_write_and_read(struct elite_spi_device *device, struct spi_transfer *transfer)
{
	unsigned char *p_rx;
	const unsigned char *p_wx;
	unsigned int transfer_cnt = transfer->len;
	int ret = transfer_cnt;
	ENTER();

	p_rx = transfer->rx_buf;
	p_wx = transfer->tx_buf;
	if(device->tx_drive_enable)
		device->tx_drive_count = transfer->len;
	spi_set_reg(device);
	spi_enable(device);
	while (transfer_cnt) 
	{
		/*
		* write out data to spi port, and  read in data from spi port
		* because wmt spi tx fifo and rx fifo are 32 bytes,
		* only write and read 32 bytes every time.
		* notify: work only for polling mode
		*/
		if (transfer_cnt > 32) 
		{
			ret = spi_polling_write_and_read_32bytes(device, p_rx, p_wx, 32);
			if (ret < 0) 
			{
				spi_disable(device);
				LEAVE();
				return ret;
			}

			p_rx += 32;
			transfer_cnt -= 32;
		}
		else 
		{
			ret = spi_polling_write_and_read_32bytes(device, p_rx, p_wx, transfer_cnt);
			if (ret < 0) 
			{
				spi_disable(device);
				LEAVE();
				return ret;
			}
			transfer_cnt = 0;
		}
	}
	spi_disable(device);
	LEAVE();
	return ret;
}
static int spi_interrupt_write_and_read(struct elite_spi_device *device, struct spi_transfer *transfer)
{
	const unsigned char *p_tx;
	unsigned char *p_rx;
	unsigned int transfer_cnt = transfer->len;
	wait_queue_head_t *event_queue;
	volatile int *event_ack;
	int ret;
	ENTER();
	SPI_PRINTF("%s  %d\n", __func__, __LINE__);

	/* wmt interrupt only support size < 32 */
	if (transfer_cnt > 32)   
	{
		LEAVE();
		return -1;
	}

	p_tx = transfer->tx_buf;
	p_rx = transfer->rx_buf;
	if(transfer->cs_change)
	{
		SPI_PRINTF("cs_changed\n");
		device->controller->cs_change = 1;
	}
	
	if(device->controller->cs_change)
	{
		spi_set_reg(device);
		device->controller->cs_change = 0;
	}

	spi_enable(device);
	ret = spi_interrupt_write_and_read_32bytes(device, p_tx, p_rx, transfer_cnt);
	if (ret < 0) 
	{
		spi_disable(device);
		LEAVE();
		return ret;
	}
	event_queue = &device->controller->tx_event_queue;
	event_ack = &device->controller->tx_event_ack;
	/* wait for transfer complete*/
	wait_event_interruptible(*event_queue, *event_ack);

	spi_disable(device);
	LEAVE();
	return ret;
}

static int spi_dma_write_and_read(struct elite_spi_device *device, struct spi_transfer *transfer)
{
	struct elite_spi_controller *controller = device->controller;
	wait_queue_head_t *event_queue_write, *event_queue_read;
	volatile int *event_ack_write, *event_ack_read;
	int timeout_cnt=0;
	int len;
	unsigned int status;
	ENTER();
	SPI_PRINTF("%s  %d\n", __func__, __LINE__);

	len = transfer->len;
	/*reset current receive count*/
	controller->cur_rx_cnt = 0;	
	/*reset current transfer count*/
	controller->cur_tx_cnt = 0;	
	/*set read in size*/
	device->rx_cnt = len;	
	/*set read in address*/
	device->rbuf = transfer->rx_buf;	
	/*set write out address. NULL means no write data*/
	device->wbuf = transfer->tx_buf;		
	/*set write out size, master needs write-for-read*/
	if (device->arbiter == SPI_ARBITER_MASTER)	
	{
		/* write out for read*/
		device->tx_cnt = len;	
	}
	else
		device->tx_cnt = 0;
	device->tx_fifo_threshold = SPI_THRESHOLD_8BYTES;
	device->rx_fifo_threshold = SPI_THRESHOLD_8BYTES;

	spi_set_reg(device);
	/*
	* if data size are bigger than allocated memory size,
	* transfer allocated memory size and finish the others in dma service routine
	*/
	if (len > SPI_DMA_CHUNK_SIZE) 
	{
		controller->wdma.xfer_cnt = SPI_DMA_CHUNK_SIZE;
		controller->rdma.xfer_cnt = SPI_DMA_CHUNK_SIZE;
	} 
	else 
	{
		controller->wdma.xfer_cnt = len;
		controller->rdma.xfer_cnt = len;
	}

	if ((device->op_mode == SPI_DMA_MODE) && (device->dma_mem_init)) 
	{
		/* master setting bcz. write-for-read*/
		if (device->arbiter == SPI_ARBITER_MASTER) 
		{
			/* if wbuf is not null, we need to start dma to write data */
			/* or wbuf is null, but tx_drive is not enable ,start dma to write dummy data to drive clock */
			if(!device->tx_drive_enable || device->wbuf != NULL)
			{
				if (REQUEST_DMA(&controller->wdma.config) < 0) 
				{
					printk(KERN_ERR "[SPI]: Error request dma, force polling mode.\n");
					goto polling_write_and_read_data;
				}
				SPI_PRINTF("SPI TX FIFO channel = %d,len=%d\n",controller->wdma.config.channel,len);

				/* tx_buf is null, write random data in the dma buffer */ 
				if (transfer->tx_buf != NULL)
					memcpy(device->iow, transfer->tx_buf, len);
			}

		}

		/* request dma channel. if there is no using dma channel, work with polling mode*/
		if(device->rbuf != NULL)
		{
			if (REQUEST_DMA(&controller->rdma.config) < 0) 
			{
				printk(KERN_ERR "[SPI]: Error request dma, force polling mode.\n");
				/* free acquired dma */
				FREE_DMA(controller->wdma.config.channel);  
				goto polling_write_and_read_data;
			}
			SPI_PRINTF("SPI RX FIFO channel = %d,len=%d\n",controller->rdma.config.channel,len);
			/* set up read in dma and enable read in dma*/
			START_DMA(controller->rdma.config.channel, device->phys_addr_r, controller->rdma.xfer_cnt);
		}

		/* assign wait queue to dma queue*/
		event_queue_read  = &controller->rdma.event_queue;
		event_ack_read   = &controller->rdma.event_ack;
		event_queue_write = &controller->wdma.event_queue;
		event_ack_write   = &controller->wdma.event_ack;

		if(device->wbuf != NULL)
			/* set up write out dma and enable write out dma*/
			START_DMA(controller->wdma.config.channel, device->phys_addr_w, controller->wdma.xfer_cnt);
		
		spi_enable(device);
	} 
	else 
	{
		LEAVE();
polling_write_and_read_data:
		/* request dma failed, use polling mode*/
		return spi_polling_write_and_read(device, transfer);

	}
	//if(!device->tx_drive_enable || device->wbuf != NULL)
	if(device->wbuf != NULL)
	{
		wait_event_interruptible(*event_queue_write, *event_ack_write);
		/* clear even ack*/
		*event_ack_write = 0; 			
	}

	if(transfer->rx_buf != 0)
	{
		wait_event_interruptible(*event_queue_read, *event_ack_read);
		/* clear even ack*/
		*event_ack_read = 0; 			
	}
	
	timeout_cnt = 0x400000;
	while (timeout_cnt) 
	{
		status = *(controller->regs.sr);
		if (status & SPI_SR_TFES_MASK)
			break;
		timeout_cnt = timeout_cnt - 1;
	}

	/* disable spi module*/
	spi_disable(device);

	/* for working with dma mode to release dma channel*/
	if (device->dma_mem_init) 
	{
		if (device->op_mode == SPI_DMA_MODE) 
		{
			/* free dma bcz this time is transfer by dma*/
			FREE_DMA(controller->wdma.config.channel);
			FREE_DMA(controller->rdma.config.channel);
		}
		/* force to dma mode for next time*/
		device->op_mode = SPI_DMA_MODE;	
	}

	if (!timeout_cnt) 
	{
		printk(KERN_ERR "[spi_dma_read_data] write for read time out\n");
		LEAVE();
		return -ETIMEOUT;
	}

	LEAVE();
	/* read in data size*/
	return controller->cur_rx_cnt;	
};

void set_callback_func(struct elite_spi_device *device, void (*callback)(unsigned char *data))
{
	ENTER();
	device->callback = callback;
	LEAVE();
}

/* choose polling,interrupt or dma mode policy*/
static enum spi_operation_mode_type elite_spi_choose_mode(struct elite_spi_device *device, struct spi_transfer *transfer)
{
	ENTER();
	if(transfer->len > 32 && device->dma_mem_init)
	{
		LEAVE();
		return SPI_DMA_MODE;
	}
	else if(transfer->len > 32)
	{
		LEAVE();
		return SPI_POLLING_MODE;
	}
	else 
	{
		LEAVE();
		return SPI_INTERRUPT_MODE;
	}
}

static void elite_handle_msg(struct elite_spi_controller *controller,struct spi_message *msg)
{
	struct spi_device *spi = msg->spi;
	struct spi_transfer *xfer;	
	struct elite_spi_device *device = controller->curdev;
	int status = 0;
	ENTER();

	list_for_each_entry(xfer, &msg->transfers, transfer_list) 
	{
		device->rbuf = xfer->rx_buf;
		device->wbuf = xfer->tx_buf;
		
		SPI_PRINTF("[handle_msg]: xfer->cs_change mode: %d\n", xfer->cs_change);
		xfer->cs_change=1;
		device->op_mode = elite_spi_choose_mode(device, xfer);
		//device->op_mode =SPI_DMA_MODE;
		if (device->op_mode == SPI_DMA_MODE)
		{
		       SPI_PRINTF("[handle_msg]: operation mode: %d\n", device->op_mode);
			status = spi_dma_write_and_read(device, xfer);
		}
		else if (device->op_mode == SPI_POLLING_MODE)
		{
		       SPI_PRINTF("[handle_msg]: operation mode: %d\n", device->op_mode);
			status = spi_polling_write_and_read(device, xfer);
		}
		else if (device->op_mode == SPI_INTERRUPT_MODE)
		{
		       SPI_PRINTF("[handle_msg]: operation mode: %d\n", device->op_mode);
			status = spi_interrupt_write_and_read(device, xfer);
		}
		else 
		{
			SPI_PRINTF("[handle_msg]: don't surpport the operation mode: %d\n", device->op_mode);
			status = -EUNKNOWNMODE;
		}
		msg->status = status;
		if (status < 0)
		{
			goto out;
		}
		if (xfer->delay_usecs)
			udelay(xfer->delay_usecs);

		msg->actual_length += xfer->len;
	}
out:
	/*wakeup the client thread */
	if (msg->complete)
		msg->complete(msg->context);
	
	LEAVE();
}
	
static void elite_spi_work(struct work_struct *work)
{
	struct elite_spi_controller *controller = container_of(work, struct elite_spi_controller, work);
	unsigned long flags;
	ENTER();
	spin_lock_irqsave(&controller->lock, flags);

	while (!list_empty(&controller->queue) && !(controller->state & SUSPND)) 
	{
		struct spi_message *msg;
		msg = container_of(controller->queue.next, struct spi_message, queue);
		list_del_init(&msg->queue);
		elite_handle_msg(controller, msg);
	}
	spin_unlock_irqrestore(&controller->lock, flags);
	LEAVE();
}

static int elite_spi_setup(struct spi_device *spi)
{
	struct elite_spi_device *device;
	struct elite_spi_controller *controller;

	ENTER();
	device = spi->controller_data;
	controller = spi_master_get_devdata(spi->master);
	/* if this spi_device's private data is not initialized, initialize it*/
	if(device == NULL)
	{
		device = (struct elite_spi_device *)kmalloc(sizeof(struct elite_spi_device), GFP_KERNEL);
		if (device == NULL) 
		{
			SPI_PRINTF("No memory for spi user record!!!\n");
			LEAVE();
			return -1;
		}
		/* reset use configuration with default setting*/
		spi_user_reset(device); 	
		/* link spi controller to the device*/
		device->controller = controller; 				

		/* allocate read in dma memory*/
		device->ior = (unsigned char *)dma_alloc_coherent(NULL, SPI_DMA_SIZE, 
							&device->phys_addr_r, GFP_KERNEL | GFP_DMA);

		/* if allocate failure, force work with polling mode*/
		if (device->ior == NULL) 
		{
			SPI_PRINTF("DMA memory allocate fail, force polling mode.\n");
			device->op_mode = SPI_POLLING_MODE;
			goto dma_mem_init_over;
		} 
		else
		{
			SPI_PRINTF("Allocate Read In DMA Memory successful\n");
			/* reset allocated memory*/
			memset(device->ior, 0x0, SPI_DMA_SIZE); 
		}

		/* allocate write out dma memory*/
		device->iow = (unsigned char *)dma_alloc_coherent(NULL, SPI_DMA_SIZE, 
					&device->phys_addr_w, GFP_KERNEL | GFP_DMA);

		/* if allocate failure, force work with polling mode, and free read-in dma memory*/
		if (device->iow == NULL) 
		{
			SPI_PRINTF("DMA memory allocate fail, force polling mode.\n");
			device->op_mode = SPI_POLLING_MODE;
			dma_free_coherent(NULL, SPI_DMA_SIZE, device->ior, device->phys_addr_r);
		} 
		else 
		{
			/* set dma resource is available*/
			device->dma_mem_init = 1;             
			device->client_init = 1;
			/* reset allocated memory*/
			memset(device->iow, 0x0, SPI_DMA_SIZE); 
			SPI_PRINTF("Allocate Write Out DMA Memory successful\n");
		}
dma_mem_init_over:
		spi_set_freq(device, spi->max_speed_hz);
		spi_set_clk_mode(device, spi->mode);
		spi_set_chipselect(device, spi->chip_select);
	}
	controller->curdev=device;

	LEAVE();
	return 0;
}
	
static int elite_spi_transfer(struct spi_device *spi,
							struct spi_message *msg)
{
	struct elite_spi_controller *controller;
	unsigned long flags;
	ENTER();

	controller = spi_master_get_devdata(spi->master);
	spin_lock_irqsave(&controller->lock, flags);

	if (controller->state & SUSPND) 
	{
		spin_unlock_irqrestore(&controller->lock, flags);
		LEAVE();
		return -ESHUTDOWN;
	}

	msg->status = -EINPROGRESS;
	msg->actual_length = 0;
	list_add_tail(&msg->queue, &controller->queue);
	queue_work(controller->workqueue, &controller->work);
	spin_unlock_irqrestore(&controller->lock, flags);
	LEAVE();
	return 0;
}
	
static void elite_spi_cleanup(struct spi_device *spi)
{
	ENTER();
	LEAVE();
}
	
static int elite_spi_probe(struct platform_device *pdev)
{
	struct elite_spi_controller *controller;
	struct spi_master *master;
	struct resource *res;
	int ret = 0;

	ENTER();
	master = spi_alloc_master(&pdev->dev, sizeof(struct elite_spi_controller));
	if (master == NULL)
	{
		dev_err(&pdev->dev, "No memory for spi_master\n");
		ret = -ENOMEM;
		LEAVE();
		return ret;
	}

	controller = spi_master_get_devdata(master);
	memset(controller, 0, sizeof(struct elite_spi_controller));

	controller->master = spi_master_get(master);
	controller->pdata = pdev->dev.platform_data;
	controller->dev = &pdev->dev;
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) 
	{
		ret = -ENXIO;
		goto fail_no_mem_resource;
	}

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (res == NULL)
	{
		ret = -EBUSY;
		goto fail_no_mem_resource;
	}
	//controller->mmio_base = res->start ;
	controller->mmio_base = ioremap(res->start, resource_size(res));
	if (controller->mmio_base == NULL) 
	{
		ret = -ENXIO;
		goto fail_no_ioremap;
	}
       SPI_PRINTF("mmio_base:0x%x\n", res->start);

	/* Setting Register address*/
	controller->regs.cr = (volatile unsigned int *)(controller->mmio_base + SPICR);
	controller->regs.sr = (volatile unsigned int *)(controller->mmio_base + SPISR);
	controller->regs.dfcr = (volatile unsigned int *)(controller->mmio_base + SPIDFCR);
	controller->regs.cre = (volatile unsigned int *)(controller->mmio_base + SPICRE);
	controller->regs.rfifo = (volatile unsigned char *)(controller->mmio_base + SPIRXFIFO);
	controller->regs.wfifo = (volatile unsigned char *)(controller->mmio_base + SPITXFIFO);
	controller->regs.srcv_cnt = (volatile unsigned int *)(controller->mmio_base + SPISRCVCNT);
	controller->regs.srcv_add_cnt = (volatile unsigned int *)(controller->mmio_base + SPISRCVADDCNT);
	controller->irq = ret = platform_get_irq(pdev, 0);
	if (ret <= 0) 
	{
		dev_err(&pdev->dev, "cannot find IRQ");
		goto err_ioremap;
	}

	if (controller->pdata == NULL) 
	{
		dev_err(&pdev->dev, "No platform data supplied\n");
		ret = -ENOENT;
		goto err_ioremap;
	}

	platform_set_drvdata(pdev, controller);
	init_completion(&controller->done);

	/* setup the master state. */
	/* the spi->mode bits understood by this driver: */
	master->mode_bits = SPI_MODE_3| SPI_CS_HIGH;
	/* most num of devices in spi bus */
	master->num_chipselect = controller->pdata->num_cs;    
	/* spi controller 0,1,2 */
	master->bus_num = controller->pdata->bus_num;			

	/* setup callback functions*/
	master->setup = elite_spi_setup;
	master->transfer = elite_spi_transfer;
	master->cleanup = elite_spi_cleanup;

       SPI_PRINTF("IRQ:%d\n",controller->irq);

	if (request_irq(controller->irq, spi_isr, IRQF_DISABLED , "SPI-IRQ", controller) < 0) 
	{
		printk(KERN_ERR "request irq failed \n");
		ret = -ENOENT;
		goto err_ioremap;
	}
	else
		printk(KERN_ERR "request irq %d success \n", controller->irq);

	/* setup work thread */
	controller->workqueue = create_singlethread_workqueue(dev_name(master->dev.parent));
	if (controller->workqueue == NULL) 
	{
		dev_err(&pdev->dev, "Unable to create workqueue\n");
		ret = -ENOMEM;
		goto fail_to_create_workqueue;
	}

	INIT_WORK(&controller->work, elite_spi_work);
	INIT_LIST_HEAD(&controller->queue);
	init_waitqueue_head(&controller->tx_event_queue);
	init_waitqueue_head(&controller->rx_event_queue);

	if (spi_register_master(master)) 
	{
		dev_err(&pdev->dev, "cannot register SPI master\n");
		ret = -EBUSY;
		goto fail_to_register_spi_master;
	}
	/*add a spi device for testing spi flash begin*/
       {
		strcpy(controller->pdata->info.modalias, "spidev");
		controller->pdata->info.max_speed_hz =12* 1000 * 1000;
		controller->pdata->info.chip_select = 0;
		controller->pdata->info.mode =  SPI_MODE_3;

		/*enable access to our primary data structure via
		* the board info's (void *)controller_data.
		*/
		controller->pdata->info.controller_data = NULL;
		controller->pdata->spidev_elite = spi_new_device(master, &(controller->pdata->info));
		if (controller->pdata->spidev_elite)
			SPI_PRINTF("spidev_elite at %s\n","spidev");
		else 
		{
			printk(KERN_WARNING "%s: spi_new_device failed\n", "spidev");
		}
	}
	/*add a spi device for testing spi flash end*/
	/* reset spi hardware and property*/
	spi_controller_reset(controller);	
	
	LEAVE();
	return 0;

fail_to_register_spi_master:
	destroy_workqueue(controller->workqueue);
fail_to_create_workqueue:
	free_irq(controller->irq, controller);
err_ioremap:
	iounmap(controller->mmio_base);
fail_no_ioremap:
	release_mem_region(res->start, resource_size(res));
fail_no_mem_resource:
	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);
	LEAVE();
	return ret;
} 

static int elite_spi_remove(struct platform_device *pdev)
{
	struct elite_spi_controller *controller = platform_get_drvdata(pdev);
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	ENTER();
	if(controller->curdev)
		spi_disable(controller->curdev);
	clk_disable(controller->clk);
	clk_put(controller->clk);
	platform_set_drvdata(pdev, NULL);
	iounmap(controller->mmio_base);
	release_mem_region(res->start, resource_size(res));
	kfree(controller);

	LEAVE();
	return 0;
} 

static int elite_spi_suspend(struct platform_device *pdev, pm_message_t state)
{
	ENTER();
	LEAVE();
	return 0;
} 

static int elite_spi_resume(struct platform_device *pdev	)
{
	ENTER();
	LEAVE();
	return 0;
} 

static void spi_platform_release(struct device *device)
{
	ENTER();
	LEAVE();
} 

static struct resource elite_spi_resource[] = {
	[0] = {
		.start = 0xD8240000,
		.end =  0xD8240000+ 0x50,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start =56,
		.end = 56,
		.flags = IORESOURCE_IRQ,
	}
};
struct elite_spi_platform_data  elite_spi_pf_data={
	0,   
	4,	   
	0,
	NULL
};
void	elite_spi_release(struct device *dev)
{
	ENTER();
	LEAVE();
}

struct platform_device elite_spi_platform_device ={	
	.name		= DEVICE_NAME,	  
	.id 			= -1,	
	.num_resources = ARRAY_SIZE(elite_spi_resource),
	.resource  =elite_spi_resource,
	.dev		= {
		.coherent_dma_mask= 0xffffffff,
		.platform_data =&elite_spi_pf_data,
		.release=elite_spi_release
	},
};


static struct platform_driver elite_spi_driver = {
	/* This name should equal to platform device name.*/
	.driver.name  = DEVICE_NAME,     
	.probe          = elite_spi_probe,
	.remove        = elite_spi_remove,
	.suspend       = elite_spi_suspend,
	.resume        = elite_spi_resume
};

static int spi_init(void)
{
	int ret;
	ENTER();
	ret = platform_device_register(&elite_spi_platform_device);
	if (ret) 
	{
		platform_device_unregister(&elite_spi_platform_device);
		LEAVE();
		return ret;
	}
	ret = platform_driver_register(&elite_spi_driver);
	if (ret) 
	{
		platform_driver_unregister(&elite_spi_driver);
		LEAVE();
		return ret;
	}

	LEAVE();
	return ret;
} 

static void spi_exit(void)
{
	ENTER();
	platform_driver_unregister(&elite_spi_driver);
	platform_device_unregister(&elite_spi_platform_device);

	LEAVE();
	return;
} 

module_init(spi_init);
module_exit(spi_exit);

MODULE_AUTHOR("S3 SW Team");
MODULE_DESCRIPTION("elite spi controller driver");
MODULE_LICENSE("GPL");

