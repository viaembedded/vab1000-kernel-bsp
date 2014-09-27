/*++
drivers/spi/elite-spiio.h
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

#ifndef ELITE_SPIIO_H
#define ELITE_SPIIO_H

#include <linux/types.h>
#include <linux/module.h>
#include <linux/semaphore.h>
#include <mach/dma.h>
#include <asm/sizes.h>
#include <mach/dma.h>

/* allocate memory for variables only in elite_spi.c */
#ifdef ELITE_SPI_C 
#       define EXTERN
#else
#       define EXTERN   extern
#endif /* ifdef ELITE_SPI_C */

#undef EXTERN

#define SPI_IOC_MAGIC	'k'
#define SPI_IOC_MAXNR	12

/* spi io control*/
/* get current working frequency of this user*/
#define SPIIOGET_FREQ		 _IOR(SPI_IOC_MAGIC,  0, unsigned int)
/* set working frequency for the user*/
#define SPIIOSET_FREQ		 _IOWR(SPI_IOC_MAGIC, 1, unsigned int)

/* get current bus clock mode of this user*/
#define SPIIOGET_CLKMODE	 _IOR(SPI_IOC_MAGIC,  2, unsigned int)
/* set bus clock mode for the user*/
#define SPIIOSET_CLKMODE	 _IOWR(SPI_IOC_MAGIC, 3, unsigned int)

/* get current bus arbiter of this user*/
#define SPIIOGET_ARBITER	 _IOR(SPI_IOC_MAGIC,  4, unsigned int)
/* set bus arbiter for the user*/
#define SPIIOSET_ARBITER	 _IOWR(SPI_IOC_MAGIC, 5, unsigned int)

/* get current operation mode of this user*/
#define SPIIOGET_OPMODE	 _IOR(SPI_IOC_MAGIC,  6, unsigned int)
/* set operation mode for the user*/
#define SPIIOSET_OPMODE	 _IOWR(SPI_IOC_MAGIC, 7, unsigned int)

/* get current SSn port mode of this user*/
#define SPIIOGET_PORTMODE _IOR(SPI_IOC_MAGIC,  8, unsigned int)
/* set ssn port mode for the user*/
#define SPIIOSET_PORTMODE  _IOWR(SPI_IOC_MAGIC, 9, unsigned int)

/* read in data from spi port*/
#define SPIIO_READ			   _IOWR(SPI_IOC_MAGIC, 10, struct spi_xfer_req_s)
/* write out data to spi port*/
#define SPIIO_WRITE			   _IOWR(SPI_IOC_MAGIC, 11, struct spi_xfer_req_s)
/* write out data from spi port then read in data to spi port*/
#define SPIIO_READANDWRITE _IOWR(SPI_IOC_MAGIC, 12, struct spi_xfer_req_s)

/*it is forced value: 25 MHz = 25000 KHz*/
#define PLL_25MHZ		                 25000  
/*it is forced value: 27 MHz = 27000 KHz*/
#define PLL_27MHZ		                 27000  
#define START_DMA(a, b, c)		 elite_start_dma(a, b, 0, c)
#define SETUP_DMA(a, b)			 elite_setup_dma(a, b)
#define REQUEST_DMA(a)                 elite_request_dma(a)
#define FREE_DMA(a)				 elite_free_dma(a)

#define ELT_PMC_BASE                     0xD8130000
#define SPI0_CLOCK_DIVISOR         (ELT_PMC_BASE + 0x33C)
#define SPI1_CLOCK_DIVISOR         (ELT_PMC_BASE + 0x340)
#define SPI2_CLOCK_DIVISOR         (ELT_PMC_BASE + 0x344)
#define PLLB_MULTIPLIER               (ELT_PMC_BASE + 0x204)

#define SPI0CLKEN                           BIT12
#define SPI1CLKEN                           BIT13
#define SPI2CLKEN                           0

/*spi port 1, irq number*/
#define	SPI_PORT1_IRQ		       IRQ_SPI1	
/*spi port 1, rx dma id*/
#define	SPI_PORT1_RX_DMA_REQ  APB_SPI_1_RX_REQ	
/*spi port 1, tx dma id*/
#define	SPI_PORT1_TX_DMA_REQ   APB_SPI_1_TX_REQ	
/*the number of spi port*/
#define SPI_PORT_NUM			1	

/************ register address ************/
/* spi port 0 base address*/
#define SPI_REG_BASE	       0xD8240000	
/* spi port 1 base address*/
#define SPI1_REG_BASE	       0xD8250000	
/* spi port 2 base address*/
#define SPI2_REG_BASE	       0xD82A0000	
#define SPI1_PORT_OFFSET   0x10000
#define SPI2_PORT_OFFSET   0x60000
/* spi control register offset base on port base address*/
#define SPICR			       0x00	
/* spi status register offset base on port base address*/
#define SPISR			       0x04	
/* spi data format control register offset base on port base address*/
#define SPIDFCR			       0x08	
#define SPICRE			       0x0C
/* spi tx fifo offset base on port base address*/
#define SPITXFIFO		       0x10	
/* spi rx fifo offset base on port base address*/
#define SPIRXFIFO		       0x30	
/* spi receive count /tx driver count */
#define SPISRCVCNT		       0x50    
#define SPISRCVADDCNT	       0x54	

/************ control register ************/
/* transmit clock driver*/
#define SPI_CR_TCD_SHIFT	21
#define SPI_CR_TCD_MASK	(BIT31|BIT30|BIT29|BIT28|BIT27|BIT26|BIT25|BIT24|BIT23|BIT22|BIT21)
/* slave selection*/
#define SPI_CR_SS_SHIFT		19
#define SPI_CR_SS_MASK		(BIT20|BIT19)
/* transmit fifo byte write method*/
#define SPI_CR_WM_SHIFT	18
#define SPI_CR_WM_MASK	(BIT18)
/* receive fifo reset*/
#define SPI_CR_RFR_SHIFT	17
#define SPI_CR_RFR_MASK	(BIT17)
/* transmit fifo reset*/
#define SPI_CR_TFR_SHIFT	16
#define SPI_CR_TFR_MASK	(BIT16)
/* dma request control*/
#define SPI_CR_DRC_SHIFT	15
#define SPI_CR_DRC_MASK	(BIT15)
/* receive fifo threshold selection*/
#define SPI_CR_RFTS_SHIFT	14
#define SPI_CR_RFTS_MASK	(BIT14)
/* transmit fifo threshold selection*/
#define SPI_CR_TFTS_SHIFT	13
#define SPI_CR_TFTS_MASK	(BIT13)
/* transmit fifo under-run interrupt*/
#define SPI_CR_TFUI_SHIFT	12
#define SPI_CR_TFUI_MASK	(BIT12)
/* transmit fifo empty interrupt*/
#define SPI_CR_TFEI_SHIFT	11
#define SPI_CR_TFEI_MASK	(BIT11)
/* receive fifo over-run interrupt*/
#define SPI_CR_RFOI_SHIFT	10
#define SPI_CR_RFOI_MASK	(BIT10)
/* receive fifo full interrupt*/
#define SPI_CR_RFFI_SHIFT	9
#define SPI_CR_RFFI_MASK	(BIT9)
/* receive fifo empty interrupt*/
#define SPI_CR_RFEI_SHIFT	8
#define SPI_CR_RFEI_MASK	(BIT8)
/* threshold irq/dma selection*/
#define SPI_CR_TIDS_SHIFT	7
#define SPI_CR_TIDS_MASK	(BIT7)
/* interrupt enable*/
#define SPI_CR_IE_SHIFT		6
#define SPI_CR_IE_MASK		(BIT6)
/* module enable*/
#define SPI_CR_ME_SHIFT	5
#define SPI_CR_ME_MASK	(BIT5)
/* module fault error interrupt*/
#define SPI_CR_MFEI_SHIFT	4
#define SPI_CR_MFEI_MASK	(BIT4)
/* master/slave mode select*/
#define SPI_CR_MSMS_SHIFT	3
#define SPI_CR_MSMS_MASK	(BIT3)
/* clock polarity select*/
#define SPI_CR_CPS_SHIFT	2
#define SPI_CR_CPS_MASK	(BIT2)
/* clock phase select*/
#define SPI_CR_CPHS_SHIFT	1
#define SPI_CR_CPHS_MASK	(BIT1)
/* module fault error feature*/
#define SPI_CR_MFEF_SHIFT	0
#define SPI_CR_MFEF_MASK	(BIT0)
/* spi control register reset value*/
#define SPI_CR_RESET_MASK	 SPI_CR_MSMS_MASK


/************ status register *************/
/* rx fifo count*/
#define SPI_SR_RFCNT_SHIFT	 24
#define SPI_SR_RFCNT_MASK	 (BIT31|BIT30|BIT29|BIT28|BIT27|BIT26|BIT25|BIT24)
/* tx fifo count*/
#define SPI_SR_TFCNT_SHIFT	16
#define SPI_SR_TFCNT_MASK	(BIT23|BIT22|BIT21|BIT20|BIT19|BIT18|BIT17|BIT16)
/* tx fifo empty status*/
#define SPI_SR_TFES_SHIFT	15
#define SPI_SR_TFES_MASK	(BIT15)
/* receive fifo threshold passed interrupt*/
#define SPI_SR_RFTPI_SHIFT	14
#define SPI_SR_RFTPI_MASK	(BIT14)
/* transmit fifo threshold passed interrupt*/
#define SPI_SR_TFTPI_SHIFT	13
#define SPI_SR_TFTPI_MASK	(BIT13)
/* transmit fifo under-run interrupt*/
#define SPI_SR_TFUI_SHIFT	12
#define SPI_SR_TFUI_MASK	(BIT12)
/* transmit fifo empty interrupt*/
#define SPI_SR_TFEI_SHIFT	11
#define SPI_SR_TFEI_MASK	(BIT11)
/* receive fifo over-run interrupt*/
#define SPI_SR_RFOI_SHIFT	10
#define SPI_SR_RFOI_MASK	(BIT10)
/* receive fifo full interrupt*/
#define SPI_SR_RFFI_SHIFT	9
#define SPI_SR_RFFI_MASK	(BIT9)
/* receive fifo empty interrupt*/
#define SPI_SR_RFEI_SHIFT	8
#define SPI_SR_RFEI_MASK	(BIT8)
/* spi busy*/
#define SPI_SR_BUSY_SHIFT	7
#define SPI_SR_BUSY_MASK	(BIT7)
/* mode fault error interrupt*/
#define SPI_SR_MFEI_SHIFT	4
#define SPI_SR_MFEI_MASK	(BIT4)

/****** data format control register ******/
/* mode fault delay count*/
#define SPI_DFCR_MFDCNT_SHIFT	16
#define SPI_DFCR_MFDCNT_MASK	(BIT23|BIT22|BIT21|BIT20|BIT19|BIT18|BIT17|BIT16)
/* tx drive count*/
#define SPI_DFCR_TDCNT_SHIFT	8
#define SPI_DFCR_TDCNT_MASK	(BIT15|BIT14|BIT13|BIT12|BIT11|BIT10|BIT9|BIT8)
/* tx drive enable*/
#define SPI_DFCR_TDE_SHIFT	7
#define SPI_DFCR_TDE_MASK	(BIT7)
/* tx no data value*/
#define SPI_DFCR_TNDV_SHIFT	6
#define SPI_DFCR_TNDV_MASK	(BIT6)
/* direct ssn enable*/
#define SPI_DFCR_DSE_SHIFT		5
#define SPI_DFCR_DSE_MASK		(BIT5)
/* direct ssn value*/
#define SPI_DFCR_DSV_SHIFT		4
#define SPI_DFCR_DSV_MASK	(BIT4)
/* ssn control*/
#define SPI_DFCR_SC_SHIFT		3
#define SPI_DFCR_SC_MASK		(BIT3)
/* ssn port mode*/
#define SPI_DFCR_SPM_SHIFT	2
#define SPI_DFCR_SPM_MASK	(BIT2)
/* receive significant bit order*/
#define SPI_DFCR_RSBO_SHIFT	1
#define SPI_DFCR_RSBO_MASK	(BIT1)
/* transmit significant bit order*/
#define SPI_DFCR_TSBO_SHIFT	0
#define SPI_DFCR_TSBO_MASK	(BIT0)
/* spi data format control register reset value*/
#define SPI_DFCR_RESET_MASK	(SPI_DFCR_DSV_MASK|SPI_DFCR_DSE_MASK)

/* [rx1C] GPIO control register for spi */
#define GPIO_SPI0_SSB	              BIT4
#define GPIO_SPI0_CLK	              BIT3
#define GPIO_SPI0_SS	                      BIT2
#define GPIO_SPI0_MISO	              BIT1
#define GPIO_SPI0_MOSI	              BIT0
/* [rx48B] GPIO PULL EN for spi */
#define GPIO_SPI0_CLK_PULL_EN   BIT0
#define GPIO_SPI0_SS_PULL_EN	   BIT3
#define GPIO_SPI0_MISO_PULL_EN BIT1
#define GPIO_SPI0_MOSI_PULL_EN BIT2
/* [rx48B] GPIO PULL UP/DOWN for spi */
#define GPIO_SPI0_CLK_PULL_UP   BIT0
#define GPIO_SPI0_SS_PULL_UP	   BIT3
#define GPIO_SPI0_MISO_PULL_UP BIT1
#define GPIO_SPI0_MOSI_PULL_UP BIT2


/* SPI CFG setting*/
#define SPI_RX_SETTING              (DMA_WRAP_MODE | DEVICE_TO_MEM)
#define SPI_TX_SETTING              (DMA_WRAP_MODE)

#define SPI_8BITS_SETTING           (DMA_WRAP_1 | DMA_BURST_1 | DMA_SIZE_8)
#define SPI_16BITS_SETTING         (DMA_WRAP_1 | DMA_BURST_8 | DMA_SIZE_16)
#define SPI_32BITS_SETTING         (DMA_WRAP_1 | DMA_BURST_8 | DMA_SIZE_32)

#define SPI_RX_DMA_8BITS_CFG     (SPI_8BITS_SETTING  | SPI_RX_SETTING)
#define SPI_RX_DMA_16BITS_CFG   (SPI_16BITS_SETTING | SPI_RX_SETTING)
#define SPI_RX_DMA_32BITS_CFG   (SPI_32BITS_SETTING | SPI_RX_SETTING)

#define SPI_TX_DMA_8BITS_CFG      (SPI_8BITS_SETTING  | SPI_TX_SETTING)
#define SPI_TX_DMA_16BITS_CFG    (SPI_16BITS_SETTING | SPI_TX_SETTING)
#define SPI_TX_DMA_32BITS_CFG    (SPI_32BITS_SETTING | SPI_TX_SETTING)

#define SPI_RX_DMA_CFG      (SPI_RX_DMA_8BITS_CFG)
#define SPI_TX_DMA_CFG      (SPI_TX_DMA_8BITS_CFG)

#define SPI0_TX_FIFO                0xD8240010
#define SPI0_RX_FIFO                0xD8240030 
#define SPI1_TX_FIFO                0xD8250010
#define SPI1_RX_FIFO                0xD8250030 


enum spi_arbiter_type
{
	SPI_ARBITER_MASTER,	
	SPI_ARBITER_SLAVE,	
	SPI_ARBITER_MAX
};

enum spi_clk_mode_type
{
	SPI_CLK_MODE_0,
	SPI_CLK_MODE_1,
	SPI_CLK_MODE_2,
	SPI_CLK_MODE_3,
	SPI_CLK_MODE_MAX
};

enum spi_operation_mode_type
{
	SPI_POLLING_MODE,
	SPI_INTERRUPT_MODE,
	SPI_DMA_MODE,
	SPI_OP_MAX
};

enum spi_ssn_port_mode_type
{
	SPI_SSN_PORT_MM,
	SPI_SSN_PORT_PTP,
	SPI_SSN_PORT_MAX
};

enum spi_ssn_type
{
	SPI_SS0,
	SPI_SS1,
	SPI_SS2,
	SPI_SS3,
	SPI_SSN_MAX	
};

enum spi_fifo_threshold_type
{
	SPI_THRESHOLD_16BYTES,
	SPI_THRESHOLD_8BYTES,
	SPI_THRESHOLD_MAX
};

struct spi_reg
{
	unsigned int volatile *cr; 	    /*control register*/
	unsigned int volatile *sr;	            /*status register*/
	unsigned int volatile *dfcr;	    /*data format control register*/
	unsigned int volatile *cre;          /*extended control register*/
	unsigned char volatile *rfifo;	    /*read fifo, i.e. receive fifo*/
	unsigned char volatile *wfifo;     /*write fifo, i.e. transfer fifo*/
	unsigned int volatile *srcv_cnt;  /*spi receive count/tx drive count*/
	unsigned int volatile *srcv_add_cnt; 
};

struct spi_dma
{
	struct dma_req_s config;
	//dmach_t dmach;
	unsigned int xfer_cnt;			 /* the number of bytes transfers by dma*/
	wait_queue_head_t event_queue;   /* event queue for controling dma complete*/
	volatile int event_ack;			 /* event ack for notifing dma complete*/
	spinlock_t spinlock;			 /* spin lock*/
};


struct spi_port
{
	int port;							/* spi port number*/
	struct semaphore port_sem;		        /* prevent from multi user confiliciton*/
	wait_queue_head_t tx_event_queue;	/* event queue for controling write out complete*/
	wait_queue_head_t rx_event_queue;	/* event queue for controling read in complete*/
	volatile int tx_event_ack;			/* event ack for notifing dma complete*/
	volatile int rx_event_ack;			/* event ack for notifing dma complete*/
	unsigned int cur_tx_cnt;			/* current write out bytes*/
	unsigned int cur_rx_cnt;			/* current read in bytes*/
	struct elite_spi_device *user;		/* spi user*/
	struct spi_reg regs;				/* spi register set*/
	/* interrupt and dma control*/
	struct spi_dma rdma;				/* read in dma information*/
	struct spi_dma wdma;				/* write out dma information*/
	int	irq_num;				                /* irq number*/
};

struct spi_xfer_req_s
{
	unsigned char *rbuf;	/* read in buffer address*/
	unsigned int rsize;		/* read in data size*/
	unsigned char *wbuf;	/* write out data address*/
	unsigned int wsize;		/* write out data size*/
	unsigned int size;		/* total size which use when read and write control by upper AP*/
};


struct elite_spi_device
{
	unsigned char name[9];
	struct elite_spi_controller *controller;
	unsigned char client_init;
	dma_addr_t phys_addr_r;			/* I/O data physical address for dma read in*/
	dma_addr_t phys_addr_w;			/* I/O data physical address for dma write out*/
	unsigned char *ior;					/* I/O data buffer for dma read in*/
	unsigned char *iow;				/* I/O data buffer for dma write out*/
	unsigned char dma_mem_init;		/* flag for recording DMA hardware/resource initial completed*/
	enum spi_fifo_threshold_type tx_fifo_threshold;   /*set tx fifo threshold*/
	enum spi_fifo_threshold_type rx_fifo_threshold;   /*set rx fifo threshold*/
	unsigned char *rbuf;				/* I/O data buffer for polling read in*/
	const unsigned char *wbuf;			/* I/O data buffer for polling write out*/
	unsigned int freq;					/* bus frequence, Unit Khz*/
	enum spi_clk_mode_type clk_mode;	/* bus clock mode*/
	enum spi_operation_mode_type op_mode;/*operation mode, polling/irq/dma*/
	/* use for output setting value when fifo goes empty*/
	unsigned char tx_drive_count;		/* the number of bytes to send out in TX drive function*/
	unsigned char tx_drive_enable;		/* enable/disable TX drive*/
	unsigned char tx_nodata_value;		/* output value, o => 0x0, 1 =>0xff*/
	/* user for master and slave control*/
	enum spi_arbiter_type arbiter;
	enum spi_ssn_port_mode_type ssn_port_mode;/* ssn port mode. 0 => multi-master, 1 => P2P*/
	enum spi_ssn_type slave_select;	/* output ssn pin {a,b,c,d}*/
	unsigned int rx_cnt;			/* the number of byte receives*/
	unsigned int tx_cnt;			/* the number of byte transfers*/
	void (*callback)(unsigned char *);
};


struct elite_spi_controller 
{
	struct completion	done;
	int 		bus_num;	                   /* bus_num : 0,1,2*/
	int		irq;
	int		len;
	int		count;
	unsigned 	state;
	void __iomem	*mmio_base;
	unsigned 	char   cs_change;
	struct clk*              clk;
	struct spi_master	*master;
	struct device		*dev;
	struct elite_spi_platform_data *pdata;
	struct elite_spi_device *curdev;       /* current device which is using spi module */
	struct semaphore port_sem;		    /* prevent from multi user confiliciton*/
	wait_queue_head_t tx_event_queue;/* event queue for controling write out complete*/
	wait_queue_head_t rx_event_queue;/* event queue for controling read in complete*/
	volatile int tx_event_ack;		    /* event ack for notifing dma complete*/	
	volatile int rx_event_ack;		     /* event ack for notifing dma complete*/
	unsigned int cur_tx_cnt;		    /* current write out bytes*/
	unsigned int cur_rx_cnt;		    /* current read in bytes*/
	struct spi_reg regs;			    /* spi register set*/
	/* interrupt and dma control*/
	struct spi_dma rdma;	                   /* read in dma information*/
	struct spi_dma wdma;	                    /* write out dma information*/

	struct workqueue_struct *workqueue;
	struct work_struct           work;
	struct list_head               queue;
	spinlock_t                       lock;
};


struct elite_spi_platform_data 
{
	int			pin_cs;	              /* simple gpio cs */
	unsigned int	num_cs;	              /* total chipselects */
	int			bus_num;              /* bus number to use. */
	struct spi_device	*spidev_elite;
	struct spi_board_info	info;
};

void set_callback_func(struct elite_spi_device *spi_user, void (*callback)(unsigned char *data));

#endif

