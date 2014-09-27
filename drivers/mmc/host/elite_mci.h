#ifndef	__MMC_ELITE_H
#define	__MMC_ELITE_H

#include <linux/wait.h>
#include <linux/pinctrl/consumer.h>

#define	DRIVER_NAME	"elite-mci"

#define HOST_SPEC_10	0
#define HOST_SPEC_20	1
#define HOST_SPEC_30	2


#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80
#define BIT8 (1<<8)
#define BIT9 (1<<9)
#define BIT10 (1<<10)
#define BIT11 (1<<11)
#define BIT12 (1<<12)
#define BIT13 (1<<13)
#define BIT14 (1<<14)
#define BIT15 (1<<15)
#define BIT23 (1<<23)

#define	MMC_ERR_NONE		0	/* Only for Android kernel 2.6.29, in MVL5, this variable is defined in mmc.h */
#define	MMC_ERR_FAILED	4	/* Only for Android kernel 2.6.29, in MVL5, this variable is defined in mmc.h */
/**************	INT	and	Base address*****************/
/*special	settings for each	one*/

#define	MMC_ELITE_BASE				0xd800a000


/*=============================================*/
/* ELITE MMC HOST Register Offset */
/*=============================================*/
#define	CTRL			0x00	/* 1 byte. command type register */
#define	CMD_IDX		    0x01	/* 1 byte. command index register */
#define	RSP_TYPE		0x02	/* 1 byte. response type regiser */
#define	CMD_ARG		    0x04	/* 4 bytes. command argument register */
#define	BUS_MODE		0x08	/* 1 byte. bus mode register */
/* SD 3.0 Host Contrller Only */
#define XBUS_MODE 	    0x09	/* 1 byte. extended bus mode register */
/* SD 3.0 Host Contrller Only */
#define HOST_CTRL2	    0x0a	/* 2 bytes. host control2 register */
#define	BLK_LEN			0x0c	/* 2 bytes. block length register */
#define	BLK_CNT		    0x0e	/* 2 bytes. block count register */
#define	RSP_0			0x10	/* 1 bytes. response register */
#define	RSP_1			0x11	/* 1 bytes*/
#define	RSP_2			0x12	/* 1 bytes*/
#define	RSP_3			0x13	/* 1 bytes*/
#define	RSP_4			0x14	/* 1 bytes*/
#define	RSP_5			0x15	/* 1 bytes*/
#define	RSP_6			0x16	/* 1 bytes*/
#define	RSP_7			0x17	/* 1 bytes*/
#define	RSP_8			0x18	/* 1 bytes*/
#define	RSP_9			0x19	/* 1 bytes*/
#define	RSP_10			0x1a	/* 1 bytes*/
#define	RSP_11			0x1b	/* 1 bytes*/
#define	RSP_12			0x1c	/* 1 bytes*/
#define	RSP_13			0x1d	/* 1 bytes*/
#define	RSP_14			0x1e	/* 1 bytes*/
#define	RSP_15			0x1f	/* 1 bytes*/
/* SD2.0 different from SD 3.0 */
#define	CURBLK_CNT	    	0x20	/* 2 bytes. current block count register */
#define FSM_STATE		0x22	/* 1 byte. FSM state register */
#define BUS_LEVEL		0x23	/* 1 byte. bus level register */
#define	INT_MASK_0		0x24	/* 1 byte*/
#define	INT_MASK_1		0x25	/* 1 byte*/
#define INTR_MASK01	    	0x24	/* 4 bytes */
#define	SD_STS_0		0x28	/* 1 byte. SD status register */
#define	SD_STS_1		0x29	/* 1 byte*/
#define	SD_STS_2		0x2a	/* 1 byte*/
#define	SD_STS_3		0x2b	/* 1 byte*/
#define	RSP_TOUT		0x2c	/* 1 byte. response timeout register */
#define DAT_TOUT		0x2d	/* 3 bytes. data timeout register. implemented but not used */
/* SD 3.0 Host Contrller Only */
#define	CLK_SEL			0x30	/* 1 byte. clock selection register. Not used */
#define	EXT_CTRL0		0x34	/* 1 byte. extended control register */
/* SD 3.0 Host Contrller Only */
#define EXT_CTRL1		0x35	/* 1 byte. */
/* SD 3.0 Host Contrller Only */
#define EXT_CTRL2		0x36	/* 1 byte. */

#define	SHDW_BLKLEN 	0x38	/* 2 bytes. shadowed block length register */
/* SD 3.0 Host Contrller Only */
#define MAN_TUNE_VAL	0x3a	/* 1 byte. MANNUL_TUNE_VALUE register */
/* SD 3.0 Host Contrller Only */
#define SD_WR_TUNE		0x3b	/* 1 byte. SD_WRITE_TUNING register */

#define	TIMER_VAL		0x3c	/* 2 bytes. timer value register */

/*
  * PDMA registers offset
  */
#define   DMA_GCR		0x100	/* DMA global control register */
#define   DMA_IER		0x104	/* DMA Interrupt Enable Register */
#define   DMA_ISR		0x108	/* DMA Interrupt Status Register */
#define   DMA_DESPR		0x10c	/* DMA Memory-Register Pointer Register */
#define   DMA_RBR		0x110	/* DMA Residual Bytes Register for Interface 0 */
#define   DMA_DAR		0x114	/* DMA Data Address Register for Interface 0 */
#define   DMA_BAR		0x118	/* DMA Branch Address Register for Interface 0 */
#define   DMA_CPR		0x11c	/* DMA Command Pointer Register for Interface 0 */
#define   DMA_CxCR		0x120	/* DMA Context Control Register */

/*
 * Host controller registers bit filed definition
 */
  
 /* Control Register */
#define	CMD_START	BIT0
#define	SPISTP	    BIT1
#define	XFER_RD_WR	BIT2
#define	FFRST		BIT3
#define	CMD_TYPE_MASK   0xf
#define CMD_TYPE_SHIFT  4

/* Command Index Register */

/* Response Type	Register */
#define	RT	(BIT0|BIT1|BIT2|BIT3)
#define	RY	BIT4

/* Command Argument	Register 0,1,2,3 */

/* Bus Mode Register */
#define	SPI			BIT0
#define	BUS_WIDTH_4	BIT1
#define	RW			BIT2
#define	SPICRC		BIT3
#define	HOST_CLK_EN	BIT4 //CST: SD Host Clock Stop
#define	SPICS		BIT5
#define	SDPWR		BIT6
#define	SFTRST		BIT7

/* Block Length Register */
#define HOST_INTR_EN    BIT15
#define DAT3_DETECT_CARD_EN BIT14
#define GPI_DETECT_CARD_EN  BIT13
#define POL_OF_CARD_DETECT  BIT12
#define TRANSACTION_ABORT   BIT11

/* Extended Bus Mode Register */
#define XBUS_DDR50_MODE_EN	BIT4
#define XBUS_BLK_nBYTE		BIT5
#define XBUS_BOOT_OP_CTRL	BIT0

/* host control2 register */
#define   CTRL2_DRV_TYPE_B		0x0000
#define   CTRL2_DRV_TYPE_A		0x0100
#define   CTRL2_DRV_TYPE_C		0x0200
#define   CTRL2_DRV_TYPE_D		0x0300

#define   CTRL2_SIG_VDD_180		BIT3
#define   CTRL2_UHS_SDR12		0x00
#define   CTRL2_UHS_SDR25		0x01
#define   CTRL2_UHS_SDR50		0x02
#define   CTRL2_UHS_SDR104		0x03
#define   CTRL2_UHS_DDR50		0x04
#define   CTRL2_SDR50_TUNNING_EN	BIT4
#define   CTRL2_TUNING_MANUL	BIT5
#define   CTRL2_EXEC_TUNING	BIT6
#define   CTRL2_SAMPLING_CLK_SEL	BIT7

#define   CTRL2_SAMPLE_CLK_SEL	BIT7

/* Interrupt Mask	Register 0 */
#define	THIE					BIT0
#define	TEIE					BIT1
#define	TAIE					BIT2
#define	RHIE					BIT3
#define	MULTI_XFER_DONE_EN	    BIT4
#define	BLOCK_XFER_DONE_EN	    BIT5
#define	CDIE					BIT6
#define	DEVICE_INSERT_EN		BIT7

/* Interrupt Mask	Register 1 */
#define	SDIO_EN				BIT0
#define	RSP_DONE_EN			BIT1
#define	RSP_TIMEOUT_EN		BIT2
#define	AUTO_STOP_EN		BIT3
#define	DATA_TIMEOUT_EN		BIT4
#define	RSP_CRC_ERR_EN		BIT5
#define	READ_CRC_ERR_EN		BIT6
#define	WRITE_CRC_ERR_EN	BIT7

/* Interrupt Mask Register */
#define INTR_THIE					(1<<0)
#define INTR_TEIE					(1<<1)	
#define INTR_TAIE					(1<<2)
#define INTR_RHIE					(1<<3)
#define INTR_MULTI_XFER_DONE		(1<<4)
#define INTR_BLOCK_XFER_DONE		(1<<5)
#define INTR_CARD_DETECT			(1<<6)
#define INTR_CARD_INSERT			(1<<7)
#define INTR_SDIO_EN				(1<<8)
#define INTR_RSP_DONE				(1<<9)
#define INTR_RSP_TIMEOUT			(1<<10)
#define INTR_AUTO_STOP				(1<<11)
#define INTR_DATA_TIMEOUT			(1<<12)
#define INTR_RSP_CRC_ERR			(1<<13)
#define INTR_READ_CRC_ERR			(1<<14)
#define INTR_WRITE_CRC_ERR		    (1<<15)

#define INTR_ALL_MASK               ((unsigned short)-1)

/* SD	Status Register	0 */
#define	TH					BIT0
#define	WRITE_PROTECT			BIT1	/* 1.	write	protected	is disabled.*/
#define	CARD_NOT_IN_SLOT		BIT2
#define	CARD_NOT_IN_SLOT_GPI	BIT3
#define	MULTI_XFER_DONE		BIT4
#define	BLOCK_XFER_DONE		BIT5
#define	SD_CD					BIT6
#define	DEVICE_INSERT			BIT7

/* SD	Status Register	1 */
#define	SDIO_INT			BIT0
#define	RSP_DONE			BIT1
#define	RSP_TIMEOUT		BIT2
#define	AUTO_STOP			BIT3
#define	DATA_TIMEOUT		BIT4
#define	RSP_CRC_ERR		BIT5
#define	READ_CRC_ERR		BIT6
#define	WRITE_CRC_ERR		BIT7

/* SD	Status Register	2 */
#define	RSP_BUSY			BIT5
#define	CLK_FREEZ_STS		BIT6
#define	CLK_FREEZ_EN		BIT7

/* Extended Control Register (Offset 0x34) */
#define   EXT_CTRL_GEN_AUTO_STOP_CMD BIT0
#define   EXT_CTRL_WIDTH_8	BIT2
#define   EXT_CTRL_HI_SPEED	BIT7
#define   EXT_CTRL_AUTOADKSTART	BIT12
#define   EXT_CTRL_RAUTOADJDONE	BIT11
#define   EXT_CTRL_RAUTOADJFAIL BIT23

/* SD	Response types*/
#define	TYPE_R0	0		/* NONE	response*/
#define	TYPE_R1	1		/* Basic response	format*/
#define	TYPE_R2	2		/* R2	response.	Used by	ALL_SEND_CID(CMD2),*/
/* SEND_CID(CMD10) and SEND_CSD(CMD9)*/
#define	TYPE_R3	3		/* R3	response.	Used by	SEND_APP_OP_COND(ACMD41)*/
#define	TYPE_R4	4		/* R4	response.	Used by	IO_SEND_OP_COND(CMD5) for SDIO */
#define	TYPE_R5	5		/* R5	response.	Used by	IO_RW_DIRECT(CMD52) & IO_RW_EXTENDED(CMD53) for SDIO */
#define	TYPE_R6	6		/* R6	response.	Used by	SEND_RELATIVE_ADDR(CMD3)*/
#define	TYPE_R7	7		/* R6	response.	Used by	SEND_RELATIVE_ADDR(CMD3)*/
#define	TYPE_R1b 	9


/*
 * DMA usage const for Rx08[Config]
 */
#define DMA_CFG_WRITE   0x0
#define DMA_CFG_READ    0x10000

/*
 * DMA	Global Control Register
 */
#define	DMA_GCR_DMA_EN	 	0x00000001	/* [0] --	DMA controller enable*/
#define	DMA_GCR_SOFTRESET 	0x00000100	/* [8] --	Software rest*/

/*
 * DMA	Interrupt	Enable Register
 */
#define	DMA_IER_INT_EN 		0x00000001 /* [0] --DMA interrupt enable */
/*
 * DMA	Interrupt	Status Register
 */
#define	DMA_IER_INT_STS 	0x00000001 /* [0] --	DMA interrupt status */
/*
 * DMA	Residual Bytes Register
 */
#define	DMA_RBR_End		 	0x80000000	/* [0] --	DMA	interrupt	status */
#define	DMA_RBR_Format	 	0x40000000	/* [0] --	DMA	interrupt	status */

/*
 * DMAContext Control Register	for	Channel	0
 */
#define DMA_READ					0x00
#define DMA_WRITE					0x01
#define DMA_CCR_RUN					0x00000080
#define DMA_CCR_IF_TO_PERIPHERAL	0x00000000
#define DMA_CCR_PERIPHERAL_TO_IF	0x00400000
#define DMA_CCR_EVTCODE				0x0000000f
#define DMA_CCR_EVENT_NO_STATUS		0x00000000
#define DMA_CCR_EVT_FF_UNDERRUN		0x00000001
#define DMA_CCR_EVT_FF_OVERRUN		0x00000002
#define DMA_CCR_EVT_DESP_READ		0x00000003
#define DMA_CCR_EVT_DATA_RW			0x00000004
#define DMA_CCR_EVT_EARLY_END		0x00000005
#define DMA_CCR_EVT_SUCCESS			0x0000000f

/*
 * Mask & shift
 */
#define  DATA_LVL_MASK	0x0f
#define  CTRL2_UHS_MODE_MASK	0x07
#define  BLK_LEN_MASK	0x3ff


/*
 *	SD PDMA	-	DMA_GCR	:	DMA	Global Control Register
 */
#define	SD_PDMA_GCR_DMA_EN	 	0x00000001	/* [0] --	DMA controller enable*/
#define	SD_PDMA_GCR_SOFTRESET 	0x00000100	/* [8] --	Software rest*/

/*
 *	SD PDMA	-	DMA_IER	:	DMA	Interrupt	Enable Register
 */
#define	SD_PDMA_IER_INT_EN 		0x00000001 /* [0] --DMA interrupt enable */
/*
 *	SD PDMA	-	DMA_ISR	:	DMA	Interrupt	Status Register
 */
#define	SD_PDMA_IER_INT_STS 	0x00000001 /* [0] --	DMA interrupt status */
/*
 *	SD PDMA	-	DMA_DESPR	:	DMA	Descriptor base	address	Pointer	Register
 */

/*
 *	SD PDMA	-	DMA_RBR	:	DMA	Residual Bytes Register
 */
#define	SD_PDMA_RBR_End		 	0x80000000	/* [0] --	DMA	interrupt	status */
#define	SD_PDMA_RBR_Format	 	0x40000000	/* [0] --	DMA	interrupt	status */

/*
 *	SD PDMA	-	DMA_CCR	:	DMAContext Control Register	for	Channel	0
 */
#define SD_PDMA_READ					0x00
#define SD_PDMA_WRITE					0x01
#define SD_PDMA_CCR_RUN					0x00000080
#define SD_PDMA_CCR_IF_TO_PERIPHERAL	0x00000000
#define SD_PDMA_CCR_PERIPHERAL_TO_IF	0x00400000
#define SD_PDMA_CCR_EVTCODE				0x0000000f
#define SD_PDMA_CCR_EVENT_NO_STATUS		0x00000000
#define SD_PDMA_CCR_EVT_FF_UNDERRUN		0x00000001
#define SD_PDMA_CCR_EVT_FF_OVERRUN		0x00000002
#define SD_PDMA_CCR_EVT_DESP_READ		0x00000003
#define SD_PDMA_CCR_EVT_DATA_RW			0x00000004
#define SD_PDMA_CCR_EVT_EARLY_END		0x00000005
#define SD_PDMA_CCR_EVT_SUCCESS			0x0000000f


#define	MAX_DESC_NUM 256

enum opmode {
    OP_NONE = 0,
    OP_READ = 1,
    OP_WRITE = 2,
};

/*
 *	PDMA Descriptor	short
 */
struct sd_pdma_desc_s {
	unsigned long volatile req_count : 16;		/* bit 0 -15 -Request count             */
	unsigned long volatile i : 1;				/* bit 16    -interrupt                 */
	unsigned long volatile reserve : 13;		/* bit 17-29 -reserved                  */
	unsigned long volatile format : 1;		/* bit 30    -The descriptor format     */
	unsigned long volatile end : 1;			/* bit 31    -End flag of descriptor list*/
	unsigned long volatile *databuffer_addr;	/* bit 31    -Data Buffer address       */
};

/*
 *	PDMA Descriptor	long
 */
struct sd_pdma_desc_l {
	unsigned long	volatile req_count:16;		/* bit 0-15-Request count	*/
	unsigned long	volatile i:1;				/* bit 16	-interrupt*/
	unsigned long	volatile reserve:13;		/* bit 17-29-reserved*/
	unsigned long	volatile format:1;		/* bit 30-The descriptor format */
	unsigned long	volatile end:1;			/* bit 31	-End	flag of descriptor list*/
	unsigned long	volatile *databuffer_addr;	/* bit 31-0	 -Data Buffer	address*/
	unsigned long	volatile *branch_addr;		/* bit 31-2-Descriptor Branch address*/
	unsigned long	volatile reserve0;		/* bit 31-0-reserved*/
};

/*=========================================*/
/* structure definition.*/
/*=========================================*/
struct elite_host {
	struct device *dev;
	struct mmc_host	*mmc;
	unsigned int host_ver;
	spinlock_t lock;
	struct resource *res;
	struct clk *mclk;
	void	__iomem		*base;
	int	 regular_irq;
	int	 dma_irq;
	unsigned long	*desc_viraddr;
	dma_addr_t desc_phyaddr;
	unsigned int desc_size;
	unsigned long	dms_intmask;
	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_data *data;
	u32 pwr;
	struct mmc_platform_data *plat;
	unsigned int sg_len;
	/* pio stuff */
	struct scatterlist *sg_ptr;
	unsigned int sg_off;
	unsigned int size;
	u32 opcode;
	unsigned char	soft_timeout;
	void *done_data;	/* completion	data */
	void (*done)(void *data);/*	completion function	*/
	wait_queue_head_t sblk_xfer_done;
	unsigned int tuning_done;
	int current_clock;
	bool runtime_suspended; /* Host is runtime suspended */
	struct regulator *sig_reg;	/* Signaling regulator */
	/* Pin modes */
	struct pinctrl *pinctrl_p;
	struct pinctrl_state *pinctrl_def;
	struct pinctrl_state *pinctrl_pulld;
	int pwrsw_pin;
	u32 reg_blk_len;
	u8  reg_int_mask_0;
};

static inline void elite_writel(struct elite_host *host, u32 val, int reg)
{
	writel(val, host->base + reg);
}

static inline void elite_writew(struct elite_host *host, u16 val, int reg)
{
	writew(val, host->base + reg);
}

static inline void elite_writeb(struct elite_host *host, u8 val, int reg)
{
	writeb(val, host->base + reg);
}

static inline u32 elite_readl(struct elite_host *host, int reg)
{
	return readl(host->base + reg);
}

static inline u16 elite_readw(struct elite_host *host, int reg)
{
	return readw(host->base + reg);
}

static inline u8 elite_readb(struct elite_host *host, int reg)
{
	return readb(host->base + reg);
}


extern int elite_mmc_init_short_desc(unsigned long *desc_addr, unsigned int req_count,
		unsigned long *buffer_addr, int end);
extern int elite_mmc_init_long_desc(unsigned long *desc_addr, unsigned int req_count,
		unsigned long *buffer_addr, unsigned long *branch_addr, int end);

#endif	/* __MMC_elite_H */
