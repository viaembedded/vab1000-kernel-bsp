/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slimbus/slimbus.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
//#include <mach/sps.h>

/* Per spec.max 40 bytes per received message */

/*
 * Need enough descriptors to receive present messages from slaves
 * if received simultaneously. Present message needs 3 descriptors
 * and this size will ensure around 10 simultaneous reports.
 */
#define MSM_SLIM_DESC_NUM		32
#define SLIM_RX_MSGQ_BUF_LEN	40

#define SLIM_USR_MC_GENERIC_ACK		0x25
#define SLIM_USR_MC_MASTER_CAPABILITY	0x0
#define SLIM_USR_MC_REPORT_SATELLITE	0x1
#define SLIM_USR_MC_ADDR_QUERY		0xD
#define SLIM_USR_MC_ADDR_REPLY		0xE
#define SLIM_USR_MC_DEFINE_CHAN		0x20
#define SLIM_USR_MC_DEF_ACT_CHAN	0x21
#define SLIM_USR_MC_CHAN_CTRL		0x23
#define SLIM_USR_MC_RECONFIG_NOW	0x24
#define SLIM_USR_MC_REQ_BW		0x28
#define SLIM_USR_MC_CONNECT_SRC		0x2C
#define SLIM_USR_MC_CONNECT_SINK	0x2D
#define SLIM_USR_MC_DISCONNECT_PORT	0x2E

#define SLIM_MSG_ASM_FIRST_WORD(l, mt, mc, dt, ad) \
		((l) | ((mt) << 5) | ((mc) << 8) | ((dt) << 15) | ((ad) << 16))

#define MSM_SLIM_NAME	"elite-slim"
#define SLIM_ROOT_FREQ 24576000

#define MSM_CONCUR_MSG	8
#define SAT_CONCUR_MSG	8
#define DEF_WATERMARK	(8 << 1)
#define DEF_ALIGN	0
#define DEF_PACK	(1 << 6)
#define ENABLE_PORT	1

#define DEF_BLKSZ	0
#define DEF_TRANSZ	0

#define SAT_MAGIC_LSB	0xD9
#define SAT_MAGIC_MSB	0xC5
#define SAT_MSG_VER	0x1
#define SAT_MSG_PROT	0x1
#define MSM_SAT_SUCCSS	0x20

#define QC_MFGID_LSB	0x2
#define QC_MFGID_MSB	0x17
#define QC_CHIPID_SL	0x10
#define QC_DEVID_SAT1	0x3
#define QC_DEVID_SAT2	0x4
#define QC_DEVID_PGD	0x5
#define QC_MSM_DEVS	5


#define DCR_INFO_7		0x20000
#define DCR_INFO_6		0x10000
#define DCR_INFO_5		0x8000
#define DCR_INFO_4		0x4000
#define DCR_INFO_3		0x2000
#define DCR_INFO_2		0x1000
#define DCR_INFO_1		0x800
#define DCT_INFO_7		0x400
#define DCT_INFO_6		0x200
#define DCT_INFO_5		0x100
#define DCT_INFO_4		0x80
#define DCT_INFO_3		0x40
#define DCT_INFO_2		0x20
#define DCT_INFO_1		0x10
#define SMR_INFO		0x8
#define SMT_INFO		0x4
#define FR_INFO			0x2
#define FL_INFO			0x1

#define MC_TX_COL		0x1
#define RECEIVED_MESSAGE		0x4
#define RECEIVED_RECONFIGURE		0x8

#define MC_TX_COL		0x1
#define PACK		0x2
#define NACK		0x4
#define NORE		0x8
#define UDEF		0x10

//subframe mode
#define SM_CSW8_SFL8           0x0
#define SM_CSW1_SFL6           0x4
#define SM_CSW1_SFL8           0x5
#define SM_CSW1_SFL24          0x6
#define SM_CSW1_SFL32          0x7
#define SM_CSW2_SFL6           0x8
#define SM_CSW2_SFL8           0x9
#define SM_CSW2_SFL24          0xa
#define SM_CSW2_SFL32          0xb
#define SM_CSW3_SFL6           0xc
#define SM_CSW3_SFL8           0xd 
#define SM_CSW3_SFL24          0xe
#define SM_CSW3_SFL32          0xf
#define SM_CSW4_SFL6           0x10
#define SM_CSW4_SFL8           0x11
#define SM_CSW4_SFL24          0x12
#define SM_CSW4_SFL32          0x13
#define SM_CSW6_SFL8           0x15
#define SM_CSW6_SFL24          0x16
#define SM_CSW6_SFL32          0x17
#define SM_CSW8_SFL24          0x18
#define SM_CSW8_SFL32          0x19
#define SM_CSW12_SFL24         0x1a
#define SM_CSW16_SFL24         0x1c
#define SM_CSW12_SFL32         0x1b
#define SM_CSW16_SFL32         0x1d
#define SM_CSW24_SFL32         0x1f

// root frequency
#define RF_15d36M		0x3
#define RF_16d8M		0x4
#define RF_19d2M		0x5
#define RF_22d5792M		0x2
#define RF_24M			0x6
#define RF_24d576M		0x1
#define RF_25M			0x7
#define RF_26M			0x8
#define RF_27M			0x9

// presence rate
#define PR_12K			0x1
#define PR_24K			0x2
#define PR_48K			0x3
#define PR_96K			0x4
#define PR_192K			0x5
#define PR_384K			0x6
#define PR_768K			0x7

#define PR_11d025K		0x9
#define PR_22d05K		0xA
#define PR_44d1K		0xB
#define PR_88d2K		0xC
#define PR_176d4K		0xD
#define PR_352d8K		0xE
#define PR_705d6K		0xF

#define PR_4K  	     		0x10
#define PR_8K  	     		0x11
#define PR_16K  	     	0x12
#define PR_32K  	     	0x13
#define PR_64K 	     		0x14
#define PR_128K 	     	0x15
#define PR_256K 	     	0x16
#define PR_512K 	     	0x17



struct dma_mem_buffer {
	void *base;
	u32 phys_base;
	u32 size;
	u32 min_size;
};
/* VIA SLIM registers */
enum mgr_reg {
	SLIM_CMP_REVISON		= 0x0,
	SLIM_CMP_HWINFO	= 0x4,
	SLIM_CMP_SYSCONFIG	= 0x10,
	SLIM_CMP_IRQSTATUS	= 0x28,
	SLIM_CMP_IRQENABLE_SET	= 0x2C,
	SLIM_CMP_IRQENABLE_CLR	= 0x30,
	SLIM_CMP_DMAENABLE_SET	= 0x34,
	SLIM_CMP_DMAENABLE_CLR	= 0x38,
	SLIM_CMP_DMA_SEL	= 0x3C,
	SLIM_CMP_IV	= 0x40,
	SLIM_CMP_MI_PC	= 0x44,
	SLIM_SMT_INFO	= 0x50,
	SLIM_SMT_MESSAGE	= 0x54,
	SLIM_SMT_CONTROL	= 0x58,
	SLIM_SMT_FIFO_SETUP	= 0x5C,
	SLIM_SMR_INFO	= 0x60,
	SLIM_SMR_MESSAGE	= 0x64,
	SLIM_SMR_CONTROL	= 0x68,
	SLIM_SMR_FIFO_SETUP	= 0x6C,
	SLIM_FL_INFO	= 0x70,
	SLIM_FL_CONTROL	= 0x74,
	SLIM_FL_SM	= 0x78,
	SLIM_FL_CG	= 0x7C,
	SLIM_FL_RF	= 0x80,
	SLIM_FR_INFO	= 0x90,
	SLIM_FR_CONTROL	= 0x98,
	SLIM_FR_CLOCK_PAUSE	= 0xA0,
	SLIM_DEV_LA_i	= 0x100,
	SLIM_DEV_EA_LO_i	= 0x104,
	SLIM_DEV_EA_HI_i	= 0x108,
	SLIM_DCT_INFO_j	= 0x200,
	SLIM_DCT_FIFO_SETUP1_j	= 0x204,
	SLIM_DCT_FIFO_SETUP2_j	= 0x208,
	SLIM_DCT_FIFO_STATUS_j	= 0x20C,
	SLIM_DCT_MAP_j	= 0x210,
	SLIM_DCT_CONFIG1_j	= 0x214,
	SLIM_DCT_CONFIG2_j	= 0x218,
	SLIM_DCR_INFO_j	= 0x300,
	SLIM_DCR_FIFO_SETUP1_j	= 0x304,
	SLIM_DCR_FIFO_SETUP2_j	= 0x308,
	SLIM_DCR_FIFO_STATUS_j	= 0x30C,
	SLIM_DCR_MAP_j	= 0x310,
	SLIM_DCR_CONFIG1_j	= 0x314,
	SLIM_DCR_CONFIG2_j	= 0x318,
	SLIM_DCT_DATA_j	= 0x1000,
	SLIM_DCR_DATA_j	= 0x1800,
};

/* Component registers */
enum comp_reg {
	COMP_CFG	= 0,
	COMP_TRUST_CFG	= 0x14,
};

enum msg_cfg {
	MGR_CFG_ENABLE		= 1,
	MGR_CFG_RX_MSGQ_EN	= 1 << 1,
	MGR_CFG_TX_MSGQ_EN_HIGH	= 1 << 2,
	MGR_CFG_TX_MSGQ_EN_LOW	= 1 << 3,
};
/* Message queue types */
enum msm_slim_msgq_type {
	MSGQ_RX		= 0,
	MSGQ_TX_LOW	= 1,
	MSGQ_TX_HIGH	= 2,
};
/* Framer registers */
enum frm_reg {
	FRM_CFG		= 0x400,
	FRM_STAT	= 0x404,
	FRM_INT_EN	= 0x410,
	FRM_INT_STAT	= 0x414,
	FRM_INT_CLR	= 0x418,
	FRM_WAKEUP	= 0x41C,
	FRM_CLKCTL_DONE	= 0x420,
	FRM_IE_STAT	= 0x430,
	FRM_VE_STAT	= 0x440,
};

/* Interface registers */
enum intf_reg {
	INTF_CFG	= 0x600,
	INTF_STAT	= 0x604,
	INTF_INT_EN	= 0x610,
	INTF_INT_STAT	= 0x614,
	INTF_INT_CLR	= 0x618,
	INTF_IE_STAT	= 0x630,
	INTF_VE_STAT	= 0x640,
};

/* Manager PGD registers */
enum pgd_reg {
	PGD_CFG			= 0x1000,
	PGD_STAT		= 0x1004,
	PGD_INT_EN		= 0x1010,
	PGD_INT_STAT		= 0x1014,
	PGD_INT_CLR		= 0x1018,
	PGD_OWN_EEn		= 0x1020,
	PGD_PORT_INT_EN_EEn	= 0x1030,
	PGD_PORT_INT_ST_EEn	= 0x1034,
	PGD_PORT_INT_CL_EEn	= 0x1038,
	PGD_PORT_CFGn		= 0x1080,
	PGD_PORT_STATn		= 0x1084,
	PGD_PORT_PARAMn		= 0x1088,
	PGD_PORT_BLKn		= 0x108C,
	PGD_PORT_TRANn		= 0x1090,
	PGD_PORT_MCHANn		= 0x1094,
	PGD_PORT_PSHPLLn	= 0x1098,
	PGD_PORT_PC_CFGn	= 0x1600,
	PGD_PORT_PC_VALn	= 0x1604,
	PGD_PORT_PC_VFR_TSn	= 0x1608,
	PGD_PORT_PC_VFR_STn	= 0x160C,
	PGD_PORT_PC_VFR_CLn	= 0x1610,
	PGD_IE_STAT		= 0x1700,
	PGD_VE_STAT		= 0x1710,
};

enum rsc_grp {
	EE_MGR_RSC_GRP	= 1 << 10,
	EE_NGD_2	= 2 << 6,
	EE_NGD_1	= 0,
};

enum mgr_intr {
	MGR_INT_RECFG_DONE	= 1 << 24,
	MGR_INT_TX_NACKED_2	= 1 << 25,
	MGR_INT_MSG_BUF_CONTE	= 1 << 26,
	MGR_INT_RX_MSG_RCVD	= 1 << 30,
	MGR_INT_TX_MSG_SENT	= 1 << 31,
};

enum frm_cfg {
	FRM_ACTIVE	= 1,
	CLK_GEAR	= 7,
	ROOT_FREQ	= 11,
	REF_CLK_GEAR	= 15,
};

enum msm_ctrl_state {
	MSM_CTRL_AWAKE,
	MSM_CTRL_SLEEPING,
	MSM_CTRL_ASLEEP,
};

struct msm_slim_sps_bam {
	u32			hdl;
	void __iomem		*base;
	int			irq;
};
/*
struct msm_slim_endp {
	struct sps_pipe			*sps;
	struct sps_connect		config;
	struct sps_register_event	event;
	struct dma_mem_buffer		buf;
	struct completion		*xcomp;
	bool				connected;
};
*/
struct msm_slim_ctrl {
	struct slim_controller  ctrl;
	struct slim_framer 	framer;
	struct device		*dev;
	void __iomem		*regs;
	struct resource		*ioarea; 
	u32			curr_bw;
	u8			msg_cnt;
	u32			tx_buf[10];
	u8			rx_msgs[MSM_CONCUR_MSG][SLIM_RX_MSGQ_BUF_LEN];
	spinlock_t		rx_lock;
	int			head;
	int			tail;
	int			irq;
	int			err;
	int			ee;
	struct completion	*wr_comp;
	struct msm_slim_sat	*satd;
//	struct msm_slim_endp	pipes[7];
	struct msm_slim_sps_bam	bam;
//	struct msm_slim_endp	rx_msgq;
	struct completion	rx_msgq_notify;
	struct task_struct	*rx_msgq_thread;
	struct clk		*rclk;
	struct mutex		tx_lock;
	u8			pgdla;
	bool			use_rx_msgqs;
	int			pipe_b;
	struct completion	reconf;
	bool			reconf_busy;
	bool			chan_active;
	enum msm_ctrl_state	state;
};

struct msm_slim_sat {
	struct slim_device	satcl;
	struct msm_slim_ctrl	*dev;
	struct workqueue_struct *wq;
	struct work_struct	wd;
	u8			sat_msgs[SAT_CONCUR_MSG][40];
	u16			*satch;
	u8			nsatch;
	bool			sent_capability;
	bool			pending_reconf;
	bool			pending_capability;
	int			shead;
	int			stail;
	spinlock_t lock;
};

static int msm_slim_rx_enqueue(struct msm_slim_ctrl *dev, u32 *buf, u8 len)
{
	spin_lock(&dev->rx_lock);
	if ((dev->tail + 1) % MSM_CONCUR_MSG == dev->head) {
		spin_unlock(&dev->rx_lock);
		dev_err(dev->dev, "RX QUEUE full!");
		return -EXFULL;
	}
	memcpy((u8 *)dev->rx_msgs[dev->tail], (u8 *)buf, len);
	dev->tail = (dev->tail + 1) % MSM_CONCUR_MSG;
	spin_unlock(&dev->rx_lock);
	return 0;
}

static int msm_slim_rx_dequeue(struct msm_slim_ctrl *dev, u8 *buf)
{
	unsigned long flags;
	spin_lock_irqsave(&dev->rx_lock, flags);
	if (dev->tail == dev->head) {
		spin_unlock_irqrestore(&dev->rx_lock, flags);
		return -ENODATA;
	}
	memcpy(buf, (u8 *)dev->rx_msgs[dev->head], 40);
	dev->head = (dev->head + 1) % MSM_CONCUR_MSG;
	spin_unlock_irqrestore(&dev->rx_lock, flags);
	return 0;
}


static void msm_get_eaddr(u8 *e_addr, u32 *buffer)
{
	e_addr[0] = (buffer[1] >> 24) & 0xff;
	e_addr[1] = (buffer[1] >> 16) & 0xff;
	e_addr[2] = (buffer[1] >> 8) & 0xff;
	e_addr[3] = buffer[1] & 0xff;
	e_addr[4] = (buffer[0] >> 24) & 0xff;
	e_addr[5] = (buffer[0] >> 16) & 0xff;
}

static bool msm_is_sat_dev(u8 *e_addr)
{
	if (e_addr[5] == QC_MFGID_LSB && e_addr[4] == QC_MFGID_MSB &&
		e_addr[2] != QC_CHIPID_SL &&
		(e_addr[1] == QC_DEVID_SAT1 || e_addr[1] == QC_DEVID_SAT2))
		return true;
	return false;
}


static irqreturn_t elite_slim_interrupt(int irq, void *d)
{
	struct msm_slim_ctrl *dev = d;
	u32 pstat;
	u32 stat = readl_relaxed(dev->regs + SLIM_CMP_IRQSTATUS);//irq status
	
	if (stat & SMT_INFO ) {//transmit and exception
		u32 smtinfo= readl_relaxed(dev->regs + SLIM_SMT_INFO);//transmittin info
		if (smtinfo & PACK){//pack			
			writel_relaxed(PACK,dev->regs + SLIM_SMT_INFO);//clear smt int
			
		}  if(smtinfo & 0x1D){//nack,mc_tx_col,nore ,udef
			//writel_relaxed(MGR_INT_TX_NACKED_2,	dev->base + MGR_INT_CLR);//exception
			//add exception
			writel_relaxed(0x1D,dev->regs + SLIM_SMT_INFO);//clear smt int
			dev->err = -EIO;
		}
		writel_relaxed(SMT_INFO,dev->regs + SLIM_CMP_IRQSTATUS);//clear smt int
		/*
		 * Guarantee that interrupt clear bit write goes through before
		 * signalling completion/exiting ISR
		 */
		mb();
		if (dev->wr_comp)
			complete(dev->wr_comp);
	}
	
	if (stat & SMR_INFO) {//	
	
		u32 rx_buf[10];
		u32 mc, mt;
		u8 len, i;
		u32 smr_info;
		smr_info = readl_relaxed(dev->regs + SLIM_SMR_INFO);
		writel_relaxed(SMR_INFO,	dev->regs + SLIM_CMP_IRQSTATUS);//clear smr int
		
		if(smr_info & MC_TX_COL){//transmit collision in Message channel
			writel_relaxed(MC_TX_COL,	dev->regs + SLIM_SMR_INFO);//clear MC_TX_COL
			dev->err = -EIO;
			mb();			
			complete(&dev->rx_msgq_notify);
		}else if(smr_info & RECEIVED_RECONFIGURE){
			writel_relaxed(0x4,	dev->regs + SLIM_SMR_INFO);//clear smr int
			mb();
			complete(&dev->reconf);
		}
		else
		{		
			rx_buf[0] = readl_relaxed(dev->regs + SLIM_SMR_MESSAGE);
			len = rx_buf[0] & 0x1F;
			for (i = 1; i < ((len + 3) >> 2); i++) {
				rx_buf[i] = readl_relaxed(dev->regs + SLIM_SMR_MESSAGE);
				dev_dbg(dev->dev, "reading data: %x\n", rx_buf[i]);
			}
			mt = (rx_buf[0] >> 5) & 0x7;
			mc = (rx_buf[0] >> 8) & 0xff;
			dev_dbg(dev->dev, "MC: %x, MT: %x\n", mc, mt);
			if (mt == SLIM_MSG_MT_DEST_REFERRED_USER ||
					mt == SLIM_MSG_MT_SRC_REFERRED_USER) {
				printk("elite sat_enqueue error\n ");	
			
			} else if (mt == SLIM_MSG_MT_CORE &&
				mc == SLIM_MSG_MC_REPORT_PRESENT) {
				u8 e_addr[6];
				msm_get_eaddr(e_addr, rx_buf);
				if (msm_is_sat_dev(e_addr)) {
					printk("elite sat_enqueue error\n ");	
				} else {
					msm_slim_rx_enqueue(dev, rx_buf, len);
					writel_relaxed(RECEIVED_MESSAGE,	dev->regs + SLIM_SMR_INFO);//clear smr int
					/*
					 * Guarantee that CLR bit write goes through
					 * before signalling completion
					 */
					mb();
					complete(&dev->rx_msgq_notify);
				}
			} else if (mc == SLIM_MSG_MC_REPLY_INFORMATION ||
					mc == SLIM_MSG_MC_REPLY_VALUE) {
				msm_slim_rx_enqueue(dev, rx_buf, len);
				writel_relaxed(RECEIVED_MESSAGE,	dev->regs + SLIM_SMR_INFO);//clear smr int
				/*
				 * Guarantee that CLR bit write goes through
				 * before signalling completion
				 */
				mb();
				complete(&dev->rx_msgq_notify);
			} else if (mc == SLIM_MSG_MC_REPORT_INFORMATION) {
				u8 *buf = (u8 *)rx_buf;
				u8 l_addr = buf[2];
				u16 ele = (u16)buf[4] << 4;
				ele |= ((buf[3] & 0xf0) >> 4);
				dev_err(dev->dev, "Slim-dev:%d report inf element:0x%x",
						l_addr, ele);
				for (i = 0; i < len - 5; i++)
					dev_err(dev->dev, "offset:0x%x:bit mask:%x",
							i, buf[i+5]);
				writel_relaxed(RECEIVED_MESSAGE,	dev->regs + SLIM_SMR_INFO);//clear smr int
				/*
				 * Guarantee that CLR bit write goes through
				 * before exiting
				 */
				mb();
			} else {
				dev_err(dev->dev, "Unexpected MC,%x MT:%x, len:%d",
							mc, mt, len);
				for (i = 0; i < ((len + 3) >> 2); i++)
					dev_err(dev->dev, "error msg: %x", rx_buf[i]);
				writel_relaxed(RECEIVED_MESSAGE,	dev->regs + SLIM_SMR_INFO);//clear smr int
				/*
				 * Guarantee that CLR bit write goes through
			 	* before exiting
				 */
				mb();
			}
		}
	}

	return IRQ_HANDLED;
}


static void elite_hw_set_port(struct msm_slim_ctrl *dev, u8 pn)
{
	//u32 set_cfg = DEF_WATERMARK | DEF_ALIGN | DEF_PACK | ENABLE_PORT;
	//u32 int_port = readl_relaxed(dev->regs + PGD_PORT_INT_EN_EEn +
	//				(dev->ee * 16));

	/* Make sure that port registers are updated before returning */
	mb();
}

static u32 *elite_get_msg_buf(struct slim_controller *ctrl, int len)
{
	struct msm_slim_ctrl *dev = slim_get_ctrldata(ctrl);
	/*
	 * Currently we block a transaction until the current one completes.
	 * In case we need multiple transactions, use message Q
	 */
	return dev->tx_buf;
}

static int elite_send_msg_buf(struct slim_controller *ctrl, u32 *buf, u8 len)
{
	int i;
	struct msm_slim_ctrl *dev = slim_get_ctrldata(ctrl);
	for (i = 0; i < (len + 3) >> 2; i++) {
		dev_dbg(dev->dev, "TX data:0x%x\n", buf[i]);
		writel_relaxed(buf[i], dev->regs + SLIM_SMT_MESSAGE);
	}
	/* Guarantee that message is sent before returning */
	mb();
	return 0;
}

static int msm_xfer_msg(struct slim_controller *ctrl, struct slim_msg_txn *txn)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct msm_slim_ctrl *dev = slim_get_ctrldata(ctrl);
	u32 *pbuf;
	u8 *puc;
	int timeout;
	int msgv = -1;
	u8 la = txn->la;
	u8 mc = (u8)(txn->mc & 0xFF);
	/*
	 * Voting for runtime PM: Slimbus has 2 possible use cases:
	 * 1. messaging
	 * 2. Data channels
	 * Messaging case goes through messaging slots and data channels
	 * use their own slots
	 * This "get" votes for messaging bandwidth
	 */
	if (!(txn->mc & SLIM_MSG_CLK_PAUSE_SEQ_FLG)){
		dev_err(dev->dev, "elite not support SLIM_MSG_CLK_PAUSE_SEQ_FLG");
	}
		
	mutex_lock(&dev->tx_lock);
	if (dev->state == MSM_CTRL_ASLEEP ||
		((!(txn->mc & SLIM_MSG_CLK_PAUSE_SEQ_FLG)) &&
		dev->state == MSM_CTRL_SLEEPING)) {
		dev_err(dev->dev, "runtime or system PM suspended state");
		mutex_unlock(&dev->tx_lock);
		return -EBUSY;
	}
	if (txn->mt == SLIM_MSG_MT_CORE &&
		mc == SLIM_MSG_MC_BEGIN_RECONFIGURATION) {
		if (dev->reconf_busy) {
			wait_for_completion(&dev->reconf);
			dev->reconf_busy = false;
		}		
	}
	txn->rl--;
	pbuf = elite_get_msg_buf(ctrl, txn->rl);
	dev->wr_comp = NULL;
	dev->err = 0;

	if (txn->dt == SLIM_MSG_DEST_ENUMADDR) {
		mutex_unlock(&dev->tx_lock);		
		return -EPROTONOSUPPORT;
	}
	if (txn->mt == SLIM_MSG_MT_CORE && txn->la == 0xFF &&
		(mc == SLIM_MSG_MC_CONNECT_SOURCE ||
		 mc == SLIM_MSG_MC_CONNECT_SINK ||
		 mc == SLIM_MSG_MC_DISCONNECT_PORT))
		la = dev->pgdla;
	if (txn->dt == SLIM_MSG_DEST_LOGICALADDR)
		*pbuf = SLIM_MSG_ASM_FIRST_WORD(txn->rl, txn->mt, mc, 0, la);
	else
		*pbuf = SLIM_MSG_ASM_FIRST_WORD(txn->rl, txn->mt, mc, 1, la);
	if (txn->dt == SLIM_MSG_DEST_LOGICALADDR)
		puc = ((u8 *)pbuf) + 3;
	else
		puc = ((u8 *)pbuf) + 2;
	if (txn->rbuf)
		*(puc++) = txn->tid;
	if ((txn->mt == SLIM_MSG_MT_CORE) &&
		((mc >= SLIM_MSG_MC_REQUEST_INFORMATION &&
		mc <= SLIM_MSG_MC_REPORT_INFORMATION) ||
		(mc >= SLIM_MSG_MC_REQUEST_VALUE &&
		 mc <= SLIM_MSG_MC_CHANGE_VALUE))) {
		*(puc++) = (txn->ec & 0xFF);
		*(puc++) = (txn->ec >> 8)&0xFF;
	}
	if (txn->wbuf)
		memcpy(puc, txn->wbuf, txn->len);
	if (txn->mt == SLIM_MSG_MT_CORE && txn->la == 0xFF &&
		(mc == SLIM_MSG_MC_CONNECT_SOURCE ||
		 mc == SLIM_MSG_MC_CONNECT_SINK ||
		 mc == SLIM_MSG_MC_DISCONNECT_PORT)) { //XXXXXX
   #if 0
		if (mc != SLIM_MSG_MC_DISCONNECT_PORT)
			dev->err = msm_slim_connect_pipe_port(dev, *puc);
		else {
			struct msm_slim_endp *endpoint = &dev->pipes[*puc];
			struct sps_register_event sps_event;
			memset(&sps_event, 0, sizeof(sps_event));
			sps_register_event(endpoint->sps, &sps_event);
			sps_disconnect(endpoint->sps);
			/*
			 * Remove channel disconnects master-side ports from
			 * channel. No need to send that again on the bus
			 */
			dev->pipes[*puc].connected = false;
			mutex_unlock(&dev->tx_lock);
			if (msgv >= 0)
				elite_slim_put_ctrl(dev);
			return 0;
		}
		if (dev->err) {
			dev_err(dev->dev, "pipe-port connect err:%d", dev->err);
			mutex_unlock(&dev->tx_lock);
			if (msgv >= 0)
				elite_slim_put_ctrl(dev);
			return dev->err;
		}
	 	*(puc) = *(puc) + dev->pipe_b;
#endif
	}
	if (txn->mt == SLIM_MSG_MT_CORE &&
		mc == SLIM_MSG_MC_BEGIN_RECONFIGURATION)
		dev->reconf_busy = true;
	dev->wr_comp = &done;
	elite_send_msg_buf(ctrl, pbuf, txn->rl);
	timeout = wait_for_completion_timeout(&done, HZ);

	if (mc == SLIM_MSG_MC_RECONFIGURE_NOW) {
		if ((txn->mc == (SLIM_MSG_MC_RECONFIGURE_NOW |
					SLIM_MSG_CLK_PAUSE_SEQ_FLG)) &&
				timeout) {
			timeout = wait_for_completion_timeout(&dev->reconf, HZ);
			dev->reconf_busy = false;
			if (timeout) {
				clk_disable(dev->rclk);
				disable_irq(dev->irq);
			}
		}
		if ((txn->mc == (SLIM_MSG_MC_RECONFIGURE_NOW |
					SLIM_MSG_CLK_PAUSE_SEQ_FLG)) &&
				!timeout) {
			dev->reconf_busy = false;
			dev_err(dev->dev, "clock pause failed");
			mutex_unlock(&dev->tx_lock);
			return -ETIMEDOUT;
		}
		if (txn->mt == SLIM_MSG_MT_CORE &&
			txn->mc == SLIM_MSG_MC_RECONFIGURE_NOW) {
			if (dev->ctrl.sched.usedslots == 0 &&
					dev->chan_active) {
				dev->chan_active = false;
				
			}
		}
	}
	mutex_unlock(&dev->tx_lock);
	
	if (!timeout)
		dev_err(dev->dev, "TX timed out:MC:0x%x,mt:0x%x", txn->mc,
					txn->mt);

	return timeout ? dev->err : -ETIMEDOUT;
}

static int elite_set_laddr(struct slim_controller *ctrl, const u8 *ea,
				u8 elen, u8 laddr)
{
	struct msm_slim_ctrl *dev = slim_get_ctrldata(ctrl);
	DECLARE_COMPLETION_ONSTACK(done);
	int timeout;
	u32 *buf;
	mutex_lock(&dev->tx_lock);
	buf = elite_get_msg_buf(ctrl, 9);
	buf[0] = SLIM_MSG_ASM_FIRST_WORD(9, SLIM_MSG_MT_CORE,
					SLIM_MSG_MC_ASSIGN_LOGICAL_ADDRESS,
					SLIM_MSG_DEST_LOGICALADDR,
					ea[5] | ea[4] << 8);
	buf[1] = ea[3] | (ea[2] << 8) | (ea[1] << 16) | (ea[0] << 24);
	buf[2] = laddr;

	dev->wr_comp = &done;
	elite_send_msg_buf(ctrl, buf, 9);
	timeout = wait_for_completion_timeout(&done, HZ);
	mutex_unlock(&dev->tx_lock);
	return timeout ? dev->err : -ETIMEDOUT;
}

static int msm_clk_pause_wakeup(struct slim_controller *ctrl)
{
#if 0
	struct msm_slim_ctrl *dev = slim_get_ctrldata(ctrl);
	enable_irq(dev->irq);
	clk_enable(dev->rclk);
	writel_relaxed(1, dev->base + FRM_WAKEUP);
	/* Make sure framer wakeup write goes through before exiting function */
	mb();
	/*
	 * Workaround: Currently, slave is reporting lost-sync messages
	 * after slimbus comes out of clock pause.
	 * Transaction with slave fail before slave reports that message
	 * Give some time for that report to come
	 * Slimbus wakes up in clock gear 10 at 24.576MHz. With each superframe
	 * being 250 usecs, we wait for 20 superframes here to ensure
	 * we get the message
	 */
	usleep_range(5000, 5000);	 
#endif 
	return 0;
}

static int msm_config_port(struct slim_controller *ctrl, u8 pn)
{
#if 0
	struct msm_slim_ctrl *dev = slim_get_ctrldata(ctrl);
	struct msm_slim_endp *endpoint;
	int ret = 0;
	if (ctrl->ports[pn].req == SLIM_REQ_HALF_DUP ||
		ctrl->ports[pn].req == SLIM_REQ_MULTI_CH)
		return -EPROTONOSUPPORT;
	if (pn >= (MSM_SLIM_NPORTS - dev->pipe_b))
		return -ENODEV;

	endpoint = &dev->pipes[pn];
	ret = msm_slim_init_endpoint(dev, endpoint);
	dev_dbg(dev->dev, "sps register bam error code:%x\n", ret);
	return ret;	 
#endif 
}

static enum slim_port_err msm_slim_port_xfer_status(struct slim_controller *ctr,
				u8 pn, u8 **done_buf, u32 *done_len)
{
#if 0
	struct msm_slim_ctrl *dev = slim_get_ctrldata(ctr);
	struct sps_iovec sio;
	int ret;
	if (done_len)
		*done_len = 0;
	if (done_buf)
		*done_buf = NULL;
	if (!dev->pipes[pn].connected)
		return SLIM_P_DISCONNECT;
	ret = sps_get_iovec(dev->pipes[pn].sps, &sio);
	if (!ret) {
		if (done_len)
			*done_len = sio.size;
		if (done_buf)
			*done_buf = (u8 *)sio.addr;
	}
	dev_dbg(dev->dev, "get iovec returned %d\n", ret);	 
#endif		 
	return SLIM_P_INPROGRESS;
}

static int msm_slim_port_xfer(struct slim_controller *ctrl, u8 pn, u8 *iobuf,
			u32 len, struct completion *comp)
{
	#if 0
	struct sps_register_event sreg;
	int ret;
	struct msm_slim_ctrl *dev = slim_get_ctrldata(ctrl);
	if (pn >= 7)
		return -ENODEV;


	ctrl->ports[pn].xcomp = comp;
	sreg.options = (SPS_EVENT_DESC_DONE|SPS_EVENT_ERROR);
	sreg.mode = SPS_TRIGGER_WAIT;
	sreg.xfer_done = comp;
	sreg.callback = NULL;
	sreg.user = &ctrl->ports[pn];
	ret = sps_register_event(dev->pipes[pn].sps, &sreg);
	if (ret) {
		dev_dbg(dev->dev, "sps register event error:%x\n", ret);
		return ret;
	}
	ret = sps_transfer_one(dev->pipes[pn].sps, (u32)iobuf, len, NULL,
				SPS_IOVEC_FLAG_INT);
	dev_dbg(dev->dev, "sps submit xfer error code:%x\n", ret);
	#endi
	return ret;	 
	#endif 
	return 0;
}

static void elite_slim_rxwq(struct msm_slim_ctrl *dev)
{
	u8 buf[40];
	u8 mc, mt, len;
	int i, ret;
	if ((msm_slim_rx_dequeue(dev, (u8 *)buf)) != -ENODATA) {
		len = buf[0] & 0x1F;
		mt = (buf[0] >> 5) & 0x7;
		mc = buf[1];
		if (mt == SLIM_MSG_MT_CORE &&
			mc == SLIM_MSG_MC_REPORT_PRESENT) {
			u8 laddr;
			u8 e_addr[6];
			for (i = 0; i < 6; i++)
				e_addr[i] = buf[7-i];

			ret = slim_assign_laddr(&dev->ctrl, e_addr, 6, &laddr);
			/* Is this Qualcomm ported generic device? */
			if (!ret && e_addr[5] == QC_MFGID_LSB &&
				e_addr[4] == QC_MFGID_MSB &&
				e_addr[1] == QC_DEVID_PGD &&
				e_addr[2] != QC_CHIPID_SL)
				dev->pgdla = laddr;
			if (!ret && !pm_runtime_enabled(dev->dev) &&
				laddr == (QC_MSM_DEVS - 1))
				pm_runtime_enable(dev->dev);

		} else if (mc == SLIM_MSG_MC_REPLY_INFORMATION ||
				mc == SLIM_MSG_MC_REPLY_VALUE) {
			u8 tid = buf[3];
			dev_dbg(dev->dev, "tid:%d, len:%d\n", tid, len - 4);
			slim_msg_response(&dev->ctrl, &buf[4], tid,
						len - 4);
			pm_runtime_mark_last_busy(dev->dev);
		} else if (mc == SLIM_MSG_MC_REPORT_INFORMATION) {
			u8 l_addr = buf[2];
			u16 ele = (u16)buf[4] << 4;
			ele |= ((buf[3] & 0xf0) >> 4);
			dev_err(dev->dev, "Slim-dev:%d report inf element:0x%x",
					l_addr, ele);
			for (i = 0; i < len - 5; i++)
				dev_err(dev->dev, "offset:0x%x:bit mask:%x",
						i, buf[i+5]);
		} else {
			dev_err(dev->dev, "unexpected message:mc:%x, mt:%x",
					mc, mt);
			for (i = 0; i < len; i++)
				dev_err(dev->dev, "error msg: %x", buf[i]);

		}
	} else
		dev_err(dev->dev, "rxwq called and no dequeue");
}

static int msm_slim_rx_msgq_thread(void *data)
{
	struct msm_slim_ctrl *dev = (struct msm_slim_ctrl *)data;
	struct completion *notify = &dev->rx_msgq_notify;
	struct msm_slim_sat *sat = NULL;
	u32 mc = 0;
	u32 mt = 0;
	u32 buffer[10];
	int index = 0;
	u8 msg_len = 0;
	int ret;

	dev_dbg(dev->dev, "rx thread started");

	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);
		ret = wait_for_completion_interruptible(notify);

		if (ret)
			dev_err(dev->dev, "rx thread wait error:%d", ret);

		/* 1 irq notification per message */
		if (!dev->use_rx_msgqs) {
			elite_slim_rxwq(dev);
			continue;
		}

		

		pr_debug("message[%d] = 0x%x\n", index, *buffer);

		
	}

	return 0;
}
static int __devinit msm_slim_init_rx_msgq(struct msm_slim_ctrl *dev)
{
	struct completion *notify = &dev->rx_msgq_notify;
#if 0
	/* Setup the transfer */
	for (i = 0; i < (MSM_SLIM_DESC_NUM - 1); i++) {
		ret = msm_slim_post_rx_msgq(dev, i);
		if (ret) {
			dev_err(dev->dev, "post_rx_msgq() failed 0x%x\n", ret);			
		}
	}
#endif 
	init_completion(notify);
	/* Fire up the Rx message queue thread */
	dev->rx_msgq_thread = kthread_run(msm_slim_rx_msgq_thread, dev,
					MSM_SLIM_NAME "_rx_msgq_thread");
	if (!dev->rx_msgq_thread) {
		dev_err(dev->dev, "Failed to start Rx message queue thread\n");
		return -EIO;
	} else{
		return 0;
	}

}

static void elite_hw_init(struct msm_slim_ctrl *dev)
{
	//u32 set_cfg = DEF_WATERMARK | DEF_ALIGN | DEF_PACK | ENABLE_PORT;
	//u32 int_port = readl_relaxed(dev->regs + PGD_PORT_INT_EN_EEn +
	//				(dev->ee * 16));

	/* Make sure that port registers are updated before returning */

	writel_relaxed(0x1D,dev->regs + SLIM_SMT_INFO);//clear smt int
	u32 stat = readl_relaxed(dev->regs + SLIM_CMP_IRQSTATUS);//irq status
	
	writel_relaxed((1<<31),dev->regs + SLIM_DCT_CONFIG1_j+ (1*0x20));//setting DCT1 enable bit		
       if(readl_relaxed(dev->regs +  SLIM_DCT_CONFIG1_j+ (1*0x20))  & (1<<31)) 
	   	printk("Channel 1 Enable !!\n");
	 
	writel_relaxed(RF_24d576M,dev->regs + SLIM_FL_RF);
	writel_relaxed(7,dev->regs + SLIM_FL_CG);
	writel_relaxed(SM_CSW12_SFL24,dev->regs + SLIM_FL_SM);

	 
}

static int __devinit elite_slim_probe(struct platform_device *pdev)
{
	struct msm_slim_ctrl *slim;
	int ret;
	struct resource		 *res;
	
	printk("elite_slim_probe \n");
	dump_stack();
	slim = kzalloc(sizeof(struct msm_slim_ctrl), GFP_KERNEL);
    	if (!slim)
    	{
        	dev_err(&pdev->dev, "no memory for elite slim private data\n");
        	return -ENOMEM;
    	}
    	memset(slim, 0, sizeof(*slim));
	/* slim init*/
	slim->dev = &pdev->dev;
	
	/* get device io memory resource */
   	 res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    	if (res == NULL)
    	{
        	dev_err(&pdev->dev, "cannot find IO resource");
        	ret = -ENOENT;
		goto err_res;
	}

    	slim->ioarea = request_mem_region(res->start, resource_size(res), pdev->name);
    	if (slim->ioarea == NULL)
    	{
        	dev_err(&pdev->dev, "cannot request mem\n");
        	ret = -ENXIO;
        	goto err_res;
    	}

    	slim->regs = ioremap(res->start, resource_size(res));
    	if (slim->regs == NULL)
    	{
        	dev_err(&pdev->dev, "cannot map IO\n");
        	ret = -ENXIO;
        	goto err_ioarea;
    	}
	printk("elite_slim_probe irq \n");	
	/* get device irq resource */ 
	slim->irq = ret = platform_get_irq(pdev, 0);
    	if (ret <= 0)
    	{
        	dev_err(&pdev->dev, "cannot find IRQ");
        	goto err_ioremap;
    	}

    	ret = request_irq(slim->irq, elite_slim_interrupt, IRQF_DISABLED, 
                       dev_name(&pdev->dev), slim);
    	if (ret != 0)
    	{
        	dev_err(&pdev->dev, "cannot claim IRQ %d\n", slim->irq);
        	goto err_ioremap;
    	}	
	printk("elite_slim request_irq \n");
	
	platform_set_drvdata(pdev, slim);
	slim_set_ctrldata(&slim->ctrl, slim);
	printk("elite_slim . \n");
	slim->ctrl.nr = pdev->id;
//	dev->ctrl.nchans = MSM_SLIM_NCHANS;
//	dev->ctrl.nports = MSM_SLIM_NPORTS;
	slim->ctrl.set_laddr = elite_set_laddr;
	slim->ctrl.xfer_msg = msm_xfer_msg;
	slim->ctrl.wakeup =  msm_clk_pause_wakeup;
	slim->ctrl.config_port = msm_config_port;
	slim->ctrl.port_xfer = msm_slim_port_xfer;
	slim->ctrl.port_xfer_status = msm_slim_port_xfer_status;
	/* Reserve some messaging BW for satellite-apps driver communication */
	slim->ctrl.sched.pending_msgsl = 30;
	printk("elite_slim .. \n");
	init_completion(&slim->reconf);
	mutex_init(&slim->tx_lock);
	spin_lock_init(&slim->rx_lock);
	printk("elite_slim ... \n");
	slim->ee = 1;
	slim->use_rx_msgqs = 1;

	//slim->bam.irq = bam_irq->start;


	ret = msm_slim_init_rx_msgq(slim);
	if (ret)
		dev_err(slim->dev, "msm_slim_init_rx_msgq failed 0x%x\n", ret);
	
	printk("elite_slim clk... \n");
	/*get slim bus rclk*/
	//slim->rclk = clk_get(slim->dev, "audio_slimbus_clk");
	slim->rclk=12 ;
	if (!slim->rclk) {
		dev_err(slim->dev, "slimbus clock not found");
		//goto err_clk_get_failed;
	}
	elite_hw_init(slim);
	
	slim->framer.rootfreq = SLIM_ROOT_FREQ >> 3;
	slim->framer.superfreq =
	slim->framer.rootfreq / SLIM_CL_PER_SUPERFRAME_DIV8;
	slim->ctrl.a_framer = &slim->framer;
	slim->ctrl.clkgear = SLIM_MAX_CLK_GEAR;
	slim->ctrl.dev.parent = &pdev->dev;

	slim->satd = kzalloc(sizeof(struct msm_slim_sat), GFP_KERNEL);
	if (!slim->satd) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "request msm_slim_sat failed\n");
		//goto err_sat_failed;
	}
	printk("elite_slim clk_set_rate \n");
	//clk_set_rate(slim->rclk, SLIM_ROOT_FREQ);
	//clk_enable(slim->rclk);
	
	/* Register with framework before enabling frame, clock */
	ret = slim_add_numbered_controller(&slim->ctrl);
	if (ret) {
		printk("error adding controller\n");
		goto err_res;
	}

	printk("elite_slim probe end \n");
#if 0	
	/* Component register initialization */
	writel_relaxed(1, dev->base + COMP_CFG);
	writel_relaxed((EE_MGR_RSC_GRP | EE_NGD_2 | EE_NGD_1),
				dev->base + COMP_TRUST_CFG);

	/*
	 * Manager register initialization
	 * If RX msg Q is used, disable RX_MSG_RCVD interrupt
	 */
	if (dev->use_rx_msgqs)
		writel_relaxed((MGR_INT_RECFG_DONE | MGR_INT_TX_NACKED_2 |
			MGR_INT_MSG_BUF_CONTE | /* MGR_INT_RX_MSG_RCVD | */
			MGR_INT_TX_MSG_SENT), dev->base + MGR_INT_EN);
	else
		writel_relaxed((MGR_INT_RECFG_DONE | MGR_INT_TX_NACKED_2 |
			MGR_INT_MSG_BUF_CONTE | MGR_INT_RX_MSG_RCVD |
			MGR_INT_TX_MSG_SENT), dev->base + MGR_INT_EN);
	writel_relaxed(1, dev->base + MGR_CFG);
	/*
	 * Framer registers are beyond 1K memory region after Manager and/or
	 * component registers. Make sure those writes are ordered
	 * before framer register writes
	 */
	wmb();

	/* Register with framework before enabling frame, clock */
	ret = slim_add_numbered_controller(&dev->ctrl);
	if (ret) {
		dev_err(dev->dev, "error adding controller\n");
		goto err_ctrl_failed;
	}

	/* Framer register initialization */
	writel_relaxed((0xA << REF_CLK_GEAR) | (0xA << CLK_GEAR) |
		(1 << ROOT_FREQ) | (1 << FRM_ACTIVE) | 1,
		dev->base + FRM_CFG);
	/*
	 * Make sure that framer wake-up and enabling writes go through
	 * before any other component is enabled. Framer is responsible for
	 * clocking the bus and enabling framer first will ensure that other
	 * devices can report presence when they are enabled
	 */
	mb();

	/* Enable RX msg Q */
	if (dev->use_rx_msgqs)
		writel_relaxed(MGR_CFG_ENABLE | MGR_CFG_RX_MSGQ_EN,
					dev->base + MGR_CFG);
	else
		writel_relaxed(MGR_CFG_ENABLE, dev->base + MGR_CFG);
	/*
	 * Make sure that manager-enable is written through before interface
	 * device is enabled
	 */
	mb();
	writel_relaxed(1, dev->base + INTF_CFG);
	/*
	 * Make sure that interface-enable is written through before enabling
	 * ported generic device inside MSM manager
	 */
	mb();
	writel_relaxed(1, dev->base + PGD_CFG);
	writel_relaxed(0x3F<<17, dev->base + (PGD_OWN_EEn + (4 * dev->ee)));
	/*
	 * Make sure that ported generic device is enabled and port-EE settings
	 * are written through before finally enabling the component
	 */
	mb();

	writel_relaxed(1, dev->base + COMP_CFG);
	/*
	 * Make sure that all writes have gone through before exiting this
	 * function
	 */
	mb();
#endif 	
	//pm_runtime_use_autosuspend(&pdev->dev);
	//pm_runtime_set_autosuspend_delay(&pdev->dev, MSM_SLIM_AUTOSUSPEND);
	//pm_runtime_set_active(&pdev->dev);

	dev_dbg(slim->dev, "MSM SB controller is up!\n");
	return 0;

err_irq:
    free_irq(slim->irq, slim);

err_ioremap:
    iounmap(slim->regs);

err_ioarea:
    release_resource(slim->ioarea);
    kfree(slim->ioarea);

err_res:
    kfree(slim);
    return ret;
}

static int __devexit elite_slim_remove(struct platform_device *pdev)
{
	struct msm_slim_ctrl *dev = platform_get_drvdata(pdev);
	struct resource *bam_mem;
	struct resource *slim_mem;
	//struct resource *slew_mem = dev->slew_mem;
	
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);

	free_irq(dev->irq, dev);
	slim_del_controller(&dev->ctrl);
	clk_put(dev->rclk);
	
	kthread_stop(dev->rx_msgq_thread);
	iounmap(dev->bam.base);
	iounmap(dev->regs);
	kfree(dev);
	bam_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"slimbus_bam_physical");
//	if (bam_mem)
//		release_mem_region(bam_mem->start, resource_size(bam_mem));
//	if (slew_mem)
//		release_mem_region(slew_mem->start, resource_size(slew_mem));
	slim_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"slimbus_physical");
	if (slim_mem)
		release_mem_region(slim_mem->start, resource_size(slim_mem));
	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int msm_slim_runtime_idle(struct device *device)
{
	dev_dbg(device, "pm_runtime: idle...\n");
	pm_request_autosuspend(device);
	return -EAGAIN;
}
#endif

/*
 * If PM_RUNTIME is not defined, these 2 functions become helper
 * functions to be called from system suspend/resume. So they are not
 * inside ifdef CONFIG_PM_RUNTIME
 */
#ifdef CONFIG_PM_SLEEP
static int msm_slim_runtime_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct msm_slim_ctrl *dev = platform_get_drvdata(pdev);
	int ret;
	dev_dbg(device, "pm_runtime: suspending...\n");
	dev->state = MSM_CTRL_SLEEPING;
	ret = slim_ctrl_clk_pause(&dev->ctrl, false, SLIM_CLK_UNSPECIFIED);
	if (ret) {
		dev_err(device, "clk pause not entered:%d", ret);
		dev->state = MSM_CTRL_AWAKE;
	} else {
		dev->state = MSM_CTRL_ASLEEP;
	}
	return ret;
}

static int msm_slim_runtime_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct msm_slim_ctrl *dev = platform_get_drvdata(pdev);
	int ret = 0;
	dev_dbg(device, "pm_runtime: resuming...\n");
	if (dev->state == MSM_CTRL_ASLEEP)
		ret = slim_ctrl_clk_pause(&dev->ctrl, true, 0);
	if (ret) {
		dev_err(device, "clk pause not exited:%d", ret);
		dev->state = MSM_CTRL_ASLEEP;
	} else {
		dev->state = MSM_CTRL_AWAKE;
	}
	return ret;
}

static int msm_slim_suspend(struct device *dev)
{
	int ret = 0;
	if (!pm_runtime_enabled(dev) || !pm_runtime_suspended(dev)) {
		dev_dbg(dev, "system suspend");
		ret = msm_slim_runtime_suspend(dev);
	}
	if (ret == -EBUSY) {
		/*
		* If the clock pause failed due to active channels, there is
		* a possibility that some audio stream is active during suspend
		* We dont want to return suspend failure in that case so that
		* display and relevant components can still go to suspend.
		* If there is some other error, then it should be passed-on
		* to system level suspend
		*/
		ret = 0;
	}
	return ret;
}

static int msm_slim_resume(struct device *dev)
{
	/* If runtime_pm is enabled, this resume shouldn't do anything */
	if (!pm_runtime_enabled(dev) || !pm_runtime_suspended(dev)) {
		int ret;
		dev_dbg(dev, "system resume");
		ret = msm_slim_runtime_resume(dev);
		if (!ret) {
			pm_runtime_mark_last_busy(dev);
			pm_request_autosuspend(dev);
		}
		return ret;

	}
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops msm_slim_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(
		msm_slim_suspend,
		msm_slim_resume
	)
	SET_RUNTIME_PM_OPS(
		msm_slim_runtime_suspend,
		msm_slim_runtime_resume,
		msm_slim_runtime_idle
	)
};

struct platform_driver elite_slim_driver = {
	.probe = elite_slim_probe,
	.remove = elite_slim_remove,
	.driver	= {
		.name = MSM_SLIM_NAME,
		.owner = THIS_MODULE,
		.pm = NULL,
	},
};

static int __init elite_slim_init(void)
{
	printk("elite_slim  platform_driver_register\n");
	return platform_driver_register(&elite_slim_driver);
}
subsys_initcall(elite_slim_init);

static void __exit elite_slim_exit(void)
{
	printk("elite_slim_exit \n");
	platform_driver_unregister(&elite_slim_driver);
}
module_exit(elite_slim_exit);

MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
MODULE_DESCRIPTION("Elite Slimbus controller");
MODULE_ALIAS("platform:elite-slim");
