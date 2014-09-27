/*
  * Support for ELITE MMC/SD host controller.
  *
  * Copyright (c) 2011 S3 Graphics co., Ltd.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as published
  * by the Free Software Foundation.
  *
  */
  
#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/pm.h>
#include <linux/completion.h>
#include <linux/pagemap.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/irq.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>

#include <asm/dma.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/memory.h>
#include <asm/scatterlist.h>
#include <asm/sizes.h>
#include <mach/pinconf-elite.h>
#include <mach/mmc.h>
#include "elite_mci.h"

#define ELITE_TIMEOUT_TIME	(HZ*2)
#define PINCTRL_DRIVER_NAME	"elite-pinctrl.0"
#define SD0_PIN_GRP_NAME	"mmc0_grp"

static unsigned int fmax = 515633;

/* SD 3.0 drive strength control */
static unsigned long group_drvstrength_type_a_conf = \
        ELITE_PINCONF_PACK(ELITE_PINCONF_PARAM_DRV, ELITE_PINCONF_PAD_DRV_TYPE_A);
static unsigned long group_drvstrength_type_b_conf = \
        ELITE_PINCONF_PACK(ELITE_PINCONF_PARAM_DRV, ELITE_PINCONF_PAD_DRV_TYPE_B);
static unsigned long group_drvstrength_type_c_conf = \
        ELITE_PINCONF_PACK(ELITE_PINCONF_PARAM_DRV, ELITE_PINCONF_PAD_DRV_TYPE_C);
static unsigned long group_drvstrength_type_d_conf = \
        ELITE_PINCONF_PACK(ELITE_PINCONF_PARAM_DRV, ELITE_PINCONF_PAD_DRV_TYPE_D);
/* SD 3.0 signal voltage control */
static unsigned long group_voltage_1_8_conf = \
        ELITE_PINCONF_PACK(ELITE_PINCONF_PARAM_VOLT, ELITE_PINCONF_PAD_1_8);
static unsigned long group_voltage_3_3_conf = \
        ELITE_PINCONF_PACK(ELITE_PINCONF_PARAM_VOLT, ELITE_PINCONF_PAD_3_3);

#ifdef CONFIG_PM_RUNTIME
static int elite_runtime_pm_get(struct elite_host *host);
static int elite_runtime_pm_put(struct elite_host *host);
#else
static inline int elite_runtime_pm_get(struct elite_host *host)
{
    return 0;
}
static inline int elite_runtime_pm_put(struct elite_host *host)
{
    return 0;
}
#endif

#ifdef DEBUG1
void elite_dump_reg(struct elite_host *host)
{
	pr_info(DRIVER_NAME ": =========== elite mmc registers dump (%s) at func(0x%p) ===========\n",
		mmc_hostname(host->mmc),  __builtin_return_address(0));

	pr_info("Command Type Reg: 0x%02x\n",
		elite_readb(host, CTRL));
	pr_info("Command Index Reg: 0x%02x\n",
		elite_readb(host, CMD_IDX));

	pr_info("Response Type Reg: 0x%02x\n",
		elite_readb(host, RSP_TYPE));
	pr_info("Command Argument Reg: 0x%08x\n",
		elite_readl(host, CMD_ARG));

	pr_info("Bus Mode Reg: 0x%02x\n",
		elite_readb(host, BUS_MODE));
	pr_info("Block Length Reg: 0x%04x\n",
		elite_readw(host, BLK_LEN));

	pr_info("Block Count Reg: 0x%04x\n",
		elite_readw(host, BLK_CNT));
	pr_info("Response0 Reg: 0x%02x\n",
		elite_readb(host, RSP_0));

	pr_info("Response1 Reg: 0x%02x\n",
		elite_readb(host, RSP_1));
	pr_info("Response2 Reg: 0x%02x\n",
		elite_readb(host, RSP_2));

	pr_info("Response3 Reg: 0x%02x\n",
		elite_readb(host, RSP_3));
	pr_info("Response4 Reg: 0x%02x\n",
		elite_readb(host, RSP_4));

	pr_info("Response5 Reg: 0x%02x\n",
		elite_readb(host, RSP_5));
	pr_info("Response6 Reg: 0x%02x\n",
		elite_readb(host, RSP_6));

	pr_info("Response7 Reg: 0x%02x\n",
		elite_readb(host, RSP_7));
	pr_info("Response8 Reg: 0x%02x\n",
		elite_readb(host, RSP_8));

	pr_info("Response9 Reg: 0x%02x\n",
		elite_readb(host, RSP_9));
	pr_info("Response10 Reg: 0x%02x\n",
		elite_readb(host, RSP_10));

	pr_info("Response11 Reg: 0x%02x\n",
		elite_readb(host, RSP_11));
	pr_info("Response12 Reg: 0x%02x\n",
		elite_readb(host, RSP_12));

	pr_info("Response13 Reg: 0x%02x\n",
		elite_readb(host, RSP_13));
	pr_info("Response14 Reg: 0x%02x\n",
		elite_readb(host, RSP_14));

	pr_info("Response15 Reg: 0x%02x\n",
		elite_readb(host, RSP_15));

	pr_info("Current Block Count Reg: 0x%04x\n",
		elite_readw(host, CURBLK_CNT));
	pr_info("Interrupt Mask0 Reg: 0x%02x\n",
		elite_readb(host, INT_MASK_0));

	pr_info("Interrupt Mask1 Reg: 0x%02x\n",
		elite_readb(host, INT_MASK_1));

	pr_info("SD Status0 Reg: 0x%02x\n",
		elite_readb(host, SD_STS_0));

	pr_info("SD Status1 Reg: 0x%02x\n",
		elite_readb(host, SD_STS_1));
	pr_info("SD Status2 Reg: 0x%02x\n",
		elite_readb(host, SD_STS_2));
	pr_info("SD Status3 Reg: 0x%02x\n",
		elite_readb(host, SD_STS_3));

	pr_info("Response Timeout Reg: 0x%02x\n",
		elite_readb(host, RSP_TOUT));

	pr_info("Clock Selection Reg: 0x%02x\n",
		elite_readb(host, CLK_SEL));
	pr_info("Extened Control0 Reg: 0x%02x\n",
		elite_readb(host, EXT_CTRL0));

	pr_info("Showdowed Block length Reg: 0x%04x\n",
		elite_readw(host, SHDW_BLKLEN));
	pr_info("Timer Value Reg: 0x%04x\n",
		elite_readw(host, TIMER_VAL));

	pr_info("DMA Global Control Reg: 0x%08x\n",
		elite_readl(host, DMA_GCR));
	pr_info("DMA Interrupt Enable Reg: 0x%08x\n",
		elite_readl(host, DMA_IER));
	pr_info("DMA Interrupt Status Reg: 0x%08x\n",
		elite_readl(host, DMA_ISR));
	pr_info("DMA Memory-Register Pointer Reg: 0x%08x\n",
		elite_readl(host, DMA_DESPR));
	pr_info("DMA Residual Bytes Register for Interface 0: 0x%08x\n",
		elite_readl(host, DMA_RBR));
	pr_info("DMA Data Address Register for Interface 0: 0x%08x\n",
		elite_readl(host, DMA_DAR));
	pr_info("DMA Branch Address Register for Interface 0: 0x%08x\n",
		elite_readl(host, DMA_BAR));
	pr_info("DMA Command Pointer Register for Interface 0: 0x%08x\n",
		elite_readl(host, DMA_CPR));
	pr_info("DMA Context Control Register: 0x%08x\n",
		elite_readl(host, DMA_CxCR));

	pr_debug(DRIVER_NAME ": ===========================================\n");
}
#else
void elite_dump_reg(struct elite_host *host) {}
#endif

static inline int elite_alloc_desc(struct elite_host *host, unsigned int bytes)
{
	void	*desc_pool = NULL;

	desc_pool = dma_alloc_coherent(host->mmc->parent, 
		bytes, &(host->desc_phyaddr), GFP_KERNEL);
	if (!desc_pool)
		return -1;
	host->desc_viraddr = (unsigned long *)desc_pool;
	host->desc_size = bytes;
	return 0;
}

static inline void elite_config_dma(unsigned long config_dir, 
	unsigned long dma_mask,struct elite_host *host)
{
	u32 regval;

	/* Enable DMA */
	elite_writel(host, DMA_GCR_DMA_EN, DMA_GCR);
	elite_writel(host, DMA_GCR_SOFTRESET, DMA_GCR);
	elite_writel(host, DMA_GCR_DMA_EN, DMA_GCR);
	/* Enable DMA interrupt */
	elite_writel(host, DMA_IER_INT_EN, DMA_IER);
	/* Make sure host could co-work with DMA */
	regval = elite_readb(host, SD_STS_2);
	elite_writeb(host, regval | CLK_FREEZ_EN, SD_STS_2);

	if (host->current_clock < 400000)
		elite_writew(host, 0x200, TIMER_VAL); /* 1024*512*(1/390K) seconds */
	else
		elite_writew(host, 0xefff, TIMER_VAL); 

	regval = elite_readl(host, DMA_ISR);
	elite_writel(host, regval | DMA_IER_INT_STS, DMA_ISR);

	/* Write DMA Descriptor Pointer Register */
	elite_writel(host, host->desc_phyaddr, DMA_DESPR);
	if (config_dir == DMA_CFG_WRITE) {
		regval = elite_readl(host, DMA_CxCR);
		elite_writel(host, regval & DMA_CCR_IF_TO_PERIPHERAL, DMA_CxCR);
	} else {
		regval = elite_readl(host, DMA_CxCR);
		elite_writel(host, regval | DMA_CCR_PERIPHERAL_TO_IF, DMA_CxCR);
	}

	host->dms_intmask = dma_mask;

	regval = elite_readl(host, DMA_CxCR);
	elite_writel(host, regval | DMA_CCR_RUN, DMA_CxCR);
}

static void elite_clr_set_irqs(struct elite_host *host, u32 clr, u32 set)
{
	u32 imsk;

	imsk = elite_readl(host, INTR_MASK01);
	imsk &= ~clr;
	imsk |= set;

	elite_writel(host, imsk, INTR_MASK01);
}

static void elite_unmask_irqs(struct elite_host *host, u32 irqs)
{
	elite_clr_set_irqs(host, 0, irqs);
}

static void elite_mask_irqs(struct elite_host *host, u32 irqs)
{
	elite_clr_set_irqs(host, irqs, 0);
}

static void elite_set_card_detection(struct elite_host *host, bool enable)
{
	u32 irqs;

	if(host->mmc->caps & MMC_CAP_NONREMOVABLE)
		return;
	
	irqs = INTR_CARD_INSERT;
	
	if (enable)
		elite_unmask_irqs(host, irqs);
	else
		elite_mask_irqs(host, irqs);
}

static void elite_enable_card_detection(struct elite_host *host)
{
	elite_set_card_detection(host, true);
}

static void elite_disable_card_detection(struct elite_host *host)
{
	elite_set_card_detection(host, false);
}

static void elite_clear_interrupt_status_regs(struct elite_host *host)
{
	/* except CD */
	elite_writeb(host, 0xbf, SD_STS_0);
	elite_writeb(host, 0xff, SD_STS_1);
}

static void elite_host_initialize(struct elite_host *host)
{
	u32 reg_tmp;

	/* reset controller */
	reg_tmp = elite_readb(host, BUS_MODE);
	elite_writeb(host, reg_tmp | SFTRST, BUS_MODE);

	/* reset response FIFO */
	reg_tmp = elite_readb(host, CTRL);
	elite_writeb(host, reg_tmp | FFRST, CTRL);

	/* enable GPI pin to detect card */
	elite_writew(host, HOST_INTR_EN | GPI_DETECT_CARD_EN, BLK_LEN);

	/* setup interrupts */
	reg_tmp = elite_readb(host, INT_MASK_0);
	elite_writeb(host, reg_tmp | DEVICE_INSERT_EN, INT_MASK_0);
}

static void elite_host_deinitialize(struct elite_host *host)
{
	u8 reg_tmp;

	/* reset controller */
	reg_tmp = elite_readb(host, BUS_MODE);
	elite_writeb(host, reg_tmp | SFTRST, BUS_MODE);

	reg_tmp = elite_readw(host, BLK_LEN);
	elite_writew(host, reg_tmp & 0x5FFF, BLK_LEN);

	/* clear interrupt status */
	elite_clear_interrupt_status_regs(host);
}

static void elite_reset(struct elite_host *host)
{
	u32 timeout = 100; //wait max 100ms
	
	elite_writeb(host, elite_readb(host, BUS_MODE) | SFTRST, BUS_MODE);

	/* hw clears the bit when it's done */
	while(elite_readb(host, BUS_MODE) & SFTRST) {
		if (timeout == 0) {
			pr_err("%s: Failed to reset\n", mmc_hostname(host->mmc));
			return;
		}
		timeout--;
		mdelay(1);
	}
}

static inline void elite_prep_cmd(struct elite_host *host, u32 opcode, u32 arg,
	unsigned int flags, u16  blk_len, u16  blk_cnt, unsigned char int_maks_0,
	unsigned char int_mask_1, unsigned char cmd_type, enum opmode op)
{
	u8 rsptype;
	u32 regval;

	/* set cmd operation code and arguments. */
	/* host->opcode is set for further use in ISR.*/
	host->opcode = opcode;
	elite_writeb(host, opcode, CMD_IDX);
	elite_writel(host, arg, CMD_ARG);

	rsptype = mmc_resp_type(host->mrq->cmd);
	/* rsptype=7 only valid for SPI commands - should be =2 for SD */
	if (rsptype == 7)
		rsptype = 2;
	/* rsptype=21 is R1B, convert for controller */
	if (rsptype == 21)
		rsptype = 9;
	if ((opcode == SD_IO_SEND_OP_COND) ||
	    (opcode == SD_IO_RW_DIRECT) ||
	    (opcode == SD_IO_RW_EXTENDED))
		rsptype |= BIT6;
	elite_writeb(host, rsptype, RSP_TYPE);

	/* reset Response FIFO */
	regval = elite_readb(host, CTRL);
	elite_writeb(host, regval | FFRST, CTRL);

	/* SD Host enable Clock */
	regval = elite_readb(host, BUS_MODE);
	elite_writeb(host, regval | HOST_CLK_EN, BUS_MODE);

	/* Set Cmd-Rsp Timeout to be maximum value. */
	elite_writeb(host, 0xfe, RSP_TOUT);

        elite_clear_interrupt_status_regs(host);

	/* set block length and block count for cmd requesting data */
	regval = elite_readw(host, BLK_LEN);
	elite_writew(host, regval & (~0x07ff), BLK_LEN);
	regval = elite_readw(host, BLK_LEN);
	elite_writew(host, regval | blk_len, BLK_LEN);
	elite_writew(host, blk_cnt, BLK_CNT);

	elite_writeb(host, elite_readb(host, INT_MASK_0)|int_maks_0, INT_MASK_0);
	elite_writeb(host, elite_readb(host, INT_MASK_1)|int_mask_1, INT_MASK_1);

	//Set Auto stop for Multi-block access
	if(cmd_type == 3 || cmd_type == 4) {
		//auto stop command set.
		regval = elite_readb(host, EXT_CTRL0);
		elite_writeb(host, regval | EXT_CTRL_GEN_AUTO_STOP_CMD, EXT_CTRL0);
		/*
		  * Enable transaction abort.
		  * When CRC error occurs, CMD 12 would be automatically issued.
		  * That is why we cannot enable R/W CRC error INTs.
		  * If we enable CRC error INT, we would handle this INT in ISR and 
		  * then issue CMD 12 via software.
		  */
		regval = elite_readw(host, BLK_LEN);
		elite_writew(host, regval | TRANSACTION_ABORT, BLK_LEN);
	} else if (cmd_type == 5 || cmd_type == 6) { /* for CMD53 */
		regval = elite_readb(host, EXT_CTRL0);
		elite_writeb(host, regval & (~EXT_CTRL_GEN_AUTO_STOP_CMD), EXT_CTRL0);
		regval = elite_readw(host, BLK_LEN);
		elite_writew(host, regval & (~TRANSACTION_ABORT), BLK_LEN);
	}

	if (op == OP_READ) {
		/* Specify the data transfer of the current command is a read operation */
		regval = elite_readb(host, CTRL);
		elite_writeb(host, regval & (~XFER_RD_WR), CTRL);
	} else if (op == OP_WRITE) {
		/* Specify the data transfer of the current command is a write operation */
		regval = elite_readb(host, CTRL);
		elite_writeb(host, regval | XFER_RD_WR, CTRL);
	}

	/* for Non data access command, command type is 0. */
	elite_writeb(host, elite_readb(host, CTRL) & 0x0f, CTRL);
	regval = elite_readb(host, CTRL);
	elite_writeb(host, regval | (cmd_type << CMD_TYPE_SHIFT), CTRL);
	elite_dump_reg(host);
}


static inline void elite_issue_command(struct elite_host *host)
{
	elite_writeb(host, elite_readb(host, CTRL) | CMD_START, CTRL);
}


static void elite_request_end(struct elite_host *host, struct mmc_request *mrq)
{
	spin_unlock(&host->lock);
	mmc_request_done(host->mmc, mrq);
	spin_lock(&host->lock);
}


void elite_wait_done(void *data)
{
	struct elite_host *host = (struct elite_host *) data;

	BUG_ON(host->done_data == NULL);
	complete(host->done_data);
	host->done_data = NULL;
	host->done = NULL;
}


static void elite_handle_request_data(struct elite_host *host)
{
	/* Mapped sg entries */
	int sg_count;
	unsigned char cmd_type = 0;
	enum opmode op = OP_NONE;	
	unsigned char mask_0 = 0;
	unsigned char mask_1 = 0;
	unsigned long dma_mask = 0;
	unsigned long dma_len;
	dma_addr_t dma_phy;		
	unsigned int sg_num;
	int loop_cnt = 10000;
	struct mmc_data *data = host->data;
	struct mmc_command *cmd = host->cmd;
	struct scatterlist *sg;

	DECLARE_COMPLETION(complete);
	
	data->bytes_xfered = 0;
	cmd->error = 0;
	data->error = 0;

	sg_count = dma_map_sg(&(host->mmc->class_dev), data->sg, data->sg_len, ((data->flags) & MMC_DATA_READ) ? 
			DMA_FROM_DEVICE : DMA_TO_DEVICE);
	if (sg_count == 0) {
		pr_err("%s: Failed to map scatterlist\n", mmc_hostname(host->mmc));
		data->error = -EINVAL;
	}

	do {
		if(cmd->opcode == SD_IO_RW_EXTENDED)
			break;
		loop_cnt--;
		host->done_data = &complete;
		host->done = &elite_wait_done;
		host->soft_timeout = 1;
		elite_prep_cmd(host, MMC_SEND_STATUS, (host->mmc->card->rca)<<16, MMC_RSP_R1 | MMC_CMD_AC, 0,
			0, 0, RSP_DONE_EN | RSP_CRC_ERR_EN | RSP_TIMEOUT_EN, 0, OP_NONE);	
		elite_issue_command(host);

		wait_for_completion_timeout(&complete, ELITE_TIMEOUT_TIME);

		if (host->soft_timeout == 1) {
			pr_err("%s: Check card status timeout\n", mmc_hostname(host->mmc));
			elite_dump_reg(host);
		}
		if (cmd->error != MMC_ERR_NONE) {
			goto end;
		}
	} while ((cmd->resp[0] & 0x1f00) != 0x900 && loop_cnt > 0); /* waiting for trans state. */

	if (!loop_cnt) { 
		pr_err("%s: Run out of loop counter waiting for card entering trans state\n", 
			mmc_hostname(host->mmc));
		data->error = -ENXIO;
		goto end;
	}

	/* SDIO read/write operations */
	if (host->cmd->opcode == SD_IO_RW_EXTENDED) {
		/* Single Block read/write */   
		if (data->blocks == 1 && data->blksz <= 512) { /* byte mode */
			/* read operation */
			if (data->flags & MMC_DATA_READ) {
				host->opcode = SD_IO_RW_EXTENDED;
        			cmd_type = 2;
        			op = OP_READ;
        			mask_0  = 0;	/* BLOCK_XFER_DONE INT skipped, we use DMA TC INT */
        			mask_1 = READ_CRC_ERR_EN |DATA_TIMEOUT_EN|RSP_CRC_ERR_EN |RSP_TIMEOUT_EN;
        			dma_mask = DMA_CCR_EVT_SUCCESS;
			} else {
				/* write operation */
				host->opcode = SD_IO_RW_EXTENDED;
        			cmd_type = 1;
        			op = OP_WRITE;
        			/* DMA TC INT skipped */
        			mask_0  = BLOCK_XFER_DONE_EN;
        			mask_1 = WRITE_CRC_ERR_EN |DATA_TIMEOUT_EN|RSP_CRC_ERR_EN|RSP_TIMEOUT_EN;
        			dma_mask = 0;
			}
		} else {/* block mode */  
			/* Multiple Block read/write operation */
			if (data->flags & MMC_DATA_READ) {
				host->opcode = SD_IO_RW_EXTENDED;
        			cmd_type = 6;
        			op = OP_READ;
        			mask_0  = 0;	/* MULTI_XFER_DONE_EN skipped */
        			mask_1 = READ_CRC_ERR_EN |DATA_TIMEOUT_EN |RSP_CRC_ERR_EN |RSP_TIMEOUT_EN;
        			dma_mask = DMA_CCR_EVT_SUCCESS;
			} else {
				/* write operation */
				host->opcode = SD_IO_RW_EXTENDED;
        			cmd_type = 5;
        			op = OP_WRITE;
        			mask_0  = MULTI_XFER_DONE_EN;
        			mask_1 = WRITE_CRC_ERR_EN|DATA_TIMEOUT_EN|RSP_CRC_ERR_EN|RSP_TIMEOUT_EN;
        			dma_mask = 0;
			}
		}
            
	} else {
		if (data->blocks == 1) {
       			if (data->flags & MMC_DATA_READ) {/* read operation */
       				host->opcode = MMC_READ_SINGLE_BLOCK;
       				cmd_type = 2;
        			op = OP_READ;
        			mask_0  = 0;	/* BLOCK_XFER_DONE INT skipped, we use DMA TC INT */
        			mask_1 = READ_CRC_ERR_EN |DATA_TIMEOUT_EN|RSP_CRC_ERR_EN|RSP_TIMEOUT_EN;
        			dma_mask = DMA_CCR_EVT_SUCCESS;
        		} else {/* write operation */
        			host->opcode = MMC_WRITE_BLOCK;
        			cmd_type = 1;
        			op = OP_WRITE;
        			/* DMA TC INT skipped */
        			mask_0  = BLOCK_XFER_DONE_EN;
        			mask_1 = WRITE_CRC_ERR_EN|DATA_TIMEOUT_EN|RSP_CRC_ERR_EN|RSP_TIMEOUT_EN;
        			dma_mask = 0;
        		}
		} else {  /* multiple blocks mode */
        		if (data->flags & MMC_DATA_READ) {/* read operation */
        			host->opcode = MMC_READ_MULTIPLE_BLOCK;
        			cmd_type = 4;
        			op = OP_READ;
        			mask_0  = 0;	/* MULTI_XFER_DONE_EN skipped */
        			mask_1 = AUTO_STOP_EN|DATA_TIMEOUT_EN|RSP_CRC_ERR_EN|RSP_TIMEOUT_EN;
        			dma_mask = 0;
        		} else {/* write operation */
        			host->opcode = MMC_WRITE_MULTIPLE_BLOCK;
        			cmd_type = 3;
        			op = OP_WRITE;
        			mask_0  = 0;	/* MULTI_XFER_DONE INT skipped */
        			mask_1 = AUTO_STOP_EN|DATA_TIMEOUT_EN|RSP_CRC_ERR_EN|RSP_TIMEOUT_EN;
        			dma_mask = 0;
			}
		}
	}
	host->done_data = &complete;
	host->done = &elite_wait_done;
	host->soft_timeout = 1;	/* If INT comes early than software timer, it would be cleared.*/

	memset(host->desc_viraddr, 0, host->desc_size);
	for_each_sg(data->sg, sg, sg_count, sg_num) {
		dma_phy = sg_dma_address(sg);
		dma_len = sg_dma_len(sg);
		elite_mmc_init_short_desc(host->desc_viraddr + (sg_num * sizeof(struct sd_pdma_desc_s)/4),
		dma_len, (unsigned long *)dma_phy, (sg_num == sg_count - 1) ? 1:0);
	}

	elite_prep_cmd(host, host->opcode, cmd->arg, cmd->flags, data->blksz - 1, \
	   data->blocks, mask_0, mask_1, cmd_type, op);

	elite_config_dma((op == OP_READ) ? DMA_CFG_READ : DMA_CFG_WRITE, dma_mask, host);
	elite_issue_command(host);
	wait_for_completion_timeout(&complete, ELITE_TIMEOUT_TIME*sg_count);

	if (host->soft_timeout == 1) {
		pr_err("%s: Timeout waiting for DMA interrupt\n", mmc_hostname(host->mmc));

		elite_dump_reg(host);
		elite_writel(host, DMA_GCR_SOFTRESET, DMA_GCR);

		/* disable interrupts */
		mask_0 = ~(BLOCK_XFER_DONE_EN | MULTI_XFER_DONE_EN);
		mask_1 = (u8)(~(WRITE_CRC_ERR_EN | READ_CRC_ERR_EN | RSP_CRC_ERR_EN |
			DATA_TIMEOUT_EN |AUTO_STOP_EN |RSP_TIMEOUT_EN | RSP_DONE_EN));
		elite_writeb(host, elite_readb(host, INT_MASK_0) & mask_0, INT_MASK_0);
		elite_writeb(host, elite_readb(host, INT_MASK_1) & mask_1, INT_MASK_1);

		data->error = -ETIMEDOUT;
		goto end;
	}

	if (cmd->error != MMC_ERR_NONE && data->error != MMC_ERR_NONE) {
		goto end;
	}

end:
	host->opcode = 0;
	if (data->error)
		data->bytes_xfered = 0;
	else
		data->bytes_xfered = data->blksz * data->blocks;

	spin_lock(&host->lock);
	//some poorly written reboot program cause the system 
	//reclaim memory before DMA finished data transfer
	if (data && data->sg) 
		dma_unmap_sg(&(host->mmc->class_dev), data->sg, data->sg_len, 
			((data->flags)&MMC_DATA_READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
	if (host->mrq)
		elite_request_end(host, host->mrq);
	spin_unlock(&host->lock);

	/* 
	 * Wait for about 50us
	 * Broadcom WIFI card need to delay a while between two consecutive
	 * data transmission
	 */
	if (host->cmd->opcode == SD_IO_RW_EXTENDED)
		usleep_range(5000, 6000);
}

/**
 * some commands (like CMD6) needs DAM to help reading 
 * block-size register 
 */
static void  elite_handle_read_blocksize_regs(struct elite_host *host)
{
	int sg_count;
	struct scatterlist *sg = host->data->sg;

	sg_count = dma_map_sg(&(host->mmc->class_dev), sg, host->data->sg_len, DMA_FROM_DEVICE);
	if (sg_count == 0) {
		pr_err("%s: Failed to map scatterlist to read block-size card regs\n",
			 mmc_hostname(host->mmc));
		host->data->error = -EINVAL;
		return;
	}

	memset(host->desc_viraddr, 0, host->desc_size);
	/* one scatterlist is sufficient */
	elite_mmc_init_long_desc(host->desc_viraddr, sg_dma_len(sg),
		(unsigned long *)sg_dma_address(sg), 0, 1);

	elite_prep_cmd(host, host->cmd->opcode, host->cmd->arg, 
		host->cmd->flags, sg_dma_len(sg)-1, 0, 0,
		(RSP_CRC_ERR_EN | RSP_TIMEOUT_EN | READ_CRC_ERR_EN | 
		DATA_TIMEOUT_EN), 2, OP_READ);

	elite_config_dma(DMA_CFG_READ, DMA_CCR_EVT_SUCCESS, host);

	elite_issue_command(host);
}


static void elite_start_command(struct elite_host *host)
{
	elite_prep_cmd(host, host->cmd->opcode, host->cmd->arg, host->cmd->flags,
		0, 0, 0, RSP_DONE_EN | RSP_CRC_ERR_EN | RSP_TIMEOUT_EN, 0, OP_NONE);
	elite_issue_command(host);
}

static void elite_fmt_check_rsp(struct elite_host *host)
{
	int idx1, idx2;
	u8 tmp_resp;
	u32 response;

	for (idx1 = 0; idx1 < 4; idx1++) {
		response = 0;
		for (idx2 = 0; idx2 < 4; idx2++) {
			if ((idx1 == 3) && (idx2 == 3))
				tmp_resp = elite_readb(host, RSP_0);
			else
				tmp_resp = elite_readb(host, RSP_0 +
						 (idx1*4) + idx2 + 1);
			response |= (tmp_resp << (idx2 * 8));
		}
		host->cmd->resp[idx1] = cpu_to_be32(response);
	}
}

static int elite_get_slot_status(struct mmc_host *mmc)
{
	struct elite_host *host = mmc_priv(mmc);
	unsigned char status_0 = elite_readb(host, SD_STS_0);

	/* eMMC chip aways in the slot */
	if(!strcmp(mmc_hostname(mmc), "mmc2"))
		return 1;

	return ((status_0 & CARD_NOT_IN_SLOT_GPI) ? 0 : 1);	
}

/*
 * call this when you need sd stack to recognize insertion or removal of card
 * that can't be told by SDHCI regs
 */
void elite_force_presence_change(struct platform_device *pdev, int state)
{
	struct device *dev = &pdev->dev;
	struct mmc_host *mmc_host = (struct mmc_host *)dev_get_drvdata(dev);
	struct elite_host *elite_host = mmc_priv(mmc_host);

        printk(KERN_DEBUG "%s : Enter\n", __func__);
        mmc_detect_change(elite_host->mmc, msecs_to_jiffies(60));
}
EXPORT_SYMBOL_GPL(elite_force_presence_change);


int elite_mmc_init_short_desc(unsigned long *desc_addr, unsigned int req_count, 
	unsigned long *buffer_addr, int end)
{
	struct sd_pdma_desc_s *curdes_s;
	curdes_s = (struct sd_pdma_desc_s *) desc_addr;
	curdes_s->req_count = req_count;
	curdes_s->i = 0;
	curdes_s->format = 0;
	curdes_s->databuffer_addr = buffer_addr;
	if (end) {
		curdes_s->end = 1;
		curdes_s->i = 1;
	}
	return 0;
}


int elite_mmc_init_long_desc(unsigned long *desc_addr, unsigned int req_count,
	unsigned long *buffer_addr, unsigned long *branch_addr, int end)
{
	struct sd_pdma_desc_l *curdes_l;
	curdes_l = (struct sd_pdma_desc_l *) desc_addr;
	curdes_l->req_count = req_count;
	curdes_l->i = 0;
	curdes_l->format = 1;
	curdes_l->databuffer_addr = buffer_addr;
	curdes_l->branch_addr = branch_addr;
	if (end) {
		curdes_l->end = 1;
		curdes_l->i = 1;
	}
	return 0;
}


static irqreturn_t elite_dma_isr(int irq, void *dev_id)
{
	struct elite_host *host = dev_id;
	u8 status_0, status_1, status_2, status_3;
	u32 pdma_event_code = 0;

	disable_irq_nosync(irq);
	spin_lock(&host->lock);

	if (host->runtime_suspended) {
		spin_unlock(&host->lock);
		pr_warning("%s: got irq while runtime suspended\n",
			mmc_hostname(host->mmc));
		return IRQ_HANDLED;
	}
	status_0 = elite_readb(host, SD_STS_0);
	status_1 = elite_readb(host, SD_STS_1);
	status_2 = elite_readb(host, SD_STS_2);
	status_3 = elite_readb(host, SD_STS_3);
	
	pdma_event_code  = elite_readl(host, DMA_CxCR) & DMA_CCR_EVTCODE;
	elite_writel(host, elite_readl(host, DMA_ISR) | DMA_IER_INT_STS, DMA_ISR);

	/* We expect cmd6 or read single block cmd run to here. */
	if (pdma_event_code == DMA_CCR_EVT_SUCCESS) {
		if (host->dms_intmask == DMA_CCR_EVT_SUCCESS) {
			host->data->error = MMC_ERR_NONE;
			host->cmd->error = MMC_ERR_NONE;
		}
	} else {
		host->data->error = -EIO; 
		host->cmd->error = -EIO; 
		dev_err(mmc_dev(host->mmc), \
			"** dma_isr PDMA failed**  event code: 0x%X\n", \
			elite_readl(host, DMA_CxCR));
		elite_dump_reg(host);
	}
	if (host->dms_intmask == DMA_CCR_EVT_SUCCESS)
		elite_fmt_check_rsp(host);
	elite_writel(host, elite_readl(host, DMA_IER) | DMA_IER_INT_EN, DMA_IER);

	if ((pdma_event_code != DMA_CCR_EVT_SUCCESS) || (host->dms_intmask == DMA_CCR_EVT_SUCCESS)) {
		if (host->done_data) {
			host->soft_timeout = 0;
			host->done(host);
		} else {
			/* Finish reading block-size SD/MMC card registers */
			dma_unmap_sg(&(host->mmc->class_dev), host->data->sg, host->data->sg_len, 
				((host->data->flags) & MMC_DATA_READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
			elite_request_end(host, host->mrq);
		}
	}
	spin_unlock(&host->lock);
	enable_irq(irq);
	
	return IRQ_HANDLED;
}

irqreturn_t elite_regular_isr(int irq, void *dev_id)
{
	u32 pdma_sts;
	struct elite_host *host = dev_id;
	u8 status_0, status_1, status_2, status_3, mask_0,mask_1;

	BUG_ON(host == NULL);

	disable_irq_nosync(irq);
	spin_lock(&host->lock);
	if (host->runtime_suspended) {
		spin_unlock(&host->lock);
		enable_irq(irq);
		pr_warning("%s: got irq while runtime suspended\n",
			mmc_hostname(host->mmc));
		return IRQ_HANDLED;
	}

	status_0 = elite_readb(host, SD_STS_0);
	status_1 = elite_readb(host, SD_STS_1);
	status_2 = elite_readb(host, SD_STS_2);
	status_3 = elite_readb(host, SD_STS_3);
	mask_0 = elite_readb(host, INT_MASK_0);
	mask_1 = elite_readb(host, INT_MASK_1);

	if ((status_0 & DEVICE_INSERT) && (mask_0 & 0x80)) {
		dev_dbg(mmc_dev(host->mmc), "Card insertion interrupt\n");
		mmc_detect_change(host->mmc, msecs_to_jiffies(500));
		elite_writeb(host, elite_readb(host, SD_STS_0) | DEVICE_INSERT, SD_STS_0);
		spin_unlock(&host->lock);
		enable_irq(irq);
		return IRQ_HANDLED;
	}

	/* SDIO interrupt */
	if ((status_1 & mask_1) &  SDIO_INT) {
		dev_dbg(mmc_dev(host->mmc), "SDIO interrupt\n");
		spin_unlock(&host->lock);
		mmc_signal_sdio_irq(host->mmc);

		if (((status_1 & mask_1) == SDIO_INT) && ((status_0 & mask_0) == 0)) {
			enable_irq(irq);
			return IRQ_HANDLED;
		}
		spin_lock(&host->lock);
	}

	pdma_sts = elite_readl(host, DMA_CxCR);
	if (((status_0 & mask_0) | (status_1 & mask_1)) == 0) {
		spin_unlock(&host->lock);
		enable_irq(irq);
		return IRQ_HANDLED;
	}

	/*******************************************************
		handling command interrupt.
	*******************************************************/
	/* cmd19 generates _only_ block data transfer done interrupt */
	if (host->opcode == MMC_SEND_TUNING_BLOCK ||
		host->opcode == MMC_SEND_TUNING_BLOCK_HS200) {
		if ((status_0 & (MULTI_XFER_DONE|BLOCK_XFER_DONE))) {
			host->tuning_done = 1;
			wake_up(&host->sblk_xfer_done);
		}
	} else if (host->opcode == MMC_WRITE_BLOCK) { /* for write single block */
		if ((status_0 & BLOCK_XFER_DONE) && (status_1 & RSP_DONE)) {
			host->data->error = MMC_ERR_NONE;
			host->cmd->error = MMC_ERR_NONE;
		} else {
			host->data->error = -EIO;
			host->cmd->error = -EIO;
			dev_err(mmc_dev(host->mmc), \
				"MMC_WRITE/READ_BLOCK error!status_0: 0x%02x |status_1: 0x%02x\n", \
				status_0,status_1);
			elite_dump_reg(host);
		}
	} else if (host->opcode == MMC_WRITE_MULTIPLE_BLOCK || host->opcode == MMC_READ_MULTIPLE_BLOCK) {
		if ((status_1 & (AUTO_STOP|RSP_DONE)) && (status_0 & MULTI_XFER_DONE)) {
			host->data->error = MMC_ERR_NONE;
			host->cmd->error = MMC_ERR_NONE;
		} else {
			host->data->error = -EIO;
			host->cmd->error = -EIO;
			dev_err(mmc_dev(host->mmc), \
				"MMC_WRITE/READ_MULTIPLE_BLOCK error!status_0: 0x%02x | status_1: 0x%02x\n", \
				status_0,status_1);
			elite_dump_reg(host);
		}
	} else if (host->opcode == MMC_READ_SINGLE_BLOCK) {
		/* we want DMA TC. If runs to here, it must be error.*/
		host->data->error = -EIO;
		host->cmd->error = -EIO;
		dev_err(mmc_dev(host->mmc), \
			"MMC_READ_SINGLE_BLOCK error!status_0: 0x%02x | status_1: 0x%02x\n", \
			status_0,status_1);
		elite_dump_reg(host);
	} else if (host->opcode == SD_IO_RW_EXTENDED) {
		/* Write operation */
		if (elite_readb(host, CTRL) & BIT2) {
			if ((elite_readb(host, CTRL) & 0xf0) == 0x10) {   /* single block write */
				if ((status_0 & BLOCK_XFER_DONE)
				  && (status_1 & RSP_DONE)) {
					host->data->error = MMC_ERR_NONE;
					host->cmd->error = MMC_ERR_NONE;
				} else {
					host->data->error = -EIO; 
					host->cmd->error = -EIO; 
					dev_err(mmc_dev(host->mmc), \
						"[%s] err4 status_0: 0x%02x | status_1: 0x%02x\n", \
						__func__,status_0,status_1);
				}
                
			} else if ((elite_readb(host, CTRL) & 0xf0) == 0x50) {
				if ((status_0 & MULTI_XFER_DONE)
			  	  && (status_1 & RSP_DONE)) {
					host->data->error = MMC_ERR_NONE;
        				host->cmd->error = MMC_ERR_NONE;
				} else {
					if ((status_1 & WRITE_CRC_ERR) || \
						(status_1 & READ_CRC_ERR)) {
        					host->data->error = -EILSEQ; 
        					host->cmd->error = -EILSEQ; 
						
					} else {
        					host->data->error = -EIO; 
        					host->cmd->error = -EIO; 
					}
					dev_err(mmc_dev(host->mmc), \
						"[%s] err4-2 status_0: 0x%02x | status_1: 0x%02x\n", \
						__func__,status_0,status_1);
				}
			} else {
				host->data->error = -EIO; 
    				host->cmd->error = -EIO; 
				dev_err(mmc_dev(host->mmc), \
					"[%s] err4-3 status_0: 0x%02x status_1: 0x%02x\n", \
					__func__,status_0,status_1);
			}
		} else {
			/* Read operation */
			host->data->error = -EIO; 
			host->cmd->error = -EIO; 
			dev_err(mmc_dev(host->mmc), "[%s()] err5\n",__func__);
		}

	} else {
		/**
		 * command, not request data
		 * the command which need data sending back,
		 * like switch_function, send_ext_csd, send_scr, send_num_wr_blocks.
		 * NOTICE: we also send status before reading or writing data, so SEND_STATUS should be excluded.
		 */
		if (host->data && host->opcode != MMC_SEND_STATUS) {
			host->data->error = -EIO; 
			host->cmd->error = -EIO; 
			dev_err(mmc_dev(host->mmc), "MMC_SEND_STATUS!\n");
			elite_dump_reg(host);
		} else {			/* Just command, no data sending back.*/
			if (status_1 & RSP_DONE) {
				elite_dump_reg(host);
				/* First, check data-response is busy or not. */
				if (host->cmd->flags == (MMC_RSP_R1B | MMC_CMD_AC)) {
					int i = 10000;
					
					while (status_2 & RSP_BUSY) {
						status_2 = elite_readb(host, SD_STS_2);
						if (--i == 0)
							break;
					}
					if (i == 0)
						dev_err(mmc_dev(host->mmc), "SD data-response always busy!");
				}
				/**
				 * for our host, even no card in slot, for SEND_STATUS also returns no error.
				 * The protocol layer depends on SEND_STATUS to check whether card is in slot or not.
				 * In fact, we can also avoid this situation by checking the response whether they are all zeros.
				 */
				if (!elite_get_slot_status(host->mmc) && host->opcode == MMC_SEND_STATUS) {
					host->cmd->retries = 0; /* No retry.*/
					host->cmd->error = -EINVAL;
					dev_err(mmc_dev(host->mmc), "no card in slot!\n");
				} else
					host->cmd->error = MMC_ERR_NONE;
			} else {
				if (status_1 & RSP_TIMEOUT) {/* RSP_Timeout .*/
					host->cmd->error = -ETIMEDOUT;
					dev_dbg(mmc_dev(host->mmc), "Respone timeout error using opcode %d!\n", host->cmd->opcode);
				} else {/* or RSP CRC error */
					host->cmd->error = -EILSEQ;
					dev_err(mmc_dev(host->mmc), "Respone CRC error!\n");
				}
				elite_dump_reg(host);
			}
		}
	}

	elite_fmt_check_rsp(host);
	
	/* disable INT */
	elite_writeb(host, elite_readb(host, INT_MASK_0) & \
		(~(BLOCK_XFER_DONE_EN | MULTI_XFER_DONE_EN)), INT_MASK_0);
	elite_writeb(host, elite_readb(host, INT_MASK_1) & \
		(~(WRITE_CRC_ERR_EN|READ_CRC_ERR_EN|RSP_CRC_ERR_EN | \
		DATA_TIMEOUT_EN|AUTO_STOP_EN|RSP_TIMEOUT_EN|RSP_DONE_EN)), \
        INT_MASK_1);

	/* clear INT status. In fact, we will clear again before issuing new command. */
	elite_writeb(host, elite_readb(host, SD_STS_0) | status_0, SD_STS_0);
	elite_writeb(host, elite_readb(host, SD_STS_1) | status_1, SD_STS_1);

	if (elite_readl(host, DMA_ISR) & DMA_IER_INT_STS)
		elite_writel(host, elite_readl(host, DMA_ISR) | DMA_IER_INT_STS, DMA_ISR);

	if (host->done_data) { /* We only use done_data when requesting data.*/
		host->soft_timeout = 0;
		host->done(host);
	} else
		elite_request_end(host, host->mrq); /* for cmd trigered interrupt without data transfer .*/

	spin_unlock(&host->lock);
	enable_irq(irq);
	return IRQ_HANDLED;
}

int elite_do_get_ro(struct elite_host *host)
{
	unsigned long flags;
	unsigned long ret = 0;
	
	spin_lock_irqsave(&host->lock, flags);
	ret = ((elite_readb(host, SD_STS_0) & WRITE_PROTECT) ? 0 : 1);
	spin_unlock_irqrestore(&host->lock, flags);
	
	return ret;
}

static int elite_get_ro(struct mmc_host *mmc)
{
	struct elite_host *host = mmc_priv(mmc);
	int ret;

	elite_runtime_pm_get(host);
	ret = elite_do_get_ro(host);
	elite_runtime_pm_put(host);
	return ret;
}

void elite_dump_host_regs(struct mmc_host *mmc)
{
	struct elite_host *host = mmc_priv(mmc);
	elite_dump_reg(host);
}


static void elite_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct elite_host *host = mmc_priv(mmc);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
    
	if (enable)
		elite_writeb(host, elite_readb(host, INT_MASK_1) | SDIO_EN, INT_MASK_1);
	else
		elite_writeb(host, elite_readb(host, INT_MASK_1) & (~SDIO_EN), INT_MASK_1);

	spin_unlock_irqrestore(&host->lock, flags);
    
}


static void elite_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct elite_host *host = mmc_priv(mmc);
	
	elite_runtime_pm_get(host);

	host->mrq = mrq;
	host->data = mrq->data;
	host->cmd = mrq->cmd;
	host->done_data = NULL;
	host->done = NULL;

	if (host->data) {
		switch (host->cmd->opcode) {
		case MMC_WRITE_BLOCK:
		case MMC_WRITE_MULTIPLE_BLOCK:
		case MMC_READ_SINGLE_BLOCK:
		case MMC_READ_MULTIPLE_BLOCK:
		case SD_IO_RW_EXTENDED:
			elite_handle_request_data(host);
			break;
		default:
			elite_handle_read_blocksize_regs(host);
			break;
		}
	} else {
		elite_start_command(host);
	}

	elite_runtime_pm_put(host);
}

static void elite_set_clock(struct elite_host *host, struct mmc_ios *ios)
{
	unsigned int clock;

	/* sdc 1 has a divider=2, so actual clock need to be double */
	if(!strcmp(dev_name(host->dev), "elite-mci.1"))
		clock = ios->clock * 2;
	else
		clock = ios->clock;
	
	if(clock != 0)
		clk_set_rate(host->mclk, clock);

	if(!strcmp(dev_name(host->dev), "elite-mci.1"))
		host->current_clock = clk_get_rate(host->mclk)/2;
	else
		host->current_clock = clk_get_rate(host->mclk);
	    
	dev_dbg(mmc_dev(host->mmc), "expected clock rate: %u, and current clock rate: %u\n", \
		ios->clock, host->current_clock);

}

static void elite_reset_hardware(struct elite_host *host)
{
	u32 reg_tmp;

	/* reset controller */
	reg_tmp = elite_readb(host, BUS_MODE);
	elite_writeb(host, reg_tmp | SFTRST, BUS_MODE);

	/* reset response FIFO */
	reg_tmp = elite_readb(host, CTRL);
	elite_writeb(host, reg_tmp | FFRST, CTRL);

	/** 
	 * enable GPI & DAT3(CD) pin to detect SD/MMC card 
	 * BIT6(CD) of SD status Register 0 (Offset 0x28) is used for
	 * tuning purpose (workaround a tuning bug of controller)
	 */
	elite_writew(host, HOST_INTR_EN | GPI_DETECT_CARD_EN | DAT3_DETECT_CARD_EN, BLK_LEN);

	/* clear interrupt status */
	elite_clear_interrupt_status_regs(host);

	/* setup interrupts (enable SI but not CD interrupt) */
	elite_writeb(host, DEVICE_INSERT_EN, INT_MASK_0);
	elite_writeb(host, DATA_TIMEOUT_EN | RSP_DONE_EN |
	       RSP_TIMEOUT_EN, INT_MASK_1);

	/* set the DMA timeout */
	elite_writew(host, 0xffff, TIMER_VAL);

	/* auto clock freezing enable */
	reg_tmp = elite_readb(host, SD_STS_2);
	elite_writeb(host, reg_tmp | CLK_FREEZ_EN, SD_STS_2);
}

/* 
 * The code below just used to workaroud GPIO & SD/MMC controller bugs 
 */
static void elite_hw_patch(struct elite_host *host)
{
	int retval;
	u8 reg_tmp;

	/* power off SD/MMC card */
	if (gpio_is_valid(host->pwrsw_pin))
		gpio_direction_output(host->pwrsw_pin, 1); 
	reg_tmp = elite_readb(host, BUS_MODE);
	/* Start SDCLK */
	elite_writeb(host, reg_tmp | HOST_CLK_EN, BUS_MODE);
	/* Stop SDCLK */
	elite_writeb(host, reg_tmp & (~HOST_CLK_EN), BUS_MODE);
	/* pull down all pins */
	retval = pinctrl_select_state(host->pinctrl_p,
				host->pinctrl_pulld);
	if (retval)
		pr_err("could not set SD/MMC host pins to pull down state\n");

	/* power on SD/MMC card */
	if (gpio_is_valid(host->pwrsw_pin))
		gpio_direction_output(host->pwrsw_pin, 0);
	/* pull up all pins */
	retval = pinctrl_select_state(host->pinctrl_p,
				host->pinctrl_def);
	if (retval)
		pr_err("could not set SD/MMC host pins to pull up state\n");
	/* Start SDCLK */
	elite_writeb(host, reg_tmp | HOST_CLK_EN, BUS_MODE);
}

/**
 * driver strength selected function (not implemented)
 * int	(*select_drive_strength)(unsigned int max_dtr, int host_drv, int card_drv); 
 */
static void elite_do_set_ios(struct elite_host *host, struct mmc_ios *ios)
{
	int retval;
	u8 ctrl, bus_mode;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	if (ios->power_mode == MMC_POWER_OFF) {
		u8 reg_tmp;

		dev_dbg(mmc_dev(host->mmc), "SD/MMC power off\n");
		reg_tmp = elite_readb(host, BUS_MODE);
		/* Stop SDCLK */
		elite_writeb(host, reg_tmp & (~HOST_CLK_EN), BUS_MODE);
		/* power off SD/MMC card */
		if (gpio_is_valid(host->pwrsw_pin))
			gpio_direction_output(host->pwrsw_pin, 1); 		
	} else if (ios->power_mode == MMC_POWER_UP) {
		dev_dbg(mmc_dev(host->mmc), "SD/MMC power up\n");
		elite_hw_patch(host);
		elite_reset_hardware(host);
	}

	elite_set_clock(host, ios);
	
	ctrl = elite_readb(host, EXT_CTRL0);
	bus_mode = elite_readb(host, BUS_MODE);
	
	if (ios->bus_width == MMC_BUS_WIDTH_8) {
		ctrl |= EXT_CTRL_WIDTH_8;
		dev_dbg(mmc_dev(host->mmc), "Bus Width Selection: 8 bits mode");
	} else if (ios->bus_width == MMC_BUS_WIDTH_4) {
		bus_mode |= BUS_WIDTH_4;
		ctrl &= (~EXT_CTRL_WIDTH_8);
		dev_dbg(mmc_dev(host->mmc), "Bus Width Selection: 4 bits mode");
	} else {
		bus_mode &= (~BUS_WIDTH_4);
		ctrl &= (~EXT_CTRL_WIDTH_8);
		dev_dbg(mmc_dev(host->mmc), "Bus Width Selection: 1 bits mode");
	}

	if (ios->timing == MMC_TIMING_SD_HS ||
		ios->timing == MMC_TIMING_MMC_HS) {
		if(!strcmp(dev_name(host->dev), "elite-mci.1"))
			ctrl |= EXT_CTRL_HI_SPEED;
		dev_dbg(mmc_dev(host->mmc), "Timing set to High-speed mode1\n");
	} else {
		ctrl &= (~EXT_CTRL_HI_SPEED);
		dev_dbg(mmc_dev(host->mmc), "Timing set to Normal-speed mode\n");
	}

	
	if (host->host_ver >= HOST_SPEC_30) {
		u16 ctrl_2;
		u8 xbus_mode;
		
		xbus_mode = elite_readb(host, XBUS_MODE);
		ctrl_2 = elite_readw(host, HOST_CTRL2); 

		/* In case of UHS-I modes, set High Speed Enable */
		if ((ios->timing == MMC_TIMING_MMC_HS200) ||
			(ios->timing == MMC_TIMING_UHS_SDR50) ||
			(ios->timing == MMC_TIMING_UHS_SDR104) ||
			(ios->timing == MMC_TIMING_UHS_DDR50) ||
			(ios->timing == MMC_TIMING_UHS_SDR25)) {
		
			ctrl |= EXT_CTRL_HI_SPEED;
			dev_dbg(mmc_dev(host->mmc), "Timing set to High-speed mode2\n");
		}

		if (ios->timing == MMC_TIMING_UHS_SDR12)
			ctrl_2 |= CTRL2_UHS_SDR12;
		else if (ios->timing == MMC_TIMING_UHS_SDR25)
			ctrl_2 |= CTRL2_UHS_SDR25;
		else if (ios->timing == MMC_TIMING_UHS_SDR50)
			ctrl_2 |= CTRL2_UHS_SDR50;
		else if (ios->timing == MMC_TIMING_UHS_SDR104)
			ctrl_2 |= CTRL2_UHS_SDR104;
		else if (ios->timing == MMC_TIMING_UHS_DDR50) {
			ctrl_2 |= CTRL2_UHS_DDR50;
			xbus_mode |= XBUS_DDR50_MODE_EN;
		}
			
		/* Set driver strength */
		if (ios->drv_type == MMC_SET_DRIVER_TYPE_A) {
			ctrl_2 |= CTRL2_DRV_TYPE_A;
			retval = pin_config_group_set(PINCTRL_DRIVER_NAME, SD0_PIN_GRP_NAME, \
					group_drvstrength_type_a_conf);
			if (retval)
				pr_err("Failed to set driver strength type a\n");
			dev_dbg(mmc_dev(host->mmc), "Driver type A\n");
			
		} else if (ios->drv_type == MMC_SET_DRIVER_TYPE_B) {
			ctrl_2 |= CTRL2_DRV_TYPE_B;
			retval = pin_config_group_set(PINCTRL_DRIVER_NAME, SD0_PIN_GRP_NAME, \
					group_drvstrength_type_b_conf);
			if (retval)
				pr_err("Failed to set driver strength type b\n");
			dev_dbg(mmc_dev(host->mmc), "Driver type B\n");
		} else if (ios->drv_type == MMC_SET_DRIVER_TYPE_C) {
			ctrl_2 |= CTRL2_DRV_TYPE_C;
			retval = pin_config_group_set(PINCTRL_DRIVER_NAME, SD0_PIN_GRP_NAME, \
					group_drvstrength_type_c_conf);
			if (retval)
				pr_err("Failed to set driver strength type c\n");
			dev_dbg(mmc_dev(host->mmc), "Driver type C\n");
		} else if (ios->drv_type == MMC_SET_DRIVER_TYPE_D) {
			ctrl_2 |= CTRL2_DRV_TYPE_D;
			retval = pin_config_group_set(PINCTRL_DRIVER_NAME, SD0_PIN_GRP_NAME, \
					group_drvstrength_type_d_conf);
			if (retval)
				pr_err("Failed to set driver strength type d\n");
			dev_dbg(mmc_dev(host->mmc), "Driver type D\n");
		}

		elite_writeb(host, xbus_mode, XBUS_MODE);
		elite_writew(host, ctrl_2, HOST_CTRL2); 
	}
	/* some cards may need to clear this bit */
	//ctrl &= (~EXT_CTRL_HI_SPEED);
	elite_writeb(host, ctrl, EXT_CTRL0);
	elite_writeb(host, bus_mode, BUS_MODE);
	
	spin_unlock_irqrestore(&host->lock, flags);
}

static void elite_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct elite_host *host = mmc_priv(mmc);

	elite_runtime_pm_get(host);
	elite_do_set_ios(host, ios);
	elite_runtime_pm_put(host);
}

static int elite_do_start_signal_voltage_switch(struct elite_host *host,
						struct mmc_ios *ios)
{
	int retval;
	u16 ctrl;
	u8 clk, pwr, present_state;

	/*
	  * Signal Voltage Switching is only applicable for Host Controllers
	  * v3.00 and above.
	  */
	if (host->host_ver < HOST_SPEC_30)
		return 0;

	ctrl = elite_readw(host, HOST_CTRL2);
	
	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
		pr_info("Swtiching to 3.3v signal voltage\n");
		ctrl |= CTRL2_SIG_VDD_180; //set 1 to disable 1.8v signaling voltage
		elite_writew(host, ctrl, HOST_CTRL2);

		retval = pin_config_group_set(PINCTRL_DRIVER_NAME, SD0_PIN_GRP_NAME, \
					group_voltage_3_3_conf);
		if (retval)
			pr_err("Failed to set 3.3v signal voltage\n");

		if (!IS_ERR(host->sig_reg)) {
			retval = regulator_set_voltage(host->sig_reg, 2700000, 3600000);
			if (retval) {
				pr_warning("%s: Switching to 3.3V signalling voltage "
					   " failed\n", mmc_hostname(host->mmc));
				return -EIO;
			}
		}

		/* Wait for 5ms */
		usleep_range(5000, 5500);
		/* 3.3V regulator output should be stable within 5 ms */
		ctrl = elite_readw(host, HOST_CTRL2);
		if (ctrl & CTRL2_SIG_VDD_180)
			return 0;
		pr_err("%s: Regulator does not output a stable 3.3v signaling voltage\n", 
			mmc_hostname(host->mmc));
		return -EIO;
	} else if ((ctrl & CTRL2_SIG_VDD_180) &&
		(ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180)) {
		/* Stop SDCLK */
		clk = elite_readb(host, BUS_MODE);
		clk &= ~HOST_CLK_EN;
		elite_writeb(host, clk, BUS_MODE);

		/* Check whether DAT[3:0] is 0000 */
		present_state = elite_readb(host, BUS_LEVEL);
		if (!(present_state & DATA_LVL_MASK)) {
			pr_info("DAT[3:0] present state prior to signal " 
					"voltage switching: 0x%X\n", present_state);
			if (!IS_ERR(host->sig_reg))
				retval = regulator_set_voltage(host->sig_reg,
					1700000, 1950000);
			else
				retval = 0;
			if (!retval) {
				/* switch to 1.8v signaling along with GPIO module */
				retval = pin_config_group_set(PINCTRL_DRIVER_NAME, SD0_PIN_GRP_NAME, \
						group_voltage_1_8_conf);
				if (retval)
					pr_err("Failed to set 1.8v signal voltage\n");
				/*
				 * Enable 1.8V Signaling by writing BIT3 to the Host Control2
				 * register
			 	*/
				ctrl &= ~CTRL2_SIG_VDD_180; //clear to enable 1.8v signaling voltage
				elite_writew(host, ctrl, HOST_CTRL2);

				/* Wait 5ms for the regulator to be stable */
				usleep_range(5000, 5500);

				ctrl = elite_readw(host, HOST_CTRL2);
				if (!(ctrl & CTRL2_SIG_VDD_180)) {
					/* Provide SDCLK again and wait for 1ms*/
					clk = elite_readb(host, BUS_MODE);
					clk |= HOST_CLK_EN;
					elite_writeb(host, clk, BUS_MODE);
					usleep_range(1000, 1500);

					/*
					 * If DAT[3:0] level is 1111b, then the card
					 * was successfully switched to 1.8V signaling.
					 */
					present_state = elite_readb(host, BUS_LEVEL);
					pr_info("DAT[3:0] present state after "
						"signal voltage switching: 0x%X\b", present_state);
					/* the card drives DAT[3:0] to high at 1.8V */
					if ((present_state & DATA_LVL_MASK) == DATA_LVL_MASK) {
						pr_info("The card was successfully switched to 1.8v signaling.\n");
						return 0;
					}
				}
			}
		}
		/*
		 * If we are here, that means the switch to 1.8V signaling
		 * failed. We power cycle the card, and retry initialization
		 * sequence by setting S18R to 0.
		 */
		pwr = elite_readb(host, BUS_MODE);
		pwr |= SFTRST;
		elite_writeb(host, pwr, BUS_MODE);
		/* Wait for 1ms as per the spec */
		usleep_range(1000, 1500);

		pr_info(DRIVER_NAME ": Switching to 1.8V signalling "
			"voltage failed, retrying with S18R set to 0\n");
		return -EAGAIN;
	} else
		/* No signal voltage switch required */
		return 0;
}

static int elite_start_signal_voltage_switch(struct mmc_host *mmc,
	struct mmc_ios *ios)
{
	struct elite_host *host = mmc_priv(mmc);
	int err;

	elite_runtime_pm_get(host);
	err = elite_do_start_signal_voltage_switch(host, ios);
	elite_runtime_pm_put(host);
	
	return err;
}

static int elite_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct elite_host *host;
	u16 ctrl, blk_len = 64, blk_cnt = 1;
	u32 ext_ctrl;
	u8 reg_tmp;
	/* 
	  * The card shall complete a sequence of 40 times CMD19 executions 
	  * in no more tham 150ms
	  */
	int tuning_loop_counter = 40; 
	unsigned long timeout = 150;
	int err = 0;
	long ret = 0;

	host = mmc_priv(mmc);

	elite_runtime_pm_get(host);
	/* Is it necessary to disable DMA interrupt here? */
	disable_irq(host->dma_irq);
	spin_lock(&host->lock);

	/** 
	 * BIT6(CD) of SD status Register 0 (Offset 0x28) is used for
	 * tuning purpose (workaround a tuning bug of controller)
	 */
	if (!(elite_readb(host, SD_STS_0) & SD_CD)) {
		pr_err("%s: BIT 6 of status 0 register must be 1\n", \
			mmc_hostname(mmc));
		goto out;
	}
	ctrl = elite_readw(host, HOST_CTRL2);
	/*
	 * The Host Controller needs tuning only in case of SDR104 mode
	 * and for SDR50 mode when Use Tuning for SDR50 is set in the
	 * Host control2 register (elite1k doesn't completely support SDR104
	 * because the its supported maximum clock rate for SD/MMC is only 200MHz).
	 * If the Host Controller supports the HS200 mode then the
	 * tuning function has to be executed (elite1k doesn't support HS200 mode).
	 */
	if (((ctrl & CTRL2_UHS_MODE_MASK) != CTRL2_UHS_SDR50)
	    && ((ctrl & CTRL2_UHS_MODE_MASK) != CTRL2_UHS_SDR104)) {
		goto out;
	} else if ((ctrl & CTRL2_UHS_MODE_MASK) == CTRL2_UHS_SDR50) {/* if SDR50 mode needs tuning */
		ctrl |= CTRL2_SDR50_TUNNING_EN;
	} else if ((ctrl & CTRL2_UHS_MODE_MASK) == CTRL2_UHS_SDR104) {
		u8 tmp;
		
		tmp = elite_readb(host, SD_WR_TUNE);
		/* for output clock */
		elite_writeb(host, tmp | 0x1f | 0x80, SD_WR_TUNE); 
	}

	/* Enable CNT1T_dlycmp  */
	ext_ctrl = elite_readl(host, EXT_CTRL0);
	ext_ctrl |= EXT_CTRL_AUTOADKSTART;
	elite_writel(host, ext_ctrl, EXT_CTRL0);
	/* figuring out the maximum CNT */
	while (!(EXT_CTRL_RAUTOADJDONE & elite_readl(host, EXT_CTRL0)));
	/* LAST_TUNING_TIMES should equals to the counting result of CNT1T_dlycmp block */
	pr_info("LAST_TUNING_TIMES: %u\n", \
		elite_readb(host, EXT_CTRL2) & 0x7f);
	if (EXT_CTRL_RAUTOADJFAIL & elite_readl(host, EXT_CTRL0))	
		pr_info("Figuring out the maximum CNT value procedure failed\n");
	ext_ctrl = elite_readl(host, EXT_CTRL0);
	ext_ctrl &= ~EXT_CTRL_AUTOADKSTART;
	ext_ctrl |= 0xe000; //DLYCMP_FINEB
	elite_writel(host, ext_ctrl, EXT_CTRL0);

	/* auto clock freezing disable before tuning started */
	reg_tmp = elite_readb(host, SD_STS_2);
	elite_writeb(host, reg_tmp & (~CLK_FREEZ_EN), SD_STS_2);

	/* Using automatic tuning */
	ctrl &= ~CTRL2_TUNING_MANUL;
	/* start tuning procedure */
	ctrl |= CTRL2_EXEC_TUNING;
	elite_writew(host, ctrl, HOST_CTRL2);
	/*
	 * Issue CMD19 repeatedly till Execute Tuning is set to 0 or the number
	 * of loops reaches 40 times or a timeout of 150ms occurs.
	 */
	timeout = 150;
	do {
		struct mmc_command cmd = {0};
		struct mmc_request mrq = {NULL};
		if (!tuning_loop_counter && !timeout)
			break;

		cmd.opcode = opcode;
		cmd.arg = 0;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
		cmd.retries = 0;
		cmd.data = NULL;
		cmd.error = 0;

		mrq.cmd = &cmd;
		host->mrq = &mrq;

		/*
		 * In response to CMD19, the card sends 64 bytes of tuning
		 * block to the Host Controller. So we set the block size
		 * to 64 here.
		 */
		if (cmd.opcode == MMC_SEND_TUNING_BLOCK_HS200) {
			if (mmc->ios.bus_width == MMC_BUS_WIDTH_8)
				blk_len = 128;
			else if (mmc->ios.bus_width == MMC_BUS_WIDTH_4)
				blk_len = 64;
		} else { //opcode == MMC_SEND_TUNING_BLOCK
			blk_len = 31; //64;
		}

		/*
		 * The tuning block is sent by the card to the host controller.
		 */
		host->cmd = &cmd;
		/* CT = 2 (single block read), op = 1 read mode */
		elite_prep_cmd(host, host->cmd->opcode, host->cmd->arg, \
				host->cmd->flags, blk_len, blk_cnt, \
				BLOCK_XFER_DONE_EN | MULTI_XFER_DONE_EN, 0, 2, OP_READ);

		//spin_unlock(&host->lock);

		elite_issue_command(host);

		spin_unlock(&host->lock);

		/* Wait for block data transfer done (0x28 bit5) interrupt */
		ret = wait_event_interruptible_timeout(host->sblk_xfer_done,
					(host->tuning_done == 1),
					msecs_to_jiffies(100));
		if (!ret) 
			pr_err("wait for event time out\n");
		else if(ret < 0)  // -ERESTARTSYS
			pr_info("event was interrupted by a signal\n");
		else
			pr_info("event evaluated to true before the timeout elapsed\n");
		
		spin_lock(&host->lock);

		if (!host->tuning_done) {
			pr_err(": Timeout waiting for "
				"block data transfer done interrupt during tuning "
				"procedure\n");
			ctrl = elite_readw(host, HOST_CTRL2);
			ctrl &= ~CTRL2_EXEC_TUNING;
			elite_writew(host, ctrl, HOST_CTRL2);

			err = -EIO;
			goto out;
		}

		host->tuning_done = 0;

		ctrl = elite_readw(host, HOST_CTRL2);
		tuning_loop_counter--;
		timeout--;
		mdelay(1);

	} while ((!(ctrl & CTRL2_SAMPLING_CLK_SEL)) && (tuning_loop_counter > 0));

	if(!tuning_loop_counter || ! timeout) {
		pr_info("The host driver has exhausted the maximum number of loops allowed\n");
		/* can presume tuning is successful if this bit turns zero */
		pr_info("Execute tuning status: 0x%04x\n", elite_readw(host, HOST_CTRL2) & CTRL2_EXEC_TUNING);
	} else {
		if (!(ctrl & CTRL2_SAMPLING_CLK_SEL)) {
			pr_err(DRIVER_NAME ": Tuning procedure"
				" failed\n");
			err = -EIO;
		}
	}
	
out:
	/* auto clock freezing enable after tuning completed */
	reg_tmp = elite_readb(host, SD_STS_2);
	elite_writeb(host, reg_tmp & CLK_FREEZ_EN, SD_STS_2);

	elite_mask_irqs(host, INTR_BLOCK_XFER_DONE);
	spin_unlock(&host->lock);
	enable_irq(host->dma_irq);
	elite_runtime_pm_put(host);
	
	return err;
}

static const struct mmc_host_ops elite_ops = {
	.request = elite_request,
	.set_ios = elite_set_ios,
	.get_ro	= elite_get_ro,
	.get_cd	= elite_get_slot_status,
	.enable_sdio_irq = elite_enable_sdio_irq,
	.start_signal_voltage_switch = elite_start_signal_voltage_switch,
	.execute_tuning	= elite_execute_tuning,
};

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int elite_mmc_regs_show(struct seq_file *s, void *data)
{
	struct mmc_host *mmc = s->private;
	struct elite_host *host = mmc_priv(mmc);
	
	seq_printf(s, "mmc%d:\n"
/*
			" enabled:\t%d\n"
			" dpm_state:\t%d\n"
			" nesting_cnt:\t%d\n"
			" ctx_loss:\t%d:%d\n"
*/
			"\nregs:\n",
			mmc->index 
/*
			mmc->enabled ? 1 : 0,
			host->dpm_state, mmc->nesting_cnt,
			host->context_loss, context_loss*/);
/*
	if (host->suspended) {
		seq_printf(s, "host suspended, can't read registers\n");
		return 0;
	}
*/
	
	elite_runtime_pm_get(host);
	
	seq_printf(s, "Command Type Reg: \t\t0x%02x\n",
		elite_readb(host, CTRL));
	seq_printf(s, "Command Index Reg: \t\t0x%02x\n",
		elite_readb(host, CMD_IDX));

	seq_printf(s, "Response Type Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_TYPE));
	seq_printf(s, "Command Argument Reg: \t\t0x%08x\n",
		elite_readl(host, CMD_ARG));

	seq_printf(s, "Bus Mode Reg: \t\t0x%02x\n",
		elite_readb(host, BUS_MODE));
	seq_printf(s, "Block Length Reg: \t\t0x%04x\n",
		elite_readw(host, BLK_LEN));

	seq_printf(s, "Block Count Reg: \t\t0x%04x\n",
		elite_readw(host, BLK_CNT));
	seq_printf(s, "Response0 Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_0));

	seq_printf(s, "Response1 Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_1));
	seq_printf(s, "Response2 Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_2));

	seq_printf(s, "Response3 Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_3));
	seq_printf(s, "Response4 Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_4));

	seq_printf(s, "Response5 Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_5));
	seq_printf(s, "Response6 Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_6));

	seq_printf(s, "Response7 Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_7));
	seq_printf(s, "Response8 Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_8));

	seq_printf(s, "Response9 Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_9));
	seq_printf(s, "Response10 Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_10));

	seq_printf(s, "Response11 Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_11));
	seq_printf(s, "Response12 Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_12));

	seq_printf(s, "Response13 Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_13));
	seq_printf(s, "Response14 Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_14));

	seq_printf(s, "Response15 Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_15));

	seq_printf(s, "Current Block Count Reg: \t\t0x%04x\n",
		elite_readw(host, CURBLK_CNT));
	seq_printf(s, "Interrupt Mask0 Reg: \t\t0x%02x\n",
		elite_readb(host, INT_MASK_0));

	seq_printf(s, "Interrupt Mask1 Reg: \t\t0x%02x\n",
		elite_readb(host, INT_MASK_1));

	seq_printf(s, "SD Status0 Reg: \t\t0x%02x\n",
		elite_readb(host, SD_STS_0));

	seq_printf(s, "SD Status1 Reg: \t\t0x%02x\n",
		elite_readb(host, SD_STS_1));
	seq_printf(s, "SD Status2 Reg: \t\t0x%02x\n",
		elite_readb(host, SD_STS_2));
	seq_printf(s, "SD Status3 Reg: \t\t0x%02x\n",
		elite_readb(host, SD_STS_3));

	seq_printf(s, "Response Timeout Reg: \t\t0x%02x\n",
		elite_readb(host, RSP_TOUT));

	seq_printf(s, "Clock Selection Reg: \t\t0x%02x\n",
		elite_readb(host, CLK_SEL));
	seq_printf(s, "Extened Control0 Reg: \t\t0x%02x\n",
		elite_readb(host, EXT_CTRL0));

	seq_printf(s, "Showdowed Block length Reg: \t\t0x%04x\n",
		elite_readw(host, SHDW_BLKLEN));
	seq_printf(s, "Timer Value Reg: \t\t0x%04x\n",
		elite_readw(host, TIMER_VAL));

	seq_printf(s, "DMA Global Control Reg: \t\t0x%08x\n",
		elite_readl(host, DMA_GCR));
	seq_printf(s, "DMA Interrupt Enable Reg: \t\t0x%08x\n",
		elite_readl(host, DMA_IER));
	seq_printf(s, "DMA Interrupt Status Reg: \t\t0x%08x\n",
		elite_readl(host, DMA_ISR));
	seq_printf(s, "DMA Memory-Register Pointer Reg: \t\t0x%08x\n",
		elite_readl(host, DMA_DESPR));
	seq_printf(s, "DMA Residual Bytes Register for Interface 0: \t\t0x%08x\n",
		elite_readl(host, DMA_RBR));
	seq_printf(s, "DMA Data Address Register for Interface 0: \t\t0x%08x\n",
		elite_readl(host, DMA_DAR));
	seq_printf(s, "DMA Branch Address Register for Interface 0: \t\t0x%08x\n",
		elite_readl(host, DMA_BAR));
	seq_printf(s, "DMA Command Pointer Register for Interface 0: \t\t0x%08x\n",
		elite_readl(host, DMA_CPR));
	seq_printf(s, "DMA Context Control Register: \t\t0x%08x\n",
		elite_readl(host, DMA_CxCR));

	elite_runtime_pm_put(host);
	return 0;
}

static int elite_mmc_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, elite_mmc_regs_show, inode->i_private);
}

static const struct file_operations elite_mmc_regs_fops = {
	.open           = elite_mmc_regs_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static void elite_mmc_debugfs(struct mmc_host *mmc)
{
	if (mmc->debugfs_root)
		debugfs_create_file("regs", S_IRUSR, mmc->debugfs_root,
			mmc, &elite_mmc_regs_fops);
}

#else

static void elite_mmc_debugfs(struct mmc_host *mmc)
{
	return;
}

#endif //CONFIG_DEBUG_FS

#ifdef CONFIG_OF

static const struct of_device_id elite_mmc_of_match[] = {
	{
		.compatible = "s3graphics,elite1000-sdmmc",
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, elite_mmc_of_match);
#endif //CONFIG_OF

static struct elite_mmc_platform_data *of_get_elite_mmc_pdata(struct device *dev)
{
	struct elite_mmc_platform_data *pdata;
	struct device_node *np = dev->of_node;
	u32 bus_width;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	if (of_find_property(np, "s3g,non-removable", NULL)) {
		pdata->nonremovable = true;
	}
	of_property_read_u32(np, "s3g,bus-width", &bus_width);
	if (bus_width == 4)
		pdata->caps |= MMC_CAP_4_BIT_DATA;
	else if (bus_width == 8)
		pdata->caps |= MMC_CAP_8_BIT_DATA;

	pdata->pwrsw_pin = of_get_named_gpio(np, "power-gpios", 0);

	return pdata;
}

static int elite_mmc_probe(struct platform_device *pdev)
{
	int ret = 0,index;
	int  irq[2] = {0};
	char con_name[64];
	struct device *dev = &pdev->dev;
	struct mmc_host *mmc_host  = NULL;
	struct elite_host *elite_host = NULL;
	struct resource *resource = NULL;
	const struct of_device_id *match;
	struct elite_mmc_platform_data *pdata = pdev->dev.platform_data;

	match = of_match_device(of_match_ptr(elite_mmc_of_match), &pdev->dev);
	if (match) {
		pdata = of_get_elite_mmc_pdata(&pdev->dev);
        if (match->data) {
		}
	}
	ret = of_property_read_u32(pdev->dev.of_node,"linux,sdmmc-index", &index);
	if(!ret) {
		pdev->id = index;
	}
	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource) {
		ret = -ENODEV;	/* No such device or address */
		pr_err("Getting platform resources failed!\n");
		goto the_end;
	}

	if (!devm_request_mem_region(dev, resource->start, SZ_1K, dev_name(dev))) {
		ret = -ENOMEM;
		pr_err("Request memory region failed!\n");
		goto the_end ;
	}

	irq[0] = platform_get_irq(pdev, 0);	/* IRQ for device */;
	irq[1] = platform_get_irq(pdev, 1);	/* IRQ for dma */;

	if (irq[0] == NO_IRQ || irq[1] == NO_IRQ) {
		ret = -ENXIO;/* No such device or address */
		pr_err("Get platform IRQ failed!\n");
		goto the_end;
	}

	/* allocate a standard msp_host structure attached with a ELITE structure */
	mmc_host = mmc_alloc_host(sizeof(struct elite_host), dev);
	if (!mmc_host) {
		ret = -ENOMEM;
		pr_err("Allocating driver's data failed!\n");
		goto the_end;
	}

	dev_set_drvdata(dev, (void *)mmc_host); /* mmc_host is driver data for the ELITE dev.*/
	elite_host = mmc_priv(mmc_host);
	elite_host->dev = dev;
#if 0
	if(!strcmp(dev_name(dev), "elite-mci.0"))
		elite_host->host_ver = HOST_SPEC_30;
	else
#endif
	elite_host->host_ver = HOST_SPEC_20;

	sprintf(con_name, "pwrsw_pin%d", pdev->id);
	if (gpio_is_valid(pdata->pwrsw_pin)) {
		if (gpio_request(pdata->pwrsw_pin, con_name)) {
			dev_err(&pdev->dev, "pwrsw pin not available\n");
			elite_host->pwrsw_pin = -ENODEV;
		} else {
			gpio_direction_output(pdata->pwrsw_pin, 0); //power on
			elite_host->pwrsw_pin = pdata->pwrsw_pin;
		}
	}

	//sprintf(con_name, "sdmmc%d", pdev->id);
	elite_host->mclk = clk_get(&pdev->dev, "sdmmc");
	if (IS_ERR(elite_host->mclk)) {
		ret = PTR_ERR(elite_host->mclk);
		elite_host->mclk = NULL;
		dev_err(&pdev->dev, "failed to get sd/mmc clock\n");
		goto fr_host;
	}	
	clk_prepare_enable(elite_host->mclk);		/* Enable the peripheral clock */
	clk_set_rate(elite_host->mclk, 400000);
	pr_debug("%s current clock rate: %lu", dev_name(dev), clk_get_rate(elite_host->mclk));

#define MMC_AUTOSUSPEND_DELAY   100	
	pm_runtime_enable(elite_host->dev);
	pm_runtime_get_sync(elite_host->dev);
	pm_runtime_set_autosuspend_delay(elite_host->dev, MMC_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(elite_host->dev);

	mmc_host->ops = &elite_ops;

	mmc_host->ocr_avail = MMC_VDD_32_33|MMC_VDD_33_34;

	mmc_host->f_min = 400000;
	if (elite_host->host_ver >= HOST_SPEC_30)
		mmc_host->f_max = 200000000; //10000000;//208000000; /* Lower frequency to workaround HW issue */
	else
		mmc_host->f_max = 50000000; //10000000;//50000000; /* Lower frequency to workaround HW issue */

	//if (mmc_host->cap2 & MMC_CAP2_POWEROFF_NOTIFY)
	//	mmc_host->power_notify_type = MMC_HOST_PW_NOTIFY_SHORT;
	//else
		mmc_host->power_notify_type = MMC_HOST_PW_NOTIFY_NONE;
	
	mmc_host->caps = MMC_CAP_4_BIT_DATA;
	mmc_host->caps |= MMC_CAP_SD_HIGHSPEED;
	mmc_host->caps |= MMC_CAP_MMC_HIGHSPEED;
	mmc_host->caps |= MMC_CAP_SDIO_IRQ;
	//mmc_host->caps |= MMC_CAP_CMD23;
	//mmc_host->caps |= MMC_CAP_NEEDS_POLL;
	
	if (elite_host->host_ver >= HOST_SPEC_30) {
		mmc_host->caps |= MMC_CAP_UHS_SDR12;
		mmc_host->caps |= MMC_CAP_UHS_SDR25;
		//mmc_host->caps |= MMC_CAP_UHS_SDR104; 
		mmc_host->caps |= MMC_CAP_UHS_SDR50;
		//mmc_host->caps |= MMC_CAP_UHS_DDR50;
		/*
		 * According to SD Host Controller spec v3.00, if the Host System
		 * can afford more than 150mA, Host Driver should set XPC to 1. Also
		 * the value is meaningful only if Voltage Support in the Capabilities
		 * register is set. The actual current value is 4 times the register
		 * value.
		 */
		//mmc_host->caps |= MMC_CAP_SET_XPC_180;
		mmc_host->caps |= MMC_CAP_MAX_CURRENT_800;
		//mmc_host->caps |= MMC_CAP_MAX_CURRENT_400; 
		
		mmc_host->caps |= MMC_CAP_DRIVER_TYPE_A;
		mmc_host->caps |= MMC_CAP_DRIVER_TYPE_C;
		mmc_host->caps |= MMC_CAP_DRIVER_TYPE_D;
		//mmc_host->caps2 |= MMC_CAP2_HS200_1_8V_SDR;
	}

#ifdef CONFIG_ANDROID
        //mmc_host->pm_flags = MMC_PM_IGNORE_PM_NOTIFY;
#endif	
	/* 
	  * Maximum number of segments. Depends on if the hardware
	  * can do scatter/gather or not.
	  * we use software sg. so we could manage even larger number.
	  */
	mmc_host->max_segs = 128;	
	/*
	  * Maximum number of sectors in one transfer. Limited by PDMA
	  */
	mmc_host->max_req_size = 16*512*(mmc_host->max_segs);
	/*
	  * Maximum segment size. Could be one segment with the maximum number
	  * of bytes. When doing hardware scatter/gather, each entry cannot be larger
	  * than 64KiB though.
	  * 0x7F*512 PDMA one descriptor can transfer 64K-1 byte
	  */
	mmc_host->max_seg_size = 65024; 
	/* 
	  * our block length register is 11 bits.
	  */
	mmc_host->max_blk_size = 2048;	
	mmc_host->max_blk_count = (mmc_host->max_req_size)/512;

	elite_host->base = devm_ioremap(dev, resource->start, SZ_1K);
	if (!elite_host->base) {
		pr_alert("IO remap failed!\n");
		ret = -ENOMEM;
		goto fr_host;
	}

	elite_host->mmc = mmc_host;
	spin_lock_init(&elite_host->lock);
	elite_host->res = resource;/* for elite_mmc_remove */

	elite_host->regular_irq = irq[0];
	elite_host->dma_irq = irq[1];

	ret = devm_request_irq(dev, elite_host->regular_irq, elite_regular_isr,
				IRQF_SHARED, DRIVER_NAME, (void *)elite_host);
	if (ret) {
		ret = -EBUSY;
		pr_err("Failed to register regular ISR!\n");
		goto pm_runtime;
	}

	ret = devm_request_irq(dev, elite_host->dma_irq, elite_dma_isr,
				IRQF_DISABLED, DRIVER_NAME, (void *)elite_host);
	if (ret) {
		ret = -EBUSY;
		pr_err("Failed to register DMA ISR!\n");
		goto pm_runtime;
	}

	ret = elite_alloc_desc(elite_host, sizeof(struct sd_pdma_desc_s) * MAX_DESC_NUM);
	if (ret == -1) {
		ret = -ENOMEM;
		pr_err("Failed to allocate DMA descriptor!\n");
		goto pm_runtime;
	}

	/**
	 * pinctrl configuration stuff
	 */
	elite_host->pinctrl_p = pinctrl_get(elite_host->dev);
	if (IS_ERR(elite_host->pinctrl_p))
		dev_err(&pdev->dev, "could not get mmc_host pinctrl\n");
	else {
		elite_host->pinctrl_def = pinctrl_lookup_state(elite_host->pinctrl_p,
							PINCTRL_STATE_DEFAULT);
		if (IS_ERR(elite_host->pinctrl_def)) {
			dev_err(&pdev->dev,
				"could not get mmc_host defstate (%li)\n",
				PTR_ERR(elite_host->pinctrl_def));
		}
		elite_host->pinctrl_pulld = pinctrl_lookup_state(elite_host->pinctrl_p,
							PINCTRL_STATE_SD_PULLDOWN);
		if (IS_ERR(elite_host->pinctrl_pulld)) {
			dev_err(&pdev->dev,
				"could not get mmc_host pull down state (%li)\n",
				PTR_ERR(elite_host->pinctrl_pulld));
		}
	}

	/* Get signaling regulator */
	elite_host->sig_reg = regulator_get(&pdev->dev, "vmmc");
	if (IS_ERR_OR_NULL(elite_host->sig_reg)) {
		if (!elite_host->sig_reg) {
			pr_info("%s: no regulator supported on this device\n", dev_name(dev));
		} else if (PTR_ERR(elite_host->sig_reg) < 0) {
			pr_info("%s: no signaling regulator found\n", dev_name(dev));
			elite_host->sig_reg = NULL;
		}

		if(elite_host->host_ver >= HOST_SPEC_30) {
			mmc_host->caps &= ~MMC_CAP_UHS_SDR12;
			mmc_host->caps &= ~MMC_CAP_UHS_SDR25;
			mmc_host->caps &= ~MMC_CAP_UHS_SDR104; 
			mmc_host->caps &= ~MMC_CAP_UHS_SDR50;
			mmc_host->caps &= ~MMC_CAP_UHS_DDR50;
			mmc_host->caps &= ~MMC_CAP_SET_XPC_180;
		} 
	} else {
		u32 curr;
 
		regulator_enable(elite_host->sig_reg);
		/* check if this regulator support 1.8v signaling voltage */
		if (!regulator_is_supported_voltage(elite_host->sig_reg, 1700000,
			1950000)) {
			mmc_host->caps &= ~MMC_CAP_UHS_SDR12;
			mmc_host->caps &= ~MMC_CAP_UHS_SDR25;
			mmc_host->caps &= ~MMC_CAP_UHS_SDR104; 
			mmc_host->caps &= ~MMC_CAP_UHS_SDR50;
			mmc_host->caps &= ~MMC_CAP_UHS_DDR50; 
			mmc_host->caps &= ~MMC_CAP_SET_XPC_180;
			pr_info("This regulator doesn't support 1.8v signaling voltage\n");
		}
		curr = regulator_get_current_limit(elite_host->sig_reg);

		curr = curr/1000;  /* convert to mA */
		pr_debug("Maximum current support: %umA", curr);
	}
	mmc_add_host(mmc_host);
	elite_mmc_debugfs(mmc_host);
	pm_runtime_mark_last_busy(elite_host->dev);
	pm_runtime_put_autosuspend(elite_host->dev);

	if (elite_host->host_ver >= HOST_SPEC_30)
		init_waitqueue_head(&elite_host->sblk_xfer_done);

	elite_reset_hardware(elite_host);

	if (pdev->id == 0)
		device_init_wakeup(&pdev->dev, 1);
	
	dev_info(mmc_dev(elite_host->mmc), "%s: Using DMA, %d-bit mode\n", mmc_hostname(mmc_host),
		(elite_host->mmc->caps & MMC_CAP_4_BIT_DATA) ? 4 : 1);

	return 0;
pm_runtime:
	pm_runtime_put_sync(elite_host->dev);
	pm_runtime_disable(elite_host->dev);
	clk_put(elite_host->mclk);

fr_host:
	dev_set_drvdata(dev, NULL);
	mmc_free_host(mmc_host);
the_end:
	pr_err("elite SD/MMc probed Failed!\n");
	return ret;
}


static int elite_mmc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mmc_host *mmc_host = (struct mmc_host *)dev_get_drvdata(dev);
	struct elite_host *elite_host;

	elite_host = mmc_priv(mmc_host);
	if (!mmc_host || !elite_host) {
		pr_alert( "remove method failed!\n");
		return -ENXIO;
	}
	mmc_remove_host(mmc_host);

	/* disable interrupt by resetting controller */
	elite_reset(elite_host);
	pm_runtime_put_sync(elite_host->dev);
	pm_runtime_disable(elite_host->dev);
	clk_disable(elite_host->mclk);
	clk_put(elite_host->mclk);
		
	dev_set_drvdata(dev, NULL);

	if (gpio_is_valid(elite_host->pwrsw_pin))
		gpio_free(elite_host->pwrsw_pin);

	if (!IS_ERR(elite_host->pinctrl_p))
		pinctrl_put(elite_host->pinctrl_p);

	if (!IS_ERR(elite_host->sig_reg)) {
		regulator_disable(elite_host->sig_reg);
		regulator_put(elite_host->sig_reg);
	}

	dma_free_coherent(elite_host->mmc->parent, elite_host->desc_size, \
	   elite_host->desc_viraddr, elite_host->desc_phyaddr);

	(void)mmc_free_host(mmc_host);
	
	return 0;
}


static void elite_mmc_shutdown(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mmc_host *mmc_host = (struct mmc_host *)dev_get_drvdata(dev);
	struct elite_host *host = mmc_priv(mmc_host);

	elite_disable_card_detection(host);
	mmc_remove_host(mmc_host);
}

#ifdef CONFIG_PM_SLEEP
static int elite_mmc_suspend(struct device *dev)
{
	struct mmc_host *mmc = (struct mmc_host *)dev_get_drvdata(dev);
	struct elite_host *host = mmc_priv(mmc);
	int ret = 0;

	if (mmc) {
		printk(KERN_INFO "Elite MMC enter suspend!\n");
		host->reg_blk_len = elite_readw(host,BLK_LEN);
		host->reg_int_mask_0 = elite_readb(host,INT_MASK_0);
		ret = mmc_suspend_host(mmc);
		
		if (device_may_wakeup(dev))
			enable_irq_wake(host->regular_irq);

		gpio_free(host->pwrsw_pin);			
		printk(KERN_INFO "Elite MMC exit suspend!\n");
	}
	
	return ret;
}


static int elite_mmc_resume(struct device *dev)
{
	struct mmc_host *mmc = (struct mmc_host *)dev_get_drvdata(dev);
	struct elite_host *host = mmc_priv(mmc);
	int ret = 0;

	if (mmc) {
		printk(KERN_INFO "Elite MMC enter resume!\n");
		gpio_request(host->pwrsw_pin,"pwrsw_pin");

		elite_writew(host, host->reg_blk_len, BLK_LEN);
		elite_writeb(host, host->reg_int_mask_0, INT_MASK_0);
		ret = mmc_resume_host(mmc);		

		if (device_may_wakeup(dev))
			disable_irq_wake(host->regular_irq);

		printk(KERN_INFO "Elite MMC exit resume!\n");
	}

	return ret;
}
#else
#define elite_mmc_suspend	NULL
#define elite_mmc_resume	NULL
#endif


#ifdef CONFIG_PM_RUNTIME
static int elite_runtime_pm_get(struct elite_host *host)
{
	return pm_runtime_get_sync(host->mmc->parent);
}

static int elite_runtime_pm_put(struct elite_host *host)
{
	pm_runtime_mark_last_busy(host->mmc->parent);
	return pm_runtime_put_autosuspend(host->mmc->parent);
}

static void elite_mmc_context_save(struct elite_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	elite_mask_irqs(host, INTR_ALL_MASK);
	spin_unlock_irqrestore(&host->lock, flags);

	synchronize_irq(host->dma_irq);
	synchronize_irq(host->regular_irq);

	spin_lock_irqsave(&host->lock, flags);
	host->runtime_suspended = true;
	spin_unlock_irqrestore(&host->lock, flags);
}

/*
 * Restore the MMC host context, if it was lost as result of a
 * power state change.
 */
static int elite_mmc_context_restore(struct elite_host *host)
{
	unsigned long flags;
	struct mmc_ios *ios = &host->mmc->ios;
	
	elite_reset(host);
	elite_do_set_ios(host, ios);
	elite_do_start_signal_voltage_switch(host, ios);

	spin_lock_irqsave(&host->lock, flags);

	host->runtime_suspended = false;

	elite_enable_card_detection(host);

	spin_unlock_irqrestore(&host->lock, flags);

	return 0;
}

static int elite_mmc_runtime_suspend(struct device *dev)
{
	struct mmc_host *mmc = (struct mmc_host *)dev_get_drvdata(dev);
	struct elite_host *host = mmc_priv(mmc);

	elite_mmc_context_save(host);
	dev_dbg(mmc_dev(host->mmc), "disabled\n");
	
	return 0;
}

static int elite_mmc_runtime_resume(struct device *dev)
{
	struct mmc_host *mmc = (struct mmc_host *)dev_get_drvdata(dev);
	struct elite_host *host = mmc_priv(mmc);

	elite_mmc_context_restore(host);
	dev_dbg(mmc_dev(host->mmc), "enabled\n");
	
	return 0;
}
#else
#define elite_mmc_runtime_suspend	NULL
#define elite_mmc_runtime_resume	NULL
#endif //CONFIG_PM_RUNTIME


static const struct dev_pm_ops elite_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(elite_mmc_suspend, elite_mmc_resume)
	SET_RUNTIME_PM_OPS(elite_mmc_runtime_suspend,
				elite_mmc_runtime_resume, NULL)
};

static struct platform_device_id elite_mmc_driver_ids[] = {
	{
		.name		= "elite-mci",
		.driver_data	= (kernel_ulong_t)NULL,
	}, {
		.name		= "elite-mci.0",
		.driver_data	= (kernel_ulong_t)NULL,
	}, {
		.name		= "elite-mci.1",
		.driver_data	= (kernel_ulong_t)NULL,
	}, {
		.name		= "elite-mci.2",
		.driver_data	= (kernel_ulong_t)NULL,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, elite_mmc_driver_ids);

static struct platform_driver elite_mmc_driver = {
	.driver = {
		.name = "s3graphics-elite-mmc",
		.owner = THIS_MODULE,
		.pm = &elite_dev_pm_ops,
		.of_match_table = of_match_ptr(elite_mmc_of_match),
	},
	.id_table	= elite_mmc_driver_ids,
	.probe      = elite_mmc_probe,
	.remove     = elite_mmc_remove,
	.shutdown   = elite_mmc_shutdown,
};

module_platform_driver(elite_mmc_driver);

module_param(fmax, uint, 0444);

MODULE_DESCRIPTION("ELITE MMC HOST to SD/MMC Bridge driver");
MODULE_AUTHOR("S3 Graphics, Inc");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:elite-mci");

