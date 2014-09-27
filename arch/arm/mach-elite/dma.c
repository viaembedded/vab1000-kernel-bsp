/*
	arch/arm/mach-elite/dma.c - DMA 4 driver

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
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#ifdef CONFIG_PM
#include <linux/pm.h>
#endif
#include <linux/delay.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/delay.h>
#include <asm/io.h>
#include <linux/dma-mapping.h>
#include <mach/dma.h>
#include <mach/io.h>
#include <linux/clk.h>

#define MAX_DESCRIPT_SIZE SZ_1K

#define DMA_IRQ_NO 47

int revise_descript(dmach_t ch);

extern unsigned int elite_read_oscr(void);


struct elite_dma_channel {
	unsigned int channel_no;
	unsigned char *mmio;
	int irq;                    /* IRQ Index used by the channel*/
	const char *device_id;      /* device name*/
	void (*callback)(void *data);    /* to call when buffers are done*/
	void *callback_data;        /* with private data ptr*/
	unsigned int device_no;
	unsigned int  chunk_size;
	int in_use;                 /* Does someone own the channel*/
	int if0descnt;                 /*  descript 0 current index*/
	int if1descnt;                 /*descript 1 current index*/
	int max_if0descnt;      /*the largest descript 0 number*/
	int max_if1descnt;      /*the largest descript 1 number*/
	int residue_if0descnt ; /*residue descript 0 count need to be xfer*/
	int residue_if1descnt ; /*residue descript 1 count need to be xfer*/
	unsigned long *if0des_addr;
	unsigned long *if1des_addr;
	dma_addr_t if0des_phy_addr ;
	dma_addr_t if1des_phy_addr ;
	unsigned int accu_size; /*accumlate size between the dma interrupt*/
	unsigned int descript_size; /*the max descript size*/

} ;

struct elite_dma_controller {
	unsigned char *mmio;
	struct elite_dma_channel dma_channels[MAX_DMA_CHANNELS];
};

struct des_attribute {
	unsigned int end_descript;
	unsigned int interrupt_en;
	unsigned int size;
	unsigned int fmt;
	dma_addr_t data_addr;
	dma_addr_t branch_addr;
};

static struct elite_dma_controller dma_controller;

static DEFINE_SPINLOCK(dma_list_lock);

/*===========================================================================*/
/*  dma_irq_handler*/
/**/
/*  return: 0*/
/*===========================================================================*/
static irqreturn_t dma_irq_handler(int irq, void *dev_id)
{
	int ch ;
	unsigned char channel_st;
	struct elite_dma_channel *dma_chan;

	dma_chan = (struct elite_dma_channel*)dev_id;
	ch = dma_chan->channel_no;

	if(((1 << ch) & readl(dma_controller.mmio + DMA_ISR)) == 0 )
		return IRQ_HANDLED;

	channel_st = readl(dma_controller.mmio + DMA_CCR + ch*4) & DMA_EVT_ID_MASK;

	writel(1 << ch , dma_controller.mmio + DMA_ISR);


	/*
	* Handle channel DMA error
	*/
	if ((channel_st == DMA_EVT_NO_STATUS)) {
		revise_descript(ch);
	} else if ((channel_st != DMA_EVT_SUCCESS) && (channel_st != DMA_EVT_NO_STATUS)) {
		/* 1. clear error/abort status*/
		/* 2. re-program src/des/cnt reg 0 and 1*/
		/* 3. write "1" to csr bit6 to reset the buffer pointer to 0*/
		/* 4. re-enable dma channle*/
		printk(KERN_ERR "ch=%d status=0x%.2x err\n\r",
			ch, channel_st) ;
		/*
		 * dma->callback(dma->callback_data) ; 
		 * if callback runs, audio driver think this descp is done
		 * Vincent 2009/05/19
		 */
		elite_resume_dma(ch);
		/* free buffer and callback to handle error*/
		return IRQ_HANDLED ;
	}
	/*
	* Decrease the channel descript usage indicator.
	*/
	if (dma_chan->residue_if0descnt > 0)
		--dma_chan->residue_if0descnt;
	if (dma_chan->callback)
		dma_chan->callback(dma_chan->callback_data) ;

	return IRQ_HANDLED;
}

int create_fmt0_descript(
	dmach_t ch,
	unsigned int device,
	struct des_attribute descript_attr,
	int interface)
{
	struct elite_dma_channel *dma_chan;
	struct dma_des_fmt0 descript;
	unsigned int req_cnt = 0;
	unsigned int des_offset;

	dma_chan = &dma_controller.dma_channels[ch];

	if ((ch >= MAX_DMA_CHANNELS) || (dma_chan->in_use == 0)) {
		printk("%s: bad DMA identifier\n", __func__) ;
		return -EINVAL ;
	}
	if (!interface) {
		des_offset = dma_chan->if0descnt * sizeof(struct dma_des_fmt0)/sizeof(unsigned long);
	} else {
		des_offset = dma_chan->if1descnt * sizeof(struct dma_des_fmt0)/sizeof(unsigned long);
	}

	if (dma_chan->device_no != device) {
		printk("%s: bad Device_NO\n", __func__) ;
		return -ENODEV ;
	}

	descript.data_addr = (unsigned long)descript_attr.data_addr;
	if (descript_attr.interrupt_en == 1)
		req_cnt |= DMA_INTEN_DES;
	if (descript_attr.end_descript == 1)
		req_cnt |= DMA_DES_END;
	if (descript_attr.size > (SZ_64K - 1))
		return -EOVERFLOW ;

	descript.req_cnt = req_cnt | descript_attr.size;

	if (!interface) {
		*(dma_chan->if0des_addr + des_offset) = descript.req_cnt;
		*(dma_chan->if0des_addr + des_offset + 1) = descript.data_addr;
		++dma_chan->if0descnt;
	} else {
		*(dma_chan->if1des_addr + des_offset) = descript.req_cnt;
		*(dma_chan->if1des_addr + des_offset + 1) = descript.data_addr;
		++dma_chan->if1descnt;
	}

	return 0;

}

int create_fmt1_descript(
	dmach_t ch,
	unsigned int device,
	struct des_attribute descript_attr,
	int interface)
{
	struct elite_dma_channel *dma_chan;
	struct dma_des_fmt1 descript;
	unsigned int req_cnt = 0;
	unsigned int des_offset;
	dma_chan = &dma_controller.dma_channels[ch];

	if ((ch >= MAX_DMA_CHANNELS) || (dma_chan->in_use == 0)) {
		printk("%s: bad DMA identifier\n", __func__) ;
		return -EINVAL ;
	}

	if (!interface) {
		des_offset = (dma_chan->max_if0descnt - 1) * sizeof(struct dma_des_fmt0) / sizeof(unsigned long);
	} else {
		des_offset = (dma_chan->max_if0descnt - 1) * sizeof(struct dma_des_fmt0) / sizeof(unsigned long);
	}

	if (dma_chan->device_no != device) {
		printk("%s: bad Device_NO\n", __func__) ;
		return -ENODEV ;
	}

	descript.data_addr = (unsigned long)descript_attr.data_addr;
	descript.branch_addr = (unsigned long)descript_attr.branch_addr;
	if (descript_attr.interrupt_en == 1)
		req_cnt |= DMA_INTEN_DES;
	if (descript_attr.end_descript == 1)
		req_cnt |= DMA_DES_END;
	if (descript_attr.fmt == 1)
		req_cnt |= DMA_FORMAT_DES1;
	if (descript_attr.size > (SZ_64K - 1))
		return -EOVERFLOW;

	descript.req_cnt = req_cnt | descript_attr.size;
	if (!interface) {
		*(dma_chan->if0des_addr + des_offset) = descript.req_cnt;
		*(dma_chan->if0des_addr + des_offset + 1) = descript.data_addr;
		*(dma_chan->if0des_addr + des_offset + 2) = descript.branch_addr;
		dma_chan->if0descnt = 0;
	} else {
		*(dma_chan->if1des_addr + des_offset) = descript.req_cnt;
		*(dma_chan->if1des_addr + des_offset + 1) = descript.data_addr;
		*(dma_chan->if1des_addr + des_offset + 2) = descript.branch_addr;
		dma_chan->if1descnt = 0;
	}


	return 0;
}

int clear_last_descript(
	dmach_t ch,
	unsigned int device)
{
	struct elite_dma_channel *dma_chan;
	unsigned int des_offset;
	dma_chan = &dma_controller.dma_channels[ch];

	if ((ch >= MAX_DMA_CHANNELS) || (dma_chan->in_use == 0)) {
		printk("%s: bad DMA identifier\n", __func__) ;
		return -EINVAL ;
	}

	if (dma_chan->device_no != device) {
		printk("%s: bad Device_NO\n", __func__) ;
		return -ENODEV ;
	}

	if ((dma_chan->if0descnt - 1 >= 0))
		des_offset = (dma_chan->if0descnt - 1) * sizeof(struct dma_des_fmt0) / sizeof(unsigned long);
	else
		des_offset = (dma_chan->max_if0descnt - 1) * sizeof(struct dma_des_fmt0) / sizeof(unsigned long);

	*(dma_chan->if0des_addr + des_offset) &= ~(DMA_DES_END);

	if (device != MEMORY_DMA_REQ) {
		return 0;
	}
	
	if ((dma_chan->if1descnt - 1 >= 0))
		des_offset = (dma_chan->if1descnt - 1) * sizeof(struct dma_des_fmt0) / sizeof(unsigned long);
	else
		des_offset = (dma_chan->max_if1descnt - 1) * sizeof(struct dma_des_fmt0) / sizeof(unsigned long);

	*(dma_chan->if1des_addr + des_offset) &= ~(DMA_DES_END);

	return 0;
}

int add_descript_m2m(
	dmach_t ch,
	dma_addr_t dma_ptr,
	dma_addr_t dma_ptr2,
	unsigned int size)
{
	struct elite_dma_channel *dma_chan;
	unsigned int residue_size;
	unsigned int xfer_size;
	unsigned int xfer_index;
	struct des_attribute descript_attr;
	int need_add_descript_count = 0;
	int ret = 0;

	descript_attr.end_descript = 0;
	descript_attr.end_descript = 0;
	descript_attr.interrupt_en = 0;

	dma_chan = &dma_controller.dma_channels[ch];
	residue_size = size;
	xfer_index = 0; 

	need_add_descript_count = size/SZ_32K ;
	if (size%SZ_32K)
		++need_add_descript_count;

	if ((ch >= MAX_DMA_CHANNELS) || (dma_chan->in_use == 0)) {
		printk("%s: bad DMA identifier\n", __func__) ;
		return -EINVAL ;
	}

	if ((dma_chan->max_if0descnt - dma_chan->residue_if0descnt) < need_add_descript_count) {
		printk("%s:dma descripts are full\n",__func__);
		return -EBUSY ;
	}
	while (residue_size > 0) {
		if (residue_size == size)
			ret = clear_last_descript(ch, dma_chan->device_no);

		xfer_size = residue_size;
		if (residue_size > SZ_32K) {
 			xfer_size =  SZ_32K;
			residue_size -= SZ_32K;
		} else {
			xfer_size = residue_size;
			residue_size = 0;
		}

		if (dma_chan->if0descnt < dma_chan->max_if0descnt - 1) {
			if (residue_size == 0) {
				descript_attr.end_descript = 1;
				descript_attr.interrupt_en = 1;
			}

			descript_attr.data_addr = dma_ptr;
			descript_attr.size = xfer_size;
			descript_attr.fmt = 0;
			ret = create_fmt0_descript(ch,
				dma_chan->device_no,
				descript_attr, 0);

			descript_attr.data_addr = dma_ptr2;
			ret = create_fmt0_descript(ch,
				dma_chan->device_no,
				descript_attr, 1); 
		} else {
			/* fmt1 descriptor construct the descriptor buffer as a ring buffer*/
			if (residue_size == 0) {
				descript_attr.end_descript = 1;
				descript_attr.interrupt_en = 1;
			}

			descript_attr.data_addr = dma_ptr;
			descript_attr.branch_addr = dma_chan->if0des_phy_addr;
			descript_attr.size = xfer_size ;
			descript_attr.fmt = 1;

			ret = create_fmt1_descript(ch,
				dma_chan->device_no,
				descript_attr, 0);

			descript_attr.data_addr = dma_ptr2;
			descript_attr.branch_addr = dma_chan->if1des_phy_addr;

			ret = create_fmt1_descript(ch,
				dma_chan->device_no,
				descript_attr, 1);
		}
		xfer_index++;
		dma_ptr += xfer_size;
		dma_ptr2 += xfer_size;
	}

	dma_chan->residue_if0descnt += xfer_index;

 	if (dma_chan->residue_if0descnt > dma_chan->max_if0descnt - 1)
 	{
		dma_chan->residue_if0descnt = dma_chan->max_if0descnt - 1;
 	}

	dma_chan->residue_if1descnt += xfer_index;

 	if (dma_chan->residue_if1descnt > dma_chan->max_if1descnt - 1)
 	{
		dma_chan->residue_if1descnt = dma_chan->max_if1descnt - 1;
 	}

	return ret;
}

int add_descript(
	dmach_t ch,
	unsigned int device,
	dma_addr_t dma_ptr,
	unsigned int size)
{
	struct elite_dma_channel *dma_chan;
	unsigned int residue_size;
	unsigned int xfer_size;
	unsigned int xfer_index;
	struct des_attribute descript_attr;
	int need_add_descript_count = 0;
	int ret = 0;

	dma_chan = &dma_controller.dma_channels[ch];
	residue_size = size;
	xfer_index = 0; 

	need_add_descript_count = size/SZ_32K ;
	if (size%SZ_32K)
		++need_add_descript_count;

	if ((ch >= MAX_DMA_CHANNELS) || (dma_chan->in_use == 0)) {
		printk("%s: bad DMA identifier\n", __func__) ;
		return -EINVAL ;
	}

	if ((dma_chan->max_if0descnt - dma_chan->residue_if0descnt) < need_add_descript_count) {
		printk("%s:dma descripts are full\n",__func__);
		return -EBUSY ;
	}
	while (residue_size > 0) {
		if (residue_size == size)
			ret = clear_last_descript(ch, device);

		xfer_size = residue_size;
		if (residue_size > SZ_32K) {
 			//xfer_size = residue_size - SZ_32K;
 			xfer_size =  SZ_32K;
			dma_chan->accu_size += xfer_size;
			residue_size -= SZ_32K;
			dma_ptr += xfer_size * xfer_index;
		} else {
			xfer_size = residue_size;
			dma_ptr += xfer_size * xfer_index;
			dma_chan->accu_size += xfer_size;
			residue_size = 0;
		}

		if (dma_chan->if0descnt < dma_chan->max_if0descnt - 1) {
			descript_attr.end_descript = 0;
			if (residue_size <= SZ_32K)
				descript_attr.end_descript = 1;
			if (dma_chan->accu_size >= dma_chan->chunk_size) {
				descript_attr.interrupt_en = 1;
				dma_chan->accu_size = 0;
				dma_chan->accu_size += xfer_size;
			}
			descript_attr.data_addr = dma_ptr;
			descript_attr.size = xfer_size ;
			descript_attr.fmt = 0;
			ret = create_fmt0_descript(ch,
				device,
				descript_attr, 0);
		} else {
			/* fmt1 descriptor construct the descriptor buffer as a ring buffer*/
			descript_attr.end_descript = 0;
			if (residue_size <= SZ_32K)
				descript_attr.end_descript = 1;
			if (dma_chan->accu_size >= dma_chan->chunk_size) {
				descript_attr.interrupt_en = 1;
				dma_chan->accu_size = 0;
				dma_chan->accu_size += xfer_size;
			}
			descript_attr.data_addr = dma_ptr;
			descript_attr.branch_addr = dma_chan->if0des_phy_addr;
			descript_attr.size = xfer_size ;
			descript_attr.fmt = 1;
			ret = create_fmt1_descript(ch,
				device,
				descript_attr, 0);
		}
		xfer_index++;
	}

	dma_chan->residue_if0descnt += xfer_index;

 	if (dma_chan->residue_if0descnt > dma_chan->max_if0descnt - 1)
 	{
		dma_chan->residue_if0descnt = dma_chan->max_if0descnt - 1;
 	}

	return ret;
}


int revise_descript(dmach_t ch)
{
	struct elite_dma_channel *dma_chan;
	unsigned long flags;
	unsigned int ret = 0;
	unsigned int des_offset = 0;
	unsigned int req_count = 0;
	unsigned int data_address = 0;
        unsigned int now_time = 0;
        unsigned int delay_time = 0;
	unsigned int dma_ccr;
	dma_chan = &dma_controller.dma_channels[ch];

	if ((ch >= MAX_DMA_CHANNELS) || (dma_chan->in_use == 0)) {
		printk("%s: bad DMA identifier\n", __func__) ;
		return -EINVAL ;
	}

	dma_ccr = readl(dma_controller.mmio + DMA_CCR + ch*4);

	if (dma_ccr & (SYSTEM_DMA_RUN))
	{
		spin_lock_irqsave(&dma_list_lock, flags);

		/* force hw to update memory register */
		dma_ccr |= (DMA_UP_MEMREG_EN);
		writel(dma_ccr, dma_controller.mmio + DMA_CCR + 4*ch);
                now_time = elite_read_oscr();

		dma_ccr = readl(dma_controller.mmio + DMA_CCR + ch*4);
		
                while (dma_ccr &  (DMA_UP_MEMREG_EN)) {
                        delay_time = elite_read_oscr() - now_time;
                        if (delay_time > 15) {
                                dma_ccr &= ~DMA_UP_MEMREG_EN;
				writel(dma_ccr, dma_controller.mmio + DMA_CCR + 4*ch);
                                break;
                        }
			dma_ccr = readl(dma_controller.mmio + DMA_CCR + ch*4);
                }

		spin_unlock_irqrestore(&dma_list_lock, flags);
	}
	req_count = readl(dma_chan->mmio + DMA_IF0RBR_CH);
	req_count &= (DMA_DES_REQCNT_MASK);
	data_address = readl(dma_chan->mmio + DMA_IF0DAR_CH);

	des_offset = readl(dma_chan->mmio + DMA_IF0CPR_CH) - dma_chan->if0des_phy_addr ;
	if (req_count  > 0) {
		*(dma_chan->if0des_addr + (des_offset / sizeof(unsigned long))) &= ~(DMA_DES_REQCNT_MASK);
		*(dma_chan->if0des_addr + (des_offset / sizeof(unsigned long)))  |= req_count;
		*(dma_chan->if0des_addr + (des_offset / sizeof(unsigned long)) + 1) = 0;
		*(dma_chan->if0des_addr + (des_offset / sizeof(unsigned long)) + 1) = data_address;
	} else {
		if (des_offset < (dma_chan->max_if0descnt - 1) * DMA_DES0_SIZE) {
			des_offset = readl(dma_chan->mmio + DMA_IF0CPR_CH);
			des_offset += 8;
			writel(des_offset, dma_chan->mmio + DMA_IF0CPR_CH);
		} else
			writel(dma_chan->if0des_phy_addr, dma_chan->mmio + DMA_IF0CPR_CH);

	}

	if (dma_chan->device_no != MEMORY_DMA_REQ) {
		return 0;
	}

	req_count = readl(dma_chan->mmio + DMA_IF1RBR_CH);
	req_count &= (DMA_DES_REQCNT_MASK);
	data_address = readl(dma_chan->mmio + DMA_IF1DAR_CH);

	des_offset = readl(dma_chan->mmio + DMA_IF1CPR_CH) - dma_chan->if1des_phy_addr ;
	if (req_count  > 0) {
		*(dma_chan->if1des_addr + (des_offset / sizeof(unsigned long))) &= ~(DMA_DES_REQCNT_MASK);
		*(dma_chan->if1des_addr + (des_offset / sizeof(unsigned long)))  |= req_count;
		*(dma_chan->if1des_addr + (des_offset / sizeof(unsigned long)) + 1) = 0;
		*(dma_chan->if1des_addr + (des_offset / sizeof(unsigned long)) + 1) = data_address;
	} else {
		if (des_offset < (dma_chan->max_if1descnt - 1) * DMA_DES0_SIZE) {
			des_offset = readl(dma_chan->mmio + DMA_IF1CPR_CH);
			des_offset += 8;
			writel(des_offset, dma_chan->mmio + DMA_IF1CPR_CH);
		} else
			writel(dma_chan->if1des_phy_addr, dma_chan->mmio + DMA_IF1CPR_CH);

	}

	return ret ;
}
/*=============================================================================*/
/**/
/* 	elite_start_dma - submit a data buffer for DMA*/
/*	Memory To Device or Device To Memory*/
/* 	@ch: identifier for the channel to use*/
/* 	@dma_ptr: buffer physical (or bus) start address*/
/*	@dma_ptr2: device FIFO address*/
/* 	@size: buffer size*/
/**/
/*	Memory To Memory*/
/* 	@ch: identifier for the channel to use*/
/* 	@dma_ptr: buffer physical (or bus) source start address*/
/*	@dma_ptr2: buffer physical (or bus) destination start address*/
/* 	@size: buffer size*/
/**/
/* 	This function hands the given data buffer to the hardware for DMA*/
/* 	access. If another buffer is already in flight then this buffer*/
/* 	will be queued so the DMA engine will switch to it automatically*/
/* 	when the previous one is done.  The DMA engine is actually toggling*/
/* 	between two buffers so at most 2 successful calls can be made before*/
/* 	one of them terminates and the callback function is called.*/
/**/
/* 	The @ch identifier is provided by a successful call to*/
/* 	elite_request_dma().*/
/**/
/* 	The @size must not be larger than %MAX_DMA_SIZE.  If a given buffer*/
/* 	is larger than that then it's the caller's responsibility to split*/
/* 	it into smaller chunks and submit them separately. If this is the*/
/* 	case then a @size of %CUT_DMA_SIZE is recommended to avoid ending*/
/* 	up with too small chunks. The callback function can be used to chain*/
/* 	submissions of buffer chunks.*/
/**/
/* 	Error return values:*/
/* 	%-EOVERFLOW:	Given buffer size is too big.*/
/* 	%-EBUSY:	Both DMA buffers are already in use.*/
/* 	%-EAGAIN:	Both buffers were busy but one of them just completed*/
/* 			but the interrupt handler has to execute first.*/
/**/
/*	This function returs 0 on success.*/
/**/
/*=============================================================================*/
int elite_start_dma(dmach_t ch, dma_addr_t dma_ptr, dma_addr_t dma_ptr2, unsigned int size)
{
	unsigned long flags;
	int ret = 0;
	struct elite_dma_channel *dma_chan = &dma_controller.dma_channels[ch];
	unsigned int dma_ccr;

	if (size == 0)
		return -EINVAL ;

	local_irq_save(flags);

	if (dma_chan->device_no == MEMORY_DMA_REQ)
		ret = add_descript_m2m(ch, dma_ptr, dma_ptr2, size);
         else
                ret = add_descript(ch, dma_chan->device_no, dma_ptr, size);

	if (ret) {
		goto start_dma_out;
	}

	dma_ccr = readl(dma_controller.mmio + DMA_CCR + 4*ch);

	if (dma_ccr & SYSTEM_DMA_RUN) {
		dma_ccr |= DMA_WAKE;
		writel(dma_ccr, dma_controller.mmio + DMA_CCR + 4*ch);
	} else {
		writel(dma_chan->if0des_phy_addr, dma_chan->mmio + DMA_IF0CPR_CH);

		if (dma_chan->device_no == MEMORY_DMA_REQ)
			writel(dma_chan->if1des_phy_addr, dma_chan->mmio + DMA_IF1CPR_CH);

		dma_ccr = readl(dma_controller.mmio + DMA_CCR + 4*ch);

		dma_ccr |= (SYSTEM_DMA_RUN | SYSTEM_DMA_REQ_EN);

		writel(dma_ccr, dma_controller.mmio + DMA_CCR + 4*ch);
	}

start_dma_out:
	local_irq_restore(flags);
	return ret ;
}

int elite_wake_dma(
	dmach_t ch,
	dma_addr_t dma_ptr,
	dma_addr_t dma_ptr2,
	unsigned int size)
{
	int ret = 0;
	struct elite_dma_channel *dma_chan;
	unsigned int dma_ccr;

	dma_chan = &dma_controller.dma_channels[ch];

	if (size == 0)
		return -EINVAL ;

	if (size > dma_chan->chunk_size)
		return -EOVERFLOW;

	dma_ccr = readl(dma_controller.mmio + DMA_CCR + 4*ch);

	if (dma_ccr & SYSTEM_DMA_RUN) {
		if (dma_chan->device_no == MEMORY_DMA_REQ) {
			ret = add_descript_m2m(ch, dma_ptr, dma_ptr2, size);
		} else {
			ret = add_descript(ch, dma_chan->device_no, dma_ptr, size);
		}
		dma_ccr = readl(dma_controller.mmio + DMA_CCR + 4*ch);
		dma_ccr |= DMA_WAKE;
		writel(dma_ccr, dma_controller.mmio + DMA_CCR + 4*ch);
	}

	return ret ;
}


/*=============================================================================*/
/**/
/*	elite_request_dma - allocate one of the DMA chanels*/
/*      @channel: Pointer to the location of the allocated channel's identifier*/
/*	@device_id: An ascii name for the claiming device*/
/*	@device: The elite peripheral targeted by this request*/
/*	@callback: Function to be called when the DMA completes*/
/*	@data: A cookie passed back to the callback function*/
/**/
/* 	This function will search for a free DMA channel and returns the*/
/* 	address of the hardware registers for that channel as the channel*/
/* 	identifier. This identifier is written to the location pointed by*/
/* 	@dma_regs. The list of possible values for @device are listed into*/
/* 	linux/include/asm-arm/arch-elite/dma.h as a dma_device_t enum.*/
/**/
/* 	Note that reading from a port and writing to the same port are*/
/* 	actually considered as two different streams requiring separate*/
/* 	DMA registrations.*/
/**/
/* 	The @callback function is called from interrupt context when one*/
/* 	of the two possible DMA buffers in flight has terminated. That*/
/* 	function has to be small and efficient while posponing more complex*/
/* 	processing to a lower priority execution context.*/
/**/
/* 	If no channels are available, or if the desired @device is already in*/
/* 	use by another DMA channel, then an error code is returned.  This*/
/* 	function must be called before any other DMA calls.*/
/**/
/*      return: 0 if successful*/
/**/
/*=============================================================================*/
int elite_request_dma(struct dma_req_s *dma_req)
{
	int ch  ;
	int descript_size = MAX_DESCRIPT_SIZE;
	struct elite_dma_channel *dma_chan = NULL;
	unsigned int dma_ccr;
	unsigned long flags;

	dma_req->channel = -1;

	spin_lock_irqsave(&dma_list_lock, flags);

	for (ch = 1 ; ch < MAX_DMA_CHANNELS ; ++ch) {
		dma_chan = &dma_controller.dma_channels[ch];
		if (dma_chan->in_use == 0)
			break ;
	}
	if (ch >= MAX_DMA_CHANNELS) {
		printk("DMA %s: no free DMA channel available\n", dma_req->device_id);
		spin_unlock_irqrestore(&dma_list_lock, flags);
		return -EBUSY;
	}

	spin_unlock_irqrestore(&dma_list_lock, flags);

	dma_req->channel            = ch;
	dma_chan->device_id      = dma_req->device_id;
	dma_chan->device_no      = dma_req->device ;
	dma_chan->callback       = dma_req->callback;
	dma_chan->callback_data  = dma_req->callback_data ;
	dma_chan->chunk_size	 = dma_req->chunk_size;
	dma_chan->in_use         = 1 ;
	dma_chan->if0descnt = 0 ;
	dma_chan->if1descnt = 0;
	dma_chan->accu_size = 0;
	dma_chan->residue_if0descnt = 0;
	dma_chan->residue_if1descnt = 0;
	dma_chan->descript_size = descript_size;

	writel(1<<ch, dma_controller.mmio + DMA_ISR);
	writel(0x0, dma_chan->mmio + DMA_IF0CPR_CH);
	writel(0x0, dma_chan->mmio + DMA_IF1CPR_CH);
	dma_chan->if0des_addr = (unsigned long *)dma_alloc_coherent(
		NULL,
		descript_size,
		&dma_chan->if0des_phy_addr,
		GFP_KERNEL);

	dma_chan->max_if0descnt = (descript_size - DMA_DES1_SIZE) / DMA_DES0_SIZE + 1;

	writel(1<<ch, dma_controller.mmio + DMA_ISR);

	dma_ccr = (dma_req->addr_wrp_bnd << 30 | dma_req->bst_len << 28 | dma_req->trans_size << 26 
	| dma_req->if12_addr_mode << 24 | dma_req->sw_req << 23 | dma_req->trans_dir << 22);

	writel(dma_ccr, dma_controller.mmio + DMA_CCR + 4*ch);

	if (dma_req->device != MEMORY_DMA_REQ) {
		dma_ccr = readl(dma_controller.mmio + DMA_CCR + 4*ch);
		dma_ccr |= dma_req->device << DMA_REQ_ID_SHIFT;
		writel(dma_ccr, dma_controller.mmio + DMA_CCR + 4*ch);
		writel(dma_req->fifo_addr, dma_chan->mmio + DMA_IF1CPR_CH);
	} else {
		dma_chan->if1des_addr = (unsigned long *)dma_alloc_coherent(NULL,
				descript_size, &dma_chan->if1des_phy_addr, GFP_KERNEL);
		dma_chan->max_if1descnt = (descript_size - DMA_DES1_SIZE)/DMA_DES0_SIZE + 1;
	}

	return 0 ;
}

/*=============================================================================*/
/**/
/*	elite_clear_dma - clear DMA pointers*/
/*	@ch:identifier for the channel to use*/
/**/
/*	This clear any DMA state so the DMA engine is ready to restart*/
/*	with new buffers through elite_start_dma(). Any buffers in flight*/
/*	are discarded.*/
/**/
/*	The @regs identifier is provided by a successful call to*/
/*	elite_request_dma().*/
/**/
/*      return: NULL*/
/*=============================================================================*/
void elite_clear_dma(dmach_t ch)
{
	unsigned long flags;
	struct elite_dma_channel *dma_chan;
	unsigned int dma_ccr;

	dma_chan = &dma_controller.dma_channels[ch];

	local_irq_save(flags);

	dma_ccr = readl(dma_controller.mmio + DMA_CCR + 4*ch);

	dma_ccr &= ~(SYSTEM_DMA_REQ_EN);
	writel(dma_ccr, dma_controller.mmio + DMA_CCR + 4*ch);
	dma_ccr = readl(dma_controller.mmio + DMA_CCR + 4*ch);
	udelay(5);
	dma_ccr &= ~(SYSTEM_DMA_RUN);
	writel(dma_ccr, dma_controller.mmio + DMA_CCR + 4*ch);
	writel(1<<ch, dma_controller.mmio + DMA_ISR);
	dma_chan->if0descnt = 0;
	dma_chan->if1descnt = 0;
	dma_chan->accu_size = 0;
	dma_chan->residue_if0descnt = 0;
	dma_chan->residue_if1descnt = 0;
	local_irq_restore(flags);
}

/*=============================================================================*/
/**/
/* 	elite_free_dma - free a elite DMA channel*/
/* 	@ch: identifier for the channel to free*/
/**/
/* 	This clears all activities on a given DMA channel and releases it*/
/* 	for future requests.  The @ch identifier is provided by a*/
/* 	successful call to elite_request_dma().*/
/**/
/*      return: NULL*/
/**/
/*=============================================================================*/
void elite_free_dma(dmach_t ch)
{
	struct elite_dma_channel *dma_chan;

	if ((unsigned) ch >= MAX_DMA_CHANNELS) {
		printk("%s: bad DMA identifier\n", __func__);
		return ;
	}

	dma_chan = &dma_controller.dma_channels[ch];

	if (dma_chan->in_use == 0) {
		printk("%s: Trying to free DMA%d\n", __func__, ch);
		return;
	}

	if (dma_chan->device_no == DEVICE_RESERVED) {
		printk("%s: Trying to free free DMA\n", __func__);
		return ;
	}

	elite_clear_dma(ch);

	dma_free_coherent(NULL,
		dma_chan->descript_size,
		(void *)dma_chan->if0des_addr,
		(dma_addr_t)dma_chan->if0des_phy_addr);

	if (dma_chan->device_no == MEMORY_DMA_REQ) 
		dma_free_coherent(NULL,
			dma_chan->descript_size,
			(void *)dma_chan->if1des_addr,
			(dma_addr_t)dma_chan->if1des_phy_addr);

	dma_chan->device_no  = DEVICE_RESERVED ;
	dma_chan->device_id  = NULL ;
	dma_chan->if0descnt = 0;
	dma_chan->if1descnt = 0;
	dma_chan->accu_size = 0;
	dma_chan->residue_if0descnt = 0;
	dma_chan->residue_if1descnt = 0;
	dma_chan->max_if0descnt = 0;
	dma_chan->max_if1descnt = 0;
	dma_chan->in_use = 0;
}

/*=============================================================================*/
/**/
/*	elite_reset_dma - reset a DMA channel*/
/*	@ch: identifier for the channel to use*/
/**/
/*	This function resets and reconfigure the given DMA channel. This is*/
/*	particularly useful after a sleep/wakeup event.*/
/**/
/*	The @ch identifier is provided by a successful call to*/
/*	request_dma().*/
/**/
/*      return: NULL*/
/**/
/*=============================================================================*/
void elite_reset_dma(dmach_t ch)
{

	if (ch >= MAX_DMA_CHANNELS) {
		printk("%s: bad DMA identifier\n", __func__);
		return;
	}

	elite_clear_dma(ch);

}


/*=============================================================================*/
/**/
/*	elite_stop_dma - stop DMA in progress*/
/*	@regs: identifier for the channel to use*/
/**/
/*	This stops DMA without clearing buffer pointers. Unlike*/
/*	clear_dma() this allows subsequent use of resume_dma()*/
/*	or get_dma_pos().*/
/**/
/*	The @regs identifier is provided by a successful call to*/
/*	request_dma().*/
/**/
/*=============================================================================*/
void elite_stop_dma(dmach_t ch)
{
	struct elite_dma_channel *dma_chan;
        unsigned int now_time = 0;
        unsigned int delay_time = 0;

	unsigned int dma_ccr;

	dma_chan = &dma_controller.dma_channels[ch];
	if ((ch >= MAX_DMA_CHANNELS) ||
	     (dma_chan->in_use == 0)) {
		printk("%s: bad DMA identifier\n", __func__);
		return;
	}

	dma_ccr = readl(dma_controller.mmio + DMA_CCR + 4*ch);

	dma_ccr &= ~(SYSTEM_DMA_REQ_EN);

	writel(dma_ccr, dma_controller.mmio + DMA_CCR + 4*ch);

	udelay(5);
	
	if (dma_ccr & (SYSTEM_DMA_RUN))
	{
		/* force hardware to update memory register */
		dma_ccr |= DMA_UP_MEMREG_EN;
		writel(dma_ccr, dma_controller.mmio + DMA_CCR + 4*ch);
		now_time = elite_read_oscr();
		while (dma_ccr& DMA_UP_MEMREG_EN) {

                delay_time = elite_read_oscr() - now_time;

                if (delay_time > 15) {
			dma_ccr &= ~DMA_UP_MEMREG_EN;
			writel(dma_ccr, dma_controller.mmio + DMA_CCR + 4*ch);
			break;
                }
		dma_ccr = readl(dma_controller.mmio + DMA_CCR + 4*ch);

            }
	}
	
	dma_ccr &= ~SYSTEM_DMA_RUN;
	writel(dma_ccr, dma_controller.mmio + DMA_CCR + 4*ch);
}

/*=============================================================================*/
/**/
/*	elite_resume_dma - resume DMA on a stopped channel*/
/*	@regs: identifier for the channel to use*/
/**/
/*	This resumes DMA on a channel previously stopped with*/
/*	elite_stop_dma().*/
/**/
/*	The @regs identifier is provided by a successful call to*/
/*	elite_request_dma().*/
/**/
/*=============================================================================*/
void elite_resume_dma(dmach_t ch)
{
	struct elite_dma_channel *dma_chan;
	unsigned int dma_ccr;

	dma_chan = &dma_controller.dma_channels[ch];

	dma_ccr = readl(dma_controller.mmio + DMA_CCR + 4*ch);

	if (dma_ccr & DMA_ACTIVE) {/*if dma was active , disable dma first*/
		dma_ccr &= ~(SYSTEM_DMA_REQ_EN);
		writel(dma_ccr, dma_controller.mmio + DMA_CCR + 4*ch);
		udelay(5);
		dma_ccr = readl(dma_controller.mmio + DMA_CCR + 4*ch);
		dma_ccr &= ~(SYSTEM_DMA_RUN);
		writel(dma_ccr, dma_controller.mmio + DMA_CCR + 4*ch);
	}
	if ((ch >= MAX_DMA_CHANNELS) ||
		(dma_chan->in_use == 0)) {
		return;
	}
	revise_descript(ch);
	dma_ccr = readl(dma_controller.mmio + DMA_CCR + 4*ch);
	dma_ccr |= (SYSTEM_DMA_RUN | SYSTEM_DMA_REQ_EN);
	writel(dma_ccr, dma_controller.mmio + DMA_CCR + 4*ch);
	
}

/*===========================================================================*/
/*  elite_dma_busy*/
/**/
/*  return: 1: busy , 0: not busy , other: fail*/
/*===========================================================================*/
int elite_dma_busy(dmach_t ch)
{
	struct elite_dma_channel *dma_chan;

	unsigned int dma_isr;

	dma_chan = &dma_controller.dma_channels[ch];
	if (ch >= MAX_DMA_CHANNELS || (dma_chan->in_use == 0)) {
		return -1 ;
	}
	
	dma_isr = readl(dma_controller.mmio + DMA_ISR);
	if (dma_isr & (1<<ch)) {
		return 0 ;
	} else
		return 1;
}

unsigned int elite_get_dma_pos(dmach_t ch)
{
	struct elite_dma_channel *dma_chan;
	unsigned long flags;
	unsigned int dma_ccr;
	unsigned int now_time;
	unsigned int delay_time;

	dma_chan = &dma_controller.dma_channels[ch];

	if ((ch >= MAX_DMA_CHANNELS) || dma_chan->in_use == 0) {
		return 0;
	}
	
	dma_ccr = readl(dma_controller.mmio + DMA_CCR + ch*4);

	if (dma_ccr & SYSTEM_DMA_RUN) {
		spin_lock_irqsave(&dma_list_lock, flags);
		dma_ccr |= DMA_UP_MEMREG_EN;
		writel(dma_ccr, dma_controller.mmio + DMA_CCR + ch*4);
		now_time = elite_read_oscr();
		dma_ccr = readl(dma_controller.mmio + DMA_CCR + ch*4);
		while (dma_ccr & DMA_UP_MEMREG_EN) {
			delay_time = elite_read_oscr() - now_time;
			if (delay_time > 15) {
				dma_ccr &= ~DMA_UP_MEMREG_EN;
				writel(dma_ccr, dma_controller.mmio + DMA_CCR + ch*4);
				break;
			}
		}
		spin_unlock_irqrestore(&dma_list_lock, flags);
	}

	return readl(dma_chan->mmio + DMA_IF0DAR_CH);
}

#ifdef CONFIG_PM
int elite_dma_suspend(void)
{
	return 0;
}

int elite_dma_resume(void)
{
	unsigned int dma_gcr;
	unsigned int dma_isr;
	unsigned int dma_ier;
	unsigned int dma_tmr;

	*(unsigned int*)0xfe130254 |= 0x30; /*enalbe dma clock*/
	udelay(1);
	
	dma_gcr = readl(dma_controller.mmio + DMA_GCR);
	dma_gcr |= DMA_GLOBAL_EN;

	dma_isr = readl(dma_controller.mmio + DMA_ISR);
	dma_isr |= ALL_INT_CLEAR;

	dma_ier = readl(dma_controller.mmio + DMA_IER);
	dma_ier |= ALL_INT_EN;

	dma_tmr = readl(dma_controller.mmio + DMA_TMR);
	dma_tmr &= ~SCHEDULE_RR_DISABLE; /*use RR schedule*/

	writel(dma_gcr, dma_controller.mmio + DMA_GCR);
	writel(dma_isr, dma_controller.mmio + DMA_ISR);
	writel(dma_ier, dma_controller.mmio + DMA_IER);
	writel(dma_tmr, dma_controller.mmio + DMA_TMR);
	
	return 0;
}

#else
#define elite_dma_suspend NULL
#define elite_dma_resume  NULL
#endif /* CONFIG_PM */

static void elite_dma_clk_init()
{
    static struct clk *dma_clk = NULL;
    
    if(dma_clk)
    {
        return;
    }

    dma_clk = clk_get_sys("elite-dma.0", "dma");
    if(IS_ERR(dma_clk))
    {
        printk(" clk_get_sys dma error \n");
    }
    else
    {
        clk_enable(dma_clk);
    }
}

/*===========================================================================*/
/*  elite_dma_init*/
/*  return: 0*/
/*===========================================================================*/
static int __init elite_dma_init(void)
{
	int ch ;
	int ret = 0;
	struct elite_dma_channel *dma_chan;
	unsigned int dma_gcr;
	unsigned int dma_tmr;

	memset(&dma_controller, 0, sizeof(dma_controller));
    
    elite_dma_clk_init();

	dma_controller.mmio = (unsigned char*)IO_ADDRESS(DMA_CTRL_CFG_BASE_ADDR);

	dma_chan = &dma_controller.dma_channels[0];

	for (ch = 0 ; ch < MAX_DMA_CHANNELS ; ++ch) {
		dma_chan[ch].channel_no = ch;
		dma_chan[ch].mmio       = dma_controller.mmio + DMA_MEM_REG_OFFSET + DMA_CHAN_REG_SIZE*ch;
		dma_chan[ch].irq        = DMA_IRQ_NO;
		dma_chan[ch].device_no  = DEVICE_RESERVED ;
		dma_chan[ch].in_use     = 0 ;
	}

	dma_gcr = readl(dma_controller.mmio + DMA_GCR);
	dma_gcr |= DMA_SW_RST;
	writel(dma_gcr, dma_controller.mmio + DMA_GCR);
	dma_gcr = readl(dma_controller.mmio + DMA_GCR);
	dma_gcr |= DMA_GLOBAL_EN;
	writel(dma_gcr, dma_controller.mmio + DMA_GCR);

	writel(ALL_INT_CLEAR, dma_controller.mmio + DMA_ISR);
	writel(ALL_INT_EN, dma_controller.mmio + DMA_IER);

	dma_tmr = readl(dma_controller.mmio + DMA_TMR);
	dma_tmr &= ~SCHEDULE_RR_DISABLE; /*use RR schedule*/
	writel(dma_tmr, dma_controller.mmio + DMA_TMR);
	writel(DMA_CTRL_CFG_BASE_ADDR + DMA_MEM_REG_OFFSET, dma_controller.mmio + 0x44);

	for (ch = 0 ; ch < MAX_DMA_CHANNELS ; ++ch) {
		writel(0x100, dma_chan[ch].mmio + DMA_IF0CPR_CH);
		writel(0x100, dma_chan[ch].mmio + DMA_IF1CPR_CH);
		writel(0x0, dma_controller.mmio + DMA_CCR + 4*ch);
	}
	for (ch = 0; ch < MAX_DMA_CHANNELS ; ++ch) {
		ret = request_irq(DMA_IRQ_NO, dma_irq_handler, IRQF_SHARED, "dma", (void*)(&dma_chan[ch]));
		if(ret) {
			goto out;
		}
	}

	return 0;
out:
	return ret;
}

/*===========================================================================*/
/*  dma_exit*/
/**/
/*  return: 0*/
/*===========================================================================*/
static void __exit
elite_dma_exit(void)
{
	int ch;
	struct elite_dma_channel *dma_chan;

	dma_chan = &dma_controller.dma_channels[0];

	for (ch = 0; ch < MAX_DMA_CHANNELS ; ++ch)
		free_irq(DMA_IRQ_NO, (void*)(&dma_chan[ch]));
}

__initcall(elite_dma_init);
__exitcall(elite_dma_exit);

EXPORT_SYMBOL(elite_request_dma);
EXPORT_SYMBOL(elite_free_dma);
EXPORT_SYMBOL(elite_clear_dma);
EXPORT_SYMBOL(elite_reset_dma);
EXPORT_SYMBOL(elite_start_dma);
EXPORT_SYMBOL(elite_stop_dma);
EXPORT_SYMBOL(elite_resume_dma);
EXPORT_SYMBOL(elite_get_dma_pos);
EXPORT_SYMBOL(elite_dma_busy);
