#include <linux/module.h>	
#include <linux/kernel.h>
#include <linux/init.h>	
#include <linux/delay.h>			
#include <linux/fs.h>			
#include <linux/cdev.h>
#include <linux/device.h>		
#include <linux/signal.h>
#include <linux/interrupt.h>	
#include <asm/io.h>				
#include <linux/mm.h>
#include <linux/gfp.h>
#include <linux/slab.h>		
#include <linux/wait.h>			
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include "dmx_module.h"

//#define LINUX_SIM
//#define DEBUG_ON

#ifdef DEBUG_ON
#define DbgOut printk
#else
#define DbgOut(fmt, ...) {}
#endif

#define DMXID               1

#if (DMXID==0)
#define DMX_REG_BASE				(0xD8188000)	
#define IRQ_NUM             (89+32)
#elif (DMXID==1)
#define DMX_REG_BASE				(0xD818A000)	
#define IRQ_NUM             (90+32)
#elif (DMXID==2)
#define DMX_REG_BASE				(0xD818C000)	
#define IRQ_NUM             (91+32)
#else
#define DMX_REG_BASE				(0xD818E000)	
#define IRQ_NUM             (92+32)
#endif

#define DMX_GLB_RST         (0xD8180380)

typedef struct {
	unsigned int value0;
	unsigned int value1;
	unsigned int value2;
}DMX_RW, *PDMX_RW;

typedef struct {
	unsigned int len;
	unsigned int value[12];
}DMX_RW_EX, *PDMX_RW_EX;

typedef struct {
	unsigned int    mode;//pid, 8, 16
	unsigned int    channel;
	unsigned char   select[16];
	unsigned char   mask[16];
	unsigned char   pattern[16];
}DMX_FDATA, *PDMX_FDATA;

typedef enum {
	MODE_8 = 0,
	MODE_16,
	MODE_PID,
}FILTER_MODE;

typedef volatile unsigned int	DMX_REG;		// 32bits Hardware register definition

typedef struct _DMX_REGS
{
	DMX_REG			DMX_CTRL_REG0;				// offset : 0x0
	DMX_REG			PID_CTRL_REG0;				// 0x4
	DMX_REG			PID_REG0[16];				// pid register
	DMX_REG			VIDEO_EVEN_CW_HI;
	DMX_REG			VIDEO_EVEN_CW_LO;
	DMX_REG			VIDEO_ODD_CW_HI;
	DMX_REG			VIDEO_ODD_CW_LO;
	DMX_REG			PCR_READY;
	DMX_REG			PCR_BASE;
	DMX_REG			PCR_EXT;
	DMX_REG			VIDEO_PTS_DTS_FIFO;
	DMX_REG			VIDEO_PTS_DTS_FLAGS;
	DMX_REG			VIDEO_PTS;
	DMX_REG			VIDEO_DTS;
	DMX_REG			PID_CTRL_REG1;
	DMX_REG			RESERVED0;
	DMX_REG			STC_CTRL;
	DMX_REG			STC_DATA_HIGH;
	DMX_REG			STC_DATA_LOW;
	DMX_REG			STC_ADJUST;
	DMX_REG			VBB_LEVEL;
	DMX_REG			ABB_LEVEL;
	DMX_REG			READ_STC_HIGH;
	DMX_REG			READ_STC_LOW;
	DMX_REG			ERR_STATUS;
	DMX_REG			ERR_MASK;	
	DMX_REG			AUDIO_EVEN_CW_HI;
	DMX_REG			AUDIO_EVEN_CW_LO;
	DMX_REG			AUDIO_ODD_CW_HI;
	DMX_REG			AUDIO_ODD_CW_LO;
	DMX_REG			AUDIO_IV_HIGH;
	DMX_REG			AUDIO_IV_LOW;
	DMX_REG			PACKET_STATUS;
	DMX_REG			PMS_ACCESS_DATA;
	DMX_REG			PMS_ACCESS_CTRL;
	DMX_REG			SECTION_FILETER_OVERALL_CTRL;
	DMX_REG			SECTION_FILETER_IND_ENABLE;
	DMX_REG			PSI_DATA_PATH3;				
	DMX_REG			PSI_DATA_PATH2;				
	DMX_REG			SBB_LEVEL;
	DMX_REG			SBB_READ_POINTER;	
	DMX_REG			SBB_WRITE_POINTER;
	DMX_REG			VBB_READ_POINTER;	
	DMX_REG			VBB_WRITE_POINTER;
	DMX_REG			PULSE_NUM;
	DMX_REG			PRESCALER_CLK_RECOVERY;
	DMX_REG			RESERVED1;
	DMX_REG			PSI_DATA_PATH1;
	DMX_REG			PSI_DATA_PATH;
	DMX_REG			ABB_READ_POINTER;
	DMX_REG			ABB_WRITE_POINTER;
	DMX_REG			VIDEO_IV_HIGH;				
	DMX_REG			VIDEO_IV_LOW;				
	DMX_REG			VIB_LEVEL;
	DMX_REG			VIB_READ_POINTER;
	DMX_REG			VIB_WRITE_POINTER;
	DMX_REG			DMX_CTRL_REG1;				// 72
	DMX_REG			DMX_BUF_CTRL;
	DMX_REG			INT_STATUS;
	DMX_REG			INT_MASK;
	DMX_REG			INT_CLEAR;
	DMX_REG			PID_REG1[16];				// 77 ~ 92
	DMX_REG			PID_FILTER_ENABLE;	
	DMX_REG			CDI_DATA_INPUT;	
	DMX_REG			EK1[4];
	DMX_REG			EK2[4];
	DMX_REG			EK3[4];
	DMX_REG			DBGREG[4];
	DMX_REG			RESERVED3;					// 111
} DMX_REGS, *PDMX_REGS;

typedef struct _dmx_hw_rst_
{
	DMX_REG     DMX_HW_RESET;
	DMX_REG     Reserved;
}DMX_HW_RST, *PDMX_HW_RST;



#if (DMXID==0)
#define DEVNAME					"DMX_0"
#elif (DMXID==1)
#define DEVNAME					"DMX_1"
#elif (DMXID==2)
#define DEVNAME					"DMX_2"
#else
#define DEVNAME					"DMX_3"
#endif

#define ADDRESS_RANGE           (0x400)

static DECLARE_WAIT_QUEUE_HEAD(DmxWaitQ);
static int flag = 0;
static int resumed = 0;

static int dmx_major = 232;
static int dmx_minor = 0;
static int int_status = 0;
static int number_of_devices = 1;
static struct class *dmx_class;
static PDMX_REGS pdmx_regs;
static PDMX_REGS pdmx_regs_bak;

static PDMX_HW_RST gp_dmx_glb_rst;

struct symboler_dev
{
	int sym_var;
	struct cdev cdev;
};
static struct symboler_dev *symboler_dev;

static void dmx_soft_reset(PDMX_REGS pregs)
{
	memset(pregs, 0, sizeof(DMX_REGS));
	//pregs->INT_MASK = 0xFFFFFFFF;
	pregs->ERR_MASK = 0xFFFFFFFF;
}

static void register_restore(void)
{
	//int i;
//	
//	for (i=0;i<(sizeof(DMX_REGS)>>2);i++)
//	{
//		pdmx_regs->DMX_CTRL_REG0+(i<<2) = pdmx_regs_bak->DMX_CTRL_REG0+(i<<2);
//	}
	pdmx_regs_bak->DMX_CTRL_REG0 &= ~0x1;
	memcpy((unsigned char*)pdmx_regs, (unsigned char*)pdmx_regs_bak, sizeof(DMX_REGS));
	
}

static void dmx_set_filter(PDMX_REGS pregs, PDMX_FDATA pfilter)
{
	int i;
	int len;
	int channel = pfilter->channel;
	if (channel > 31)
		return;
	if (pfilter->mode == MODE_PID)
	{
		//no action
		return;
	}
	else if (pfilter->mode == MODE_8)
	{
		len = 8;
		
	}else
	{
		len = 16;
	}

	for (i=0;i<len;i++)
	{
		pregs->PMS_ACCESS_DATA = pfilter->select[i]<<16|pfilter->mask[i]<<8|pfilter->pattern[i];		
		pregs->PMS_ACCESS_CTRL =0x80000000|(i<<5)|(unsigned int)(channel);
		
		//DbgOut(KERN_INFO "Filter set:%08x %08x", pregs->PMS_ACCESS_CTRL, pregs->PMS_ACCESS_DATA);
	}

}

//define a argument of tasklet struct
static struct tasklet_struct dmx_tasklet;


static void int_bottom_half_handler(unsigned long data)
{
	DbgOut(KERN_INFO "This is tasklet handler..%ld\n", data);
	flag = 1;
	wake_up(&DmxWaitQ);
}

static irqreturn_t int_top_half_handler(int irq, void *handle, struct pt_regs* regs)
{
#ifndef LINUX_SIM
	pdmx_regs->INT_MASK = 0x07;
	int_status = pdmx_regs->INT_STATUS;
	pdmx_regs->INT_CLEAR = pdmx_regs->INT_STATUS;
	tasklet_schedule(&dmx_tasklet);
#endif
	DbgOut("In int top half\n");
	return IRQ_HANDLED;
}


static int dmx_open(struct inode *ino, struct file *filep)
{
	int ret = 0;	
#ifndef LINUX_SIM
	
	ret = request_irq(IRQ_NUM, int_top_half_handler, IRQF_DISABLED, DEVNAME, NULL);
	if (ret)
	{
		DbgOut(KERN_INFO "Failed register dmx's IRQ\n");
	}
	tasklet_init(&dmx_tasklet, (void*)&int_bottom_half_handler, 123);
	tasklet_disable(&dmx_tasklet);
	tasklet_enable(&dmx_tasklet);
#endif
	DbgOut (KERN_INFO "Dmx module open. IRQ num is %d\n", IRQ_NUM);
	return ret;
}

static int dmx_release(struct inode *ino, struct file *filep)
{
#ifndef LINUX_SIM
	free_irq(IRQ_NUM, NULL);
#endif
	DbgOut (KERN_INFO "Dmx module fini......\n");
	return 0;
}

static ssize_t dmx_read(struct file *pfile, char __user *user_buf, size_t len, loff_t *off)  
{  
	DbgOut(KERN_INFO "Dmx module read...\n");
	//wait_event_interruptible(DmxWaitQ, flag != 0);
	//flag = 0;
        if(copy_to_user(user_buf, (char*)&resumed, sizeof(resumed)))
		return -1;
	if (resumed) resumed = 0;
	return 0;
}

static ssize_t dmx_write(struct file *pfile, const char *user_buf, size_t len, loff_t *off)  
{ 
	DbgOut(KERN_INFO "Dmx module write...\n");
	flag = 1;
	//wake_up(&DmxWaitQ);
	return len;
}

static long dmx_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int  ch_num;
	PDMX_RW pdmx;
	DMX_RW  rw;
	DMX_RW_EX rwex;
	PDMX_RW_EX pdmxex;
	PDMX_FDATA pfilter;
	DMX_FDATA filter;
	
	//DbgOut(KERN_INFO "command: %d \n", cmd);
	switch (cmd)
	{
		case IOCTL_INT_STATUS:
			pdmx = (PDMX_RW)arg;
			rw.value0 = int_status;
			//clear int here
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			//pdmx_regs->INT_CLEAR = rw.value0;
			DbgOut(KERN_INFO "IOCTL_INT_STATUS: %08x\n", rw.value0);
			break;
		case IOCTL_INT_CLEAR:
			pdmx = (PDMX_RW)arg;
			//clear int here
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			pdmx_regs->INT_CLEAR = rw.value0;
			DbgOut(KERN_INFO "IOCTL_INT_CLEAR: %08x\n", rw.value0);
			break;
		case IOCTL_INT_CONFIG:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_INT_CONFIG: %08x\n", rw.value0);
			pdmx_regs->INT_MASK = rw.value0;
			break;
		case IOCTL_ERROR_STATUS:
			pdmx = (PDMX_RW)arg;
			rw.value0 = pdmx_regs->ERR_STATUS;
			//clear int here
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_ERROR_STATUS: %08x\n", rw.value0);
			break;
		case IOCTL_ERROR_MASKS:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_ERROR_MASKS: %08x\n", rw.value0);
			pdmx_regs->ERR_MASK = rw.value0;
			break;
		case IOCTL_INIT_FILTER:
			pdmx = (PDMX_RW)arg;
			DbgOut(KERN_INFO "IOCTL_INIT_FILTER: %08x\n", pdmx->value0);
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_INIT_FILTER: %08x\n", rw.value0);
			pdmx_regs->SECTION_FILETER_OVERALL_CTRL = rw.value0;
			break;
		case IOCTL_INIT_TSH:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_INIT_TSH: %08x\n", rw.value0);
			pdmx_regs->DMX_BUF_CTRL = rw.value0;
			break;
		case IOCTL_GET_CACFG:
			pdmx = (PDMX_RW)arg;
			rw.value0 = pdmx_regs->DMX_CTRL_REG0;
			rw.value1 = pdmx_regs->DMX_CTRL_REG1;
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			break;
		case IOCTL_DMX_CTRL:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_DMX_CTRL: %08x %08x\n", rw.value0, rw.value1);
			if ((rw.value0&0x01) == 0)
			{
				dmx_soft_reset(pdmx_regs);
				pdmx_regs->DMX_CTRL_REG1 |= 0x10;
				udelay(100);				
			}
			else
				pdmx_regs->DMX_CTRL_REG1 &= ~0x10;
			pdmx_regs->DMX_CTRL_REG0 = rw.value0;
			break;
		case IOCTL_SOFT_RESET:
			pdmx_regs->STC_DATA_HIGH = 0xFFFFFFF;
			pdmx_regs->STC_DATA_LOW  = 0;
			pdmx_regs->STC_CTRL = 0x05;
			pdmx_regs->DMX_CTRL_REG1 |= 0x10;
			udelay(100);
			pdmx_regs->DMX_CTRL_REG0 &= ~0x1;
			pdmx_regs->DMX_CTRL_REG0 |=  0x1;
			pdmx_regs->DMX_CTRL_REG1 &= ~(0x10);
			break;
		case IOCTL_INIT_VBB:
			break;
		case IOCTL_INIT_ABB:
			break;
		case IOCTL_INIT_VIB:
			break;
		case IOCTL_INIT_SBB:
			rw.value0 = pdmx_regs->DBGREG[0];
			rw.value1 = pdmx_regs->DBGREG[1];
			rw.value2 = pdmx_regs->DBGREG[2];
			pdmx = (PDMX_RW)arg;
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			break;
		case IOCTL_GET_SBBWO:
			pdmx = (PDMX_RW)arg;
			rw.value0 = pdmx_regs->SBB_WRITE_POINTER;
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_GET_SBBWO: %08x\n", rw.value0);
			break;
		case IOCTL_GET_SBBRO:
			pdmx = (PDMX_RW)arg;
			rw.value0 = pdmx_regs->SBB_READ_POINTER;
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_GET_SBBRO: %08x\n", rw.value0);
			break;
		case IOCTL_GET_VBBWO:
			pdmx = (PDMX_RW)arg;
			rw.value0 = pdmx_regs->VBB_WRITE_POINTER;
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_GET_VBBWO: %08x\n", rw.value0);
			break;
		case IOCTL_GET_VBBRO:
			pdmx = (PDMX_RW)arg;
			rw.value0 = pdmx_regs->VBB_READ_POINTER;
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_GET_VBBRO: %08x\n", rw.value0);
			break;
		case IOCTL_GET_ABBWO:
			pdmx = (PDMX_RW)arg;
			rw.value0 = pdmx_regs->ABB_WRITE_POINTER;
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_GET_ABBWO: %08x\n", rw.value0);
			break;
		case IOCTL_GET_ABBRO:
			pdmx = (PDMX_RW)arg;
			rw.value0 = pdmx_regs->ABB_READ_POINTER;
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_GET_ABBRO: %08x\n", rw.value0);
			break;
		case IOCTL_GET_VIBWO:
			pdmx = (PDMX_RW)arg;
			rw.value0 = pdmx_regs->VIB_WRITE_POINTER;
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_GET_VIBWO: %08x\n", rw.value0);
			break;
		case IOCTL_GET_VIBRO:
			pdmx = (PDMX_RW)arg;
			rw.value0 = pdmx_regs->VIB_READ_POINTER;
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_GET_VIBRO: %08x\n", rw.value0);
			break;
		case IOCTL_GET_VBBLVL:
			pdmx = (PDMX_RW)arg;
			rw.value0 = pdmx_regs->VBB_LEVEL;
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_GET_VBBLVL: %08x\n", rw.value0);
			break;
		case IOCTL_GET_VIBLVL:
			pdmx = (PDMX_RW)arg;
			rw.value0 = pdmx_regs->VIB_LEVEL;
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_GET_VIBLVL: %08x\n", rw.value0);
			break;
		case IOCTL_GET_ABBLVL:
			pdmx = (PDMX_RW)arg;
			rw.value0 = pdmx_regs->ABB_LEVEL;
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_GET_ABBLVL: %08x\n", rw.value0);
			break;
		case IOCTL_GET_SBBLVL:
			pdmx = (PDMX_RW)arg;
			rw.value0 = pdmx_regs->SBB_LEVEL;
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_GET_SBBLVL: %08x\n", rw.value0);
			break;
		case IOCTL_SET_VBBOFF:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_SET_VBBOFF: %08x\n", rw.value0);
			pdmx_regs->VBB_READ_POINTER = rw.value0;
			break;
		case IOCTL_SET_VIBOFF:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_SET_VIBOFF: %08x\n", rw.value0);
			pdmx_regs->VIB_READ_POINTER = rw.value0;
			break;
		case IOCTL_SET_ABBOFF:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_SET_ABBOFF: %08x\n", rw.value0);
			pdmx_regs->ABB_READ_POINTER = pdmx->value0;
			break;
		case IOCTL_SET_SBBOFF:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_SET_SBBOFF: %08x\n", rw.value0);
			pdmx_regs->SBB_READ_POINTER = pdmx->value0;
			break;
		case IOCTL_SET_FPID:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_SET_FPID: %08x %08x\n", rw.value0, rw.value1);
			ch_num = rw.value0;
			if (ch_num<32)
			{
				pdmx_regs->PID_REG0[ch_num>>1] &= (unsigned int)(0xFFFF<<((ch_num&0x01)?0:16));
				pdmx_regs->PID_REG0[ch_num>>1] |= (((0x1FFF)&pdmx->value1)<<((ch_num&0x01)?16:0));
				DbgOut(KERN_INFO "IOCTL_SET_FPID: %08x\n", pdmx_regs->PID_REG0[ch_num>>1]);
			}
			else
			{
				ch_num -= 32;
				pdmx_regs->PID_REG1[ch_num>>1] &= (unsigned int)(0xFFFF<<((ch_num&0x01)?0:16));
				pdmx_regs->PID_REG1[ch_num>>1] |= (((0x1FFF)&pdmx->value1)<<((ch_num&0x01)?16:0));
				DbgOut(KERN_INFO "IOCTL_SET_FPID 1: %08x\n", pdmx_regs->PID_REG1[ch_num>>1]);
			}
			
			break;
		case IOCTL_SET_FPAT:
			pfilter = (PDMX_FDATA)arg;
			/*pause all section filter*/
			//pdmx_regs->SECTION_FILETER_OVERALL_CTRL |= 0x00000020;
			//while((pdmx_regs->SECTION_FILETER_OVERALL_CTRL & 0x80000000) == 0)
			//{
			//	udelay(100);
			//}
			if (copy_from_user(&filter, pfilter, sizeof(DMX_FDATA)))
			{
				pdmx_regs->SECTION_FILETER_OVERALL_CTRL &= ~0x00000020;
				return -EACCES;
			}
			dmx_set_filter(pdmx_regs, &filter);
			//pdmx_regs->SECTION_FILETER_OVERALL_CTRL &= ~0x00000020;
			/*restart all section filter, do we need to clear*/
			break;
		case IOCTL_SET_FSTART:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_SET_FSTART: %08x\n", rw.value0);
			pdmx_regs->SECTION_FILETER_IND_ENABLE |= 1<<rw.value0;
			DbgOut(KERN_INFO "IOCTL_SET_FSTART: %08x\n", pdmx_regs->SECTION_FILETER_IND_ENABLE);
			break;
		case IOCTL_SET_FSTOP:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_SET_FSTOP: %08x\n", rw.value0);
			pdmx_regs->SECTION_FILETER_IND_ENABLE &= ~(1<<rw.value0);
			DbgOut(KERN_INFO "IOCTL_SET_FSTOP: %08x\n", pdmx_regs->SECTION_FILETER_IND_ENABLE);
			break;
		case IOCTL_SET_VPID:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_SET_VPID: %08x\n", rw.value0);
			pdmx_regs->PID_REG0[0] = (pdmx_regs->PID_REG0[0]&0x1FFF0000)|(rw.value0&0x1FFF);
			DbgOut(KERN_INFO "IOCTL_SET_VPID: %08x\n", pdmx_regs->PID_REG0[0]);
			break;
		case IOCTL_SET_APID:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_SET_APID: %08x\n", rw.value0);
			pdmx_regs->PID_REG0[0] = (pdmx_regs->PID_REG0[0]&0xFFFF)|((rw.value0&0x1FFF)<<16);
			DbgOut(KERN_INFO "IOCTL_SET_APID: %08x\n", pdmx_regs->PID_REG0[0]);
			break;
		case IOCTL_SET_VSTART:
			pdmx = (PDMX_RW)arg;
			pdmx_regs->PID_CTRL_REG0 |= 0x1;
			DbgOut(KERN_INFO "IOCTL_SET_VSTART: %08x\n", pdmx_regs->PID_CTRL_REG0);
			break;
		case IOCTL_SET_ASTART:	
			pdmx = (PDMX_RW)arg;
			pdmx_regs->PID_CTRL_REG0 |= 0x2;
			DbgOut(KERN_INFO "IOCTL_SET_ASTART: %08x\n", pdmx_regs->PID_CTRL_REG0);
			break;
		case IOCTL_SET_VSTOP:
			pdmx = (PDMX_RW)arg;
			pdmx_regs->PID_CTRL_REG0 &= ~0x1;
			DbgOut(KERN_INFO "IOCTL_SET_VSTOP: %08x\n", pdmx_regs->PID_CTRL_REG0);
			break;
		case IOCTL_SET_ASTOP:
			pdmx = (PDMX_RW)arg;
			pdmx_regs->PID_CTRL_REG0 &= ~0x2;
			DbgOut(KERN_INFO "IOCTL_SET_ASTOP %08x\n", pdmx_regs->PID_CTRL_REG0);
			break;
		case IOCTL_SET_CSTART:	
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_SET_CSTART: %08x\n", rw.value0);
			if (rw.value0 >= 32)
				pdmx_regs->PID_CTRL_REG1 |= (1<<(rw.value0-32));
			else
				pdmx_regs->PID_CTRL_REG0 |= (1<<rw.value0);
			break;
		case IOCTL_SET_CSTOP:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_SET_CSTOP: %08x\n", rw.value0);
			if (rw.value0 >= 32)
				pdmx_regs->PID_CTRL_REG1 &= ~(1<<(rw.value0-32));
			else
				pdmx_regs->PID_CTRL_REG0 &= ~(1<<rw.value0);
			break;
		case IOCTL_SET_CMODE:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_SET_CMODE: %08x\n", rw.value0);
			if (rw.value0<16)
			{
				if (rw.value1)
				{
						pdmx_regs->PSI_DATA_PATH |= (1<<(2*rw.value0));
						pdmx_regs->PSI_DATA_PATH &= ~(1<<(2*rw.value0+1));  
				}
				else
				{
						pdmx_regs->PSI_DATA_PATH &= ~(1<<(2*rw.value0));
						pdmx_regs->PSI_DATA_PATH &= ~(1<<(2*rw.value0+1)); 
				}
				break;
			}
			if (rw.value0<32)
			{
				rw.value2 = rw.value0 - 16;
				if (rw.value1)
				{
						pdmx_regs->PSI_DATA_PATH1 |= (1<<(2*rw.value2));
						pdmx_regs->PSI_DATA_PATH1 &= ~(1<<(2*rw.value2+1));  
				}
				else
				{
						pdmx_regs->PSI_DATA_PATH1 &= ~(1<<(2*rw.value2));
						pdmx_regs->PSI_DATA_PATH1 &= ~(1<<(2*rw.value2+1)); 
				}
				break;
			}
			if (rw.value0<48)
			{
				rw.value2 = rw.value0 - 32;
				if (rw.value1)
				{
						pdmx_regs->PSI_DATA_PATH2 |= (1<<(2*rw.value2));
						pdmx_regs->PSI_DATA_PATH2 &= ~(1<<(2*rw.value2+1));  
				}
				else
				{
						pdmx_regs->PSI_DATA_PATH2 &= ~(1<<(2*rw.value2));
						pdmx_regs->PSI_DATA_PATH2 &= ~(1<<(2*rw.value2+1)); 
				}
				break;
			}
			if (rw.value0<64)
			{
				rw.value2 = rw.value0 - 48;
				if (rw.value1)
				{
						pdmx_regs->PSI_DATA_PATH3 |= (1<<(2*rw.value2));
						pdmx_regs->PSI_DATA_PATH3 &= ~(1<<(2*rw.value2+1));  
				}
				else
				{
						pdmx_regs->PSI_DATA_PATH3 &= ~(1<<(2*rw.value2));
						pdmx_regs->PSI_DATA_PATH3 &= ~(1<<(2*rw.value2+1)); 
				}
				break;
			}
			break;
		case IOCTL_SET_PCRPID:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw.value0, pdmx, sizeof(unsigned int)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_SET_PCRPID: %08x\n", rw.value0);
			pdmx_regs->PID_REG0[1] = (pdmx_regs->PID_REG0[1]&0x1FFF0000)|(0x1FFF&rw.value0);
			break;
		case IOCTL_SET_PCRDISABLE:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw.value0, pdmx, sizeof(unsigned int)))
			{
				return -EACCES;
			}
			DbgOut(KERN_INFO "IOCTL_SET_PCRDISABLE\n");
			pdmx_regs->PID_REG0[1] = (pdmx_regs->PID_REG0[1]&0x1FFF0000)|(0x1FFF);
			break;
		case IOCTL_INIT_CA:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw.value0, pdmx, sizeof(unsigned int)))
			{
				return -EACCES;
			}
			pdmx_regs->DMX_CTRL_REG1 = rw.value0;
			DbgOut(KERN_INFO "IOCTL_INIT_CA %08x\n", pdmx_regs->DMX_CTRL_REG1);
			break;
		case IOCTL_INIT_EKS:
			pdmxex = (PDMX_RW_EX)arg;
			if (copy_from_user(&rwex, pdmxex, sizeof(DMX_RW_EX)))
			{
				return -EACCES;
			}
			pdmx_regs->EK2[0] = rwex.value[0];
			pdmx_regs->EK2[1] = rwex.value[1];
			pdmx_regs->EK2[2] = rwex.value[2];
			pdmx_regs->EK2[3] = rwex.value[3];
			DbgOut(KERN_INFO "ioctl_init_EK2 %08x %08x %08x %08x\n", pdmx_regs->EK2[0], pdmx_regs->EK2[1], pdmx_regs->EK2[2], pdmx_regs->EK2[3]);
			if (rwex.len==8)
			{
				pdmx_regs->EK3[0] = rwex.value[4];
				pdmx_regs->EK3[1] = rwex.value[5];
				pdmx_regs->EK3[2] = rwex.value[6];
				pdmx_regs->EK3[3] = rwex.value[7];
				DbgOut(KERN_INFO "ioctl_init_EK2 %08x %08x %08x %08x\n", pdmx_regs->EK3[0], pdmx_regs->EK3[1], pdmx_regs->EK3[2], pdmx_regs->EK3[3]);
			}
			
			break;
		case IOCTL_SET_CW:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -EACCES;
			}
			if ((rw.value2&0x10000) && (rw.value2&0x0001))
			{				
				pdmx_regs->VIDEO_EVEN_CW_HI = rw.value1;
				pdmx_regs->VIDEO_EVEN_CW_LO  = rw.value0;
				DbgOut(KERN_INFO "IOCTL_SET_CW V/E %08x %08x\n", pdmx_regs->VIDEO_EVEN_CW_HI, pdmx_regs->VIDEO_EVEN_CW_LO);
			}else if((rw.value2&0x10000) && !(rw.value2&0x0001))
			{
				pdmx_regs->VIDEO_ODD_CW_HI = rw.value1;
				pdmx_regs->VIDEO_ODD_CW_LO  = rw.value0;
				DbgOut(KERN_INFO "IOCTL_SET_CW V/O %08x %08x\n", pdmx_regs->VIDEO_ODD_CW_HI, pdmx_regs->VIDEO_ODD_CW_LO);
			}else if(!(rw.value2&0x10000) && (rw.value2&0x0001))
			{
				pdmx_regs->AUDIO_EVEN_CW_HI = rw.value1;
				pdmx_regs->AUDIO_EVEN_CW_LO  = rw.value0;
				DbgOut(KERN_INFO "IOCTL_SET_CW A/E %08x %08x\n", pdmx_regs->AUDIO_EVEN_CW_HI, pdmx_regs->AUDIO_EVEN_CW_LO);				
			}else
			{
				pdmx_regs->AUDIO_ODD_CW_HI = rw.value1;
				pdmx_regs->AUDIO_ODD_CW_LO  = rw.value0;
				DbgOut(KERN_INFO "IOCTL_SET_CW A/O %08x %08x\n", pdmx_regs->AUDIO_ODD_CW_HI, pdmx_regs->AUDIO_ODD_CW_LO);				
			}
					
			if (pdmx_regs->DMX_CTRL_REG1&0x2)//key ladder mode
			{
				if (!(rw.value2&0x10000) && (rw.value2&0x0001))
				{
					pdmx_regs->DMX_CTRL_REG1 |= (0x11<<10);  //set even&&audio cw update flag
				}
				else if(!(rw.value2&0x10000) && !(rw.value2&0x0001))
				{
						pdmx_regs->DMX_CTRL_REG1 &= ~(1<<10); //set odd cw update flag
						pdmx_regs->DMX_CTRL_REG1 |= (0x01<<11); //set audio cw
				}
				else if((rw.value2&0x10000) && (rw.value2&0x0001))
				{
						pdmx_regs->DMX_CTRL_REG1 |= (1<<10); //set even cw update flag
						pdmx_regs->DMX_CTRL_REG1 &= ~(0x01<<11); //set video					
				}
				else
				{
						pdmx_regs->DMX_CTRL_REG1 &= ~(1<<10); //set odd cw update flag
						pdmx_regs->DMX_CTRL_REG1 &= ~(0x01<<11); //set video						
				}
				while(pdmx_regs->DMX_CTRL_REG1&0x80000000)
				{
					udelay(10);
				}
				pdmx_regs->DMX_CTRL_REG1 |= 0x01; //start key ladder
				DbgOut(KERN_INFO "IOCTL_SET_CW %08x\n", pdmx_regs->DMX_CTRL_REG1);	
			}
						
			break;
		case IOCTL_GET_PCR:
			pdmx = (PDMX_RW)arg;
			rw.value0 = 0;
			rw.value1 = 0;
			if (pdmx_regs->PCR_READY)
			{
				rw.value0 = pdmx_regs->PCR_BASE;
				rw.value1 = pdmx_regs->PCR_EXT;
			}
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -1;
			}
			break;
		case IOCTL_GET_STC:
			pdmx = (PDMX_RW)arg;
			pdmx_regs->STC_CTRL = 0x03;
			rw.value0 = pdmx_regs->READ_STC_HIGH;
			if (copy_to_user(pdmx, &rw, sizeof(DMX_RW)))
			{
				return -1;
			}
			break;
		case IOCTL_SET_STC:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -1;
			}
			pdmx_regs->STC_DATA_HIGH = rw.value0;
			pdmx_regs->STC_DATA_LOW  = 0;
			pdmx_regs->STC_CTRL = 0x05;
			break;
		case IOCTL_TURN_OFF:
			pdmx = (PDMX_RW)arg;
			if (copy_from_user(&rw, pdmx, sizeof(DMX_RW)))
			{
				return -1;
			}
			if (rw.value1) /* globle reset for demux */
			{
				if (rw.value0)
				{
					/* enable */
					gp_dmx_glb_rst->DMX_HW_RESET = 0x1;
					register_restore();
					printk("after enable and recover the dmx control register is %08x\n", pdmx_regs->DMX_CTRL_REG0);
				}
				else
				{
					/* back up demux register here */
					memcpy((unsigned char*)pdmx_regs_bak, (unsigned char*)pdmx_regs, sizeof(DMX_REGS));
					/* disable */
					gp_dmx_glb_rst->DMX_HW_RESET = 0x0;
#ifdef LINUX_SIM 					
					memset(pdmx_regs, 0, sizeof(DMX_REGS));
#endif					
					printk("after disable the dmx control register is %08x\n", pdmx_regs->DMX_CTRL_REG0);
				}
			}
			else
			{
				if (rw.value0)
				{
					pdmx_regs->DMX_CTRL_REG1 &= ~(0x10);
					pdmx_regs->DMX_CTRL_REG0 |= 0x1;
					printk("after reenable the vbb read pointer is %08x\n", pdmx_regs->VBB_READ_POINTER);
				}
				else
				{
					pdmx_regs->STC_DATA_HIGH = 0xFFFFFFF;
					pdmx_regs->STC_DATA_LOW  = 0;
					pdmx_regs->STC_CTRL = 0x05;
					pdmx_regs->DMX_CTRL_REG1 |=  (0x10);
					udelay(100);
					pdmx_regs->DMX_CTRL_REG0 &=  ~0x1;
				}
			}
			break;
		case 0x200:
			int_bottom_half_handler(123);
			break;
		default:
			break;
	}
	return 0;
}


static unsigned int dmx_poll(struct file *filep, struct poll_table_struct *pts)
{
	int mask = 0;
	//DbgOut(KERN_INFO "Kernel: In mypoll");
	poll_wait(filep, &DmxWaitQ, pts);

	if (flag != 0)
	{
		mask = POLLIN | POLLRDNORM;
	}
	flag = 0;
	return mask;
}

static struct file_operations dmx_fops = 
{
	.owner			=	THIS_MODULE,
	.open			  =	dmx_open,
	.release		=	dmx_release,
	.read			  =	dmx_read,  
	.write			=	dmx_write, 
	.unlocked_ioctl	=	dmx_ioctl,
	.poll			  =	dmx_poll,
};

/*
 * notify clients before sleep
 */
static int dmx_suspend(struct platform_device *dev, pm_message_t state)
{
	DbgOut(KERN_INFO "demux module suspended...\n");
	return 0;
}

/*
 * reset bus after sleep
 */
static int dmx_resume(struct platform_device *dev)
{
	DbgOut(KERN_INFO "demux module resumed...\n");
	resumed = 1;
	return 0;
}


static struct platform_driver dmx_pfdrv = {
	.driver = {
		.name = "dmx1",
	},
#ifdef CONFIG_PM
	.suspend = dmx_suspend,
	.resume = dmx_resume,
#endif
};

static struct platform_device dmx_pfdev = {
	.name = "dmx1",
};

static int __init
dmx_dummy_probe(struct platform_device *dev)
{
	if (dev == &dmx_pfdev)
		return 0;
	return -ENODEV;
}

static int __init dmx_init(void)
{
//	int ret, result, error;
//	char name[16];
//	dev_t devno = MKDEV(dmx_major, dmx_minor+DMXID);
//	sprintf(name, "dmx%d", DMXID);
//	result = register_chrdev_region (devno, number_of_devices, name);
//	if (result < 0)
//	{
//		DbgOut (KERN_WARNING "dmx: can't get major number %d\n", dmx_major);
//		return result;
//	}
//	symboler_dev = kmalloc(sizeof(struct symboler_dev), GFP_KERNEL);
//	if(!symboler_dev)
//	{
//		ret = -ENOMEM;
//		DbgOut("create device failed.\n");
//		return ret;
//	}
//	else
//	{
//		symboler_dev->sym_var =0;
//		cdev_init(&symboler_dev->cdev, &dmx_fops);
//		symboler_dev->cdev.owner = THIS_MODULE;
//		symboler_dev->cdev.ops = &dmx_fops;
//		error = cdev_add(&symboler_dev->cdev, devno, 1);
//		if (error)
//			DbgOut(KERN_NOTICE "Error %d adding demux device", error);
//	}

	if (register_chrdev(dmx_major, "dmx1", &dmx_fops)) {
		printk(KERN_ERR "dmx: unable to get major %d\n", dmx_major);
		return -1;
	}
	
	/* create your own class under /sysfs */
	dmx_class = class_create(THIS_MODULE, DEVNAME);
	if(IS_ERR(dmx_class)) 
	{
		DbgOut("Err: failed in creating class.\n");
		return -1; 
	}

	/* register your own device in sysfs, and this will cause udev to create corresponding device node */
	device_create(dmx_class, NULL, MKDEV(dmx_major, dmx_minor+DMXID), NULL, "demux%d", DMXID);
	platform_device_register(&dmx_pfdev);
	platform_driver_probe(&dmx_pfdrv, dmx_dummy_probe);
	
	// add code
#ifndef LINUX_SIM
	pdmx_regs = (PDMX_REGS) ioremap(DMX_REG_BASE, ADDRESS_RANGE);
	pdmx_regs_bak = (PDMX_REGS)kmalloc(sizeof(DMX_REGS), GFP_KERNEL);
	gp_dmx_glb_rst = (PDMX_HW_RST)ioremap(DMX_GLB_RST, sizeof(DMX_HW_RST));
#else
	// do simulation with kmalloc
	pdmx_regs = (PDMX_REGS)kmalloc(sizeof(DMX_REGS), GFP_KERNEL);
	pdmx_regs_bak = (PDMX_REGS)kmalloc(sizeof(DMX_REGS), GFP_KERNEL);
	gp_dmx_glb_rst = (DMX_REG*)kmalloc(sizeof(DMX_HW_RST), GFP_KERNEL);
#endif
    gp_dmx_glb_rst->DMX_HW_RESET = 0x1;
	//dmx_soft_reset(pdmx_regs);
	DbgOut(KERN_INFO "Dmx module init ok...\n");
	return 0;
}

module_init(dmx_init);

static void __exit dmx_exit(void)
{
	//dev_t devno = MKDEV (dmx_major, dmx_minor+DMXID);

	//cdev_del (&symboler_dev->cdev);

	device_destroy(dmx_class, MKDEV(dmx_major, dmx_minor+DMXID));
	class_destroy(dmx_class);                              

	//unregister_chrdev_region (devno, number_of_devices);
#ifndef LINUX_SIM	
	iounmap(pdmx_regs);
	iounmap(gp_dmx_glb_rst);
	kfree(pdmx_regs_bak);
#else
	kfree(pdmx_regs);
	kfree(pdmx_regs_bak);
	kfree(gp_dmx_glb_rst);
#endif	
	DbgOut(KERN_INFO "Dmx module cleaned up...\n");
}

module_exit(dmx_exit);

MODULE_AUTHOR("LJ");
MODULE_DESCRIPTION("Dmx Driver Module");
MODULE_LICENSE("GPL");

