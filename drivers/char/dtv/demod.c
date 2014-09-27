#include <linux/module.h>	
#include <linux/kernel.h>
#include <linux/init.h>			
#include <linux/fs.h>			
#include <linux/cdev.h>
#include <linux/device.h>		
#include <linux/delay.h>		
#include <linux/signal.h>
#include <linux/interrupt.h>	
#include <asm/io.h>				
#include <linux/mm.h>
#include <linux/gfp.h>
#include <linux/slab.h>		
#include <linux/wait.h>			
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include "demod.h"

//#define LINUX_SIM
//#define DEBUG_ON

#ifdef DEBUG_ON
#define DbgOut printk
#else
#define DbgOut(fmt, ...) {}
#endif

static DEFINE_MUTEX(dmd_mutex);

#define DMD_REG_BASE            (0xD8180000)

typedef volatile unsigned int	DMD_REG;		// 32bits Hardware register definition

typedef struct _DMD_REGS
{
	DMD_REG			DMD_CTRL_GCC;				// 0x000, gated clock control  *
	DMD_REG			DMD_CTRL_MER;				// 0x001, Module Enable Register *
	DMD_REG			DMD_CTRL_MSR;				// 0x002, Module Select Register 1 *
	DMD_REG			DMD_CTRL_TCS;				// 0x003, TsClkSel
	DMD_REG			DMD_CTRL_TPL;				// 0x004, Ts Packet Lenth
	DMD_REG			DMD_CTRL_MUXTP;			// 0x005, Mux control for  TsPort
	DMD_REG			DMD_TS_THRESH;			// 0x006, Ts Thres
	DMD_REG			DMD_TS_MAXCNT;			// 0x007, Ts max count
	DMD_REG			DMD_TS_MINCNT;			// 0x008, Ts min count
	DMD_REG			DMD_TS_ERRCNT;			// 0x009, Ts error count
	DMD_REG			DMD_TS_WRPTR;			  // 0x00a, Ts write pointer
	DMD_REG			DMD_TS_RDPTR;			  // 0x00b, Ts read pointer
	DMD_REG			DMD_USB_ENA; 			  // 0x00c, enable USB
	DMD_REG 		DMD_ADC_OM;         // 0x00d, adc mode control, 11,pdn, 10,standby, 00, active
	DMD_REG 		DMD_RESERVE1[82];   // from 0xe-0x5f
	DMD_REG			DMD_TS_FLTPID[16];	// 0x060-0x6f, 16 Ts filter pid
	DMD_REG			DMD_RESERVE2[112];	// 0x070-0xdf, 
	DMD_REG			DMD_DBGPT_SET;		  // 0x0e0, debug port setting
	DMD_REG			DMD_DBG_SELHI;		  // 0x0e1, debug select high
	DMD_REG			DMD_DBG_SELLO;		  // 0x0e2, debug select low
	DMD_REG     DMD_RESERVE3[13];   // 0x0e3-0xef 13 dword.
	DMD_REG     DMD_MEM_RESET;      // 0x0f0 mem reset   *
	DMD_REG     DMD_RESERVE4[15];   // 0x0f1-0xff 15 dword.
	/*Demod Register Definition from here [0x100~]*/
	DMD_REG     DMD_READ_IFOUT;     // 0X100 ReadIfOut
	DMD_REG     DMD_READ_NULL;      // 0X101 null
	DMD_REG     DMD_WRTE_THRD12;    // 0X102 WriteThrd12 
	DMD_REG     DMD_PGA_THRED;      // 0X103 PGA_THRED
	DMD_REG     DMD_PGA_WIND;       // 0X104 PGA WIND
	DMD_REG     DMD_PGA_SPEED;      // 0X105 PGA_SPEED
	DMD_REG     DMD_RESERVED5[2];   // 0X106 - 0x107
	DMD_REG     DMD_AGC_TUN;        // 0X108 PGA_THRED
	DMD_REG     DMD_RESERVED6[7];   // 0X109 - 0x10f
	DMD_REG     DMD_IF_HIGH;        // 0X110 if high
	DMD_REG     DMD_IF_LOW;         // 0X111 if low
	DMD_REG     DMD_IF_DEF;         // 0X112 if def
	DMD_REG     DMD_RF_HIGH;        // 0X113 rf high
	DMD_REG     DMD_RF_LOW;         // 0X114 rf low
	DMD_REG     DMD_RF_DEF;         // 0X115 rf def
	DMD_REG     DMD_RESERVED7[2];   // 0X116 - 0x117
	DMD_REG     DMD_WRTE_THRD1;     // 0X118 write thrd1
	DMD_REG     DMD_WRTE_AGC1H;     // 0X119 write agc1h
	DMD_REG     DMD_WRTE_SHIFT;     // 0X11a write shift
	DMD_REG     DMD_WRTE_AGC1S;     // 0X11b write agc1 speed
	DMD_REG     DMD_WRTE_RFSLP;     // 0X11c write rf slp
	DMD_REG     DMD_WRTE_IFSLP;     // 0X11d write if slp
	DMD_REG     DMD_WRTE_STEP3;     // 0X11e write step 3db
	DMD_REG     DMD_WRTE_PGASLP;    // 0X11f write pga slp and updnstep
	DMD_REG     DMD_READ_AGC1;      // 0X120 read agc1
	DMD_REG     DMD_READ_SHIFT;     // 0X121 read shift
	DMD_REG     DMD_WRTE_OFW;       // 0X122 write over flow window
	DMD_REG     DMD_READ_OFC;       // 0X123 read over flow count
	DMD_REG     DMD_RESERVED8[12];  // 0x124-0x12f
	DMD_REG     DMD_WRTE_BB;        // 0X130 write BB  *
	DMD_REG     DMD_WRTE_FREQOFF;   // 0X131 write frequence offset
	DMD_REG     DMD_RESERVED9[14];  // 0X132-0x13f
	DMD_REG     DMD_DECI_GAINSEL;   // 0X140 deci gain sel
	DMD_REG     DMD_WRTE_THRD2;     // 0X141 write thrd2
	DMD_REG     DMD_WRTE_AGC2S;     // 0X142 write AGC2 step
	DMD_REG     DMD_WRTE_AGC2H;     // 0X143 write AGC2 h
	DMD_REG     DMD_RESERVED10[12]; // 0X144-0x14f
	DMD_REG     DMD_WRTE_SFMODE;    // 0X150 write str force mode
	DMD_REG     DMD_WRTE_SFMENA;    // 0X151 write str force mode enable
	DMD_REG     DMD_WRTE_SESMOD;    // 0X152 write str err shift
	DMD_REG     DMD_WRTE_SESENA;    // 0X153 write str err shift enable
	DMD_REG     DMD_READ_SFOUT;     // 0X154 read str frequence out
	DMD_REG     DMD_WRTE_SFIN;      // 0X155 write str frequence in start
	DMD_REG     DMD_READ_SFUSED;    // 0X156 read str frequence used
	DMD_REG     DMD_WRTE_SLOCKTH1;  // 0X157 write str lock th1
	DMD_REG     DMD_WRTE_SLOCKTH2;  // 0X158 write str lock th2
	DMD_REG     DMD_WRTE_SLOCKSTP1; // 0X159 write str lock step1
	DMD_REG     DMD_WRTE_SLOCKSTP2; // 0X15a write str lock step2
	DMD_REG     DMD_WRTE_SFREQIN;   // 0X15b str freq in            *
	DMD_REG     DMD_WRTE_SFINEND;   // 0X15c write str freq in end
	DMD_REG     DMD_WRTE_SFINSTEP;  // 0X15d write str freq in STEP
	DMD_REG     DMD_WRTE_SBSENA;    // 0X15e write str bs enable *
	DMD_REG     DMD_WRTE_STIMOUT;   // 0X15f write str time out
	DMD_REG     DMD_READ_AGC2;      // 0X160 read agc2
	DMD_REG     DMD_READ_SHIFT2;    // 0X161 read shift
	DMD_REG     DMD_RESERVED11[14]; // 0X162-0x16f
	DMD_REG     DMD_IBFIR_LOW[21];  // 0x170-0x184
	DMD_REG     DMD_IBFIR_HL;       // 0x185
	DMD_REG     DMD_IBFIR_HIGH[11]; // 0x186-0x190
	DMD_REG     DMD_RESERVED12[111]; // 0X191-0x1ff
	/*EQL Register Definition from here [0x200~]*/
	DMD_REG     DMD_READ_EQEAVG;    // 0x200 eql error average
	DMD_REG     DMD_RESERVED13[5];  // 0X201-0x205
	DMD_REG     DMD_WRTE_QAM16;     // 0x206 WriteDataSclQAM16
	DMD_REG     DMD_WRTE_QAM32;     // 0x207 WriteDataSclQAM32
	DMD_REG     DMD_WRTE_QAM64;     // 0x208 WriteDataSclQAM64
	DMD_REG     DMD_WRTE_QAM128;    // 0x209 WriteDataSclQAM128
	DMD_REG     DMD_WRTE_QAM256;    // 0x20a WriteDataSclQAM256
	DMD_REG     DMD_RESERVED14[5];  // 0X20b-0x20f
	DMD_REG     DMD_WRTE_FFELK;     // 0x210 WriteFFELeakage
	DMD_REG     DMD_WRTE_DFELK;     // 0x211 WriteDFELeakage
	DMD_REG     DMD_WRTE_FFEDCE;    // 0x212 WriteFFE DC enable
	DMD_REG     DMD_WRTE_EQLCTL;    // 0x213 Write eql ctl
	DMD_REG     DMD_WRTE_EQTF;      // 0x214 Write eql trck flag
	DMD_REG     DMD_RESERVED15;     // 0x215
	DMD_REG     DMD_EQL_SHIFT[20];  // 0x216-0X229 eql shift reg
	DMD_REG     DMD_RESERVED16[114];// 0X22a-0x29b
	DMD_REG     DMD_EQL_SHIFTH;     // 0x29c eql shift top bits
	DMD_REG     DMD_RESERVED17[8];  // 0X29d-0x2a4
	DMD_REG     DMD_WRTE_FFECS;     // 0x2a5 Write FFE CMA SHIFT
	DMD_REG     DMD_WRTE_DFECS;     // 0x2a6 Write DFE CMA SHIFT
	DMD_REG     DMD_WRTE_FFELMSS;   // 0x2a7 Write FFE LMS SHIFT
	DMD_REG     DMD_WRTE_DFELMSS;   // 0x2a8 Write DFE LMS SHIFT
	DMD_REG     DMD_RESERVED18;     // 0x2a9
	DMD_REG     DMD_WRTE_SSLICERE;  // 0x2aa Write soft slicer enable
	DMD_REG     DMD_RESERVED19[5];  // 0X2ab-0x2af	
	DMD_REG     DMD_WRTE_ER2QM4;    // 0x2b0 WriteEqlR2QAM4
	DMD_REG     DMD_WRTE_ER2QM16;   // 0x2b1 WriteEqlR2QAM16
	DMD_REG     DMD_WRTE_ER2QM32;   // 0x2b2 WriteEqlR2QAM32
	DMD_REG     DMD_WRTE_ER2QM64;   // 0x2b3 WriteEqlR2QAM64
	DMD_REG     DMD_WRTE_ER2QM128;  // 0x2b4 WriteEqlR2QAM128
	DMD_REG     DMD_WRTE_ER2QM256;  // 0x2b5 WriteEqlR2QAM256
	DMD_REG     DMD_RESERVED20[5];  // 0X2b6-0x2ba
	DMD_REG     DMD_WRTE_FFEPNS;    // 0x2bb Write FFE PN SHIFT
	DMD_REG     DMD_WRTE_DFEPNS;    // 0x2bc Write DFE PN SHIFT
	DMD_REG     DMD_WRTE_FFESHIFT;  // 0x2bd Write FFE SHIFT
	DMD_REG     DMD_WRTE_DFESHIFT;  // 0x2be Write DFE SHIFT
	DMD_REG     DMD_FRM_FFELKSTP1;  // 0x2bf frm ffe leakage step1 [0-1]
	DMD_REG     DMD_FRM_FFELKSTP2;  // 0x2c0 frm ffe leakage step2 [0-27]
	DMD_REG     DMD_RESERVED21[3];  // 0X2c1-0x2c3
	DMD_REG     DMD_FRM_DFEOUTS;    // 0x2c4 frm dfe out shift
	DMD_REG     DMD_WRTE_DFE2FFEE;  // 0x2c5 write frm ffe to def enable
	DMD_REG     DMD_WRTE_FFESB;     // 0x2c6 write frm ffe shift burst
	DMD_REG     DMD_WRTE_DFESB;     // 0x2c7 write frm dfe shift burst
	DMD_REG     DMD_DVBC_BSTENA;    // 0x2c8 write dvbc burst det enable
	DMD_REG     DMD_DVBC_BSTTHR;    // 0x2c9 write dvbc burst det thr
	DMD_REG     DMD_DVBC_BSTCNT;    // 0x2ca write dvbc burst det cnt
	DMD_REG     DMD_DVBC_EQEAVG;    // 0x2cb read dvbc eql error avg
	DMD_REG     DMD_RESERVED22;     // 0x2cc
	DMD_REG     DMD_DVBC_BSTFLG;    // 0x2cd read dvbc burst flag
	DMD_REG     DMD_RESERVED23[50]; // 0X2ce-0x2ff
	
	/*DVBC Register Definition RegAddr[11:8] == 4'b1011 from here [0x300~]*/
	DMD_REG     DMD_WRTE_MT;        // 0x300 write mt      *
	DMD_REG     DMD_READ_CFFREQ;    // 0x301 read cr freq
	DMD_REG     DMD_READ_CRPHNLVL;  // 0x302 read cr phn level
	DMD_REG     DMD_READ_CRKIP;     // 0x303 read cr ki/kp in use
	DMD_REG     DMD_WRTE_FREQFBTL;  // 0x304 Write Freq fb Th Lo
	DMD_REG     DMD_WRTE_FREQFBTH;  // 0x305 Write Freq fb Th Hi
	DMD_REG     DMD_WRTE_CRFBWENA;  // 0x306 Write Cr Force Bw En 
	DMD_REG     DMD_RESERVED24;     // 0x307 
	DMD_REG     DMD_WRTE_CRFKIP;    // 0x308 Write Cr Force Ki/Kp
	DMD_REG     DMD_WRTE_CRPTH1;    // 0x309 Write Cr Phn Th1
	DMD_REG     DMD_WRTE_CRPTH2;    // 0x30a Write Cr Phn Th2
	DMD_REG     DMD_WRTE_CRPTH3;    // 0x30b Write Cr Phn Th3
	DMD_REG     DMD_WRTE_CRPTH4;    // 0x30c Write Cr Phn Th4
	DMD_REG     DMD_WRTE_CRPETH1;   // 0x30d Write Cr Pe Th1
	DMD_REG     DMD_WRTE_CRPETH2;   // 0x30e Write Cr Pe Th2
	DMD_REG     DMD_WRTE_CRPETH3;   // 0x30f Write Cr Pe Th3
	DMD_REG     DMD_READ_MTDVBC;    // 0x310 read mt in use and read dvbc state
	DMD_REG     DMD_READ_RSECNT;    // 0x311 read rs error count
	DMD_REG     DMD_READ_RSBER;     // 0x312 read rs ber
	DMD_REG     DMD_RESERVED25[2];  // 0x313-0x314
	DMD_REG     DMD_WRTE_CRTMOUT;   // 0x315 write cr timeout
	DMD_REG     DMD_WRTE_CMAN1;     // 0x316 write cma num1
	DMD_REG     DMD_WRTE_CMAN2;     // 0x317 write cma num2
	DMD_REG     DMD_WRTE_LMSNUM;    // 0x318 Write Lms Num
	DMD_REG     DMD_WRTE_TSTMOUT;   // 0x319 Write Ts Time Out  
	DMD_REG     DMD_WRTE_RECAPN1;   // 0x31a Write Re Cap Num1
	DMD_REG     DMD_WRTE_RECAPN2;   // 0x31b Write Re Cap Num2
	DMD_REG     DMD_WRTE_RETRNN;    // 0x31c Write Re Train Num
	DMD_REG     DMD_WRTE_SPTRUMM;   // 0x31d Write Spectrum Mode
	DMD_REG     DMD_WRTE_CRFREQFBE; // 0x31e Write Cr Freq Fb En
	DMD_REG     DMD_WRTE_CRBSTES;   // 0x31f Write Cr Burst Err Shift 
	DMD_REG     DMD_READ_TSLOCK;    // 0x320 Read Ts Locked
	DMD_REG     DMD_WRTE_TSLOCKTH;  // 0x321 Write Ts Lock Th
	DMD_REG     DMD_WRTE_TSUNLOCKTH;// 0x322 Write Ts UnLock Th
	DMD_REG     DMD_WRTE_CRFBSIGN;  // 0x323 Write Cr Fb Sign
	DMD_REG     DMD_WRTE_CRFFOE;    // 0x324 Write Cr Force Freq Offset En
	DMD_REG     DMD_WRTE_CRFFOFF;   // 0x325 Write Cr Force Freq Offset
} DMD_REGS, *PDMD_REGS;

#define DEVNAME					        "Demod"
#define ADDRESS_RANGE           (0x1000)

int dmd_major = 235;
int dmd_minor = 0;
int number_of_devices = 1;
struct class *dmd_class;
PDMD_REGS pdmd_regs;

struct symboler_dev
{
	int sym_var;
	struct cdev cdev;
};
struct symboler_dev *symboler_dev;

//	*(0xD8180500) = 0x1;
//	*(0xD818050c) = 0x18;
//	*(0xD8180c78) = 0x0;
//	*(0xD8180b20) = 0x1;
//	*(0xD8180C00) = 0x3;
//	*(0xD818057c) = 0x8;
static int elite_get_chip_version()
{ 
    int iVersion = 0;
    volatile unsigned int * pChipVersionNumber = (unsigned int *)ioremap(0xD80A8500, 4);

    if(pChipVersionNumber)
    {
        iVersion = (*pChipVersionNumber)&0x0F;
				DbgOut(KERN_INFO " the chip id is %02x\n", iVersion);
        iounmap(pChipVersionNumber);
    }

    return iVersion;
}

static int dmd_open(struct inode *ino, struct file *filep)
{
	int ret = 0;
	//in default, open will make demodulator work at 36.16MHz 6.875M symbolrate AUTO QAM
	pdmd_regs->DMD_CTRL_MSR  = 0x00;//disable ts output
	pdmd_regs->DMD_CTRL_MER  = 0x00;//disable DataIN, demod and dvbc modules
	pdmd_regs->DMD_MEM_RESET = 0x00;//mem cell rest, mem ctrl rest
	pdmd_regs->DMD_MEM_RESET = 0x03;//memory rest
	udelay(1000);
	pdmd_regs->DMD_CTRL_MSR  = 0x80;//enable ts output
	pdmd_regs->DMD_USB_ENA   = 0x00;
	pdmd_regs->DMD_AGC_TUN   = 0x40;//enable PGA
	pdmd_regs->DMD_CTRL_GCC  = 0xff;//enable clock
	pdmd_regs->DMD_CTRL_MER  = 0xff;//enable DataIN, demod and dvbc modules
	DbgOut (KERN_INFO "Demod module open.\n");
	return ret;
}

static int dmd_release(struct inode *ino, struct file *filep)
{
	DbgOut (KERN_INFO "Demod module fini......\n");
	pdmd_regs->DMD_CTRL_MSR  = 0x00;//disable ts output
	pdmd_regs->DMD_CTRL_MER  = 0x00;//disable DataIN, demod and dvbc modules
	
	return 0;
}

static ssize_t dmd_read(struct file *pfile, char __user *user_buf, size_t len, loff_t *off)  
{  
	DbgOut(KERN_INFO "Demod module read...\n");
	return 0;
}

static ssize_t dmd_write(struct file *pfile, const char *user_buf, size_t len, loff_t *off)  
{ 
	DbgOut(KERN_INFO "Demod module write...\n");
	return len;
}

static long dmd_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	//DbgOut(KERN_INFO "command: %d \n", cmd);
	unsigned int* pdmd;
	unsigned int  data;
	switch (cmd)
	{
		case IOCTL_SET_LOCK:
			pdmd_regs->DMD_CTRL_MSR  = 0x00;//disable ts output
			pdmd_regs->DMD_CTRL_MER  = 0x00;//disable DataIN, demod and dvbc modules
			pdmd_regs->DMD_WRTE_SBSENA = 0x00;		
			DbgOut (KERN_INFO "lock and enable SBS\n");	
			break;
            //liudj
		case IOCTL_GET_SIGQ:
			data = pdmd_regs->DMD_READ_EQEAVG & 0xff ;
			pdmd = (unsigned int*)arg;
			if (copy_to_user(pdmd, &data, sizeof(unsigned int)))
			{
				return -EACCES;
			}
			DbgOut (KERN_INFO "signal quality is %d\n", 0);
			break;
            //liudj
		case IOCTL_GET_SIGL:
			data = pdmd_regs->DMD_READ_IFOUT & 0xff ;
			pdmd = (unsigned int*)arg;
			if (copy_to_user(pdmd, &data, sizeof(unsigned int)))
			{
				return -EACCES;
			}
			DbgOut (KERN_INFO "signal level is %d\n", 0);
			break;
		case IOCTL_GET_SIGS:
			DbgOut (KERN_INFO "signal strength is %d\n", 0);
			break;
		case IOCTL_CHECK_LOCK:
			data = pdmd_regs->DMD_READ_TSLOCK&0x1;
			pdmd = (unsigned int*)arg;
			if (copy_to_user(pdmd, &data, sizeof(unsigned int)))
			{
				return -EACCES;
			}
			DbgOut (KERN_INFO "Lock state is %d\n", 0);
			break;
		case IOCTL_SET_SYM:
			pdmd = (unsigned int*)arg;
			if (copy_from_user(&data, pdmd, sizeof(unsigned int)))
			{
				return -EACCES;
			}
			pdmd_regs->DMD_WRTE_SBSENA = 0x00;
			//set variable symbol rate or not, if var, then start, end, step need to be set
			//FSYM = ( SAMPLE / ( FSYM )/ 2.0 * ( 1<<24) + ( 1<<14) ) ;
			//we shall put float math to user mode
			DbgOut (KERN_INFO "SYM is %d\n", data);
			pdmd_regs->DMD_WRTE_SFREQIN = data&0xFFFFFFF;
			break;
		case IOCTL_SET_QAM:
			pdmd = (unsigned int*)arg;
			if (copy_from_user(&data, pdmd, sizeof(unsigned int)))
			{
				return -EACCES;
			}
			DbgOut (KERN_INFO "QAM is %d\n", data);
			pdmd_regs->DMD_WRTE_MT      = data&0x7;
			break;
		case IOCTL_SET_AUTOAGC:
			//set auto enable
			pdmd = (unsigned int*)arg;
			if (copy_from_user(&data, pdmd, sizeof(unsigned int)))
			{
				return -EACCES;
			}
			DbgOut (KERN_INFO "AUTOAGC is %d\n", data);
			pdmd_regs->DMD_AGC_TUN   = data&0x40;//enable PGA
			break;
		case IOCTL_SET_IF:
			pdmd = (unsigned int*)arg;
			if (copy_from_user(&data, pdmd, sizeof(unsigned int)))
			{
				return -EACCES;
			}
			//we shall put float math to user mode
			DbgOut (KERN_INFO "IF is %d\n", data);
			pdmd_regs->DMD_WRTE_BB = data&0xFFFFFF;
			break;
		case IOCTL_SET_ENABLE:
			DbgOut (KERN_INFO "enable ts and demod dvbc modules\n");
			pdmd_regs->DMD_CTRL_MSR  = 0x80;//enable ts output
			pdmd_regs->DMD_AGC_TUN   = 0x40;//enable PGA
			pdmd_regs->DMD_CTRL_GCC  = 0xff;//enable clock
			pdmd_regs->DMD_CTRL_MER  = 0xff;//enable DataIN, demod and dvbc modules
			break;
		case IOCTL_SET_DISABLE:
			DbgOut (KERN_INFO "disable ts and demod dvbc modules\n");
			pdmd_regs->DMD_CTRL_MSR  = 0x00;//disable ts output
			pdmd_regs->DMD_CTRL_MER  = 0x00;//disable DataIN, demod and dvbc modules
			pdmd_regs->DMD_MEM_RESET = 0x00;
			pdmd_regs->DMD_MEM_RESET = 0x03;
			break;
		case IOCTL_SET_ADCOM:
			DbgOut (KERN_INFO "set adc om\n");
			pdmd = (unsigned int*)arg;
			if (copy_from_user(&data, pdmd, sizeof(unsigned int)))
			{
				return -EACCES;
			}
			//we shall put float math to user mode
			DbgOut (KERN_INFO "om is %d\n", data);
			pdmd_regs->DMD_ADC_OM = data&0x3;
			break;
		case IOCTL_DMD_RESET:
			DbgOut (KERN_INFO "demod reset \n");
			pdmd_regs->DMD_CTRL_MER = 0x00;
			pdmd_regs->DMD_CTRL_MER = 0xFF;
			break;
		case IOCTL_SET_REG:
			pdmd = (unsigned int*)arg;
			if (copy_from_user(&data, pdmd, sizeof(unsigned int)))
			{
				return -EACCES;
			}
			//we shall put float math to user mode			

#ifndef 	LINUX_SIM		
			{
				volatile unsigned int * register0 = (unsigned int *)ioremap((DMD_REG_BASE + (data&0xFFFF)), 4);
				if (register0)
				{
					*register0 = (data>>16);
					iounmap(register0);
				}
			}
#else			
			*(volatile unsigned int*)((int)pdmd_regs + (data&0xFFFF)) = data>>16;
#endif
			break;		
		default:
			DbgOut (KERN_INFO "unknow ioctl command %02x\n", cmd);
			break;
	}
	return 0;
}


struct file_operations dmd_fops = 
{
	.owner			=	THIS_MODULE,
	.open			  =	dmd_open,
	.release		=	dmd_release,
	.read			  =	dmd_read,  
	.write			=	dmd_write, 
	.unlocked_ioctl	=	dmd_ioctl,
};

#ifdef CONFIG_PM
/*
 * notify clients before sleep
 */
static int dmd_suspend(struct platform_device *dev, pm_message_t state)
{
	DbgOut(KERN_INFO "Demod module suspended...\n");
	return 0;
}

/*
 * reset bus after sleep
 */
static int dmd_resume(struct platform_device *dev)
{
	DbgOut(KERN_INFO "Demod module resumed...\n");
	return 0;
}
#endif /* CONFIG_PM */

static struct platform_driver dmd_pfdrv = {
	.driver = {
		.name = "dmd",
	},
#ifdef CONFIG_PM
	.suspend = dmd_suspend,
	.resume = dmd_resume,
#endif
};

static struct platform_device dmd_pfdev = {
	.name = "dmd",
};

static int __init
dmd_dummy_probe(struct platform_device *dev)
{
	if (dev == &dmd_pfdev)
		return 0;
	return -ENODEV;
}

static int __init dmd_init(void)
{
//	int ret, result, error;
	
//dev_t devno = MKDEV(dmd_major, dmd_minor);
//
//result = register_chrdev_region (devno, number_of_devices, "dmd");
//if (result < 0)
//{
//	DbgOut (KERN_WARNING "dmd: can't get major number %d\n", dmd_major);
//	return result;
//}
//symboler_dev = kmalloc(sizeof(struct symboler_dev), GFP_KERNEL);
//if(!symboler_dev)
//{
//	ret = -ENOMEM;
//	DbgOut("create device failed.\n");
//	return ret;
//}
//else
//{
//	symboler_dev->sym_var =0;
//	cdev_init(&symboler_dev->cdev, &dmd_fops);
//	symboler_dev->cdev.owner = THIS_MODULE;
//	symboler_dev->cdev.ops = &dmd_fops;
//	error = cdev_add(&symboler_dev->cdev, devno, 1);
//	if (error)
//		DbgOut(KERN_NOTICE "Error %d adding demux device", error);
//}
	
	if (register_chrdev(dmd_major, "dmd", &dmd_fops)) {
		printk(KERN_ERR "demod: unable to get major %d\n", dmd_major);
		return -1;
	}	

	/* create your own class under /sysfs */
	dmd_class = class_create(THIS_MODULE, DEVNAME);
	if(IS_ERR(dmd_class)) 
	{
		DbgOut("Err: failed in creating class.\n");
		return -1; 
	}

	/* register your own device in sysfs, and this will cause udev to create corresponding device node */
	device_create(dmd_class, NULL, MKDEV(dmd_major, dmd_minor), NULL, "demod");
	platform_device_register(&dmd_pfdev);
	platform_driver_probe(&dmd_pfdrv, dmd_dummy_probe);
	
	// add code
#ifndef LINUX_SIM
	pdmd_regs = (PDMD_REGS) ioremap(DMD_REG_BASE, ADDRESS_RANGE);
#else
	// do simulation with kmalloc
	pdmd_regs = (PDMD_REGS)kmalloc(ADDRESS_RANGE, GFP_KERNEL);
#endif
	elite_get_chip_version();
	DbgOut(KERN_INFO "Demod module init ok...\n");
#ifdef LINUX_SIM

	
#endif
	return 0;
}

module_init(dmd_init);

static void __exit dmd_exit(void)
{
	//dev_t devno = MKDEV (dmd_major, dmd_minor);

	//cdev_del (&symboler_dev->cdev);

	device_destroy(dmd_class, MKDEV(dmd_major, dmd_minor));
	class_destroy(dmd_class);                              

	//unregister_chrdev_region (devno, number_of_devices);
	iounmap(pdmd_regs);
	DbgOut(KERN_INFO "Demod module cleaned up...\n");
}

module_exit(dmd_exit);

MODULE_AUTHOR("Jianqiang Shi");
MODULE_DESCRIPTION("Demod Driver Module");
MODULE_LICENSE("GPL");

