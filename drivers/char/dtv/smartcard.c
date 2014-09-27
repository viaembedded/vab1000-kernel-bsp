/*
 * Driver for elite 1000 and smart card Controllers
 *
 * Copyright (C) S3 Graphics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

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
#include "smartcard.h"

//#define SC_DEBUG
#ifdef SC_DEBUG

#define SC_PRINT(fmt, args...)	 printk(KERN_NOTICE "sc_kernel %s:%d: "fmt, __func__ , __LINE__,## args)
#define ENTER() printk(KERN_NOTICE "sc_kernel Enter %s, file:%s line:%d\n", \
		__func__, __FILE__, __LINE__)

#define LEAVE() printk(KERN_NOTICE "sc_kernel Exit %s, file:%s line:%d\n", \
		__func__, __FILE__, __LINE__)

#else
#define SC_PRINT(fmt, args...) 
#define ENTER()
#define LEAVE()
#endif

static t_sc9600_config sc9600_atr_config;
static t_sc9600_config sc9600_current_config;

static DECLARE_WAIT_QUEUE_HEAD(wait_fd_block);

static t_sc9600_config sc9600_base_config = {
	  SC_STANDARD_ISO7816,	/*SC_STANDARD_EMV2000, ISO7816 is  protocol standard */
	  0,					/* 0 = No effect on RSARESET; or 1 = Invert the RSARESET signal */
	  0,					/* 1 = it transmits two CRC check bytes in the epilogue field; 0 = only one byte */
	  0,					/*1, T=0, T=1 ,T14*/   
	  0,					/* 1 = use PTS/PPS if avaliable, 0 = not use PTS/PPS 	   */
	  15,					/* CWI. Max time between characters from card: CWT = (2^CWI + 11) work ETU  */
	  0,					/* 1 = auto convention;	0 = Prevent the autodetection circuit from overwriting the convention value. */
	  0,					/* 0 = Use the direct convention; 1 = Use the inverse convention */
	  0,					/* 1 = auto ETU;   0 = Prevent the automatic baud rate determining circuit from overwriting the ETU value. */
	  372,				/* ETU */
	  2,					/* Guardtime */
	  RST_ACT_CNT_TDA8004,  /* Time delay (in us) between the activation of RSACMDVCC and the activation of RSARESET. */
	  IN_EN_CNT_TDA8004,	/* the time delay (how many clock cycle) between the activation of RSARESET and the enabling of smart card input data.
							range: 200 ~ 400 clock cycle  */
	  0,					/* Force GuardTime. This allows the support of 0 ETU extra guard time of characters sent in
					        the same direction, while providing 15 ETU extra guard time during a change in the direction of the data transfer. */
	  0,					/* clock rate, now only 2.5MHz  */
	  (1<<0)&(1<<1),		/* Bitfield of valid protocols from ATR*/
	  4,					/* BWI  */
	  10,					/* WI  */
	  0,					/* PTS0:   Do not send PTS1/2/3, use default FI&DI */
	  0x11, 				/* PTS1	  FI=1, DI=1*/
	  0x01, 				/* PTS2	  support N=255 */
	  0, 					/* PTS3 */
	  24,                             /* N */
	  1,                             /* P */
	  1                              /* B */
};

struct class * elite_sc_class=NULL;

/**
* elite_sci_timer_handler - the timeout handle for kernel timer
* @arg : the arg for timer handler 
*/
void elite_sci_timer_handler(unsigned long arg);

static void sc_regs_dump(struct elite_sci *s)
{
	SC_PRINT("SCTMA read 0x%x\n",sc_readl(s,SCTMA));
	SC_PRINT("SCRMA read 0x%x\n",sc_readl(s,SCRMA));
	SC_PRINT("SCTMS read 0x%x\n",sc_readw(s,SCTMS));
	SC_PRINT("SCS read 0x%x\n",sc_readw(&s,SCS));
	SC_PRINT("SCCMD read 0x%x\n",sc_readw(s,SCCMD));
	SC_PRINT("SCRMS read 0x%x\n",sc_readw(s,SCRMS));
	SC_PRINT("SCETU read 0x%x\n",sc_readw(s,SCETU));
	SC_PRINT("SCCWI read 0x%x\n",sc_readw(s,SCCWI));
	SC_PRINT("SCCD read 0x%x\n",sc_readw(s,SCCD));
	SC_PRINT("SCGT read 0x%x\n",sc_readw(s,SCGT));
	SC_PRINT("SCIE read 0x%x\n",sc_readw(s,SCIE));
	SC_PRINT("SCRA read 0x%x\n",sc_readw(s,SCRA));
	SC_PRINT("SCES read 0x%x\n",sc_readw(s,SCES));
	SC_PRINT("SCFC read 0x%x\n",sc_readw(s,SCFC));
	SC_PRINT("SCBC read 0x%x\n",sc_readw(s,SCBC));
	SC_PRINT("SCPC read 0x%x\n",sc_readw(s,SCPC));
	SC_PRINT("SCNC read 0x%x\n",sc_readw(s,SCNC));
	SC_PRINT("SCIS read 0x%x\n",sc_readw(s,SCIS));
	SC_PRINT("SCIM read 0x%x\n",sc_readw(s,SCIM));
	SC_PRINT("SCD_T14 read 0x%x\n",sc_readw(s,SCD_T14));
} 

/**
 * elite_sci_set_hwconf_tohw - config the registers according to the hareware configuration
 * @s : the elite smart card device 
 *
 */
static void elite_sci_set_hwconf_tohw (struct elite_sci *s)
{

	u16 temp_data=0;
	ENTER();

	if (sc9600_current_config.inverse_reset)
		/*inverse reset*/
		sc_writew(s,sc_readw(s,SCCMD)|SC_SCCMND_IRS,SCCMD);
	else
		/*normal reset*/
		sc_writew(s,sc_readw(s,SCCMD)&(~SC_SCCMND_IRS),SCCMD);

	if (sc9600_current_config.two_crc)
		/*two byte CRC*/
		sc_writew(s,sc_readw(s,SCCMD)|SC_SCCMND_CRC,SCCMD); 
	else
		/*one byte CRC*/
		sc_writew(s,sc_readw(s,SCCMD)&(~SC_SCCMND_CRC),SCCMD);

	if (sc9600_current_config.protocol_type)
		/*PT=1*/
		sc_writew(s,sc_readw(s,SCCMD)|SC_SCCMND_PT,SCCMD); 
	else
		/*PT=0*/
		sc_writew(s,sc_readw(s,SCCMD)&(~SC_SCCMND_PT),SCCMD);  


	/*set CWI in SCCWI*/
	sc_writew(s,(sc9600_current_config.cwi)&SC_SCCWI_CWI,SCCWI);  

	/*set SCETU*/
	if (sc9600_current_config.auto_convention)
		/*autodetection*/
		sc_writew(s,sc_readw(s,SCETU)&(~SC_SCETU_CP),SCETU);
	else
		/*Prevent the autodetection*/
		sc_writew(s,sc_readw(s,SCETU)|SC_SCETU_CP,SCETU);  

	if (sc9600_current_config.inverse_convention)
		/*inverse convention*/
		sc_writew(s,sc_readw(s,SCETU)|SC_SCETU_C,SCETU);	
	else
		/*direct convention*/
		sc_writew(s,sc_readw(s,SCETU)&(~SC_SCETU_C),SCETU);

	if (sc9600_current_config.auto_etu)
		/*autodetection*/
		sc_writew(s,sc_readw(s,SCETU)&(~SC_SCETU_EP),SCETU);
	else
		/*Prevent the autodetection*/
		sc_writew(s,sc_readw(s,SCETU)|SC_SCETU_EP,SCETU);  

	/*clear ETU to 0 first*/
	sc_writew(s,(~SC_SCETU_ETU),SCETU);

	if (sc9600_current_config.etu)
		sc_writew(s,(sc9600_current_config.etu-1)&SC_SCETU_ETU,SCETU);
	else
		sc_writew(s,sc_readw(s,SCETU)|((INITIAL_F_VALUE-1)&SC_SCETU_ETU),SCETU); 

	/*set SCGT*/
	if (sc9600_current_config.guard_time)
		sc_writew(s,(sc9600_current_config.guard_time-1),SCGT); 
	else
		sc_writew(s,0,SCGT); 

	/*set SCRA*/
	temp_data=COMPUTE_SCRA(s->freq_rsaclk, CLK_DIV_TDA8004, sc9600_current_config.reset_time);
	sc_writew(s,temp_data,SCRA); 

	/*set SCIE*/
	sc_writew(s,temp_data+sc9600_current_config.atr_response_time,SCIE); 

	/*set SCES*/
	if (sc9600_current_config.force_guard_time)
		/*force guardtime*/
		sc_writew(s,sc_readw(s,SCES)|SC_SCES_FG,SCES);	   
	else
		/*do not force guardtime*/
		sc_writew(s,sc_readw(s,SCES)&(~SC_SCES_FG),SCES);

	LEAVE();
}


/**
 * elite_sci_hw_reset - reset the hardware
 * @s : the elite smart card device 
 * @level: the reset type ,include SC_COLD_RESET,SC_WARM_RESET,SC_PIPELINE_RESET
 *
 */
static int elite_sci_hw_reset(struct elite_sci *s, sc_reset_level_type level)
{	
	ENTER();
	SC_PRINT("%s with level=%d enter\n",__func__,level);

    /* Switch on Reset type */
	switch (level)
	{
		/*COLD RESET*/
		case(SC_COLD_RESET):
		{
			
			sc9600_current_config = sc9600_atr_config;		
			elite_sci_set_hwconf_tohw(s);	
			
			s->rst_val = 0;
			s->nr_trigg = 0;
			s->atr_icd_count = 0;
			s->atr_nr_count = 0;
			s->atr_long_count = 0;
			memset(s->rx_buffer, 0x00, SC_BUFFER_SIZE);
			memset(s->tx_buffer, 0x00, SC_BUFFER_SIZE);
  
			/* start answer to reset*/
			sc_writew(s,  SC_SCCMND_SAR|sc_readw(s,SCCMD), SCCMD); 
          
			/* clear interrupt status */
			sc_writew(s, 0xffff,SCIS); 
			/* Set the appropriate IMR bits for ATR */
			/* masking interrupts */
			sc_writew(s, 0x0,SCIM); 

			/* enable Force Guard time */
			sc_writew(s,sc_readw(s,SCES)|SC_SCES_FG,SCES); 
			/* unmasking interrupts */
			sc_writew(s, 0xFFFF,SCIM); 

			/* Setting memory address in register for Rx buffer */
			s->rx_size = SC_BUFFER_SIZE;
			sc_writel(s,  (u32)s->rx_buffer_phy,SCRMA);

			sc_writel(s,  0, SCTMA);
			sc_writew(s,  0, SCTMS);
			sc_writew(s,  0, SCRMS);


			sc_writew(s,  0, SCS);
			if(sc9600_current_config.protocol_type==SC_PROTOCOL_TYPE_14)
			{
				sc_writew(s,  0x1, SCD_T14);
				sc_writew(s,((~SC_INT_ISR_PARITY)&sc_readw(s,SCIM)),SCIM);
				SC_PRINT(" enter T14 branch!\n");	
			}
			else
			{
				sc_writew(s,  0, SCD_T14);
				SC_PRINT(" enter T0 branch!\n");
			}
			s->t_sample = COMPUTE_US_PER_CLKCYCLES(ATR_NR_SAMPLE, 0, s->freq_rsaclk, CLK_DIV_TDA8004);
			SC_PRINT("t_sample:%d us  and HZ:%d \n",s->t_sample,HZ);

			 /* RST and VCC VCC go LOW*/
			/* Manual says that RST bit has no effect, if RL bit ==0 */
			sc_writew(s,sc_readw(s,SCCMD)|SC_SCCMND_SMR,SCCMD);
			sc_writew(s,sc_readw(s,SCCMD)&(~SC_SCCMND_RST),SCCMD);
			sc_writew(s,sc_readw(s,SCCMD)&(~SC_SCCMND_COE),SCCMD);
			sc_writew(s,sc_readw(s,SCCMD)&(~SC_SCCMND_VCC),SCCMD);
			mdelay(10);
           
			sc_writew(s,sc_readw(s,SCCMD)&(~SC_SCCMND_SMR),SCCMD);
			sc_writew(s,sc_readw(s,SCCMD)|SC_SCCMND_VCC,SCCMD);
			sc_writew(s,sc_readw(s,SCCMD)|SC_SCCMND_COE,SCCMD);

			mdelay(1);

			/* Start NR timer */
			s->timer.data=(unsigned long)s;
			s->timer.function=elite_sci_timer_handler;
			s->timer.expires=jiffies+usecs_to_jiffies(s->t_sample);
			SC_PRINT("t_sample to jiffies:%d\n",usecs_to_jiffies(s->t_sample));
			del_timer(&s->timer);
			s->need_timer=1;
			add_timer(&s->timer);
			
			sc_writew(s,sc_readw(s,SCCMD)|SC_SCCMND_RST,SCCMD);
			sc_regs_dump(s);

			break;
		}

		/* WARM RESET:
		* According to the EMV specs [I-13], the only thing that I shall do
		* is to set the RESET line Low and then again HIGH after 400-40,000 clkcycles.
		*  - CLK and Vcc shall be HIGH during this period
		*  - I/O line shall be HIGH again after 200clkcycles from the time RST
		* has gone LOW: before it is "INDETERMINATE".
		* The only way to ask a WARM RESET to the card, anyway, is to call
		* again the reset through the bits in the commmand register. It does
		* not work simply setting RST LOW and then HIGH. I mean, the card goes
		* for a WARMRESET, but the AViA is not expecting bytes...
		*/
		case(SC_WARM_RESET):
		{
			sc9600_current_config = sc9600_atr_config;
			elite_sci_set_hwconf_tohw (s);
			
			s->rst_val = 0;
			s->nr_trigg = 0;
			s->atr_icd_count = 0;
			s->atr_nr_count = 0;
			s->atr_long_count = 0;
			
			memset(s->rx_buffer, 0x00, SC_BUFFER_SIZE);
			memset(s->tx_buffer, 0x00, SC_BUFFER_SIZE);  
            
			/* start answer to reset*/
			sc_writew(s,  SC_SCCMND_SAR|sc_readw(s,SCCMD), SCCMD); 

			/* masking interrupts */	
			sc_writew(s, 0x0,SCIM); 
			/* clear bits */	
			sc_writew(s, 0xFFFF,SCIS); 
			/* unmasking interrupts */
			sc_writew(s, 0xFFFF,SCIM); 
			if(sc9600_current_config.protocol_type==SC_PROTOCOL_TYPE_14)
			{
				sc_writew(s,  0x1, SCD_T14);
				sc_writew(s,((~SC_INT_ISR_PARITY)&sc_readw(s,SCIM)),SCIM);
				SC_PRINT(" enter T14 branch!\n");			
			}
			else
			{
				sc_writew(s,  0, SCD_T14);
				SC_PRINT(" enter T0 branch!\n");
			}
			sc_writew(s, SC_SCES_FG,SCES);
			/* Unlock the reset bit */
			sc_writew(s,sc_readw(s,SCCMD)&(~SC_SCCMND_RL),SCCMD);

			s->rx_size = SC_BUFFER_SIZE;
			sc_writel(s,  (u32)s->rx_buffer_phy,SCRMA);

			/* RST goes HIGH*/
			sc_writew(s,sc_readw(s,SCCMD)|SC_SCCMND_RL,SCCMD);
			sc_writew(s,sc_readw(s,SCCMD)|SC_SCCMND_RST,SCCMD);

			/* RST goes LOW*/
			/* Manual says that RST bit has no effect, if RL bit ==0 */
			sc_writew(s,sc_readw(s,SCCMD)&(~(SC_SCCMND_RST)),SCCMD);

			mdelay(1);
			s->timer.data=(unsigned long)s;
			s->timer.function=elite_sci_timer_handler;
			s->timer.expires=jiffies+usecs_to_jiffies(s->t_sample);
			del_timer(&s->timer);
			s->need_timer=1;
			add_timer(&s->timer);

			sc_writew(s,sc_readw(s,SCCMD)|SC_SCCMND_RST,SCCMD);
			/* enable CLK*/
			sc_writew(s,sc_readw(s,SCCMD)|SC_SCCMND_COE,SCCMD);
			/* Start NR timer */
			SC_PRINT("t_sample:%d us  and HZ:%d \n",s->t_sample,HZ);
			
			/* I have to go through a reset of the pipeline */
			break;
		}

		/*	PIPELINE RESET*/
		case(SC_PIPELINE_RESET):
		{
			/* NOTE: Do not change Level or State for a pipeline reset */
			sc_writew(s,sc_readw(s,SCCMD)|SC_SCCMND_SMR,SCCMD);
			mdelay(20);
			sc_writew(s,sc_readw(s,SCCMD)&(~SC_SCCMND_SMR),SCCMD);
			break;
		}

		default:
		{
			LEAVE();
			return -EINVAL;
		}
	}
	
	LEAVE();
	return 0;
}

/**
 * elite_sci_hw_init - init the hardware
 * @s : the elite smart card device 
 *
 */
static int elite_sci_hw_init(struct elite_sci *s)
{

	u16 rst_act_cnt=0;
	u16 in_en_cnt=0;
	u16 N= 0;
	u16 P=0;
	u16 B=0;
	u16 ETU=0;
	ENTER();

	N= sc9600_base_config.n_value;
	P=sc9600_base_config.p_value;
	B=sc9600_base_config.b_value;
	ETU=sc9600_base_config.etu;
	/* Clear all interrupt status values after reset */
	sc_writew(s, 0xFFFF,SCIS); 
	sc_writew(s, P, SCPC); 
	sc_writew(s, N, SCNC);
	sc_writew(s, B, SCBC);
	SC_PRINT("N:0x%x,P:0x%x,B:0x%x,ETU:0X%x\n",N,P,B,ETU);
	s->freq_rsaclk =(B*24000)/((2*N*(B-P))+(2*(N+1)*P)) ;// 4 Mhz or 6MHZ
	SC_PRINT("freq_rsaclk:0x%x\n",s->freq_rsaclk );
	s->chip_type =SC_IO_TDA8034;
	if (s->chip_type==SC_IO_TDA8004)
	{
		/* the init freq of rsaclk is all 3MHZ(6000K/2) */
		rst_act_cnt = COMPUTE_SCRA(s->freq_rsaclk, CLK_DIV_TDA8004, RST_ACT_CNT_TDA8004);
		in_en_cnt = rst_act_cnt + IN_EN_CNT_TDA8004;
		sc_writew(s, (SC_SCCD_EXT | CL_SC_SCCD_CLK_DIV_TDA8004)&SCCD_VALUE_MASK, SCCD);
		sc_writew(s, rst_act_cnt, SCRA);
		sc_writew(s,  in_en_cnt, SCIE);
		sc_writew(s,  (SC_SCCMND_IE | SC_SCCMND_VCC | SC_SCCMND_COE)|sc_readw(s,SCCMD), SCCMD);
	}
	else
	{
		sc_writew(s, 0x1f&SCCD_VALUE_MASK, SCCD);  
		sc_writew(s, SC_SCCMND_VCC, SCCMD); 
		sc_writew(s, (SC_SCCMND_IE | SC_SCCMND_RL | SC_SCCMND_COE)|sc_readw(s,SCCMD), SCCMD); 
	}
	/* Computing microseconds from clockcycles and etus for time-out events. 
	* The t_sample is the period (in us) that is used by the timer to generate interrupts. 
	*/
	/* Period for no reply interrupt: the HW is triggering a NR interrupt at 37240 clk cycles.
	* I've to check then the RD bit to see if I got a byte 
	* (it goes from 1->0 when the whole byte has been received).
	* The ORGA is sending one after 40,000 clkclyes: 
	* I allow 4000 clockcycles since then towait for the first byte. 
	*/
	s->t_sample = COMPUTE_US_PER_CLKCYCLES(ATR_NR_SAMPLE, 0, s->freq_rsaclk, CLK_DIV_TDA8004);

	/* No Reply parameter */
	s->atr_nr_max = ATR_NR_INTERVAL/ATR_NR_SAMPLE;

	/* Setting ICD parameter */
	s->atr_icd_max = COMPUTE_US_PER_ETUS(ATR_ICD_MAX, 0, ETU, s->freq_rsaclk, CLK_DIV_TDA8004);
	s->atr_icd_max = (s->atr_icd_max) / (s->t_sample);

	if((s->atr_icd_max) % (s->t_sample))
	{
		s->atr_icd_max++;
	}

	//SC_PRINT("sys HZ:%d \n",HZ);
	printk(KERN_NOTICE"sys HZ:%d \n",HZ);
	s->ms_per_tick = 1000 / HZ; /* number of us in one tick*/
	if(1000 % HZ)
	{
		s->ms_per_tick++;
	}
	
	/* Setting long ATR parameter */
	s->atr_long_max =  COMPUTE_US_PER_ETUS(ATR_TIME_MAX, 0, ETU, s->freq_rsaclk, CLK_DIV_TDA8004);
	s->atr_long_max = (s->atr_long_max) / (s->t_sample);

	if((s->atr_long_max) % (s->t_sample))
	{
		s->atr_long_max++;
	}
	mdelay(10);
	sc9600_atr_config = sc9600_base_config;
	//SC_PRINT("t_sample:%d ,atr_nr_max:%d,atr_icd_max:%d,ms_per_tick:%d,atr_long_max:%d",s->t_sample,s->atr_nr_max,s->atr_icd_max,s->ms_per_tick,s->atr_long_max);
	printk(KERN_NOTICE"t_sample:%d ,atr_nr_max:%d,atr_icd_max:%d,ms_per_tick:%d,atr_long_max:%d",s->t_sample,s->atr_nr_max,s->atr_icd_max,s->ms_per_tick,s->atr_long_max);
	LEAVE();
	return 0;
}



/**
 * elite_sci_hw_detect - detect whether or not have card present
 * @s : the elite smart card device 
 *
 */
static int elite_sci_hw_detect(struct elite_sci *s)
{
	u32 status=0;
	ENTER();
	/* Use the Cards OFFN level to detect card presence */
	status = sc_readl(s, SCCMD);

	if (status & SC_SCCMND_OFF)
	{
		LEAVE();
		return 1;
	}
	else
	{
		LEAVE();
		return 0;
	}
}

/**
 * elite_sci_hw_detect_enable - it enables the CD bit
 * @s : the elite smart card device 
 *
 */
static int	elite_sci_hw_detect_enable(struct elite_sci *s)
{

	ENTER();

	/* clear bits */	
	sc_writew(s, (sc_readw(s,SCIS)&(SC_INT_ISR_OFF)),SCIS); 
	/* unmasking interrupts */
	sc_writew(s, (sc_readw(s,SCIM)&((SC_INT_ISR_OFF|SC_INT_ISR_MESS))),SCIM); 

	LEAVE();
	return 0;

}


/**
 * elite_sci_hw_detect - deactive the smart card
 * @s : the elite smart card device 
 *
 */
static int elite_sci_hw_deactive(struct elite_sci *s)
{

	ENTER();
	/* masking interrupts */ 
	sc_writew(s, 0x0,SCIM);	  

	/*Disable everything to card, 1st 
	* Setting RL to 1 forces the external-RST signal to follow
	* the reset bit (RST) of this register: RST goes LOW 
	*/
	sc_writew(s,SC_SCCMND_RL | sc_readw(s,SCCMD), SCCMD);
	sc_writew(s,(~SC_SCCMND_RST) & sc_readw(s,SCCMD), SCCMD);

	/* Clock LOW */
	sc_writew(s,(~SC_SCCMND_COE) & sc_readw(s,SCCMD), SCCMD);

	/* I/0 LOW */
	sc_writew(s,(~SC_SCCMND_IE) & sc_readw(s,SCCMD), SCCMD);

	/* Vcc LOW */
	sc_writew(s,(~SC_SCCMND_VCC) & sc_readw(s,SCCMD), SCCMD);

	/* TCK error: SAR low*/
	sc_writew(s,(~SC_SCCMND_SAR) & sc_readw(s,SCCMD), SCCMD);
	sc_writew(s, SC_SCCMND_SMR| sc_readw(s,SCCMD), SCCMD);

	mdelay(2);
	sc_writew(s,( ~SC_SCCMND_SMR)& sc_readw(s,SCCMD), SCCMD);
	
	/* unmask interrupts */ 
	sc_writew(s,((SC_INT_ISR_OFF | SC_INT_ISR_MESS))| sc_readw(s,SCIM),SCIM);	

	LEAVE();
	return 0;
}

/**
 * elite_sci_hw_terminate - terminate the hardware
 * @s : the elite smart card device 
 *
 */
static int elite_sci_hw_terminate(struct elite_sci *s)
{
	int ret=0;
	ENTER();
	ret=elite_sci_hw_deactive(s);
	LEAVE();
	return ret;
}

/**
* elite_sci_hw_io_command - send the data in buufer and expect the response
* @s : the elite smart card device 
* @buffer : the send data buffer 
* @size : the send data size 
* @reply_size : the expect data size  of response 
*/
static int elite_sci_hw_io_command(struct elite_sci *s,  u8 *buffer,u16 size,u16 reply_size)
{

	ENTER();
	/* Set the appropriate IMR bits for IO:
	*  IA is for Illegal Acknowledge: a procedure with a value other than 6x or 9x
	*/
	sc_writew(s,  ( ~SC_SCES_IA)& sc_readw(s,SCES), SCES);
	
	/* FIXME: Spurious interrupt on 1221-100, IA fired */
	/* clear ISR bits*/
	sc_writew(s, 0xffff,SCIS); 

	memset(s->rx_buffer, 0x00, SC_BUFFER_SIZE); 

	/*unmask interrupts*/
	sc_writew(s, 0xffff,SCIM);  
	if(sc9600_current_config.protocol_type==SC_PROTOCOL_TYPE_14)
	{
		sc_writew(s,((~SC_INT_ISR_PARITY)&sc_readw(s,SCIM)),SCIM);
	}
	memcpy(s->tx_buffer, buffer, size);
	s->tx_size = size;

	/* [T=0 protocol] the first time I send a message it's 0
	*since ICC shall return only status bytes
	*that indicate the real number of bytes
	*that are going to be sent
	*/
	s->rx_size = reply_size;  

	sc_writel(s, (u32)s->tx_buffer_phy,SCTMA);	
	sc_writew(s, s->tx_size,SCTMS);  
	sc_writel(s, (u32)s->rx_buffer_phy,SCRMA); 
	
	if(s->rx_size ==0)
	{
		sc_writew(s,0,SCRMS); 
	}
	else
	{
		sc_writew(s,s->rx_size-1,SCRMS); 
	}
	
	/*keeps track of a PE event*/
	s->parity_error = 0; 
	sc_writew(s,sc_readw(s,SCCMD)|SC_SCCMND_SMR,SCCMD);
	/* takes care of allowing at least 16etus when exchanging data*/
	mdelay(2); 
	sc_writew(s,sc_readw(s,SCCMD)&(~SC_SCCMND_SMR),SCCMD);

	/* case T=0
	*in SCRMS:
	*- Write only the total number of bytes in data, not procedure/status bytes.
	*- Write the total number of bytes expected minus 1.
	*- It keeps track of the number of bytes remaining in the message
	*/
	if ((s->protocol == SC_PROTOCOL_TYPE_0)||(s->protocol == SC_PROTOCOL_TYPE_14))
	{
 
		if(s->rx_size==0)
		{
			SC_PRINT("enter  SC_PROTOCOL_TYPE_0 with rx size is zero! \n"); 	       
			/*no data from card*/
			//sc_writew(s,  (~SC_SCCMND_IE) & sc_readw(s,SCCMD), SCCMD);
			sc_writew(s,  (~SC_SCCMND_RD) & sc_readw(s,SCCMD), SCCMD);
			sc_writew(s, SC_SCCMND_TE|SC_SCCMND_RE |sc_readw(s,SCCMD), SCCMD);
		}
		else 
		{
			SC_PRINT("enter  SC_PROTOCOL_TYPE_0 with rx size isn't zero! \n"); 	       
			 /*data from card*/
			sc_writew(s, s->rx_size-1, SCRMS);
			sc_writew(s, (SC_SCCMND_TE | SC_SCCMND_RE |SC_SCCMND_RD)| sc_readw(s,SCCMD), SCCMD);
		}
            
	}
	
	LEAVE();
	return 0;
}

/**
* eliste_sci_cardconf - configure the hardware according to the atr info after reset
* @s : the elite smart card device 
* @atr_info : the atr information from the smart card 
*/
static int	eliste_sci_cardconf(struct elite_sci *s, sc_atr_info_type *atr_info)
{

	/*0 => direct convention , 1 => inverse convention*/
	u16   convention=0; 
	ENTER();
	if (atr_info->fi > 1)	 
	{
		/*modify clock*/
		printk(KERN_NOTICE "atr's fi eque %d above one\n",	atr_info->fi );
	}

	convention = (atr_info->ts == SC_INVERSE_CONVENTION)? SC_SCETU_C : 0;
	/*compute timer new period*/
	s->t_sample = COMPUTE_US_PER_ETUS(ATR_ICD_MIN, 0, atr_info->etu, s->freq_rsaclk, CLK_DIV_TDA8004);

	/* I'm subtracting one for the ETU register requirement */
	sc_writew(s, (((atr_info->etu) - 1) | convention), SCETU);

	/* Sorry for the switch, but it's the only way I've found:
	* HW is not handling the CWT correctly: I'm failing the tests.
	* As you can see the value for the card to be
	* active/deactivated are pretty the same. I've found out
	* that with these values I'm passing both set of tests and
	* I'm using them. [GB] 
	*/
	if (atr_info->mode == SC_PROTOCOL_TYPE_1)
	{
		u32 cwt;
		
		/*cwt in etus [0->43]*/
		cwt = (1 << (atr_info->cwi)) + 11; 

		/* compute CWT in us */
		cwt = (COMPUTE_US_PER_ETUS(cwt, 0, atr_info->etu, s->freq_rsaclk, CLK_DIV_TDA8004));
		
		/* length in ms for a block: IFSC has the size of the INF field, 
		*  I add the following4 mandatory bytes. [GB] 
		*/
		s->block_length = ((atr_info->ifsc + 4) * cwt) /1000;
		/* value in tick */
		s->block_length = s->block_length / s->ms_per_tick;

		if(s->block_length % s->ms_per_tick)
		{
			s->block_length++;
		}

		switch(atr_info->cwi)
		{
			case(1): 
			{ 
		        	/*ok @13etus - deactivate @15etus */
				s->t_sample = COMPUTE_US_PER_ETUS(7, 0,
				atr_info->etu, s->freq_rsaclk, CLK_DIV_TDA8004);
				s->cwt_max = 2;
				break;
			}

			case(2): 
			{
		        	/*ok @15etus - deactivate @17etus*/
				s->t_sample = COMPUTE_US_PER_ETUS(4, 0,
				atr_info->etu, s->freq_rsaclk, CLK_DIV_TDA8004);
				s->cwt_max = 4;
				break;
			}

			case(3): 
			{
		       	 /*ok @19etus - deactivate @21etus*/
				s->t_sample = COMPUTE_US_PER_ETUS(5, 0,
				atr_info->etu, s->freq_rsaclk, CLK_DIV_TDA8004);
				s->cwt_max = 4;
				break;
			}

			case(4): 
			{
		        	/*ok @27etus - deactivate @30etus*/
				s->t_sample = COMPUTE_US_PER_ETUS(7, 0,
				atr_info->etu, s->freq_rsaclk, CLK_DIV_TDA8004);
				s->cwt_max = 4;
				break;
			}

			case(5): 
			{
		        	/*ok @43etus - deactivate @48etus*/
				s->t_sample = COMPUTE_US_PER_ETUS(4, 0,
				atr_info->etu, s->freq_rsaclk, CLK_DIV_TDA8004);
				s->cwt_max = 11;
				break;
			}

			default:
			{
				if(sc9600_current_config.standard == SC_STANDARD_ISO7816)
				{
					s->t_sample = COMPUTE_US_PER_ETUS(40, 0,
										   atr_info->etu, s->freq_rsaclk, CLK_DIV_TDA8004);
					s->cwt_max=((1<<atr_info->cwi)/40);
					if ((1<<atr_info->cwi)%40) 
						s->cwt_max++;

				}
				else
				{
					/* Anyway, I should not fall in this case, since I'm checking the
					* value passed by the ATR before entering here. 
					*/
					printk(KERN_ERR "atr'scwi is invalid\n" );
					return -EINVAL;
				}
				break;
			}
		}

		sc_writew(s, 15, SCCWI);
		sc_writew(s,  SC_SCCMND_PT| sc_readw(s,SCCMD), SCCMD);
		s->bgt = T1_BGT_TIME * (atr_info->etu) / ((s->freq_rsaclk)/ (1+CLK_DIV_TDA8004)); //ms
		s->bgt = (s->bgt) / (s->ms_per_tick); /* number of ticks*/
		if((s->bgt) < 2 ) 
			 s->bgt = 2;

		/*
		* if (N == 0xFF), the value atrInfo->N == -1
		* so that, since +1 is added to the value in the register, we finally have:
		* - 12 etus for T=0
		* - 11 etus for T=1
		*[min character duration for subsequent Tx]
		*/
		if(atr_info->n == 0xFF)
		{
			sc_writew(s, 0, SCGT);	
		}
		else
		{
			sc_writew(s, (atr_info->n+1), SCGT); 
		}

	} // end of config for T=1!

	else
	{
		/*EMV96: The turn around ICD time for Rx<->Rx is 12 etu and
		*  for Tx<->Rx is 16 etu. 11 etu is spent for the
		*  character frame which means we need 1 more for
		*  Rx<->Rx and 5 more for Tx<->Rx. The 9600 has
		*  this feature with SCGT and SCES settings 
		*/
		if(atr_info->n == 0xFF)
		{
			atr_info->n = 0;
			sc_writew(s, 1, SCGT);	
			sc_writew(s,SC_SCES_FG, SCES);	
		}
		else
		{
			if (atr_info->n>5)
			{
				sc_writew(s, atr_info->n, SCGT); 
				sc_writew(s,  (~SC_SCES_FG)& sc_readw(s,SCES), SCES);
			}
			else
			{
				sc_writew(s, (atr_info->n + 5), SCGT);	 
				sc_writew(s, SC_SCES_FG, SCES); 
			}
		}

		{
			u32 tmp_d;
			tmp_d=960 * (atr_info->wi);
			switch (atr_info->di)
			{
				case 2:  tmp_d*=2; break;
				case 3:  tmp_d*=4; break;
				case 4:  tmp_d*=8; break;
				case 5:  tmp_d*=16; break;
				case 6:  tmp_d*=32; break;
				case 8:  tmp_d*=12; break;
				case 9:  tmp_d*=20; break;
				case 10: tmp_d/=2; break;
				case 11: tmp_d/=4; break;
				case 12: tmp_d/=8; break;
				case 13: tmp_d/=16; break;
				case 14: tmp_d/=32; break;
				case 15: tmp_d/=64; break;
				default: break;
			}
			s->wwt = COMPUTE_US_PER_ETUS(tmp_d, 0, (atr_info->etu),s->freq_rsaclk, CLK_DIV_TDA8004)
					 +COMPUTE_US_PER_ETUS((tmp_d * 3) / 100, 0, (atr_info->etu),s->freq_rsaclk, CLK_DIV_TDA8004); // +3% tolerance
		}
		sc_writew(s,  (~SC_SCCMND_PT)& sc_readw(s,SCCMD), SCCMD);
	}

	LEAVE();
	return 0;
}

/**
* elite_sci_set_userspec_atr - configure the hardware according to user spec hardware configuration
* @s : the elite smart card device	
*/
void elite_sci_set_userspec_atr (struct elite_sci *s)
{
	ENTER();
	/*modify ATR parameters*/
	s->atr_info.etu=sc9600_current_config.etu;
	s->atr_info.mode=sc9600_current_config.protocol_type;
	s->atr_info.cwi=sc9600_current_config.cwi;
	s->atr_info.bwi=sc9600_current_config.bwi;
	s->atr_info.n=sc9600_current_config.guard_time-2;
	s->atr_info.wi=sc9600_current_config.wi;

	/*setting protocol*/
	s->protocol = s->atr_info.mode;  

	/*call ScHwCardConfig to let the new ATR parameters take effect*/
	eliste_sci_cardconf (s, &s->atr_info);

	/*modify other HW settings*/
	elite_sci_set_hwconf_tohw(s);
	LEAVE();
}


/**
* elite_sci_process_atr -  get the atr info from the buffer
* @s : the elite smart card device	
* @buffer : smart card atr response message buffer	
*/
int  elite_sci_process_atr(struct elite_sci *s, u8 *buffer)
{

	sc_atr_info_type	*atr_info = &(s->atr_info);
	u8	   index= 0;
	u8	   bitmask_yx = 0;
	u8	   interface_x = 1;
	u8	   mode_t = 0;
	u8	   tax, tbx, tcx, tdx;
	int 	   err = 0;
	u32    atr_end_data;
	u16    atr_status;
	u8	   i=0;

	/* NOTE: the processing of the ATR is very cryptic.
	* I will try to explain things as I go along. The naming convention
	* of TA1, TB1, etc is from the smartcard standard ISO/IEC 7816-3,
	* so don't blame me. OK, here we go 
	*/

	ENTER();
	/* Initialize Defaults for ATR Information */
	atr_info->length = 0;
	atr_info->ts = SC_DIRECT_CONVENTION;
	atr_info->fi = 1;
	atr_info->di = 1;
	atr_info->etu = sc9600_current_config.etu;
	atr_info->tb1 = FALSE; /*for a cold reset it must be present and equals to 0x00*/
	atr_info->ii = 0;
	atr_info->pi1 = 0;
	atr_info->n = 0;	       /*value of TC1: if not present in ATR shall be 0*/
	atr_info->tb2 = FALSE; /* shall reject ATR if TB2 is present*/
	atr_info->pi2 = 0;
	atr_info->mode_parameters = FALSE;
	atr_info->wi = 10;				 /*Work Waiting Time integer [in TC2]: default value if TC2 is not present*/
	atr_info->specific_mode = FALSE; /* if TA2 is missing: nogotiable mode*/
	atr_info->mode = 0; 			 /* by default, if TD1 is absent*/
	atr_info->mode_changable = 0;
	atr_info->ifsc =  0x20; /* value received with TA3: this is the default value if TA3 is not present.*/
	atr_info->td2 = FALSE;
	atr_info->tb3 = FALSE;	/* reject ATR if not present*/
	atr_info->cwi = 0;
	atr_info->bwi = 0;
	atr_info->rc = 0;		/*TC3: block error detection code, reject ATR if != 0x00*/
	atr_info->history_size = 0;
	atr_info->history_ptr = NULL;

	SC_PRINT("ScProcessATR \n");

	for(i=0; i < 32; i++) 
	{
		atr_info->buffer[i] = buffer[i];
	}

	/* AViA silicon problem!
	*If the ATR is odd in length and does not have history bytes, the last bit is stuck in
	* the upper byte of the SCS register: so I'm getting it and putting it at the end of the
	*buffer used to parse the ATR.
	*I'm doing it in this way:
	* - I'm checking the final value in the SCRMA register: the value is incremented
	*every time two bytes are received. In case of an odd number of bytes received,
	*it has the last even number that has been updated (e.g., if I'm getting 5 bytes, SCRMA
	*has the initial value + 4)
	*- I know the initial address.
	*So if I subtract the initial value to the value got from SCRMA, I finally get the index
	*where to copy the byte stuck in the SCS register. [GB]
	*/
	atr_end_data = (u32) sc_readl(s,SCRMA);
	SC_PRINT("atr_end_data is 0x%x \n",atr_end_data);
	atr_end_data = atr_end_data - ((u32)s->rx_buffer_phy);
	SC_PRINT("atr_end_data - rx_buffer_phy is 0x%x \n",atr_end_data);
	if(atr_end_data > 2)
	{
		atr_status = sc_readw(s,SCS) ;
		atr_status >>= 8;
		atr_info->buffer[atr_end_data] = atr_status;
	}
	else /*  */
	{
		atr_status =  sc_readw(s,SCS);
		atr_status >>= 8;
		atr_info->buffer[2] = atr_status;
	}

	/* The initial character in the ATR response is called TS
	*TS provides a bit synchronization sequence and a data convention,
	*basically to the software this means two values are valid and
	*hardware handles the reset
	*/
	atr_info->ts = atr_info->buffer[index++];

	/* let's double-check */
	if( ((atr_info->ts) != SC_DIRECT_CONVENTION) && ((atr_info->ts) != SC_INVERSE_CONVENTION) )
	{
		/* If it's wrong: deactivate the card, but here it might be too late! */
		printk(KERN_ERR"atr_info's ts=%d is invalid\n",atr_info->ts);
		LEAVE();
		return -EINVAL ;
	}

	/* The next character is the Format Character T0
	*T0 contains the first Yx bitmask for the next group of data
	*and K the historical extra bits at the end of the message
	*/
	bitmask_yx = atr_info->buffer[index++];
	if(!bitmask_yx)
	{
		printk(KERN_ERR"bitmask_yx=%d is invalid\n",bitmask_yx);
		LEAVE();
		return -EINVAL ;
	}

	atr_info->history_size = bitmask_yx &0x0f ;
	atr_info->t0 = bitmask_yx &0xF0;

	/* Now we will walk through the series of TAx, TBx, TCx, TDx's
	*until all have been processed, Initial modeT = 0.
	*Initial interfaceX = 1.
	*EMV specifies meaning of byte until TC3, so I'll just keep the
	*value of the other bytes (TA4, TB4...) for now. 
	*/
	while (bitmask_yx)
	{
		if (bitmask_yx & 0x10)
		{	  
			SC_PRINT(" enter 0x10 branch\n");

			/* TAx present */
			tax = atr_info->buffer[index++];
			if(interface_x == 1)
			{
				/* TA1 codes FI over the most significant half byte */
				atr_info->fi = (tax & 0xF0) >> 4;

				/* TA1 codes DI over the least significant half */
				atr_info->di = (tax & 0x0F);
				SC_PRINT("processatr,atrinfo,fi=%d,ds=%d,etu=%d\n",atr_info->fi, atr_info->di, atr_info->etu);
			}

			if(interface_x == 2)
			{
				/* TA2 indicates the specific mode of operation */
				/* If TA2 is not present, negotiable mode is denoted */
				SC_PRINT("interfacX is 2, set specificMode to True\n");
				atr_info->specific_mode = TRUE;
				atr_info->mode = tax & 0x0F;
				atr_info->mode_changable = !!(tax & 0x80);
				atr_info->mode_parameters = !!(tax & 0x10);
			}

			if(interface_x == 3)
			{
				SC_PRINT("interfacX is 3, set specificMode to True\n");
				if(atr_info->mode == SC_PROTOCOL_TYPE_1)
				{
					/* TA3 gives the IFSC for the card */
					atr_info->ifsc = tax;
				}
			}

			if(interface_x == 4)
			{
				atr_info->ta4 = tax;
			}
		}

		if (bitmask_yx & 0x20)
		{	 
			SC_PRINT(" enter 0x20 branch\n");
			/* TBx present */
			tbx = atr_info->buffer[index++];
			if (interface_x == 1)
			{
				/* TB1 bit b8 equals 0 */
				atr_info->tb1 = TRUE;
				/* TB1 codes II over bits b7 & b6 */
				atr_info->ii = (tbx & 0x60) >> 5;
				/* TB1 codes PI1 over bits b5 to b1 */
				atr_info->pi1 = (tbx & 0x1F);
				/* I and P define the active state at VPP. This is not
				*being done, but if it needs to be here is where to do it.
				*EMV [I-24] says: Vpp shall never be generated
				*/
			}

			if (interface_x == 2)
			{
				/* TB2 code PI2 over the eight bits */
				atr_info->tb2 = TRUE;
				atr_info->pi2 = tbx;
			}

			if (interface_x == 3)
			{
				/* TB2 code PI2 over the eight bits */
				atr_info->tb3 = TRUE;
				if(atr_info->mode == SC_PROTOCOL_TYPE_1)
				{
					/* TBi (i>2) code the CWTint (character waiting time) and BWTint (block..) */
					atr_info->cwi = tbx & 0x0F;
					atr_info->bwi = (tbx & 0xF0) >> 4;
				}
			}

			if(interface_x == 4)
			{
				atr_info->tb4 = tbx;
			}
		}

		if (bitmask_yx & 0x40)
		{	
			SC_PRINT(" enter 0x40 branch\n");
			/* TCx present */
			tcx = atr_info->buffer[index++];
			if (interface_x == 1)
			{
				/* TC1 codes N over the eight bits */
				atr_info->n = tcx ;
			}
			if (interface_x == 2)
			{
				/* TC2 code WI over the eight bits */
				atr_info->wi = tcx ;
			}
			if (interface_x == 3)
			{
				/* TC3 specifies the error detection code,
				if T=1 is used */
				if(atr_info->mode == SC_PROTOCOL_TYPE_1)
				{
					/* b1 = 1 use CRC, = 0 use LRC */
					atr_info->rc = tcx	& 0x01;
				}
			}
			if(interface_x == 4)
			{
				atr_info->tc4 = tcx ;
			}
		}

		if (bitmask_yx & 0x80)
		{	
			SC_PRINT(" enter 0x80 branch\n");
			/* TDx present */
			tdx = atr_info->buffer[index++];
			/* TDx always codes Yx+1 bitmask and T mode */
			bitmask_yx = (tdx & 0xF0);
			mode_t = (tdx & 0x0F);
			if(interface_x == 1)
			{
				/* Set the mode for this ATR */
				atr_info->mode = mode_t;
			}
			if(interface_x == 2) /* EMV does not have TD3 or more */
			{
				atr_info->td2 = TRUE;
				atr_info->td2_isnibble =  mode_t; /* 0x1 or 0xe */
				SC_PRINT("atrInfo->TD2_lsNibble is present %d \n", atr_info->td2_isnibble);
			}
		}
		else
		{
			bitmask_yx = 0;
		}

		interface_x++;
		if (interface_x > 0x0F)
		{
			/* There is no way there are more that 15 interfaces */
			printk(KERN_ERR"interface bigger than 15 %d \n", interface_x);
			LEAVE();
			return -EINVAL ;
		}
	}

	/* OK, now that that has been completed, save ptr to the historical characters */
	SC_PRINT("atr_info's history_size=%d\n",atr_info->history_size);
	if (atr_info->history_size != 0)
	{
		atr_info->history_ptr = &(atr_info->buffer[index]);
		index += atr_info->history_size;
	}

	/* And the last character should be the Check character */
	atr_info->length = index;
	/*one more for the following checking loop */
	index++; 
	/* Need to do the TCK  from T0->TCK */
	if (mode_t != 0) 
	{
		int i, check;
		for(i=1, check=0; i<index; check ^= (atr_info->buffer[i++]));
		if (check != 0)
		{
			atr_info->level = SC_WARM_RESET; /* EMV96 HACK, jump right to end */
			printk(KERN_ERR"TCK check failed %d \n", check);
			LEAVE();
			return -EINVAL ;
		}
	}
	/*setting protocol */
	s->protocol = atr_info->mode;  
	LEAVE();
	if(s->protocol ==SC_PROTOCOL_TYPE_14)
	{
		SC_PRINT("GET THE T14 card!\n");
	}
	return err;
}

/**
* elite_sci_process_atr -  calcuate the etu according to the FI and DI 
* @fint :FI is used to determine the value of F, the clock rate 
* conversion factor, which may be used to modify the frequency of the clock 
* provided by the terminal subsequent to the answer to reset
* @dint : DI is used to determine the value of D, the bit rate adjustment 
* factor, which may be used to adjust the bit duration used subsequent to the 
* answer to reset
*During the answer to reset, the bit duration is known as the initial etu, and is given by the following equation: 
*initial etu=372/f seconds, where f is in Hertz
*
*Following the answer to reset , the bit duration is known as the current etu, and is given by the following equation: 
*current etu=F/Df seconds, where f is in Hertz 
*
*Note:	For the basic answer(s) to reset described in this specification, only values of 
*F = 372 and D = 1 are supported. In the following sections, "etu" indicates current etu 
*unless otherwise specified.
*/
u16 elite_sci_calc_etu(u8 fint, u8 dint)
{
	u16 			 initial_etu = 372;
	u16 			 work_etu = 372;
	ENTER();

	/* Again, this is fairly interesting procedure on how to convert FIint
	*  and DInt into a working ETU. Follow closely 
	*/

	/* First what is our smart card controller frequency */
	/* Bit is set, means we are at 2.503125 MHz else 5.0625MHz */

	/* Next translate our rate conversion factor Interger into F */
	switch(fint) 
	/* Used conversion table in IS0 7816-3 for values, coded this way for easy read*/
	{
		case(0):
		/* WORK:Internal Clock, treat differently initialEtu = contrlFreq/9600*/
		break;
		case(1): initial_etu = 372;break;
		case(2): initial_etu = 558;break;
		case(3): initial_etu = 744;break;
		case(4): initial_etu = 1116;break;
		case(5): initial_etu = 1488;break;
		case(6): initial_etu = 1860;break;
		case(9): initial_etu = 512;break;
		case(10): initial_etu = 768;break;
		case(11): initial_etu = 1024;break;
		case(12): initial_etu = 1536;break;
		case(13): initial_etu = 2048;break;
		default:
		{
			initial_etu = 372;
		}
	}


	/* Now adjust using the Bit rate adjustment factor D */
	switch(dint)
	/*Used conversion table in IS0 7816-3,coded for easiest read */
	{
		case(1):work_etu = initial_etu;break;
		case(2):work_etu = initial_etu/2;break;
		case(3):work_etu = initial_etu/4;break;
		case(4):work_etu = initial_etu/8;break;
		case(5):work_etu = initial_etu/16;break;
		case(6):work_etu = initial_etu/32;break;
		case(8):work_etu = initial_etu/12;break;
		case(9):work_etu = initial_etu/20;break;
		case(10):work_etu = initial_etu*2;break;
		case(11):work_etu = initial_etu*4;break;
		case(12):work_etu = initial_etu*8;break;
		case(13):work_etu = initial_etu*16;break;
		case(14):work_etu = initial_etu*32;break;
		case(15):work_etu = initial_etu*64;break;
		default:
		{
			work_etu = initial_etu;
			break;
		}
	}
	LEAVE();
	return work_etu;
}

/**
* elite_sci_parse_pts_to_atr -update the atr info according to the pts response buffer 
* @s : the elite smart card device	
* @r_pts : smart card pts response buffer  
*/
static int elite_sci_parse_pts_to_atr(struct elite_sci *s,u8 *r_pts)
{

	sc_atr_info_type	*atr_info = &s->atr_info;
	u8 pck;
	u8 i;
	ENTER();
	if (r_pts[0]!=0xff) 
	{
		printk(KERN_ERR"parse_pts_to_atr:rPTS[0] is not equal 0xff\n");
		LEAVE();
		return	-EINVAL;
	}
	if ((r_pts[1]&~0x10)!=(sc9600_current_config.pts0&~0x10)) 
	{
		/*echo PTS0 exept bit 5*/
		printk(KERN_ERR"parse_pts_to_atr:rPTS[1] is invalid\n");
		LEAVE();
		return	-EINVAL;
	}

	pck=r_pts[0]^r_pts[1];
	i=2;		/*next byte*/
	if (r_pts[1]&0x10)	/*echo PTS1*/
	{
		atr_info->fi=(r_pts[i]&0xf0)>>4;
		atr_info->di=r_pts[i]&0x0f;
		pck^=r_pts[i];
		i++;
	}
	else
	{
		atr_info->fi=1;
		atr_info->di=1;
	}

	atr_info->etu = elite_sci_calc_etu(atr_info->fi, atr_info->di);    
	if (r_pts[1]&0x20)	/*echo PTS2*/
	{
		if (r_pts[i]==sc9600_current_config.pts2)
		{
			pck^=r_pts[i];
			i++;
		}
		else
		{
			printk(KERN_ERR"parse_pts_to_atr:error 2\n");
			LEAVE();
			return	-EINVAL;
		}
	}

	if (r_pts[1]&0x40)	/*echo PTS3*/
	{
		if (r_pts[i]==sc9600_current_config.pts3)
		{
			pck^=r_pts[i];
			i++;
		}
		else
		{
			printk(KERN_ERR"parse_pts_to_atr:error 3\n");
			LEAVE();
			return	-EINVAL;
		}
	}

	if (pck!=r_pts[i])	
	{
		/*PCK error*/
		printk(KERN_ERR"parse_pts_to_atr:error 4\n");
		LEAVE();
		return	-EINVAL; 
	}
	LEAVE();
	return 0;
}

/**
* elite_sci_send_pts_request -	TTL->ICC send pts message to negotiate the PTS protocol 
* @s : the elite smart card device	
* @r_pts : smart card pts response buffer  
*/
static int elite_sci_send_pts_request(struct elite_sci *s)
{

	u8 buffer[6];
	u8 size,reply_size;
	u8 pck;
	ENTER();
	/* takes care of allowing at least 16etus when exchanging data*/
	mdelay(2); 

	/*Clear IA bit*/
	sc_writew(s,sc_readw(s,SCES)&(~SC_SCES_IA),SCES);
	/*clear ISR bits*/
	sc_writew(s, 0xffff,SCIS); 
	/*unmask interrupts*/
	sc_writew(s, 0xffff,SCIM);  
	if(sc9600_current_config.protocol_type==SC_PROTOCOL_TYPE_14)
	{
		sc_writew(s,  0x1, SCD_T14);
		sc_writew(s,((~SC_INT_ISR_PARITY)&sc_readw(s,SCIM)),SCIM);
		SC_PRINT(" enter T14 branch!\n");
	}
	else
	{
		sc_writew(s,  0, SCD_T14);
		SC_PRINT(" enter T0 branch!\n");
	}
	/*PTS structure*/
	buffer[0]=0xff; 	/*PTSS*/
	sc9600_current_config.pts0 &= 0xf0;
	sc9600_current_config.pts0 |= (s->atr_info.mode & 0x0f);
	buffer[1]=sc9600_current_config.pts0;  /*PTS0*/
	pck=buffer[0]^buffer[1];
	size=2; 		/*buffer[2]*/
	if (buffer[1]&0x10) 	/*send PTS1 ?*/
	{
		buffer[size]=sc9600_current_config.pts1;
		pck^=buffer[size];
		size++;
	}
	if (buffer[1]&0x20)    /*send PTS2 ?*/
	{
		buffer[size]=sc9600_current_config.pts2;
		pck^=buffer[size];
		size++;
	}
	if (buffer[1]&0x40) 	/*send PTS3 ?*/
	{
		buffer[size]=sc9600_current_config.pts3;
		pck^=buffer[size];
		size++;
	}
	buffer[size]=pck;
	size++;

	/*PTS reply size*/
	/*PTS2,3 should be echoed by the Card, PTS response size equal to PTS requset*/
	reply_size=size; 

	/*PTS memory reg*/
	memset(s->rx_buffer, 0x00, SC_BUFFER_SIZE);
	memset(s->tx_buffer, 0x00, SC_BUFFER_SIZE);
	memcpy(s->tx_buffer, buffer, size);

	sc_writel(s,(u32)s->tx_buffer_phy,SCTMA);
	sc_writel(s,(u32)s->rx_buffer_phy,SCRMA);
	s->tx_size = size;
	s->rx_size = reply_size;

	sc_writew(s,s->tx_size ,SCTMS);
	sc_writew(s,s->rx_size-1,SCRMS);

	/*timer settings*/
	s->nr_trigg = 0;
	s->atr_icd_count = 0;
	s->atr_nr_count = 0;
	s->atr_long_count = 0;

	/*N*/
	if (s->atr_info.n != 0xff)
	{
		sc_writew(s,(s->atr_info.n+1),SCGT);
	}

	SC_PRINT("start pts...\n");
	/*command register: start PTS*/
	SC_PRINT("t_sample:%d us  and HZ:%d \n",s->t_sample,HZ);

	s->timer.data=(unsigned long)s;
	s->timer.function=elite_sci_timer_handler;
	s->timer.expires=jiffies+usecs_to_jiffies(s->t_sample);
	s->need_timer=1;
	add_timer(&s->timer);
	
	sc_writew(s,sc_readw(s,SCCMD)|(SC_SCCMND_TE | SC_SCCMND_RE | SC_SCCMND_PTS | SC_SCCMND_RD),SCCMD);

	LEAVE();
	return 0;
}

/**
* elite_sc_recv_pts_response -	ICC->TTL receive the pts response from the ICC 
* @s : the elite smart card device	
* @r_pts : smart card pts response buffer  
*/
static int elite_sc_recv_pts_response(struct elite_sci *s,u8 *r_pts)
{

	struct	sc_message* msg=NULL;
	int ret=0;
	int i=0;
	unsigned long flags;
	ENTER();
	/* Wait for response event */
	if(wait_event_interruptible(wait_fd_block,!list_empty(&(s ->messages))))
	{
		SC_PRINT("%s leave in line %d with -ERESTARTSYS\n",__func__,__LINE__);
		LEAVE();
		return -ERESTARTSYS;
	}

	msg=container_of(s ->messages.next, struct sc_message, queue);
	BUG_ON(!msg);
	spin_lock_irqsave(&s->sc_lock ,flags);
	list_del(s ->messages.next);
	spin_unlock_irqrestore(&s->sc_lock, flags);
	/* Switch on response */
	switch(msg->io_resp.event)
	{
		case(SC_EVENT_MC):
		{
			SC_PRINT(":SC_EVENT_MC\n");
			mdelay(2);
			{
				u16  status, mask; /*ISR and IMR are 16bits-regs*/
				status = sc_readw(s, SCIS);
				mask = sc_readw(s, SCIM);
				status &= mask;
				if( (status & SC_INT_ISR_PARITY) == SC_INT_ISR_PARITY)
				{
					printk(KERN_ERR"get the PE error \n");
					LEAVE();
					ret= -EIO; 
					goto ret;
				}
			}
			break;
		}
		case (SC_EVENT_PE):
			SC_PRINT(":SC_EVENT_PE\n");
		default:
		{
			printk(KERN_ERR"get the io error \n");
			LEAVE();
			ret= -EIO;
			goto ret;
		}
	}

	for(i=0; i < s->rx_size; i++)
	{
		r_pts[i] = msg->io_resp.data[i];
	}
ret:
	if(msg)
       {
		 kfree(msg);
	}
	LEAVE();
	return ret;
}

/**
* elite_sci_pts_negotiation - the whole process of PTS negotiate between ICC and TTL 
* @s : the elite smart card device	
*/
static int elite_sci_pts_negotiation(struct elite_sci *s)
{
	u8 r_pts[6];
	int ret=0;
	ENTER();

	if ( (ret=elite_sci_send_pts_request(s)) !=0 )	  
	{
		printk(KERN_ERR"pts_negotiation, error 1\n");
		LEAVE();
		return	ret;
	}

	if ( (ret=elite_sc_recv_pts_response(s, r_pts) )!=0 ) 
	{
		printk(KERN_ERR"pts_negotiation, error 2\n");
		LEAVE();
		return	ret;
	}

	if ( (ret=elite_sci_parse_pts_to_atr(s,r_pts) )!=0 ) 
	{
		printk(KERN_ERR"pts_negotiation, error 3\n");
		LEAVE();
		return	ret;
	}
	
	LEAVE();
	return	ret;
}

/**
* elite_sci_check_atrconf - the atr info of smart card is valid or not 
* @s : the elite smart card device	
*/
static int elite_sci_check_atrconf(struct elite_sci *s)
{
	sc_atr_info_type	*atr_info = &s->atr_info;
	int   err = 0;

	ENTER();

	/* EMV96: Comfirm the ATR values are correct */
	if(atr_info->length > 32)
	{
		/* ATR exceeds maximum allowed*/
		printk(KERN_ERR"atr_info's length is great than 32\n");
		LEAVE();
		return	-EINVAL;
	}

	if ((atr_info->ts != SC_INVERSE_CONVENTION) &&
		(atr_info->ts != SC_DIRECT_CONVENTION))
	{
		/* Wrong Atr TS Convention: reject ICC*/
		printk(KERN_ERR"atr_info's ts is invalid\n");
		LEAVE();
		return	-EINVAL;
	}

	/*check for PTS/PPS*/
	if ( (((atr_info->fi) != 1 ) || ((atr_info->di) != 1))	&& (atr_info->specific_mode == FALSE) ) //Negotiable mode - TA2 absent
	{
		if(!sc9600_current_config.use_pts)	
		{
			/*not support PTS / PPS*/
			/*Negotiable mode - TA2 absent: use default parameter, should support PTS/PPS later*/
			atr_info->fi=1;
			atr_info->di=1;
		}
		else   
		{
			/*support PTS / PPS*/
			if (atr_info->level != SC_COLD_RESET)	 
			{
				/*not cold reset, no PTS*/
				/* only TA1 == 0x11 is ok if negotiable mode [TA2 absent]!*/
				printk(KERN_ERR"atr_info's level isn't SC_COLD_RESET\n");
				LEAVE();
				return	-EINVAL;
			}
		}
	}

  
	if(sc9600_current_config.standard != SC_STANDARD_ISO7816)  /*EMV*/
	{
		if( (((atr_info->fi) > 1 ) || ((atr_info->di) > 3)) && (atr_info->specific_mode == TRUE) && (atr_info->mode_parameters == FALSE))
		{
			/*specific mode with b5 of TA2 is 0: if TA1>0x13, reject the ATR*/
			/* wrong TA1 for EMV96 [I-23]*/
			printk(KERN_ERR"scAtrConfigCheck,invalid 3\n");
			LEAVE();
			return	-EINVAL;
		}
	}


	if(((atr_info->pi1 != 0) || (atr_info->ii != 0) ) &&(atr_info->level == SC_COLD_RESET))
	{
		/* Vpp Requested during Cold reset: reject ATR*/
		printk(KERN_ERR"scAtrConfigCheck,invalid 4\n");
		LEAVE();
		return	-EINVAL;
	}

	if((atr_info->tb1 == FALSE ) &&(atr_info->level == SC_COLD_RESET))
	{
		/*TB1 Must be present at Cold reset: reject ATR*/
		printk(KERN_ERR"scAtrConfigCheck,invalid 5\n");
		LEAVE();
		return	-EINVAL;
	}
	#if 0
	if(atr_info->mode > SC_PROTOCOL_TYPE_1)
	{
		/* ATR Has invalid mode: reject ATR*/
		printk(KERN_ERR"scAtrConfigCheck,invalid 6\n");
		LEAVE();
		return	-EINVAL;
	}
	#endif

	if(atr_info->mode_parameters == TRUE) /*TA2:b5=1*/
	{
		if(sc9600_current_config.standard == SC_STANDARD_ISO7816)  /*ISO 7816*/
		{
			atr_info->fi=1;
			atr_info->di=1;
		}
		else		/*EMV*/
		{
			/* Implicitly defined paramters: reject ATR b5 of TA2 is 1.*/
			printk(KERN_ERR"scAtrConfigCheck,invalid 7\n");
			LEAVE();
			return	-EINVAL;
		}
	}


	if(atr_info->pi2 != 0)
	{
		/* VPP override voltage specified [actually this is the value in TB2]*/
		printk(KERN_ERR"scAtrConfigCheck,invalid 8\n");
		LEAVE();
		return	-EINVAL;
	}

	if(atr_info->tb2 != FALSE)
	{
		/* Reject all ATR's with TB2*/
		printk(KERN_ERR"scAtrConfigCheck,invalid 9\n");
		LEAVE();
		return	-EINVAL;
	}

	if((atr_info->wi == 0) || (atr_info->wi > 0x0A))
	/* Need to accept 0x01-0x09 if able to support them [TEST 1724] */
	{
		/* Work waiting time other than support value: reject ATR*/
		printk(KERN_ERR"scAtrConfigCheck,invalid 10\n");
		LEAVE();
		return	-EINVAL;
	}

	if(atr_info->ifsc < 0x10)
	{
		/*Information Field length invalid: it shall be between 0x10 to 0xFE*/
		printk(KERN_ERR"scAtrConfigCheck,invalid 11\n");
		LEAVE();
		return	-EINVAL;
	}

	if(atr_info->ifsc == 0xFF)
	{
		/* Information Field length invalid: value 0xFF not allowed*/
		printk(KERN_ERR"scAtrConfigCheck,invalid 12\n");
		LEAVE();
		return	-EINVAL;
	}


	/* Check: page I-28*/
	if(atr_info->mode == SC_PROTOCOL_TYPE_1)
	{
		/* Extra Wait time invalid CWI*/
		if(atr_info->cwi > 0x05)
		{
			if(sc9600_current_config.standard != SC_STANDARD_ISO7816)  /*EMV*/
			{
				 printk(KERN_ERR"scAtrConfigCheck,invalid 13\n");
				 LEAVE();
				  return  -EINVAL;
			}
		}



		/* Extra Wait time invalid BWI*/
		if(sc9600_current_config.standard == SC_STANDARD_ISO7816) /*ISO7816*/
		{
			if(atr_info->bwi > 9)
			{
				printk(KERN_ERR"scAtrConfigCheck,invalid 14\n");
				LEAVE();
				return	-EINVAL;
			}
		}
		else		/*EMV*/
		{
			if(atr_info->bwi > 0x04)
			{
				printk(KERN_ERR"scAtrConfigCheck,invalid 15\n");
				LEAVE();
				return	-EINVAL;
			}
		}

		/* I'm checking whether 2^CWI < (N +1) or not: if N=0xFF, the value
		*  N=-1 should be used, so that the disequation is always never true. [GB]
		*/
		if((atr_info->n) != 0xFF)
		{
			if((0x01<<(atr_info->cwi)) < ((atr_info->n)+1))
			{
				printk(KERN_ERR"scAtrConfigCheck,invalid 16\n");
				LEAVE();
				return	-EINVAL;
			}
		}
	}

	if( (atr_info->mode == 1) && (atr_info->tb3 == FALSE) )
	{
		/* No TB3 present for T=1 mode*/
		printk(KERN_ERR"scAtrConfigCheck,invalid 17\n");
		LEAVE();
		return	-EINVAL;
	}


	if(atr_info->td2)
	{
		if(sc9600_current_config.standard == SC_STANDARD_ISO7816)  //ISO
		{
			if (atr_info->mode>atr_info->td2_isnibble)	  /*TD1:T>TD2:T ?*/
			{
				printk(KERN_ERR"scAtrConfigCheck,invalid 18\n");
				LEAVE();
				return	-EINVAL;
			}
		}
		else	/*EMV*/
		{
			if(atr_info->td2_isnibble != 0x1)
			{
				if((atr_info->mode == SC_PROTOCOL_TYPE_0) && (atr_info->td2_isnibble != 0x0E))
				{
					printk(KERN_ERR"scAtrConfigCheck,invalid 19\n");
					LEAVE();
					return	-EINVAL;
				}
				if(atr_info->mode == SC_PROTOCOL_TYPE_1)
				{
					printk(KERN_ERR"scAtrConfigCheck,invalid 20\n");
					LEAVE();
					return	-EINVAL;
				}
			}
		}
	}


	if(atr_info->rc != 0x00)
	{
		/* Redundancy Check Invalid: should be NULL [I-28]*/
		printk(KERN_ERR"scAtrConfigCheck,invalid 21\n");
		LEAVE();
		return	-EINVAL;
	}

	SC_PRINT("scAtrConfigCheck,sc9600_current_config.use_pts=%d\n",sc9600_current_config.use_pts);

	/*check for PTS/PPS Negotiation begin */
	if ((sc9600_current_config.use_pts) && (err==0))	/*support PTS/PPS ?*/
	{
		SC_PRINT("support pts/pps,atrInfo->level=%d,atrInfo->specificMode=%d\n",atr_info->level,atr_info->specific_mode);
		if (atr_info->level == SC_COLD_RESET)	 /*cold reset*/
		{
			if (atr_info->specific_mode == FALSE) /*Negotiable Mode*/
			{
				if ( (atr_info->mode== SC_IO_MODE_T0) || (atr_info->mode== SC_IO_MODE_T1)||(atr_info->mode== SC_IO_MODE_T14)  )   //T=0 or T=1
				{
					/* FI&DI not default, or N=255, or more than one protocol type is supported by the card*/
					if ( (atr_info->fi!= 1) || (atr_info->di!= 1) ||(atr_info->n==255) || 
					((atr_info->td2) && (atr_info->td2_isnibble!=atr_info->mode)))
					{
						err=elite_sci_pts_negotiation(s);
					}
				}
			}
		}
	}
	/*Chuckle for PTS/PPS Negotiation end */
	LEAVE();
	return err;
}

/**
* elite_sci_isr - the ISR handle for smart card interrput
* @s : the elite smart card device 
* @dev_id : the dev id for this interrupt 
* @regs : the regs of current 
*/
static irqreturn_t elite_sci_isr(int irq, void *dev_id)
{
	u16 status=0;
	u16 mask=0;
	struct sc_message* msg=NULL;
	struct elite_sci *s= (struct elite_sci *)dev_id;
	ENTER();
	BUG_ON(!s);
	
	/* Confirm that the interrupt is for this routine */
	status = sc_readw(s, SCIS);
	mask = sc_readw(s, SCIM);
	//SC_PRINT("SCIS:0x%x,SCIM:0x%x\n",sc_readw(s, SCIS),sc_readw(s, SCIM));
	 printk(KERN_NOTICE"SCIS:0x%x,SCIM:0x%x\n",sc_readw(s, SCIS),sc_readw(s, SCIM));

	status &= mask;
	if (status == 0)
	{
		/* Interrupt handle called but interrupt not for us */
		 printk(KERN_ERR"status is zero!\n");
		return IRQ_HANDLED;
        
	}
	/* Clear the interrupt status and mask bits for the interrupts */
	sc_writew(s,status,SCIS);
	//sc_writew(s,status,SCIM);
	sc_writew(s,((~status)&sc_readw(s,SCIM)),SCIM);

	/*if status is SC_INT_ISR_CWT
	* character waiting time, T=1 mode only!
	*/
	if((status & SC_INT_ISR_CWT) == SC_INT_ISR_CWT) 
	{
		//SC_PRINT("enter CWT branch!\n");
		printk(KERN_NOTICE"enter CWT branch!\n");
		s->need_timer=0;
		del_timer(&s->timer);

		msg=kzalloc(sizeof(struct sc_message),GFP_ATOMIC);
		if(!msg)
		{
			LEAVE();
			return IRQ_NONE;
		}
		msg->io_resp.event = SC_EVENT_CWT;
		INIT_LIST_HEAD(&(msg->queue));
		list_add(&(s->messages),&(msg->queue));
	}

	/*if status is SC_INT_ISR_OFF
	* for card detect
	*/
	if((status & SC_INT_ISR_OFF) == SC_INT_ISR_OFF)   
	{
		//SC_PRINT("enter CARD DETECT branch!\n");	
		printk(KERN_NOTICE"enter CARD DETECT branch!\n");	
		s->need_timer=0;
		del_timer(&s->timer);
		/* Generate a CD event to Task */
		msg=kzalloc(sizeof(struct sc_message),GFP_ATOMIC);
		if(!msg)
		{
			LEAVE();
			return IRQ_NONE;
		}
		msg->io_resp.event  = SC_EVENT_CD;
		INIT_LIST_HEAD(&(msg->queue));
		list_add(&(s->messages),&(msg->queue));
		/* Generate a CD event to Task */
		{
			struct sc_message* event_msg=kzalloc(sizeof(struct sc_message),GFP_ATOMIC);
			if(!event_msg)
			{
				LEAVE();
				return IRQ_NONE;
			}
			event_msg->io_resp.event= SC_EVENT_CD;
			INIT_LIST_HEAD(&(event_msg->queue));
			list_add(&(s->event_messages),&(event_msg->queue));
		}
	}

	/*if status is SC_INT_ISR_PWROFF for power off
	*/
	if((status & SC_INT_ISR_PWROFF) == SC_INT_ISR_PWROFF)  
	{
		//SC_PRINT("enter PWROFF branch!\n");
		
		printk(KERN_NOTICE"enter PWROFF branch!\n");
		s->need_timer=0;
		del_timer(&s->timer);
		/* Generate a PO event to Task */
		msg=kzalloc(sizeof(struct sc_message),GFP_ATOMIC);
		if(!msg)
		{
			LEAVE();
			return -ENOMEM;
		}
		msg->io_resp.event = SC_EVENT_PO;
		INIT_LIST_HEAD(&(msg->queue));
		list_add(&(s->messages),&(msg->queue));
	}

	/*if status is SC_INT_ISR_NOREPLY
	* for no reply
	*/
	if((status & SC_INT_ISR_NOREPLY) == SC_INT_ISR_NOREPLY)  
	{
		//SC_PRINT("enter NOREPLY branch!\n");		
		
		printk(KERN_NOTICE"enter NOREPLY branch!\n");		
		/* mask NR and clear bit */
		s->nr_trigg = 1;
	}

	/*if status is SC_INT_ISR_MESS
	* for message complete
	*/
	if( (status & SC_INT_ISR_MESS) == SC_INT_ISR_MESS)	
	{
		//SC_PRINT("enter MESS branch!\n");
		printk(KERN_NOTICE"enter MESS branch!\n");
		s->need_timer=0;	
		/* Clear the interrupt status and mask bits for the timer interrupts */
		del_timer(&s->timer);
		s->wwt_count = 0;

		/* Generate a MC event and send it*/
		msg=kzalloc(sizeof(struct sc_message),GFP_ATOMIC);
		if(!msg)
		{
			LEAVE();
			return -ENOMEM;
		}
		msg->io_resp.event = SC_EVENT_MC;
		msg->io_resp.size = s->rx_size;
		INIT_LIST_HEAD(&(msg->queue));
		{
			u16 i = 0;
			for(i=0; i < msg->io_resp.size ; i++) 
			{
				/*SC_BUFFER_SIZE*/
				msg->io_resp.data[i] = s->rx_buffer[i];
				SC_PRINT("0x%x",s->rx_buffer[i]);
			}
			SC_PRINT("\n");
		}
		list_add(&(s->messages),&(msg->queue));

	}


	/*if status is SC_INT_ISR_PARITY
	* for parity error
	*/
	if( (status & SC_INT_ISR_PARITY) == SC_INT_ISR_PARITY)	  
	{
		//SC_PRINT("enter PARITY branch!\n");	
		
		printk(KERN_NOTICE"enter PARITY branch!\n");	
		s->need_timer=0;
		del_timer(&s->timer);
		if(sc_readw(s,SCES)|SC_SCES_IA)
		{
			sc_writew(s,sc_readw(s,SCES)&(~SC_SCES_IA),SCES);
		}

		/* Generate a PE event to Task */
		msg=kzalloc(sizeof(struct sc_message),GFP_ATOMIC);
		if(!msg)
		{
			LEAVE();
			return -ENOMEM;
		}

		/* Generate a PE event to Task */
		s->parity_error = 1;
		msg->io_resp.event = SC_EVENT_PE;
		INIT_LIST_HEAD(&(msg->queue));
		list_add(&(s->messages),&(msg->queue));
	}

	if(!list_empty(&(s->messages)))
	{
		wake_up_interruptible(&wait_fd_block);
		kill_fasync(&s->cmd_async,SIGIO, POLL_IN);
	}

	LEAVE();
	return IRQ_HANDLED;
}


/**
* elite_sci_timer_handler - the timeout handle for kernel timer
* @arg : the arg for timer handler 
*/
void elite_sci_timer_handler(unsigned long arg)
{
	struct elite_sci *s= (struct elite_sci *)arg;
	sc_atr_info_type	*atr_info = &(s->atr_info);
	u16 	rd_status=0;
	s32 need_timer=s->need_timer;
	BUG_ON(!s);
	ENTER();
	if(!need_timer)
	{
		LEAVE();
		return ;
	}
	/* checking if a byte has been received. rd_status is equal to
	*- 0 if a byte has been received
	* - it's CL_SC_SCCMND_RD if no byte has been received 
	*/
	rd_status = sc_readw(s,SCCMD)&SC_SCCMND_RD;
	if(rd_status == 0)
	{
		sc_writew(s,sc_readw(s,SCCMD)|SC_SCCMND_RD,SCCMD);
	}

	/* checking for RST line to go high */
	if(((sc_readw(s,SCCMD)&SC_SCCMND_RST) == SC_SCCMND_RST) && (s->rst_val == 0))
	{
		s->rst_val = 1;
		//SC_PRINT("enter SC_SCCMND_RST branch!\n");	
		printk(KERN_NOTICE"enter SC_SCCMND_RST branch!\n");
		/* HACK: Once the ATR has started and the RST line is high, prevent the
		*9600 from prematurely lowering the RST line. This code is here for
		*convenience and does not relate to time-out handling 
		*/
		if ((sc_readw(s,SCCMD)&SC_SCCMND_RST) &&
		((sc_readw(s,SCCMD)&SC_SCCMND_RST)&SC_SCCMND_RL != SC_SCCMND_RL))
		{
			sc_writew(s,sc_readw(s,SCCMD)|SC_SCCMND_RL,SCCMD);
		}
	}

	switch(s->state)
	{
		case(SC_STATE_ATR):
		{
			/* ATR, I have to check for:
			*- First character received within 40,000 clkcycles
			*- Inter Character Delay max 9,600 etus
			*- All ATR received within 19,200 etus	
			*/

			if(atr_info->ts == 0) 
			{
				/*still haven't got a byte*/
				/* I'm checking if I got the first byte in the status register: I have
				* to do that because the RD bit is updated later than this register.
				* This is due to the fact that TS is used by the HW for synchronization and
				* other issues (see spec and manual). 
				*/
				
				u16  atr_status =sc_readw(s,SCS);
				u8	 atr_ts = atr_status >> 8;
				if(s->nr_trigg == 1)
				{
					//SC_PRINT("enter SC_STATE_ATR nr_trigg branch!\n");	
					printk(KERN_NOTICE"enter SC_STATE_ATR nr_trigg branch!\n");	
					s->atr_nr_count++;
				}

				if(((atr_ts == 0x3F) || (atr_ts == 0x3B)) || (rd_status == 0))
				{
					atr_info->ts = 1;
					s->atr_nr_count = 0;
					//SC_PRINT("enter SC_STATE_ATR atr_ts branch!\n");	
					printk(KERN_NOTICE"enter SC_STATE_ATR atr_ts branch!\n");	
					/* I got the byte, so some time has
					*already passed from the leading edge of TS 
					*/
					s->atr_icd_count++;

				}
			}
			/* got the first byte: I'm now checking time-out condition for
			*- delay between bytes received
			*- all ATR shall be received in 19,200 etus 
			*/
			else
			{
				if(rd_status == SC_SCCMND_RD) 
				{
					/*no byte received*/
					s->atr_icd_count++;
					s->atr_long_count++;
					//SC_PRINT("enter SC_STATE_ATR SC_SCCMND_R branch!\n");	
					printk(KERN_NOTICE"enter SC_STATE_ATR SC_SCCMND_R branch!\n");	
				}
				else
				{
					s->atr_icd_count = 0;
					s->atr_long_count++;
					//SC_PRINT("enter SC_STATE_ATR atr_long_count branch!\n");
					printk(KERN_NOTICE"enter SC_STATE_ATR atr_long_count branch!\n");	
				}
			}

			/* added +1 and now pass the NR tests: I get the NR @37,240 clkcycles.
			*  I should deactivate the card when I get the first byte at 42,001 (start time) +
			* 3720 (cycles for 1 byte) = 45,000 clk cycles.
			* NR-polling is 2000 clk 
			*/
			if(((s->atr_nr_count) == (s->atr_nr_max + 1)) && (s->nr_trigg)) 
			{
				/*got first byte within 40,000clkcycles*/
				struct sc_message *msg=NULL;
				unsigned long flags;
				/*masking interrupts*/	
				sc_writew(s, 0x0,SCIM);	
				/*clear ISR bits*/
				sc_writew(s, 0xffff,SCIS); 
				s->need_timer=0;
				del_timer(&s->timer);
				
				/* Generate a CD event to Task */
				msg=kzalloc(sizeof(struct sc_message),GFP_ATOMIC);
				if(!msg)
				{
					printk(KERN_ERR "nono enough memory\n" );
					LEAVE();
					return ;
				}
				printk(KERN_NOTICE"enter SC_STATE_ATR SC_EVENT_NR branch!\n");	

				//SC_PRINT("enter SC_STATE_ATR SC_EVENT_NR branch!\n");	
				msg->io_resp.event = SC_EVENT_NR;
				INIT_LIST_HEAD(&(msg->queue));
				spin_lock_irqsave(&s->sc_lock,flags);
				list_add(&(s->messages),&(msg->queue));
				spin_unlock_irqrestore(&s->sc_lock, flags);
				if(!list_empty(&(s->messages)))
				{
					wake_up_interruptible(&wait_fd_block);
					kill_fasync(&s->cmd_async,SIGIO, POLL_IN);
				}
		              	need_timer=0;		
				break;
			}


			if((s->atr_icd_count) == (s->atr_icd_max))
			{
				struct sc_message *msg=NULL;
				unsigned long flags;
				/* No bytes have been sent since timer was started:send ICD event */
				/*masking interrupts*/	
				sc_writew(s, 0x0,SCIM);	
				/*clear ISR bits*/
				sc_writew(s, 0xFFFF,SCIS); 

				s->state = SC_STATE_DEACTIVATED;
				s->need_timer=0;
				del_timer(&s->timer);

				msg=kzalloc(sizeof(struct sc_message),GFP_ATOMIC);
				if(!msg)
				{
					printk(KERN_ERR "nono enough memory\n" );
					LEAVE();
					return ;
				}
				printk(KERN_NOTICE"enter SC_STATE_ATR SC_EVENT_ICD branch!\n");	
				//SC_PRINT("enter SC_STATE_ATR SC_EVENT_ICD branch!\n");	
				msg->io_resp.event = SC_EVENT_ICD;	
				INIT_LIST_HEAD(&(msg->queue));	
				spin_lock_irqsave(&s->sc_lock,flags);
				list_add(&(s->messages),&(msg->queue));
				spin_unlock_irqrestore(&s->sc_lock, flags);
				if(!list_empty(&(s->messages)))
				{
					wake_up_interruptible(&wait_fd_block);
					kill_fasync(&s->cmd_async,SIGIO, POLL_IN);
				}
		              need_timer=0;	
				break;
			}
			/* all ATR in 19,200etus?*/
			if ((s->atr_long_count) == (s->atr_long_max)) 
			{	
				struct sc_message *msg=NULL;
				unsigned long flags;
				/*masking interrupts*/	
				sc_writew(s, 0x0,SCIM);	
				/*clear ISR bits*/
				sc_writew(s, 0xffff,SCIS); 
				s->need_timer=0;
				del_timer(&s->timer);
				msg=kzalloc(sizeof(struct sc_message),GFP_ATOMIC);
				if(!msg)
				{
					printk(KERN_ERR "nono enough memory\n" );
					LEAVE();
					return ;
				}
				printk(KERN_NOTICE"enter SC_STATE_ATR SC_EVENT_LONG_ATR  branch!\n");	
				//SC_PRINT("enter SC_STATE_ATR SC_EVENT_ICD branch!\n");	
				msg->io_resp.event = SC_EVENT_LONG_ATR;
				INIT_LIST_HEAD(&(msg->queue));
				spin_lock_irqsave(&s->sc_lock,flags);
				list_add(&(s->messages),&(msg->queue));
				spin_unlock_irqrestore(&s->sc_lock, flags);
				if(!list_empty(&(s->messages)))
				{
					wake_up_interruptible(&wait_fd_block);
					kill_fasync(&s->cmd_async,SIGIO, POLL_IN);
				}
				need_timer=0;	
			}
			break;
		} //end of case ATR

		case(SC_STATE_IO):
		{
			/* Handling T0 timing events */
			if((s->protocol == SC_PROTOCOL_TYPE_0)||(s->protocol == SC_PROTOCOL_TYPE_14))
			{
				if(rd_status == SC_SCCMND_RD)
				{
					/*no character received yet, increase counter*/
					s->wwt_count++;
					//SC_PRINT("enter SC_STATE_IO wwt_count++ branch!\n");	
					printk(KERN_NOTICE"enter SC_STATE_IO wwt_count++ branch!\n");	

				}
				else
				{
					s->wwt_count = 0;
					//SC_PRINT("enter SC_STATE_IO wwt_count = 0 branch!\n");
					printk(KERN_NOTICE"enter SC_STATE_IO wwt_count = 0 branch!\n");
				}

				if ((s->wwt_count) == (s->wwt_max))
				{
					struct sc_message *msg=NULL;
					unsigned long flags;
					/* Turn off Transmitting so that no late data goes out */
					sc_writew(s,sc_readw(s,SCCMD)&(~(SC_SCCMND_TE | SC_SCCMND_RE)),SCCMD);
					sc_writew(s,0,SCRMS);
					sc_writew(s,0,SCRMS);
					/*masking interrupts*/	
					sc_writew(s, 0x0,SCIM);
					/*clear ISR bits*/
					sc_writew(s, 0xFFFF,SCIS); 

					/* Deactivate the card: we only have 960etus, so I do it here [GB] */
					elite_sci_hw_deactive(s);
					s->need_timer=0;
					del_timer(&s->timer);
					
					s->state = SC_STATE_DEACTIVATED;
					msg=kzalloc(sizeof(struct sc_message),GFP_ATOMIC);
					if(!msg)
					{
						printk(KERN_ERR "nono enough memory\n" );
						LEAVE();
						return ;
					}
					//SC_PRINT("enter SC_STATE_IO SC_EVENT_WWT branch!\n");
					printk(KERN_NOTICE"enter SC_STATE_IO SC_EVENT_WWT branch!\n");
					msg->io_resp.event= SC_EVENT_WWT;
					INIT_LIST_HEAD(&(msg->queue));
					spin_lock_irqsave(&s->sc_lock,flags);
					list_add(&(s->messages),&(msg->queue));
					spin_unlock_irqrestore(&s->sc_lock, flags);
					if(!list_empty(&(s->messages)))
					{
						wake_up_interruptible(&wait_fd_block);
						kill_fasync(&s->cmd_async,SIGIO, POLL_IN);
					}
			              need_timer=0;	
				}

			} // end of case 'T0'

			/* Handling T1 timing events */
			if(s->protocol == SC_PROTOCOL_TYPE_1)
			{
				if((rd_status == SC_SCCMND_RD))
				{
					/*no character received yet, increase counter*/
					if(s->got_byte == 0) s->bwt_count++; 
					else s->cwt_count++;
					//SC_PRINT("enter SC_STATE_IO SC_SCCMND_RD branch!\n");
					printk(KERN_NOTICE"enter SC_STATE_IO SC_SCCMND_RD branch!\n");
				}
				else
				{
					if(s->got_byte == 0)
					{
						s->got_byte = 1;
						s->bwt_count = 0;
					}
					else s->cwt_count = 0;
					//SC_PRINT("enter SC_STATE_IO !!SC_SCCMND_RD branch!\n");
					printk(KERN_NOTICE"enter SC_STATE_IO !!SC_SCCMND_RD branch!\n");
				}

				if((s->bwt_count) == (s->bwt_max))
				{ 
					struct sc_message *msg=NULL;
					unsigned long flags;
					/* ICC has not responded to the last character sent by the terminal within the correct time */
					/*masking interrupts*/	
					sc_writew(s, 0x0,SCIM);
					/*clear ISR bits*/
					sc_writew(s, 0xFFFF,SCIS); 

					elite_sci_hw_deactive(s);
					s->need_timer=0;
					del_timer(&s->timer);
					s->state = SC_STATE_DEACTIVATED;

					msg=kzalloc(sizeof(struct sc_message),GFP_ATOMIC);
					if(!msg)
					{
						printk(KERN_ERR "nono enough memory\n" );
						LEAVE();
						return ;
					}
					printk(KERN_NOTICE"enter SC_STATE_IO SC_EVENT_BWT branch!\n");
					//SC_PRINT("enter SC_STATE_IO SC_EVENT_BWT branch!\n");
					msg->io_resp.event= SC_EVENT_BWT;
					INIT_LIST_HEAD(&(msg->queue));
					spin_lock_irqsave(&s->sc_lock,flags);
					list_add(&(s->messages),&(msg->queue));
					spin_unlock_irqrestore(&s->sc_lock, flags);
					if(!list_empty(&(s->messages)))
					{
						wake_up_interruptible(&wait_fd_block);
						kill_fasync(&s->cmd_async,SIGIO, POLL_IN);
					}
					need_timer=0;	
				}

				if((s->cwt_count) == (s->cwt_max))
				{
					struct sc_message *msg=NULL;
					unsigned long flags;
					/* ICC has not responded to the last character sent by the terminal within the correct time:
					*this might be due to wrong lenght of the LEN field, set to FF: in this case I could not
					*deactivate the card but send an R-block [Test 1772]
					*/
					/*masking interrupts*/	
					sc_writew(s, 0x0,SCIM);
					/*clear ISR bits*/
					sc_writew(s, 0xFFFF,SCIS); 
					s->need_timer=0;
					del_timer(&s->timer);

					msg=kzalloc(sizeof(struct sc_message),GFP_ATOMIC);
					if(!msg)
					{
						printk(KERN_ERR "nono enough memory\n" );
						LEAVE();
						return ;
					}
					printk(KERN_NOTICE"enter SC_STATE_IO SC_EVENT_CWT branch!\n");
					//SC_PRINT("enter SC_STATE_IO SC_EVENT_CWT branch!\n");
					msg->io_resp.event= SC_EVENT_CWT;
					INIT_LIST_HEAD(&(msg->queue));
					spin_lock_irqsave(&s->sc_lock,flags);
					list_add(&(s->messages),&(msg->queue));
					spin_unlock_irqrestore(&s->sc_lock, flags);
					if(!list_empty(&(s->messages)))
					{
						wake_up_interruptible(&wait_fd_block);
						kill_fasync(&s->cmd_async,SIGIO, POLL_IN);
					}
			        	need_timer=0;	
				}
			} // end of case 'T1'

			break;
		}//end case IO
		default:
		{
			printk(KERN_ERR "smart card's state is invalid!\n" );
			LEAVE();
			return ;

		}
	}
	if(need_timer)
	{
	   	//SC_PRINT("t_sample:%d us and HZ:%d \n",s->t_sample,HZ);
	   	printk(KERN_NOTICE"t_sample:%d us and HZ:%d \n",s->t_sample,HZ);
		mod_timer(&s->timer,jiffies+usecs_to_jiffies(s->t_sample));
	}
	LEAVE();
	return;
}

/**
* elite_sci_open - open the elite smart card file
* @inode : the inode of this smart card file in VS filesystem
* @file : the file description 
*/
int  elite_sci_open (struct inode * inode ,struct file * file)
{
	struct elite_sci *elist_sci =NULL;
	int ret=0;
	ENTER();
	elist_sci=container_of(inode->i_cdev, struct elite_sci, cdev);
	BUG_ON(!elist_sci);
	if (down_interruptible (&elist_sci->sem))
	{
		LEAVE();
		return -ERESTARTSYS;
	}
	file->private_data = elist_sci;
	memset(elist_sci->tx_buffer,0,SC_BUFFER_SIZE);
	memset(elist_sci->rx_buffer,0,SC_BUFFER_SIZE);
	elist_sci->state=SC_STATE_NOCARD;
	INIT_LIST_HEAD(&(elist_sci->messages));
	INIT_LIST_HEAD(&(elist_sci->event_messages));
	init_timer(&(elist_sci->timer));
	up(&(elist_sci->sem));
	LEAVE();
	return ret;
}	

/**
* elite_sci_release - close the elite smart card file
* @inode : the inode of this smart card file in VS filesystem
* @file : the file description 
*/
int elite_sci_release (struct inode  * inode ,struct file * file)
{
	struct elite_sci *elist_sci=file->private_data;
	BUG_ON(!elist_sci);
	ENTER();
	if (down_interruptible (&elist_sci->sem))
	{
		LEAVE();
		return -ERESTARTSYS;
	}
   	while(!list_empty(&(elist_sci->messages)))
	{
		SC_PRINT("******enter  free msg ********\n");
		struct sc_message* msg=NULL;
		unsigned long flags;
		msg=container_of(elist_sci->messages.next, struct sc_message, queue);
		BUG_ON(!msg);
		spin_lock_irqsave(&elist_sci->sc_lock ,flags);
		list_del(elist_sci->messages.next);
		spin_unlock_irqrestore(&elist_sci->sc_lock, flags);
		if(msg)
		{
			kfree(msg);
		}
	}
	while(!list_empty(&(elist_sci->event_messages)))
	{
		SC_PRINT("******enter  free msg ********\n");
		struct sc_message* msg=NULL;
		unsigned long flags;
		msg=container_of(elist_sci->event_messages.next, struct sc_message, queue);
		BUG_ON(!msg);
		spin_lock_irqsave(&elist_sci->sc_lock ,flags);
		list_del(elist_sci->event_messages.next);
		spin_unlock_irqrestore(&elist_sci->sc_lock, flags);
		if(msg)
		{
			kfree(msg);
		}
	}

	fasync_helper(-1, file,0, &elist_sci->cmd_async);
	elite_sci_hw_terminate(elist_sci);
	up(&(elist_sci->sem));
	LEAVE();
	return	0;
	
}

/**
* elite_sci_release - close the elite smart card file
* @inode : the inode of this smart card file in VS filesystem
* @file : the file description 
* @cmd : the cmd from the user space 
* @data : the data from the user space 
*/
long elite_sci_unlocked_ioctl (struct file * file, unsigned int cmd, unsigned long data)
{

	void __user *argp = (void __user *)data;
	struct elite_sci *s=file->private_data;
	BUG_ON(!s);
	ENTER();
	switch (cmd) 
	{
		case SMART_CARD_GETBASECONFIG:
		{
			if (copy_to_user(argp, &sc9600_base_config, sizeof(sc9600_base_config)))
				 return -EFAULT;
			LEAVE();
			return  0;
		}
		case SMART_CARD_GETHWCONFIG:
			return copy_to_user(argp, &sc9600_current_config, sizeof(sc9600_current_config)) ? -EFAULT : 0;

		case SMART_CARD_GETATRCONFIG:
			return copy_to_user(argp, &sc9600_atr_config, sizeof(sc9600_atr_config)) ? -EFAULT : 0;

		case SMART_CARD_CARDDETECT:
		{
			int result =0;
			result=elite_sci_hw_detect(s);
			return copy_to_user(argp, &result, sizeof(int)) ? -EFAULT : 0;
		}

		case SMART_CARD_GET_STATUS:
			return copy_to_user(argp, &s->state, sizeof(int)) ? -EFAULT : 0;

		case SMART_CARD_GET_SCSSTATUS:
		{ 
			u16 result =0;
			result=sc_readw(s,SCS);
			return copy_to_user(argp, &result, sizeof(u16)) ? -EFAULT : 0;
		}

		case SMART_CARD_GET_SCBGT:
			return copy_to_user(argp, &(s->bgt), sizeof(u8)) ? -EFAULT : 0;

		case SMART_CARD_GET_SCPE :
			return copy_to_user(argp, &(s->parity_error), sizeof(u8)) ? -EFAULT : 0;

		case SMART_CARD_GET_SCIM :
		{
			u16 result =0;
			result=sc_readw(s,SCIM)&SCIM_VALUE_MASK;
			return copy_to_user(argp, &result, sizeof(u16)) ? -EFAULT : 0;
		}

		case SMART_CARD_GET_SCIS:
		{
			u16 result =0;
			result=sc_readw(s,SCIS)&SCIS_VALUE_MASK;
			return copy_to_user(argp, &result, sizeof(u16)) ? -EFAULT : 0;
		}

		case SMART_CARD_GET_RXSIZE:
			return copy_to_user(argp, &(s->rx_size), sizeof(u16)) ? -EFAULT : 0;

		case SMART_CARD_GET_SCRMS:
		{
			u16 result =0;
			result=sc_readw(s,SCRMS)&SCIM_VALUE_MASK;
			return copy_to_user(argp, &result, sizeof(u16)) ? -EFAULT : 0;
		}

		case SMART_CARD_GET_ATRINFO:
			return copy_to_user(argp, &(s->atr_info), sizeof(sc_atr_info_type)) ? -EFAULT : 0;

		case SMART_CARD_DEACT:	
			return elite_sci_hw_deactive(s);

		
		case SMART_CARD_RESET:
		{
			sc_reset_level_type level;
			int res=0;
			if (copy_from_user(&level, argp, sizeof(int)))
				return -EFAULT;
			res=elite_sci_hw_reset(s, level);
			return res;
		}
		case SMART_CARD_SET_STATUS:
		{
			u8 status;
			u8 ret=0;
			if (copy_from_user(&status, argp, sizeof(u8)))
				return -EFAULT;
			s->state =status;
			if(s->state==SC_STATE_INIT)
			{
				ret=elite_sci_hw_init(s);
				if (ret) 
				{
					SC_PRINT("elite smart card elite_sci_hw_init fail!\n");
				}
			}
			return ret;
		}	
		case SMART_CARD_SETCONFIG:
		{
			switch (s->state)
			{
				case SC_STATE_NOCARD:
				{	
					SC_PRINT("enter no card branch!\n");
					if (copy_from_user(&sc9600_base_config, argp, sizeof(t_sc9600_config)))
						return -EFAULT;
					SC_PRINT("N:0x%x,P:0x%x,B:0x%x,ETU:0X%x\n",sc9600_base_config.n_value,sc9600_base_config.p_value,sc9600_base_config.b_value,sc9600_base_config.etu);
					break;
				}
				case SC_STATE_DEACTIVATED:
				{
					if (copy_from_user(&sc9600_atr_config, argp, sizeof(t_sc9600_config)))
						return -EFAULT;
					break;
				}
					
				case SC_STATE_IDLE:
				{
					if (copy_from_user(&sc9600_current_config, argp, sizeof(t_sc9600_config)))
						return -EFAULT;

					elite_sci_set_userspec_atr(s);
					break;
				}
				default:
				{
					printk(KERN_ERR"state %d is invalid\n",s->state);
					LEAVE();
					return -EINVAL;
				}
			}
		}
		case SMART_CARD_SET_SCRMS:
		{
			u16 scrms=0;
			if (copy_from_user(&scrms, argp, sizeof(u16)))
				return -EFAULT;

			sc_writew( s, scrms&SCRMS_VALUE_MASK,SCRMS);
			break;
		}
		case SMART_CARD_SET_ATR:
		{
			u8 buffer[SC_BUFFER_SIZE];
			if (copy_from_user(buffer, argp, sizeof(buffer)))
				return -EFAULT;

			return elite_sci_process_atr(s, buffer);
		}
		case  SMART_CARD_CHECK_ATR:
		{
			return elite_sci_check_atrconf(s);
		}
		case SMART_CARD_SET_ATRINFO:
		{
			 sc_atr_info_type atr_info;
			if (copy_from_user(&atr_info, argp, sizeof(sc_atr_info_type )))
				return -EFAULT;
			return eliste_sci_cardconf(s, &atr_info);
		} 
		case SMART_CARD_DETECT_ENABLE:
		{
			return elite_sci_hw_detect_enable(s);
		}
		case SMART_CARD_GET_EVENT:
		{
			if(!list_empty(&(s->event_messages)))
			{
				struct sc_message* msg=NULL;
				msg=container_of(s ->event_messages.next, struct sc_message, queue);
				BUG_ON(!msg);
				if (copy_to_user(argp,&(msg->io_resp) , sizeof(struct sc_rx_message)))
				{	
					LEAVE();
					return -EFAULT;
				}
				list_del(s->event_messages.next);
				if(msg)
				{
					kfree(msg);
				}	
			}
			else
			{
			       LEAVE();
			       return -EAGAIN;
			}
		}	
	}
	LEAVE();
	return 0;
}

/**
* elite_sci_fasyn - register the fasync method
* @fd :the smart card file in VS filesystem
* @file : the file description 
* @on : whether or not register this fasync method 
*/
static int elite_sci_fasyn(int fd, struct file *filp, int on)
{
	 struct elite_sci *elist_sci=filp->private_data;
	return fasync_helper(fd, filp, on, &elist_sci->cmd_async);
}

/**
* elite_sci_poll - poll method for FS's polling or select method
* @filp : the file description 
* @wait : wait table of suspended process 
*/
unsigned int elite_sci_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	struct elite_sci *s = filp->private_data;
	ENTER();
	poll_wait(filp, &wait_fd_block, wait);

	if(!list_empty(&(s->messages)))
	{
		mask |=POLL_IN;
	}
   	SC_PRINT("make is POLLIN=%d",(mask & POLL_IN));
	LEAVE();
	return mask|POLL_OUT;
}

/**
* elite_sci_read - read message from the smart card
* @filp : the file description 
* @buf: the buf for receive message in user space 
* @count: the count want to receive 
* @ppos: the current pos of fd 
*/
static ssize_t elite_sci_read(struct file *filp, char __user *buf,
			size_t count, loff_t *ppos)
{
	struct elite_sci *s = filp->private_data;
	int init_count=count;
	
	ENTER();
	if (count == 0)
	{
		LEAVE();
		return 0;
	}
	if ((filp->f_flags & O_NONBLOCK) &&(list_empty(&(s ->messages))))
	{
		LEAVE();
		return -EAGAIN;
	}
	if(wait_event_interruptible(wait_fd_block,!list_empty(&(s ->messages))))
	{
		if (filp->f_flags & O_NONBLOCK)
		{
			LEAVE();
			return -EAGAIN;
		}
		LEAVE();
		return -ERESTARTSYS;
	}

	while((!list_empty(&(s->messages)))&&(count/sizeof(struct sc_rx_message)))
	{
		struct sc_message* msg=NULL;
		unsigned long flags;
		msg=container_of(s ->messages.next, struct sc_message, queue);
		BUG_ON(!msg);
		if (copy_to_user(buf,&(msg->io_resp) , sizeof(struct sc_rx_message)))
		{	
			LEAVE();
			return -EFAULT;
		}
		spin_lock_irqsave(&s->sc_lock ,flags);
		list_del(s ->messages.next);
		spin_unlock_irqrestore(&s->sc_lock, flags);
		if(msg)
		{
			kfree(msg);
		}
		count -=sizeof(struct sc_rx_message);
		*ppos+=sizeof(struct sc_rx_message);
	}

	LEAVE();
	return init_count-count;
}

/**
* elite_sci_write - write message to the smart card
* @filp : the file description 
* @buf: the buf contain message want to write in user space 
* @count: the count want to write 
* @ppos: the current pos of fd 
*/
static ssize_t elite_sci_write(struct file *filp, const char __user *buf,
												size_t count, loff_t *ppos)
{
	struct elite_sci *s = filp->private_data;
	int init_count=count;
	BUG_ON(!s);
	ENTER();
	if (count == 0)
	{
		LEAVE();
		return 0;
	}
	if (down_interruptible (&s ->sem))
	{
		LEAVE();
		return -ERESTARTSYS;
	}
   	SC_PRINT("count=%d,buf=0x%x\n",count,buf);
	while(count/sizeof(struct sc_tx_message))
	{
		struct	sc_tx_message tx_msg;
		if (copy_from_user(&tx_msg, buf, sizeof(struct sc_tx_message)))
		{	
			up(&(s->sem));
			LEAVE();
			return -EFAULT;
		}
		/*{
			int i=0;
			SC_PRINT("tx_msg.size %d",tx_msg.size);
			for(i=0;i<tx_msg.size;i++)
			{
				SC_PRINT("0x%x",tx_msg.buffer[i]);  
			}
			SC_PRINT("\n"); 
		}*/
		elite_sci_hw_io_command(s,tx_msg.buffer, tx_msg.size,tx_msg.reply_size);
		count -=sizeof(struct  sc_tx_message);
		*ppos+=sizeof(struct  sc_tx_message);
	}
	up(&(s ->sem));
	LEAVE();
	return	 init_count-count;
}

static struct file_operations elite_sci_fops = 
{
	.owner	 = THIS_MODULE,
	.open	 = elite_sci_open,
	.release = elite_sci_release,
	.unlocked_ioctl   = elite_sci_unlocked_ioctl,
	.fasync=elite_sci_fasyn,
	.poll=elite_sci_poll,
	.read		= elite_sci_read,
	.write		= elite_sci_write,
#if defined(__x86_64__) && defined(HAVE_COMPAT_IOCTL)
	.compat_ioctl = elite_sci_unlocked_ioctl,
#endif
};

/**
* elite_sci_probe - probe method for driver to find device
* @pdev : the smart card dev 
*/
static int elite_sci_probe(struct platform_device *pdev)
{
	int ret=0;
	dev_t devno = 0;
	struct resource * res;

	struct elite_sci *s;
	ENTER();
	s = kzalloc(sizeof(struct elite_sci), GFP_KERNEL);

	if (!s) {
		printk(KERN_ERR"elite smart card interface driver allocate memory for elite_sci fail!\n");
		ret = -ENOMEM;
		goto fail;
	}

	s->dma_size = (SC_BUFFER_SIZE*2 + (PAGE_SIZE-1)) & (~(PAGE_SIZE-1));
	s->dma_buffer = dma_alloc_coherent(&pdev->dev, s->dma_size, &s->dma_handle, GFP_KERNEL);
	if (!s) {
		printk(KERN_ERR"elite smart card interface driver allocate dma buffer fail!\n");
		ret = -ENOMEM;
		goto fail;
	}

	s->tx_buffer = s->dma_buffer;
	s->rx_buffer = s->dma_buffer + SC_BUFFER_SIZE;

	s->tx_buffer_phy = s->dma_handle;
	s->rx_buffer_phy = s->dma_handle + SC_BUFFER_SIZE;
    	printk(KERN_INFO"rx_buffer:0x%x rx_buffer_phy:0x%x tx_buffer:0x%x tx_buffer_phy:0x%x \n",s->rx_buffer,s->rx_buffer_phy,s->tx_buffer,s->tx_buffer_phy);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (unlikely(!res))
	{
		printk(KERN_ERR"elite smart card interface get ioresource_mem fail!\n");
		ret = -ENXIO;
		goto fail;
	}
	s->mmio   = (void __iomem *)ioremap(res->start,0x30);

	//s->mmio = (unsigned char *)res->start;
   	printk(KERN_ERR"elite smart card interface get mem start 0x%x\n",s->mmio);
	//s->irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	//if (unlikely(s->irq < 0)) {
	//	printk(KERN_ERR"eltie smart card interface driver get irq fail!\n");
	//	ret = -ENXIO;
	//	goto fail;
	//}
    	s->irq = 99;
	 printk(KERN_ERR"elite smart card interface get IRQ start 0x%x\n",s->irq);

    	devno = MKDEV(ELITE_SCI_MAJOR, 0);	
	ret=register_chrdev_region(devno,1,"elite_sc");
	if(ret)
	{
		printk(KERN_ERR"eltie smart card interface driver get irq fail!\n");
		ret = -EEXIST;
		goto fail;
	}
	cdev_init(&s->cdev, &elite_sci_fops);	
	s->cdev.owner = THIS_MODULE;	
	s->cdev.ops = &elite_sci_fops;		
	ret = cdev_add (&s->cdev, devno, 1);
	if (ret) {
		printk("elite smart card interface register chrdev fail!\n");
		goto fail;
	}
	elite_sc_class = class_create(THIS_MODULE, "sc_class");
	if(IS_ERR(elite_sc_class)) 
	{
		printk("Err: failed in creating class.\n");
		goto fail;
	} 
  
	/* register your own device in sysfs, and this will cause udev to create corresponding device node */
	device_create( elite_sc_class, NULL, MKDEV(ELITE_SCI_MAJOR, 0), NULL,"%s","elite_sc");

	sema_init(&(s->sem),1);
	platform_set_drvdata(pdev, s);
	spin_lock_init(&(s->sc_lock));
	ret = request_irq(s->irq, elite_sci_isr ,IRQF_DISABLED, "elite-sci", s);
	if (ret) {
		printk(KERN_ERR"elite smart card interface driver request irq fail!\n");
		goto fail;
	}
	LEAVE();
	return ret;

fail:
	if (s) {
		if (s->dma_handle) 
		{
			dma_free_coherent(&pdev->dev, s->dma_size, s->dma_buffer, s->dma_handle);
		}
		cdev_del(&s->cdev);
		kfree(s);
	}

	LEAVE();
	return ret;
}

/**
* elite_sci_remove - remove method for driver to remove device
* @pdev : the smart card dev 
*/
static int elite_sci_remove(struct platform_device *pdev)
{
	struct elite_sci *elist_sci = platform_get_drvdata(pdev);
	ENTER();
	BUG_ON(!elist_sci);
	free_irq(elist_sci->irq, NULL);
	cdev_del(&(elist_sci->cdev));
	device_destroy(elite_sc_class, MKDEV(ELITE_SCI_MAJOR, 0));        
	class_destroy(elite_sc_class);                              
	unregister_chrdev_region (MKDEV(ELITE_SCI_MAJOR, 0), 1);
	platform_set_drvdata(pdev, NULL);
	kfree(elist_sci);
	LEAVE();
	return 0;
}

/**
* elite_sci_suspend - suspend method for driver to suspend device
* @pdev : the smart card dev 
*/
static int elite_sci_suspend(struct platform_device *pdev,pm_message_t state)
{
	int ret = 0;
	ENTER();

	LEAVE();
	return ret;
}

/**
* elite_sci_resume -suspend method for driver to resume device
* @pdev : the smart card dev 
*/
static int elite_sci_resume(struct platform_device *pdev)
{
	int ret = 0;
	ENTER();

	LEAVE();
	return ret;
}

static struct resource elite_sci_resource[] = {
	[0] = {
		.start = 0xD8160000, //FE360000,
		.end =  0xD8160000 + 0x30, //0xFE360000+ 0x30,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start =99,
		.end = 99,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device elite_sci_device ={	
	.name		= "elite-sci",	  
	.id 			= -1,	
	.num_resources = ARRAY_SIZE(elite_sci_resource),
	.resource         =elite_sci_resource,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_driver elite_sci_driver = {
	.probe	= elite_sci_probe,
	.remove 	= elite_sci_remove,
	.suspend	= elite_sci_suspend,
	.resume 	= elite_sci_resume,
	.driver = {
		.name = "elite-sci",
		.owner = THIS_MODULE,
	}
};

static int elite_sci_init(void)
{
	int ret=0;
	ENTER();
	ret = platform_device_register(&elite_sci_device);	 
	if(ret) 	   
		return ret;
	LEAVE();
	return platform_driver_register(&elite_sci_driver); 
}

static void elite_sci_exit(void)
{	
	ENTER();
	platform_device_unregister(&elite_sci_device);
	platform_driver_unregister(&elite_sci_driver);
	LEAVE();
}

module_init(elite_sci_init);
module_exit(elite_sci_exit);

MODULE_LICENSE("GPL");


