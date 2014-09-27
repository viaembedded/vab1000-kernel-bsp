/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <mach/io.h>
#include <asm/irq.h>
#include <linux/rtc.h>
#include "elite_rtc.h"

#ifndef DPRINTK
#define DPRINTK                 printk
#endif

#define DRIVER_VERSION          "0.54"

#define RTC_DEF_FREQ             (32768)
#define RTC_DEF_DIVIDER        (32768 - 1)
#define IRQ_RTC_ALARM            IRQ_RTC0
#define IRQ_RTC_UPDATE          IRQ_RTC1
/*
 * Following are the bits from a classic RTC we want to mimic
 */
#define RTC_IRQF                       0x80      /* any of the following 3 is active */
#define RTC_PF                           0x40
#define RTC_AF                           0x20
#define RTC_UF                           0x10

static DEFINE_SPINLOCK(elite_rtc_lock);

/*
 * For RTTS and RTCT registers
 */
struct bcd_time_t {
	unsigned int sec:7;            /* second       */
	unsigned int min:7;            /* minute       */
	unsigned int hour:6;           /* hour         */
	unsigned int wday:3;           /* day of week  */
	unsigned int rev:8;            /* reversed     */
	unsigned int flag:1;           /* invalid flag */
};

/*
 * For RTDS and RTCD registers
 */
struct bcd_date_t {
	unsigned int mday:6;   /* day of month     */
	unsigned int mon:5;    /* month            */
	unsigned int year:8;   /* year             */
	unsigned int cen:1;    /* century after 2k */
	unsigned int rev:11;   /* reversed         */
	unsigned int flag:1;   /* invalid flag     */
};

/*
 * For RTAS register
 */
struct bcd_alarm_t {
	unsigned int sec:7;    /* second      */
	unsigned int min:7;    /* minute      */
	unsigned int hour:6;   /* hour        */
	unsigned int mday:6;   /* day of week */
	unsigned int cmp:4;    /* compare     */
	unsigned int rev:2;    /* reversed    */
};

struct elite_alarm_t {
	int type;
	int sec;                        /* seconds after the minute - [0,59] */
	int min;                        /* minutes after the hour - [0,59]   */
	int hour;                       /* hours since midnight - [0,23]     */
	int mday;                       /* day of the month - [1,31]         */
};

/*
 * ELITE RTC operation structure.
 */
struct elite_rtc_t {
	void __iomem           *regs;
	struct resource        *ioarea;
	unsigned int               rtct;
	unsigned int               rtcd;
	unsigned int               rtas;
};

static struct elite_rtc_t rtc;

/*
 * This struct keep the alarm setting.
 */
static struct rtc_time rtc_alarm = {
	.tm_year        = 100,          /* 2000 */
	.tm_mon         = 0,            /* Jan  */
	.tm_mday        = 1,            /* 1st  */
	.tm_hour        = 0,
	.tm_min         = 0,
	.tm_sec         = 0,
};

/* elite_set_time()
 *
 * Write new setting to RTC Time Set Register.
 *
 * Note: Check RTC Write Stauts Register first before writing
 *       new value to RTC Time Set Register.
 */
static int elite_set_time(unsigned int value)
{
	int ret = 0;

	/*
	 * Check for free writing
	 */
	if ((readl(rtc.regs +RTWS_ADDR) & RTWS_TIMESET) == 0) {
		writel((value & RTTS_TIME),(rtc.regs +RTTS_ADDR));
	} else {
		printk(KERN_ERR "rtc_err : RTTS register write busy!\n");
		ret = -EBUSY;
	}
	return ret;
}

/* elite_set_date()
 *
 * Write new setting to RTC Date Set Register.
 *
 * Note: Check RTC Write Stauts Register first before writing
 *       new value to RTC Date Set Register.
 */
static int elite_set_date(unsigned int value)
{
	int ret = 0;

	/*
	 * Check for free writing
	 */
	if ((readl(rtc.regs +RTWS_ADDR) & RTWS_DATESET) == 0)
		writel((value & RTDS_DATE) ,(rtc.regs +RTDS_ADDR) );
	else
		ret = -EBUSY;

	return ret;
}

/* elite_set_alarm()
 *
 * Parsing tm structure and then set to RTAS register.
 *
 * Note: Check RTC Write Stauts Register first before writing
 *       new value to RTC Alarm Set Register.
 *       Current policy only can set the alarm to day of month.
 */
static int elite_set_alarm(struct rtc_time *alarm)
{
	int ret = 0;
	unsigned int value;
	volatile unsigned int i ;

	/*
	 * Following one is a good example if you wanna
	 * see the result in RTAS register.
	 * You can clearly see BCD in time and date structure.
	 */

	/*
	 * Check structure "alarm".
	 */
	value = 0;

	if ((alarm->tm_sec >= 0) && (alarm->tm_sec < 60)) {   /* [0,59] valid */
		value |= RTAS_SEC(alarm->tm_sec);
		value |= RTAS_CMPSEC;
	}

	if ((alarm->tm_min >= 0) && (alarm->tm_min < 60)) {   /* [0,59] valid */
		value |= RTAS_MIN(alarm->tm_min);
		value |= RTAS_CMPMIN;
	}

	if ((alarm->tm_hour >= 0) && (alarm->tm_hour < 24)) { /* [0,23] valid */
		value |= RTAS_HOUR(alarm->tm_hour);
		value |= RTAS_CMPHOUR;
	}

	if ((alarm->tm_mday > 0) && (alarm->tm_mday <= 31)) { /* [1,31] valid */
		value |= RTAS_DAY(alarm->tm_mday);
		value |= RTAS_CMPDAY;
	}

	/*
	 * Update new alarm time.
	 */
	if (value) {
		/*delay waite RTWS_ALARMSET*/
		i=0;
		while((readl(rtc.regs +RTWS_ADDR) & RTWS_ALARMSET) != 0 )
		{
			i++;
			if(i==1000)
				break;
		}
		if ((readl(rtc.regs +RTWS_ADDR) & RTWS_ALARMSET) == 0)    /* write free */
			writel(value,(rtc.regs+RTAS_ADDR));
		else
			ret = -EBUSY;
	}
	return ret;
}

static inline int rtc_periodic_alarm(struct rtc_time *tm)
{
	return  (tm->tm_year == -1) ||
		((unsigned)tm->tm_mon >= 12) ||
		((unsigned)(tm->tm_mday - 1) >= 31) ||
		((unsigned)tm->tm_hour > 23) ||
		((unsigned)tm->tm_min > 59) ||
		((unsigned)tm->tm_sec > 59);
}


static irqreturn_t
elite_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = to_platform_device(dev_id);
	struct rtc_device *elite_rtc = platform_get_drvdata(pdev);
	unsigned int rtis;
	unsigned long events = 0;

	spin_lock(&elite_rtc_lock);

	/*
	 * Save status in a shadow register
	 * Clear interrupt source.
	 */
	rtis = readl(rtc.regs+RTIS_ADDR) ;
	writel(rtis,rtc.regs+RTIS_ADDR);	

	/*
	 * Clear alarm interrupt if it has occurred
	 */
	if (rtis & RTIS_ALARM)
		writel((readl(rtc.regs+RTAS_ADDR) & (~RTAS_CMPMASK)),(rtc.regs+RTAS_ADDR));		
	
	/*
	 * Update irq data & counter
	 */
	if (rtis & RTIS_ALARM)
		events |= (RTC_AF|RTC_IRQF);
	if (rtis & RTIS_UPDATE)
		events |= (RTC_UF|RTC_IRQF);

	rtc_update_irq(elite_rtc, 1, events);         /* in rtctime.c */

	if ((rtis & RTIS_ALARM) && rtc_periodic_alarm(&rtc_alarm))
		elite_set_alarm(&rtc_alarm);   /* in rtctime.c */

	spin_unlock(&elite_rtc_lock);

	return IRQ_HANDLED;
}

static int elite_rtc_open(struct device *dev)
{
	int ret;

	ret = request_irq(IRQ_RTC_UPDATE,
		elite_rtc_interrupt,
		IRQF_DISABLED,
		"rtc_ticks",
		dev);
	if (ret) {
		printk(KERN_ERR "rtc: IRQ%d already in use.\n", IRQ_RTC_UPDATE);
		goto fail_update;
	}

	ret = request_irq(IRQ_RTC_ALARM,
		elite_rtc_interrupt,
		IRQF_DISABLED,
		"rtc_alarm",
		dev);
	if (ret) {
		printk(KERN_ERR "rtc: IRQ%d already in use.\n", IRQ_RTC_ALARM);
		goto fail_alarm;
	}

	return 0;

fail_alarm:
	free_irq(IRQ_RTC_UPDATE, NULL);
fail_update:
	return ret;
}

static void elite_rtc_release(struct device *dev)
{
	spin_lock_irq(&elite_rtc_lock);

	/*
	 * Release RTC resource.
	 */	
	writel( (readl(rtc.regs+RTCC_ADDR) & ( ~RTCC_INTENA)),(rtc.regs+RTCC_ADDR) );
	writel(0,rtc.regs +RTAS_ADDR);
	rtc.rtas = 0;

	spin_unlock_irq(&elite_rtc_lock);

	/*
	 * Release IRQ resource.
	 */
	free_irq(IRQ_RTC_UPDATE, dev);
	free_irq(IRQ_RTC_ALARM, dev);
}

static int elite_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case RTC_AIE_OFF:
	/*
	 * Disable Alarm Interrupt
	 */
		spin_lock_irq(&elite_rtc_lock);
	/*
	 * Save RTAS to shadow register.
	 */
		rtc.rtas = readl(rtc.regs +RTAS_ADDR);		
		writel( (readl(rtc.regs +RTAS_ADDR) & (~RTAS_CMPMASK)  ), (rtc.regs +RTAS_ADDR));
		spin_unlock_irq(&elite_rtc_lock);
		return 0;
	case RTC_AIE_ON:
	/*
	 * Enable Alarm Interrupt
	 */
		spin_lock_irq(&elite_rtc_lock);
	/*
	 * Since the alarm time setting was not changed,
	 * we only recover compare bits from shadow register.
	 */
	    writel( (readl(rtc.regs +RTAS_ADDR) | (RTAS_CMPMASK)  ), (rtc.regs +RTAS_ADDR));
		spin_unlock_irq(&elite_rtc_lock);
		return 0;
	case RTC_UIE_OFF:
	/*
	* Disable Update Interrupt
	*/
		spin_lock_irq(&elite_rtc_lock);		
		writel( (readl(rtc.regs +RTCC_ADDR) & (~RTCC_INTENA)  ), (rtc.regs +RTCC_ADDR));
		spin_unlock_irq(&elite_rtc_lock);
		return 0;
	case RTC_UIE_ON:
	/*
	 * Enable Update Interrupt
	 */
		spin_lock_irq(&elite_rtc_lock);
		writel( (readl(rtc.regs +RTCC_ADDR) | (RTCC_INTENA)  ), (rtc.regs +RTCC_ADDR));
		spin_unlock_irq(&elite_rtc_lock);
		return 0;

	/*
	 * Current RTC driver does not support following ioctls
	 */

	case RTC_PIE_OFF:       /* Disable Periodic Interrupt */
	case RTC_PIE_ON:        /* Enable Periodic Interrupt  */
	case RTC_IRQP_READ:     /* Read IRQ rate              */
	case RTC_IRQP_SET:      /* Set IRQ rate               */
		return -EINVAL;
	}
	return -EINVAL;
}

int elite_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	if (tm == NULL) {      /* for debugging */
		printk(KERN_WARNING "RTC: %s *tm is invalid!\n", __func__);
		return -EINVAL;
	}

	rtc.rtct = readl(rtc.regs +RTCT_ADDR);
	rtc.rtcd = readl(rtc.regs +RTCD_ADDR);

	if ((rtc.rtct & RTCT_INVALID) || (rtc.rtcd & RTCD_INVALID)) {
		printk(KERN_WARNING "RTC: RTCD/RTCT register invalid flag on!\n");
		printk(KERN_WARNING "RTC: RTCD=0x%.8x RTCT=0x%.8x\n",
		       rtc.rtct ,
		       rtc.rtcd);
		return -EINVAL;
	}

	/*
	 * BCD2BIN translation
	 */
	tm->tm_sec  = RTCT_SEC(rtc.rtct);

	tm->tm_min  = RTCT_MIN(rtc.rtct);

	tm->tm_hour = RTCT_HOUR(rtc.rtct);

	tm->tm_wday = RTCT_DAY(rtc.rtct);

	tm->tm_mday = RTCD_MDAY(rtc.rtcd);

	tm->tm_mon  = RTCD_MON(rtc.rtcd) - 1;       /* 0 means January */

	tm->tm_year = RTCD_YEAR(rtc.rtcd) + ((RTCD_CENT(rtc.rtcd)) * 100 + 100);

	return 0;
}

/* elite_rtc_set_time()
 *
 * Setup the RTC date and time.
 *
 * In: tm, a rtc_time structure.
 */
static int elite_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	int ret = 0;
	unsigned int value;

	/*
	 * Following two are good examples if you wanna
	 * see the result in RTTS and RTDS registers.
	 * You can clearly see BCD in time and date structure.
	 */

	if ((tm->tm_sec < 0) || (tm->tm_sec > 59)) {          /* [0,59] valid */
		printk(KERN_ERR "rtc_err : invalid sec\n");
		ret = -EINVAL;
		goto out;
	}

	if ((tm->tm_min < 0) || (tm->tm_min > 59)) {          /* [0,59] valid */
		printk(KERN_ERR "rtc_err : invalid min\n");
		ret = -EINVAL;
		goto out;
	}

	if ((tm->tm_hour < 0) || (tm->tm_hour > 23)) {        /* [0,23] valid */
		printk(KERN_ERR "rtc_err : invalid hour\n");
		ret = -EINVAL;
		goto out;
	}

	if ((tm->tm_wday < 0) || (tm->tm_wday > 6)) {         /* [0,6] valid */
		printk(KERN_ERR "rtc_err : invalid wday\n");
		ret = -EINVAL;
		goto out;
	}

	if ((tm->tm_mday < 0) || (tm->tm_mday > 31)) {        /* [1,31] valid */
		printk(KERN_ERR "rtc_err : invalid mday\n");
		ret = -EINVAL;
		goto out;
	}

	if ((tm->tm_mon < 0) || (tm->tm_mon > 11)) {          /* [0,11] valid */
		printk(KERN_ERR "rtc_err : invalid mon\n");
		ret = -EINVAL;
		goto out;
	}

	if (tm->tm_year < 100) {                              /* >= 100 valid */
		printk(KERN_ERR "rtc_err : invalid year\n");
		ret = -EINVAL;
		goto out;
	}

	value = RTTS_SEC(tm->tm_sec) | RTTS_MIN(tm->tm_min) | \
		RTTS_HOUR(tm->tm_hour) | RTTS_DAY(tm->tm_wday);

	ret = elite_set_time(value);

	if (ret)
		goto out;

	value = RTDS_MDAY(tm->tm_mday) | RTDS_MON(tm->tm_mon + 1) | \
		RTDS_YEAR(tm->tm_year%100) | RTDS_CENT((tm->tm_year/100) - 1);

	ret = elite_set_date(value);
out:
	return ret;
}

static int elite_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	memcpy(&alrm->time, &rtc_alarm, sizeof(struct rtc_time));
	alrm->pending = ((readw(rtc.regs+RTIS_ADDR) & RTIS_ALARM) ? 1 : 0);
	return 0;
}

/*
 * Handle wakeup alarm API information defined in include/linux/rtc.h
 */
int elite_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	//;elite1k-520016e-Ned-05-start
	struct rtc_wkalrm alm_now;
	int status;
	status = elite_rtc_read_time(NULL, &alm_now.time);
	if (status < 0) {
		return;
	}
	unsigned long new;
	unsigned long now;
	rtc_tm_to_time(&alrm->time, &new);
	rtc_tm_to_time(&alm_now.time, &now);

	if(new - now < 10)
	{
		return -1;
	}
	//;elite1k-520016e-Ned-05-end
	int ret;

	spin_lock_irq(&elite_rtc_lock);

	ret = elite_set_alarm(&alrm->time);

	if (ret == 0) {
		memcpy(&rtc_alarm, &alrm->time, sizeof(struct rtc_time));

		if (alrm->enabled)
			enable_irq_wake(IRQ_RTC_ALARM);
		else
			disable_irq_wake(IRQ_RTC_ALARM);
	}
	spin_unlock_irq(&elite_rtc_lock);

	return ret;
}

static int elite_rtc_proc(struct device *dev, struct seq_file *seq)
{
	int is_testmode = (RTTM_VAL & RTTM_ENABLE);
	seq_printf(seq, "test_mode\t: %s\n",
    		(is_testmode) ? "enabled" : "disabled");
    
	seq_printf(seq, "%s\t: 0x%08x\n",
    		((is_testmode) ? "divider" : "calibration"),
    		RTTC_VAL);
	
	seq_printf(seq, "alarm_IRQ\t: %s\n",
    		(RTIS_VAL & RTIS_ALARM) ? "yes" : "no");
	
	seq_printf(seq, "ticks_IRQ\t: %s\n",
    		(RTIS_VAL & RTIS_UPDATE) ? "yes" : "no");
    
	return 0;

}
static int elite_rtc_alarm_irq(struct device *dev,  unsigned int enabled)
{
	if(enabled==1){
		/*
		 * Enable Alarm Interrupt
		 */
		spin_lock_irq(&elite_rtc_lock);
		/*
		 * Since the alarm time setting was not changed,
		 * we only recover compare bits from shadow register.
		 */
	   	 writel( (readl(rtc.regs +RTAS_ADDR) | (RTAS_CMPMASK)  ), (rtc.regs +RTAS_ADDR));
		spin_unlock_irq(&elite_rtc_lock);

	}
	else {
		/*
	 	* Disable Alarm Interrupt
		 */
		spin_lock_irq(&elite_rtc_lock);
		/*
		 * Save RTAS to shadow register.
	 	*/
		rtc.rtas = readl(rtc.regs +RTAS_ADDR);		
		writel( (readl(rtc.regs +RTAS_ADDR) & (~RTAS_CMPMASK)  ), (rtc.regs +RTAS_ADDR));
		spin_unlock_irq(&elite_rtc_lock);
	}
	return 0;
}


/*
 * Wrap "rtc_ops" to "file_operations" in common/rtctime.c
 */
static const struct rtc_class_ops elite_rtc_ops = {
	.open           = elite_rtc_open,
	.release        = elite_rtc_release,
	.ioctl          = elite_rtc_ioctl,
	.read_time      = elite_rtc_read_time,
	.set_time       = elite_rtc_set_time,
	.read_alarm     = elite_rtc_read_alarm,
	.set_alarm      = elite_rtc_set_alarm,
	.proc           = elite_rtc_proc,
	.alarm_irq_enable     = elite_rtc_alarm_irq,
};

static int rtc_on = 1;

static int __init no_rtc(char *str)
{
	rtc_on = 0;
	return 1;
}

__setup("nortc", no_rtc);

static int elite_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *elite_rtc;
	struct resource *res;
	int ret = 0;
	
	/* get device io memory resource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL){
		dev_err(&pdev->dev, "cannot find IO resource");
		ret = -ENOENT;
	}

	rtc.ioarea = request_mem_region(res->start, resource_size(res), pdev->name);
	if (rtc.ioarea == NULL){
		dev_err(&pdev->dev, "cannot request mem\n");
		ret = -ENXIO;
	}

	rtc.regs = ioremap(res->start, resource_size(res));
	if (rtc.regs == NULL){
		dev_err(&pdev->dev, "cannot map IO\n");
		ret = -ENXIO;
		goto err_ioarea;
	}	

	if ((rtc_on) && (RTC_ENABLE > 0x34260102)) {
		
		printk("elite_rtc on\n");
		
		if ((readl(rtc.regs+RTTM_ADDR) & RTTM_ENABLE) && (readl(rtc.regs+RTTC_ADDR) == 0)) {
			
			printk(KERN_WARNING "rtc_warn: initializing default clock divider value\n");
			writel(RTC_DEF_DIVIDER,rtc.regs+RTTC_ADDR);
		
		}

		if ((readl(rtc.regs+RTCC_ADDR) & RTCC_ENA) == 0) {
			
			/*
			* Reset RTC alarm settings
			*/            
		 	writel(0,rtc.regs+RTAS_ADDR);	

			/*
			* Reset RTC test mode.
			*/
			writel(0,rtc.regs+RTTM_ADDR);		
			/*
			* Adjust RTC clock
			*/
			writel(0x8300,rtc.regs+RTTC_ADDR);	
			printk(KERN_ERR"elite-rtc RTTC is %x",readl(rtc.regs+RTTC_ADDR));

			/*
             			* Disable all RTC control functions.
			* Set to 24-hr format and update type to each second.
			* Disable sec/min update interrupt.
			* Let RTC free run without interrupts.
			*/
			writel( (RTCC_ENA | RTCC_INTTYPE),rtc.regs+RTCC_ADDR);			

			if (readl(rtc.regs+RTCD_ADDR)== 0) {
				while (readl(rtc.regs+RTWS_ADDR) & RTWS_DATESET);
				writel(readl(rtc.regs+RTDS_ADDR),rtc.regs+RTDS_ADDR);			
			}
		}
		
		device_init_wakeup(&pdev->dev, 1);
    
		elite_rtc = rtc_device_register(pdev->name, &pdev->dev, &elite_rtc_ops, THIS_MODULE);

#ifndef CONFIG_SKIP_DRIVER_MSG
		printk(KERN_INFO "ELITE Real Time Clock driver v" DRIVER_VERSION " initialized: %s\n",
			elite_rtc ? "ok" : "failed");
#endif

		if (IS_ERR(elite_rtc))
			return PTR_ERR(elite_rtc);
    
		platform_set_drvdata(pdev, elite_rtc);
	}

	dev_info(&pdev->dev," driver probed done\n");
	
	return 0;

err_ioremap:
	iounmap(rtc.regs);

err_ioarea:
	release_resource(rtc.ioarea);
	kfree(rtc.ioarea);
	
	return ret;
	
}

static int elite_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *elite_rtc = platform_get_drvdata(pdev);                
	
 	if (elite_rtc) {
	    writel((readl(rtc.regs+RTCC_ADDR) &(~RTCC_INTENA)),rtc.regs+RTCC_ADDR);	
	    rtc_device_unregister(elite_rtc);
 	}
	
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int elite_rtc_suspend(struct device *dev)
{
	if (device_may_wakeup(dev))
		enable_irq_wake(IRQ_RTC_ALARM);
	return 0;
}

static int elite_rtc_resume(struct device *dev)
{
	if (device_may_wakeup(dev))
		disable_irq_wake(IRQ_RTC_ALARM);
	return 0;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int elite_rtc_runtime_suspend(struct device *dev)
{
	return 0;
}
static int elite_rtc_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops elite_rtc_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(elite_rtc_suspend, elite_rtc_resume)
	SET_RUNTIME_PM_OPS(elite_rtc_runtime_suspend,
				elite_rtc_runtime_resume, NULL)
};

#ifdef CONFIG_OF
static const struct of_device_id elite_rtc_match[] = {
	{ .compatible = "s3graphics,elite1000-rtc" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, elite_rtc_match);
#endif

static struct platform_device_id elite_rtc_driver_ids[] = {
	{
		.name  = "elite-rtc",
		.driver_data	= (kernel_ulong_t)NULL,
	}, {
		.name  = "elite-rtc.0",
		.driver_data	= (kernel_ulong_t)NULL,
	}, 	{ /* sentinel */ },
};

static struct platform_driver elite_rtc_driver = {
	.probe		= elite_rtc_probe,
	.remove		= elite_rtc_remove,
	.suspend	= elite_rtc_suspend,
	.resume		= elite_rtc_resume,
	.id_table	= elite_rtc_driver_ids,
	.driver		= {
		.name		= "s3graphics-elite-rtc",
		.pm		= &elite_rtc_dev_pm_ops,
		.of_match_table = of_match_ptr(elite_rtc_match),
	},
};

module_platform_driver(elite_rtc_driver);

MODULE_DESCRIPTION("ELITE Real Time Clock Driver");
MODULE_LICENSE("GPL");
