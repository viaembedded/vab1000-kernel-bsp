/*
 * linux/drivers/char/watchdog/elite_wdt.c
 *
 * Copyright (C) 2012 S3 Graphics, Inc.
 *
 * This program is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 2 of the License, or 
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/slab.h>
#define  WDT_SEC    250000
#define  WDT_TEN_SEC         2500000     

//PMU_WD_FULLCHIP_RESET_EN
#define ENABLE_WDT     *(unsigned int*)0xfe39cb10
static unsigned int wdt_timeout = WDT_TEN_SEC;
static bool nowayout  = WATCHDOG_NOWAYOUT;
static DEFINE_SPINLOCK(io_lock);
#define	PMP_WDTC_ADDR          (0x00)
#define	PMP_WDMOD_ADDR       (0x04)
#define	PMP_WDFEED_ADDR      (0x08)
#define	PMP_WDTV_ADDR          (0x0C)
/*
 * ELITE watch dog operation structure.
 */
struct elite_wdt_t {
	void __iomem           *wdt_base;
	struct resource        *ioarea;
};
static struct elite_wdt_t wdt;
#define ELITE_WDT_DEBUGINFO	0
static int elite_wdt_ping(struct watchdog_device *wdd)
{

#if ELITE_WDT_DEBUGINFO
	printk("wdt_ping [%x %x %x %x %x]\n",readl(wdt.wdt_base+PMP_WDTC_ADDR),readl(wdt.wdt_base+PMP_WDMOD_ADDR),readl(wdt.wdt_base+PMP_WDFEED_ADDR),readl(wdt.wdt_base+PMP_WDTV_ADDR),readl(0xfe39cb10));
#endif 

	spin_lock(&io_lock);

	ENABLE_WDT=0x0;//patch for elite wdt
	
	writel(0xaa,wdt.wdt_base+PMP_WDFEED_ADDR);
	writel(0x55,wdt.wdt_base+PMP_WDFEED_ADDR);
	writel(0x3,wdt.wdt_base+PMP_WDMOD_ADDR);
	
	ENABLE_WDT=0x1;//patch for elite wdt
	
	spin_unlock(&io_lock);
	
#if ELITE_WDT_DEBUGINFO	
	printk("         [%x %x %x %x %x]\n\n",readl(wdt.wdt_base+PMP_WDTC_ADDR),readl(wdt.wdt_base+PMP_WDMOD_ADDR),readl(wdt.wdt_base+PMP_WDFEED_ADDR),readl(wdt.wdt_base+PMP_WDTV_ADDR),readl(0xfe39cb10));
#endif 

	return 0;
}

static int elite_wdt_start(struct watchdog_device *wdd)
{

#if ELITE_WDT_DEBUGINFO
	printk("wdt_start [%x %x %x %x %x]\n",readl(wdt.wdt_base+PMP_WDTC_ADDR),readl(wdt.wdt_base+PMP_WDMOD_ADDR),readl(wdt.wdt_base+PMP_WDFEED_ADDR),readl(wdt.wdt_base+PMP_WDTV_ADDR),readl(0xfe39cb10));
#endif 

	spin_lock(&io_lock);	
	
	ENABLE_WDT=0x0;//patch for elite wdt
	
	writel(0x0,wdt.wdt_base+PMP_WDMOD_ADDR);
	writel(0xaa,wdt.wdt_base+PMP_WDFEED_ADDR);
	writel(0x56,wdt.wdt_base+PMP_WDFEED_ADDR);

	ENABLE_WDT=0x1;//enable wdt reset
	
	writel(0x3,wdt.wdt_base+PMP_WDMOD_ADDR);
	writel(0xaa,wdt.wdt_base+PMP_WDFEED_ADDR);
	writel(0x55,wdt.wdt_base+PMP_WDFEED_ADDR);
	
	spin_unlock(&io_lock);
	
#if ELITE_WDT_DEBUGINFO	
	printk("          [%x %x %x %x %x]\n\n",readl(wdt.wdt_base+PMP_WDTC_ADDR),readl(wdt.wdt_base+PMP_WDMOD_ADDR),readl(wdt.wdt_base+PMP_WDFEED_ADDR),readl(wdt.wdt_base+PMP_WDTV_ADDR),readl(0xfe39cb10));
#endif 

	return 0;
}

static int elite_wdt_stop(struct watchdog_device *wdd)
{

#if ELITE_WDT_DEBUGINFO
	printk("wdt_stop [%x %x %x %x %x]\n",readl(wdt.wdt_base+PMP_WDTC_ADDR),readl(wdt.wdt_base+PMP_WDMOD_ADDR),readl(wdt.wdt_base+PMP_WDFEED_ADDR),readl(wdt.wdt_base+PMP_WDTV_ADDR),readl(0xfe39cb10));
#endif 

	spin_lock(&io_lock);

	ENABLE_WDT=0x0;//disable wdt reset
	
	spin_unlock(&io_lock);
	
#if ELITE_WDT_DEBUGINFO	
	printk("         [%x %x %x %x %x]\n\n",readl(wdt.wdt_base+PMP_WDTC_ADDR),readl(wdt.wdt_base+PMP_WDMOD_ADDR),readl(wdt.wdt_base+PMP_WDFEED_ADDR),readl(wdt.wdt_base+PMP_WDTV_ADDR),readl(0xfe39cb10));
#endif 

	return 0;
}
static int elite_wdt_set_timeout(struct watchdog_device *wdd, unsigned timeout)
{

#if ELITE_WDT_DEBUGINFO
	printk("set_timeout [%x %x %x %x %x]\n",readl(wdt.wdt_base+PMP_WDTC_ADDR),readl(wdt.wdt_base+PMP_WDMOD_ADDR),readl(wdt.wdt_base+PMP_WDFEED_ADDR),readl(wdt.wdt_base+PMP_WDTV_ADDR),readl(0xfe39cb10));
	printk("elite_wdt_set_timeout timeout is %d\n",timeout);
#endif 

	if (timeout < 1)
		return -EINVAL;	

	wdt_timeout=timeout*WDT_SEC;	
	writel(wdt_timeout,wdt.wdt_base+PMP_WDTC_ADDR);
	wdd->timeout=timeout;
	
#if ELITE_WDT_DEBUGINFO	
	printk("            [%x %x %x %x %x]\n\n",readl(wdt.wdt_base+PMP_WDTC_ADDR),readl(wdt.wdt_base+PMP_WDMOD_ADDR),readl(wdt.wdt_base+PMP_WDFEED_ADDR),readl(wdt.wdt_base+PMP_WDTV_ADDR),readl(0xfe39cb10));
#endif 

	return 0;
}

static const struct watchdog_info elite_wdt_info = {
	.options = WDIOF_SETTIMEOUT |WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "elite Watchdog",
};

static const struct watchdog_ops elite_wdt_ops = {
	.owner = THIS_MODULE,
	.start = elite_wdt_start,
	.stop = elite_wdt_stop,
	.ping = elite_wdt_ping,
	.set_timeout= elite_wdt_set_timeout, //设置时间间隔

};

static struct watchdog_device elite_wdt_dev = {
	.info = &elite_wdt_info,
	.ops = &elite_wdt_ops,
};

static int elite_wdt_probe(struct platform_device *pdev)
{
	printk("elite_wdt_probe \n");

	wdt.ioarea = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (wdt.ioarea  == NULL) {
		dev_err(pdev, "no memory resource specified\n");
		return -ENOENT;
	}
	
	wdt.wdt_base = devm_request_and_ioremap(&pdev->dev, wdt.ioarea);
	if (!wdt.wdt_base){
		dev_err(pdev, "request resource and io failed\n");
		return -ENOMEM;
	}
	elite_wdt_dev.max_timeout=10;
	elite_wdt_dev.min_timeout=1;
	
	writel(wdt_timeout,wdt.wdt_base+PMP_WDTC_ADDR);
	elite_wdt_set_timeout(&elite_wdt_dev,10);
	watchdog_set_nowayout(&elite_wdt_dev, nowayout);	
	return watchdog_register_device(&elite_wdt_dev);
}

static int elite_wdt_remove(struct platform_device *pdev)
{
	printk("elite_wdt_remove \n");
	watchdog_unregister_device(&elite_wdt_dev);
	return 0;
}

#ifdef CONFIG_PM
static int elite_wdt_suspend(struct platform_device *dev, pm_message_t state)
{
	/* Save watchdog state, and turn it off. */
	

	/* Note that WTCNT doesn't need to be saved. */
	

	return 0;
}

static int elite_wdt_resume(struct platform_device *dev)
{
	/* Restore watchdog state. */

	

	return 0;
}

#else
#define elite_wdt_suspend NULL
#define elite_wdt_resume  NULL
#endif /* CONFIG_PM */

/***************************************************************************
	platform device struct define
****************************************************************************/
static struct platform_device elite_wdt_device = {
    .name           = "elite-wdt",
    .id             = 0,    
};
static struct platform_driver elite_wdt_driver = {
	.probe		= elite_wdt_probe,
	.remove		= __devexit_p(elite_wdt_remove),
	.suspend	= elite_wdt_suspend,
	.resume		= elite_wdt_resume,
	.driver		= {
		.name	= "elite-wdt",
		.owner	= THIS_MODULE,
	},
};
static int __init elite_wdt_init(void)
{
	printk("elite Watchdog Timer init\n");
	int ret;
   
	ret = platform_driver_register(&elite_wdt_driver);
	if(ret)
	{
		printk("[elite_rtc_init]Error: Can not register ELITE RTC driver\n");
		platform_device_unregister(&elite_wdt_device);
        return ret;
    }
    return 0;
}

static void __exit elite_wdt_exit(void)
{
	printk("elite Watchdog Timer exit\n");
	platform_driver_unregister(&elite_wdt_driver);
	platform_device_unregister(&elite_wdt_device);
}


module_init(elite_wdt_init);
module_exit(elite_wdt_exit);

MODULE_DESCRIPTION("ELITE Watch Dog Driver");
MODULE_LICENSE("GPL");


