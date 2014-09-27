/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <media/rc-core.h>
#include <media/gpio-ir-recv.h>
#include <mach/wakeup_types.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "./elite-gpio-ir-recv.h"

static struct elite_gpio_rc_dev *static_elite_gpio_dev = NULL;

static irqreturn_t elite_gpio_ir_recv_irq(int irq, void *dev_id)
{
	int gval;
	int rc = 0;
	int  irq_id = (int)dev_id;
	enum raw_event_type type = IR_SPACE;

	if (irq_id == ELITE_GPIO_INT_RISING)
	{
		gval = 1;
	}
	else if (irq_id == ELITE_GPIO_INT_FALLING)
	{
		gval = 0;
	}
	else
	{
		goto err_get_value;
	}

	if (static_elite_gpio_dev->active_low)
		gval = !gval;

	if (gval == 1)
		type = IR_PULSE;
    
	rc = ir_raw_event_store_edge(static_elite_gpio_dev->rcdev, type);
	if (rc < 0)
		goto err_get_value;

	ir_raw_event_handle(static_elite_gpio_dev->rcdev); 

err_get_value:
	return IRQ_HANDLED;
}

/* enable & status bits */
#define WAKEUP_SD3_CARD_DETECT_STAT_EN	(1UL<<30)
#define WAKEUP_SD0_CARD_DETECT_STAT_EN	(1UL<<29)
#define WAKEUP_CIR_STAT_EN				(1UL<<22)
#define WAKEUP_USB_DEV_ATTACH_STAT_EN	(1UL<<21)
#define WAKEUP_USB_HOST_STAT_EN			(1UL<<20)
#define WAKEUP_RTC_STAT_EN				(1UL<<15)
#define WAKEUP_POWER_BTN_STAT_EN		(1UL<<14)
#define WAKEUP_INTERNAL_IP3_STAT_EN		(1UL<<7)
#define WAKEUP_INTERNAL_IP2_STAT_EN		(1UL<<6)
#define WAKEUP_INTERNAL_IP1_STAT_EN		(1UL<<5)
#define WAKEUP_INTERNAL_IP0_STAT_EN		(1UL<<4)
#define WAKEUP_GPIO3_STAT_EN			(1UL<<3)
#define WAKEUP_GPIO2_STAT_EN			(1UL<<2)
#define WAKEUP_GPIO1_STAT_EN			(1UL<<1)
#define WAKEUP_GPIO0_STAT_EN			(1UL<<0)

static int wakeup_bits[] = {
	WAKEUP_GPIO0_STAT_EN,
    WAKEUP_GPIO1_STAT_EN,
    WAKEUP_GPIO2_STAT_EN,
    WAKEUP_GPIO3_STAT_EN,
    WAKEUP_INTERNAL_IP0_STAT_EN,
    WAKEUP_INTERNAL_IP1_STAT_EN,
    WAKEUP_INTERNAL_IP2_STAT_EN,
    WAKEUP_INTERNAL_IP3_STAT_EN,
    WAKEUP_POWER_BTN_STAT_EN,
    WAKEUP_RTC_STAT_EN,
    WAKEUP_USB_HOST_STAT_EN,
    WAKEUP_USB_DEV_ATTACH_STAT_EN,
    WAKEUP_CIR_STAT_EN,
    WAKEUP_SD0_CARD_DETECT_STAT_EN,
    WAKEUP_SD3_CARD_DETECT_STAT_EN,
};
static int last_wakeup_type = 0;
static char *last_wakeup_proc_name="last_wakeup";
static struct proc_dir_entry *last_wakeup_proc_entry;
extern unsigned int last_wakeup_status;

static ssize_t elite_last_wakeup_show(struct seq_file *m, void *v)
{
	int i;
	for(i = 0; i < (sizeof(wakeup_bits)/sizeof(wakeup_bits[0]));i++)
	{
		if (last_wakeup_status&wakeup_bits[i])
		{
			last_wakeup_type = i + 1;
			break;
		}
	}	
	seq_printf(m, "%d\n", last_wakeup_type);
	
	return 0;
}

static int proc_elite_last_wakeup_open(struct inode *inode, struct file *file)
{
    return single_open(file, elite_last_wakeup_show, PDE(inode)->data);
}

static const struct file_operations elite_last_wakeup_proc_ops = {
    .owner = THIS_MODULE,
    .open = proc_elite_last_wakeup_open,
    .read = seq_read,
    .release = single_release,
};

int elite_last_wakeup_create_procfs(void)
{
    last_wakeup_proc_entry = proc_create_data(last_wakeup_proc_name, 0444, NULL, &elite_last_wakeup_proc_ops, NULL);
    
    if (!last_wakeup_proc_entry) 
        return -1;

    return 0;
}

static int __devinit elite_gpio_ir_recv_probe(struct platform_device *pdev)
{
	int rc;
	struct rc_dev *rcdev = NULL;
	struct elite_gpio_rc_dev *elite_gpio_rc_dev;
	const struct elite_gpio_ir_recv_platform_data *pdata =
					pdev->dev.platform_data;

	if (!pdata)
	{   
		dev_err(&pdev->dev, "failed to get platform_data\n");
		return -EINVAL;
	}
       
	if (pdata->gpio_rising < 0 ||pdata->gpio_falling < 0 )
	{   
		dev_err(&pdev->dev, " Invalid argument\n");
		return -EINVAL;
	}

	elite_gpio_rc_dev = kzalloc(sizeof(struct elite_gpio_rc_dev), GFP_KERNEL);
	if (!elite_gpio_rc_dev)
		return -ENOMEM;
	
	static_elite_gpio_dev = elite_gpio_rc_dev;
	
	rcdev = rc_allocate_device();
	if (!rcdev) {
		rc = -ENOMEM;
		goto err_allocate_device;
	}

	rcdev->driver_type = RC_DRIVER_IR_RAW;
	rcdev->allowed_protos = RC_TYPE_RC5 | RC_TYPE_NEC | RC_TYPE_OCN;
	rcdev->input_name = ELITE_GPIO_IR_DEVICE_NAME;
	rcdev->input_id.bustype = BUS_HOST;
	rcdev->driver_name = ELITE_GPIO_IR_DRIVER_NAME;
	rcdev->map_name = RC_MAP_ELITE_TV;

	elite_gpio_rc_dev->rcdev = rcdev;
	elite_gpio_rc_dev->gpio_rising= pdata->gpio_rising;
	elite_gpio_rc_dev->gpio_falling= pdata->gpio_falling;
	elite_gpio_rc_dev->active_low = pdata->active_low;
	rc = gpio_request(pdata->gpio_rising, "elite-gpio-ir-rising");
	if (rc < 0)
	{
		goto err_gpio_request1;
	}

	rc  = gpio_direction_input(pdata->gpio_rising);
	if (rc < 0)
	{               
		goto err_gpio_direction_input1;
	}

	rc = gpio_request(pdata->gpio_falling, "elite-gpio-ir-falling");
	if (rc < 0)
		goto err_gpio_request2;

	rc  = gpio_direction_input(pdata->gpio_falling);   
	if (rc < 0)
		goto err_gpio_direction_input2;

	rc = rc_register_device(rcdev);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to register rc device\n");
		goto err_register_rc_device;
	}

	platform_set_drvdata(pdev, elite_gpio_rc_dev);

	rc = request_any_context_irq(gpio_to_irq(pdata->gpio_rising),
				elite_gpio_ir_recv_irq,
			 IRQF_TRIGGER_RISING ,
					"elite-gpio-ir-rising", (void *)ELITE_GPIO_INT_RISING);

	if (rc < 0)
	{      
		goto err_request_irq1;
	}

	rc = request_any_context_irq(gpio_to_irq(pdata->gpio_falling),
			elite_gpio_ir_recv_irq,
			IRQF_TRIGGER_FALLING,
			"elite-gpio-ir-falling", (void *)ELITE_GPIO_INT_FALLING);
	if (rc < 0)
	{   
		goto err_request_irq2;
	}
	
#ifdef CONFIG_PROC_FS
	elite_last_wakeup_create_procfs();
#endif

	printk("Elite GPIO IR Driver Probe OK!\n");	
	return 0;

err_request_irq2:
	free_irq(gpio_to_irq(pdata->gpio_rising),ELITE_GPIO_INT_RISING);
err_request_irq1:
	platform_set_drvdata(pdev, NULL);
	rc_unregister_device(rcdev);
err_register_rc_device:
err_gpio_direction_input2:
	gpio_free(pdata->gpio_falling);
err_gpio_direction_input1:
err_gpio_request2:
	gpio_free(pdata->gpio_rising);
err_gpio_request1:
	rc_free_device(rcdev);
	rcdev = NULL;
err_allocate_device:
	kfree(elite_gpio_rc_dev);
	static_elite_gpio_dev = NULL;
	return rc;

}

static int __devexit elite_gpio_ir_recv_remove(struct platform_device *pdev)
{
	struct elite_gpio_rc_dev *elite_gpio_rc_dev = platform_get_drvdata(pdev);

	free_irq(gpio_to_irq(elite_gpio_rc_dev->gpio_rising), (void *)ELITE_GPIO_INT_RISING);
	free_irq(gpio_to_irq(elite_gpio_rc_dev->gpio_falling), (void *)ELITE_GPIO_INT_FALLING);
	platform_set_drvdata(pdev, NULL);
	rc_unregister_device(elite_gpio_rc_dev->rcdev);
	gpio_free(elite_gpio_rc_dev->gpio_rising);
 	gpio_free(elite_gpio_rc_dev->gpio_falling);   
	rc_free_device(elite_gpio_rc_dev->rcdev);
	kfree(elite_gpio_rc_dev);  
	return 0;
}

#ifdef CONFIG_PM
static int elite_gpio_ir_recv_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct elite_gpio_rc_dev *elite_gpio_rc_dev = platform_get_drvdata(pdev);

	if(device_may_wakeup(dev))
	{   
		enable_irq_wake(gpio_to_irq(elite_gpio_rc_dev->gpio_rising));
 		enable_irq_wake(gpio_to_irq(elite_gpio_rc_dev->gpio_falling));       
	}
	else
	{   
		disable_irq(gpio_to_irq(elite_gpio_rc_dev->gpio_rising));
		disable_irq(gpio_to_irq(elite_gpio_rc_dev->gpio_falling));
	}

	return 0;
}

static int elite_gpio_ir_recv_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct elite_gpio_rc_dev *elite_gpio_rc_dev = platform_get_drvdata(pdev);

	if (device_may_wakeup(dev))
	{   
		disable_irq_wake(gpio_to_irq(elite_gpio_rc_dev->gpio_rising));
		disable_irq_wake(gpio_to_irq(elite_gpio_rc_dev->gpio_falling));
	}
	else
	{   
		enable_irq(gpio_to_irq(elite_gpio_rc_dev->gpio_rising));
		enable_irq(gpio_to_irq(elite_gpio_rc_dev->gpio_falling));
	}
	/*wake up android */
	input_report_key(elite_gpio_rc_dev->rcdev->input_dev, 538, 1);
	input_report_key(elite_gpio_rc_dev->rcdev->input_dev, 538, 0);
	input_sync(elite_gpio_rc_dev->rcdev->input_dev);

	return 0;
}

static const struct dev_pm_ops elite_gpio_ir_recv_pm_ops = {
	.suspend        = elite_gpio_ir_recv_suspend,
	.resume         = elite_gpio_ir_recv_resume,
};
#endif

static struct platform_driver elite_gpio_ir_recv_driver = {
	.probe  = elite_gpio_ir_recv_probe,
	.remove = __devexit_p(elite_gpio_ir_recv_remove),
	.driver = {
		.name   = ELITE_GPIO_IR_DRIVER_NAME,
		.owner  = THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &elite_gpio_ir_recv_pm_ops,
#endif
	},
};

static int __init elite_gpio_ir_recv_init(void)    
{
	platform_device_register(&elite_gpio_rc_device);
	return platform_driver_register(&elite_gpio_ir_recv_driver);
}
module_init(elite_gpio_ir_recv_init);

static void __exit elite_gpio_ir_recv_exit(void)
{
	platform_driver_unregister(&elite_gpio_ir_recv_driver);
	platform_device_unregister(&elite_gpio_rc_device);
}
module_exit(elite_gpio_ir_recv_exit);

MODULE_DESCRIPTION("Elite GPIO IR Receiver driver");
MODULE_LICENSE("GPL v2");

