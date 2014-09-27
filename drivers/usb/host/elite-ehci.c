/*
 * Driver for ELite EHCI
 *
 * Copyright (C) 2013 S3 Graphics
 *
 * Author : htjeff <htjeff@s3graphics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/usb/ehci_def.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>

#include <mach/iomap.h>
#include <mach/ehci.h>

#ifdef CONFIG_ELITE_CROSS4k_PATCH
#include "elite-cross4k-patch.c"
#endif


extern unsigned int system_rev;

static void ehci_pci_init(struct usb_hcd *hcd);

static struct clk* uhdc_clk = NULL;

void ehci_disable_port_owner(struct usb_hcd *hcd )
{
    volatile unsigned int  temp32;
    struct ehci_regs __iomem *regs = hcd->regs+sizeof(struct ehci_caps);

    temp32 = __raw_readw(&(regs->configured_flag));

    if (temp32 == 0x00000001)
        __raw_writew( 0, &(regs->configured_flag ));

    while (1 == __raw_readw(&(regs->configured_flag)))
             ;

    __raw_writew( (__raw_readw(&regs->port_status[0]))|1<<13,
                  (&regs->port_status[0]));
    __raw_writew( (__raw_readw(&regs->port_status[1]))|1<<13,
                  (&regs->port_status[1]));
    __raw_writew( (__raw_readw(&regs->port_status[2]))|1<<13,
                  (&regs->port_status[2]));
    __raw_writew( (__raw_readw(&regs->port_status[3]))|1<<13,
                  (&regs->port_status[3]));
}


/* called during probe() after chip reset completes */
static int elite_ehci_setup(struct usb_hcd *hcd)
{
    struct ehci_hcd     *ehci = hcd_to_ehci(hcd);
    int         retval;

    ehci->caps = hcd->regs;
    ehci->regs = hcd->regs +
                 HC_LENGTH(ehci, ehci_readl(ehci, &ehci->caps->hc_capbase));

    dbg_hcs_params(ehci, "reset");

    /* cache this readonly data; minimize chip reads */
    ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);
    retval = ehci_halt(ehci);
    if (retval)
        return retval;

    /* data structure init */
    retval = ehci_init(hcd);
    if (retval)
        return retval;

    ehci_reset(ehci);
    ehci_port_power(ehci, 1);

    device_init_wakeup(&(hcd->self.root_hub->dev), 1);

    return 0;
}



#ifdef CONFIG_ELITE_CROSS4k_PATCH
static int elite_hcd_map_urb_for_dma(struct usb_hcd *hcd, struct urb *urb,
                                     gfp_t mem_flags)
{
    int ret;
    ret = usb_hcd_map_urb_for_dma(hcd, urb, mem_flags);
    if (ret)
        return ret;
    ret =  elite_create_cross4k(urb, mem_flags);
    if(ret)
        usb_hcd_unmap_urb_for_dma(hcd, urb);
    
    return ret;

}

static void elite_hcd_unmap_urb_for_dma(struct usb_hcd *hcd, struct urb *urb)
{
    elite_destroy_cross4k(urb);
    usb_hcd_unmap_urb_for_dma(hcd, urb);
}
#endif

static const struct hc_driver elite_ehci_hc_driver = {
    .description        = "elite-ehci",
    .product_desc       = "ELITE EHCI Host Controller",
    .hcd_priv_size      = sizeof(struct ehci_hcd),
    .irq            = ehci_irq,
    .flags          = HCD_MEMORY | HCD_USB2,
    .reset          = elite_ehci_setup,
    .start          = ehci_run,
    .stop           = ehci_stop,

#ifdef CONFIG_ELITE_CROSS4k_PATCH
    .map_urb_for_dma = elite_hcd_map_urb_for_dma,
    .unmap_urb_for_dma = elite_hcd_unmap_urb_for_dma,
#endif

    .shutdown       = ehci_shutdown,
    .urb_enqueue        = ehci_urb_enqueue,
    .urb_dequeue        = ehci_urb_dequeue,
    .endpoint_disable   = ehci_endpoint_disable,
    .endpoint_reset     = ehci_endpoint_reset,
    .get_frame_number   = ehci_get_frame,
    .hub_status_data    = ehci_hub_status_data,
    .hub_control        = ehci_hub_control,
#if defined(CONFIG_PM)
    .bus_suspend        = ehci_bus_suspend,
    .bus_resume     =  ehci_bus_resume,
#endif
    .relinquish_port    = ehci_relinquish_port,
    .port_handed_over   = ehci_port_handed_over,

    .clear_tt_buffer_complete   = ehci_clear_tt_buffer_complete,
};


static int elit_ehci_remove(struct platform_device *pdev)
{
    struct usb_hcd *hcd = platform_get_drvdata(pdev);
    struct elite_usb_platform_data *pdata = pdev->dev.platform_data;

    clk_disable(uhdc_clk);
    clk_put(uhdc_clk);
    usb_remove_hcd(hcd);
    iounmap(hcd->regs - 0x100);     // from ehci into pci config
    usb_put_hcd(hcd);

    if ((pdata->success_pin & 1))
        gpio_free(pdata->pw0_gpio_pin);

    if ((system_rev & 0xfff0) == 0x073a0) {
        if ((pdata->success_pin & 2))
            gpio_free(pdata->pwother_gpio_pin);
    }

    return 0;
}


static void ehci_pci_init(struct usb_hcd *hcd)
{

    __raw_writeb(0x40, (hcd->regs - 0x100 + 0x85));
    __raw_writeb(__raw_readb(((hcd->regs-0x100) + 0x49))|(0x3<<2),
                 ((hcd->regs-0x100) + 0x49));

    //Step1. Assign EHCI MMIO Address

    __raw_writel(ELITE_USB20_HOST_CFG_BASE + 0x900, ((hcd->regs-0x100) + PCI_BASE_ADDRESS_0));

    //Step2. Enable Bus Master, Memory Space
    __raw_writel(PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER|
                 __raw_readl(((hcd->regs-0x100) + PCI_COMMAND)),
                 ((hcd->regs-0x100) + PCI_COMMAND));




    //Step3. Set EHCI Sleep Timer Rx4B[6:5]=01b
    __raw_writel((__raw_readl(((hcd->regs-0x100) + 0x48))& 0x00FFFFFF) |0x29000000,
                 ((hcd->regs-0x100) + 0x48));

    // phy setting
    // HS Termination
    __raw_writeb((__raw_readb(((hcd->regs-0x100) + 0x5b))& 0xf0) |0xf,
                 ((hcd->regs-0x100) + 0x5b));
    __raw_writeb((__raw_readb(((hcd->regs-0x100) + 0x5a))& 0x0f) |0xf0,
                 ((hcd->regs-0x100) + 0x5a));
    __raw_writeb((__raw_readb(((hcd->regs-0x100) + 0x5e))& 0x0f) |0xd0,
                 ((hcd->regs-0x100) + 0x5e));
    // Disconnection Level
    __raw_writeb((__raw_readb(((hcd->regs-0x100) + 0x4f))& (0x0f)) |0xa0,
                 ((hcd->regs-0x100) + 0x4f));
    __raw_writeb((__raw_readb(((hcd->regs-0x100) + 0x41))& (~0xc0)) |0x80,
                 ((hcd->regs-0x100) + 0x41));

    // Squelch Level
    __raw_writeb((__raw_readb(((hcd->regs-0x100) + 0x4c))& (0x0f)) |0xf0,
                 ((hcd->regs-0x100) + 0x4c));
    __raw_writeb((__raw_readb(((hcd->regs-0x100) + 0x4f))& (~0x03)) |0x3,
                 ((hcd->regs-0x100) + 0x4f));
    // Slew Rate
    __raw_writeb((__raw_readb(((hcd->regs-0x100) + 0x56))& (~0x38)) |(0x7<<3),
                 ((hcd->regs-0x100) + 0x56));
    __raw_writeb((__raw_readb(((hcd->regs-0x100) + 0x57))& (0xc)) |0x3f,
                 ((hcd->regs-0x100) + 0x57));
}



#ifdef CONFIG_OF
static const struct of_device_id elite_ehci_match[] = {
	{ .compatible = "s3graphics,elite1000-ehci" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, elite_ehci_match);
#endif

static struct elite_usb_platform_data *of_get_elite_usb_pdata(struct device *dev)
{
	struct elite_usb_platform_data *pdata;
	struct device_node *np = dev->of_node;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	pdata->pw0_gpio_pin = of_get_named_gpio(np, "pwr-gpios", 0);

	pdata->pwother_gpio_pin = of_get_named_gpio(np, "pwr123-gpios", 0);

	pdata->success_pin = 0;
	
	dev_info(dev, "power pin: %d, power123 pin: %d\n",
		pdata->pw0_gpio_pin, pdata->pwother_gpio_pin);

	return pdata;
}

static int elite_power_gpio_config(struct platform_device *pdev)
{
    const struct of_device_id *match;
    struct elite_usb_platform_data *pdata = pdev->dev.platform_data;

    match = of_match_device(of_match_ptr(elite_ehci_match), &pdev->dev);
    if (match)
        pdata = of_get_elite_usb_pdata(&pdev->dev);

    if (gpio_is_valid(pdata->pw0_gpio_pin)) {
        if (gpio_request(pdata->pw0_gpio_pin, "usb_pw_0_pin")) {
            dev_err(&pdev->dev, "Fail to request PW port 0 pin\n");
            return -ENODEV;
        } else {
            pdata->success_pin |= 1;
            gpio_direction_output(pdata->pw0_gpio_pin, 1); //power on
            gpio_set_value(pdata->pw0_gpio_pin, 1);
        }
    } else
        dev_err(&pdev->dev, "usb PW port 0 pin not available\n");

    if ((system_rev & 0xfff0) == 0x073a0) {
        if (gpio_is_valid(pdata->pwother_gpio_pin)) {
            if (gpio_request(pdata->pwother_gpio_pin, "usb_pw_123_pin")) {
                dev_err(&pdev->dev, "Fail to request PW port 1/2/3 pin\n");
                return -ENODEV;
            } else {
                pdata->success_pin |= 2;
                gpio_direction_output(pdata->pwother_gpio_pin, 1); //power on
                gpio_set_value(pdata->pwother_gpio_pin, 1);
            }
        } else
            dev_err(&pdev->dev, "usb PW port 1/2/3 pin not available\n");
    }

    return 0;
}

static u64 elite_ehci_dma_mask = 0xFFFFF000;

int elite_ehci_probe(struct platform_device *pdev)
{
    const struct hc_driver  *driver;
    struct usb_hcd *hcd;
    int retval;
    struct resource *res;
    int irq;
    struct device *dev = &pdev->dev;

    if (usb_disabled())
        return -ENODEV;

    if (!pdev->dev.dma_mask)
	pdev->dev.dma_mask = &elite_ehci_dma_mask;

    driver = &elite_ehci_hc_driver;

    res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (!res) {
        dev_err(&pdev->dev,
                "Found Elite hcd with no IRQ. Check %s reource list!\n",
                dev_name(&pdev->dev));
        return -ENODEV;
    }
    irq = res->start;

    hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
    if (!hcd) {
        retval = -ENOMEM;
        goto fail_create_hcd;
    }

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev,
                "Found Elite hcd  with no register addr. Check %s reource list!\n",
                dev_name(&pdev->dev));
        retval = -ENODEV;
        goto fail_request_resource;
    }

    /* access mem */
    hcd->rsrc_start = res->start  + 0x100;  // from pci base into ehci base
    hcd->rsrc_len = resource_size(res) - 0x100;

    hcd->regs = ioremap(res->start, resource_size(res)) + 0x100;

    if (!hcd->regs) {
        dev_err(dev, "Elite EHCI ioremap failed\n");
        retval = -ENOMEM;
        goto fail_request_resource;

    }
    elite_power_gpio_config(pdev);
    ehci_pci_init(hcd);

    uhdc_clk = clk_get(&pdev->dev, "uhdc");
    if(IS_ERR(uhdc_clk)) {
        printk("elite EHCI : Fail to get uhdc clock %d\n", PTR_ERR(uhdc_clk));
        retval = -ENOMEM;
        uhdc_clk = NULL;
        goto fail_request_resource;
    }
    printk("USB clock rate %d \n", clk_get_rate(uhdc_clk));
    clk_enable(uhdc_clk);
    set_bit(HCD_FLAG_POLL_RH, &hcd->flags);
    platform_set_drvdata(pdev, hcd);

    retval = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
    if (retval)
        goto fail_request_resource;

    device_init_wakeup(&pdev->dev, 1);

    dev_info(&pdev->dev, "EHCI driver probed\n");

    return retval;

fail_request_resource:
    usb_put_hcd(hcd);
fail_create_hcd:
    dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), retval);
    return retval;
}


#if defined(CONFIG_PM_SLEEP)

static int elite_ehci_suspend(struct device *dev) 
{
    int ret;
    struct usb_hcd *hcd  = platform_get_drvdata(to_platform_device(dev));
    struct ehci_hcd *ehci;
    
    if(hcd == NULL) {
        dev_info(dev, "Elite EHCI HCD delete \n");
        return 0;
    }
    
    ehci = hcd_to_ehci(hcd);
    ehci_info(ehci,"Elite EHCI bus suspend \n");

    __raw_writew(__raw_readw((hcd->regs-0x100) + 0x84) |(0x103), ((hcd->regs-0x100) + 0x84));
    __raw_writew(__raw_readw((hcd->regs-0x100) + 0x62) |(0x9e), ((hcd->regs-0x100) + 0x62));
    mb();
    
    if (device_may_wakeup(hcd->self.controller))
        enable_irq_wake(IRQ_UHDC);
    
    return 0;
}

static int elite_ehci_resume(struct device *dev)
{
    struct usb_hcd *hcd = platform_get_drvdata(to_platform_device(dev));
    struct ehci_hcd *ehci ;

    if(hcd == NULL) {
        dev_info(dev, "Elite EHCI HCD delete \n");
        return 0;
    }
    ehci = hcd_to_ehci(hcd);
    ehci_info(ehci,"Elite EHCI bus resume.\n");

    if (device_may_wakeup(hcd->self.controller))
        disable_irq_wake(IRQ_UHDC);

    ehci_pci_init(hcd);
    __raw_writew(__raw_readw((hcd->regs-0x100) + 0x84) & (~0x003), ((hcd->regs-0x100) + 0x84));
    __raw_writew(__raw_readw((hcd->regs-0x100) + 0x62) & (~0x1e), ((hcd->regs-0x100) + 0x62));

    mb();
    return 0;
}

#endif //CONFIG_PM_SLEEP

#ifdef CONFIG_PM_RUNTIME
static int elite_ehci_runtime_suspend(struct device *dev)
{

    struct usb_hcd *hcd = platform_get_drvdata(to_platform_device(dev));
    struct ehci_hcd *ehci ;

    if(hcd == NULL) {
        dev_info(dev, "Elite EHCI HCD delete \n");
        return 0;
    }
    ehci = hcd_to_ehci(hcd);
    ehci_info(ehci,"Elite EHCI rumtime suspend.\n");
    
    //__raw_writew(__raw_readw((hcd->regs-0x100) + 0x84) |(0x100), ((hcd->regs-0x100) + 0x84));
    //__raw_writew(__raw_readw((hcd->regs-0x100) + 0x62) |(0x9e), ((hcd->regs-0x100) + 0x62));
    mb();
    return 0;
}


static int elite_ehci_runtime_resume(struct device *dev)
{
    struct usb_hcd *hcd = platform_get_drvdata(to_platform_device(dev));
    struct ehci_hcd *ehci ;

    if(hcd == NULL) {
        dev_info(dev, "Elite EHCI HCD delete \n");
        return 0;
    }
    ehci = hcd_to_ehci(hcd);
    ehci_info(ehci,"Elite EHCI runtime resume.\n");

    ehci_pci_init(hcd);
    __raw_writew(__raw_readw((hcd->regs-0x100) + 0x84) & (~0x003), ((hcd->regs-0x100) + 0x84));
    __raw_writew(__raw_readw((hcd->regs-0x100) + 0x62) & (~0x1e), ((hcd->regs-0x100) + 0x62));

    mb();
    return 0;

}
#endif //CONFIG_PM_RUNTIME

static const struct dev_pm_ops elite_ehci_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(elite_ehci_suspend, elite_ehci_resume)
	SET_RUNTIME_PM_OPS(elite_ehci_runtime_suspend,
				elite_ehci_runtime_resume, NULL)
};

static struct platform_device_id elite_ehci_driver_ids[] = {
    {
        .name  = "elite-ehci",
        .driver_data	= (kernel_ulong_t)NULL,
    }, {
        .name  = "elite-ehci.0",
        .driver_data	= (kernel_ulong_t)NULL,
    }, 
    { /* sentinel */ },
};

static struct platform_driver elite_ehci_driver = {
    .id_table   = elite_ehci_driver_ids,
    .probe      = elite_ehci_probe,
    .remove     = elit_ehci_remove,
    .shutdown   = usb_hcd_platform_shutdown,
    .driver = {
        .name   = "elite-ehci",
        .owner  = THIS_MODULE,
        .pm = &elite_ehci_pm_ops,
        .of_match_table = of_match_ptr(elite_ehci_match),
    }
};







