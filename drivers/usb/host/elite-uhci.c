/*
 * Driver for ELite UHCI
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
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <mach/iomap.h>
#include <linux/usb/ehci_def.h>


static void elite_uhci_pci_init(struct usb_hcd *hcd);


#ifdef CONFIG_PM

static int   elite_read_config_word(struct usb_hcd *hcd, u16 reg, u16* data)
{
    *data =  __raw_readw(((hcd->regs-0x100) + reg));
    return 0;
}

static int   elite_write_config_word(struct usb_hcd *hcd, u16 reg, u16 data)
{
    __raw_writew(data, ((hcd->regs-0x100) + reg));
    return 0;
}


static int elite_uhci_suspend(struct usb_hcd *hcd, bool do_wakeup)
{
    struct uhci_hcd *uhci = hcd_to_uhci(hcd);
    int rc = 0;
    u16 pmc_enable = 0;

    //  dev_info(uhci_dev(uhci), "Elite UHCI pci suspend.\n");
    printk(  "Elite UHCI pci suspend.\n");
    spin_lock_irq(&uhci->lock);
    if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags) || uhci->dead)
        goto done_okay;     /* Already suspended or dead */

    if (uhci->rh_state > UHCI_RH_SUSPENDED) {
        dev_warn(uhci_dev(uhci), "Root hub isn't suspended!\n");
        rc = -EBUSY;
        goto done;
    };

    /* All PCI host controllers are required to disable IRQ generation
     * at the source, so we must turn off PIRQ.
     */
    elite_write_config_word(hcd, USBLEGSUP, 0);
    mb();
    clear_bit(HCD_FLAG_POLL_RH, &hcd->flags);

    /* FIXME: Enable non-PME# remote wakeup? */
    elite_read_config_word(hcd, 0x84, &pmc_enable);
    pmc_enable |= 0x103;
    elite_write_config_word(hcd, 0x84, pmc_enable);

done_okay:
    clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
done:
    spin_unlock_irq(&uhci->lock);
    return rc;
}


static int elite_uhci_resume(struct usb_hcd *hcd, bool hibernated)
{
    struct uhci_hcd *uhci = hcd_to_uhci(hcd);
    u16 pmc_enable = 0;

    printk(  "Elite UHCI pci resume.\n");
    uhci_writeb(uhci, uhci_readb(uhci, USBINTR) & (~USBINTR_RESUME), USBINTR);

    elite_read_config_word(hcd, 0x84, &pmc_enable);
    pmc_enable &= ~0x03;
    elite_write_config_word(hcd, 0x84, pmc_enable);

    elite_uhci_pci_init(hcd);

    dev_dbg(uhci_dev(uhci), "%s\n", __func__);

    /* Since we aren't in D3 any more, it's safe to set this flag
     * even if the controller was dead.
     */
    set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
    mb();

    spin_lock_irq(&uhci->lock);

    /* Make sure resume from hibernation re-enumerates everything */
    if (hibernated)
        uhci_hc_died(uhci);

    /* FIXME: Disable non-PME# remote wakeup? */

    /* The firmware or a boot kernel may have changed the controller
     * settings during a system wakeup.  Check it and reconfigure
     * to avoid problems.
     */
    else
        check_and_reset_hc(uhci);
    /* If the controller was dead before, it's back alive now */
    configure_hc(uhci);

    if (uhci->rh_state == UHCI_RH_RESET)
        /* The controller had to be reset */
        usb_root_hub_lost_power(hcd->self.root_hub);

    spin_unlock_irq(&uhci->lock);

    /* If interrupts don't work and remote wakeup is enabled then
     * the suspended root hub needs to be polled.
     */
    if (!uhci->RD_enable && hcd->self.root_hub->do_remote_wakeup)
        set_bit(HCD_FLAG_POLL_RH, &hcd->flags);

    /* Does the root hub have a port wakeup pending? */
    usb_hcd_poll_rh_status(hcd);

    return 0;
}

static int elite_uhci_rh_resume(struct usb_hcd *hcd)
{
    u16 pmc_enable = 0;
    int ret;
    struct uhci_hcd *uhci = hcd_to_uhci(hcd);
    dev_info(uhci_dev(uhci),"Elite UHCI bus resume.\n");
    elite_uhci_pci_init(hcd);
    elite_read_config_word(hcd, 0x84, &pmc_enable);
    pmc_enable &= ~0x03;
    elite_write_config_word(hcd, 0x84, pmc_enable);

    elite_read_config_word(hcd, 0x48, &pmc_enable);
    pmc_enable &= ~0x0300;
    elite_write_config_word(hcd, 0x48, pmc_enable);
    configure_hc(uhci);
    ret = uhci_rh_resume(hcd);
    return ret;
}
static int elite_uhci_rh_suspend(struct usb_hcd *hcd)
{
    u16 pmc_enable = 0;
    struct uhci_hcd *uhci = hcd_to_uhci(hcd);
    dev_info(uhci_dev(uhci), "Elite UHCI bus suspend.\n");
    
    elite_read_config_word(hcd, 0x84, &pmc_enable);
    pmc_enable |= 0x103;
    elite_write_config_word(hcd, 0x84, pmc_enable);

    elite_read_config_word(hcd, 0x48, &pmc_enable);
    pmc_enable |= 0x0f00;
    elite_write_config_word(hcd, 0x48, pmc_enable);
    return uhci_rh_suspend(hcd);
}
#endif

static int uhci_elite_init(struct usb_hcd *hcd)
{
    struct uhci_hcd *uhci = hcd_to_uhci(hcd);

    /*
     * Probe to determine the endianness of the controller.
     * We know that bit 7 of the PORTSC1 register is always set
     * and bit 15 is always clear.  If uhci_readw() yields a value
     * with bit 7 (0x80) turned on then the current little-endian
     * setting is correct.  Otherwise we assume the value was
     * byte-swapped; hence the register interface and presumably
     * also the descriptors are big-endian.
     */
    if (!(uhci_readw(uhci, USBPORTSC1) & 0x80)) {
        uhci->big_endian_mmio = 1;
        uhci->big_endian_desc = 1;
    }

    uhci->rh_numports = uhci_count_ports(hcd);

    /* Set up pointers to to generic functions */
    uhci->reset_hc = uhci_generic_reset_hc;
    uhci->check_and_reset_hc = uhci_generic_check_and_reset_hc;
    /* No special actions need to be taken for the functions below */
    uhci->configure_hc = NULL;
    uhci->resume_detect_interrupts_are_broken = NULL;
    uhci->global_suspend_mode_is_broken = NULL;
    uhci->oc_low = 1;
    /* Reset if the controller isn't already safely quiescent. */
    check_and_reset_hc(uhci);

    device_init_wakeup(&(hcd->self.root_hub->dev), 1);

    return 0;
}

static const struct hc_driver elite_uhci_hc_driver = {
    .description =      hcd_name,
    .product_desc =     "UHCI Host Controller",
    .hcd_priv_size =    sizeof(struct uhci_hcd),

    /* Generic hardware linkage */
    .irq =          uhci_irq,
    .flags =        HCD_MEMORY |HCD_USB11,

    /* Basic lifecycle operations */
    .reset =        uhci_elite_init,
    .start =        uhci_start,
#ifdef CONFIG_PM
    .bus_suspend =      elite_uhci_rh_suspend,
    .bus_resume =       elite_uhci_rh_resume,
#endif
    .stop =                uhci_stop,

    .urb_enqueue =      uhci_urb_enqueue,
    .urb_dequeue =      uhci_urb_dequeue,

    .endpoint_disable = uhci_hcd_endpoint_disable,
    .get_frame_number = uhci_hcd_get_frame_number,

    .hub_status_data =  uhci_hub_status_data,
    .hub_control =      uhci_hub_control,
};



static int elit_uhci_remove(struct platform_device *pdev)
{
    struct usb_hcd *hcd = platform_get_drvdata(pdev);
    usb_remove_hcd(hcd);
    iounmap(hcd->regs - 0x100);
    usb_put_hcd(hcd);
    return 0;
}


static void elite_uhci_pci_init(struct usb_hcd *hcd)
{

    //  __raw_writeb(0x40, (hcd->regs - 0x100 + 0x85));
    //  __raw_writeb(__raw_readb(((hcd->regs-0x100) + 0x49))|(0x3<<2),
    //                           ((hcd->regs-0x100) + 0x49));

    //Step1. Assign UHCI MMIO Address

    __raw_writel(hcd->rsrc_start , ((hcd->regs-0x100) + PCI_BASE_ADDRESS_4));

    //Step2. Enable Bus Master, Memory Space
    __raw_writel(PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER|
                 __raw_readl(((hcd->regs-0x100) + PCI_COMMAND)),
                 ((hcd->regs-0x100) + PCI_COMMAND));



}

static u64 elite_uhci_dma_mask = 0xFFFFF000;

int elite_uhci_probe(struct platform_device *pdev)
{
    const struct hc_driver  *driver;
    struct usb_hcd      *hcd;
    int         retval;
    struct resource *res;
    int irq;
    struct uhci_hcd *uhci = NULL;

    if (usb_disabled())
        return -ENODEV;

    if (!pdev->dev.dma_mask)
        pdev->dev.dma_mask = &elite_uhci_dma_mask;

    driver = &elite_uhci_hc_driver;

    res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (!res) {
        dev_err(&pdev->dev,
                "Found Elite hcd with no IRQ. Check %s reource list!\n",
                dev_name(&pdev->dev));
        return -ENODEV;
    }
    irq = res->start;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev,
                "Found Elite hcd  with no register addr. Check %s reource list!\n",
                dev_name(&pdev->dev));
        return -ENODEV;
    }

    hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
    if (!hcd) {
        retval = -ENOMEM;
        goto fail_create_hcd;
    }

    /* access io */
    hcd->rsrc_start = res->start + 0x100;
    hcd->rsrc_len = resource_size (res) - 0x100;

    hcd->regs = ioremap(res->start, resource_size (res)) + 0x100;
    if (!hcd->regs) {
        printk(KERN_ERR "%s: ioremap failed\n", __FILE__);
        retval = -ENOMEM;
        goto err_ioremap;
    }

    uhci = hcd_to_uhci(hcd);

    uhci->regs = hcd->regs;
    elite_uhci_pci_init(hcd);
        
    retval = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
    if (retval)
        goto fail_request_resource;

    device_init_wakeup(&pdev->dev, 1);

    dev_info(&pdev->dev, "UHCI driver probed\n");

    return retval;


fail_request_resource:
    iounmap(hcd->regs -0x100);
err_ioremap:
    usb_put_hcd(hcd);
fail_create_hcd:
    dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), retval);
    return retval;
}

static struct platform_device_id elite_uhci_driver_ids[] = {
    {
        .name  = "elite-uhci",
        .driver_data	= (kernel_ulong_t)NULL,
    }, {
        .name  = "elite-uhci.0",
        .driver_data	= (kernel_ulong_t)NULL,
    }, {
        .name  = "elite-uhci.1",
        .driver_data	= (kernel_ulong_t)NULL,
    }, 
    { /* sentinel */ },
};

#ifdef CONFIG_OF
static const struct of_device_id elite_uhci_match[] = {
	{ .compatible = "s3graphics,elite1000-uhci" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, elite_uhci_match);
#endif

static struct platform_driver elite_uhci_driver = {
    .id_table  = elite_uhci_driver_ids,
    .probe      = elite_uhci_probe,
    .remove     = elit_uhci_remove,
    .shutdown   = usb_hcd_platform_shutdown,
    .driver = {
        .name   = "elite-uhci",
        .owner  = THIS_MODULE,
        .of_match_table = of_match_ptr(elite_uhci_match),
    }
};
