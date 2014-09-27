/*
 * ELITE WakeupCfg Source file
 *
 * Copyright (C) 2012 S3 Graphics, Inc.
 *	Linux-OpenGL-Platform Team
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <asm/hardware/gic.h>

#include <mach/irqs.h>
#include <mach/io.h>

#define ELITE_PMC_BASE			0xD8130000

#define PMC_REG_WAKEUP_STATUS				0x0014
#define PMC_REG_WAKEUP_EN					0x001c
#define PMC_REG_GP_TYPE						0x0020
#define PMC_REG_CARD_DETECT_TYPE				0x0024
#define PMC_REG_CARD_DETECT_STATUS			0x0028
/* Card Attachment Debounce Control and Interrupt Type Register */
#define PMC_REG_CARD_DEBOUCE_CTRL_INT		0x002c

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
#define WAKEUP_GPIO3_STAT_EN				(1UL<<3)
#define WAKEUP_GPIO2_STAT_EN				(1UL<<2)
#define WAKEUP_GPIO1_STAT_EN				(1UL<<1)
#define WAKEUP_GPIO0_STAT_EN				(1UL<<0)

#define WAKEUP_SIGNAL_LEVEL_LOW			0
#define WAKEUP_SIGNAL_LEVEL_HIGH			1
#define WAKEUP_SIGNAL_EDGE_FALLING		2
#define WAKEUP_SIGNAL_EDGE_RISING		3
#define WAKEUP_SIGNAL_EDGE_GENERATE		4
#define WAKEUP_SIGNAL_MASK				0xf

/* for General Purpose Wake-up Type */
#define WAKEUP_GP_GPIO0_SHIFT				0
#define WAKEUP_GP_GPIO1_SHIFT				4
#define WAKEUP_GP_GPIO2_SHIFT				8
#define WAKEUP_GP_GPIO3_SHIFT				12
#define WAKEUP_GP_INTERNAL_IP0_SHIFT		16
#define WAKEUP_GP_INTERNAL_IP1_SHIFT		20
#define WAKEUP_GP_INTERNAL_IP2_SHIFT		24
#define WAKEUP_GP_INTERNAL_IP3_SHIFT		28

#define WAKEUP_SD3_CARD_DETECT_SHIFT	24
#define WAKEUP_SD0_CARD_DETECT_SHIFT	20

#define WAKEUP_SD3_CARD_DETECT_STA_SHIFT		28
#define WAKEUP_SD0_CARD_DETECT_STA_SHIFT		27
#define WAKEUP_SD3_CARD_DETECT_STA_FLG_SHIFT	20
#define WAKEUP_SD0_CARD_DETECT_STA_FLG_SHIFT	19

#define WAKEUP_SD3_DETECT_INT_TYPE_SHIFT		24
#define WAKEUP_SD0_DETECT_INT_TYPE_SHIFT		22
#define WAKEUP_SD_DETECT_INT_TYPE_MASK			2
#define WAKEUP_CARD_DETECT_SAMP_TIME_SHIFT			1
#define WAKEUP_CARD_DETECT_DEBOUNCE_TIME_SFIFT	0

#define WAKEUP_SD_DETECT_INT_DETECT			1
#define WAKEUP_SD_DETECT_INT_UNDETECT		2
#define WAKEUP_SD_DETECT_INT_BOTH			3

enum wakeup_irqs {
	SD0 = 0,
	SD3 = 1,
	CIR,
	UHDC,
	UDC,
	RTC,
	WAKEUP_IRQ_NUM,
};

unsigned int last_wakeup_status = 0;
static u32 pmc_wake_irqs[WAKEUP_IRQ_NUM] = {0};
//static u32 pmc_saved_regs[WAKEUP_IRQ_NUM] = {0};


static void __iomem *pmc_base = (void __iomem *)IO_ADDRESS(ELITE_PMC_BASE);

static inline void pmc_writel(u32 value, u32 reg) 
{
	writel_relaxed(value, pmc_base + reg);
}

static inline u32 pmc_readl(u32 reg)
{
	return readl_relaxed(pmc_base + reg);
}

void wakeupcfg_pre_suspend(void)
{
	int i;
	u32 val = 0;
	u32 strapping;

	strapping = readl(IO_ADDRESS(0xd8110100));

	val |= WAKEUP_SIGNAL_EDGE_FALLING<<WAKEUP_GP_GPIO0_SHIFT;
	val |= WAKEUP_SIGNAL_EDGE_FALLING<<WAKEUP_GP_GPIO1_SHIFT;
	val |= WAKEUP_SIGNAL_EDGE_RISING<<WAKEUP_GP_GPIO2_SHIFT;
	val |= WAKEUP_SIGNAL_EDGE_RISING<<WAKEUP_GP_GPIO3_SHIFT;
	pmc_writel(val, PMC_REG_GP_TYPE);

	val = 0;
	val |= WAKEUP_SIGNAL_EDGE_RISING<<WAKEUP_SD3_CARD_DETECT_SHIFT;
	val |= WAKEUP_SIGNAL_EDGE_GENERATE<<WAKEUP_SD0_CARD_DETECT_SHIFT;
	pmc_writel(val, PMC_REG_CARD_DETECT_TYPE);

	val = 0;
	val |= WAKEUP_SD_DETECT_INT_DETECT<<WAKEUP_SD3_DETECT_INT_TYPE_SHIFT;
	val |= WAKEUP_SD_DETECT_INT_DETECT<<WAKEUP_SD0_DETECT_INT_TYPE_SHIFT;
	val |= 1<<WAKEUP_CARD_DETECT_SAMP_TIME_SHIFT; //8ms
	val |= 1<<WAKEUP_CARD_DETECT_DEBOUNCE_TIME_SFIFT; //512ms
	pmc_writel(val, PMC_REG_CARD_DEBOUCE_CTRL_INT);
	
	val = 0;
	if (strapping & (1<<31)) {
		/* power auto enable can't use power button to wakeup*/
		val |= WAKEUP_POWER_BTN_STAT_EN;
	}
	val |= WAKEUP_GPIO0_STAT_EN;
	val |= WAKEUP_GPIO1_STAT_EN;
	val |= WAKEUP_GPIO2_STAT_EN;
	val |= WAKEUP_GPIO3_STAT_EN;
	
	for (i = 0; i < WAKEUP_IRQ_NUM; i++) {
		val |= pmc_wake_irqs[i];
	}

	pmc_writel(val, PMC_REG_WAKEUP_EN);
	pmc_writel(0xffffffff, PMC_REG_WAKEUP_STATUS);
}

void wakeupcfg_post_resume(void)
{
	u32 val, val2;

	val = pmc_readl(PMC_REG_WAKEUP_STATUS);
	last_wakeup_status = val;
	if (val & WAKEUP_SD3_CARD_DETECT_STAT_EN) {
		printk(KERN_INFO "Wakeuped by SD3 card detect\n");
		val2 = WAKEUP_SD3_CARD_DETECT_STAT_EN;
		pmc_writel(val2, PMC_REG_WAKEUP_STATUS);
	}
	if (val & WAKEUP_SD0_CARD_DETECT_STAT_EN) {
		printk(KERN_INFO "Wakeuped by SD0 card detect\n");
		val2 = WAKEUP_SD0_CARD_DETECT_STAT_EN;
		pmc_writel(val2, PMC_REG_WAKEUP_STATUS);
	}
	if (val & WAKEUP_CIR_STAT_EN) {
		printk(KERN_INFO "Wakeuped by CIR\n");
		val2 = WAKEUP_CIR_STAT_EN;
		pmc_writel(val2, PMC_REG_WAKEUP_STATUS);
	}
	if (val & WAKEUP_USB_DEV_ATTACH_STAT_EN) {
		printk(KERN_INFO "Wakeuped by USB device attach\n");
		val2 = WAKEUP_USB_DEV_ATTACH_STAT_EN;
		pmc_writel(val2, PMC_REG_WAKEUP_STATUS);
	}
	if (val & WAKEUP_USB_HOST_STAT_EN) {
		printk(KERN_INFO "Wakeuped by USB host\n");
		val2 = WAKEUP_USB_HOST_STAT_EN;
		pmc_writel(val2, PMC_REG_WAKEUP_STATUS);
	}
	if (val & WAKEUP_RTC_STAT_EN) {
		printk(KERN_INFO "Wakeuped by RTC\n");
		val2 = WAKEUP_RTC_STAT_EN;
		pmc_writel(val2, PMC_REG_WAKEUP_STATUS);
	}
	if (val & WAKEUP_POWER_BTN_STAT_EN) {
		printk(KERN_INFO "Wakeuped by Power button\n");
		val2 = WAKEUP_POWER_BTN_STAT_EN;
		pmc_writel(val2, PMC_REG_WAKEUP_STATUS);
	}
	if (val & WAKEUP_GPIO3_STAT_EN) {
		printk(KERN_INFO "Wakeuped by GPIO3\n");
		val2 = WAKEUP_GPIO3_STAT_EN;
		pmc_writel(val2, PMC_REG_WAKEUP_STATUS);
	}
	if (val & WAKEUP_GPIO2_STAT_EN) {
		printk(KERN_INFO "Wakeuped by GPIO2\n");
		val2 = WAKEUP_GPIO2_STAT_EN;
		pmc_writel(val2, PMC_REG_WAKEUP_STATUS);
	}
	if (val & WAKEUP_GPIO1_STAT_EN) {
		printk(KERN_INFO "Wakeuped by GPIO1\n");
		val2 = WAKEUP_GPIO1_STAT_EN;
		pmc_writel(val2, PMC_REG_WAKEUP_STATUS);
	}
	if (val & WAKEUP_GPIO0_STAT_EN) {
		printk(KERN_INFO "Wakeuped by GPIO0\n");
		val2 = WAKEUP_GPIO0_STAT_EN;
		pmc_writel(val2, PMC_REG_WAKEUP_STATUS);
	}


	val = pmc_readl(PMC_REG_CARD_DETECT_STATUS);
	printk(KERN_INFO "PMC card detect interrupt status: %x\n", val);

	val2 = pmc_readl(PMC_REG_WAKEUP_STATUS);

	if (val2 != 0x0) {
		printk(KERN_ERR "PMC wake up status(%x) error!!!\n", val2);
	}
}
extern unsigned int system_rev;
static int wakeupcfg_irq_set_wake(struct irq_data *data, unsigned int on)
{
	/* Sanity check for SPI irq */
	if (data->irq < 32)
		return -EINVAL;

	switch(data->irq) {
	case IRQ_SDC0:
		pmc_wake_irqs[SD0] = on?WAKEUP_SD0_CARD_DETECT_STAT_EN:0;
		break;
#if 0
	case IRQ_SDC3:
		pmc_wake_irqs[SD3] = on?WAKEUP_SD3_CARD_DETECT_STAT_EN:0;
		break;
#endif
	case IRQ_CIR:
		pmc_wake_irqs[CIR] = on?WAKEUP_CIR_STAT_EN:0;
		break;
	
	case IRQ_UHDC:
		if ((system_rev&0xfff0) == 0x073a0)
			pmc_wake_irqs[UHDC] = on?WAKEUP_USB_HOST_STAT_EN:0;
		else
			pmc_wake_irqs[UHDC] = 0;
		break;
#if 0
	case IRQ_UDC:
		pmc_wake_irqs[UDC] = on?WAKEUP_USB_DEV_ATTACH_STAT_EN:0;
		break;
#endif
	case IRQ_RTC0:
	case IRQ_RTC1:
		pmc_wake_irqs[RTC] = on?WAKEUP_RTC_STAT_EN:0;
		break;
	default:
		return -EINVAL;
	}
	
	return 0; /* always allow wakeup */
}

/* *
  * Elite SoC doesn't implement an IRQ controller extention for GIC,
  * so, leave the function body blank here
  *
  * @d: per irq and irq chip data passed down to chip functions
  */
static void wakeupcfg_irq_unmask(struct irq_data *d)
{
	/* Sanity check for SPI irq */



	if (d->irq < 32)
		return;
	
	/* do something if you have an IRQ controller externsion */
}


/* *
  * Elite SoC doesn't implement an IRQ controller extention for GIC,
  * so, leave the function body blank here
  *
  * @d: per irq and irq chip data passed down to chip functions
  */
static void wakeupcfg_irq_mask(struct irq_data *d)
{
	/* Sanity check for SPI irq */
	if (d->irq < 32)
		return;
	
	/* do something if you have an IRQ controller externsion */
}

void __init elite_init_wakeupcfg(void)
{
#ifdef CONFIG_OF
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "s3graphics,elite1000-pmc");
	if (!np) {
		pr_err("%s: can't find PMC device tree node\n",
			__func__);
	}
	pmc_base = of_iomap(np, 0);
	if (!pmc_base) {
		pr_err("%s: failed to map PMC registers\n", __func__);
		of_node_put(np);
		pmc_base = (void __iomem *)IO_ADDRESS(ELITE_PMC_BASE);
	}
	of_node_put(np);
#endif
        pmc_writel(0xffffffff, PMC_REG_WAKEUP_STATUS);
        writel(0xfffffff, IO_ADDRESS(0xd839c6cc));
        pmc_writel(0, PMC_REG_WAKEUP_EN);
	/*
	 * Override GIC architecture specific functions
	 */
	gic_arch_extn.irq_set_wake = wakeupcfg_irq_set_wake;
	gic_arch_extn.irq_unmask = wakeupcfg_irq_unmask;
	gic_arch_extn.irq_mask = wakeupcfg_irq_mask;
	/* gic_arch_extn.flags = IRQCHIP_MASK_ON_SUSPEND; */
}

