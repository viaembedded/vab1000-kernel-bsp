/*
 * Support functions for elite GPIO
 *
 * Copyright (C) 2012 S3 Graphics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/syscore_ops.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/irqdomain.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/pinctrl/consumer.h>

#include <asm/irq.h>
#include <mach/irqs.h>
#include <asm/gpio.h>
#include <asm/mach/irq.h>

#define ELITE_NR_GPIOS		ARCH_NR_GPIOS
#define ELITE_NR_BANKS		61

#define REG_NR_INPUT_DAT	19
#define REG_NR_EN		19
#define REG_NR_OUTPUT_EN	19
#define REG_NR_OUTPUT_DAT	19
#define REG_NR_INTERRUPT	3
#define REG_NR_PULL_UPDW_EN	19
#define REG_NR_PULL_UPDW_CTRL0	19
#define REG_NR_PULL_UPDW_CTRL1	19

#define INT_GPIO_NR		12
#define GPIO_IRQ_TYPE_EDGE_RISING	0x3
#define GPIO_IRQ_TYPE_EDGE_FALLING	0x2
#define GPIO_IRQ_TYPE_CONFIG(x, type)	((type)<<((x)*2))
#define GPIO_IRQ_TYPE_CLR(x)			(~((0x3)<<((x)*2)))

#define ELITE_GPIO_TO_IRQ(gpio) (IRQ_END + (gpio))
#define ELITE_IRQ_TO_GPIO(irq)  ((irq)-IRQ_END)

#define REG_INPUT_DAT(x)		((x)/8)
#define REG_ENABLE(x)			(((x)/8) + 0x40) //pin mux register
#define REG_OUTPUT_EN(x)		(((x)/8) + 0x80)
#define REG_OUTPUT_DAT(x)		(((x)/8) + 0xc0)
#define REG_PULL_UP_DOWN_EN(x)		(((x)/8) + 0x480)
#define REG_PULL_UP_DOWN_CTRL0(x)	(((x)/8) + 0x4c0)
#define REG_PULL_UP_DOWN_CTRL1(x)	(((x)/8) + 0x5c0)

#define REG_PIN_SHARE            	0x200
#define REG_USB_OP_MODE          	0x10c

#define REG_INT_LVL(x)			(0x300)
#define REG_INT_STA(x)			(0x304)
#define REG_INT_CLR(x)			(0x304)
#define REG_INT_MASK(x)			(0x308)

#define REG_BIT(x)			((x) % 8)

struct elite_gpio_ctrl {
	void __iomem *base;
	int irq;

	int num_irq;

	spinlock_t lock;
	struct gpio_chip *chip;
	struct clk *clk;

	struct device *dev;

#ifdef CONFIG_PM_SLEEP
	u8 indat[REG_NR_INPUT_DAT];
	u8 gpio_usb;
	u8 en[REG_NR_EN];
	u8 en_gp60;
	u8 outen[REG_NR_OUTPUT_EN];
	u8 outdat[REG_NR_OUTPUT_DAT];
	u8 outdat_gp36;
	u8 outdat_gp60;
	u32 usb_op;
	u32 pin_share;
	u32 intr[REG_NR_INTERRUPT];
	u8 pull_updw_en[REG_NR_PULL_UPDW_EN];
	u8 pull_updw_en_gp60;
	u8 pull_updw_en_gp61;
	u8 pull_updw_ctrl0[REG_NR_PULL_UPDW_CTRL0];
	u8 pull_updw_ctrl0_gp60;
	u8 pull_updw_ctrl0_gp61;
	u8 pull_updw_ctrl1[REG_NR_PULL_UPDW_CTRL1];
	u8 pull_updw_ctrl1_gp60;
	u8 pull_updw_ctrl1_gp61;
#endif //CONFIG_PM_SLEEP
};


struct elite_gpio_platform_data {
	u16 virtual_irq_start;
};


static void __iomem *regs_base;
struct irq_domain *irq_domain;
static struct elite_gpio_ctrl gpio_ctrl = {0};

static inline void elite_gpio_writeb(u32 val, u32 reg)
{
	__raw_writeb(val, regs_base + reg);
}

static inline u8 elite_gpio_readb(u32 reg)
{
	return __raw_readb(regs_base + reg);
}

static inline void elite_gpio_writel(u32 val, u32 reg)
{
	__raw_writel(val, regs_base + reg);
}

static inline u32 elite_gpio_readl(u32 reg)
{
	return __raw_readl(regs_base + reg);
}

static void elite_gpio_bit_writeb(u32 reg, int gpio, int value)
{
	u8 val;

	val = elite_gpio_readb(reg);
	if(value)
		elite_gpio_writeb((1 << REG_BIT(gpio)) | val, reg);
	else
		elite_gpio_writeb((~(1 << REG_BIT(gpio))) & val, reg);
}

static int elite_gpio_bit_readb(u32 reg, int gpio)
{
	return (elite_gpio_readb(reg) >> REG_BIT(gpio)) & 0x01;
}

static void elite_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	elite_gpio_bit_writeb(REG_OUTPUT_DAT(offset), offset, value);
}

static int elite_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	return elite_gpio_bit_readb(REG_INPUT_DAT(offset), offset);
}

static int elite_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	elite_gpio_bit_writeb(REG_OUTPUT_EN(offset), offset, 0);
	return 0;
}

static int elite_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
					int value)
{
	elite_gpio_bit_writeb(REG_OUTPUT_EN(offset), offset, 1);
	elite_gpio_bit_writeb(REG_OUTPUT_DAT(offset), offset, value);
	return 0;
}

static int elite_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	return pinctrl_request_gpio(offset);
}

static void elite_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	pinctrl_free_gpio(offset);
}

static int elite_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return irq_find_mapping(irq_domain, offset);
}

static struct gpio_chip elite_gpio_chip = {
	.label			= "elite-gpio",
	.request		= elite_gpio_request,
	.free			= elite_gpio_free,
	.direction_input	= elite_gpio_direction_input,
	.get			= elite_gpio_get,
	.direction_output	= elite_gpio_direction_output,
	.set			= elite_gpio_set,
	.to_irq			= elite_gpio_to_irq,
	.base			= 0,
	.ngpio			= ELITE_NR_GPIOS,
};

static void elite_gpio_int_bit_writel(u32 reg, int gpio, int value)
{
	u32 val;

	val = elite_gpio_readl(reg);
	
	if(value)
		elite_gpio_writel((1 << gpio) | val, reg);
	else
		elite_gpio_writel((~(1 << gpio)) & val, reg);
	 
}

static void elite_gpio_irq_ack(struct irq_data *d)
{
	int gpio = ELITE_IRQ_TO_GPIO(d->irq);

	elite_gpio_int_bit_writel(REG_INT_CLR(gpio), gpio, 1);
}

static void elite_gpio_irq_mask(struct irq_data *d)
{
	int gpio = ELITE_IRQ_TO_GPIO(d->irq);

	elite_gpio_int_bit_writel(REG_INT_MASK(gpio), gpio, 1);
}

static void elite_gpio_irq_unmask(struct irq_data *d)
{
	int gpio = ELITE_IRQ_TO_GPIO(d->irq);

	elite_gpio_int_bit_writel(REG_INT_MASK(gpio), gpio, 0);
}

static int elite_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	int gpio = ELITE_IRQ_TO_GPIO(d->irq);
	struct elite_gpio_ctrl *ctrl = irq_data_get_irq_chip_data(d);
	int lvl_type;
	int val;
	unsigned long flags;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		lvl_type = GPIO_IRQ_TYPE_EDGE_RISING;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		lvl_type = GPIO_IRQ_TYPE_EDGE_FALLING;
		break;

	case IRQ_TYPE_EDGE_BOTH:
	case IRQ_TYPE_LEVEL_HIGH:
		/* Not supported by elite1k */
	case IRQ_TYPE_LEVEL_LOW:
		/* Not supported by elite1k */
		return -EINVAL;

	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&ctrl->lock, flags);

	val = elite_gpio_readl(REG_INT_LVL(gpio));
	val &= GPIO_IRQ_TYPE_CLR(gpio);
	val |= GPIO_IRQ_TYPE_CONFIG(gpio, lvl_type);
	elite_gpio_writel(val, REG_INT_LVL(gpio));

	spin_unlock_irqrestore(&ctrl->lock, flags);

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		__irq_set_handler_locked(d->irq, handle_level_irq);
	else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		__irq_set_handler_locked(d->irq, handle_edge_irq);

	return 0;
}

static void elite_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct elite_gpio_ctrl *bank;
	int pin;
	int unmasked = 0;
	u32 gpio = 0;
	
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	bank = irq_get_handler_data(irq);

	unsigned long sta = (u32)elite_gpio_readl(REG_INT_STA(gpio)) &
			(~ elite_gpio_readl(REG_INT_MASK(gpio)));
	u32 lvl = elite_gpio_readl(REG_INT_LVL(gpio));

	for_each_set_bit(pin, &sta, INT_GPIO_NR) {
		elite_gpio_writel(1 << pin, REG_INT_STA(gpio));

		/* if gpio is edge triggered, clear condition
		 * before executing the hander so that we don't
		 * miss edges
		 */
		if (lvl & (0x2 << pin)) {
			unmasked = 1;
			chained_irq_exit(chip, desc);
		}

		generic_handle_irq(gpio_to_irq(gpio + pin));
	}

	if (!unmasked)
		chained_irq_exit(chip, desc);

}

#ifdef CONFIG_PM_SLEEP
static int elite_gpio_wake_enable(struct irq_data *d, unsigned int enable)
{
	struct elite_gpio_ctrl *ctrl= irq_data_get_irq_chip_data(d);
	return irq_set_irq_wake(ctrl->irq, enable);
}
#endif

static struct irq_chip elite_gpio_irq_chip = {
	.name		= "elite-GPIO",
	.irq_ack	= elite_gpio_irq_ack,
	.irq_mask	= elite_gpio_irq_mask,
	.irq_unmask	= elite_gpio_irq_unmask,
	.irq_set_type	= elite_gpio_irq_set_type,
#ifdef CONFIG_PM_SLEEP
	.irq_set_wake	= elite_gpio_wake_enable,
#endif
};

static struct of_device_id elite_gpio_of_match[] __devinitdata = {
	{ .compatible = "s3graphics,elite1000-gpio", .data = NULL },
	{ },
};

/* This lock class tells lockdep that GPIO irqs are in a different
 * category than their parents, so it won't report false recursion.
 */
static struct lock_class_key gpio_lock_class;

static int __devinit elite_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	const struct of_device_id *match;
	struct elite_gpio_platform_data *pdata;
	struct resource *res;
	struct elite_gpio_ctrl *gctrl  = &gpio_ctrl;
	int irq_base;
	int gpio;


	match = of_match_device(of_match_ptr(elite_gpio_of_match), dev);

	pdata = match ? match->data : dev->platform_data;
	
	irq_base = irq_alloc_descs(-1, 0, INT_GPIO_NR, 0);
	if (irq_base < 0) {
		dev_err(dev, "Couldn't allocate IRQ numbers\n");
		return -ENODEV;
	}

	irq_domain = irq_domain_add_legacy(node, INT_GPIO_NR, irq_base,
					     0, &irq_domain_simple_ops, NULL);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing IRQ resource\n");
		return -ENODEV;
	}

	gctrl->irq = res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing MEM resource\n");
		return -ENODEV;
	}

	regs_base = devm_request_and_ioremap(&pdev->dev, res);
	if (!regs_base) {
		dev_err(&pdev->dev, "Couldn't ioremap regs\n");
		return -ENODEV;
	}

	gctrl->dev = &pdev->dev;
#if 0
	gctrl->clk = clk_get(&pdev->dev, "gpio");
	if (IS_ERR(gctrl->clk)) {
		dev_err(gctrl->dev, "Could not get gpio dbck\n");
		return -ENODEV;
	}
	clk_enable(gctrl->clk);
#endif
#ifdef CONFIG_OF_GPIO
	elite_gpio_chip.of_node = pdev->dev.of_node;
#endif

	gpiochip_add(&elite_gpio_chip);
	gctrl->chip = &elite_gpio_chip;

	for (gpio = 0; gpio < INT_GPIO_NR; gpio++) {
		int irq = irq_find_mapping(irq_domain, gpio);

		/* No validity check; all elite GPIOs are valid IRQs */

		irq_set_lockdep_class(irq, &gpio_lock_class);
		irq_set_chip_data(irq, gctrl);
		irq_set_chip_and_handler(irq, &elite_gpio_irq_chip,
					 handle_simple_irq);
		set_irq_flags(irq, IRQF_VALID);
	}

	irq_set_chained_handler(gctrl->irq, elite_gpio_irq_handler);
	irq_set_handler_data(gctrl->irq, gctrl);

	spin_lock_init(&gctrl->lock);

	pr_info("GPIO lib driver probed\n");

	return 0;
}


#ifdef CONFIG_PM_SLEEP

static int elite_gpio_suspend(void)
{
	int i;
	unsigned long flags;

	local_irq_save(flags);
	
	/* Save input data registers */
	for(i=0; i<REG_NR_INPUT_DAT; i++) {
		gpio_ctrl.indat[i] = elite_gpio_readb(i);
	}

	gpio_ctrl.gpio_usb = elite_gpio_readb(0x3c);
	
	/* enable registers */
	for(i=0; i<REG_NR_EN ; i++) {
		gpio_ctrl.en[i] = elite_gpio_readb(i + 0x40);
	}
	
	gpio_ctrl.en_gp60 = elite_gpio_readb(0x7c);

	/* output enable regsiters */
	for(i=0; i<REG_NR_OUTPUT_EN; i++) {
		gpio_ctrl.outen[i] = elite_gpio_readb(i + 0x80);
	}

	for(i=0; i<REG_NR_OUTPUT_DAT; i++) {
		gpio_ctrl.outdat[i] = elite_gpio_readb(i + 0xc0);
	}

	gpio_ctrl.outdat_gp36 = elite_gpio_readb(0xe4);

	gpio_ctrl.outdat_gp60 = elite_gpio_readb(0xfc);

	gpio_ctrl.usb_op = elite_gpio_readl(REG_USB_OP_MODE);

	gpio_ctrl.pin_share = elite_gpio_readl(REG_PIN_SHARE);

	for(i=0; i<REG_NR_INTERRUPT; i++) {
		gpio_ctrl.intr[i] = elite_gpio_readl(i + 0x300);
	}

	/* pull up/down enable */
	for(i=0; i<REG_NR_PULL_UPDW_EN; i++) {
		gpio_ctrl.pull_updw_en[i] = elite_gpio_readb(i + 0x480);
	}

	gpio_ctrl.pull_updw_en_gp60 = elite_gpio_readb(0x4bc);
	gpio_ctrl.pull_updw_en_gp61 = elite_gpio_readb(0x4bd);

	/* pull up/down control0 */
	for(i=0; i<REG_NR_PULL_UPDW_CTRL0; i++) {
		gpio_ctrl.pull_updw_ctrl0[i] = elite_gpio_readb(i + 0x4c0);
	}

	gpio_ctrl.pull_updw_ctrl0_gp60 = elite_gpio_readb(0x4fc);
	gpio_ctrl.pull_updw_ctrl0_gp61 = elite_gpio_readb(0x4fd);

	/* pull up/down control1 */
	for(i=0; i<REG_NR_PULL_UPDW_CTRL1; i++) {
		gpio_ctrl.pull_updw_ctrl1[i] = elite_gpio_readb(i + 0x5c0);
	}

	gpio_ctrl.pull_updw_ctrl1_gp60 = elite_gpio_readb(0x5fc);
	gpio_ctrl.pull_updw_ctrl1_gp61 = elite_gpio_readb(0x5fd);

	local_irq_restore(flags);

	return 0;
}

static void elite_gpio_resume(void)
{
	int i;
	unsigned long flags;

	local_irq_save(flags);
	
	/* Save input data registers */
	for(i=0; i<REG_NR_INPUT_DAT; i++) {
		elite_gpio_writeb(gpio_ctrl.indat[i], i);
	}

	elite_gpio_writeb(gpio_ctrl.gpio_usb, 0x3c);
	
	/* enable registers */
	for(i=0; i<REG_NR_EN ; i++) {
		elite_gpio_writeb(gpio_ctrl.en[i], i + 0x40);
	}
	
	elite_gpio_writeb(gpio_ctrl.en_gp60, 0x7c);

	/* output enable regsiters */
	for(i=0; i<REG_NR_OUTPUT_EN; i++) {
		elite_gpio_writeb(gpio_ctrl.outen[i], i + 0x80);
	}

	for(i=0; i<REG_NR_OUTPUT_DAT; i++) {
		elite_gpio_writeb(gpio_ctrl.outdat[i], i + 0xc0);
	}

	elite_gpio_writeb(gpio_ctrl.outdat_gp36, 0xe4);

	elite_gpio_writeb(gpio_ctrl.outdat_gp60, 0xfc);

	elite_gpio_writel(gpio_ctrl.usb_op, REG_USB_OP_MODE);

	elite_gpio_writel(gpio_ctrl.pin_share, REG_PIN_SHARE);

	for(i=0; i<REG_NR_INTERRUPT; i++) {
		elite_gpio_writel(gpio_ctrl.intr[i], i + 0x300);
	}

	/* pull up/down enable */
	for(i=0; i<REG_NR_PULL_UPDW_EN; i++) {
		elite_gpio_writeb(gpio_ctrl.pull_updw_en[i], i + 0x480);
	}

	elite_gpio_writeb(gpio_ctrl.pull_updw_en_gp60, 0x4bc);
	elite_gpio_writeb(gpio_ctrl.pull_updw_en_gp61, 0x4bd);

	/* pull up/down control0 */
	for(i=0; i<REG_NR_PULL_UPDW_CTRL0; i++) {
		elite_gpio_writeb(gpio_ctrl.pull_updw_ctrl0[i], i + 0x4c0);
	}

	elite_gpio_writeb(gpio_ctrl.pull_updw_ctrl0_gp60, 0x4fc);
	elite_gpio_writeb(gpio_ctrl.pull_updw_ctrl0_gp61, 0x4fd);

	/* pull up/down control1 */
	for(i=0; i<REG_NR_PULL_UPDW_CTRL1; i++) {
		elite_gpio_writeb(gpio_ctrl.pull_updw_ctrl1[i], i + 0x5c0);
	}

	elite_gpio_writeb(gpio_ctrl.pull_updw_ctrl1_gp60, 0x5fc);
	elite_gpio_writeb(gpio_ctrl.pull_updw_ctrl1_gp61, 0x5fd);

	local_irq_restore(flags);
}

static struct syscore_ops elite_gpio_syscore_ops = {
	.suspend	= elite_gpio_suspend,
	.resume		= elite_gpio_resume,
};
#endif //CONFIG_PM_SLEEP

static struct platform_driver elite_gpio_driver = {
	.driver		= {
		.name	= "elite-gpio",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF_GPIO
		.of_match_table = elite_gpio_of_match,
#endif
	},
	.probe		= elite_gpio_probe,
};

static int __init elite_gpio_init(void)
{
	return platform_driver_register(&elite_gpio_driver);
}
postcore_initcall(elite_gpio_init);

#ifdef CONFIG_PM_SLEEP
static int __init elite_gpio_sysinit(void)
{
	register_syscore_ops(&elite_gpio_syscore_ops);

	return 0;
}

arch_initcall(elite_gpio_sysinit);
#endif

#ifdef	CONFIG_DEBUG_FS
static int dbg_gpio_show(struct seq_file *s, void *unused)
{
	int i;

	for (i = 0; i < 8; i++) {
		seq_printf(s,
			"%d: %02x %02x %02x %02x %02x %02x %06x\n",
			i,
			elite_gpio_bit_readb(REG_INPUT_DAT(i), i));
	}
	return 0;
}

static int dbg_gpio_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_gpio_show, &inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_gpio_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init elite_gpio_debuginit(void)
{
	(void) debugfs_create_file("elite_gpio", S_IRUGO,
					NULL, NULL, &debug_fops);
	return 0;
}
late_initcall(elite_gpio_debuginit);
#endif

