/*
 * linux/drivers/input/keyboard/elite_kpad.c
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
#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/input/matrix_keypad.h>
#include <linux/clk.h>
#include <asm/mach-types.h>
#include <asm/irq.h>
#include <mach/types.h>
#include <mach/iomap.h>
#include <linux/slab.h>
#include "elite_kpad.h"

#define DETECT_RELEASE 0

#define kpad_readl(off)	        __raw_readl(keypad->mmio_base + (off))
#define kpad_writel(off, v)	__raw_writel((v), keypad->mmio_base + (off))
 
 /*4*4 martix */
static unsigned int elite_matrix_keys[] = {
#if 0
	KEY(0, 0, KEY_0), // keycode: 11
	KEY(0, 1, KEY_1), // keycode: 2
	KEY(0, 2, KEY_2), // keycode: 3
#endif
	/* to support Android Recovery */
	KEY(0, 0, KEY_UP), // keycode: 103
	KEY(0, 1, KEY_DOWN), // keycode: 108
	KEY(0, 2, KEY_POWER), // keycode: 116
	KEY(0, 3, KEY_3), // keycode: 4

	KEY(1, 0, KEY_4), // keycode: 5
	KEY(1, 1, KEY_5), // keycode: 6
	KEY(1, 2, KEY_6), // keycode: 7
	KEY(1, 3, KEY_7), // keycode: 8	
	
	KEY(2, 0, KEY_8), // keycode: 9
	KEY(2, 1, KEY_9), // keycode: 10	
	KEY(2, 2, KEY_A), // keycode: 30
	KEY(2, 3, KEY_B), // keycode: 48

	KEY(3, 0, KEY_C), // keycode: 46
	KEY(3, 1, KEY_D), // keycode: 32
	KEY(3, 2, KEY_E), // keycode: 18
	KEY(3, 3, KEY_F)  // keycode: 33
};
    
static unsigned int elite_direct_keys[] ={
	KEY_VOLUMEUP,	// keycode: 115
	KEY_VOLUMEDOWN,	// keycode: 114
	KEY_BACK,	// keycode: 158
	KEY_MENU,	// keycode: 139
};

static void elite_kpad_build_keycode(struct elite_kpad *keypad)
{
	struct input_dev *input_dev = keypad->input_dev;
	unsigned short keycode;
	int i;

	for (i = 0; i < ARRAY_SIZE(elite_matrix_keys); i++) {
		unsigned int key = elite_matrix_keys[i];
		unsigned int row = KEY_ROW(key);
		unsigned int col = KEY_COL(key);
		unsigned int scancode = MATRIX_SCAN_CODE(row, col,
							 MATRIX_ROW_SHIFT);

		keycode = KEY_VAL(key);
		keypad->keycodes[scancode] = keycode;
		__set_bit(keycode, input_dev->keybit);
	}

	for (i = 0; i < ARRAY_SIZE(elite_direct_keys); i++) {
		keycode = elite_direct_keys[i];
		keypad->keycodes[MAX_MATRIX_KEY_NUM + i] = keycode;
		__set_bit(keycode, input_dev->keybit);
	}

	__clear_bit(KEY_RESERVED, input_dev->keybit);
}

static void elite_kpad_scan_matrix(struct elite_kpad *keypad)
{
	struct input_dev *input_dev = keypad->input_dev;
	int row, col, num_keys_pressed = 0;
	uint32_t new_state[MAX_COLUMN];
	uint32_t kpas = kpad_readl(MATRIX_PRI_AUTO_SCAN);


	if(!(kpas & KPC_MI_VALID)) return;
	num_keys_pressed = KPC_MI_MUKP(kpas);

	memset(new_state, 0, sizeof(new_state));

	if (num_keys_pressed == 0)
		goto scan;

	if (num_keys_pressed == 1) {
		col = KPC_MI_CP(kpas);
		row = KPC_MI_RP(kpas);

		/* if invalid row/col, treat as no key pressed */
		if (col >= MAX_COLUMN || row >= MAX_ROW)
			goto scan;

		new_state[col] = (1 << row);
		goto scan;
	}

	if (num_keys_pressed > 1) {
		uint32_t kpasmkp[4];
		int i;
              
		kpasmkp[0] = kpad_readl(MM_SCAN0);
		kpasmkp[1] = kpad_readl(MM_SCAN1);
		kpasmkp[2] = kpad_readl(MM_SCAN2);
		kpasmkp[3] = kpad_readl(MM_SCAN3);

		for(i = 0 ; i < MAX_COLUMN/2; i++) {
			if(kpasmkp[i] & KPC_MI_SCAN_VALID) {
				new_state[2*i] = kpasmkp[i] & 0xff;
				new_state[2*i + 1] = (kpasmkp[i] >> 16) & 0xff;
			}
#if DETECT_RELEASE
			else {
				new_state[2*i] = keypad->matrix_key_state[2*i];
				new_state[2*i + 1] = keypad->matrix_key_state[2*i + 1];
			}
#endif
		}
	}
scan:
	for (col = 0; col < MAX_COLUMN; col++) {
		uint32_t bits_changed;
		int code;
#if DETECT_RELEASE
		bits_changed = keypad->matrix_key_state[col] ^ new_state[col];
#else
		bits_changed = new_state[col];
#endif
		if (bits_changed == 0)
			continue;

		for (row = 0; row < MAX_ROW; row++) {
			if ((bits_changed & (1 << row)) == 0)
				continue;

			code = MATRIX_SCAN_CODE(row, col, MATRIX_ROW_SHIFT);
			input_event(input_dev, EV_MSC, MSC_SCAN, code);
#if DETECT_RELEASE
			input_report_key(input_dev, keypad->keycodes[code],
					 new_state[col] & (1 << row));
#else
			input_report_key(input_dev, keypad->keycodes[code], 1); // press down key
			input_report_key(input_dev, keypad->keycodes[code], 0); // and release it
#endif
		}
	}
	input_sync(input_dev);
	memcpy(keypad->matrix_key_state, new_state, sizeof(new_state));
}

static void elite_kpad_scan_direct(struct elite_kpad *keypad)
{
	struct input_dev *input_dev = keypad->input_dev;
	unsigned int new_state;
	uint32_t kpdk;
        uint32_t bits_changed;

	int i;

	kpdk = kpad_readl(DIRECT_INPUT_SCAN);
        if(!(kpdk & KPC_DI_VALID))return;
    
	new_state = kpdk & keypad->direct_key_mask;
    
#if DETECT_RELEASE
	bits_changed = keypad->direct_key_state ^ new_state;
#else
        bits_changed = new_state;
#endif
	if (bits_changed == 0)
		return;

	for (i = 0; i < MAX_DIRECT_KEY_NUM; i++) {
		if (bits_changed & (1 << i)) {
			int code = MAX_MATRIX_KEY_NUM + i;

			input_event(input_dev, EV_MSC, MSC_SCAN, code);
#if DETECT_RELEASE
			input_report_key(input_dev, keypad->keycodes[code],
					 new_state & (1 << i));
#else
			input_report_key(input_dev, keypad->keycodes[code], 1); // press down key
			input_report_key(input_dev, keypad->keycodes[code], 0); // and release it
#endif
		}
	}
	input_sync(input_dev);
	keypad->direct_key_state = new_state;
}

static irqreturn_t elite_kpad_irq_handler(int irq, void *dev_id)
{
	struct elite_kpad *keypad = dev_id;
	unsigned long st = kpad_readl(KPAD_STATUS);

	if (st & KPC_SDI) {
		elite_kpad_scan_direct(keypad);
		kpad_writel(KPAD_STATUS, KPC_SDI);    /*clear*/

	}
	if (st & KPC_SMI) {
		elite_kpad_scan_matrix(keypad);
		kpad_writel(KPAD_STATUS, KPC_SMI);     /*clear*/

	}
	return IRQ_HANDLED;
}


static void elite_kpad_hw_init(struct elite_kpad *keypad)
{
	unsigned long mc = 0;
	unsigned long dc = 0;
	unsigned long st = 0;
       
	/* enable matrix keys with automatic scan */
	mc = ((MAX_COLUMN - 1) << 8) | ((MAX_ROW - 1) << 12) | KPC_ME | KPC_MIE | KPC_MASACT;

	keypad->direct_key_mask = ((2 <<  ARRAY_SIZE(elite_direct_keys)) - 1) ;

	/* enable direct key */
	dc = KPC_DE | KPC_DIE | KPC_DASACT | (keypad->direct_key_mask << 16);

	/* Clean keypad matrix input control registers. */
	kpad_writel(KPAD_MATRIX_CONTROL, 0);

	/*
	 * Simply disable keypad direct input function first.
	 * Also clear all direct input enable bits.
	 */
	kpad_writel(KPAD_DIRECT_CONTROL, 0);

	/*
	 * Simply clean any exist keypad matrix status.
	 */
	st = kpad_readl(KPAD_STATUS);	
	kpad_writel(KPAD_STATUS, st);
	st = kpad_readl(KPAD_STATUS);
	if (st != 0)
		dev_err(keypad->dev, "failed to clear status register\n");

	kpad_writel(KPAD_MATRIX_CONTROL, mc);
	kpad_writel(KPAD_DIRECT_CONTROL, dc);
	/*
	 * Set keypad debounce time to be about 125 ms.
	 */
	kpad_writel(KPAD_MATRIX_INTERVAL, 0x0FFF | (0x4<<16));
	kpad_writel(KPAD_DIRECT_INTERVAL, 0x0FFF);
}

static int elite_kpad_open(struct input_dev *dev)
{
	struct elite_kpad *keypad = input_get_drvdata(dev);

	/* Enable unit clock */
	clk_enable(keypad->clk);
	elite_kpad_hw_init(keypad);

	return 0;
}

static void elite_kpad_close(struct input_dev *dev)
{
	struct elite_kpad *keypad = input_get_drvdata(dev);

	synchronize_irq(keypad->irq);

	/* Disable clock unit */
	clk_disable(keypad->clk);
}

static int __devinit elite_kpad_probe(struct platform_device *pdev)
{
	struct elite_kpad *keypad;
	struct input_dev *input_dev;
	struct resource *res;
	int irq, error;

	keypad = devm_kzalloc(&pdev->dev, sizeof(struct elite_kpad), GFP_KERNEL);
	if (!keypad) {
		dev_err(&pdev->dev, "Can't alloc memory for KeyPad device\n");
		return -ENOMEM;
	}

        keypad->dev = &pdev->dev;

	platform_set_drvdata(pdev, keypad);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get I/O memory\n");
		return -ENXIO;
	}

	if (!devm_request_mem_region(&pdev->dev, res->start, resource_size(res), pdev->name)) {
		dev_err(&pdev->dev, "failed to request I/O memory\n");
		return -EBUSY;
	}

	keypad->mmio_base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (keypad->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to remap I/O memory\n");
		return -ENOMEM;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get keypad irq\n");
		return -ENXIO;
	}

        error = devm_request_irq(&pdev->dev, irq, elite_kpad_irq_handler, IRQF_DISABLED,
			    pdev->name, keypad);
	if (error) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		return -ENXIO;
	}

	keypad->clk = clk_get(&pdev->dev, "kpad");
	if (IS_ERR(keypad->clk)) {
		dev_err(&pdev->dev, "failed to get keypad clock\n");
		return PTR_ERR(keypad->clk);
	}

	input_dev = input_allocate_device();
	if (!keypad || !input_dev) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		error = -ENOMEM;
		goto failed_put_clk;
	}

        input_dev->open = elite_kpad_open;
	input_dev->close = elite_kpad_close;

	keypad->input_dev = input_dev;
	keypad->irq = irq;
	input_dev->name = pdev->name;
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;
	input_dev->dev.parent = &pdev->dev;

	input_dev->keycode = keypad->keycodes;
	input_dev->keycodesize = sizeof(keypad->keycodes[0]);
	input_dev->keycodemax = ARRAY_SIZE(keypad->keycodes);

	input_set_drvdata(input_dev, keypad);

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);
	input_set_capability(input_dev, EV_MSC, MSC_SCAN);

	elite_kpad_build_keycode(keypad);

	/* Register the input device */
	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed_free;
	}

	/* keypad is incapable of waking up system */
	device_init_wakeup(&pdev->dev, 0);

	dev_info(&pdev->dev, "kpad driver probed done\n");

	return 0;

failed_free:
	input_free_device(input_dev);
failed_put_clk:
	clk_put(keypad->clk);
	return error;
}

static int __devexit elite_kpad_remove(struct platform_device *pdev)
{
	struct elite_kpad *keypad = platform_get_drvdata(pdev);

	clk_put(keypad->clk);
	input_unregister_device(keypad->input_dev);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int elite_kpad_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct elite_kpad *keypad = platform_get_drvdata(pdev);

	clk_disable(keypad->clk);

	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(keypad->irq);

	return 0;
}

static int elite_kpad_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct elite_kpad *kpad = platform_get_drvdata(pdev);
	struct input_dev *input_dev = kpad->input_dev;

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(kpad->irq);

	mutex_lock(&input_dev->mutex);

	if (input_dev->users) {
		/* Enable unit clock */
		clk_enable(kpad->clk);
		elite_kpad_hw_init(kpad);
	}

	mutex_unlock(&input_dev->mutex);

	return 0;
}

#endif
static SIMPLE_DEV_PM_OPS(elite_kpad_pm_ops, elite_kpad_suspend, elite_kpad_resume);
#ifdef CONFIG_OF
static const struct of_device_id elite_kpad_match[] = {
	{ .compatible = "s3graphics,elite1000-kpad" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, elite_kapd_match);
#endif

static struct platform_device_id elite_kpad_driver_ids[] = {
	{
		.name  = "elite-kpad",
		.driver_data	= (kernel_ulong_t)NULL,
	}, {
		.name  = "elite-kpad.0",
		.driver_data	= (kernel_ulong_t)NULL,
	}, { /* sentinel */ },
};
 
static struct platform_driver elite_kpad_driver = {
	.probe		= elite_kpad_probe,
	.remove		= __devexit_p(elite_kpad_remove),
	.id_table	= elite_kpad_driver_ids,
	.driver		= {
		.name	= "elite-kpad",
		.owner	= THIS_MODULE,
		.pm	= &elite_kpad_pm_ops,
		.of_match_table = of_match_ptr(elite_kpad_match),
	},
};
module_platform_driver(elite_kpad_driver);

MODULE_ALIAS("platform:elitexxx_kpad");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("S3 Graphics, Inc.");
MODULE_DESCRIPTION("Kpad driver for S3 E-litexxx SoC");
