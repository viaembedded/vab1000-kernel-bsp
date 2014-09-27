/*
 * Touch Screen driver for ILI's I2C connected touch screen panels
 *   Copyright (c) 2009 Daniel Mack <daniel@caiaq.de>
 *
 * See ILI's software guide for the protocol specification:
 *   http://home.eeti.com.tw/web20/eg/guide.htm
 *
 * Based on migor_ts.c
 *   Copyright (c) 2008 Magnus Damm
 *   Copyright (c) 2007 Ujjwal Pande <ujjwal@kenati.com>
 *
 * This file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU  General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <asm/io.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/kthread.h>

struct ili_finger {
	int status;
	int x;
	int y;
};

struct ili_ts_priv {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
	struct mutex mutex;
	struct ili_finger finger[2];
	int irq;
        struct task_struct *thread;
	struct timer_list timer;
	int in_irq;
	int use_irq;
};

static int ili_ts_report(struct ili_ts_priv *priv);
#define ILI_TS_BITDEPTH		(11)
#define ILI_MAXXVAL		(0x1300)
#define ILI_MAXYVAL		(0xb00)

static void ili_ts_timer(unsigned long handle)
{
    	struct ili_ts_priv *priv = (void *)handle;
    	ili_ts_report(priv);
}

static int ili_ts_report(struct ili_ts_priv *priv)
{
	struct i2c_client *client = priv->client;
	struct input_dev *input = priv->input;
	struct ili_finger *finger = priv->finger;
	int ret;

	int finger_num;
	int status = 0;
	int x[2] = {0}; 
	int y[2] = {0};

	u8 buf_reg[1] = {0x10};
	u8 buf_data[10] = {0};


	do {
		ret = i2c_master_send(client, buf_reg, 1);
		if (ret != 1) {
			//printk("touchscreen: ilitek i2c write fail \n");
		}
		ret = i2c_master_recv(client, buf_data, 9);
		x[0] = *(u16 *)(&buf_data[1]);
		y[0] = *(u16 *)(&buf_data[3]);
		x[1] = *(u16 *)(&buf_data[5]);
		y[1] = *(u16 *)(&buf_data[7]);

		status = buf_data[0] & 0x3;
		if(status == 0x0) {
			finger_num = 0;
			finger[0].status = 0;
			finger[0].status = 0;
		} else if (status == 0x1) {
			finger_num = 1;
			finger[0].status = 1;
			finger[1].status = 0;
			finger[0].x = x[0];
			finger[0].y = y[0];

		} else if (status == 0x2) {
			finger_num = 1;
			finger[0].status = 0;
			finger[1].status = 1;
			finger[1].x = x[1];
			finger[1].y = y[1];
		} else if (status == 0x3) {
			finger_num = 2;
			finger[0].status = 1;
			finger[1].status = 1;
			finger[0].x = x[0];
			finger[0].y = y[0];
			finger[1].x = x[1];
			finger[1].y = y[1];
		} else {
			printk("finger status is %d\n", status);
		}
	} while(priv->in_irq && finger[0].status == 0);

#ifndef ILITEK_MULTI_OUTCH
	if(finger[0].status) {
		input_report_abs(input, ABS_X, finger[0].x);
		input_report_abs(input, ABS_Y, finger[0].y);
		input_report_key(input, BTN_TOUCH, 1);
		input_sync(input);
		if(priv->use_irq)
			mod_timer(&priv->timer, jiffies + msecs_to_jiffies(50));
	} else {
		input_report_key(input, BTN_TOUCH, 0);
		input_sync(input);
	}
#else

	for(i = 0; i < 2; i++) {
		if(!finger[i].status)
			continue;
		input_report_abs(input, ABS_MT_TRACKING_ID, i);
		input_report_abs(input, ABS_MT_POSITION_X, finger[i].x);
		input_report_abs(input, ABS_MT_POSITION_Y, finger[i].y);
		input_report_abs(input, ABS_MT_TOUCH_MAJOR, 1);
		input_mt_sync(input);
		if(priv->use_irq)
			mod_timer(&priv->timer, jiffies + msecs_to_jiffies(50));
	}
		input_report_key(input, BTN_TOUCH, finger_num > 0);
	if(status == 0) {
		input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
		input_mt_sync(input);
	}


	input_sync(input);
#endif
	return 0;
}

static irqreturn_t ili_ts_isr(int irq, void *dev_id)
{
	//struct ili_ts_priv *priv = dev_id;
	*(u16 *)0xd80a8504 = 0x1;
	writel(0x1, IO_ADDRESS(0xd80a8504));
	disable_irq_nosync(irq);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t ili_ts_isr_thread(int irq, void *dev_id)
{
	struct ili_ts_priv *priv = (struct ili_ts_priv *)dev_id;
	*(u16 *)0xd80a8504 = 0x1;
	writew(0x1, IO_ADDRESS(0xd80a8504));
	ili_ts_report(priv);
	return IRQ_HANDLED;
}

static int ili_ts_polling_thread(void *arg)
{
	struct ili_ts_priv *priv = (struct ili_ts_priv *)arg;
	printk("ili_ts_polling_thread\n");
	while(1) {
		if(kthread_should_stop()){
			printk("ili_ts_polling_thread stop\n");
			break;
		}
		ili_ts_report(priv);
		msleep_interruptible(100);
	}
	printk("ili_ts_polling_thread exit\n");
	return 0;
}

static int __devinit ili_ts_probe(struct i2c_client *client,
				   const struct i2c_device_id *idp)
{
	struct ili_ts_priv *priv;
	struct input_dev *input;
	int err = -ENOMEM;
	int use_irq = 0;
	printk("elite_ili_probe\n");

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		goto err0;
	}

	mutex_init(&priv->mutex);
	input = input_allocate_device();

	if (!input) {
		dev_err(&client->dev, "Failed to allocate input device.\n");
		goto err1;
	}

	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);

#ifndef ILITEK_MULTI_OUTCH
	input_set_abs_params(input, ABS_X, 0, ILI_MAXXVAL, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, ILI_MAXYVAL, 0, 0);
#else
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, ILI_MAXXVAL, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ILI_MAXYVAL, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 0xff, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 2, 0, 0);
#endif

	input->name = "ilitek";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	input_set_drvdata(input, priv);

	priv->client = client;
	priv->input = input;
//	priv->irq = client->irq;
	priv->irq = IRQ_GPIO;//88;
	priv->use_irq = use_irq;

	i2c_set_clientdata(client, priv);

	err = input_register_device(input);
	if (err)
		goto err1;

	if(use_irq) {
		init_timer(&priv->timer);
		priv->timer.data = (unsigned long)priv;
		priv->timer.function = ili_ts_timer;

		*(u16 *)0xd80a8504 = 0x1;
		writew(0x1, IO_ADDRESS(0xd80a8504));
		writel(0x2000, IO_ADDRESS(0xd839c408));
		writel(0x3000, IO_ADDRESS(0xd839c408));
		err = request_threaded_irq(IRQ_GPIO, NULL,ili_ts_isr_thread, IRQF_SHARED | IRQF_ONESHOT, client->name, priv);

		if (err) {
			dev_err(&client->dev, "Unable to request touchscreen IRQ. fail %d.\n", err);
			goto err2;
		}
	} else {
		priv->thread = kthread_create(ili_ts_polling_thread, priv, "ili_ts_thread");
		if (IS_ERR(priv->thread)) {
			priv->thread = NULL;
			printk("Unable create ili_ts_thread\n");
		} else {
			wake_up_process(priv->thread);
		}
	}
	return 0;

err2:
	input_unregister_device(input);
	input = NULL; /* so we dont try to free it below */
err1:
	input_free_device(input);
	kfree(priv);
err0:
	return err;
}

static int __devexit ili_ts_remove(struct i2c_client *client)
{
	struct ili_ts_priv *priv = i2c_get_clientdata(client);

	free_irq(priv->irq, priv);
	/*
	 * ili_ts_stop() leaves IRQ disabled. We need to re-enable it
	 * so that device still works if we reload the driver.
	 */
	input_unregister_device(priv->input);
	kfree(priv);

	return 0;
}

#ifdef CONFIG_PM
static int ili_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int ili_ts_resume(struct i2c_client *client)
{
	return 0;
}
#else
#define ili_ts_suspend NULL
#define ili_ts_resume NULL
#endif

#ifdef CONFIG_OF
static const struct of_device_id elite_touchscreen_match[] = {
	{ .compatible = "s3graphics,elite1000-touchscreen" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, elite_touchscreen_match);
#endif

static const struct i2c_device_id ili_ts_id[] = {
	{ "ilitek", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ili_ts_id);

static struct i2c_driver ili_ts_driver = {
	.driver = {
		.name = "ilitek",
		.of_match_table = of_match_ptr(elite_touchscreen_match),
	},
	.probe = ili_ts_probe,
	.remove = __devexit_p(ili_ts_remove),
	.suspend = ili_ts_suspend,
	.resume = ili_ts_resume,
	.id_table = ili_ts_id,
};

static int __init ili_ts_init(void)
{
	printk("ili_ts_init\n");
	return i2c_add_driver(&ili_ts_driver);
}

static void __exit ili_ts_exit(void)
{
	i2c_del_driver(&ili_ts_driver);
}

MODULE_DESCRIPTION("ILI Touchscreen driver");
MODULE_AUTHOR("Daniel Mack <daniel@caiaq.de>");
MODULE_LICENSE("GPL");

module_init(ili_ts_init);
module_exit(ili_ts_exit);

