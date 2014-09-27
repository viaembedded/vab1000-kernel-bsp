/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mfd/core.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/regulator/consumer.h>
#include <sound/soc.h>
#include <linux/slimbus/slimbus.h>
#define TABLA_SLIM_GLA_MAX_RETRIES 5
#define TABLA_REGISTER_START_OFFSET 0x800
#define TABLA_SLIM_RW_MAX_TRIES 3

#define MAX_TABLA_DEVICE	4
#define TABLA_I2C_MODE	0x03
struct tabla {
	struct device *dev;
	struct slim_device *slim;
	struct slim_device *slim_slave;
	struct mutex io_lock;
	struct mutex xfer_lock;
	struct mutex irq_lock;
	u8 version;


	int reset_gpio;

	int (*read_dev)(struct tabla *tabla, unsigned short reg,
			int bytes, void *dest, bool interface_reg);
	int (*write_dev)(struct tabla *tabla, unsigned short reg,
			 int bytes, void *src, bool interface_reg);

};

static int tabla_intf;

static int tabla_read(struct tabla *tabla, unsigned short reg,
		       int bytes, void *dest, bool interface_reg)
{
	int ret;
	u8 *buf = dest;

	if (bytes <= 0) {
		dev_err(tabla->dev, "Invalid byte read length %d\n", bytes);
		return -EINVAL;
	}

	ret = tabla->read_dev(tabla, reg, bytes, dest, interface_reg);
	if (ret < 0) {
		dev_err(tabla->dev, "Tabla read failed\n");
		return ret;
	} else
		dev_dbg(tabla->dev, "Read 0x%02x from R%d(0x%x)\n",
			 *buf, reg, reg);

	return 0;
}
int tabla_reg_read(struct tabla *tabla, unsigned short reg)
{
	u8 val;
	int ret;

	mutex_lock(&tabla->io_lock);
	ret = tabla_read(tabla, reg, 1, &val, false);
	mutex_unlock(&tabla->io_lock);

	if (ret < 0)
		return ret;
	else
		return val;
}
EXPORT_SYMBOL_GPL(tabla_reg_read);

static int tabla_write(struct tabla *tabla, unsigned short reg,
			int bytes, void *src, bool interface_reg)
{
	u8 *buf = src;

	if (bytes <= 0) {
		pr_err("%s: Error, invalid write length\n", __func__);
		return -EINVAL;
	}

	dev_dbg(tabla->dev, "Write %02x to R%d(0x%x)\n",
		 *buf, reg, reg);

	return tabla->write_dev(tabla, reg, bytes, src, interface_reg);
}

int tabla_reg_write(struct tabla *tabla, unsigned short reg,
		     u8 val)
{
	int ret;

	mutex_lock(&tabla->io_lock);
	ret = tabla_write(tabla, reg, 1, &val, false);
	mutex_unlock(&tabla->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tabla_reg_write);

static u8 tabla_pgd_la;
static u8 tabla_inf_la;

int tabla_get_logical_addresses(u8 *pgd_la, u8 *inf_la)
{
	*pgd_la = tabla_pgd_la;
	*inf_la = tabla_inf_la;
	return 0;

}
EXPORT_SYMBOL_GPL(tabla_get_logical_addresses);

int tabla_interface_reg_read(struct tabla *tabla, unsigned short reg)
{
	u8 val;
	int ret;

	mutex_lock(&tabla->io_lock);
	ret = tabla_read(tabla, reg, 1, &val, true);
	mutex_unlock(&tabla->io_lock);

	if (ret < 0)
		return ret;
	else
		return val;
}
EXPORT_SYMBOL_GPL(tabla_interface_reg_read);

int tabla_interface_reg_write(struct tabla *tabla, unsigned short reg,
		     u8 val)
{
	int ret;

	mutex_lock(&tabla->io_lock);
	ret = tabla_write(tabla, reg, 1, &val, true);
	mutex_unlock(&tabla->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tabla_interface_reg_write);

int tabla_bulk_read(struct tabla *tabla, unsigned short reg,
		     int count, u8 *buf)
{
	int ret;

	mutex_lock(&tabla->io_lock);

	ret = tabla_read(tabla, reg, count, buf, false);

	mutex_unlock(&tabla->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tabla_bulk_read);

int tabla_bulk_write(struct tabla *tabla, unsigned short reg,
		     int count, u8 *buf)
{
	int ret;

	mutex_lock(&tabla->io_lock);

	ret = tabla_write(tabla, reg, count, buf, false);

	mutex_unlock(&tabla->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tabla_bulk_write);

static int tabla_slim_read_device(struct tabla *tabla, unsigned short reg,
				int bytes, void *dest, bool interface)
{
	int ret;
	struct slim_ele_access msg;
	int slim_read_tries = TABLA_SLIM_RW_MAX_TRIES;
	msg.start_offset = TABLA_REGISTER_START_OFFSET + reg;
	msg.num_bytes = bytes;
	msg.comp = NULL;

	while (1) {
		mutex_lock(&tabla->xfer_lock);
		ret = slim_request_val_element(interface ?
					       tabla->slim_slave : tabla->slim,
					       &msg, dest, bytes);
		mutex_unlock(&tabla->xfer_lock);
		if (likely(ret == 0) || (--slim_read_tries == 0))
			break;
		usleep_range(5000, 5000);
	}

	if (ret)
		pr_err("%s: Error, Tabla read failed (%d)\n", __func__, ret);

	return ret;
}
/* Interface specifies whether the write is to the interface or general
 * registers.
 */
static int tabla_slim_write_device(struct tabla *tabla, unsigned short reg,
				   int bytes, void *src, bool interface)
{
	int ret;
	struct slim_ele_access msg;
	int slim_write_tries = TABLA_SLIM_RW_MAX_TRIES;
	msg.start_offset = TABLA_REGISTER_START_OFFSET + reg;
	msg.num_bytes = bytes;
	msg.comp = NULL;

	while (1) {
		mutex_lock(&tabla->xfer_lock);
		ret = slim_change_val_element(interface ?
					      tabla->slim_slave : tabla->slim,
					      &msg, src, bytes);
		mutex_unlock(&tabla->xfer_lock);
		if (likely(ret == 0) || (--slim_write_tries == 0))
			break;
		usleep_range(5000, 5000);
	}

	if (ret)
		pr_err("%s: Error, Tabla write failed (%d)\n", __func__, ret);

	return ret;
}


static int tabla_slim_probe(struct slim_device *slim)
{
	struct tabla *tabla;
	int ret = 0;
	int sgla_retry_cnt;

//	dev_info(&slim->dev, "Initialized slim device %s\n", slim->name);
	printk("Initialized slim device %s\n", slim->name);
#if 0	
//	pdata = slim->dev.platform_data;

	if (!pdata) {
		dev_err(&slim->dev, "Error, no platform data\n");
		ret = -EINVAL;
		goto err;
	}
#endif
	tabla = kzalloc(sizeof(struct tabla), GFP_KERNEL);
	if (tabla == NULL) {
		pr_err("%s: error, allocation failed\n", __func__);
		ret = -ENOMEM;
		goto err;
	}
	if (!slim->ctrl) {
		pr_err("Error, no SLIMBUS control data\n");
		ret = -EINVAL;
		goto err_tabla;
	}
	tabla->slim = slim;
	slim_set_clientdata(slim, tabla);
	tabla->dev = &slim->dev;
	
	usleep_range(5, 5);
	

/*
	ret = slim_get_logical_addr(tabla->slim, tabla->slim->e_addr,
		ARRAY_SIZE(tabla->slim->e_addr), &tabla->slim->laddr);
	if (ret) {
		pr_err("fail to get slimbus logical address %d\n", ret);
		goto err;
	}
*/	
	tabla->read_dev = tabla_slim_read_device;
	tabla->write_dev = tabla_slim_write_device;

	{
		int ret;
		int src[16];
	struct slim_ele_access msg;
	unsigned short reg;
	 int bytes;
	int slim_write_tries = TABLA_SLIM_RW_MAX_TRIES;
	reg=4;
	bytes=1;
	msg.start_offset = TABLA_REGISTER_START_OFFSET + reg;
	msg.num_bytes = bytes;
	msg.comp = NULL;


	ret = slim_change_val_element(tabla->slim,
					      &msg, src, bytes);
	
	printk("%s: Error, Tabla write failed (%d)\n", __func__, ret);
	}

//	tabla_pgd_la = tabla->slim->laddr;

	//tabla->slim_slave = &pdata->slimbus_slave_device;

	//ret = slim_add_device(slim->ctrl, tabla->slim_slave);
	if (ret) {
		pr_err("%s: error, adding SLIMBUS device failed\n", __func__);
		goto err;
	}

	sgla_retry_cnt = 0;

	while (1) {
		ret = slim_get_logical_addr(tabla->slim_slave,
			tabla->slim_slave->e_addr,
			ARRAY_SIZE(tabla->slim_slave->e_addr),
			&tabla->slim_slave->laddr);
		if (ret) {
			if (sgla_retry_cnt++ < TABLA_SLIM_GLA_MAX_RETRIES) {
				/* Give SLIMBUS slave time to report present
				   and be ready.
				 */
				usleep_range(1000, 1000);
				pr_debug("%s: retry slim_get_logical_addr()\n",
					__func__);
				continue;
			}
			pr_err("fail to get slimbus slave logical address"
				" %d\n", ret);
			goto err_slim_add;
		}
		break;
	}


	return ret;

err_slim_add:
	slim_remove_device(tabla->slim_slave);

err_tabla:
	kfree(tabla);
err:
	return ret;
}
struct tabla_pdata {
	struct slim_device slimbus_slave_device;
	
};
static struct tabla_pdata tabla_platform_data = {
	.slimbus_slave_device = {
		.name = "tabla-slave",
		.e_addr = {0, 0, 0x10, 0, 0x17, 2},
	},
	
};
struct slim_device wcd5102_slim_tabla = {
	.name = "wcd5102-slim",
	.e_addr = {0, 1, 0x10, 0, 0x17, 2},
	.dev = {
		.platform_data = &tabla_platform_data,
	},
};

 int wcd5102_probe(void)
 {
 	struct slim_device *slim;
	struct tabla *tabla;
	int ret = 0;
	int sgla_retry_cnt;
	slim=&wcd5102_slim_tabla;
	
	printk( "wcd5102_probe \n");
	printk( "Initialized slim device %s\n", slim->name);
	
	tabla = kzalloc(sizeof(struct tabla), GFP_KERNEL);
	if (tabla == NULL) {
		printk("%s: error, allocation failed\n", __func__);
		ret = -ENOMEM;
		goto err;
	}
	if (!slim->ctrl) {
		printk("Error, no SLIMBUS control data\n");
		ret = -EINVAL;
		goto err_tabla;
	}
	tabla->slim = slim;
	slim_set_clientdata(slim, tabla);
	tabla->dev = &slim->dev;
	
	usleep_range(5, 5);

	ret = slim_get_logical_addr(tabla->slim, tabla->slim->e_addr,
		ARRAY_SIZE(tabla->slim->e_addr), &tabla->slim->laddr);
	if (ret) {
		printk("fail to get slimbus logical address %d\n", ret);
		goto err;
	}
	tabla->read_dev = tabla_slim_read_device;
	tabla->write_dev = tabla_slim_write_device;

	tabla_pgd_la = tabla->slim->laddr;
	
	printk("tabla_pgd_la = %d \n",tabla_pgd_la);
	//tabla->slim_slave = &pdata->slimbus_slave_device;

	//ret = slim_add_device(slim->ctrl, tabla->slim_slave);
	if (ret) {
		printk("%s: error, adding SLIMBUS device failed\n", __func__);
		goto err;
	}

	sgla_retry_cnt = 0;

	while (1) {
		ret = slim_get_logical_addr(tabla->slim_slave,
			tabla->slim_slave->e_addr,
			ARRAY_SIZE(tabla->slim_slave->e_addr),
			&tabla->slim_slave->laddr);
		if (ret) {
			if (sgla_retry_cnt++ < TABLA_SLIM_GLA_MAX_RETRIES) {
				/* Give SLIMBUS slave time to report present
				   and be ready.
				 */
				usleep_range(1000, 1000);
				pr_debug("%s: retry slim_get_logical_addr()\n",
					__func__);
				continue;
			}
			printk("fail to get slimbus slave logical address"
				" %d\n", ret);
			goto err_slim_add;
		}
		break;
	}


	return ret;

err_slim_add:
	slim_remove_device(tabla->slim_slave);

err_tabla:
	kfree(tabla);
err:
	return ret;
}

static int tabla_slim_remove(struct slim_device *pdev)
{
	struct tabla *tabla;

	tabla = slim_get_devicedata(pdev);
	slim_remove_device(tabla->slim_slave);
	return 0;
}

static int tabla_resume(struct tabla *tabla)
{
	int ret = 0;

	return ret;
}

static int tabla_slim_resume(struct slim_device *sldev)
{
	struct tabla *tabla = slim_get_devicedata(sldev);
	return tabla_resume(tabla);
}



static int tabla_suspend(struct tabla *tabla, pm_message_t pmesg)
{
	int ret = 0;

	return ret;
}

static int tabla_slim_suspend(struct slim_device *sldev, pm_message_t pmesg)
{
	struct tabla *tabla = slim_get_devicedata(sldev);
	return tabla_suspend(tabla, pmesg);
}



static const struct slim_device_id slimtest_id[] = {
	{"wcd5102-slim", 0},
	{}
};

static struct slim_driver tabla_slim_driver = {
	.driver = {
		.name = "wcd5102-slim",
		.owner = THIS_MODULE,
	},
	.probe = tabla_slim_probe,
	.remove = tabla_slim_remove,
	.id_table = slimtest_id,
	.resume = tabla_slim_resume,
	.suspend = tabla_slim_suspend,
};

MODULE_DEVICE_TABLE(i2c, tabla_id_table);

static int __init tabla_init(void)
{
	int ret1, ret2, ret3;

	printk("slim wcd5102\n");
	ret1 = slim_driver_register(&tabla_slim_driver);
	if (ret1 != 0)
		printk("Failed to register tabla SB driver: %d\n", ret1);

	//ret2 = slim_driver_register(&tabla2x_slim_driver);
//	if (ret2 != 0)
	//	printk("Failed to register tabla2x SB driver: %d\n", ret2);
	printk("slim wcd5102 end =%d\n",ret1);
//	wcd5102_probe();

	return (ret1 && ret2 && ret3) ? -1 : 0;
}
module_init(tabla_init);

static void __exit tabla_exit(void)
{
}
module_exit(tabla_exit);

MODULE_DESCRIPTION("Tabla core driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
