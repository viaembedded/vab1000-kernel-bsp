/*
 * arch/arm/mach-elite/elite_clocks.c
 *
 * Copyright (C) 2012 S3 Graphics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/list_sort.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/reboot.h>

#include "dvfs_elite.h"
#include "cpu-elite.h"

static DEFINE_MUTEX(dvfs_lock);

struct dvfs_rail elite_dvfs_voltage_vdd_cpu = {
	.reg_id = "vdd_cpu",
	.max_millivolts = 1000,
	.min_millivolts = 700,
        .disabled = true,
};

struct regulator {
	struct device *dev;
	struct list_head list;
	int uA_load;
	int min_uV;
	int max_uV;
	char *supply_name;
	struct device_attribute dev_attr;
	struct regulator_dev *rdev;
	struct dentry *debugfs;
};


/* Sets the voltage on a dvfs rail to a specific value, and updates any
 * rails that depend on this rail. */
int dvfs_rail_set_voltage(struct dvfs_rail *rail)
{
    int ret = 0;
    int min_millivolts,max_millivolts;
    int old_min_uv,old_max_uv;
    int v;

         //printk(KERN_DEBUG "************%s: start ! rail->reg = %x***************** \n",__func__,(unsigned int)&rail->reg);
   if (!rail->reg) {
        return -EINVAL;
    }

    //if (rail->disabled)
    //    return 0;
    rail->updating = true;
    min_millivolts = max_millivolts = rail->updating_millivolts;
    old_min_uv=rail->reg->min_uV;
    old_max_uv=rail->reg->max_uV;
    //printk(KERN_DEBUG "******************%s: try to set voltage to %d ********************** \n",__func__,min_millivolts);
    ret = regulator_set_voltage(rail->reg,min_millivolts,max_millivolts);
    //printk(KERN_DEBUG "******************%s: ret =%x  ********************** \n",__func__,ret);
    rail->updating = false;
    if (ret) {
	rail->reg->min_uV=old_min_uv;
	rail->reg->max_uV=old_max_uv;
        //pr_err("##%d:Failed to set dvfs regulator %s\n",ret,rail->reg_id);
        //printk("******************%s: failed to set voltage to %d ********************** \n",__func__,min_millivolts);
       goto out;
    } 
    //else
    //    printk(KERN_DEBUG "******************%s: set voltage to %d ********************** \n",__func__,min_millivolts);
    //verifivation:
	v = regulator_get_voltage(rail->reg);
	if (v < 0) {
		rail->reg->min_uV=old_min_uv;
                rail->reg->max_uV=old_max_uv;
		pr_err("elite_dvfs: failed to get %s voltage\n",
		       rail->reg_id);
		return v;
	}
    //printk(KERN_DEBUG "******************%s: verify :voltage = %d ! ********************** \n",__func__,(unsigned int)v);

    if(v!=rail->updating_millivolts)
    {
       rail->reg->min_uV=old_min_uv;
       rail->reg->max_uV=old_max_uv;
       //printk("******************%s: verify :voltage = %d: set :voltage = %d ! ********************** \n",__func__,(unsigned int)v,(unsigned int)rail->updating_millivolts);
        return -1;
    }


    rail->current_millivolts = rail->updating_millivolts;

out:
	return ret;
}

/*
  make menuconfig
  Device Driver -->
* Voltage and Current Regulator Support -->
* wolfson microsoftelectronics WM931x PMIC regulators
*/
static int dvfs_rail_connect_to_regulator(struct dvfs_rail *rail)
{
	struct regulator *reg;
	int v;

	if (!rail->reg) {
		reg = regulator_get(NULL, rail->reg_id);
		if (IS_ERR(reg)) {
			pr_err("elite_dvfs: failed to connect %s rail\n",
			       rail->reg_id);
			return -EINVAL;
		}
        rail->reg = reg;
        //printk(KERN_DEBUG "******************%s: rail->reg = %x ! ********************** \n",__func__,(unsigned int)rail->reg);
	}

	v = regulator_get_voltage(rail->reg);
	if (v < 0) {
		pr_err("elite_dvfs: failed initial get %s voltage\n",
		       rail->reg_id);
		return v;
	}
	rail->current_millivolts = v / 1000;
        rail->disabled = false;  //enable rail
        //printk(KERN_DEBUG "******************%s: voltage = %d ! ********************** \n",__func__,(unsigned int)v);
	return 0;
}


bool debug_dvfs = false;
int __init elite_dvfs_late_init(void)
{
    bool connected = true;

    printk(KERN_DEBUG "******************%s: start ********************** \n",__func__);
    if(debug_dvfs)
        return 0;
    mutex_lock(&dvfs_lock);

    if (dvfs_rail_connect_to_regulator(&elite_dvfs_voltage_vdd_cpu))
        connected = false;

    if (!connected)
        pr_err("elite_dvfs: failed on connect_to_regulator\n");
    else 
        printk(KERN_DEBUG "******************%s: dvfs_rail_connect_to_regulator success ! ********************** \n",__func__);
    mutex_unlock(&dvfs_lock);

    return 0;
}
late_initcall(elite_dvfs_late_init);





