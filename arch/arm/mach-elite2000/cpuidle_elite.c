/*
 * arch/arm/mach-elite/cpuidle.c
 *
 * CPU idle driver for Elite 800 cpus
 *
 * Copyright (c) 2011, S3 Graphics INC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/suspend.h>
#include <linux/tick.h>

#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>
#include <asm/localtimer.h>

#include <mach/irqs.h>

//#include "cpu-elite.h"
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <mach/io.h>

unsigned int dbgcmd=0;

unsigned long g_pg_state=0;

static DEFINE_PER_CPU(struct cpuidle_device , idle_devices);

static int elite_idle_enter_cg(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int index);

static int elite_idle_enter_pg(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int index);

extern int power_hardware_debug_init(void);

extern unsigned int elite_power_gating(unsigned int us,struct cpuidle_device *dev);
//============================================
#define GIC_INTERFACE_OPEN 0x01
#define GIC_INTERFACE_CLOSE 0x00


#define WFI_CFG_PG  0x01
#define WFI_CFG_CG  0x00

#define A7_CORE0_WFI_CFG  0xc034
#define A7_CORE1_WFI_CFG  0xc038
#define A7_CORE2_WFI_CFG  0xc03c
#define A7_CORE3_WFI_CFG  0xc040

enum cpu_id {
	cpu0 = 0,
	cpu1,
	cpu2,
	cpu3,
};

union corex_wfi_cfg {
	struct {    
		unsigned int cfgwfi:4;
		unsigned int reserved:28;
	}cfg_wfi;  
	unsigned int val;
};
//============================================
struct cpuidle_driver elite_idle = {
	.name = "elite_idle",
	.owner = THIS_MODULE,
	.state_count = 2,
	.states = {
		[0] = {
			.enter			= elite_idle_enter_cg,
			.exit_latency		= 10,
			.target_residency	= 10,
			.power_usage		= 600,
			.flags			= CPUIDLE_FLAG_TIME_VALID,
			.name			= "CG",
			.desc			= "CPU clock gating",
		},

		[1] = {
			.enter			= elite_idle_enter_pg,
			.exit_latency		= 200,
			.target_residency	= 400,
			.power_usage		= 0,
			.flags			= CPUIDLE_FLAG_TIME_VALID,
			.name			= "PG",
			.desc			= "CPU power gating",
		},


	},
};

static inline void elite_flow_wfi(struct cpuidle_device *dev)
{

	stop_critical_timings();
	dsb();
	__asm__ volatile ("wfi");
	start_critical_timings();
}


void elite_power_cfg(union corex_wfi_cfg corexwficfg,enum cpu_id cpuid)
{
	void __iomem *corex_wfi_cfg_addr;
	corex_wfi_cfg_addr = (void __iomem *)IO_ADDRESS(0xd8390000);;

	switch (cpuid) {
		case cpu0:
			corex_wfi_cfg_addr += A7_CORE0_WFI_CFG; 
			break;
		case cpu1:
			corex_wfi_cfg_addr += A7_CORE1_WFI_CFG; 
			break;
		case cpu2:
			corex_wfi_cfg_addr += A7_CORE2_WFI_CFG; 
			break;
		case cpu3:
			corex_wfi_cfg_addr += A7_CORE3_WFI_CFG; 
			break;
		default:
			BUG();
	}
	__raw_writel(corexwficfg.val,corex_wfi_cfg_addr);
}


static int elite_idle_enter_cg(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int index)
{
	local_irq_disable();
	local_fiq_disable();

	elite_flow_wfi(dev);
	local_fiq_enable();
	local_irq_enable();
	return index;
}


void elite_idle_enter_pg_cpu(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int index,s64 request)
{
	unsigned int intsleep_time;
	struct cpuidle_state *state;
	s64 sleep_time;

	state = &(drv->states[index]);
	sleep_time = request - state->target_residency ;
	intsleep_time = (unsigned int)sleep_time;
	elite_power_gating(intsleep_time,dev);
}

void gic_interface_control(unsigned int control_data)
{
	//base addr is 0xd9008000,but map phy_addr 0xd9009000,so interface addr is 0xd900a000,vir is 0xfe402000
	void __iomem *GIC_ADDR;
	GIC_ADDR = (void __iomem *)IO_ADDRESS(0xd9009000);
	writel(control_data,GIC_ADDR+0x1000);
}

static int elite_idle_enter_pg(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int index)
{
	struct cpuidle_state *state;
	union corex_wfi_cfg wfi_cfg;
	int cpu=dev->cpu;
	s64 request;
	state = &(drv->states[cpu]);

	if (!need_resched() && g_pg_state==1)
	{
		request = ktime_to_us(tick_nohz_get_sleep_length());
		local_irq_disable();
		local_fiq_disable();
		gic_interface_control(GIC_INTERFACE_CLOSE);

		wfi_cfg.cfg_wfi.cfgwfi = WFI_CFG_PG;
		elite_power_cfg(wfi_cfg,cpu);

		elite_idle_enter_pg_cpu(dev, drv,index,request);

		wfi_cfg.cfg_wfi.cfgwfi = WFI_CFG_CG;
		elite_power_cfg(wfi_cfg,cpu);

		gic_interface_control(GIC_INTERFACE_OPEN);
		dsb();

		local_fiq_enable();
		local_irq_enable();
	}
	else
	{  
		return elite_idle_enter_cg(dev, drv,index);
	}

	return index;
}

static int __init elite_cpuidle_init(void)
{
	unsigned int cpu;
	int ret;
	struct cpuidle_device *dev;
	struct cpuidle_driver *drv = &elite_idle;
	void __iomem *PMU_POWER_MONITOR_CTRL;
	void __iomem *A7_CORE0_WFI_EN;

	printk(KERN_DEBUG "%s: \n", __func__);

	ret = cpuidle_register_driver(&elite_idle);
	if (ret) {
		pr_err("CPUidle driver registration failed\n");
		return ret;
	}

	for_each_possible_cpu(cpu) {
		dev = &per_cpu(idle_devices, cpu);
		dev->cpu = cpu;

		dev->state_count = drv->state_count;

		if (cpuidle_register_device(dev)){
			pr_err("CPU%u: error initializing idle loop\n", cpu);
			return ret;
		}
	}
	printk(KERN_DEBUG "%s: start \n", __func__);
	//====================================================================
	PMU_POWER_MONITOR_CTRL = (void __iomem *)IO_ADDRESS(0xd8391100);
	writel(0x01<<1|0X03<<8,PMU_POWER_MONITOR_CTRL);

	A7_CORE0_WFI_EN = (void __iomem *)IO_ADDRESS(0xd813c408);
	__raw_writel(0x04,A7_CORE0_WFI_EN);
	//====================================================================
	power_hardware_debug_init();
	return 0;	
}
device_initcall(elite_cpuidle_init);
