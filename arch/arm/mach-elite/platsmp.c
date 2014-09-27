/*
 *  linux/arch/arm/mach-elite/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <asm/mach-types.h>
#include <asm/localtimer.h>
#include <asm/unified.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <asm/smp_scu.h>
#include "cpu-elite.h"
#include "elite-pg.h"
#include <asm/hardware/gic.h>
#include "scm.h"


extern void elite_secondary_startup(void);
extern void elite_hotplug_startup(void);
extern void elite_turn_on_neon(int cpu);
/*
 * control for which core is the next to come out of the secondary
 * boot "holding pen"
 */

static void __iomem *scu_base_addr = (void __iomem *)IO_ADDRESS(ELITE_ARM_PERIF_SCU_BASE);
static DEFINE_SPINLOCK(boot_lock);

static DECLARE_BITMAP(cpu_init_bits, CONFIG_NR_CPUS) __read_mostly;
const struct cpumask *const cpu_init_mask = to_cpumask(cpu_init_bits);
#define cpu_init_map (*(cpumask_t *)cpu_init_mask)


extern void elite_set_pg_timer(unsigned int us,int cpu);
extern void elite_cpu_wfi_cfg(int cpu, int mode);


static inline unsigned int get_core_count(void)
{
	void __iomem *scu_base = scu_base_addr;
	if (scu_base) {
		return scu_get_core_count(scu_base);
	}
	return 1;
}

unsigned int get_boot_vector(int cpu)
{
	if (!svc_system_is_secureboot()) {
		return readl((char*)ELITE_PMU_BASE + MCSBOOT0 + cpu*4);
	} else {

		unsigned int address;
		svc_system_get_boot_vector(cpu, &address);
		return address;
	}
}


void wakeup_secondary(int cpu, unsigned int boot_vector)
{
	if  (!svc_system_is_secureboot()) {
		volatile unsigned int *general_cfg_status;
		unsigned int cpu_status;
		general_cfg_status = (volatile unsigned int*)((char*)ELITE_PMU_BASE + GENERAL_CFG0_STATUS);

		cpu_status = ((*general_cfg_status) >> ((3-cpu)*4));

		if((cpu_status & 0xa) == 0xa)
		{
			elite_set_pg_timer(0, cpu);
			udelay(10);
			elite_set_pg_timer(0xbeef,cpu);
		}

		*(volatile unsigned int*)((char*)ELITE_PMU_BASE + MCSBOOT0  + cpu*4) = boot_vector;

        	__asm__ volatile ("sev");
	} else {
		svc_system_boot_secondary(cpu, boot_vector);
	}
}


void __cpuinit platform_secondary_init(unsigned int cpu)
{
	trace_hardirqs_off();

	gic_secondary_init(0);

	spin_lock(&boot_lock);

	cpu_set(cpu, cpu_init_map);

	spin_unlock(&boot_lock);

}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;
	unsigned int boot_vector;

	/*
	 * set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	flush_cache_all();

#ifdef CONFIG_PM
	if (cpumask_test_cpu(cpu, cpu_init_mask))
	{
		boot_vector = virt_to_phys(elite_hotplug_startup);
	}
	else
#endif
	{
		boot_vector = virt_to_phys(elite_secondary_startup);
	}

	smp_wmb();

	wakeup_secondary(cpu, boot_vector);

	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {

		if (boot_vector != get_boot_vector(cpu))
			break;

		udelay(10);
	}

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return 0;
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init smp_init_cpus(void)
{
	unsigned int i, ncores = get_core_count();

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);

	set_smp_cross_call(gic_raise_softirq);
}

void __init platform_smp_prepare_cpus(unsigned int max_cpus)
{
	scu_enable(scu_base_addr);
}

#ifdef CONFIG_HOTPLUG_CPU
int platform_cpu_kill(unsigned int cpu)
{
	return 1;
}

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
void platform_cpu_die(unsigned int cpu)
{
	printk(KERN_NOTICE "CPU%u: shutdown\n", cpu);

	flush_cache_all();
	barrier();

#ifdef CONFIG_PM

	elite_cpu_wfi_cfg(cpu, WFI_CFG_ENTER_MULTICORE_LP_MODE);

	writel(0xa, (void*)((char*)ELITE_PMU_BASE+CPU_LP_CFG_CORE1));

	flush_cache_all();

	barrier();

	*(volatile unsigned int*)((char*)ELITE_PMU_BASE + MCSBOOT0  + cpu*4) = 0;

	__cortex_a9_save(virt_to_phys(elite_hotplug_startup), 0);

	elite_cpu_wfi_cfg(cpu, 0);

	writel(0x0, (void*)((char*)ELITE_PMU_BASE+CPU_LP_CFG_CORE1));
	*(volatile unsigned int*)((char*)ELITE_PMU_BASE + MCSBOOT0  + cpu*4) = 0;

#else //not define CONFIG_PM
	__asm__ volatile("wfi");
#endif
	barrier();

}

int platform_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	return cpu == 0 ? -EPERM : 0;
}

#endif

