/*
 *  linux/arch/arm/mach-elite/headsmp.S
 *
 *  Copyright (c) 2003 ARM Limited
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/assembler.h>


#define ELITE_ARM_PERIF_BASE            0xD8018000

/*
 *	elite_pg_startup
 *
 */
ENTRY(elite_pg_startup)
	bl	__invalidate_cpu_state

	mrc	p15, 0, r0, c1, c0, 1
	orr	r0, r0, #(1 << 6) | (1 << 0)	@ re-enable coherency
	mcr	p15, 0, r0, c1, c0, 1

	adr	r4, __elite_pg_data
	ldmia	r4, {r5, r7, r12}
	mov	r1, r12
	sub	r4, r4, r5
	ldr	r0, [r7, r4]
	b	__return_to_virtual
ENDPROC(elite_pg_startup)

	.type	__elite_pg_data, %object
__elite_pg_data:
	.long	.
	.long	elite_pgd_phys
	.long	__cortex_a9_restore
	.size	__elite_pg_data, . - __elite_pg_data


/*
 * Elite specific entry point for secondary CPUs.  This provides
 * a "holding pen" into which all secondary cores are held until we're
 * ready for them to initialise.
 */
#if defined(CONFIG_SMP)
ENTRY(elite_secondary_startup)
	bl      __invalidate_cpu_state
	b	secondary_startup
#endif /* defined(CONFIG_SMP) */

ENTRY(elite_hotplug_startup)
	setmode	PSR_F_BIT | PSR_I_BIT | SVC_MODE, r9
	bl      __invalidate_cpu_state

	adr	r4, elite_hotplug_data
	ldmia	r4, {r5, r7, r12}
	mov	r1, r12
	sub	r4, r4, r5
	ldr	r0, [r7, r4]
	b	__return_to_virtual
ENDPROC(elite_hotplug_startup)

	.type	elite_hotplug_data, %object
elite_hotplug_data:
	.long	.
	.long	elite_pgd_phys
	.long	__cortex_a9_restore
	.size	eltie_hotplug_data, . - elite_hotplug_data

