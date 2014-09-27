/*
 *  linux/arch/arm/mach-elite/irq.c
 *
 *	This program is free software: you can redistribute it and/or modify it under the
 *	terms of the GNU General Public License as published by the Free Software Foundation,
 *	either version 2 of the License, or (at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful, but WITHOUT
 *	ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *	PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *	You should have received a copy of the GNU General Public License along with
 *	this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/ptrace.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <mach/irqs.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <asm/hardware/gic.h>

#ifdef CONFIG_OF
static const struct of_device_id elite1000_dt_irq_match[] = {
	{ .compatible = "arm,cortex-a9-gic", .data = gic_of_init, },
	{},
};
#endif

void __init elite_init_irq(void)
{
	if (!of_have_populated_dt())
		gic_init(0, 27, (void __iomem*)IO_ADDRESS(ELITE_ARM_PERIF_GIC_DIST_BASE), 
					(void __iomem*)IO_ADDRESS(ELITE_ARM_PERIF_GIC_CPU_BASE));
#ifdef CONFIG_OF
	else
		of_irq_init(elite1000_dt_irq_match);
#endif
}


