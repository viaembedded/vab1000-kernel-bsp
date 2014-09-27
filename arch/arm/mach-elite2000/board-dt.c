/*
 * linux/arch/arm/mach-elite/board-nand-dt.c

 * elite generic architecture level codes

 * This program is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <asm/mach-types.h>
#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>
#include <asm/irq.h>
#include <asm/sizes.h>
#include <asm/io.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <asm/hardware/gic.h>
#include <asm/pgalloc.h>

#include <mach/iomap.h>

#include "board.h"
#include "cpu-elite.h"
#include "elite-pg.h"

unsigned long elite_pgd_phys;
static pgd_t *elite_pgd;
extern void elite_pmu_enable(void);
extern void elite_init_wakeupcfg(void);
extern void elite_pmu_reset(char mode, const char *cmd);
extern void elite_pmu_power_off(char mode, const char *cmd);
void *elite_context_area = NULL;
extern unsigned int system_rev;


static int elite_create_pg_pgtable(void)
{
#ifdef CONFIG_MMU
	int i;
	pmd_t *pmd;

	unsigned long addr_v[] = {
		(unsigned long)elite_context_area,
		(unsigned long)virt_to_phys(elite_pg_startup),
		(unsigned long)__cortex_a9_restore,
	};
	unsigned long addr_p[] = {
		(unsigned long)virt_to_phys(elite_context_area),
		(unsigned long)virt_to_phys(elite_pg_startup),
		(unsigned long)virt_to_phys(__cortex_a9_restore),
	};
	unsigned int flags = PMD_TYPE_SECT | PMD_SECT_AP_WRITE |
		PMD_SECT_WBWA | PMD_SECT_S;

	elite_pgd = pgd_alloc(&init_mm);
	if (!elite_pgd)
		return -ENOMEM;

	for (i=0; i<ARRAY_SIZE(addr_p); i++) {
		unsigned long v = addr_v[i];
		pmd = pmd_offset(elite_pgd + pgd_index(v), v);
		*pmd = __pmd((addr_p[i] & PGDIR_MASK) | flags);
		flush_pmd_entry(pmd);
		outer_clean_range(__pa(pmd), __pa(pmd + 1));
	}

	elite_pgd_phys = virt_to_phys(elite_pgd);
	__cpuc_flush_dcache_area(&elite_pgd_phys,
		sizeof(elite_pgd_phys));
	outer_clean_range(__pa(&elite_pgd_phys),
		__pa(&elite_pgd_phys+1));

	__cpuc_flush_dcache_area(&elite_context_area,
		sizeof(elite_context_area));
	outer_clean_range(__pa(&elite_context_area),
		__pa(&elite_context_area+1));
#endif 
	return 0;
}

/*
 * The following lookup table is used to override device names when devices
 * are registered from device tree.
 *
 * For drivers that require platform data to be provided from the machine
 * file, a platform data pointer can also be supplied along with the
 * devices names. Usually, the platform data elements that cannot be parsed
 * from the device tree by the drivers (example: function pointers) are
 * supplied. But it should be noted that this is a temporary mechanism and
 * at some point, the drivers should be capable of parsing all the platform
 * data from the device tree.
 */
static const struct of_dev_auxdata elite2000_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("s3graphics,elite2000-uart", ELITE_UART1_BASE,
				"elite-uart.0", NULL),

	OF_DEV_AUXDATA("s3graphics,elite2000-uart", ELITE_UART2_BASE,
				"elite-uart.1", NULL),

	OF_DEV_AUXDATA("s3graphics,elite2000-xtensa", ELITE_XTENSA_CTRL_BASE,
                                "elite-xtensa", NULL),

	OF_DEV_AUXDATA("s3graphics,elite2000-sdhci", 0xd800a800,
				"sdhci-elite.0", NULL),
		
	OF_DEV_AUXDATA("s3graphics,elite2000-sdhci", 0xd800a000,
				"sdhci-elite.1", NULL),

	{},
};

static void __init elite2000_dt_machine_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table,
				elite2000_auxdata_lookup, NULL);

	elite_context_area = kzalloc(CONTEXT_SIZE_BYTES * MAXCPUS, GFP_KERNEL);

	elite_create_pg_pgtable();

	elite_pmu_enable();
}

static const char *elite2000_dt_compat[] __initdata = {
	"s3graphics,elite2000",
	NULL
};

//DT_MACHINE_START(ELITE1000_DT, "S3GRAPHICS ELITE1000 (Flattened Device Tree)")
DT_MACHINE_START(ELITE1000_DT, "elite2000")
	.init_irq	= elite_init_irq,
	.map_io		= elite_map_io,
	.init_early	= elite_init_early,
	.handle_irq	= gic_handle_irq,
	.init_machine	= elite2000_dt_machine_init,
	.timer		= &elite_timer,
	.dt_compat	= elite2000_dt_compat,
#ifdef CONFIG_PM
	.restart    = elite_pmu_reset,
#endif
MACHINE_END

