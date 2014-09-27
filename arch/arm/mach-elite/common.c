#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/io.h>

#include <asm/hardware/cache-l2x0.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/iomap.h>
#include <mach/io.h>
#include "board.h"
#include "clock.h"
#include "scm.h"

extern unsigned int system_rev;

#define L2X0_PREFETCH_CTRL_DOUBLE_LINEFILL 0x40000000
#define L2X0_PREFETCH_CTRL_DROP            0x01000000

static void __init elite_init_cache(void)
{
#ifdef CONFIG_CACHE_L2X0
	void __iomem *l2x0_base = (void __iomem *)IO_ADDRESS(ELITE_ARM_PERIF_L310_BASE);
        unsigned int prefetch_ctrl;
	
	if (svc_system_is_secureboot()) {
		svc_system_enable_l2cache();
	} else {
		writel(0x111, l2x0_base + L2X0_TAG_LATENCY_CTRL);
		writel(0x111, l2x0_base + L2X0_DATA_LATENCY_CTRL);
		prefetch_ctrl = readl(l2x0_base + L2X0_PREFETCH_CTRL);
		prefetch_ctrl |= L2X0_PREFETCH_CTRL_DOUBLE_LINEFILL;
		prefetch_ctrl |= L2X0_PREFETCH_CTRL_DROP;
		writel(prefetch_ctrl, l2x0_base + L2X0_PREFETCH_CTRL);
	}

	l2x0_init(l2x0_base, 0x72050001, 0x80000ffe);
#endif
}

void __init elite_init_early(void)
{
	unsigned int chip_rev;
	/* this register is programmed in bootloader (bl2)*/
	system_rev = readl(IO_ADDRESS(0xd839341c));
	chip_rev = readl(IO_ADDRESS(0xd80a8500)) & 0xf;
	if (chip_rev == 2)
	    system_rev = (system_rev & (~0xf)) | chip_rev;

	elite_init_cache();

	arch_ioremap_caller = elite_ioremap_caller;
	arch_iounmap = elite_iounmap;

	elite_soc_init_clocks();
}
