#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <mach/io.h>
#include <asm/mach/map.h>
#include <asm/page.h>
#include <mach/iomap.h>

static struct map_desc elite_io_desc[] __initdata = {
	{
		.virtual	= IO_ELITE_VIRT,
		.pfn		= __phys_to_pfn(IO_ELITE_PHYS),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	},
        {
                .virtual        = IO_L2CACHE_VIRT,
                .pfn            = __phys_to_pfn(IO_L2CACHE_PHYS),
		.length		= SZ_4K,
	},

	{
                .virtual        = IO_GIC400_VIRT,
                .pfn            = __phys_to_pfn(IO_GIC400_PHYS),
		.length		= SZ_16K,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= IO_PCIE_VIRT,
		.pfn		= __phys_to_pfn(IO_PCIE_PHYS),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	},
};

void __init elite_map_io(void)
{
	iotable_init(elite_io_desc, ARRAY_SIZE(elite_io_desc));
}

void __iomem *elite_ioremap_caller(unsigned long phys_addr, size_t size,
	unsigned int mtype, void *caller)
{
	void __iomem *v = (void __iomem*)IO_ADDRESS(phys_addr);

	if (v != NULL)
		return v;

	return __arm_ioremap_caller(phys_addr, size, mtype, caller);
}


EXPORT_SYMBOL(elite_ioremap_caller);

void elite_iounmap(volatile void __iomem *addr)
{
	unsigned long virt = (unsigned long)addr;
	
	if(virt >= VMALLOC_START && virt < VMALLOC_END)
		__iounmap(addr);
}

EXPORT_SYMBOL(elite_iounmap);
