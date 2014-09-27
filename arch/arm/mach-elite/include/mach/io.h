/*
 *  arch/arm/mach-elite/include/mach/io.h
 *
 *  Copyright (C) 2003 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __MACH_ELITE_IO_H
#define __MACH_ELITE_IO_H


#define IO_ELITE_PHYS	0xd8000000
#define IO_ELITE_VIRT	0xFE000000
#define IO_ELITE_SIZE	SZ_4M

#define IO_L2CACHE_PHYS	0xd9000000
#define IO_L2CACHE_VIRT	(IO_ELITE_VIRT + IO_ELITE_SIZE)
#define IO_L2CACHE_SIZE SZ_4K

#define IO_PCIE_PHYS	0xcfff0000
#define IO_PCIE_VIRT	(IO_L2CACHE_VIRT + IO_L2CACHE_SIZE)
#define IO_PCIE_SIZE SZ_64K

#define IO_TO_VIRT_BETWEEN(p, st, sz)   ((p) >= (st) && (p) < ((st) + (sz)))
#define IO_TO_VIRT_XLATE(p, pst, vst)   (((p) - (pst) + (vst)))



#define IO_ADDRESS(x) (\
	IO_TO_VIRT_BETWEEN((x), IO_ELITE_PHYS, IO_ELITE_SIZE)? \
		IO_TO_VIRT_XLATE((x), IO_ELITE_PHYS, IO_ELITE_VIRT): \
	IO_TO_VIRT_BETWEEN((x), IO_L2CACHE_PHYS, IO_L2CACHE_SIZE)? \
		IO_TO_VIRT_XLATE((x), IO_L2CACHE_PHYS, IO_L2CACHE_VIRT): \
	IO_TO_VIRT_BETWEEN((x), IO_PCIE_PHYS, IO_PCIE_SIZE)? \
		IO_TO_VIRT_XLATE((x), IO_PCIE_PHYS, IO_PCIE_VIRT): \
	0)


#define __io(a)   IO_ADDRESS(a + 0xcfff0000)

#ifndef __ASSEMBLER__

void __iomem *elite_ioremap_caller(unsigned long addr, size_t size,
					   unsigned int mtype, void *caller);
void elite_iounmap(volatile void __iomem *addr);
#endif

#endif

