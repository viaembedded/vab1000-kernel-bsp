/*
 *  arch/arm/mach-elite/include/mach/uncompress.h
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

#ifndef __MACH_ELITE_UNCOMPRESS_H
#define __MACH_ELITE_UNCOMPRESS_H

#include <mach/iomap.h>
#include <linux/types.h>

#define UART(x)   	(*(volatile unsigned long *)(x + ELITE_UART2_BASE))
#define URUSR   	0x001c
#define URFCR		0x0020
#define URTDR		0x0000

#define URUSR_TXDBSY	0x2
#define URFCR_FIFOEN    0x1
/*
 * This does not append a newline
 */
static inline void putc(int c)
{
	while (UART(URUSR) & URUSR_TXDBSY);

	UART(URFCR) &= ~URFCR_FIFOEN;

	UART(URTDR) = c;
}

static inline void flush(void)
{
}

static inline void arch_decomp_setup(void)
{
}

#define arch_decomp_wdog()


#endif
