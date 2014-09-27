/*
 *  arch/arm/mach-elite/include/mach/irqs.h
 *
 *  Copyright (C) 2003 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd.
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

#ifndef __MACH_ELITE_IRQS_H
#define __MACH_ELITE_IRQS_H


/* 
 * Elite 800 irq no, This part should be updated by relative driver coder.
 */
#ifdef CONFIG_ARCH_ELITE_1000

#define IRQ_PRI_BASE 32
#define IRQ_LOCALTIMER		29
#define IRQ_LOCALWDOG		30

#define IRQ_SE			(0 + IRQ_PRI_BASE)
#define IRQ_SDC1		(1 + IRQ_PRI_BASE)
#define IRQ_SDC1_DMA		(2 + IRQ_PRI_BASE)
#define IRQ_SATA		(3 + IRQ_PRI_BASE)
#define IRQ_GFX_INTA		(4 + IRQ_PRI_BASE)	
#define IRQ_GFX_INTB		(5 + IRQ_PRI_BASE)
#define IRQ_HAM_NEON_DVFS_INT2	(6 + IRQ_PRI_BASE)
#define IRQ_HAM_NEON_DVFS_INT3	(7 + IRQ_PRI_BASE)
#define IRQ_KPAD		(8 + IRQ_PRI_BASE)
#define IRQ_MSGC_PXE0		(9 + IRQ_PRI_BASE)
#define IRQ_SATAPME		(10 + IRQ_PRI_BASE)
#define IRQ_MSGC_PXE1		(11 + IRQ_PRI_BASE)
#define IRQ_TZPC_NS		(12 + IRQ_PRI_BASE)
#define IRQ_DMA_NONS		(15 + IRQ_PRI_BASE)
#define IRQ_APBB		(16 + IRQ_PRI_BASE)
#define IRQ_DMA_SECURE		(17 + IRQ_PRI_BASE)
#define IRQ_I2C1		(18 + IRQ_PRI_BASE)
#define IRQ_I2C0		(19 + IRQ_PRI_BASE)
#define IRQ_SDC0		(20 + IRQ_PRI_BASE)
#define IRQ_SDC0_DMA		(21 + IRQ_PRI_BASE)
#define IRQ_PMC_WAKEUP		(22 + IRQ_PRI_BASE)
#define IRQ_DSP2ARM		(23 + IRQ_PRI_BASE)
#define IRQ_SPI0		(24 + IRQ_PRI_BASE)
#define IRQ_SPI1		(25 + IRQ_PRI_BASE)
#define IRQ_UHDC		(26 + IRQ_PRI_BASE)
#define IRQ_MIU_A9_TS		(27 + IRQ_PRI_BASE)
#define IRQ_NFC 		(28 + IRQ_PRI_BASE)
#define IRQ_NFC_DMA		(29 + IRQ_PRI_BASE)
#define IRQ_EINTG		(31 + IRQ_PRI_BASE)
#define IRQ_UART0		(32 + IRQ_PRI_BASE)
#define IRQ_UART1		(33 + IRQ_PRI_BASE)
#define IRQ_PMU_CPU		(34 + IRQ_PRI_BASE)
#define IRQ_MB_CPU		(35 + IRQ_PRI_BASE)
#define IRQ_PMC_TIMER0		(36 + IRQ_PRI_BASE)
#define IRQ_PMC_TIMER1		(37 + IRQ_PRI_BASE)
#define IRQ_PMC_TIMER2		(38 + IRQ_PRI_BASE)
#define IRQ_PMC_TIMER3		(39 + IRQ_PRI_BASE)
#define IRQ_HAM_NEON_DVFS_INT0	(40 + IRQ_PRI_BASE)
#define IRQ_HPM_CPU		(41 + IRQ_PRI_BASE)
#define IRQ_HAM_NEON_DVFS_INT1	(46 + IRQ_PRI_BASE)
#define IRQ_UART2		(47 + IRQ_PRI_BASE)
#define IRQ_RTC0		(48 + IRQ_PRI_BASE)
#define IRQ_RTC1		(49 + IRQ_PRI_BASE)
#define IRQ_CIR 		(55 + IRQ_PRI_BASE)
#define IRQ_SDC2		(56 + IRQ_PRI_BASE)
#define IRQ_SDC2_DMA		(57 + IRQ_PRI_BASE)
#define IRQ_RNG_CPU		(64 + IRQ_PRI_BASE)
#define IRQ_TZMA_SW		(65 + IRQ_PRI_BASE)
#define IRQ_TZMA_SR		(66 + IRQ_PRI_BASE)
#define IRQ_SMART_CARD0 	(67 + IRQ_PRI_BASE)
#define IRQ_SMART_CARD1 	(68 + IRQ_PRI_BASE)
#define IRQ_NS_TIMER0		(69 + IRQ_PRI_BASE)
#define IRQ_NS_TIMER1		(70 + IRQ_PRI_BASE)
#define IRQ_SE_TIMER		(73 + IRQ_PRI_BASE)
#define IRQ_SE_WD		(74 + IRQ_PRI_BASE)
#define IRQ_AUDPRF0		(75 + IRQ_PRI_BASE)
#define IRQ_AUDPRF1		(76 + IRQ_PRI_BASE)
#define IRQ_AUDPRF2		(77 + IRQ_PRI_BASE)
#define IRQ_AUDPRF3		(78 + IRQ_PRI_BASE)
#define IRQ_AUDPRF4		(79 + IRQ_PRI_BASE)
#define IRQ_AUDPRF5		(80 + IRQ_PRI_BASE)
#define IRQ_AUDPRF6		(81 + IRQ_PRI_BASE)
#define IRQ_AUDPRF7		(82 + IRQ_PRI_BASE)
#define IRQ_AUDPRF8		(83 + IRQ_PRI_BASE)
#define IRQ_GPIO		(84 + IRQ_PRI_BASE)
#define IRQ_DMX0		(89 + IRQ_PIR_BASE)
#define IRQ_DMX1		(90 + IRQ_PRI_BASE)
#define IRQ_DMX2		(91 + IRQ_PIR_BASE)
#define IRQ_DMX3		(92 + IRQ_PRI_BASE)

#define IRQ_END			(IRQ_DMX3 + 1)


#define IRQ_DMA_CH_0		IRQ_DMA_NONS
#define IRQ_DMA_CH_1		IRQ_DMA_NONS
#define IRQ_DMA_CH_2		IRQ_DMA_NONS
#define IRQ_DMA_CH_3		IRQ_DMA_NONS
#define IRQ_DMA_CH_4		IRQ_DMA_NONS
#define IRQ_DMA_CH_5		IRQ_DMA_NONS
#define IRQ_DMA_CH_6		IRQ_DMA_NONS
#define IRQ_DMA_CH_7		IRQ_DMA_NONS
#define IRQ_DMA_CH_8		IRQ_DMA_NONS
#define IRQ_DMA_CH_9		IRQ_DMA_NONS
#define IRQ_DMA_CH_10		IRQ_DMA_NONS
#define IRQ_DMA_CH_11		IRQ_DMA_NONS
#define IRQ_DMA_CH_12		IRQ_DMA_NONS
#define IRQ_DMA_CH_13		IRQ_DMA_NONS
#define IRQ_DMA_CH_14		IRQ_DMA_NONS
#define IRQ_DMA_CH_15		IRQ_DMA_NONS

#define NR_IRQS 137
#endif


#endif
