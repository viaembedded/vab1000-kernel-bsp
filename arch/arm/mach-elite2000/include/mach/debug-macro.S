/* arch/arm/mach-elite/include/mach/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
	@Current kernel I/O mmap design is phys = virt

	.macro	addruart,rp,rv,tmp
	ldr		\rp, =0xd8200000   @ UART1_BASE = 0xd8200000
	ldr		\rv, =0xfe200000   
	.endm

	.macro	senduart,rd,rx
	ldr	r2,  [\rx, #0x20]           @ FIFO control register
	tst	r2,  #0x01                  @ Check FIFOEN bit
	addne   \rx, \rx, #0x1000
	streq	\rd, [\rx]              @ Transmit data register
	.endm

	.macro	waituart,rd,rx
1001:
	ldr	\rd, [\rx, #0x1C]       @ Status register
	tst	\rd, #2                 @ Transmit busy
	bne	1001b                   @ If busy then wait
	.endm

	.macro	busyuart,rd,rx
1001:
	ldr	\rd, [\rx, #0x1C]       @ Status register
	tst	\rd, #2                 @ Transmit busy
	bne	1001b
	.endm

