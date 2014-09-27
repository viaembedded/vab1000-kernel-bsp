/*
 * linux/drivers/input/keyboard/elite_kpad.h
 * 
 * Copyright (C) 2012 S3 Graphics, Inc.
 *
 * This program is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 2 of the License, or 
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#ifndef __ASM_ARCH_KPAD_H
#define __ASM_ARCH_KPAD_H

#define KPAD_MATRIX_CONTROL 	0x0
#define KPAD_DIRECT_CONTROL 	0x4
#define KPAD_STATUS		0xc

#define INVERT_INPUT_CONTROL	0x8
#define KEYPAD_STATUS		0xc
#define MATRIX_PRI_AUTO_SCAN	0x10
#define DIRECT_INPUT_SCAN	0x14
#define MANUAL_MATRIX_SCAN	0x18
#define ROW_INPUT		0x1c
#define MM_SCAN0		0x20
#define MM_SCAN1		0x24
#define MM_SCAN2		0x28
#define MM_SCAN3		0x2c
#define KPAD_MATRIX_INTERVAL	0x30
#define KPAD_DIRECT_INTERVAL	0x34

/*matrix input control*/
#define KPC_ME              (0x1 << 0)  /* Matrix Keypad Enable */
#define KPC_MIE             (0x1 << 1)  /* Matrix Interrupt Enable */
#define KPC_MAS             (0x1 << 2)  /* Automatic Scan bit */
#define KPC_MASACT          (0x1 << 3)  /* Automatic Scan on Activity */
#define KPC_MIMKP           (0x1 << 4)  /* Ignore Multiple Key Press */

/*direct input control*/
#define KPC_DE              (0x1 <<  1)  /* Direct Keypad Enable */
#define KPC_DIE             (0x1 <<  0)  /* Direct Keypad interrupt Enable */
#define KPC_DASACT          (0x1 << 3)  /* Automatic Scan on Activity */

/*interrupt status*/
#define KPC_SDI             (0x1 << 3)  /* Auto scan Direct key interrupt bit */
#define KPC_SMI             (0x1 << 1)  /* Auto scan Matrix interrupt bit */

/*direct input*/
#define KPC_DI_VALID        (0x1<<31)

/*matrix primary input*/
#define KPC_MI_VALID        (0x1<<31)
#define KPC_MI_MUKP(n)	    (((n) >> 29) & 0x3)
#define KPC_MI_RP(n)	    (((n) >> 4) & 0x7)
#define KPC_MI_CP(n)	    ((n) & 0x7)

/*matrix scan input*/
#define KPC_MI_SCAN_VALID   (0x1<<31)

#define MAX_ROW 	        4
#define MAX_COLUMN 	        4
#define MATRIX_ROW_SHIFT	2
#define MAX_DIRECT_KEY_NUM	4

#define MAX_MATRIX_KEY_NUM	(MAX_ROW * MAX_COLUMN)
#define MAX_KPAD_KEYS		(MAX_MATRIX_KEY_NUM + MAX_DIRECT_KEY_NUM)


struct elite_kpad 
{	
	struct device * dev;
	
	struct clk *clk;
	struct input_dev *input_dev;

	void __iomem *mmio_base;
	int irq;

	unsigned short keycodes[MAX_KPAD_KEYS];

	/* state row bits of each column scan */
	uint32_t matrix_key_state[MAX_COLUMN];
	uint32_t direct_key_state;

	unsigned int direct_key_mask;
};

#endif /* __ASM_ARCH_KPAD_H */
