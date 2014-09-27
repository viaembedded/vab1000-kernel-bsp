/*
 * linux/arch/arm/mach-elite/include/mach/pin-mux.h
 *
 * Copyright (C) 2012 S3 Graphics Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MACH_ELITE_PINMUX_H
#define __MACH_ELITE_PINMUX_H


enum pmx_reg_op {
	BITWISE_NONE = 0,
	BITWISE_OR = 1,
	BITWISE_AND = 2,
	BITWISE_COMP = 3
};

struct elite_group {
	const char *name;
	const unsigned int *pins;
	const unsigned num_pins;
};

struct elite_pmx_func {
	const char *name;
	const char * const *groups;
	const unsigned num_groups;
	enum pmx_reg_op regop;
	unsigned int val;
};

struct elite_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctrl;

	void __iomem *virt_base;
};

#endif //__MACH_ELITE_PINMUX_H
