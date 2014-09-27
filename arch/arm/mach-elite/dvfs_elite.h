/*
 * arch/arm/mach-elite/elite_clocks.c
 *
 * Copyright (C) 2012 S3 Graphics, Inc.
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

#ifndef _ELITE_DVFS_H_
#define _ELITE_DVFS_H_


struct dvfs_rail {
	const char *reg_id;
	bool disabled;
	bool updating;

	struct regulator *reg;
        int max_millivolts;
        int min_millivolts;
	int current_millivolts;
	int updating_millivolts;
};


#endif
