/*
 * MMC definitions for ELITE1K
 *
 * Copyright (C) 2013 S3 Graphics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ELITE1K_MMC_H
#define __ELITE1K_MMC_H

struct elite_mmc_platform_data {
	struct device *dev;
	unsigned int max_freq;
	u32 caps;
	u32 pm_caps;
	int pwrsw_pin;
	bool nonremovable;
};

#endif

