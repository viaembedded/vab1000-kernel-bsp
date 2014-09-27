/*
 * pinctrl configuration definitions for the S3 Graphics Elite pinctrl
 *
 * Copyright (c) 2012, S3 Graphics.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef __PINCONF_ELITE_H__
#define __PINCONF_ELITE_H__

enum elite_pinconf_param {
	ELITE_PINCONF_PARAM_FUNC_EN,
	/* arg: elite_pinconf_pull */
	ELITE_PINCONF_PARAM_PULL,
	ELITE_PINCONF_PARAM_DRV,
	ELITE_PINCONF_PARAM_VOLT,
	ELITE_PINCONF_PARAM_USB,
};

enum elite_pinconf_pull {
	ELITE_PINCONF_PULL_NONE = -1,
	ELITE_PINCONF_PULL_DOWN = 0,
	ELITE_PINCONF_PULL_UP = 1,
};

enum elite_pinconf_drvstrength {
	ELITE_PINCONF_PAD_DRV_TYPE_B = 0,
	ELITE_PINCONF_PAD_DRV_TYPE_A,
	ELITE_PINCONF_PAD_DRV_TYPE_C,
	ELITE_PINCONF_PAD_DRV_TYPE_D,
};

enum elite_pinconf_voltage {
	ELITE_PINCONF_PAD_1_8 = 0, //1.8V
	ELITE_PINCONF_PAD_3_3, //3.3v
};

enum elite_pinconf_usb_mode {
	ELITE_PINCONF_DEVICE = 0,
	ELITE_PINCONF_HOST = 1,
	ELITE_PINCONF_IDPIN = 2,
};


#define ELITE_PINCONF_PACK(_param_, _arg_) ((_param_) << 16 | (_arg_))
#define ELITE_PINCONF_UNPACK_PARAM(_conf_) ((_conf_) >> 16)
#define ELITE_PINCONF_UNPACK_ARG(_conf_) ((_conf_) & 0xffff)

/*
 * Extra pin control state definitions for SD
 */
#define PINCTRL_STATE_SD_PULLDOWN "sd_pulldown"


#endif //__PINCONF_ELITE_H__
