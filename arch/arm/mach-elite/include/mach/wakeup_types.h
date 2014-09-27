/*
 * wakeup type definitions for ELITE1K
 *
 * Copyright (C) 2013 S3 Graphics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ELITE1K_WAKEUP_TYPE_H
#define __ELITE1K_WAKEUP_TYPE_H
enum WAKEUP_TYPE {
    WAKEUP_GPIO0 = 1, 
    WAKEUP_GPIO1,
    WAKEUP_GPIO2,
    WAKEUP_GPIO3,
    WAKEUP_INTERNAL_IP0,
    WAKEUP_INTERNAL_IP1,
    WAKEUP_INTERNAL_IP2,
    WAKEUP_INTERNAL_IP3,
    WAKEUP_POWER_BTN,
    WAKEUP_RTC,
    WAKEUP_USB_HOST,
    WAKEUP_USB_DEV,
    WAKEUP_CIR,
    WAKEUP_SD0_CARD_DETECT,
    WAKEUP_SD3_CARD_DETECT,
};
#endif
