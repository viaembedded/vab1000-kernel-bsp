/*
 * EHCI definitions for ELITE1K
 *
 * Copyright (C) 2013 S3 Graphics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MACH_ELITE1K_EHCI_H
#define __MACH_ELITE1K_EHCI_H

struct elite_usb_platform_data {
    struct device *dev;
    int pw0_gpio_pin;
    int pwother_gpio_pin;
    int success_pin;
};


#endif //__MACH_ELITE1K_EHCI_H
