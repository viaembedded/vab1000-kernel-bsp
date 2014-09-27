#!/bin/sh
export ARCH=arm
export CROSS_COMPILE=arm-none-linux-gnueabi-
make mrproper
make elite_dtv_defconfig
make uImage -j4 KCFLAGS="-mno-unaligned-access -mcpu=cortex-a9"

