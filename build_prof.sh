#!/bin/sh
export ARCH=arm
export CROSS_COMPILE=arm-none-linux-gnueabi-
make mrproper
make elite_prof_defconfig
make uImage -j4 KCFLAGS="-mcpu=cortex-a9"

