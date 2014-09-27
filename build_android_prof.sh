#!/bin/sh
export ARCH=arm
export CROSS_COMPILE=arm-none-linux-gnueabi-
export CONFIG_PATH=arch/arm/configs
make mrproper
cat $CONFIG_PATH/elite_android_dt_defconfig > $CONFIG_PATH/elite_android_prof_defconfig
echo "CONFIG_PROFILING=y" >> $CONFIG_PATH/elite_android_prof_defconfig
echo "CONFIG_LOCAL_TIMERS=y" >> $CONFIG_PATH/elite_android_prof_defconfig
echo "CONFIG_PERF_EVENTS=y" >> $CONFIG_PATH/elite_android_prof_defconfig
echo "CONFIG_HW_PERF_EVENTS=y" >> $CONFIG_PATH/elite_android_prof_defconfig
echo "CONFIG_ENABLE_DEFAULT_TRACERS=y" >> $CONFIG_PATH/elite_android_prof_defconfig
make elite_android_prof_defconfig
make uImage -j4 KCFLAGS="-mcpu=cortex-a9"
make elite1000-nand.dtb

