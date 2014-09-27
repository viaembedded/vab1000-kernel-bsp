#
# Makefile for the linux kernel.
#

# CPU-specific support

obj-y += irq.o
obj-y += timer.o
obj-y += dma.o
obj-y += common.o
obj-y += io.o
obj-y += cortex-a9.o headsmp.o
obj-y += clock.o
obj-y += scm.o
obj-$(CONFIG_MTD_NAND_ELITE) += kernel_mtd.o
obj-$(CONFIG_PCI) += pcie.o
obj-$(CONFIG_SMP) += platsmp.o
obj-$(CONFIG_PM) += suspend.o
# cpu power management
obj-$(CONFIG_CPU_FREQ) += cpufreq.o
obj-$(CONFIG_CPU_FREQ) += dvfs_elite.o
#CFLAGS_dvfs_elite.o :=-O0
obj-$(CONFIG_CPU_IDLE) += cpuidle_elite.o
obj-y += cpu-elite.o

obj-$(CONFIG_ARCH_ELITE_1000) += elite_clocks.o
obj-$(CONFIG_ARCH_ELITE_1000) += elite-wakeupcfg.o

# ELITE 1000 board-specific support
obj-$(CONFIG_MACH_NAND)	+= board-nand.o
obj-$(CONFIG_MACH_EMMC)	+= board-emmc.o
obj-$(CONFIG_MACH_NAND_DT) += board-nand-dt.o
#;ysw_debug - start
obj-$(CONFIG_MACH_EMMC_DT) += board-emmc-dt.o
#;ysw_debug - end
