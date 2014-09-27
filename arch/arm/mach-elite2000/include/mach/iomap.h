/*
 * arch/arm/mach-elite/include/mach/iomap.h
 *
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

#ifndef __MACH_ELITE_IOMAP_H
#define __MACH_ELITE_IOMAP_H

#include <asm/sizes.h>

/*
 * Cortex-A9 ASIC peripherals addresses
 */
#define ELITE_ARM_PERIF_SCU_BASE        0xD8018000	/* SCU registers */
#define ELITE_ARM_PERIF_GIC_CPU_BASE    0xD900A000	/* Private GIC CPU interface */
#define ELITE_ARM_PERIF_GIC_DIST_BASE   0xD9009000	/* Private GIC distributor */

#define ELITE_ARM_PERIF_GLT_BASE        0xD8018200	/* Global timer */
#define ELITE_ARM_PERIF_TWD_BASE        0xD8018600
#define ELITE_ARM_PERIF_TWD_SIZE        0x00000100
#define ELITE_ARM_PERIF_L310_BASE       0xD9000000	/* L310 registers */

/*
 * Peripheral addresses
 */
#define ELITE_MEMORY_CTRL_CFG_BASE               0xD8000000	/* 1K , 8/16/32 RW */
#define ELITE_DMA_CTRL_CFG_BASE                  0xD8001800	/* 1K , 8/16/32 RW */
#define ELITE_SF_MEM_CTRL_CFG_BASE               0xD8002000	/* 1K , 8/16/32 RW */
#define ELITE_SPI_MEM_CTRL_CFG_BASE              0xD8003000	/* 1K , 8/16/32 RW */
#define ELITE_ETHERNET_MAC_0_CFG_BASE            0xD8004000	/* 1K , 8/16/32 RW */
#define ELITE_ETHERNET_MAC_1_CFG_BASE            0xD8005000	/* 1K , 8/16/32 RW */
#define ELITE_SECURITY_ENGINE_CFG_BASE           0xD8006000	/* 1K , 8/16/32 RW */
#define ELITE_USB20_HOST_CFG_BASE                0xD8007000	/* 1K , 8/16/32 RW */
#define ELITE_PATA_CTRL_CFG_BASE                 0xD8008000	/* 1K , 8/16/32 RW */
#define ELITE_NF_CTRL_CFG_BASE                   0xD8009000	/* 1K , 8/16/32 RW */
#define ELITE_NORF_CTRL_CFG_BASE                 0xD8009400	/* 1K , 8/16/32 RW */
#define ELITE_SDMMC0_BASE                        0xD800A000	/* 1K , 8/16/32 RW */
#define ELITE_SDMMC1_BASE                        0xD800A400	/* 1K , 8/16/32 RW */
#define ELITE_SDMMC2_BASE                        0xD800A800	/* 1K , 8/16/32 RW */
#define ELITE_MS_CTRL_CFG_BASE                   0xD800B000	/* 1K , 8/16/32 RW */
#define ELITE_CF_CTRL_CFG_BASE                   0xD800C000	/* 1K , 8/16/32 RW */
#define ELITE_SATA_CTRL_CFG_BASE                 0xD800D000	/* 1K , 8/16/32 RW */
#define ELITE_XOR_CTRL_CFG_BASE                  0xD800E000	/* 1K , 8/16/32 RW */
#define ELITE_LPC_CTRL_CFG_BASE                  0xD8040000	/* 1K , 8/16/32 RW */

#define ELITE_GFX_MMIO_BASE                      0xD80A0000
#define ELITE_XTENSA_CTRL_BASE                   0xD80EFC00
#define ELITE_RTC_BASE                           0xD8100000	/* 64K  */
#define ELITE_GPIO_BASE                          0xD8110000	/* 64K  */
#define ELITE_SYSTEM_CFG_CTRL_BASE               0xD8120000	/* 64K  */
#define ELITE_PM_CTRL_BASE                       0xD8130000	/* 64K  */
#define ELITE_INTERRUPT_CTRL_BASE                0xD8140000	/* 64K  */
#define ELITE_INTERRUPT1_CTRL_BASE               0xD8150000	/* 64K  */
#define ELITE_UART1_BASE                         0xD8200000	/* 64K  */
#define ELITE_UART2_BASE                         0xD82b0000	/* 64K  */
#define ELITE_UART3_BASE                         0xD8210000	/* 64K  */
#define ELITE_UART4_BASE                         0xD82c0000	/* 64K  */
#define ELITE_UART5_BASE                         0xD8370000	/* 64K  */
#define ELITE_UART6_BASE                         0xD8380000	/* 64K  */
#define ELITE_SPI_BASE                           0xD8240000	/* 64K  */
#define ELITE_KPAD_BASE                          0xD8260000	/* 64K  */
#define ELITE_I2C0_BASE                          0xD8280000     /* 64K  */
#define ELITE_CIR_BASE                           0xD8270000
#define ELITE_I2C1_BASE                          0xD8320000
#define ELITE_I2S_BASE                           0xD8330000
#define ELITE_PCM_BASE                           0xD82D0000

#define PMP_WDT_BASE  							 0xD839E040
#define ELITE_PMU_BASE				 			 0xD8390000


#define ELITE_SECUREBOOT_REG			0xD8393418
#endif
