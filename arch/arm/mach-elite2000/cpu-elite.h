/*
 * arch/arm/mach-elite/cpu-elite.h
 *
 * Declarations for cpu power management code
 *
 * Copyright (c) 2011, S3 GRAPHICS INC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

//only support 2 or 4 cpus:
#define MAXCPUS 2

#ifndef __MACH_ELITE_CPU_ELITE_H
#define __MACH_ELITE_CPU_ELITE_H


#define ELITE_PMC_BASE                  0xD8390000
//#define ELITE_PMC_BASE                  0xD80F0000
//#define ELITE_PMC_BASE                  0xD8130000
#define ELITE_PMU_BASE                  (void __iomem *)IO_ADDRESS(ELITE_PMC_BASE)
#define GIC_DIST_BASE                   (void __iomem *)IO_ADDRESS(0xd8019000)
#define GIC_CPUINTERFACE_BASE           (void __iomem *)IO_ADDRESS(0xd8018100)

#define ELITE_PCIE0_PHY_BASE            0xD80B0000
#define ELITE_PCIE1_PHY_BASE            0xD80C0000
#define ELITE_PCIE0_BASE                (void __iomem *)IO_ADDRESS(ELITE_PCIE0_PHY_BASE)
#define ELITE_PCIE1_BASE                (void __iomem *)IO_ADDRESS(ELITE_PCIE1_PHY_BASE)

#define MCBOOT0                         0x3400
#define MCBOOT1                         0x3404
#define MCBOOT2                         0x3408
#define MCBOOT3                         0x340C
#define MCBOOT4                         0x3410
#define MCBOOT5                         0x3414
#define MCBOOT6                         0x3418
#define MCBOOT7                         0x341C

#define MCSBOOT0                         0xE018
#define MCSBOOT1                         0xE01c
#define MCSBOOT2                         0xE020
#define MCSBOOT3                         0xE024
#define MCSBOOT4                         0xE028
#define MCSBOOT5                         0xE02c
#define MCSBOOT6                         0xE030
#define MCSBOOT7                         0xE034


#define DAP_CTRL                        0x6000
#define PMU_CORE0_PTIMER_COUNTER_REG    0xC780
#define PMU_CORE1_PTIMER_COUNTER_REG    0xC784
#define PMU_CORE2_PTIMER_COUNTER_REG    0xC788
#define PMU_CORE3_PTIMER_COUNTER_REG    0xC78C
#define PMU_CORE0_PTIMER_CTRL_REG       0xC790
#define PMU_CORE1_PTIMER_CTRL_REG       0xC794
#define PMU_CORE2_PTIMER_CTRL_REG       0xC798
#define PMU_CORE3_PTIMER_CTRL_REG       0xC79c



#define CORE0_WFI_CFG                   0xC000
#define CORE1_WFI_CFG                   0xC004
#define CORE2_WFI_CFG                   0xC008
#define CORE3_WFI_CFG                   0xC00C
#define CPU_LP_CFG_CORE0                0xC010
#define CPU_LP_CFG_CORE1                0xC014
#define CPU_LP_CFG_CORE2                0xC018
#define CPU_LP_CFG_CORE3                0xC01c
#define SYS_LP_CFG0                     0xC024
#define SYS_LP_CFG1                     0xC028
#define STM_CFG                         0xC02C
#define GENERAL_CFG0_CORE0              0xC034
#define GENERAL_CFG0_CORE1              0xC038
#define GENERAL_CFG0_CORE2              0xC03c
#define GENERAL_CFG0_CORE3              0xC040
#define GENERAL_CFG0_STATUS             0xC060
#define CPUCore_DVFS                    0xC200
#define PDMCONT_CMDFIFO                 0xc1f0
#define DVFS_CPU_PLL_CTRL               0xc340
#define DVFS_CMDFIFO                    0xc3f0
#define AVS_CMDFIFO                     0xc3f4
#define PMU_STATUS                      0xc400
#define PMU_CFG                         0xc408
#define PMU_SW_FULLCHIP_RESET           0xcb0c


#define PMU_CG_CORE0_COUNT              0xc700
#define PMU_CG_CORE0_CYCCOUNT           0xc708
#define PMU_CG_CORE1_COUNT              0xc710
#define PMU_CG_CORE1_CYCCOUNT           0xc718
#define PMU_CG_CORE2_COUNT              0xc720
#define PMU_CG_CORE2_CYCCOUNT           0xc728
#define PMU_CG_CORE3_COUNT              0xc730
#define PMU_CG_CORE3_CYCCOUNT           0xc738
#define PMU_PG_CORE0_COUNT              0xc740
#define PMU_PG_CORE0_CYCCOUNT           0xc748
#define PMU_PG_CORE1_COUNT              0xc750
#define PMU_PG_CORE1_CYCCOUNT           0xc758
#define PMU_PG_CORE2_COUNT              0xc760
#define PMU_PG_CORE2_CYCCOUNT           0xc768
#define PMU_PG_CORE3_COUNT              0xc770
#define PMU_PG_CORE3_CYCCOUNT           0xc778

#define OS_TIMER_MATCH_REG0             0xC680
#define OS_TIMER_MATCH_REG1             0xC684
#define OS_TIMER_MATCH_REG2             0xC688
#define OS_TIMER_MATCH_REG3             0xC68C
#define OS_TIMER_COUNT_REG              0xC690
#define OS_TIMER_STATUS_REG             0xC694
#define OS_TIMER_WATCHDOG_ENABLE_REG    0xC698
#define OS_TIMER_INTRRUPT_ENABLE_REG    0xC69C
#define OS_TIMER_CONTROL_REG            0xC6A0
#define OS_TIMER_ACCESS_STATUS_REG      0xC6A4
#define EINT0_WAKEUP_IRQ_STATUS         0xC6CC

#define DAP_HAM_MUX_SEL                 0xc498
#define HAM_CFG                         0xC900
#define HAM_NEON_PGREF                  0xC944
#define HAM_NEON_CNT_0                  0xC948
#define HAM_NEON_CNT_1                  0xC94C
#define HAM_NEON_CNT_2                  0xC950
#define HAM_NEON_CNT_3                  0xC954
#define HAM_IRQ_STS                     0xC958
#define HAM_NEON_IRQ_EN                 0xC95C
#define HAM_NEON_IRQ_CLR                0xC960


#define PWR_GROUP_MASK0			0xc41c
#define PWR_GROUP_MASK1			0xc420
//define for HAM_IRQ_STS
#define HAM_NEON0_PG_IRQ                1<<2
#define HAM_NEON1_PG_IRQ                1<<3
#define HAM_NEON2_PG_IRQ                1<<4
#define HAM_NEON3_PG_IRQ                1<<5

//define for PMU_DOMAIN_BUSY_STATUS
#define NEON_FPU0_BUSY                  1<<7
#define NEON_FPU1_BUSY                  1<<5
#define NEON_FPU2_BUSY                  1<<3
#define NEON_FPU3_BUSY                  1<<1
#define PMU_DOMAIN_BUSY_STATUS          0xC814

#define PMU_DVFS_TABLE_REG 0xc240

#define CONTEXT_SIZE_BYTES_SHIFT 10
#define CONTEXT_SIZE_BYTES (1<<CONTEXT_SIZE_BYTES_SHIFT)


//define for dbgcmd
#define DBGCMD_BIT_CLOCK_GATING             0
#define DBGCMD_BIT_POWER_GATING             1
#define DBGCMD_BIT_CLOCK_GATING_HW    2
#define DBGCMD_BIT_POWER_GATING_HW    3
#define DBGCMD_CLOCK_GATING    1<<DBGCMD_BIT_CLOCK_GATING
#define DBGCMD_POWER_GATING    1<<DBGCMD_BIT_POWER_GATING
#define DBGCMD_CLOCK_GATING_HW    1<<DBGCMD_BIT_CLOCK_GATING_HW
#define DBGCMD_POWER_GATING_HW    1<<DBGCMD_BIT_POWER_GATING_HW

//define for dbg_dvfs_cmd
#define DBG_DVFS_CMD_BIT_DVFS_ON                  0
#define DBG_DVFS_CMD_BIT_DVFS_HW                  1
#define DBG_DVFS_CMD_BIT_DVFS_CPU_ONOFF           2
#define DBG_DVFS_CMD_DVFS_ON    1<<DBG_DVFS_CMD_BIT_DVFS_ON
#define DBG_DVFS_CMD_DVFS_HW    1<<DBG_DVFS_CMD_BIT_DVFS_HW
#define DBG_DVFS_CMD_DVFS_CPU_ONOFF    1<<DBG_DVFS_CMD_BIT_DVFS_CPU_ONOFF

//define irq for neon monitor
#define ELITE_NEON0_MONITOR_CORES_IRQ 72
#define ELITE_NEON1_MONITOR_CORES_IRQ 78

//define for pmu_cfg
#define PMU_CFG_AVS_EN  0x10
#define PMU_CFG_DVFS_EN 0x08
#define PMU_CFG_PWR_EN  0x04
#define PMU_CFG_CORE0_PG_NO_ABORT  0x10000000
#define PMU_CFG_CORE1_PG_NO_ABORT  0x20000000
#define PMU_CFG_CORE2_PG_NO_ABORT  0x40000000
#define PMU_CFG_CORE3_PG_NO_ABORT  0x80000000


enum cpu_state {
   CPU_ON = 0,
   CPU_CG,
   CPU_PG,
   CPU_OFF,
};

enum cpu_id {
   cpu0 = 0,
   cpu1,
   cpu2,
   cpu3,
};


struct elite_voltage_table {
	unsigned long	        	frequency;
	unsigned long	        	voltage;
};


#define WFI_CFG_KEEP_ORIGINAL_ARM_BEHAVIOR  0
#define WFI_CFG_ENTER_MULTICORE_LP_MODE     1
#define WFI_CFG_ENTER_CPU_STANDBY_MODE      2
#define WFI_CFG_ENTER_CPU_PWR_GATING_MODE   3
#define WFI_CFG_ENTER_SYS_STANDBU_MODE      4
#define WFI_CFG_ENTER_SYS_PWR_GATING_MODE   5
#define WFI_CFG_ENTER_STM_MODE              6
#define WFI_CFG_TRIGGER_REGISTER_BASED_MODE 7
union corex_wfi_cfg {
   struct {    
        unsigned int cfgwfi:4;
        unsigned int reserved:28;
   }cfg_wfi;  
   unsigned int val;
};

#define GENERAL_CFG0_ARM_L2RAM_ON          0
#define GENERAL_CFG0_ARM_L2RAM_RETENTION   1
#define GENERAL_CFG0_ARM_L2RAM_OFF         2
#define GENERAL_CFG0_ARM_L2RAM_KEEP        3

#define GENERAL_CFG0_ARM_SCU_L2CTRL_ON     0    
#define GENERAL_CFG0_ARM_SCU_L2CTRL_CG     1    
#define GENERAL_CFG0_ARM_SCU_L2CTRL_PG     2    
#define GENERAL_CFG0_ARM_SCU_L2CTRL_KEEP   3    

#define GENERAL_CFG0_NEON_FPU_ON           0
#define GENERAL_CFG0_NEON_FPU_CG           1
#define GENERAL_CFG0_NEON_FPU_PG           2
#define GENERAL_CFG0_NEON_FPU_KEEP         3

#define GENERAL_CFG0_ARM_CORE_ON           0
#define GENERAL_CFG0_ARM_CORE_CG           1
#define GENERAL_CFG0_ARM_CORE_PG           2
#define GENERAL_CFG0_ARM_CORE_KEEP         3

#define ARM_CORE_ON           0xC
#define ARM_CORE_CG           0xD
#define ARM_CORE_PG           0xA
#define ARM_CORE_KEEP         0xF


union corex_general_cfg0 {
   struct {    
        unsigned int arm_core:2;
        unsigned int neon_fpu:2;
        unsigned int arm_scu_l2ctrl:2;
        unsigned int arm_l2ram:2;
        unsigned int reserved:24;
   }general_cfg0;  
   unsigned int val;
};

union cpu_lp_cfg_core{
   struct {    
        unsigned int arm_core:2;
        unsigned int neon_fpu:2;
        unsigned int reserved:24;
   }field;  
   unsigned int val;
};

union sys_lp_cfg0 {
    struct {
        unsigned int pmpiram:2;
        unsigned int audio:2;
        unsigned int pcie_sata:2;
        unsigned int reserved1:2;
        unsigned int diu:2;
        unsigned int bmu1:2;
        unsigned int miu0:2;
        unsigned int miu1:2;
        unsigned int bmu0:2;
        unsigned int peripherals:2;
        unsigned int dram:2;
        unsigned int reserved2:10;
    };
    unsigned int val;
};

union sys_lp_cfg1 {
    struct {
        unsigned int pmpiram:2;
        unsigned int audio:2;
        unsigned int pcie_sata:2;
        unsigned int reserved1:2;
        unsigned int diu:2;
        unsigned int bmu1:2;
        unsigned int miu0:2;
        unsigned int miu1:2;
        unsigned int bmu0:2;
        unsigned int peripherals:2;
        unsigned int dram:2;
        unsigned int reserved2:10;
    };
    unsigned int val;
};

union stm_cfg {
    struct {
        unsigned int enmem:1;
        unsigned int envdd:1;
        unsigned int envcc:1;
        unsigned int reserved:29;
     };
     unsigned int val;
};

/*
union cpu_pll_ctrl{
   struct {    
        unsigned int cpupll_fn:10;
        unsigned int cpupll_int:7;
        unsigned int reserved1:3;
        unsigned int cpupll_r:3;
        unsigned int reserved2:5;
        unsigned int cpupll_load:1;
        unsigned int cpupll_pd:1;
        unsigned int reserved3:2;
   }field;  
   unsigned int val;
};
*/

#define GENERAL_CFG0_STATUS_ARM_L2RAM_ON                0
#define GENERAL_CFG0_STATUS_ARM_L2RAM_RETENTION         1
#define GENERAL_CFG0_STATUS_ARM_L2RAM_OFF               2

#define GENERAL_CFG0_STATUS_ARM_SCU_L2CTRL_ON           0
#define GENERAL_CFG0_STATUS_ARM_SCU_L2CTRL_CG           1
#define GENERAL_CFG0_STATUS_ARM_SCU_L2CTRL_PG           2

#define GENERAL_CFG0_STATUS_NEON_FPU0_ON                0
#define GENERAL_CFG0_STATUS_NEON_FPU0_CG                1
#define GENERAL_CFG0_STATUS_NEON_FPU0_PG                2

#define GENERAL_CFG0_STATUS_ARM_CORE0_ON                0
#define GENERAL_CFG0_STATUS_ARM_CORE0_CG                1
#define GENERAL_CFG0_STATUS_ARM_CORE0_PG                2

#define GENERAL_CFG0_STATUS_NEON_FPU1_ON                0
#define GENERAL_CFG0_STATUS_NEON_FPU1_CG                1
#define GENERAL_CFG0_STATUS_NEON_FPU1_PG                2

#define GENERAL_CFG0_STATUS_ARM_CORE1_ON                0
#define GENERAL_CFG0_STATUS_ARM_CORE1_CG                1
#define GENERAL_CFG0_STATUS_ARM_CORE1_PG                2

#define GENERAL_CFG0_STATUS_NEON_FPU2_ON                0
#define GENERAL_CFG0_STATUS_NEON_FPU2_CG                1
#define GENERAL_CFG0_STATUS_NEON_FPU2_PG                2

#define GENERAL_CFG0_STATUS_ARM_CORE2_ON                0
#define GENERAL_CFG0_STATUS_ARM_CORE2_CG                1
#define GENERAL_CFG0_STATUS_ARM_CORE2_PG                2

#define GENERAL_CFG0_STATUS_NEON_FPU3_ON                0
#define GENERAL_CFG0_STATUS_NEON_FPU3_CG                1
#define GENERAL_CFG0_STATUS_NEON_FPU3_PG                2

#define GENERAL_CFG0_STATUS_ARM_CORE3_ON                0
#define GENERAL_CFG0_STATUS_ARM_CORE3_CG                1
#define GENERAL_CFG0_STATUS_ARM_CORE3_PG                2

#define GENERAL_CFG0_STATUS_ON                          0
#define GENERAL_CFG0_STATUS_CG                          1
#define GENERAL_CFG0_STATUS_PG                          2



union general_cfg0_status {
   struct {    
        unsigned int arm_core3:2;
        unsigned int neon_fpu3:2;
        unsigned int arm_core2:2;
        unsigned int neon_fpu2:2;
        unsigned int arm_core1:2;
        unsigned int neon_fpu1:2;
        unsigned int arm_core0:2;
        unsigned int neon_fpu0:2;
        unsigned int arm_scu_l2ctrl:2;
        unsigned int arm_l2ram:2;           
        unsigned int reserved:12;
   }status;  
   unsigned int val;
};


union pmu_status {
   struct {                                        // bit
        unsigned int command_avs_busy:1;           // 0
        unsigned int command_dvfs_busy:1;          // 1
        unsigned int command_pwr_op_busy:1;        // 2
        unsigned int reg_pwr_op_full_done_busy:1;  // 3
        unsigned int i2c_operation_busy:1;         // 4
        unsigned int core0_reg0_pwr_busy:1;        // 5
        unsigned int core1_reg0_pwr_busy:1;        // 6
        unsigned int core2_reg0_pwr_busy:1;        // 7 
        unsigned int core3_reg0_pwr_busy:1;        // 8
        unsigned int gen1_sub0_op_busy:1;          // 9
        unsigned int gen1_sub1_op_busy:1;          // 10
        unsigned int gen1_sub2_op_busy:1;          // 11
        unsigned int gen1_sub3_op_busy:1;          // 12
        unsigned int gen1_sub4_op_busy:1;          // 13
        unsigned int gen1_sub5_op_busy:1;          // 14
        unsigned int gen1_sub6_op_busy:1;          // 15
        unsigned int reg_dvfs_busy:1;              // 16
        unsigned int core0_mswwfi_busy:1;          // 17
        unsigned int core1_mswwfi_busy:1;          // 18
        unsigned int core2_mswwfi_busy:1;          // 19
        unsigned int core3_mswwfi_busy:1;          // 20
        unsigned int reset_task_busy:1;            // 21
        unsigned int reserved:10;                  // 22-31
   }status;  
   unsigned int val;
};

union cpucore_dvfs {
   struct {    
       unsigned int cpu0_lp_status:2;
       unsigned int cpu1_lp_status:2;
       unsigned int cpu2_lp_status:2;
       unsigned int cpu3_lp_status:2;
       unsigned int cur_dvfs_index:4;
       unsigned int avs_vol:8;
       unsigned int reserved:8;
       unsigned int tar_dvfs_index:4;
   }status;  
   unsigned int val;
};

struct pmu_command {
   union {
      struct {
       unsigned int vol_index:4;
       unsigned int reserved:17;
       unsigned int voltage_domain:3;
       unsigned int opcode:8;
      }vol_dvfs; 
      struct {
       unsigned int freq_index:4;
       unsigned int reserved:17;
       unsigned int voltage_domain:3;
       unsigned int opcode:8;
       }clk_dvfs; 
       unsigned int val;
   }u;
};

/*
union dvfs_table {
   struct {    
        uint64_t r:3;
        uint64_t fn:10;
        uint64_t fint:7;
        uint64_t freq_divider:4;
        uint64_t vol:16;
        uint64_t perip_divider:4;
        uint64_t reserved:20;
   }table;  
   struct {    
        unsigned int low;
        unsigned int high;
   }val;  
};
*/
#endif //__MACH_ELITE_CPU_ELITE_H
