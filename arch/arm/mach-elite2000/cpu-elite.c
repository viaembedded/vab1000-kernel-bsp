/*
 * arch/arm/mach-elite/cpu-elite.c
 * elite base function for cpu power management
 *
 * Copyright (c) 2011, S3 Graphics INC.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <asm/clkdev.h>

#include <mach/iomap.h>
#include "cpu-elite.h"
#include <mach/iomap.h>
#include <mach/io.h>
#include "elite-pg.h"
#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>

#include <asm/hardware/cache-l2x0.h>
#include <asm/hardware/gic.h>
#include <asm/pgalloc.h>
#include <asm/tlbflush.h>

#include <linux/tick.h>
#include <linux/cpuidle.h>
#include "clock.h"

#include <linux/rtc.h>

#define DVFS_TABLE_SIZE 16
#define REFERENCE_CLK 24000

bool elite_neon_is_on[MAXCPUS];
//last one is reserved as CPUFREQ_TABLE_END
#define MAX_FREQ_TABLE_SIZE 17
struct cpufreq_frequency_table cpufreq_table[MAX_FREQ_TABLE_SIZE];
unsigned long  elite_pgt_phys;
extern unsigned int dbgcmd;
unsigned int scratch[16];
static DEFINE_MUTEX(elite_neon_lock);
#ifdef CONFIG_CPU_FREQ
extern unsigned int dbg_dvfs_cmd;



union general_cfg0_status sw_cpustatus;
void elite_cpufreq_subinit(void)
{
    sw_cpustatus.status.arm_core0 = CPU_ON;
    sw_cpustatus.status.arm_core1 = CPU_ON;
    sw_cpustatus.status.arm_core2 = CPU_ON;
    sw_cpustatus.status.arm_core3 = CPU_ON;
}


#define PMU_CMD_VOL_DVFS 0x12
#define PMU_CMD_CLK_DVFS 0x13
#define CPU_VOLTAGE_DOMAIN 0
int elite_cpu_clk_set_rate(struct clk *c, unsigned int freq_idx)
{
	int ret=0;
        struct pmu_command pmucmd;
        void __iomem *pmu_cmd_addr;
 	printk(KERN_DEBUG "%s:Set CPU clock index  to %d\n", __func__,freq_idx);
        pmu_cmd_addr = ELITE_PMU_BASE + DVFS_CMDFIFO;
        pmucmd.u.clk_dvfs.opcode = PMU_CMD_CLK_DVFS;
        pmucmd.u.clk_dvfs.voltage_domain = CPU_VOLTAGE_DOMAIN;
        pmucmd.u.clk_dvfs.freq_index = freq_idx;
        if((dbg_dvfs_cmd & DBG_DVFS_CMD_DVFS_HW) == DBG_DVFS_CMD_DVFS_HW)
        {
            printk(KERN_DEBUG "write(%x,%lx) in %s \n",(unsigned int)pmucmd.u.val,(unsigned long)pmu_cmd_addr,__func__);
            writel(pmucmd.u.val,pmu_cmd_addr);
        }
	return ret;
}

int elite_cpu_voltage_set(struct clk *c, unsigned int voltage_idx)
{
	int ret=0;
        struct pmu_command pmucmd;
        void __iomem *pmu_cmd_addr;
 	printk(KERN_DEBUG "%s:Set CPU voltage index  to %d\n", __func__,voltage_idx);
        pmu_cmd_addr = ELITE_PMU_BASE + DVFS_CMDFIFO;
        pmucmd.u.vol_dvfs.opcode = PMU_CMD_VOL_DVFS;
        pmucmd.u.vol_dvfs.voltage_domain = CPU_VOLTAGE_DOMAIN;
        pmucmd.u.vol_dvfs.vol_index = voltage_idx;
        if((dbg_dvfs_cmd & DBG_DVFS_CMD_DVFS_HW) == DBG_DVFS_CMD_DVFS_HW)
        {
            printk(KERN_DEBUG "write(%x,%lx) in %s \n",(unsigned int)pmucmd.u.val,(unsigned long)pmu_cmd_addr,__func__);
            writel(pmucmd.u.val,pmu_cmd_addr);
        }
	return ret;
}


struct cpufreq_frequency_table *build_cpufreq_table(struct elite_cpufreq_table_data *table_data)
{
	int i;

	//for (i = 0; i < ARRAY_SIZE(table_data); i++) {
	for (i = 0; i < DVFS_TABLE_SIZE; i++) {
             cpufreq_table[i].index = table_data->elite_freq_table[i].index;
             cpufreq_table[i].frequency = table_data->elite_freq_table[i].frequency;
	}
    cpufreq_table[i].index = i;
    cpufreq_table[i].frequency = CPUFREQ_TABLE_END;

    return &cpufreq_table[0];
}


/*
struct elite_cpufreq_table_data *elite_cpufreq_table_get(int max_rate)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(cpufreq_tables); i++) {
		struct cpufreq_policy policy;
		ret = cpufreq_frequency_table_cpuinfo(
			&policy, cpufreq_tables[i].freq_table);
		BUG_ON(ret);
		if (policy.max == max_rate)
			return &cpufreq_tables[i];
	}
	pr_err("%s: No cpufreq table matching cpu range", __func__);
	BUG();
	return &cpufreq_tables[0];
}
*/


int elite_get_command_slots(void)
{
  void __iomem *pmu_cmd_addr;
  int slots;
  pmu_cmd_addr = (void __iomem *)ELITE_PMU_BASE + DVFS_CMDFIFO/sizeof(void __iomem);
  if((dbg_dvfs_cmd & DBG_DVFS_CMD_DVFS_HW) == DBG_DVFS_CMD_DVFS_HW)
  {
      slots = readl(pmu_cmd_addr);
      printk(KERN_DEBUG "slots = readl(%lx) = %x in %s \n",(unsigned long)pmu_cmd_addr,slots,__func__);
  }
  else 
      slots = 2;
  return slots;
}

void elite_get_cpus_status(union general_cfg0_status *cpustatus)
{
  void __iomem *general_cfg0_status_addr;
  general_cfg0_status_addr = ELITE_PMU_BASE + GENERAL_CFG0_STATUS;
  if((dbg_dvfs_cmd & DBG_DVFS_CMD_DVFS_HW) == DBG_DVFS_CMD_DVFS_HW)
  {
      cpustatus->val = readl(general_cfg0_status_addr);
      printk(KERN_DEBUG "cpustatus->val = readl(%lx) = %x in %s \n",(unsigned long)general_cfg0_status_addr,cpustatus->val,__func__);
  }
  else 
      cpustatus->val = sw_cpustatus.val;
}

void elite_set_cpus_status(union general_cfg0_status *cpustatus)
{
  void __iomem *general_cfg0_status_addr;
  general_cfg0_status_addr = ELITE_PMU_BASE + GENERAL_CFG0_STATUS;
  if((dbg_dvfs_cmd & DBG_DVFS_CMD_DVFS_HW) == DBG_DVFS_CMD_DVFS_HW)
  {
      writel(cpustatus->val,general_cfg0_status_addr);
      printk(KERN_DEBUG "writel(%x,%lx) in %s \n",cpustatus->val,(unsigned long)general_cfg0_status_addr,__func__);
  }
  else 
      sw_cpustatus.val = cpustatus->val;
}

void elite_get_pmu_status(union pmu_status *pmustatus)
{
  void __iomem *pmu_status_addr;
  pmu_status_addr = ELITE_PMU_BASE + PMU_STATUS;
  if((dbg_dvfs_cmd & DBG_DVFS_CMD_DVFS_HW) == DBG_DVFS_CMD_DVFS_HW)
  {
      pmustatus->val = readl(pmu_status_addr);
      printk(KERN_DEBUG "pmustatus->val = readl(%lx) = %x in %s \n",(unsigned long)pmu_status_addr,pmustatus->val,__func__);
  }
  else 
      pmustatus->status.command_dvfs_busy = 0;
}

void elite_get_cpucore_dvfs(union cpucore_dvfs *cpucore_dvfs)
{
  void __iomem *cpucore_dvfs_addr;
  cpucore_dvfs_addr = ELITE_PMU_BASE + CPUCore_DVFS;
  if((dbg_dvfs_cmd & DBG_DVFS_CMD_DVFS_HW) == DBG_DVFS_CMD_DVFS_HW)
  {
      cpucore_dvfs->val = readl(cpucore_dvfs_addr);
      printk(KERN_DEBUG "cpucore_dvfs->val = readl(%lx) = %x in %s \n",(unsigned long)cpucore_dvfs_addr,cpucore_dvfs->val,__func__);
  }
}

void elite_dump_dvfs_table_to_pmu(struct elite_cpufreq_table_data *table_data)
{
 int i;
 unsigned int intN;
 void __iomem *pmu_dvfs_table_addr;

 union dvfs_table dvfstable;
 struct elite_cpufreq_frequency_table *elite_freq_table;
 unsigned long	        	root_frequency,frequency;

 if((dbg_dvfs_cmd & DBG_DVFS_CMD_DVFS_HW) != DBG_DVFS_CMD_DVFS_HW)
   return;
 elite_freq_table = table_data->elite_freq_table;
 root_frequency = table_data->root_frequency;
 pmu_dvfs_table_addr = (void __iomem *)ELITE_PMU_BASE + PMU_DVFS_TABLE_REG;
 printk(KERN_DEBUG "DVFS TABLE :\n");
 printk(KERN_DEBUG "      IDX  , p-div  , voltage, f-div  ,  int   ,   fn   ,   r     \n");
 dvfstable.val.low = 0; //clear
 dvfstable.val.high = 0; //clear
 for(i=0;i<DVFS_TABLE_SIZE;i++)
 {
    dvfstable.table.vol = elite_freq_table->voltage;
    dvfstable.table.perip_divider = elite_freq_table->perip_divider;
    dvfstable.table.freq_divider = elite_freq_table->freq_divider;
    if(elite_freq_table->freq_divider!=0)
        frequency = root_frequency;
    else
        frequency = elite_freq_table->frequency;
    intN = frequency / (REFERENCE_CLK /1000);
    dvfstable.table.fint = intN/1000 - 2;
    dvfstable.table.fn = (intN - dvfstable.table.fint *1000)*1024/1000; 
    dvfstable.table.r = 0;
    printk(KERN_DEBUG "%8d,%8d,%8d,%8d,%8x,%8x,%8x,%8x,%8x\n",i,dvfstable.table.perip_divider,dvfstable.table.vol,dvfstable.table.freq_divider,dvfstable.table.fint,dvfstable.table.fn,dvfstable.table.r,dvfstable.val.low,dvfstable.val.high);
    writel(dvfstable.val.low,pmu_dvfs_table_addr);
    pmu_dvfs_table_addr += 4 ;
    writel(dvfstable.val.high,pmu_dvfs_table_addr);
    pmu_dvfs_table_addr += 4;
    elite_freq_table++;
 }
}

#endif //#ifdef CONFIG_CPU_FREQ

#define PMU_PTIMER_ENABLE 1
void elite_set_pg_timer(unsigned int us,int cpu)
{
  void __iomem *pmu_ptimer_counter_reg_addr;
  void __iomem *pmu_ptimer_ctrl_reg_addr;
  unsigned pmu_ptimer_ctrl_val;
  pmu_ptimer_counter_reg_addr = (void __iomem *)ELITE_PMU_BASE;
  pmu_ptimer_ctrl_reg_addr = (void __iomem *)ELITE_PMU_BASE;
  switch (cpu) {
  case cpu0:
       pmu_ptimer_counter_reg_addr += 0xc7a0;//PMU_CORE0_PTIMER_COUNTER_REG; 
       pmu_ptimer_ctrl_reg_addr += 0xc7b0;//PMU_CORE0_PTIMER_CTRL_REG; 
       break;
  case cpu1:
       pmu_ptimer_counter_reg_addr += PMU_CORE1_PTIMER_COUNTER_REG; 
       pmu_ptimer_ctrl_reg_addr += PMU_CORE1_PTIMER_CTRL_REG; 
       break;
  case cpu2:
       pmu_ptimer_counter_reg_addr += PMU_CORE2_PTIMER_COUNTER_REG; 
       pmu_ptimer_ctrl_reg_addr += PMU_CORE2_PTIMER_CTRL_REG; 
       break;
  case cpu3:
       pmu_ptimer_counter_reg_addr += PMU_CORE3_PTIMER_COUNTER_REG; 
       pmu_ptimer_ctrl_reg_addr += PMU_CORE3_PTIMER_CTRL_REG; 
       break;
  default:
       BUG();
  }
  //if((dbgcmd & DBGCMD_POWER_GATING_HW) == DBGCMD_POWER_GATING_HW)
  {
      //clear to make counter witable
      pmu_ptimer_ctrl_val = 0;
      __raw_writel(pmu_ptimer_ctrl_val,pmu_ptimer_ctrl_reg_addr);
      __raw_writel(us,pmu_ptimer_counter_reg_addr);
  }
}
void elite_turn_on_pg_timer(unsigned int us,int cpu)
{
  void __iomem *pmu_ptimer_counter_reg_addr;
  void __iomem *pmu_ptimer_ctrl_reg_addr;
  unsigned pmu_ptimer_ctrl_val;
  pmu_ptimer_counter_reg_addr = (void __iomem *)ELITE_PMU_BASE;
  pmu_ptimer_ctrl_reg_addr = (void __iomem *)ELITE_PMU_BASE;
  switch (cpu) {
  case cpu0:
       pmu_ptimer_counter_reg_addr += 0xc7a0;//PMU_CORE0_PTIMER_COUNTER_REG; 
       pmu_ptimer_ctrl_reg_addr += 0xc7b0;//PMU_CORE0_PTIMER_CTRL_REG; 
       break;
  case cpu1:
       pmu_ptimer_counter_reg_addr += PMU_CORE1_PTIMER_COUNTER_REG; 
       pmu_ptimer_ctrl_reg_addr += PMU_CORE1_PTIMER_CTRL_REG; 
       break;
  case cpu2:
       pmu_ptimer_counter_reg_addr += PMU_CORE2_PTIMER_COUNTER_REG; 
       pmu_ptimer_ctrl_reg_addr += PMU_CORE2_PTIMER_CTRL_REG; 
       break;
  case cpu3:
       pmu_ptimer_counter_reg_addr += PMU_CORE3_PTIMER_COUNTER_REG; 
       pmu_ptimer_ctrl_reg_addr += PMU_CORE3_PTIMER_CTRL_REG; 
       break;
  default:
       BUG();
  }
  //if((dbgcmd & DBGCMD_POWER_GATING_HW) == DBGCMD_POWER_GATING_HW)
  {
      pmu_ptimer_ctrl_val = PMU_PTIMER_ENABLE;
      __raw_writel(pmu_ptimer_ctrl_val,pmu_ptimer_ctrl_reg_addr); //gxn
  }
}

#ifdef CONFIG_CPU_IDLE
extern void elite_pg_startup(void);
void __cortex_a9_save(unsigned int restore_address,unsigned int mode);
void elite_get_cg_times(__u64 *times,enum cpu_id cpuid)
{
  void __iomem *cg_times_addr;
  int *timelow;
  int *timeHigh;
  int low,high;
        
  cg_times_addr = (void __iomem *)ELITE_PMU_BASE;
  switch (cpuid) {
  case cpu0:
       cg_times_addr += PMU_CG_CORE0_COUNT; 
       break;
  case cpu1:
       cg_times_addr += PMU_CG_CORE1_COUNT; 
       break;
  case cpu2:
       cg_times_addr += PMU_CG_CORE2_COUNT; 
       break;
  case cpu3:
       cg_times_addr += PMU_CG_CORE3_COUNT; 
       break;
  default:
       BUG();
  }
  if((dbgcmd & DBGCMD_CLOCK_GATING_HW) == DBGCMD_CLOCK_GATING_HW)
  {
      timelow = (int *)&times[cpuid];
      timeHigh = timelow + 1;
      *timelow = __raw_readl(cg_times_addr);
      low = *timelow;
      //printk(KERN_DEBUG "__raw_readl(%x) = %x in %s \n",(unsigned int)cg_times_addr,low,__func__);
      cg_times_addr += 4;
      *timeHigh = __raw_readl(cg_times_addr);
      high = *timeHigh;
      //printk(KERN_DEBUG "__raw_readl(%x) = %x in %s \n",(unsigned int)cg_times_addr,high,__func__);
  }
}

void elite_get_cg_cycles(__u64 *cycles,enum cpu_id cpuid)
{
  void __iomem *cg_cycles_addr;
  int *cyclelow;
  int *cycleHigh;
  int low,high;
  cg_cycles_addr = (void __iomem *)ELITE_PMU_BASE;
  switch (cpuid) {
  case cpu0:
       cg_cycles_addr += PMU_CG_CORE0_CYCCOUNT; 
       break;
  case cpu1:
       cg_cycles_addr += PMU_CG_CORE1_CYCCOUNT; 
       break;
  case cpu2:
       cg_cycles_addr += PMU_CG_CORE2_CYCCOUNT; 
       break;
  case cpu3:
       cg_cycles_addr += PMU_CG_CORE3_CYCCOUNT; 
       break;
  default:
       BUG();
  }
  if((dbgcmd & DBGCMD_CLOCK_GATING_HW) == DBGCMD_CLOCK_GATING_HW)
  {
      cyclelow = (int *)&cycles[cpuid];
      cycleHigh = cyclelow + 1;
      *cyclelow = __raw_readl(cg_cycles_addr);
      low = *cyclelow;
      //printk(KERN_DEBUG "__raw_readl(%x) = %x in %s \n",(unsigned int)cg_cycles_addr,low,__func__);
      cg_cycles_addr += 4;
      *cycleHigh = __raw_readl(cg_cycles_addr);
      high = *cycleHigh;
      //printk(KERN_DEBUG "__raw_readl(%x) = %x in %s \n",(unsigned int)cg_cycles_addr,high,__func__);
  }
}

void elite_get_pg_times(int *times,enum cpu_id cpuid)
{
  void  __iomem *pg_times_addr;
  int *timelow;
  int *timeHigh;
  int low,high;
  pg_times_addr = (void __iomem *)ELITE_PMU_BASE;
  switch (cpuid) {
  case cpu0:
       pg_times_addr += PMU_PG_CORE0_COUNT; 
       break;
  case cpu1:
       pg_times_addr += PMU_PG_CORE1_COUNT; 
       break;
  case cpu2:
       pg_times_addr += PMU_PG_CORE2_COUNT; 
       break;
  case cpu3:
       pg_times_addr += PMU_PG_CORE3_COUNT; 
       break;
  default:
       BUG();
  }
  if((dbgcmd & DBGCMD_POWER_GATING_HW) == DBGCMD_POWER_GATING_HW)
  {
      timelow = (int *)&times[cpuid];
      timeHigh = timelow + 1;
      *timelow = __raw_readl(pg_times_addr);
      low = *timelow;
      //printk(KERN_DEBUG "__raw_readl(%x) = %x in %s \n",(unsigned int)pg_times_addr,low,__func__);
      pg_times_addr += 4;
      *timeHigh = __raw_readl(pg_times_addr);
      high = *timeHigh;
      //printk(KERN_DEBUG "__raw_readl(%x) = %x in %s \n",(unsigned int)pg_times_addr,high,__func__);
  }
}

void elite_get_pg_cycles(int *cycles,enum cpu_id cpuid)
{
  void __iomem *pg_cycles_addr;
  int *cyclelow;
  int *cycleHigh;
  int low,high;
  pg_cycles_addr = (void __iomem *)ELITE_PMU_BASE;
  switch (cpuid) {
  case cpu0:
       pg_cycles_addr += PMU_PG_CORE0_CYCCOUNT; 
       break;
  case cpu1:
       pg_cycles_addr += PMU_PG_CORE1_CYCCOUNT; 
       break;
  case cpu2:
       pg_cycles_addr += PMU_PG_CORE2_CYCCOUNT; 
       break;
  case cpu3:
       pg_cycles_addr += PMU_PG_CORE3_CYCCOUNT; 
       break;
  default:
       BUG();
  }
  if((dbgcmd & DBGCMD_POWER_GATING_HW) == DBGCMD_POWER_GATING_HW)
  {
      cyclelow = (int *)&cycles[cpuid];
      cycleHigh = cyclelow + 1;
      *cyclelow = __raw_readl(pg_cycles_addr);
      low = *cyclelow;
      //printk(KERN_DEBUG "__raw_readl(%x) = %x in %s \n",(unsigned int)pg_cycles_addr,low,__func__);
      pg_cycles_addr +=4;
      *cycleHigh = __raw_readl(pg_cycles_addr);
      high = *cycleHigh;
      //printk(KERN_DEBUG "__raw_readl(%x) = %x in %s \n",(unsigned int)pg_cycles_addr,high,__func__);
  }
}

void elite_corex_wfi_cfg(union corex_wfi_cfg corexwficfg,enum cpu_id cpuid)
{
  void __iomem *corex_wfi_cfg_addr;
  corex_wfi_cfg_addr = (void __iomem *)ELITE_PMU_BASE;
  switch (cpuid) {
  case cpu0:
       corex_wfi_cfg_addr += CORE0_WFI_CFG; 
       break;
  case cpu1:
       corex_wfi_cfg_addr += CORE1_WFI_CFG; 
       break;
  case cpu2:
       corex_wfi_cfg_addr += CORE2_WFI_CFG; 
       break;
  case cpu3:
       corex_wfi_cfg_addr += CORE3_WFI_CFG; 
       break;
  default:
       BUG();
  }
  //printk(KERN_DEBUG "to __raw_writel(%x,%x) in %s \n",corexwficfg.val,(unsigned int)corex_wfi_cfg_addr,__func__);
  //if(((dbgcmd & DBGCMD_CLOCK_GATING_HW) == DBGCMD_CLOCK_GATING_HW) ||
  //   ((dbgcmd & DBGCMD_POWER_GATING_HW) == DBGCMD_POWER_GATING_HW))
    { 
      //printk(KERN_DEBUG "right before __raw_writel(%x,%x) in %s \n",corexwficfg.val,(unsigned int)corex_wfi_cfg_addr,__func__);
      //__raw___raw_writel(corexwficfg.val,corex_wfi_cfg_addr);
      __raw_writel(corexwficfg.val,corex_wfi_cfg_addr);
      //printk(KERN_DEBUG "right after __raw_writel(%x,%x) in %s \n",corexwficfg.val,(unsigned int)corex_wfi_cfg_addr,__func__);
     }
}

void elite_cpu_lp_cfg_corex(int val,enum cpu_id cpuid)
{
  void __iomem *cpu_lp_cfg_addr;
  cpu_lp_cfg_addr = (void __iomem *)ELITE_PMU_BASE;
  switch (cpuid) {
  case cpu0:
       cpu_lp_cfg_addr += CPU_LP_CFG_CORE0; 
       break;
  case cpu1:
       cpu_lp_cfg_addr += CPU_LP_CFG_CORE1; 
       break;
  case cpu2:
       cpu_lp_cfg_addr += CPU_LP_CFG_CORE2; 
       break;
  case cpu3:
       cpu_lp_cfg_addr += CPU_LP_CFG_CORE3; 
       break;
  default:
       BUG();
 }
 //if(((dbgcmd & DBGCMD_CLOCK_GATING_HW) == DBGCMD_CLOCK_GATING_HW) ||
 //    ((dbgcmd & DBGCMD_POWER_GATING_HW) == DBGCMD_POWER_GATING_HW))
 { 
      //__raw_writel(val,cpu_lp_cfg_addr);
      __raw_writel(val,cpu_lp_cfg_addr);
 }
}

bool elite_pg_is_on(void)
{
  //if the value in MCBOOT7 is 1, turn on CPU power gating:
  void __iomem *addr;
  addr = (void __iomem *)ELITE_PMU_BASE + MCBOOT7;
  return (readl(addr)==1);
}


/*
void set_general_cfg0_corex(union general_cfg0_corex generalcfg0corex,enum cpu_id cpuid)
{
  __iomem *general_cfg0_corex_addr;
  general_cfg0_corex_addr = ELITE_PMU_BASE;
  switch (cpuid) {
  case cpu0:
       general_cfg0_corex_addr += CORE0_WFI_CFG/sizeof(__iomem); 
       break;
  case cpu1:
       general_cfg0_corex_addr += CORE1_WFI_CFG/sizeof(__iomem); 
       break;
  case cpu2:
       general_cfg0_corex_addr += CORE2_WFI_CFG/sizeof(__iomem); 
       break;
  case cpu3:
       general_cfg0_corex_addr += CORE3_WFI_CFG/sizeof(__iomem); 
       break;
  default:
       BUG();
  }

  if(((dbgcmd & DBGCMD_CLOCK_GATING_HW) == DBGCMD_CLOCK_GATING_HW) ||
     ((dbgcmd & DBGCMD_POWER_GATING_HW) == DBGCMD_POWER_GATING_HW))
    { 
      printf("right before __raw_writel(%x,%x) in %s \n",generalcfg0corex.val,(unsigned int)general_cfg0_corex_addr,__func__);
      __raw_writel(corexwficfg.val,general_cfg0_corex_addr);
      printf("right after __raw_writel(%x,%x) in %s \n",generalcfg0corex.val,(unsigned int)general_cfg0_corex_addr,__func__);
     }
}
*/

unsigned int irqdw0,irqdw1,irqdw2;
void elite_clear_irq(void)
{
  void __iomem *clear_irq_addr;
  void __iomem *set_irq_addr;
  unsigned clear_irq_val;
  set_irq_addr = (void __iomem *)GIC_DIST_BASE + 0x100;
  irqdw0 = readl(set_irq_addr);
  set_irq_addr = (void __iomem *)GIC_DIST_BASE + 0x104;
  irqdw1 = readl(set_irq_addr);
  set_irq_addr = (void __iomem *)GIC_DIST_BASE + 0x108;
  irqdw2 = readl(set_irq_addr);

  clear_irq_val = 0xffffffff;
  clear_irq_addr = (void __iomem *)GIC_DIST_BASE + 0x180;
  writel(clear_irq_val,clear_irq_addr); 
  clear_irq_addr = (void __iomem *)GIC_DIST_BASE + 0x184;
  writel(clear_irq_val,clear_irq_addr); 
  clear_irq_addr = (void __iomem *)GIC_DIST_BASE + 0x188;
  writel(clear_irq_val,clear_irq_addr); 
}
void elite_set_irq(void)
{
  void __iomem *set_irq_addr;
  set_irq_addr = (void __iomem *)GIC_DIST_BASE + 0x100;
  writel(irqdw0,set_irq_addr); 
  set_irq_addr = (void __iomem *)GIC_DIST_BASE + 0x104;
  writel(irqdw1,set_irq_addr); 
  set_irq_addr = (void __iomem *)GIC_DIST_BASE + 0x108;
  writel(irqdw2,set_irq_addr); 
}
void elite_disable_irq(void)
{
  void __iomem *disable_irq_addr;
  unsigned disable_irq_val;
  disable_irq_val = 0;
  disable_irq_addr = (void __iomem *)GIC_CPUINTERFACE_BASE + 0x000;
  writel(disable_irq_val,disable_irq_addr); 
}
void elite_enable_irq(void)
{
  void __iomem *enable_irq_addr;
  unsigned enable_irq_val;
  enable_irq_val = 1;
  enable_irq_addr = (void __iomem *)GIC_CPUINTERFACE_BASE + 0x000;
  writel(enable_irq_val,enable_irq_addr); 
}
unsigned int elite_pg_timer_remain(int cpu)
{
  void __iomem *pmu_ptimer_counter_reg_addr;
  unsigned int pmu_ptimer_counter_val;
  pmu_ptimer_counter_reg_addr = (void __iomem *)ELITE_PMU_BASE;
  switch (cpu) {
  case cpu0:
       pmu_ptimer_counter_reg_addr += PMU_CORE0_PTIMER_COUNTER_REG; 
       break;
  case cpu1:
       pmu_ptimer_counter_reg_addr += PMU_CORE1_PTIMER_COUNTER_REG; 
       break;
  case cpu2:
       pmu_ptimer_counter_reg_addr += PMU_CORE2_PTIMER_COUNTER_REG; 
       break;
  case cpu3:
       pmu_ptimer_counter_reg_addr += PMU_CORE3_PTIMER_COUNTER_REG; 
       break;
  default:
       BUG();
  }
  if((dbgcmd & DBGCMD_POWER_GATING_HW) == DBGCMD_POWER_GATING_HW)
  {
      pmu_ptimer_counter_val = readl(pmu_ptimer_counter_reg_addr);
      //printk(KERN_DEBUG "readl(%x) = %x in %s \n",(unsigned int)pmu_ptimer_counter_reg_addr,pmu_ptimer_counter_val,__func__);
  }
  else 
      pmu_ptimer_counter_val = 0;
  return pmu_ptimer_counter_val;
}

#define POWER_GATING_MODE 0
unsigned int elite_power_gating(unsigned int us,struct cpuidle_device *dev)
{
    unsigned int mode;
    unsigned int remain=0;
    unsigned long orig=0;
    void __iomem *pg_mcboot_addr;
    void __iomem *pg_dbg_addr;
    union corex_wfi_cfg wfi_cfg;
	int clk=27; //600/us
    int cpu=dev->cpu;


     //pg_mcboot_addr = (void __iomem *)ELITE_PMU_BASE;
     pg_mcboot_addr = (void __iomem *)0xfe130000;
     switch (cpu) {
     case 0:
         pg_mcboot_addr += MCSBOOT0;
         break;
     case 1:
         pg_mcboot_addr += MCSBOOT1;
         break;
     case 2:
         pg_mcboot_addr += MCSBOOT2;
         break;
     case 3:
         pg_mcboot_addr += MCSBOOT3;
         break;
         default:
         BUG();
     }
	     orig = __raw_readl(pg_mcboot_addr);
	     __raw_writel(virt_to_phys(elite_pg_startup), pg_mcboot_addr);

	if(1)     
	{
         int cycles;
         cycles = us*clk;
	//cycles = 1*clk;
         //cycles = 10;
	printk("@@@@@$s:PG IN\n",__func__);
        elite_set_pg_timer(cycles,cpu);
        elite_turn_on_pg_timer(cycles,cpu);
        stop_critical_timings();
	flush_cache_all();
	barrier();
         __cortex_a9_save(virt_to_phys(elite_pg_startup),0);
	printk("@@@@@%s:PG BACK\n",__func__);
       }

 
    	barrier();
	start_critical_timings();

        elite_set_pg_timer(us,cpu);
	remain =elite_pg_timer_remain(cpu)/clk;

        __raw_writel(orig, pg_mcboot_addr);


	return remain;
}
#endif //#ifdef CONFIG_CPU_IDLE

extern wakeupcfg_pre_suspend(void);
extern void elite_cpu_wfi_cfg(int cpu, int mode);
extern int elite_rtc_read_time(struct device *dev, struct rtc_time *time);
extern int elite_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm);

#if 0
static void elite_set_wake_alarm(void)
{
    struct rtc_wkalrm       alm;
    unsigned long now;
    int status;

    status = elite_rtc_read_time(NULL, &alm.time);
    if (status < 0) {
        return;
    }

    rtc_tm_to_time(&alm.time, &now);

    memset(&alm, 0, sizeof alm);
    rtc_time_to_tm(now + 1, &alm.time);
    alm.enabled = true;
    printk("now =%d begin reboot\n", now);

    status = elite_rtc_set_alarm(NULL, &alm);
}

void elite_fake_reset()
{
    int i;
    void __iomem *stm_reset_addr;
    void __iomem *gic_cpu_base_addr = (void __iomem *)IO_ADDRESS(ELITE_ARM_PERIF_GIC_CPU_BASE);
    void __iomem *stm_cfg_addr;

    gic_cpu_base_addr += GIC_CPU_CTRL;

    for(i = 0; i < NR_CPUS; i++)
    {
        elite_cpu_wfi_cfg(i, WFI_CFG_ENTER_STM_MODE);
    }

    stm_reset_addr = (void __iomem *)ELITE_PMU_BASE;

    stm_reset_addr += PMU_CFG;

    /* clear pmc interrupt */
    writel(0xffffffff, IO_ADDRESS(0xd813001c));
    /*clear pmu interrupt */
    writel(0xffffffff, IO_ADDRESS(0xd839c6cc));

    /* set reboot flag */
    writel(0x1 , IO_ADDRESS(0xd839340c));
    writel(0x0, gic_cpu_base_addr);

    elite_set_wake_alarm();

    wakeupcfg_pre_suspend();

    writel(readl(stm_reset_addr)|(1<<25),stm_reset_addr);

    while(!(readl(stm_reset_addr)&(1<<25)));

    stm_cfg_addr = (void __iomem *)ELITE_PMU_BASE;

    stm_cfg_addr += STM_CFG;

    writel(0x0,stm_cfg_addr);

    while(readl(stm_cfg_addr));

    writel((readl(IO_ADDRESS(0xd839D200))|(1<<16))&(~(1<<17)),IO_ADDRESS(0xd839D200));

    asm volatile("wfi");
}
#endif
void elite_pmu_reset(char mode, const char *cmd)
{

        void __iomem *corex_chip_reset_addr;
	void __iomem *elite_pe0_reset_addr;
	void __iomem *elite_pe1_reset_addr;
	
        corex_chip_reset_addr = (void __iomem *)ELITE_PMU_BASE;
        corex_chip_reset_addr += PMU_SW_FULLCHIP_RESET;

	elite_pe0_reset_addr= (void __iomem *)ELITE_PCIE0_BASE;
	elite_pe0_reset_addr += 0x254;
	elite_pe1_reset_addr= (void __iomem *)ELITE_PCIE1_BASE;
	elite_pe1_reset_addr += 0x254;

        writel(0x0, (void __iomem *)ELITE_PMU_BASE + 0xc06c);        
	if (cmd) { //inform bootloader to enter Android recovery mode
		pr_info("System requested to enter recovery mode with cmd: %s\n", cmd);
		if (!strcmp(cmd, "recovery")) {//using Vsus power domain GENERAL_CFG1_WAKEUP register
			writel(0x2, (void __iomem *)ELITE_PMU_BASE + 0xc06c);
		}
	}
	writel((readl(elite_pe0_reset_addr)& ~0x80000000),elite_pe0_reset_addr);      
	writel((readl(elite_pe1_reset_addr)& ~0x80000000),elite_pe1_reset_addr);    
       // writel(0x1,corex_chip_reset_addr);
       // elite_fake_reset();
	
}

void elite_pmu_power_off(char mode, const char *cmd)
{

        void __iomem *corex_pmu_cfg_addr;
        void __iomem *corex_pmu_wfi_cfg;
	
        corex_pmu_cfg_addr = (void __iomem *)ELITE_PMU_BASE;
        corex_pmu_wfi_cfg = (void __iomem *)ELITE_PMU_BASE;
	
        corex_pmu_cfg_addr += PMU_CFG;	
        writel(0x800004,corex_pmu_cfg_addr);

        corex_pmu_wfi_cfg += CORE0_WFI_CFG;	
        writel(0x7,corex_pmu_wfi_cfg);	

        asm volatile("wfi"); 
	
}

#define DBGSWENABLE 0x00000002
#define PWRRDY_EN   0x00010000
#define GENERALCFGTRIGGER 0x30000000
#define PMUPGSYNC 0x08000000
#define PMUGRPMASKENABLE 0x00000400
void   elite_pmu_enable(void)
{
        void __iomem *pmu_xxx_addr;
        unsigned long pmu_xxx_val;
        int i;

   	pmu_xxx_addr = (void __iomem *)ELITE_PMU_BASE + 0XC500;
        pmu_xxx_val = 0x4;
		for(i=0;i<22;i++)
		{
                        if((i==0) || (i==(0x24>>2)) || (i==(0x28>>2)))
                        {
                            pmu_xxx_val = 4;//0xA;
                        }
                        else if(i==(0x08>>2))
                        {
                            pmu_xxx_val = 4;//0x64;
                        }
                        else
                            pmu_xxx_val = 0x4;
                        __raw_writel(pmu_xxx_val,pmu_xxx_addr);
			pmu_xxx_addr += 4;
		}
		
	//adjust PG interval
   	pmu_xxx_addr = (void __iomem *)ELITE_PMU_BASE + 0XC4c8;
        pmu_xxx_val = __raw_readl(pmu_xxx_addr);
        pmu_xxx_val |= 0x03000000;
        __raw_writel(pmu_xxx_val,pmu_xxx_addr);

	pmu_xxx_addr = (void __iomem *)ELITE_PMU_BASE + PMU_CFG;
        pmu_xxx_val = PMU_CFG_PWR_EN | PMU_CFG_DVFS_EN;
        //pmu_xxx_val |= PMU_CFG_CORE0_PG_NO_ABORT | PMU_CFG_CORE1_PG_NO_ABORT | PMU_CFG_CORE2_PG_NO_ABORT | PMU_CFG_CORE3_PG_NO_ABORT ;
        //pmu_xxx_val |= PWRRDY_EN;
	//pmu_xxx_val |= GENERALCFGTRIGGER;
	pmu_xxx_val |= PMUPGSYNC;
	pmu_xxx_val |= PMUGRPMASKENABLE;
        __raw_writel(pmu_xxx_val,pmu_xxx_addr);

	//set GROUP mask

   	pmu_xxx_addr = (void __iomem *)ELITE_PMU_BASE + 0XC41c;
        pmu_xxx_val = __raw_readl(pmu_xxx_addr);
        pmu_xxx_val |= 0x00000500;
        __raw_writel(pmu_xxx_val,pmu_xxx_addr);

        //set WFE trigger enable

        pmu_xxx_addr = (void __iomem *)ELITE_PMU_BASE + 0X3300;
        pmu_xxx_val = __raw_readl(pmu_xxx_addr);
        pmu_xxx_val |= 0x0000000C;   //bit2 for CPU0,bit3 for CPU1
        __raw_writel(pmu_xxx_val,pmu_xxx_addr);

	//enable CP14 external access
	pmu_xxx_addr = (void __iomem *)ELITE_PMU_BASE + DAP_CTRL;
	pmu_xxx_val = __raw_readl(pmu_xxx_addr);
        pmu_xxx_val |= DBGSWENABLE;
        __raw_writel(pmu_xxx_val,pmu_xxx_addr);

	return;
}

unsigned int elite_get_gic_base(void)
{
    unsigned int gic_base;            /* to be used by inline assembler */

    asm(                              /* invoke the inline assembler */
      "MRC p15,4,%[r0],c15,c0,0"      /* Read MPIDR */
      :[r0]"=r" (gic_base)            /* output */
      :                               /* no input */     
    );
   return gic_base;
}

unsigned int elite_read_cpuid(void)
{
    unsigned int cpuid;   

   //cpuid = smp_processor_id();
    asm(                              /* invoke the inline assembler */
      "MRC p15,0,%[r0],c0,c0,5"      /*  Read Configuration Base Address Register */
      :[r0]"=r" (cpuid)              /* output */
      :                               /* no input */     
    );
    cpuid = cpuid & 0xf;
   //if(cpuid!=0)
   //  printk(KERN_DEBUG "cpuid = %x in %s ",cpuid,__func__); 
   return cpuid;
}

bool elite_neon_is_busy(void)
{
    unsigned int neon_busy_reg_val;   

    asm volatile (                              /* invoke the inline assembler */
      "MRC p15,0,%[r0],c15,c1,0"      /*Read NEON busy  Register */
      :[r0]"=r" (neon_busy_reg_val)   /* output */
      :                               /* no input */     
    );
   if(neon_busy_reg_val & 1)
       return true;
   else
       return false;
}




void elite_set_gic_target_register(int cpu)
{
    void __iomem *gic_disc_target_addr;
    gic_disc_target_addr = elite_get_gic_base + GIC_DIST_TARGET;
    writel(1<<cpu, gic_disc_target_addr);
}


void elite_power_on_neon(int cpu) 
{
  void __iomem *corex_general_cfg0_addr;
  void __iomem *general_cfg0_status_addr;
  union corex_general_cfg0 generalcfg0;

  corex_general_cfg0_addr = (void __iomem *)ELITE_PMU_BASE;
  general_cfg0_status_addr = (void __iomem *)ELITE_PMU_BASE + GENERAL_CFG0_STATUS;

  switch (cpu) {
  case cpu0:
       corex_general_cfg0_addr += GENERAL_CFG0_CORE0; 
       break;
  case cpu1:
       corex_general_cfg0_addr += GENERAL_CFG0_CORE1; 
       break;
  case cpu2:
       corex_general_cfg0_addr += GENERAL_CFG0_CORE2; 
       break;
  case cpu3:
       corex_general_cfg0_addr += GENERAL_CFG0_CORE3; 
       break;
  default:
       BUG();
  }
  if(cpu == cpu0)
  {
      generalcfg0.general_cfg0.arm_l2ram = GENERAL_CFG0_ARM_L2RAM_KEEP;
      generalcfg0.general_cfg0.arm_scu_l2ctrl = GENERAL_CFG0_ARM_SCU_L2CTRL_KEEP;
  }
  generalcfg0.general_cfg0.neon_fpu = GENERAL_CFG0_NEON_FPU_ON;
  generalcfg0.general_cfg0.arm_core = GENERAL_CFG0_ARM_CORE_KEEP;
  writel(generalcfg0.val,corex_general_cfg0_addr);
  //asm volatile("wfi");                    /* trigger, needed?*/

}

void elite_turn_on_neon(int cpu) 
{
  void __iomem *corex_general_cfg0_addr;
  void __iomem *general_cfg0_status_addr;
  unsigned int cpacr_val;   
  int status;
  union corex_general_cfg0 generalcfg0;
  union general_cfg0_status generalcfg0status;

  printk(KERN_DEBUG "%s \n", __func__);
  // read SIMD extension:
  asm(                              /* invoke the inline assembler */
      "MRC p15,0,%[r0],c1,c0,2"       /* Read coprocessor access control register */
      :[r0]"=r" (cpacr_val)           /* output */
      :                               /* no input */     
     );
  //printk(KERN_DEBUG "CPACR=%x before set in %s \n", cpacr_val,__func__);
  corex_general_cfg0_addr = (void __iomem *)ELITE_PMU_BASE;
  general_cfg0_status_addr = (void __iomem *)ELITE_PMU_BASE + GENERAL_CFG0_STATUS;

  switch (cpu) {
  case cpu0:
       corex_general_cfg0_addr += GENERAL_CFG0_CORE0; 
       break;
  case cpu1:
       corex_general_cfg0_addr += GENERAL_CFG0_CORE1; 
       break;
  case cpu2:
       corex_general_cfg0_addr += GENERAL_CFG0_CORE2; 
       break;
  case cpu3:
       corex_general_cfg0_addr += GENERAL_CFG0_CORE3; 
       break;
  default:
       BUG();
  }
  if(cpu == cpu0)
  {
      generalcfg0.general_cfg0.arm_l2ram = GENERAL_CFG0_ARM_L2RAM_KEEP;
      generalcfg0.general_cfg0.arm_scu_l2ctrl = GENERAL_CFG0_ARM_SCU_L2CTRL_KEEP;
  }
  generalcfg0.general_cfg0.neon_fpu = GENERAL_CFG0_NEON_FPU_ON;
  generalcfg0.general_cfg0.arm_core = GENERAL_CFG0_ARM_CORE_KEEP;
  writel(generalcfg0.val,corex_general_cfg0_addr);
  //asm volatile("wfi");                    /* trigger, needed?*/

  // enable SIMD extension:
  asm(                              /* invoke the inline assembler */
      "MRC p15,0,%[r0],c1,c0,2"       /* Read coprocessor access control register */
      :[r0]"=r" (cpacr_val)           /* output */
      :                               /* no input */     
     );
  //printk(KERN_DEBUG "CPACR=%x before set in %s \n", cpacr_val,__func__);
  //cpacr_val |= (3<<20);            // set bit21,20 to turn on CP10
  //cpacr_val &= ~(3<<30);            // clear cpacr's bit31 b30 so that SIMD does not cause unfefined instruction
  cpacr_val |= (3<<20);             // set bit21,20 to turn on CP10
  cpacr_val &= ~(1<<31);            // clear cpacr's bit31 b30 so that SIMD does not cause unfefined instruction
  asm volatile(                     /* invoke the inline assembler */
     "MCR p15,0,%[r0],c1,c0,2"       /* Write coprocessor access control register */
      :                               /* no output */     
      :[r0]"r" (cpacr_val)            /* intput */
     );
  //printk(KERN_DEBUG "CPACR=%x after set in %s \n", cpacr_val,__func__);

  //polling general_cfg0_status to make sure neon is on
  for(;1;)
  {
      generalcfg0status.val = readl(general_cfg0_status_addr);
      switch (cpu) {
      case cpu0:
           status = generalcfg0status.status.neon_fpu0;
           //if(generalcfg0status.status.neon_fpu0 == GENERAL_CFG0_STATUS_NEON_FPU0_ON)
           break;
      case cpu1:
           status = generalcfg0status.status.neon_fpu1;
           break;
      case cpu2:
           status = generalcfg0status.status.neon_fpu2;
           break;
      case cpu3:
           status = generalcfg0status.status.neon_fpu3;
           break;
      default:
           BUG();
      }
      if(status == GENERAL_CFG0_STATUS_ON)
      {
           //printk(KERN_DEBUG "GENERAL_CFG0_STATUS_ON in %s \n", __func__);
           break;
      }
  }
    scratch[1] +=1;
	elite_neon_is_on[cpu] = true;
}

void elite_turn_off_neon(int cpu)
{
  void __iomem *corex_general_cfg0_addr;
  union corex_general_cfg0 generalcfg0;
  unsigned int cpacr_val;   
  
  printk(KERN_DEBUG "%s \n", __func__);
  // enable SIMD extension:
  asm(                              /* invoke the inline assembler */
      "MRC p15,0,%[r0],c1,c0,2"       /* Read coprocessor access control register */
      :[r0]"=r" (cpacr_val)           /* output */
      :                               /* no input */     
     );
  //printk(KERN_DEBUG "CPACR=%x before set in %s \n", cpacr_val,__func__);
  corex_general_cfg0_addr = (void __iomem *)ELITE_PMU_BASE;
  switch (cpu) {
  case cpu0:
       corex_general_cfg0_addr += GENERAL_CFG0_CORE0; 
       break;
  case cpu1:
       corex_general_cfg0_addr += GENERAL_CFG0_CORE1; 
       break;
  case cpu2:
       corex_general_cfg0_addr += GENERAL_CFG0_CORE2; 
       break;
  case cpu3:
       corex_general_cfg0_addr += GENERAL_CFG0_CORE3; 
       break;
  default:
       BUG();
  }
  if(cpu == cpu0)
  {
      generalcfg0.general_cfg0.arm_l2ram = GENERAL_CFG0_ARM_L2RAM_KEEP;
      generalcfg0.general_cfg0.arm_scu_l2ctrl = GENERAL_CFG0_ARM_SCU_L2CTRL_KEEP;
  }
  generalcfg0.general_cfg0.neon_fpu = GENERAL_CFG0_NEON_FPU_PG;
  generalcfg0.general_cfg0.arm_core = GENERAL_CFG0_ARM_CORE_KEEP;
  writel(generalcfg0.val,corex_general_cfg0_addr);
  //asm volatile("wfi");                    /* trigger, needed?*/

  // disable SIMD extension:
  asm(                              /* invoke the inline assembler */
      "MRC p15,0,%[r0],c1,c0,2"       /* Read coprocessor access control register */
      :[r0]"=r" (cpacr_val)           /* output */
      :                               /* no input */     
     );
  //printk(KERN_DEBUG "CPACR=%x before set in %s \n",cpacr_val, __func__);
  //cpacr_val &= ~(3<<20);            // clear bit21,20 to turn off CP10
  //cpacr_val |= (3<<30);             // set cpacr's bit31 b30 so that SIMD cause unfefined instruction
  //cpacr_val &= ~(3<<20);            // clear bit21,20 to turn off CP10
  cpacr_val |= (1<<31 );              // set cpacr's bit31 b30 so that SIMD cause unfefined instruction
   asm volatile(                     /* invoke the inline assembler */
     "MCR p15,0,%[r0],c1,c0,2"       /* Write coprocessor access control register */
      :                               /* no output */     
      :[r0]"r" (cpacr_val)            /* intput */
     );
  //printk(KERN_DEBUG "CPACR=%x after set in %s \n", cpacr_val,__func__);
    scratch[2] +=1;
	elite_neon_is_on[cpu] = false;
}
void elite_power_off_neon(int cpu)
{
  void __iomem *corex_general_cfg0_addr;
  union corex_general_cfg0 generalcfg0;
  
  corex_general_cfg0_addr = (void __iomem *)ELITE_PMU_BASE;
  switch (cpu) {
  case cpu0:
       corex_general_cfg0_addr += GENERAL_CFG0_CORE0; 
       break;
  case cpu1:
       corex_general_cfg0_addr += GENERAL_CFG0_CORE1; 
       break;
  case cpu2:
       corex_general_cfg0_addr += GENERAL_CFG0_CORE2; 
       break;
  case cpu3:
       corex_general_cfg0_addr += GENERAL_CFG0_CORE3; 
       break;
  default:
       BUG();
  }
  if(cpu == cpu0)
  {
      generalcfg0.general_cfg0.arm_l2ram = GENERAL_CFG0_ARM_L2RAM_KEEP;
      generalcfg0.general_cfg0.arm_scu_l2ctrl = GENERAL_CFG0_ARM_SCU_L2CTRL_KEEP;
  }
  generalcfg0.general_cfg0.neon_fpu = GENERAL_CFG0_NEON_FPU_PG;
  generalcfg0.general_cfg0.arm_core = GENERAL_CFG0_ARM_CORE_KEEP;
  writel(generalcfg0.val,corex_general_cfg0_addr);
  //asm volatile("wfi");                    /* trigger, needed?*/

}

void elite_turn_off_neons(void)
{
  void __iomem *corex_general_cfg0_addr;
  union corex_general_cfg0 generalcfg0;
  
  printk(KERN_DEBUG "%s \n", __func__);
  // turn off neon0
  corex_general_cfg0_addr = (void __iomem *)ELITE_PMU_BASE;
  corex_general_cfg0_addr += GENERAL_CFG0_CORE0; 
  generalcfg0.general_cfg0.arm_l2ram = GENERAL_CFG0_ARM_L2RAM_KEEP;
  generalcfg0.general_cfg0.arm_scu_l2ctrl = GENERAL_CFG0_ARM_SCU_L2CTRL_KEEP;
  generalcfg0.general_cfg0.neon_fpu = GENERAL_CFG0_NEON_FPU_PG;
  generalcfg0.general_cfg0.arm_core = GENERAL_CFG0_ARM_CORE_KEEP;
  writel(generalcfg0.val,corex_general_cfg0_addr);
  // turn off neon1,2,3
  corex_general_cfg0_addr = (void __iomem *)ELITE_PMU_BASE;
  corex_general_cfg0_addr += GENERAL_CFG0_CORE1; 
  generalcfg0.general_cfg0.neon_fpu = GENERAL_CFG0_NEON_FPU_PG;
  generalcfg0.general_cfg0.arm_core = GENERAL_CFG0_ARM_CORE_KEEP;
  writel(generalcfg0.val,corex_general_cfg0_addr);
  corex_general_cfg0_addr = (void __iomem *)ELITE_PMU_BASE;
  corex_general_cfg0_addr += GENERAL_CFG0_CORE2; 
  writel(generalcfg0.val,corex_general_cfg0_addr);
  corex_general_cfg0_addr = (void __iomem *)ELITE_PMU_BASE;
  corex_general_cfg0_addr += GENERAL_CFG0_CORE3; 
  writel(generalcfg0.val,corex_general_cfg0_addr);
  //asm volatile("wfi");                    /* trigger, needed?*/

}

#define DAP_HAM_MUL_SEL_DAP 0
#define DAP_HAM_MUL_SEL_HAM 1
void write_ham(unsigned int val,void __iomem *addr)
{
  void __iomem *dap_ham_mux_sel_addr;
  unsigned int dap_ham_mux_sel_val;
  dap_ham_mux_sel_addr = (void __iomem *)ELITE_PMU_BASE + DAP_HAM_MUX_SEL;
  dap_ham_mux_sel_val = DAP_HAM_MUL_SEL_HAM;
  writel(dap_ham_mux_sel_val,dap_ham_mux_sel_addr);
  writel(val,addr);
  //printk(KERN_DEBUG "writel(%x,%x) in %s \n",val,(unsigned int)addr,__func__);
  dap_ham_mux_sel_val = DAP_HAM_MUL_SEL_DAP;
  writel(dap_ham_mux_sel_val,dap_ham_mux_sel_addr);
}
unsigned int read_ham(void __iomem *addr)
{
  void __iomem *dap_ham_mux_sel_addr;
  unsigned int dap_ham_mux_sel_val,read_val;
  dap_ham_mux_sel_addr = (void __iomem *)ELITE_PMU_BASE + DAP_HAM_MUX_SEL;
  dap_ham_mux_sel_val = DAP_HAM_MUL_SEL_HAM;
  writel(dap_ham_mux_sel_val,dap_ham_mux_sel_addr);
  read_val = readl(addr);
  dap_ham_mux_sel_val = DAP_HAM_MUL_SEL_DAP;
  writel(dap_ham_mux_sel_val,dap_ham_mux_sel_addr);
  return read_val;
}
void read_ham_counter(void)
{
  void __iomem *dap_ham_mux_sel_addr;
  void __iomem *ham_neon_cnt0_addr;
  unsigned int dap_ham_mux_sel_val,read_val;
  ham_neon_cnt0_addr = (void __iomem *)ELITE_PMU_BASE + HAM_NEON_CNT_0;
  dap_ham_mux_sel_addr = (void __iomem *)ELITE_PMU_BASE + DAP_HAM_MUX_SEL;
  dap_ham_mux_sel_val = DAP_HAM_MUL_SEL_HAM;
  writel(dap_ham_mux_sel_val,dap_ham_mux_sel_addr);
  printk(KERN_DEBUG "writel(%x,%x) in %s \n",dap_ham_mux_sel_val,(unsigned int)dap_ham_mux_sel_addr,__func__);
  read_val = readl(dap_ham_mux_sel_addr);
  printk(KERN_DEBUG "*******readl(%x) = %x in %s *********\n",(unsigned int)dap_ham_mux_sel_addr,read_val,__func__);

  read_val = readl(ham_neon_cnt0_addr);
  printk(KERN_DEBUG "*******ham cnt:readl(%x) = %x in %s *********\n",(unsigned int)ham_neon_cnt0_addr,read_val,__func__);

  dap_ham_mux_sel_val = DAP_HAM_MUL_SEL_DAP;
  writel(dap_ham_mux_sel_val,dap_ham_mux_sel_addr);
  printk(KERN_DEBUG "writel(%x,%x) in %s \n",dap_ham_mux_sel_val,(unsigned int)dap_ham_mux_sel_addr,__func__);
  read_val = readl(dap_ham_mux_sel_addr);
  printk(KERN_DEBUG "*******readl(%x) = %x in %s *********\n",(unsigned int)dap_ham_mux_sel_addr,read_val,__func__);

}
unsigned int read_ham_irq_sts(void)
{
  void __iomem *dap_ham_mux_sel_addr;
  void __iomem *ham_irq_sts_addr;
  unsigned int dap_ham_mux_sel_val,read_val;
  ham_irq_sts_addr = (void __iomem *)ELITE_PMU_BASE + HAM_IRQ_STS;
  dap_ham_mux_sel_addr = (void __iomem *)ELITE_PMU_BASE + DAP_HAM_MUX_SEL;
  dap_ham_mux_sel_val = DAP_HAM_MUL_SEL_HAM;
  writel(dap_ham_mux_sel_val,dap_ham_mux_sel_addr);
  //printk(KERN_DEBUG "writel(%x,%x) in %s \n",dap_ham_mux_sel_val,(unsigned int)dap_ham_mux_sel_addr,__func__);
  //read_val = readl(dap_ham_mux_sel_addr);
  //printk(KERN_DEBUG "*******readl(%x) = %x in %s *********\n",(unsigned int)dap_ham_mux_sel_addr,read_val,__func__);

  read_val = readl(ham_irq_sts_addr);
  //printk(KERN_DEBUG "*******ham_irq_sts:readl(%x) = %x in %s *********\n",(unsigned int)ham_irq_sts_addr,read_val,__func__);

  dap_ham_mux_sel_val = DAP_HAM_MUL_SEL_DAP;
  writel(dap_ham_mux_sel_val,dap_ham_mux_sel_addr);
  //printk(KERN_DEBUG "writel(%x,%x) in %s \n",dap_ham_mux_sel_val,(unsigned int)dap_ham_mux_sel_addr,__func__);
  //read_val = readl(dap_ham_mux_sel_addr);
  //printk(KERN_DEBUG "*******readl(%x) = %x in %s *********\n",(unsigned int)dap_ham_mux_sel_addr,read_val,__func__);
   return read_val;
}

void dump_status(void)
{
  unsigned int reg_val;   

  printk(KERN_DEBUG "************start  %s *********\n",__func__);
    //dump ham status
    read_ham_irq_sts();
    //dump cpu satus
  // 1:read SIMD extension:
  asm(                              /* invoke the inline assembler */
      "MRC p15,0,%[r0],c1,c0,2"       /* Read coprocessor access control register */
      :[r0]"=r" (reg_val)           /* output */
      :                               /* no input */     
     );
  printk(KERN_DEBUG "1: CPACR(MRC p15,0,r,c1,c0,2)=%x in %s \n", reg_val,__func__);

  //2:
    asm(                              /* invoke the inline assembler */
      "MRC p15,0,%[r0],c15,c1,0"      /*Read NEON busy  Register */
      :[r0]"=r" (reg_val)   /* output */
      :                               /* no input */     
    );
  printk(KERN_DEBUG "2:NEON status(MRC p15,0,r,c15,c1,0) = %x in %s ",reg_val,__func__); 


  //3:
  asm(                              /* invoke the inline assembler */
      "MRC p15,0,%[r0],c1,c0,0"       /* Read coprocessor access security control register */
      :[r0]"=r" (reg_val)           /* output */
      :                               /* no input */     
     );
  printk(KERN_DEBUG "3:SCTLR(MRC p15,0,r,c1,c0,0)=%x in %s \n",reg_val, __func__);
  printk(KERN_DEBUG "************end  %s *********\n",__func__);
}

#define HAM_NEON_PGREF_LOADING 0x1000
#define HAMCNTEN 1<<5
unsigned int elite_read_ham_neon_cnt(int cpu)
{
  void __iomem *ham_neon_cnt_addr;
  unsigned int ham_neon_cnt_val;

  switch (cpu) {
  case cpu0:
       ham_neon_cnt_addr = (void __iomem *)ELITE_PMU_BASE + HAM_NEON_CNT_0;
       break;
  case cpu1:
       ham_neon_cnt_addr = (void __iomem *)ELITE_PMU_BASE + HAM_NEON_CNT_1;
       break;
  case cpu2:
       ham_neon_cnt_addr = (void __iomem *)ELITE_PMU_BASE + HAM_NEON_CNT_2;
       break;
  case cpu3:
       ham_neon_cnt_addr = (void __iomem *)ELITE_PMU_BASE + HAM_NEON_CNT_3;
       break;
  default:
       BUG();
  }
  ham_neon_cnt_val = read_ham(ham_neon_cnt_addr);
  return ham_neon_cnt_val;
}
void elite_set_neon_monitor_timer(void)
{
  void __iomem *ham_neon_pgref_reg_addr;
  unsigned int ham_neon_pgref_val;
  void __iomem *ham_cfg_addr;
  unsigned int ham_cfg_val;
  printk(KERN_DEBUG "%s \n", __func__);
  ham_neon_pgref_reg_addr = (void __iomem *)ELITE_PMU_BASE + HAM_NEON_PGREF;
  ham_neon_pgref_val = HAM_NEON_PGREF_LOADING;
  write_ham(ham_neon_pgref_val,ham_neon_pgref_reg_addr);

  ham_cfg_addr = (void __iomem *)ELITE_PMU_BASE + HAM_CFG;
  ham_cfg_val = HAMCNTEN; 
  write_ham(ham_cfg_val,ham_cfg_addr);
}

#define HAM_NEON_IRQ_MASK 0x0000003C
void elite_ham_neon_irq_clr(int cpu)
{
  void __iomem *ham_irq_sts_addr;
  unsigned int ham_irq_sts_val;

  ham_irq_sts_addr = (void __iomem *)ELITE_PMU_BASE + HAM_IRQ_STS;
  ham_irq_sts_val = read_ham(ham_irq_sts_addr);
  ham_irq_sts_val = ham_irq_sts_val & (4<<cpu); //bit2 for cpu0;
  //clear HAM_NEON irq:
  write_ham(ham_irq_sts_val,ham_irq_sts_addr);
}
void elite_turn_on_neon_monitor_timer(int cpu)
{
  void __iomem *ham_neon_irq_en_reg_addr;
  unsigned int ham_neon_irq_en_val;
  //printk(KERN_DEBUG "%s \n", __func__);
  ham_neon_irq_en_reg_addr = (void __iomem *)ELITE_PMU_BASE + HAM_NEON_IRQ_EN;
  ham_neon_irq_en_val = 1<<cpu; 
  write_ham(ham_neon_irq_en_val,ham_neon_irq_en_reg_addr);
}
void elite_turn_off_neon_monitor_timer(int cpu)
{
  void __iomem *ham_neon_irq_clr_reg_addr;
  unsigned int ham_neon_irq_clr_val;
  //printk(KERN_DEBUG "%s \n", __func__);
  ham_neon_irq_clr_reg_addr = (void __iomem *)ELITE_PMU_BASE + HAM_NEON_IRQ_CLR;
  ham_neon_irq_clr_val = 1<<cpu;; 
  write_ham(ham_neon_irq_clr_val,ham_neon_irq_clr_reg_addr);
}

//called by vfp_support_entry
//entry point:
int      on_neonconter0=0;
int      on_neonconter1=0;
void turn_on_neon(int cpu) 
{
    elite_turn_on_neon(cpu);
    elite_turn_on_neon_monitor_timer(cpu);
    if(cpu==0)
    {
      on_neonconter0++;
      *((int*)0xfe393408) = on_neonconter0;
    }
    if(cpu==1)
    {
      on_neonconter1++;
      *((int*)0xfe39340c) = on_neonconter1;
    }
}

#define HAM_NEON3_PG_IRQ_MASK 1<<5
#define HAM_NEON2_PG_IRQ_MASK 1<<4
#define HAM_NEON1_PG_IRQ_MASK 1<<3
#define HAM_NEON0_PG_IRQ_MASK 1<<2
int neonconter0=0;
int neonconter1=0;
static irqreturn_t elite_neon_turnoff_irs(int irq, void *dev)
{
          
    //void __iomem *ham_irq_sts_reg_addr;
    //unsigned int ham_irq_sts_val;
    unsigned int cpuid,irq_source,irq_mask;
    static spinlock_t irq_lock;
    
    spin_lock_irq(&irq_lock);
    cpuid = elite_read_cpuid();
    irq_source = read_ham_irq_sts();
    irq_mask = 4<<cpuid;

    if(cpuid==0)
      *((int*)0xfe393418) = irq_source+0x80000000;//cpuid+1;
    else if(cpuid==1)
      *((int*)0xfe39341c) = irq_source+0x80000000;//cpuid+1;
    else
      *((int*)0xfe393418) = 0XDEADBEEF;

    if((irq_source & irq_mask) !=  irq_mask)
    {
       spin_unlock_irq(&irq_lock);
       return IRQ_NONE;
    }
    if(!elite_neon_is_busy())
    {
       elite_turn_off_neon_monitor_timer(cpuid);
       //elite_turn_off_neon(cpuid);
       if(cpuid==0)
       {
          neonconter0++;
          *((int*)0xfe393400) = neonconter0;
       }
       else if(cpuid==1)
       {
          neonconter1++;
          *((int*)0xfe393404) = neonconter1;
       }
    }
    elite_ham_neon_irq_clr(cpuid);
    spin_unlock_irq(&irq_lock);
    return IRQ_HANDLED;
}


void elite_gic_affinity_to_all(unsigned int irq)
{
    void __iomem *gic_dist_base = (void __iomem *)IO_ADDRESS(ELITE_ARM_PERIF_GIC_DIST_BASE);
    void __iomem *irq_target_addr;
    int val,shift,mask;
    irq_target_addr = gic_dist_base + GIC_DIST_TARGET + (irq & (~0x3));
    val = readl(irq_target_addr);
    shift = (irq & 0x3)<<3;
    mask = 0xff << shift;

    if(irq==72)
        val = (val & ~mask) | (1 << shift); //target to cpu0
    else if(irq==78)
      val = (val & ~mask) | (2 << shift);  //target to cpu1

    //val = (val & ~mask) | (3 << shift);
   printk(KERN_DEBUG "%s:@@@@@@@@@@@@@@@@@@@@@irq_target_addr =%x val = %x\n", __func__,(unsigned int)irq_target_addr,val);
    writel(val,irq_target_addr);

}
void elite_init_neon_monitor(void)
{
        unsigned int irq = 0;
	int ret;

        //return;
        printk(KERN_DEBUG "%s \n", __func__);

        irq = ELITE_NEON0_MONITOR_CORES_IRQ;
        //set up irs for turning off NEON IRQF_NOAUTOEN
        ret = request_irq(irq, elite_neon_turnoff_irs,
		IRQF_DISABLED, "elite neon0 monitor ", NULL);
        if (ret) {
	    pr_err("%s: Failed to set irs for neon0 monitor\n", __func__);
            return ;
        }
        elite_gic_affinity_to_all(irq);

        irq = ELITE_NEON1_MONITOR_CORES_IRQ;
        //set up irs for turning off NEON IRQF_NOAUTOEN
        ret = request_irq(irq, elite_neon_turnoff_irs,
		IRQF_DISABLED, "elite neon1 monitor ", NULL);
        if (ret) {
	    pr_err("%s: Failed to set irs for neon1 monitor\n", __func__);
            return ;
        }
        elite_gic_affinity_to_all(irq);

        //set NEON monitor timer:
        elite_set_neon_monitor_timer();
        //elite_turn_off_neons();
        //dump_status();
        elite_neon_is_on[0] = true;
        elite_turn_on_neon_monitor_timer(0);
        elite_neon_is_on[0] = true;
        elite_turn_on_neon_monitor_timer(1);
}

void elite_neon_test_func(struct work_struct *work)
{

    void __iomem *pmu_domain_busy_status_addr;
    //unsigned int reg_val;
    //unsigned int busy_status;   
    unsigned int i;
    //unsigned int old,new;
    int irq_count;
    mutex_lock(&elite_neon_lock);
    printk(KERN_DEBUG "Start %s \n", __func__);
    pmu_domain_busy_status_addr = (void __iomem *)ELITE_PMU_BASE + PMU_DOMAIN_BUSY_STATUS;
    //scratch[0]=0;//clear interrupt count
    irq_count = scratch[0];
    //elite_turn_on_neon_monitor_timer(0);
    //elite_turn_off_neon(0);
    //scratch[1]=0; //clear inturupt flag
    __asm__ volatile ("wfi"); //e3a00101 	mov	r0, #1073741824	; 0x40000000
    __asm__ volatile ("wfi"); //eee80a10 	vmsr	fpexc, r0
    for(i=0;i<0x1000;i++)     //e3a03a02 	mov	r3, #8192	; 0x2000
    {

         __asm__ volatile ("wfi");//f2200900 	vmla.i32	d0, d0, d0
         __asm__ volatile ("wfi");
         __asm__ volatile ("wfi");
         __asm__ volatile ("wfi");
         __asm__ volatile ("wfi");
         __asm__ volatile ("wfi");
         __asm__ volatile ("wfi");
         __asm__ volatile ("wfi");
         __asm__ volatile ("wfi");
         __asm__ volatile ("wfi");
         __asm__ volatile ("wfi");
         __asm__ volatile ("wfi");
         __asm__ volatile ("wfi");
         __asm__ volatile ("wfi");
         __asm__ volatile ("wfi");
         __asm__ volatile ("wfi");


/* neon instruction 
 2000018:	e3a00101 	mov	r0, #1073741824	; 0x40000000
 200001c:	eee80a10 	vmsr	fpexc, r0
 2000020:	f2200900 	vmla.i32	d0, d0, d0
 */
/*
         if(i<4)
         {
             old = elite_read_ham_neon_cnt(0);
             for(;1;)
             {
                 new = elite_read_ham_neon_cnt(0);
                 if(old>new)
                    break;
                 old=new;
             }
             if(scratch[1]==1)
             {
                scratch[2] = elite_read_ham_neon_cnt(0);
                scratch[3] = elite_read_ham_neon_cnt(0);
                elite_turn_on_neon_monitor_timer(0);
                scratch[4] = elite_read_ham_neon_cnt(0);
                scratch[5] = elite_read_ham_neon_cnt(0);
                scratch[1]=0;//clear interrupt flag
              
             }
        }
        else
        {
                scratch[6] = elite_read_ham_neon_cnt(0);
                scratch[7] = elite_read_ham_neon_cnt(0);
        }
*/

    } //end of neon loop
    if(irq_count != scratch[0])
    printk(KERN_DEBUG "interrupt happens :irq_count=%x,scratch[0]=%x  in %s ",irq_count,scratch[0],__func__); 
    else
    printk(KERN_DEBUG "no interrupt happens ,irq_count=%x,scratch[0]=%x  in %s ",irq_count,scratch[0],__func__); 
        //asm volatile ("vmla.i32 d0,d0,d0          \n\t"); //f2 20 09 00
        //e320f000 	nop	{0}

    printk(KERN_DEBUG "scratch[0] =%x scratch[1] =%x scratch[2] =%x  in %s ",scratch[0],scratch[1],scratch[2],__func__); 
    mutex_unlock(&elite_neon_lock);
}


