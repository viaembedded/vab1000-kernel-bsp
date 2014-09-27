/*
 * arch/arm/mach-elite/elite_clocks.c
 *
 * Copyright (C) 2012 S3 Graphics, Inc.
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
#include <linux/clkdev.h>
#include <linux/clk.h>
#include <linux/syscore_ops.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <mach/io.h>

#include "clock.h"
#include "dvfs_elite.h"

/* Base Address definitions */
#define ELITE_PMC_BASE			0xD8130000
#define ELITE_PMC_SIZE			SZ_64K

#define ELITE_PMU				0xD8390000

/* Register definition */
#define PMU_DVFS_CMDFIFO                    	0xc3f0
#define PMU_DVFS_TABLE_REG 			0xc240
#define DVFS_CPU_PLL_CTRL                       0xc340
#define DVFS_CPU_PLL_TEST_CTRL	                0xC344
#define DEBUG_INFO_CTRL                         0xE080
#define DVFS_CPU_PLL_STATUS	                0xC348
#define DVFS_CPU_PLL_INPUT_CLK_RATIO_SEL        0xC34C

#define PMC_PERIPH_UPDATE_STATUS	0x000
#define PMC_PLL_UPDATE_STATUS		0X004
#define PMC_PLLB_MULT			0x208
#define PMC_PLLB_MISC			0x20c
#define PMC_PLLC_MULT			0x210
#define PMC_PLLC_MISC			0x214
#define PMC_WMTPLL			0x220
#define PMC_CKSRC0			0x224
#define PMC_CKSRC1			0x228
#define PMC_CKSRC2			0x22c

#define CLK_ENABLE_REG0		0x250
#define CLK_ENABLE_REG1		0x254
#define CLK_ENABLE_NONE		0

#define LOW_PULSE_LONGER	0
#define HIGH_PULSE_LONGER	1

#define SECONDARY_PERI_UART	(1<<0)
#define SECONDARY_PERI_TIMER	(1<<1)
#define SECONDARY_PERI_SATA	(1<<2)

/* PMU command */
#define PMU_CMD_VOL_DVFS	0x12
#define PMU_CMD_CLK_DVFS	0x13
/* domain */
#define CPU_VOLTAGE_DOMAIN	0

#define DVFS_TABLE_SIZE		16
#define REFERENCE_CLK		27000
#define FREQ_TABLE_SIZE		(16 + 1)


/* FIXME: recommended safety delay after lock is detected */
#define PLL_POST_EVENT_DELAY             10

void __init elite_cpu_init_clocks(unsigned int idx);
extern int dvfs_rail_set_voltage(struct dvfs_rail *rail);
extern struct dvfs_rail elite_dvfs_voltage_vdd_cpu;
extern bool debug_dvfs;

/*
  * Used to lock clock enable registers. these registers are shared by most clocks
  */
static DEFINE_SPINLOCK(clk_en_lock);
/*
  * Used to lock Updating Clock Divisor Status Registers. these registers are shared by most clocks
  */
static DEFINE_SPINLOCK(clk_stat_lock);


static void __iomem *reg_pmc_base = (void *)IO_ADDRESS(ELITE_PMC_BASE);
static void __iomem *reg_pmu_base = (void *)IO_ADDRESS(ELITE_PMU);

static inline void pmc_writeb(u8 value, u32 reg)
{
	__raw_writeb(value, reg_pmc_base + reg);
}

static inline u8 pmc_readb(u32 reg)
{
	return __raw_readb(reg_pmc_base + reg);
}
	
static inline void pmc_writew(u16 value, u32 reg) 
{
	__raw_writew(value, reg_pmc_base + reg);
}

static inline u16 pmc_readw(u16 reg) 
{
	return __raw_readw(reg_pmc_base + reg);
}

static inline void pmc_writel(u32 value, u32 reg) 
{
	__raw_writel(value, reg_pmc_base + reg);
}

static inline u32 pmc_readl(u32 reg)
{
	return __raw_readl(reg_pmc_base + reg);
}

static inline void pmu_writel(u32 value, u32 reg)
{
	__raw_writel(value, reg_pmu_base + reg);
}

static inline u32 pmu_readl(u32 reg)
{
	return __raw_readl(reg_pmu_base + reg);
}

static int elite_pll_clk_wait_for_event(struct clk *c, u32 event_reg, u32 event_bit)
{
#if 0
	unsigned long flags;
	int i;

	spin_lock_irqsave(&clk_stat_lock, flags);
	if (event_reg == PMC_WMTPLL) {
		for (i = 0; i < 600/* c->pmcpll_lockdelay */; i++) {
			if (pmc_readl(event_reg) & (1<< event_bit)) {
				udelay(PLL_POST_EVENT_DELAY);
				spin_unlock_irqrestore(&clk_en_lock, flags);
				return 0;
			}
			udelay(2);              /* timeout = 2 * lock time */
		}
	} else {
		for (i = 0; i < 100/* c->pmcpll_lockdelay*/; i++) {
			if (!(pmc_readl(event_reg) & (1<< event_bit))) {
				udelay(PLL_POST_EVENT_DELAY);
				spin_unlock_irqrestore(&clk_en_lock, flags);
				return 0;
			}
			udelay(2);              /* timeout = 2 * lock time */
		}
	}
	spin_unlock_irqrestore(&clk_stat_lock, flags);
	pr_err("Timed out waiting for lock bit on pll %s", c->name);
	return -1;
#endif
	usleep_range(1000, 1500);
	return 0;
}

static void elite_ckgsrc_init(struct clk *c)
{
	union ckgsrc0 src0;
	union ckgsrc1 src1;
	union ckgsrc2 src2;

	src0.val = pmc_readl(c->reg1);
	src1.val = pmc_readl(c->reg2);
	src2.val = pmc_readl(c->reg3);

	pr_info("drvstr_p: %u, drvstr_n: %u, "
		"drvstr_sel: %u, cp: %u, "
		"skew: %u, vddio: %u, "
		"predrv: %u, sataclk_sel: %u, "
		"tmode: %u, test_tno: %u, "
		"xtal_swmode: %u\n", 
		src0.field.drvstr_p, src0.field.drvstr_n,
		src0.field.drvstr_sel, src0.field.cp,
		src0.field.skew, src0.field.vddio,
		src0.field.predrv, src0.field.sataclk_sel,
		src0.field.tmode, src0.field.test_tno,
		src0.field.xtal_swmode
	);
	pr_info("pu_io: %u, tno_io: %u, "
		"sscsprd: %u, sscma: %u, "
		"sscen: %u, sscsprd_b: %u, "
		"sscma_b: %u, sscen_b: %u, "
		"xtal_swmode: %u\n", 
		src1.field.pu_io, src1.field.tno_io,
		src1.field.sscsprd, src1.field.sscma,
		src1.field.sscen, src1.field.sscsprd_b,
		src1.field.sscma_b,src1.field.sscen_b,
		src1.field.xtal_swmode
	);
	pr_info("sm_pu: %u, sm_rst:%u, "
		"sm_freeze: %u, sm_nout: %u, "
		"sm_pout: %u\n", 
		src2.field.sm_pu, src2.field.sm_rst,
		src2.field.sm_freeze, src2.field.sm_nout, 
		src2.field.sm_pout
	);
}

static int elite_ckgsrc_enable(struct clk *c)
{
	pr_debug("%s on clock %s\n", __func__, c->name);
	return 0;
}

static void elite_ckgsrc_disable(struct clk *c)
{
	pr_debug("%s on clock %s\n", __func__, c->name);
	BUG();
}

static struct clk_ops elite_ckgsrc_clk_ops = {
	.init		= &elite_ckgsrc_init,
	.enable		= &elite_ckgsrc_enable,
	.disable	= &elite_ckgsrc_disable,
};

static void elite_cpupll_clk_init(struct clk *c)
{
	pr_debug("%s on clock %s\n", __func__, c->name);
}

static struct clk_ops elite_cpupll_clk_ops = {
	.init			= &elite_cpupll_clk_init,
};

union dvfs_table dvfstable[DVFS_TABLE_SIZE];
#define DVFS_SUPPORT_BY_PMP 0
#if !DVFS_SUPPORT_BY_PMP
#define CPUPLL_GOODTIME 0    //10     //in us
#define VOLTAGE_GOODTIME_PER25MV 25 // 200     //in us
int elite_cpu_set_rate(struct clk *c, unsigned int freq_idx);

#endif
static void elite_cpu_clk_init(struct clk *c)
{
	struct elite_cpufreq_frequency_table *elite_freq_table;
	unsigned long	root_frequency,frequency;
	unsigned int pmu_dvfs_table_addr = 0;
	unsigned int intN;
	int i;


	//initialize DVFS table:
	elite_freq_table = c->u.table_data->elite_freq_table;
	root_frequency = c->u.table_data->root_frequency;
	
	for(i=0; i<DVFS_TABLE_SIZE; i++) {
    	    dvfstable[i].val.low = 0; 
	    dvfstable[i].val.high = 0; 
            dvfstable[i].table.vol = elite_freq_table->voltage;
            dvfstable[i].table.perip_divider = elite_freq_table->perip_divider;
            dvfstable[i].table.freq_divider = elite_freq_table->freq_divider;
            if(elite_freq_table->freq_divider!=0)
            	frequency = root_frequency;
            else
			{
            	frequency = elite_freq_table->frequency;
				if(frequency == root_frequency)
					dvfstable[i].table.base = 1;     //need to program pll
				else 
					dvfstable[i].table.base = 0;
			}
            intN = frequency / (REFERENCE_CLK /1000);
            dvfstable[i].table.fint = intN/1000 - 2;
            dvfstable[i].table.fn = (intN - dvfstable[i].table.fint *1000)*1024/1000; 
            dvfstable[i].table.r = 0;
            pmu_writel(dvfstable[i].val.low, PMU_DVFS_TABLE_REG + pmu_dvfs_table_addr);
            pmu_dvfs_table_addr += 4 ;
            pmu_writel(dvfstable[i].val.high, PMU_DVFS_TABLE_REG + pmu_dvfs_table_addr);
            pmu_dvfs_table_addr += 4;
            elite_freq_table++;
        }
}

static int elite_cpu_clk_enable(struct clk *c)
{
	pr_debug("%s on clock %s\n", __func__, c->name);
	return 0;
}

static void elite_cpu_clk_disable(struct clk *c)
{
	pr_debug("%s on clock %s\n", __func__, c->name);
	BUG();
}

static int elite_cpu_clk_set_parent(struct clk *c, struct clk *p)
{
	const struct clk_ckgsrc_sel *sel = c->srcs;
	
	pr_debug("%s: %s %s\n", __func__, c->name, p->name);
	
	for (sel = c->srcs; sel->src != NULL; sel++) {
		if (sel->src == p) {
			clk_enable(p);
			elite_clk_reparent(c, p);
			
			return 0;
		}
	}

	return -EINVAL;
}

#ifdef CONFIG_CPU_FREQ
#if DVFS_SUPPORT_BY_PMP
int elite_cpu_set_voltage(struct clk *c, unsigned int voltage_idx)
{
	struct pmu_cmd cmd;
 
 	cmd.u.vol_dvfs.opcode = PMU_CMD_VOL_DVFS;
	cmd.u.vol_dvfs.voltage_domain = CPU_VOLTAGE_DOMAIN;
	cmd.u.vol_dvfs.vol_index = voltage_idx;
 	pmu_writel(cmd.u.val, PMU_DVFS_CMDFIFO);
	        		 
 	return 0;
}

int elite_cpu_set_rate(struct clk *c, unsigned int freq_idx)
{
	int ret=0;
	struct pmu_cmd  cmd;

	cmd.u.clk_dvfs.opcode = PMU_CMD_CLK_DVFS;
	cmd.u.clk_dvfs.voltage_domain = CPU_VOLTAGE_DOMAIN;
	cmd.u.clk_dvfs.freq_index = freq_idx;
  	pmu_writel(cmd.u.val, PMU_DVFS_CMDFIFO);
	
 	return ret;

}

static int elite_cpu_clk_set_rate(struct clk *c, unsigned long rate)
{
	int ret = -ENOSYS;
	unsigned int idx;
	struct elite_cpufreq_frequency_table *elite_freq_table;

	elite_freq_table = c->u.table_data->elite_freq_table;
	//search for voltage matching with rate
	for( idx = 0; idx < FREQ_TABLE_SIZE; idx++) {
		if(elite_freq_table[idx].frequency == rate)
			break;
	}    
        
	pr_debug("%s: rate=%lu, idx=%d, oldrate=%lu \n", 
		__func__, rate, idx, c->oldrate);
	
	if(idx == FREQ_TABLE_SIZE) {
		BUG();
		goto out;   //voltage table missing item
	}
	if (rate > c->oldrate) {
	    ret = elite_cpu_set_voltage(c,idx);
	    if (ret)
	       goto out;
	}
	
	ret = elite_cpu_set_rate(c, idx);
	if (ret)
	    goto out;
	
	if (rate < c->oldrate)
		ret = elite_cpu_set_voltage(c,idx);

	c->rate = c->oldrate = rate;
	c->idx = idx;
	c->voltage = elite_freq_table[idx].voltage;
	return 0;
out:
	return ret;

}
#else
int elite_cpu_set_voltage(struct clk *c, unsigned int voltage_idx)
{
    struct dvfs_rail *elite_rail;
    int ret = 0;

    elite_rail = &elite_dvfs_voltage_vdd_cpu;

    if(elite_rail->disabled)
       return -ENOSYS;
        if(debug_dvfs)
            return 0;
        elite_rail->updating_millivolts = dvfstable[voltage_idx].table.vol*1000;
    ret = dvfs_rail_set_voltage(elite_rail);
    return ret;
}

#define CPUPLLCMD_PARKING 0
#define CPUPLLCMD_CPUPLL 1
void elite_switch_pll(int switch_pll_cmd)
{
#if 0 //
    union cpu_pll_test_ctrl cpupll_test_ctrl;

    switch (switch_pll_cmd) {
    case CPUPLLCMD_PARKING:
	 //1.1:parking cpu PLL
	 cpupll_test_ctrl.val = pmu_readl(DVFS_CPU_PLL_TEST_CTRL);
         cpupll_test_ctrl.field.cpupll_mux_sel = CPU_PLL_SEL_PARKING;
	 pmu_writel(cpupll_test_ctrl.val,DVFS_CPU_PLL_TEST_CTRL);
         break;
    case CPUPLLCMD_CPUPLL:
         cpupll_test_ctrl.val = pmu_readl(DVFS_CPU_PLL_TEST_CTRL);
         cpupll_test_ctrl.field.cpupll_mux_sel = CPU_PLL_SEL_CPUPLL;
	 pmu_writel(cpupll_test_ctrl.val,DVFS_CPU_PLL_TEST_CTRL);
         break;
    default:
         BUG();
#else
    union debug_info_ctrl debuginfoctrl;

    switch (switch_pll_cmd) {
    case CPUPLLCMD_PARKING:
         debuginfoctrl.val = pmu_readl(DEBUG_INFO_CTRL);
         debuginfoctrl.field.cpu_clk_mux_sel = 0;
	 pmu_writel(debuginfoctrl.val,DEBUG_INFO_CTRL);
         break;
    case CPUPLLCMD_CPUPLL:
         debuginfoctrl.val = pmu_readl(DEBUG_INFO_CTRL);
         debuginfoctrl.field.cpu_clk_mux_sel = 1;
	 pmu_writel(debuginfoctrl.val,DEBUG_INFO_CTRL);
         break;
    default:
         BUG();
#endif
  }	
}
void elite_cpu_set_divider_for_parking(unsigned int idx)
{
	    union cpu_pll_input_clk_ratio_sel cpupll_input_clk_ratio_sel;
        union dvfs_table *dvfstable_next;
        dvfstable_next = &dvfstable[idx];
        cpupll_input_clk_ratio_sel.val = pmu_readl(DVFS_CPU_PLL_INPUT_CLK_RATIO_SEL);
        cpupll_input_clk_ratio_sel.field.peri_ratio_sel = dvfstable_next->table.perip_divider;
        pmu_writel(cpupll_input_clk_ratio_sel.val,DVFS_CPU_PLL_INPUT_CLK_RATIO_SEL);
}
bool elite_changepll_needed(struct clk *c, unsigned int idx)
{
        union dvfs_table *dvfstable_last;
        union dvfs_table *dvfstable_next;
        dvfstable_last = &dvfstable[c->idx];
        dvfstable_next = &dvfstable[idx];
        if((dvfstable_next->table.freq_divider != 0) &&
           (dvfstable_last->table.freq_divider != 0))
		   return false;
        if((dvfstable_next->table.freq_divider == 0) &&
           (dvfstable_last->table.freq_divider == 0))
		   return true;
		if((dvfstable_next->table.freq_divider == 0) &&
			dvfstable_next->table.base == 1)
			return  false;
		if((dvfstable_last->table.freq_divider == 0) &&
			dvfstable_last->table.base == 1)
			return  false;
		return true;
}
bool elite_parking_needed(struct clk *c, unsigned int idx)
{
        if(elite_changepll_needed(c,idx))
			return true;
		else 
			return true;      //force parking or randomly hang in 30 hours on elite1000
}
int elite_cpu_set_rate(struct clk *c, unsigned int idx)
{
	union cpu_pll_ctrl cpupll_ctrl;
	union cpu_pll_status    cpupll_status;
	union cpu_pll_input_clk_ratio_sel cpupll_input_clk_ratio_sel;
        union dvfs_table *dvfstable_last;
        union dvfs_table *dvfstable_next;

        dvfstable_last = &dvfstable[c->idx];
        dvfstable_next = &dvfstable[idx];
	
  
	if(!elite_changepll_needed(c,idx))
	{
            cpupll_input_clk_ratio_sel.val = pmu_readl(DVFS_CPU_PLL_INPUT_CLK_RATIO_SEL);
            cpupll_input_clk_ratio_sel.field.ratio_sel = dvfstable_next->table.freq_divider;
            pmu_writel(cpupll_input_clk_ratio_sel.val,DVFS_CPU_PLL_INPUT_CLK_RATIO_SEL);

	}else{
	//set pll,power down, manul enable.
        cpupll_ctrl.val = 0;
        cpupll_ctrl.field.fn = dvfstable_next->table.fn;
        cpupll_ctrl.field.fint = dvfstable_next->table.fint;
        cpupll_ctrl.field.r = dvfstable_next->table.r;
        cpupll_ctrl.field.pd = 1;
        cpupll_ctrl.field.menable = 1;
        pmu_writel(cpupll_ctrl.val,DVFS_CPU_PLL_CTRL);
        
        //verify:
        //cpupll_ctrl.val = 0;
        //cpupll_ctrl.val = pmu_readl(DVFS_CPU_PLL_CTRL);


	//set divider:
        cpupll_input_clk_ratio_sel.val = pmu_readl(DVFS_CPU_PLL_INPUT_CLK_RATIO_SEL);
        cpupll_input_clk_ratio_sel.field.ratio_sel = dvfstable_next->table.freq_divider;
        pmu_writel(cpupll_input_clk_ratio_sel.val,DVFS_CPU_PLL_INPUT_CLK_RATIO_SEL);

        //verify:
        //cpupll_input_clk_ratio_sel.val = 0;
        //cpupll_input_clk_ratio_sel.val = pmu_readl(DVFS_CPU_PLL_INPUT_CLK_RATIO_SEL);


	//wait for power down:
        //udelay(10);
        //set power up:
        cpupll_ctrl.field.pd = 0;
        pmu_writel(cpupll_ctrl.val,DVFS_CPU_PLL_CTRL);
        //wait for power up:
        udelay(10);             //wait 3 us at least or randomly hang,set to 10 for safety

        //enable pll
        cpupll_ctrl.field.load = 1;
        pmu_writel(cpupll_ctrl.val,DVFS_CPU_PLL_CTRL);


	    do {
	        cpupll_status.val = pmu_readl(DVFS_CPU_PLL_STATUS);
	    }while(cpupll_status.field.cpupll_locked==0);

	}
        return 0;
}

static int elite_cpu_clk_set_rate(struct clk *c, unsigned long rate)
{
        int ret = -ENOSYS;
        int deltaVoltage;
        unsigned long delaytime;
        unsigned int idx;
        struct elite_cpufreq_frequency_table *elite_freq_table;
        union dvfs_table *dvfstable_last;
        union dvfs_table *dvfstable_next;
        struct dvfs_rail *elite_rail;
        elite_rail = &elite_dvfs_voltage_vdd_cpu;
        if(elite_rail->disabled)
            return 0;

        elite_freq_table = c->u.table_data->elite_freq_table;
        //search for voltage matching with rate
        for( idx = 0; idx < FREQ_TABLE_SIZE; idx++) {
            if(elite_freq_table[idx].frequency == rate)
                break;
        }    
        
        pr_debug("%s: rate=%lu, idx=%d, oldrate=%lu \n", 
		__func__, rate, idx, c->oldrate);
	
        
		if(idx == FREQ_TABLE_SIZE) {
            BUG();
            goto out;   //voltage table missing item
        }
        if(c->rate==rate)
        {
            return 0;        //rate does not change
        }

        dvfstable_last = &dvfstable[c->idx];
        dvfstable_next = &dvfstable[idx];

        c->done = false;
		if(elite_parking_needed(c,idx))
		{ 
			elite_cpu_set_divider_for_parking(idx);
		}
        if ((rate > c->oldrate) && 
            (dvfstable_next->table.vol !=  c->voltage)) {
            //case rate goes up and voltage changes:
            ret = elite_cpu_set_voltage(c,idx);
	    if(ret){
                pr_debug("%s failed on calling elite_cpu_set_voltage\n", __func__);
                goto out;   //voltage failed
            }
            if( dvfstable_next->table.vol > c->voltage)
            deltaVoltage = dvfstable_next->table.vol - c->voltage;
            else
                deltaVoltage = c->voltage - dvfstable_next->table.vol;
            delaytime = VOLTAGE_GOODTIME_PER25MV * (deltaVoltage + 24)/25;
            udelay(delaytime);
	}
		if(elite_parking_needed(c,idx))
		{ 
            elite_switch_pll(CPUPLLCMD_PARKING);
        }
        ret = elite_cpu_set_rate(c, idx);
        if (ret)
            pr_debug("%s failed on calling elite_cpu_set_rate\n", __func__);
	
        udelay(CPUPLL_GOODTIME);

        if ((rate < c->oldrate) && 
            (dvfstable_next->table.vol !=  c->voltage)) {
            //case:rate goes down:
            ret = elite_cpu_set_voltage(c,idx);
	    if(ret){
                pr_debug("%s failed on calling elite_cpu_set_voltage\n", __func__);
                ret = 0;    //voltage failed but frequency change is done
                if(elite_parking_needed(c,idx))
                {
                   elite_switch_pll(CPUPLLCMD_CPUPLL);
                }
                c->rate = c->oldrate = rate;
                c->idx = idx;
                c->done = true;
		return 0;
                //goto out;   //voltage failed
            }
            if(c->voltage >= dvfstable_next->table.vol)
            deltaVoltage = c->voltage - dvfstable_next->table.vol;
            else
                deltaVoltage = dvfstable_next->table.vol - c->voltage;
            delaytime = VOLTAGE_GOODTIME_PER25MV * (deltaVoltage + 24)/25;
            udelay(delaytime);
	}

		if(elite_parking_needed(c,idx))
        {
            elite_switch_pll(CPUPLLCMD_CPUPLL);
        }
	c->rate = c->oldrate = rate;
	c->idx = idx;
	c->voltage = dvfstable_next->table.vol;
	c->done = true;
        //printk(KERN_DEBUG "%s: ,frequency scalling #%x  is done \n",__func__,c->nFreqScalling);
	return 0;
out:
	c->done = true;
	return ret;

}
#endif // DVFS_SUPPORT_BY_PMP
#endif //CONFIG_CPU_FREQ


static struct clk_ops elite_cpu_clk_ops = {
	.init		= &elite_cpu_clk_init,
	.enable		= &elite_cpu_clk_enable,
	.disable	= &elite_cpu_clk_disable,
	.set_parent	= &elite_cpu_clk_set_parent,
#ifdef CONFIG_CPU_FREQ
	.set_rate	= &elite_cpu_clk_set_rate,
#endif
};

static void get_div_from_register(struct clk *c)
{
	if (UINT_MAX == c->reg1)
		return;

	if (c->u.periph.max_div < 255)
		c->div = pmc_readb(c->reg1);
	else
		c->div = pmc_readw(c->reg1);
	if (!c->div) {
		c->div = c->u.periph.max_div;
	}

	if (!strncmp(c->name, "sdmmc", 5))
		c->div *= 3;
}

static void elite_peri_clk_init(struct clk *c)
{
	unsigned long flags;

	spin_lock_irqsave(&clk_en_lock, flags);
	get_div_from_register(c);
	spin_unlock_irqrestore(&clk_en_lock, flags);
}

static int elite_peri_clk_enable(struct clk *c)
{
	unsigned long flags;

	spin_lock_irqsave(&clk_en_lock, flags);

	if (UINT_MAX == c->reg2) {
		spin_unlock_irqrestore(&clk_en_lock, flags);
		return -ENOTSUPP;
	}
	
	pmc_writel(pmc_readl(c->reg2) | (1 << c->u.periph.bit_en), 
		c->reg2);	

	spin_unlock_irqrestore(&clk_en_lock, flags);

	return 0;
}


static void elite_peri_clk_disable(struct clk *c)
{
	struct clk *child;
	unsigned long flags;

	spin_lock_irqsave(&clk_en_lock, flags);
	if (UINT_MAX == c->reg2) {
		spin_unlock_irqrestore(&clk_en_lock, flags);
		return;
	}

	/* Can't disable a clock if it has a child who already has already been enabled */
	if(!strcmp(c->name, "uhdc"))
		list_for_each_entry(child, &c->child_list, u.periph.node)
			if (child->u.periph.enabled) {
				spin_unlock_irqrestore(&clk_en_lock, flags);
				return;
			}

	pmc_writel(pmc_readl(c->reg2) & (~(1 << c->u.periph.bit_en)), 
		c->reg2);

	spin_unlock_irqrestore(&clk_en_lock, flags);
}


static int elite_peri_clk_set_rate(struct clk *c, unsigned long rate)
{
	unsigned long parent_rate = clk_get_rate(c->parent);
	u32 val = 0;

	if ((UINT_MAX == c->reg1)
	  || (UINT_MAX == c->stat_shift)
	  || (rate > c->max_rate))
		return -EINVAL;

	if (!strncmp(c->name, "sdmmc", 5))
		parent_rate /= 3;

	parent_rate /= rate;
	if ((parent_rate > 0) && (parent_rate < c->u.periph.max_div)) {
		val = parent_rate;
		if (!strncmp(c->name, "sdmmc", 5))
			c->div = parent_rate*3;
		else
			c->div = val;
	} else if (c->u.periph.max_div == parent_rate) {
		val = 0;
		c->div = c->u.periph.max_div;
		if (!strncmp(c->name, "sdmmc", 5))
			c->div *= 3;
	}
	if (c->u.periph.max_div < 255)
		pmc_writeb(val, c->reg1);
	else
		pmc_writew(val, c->reg1);
	
	if((val % 2) && (val > 1)) {
		if (likely(val < (u8)-1))
			pmc_writeb(HIGH_PULSE_LONGER, c->reg1 + 1);
		else
			pmc_writew(HIGH_PULSE_LONGER, c->reg1 + 2);
	}
	
	/* wait for update completed */
	if(elite_pll_clk_wait_for_event(c, PMC_PERIPH_UPDATE_STATUS, c->stat_shift))
		return -EAGAIN;
	
	get_div_from_register(c);

	return 0;
}

static unsigned long elite_peri_clk_round_rate(struct clk *c,
	unsigned long rate)
{
	int mod;
	unsigned long dividend = clk_get_rate(c->parent);
	pr_debug("%s: %s %lu\n", __func__, c->name, rate);

	mod = dividend % rate;
	if (!mod)
		return rate;
	
	return rate - mod;
}

static struct clk_ops elite_peri_clk_ops = {
	.init		= &elite_peri_clk_init,
	.enable		= &elite_peri_clk_enable,
	.disable	= &elite_peri_clk_disable,
	.set_rate	= &elite_peri_clk_set_rate,
	.round_rate	= &elite_peri_clk_round_rate,
};


static void elite_secondary_peri_clk_init(struct clk *c)
{
	c->parent = elite_get_clock_by_name("uhdc");

	BUG_ON(!c->parent);
	
	c->u.periph.enabled = false;
	
	list_add_tail(&c->u.periph.node, &c->parent->child_list);
	
	if(SECONDARY_PERI_UART == c->secondary_peri)
		c->div = 2;
	else if(SECONDARY_PERI_TIMER == c->secondary_peri)
		c->div = 4;
	else if(SECONDARY_PERI_SATA == c->secondary_peri)
		c->div = 1;
}

static int elite_secondary_peri_clk_enable(struct clk *c)
{
	unsigned long flags;

	spin_lock_irqsave(&clk_en_lock, flags);
	if (!c->u.periph.bit_en) {
		spin_unlock_irqrestore(&clk_en_lock, flags);
		return -ENOTSUPP;
	}
	
	pmc_writel(pmc_readl(c->reg1) | c->u.periph.bit_en << 1, 
		c->reg1);
	c->u.periph.enabled = true;
	spin_unlock_irqrestore(&clk_en_lock, flags);

	return 0;
}

static void elite_secondary_peri_clk_disable(struct clk *c)
{
	unsigned long flags;

	spin_lock_irqsave(&clk_en_lock, flags);
	if (UINT_MAX == c->reg1) {
		spin_unlock_irqrestore(&clk_en_lock, flags);
		return;
	}
	
	pmc_writel(pmc_readl(c->reg1) & (~c->u.periph.bit_en << 1), 
		c->reg1);
	c->u.periph.enabled = false;
	spin_unlock_irqrestore(&clk_en_lock, flags);
}

static int elite_secondary_peri_clk_set_parent(struct clk *c, struct clk *p)
{
	struct clk *parent  = elite_get_clock_by_name("uhdc");
	
	/* Parent is fixed to UHDC */
	if (p != parent)
		return -EINVAL;
	
	c->parent = parent;

	return 0;
}

static int elite_secondary_peri_clk_set_rate(struct clk *c, unsigned long rate)
{
	/* DIV is constant */
	return -EINVAL;
}

static unsigned long elite_secondary_peri_clk_round_rate(struct clk *c,
	unsigned long rate)
{
	unsigned long parent_rate = clk_get_rate(c->parent);
	unsigned long ret = 0;

	if(SECONDARY_PERI_UART == c->secondary_peri)
		ret = parent_rate/2;
	else if(SECONDARY_PERI_TIMER == c->secondary_peri)
		ret = parent_rate/4;
	else if(SECONDARY_PERI_SATA == c->secondary_peri)
		ret = parent_rate;

	return ret;
}


static struct clk_ops elite_secondary_peri_clk_ops = {
	.init		= &elite_secondary_peri_clk_init,
	.enable		= &elite_secondary_peri_clk_enable,
	.disable	= &elite_secondary_peri_clk_disable,
	.set_parent	= &elite_secondary_peri_clk_set_parent,
	.set_rate	= &elite_secondary_peri_clk_set_rate,
	.round_rate	= &elite_secondary_peri_clk_round_rate,
};

static void elite_i2s_clk_init(struct clk *c)
{
	c->parent = elite_get_clock_by_name("pmcpll_c");
}

static int elite_i2s_clk_set_parent(struct clk *c, struct clk *p)
{
	struct clk *parent  = elite_get_clock_by_name("pmcpll_c");
	
	/* Parent is fixed to PLL C  */
	if (p != parent)
		return -EINVAL;
	
	c->parent = parent;

	return 0;
}


static struct clk_ops elite_i2s_clk_ops = {
	.init		= &elite_i2s_clk_init,
	.enable		= &elite_peri_clk_enable, //share the same routine with periphera
	.disable	= &elite_peri_clk_disable, //share the same routine with periphera
	.set_parent	= &elite_i2s_clk_set_parent,
	.set_rate	= &elite_peri_clk_set_rate, //share the same routine with periphera
	.round_rate	= &elite_peri_clk_round_rate, //share the same routine with periphera
};

static int elite_pcie_clk_enable(struct clk *c)
{
	unsigned long flags;

	spin_lock_irqsave(&clk_en_lock, flags);
	if (UINT_MAX == c->reg1) {
		spin_unlock_irqrestore(&clk_en_lock, flags);
		return -ENOTSUPP;
	}
	
	pmc_writel(pmc_readl(c->reg1) | (1 << c->u.periph.bit_en), 
		c->reg1);	

	spin_unlock_irqrestore(&clk_en_lock, flags);

	return 0;
}

static void elite_pcie_clk_disable(struct clk *c)
{
	unsigned long flags;

	spin_lock_irqsave(&clk_en_lock, flags);
	if (UINT_MAX == c->reg1) {
		spin_unlock_irqrestore(&clk_en_lock, flags);
		return;
	}
	
	pmc_writel(pmc_readl(c->reg1) & (~(1 << c->u.periph.bit_en)), 
		c->reg1);

	spin_unlock_irqrestore(&clk_en_lock, flags);
}

/* PLLPCIEA and PLLPCIEB output 100MHz 66MHz 33MHz to PCIe */
static struct clk_ops elite_pcie_clk_ops = {
	.enable		= &elite_pcie_clk_enable,
	.disable	= &elite_pcie_clk_disable,
};

static void elite_pmcpll_init(struct clk *c)
{
	struct pmc_pll *pll = c->u.pmcpll;
	union pll_reg_mult mult; 
	union pll_reg_misc misc;
	
	mult.mult = pmc_readl(c->reg1);
	/* detect PLL current clock rate */
	for (; pll->freq != 0; pll++) {
		if (mult.mult == pll->u_mult.mult) {
			c->rate = pll->freq;
			break;
		}
	}

	if (!pll->freq) {
		pr_err("Invalid default freq, r: %d, int: %d, fn: %d\n",
			mult.s_mult.r, mult.s_mult.int1, mult.s_mult.fn);
		BUG();
	}

	c->rate = pll->freq;

	pr_info ("%s: detected PLL frequency: %lu Hz\n", c->name, c->rate);

	misc.misc = pmc_readl(c->reg2);
	pll->u_mult = mult;
	pll->u_misc = misc; 
	pr_info("sscg_flag1: %d, sscg_sel: %d, "
		"sscg_flag1: %d, sscg_tune: %d, "
		"lfp: %d, cp: %d, t: %d, tst: %d, "
		"order: %d, testb: %d\n",
		misc.s_misc.sscg_flag1, 
		misc.s_misc.sscg_sel,
		misc.s_misc.sscg_flag1,
		misc.s_misc.sscg_tune,
		misc.s_misc.lfp, misc.s_misc.cp,
		misc.s_misc.t, misc.s_misc.tst,
		misc.s_misc.order, misc.s_misc.testb
	);
}

static int elite_pmcpll_enable(struct clk *c)
{
	pr_debug("%s on clock %s, enabled already\n", __func__, c->name);
	return 0;
}

static void elite_pmcpll_disable(struct clk *c)
{
	pr_debug("%s on clock %s, can't be disabled\n", __func__, c->name);
	BUG();
}

static int elite_pmcpll_set_parent(struct clk *c, struct clk *p)
{
	pr_debug("pll has a static parent\n");
	return -EINVAL;
}

static int elite_pmcpll_clk_set_rate(struct clk *c, 
	unsigned long rate)
{
	struct pmc_pll *pll = c->u.pmcpll;
	union pll_reg_mult mult; 
	union pll_reg_misc misc;

	//mult.mult = pmc_readl(c->reg1);
	for (; pll != NULL; pll++) {
		if (rate == pll->freq)
			break;
	}

	if (!pll) {
		pr_err("Invalid frequency: %lu\n", rate);
		return -EINVAL;
	}
	mult.mult = pll->u_mult.mult;
	misc.misc = pll->u_misc.misc;

	/* disable PLL */
	misc.s_misc.pd = 1;
	pmc_writel(misc.misc, c->reg2);

	/* set the expected value of INT/FN/R */
	pmc_writel(mult.mult, c->reg1);
	/* wait for 1ms as per the spec */
	usleep_range(1000, 1500);

	/* enable PLL */
	misc.s_misc.pd = 0;
	pmc_writel(misc.misc, c->reg2);

	/* wait for update completed */
	if(elite_pll_clk_wait_for_event(c, PMC_PLL_UPDATE_STATUS, c->stat_shift))
		return -EAGAIN;
	if(elite_pll_clk_wait_for_event(c, PMC_WMTPLL, c->lock_shift))
		return -EAGAIN;

	/* wait for 60ms to make sure that PLL has already been steady */
	msleep(60);
    c->rate = rate;
	return 0;
}


static struct clk_ops elite_pmcpll_clk_ops = {
	.init			= &elite_pmcpll_init,
	.enable			= &elite_pmcpll_enable,
	.disable		= &elite_pmcpll_disable,
	.set_parent		= &elite_pmcpll_set_parent,
	.set_rate		= &elite_pmcpll_clk_set_rate,
};


/* Clock definitions */
static struct clk elite_ckgsrc_24m = {
	.name = "ckgsrc_24m",
	.rate = 24000000,
	.reg1 = PMC_CKSRC0, //CKGSRC control register 0
	.reg2 = PMC_CKSRC1, //CKGSRC control register 1
	.reg3 = PMC_CKSRC2, //CKGSRC control register 2
	.ops  = &elite_ckgsrc_clk_ops,
};

#if 0
static struct clk_pll_div_setting elite_pll_b_freq_table[] = {
	{24000000, 20000000, 24, 683, 5},
	{24000000, 20000000, 51, 341, 6},
	{24000000, 1000000000, 39, 683, 0},
	{24000000, 1100000000, 43, 853, 0},
	{24000000, 1500000000, 60, 512, 0},
	{0, 0, 0, 0, 0}
};
#endif

/*
  * CPUs frequency
  */
/* divider table:
 HW value       ratio
 0               1:1
 1:              7:8
 2               3:4
 3               2:3
 4               1:2
 5               1:3
 6               1:4
 7               1:6
 8               1:10
 9               1:20
*/
static struct elite_cpufreq_frequency_table elite_freq_table_1p0GHz[] = {
//        idx  freq      vol   div    p_div
	{  0,  50000,    725,   9,     9 },
	{  1, 100000,    725,   8,     9 },
	{  2, 166000,    750,   7,     8 },
	{  3, 250000,    775,   6,     7 },
	{  4, 333000,    800,   5,     6 },
	{  5, 500000,    825,   4,     5 },
	{  6, 666000,    900,   3,     4 },
	{  7, 750000,    900,   2,     4 },
	{  8, 875000,    900,   1,     4 },
	{  9, 1000000,   900,   0,     0 },
	{ 10, 1000000,   900,   0,     0 },
	{ 11, 1000000,   900,   0,     0 },
	{ 12, 1000000,   900,   0,     0 },
	{ 13, 1000000,   900,   0,     0 },
	{ 14, 1000000,   900,   0,     0 },
	{ 15, 1000000,   900,   0,     0 },
	/*
	{  0, 50000,     900,   9,     9 },
	{  1, 100000,    900,   8,     8 },
	{  2, 166000,    900,   7,     7 },
	{  3, 250000,    900,   6,     6 },
	{  4, 333000,    900,   5,     5 },
	{  5, 500000,    900,   4,     4 },
	{  6, 666000,   1000,   3,     3 },
	{  7, 750000,   1000,   2,     2 },
	{  8, 875000,   1000,   1,     1 },
	{  9, 1000000,  1000,   0,     0 },
	{ 10, 1000000,  1000,   0,     0 },
	{ 11, 1000000,  1000,   0,     0 },
	{ 12, 1000000,  1000,   0,     0 },
	{ 13, 1000000,  1000,   0,     0 },
	{ 14, 1000000,  1000,   0,     0 },
	{ 15, 1000000,  1000,   0,     0 },
	*/
};

static struct elite_cpufreq_frequency_table elite_freq_table_1p2GHz[] = {
	{  0, 60000,     800,  9,     9 },
	{  1, 120000,    800,  8,     8 },
	{  2, 200000,    800,  7,     7 },
	{  3, 300000,    800,  6,     6 },
	{  4, 400000,    800,  5,     5 },
	{  5, 600000,    800,  4,     4 },
	{  6, 800000,   1000,  3,     3 },
	{  7, 900000,   1100,  2,     2 },
	{  8, 1050000,  1200,  1,     1 },
	{  9, 1200000,  1200,  0,     0 },
	{ 10, 1300000,  1200,  0,     0 },
	{ 11, 1400000,  1200,  0,     0 },
	{ 12, 1500000,  1200,  0,     0 },
	{ 13, 1600000,  1200,  0,     0 },
	{ 14, 1600000,  1200,  0,     0 },
	{ 15, 1600000,  1200,  0,     0 },
};

static struct elite_cpufreq_table_data cpufreq_tables[] = {
	{ elite_freq_table_1p0GHz, 1000000,2, 6 },
	{ elite_freq_table_1p2GHz, 1200000,2, 7 },
};

static struct clk elite_cpupll = {
	.name	= "cpupll",
	.rate	= 24000000,
	.ops	= &elite_cpupll_clk_ops,
	.parent	= &elite_ckgsrc_24m,
};

#define PMC_PLL(_freq, _int, _fn, _r) \
	{                                 \
		.freq = _freq,          \
		.u_mult.s_mult = {      \
			.r = _r,                \
			.int1 = _int,           \
			.fn = _fn,            \
		},                       \
		.u_misc.s_misc = {   \
			.sscg_flag1 = 0,     \
			.sscg_sel = 0,        \
			.sscg_tune = 0,      \
			.lfp = 0,                 \
			.cp = 2,                 \
			.t = 3,                   \
			.tst = 0,                \
			.order = 0,             \
			.testb = 0,             \
		},                     \
	}


/* peripheral PLL */
static struct pmc_pll pll_b[] = {
	PMC_PLL(1200000000UL, 42, 456, 0),
	PMC_PLL(0, 0, 0, 0),
};

/* audio (I2S) PLL */
static struct pmc_pll pll_c[] = {
	PMC_PLL(16384000UL, 17, 428, 5),
	PMC_PLL(24576000UL, 27, 130, 5),
	PMC_PLL(22579200UL, 24, 779, 5),
	PMC_PLL(33868800UL, 38, 144, 5),
	PMC_PLL(36864000UL, 41, 707, 5),
	PMC_PLL(27324000UL, 30, 786, 5),
	PMC_PLL(0, 0, 0, 0),
};

static struct clk elite_pmcpll_b = {
	.name	= "pmcpll_b",
	.ops	= &elite_pmcpll_clk_ops,
	.parent	= NULL,
	.reg1	= PMC_PLLB_MULT, // PLL_B Multiplier and Range Values Register
	.reg2	= PMC_PLLB_MISC,  // PLL_B misc signal control Register
	.stat_shift = 1, //Updating PLLB value status bit
	.lock_shift = 17, //WMT pllb_lock bit
	.u = {
		.pmcpll = pll_b,
	},
};


static struct clk_ckgsrc_sel elite_clk_m_sel[] = {
	{ .src = &elite_cpupll, .value = 0},
	{ .src = &elite_pmcpll_b,  .value = 1},
	{ NULL , 0},
};

static struct clk elite_cpu_clk = {
	.name      = "cpu",
	.srcs	   = elite_clk_m_sel,
	.ops       = &elite_cpu_clk_ops,
	.parent    = &elite_cpupll,
	.u = {
                .table_data = &cpufreq_tables[0],
	},
};

static struct clk elite_pmcpll_c = {
	.name	= "pmcpll_c", // for I2S
    .max_rate = 36864000, 
	.ops	= &elite_pmcpll_clk_ops,
	.parent	= NULL,
	.reg1	= PMC_PLLC_MULT, // PLL_C Multiplier and Range Values Register
	.reg2	= PMC_PLLC_MISC,  // PLL_C misc signal control Register
	.stat_shift = 2, //Updating PLLC value status bit
	.lock_shift = 18, //WMT pllc_lock bit
	.u = {
		.pmcpll = pll_c,
	},
};

#define PERI_CLK(_name, _dev, _max_rate,  _reg_div, _reg_en, _stat_shift, _bit_en, _max_div) \
	{						\
		.name      = _name,			\
		.lookup    = {				\
			.dev_id    = _dev,		\
			.con_id	   = _name,		\
		},					\
		.max_rate = _max_rate,			\
		.ops       = &elite_peri_clk_ops,	\
		.parent	= &elite_pmcpll_b,	\
		.reg1      = _reg_div,		\
		.reg2	= _reg_en,			\
		.lock_shift = _stat_shift,	\
		.u.periph = {		\
			.bit_en =  _bit_en,	\
			.max_div = _max_div,	\
		}				\
	}

static struct clk elite_peri_clks[] = {
	PERI_CLK("gfx", "elite-gfx.0", 1000000000, UINT_MAX, CLK_ENABLE_REG0, UINT_MAX, 31, 1),
	PERI_CLK("gpio", "elite-gpio.0", 240000000, UINT_MAX, CLK_ENABLE_REG0, UINT_MAX, 11, 1),
	PERI_CLK("cir", "elite-cir.0", 240000000, UINT_MAX, CLK_ENABLE_REG0, UINT_MAX, 17, 1),
	PERI_CLK("kpad", "elite-kpad.0", 240000000, UINT_MAX, CLK_ENABLE_REG0, UINT_MAX, 9, 1),
	PERI_CLK("tzpc", "elite-tzpc.0", 240000000, UINT_MAX, CLK_ENABLE_REG0, UINT_MAX, 8, 1),
	PERI_CLK("rtc", "elite-rtc.0", 240000000, UINT_MAX, CLK_ENABLE_REG0, UINT_MAX, 7, 1),
	PERI_CLK("pdma", "elite-pdma.0", 240000000, UINT_MAX, CLK_ENABLE_REG1, UINT_MAX, 9, 1),
	PERI_CLK("dma", "elite-dma.0", 240000000, UINT_MAX, CLK_ENABLE_REG1, UINT_MAX, 5, 1),
	PERI_CLK("pxe_bist", "pxe_bist", 240000000, 0x300, CLK_ENABLE_NONE, 7, UINT_MAX, 32),
	PERI_CLK("ahb", "ahb", 240000000, 0x304, CLK_ENABLE_NONE, 8, UINT_MAX, 8),
	PERI_CLK("uhdc", "elite-ehci.0", 48000000, 0x308, CLK_ENABLE_REG1, 31, 7, 32),
	PERI_CLK("sf", "elite-sf.0", 100000000, 0x314, CLK_ENABLE_REG1, 19, 23, 32), //SPI flash
	PERI_CLK("dsp", "elite-xtensa.0", 600000000, 0x31c, CLK_ENABLE_REG1, 12, 20, 32), //DSP
	PERI_CLK("pcm", "elite-pcm.0", 120000000, 0x320, CLK_ENABLE_REG0, 21, 20, 32),
	PERI_CLK("sdmmc", "elite-mci.0", 200000000, 0x328, CLK_ENABLE_REG1, 22, 18, 2048),
	PERI_CLK("nand", "elite-nand.0", 100000000, 0x330, CLK_ENABLE_REG1, 25, 16, 32), //nand flash
	PERI_CLK("ngc", "elite-ngc.0", 240000000, 0x334, CLK_ENABLE_REG1, 23, 17, 32),//nor
	PERI_CLK("sdmmc", "elite-mci.1", 100000000, 0x338, CLK_ENABLE_REG0, 20, 16, 2048),
	PERI_CLK("spi", "elite-spi.0", 100000000, 0x33c, CLK_ENABLE_REG0, 26, 12, 64),
	PERI_CLK("spi", "elite-spi.1", 100000000, 0x340, CLK_ENABLE_REG0, 27, 13, 64),
	PERI_CLK("sdmmc", "elite-mci.2", 50000000, 0x344, CLK_ENABLE_REG0, 18, 25, 2048),
	PERI_CLK("pwm", "elite-pwm.0", 50000000, 0x348, CLK_ENABLE_REG0, 14, 20, 32),
	PERI_CLK("rndnm", "elite-rndnm.1", 200000000, 0x354, CLK_ENABLE_REG0, 17, 18, 64),//random number
	PERI_CLK("se", "elite-se.0", 300000000, 0x35c, CLK_ENABLE_REG1, 9, 24, 32),//security engine
	PERI_CLK("paxi", "paxi", 600000000, 0x368, CLK_ENABLE_REG0, 28, UINT_MAX, 32),
	PERI_CLK("i2c", "elite-i2c.0", 40000000, 0x36c, CLK_ENABLE_REG0, 5, 5, 64),
	PERI_CLK("i2c", "elite-i2c.1", 40000000, 0x370, CLK_ENABLE_REG0, 6, 0, 64),
};

#define SECONDARY_PERI_CLK(_name, _dev, _sec_peri, _reg_en, _bit_en, _max_div) \
	{						\
		.name      = _name,			\
		.lookup    = {				\
			.dev_id    = _dev,		\
			.con_id	   = _name,		\
		},					\
		.ops       = &elite_secondary_peri_clk_ops,	\
		.secondary_peri      = _sec_peri,		\
		.reg1	= _reg_en,			\
		.u.periph = {		\
			.bit_en =  _bit_en,	\
			.max_div = _max_div,	\
		}				\
	}

static struct clk elite_secondary_peri_clks[] = {
	SECONDARY_PERI_CLK("sata", "elite-sata.0", SECONDARY_PERI_SATA, CLK_ENABLE_REG1, 19, 1),
	SECONDARY_PERI_CLK("uart", "elite-uart.0", SECONDARY_PERI_UART, CLK_ENABLE_REG0, 1, 2),
	SECONDARY_PERI_CLK("uart", "elite-uart.1", SECONDARY_PERI_UART, CLK_ENABLE_REG0, 2, 2),
	SECONDARY_PERI_CLK("uart", "elite-uart.2", SECONDARY_PERI_UART, CLK_ENABLE_REG0, 3, 2),
	SECONDARY_PERI_CLK("uart", "elite-uart.3", SECONDARY_PERI_UART, CLK_ENABLE_REG0, 4, 2),
	SECONDARY_PERI_CLK("uart", "elite-uart.4", SECONDARY_PERI_UART, CLK_ENABLE_REG0, 14, 2),
};

static struct clk elite_i2s_clk = {
	.name	= "i2s",
	.lookup	= {	
		.dev_id    = "elite-i2s.0",
		.con_id	   = NULL,
	},
    .max_rate = 36864000, 
	.ops	= &elite_i2s_clk_ops,
	.reg1	= 0x324, 
	.reg2	= CLK_ENABLE_REG1,  
	.lock_shift = 24, 
	.u.periph = {
		.bit_en =  15,	
		.max_div = 64,
	}	
};

#define PCIE_CLK(_name, _dev, _reg_en, _bit_en) \
	{						\
		.name      = _name,			\
		.lookup    = {				\
			.dev_id    = _dev,		\
			.con_id	   = NULL,		\
		},					\
		.ops       = &elite_pcie_clk_ops,	\
		.reg1      = _reg_en,		\
		.u.periph = {		\
			.bit_en =  _bit_en,	\
		}				\
	}

static struct clk elite_pcie_clks[] = {
	PCIE_CLK("pcie0", NULL, CLK_ENABLE_REG0, 28),
	PCIE_CLK("pcie1", NULL, CLK_ENABLE_REG1, 8),
	PCIE_CLK("pcie2", NULL, CLK_ENABLE_REG1, 22),
};


static struct clk *elite_pclks[] = {
	&elite_cpupll,
	&elite_cpu_clk,
	&elite_pmcpll_b,
	&elite_pmcpll_c,
	&elite_i2s_clk,
};

static void elite_register_clock(struct clk *c)
{
	elite_clk_init(c);

	INIT_LIST_HEAD(&c->child_list);
	
	if (!c->lookup.dev_id && !c->lookup.con_id)
		c->lookup.con_id = c->name;
	c->lookup.clk = c;
	
	clkdev_add(&c->lookup);
}

#ifdef CONFIG_PM_SLEEP
static u32 elite_pmc_save[64];

void elite_pm_do_save(void)
{
    u32 *ptr = elite_pmc_save;

    *ptr++ = pmc_readl(PMC_PLLB_MULT);
    *ptr++ = pmc_readl(PMC_PLLB_MISC);
    *ptr++ = pmc_readl(PMC_PLLC_MULT);
    *ptr++ = pmc_readl(PMC_PLLC_MISC);
    *ptr++ = pmc_readl(PMC_WMTPLL);
    *ptr++ = pmc_readl(PMC_CKSRC0);
    *ptr++ = pmc_readl(PMC_CKSRC1);
    *ptr++ = pmc_readl(PMC_CKSRC2);
    *ptr++ = pmc_readl(CLK_ENABLE_REG0);
    *ptr++ = pmc_readl(CLK_ENABLE_REG1);
    *ptr++ = pmc_readl(0x300); //ARM clock
    *ptr++ = pmc_readl(0x304);
    *ptr++ = pmc_readl(0x308);
    *ptr++ = pmc_readl(0x314);
    *ptr++ = pmc_readl(0x31c);
    *ptr++ = pmc_readl(0x320);
    *ptr++ = pmc_readl(0x324);
    *ptr++ = pmc_readl(0x328);
    *ptr++ = pmc_readl(0x330);
    *ptr++ = pmc_readl(0x338);
    *ptr++ = pmc_readl(0x33c);
    *ptr++ = pmc_readl(0x340);
    *ptr++ = pmc_readl(0x344);
    *ptr++ = pmc_readl(0x348);
    *ptr++ = pmc_readl(0x354);
    *ptr++ = pmc_readl(0x35c);
    *ptr++ = pmc_readl(0x368);
    *ptr++ = pmc_readl(0x36c);
    *ptr++ = pmc_readl(0x370);
    *ptr++ = pmc_readl(0x400);
}

void elite_pm_do_restore_core(void)
{
    u32 *ptr = elite_pmc_save;

    pmc_writel(*ptr++, PMC_PLLB_MULT);
    pmc_writel(*ptr++, PMC_PLLB_MISC);
    pmc_writel(*ptr++, PMC_PLLC_MULT);
    pmc_writel(*ptr++, PMC_PLLC_MISC);
    pmc_writel(*ptr++, PMC_WMTPLL);
    pmc_writel(*ptr++, PMC_CKSRC0);
    pmc_writel(*ptr++, PMC_CKSRC1);
    pmc_writel(*ptr++, PMC_CKSRC2);
    pmc_writel(*ptr++, CLK_ENABLE_REG0);
    pmc_writel(*ptr++, CLK_ENABLE_REG1);
    pmc_writel(*ptr++, 0x300); // ARM clock
    pmc_writel(*ptr++, 0x304);
    pmc_writel(*ptr++, 0x308);
    pmc_writel(*ptr++, 0x314);
    pmc_writel(*ptr++, 0x31c);
    pmc_writel(*ptr++, 0x320);
    pmc_writel(*ptr++, 0x324);
    pmc_writel(*ptr++, 0x328);
    pmc_writel(*ptr++, 0x330);
    pmc_writel(*ptr++, 0x338);
    pmc_writel(*ptr++, 0x33c);
    pmc_writel(*ptr++, 0x340);
    pmc_writel(*ptr++, 0x344);
    pmc_writel(*ptr++, 0x348);
    pmc_writel(*ptr++, 0x354);
    pmc_writel(*ptr++, 0x35c);
    pmc_writel(*ptr++, 0x368);
    pmc_writel(*ptr++, 0x36c);
    pmc_writel(*ptr++, 0x370);
    pmc_writel(*ptr++, 0x400);
}

static int elite_clock_suspend(void)
{
	elite_pm_do_save();

	return 0;
}

static void elite_clock_resume(void)
{
	elite_pm_do_restore_core();
}

struct syscore_ops elite_clock_syscore_ops = {
	.suspend	= elite_clock_suspend,
	.resume		= elite_clock_resume,
};
#endif

#ifdef CONFIG_OF
static const struct of_device_id matches[] __initconst = {
	{ .compatible = "s3graphics,elite1000-pmc" },
	{ }
};
#endif


void __init elite_soc_init_clocks(void)
{
	int i;

#if 0
//#ifdef CONFIG_OF
	if (of_have_populated_dt()) {
		struct device_node *np;

		np = of_find_matching_node(NULL, matches);
		if (np) {
			np = of_find_compatible_node(NULL, NULL, "s3graphics,elite1000-pmc");
			reg_pmc_base = of_iomap(np, 0);
			BUG_ON(!reg_pmc_base);
		}
	}
#endif

	for (i = 0; i < ARRAY_SIZE(elite_pclks); i++)
		elite_register_clock(elite_pclks[i]);
	for (i = 0; i < ARRAY_SIZE(elite_peri_clks); i++)
		elite_register_clock(&elite_peri_clks[i]);
	for (i = 0; i < ARRAY_SIZE(elite_pcie_clks); i++)
		elite_register_clock(&elite_pcie_clks[i]);
	for (i = 0; i < ARRAY_SIZE(elite_secondary_peri_clks); i++)
		elite_register_clock(&elite_secondary_peri_clks[i]);
	
#ifdef CONFIG_PM_SLEEP
	register_syscore_ops(&elite_clock_syscore_ops);
#endif
}


void __init elite_cpu_init_clocks(unsigned int idx)
{
	union dvfs_table *dvfstable_cur;
	struct elite_cpufreq_frequency_table *elite_freq_table;

	//elite_register_clock(&elite_cpu_clk);

    dvfstable_cur = &dvfstable[idx];
	elite_freq_table = elite_cpu_clk.u.table_data->elite_freq_table;

	elite_cpu_clk.rate = elite_cpu_clk.oldrate = elite_freq_table[idx].frequency;
	elite_cpu_clk.idx = idx;
	elite_cpu_clk.voltage = dvfstable_cur->table.vol;
	elite_cpu_clk.done = true;
    elite_cpu_clk.nFreqScalling = 0;

}

