/*
 * arch/arm/mach-elite/clock.h
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

#ifndef __MACH_ELITE_CLOCK_H
#define __MACH_ELITE_CLOCK_H

#include <linux/clkdev.h>
#include <linux/list.h>
#include <linux/spinlock.h>

struct clk;

struct clk_ckgsrc_sel {
	struct clk *src;
	u32 value;
};

struct clk_pll_div_setting {
	unsigned long	refclk; //input rate
	unsigned long	pllclk; //output rate
	u16		int1;
	u16		fn;
	u8		r;
};

union wmt_pll {
	struct {
		u32 pll_gfxpll_tst:8; //refer to PLL spec
		u32 pll_clksel_mux:1; //no use
		u32 pll_clksel_96m_24m:1; //no use
		u32 pll_gfxpll_eclksel:2; //no use
		u32 pll_isel:2; //refer to PLL spec
		u32 pll_xclko_en:1; //no use
		u32 resv1:1;
		u32 plla_lock:1;
		u32 pllb_lock:1;
		u32 pllc_lock:1;
		u32 resv2:1;
		u32 clksrc_plla_ok:1;
		u32 clksrc_pllb_ok:1;
		u32 resv3:10;
	}field;
	u32 val;
};

/* PLL Multiplier and Range Values Register */
union pll_reg_mult {
	struct {
		u32 r:3;
		u32 resv1:5;
		u32 int1:7;
		u32 resv2:1;
		u32 fn:10;
		u32 resv3:6;
	} s_mult;
	u32 mult;
};

/* PLL misc signal control Register */
union pll_reg_misc {
	struct {
		u32 sscg_flag1:1;
		u32 sscg_sel:1;
		u32 sscg_flag:1;
		u32 sscg_tune:1;
		u32 lfp:2;
		u32 cp:3;
		u32 t:3;
		u32 tst:8;
		u32 order:1;
		u32 testb:1;
		u32 pd:1;
		u32 resv:10;
	} s_misc;
	u32 misc;
};

struct pmc_pll {
	unsigned long freq;
	union pll_reg_mult u_mult;
	union pll_reg_misc u_misc;
};

/* CKGSRC misc signal control Register 0 */
union ckgsrc0 {
	struct {
		u32 drvstr_p:4;
		u32 drvstr_n:4;
		u32 drvstr_sel:1;
		u32 cp:3;
		u32 skew:2;
		u32 vddio:3;
		u32 predrv:2;
		u32 sataclk_sel:1;
		u32 tmode:3;
		u32 test_tno:1;
		u32 resv1:1;
		u32 xtal_swmode:1;
		u32 resv2:6;
	}field;
	u32 val;
};

/* CKGSRC misc signal control Register 1 */
union ckgsrc1 {
	struct {
		u32 pu_io:6;
		u32 resv1:2;
		u32 tno_io:6; //2:0 are controlled by PCIE.
		u32 resv2:2;
		u32 sscsprd:1;
		u32 sscma:3;
		u32 sscen:1;
		u32 resv3:3;
		u32 sscsprd_b:4;
		u32 sscma_b:1;
		u32 sscen_b:3;
		u32 xtal_swmode:1;
		u32 resv4:3;
	}field;
	u32 val;
};

/* CKGSRC misc signal control Register 2 */
union ckgsrc2 {
	struct {
		u32 sm_pu:1;
		u32 sm_rst:1;
		u32 sm_freeze:1;
		u32 resv1:5;
		u32 sm_nout:4; /* FIXME: or sm_pout? */
		u32 sm_pout:4; /* FIXME: or sm_nout? */
		u32 resv2:16;
	}field;
	u32 val;
};

struct ckgsrc {
	union ckgsrc0 src0;
	union ckgsrc1 src1;
	union ckgsrc2 src2;
};

struct pmu_cmd {
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

/* CPU pll control register */
/*
union cpu_pll{
	struct {    
		u32 fn:10;
		u32 int1:7;
		u32 resv1:3;
		u32 r:3;
		u32 resv2:5;
		u32 load:1;
		u32 pd:1;
		u32 resv3:2;
	}field;  
	u32 val;
};
*/
union cpu_pll_ctrl{                       
   struct {                                // bit
        unsigned int fn:10;                // 0-9
        unsigned int fint:7;               // 10-16
        unsigned int reserved1:3;          // 17-19
        unsigned int r:3;                  // 20-22
        unsigned int reserved2:4;          // 23-26
	unsigned int menable:1;            // 27
        unsigned int load:1;               // 28
        unsigned int pd:1;                 // 29
        unsigned int reserved3:2;          // 30-31
   }field;  
   unsigned int val;
};



#define CPU_PLL_SEL_PARKING 0
#define CPU_PLL_SEL_CPUPLL  1
union cpu_pll_test_ctrl{                       
   struct {                                 // bit
	unsigned int cpupll_testb:1;        // 0
        unsigned int cpupll_order:1;        // 1
        unsigned int cpupll_tst:8;          // 2-9
        unsigned int cpupll_t:3;            // 10-12
	unsigned int cpupll_cp:3;           // 13-15
	unsigned int cpupll_1fp:3;          // 16-17
	unsigned int cpupll_mux_sel:1;      // 18   0:parking, 1:cpupll
	unsigned int cpuplls_isel:2;        // 19-20
	unsigned int cpuplls_tst:3;         // 21-28
	unsigned int reserved:3;            // 29-31
   }field;  
   unsigned int val;
};

union debug_info_ctrl{                       
   struct {                                 // bit
	unsigned int reserved0:18;          // 0-17
	unsigned int cpu_clk_mux_sel:1;     // 18 0:cpu pll,1:peri pll
	unsigned int reserved1:13;          // 19-31
   }field;  
   unsigned int val;
};

union cpu_pll_status{                       
   struct {                                 // bit
	unsigned int fn:10;                 // 0-9
	unsigned int fint:7;                // 10-16
	unsigned int reserved1:3;           // 17-19
	unsigned int r:3;                   // 20-22
	unsigned int reserved2:5;           // 23-27
	unsigned int load:1;                // 28
	unsigned int pd:1;                  // 29
	unsigned int reserved3:1;           // 30
	unsigned int cpupll_locked:1;       // 31
   }field;  
   unsigned int val;
};

union cpu_pll_input_clk_ratio_sel{                       
   struct {                                 // bit
	unsigned int ratio_sel:4;           // 0-3
	unsigned int peri_ratio_sel:3;      // 4-7
	unsigned int external_clk:1;        // 8
	unsigned int reserved:24;           // 9-31
   }field;  
   unsigned int val;
};

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

struct elite_cpufreq_frequency_table {
	unsigned int	index;     /* any */
	unsigned int	frequency; /* kHz - doesn't need to be in ascending
				    * order */
	unsigned long	voltage;
	unsigned int freq_divider;
	unsigned int perip_divider;
};

struct elite_cpufreq_table_data {
	struct elite_cpufreq_frequency_table *elite_freq_table; //interface with hardware
	unsigned long	        	root_frequency;//if divider!=0,then use this to generate PLL.
	int throttle_lowest_index;
	int throttle_highest_index;
};


struct clk_ops {
	void		(*init)(struct clk *);
	int		(*enable)(struct clk *);
	void		(*disable)(struct clk *);
	int		(*set_parent)(struct clk *, struct clk *);
	int		(*set_rate)(struct clk *, unsigned long);
	unsigned long	(*round_rate)(struct clk *, unsigned long);
};

struct clk {
	/* node for master clocks list */
	struct list_head	node;
	struct clk_lookup	lookup;

#ifdef CONFIG_DEBUG_FS
	struct dentry		*dent;
#endif
        //for cpu:
	unsigned long	 voltage;
	unsigned long	 oldrate; //meaningful in clk_set_rate_elite 
	int  idx;
        bool done; 
        int  nFreqScalling;

	struct clk_ops		*ops;
	unsigned long		rate;
	unsigned long		max_rate;
	unsigned long		min_rate;
	const char		*name;

	int			pmcpll_lockdelay;
	struct clk		*parent;
	struct list_head	child_list;
	unsigned int div;

	struct clk_ckgsrc_sel	*srcs;
	u32 reg1;
	u32 reg2;
	u32 reg3;
	u32 stat_shift;
	u32 lock_shift;

	u32 secondary_peri;
	
	union {
		struct {
			u32	max_div;
			u32	bit_en;
			bool	enabled;
			struct list_head node;
		} periph;
		struct {
			u32 sel;
			u32 reg_mask;
		} mux;
		struct ckgsrc src;
		struct pmc_pll *pmcpll;
		struct elite_cpufreq_table_data *table_data;
	} u;

	spinlock_t spinlock;
};

struct clk *elite_get_clock_by_name(const char *name);
void elite_clk_init(struct clk *clk);
int elite_clk_reparent(struct clk *c, struct clk *parent);

void __init elite_soc_init_clocks(void);

#endif //__MACH_ELITE_CLOCK_H
