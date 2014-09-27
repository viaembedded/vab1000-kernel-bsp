/*
 * arch/arm/mach-elite/cpuidle.c
 *
 * CPU idle driver for Elite 800 cpus
 *
 * Copyright (c) 2011, S3 Graphics INC.
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

#include <linux/module.h>
#ifdef CONFIG_CPU_IDLE
#include <linux/kernel.h>
#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/suspend.h>
#include <linux/tick.h>

#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>
#include <asm/localtimer.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
//#include <mach/legacy_irq.h>
//#include <mach/suspend.h>

//#include "power.h"
#include "cpu-elite.h"
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <mach/io.h>
//for neon test:
//#include <linux/delay.h>
//static DEFINE_MUTEX(elite_neon_lock);
#define NEON_DELAY		msecs_to_jiffies(2000)
static DEFINE_MUTEX(elite_pg_lock);


extern bool elite_neon_is_on[MAXCPUS];
extern void *elite_context_area;
unsigned int *mycmd = NULL;

static bool pg_disabled_by_suspend = false;

static int elite_pg_exit_latency;
int adjusted_latency;
unsigned long g_pg_state=0;
EXPORT_SYMBOL(g_pg_state);

DEFINE_PER_CPU(unsigned int,in_pg);
DEFINE_SPINLOCK(pg_dvfs_lock);
//define for debug by vfs:
unsigned int dbgcmd=0;
static struct {
	unsigned int gov_req_cg_count[MAXCPUS];
	unsigned int cpu_enter_cg_count[MAXCPUS];
	unsigned int gov_req_pg_count[MAXCPUS];
	unsigned int cpu_enter_pg_count[MAXCPUS];
	unsigned long long cg_time[MAXCPUS];
	unsigned long long pg_full_sleep_time[MAXCPUS];
	unsigned long long pg_partial_sleep_time[MAXCPUS];
	unsigned long long in_pg_time[MAXCPUS];
	unsigned int pg_full_sleep_count[MAXCPUS];
	unsigned int pg_partial_sleep_count[MAXCPUS];

	unsigned int cpu_ready_count[2];
	unsigned long long cpu_wants_lp2_time[2];
	unsigned long long in_lp2_time;
	unsigned int both_idle_count;
	unsigned int tear_down_count;
	unsigned int lp2_count;
	unsigned int lp2_completed_count;
	unsigned int lp2_count_bin[32];
	unsigned int lp2_completed_count_bin[32];
	unsigned int lp2_int_count[NR_IRQS];
	unsigned int last_lp2_int_count[NR_IRQS];
} idle_stats;


static int elite_idle_enter_cg(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int index);
static int elite_idle_enter_pg(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int index);

struct cpuidle_driver elite_idle = {
	.name = "elite_idle",
	.owner = THIS_MODULE,
	.state_count = 2,
	.states = {
		[0] = {
			.enter			= elite_idle_enter_cg,
			.exit_latency		= 10,
			.target_residency	= 10,
			.power_usage		= 600,
			.flags			= CPUIDLE_FLAG_TIME_VALID,
			.name			= "CG",
			.desc			= "CPU clock gating",
		},

		[1] = {
			.enter			= elite_idle_enter_pg,
			.exit_latency		= 200,
			.target_residency	= 500,
			.power_usage		= 0,
			.flags			= CPUIDLE_FLAG_TIME_VALID,
			.name			= "PG",
			.desc			= "CPU power gating",
		},

	},
};

//extern functions:
#ifdef CONFIG_PERF_EVENTS
extern void counter_output(const char *name, s64 count, int cpu);
#endif
extern void elite_get_cg_times(__u64 *times,enum cpu_id cpuid);
extern void elite_get_cg_cycles(__u64 *cycles,enum cpu_id cpuid);
extern void elite_get_pg_times(__u64 *times,enum cpu_id cpuid);
extern void elite_get_pg_cycles(__u64 *cycles,enum cpu_id cpuid);
extern void elite_corex_wfi_cfg(union corex_wfi_cfg,enum cpu_id cpuid);
extern void elite_wfe_cfg(bool on,enum cpu_id cpuid);
extern void elite_cpu_lp_cfg_corex(int val,enum cpu_id cpuid);
extern void elite_set_pg_timer(unsigned int us,int cpu);
extern unsigned int elite_power_gating(unsigned int us,struct cpuidle_device *dev);
extern void elite_turn_off_neon(int cpu);
extern void elite_pmu_enable(void);
extern unsigned int elite_create_pg_pgtable(void);
extern void elite_init_neon_monitor(void);
extern void elite_dump_dvfs_table_to_pmu(void);
extern void read_ham_counter(void);
extern void read_ham_irq_sts(void);
extern void elite_neon_test_func(struct work_struct *work);
extern bool elite_pg_is_on(void);

extern void elite_set_irq(void);
extern void elite_clear_irq(void);
extern void elite_enable_irq(void);
extern void elite_disable_irq(void);
extern int  set_irq_affinity(int irq, unsigned int cpu);
extern bool   is_pluging_unpluging;
extern volatile int   in_dvfs;
extern volatile int   cpu_dynamic_clk;
//volatile int   in_pg[2] = {0,0};
static DEFINE_PER_CPU(struct cpuidle_device , idle_devices);



void init_idle_stats(void)
{
	int i;
	printk("****** &idle_state  = %x ******\n",(unsigned int)&idle_stats);
	for(i=0;i<MAXCPUS;i++)
	{
		idle_stats.gov_req_cg_count[i] = 0;
		idle_stats.cpu_enter_cg_count[i] = 0;
		idle_stats.gov_req_pg_count[i] = 0;
		idle_stats.cpu_enter_pg_count[i] = 0;
		idle_stats.cg_time[i] = 0;
		idle_stats.pg_full_sleep_time[i] = 0;
		idle_stats.pg_partial_sleep_time[i] = 0;
		idle_stats.in_pg_time[i] = 0;
		idle_stats.pg_full_sleep_count[i] = 0;
		idle_stats.pg_partial_sleep_count[i] = 0;

	}


}

static inline void elite_flow_wfi(struct cpuidle_device *dev)
{

	stop_critical_timings();
	dsb();
	__asm__ volatile ("wfi");
	start_critical_timings();
}

static inline void elite_flow_wfe(struct cpuidle_device *dev)
{

	stop_critical_timings();
	dsb();
	__asm__ volatile ("wfe");//wfecg
	start_critical_timings();
}

int PGCount=0;
void elite_idle_enter_pg_cpu(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int index,s64 request)
{
	struct cpuidle_state *state;
	s64 left_time,sleep_time;
	unsigned int intsleep_time;
	//bool sleep_completed = false;
	int cpu=dev->cpu;

	state = &(drv->states[index]);

	sleep_time = request - adjusted_latency;
	intsleep_time = (unsigned int)sleep_time;
	idle_stats.cpu_enter_pg_count[cpu]++;
	left_time = elite_power_gating(intsleep_time,dev);
	if (left_time == 0)
	{
		idle_stats.pg_full_sleep_time[cpu] += sleep_time;
		idle_stats.pg_full_sleep_count[cpu]++;
		//sleep_completed = true;
	}
	else
	{
		idle_stats.pg_partial_sleep_time[cpu] += sleep_time - left_time;
		idle_stats.pg_partial_sleep_count[cpu]++;
	}
}



static int elite_idle_enter_cg(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int index)
{
#ifdef CONFIG_PERF_EVENTS
	char  trace_counter_name[20];
	s64   trace_counter;
#endif
	ktime_t enter, exit;
	s64 us;
	static __u64 cg_times_before_cg[MAXCPUS],cg_times_after_cg[MAXCPUS];
	static __u64 cg_cycles_before_cg[MAXCPUS],cg_cycles_after_cg[MAXCPUS];
	static __u64 cg_times[MAXCPUS],cg_cycles[MAXCPUS];
	union corex_wfi_cfg wfi_cfg;
	int cpu=dev->cpu;

	idle_stats.gov_req_cg_count[cpu]++;
	local_irq_disable();
	local_fiq_disable();
	enter = ktime_get();

	elite_get_cg_times((__u64 *)&cg_times_before_cg,cpu);
	elite_get_cg_cycles((__u64 *)&cg_cycles_before_cg,cpu);

	if (!need_resched())
	{
		idle_stats.cpu_enter_cg_count[cpu]++;
		wfi_cfg.cfg_wfi.cfgwfi = WFI_CFG_KEEP_ORIGINAL_ARM_BEHAVIOR;
		elite_corex_wfi_cfg(wfi_cfg,cpu);
		elite_cpu_lp_cfg_corex(ARM_CORE_CG,cpu);

		elite_flow_wfi(dev);
		elite_get_cg_times((__u64 *)&cg_times_after_cg,cpu);
		cg_times[cpu] = cg_times_after_cg[cpu] - cg_times_before_cg[cpu];
	}
	elite_get_cg_cycles((__u64 *)&cg_cycles_after_cg,cpu);
	cg_cycles[cpu] = cg_cycles_after_cg[cpu] - cg_cycles_before_cg[cpu];
	exit = ktime_sub(ktime_get(), enter);
	local_fiq_enable();
	local_irq_enable();
	us = ktime_to_us(exit);
	dev->last_residency = us;
	idle_stats.cg_time[cpu] +=us;

#ifdef CONFIG_PERF_EVENTS
	strcpy((char *)&trace_counter_name, "clock_gating_count");
	trace_counter = cg_times_after_cg[cpu];
	counter_output((char *)&trace_counter_name,trace_counter,cpu);
	strcpy((char *)&trace_counter_name, "clock_gating_cycles");
	trace_counter = cg_cycles_after_cg[cpu];
	counter_output((char *)&trace_counter_name,trace_counter,cpu);
#endif
	return index;
}

static int elite_idle_enter_pg(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int index)
{



#ifdef CONFIG_PERF_EVENTS
	char  trace_counter_name[20];
	s64   trace_counter;
#endif
	ktime_t enter, exit;
	s64 us,request;
	struct cpuidle_state *state;
	static __u64  pg_times_before_pg[MAXCPUS],pg_times_after_pg[MAXCPUS];
	static __u64  pg_cycles_before_pg[MAXCPUS],pg_cycles_after_pg[MAXCPUS];
	union corex_wfi_cfg wfe_cfg;
	union corex_wfi_cfg wfi_cfg;
	int cpu=dev->cpu; 
	int offset;

	state = &(drv->states[index]);


	idle_stats.gov_req_pg_count[cpu]++;
	dbgcmd = *mycmd;


	//if (pg_disabled_by_suspend || elite_neon_is_on[cpu] || in_dvfs)
	if ( pg_disabled_by_suspend || in_dvfs || 0 == g_pg_state )
		//	if ( pg_disabled_by_suspend ||  0 == g_pg_state )       
	{
		return elite_idle_enter_cg(dev, drv,index);
	}

	if((dbgcmd & DBGCMD_POWER_GATING) != DBGCMD_POWER_GATING)
	{  
		local_irq_disable();
		local_fiq_disable();
		enter = ktime_get();
		if (!need_resched())
		{
			wfi_cfg.cfg_wfi.cfgwfi = WFI_CFG_KEEP_ORIGINAL_ARM_BEHAVIOR;
			elite_corex_wfi_cfg(wfi_cfg,cpu);
			elite_flow_wfi(dev);
		}
		exit = ktime_sub(ktime_get(), enter);
		local_fiq_enable();
		local_irq_enable();
		us = ktime_to_us(exit);
		dev->last_residency = us;
		return index;
	}

	if (!need_resched())
	{
		offset = 200; //this parameter is to offset software cost.
		request = ktime_to_us(tick_nohz_get_sleep_length());
		adjusted_latency = 38000/cpu_dynamic_clk + state->target_residency + offset;
		if (request <= adjusted_latency||in_dvfs) 
			return elite_idle_enter_cg(dev, drv,index);
		spin_lock(&pg_dvfs_lock);
		get_cpu_var(in_pg)++;
		put_cpu_var(in_pg);
		spin_unlock(&pg_dvfs_lock);
		//===================================================
		void __iomem *reg_reset_status;
		reg_reset_status = (void __iomem *)IO_ADDRESS(0xd839c414);
		writel(readl(reg_reset_status)|0x10,reg_reset_status);
		//===================================================
		elite_turn_off_neon(cpu);
		local_irq_disable();
		//clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &dev->cpu);
		local_fiq_disable();
		enter = ktime_get();
		elite_get_pg_times((__u64 *)&pg_times_before_pg,cpu);
		elite_get_pg_cycles((__u64 *)&pg_cycles_before_pg,cpu);
		wfe_cfg.cfg_wfi.cfgwfi = WFI_CFG_ENTER_MULTICORE_LP_MODE;
		elite_corex_wfi_cfg(wfe_cfg,cpu);
		elite_cpu_lp_cfg_corex(ARM_CORE_PG,cpu);
		elite_idle_enter_pg_cpu(dev, drv,index,request);
		elite_get_pg_times((__u64 *)&pg_times_after_pg,cpu);
		elite_get_pg_cycles((__u64 *)&pg_cycles_after_pg,cpu);
		wfi_cfg.cfg_wfi.cfgwfi = WFI_CFG_KEEP_ORIGINAL_ARM_BEHAVIOR;
		elite_corex_wfi_cfg(wfi_cfg,cpu);
		elite_cpu_lp_cfg_corex(ARM_CORE_ON,cpu);
		smp_mb();
		smp_rmb();

		if((cpu==0) && (*((int*)0xfe39c010)==0xa))
		{
			asm volatile ("bkpt 0x1");
		}
		if((cpu==1) && (*((int*)0xfe39c014)==0xa))
		{
			asm volatile ("bkpt 0x1");
		}
		exit = ktime_sub(ktime_get(), enter);
		local_fiq_enable();
		//clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &dev->cpu);
		local_irq_enable();
		smp_rmb();
		spin_lock(&pg_dvfs_lock);
		get_cpu_var(in_pg)--;
		put_cpu_var(in_pg);
		spin_unlock(&pg_dvfs_lock);


		us = ktime_to_us(exit);
		dev->last_residency = us;


	}
	else
	{  
		return elite_idle_enter_cg(dev, drv,index);
	}


	idle_stats.in_pg_time[cpu] += us;
	dev->last_residency = us;
#ifdef CONFIG_PERF_EVENTS
	strcpy((char *)&trace_counter_name, "power_gating_count");
	trace_counter = pg_times_after_pg[cpu];
	counter_output((char *)&trace_counter_name,trace_counter,cpu);
	strcpy((char *)&trace_counter_name, "power_gating_cycles");
	trace_counter = pg_cycles_after_pg[cpu];
	counter_output((char *)&trace_counter_name,trace_counter,cpu);
#endif
	return index;
}


static int elite_cpuidle_pm_notify(struct notifier_block *nb,
		unsigned long event, void *dummy)
{
	if (event == PM_SUSPEND_PREPARE)
		pg_disabled_by_suspend = true;
	else if (event == PM_POST_SUSPEND)
		pg_disabled_by_suspend = false;

	return NOTIFY_OK;
}

static struct notifier_block elite_cpuidle_pm_notifier = {
	.notifier_call = elite_cpuidle_pm_notify,
};

//=========================================================================================================
static struct kobject *parent;
static struct kobject *child_kset_cpu0,*child_kset_cpu1;
static struct kset *c_kset_cpu0,*c_kset_cpu1;
#define PG_CPU0_ADDR  (void __iomem*)IO_ADDRESS(0xD839C740)
#define PG_CPU1_ADDR   (void __iomem*)IO_ADDRESS(0xD839C750)

#define CG_CPU0_ADDR   (void __iomem*)IO_ADDRESS(0xD839C700)
#define CG_CPU1_ADDR   (void __iomem*)IO_ADDRESS(0xD839C710)
static ssize_t att_show(struct kobject *kobj,struct attribute *attr ,char *buf)
{
	if(!strcmp("cpu0",kobj->name))
	{
		if(!strcmp("cpu_pg_times",attr->name))
			sprintf(buf,"%lld",*(unsigned long long*)PG_CPU0_ADDR);
		if(!strcmp("cpu_cg_times",attr->name))
			sprintf(buf,"%lld",*(unsigned long long*)CG_CPU0_ADDR);
	}
	else
	{
		if(!strcmp("cpu_pg_times",attr->name))
			sprintf(buf,"%lld",*(unsigned long long*)PG_CPU1_ADDR);
		if(!strcmp("cpu_cg_times",attr->name))
			sprintf(buf,"%lld",*(unsigned long long*)CG_CPU1_ADDR);
	}

	sprintf(&buf[8],"%s","\n");
	return 9;
}


static const struct sysfs_ops att_ops={
	.show=att_show,
};
static struct kobj_type  cpu_ktype={
	.sysfs_ops=&att_ops,
};

static struct attribute cpu0_pg_att={
	.name ="cpu_pg_times",
	.mode = S_IRUGO | S_IWUSR,
};

static struct attribute cpu1_pg_att={
	.name ="cpu_pg_times",
	.mode = S_IRUGO | S_IWUSR,
};

static struct attribute cpu0_cg_att={
	.name ="cpu_cg_times",
	.mode = S_IRUGO | S_IWUSR,
};

static struct attribute cpu1_cg_att={
	.name ="cpu_cg_times",
	.mode = S_IRUGO | S_IWUSR,
};

//==========================================================
static ssize_t att_pg_show(struct kobject *kobj,struct attribute *attr ,char *buf)
{
	size_t count=0;
	count += sprintf(&buf[count],"%lu\n",g_pg_state);
	return count;
}

static ssize_t att_pg_store(struct kobject *kobj,struct attribute *attr ,const char *buf,size_t count)
{
	unsigned long tmp;
	tmp=buf[0]-'0';

	if(tmp==0||tmp==1)
	{
		g_pg_state=tmp;
		return count;
	}
	else
		return -1;

}

static const struct sysfs_ops att_pg_ops={
	.show=att_pg_show,
	.store=att_pg_store,
};

static struct attribute pg_state_att={
	.name ="pg_state",
	.mode = S_IRUGO | S_IWUSR,
};

static struct kobj_type  pg_state_ktype={
	.sysfs_ops=&att_pg_ops,
};


//==========================================================


static int pg_info_init(void)
{
	int err;

	parent = kobject_create_and_add("cpu_power_info",NULL);
	//===============================================================
	parent->ktype = &pg_state_ktype;
	err=sysfs_create_file(parent,&pg_state_att);
	if(err)
		return err;
	//===============================================================
	child_kset_cpu0 = kzalloc(sizeof(*child_kset_cpu0),GFP_KERNEL);
	if(!child_kset_cpu0)
		return -1;

	err=kobject_init_and_add(child_kset_cpu0,&cpu_ktype,parent,"cpu0");
	if(err)
		return err;
	err=sysfs_create_file(child_kset_cpu0,&cpu0_pg_att);
	if(err)
		return err;

	err=sysfs_create_file(child_kset_cpu0,&cpu0_cg_att);
	if(err)
		return err;
	//===============================================================
	child_kset_cpu1 = kzalloc(sizeof(*child_kset_cpu1),GFP_KERNEL);
	if(!child_kset_cpu1)
		return -1;

	err=kobject_init_and_add(child_kset_cpu1,&cpu_ktype,parent,"cpu1");
	if(err)
		return err;

	err=sysfs_create_file(child_kset_cpu1,&cpu1_pg_att);
	if(err)
		return err;

	err=sysfs_create_file(child_kset_cpu1,&cpu1_cg_att);
	if(err)
		return err;
	//================================================================

	return 0;
}
//==============================================================================================

static int __init elite_cpuidle_init(void)
{
	unsigned int cpu;
	int ret;
	struct cpuidle_device *dev;
	struct cpuidle_driver *drv = &elite_idle;


	printk(KERN_DEBUG "%s: \n", __func__);
	if (pg_disabled_by_suspend){
		return 0;
	}
	ret = cpuidle_register_driver(&elite_idle);
	if (ret) {
		pr_err("CPUidle driver registration failed\n");
		return ret;
	}

	for_each_possible_cpu(cpu) {
		dev = &per_cpu(idle_devices, cpu);
		dev->cpu = cpu;

		dev->state_count = drv->state_count;

		if (cpuidle_register_device(dev)){
			pr_err("CPU%u: error initializing idle loop\n", cpu);
			return ret;
		}
	}
	printk(KERN_DEBUG "%s: start \n", __func__);
	mycmd = (unsigned int *)kzalloc(8, GFP_KERNEL);
	if(mycmd)
	{
		printk(KERN_DEBUG "%s:gxn mycmd address =%x \n", __func__,(unsigned int)mycmd);
		//Hack
		if(elite_pg_is_on())
			*mycmd= DBGCMD_CLOCK_GATING | DBGCMD_POWER_GATING | DBGCMD_CLOCK_GATING_HW | DBGCMD_POWER_GATING_HW;
		else
			*mycmd= DBGCMD_CLOCK_GATING | DBGCMD_CLOCK_GATING_HW | DBGCMD_POWER_GATING_HW;

		printk(KERN_DEBUG "%s:gxn *mycmd =%x \n", __func__,(unsigned int)*mycmd);
	}
	else
		printk(KERN_DEBUG "%s:allocate mycmd error\n", __func__);

	elite_pg_exit_latency = 200; //estimated value in us

	register_pm_notifier(&elite_cpuidle_pm_notifier);
	init_idle_stats();
	//init NEON monitor, should be moved to platform initialize function eventually
	//if(elite_pg_is_on())
	//elite_init_neon_monitor();

	pg_info_init(); 

	return 0;
}

static void __exit elite_cpuidle_exit(void)
{
	sysfs_remove_file(parent,&pg_state_att);

	sysfs_remove_file(child_kset_cpu0,&cpu0_pg_att);
	sysfs_remove_file(child_kset_cpu0,&cpu0_cg_att);
	kset_unregister(c_kset_cpu0);
	kobject_del(child_kset_cpu0);

	sysfs_remove_file(child_kset_cpu1,&cpu1_pg_att);
	sysfs_remove_file(child_kset_cpu1,&cpu1_cg_att);
	kset_unregister(c_kset_cpu1);
	kobject_del(child_kset_cpu1);

	kobject_del(parent);

	cpuidle_unregister_driver(&elite_idle);
}

device_initcall(elite_cpuidle_init);

#ifdef CONFIG_DEBUG_FS
bool showcmd = false;
ktime_t begin_data, end_data;
static int elite_cpuidle_debug_show(struct seq_file *s, void *data)
{
	s64 time_elapsed,t;
	int percent;
	int i,j;

	if(!showcmd)
	{
		seq_printf(s, "------------Clear idle_stats ------------\n");
		init_idle_stats();
		showcmd = true;
		begin_data = ktime_get();
		return 0;
	}

	showcmd = false;
	end_data = ktime_sub(ktime_get(), begin_data);
	time_elapsed = ktime_to_us(end_data);
	//dump_stack();
	for(i=0;i<MAXCPUS;i++)
	{   
		seq_printf(s, "------------Begin idle_stats ------------\n");
		seq_printf(s, "Start CPU%d data:\n",i);
		seq_printf(s, "------------Total elapsed time %lld us------------\n",time_elapsed);
		seq_printf(s, "gov_req_cg_count[%d]=%8d\n",i,idle_stats.gov_req_cg_count[i]);
		seq_printf(s, "cpu_enter_cg_count[%d]=%8d\n",i,idle_stats.cpu_enter_cg_count[i]);
		seq_printf(s, "gov_req_pg_count[%d]=%8d\n",i,idle_stats.gov_req_pg_count[i]);
		seq_printf(s, "cpu_enter_pg_count[%d]=%8d\n",i,idle_stats.cpu_enter_pg_count[i]);
		seq_printf(s, "cg_time[%d]=%8u us\n",i,(int)idle_stats.cg_time[i]);
		seq_printf(s, "pg_full_sleep_time[%d]=%8u us\n",i,(int)idle_stats.pg_full_sleep_time[i]);
		seq_printf(s, "pg_partial_sleep_time[%d]=%8u us\n",i,(int)idle_stats.pg_partial_sleep_time[i]);
		seq_printf(s, "in_pg_time[%d]=%8u us\n",i,(int)idle_stats.in_pg_time[i]);
		seq_printf(s, "pg_full_sleep_count[%d]=%8d\n",i,idle_stats.pg_full_sleep_count[i]);
		seq_printf(s, "pg_partial_sleep_count[%d]=%8d\n",i,idle_stats.pg_partial_sleep_count[i]);
		percent = 0;
		t = idle_stats.cg_time[i]*100;
		for(j=0;j<100;j++)
		{
			if( t > time_elapsed)
			{
				percent++;
				t -= time_elapsed;
			}
			else 
				break;
		}
		seq_printf(s, "cg percentage of total time on cpu%d = %d\n",i,percent);

		percent = 0;
		t = idle_stats.in_pg_time[i]*100;
		for(j=0;j<100;j++)
		{
			if( t > time_elapsed)
			{
				percent++;
				t -= time_elapsed;
			}
			else 
				break;
		}
		seq_printf(s, "pg percentage of total time on cpu%d = %d\n",i,percent);
		seq_printf(s, "cpuidle:dbgcmd = %d\n",dbgcmd);
		seq_printf(s, "------------End  idle_stats ------------\n");
	}

	return 0;
}

static int elite_cpuidle_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, elite_cpuidle_debug_show, inode->i_private);
}

static const struct file_operations elite_cpuidle_debug_ops = {
	.open		= elite_cpuidle_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

//-------------debug---------------
static int cpuidle_debug_set(void *data, u64 val)
{
	dbgcmd = (unsigned int)val;
	return 0;
}
static int cpuidle_debug_get(void *data, u64 *val)
{
	*val = (u64)dbgcmd;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(cpuidle_debug_fops, cpuidle_debug_get, cpuidle_debug_set, "%llu\n");

//-------------debug---------------

static int __init elite_cpuidle_debug_init(void)
{
	struct dentry *dir;
	struct dentry *d;

	dir = debugfs_create_dir("cpuidle", NULL);
	if (!dir)
	{
		printk(KERN_DEBUG "create cpuidle directory failed in %s: \n", __func__);
		return -ENOMEM;

	}
	printk(KERN_DEBUG "create cpuidle directory successfully in %s: \n", __func__);
	if (!debugfs_create_file("dbgcmd", 0644, dir, NULL, &cpuidle_debug_fops))
		return -ENOMEM;

	d = debugfs_create_file("idle_stats", S_IRUGO, dir, NULL,
			&elite_cpuidle_debug_ops);
	if (!d)
		return -ENOMEM;

	return 0;
}

late_initcall(elite_cpuidle_debug_init);
#endif //#ifdef CONFIG_DEBUG_FS
#endif //#ifdef CONFIG_CPU_IDLE
