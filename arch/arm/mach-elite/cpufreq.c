/*
 * arch/arm/mach-elite/cpufreq.c
 *
 * CPU state save & restore for CPU power power management
 *
 *   Based on arch/arm/mach-tegra/cpu-tegra.c Copyright (c) 2010, NVIDIA Corporation.
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
#ifdef CONFIG_CPU_FREQ
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/suspend.h>
#include <linux/debugfs.h>
#include <linux/cpu.h>
#include <asm/smp_twd.h>
#include <asm/system.h>
#include <linux/hrtimer.h>
#include "cpu-elite.h"
#include "clock.h"


#define DVFS_TABLE_SIZE		16
static struct cpufreq_frequency_table *freq_table;
bool   is_pluging_unpluging=false;
//extern volatile int   in_pg[2];
DECLARE_PER_CPU(unsigned int,in_pg);
extern spinlock_t pg_dvfs_lock;

volatile int   in_dvfs;

#define NUM_CPUS	2

static struct clk *cpu_clk = NULL;
int cpu_dynamic_clk = 1000; //(1000M);
static unsigned long target_cpu_speed[NUM_CPUS];
DEFINE_MUTEX(elite_cpu_lock);
static bool is_suspended = false;

unsigned int elite_getspeed(unsigned int cpu);
int elite_update_cpu_speed(unsigned long rate);
static unsigned long elite_cpu_highest_speed(void);

//define for debug by vfs:
unsigned int dbg_dvfs_cmd=DBG_DVFS_CMD_DVFS_ON | DBG_DVFS_CMD_DVFS_HW; //| DBG_DVFS_CMD_DVFS_CPU_ONOFF;
extern unsigned int dbgcmd;

enum cpu_op {
	turn_off_cpu1 = 0,
	turn_on_cpu1,
	turn_off_cpu2and3,
	turn_on_cpu2and3,
};

ktime_t time_last_onoff;
bool doing_cpuonoff=false;
int cpuidwork;
int cpuopwork;
#define MIN_DURATION 10000000
static struct delayed_work cpu_onoff_work;
static struct workqueue_struct *cpu_onoff_workqueue;
static unsigned int min_frequency,max_frequency;

//declaration of external function
#ifdef CONFIG_PERF_EVENTS
extern void counter_output(const char *name, s64 count, int cpu);
#endif
extern void elite_get_cpus_status(union general_cfg0_status *cpustatus);
extern void elite_set_cpus_status(union general_cfg0_status *cpustatus);
extern int elite_get_command_slots(void);
extern void elite_get_pmu_status(union pmu_status *pmustatus);
extern void elite_get_cpucore_dvfs(union cpucore_dvfs *cpucore_dvfs);
extern int clk_set_rate(struct clk *c, unsigned long rate);
extern struct cpufreq_frequency_table *build_cpufreq_table(struct elite_cpufreq_table_data *table_data);
void elite_dump_dvfs_table_to_pmu(struct elite_cpufreq_table_data *table_data);
extern struct clk *elite_get_clk_cpu(void);
extern void elite_cpufreq_subinit(void);
extern void __init elite_cpu_init_clocks(unsigned int idx);
static struct {
	unsigned int numbers_online_cpus;
	unsigned int current_voltage;
	unsigned int current_frequency;
	unsigned int current_idx;
	unsigned int count_total_speed_update;
	unsigned int count_on_frequency[DVFS_TABLE_SIZE];
	s64          time_on_frequency[DVFS_TABLE_SIZE];
	ktime_t      current_begin_time;
	unsigned int count_4cpu_online;
	unsigned int count_2cpu_online;
	unsigned int count_1cpu_online;
	unsigned int count_4cpu_to_2cpu;
	unsigned int count_2cpu_to_1cpu;
	unsigned int count_1cpu_to_2cpu;
	unsigned int count_2cpu_to_4cpu;
} dvfs_stats;

void init_dvfs_stats(void)
{
	int i;
	printk(KERN_DEBUG "%s: \n", __func__);
	dvfs_stats.numbers_online_cpus = MAXCPUS;
	dvfs_stats.current_voltage = cpu_clk->voltage;
	dvfs_stats.current_frequency = cpu_clk->rate;
	dvfs_stats.count_total_speed_update = 0;
	dvfs_stats.current_begin_time = ktime_get();
	for(i=0;i<DVFS_TABLE_SIZE;i++)
	{
		dvfs_stats.count_on_frequency[i] = 0;
		dvfs_stats.time_on_frequency[i] = 0;
	}
	dvfs_stats.count_4cpu_online = 0;
	dvfs_stats.count_2cpu_online = 0;
	dvfs_stats.count_1cpu_online = 0;
	dvfs_stats.count_4cpu_to_2cpu = 0;
	dvfs_stats.count_2cpu_to_1cpu = 0;
	dvfs_stats.count_1cpu_to_2cpu = 0;
	dvfs_stats.count_2cpu_to_4cpu = 0;

}

#ifdef CONFIG_ELITE_THERMAL_THROTTLE
/* CPU frequency is gradually lowered when throttling is enabled */
#define THROTTLE_DELAY		msecs_to_jiffies(2000)
#define elite_cpu_is_throttling() (is_throttling)
static struct delayed_work throttle_work;
static struct workqueue_struct *workqueue;
bool is_throttling = false;
static int throttle_lowest_index;
static int throttle_highest_index;
int throttle_index;
static int throttle_next_index;

void elite_throttling_enable(bool enable)
{
}
static void elite_throttle_work_func(struct work_struct *work)
{
	unsigned int current_freq;

	mutex_lock(&elite_cpu_lock);
	current_freq = elite_getspeed(0);
	throttle_index = throttle_next_index;

	printk(KERN_DEBUG "elite_throttle_work_func: Throttle_idx==%d ,throttle_frq=%d,current_freq=%d\n",throttle_index,freq_table[throttle_index].frequency,current_freq);	
	if (freq_table[throttle_index].frequency < current_freq)
		elite_update_cpu_speed(freq_table[throttle_index].frequency);

	if (throttle_index > throttle_lowest_index) {
		throttle_next_index = throttle_index - 1;
		queue_delayed_work(workqueue, &throttle_work, THROTTLE_DELAY);
	}

	mutex_unlock(&elite_cpu_lock);
}

/*
 * elite_throttling_enable
 * This function may sleep
 */
void elite_throttling_enable(bool enable)
{
	mutex_lock(&elite_cpu_lock);

	printk(KERN_DEBUG "elite_throttling_enable(enable=%d)\n",enable);
	if (enable && !is_throttling) {
		unsigned int current_freq = elite_getspeed(0);

		is_throttling = true;

		for (throttle_index = throttle_highest_index;
				throttle_index >= throttle_lowest_index;
				throttle_index--)
			if (freq_table[throttle_index].frequency
					< current_freq)
				break;

		throttle_index = max(throttle_index, throttle_lowest_index);
		throttle_next_index = throttle_index;
		queue_delayed_work(workqueue, &throttle_work, 0);
	} else if (!enable && is_throttling) {
		cancel_delayed_work_sync(&throttle_work);
		is_throttling = false;
		/* restore speed requested by governor */
		elite_update_cpu_speed(elite_cpu_highest_speed());
	}

	mutex_unlock(&elite_cpu_lock);
}
EXPORT_SYMBOL_GPL(elite_throttling_enable);

static unsigned int throttle_governor_speed(unsigned int requested_speed)
{
	return elite_cpu_is_throttling() ?
		min(requested_speed, freq_table[throttle_index].frequency) :
		requested_speed;
}

static ssize_t show_throttle(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", is_throttling);
}

cpufreq_freq_attr_ro(throttle);


#else /* CONFIG_ELITE_THERMAL_THROTTLE */
#define elite_cpu_is_throttling() (0)
#define throttle_governor_speed(requested_speed) (requested_speed)

void elite_throttling_enable(bool enable)
{
}
#endif /* CONFIG_ELITE_THERMAL_THROTTLE */


#ifdef CONFIG_DEBUG_FS
#ifdef CONFIG_ELITE_THERMAL_THROTTLE
static int throttle_debug_set(void *data, u64 val)
{
	elite_throttling_enable(val);
	return 0;
}
static int throttle_debug_get(void *data, u64 *val)
{
	*val = (u64) is_throttling;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(throttle_fops, throttle_debug_get, throttle_debug_set, "%llu\n");
#endif /* CONFIG_ELITE_THERMAL_THROTTLE */

static int cpudvfs_debug_set(void *data, u64 val)
{
	dbg_dvfs_cmd = (unsigned int)val;
	return 0;
}
static int cpudvfs_debug_get(void *data, u64 *val)
{
	*val = (u64)dbg_dvfs_cmd;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(cpuidle_debug_fops, cpudvfs_debug_get, cpudvfs_debug_set, "%llu\n");

bool dvfs_showcmd = false;
ktime_t dvfs_begin_data, dvfs_end_data;
static int elite_dvfs_debug_show(struct seq_file *s, void *data)
{
	s64 time_elapsed,t;
	ktime_t time_delta;
	struct cpufreq_frequency_table			*myfreq_table;
	int percent;
	int i,j;

	if(!dvfs_showcmd)
	{
		seq_printf(s, "------------Clear DVFS_statistic ------------\n");
		init_dvfs_stats();
		dvfs_showcmd = true;
		dvfs_begin_data = ktime_get();
		return 0;
	}

	dvfs_showcmd = false;
	dvfs_end_data = ktime_get();
	time_delta = ktime_sub(dvfs_end_data, dvfs_begin_data);
	time_elapsed = ktime_to_us(time_delta);
	time_delta = ktime_sub(dvfs_end_data, dvfs_stats.current_begin_time);
	dvfs_stats.time_on_frequency[dvfs_stats.current_idx] += ktime_to_us(time_delta);
	seq_printf(s, "------------Begin DVFS_statistic ------------\n");
	seq_printf(s, "------------Total elapsed time %lld us------------\n",time_elapsed);
	seq_printf(s, "number of online cpus =%d \n", dvfs_stats.numbers_online_cpus);
	seq_printf(s, "current voltage =%d \n", dvfs_stats.current_voltage);
	seq_printf(s, "current frequency =%d \n", dvfs_stats.current_frequency);
	seq_printf(s, "count_total_speed_update =%d \n", dvfs_stats.count_total_speed_update);
	myfreq_table = freq_table;
	for(i=0;i<DVFS_TABLE_SIZE;i++)
	{
		seq_printf(s, "coun_on_frequency:%d =%d \n", myfreq_table->frequency,dvfs_stats.count_on_frequency[i]);
		myfreq_table++;
	}
	myfreq_table = freq_table;
	for(i=0;i<DVFS_TABLE_SIZE;i++)
	{
		seq_printf(s, "time_on_frequency:%d =%lld \n", myfreq_table->frequency,dvfs_stats.time_on_frequency[i]);
		percent = 0;
		t = dvfs_stats.time_on_frequency[i]*100;
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
		seq_printf(s, "percent_on_frequency:%d =%d \n", myfreq_table->frequency,percent);
		myfreq_table++;
	}
	seq_printf(s, "count_4cpu_online =%d \n", dvfs_stats.count_4cpu_online);
	seq_printf(s, "count_2cpu_online =%d \n", dvfs_stats.count_2cpu_online);
	seq_printf(s, "count_1cpu_online =%d \n", dvfs_stats.count_1cpu_online);
	seq_printf(s, "count_4cpu_to_2cpu =%d \n", dvfs_stats.count_4cpu_to_2cpu);
	seq_printf(s, "count_2cpu_to_1cpu =%d \n", dvfs_stats.count_2cpu_to_1cpu);
	seq_printf(s, "count_1cpu_to_2cpu =%d \n", dvfs_stats.count_1cpu_to_2cpu);
	seq_printf(s, "count_2cpu_to_4cpu =%d \n", dvfs_stats.count_2cpu_to_4cpu);

	seq_printf(s, "%s:dbg_dvfs_cmd = %d\n",__func__,dbg_dvfs_cmd);

	return 0;
}

static int elite_dvfs_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, elite_dvfs_debug_show, inode->i_private);
}

static const struct file_operations elite_dvfs_debug_ops = {
	.open		= elite_dvfs_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct dentry *cpu_elite_debugfs_root;
static int __init elite_cpu_debug_init(void)
{

	printk(KERN_DEBUG "%s: \n", __func__);
	cpu_elite_debugfs_root = debugfs_create_dir("cpudvfs", 0);

	if (!cpu_elite_debugfs_root)
	{
		printk(KERN_DEBUG "create cpudvfs directory failed in %s: \n", __func__);
		return -ENOMEM;
	}
	printk(KERN_DEBUG "create cpudvfs directory successfully in %s: \n", __func__);

#ifdef CONFIG_ELITE_THERMAL_THROTTLE
	if (!debugfs_create_file("throttle", 0644, cpu_elite_debugfs_root, NULL, &throttle_fops))
		goto err_out;
#endif /* CONFIG_ELITE_THERMAL_THROTTLE */

	if (!debugfs_create_file("dbgcmd", 0644, cpu_elite_debugfs_root, NULL, &cpuidle_debug_fops))
		goto err_out;

	if (!debugfs_create_file("dvfs_stats", S_IRUGO, cpu_elite_debugfs_root, NULL, &elite_dvfs_debug_ops))
		goto err_out;

	return 0;

err_out:
	printk(KERN_DEBUG "%s:error out \n", __func__);
	debugfs_remove_recursive(cpu_elite_debugfs_root);
	return -ENOMEM;

}

static void __exit elite_cpu_debug_exit(void)
{
	printk(KERN_DEBUG "%s: \n", __func__);
	debugfs_remove_recursive(cpu_elite_debugfs_root);
}

late_initcall(elite_cpu_debug_init);
module_exit(elite_cpu_debug_exit);
#endif /* CONFIG_DEBUG_FS */

int elite_verify_speed(struct cpufreq_policy *policy)
{
	printk(KERN_DEBUG "%s: \n", __func__);
	return cpufreq_frequency_table_verify(policy, freq_table);
}

unsigned int elite_getspeed(unsigned int cpu)
{
	unsigned long rate;

	rate = cpu_clk->rate;
	printk(KERN_DEBUG "%s: rate =%ld\n", __func__,rate);
	return rate;
}

void queue_cpu(enum cpu_op cpuop)
{
	int duration;
	ktime_t time_current;

	//init parameter for cpu_onoff_work;
	cpuopwork = cpuop;
	//
	time_current = ktime_get();
	duration = ktime_to_us(ktime_sub(time_current, time_last_onoff));
	if(duration > MIN_DURATION) 
	{
		doing_cpuonoff=true;
		queue_delayed_work(cpu_onoff_workqueue, &cpu_onoff_work, 0);
		time_last_onoff = ktime_get();
	}
}

static void cpu_onoff_work_func(struct work_struct *work)
{
#ifdef CONFIG_HOTPLUG_CPU
	union general_cfg0_status cpustatus;


	mutex_lock(&elite_cpu_lock);
	is_pluging_unpluging=true;
	//elite_get_cpus_status(&cpustatus);
	switch (cpuopwork) {
		case turn_on_cpu1:
			cpu_up(1);
			cpustatus.status.arm_core1 = CPU_ON;
			printk(KERN_DEBUG "cpu1 up\n");
			dvfs_stats.numbers_online_cpus = 2;
			dvfs_stats.count_1cpu_to_2cpu++;
			doing_cpuonoff=false;
			break;

		case turn_off_cpu1:
			cpu_down(1);
			cpustatus.status.arm_core1 = CPU_OFF;
			printk(KERN_DEBUG "cpu1 down\n");
			dvfs_stats.numbers_online_cpus = 1;
			dvfs_stats.count_2cpu_to_1cpu++;
			doing_cpuonoff=false;
			break;

		case turn_on_cpu2and3:
			cpu_up(2);
			cpu_up(3);
			cpustatus.status.arm_core2 = CPU_ON;
			cpustatus.status.arm_core3 = CPU_ON;
			printk(KERN_DEBUG "cpu2 and cpu3 up\n");
			dvfs_stats.numbers_online_cpus = 4;
			dvfs_stats.count_2cpu_to_4cpu++;
			doing_cpuonoff=false;
			break;

		case turn_off_cpu2and3:
			cpu_down(2);
			cpu_down(3);
			cpustatus.status.arm_core2 = CPU_OFF;
			cpustatus.status.arm_core3 = CPU_OFF;
			printk(KERN_DEBUG "cpu2 and cpu3 down\n");
			dvfs_stats.numbers_online_cpus = 2;
			dvfs_stats.count_4cpu_to_2cpu++;
			doing_cpuonoff=false;
			break;
	}

	//elite_set_cpus_status(&cpustatus);
	is_pluging_unpluging=false;
	mutex_unlock(&elite_cpu_lock);
#endif //CONFIG_HOTPLUG_CPU
}

void cpus_onoff(unsigned long rate)
{
	unsigned int Frequency_threshold_min =max_frequency/10;
	unsigned int Frequency_threshold_max =max_frequency*1/5;
	union general_cfg0_status cpustatus;

	cpustatus.val=0xffffffff;
	if(rate < Frequency_threshold_min)
	{
		//elite_get_cpus_status(&cpustatus);
		if(MAXCPUS==2)
		{
			if(cpustatus.status.arm_core1 == CPU_ON) 
			{
				queue_cpu(turn_off_cpu1);    
			}   
		}
		else if(MAXCPUS==4)
		{
			if((cpustatus.status.arm_core2 == CPU_ON) &&
					(cpustatus.status.arm_core3 == CPU_ON))
			{
				queue_cpu(turn_off_cpu2and3);    
			}   
			else if(cpustatus.status.arm_core1 == CPU_ON) 
			{
				queue_cpu(turn_off_cpu1);    
			}   
		}

	}
	else if(rate > Frequency_threshold_max)
	{
		//elite_get_cpus_status(&cpustatus);
		if(MAXCPUS==2)
		{
			if(cpustatus.status.arm_core1 != CPU_ON) 
			{
				printk(KERN_DEBUG "%s:to queue turn_on_cpu1 \n", __func__);
				queue_cpu(turn_on_cpu1);    
			}   
		}
		else if(MAXCPUS==4)
		{
			if(cpustatus.status.arm_core1 != CPU_ON) 
			{
				printk(KERN_DEBUG "%s:to queue turn_on_cpu1 \n", __func__);
				queue_cpu(turn_on_cpu1);    
			}   
			else if((cpustatus.status.arm_core2 != CPU_ON) &&
					(cpustatus.status.arm_core3 != CPU_ON))
			{
				printk(KERN_DEBUG "%s:to queue turn_on_cpu2and3 \n", __func__);
				queue_cpu(turn_on_cpu2and3);    
			}   
		} 
	}
	//verify cpu state:
	if(cpustatus.val!=0xffffffff)
	{
		if(cpustatus.status.arm_core1 == CPU_ON)
			printk(KERN_DEBUG "cpu1 is on\n");
		else
			printk(KERN_DEBUG "cpu1 is off\n");

		if(MAXCPUS==4)
		{
			if(cpustatus.status.arm_core2 == cpustatus.status.arm_core3)
			{
				if(cpustatus.status.arm_core2 == CPU_ON)
					printk(KERN_DEBUG "cpu2 cpu3 are on\n");
				else
					printk(KERN_DEBUG "cpu2 cpu3 are off\n");
			}else
				printk(KERN_DEBUG "warning:cpu2 cpu3 are not in same state\n");

		}


	}

}

int elite_update_cpu_speed(unsigned long rate)
{
	int ret = 0,count=0,i=0;
	struct cpufreq_freqs freqs;

	if(dvfs_stats.numbers_online_cpus ==1)
		dvfs_stats.count_1cpu_online++;
	else if(dvfs_stats.numbers_online_cpus ==2)
		dvfs_stats.count_2cpu_online++;
	else if(dvfs_stats.numbers_online_cpus ==4)
		dvfs_stats.count_4cpu_online++;

	//turn on/off CPUs
	if((dbg_dvfs_cmd & DBG_DVFS_CMD_DVFS_CPU_ONOFF) == DBG_DVFS_CMD_DVFS_CPU_ONOFF)
		cpus_onoff(rate);

	if((dbg_dvfs_cmd & DBG_DVFS_CMD_DVFS_ON) != DBG_DVFS_CMD_DVFS_ON)
		return ret;
	freqs.old = elite_getspeed(0);
	freqs.new = rate;
	if (freqs.old == freqs.new)
		return ret;

	spin_lock(&pg_dvfs_lock);

	for_each_online_cpu(i)
		count +=per_cpu(in_pg,i);

	if(count!=0)
	{
		spin_unlock(&pg_dvfs_lock);
		return ret;
	}

	in_dvfs = 1;

	spin_unlock(&pg_dvfs_lock);

	for_each_online_cpu(freqs.cpu)
	{
		if(freqs.cpu==0)
			cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	}
#ifdef CONFIG_CPU_FREQ_DEBUG
	printk(KERN_DEBUG "cpufreq-lite: transition: %u --> %u\n",
			freqs.old, freqs.new);
#endif
	cpu_clk->nFreqScalling++;
	//printk(KERN_DEBUG "%s: Freqency Scalling #%x\n",__func__,cpu_clk->nFreqScalling);
	ret = clk_set_rate(cpu_clk, freqs.new); 

	spin_lock(&pg_dvfs_lock);
	in_dvfs = 0;
	spin_unlock(&pg_dvfs_lock);

	if (ret) {
		printk(KERN_DEBUG "Failed to set cpu frequency to %d kHz\n",
				freqs.new);
		return ret;
	}

	if(freqs.new!=elite_getspeed(0))
	{
		printk("########%s:%d\n",__func__,elite_getspeed(0));
		return -1;
	}

	dvfs_stats.current_voltage = cpu_clk->voltage;
	dvfs_stats.current_frequency = cpu_clk->rate;
	dvfs_stats.count_total_speed_update++;

	for_each_online_cpu(freqs.cpu)
	{
		if(freqs.cpu==0)
			cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}
	return 0;
}

EXPORT_SYMBOL(elite_update_cpu_speed);
static unsigned long elite_cpu_highest_speed(void) {
	unsigned long rate = 0;
	int i;

	for_each_online_cpu(i)
		rate = max(rate, target_cpu_speed[i]);
	return rate;
}

static int elite_target(struct cpufreq_policy *policy,
		unsigned int target_freq,
		unsigned int relation)
{
#ifdef CONFIG_PERF_EVENTS
	char  trace_counter_name[20];
	s64   trace_counter;
#endif
	int idx;
	unsigned int freq;
	unsigned int new_speed;
	int ret = 0;
	//printk(KERN_DEBUG "%s: cpu%d target freq=%d\n", __func__,policy->cpu,target_freq);
	//tmp
	//debug_addr = ELITE_PMU_BASE + PMU_CORE3_PTIMER_COUNTER_REG/sizeof(__iomem);
	//mydbgcmd = readl(debug_addr);
	//printk(KERN_DEBUG "%s:dbgcmd=%d \n", __func__,mydbgcmd);

	mutex_lock(&elite_cpu_lock);

	if (is_suspended) {
		ret = -EBUSY;
		goto out;
	}


	if(doing_cpuonoff){
		ret = -EBUSY;
		goto out;
	}
	cpufreq_frequency_table_target(policy, freq_table, target_freq,
			relation, &idx);

	freq = freq_table[idx].frequency;

	//printk(KERN_DEBUG "%s: freq=%d,idx=%d\n", __func__,freq,idx);
	target_cpu_speed[policy->cpu] = freq;

	if(policy->cpu==1){
		policy->cur=freq;
		ret = 0;
		goto out;
	}
	
	new_speed = throttle_governor_speed(elite_cpu_highest_speed());
	//printk(KERN_DEBUG "%s: adjusted freq=%d\n", __func__,new_speed);
	ret = elite_update_cpu_speed(new_speed);
	//ret = elite_update_cpu_speed(freq);
	if(!ret)
	{
#ifdef CONFIG_DEBUG_FS
		ktime_t  t,time_delta;
		int last_idx;
		last_idx = dvfs_stats.current_idx;
		t = ktime_get();
		time_delta = ktime_sub(t, dvfs_stats.current_begin_time);
		dvfs_stats.current_begin_time = t;
		dvfs_stats.time_on_frequency[last_idx] += ktime_to_us(time_delta);
		dvfs_stats.count_on_frequency[idx]++;
		//printk(KERN_INFO "%s: dvfs_stats.time_on_frequency[%d] =%lld\n", __func__,last_idx,dvfs_stats.time_on_frequency[last_idx]);
		dvfs_stats.current_idx = idx;
#endif // CONFIG_DEBUG_FS
		cpu_dynamic_clk = new_speed/1000; //(current clock in M)
	}
#ifdef CONFIG_PERF_EVENTS
	strcpy((char *)&trace_counter_name, "cpu_frequency");
	//cpu_clk = elite_get_clk_cpu();
	trace_counter =cpu_clk->rate*1000;
	counter_output((char *)&trace_counter_name,trace_counter,0);
#endif

out:
	mutex_unlock(&elite_cpu_lock);
	return ret;
}


static int elite_pm_notify(struct notifier_block *nb, unsigned long event,
		void *dummy)
{
	printk(KERN_DEBUG "%s: \n", __func__);
	mutex_lock(&elite_cpu_lock);
	if (event == PM_SUSPEND_PREPARE) {
		is_suspended = true;
		pr_info("Elite cpufreq suspend: setting frequency to %d kHz\n",
				freq_table[0].frequency);
		//elite_update_cpu_speed(freq_table[0].frequency);
	} else if (event == PM_POST_SUSPEND) {
		is_suspended = false;
	}
	mutex_unlock(&elite_cpu_lock);

	return NOTIFY_OK;
}

static struct notifier_block elite_cpu_pm_notifier = {
	.notifier_call = elite_pm_notify,
};

static int elite_cpu_init(struct cpufreq_policy *policy)
{

	printk(KERN_DEBUG "%s: \n", __func__);
	if (is_suspended) {
		return 0;
	}
	if (policy->cpu >= NUM_CPUS)
		return -EINVAL;

	cpu_clk = clk_get_sys(NULL, "cpu");
	//------patch--------------
	if (IS_ERR(cpu_clk))
	{
		//elite_cpu_init_clocks(10);
		cpu_clk = clk_get_sys(NULL, "cpu");
		printk(KERN_DEBUG "%s:patch successful cpu-clk = %x \n", __func__,(unsigned int)cpu_clk);
		return PTR_ERR(cpu_clk);
	}
	elite_cpu_init_clocks(10);
	dvfs_stats.current_idx = 10;
	cpu_dynamic_clk = cpu_clk->rate/1000; //(current clock in M)
	if (IS_ERR(cpu_clk))
	{
		printk(KERN_DEBUG "%s:cpu-clk = %x \n", __func__,(unsigned int)cpu_clk);
		return PTR_ERR(cpu_clk);
	}


	clk_enable(cpu_clk);
	freq_table = build_cpufreq_table( cpu_clk->u.table_data);//traslate elite frequncy table to cpu frequncy table 

	cpufreq_frequency_table_cpuinfo(policy, freq_table); 
	cpufreq_frequency_table_get_attr(freq_table, policy->cpu);//save table to OS, all CPUs share same table
	policy->cur = elite_getspeed(0);
	target_cpu_speed[policy->cpu] = policy->cur;
	min_frequency = policy->min;
	max_frequency = policy->max;
	cpu_clk->max_rate = policy->max;
	cpu_clk->min_rate = policy->min;
	cpu_clk->oldrate = cpu_clk->rate;

	printk(KERN_DEBUG "%s:policy->min = %d policy->max = %d \n", __func__,(unsigned int)policy->min,(unsigned int)policy->max);

	/* FIXME: what's the actual transition time? */
	policy->cpuinfo.transition_latency = 100 * 1000;

	policy->shared_type = CPUFREQ_SHARED_TYPE_ALL;
	cpumask_copy(policy->related_cpus, cpu_possible_mask);

	if (policy->cpu == 0) {
		register_pm_notifier(&elite_cpu_pm_notifier);
	}

	time_last_onoff = ktime_get();
	return 0;
}

static int elite_cpu_exit(struct cpufreq_policy *policy)
{
	printk(KERN_DEBUG "%s: start\n", __func__);
	printk(KERN_DEBUG "%s: &freq_table= %x\n", __func__,(unsigned int)&freq_table);
	cpufreq_frequency_table_cpuinfo(policy, freq_table); //update policy according to freq_table
	printk(KERN_DEBUG "%s: end\n", __func__);
	return 0;
}

static struct freq_attr *elite_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
#ifdef CONFIG_ELITE_THERMAL_THROTTLE
	&throttle,
#endif
	NULL,
};

static struct cpufreq_driver elite_cpufreq_driver = {
	.verify		= elite_verify_speed,
	.target		= elite_target,
	.get		= elite_getspeed,
	.init		= elite_cpu_init,
	.exit		= elite_cpu_exit,
	.name		= "elite",
	.attr		= elite_cpufreq_attr,
};

static int __init elite_cpufreq_init(void)
{
	printk(KERN_DEBUG "elite_cpufreq_init\n");
#ifdef CONFIG_ELITE_THERMAL_THROTTLE
	/*
	 * High-priority, others flags default: not bound to a specific
	 * CPU, has rescue worker task (in case of allocation deadlock,
	 * etc.).  Single-threaded.
	 */
	workqueue = alloc_workqueue("cpu-elite",
			WQ_HIGHPRI | WQ_UNBOUND | WQ_RESCUER, 1);
	if (!workqueue)
		return -ENOMEM;
	printk(KERN_DEBUG "elite_cpufreq_init before INIT_DELAYED_WORK\n");
	INIT_DELAYED_WORK(&throttle_work, elite_throttle_work_func);

	throttle_lowest_index = table_data->throttle_lowest_index;
	throttle_highest_index = table_data->throttle_highest_index;
#endif
	cpu_onoff_workqueue = alloc_workqueue("cpu_onoff-elite",
			WQ_HIGHPRI | WQ_UNBOUND | WQ_RESCUER, 1);
	if (!cpu_onoff_workqueue)
		return -ENOMEM;
	INIT_DELAYED_WORK(&cpu_onoff_work, cpu_onoff_work_func);


	//init_dvfs_stats();
	elite_cpufreq_subinit();
	return cpufreq_register_driver(&elite_cpufreq_driver);
}

static void __exit elite_cpufreq_exit(void)
{
#ifdef CONFIG_ELITE_THERMAL_THROTTLE
	destroy_workqueue(workqueue);
#endif
	cpufreq_unregister_driver(&elite_cpufreq_driver);
}


MODULE_AUTHOR("Colin Cross <ccross@android.com>");
MODULE_DESCRIPTION("cpufreq driver for elite 1000");
MODULE_LICENSE("GPL");
module_init(elite_cpufreq_init);
module_exit(elite_cpufreq_exit);

#endif //#ifdef CONFIG_CPU_FREQ


