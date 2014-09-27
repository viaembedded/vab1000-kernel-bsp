/*
 * arch/arm/mach-elite/timer.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/sched.h>
#include <linux/cnt32_to_63.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/rtc.h>

#include <asm/div64.h>
#include <asm/mach/irq.h>
#include <asm/localtimer.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include "board.h"
#include <asm/smp_twd.h>
#include <asm/mach/time.h>
#include <asm/sched_clock.h>
#include <linux/proc_fs.h>
#include <linux/module.h>


/*
 * This is PXA's sched_clock implementation. This has a resolution
 * of at least 308 ns and a maximum value of 208 days.
 *
 * The return value is guaranteed to be monotonic in that range as
 * long as there is always less than 582 seconds between successive
 * calls to sched_clock() which should always be the case in practice.
 */

#define REG32_VAL(x) *((volatile unsigned int*)(x))

#ifdef CONFIG_HAVE_ARM_TWD

static DEFINE_TWD_LOCAL_TIMER(twd_local_timer, ELITE_ARM_PERIF_TWD_BASE, IRQ_LOCALTIMER);
extern int elite_rtc_read_time(struct device *dev, struct rtc_time *time);
static void __init elite_twd_init(void)
{
	int err = twd_local_timer_register(&twd_local_timer);
	if (err)
		pr_err("twd_local_timer_register failed %d\n", err);
}
#else
#define elite_twd_init()	do {} while(0)
#endif

#if 0
void read_persistent_clock(struct timespec *ts)
{
	struct rtc_time tm;
	unsigned long now;
	int status;

	status = elite_rtc_read_time(NULL, &tm);
	if (status < 0) {
		ts->tv_nsec=0;
		ts->tv_sec=now;
		return;
	}
	rtc_tm_to_time(&tm, &now);
	printk("read_persistent_clock\n");
	ts->tv_nsec=0;
	ts->tv_sec=now;
}
#endif

#ifdef CONFIG_ARM_GLOBAL_TIMER

#define GTCR_LOWER	REG32_VAL(IO_ADDRESS(0xd8018200))
#define GTCR_UPPER	REG32_VAL(IO_ADDRESS(0xd8018204))
#define GTCTR		REG32_VAL(IO_ADDRESS(0xd8018208))
#define GTISR		REG32_VAL(IO_ADDRESS(0xd801820c))
#define GTCVR_LOWER	REG32_VAL(IO_ADDRESS(0xd8018210))
#define GTCVR_UPPER	REG32_VAL(IO_ADDRESS(0xd8018214))

#define GTAIR		REG32_VAL(IO_ADDRESS(0xd8018218))

#define GTCTR_ENABLE        (1 << 0)
#define GTCTR_ONESHOT       (0 << 3)
#define GTCTR_PERIODIC      (1 << 3)
#define GTCTR_IT_ENABLE     (1 << 2)
#define GTCTR_COMP_ENABLE   (1 << 1)

static unsigned long timer_reload;
static unsigned long read_gtc(void)
{
	u64 pre_upper, upper, lower;
	pre_upper = GTCR_UPPER;
	do {
		lower = GTCR_LOWER;
		upper = GTCR_UPPER;
		if (upper != pre_upper)
			pre_upper = upper;
		else
			break;
	} while (1);
	return lower + (upper << 32);
}

static void write_cvr(u64 event_time)
{
	GTCTR &= ~0x02;
	GTCVR_LOWER = (unsigned long)event_time;
	GTCVR_UPPER = (unsigned long)(event_time >> 32);
	GTCTR |= 0x02;
	GTISR |= 0x1;
}

static void write_gtc(u64 event_time)
{
	GTCTR &= ~0x1;
	GTCR_LOWER = (unsigned long)event_time;
	GTCR_UPPER = (unsigned long)(event_time >> 32);
	GTCTR |= 0x1;
}

static irqreturn_t elite_glt_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;
	GTCTR &= ~0x04;
	GTISR |= 0x1;
	c->event_handler(c);
	GTCTR |= 0x04;

	return IRQ_HANDLED;
}

static int
elite_glt_set_next_event(unsigned long delta, struct clock_event_device *dev)
{
	u64 next;
	u64 cur;
	GTISR |= 0x1;
	cur = read_gtc();
	next = cur + delta;
	write_cvr(next);
	write_gtc(cur);
	GTCTR |= 0x04;

	return 0;
}

static void
elite_glt_set_mode(enum clock_event_mode mode, struct clock_event_device *dev)
{
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		/* timer load already set up */
		GTCTR = GTCTR_ENABLE | GTCTR_IT_ENABLE
		        | GTCTR_PERIODIC | GTCTR_COMP_ENABLE;
		GTAIR = timer_reload;
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		/* period set, and timer enabled in 'next_event' hook */
		GTCTR = GTCTR_ENABLE | GTCTR_ONESHOT | GTCTR_IT_ENABLE;
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		break;
	}
}

static cycle_t elite_clocksource_read(struct clocksource *cs)
{
	return cnt32_to_63(read_gtc());
}

static struct clocksource elite_clocksource = {
	.name	= "timer_elite_source",
	.rating	= 300,
	.read	= elite_clocksource_read,
	.mask	= 0x7FFFFFFFFFFFFFFFULL,
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

#if 0
unsigned long long sched_clock(void)
{
	return clocksource_cyc2ns(elite_clocksource.read(&elite_clocksource),
	                          elite_clocksource.mult, elite_clocksource.shift);
}
#endif

#define elite_timer_suspend NULL
#define elite_timer_resume NULL

cycle_t elite_read_oscr(void)
{
	return cnt32_to_63(read_gtc());
}

static struct clock_event_device __percpu *elite_glt;


static void elite_timer_init(void)
{
	unsigned long clock_tick_rate = 12000000;
	unsigned long flags;

	struct clock_event_device *ckevt_elite_glt;

	elite_glt = alloc_percpu(struct clock_event_device);

	ckevt_elite_glt = this_cpu_ptr(elite_glt);

	if (ckevt_elite_glt == NULL) {
		printk("allocate memory for global timer fail!\n");
	}

	ckevt_elite_glt->name = "glt";
	ckevt_elite_glt->features = CLOCK_EVT_FEAT_PERIODIC;
	ckevt_elite_glt->shift = 20;
	ckevt_elite_glt->rating = 200;
	ckevt_elite_glt->set_next_event = elite_glt_set_next_event;
	ckevt_elite_glt->set_mode = elite_glt_set_mode;
	ckevt_elite_glt->irq = 27;

	GTCTR = 0x01 | 0x100;
	GTISR |= 0x1;

	write_gtc(0x0ULL);
	write_cvr(0x40000ULL);

	timer_reload = clock_tick_rate / HZ;
	ckevt_elite_glt->mult =
	    div_sc(clock_tick_rate, NSEC_PER_SEC, ckevt_elite_glt->shift);
	ckevt_elite_glt->max_delta_ns =
	    clockevent_delta2ns(0xffffffff, ckevt_elite_glt);
	ckevt_elite_glt->min_delta_ns =
	    clockevent_delta2ns(0xf, ckevt_elite_glt);
	ckevt_elite_glt->cpumask = cpumask_of(0);

	local_irq_save(flags);
	request_percpu_irq(27, elite_glt_interrupt, "glt", elite_glt);
	enable_percpu_irq(27, 0);
	local_irq_restore(flags);

	GTCTR |= 0x4;
	clockevents_register_device(ckevt_elite_glt);

#if 1
	if (clocksource_register_hz(&elite_clocksource, clock_tick_rate)) {
		printk(KERN_ERR "Failed to register clocksource\n");
		BUG();
	}
#endif
	elite_twd_init();

}


#else

#define ARM_GLOBAL_TIME

#define GT_COUNTER0	(IO_ADDRESS(0xd8018200))
#define GT_COUNTER1	(IO_ADDRESS(0xd8018204))
#define GT_CONTROL	(IO_ADDRESS(0xd8018208))
#define GT_CONTROL_TIMER_ENABLE         BIT(0)  /* this bit is NOT banked */
#define GT_CONTROL_COMP_ENABLE          BIT(1)  /* banked */
#define GT_CONTROL_IRQ_ENABLE           BIT(2)  /* banked */
#define GT_CONTROL_AUTO_INC             BIT(3)  /* banked */

/* #define DEBUG_TIMER_DELTA */

#define ARMTIMER_TO_NS(t) ((t)<<1)
#define PMUIMER_TO_NS(t) ((t)*1000)


static u64 gt_counter_read(void)
{
	u64 counter;
	u32 lower;
	u32 upper, old_upper;

	upper = readl_relaxed(GT_COUNTER1);
	do {
		old_upper = upper;
		lower = readl_relaxed(GT_COUNTER0);
		upper = readl_relaxed(GT_COUNTER1);
	} while (upper != old_upper);

	counter = upper;
	counter <<= 32;
	counter |= lower;
	return counter;
}

static cycle_t gt_clocksource_read(struct clocksource *cs)
{
	cycle_t res = gt_counter_read();
#ifdef DEBUG_TIMER_DELTA
	if(unlikely(clocksource_cyc2ns((res - cs->cycle_last) & cs->mask, cs->mult, cs->shift)  > NSEC_PER_SEC)) {
		printk("----- Detect clock source error -------\n");
		printk("CS last %Lu,  new %Lu (cpu = %d)\n", cs->cycle_last, res , smp_processor_id());
		printk("----- Detect clock source error -------\n");
	}
#endif
	return res;
}

static u64 suspend_gt_count;
void gt_suspend(struct clocksource *cs)
{
	suspend_gt_count = cs->read(cs);
}
void gt_resume(struct clocksource *cs)
{
	writel(0, GT_CONTROL);
	writel((u32)(suspend_gt_count&0xffffffff), GT_COUNTER0);
	writel((u32)((suspend_gt_count  >> 32)&0xffffffff), GT_COUNTER1);
	/* enables timer on all the cores */
	writel(GT_CONTROL_TIMER_ENABLE, GT_CONTROL);
}


static struct clocksource gt_clocksource = {
	.name   = "arm_global_timer",
	.suspend = gt_suspend,
	.resume = gt_resume,
	.rating = 300,
	.read   = gt_clocksource_read,
	.mask   = CLOCKSOURCE_MASK(64),
	.flags  = CLOCK_SOURCE_IS_CONTINUOUS,
};

static void  gt_clocksource_init(void)
{
	printk("Register arm global time source\n");
	writel(0, GT_CONTROL);
	writel(0, GT_COUNTER0);
	writel(0, GT_COUNTER1);
	/* enables timer on all the cores */
	writel(GT_CONTROL_TIMER_ENABLE, GT_CONTROL);

	clocksource_register_hz(&gt_clocksource, 500*1000000);
}


#define PMU_TIMER_BASE 	0x4000
#define PMU_TIMER0 	0x0
#define PMU_TIMER1 	0x28

#define PMU_TIMER_IR	0x0
#define PMU_TIMER_TCR	0x4
#define PMU_TIMER_TC	0x8
#define PMU_TIMER_PR	0xc
#define PMU_TIMER_PC	0x10
#define PMU_TIMER_MR0	0x14
#define PMU_TIMER_MR1	0x18
#define PMU_TIMER_MR2	0x1C
#define PMU_TIMER_MR3	0x20
#define PMU_TIMER_MCR	0x24

static void __iomem *timer_reg_base = IO_ADDRESS(ELITE_PMU_BASE+PMU_TIMER_BASE);

static void timer_writel(unsigned int val, unsigned int offset)
{
	writel(val, timer_reg_base + offset);
}

static unsigned int timer_readl(unsigned int offset)
{
	return readl(timer_reg_base + offset);
}

static unsigned int elite_read_sched_clock(void)
{
	return timer_readl(PMU_TIMER1 + PMU_TIMER_TC);
}

cycle_t elite_read_oscr(void)
{
#ifdef ARM_GLOBAL_TIME
	return (gt_counter_read() >> 9 );  /* >>9 == 500*/
#else
	return timer_readl(PMU_TIMER1 + PMU_TIMER_TC);
#endif
}


static int elite_timer_set_next_event(unsigned long cycles, struct clock_event_device *evt)
{
	timer_writel(0x2, PMU_TIMER0 + PMU_TIMER_TCR);

	if (cycles == 0) {
		/* set match register to 0 will cause timer has no interrupt , so patch it*/
		cycles += 1;
	}
	timer_writel(cycles, PMU_TIMER0 + PMU_TIMER_MR0);
	timer_writel(0x1, PMU_TIMER0 + PMU_TIMER_TCR);
	return 0;
}

static void elite_timer_set_mode(enum clock_event_mode mode, struct clock_event_device *evt)
{
	timer_writel(0x2, PMU_TIMER0 + PMU_TIMER_TCR);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		timer_writel(3, PMU_TIMER0 + PMU_TIMER_MCR);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		timer_writel(5, PMU_TIMER0 + PMU_TIMER_MCR);
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		break;
	case CLOCK_EVT_MODE_RESUME:
		timer_writel(0x2, PMU_TIMER1 + PMU_TIMER_TCR);
		timer_writel(0x0, PMU_TIMER1 + PMU_TIMER_PR);
		timer_writel(0x1, PMU_TIMER1 + PMU_TIMER_TCR);
		timer_writel(3, PMU_TIMER0 + PMU_TIMER_MCR);
		timer_writel(2, PMU_TIMER0 + PMU_TIMER_TCR);
		timer_writel(0x0, PMU_TIMER0 + PMU_TIMER_PR);
		timer_writel(10000, PMU_TIMER0 + PMU_TIMER_MR0);
		break;
	default:
		break;
	}
	timer_writel(0x1, PMU_TIMER0 + PMU_TIMER_TCR);
}

static irqreturn_t elite_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = (struct clock_event_device *)dev_id;
	timer_writel(1, PMU_TIMER0 + PMU_TIMER_IR);
	evt->event_handler(evt);
	return IRQ_HANDLED;
}


static struct clock_event_device elite_clkevt_device = {
	.name = "elite_timer",
	.rating = 300,
	.features = CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC,
	.set_next_event = elite_timer_set_next_event,
	.set_mode = elite_timer_set_mode,
};

static struct irqaction elite_timer_irq = {
	.name 	= "elite_timer0",
	.flags	= IRQF_DISABLED | IRQF_TIMER | IRQF_TRIGGER_HIGH,
	.handler	= elite_timer_interrupt,
	.dev_id	= &elite_clkevt_device,
	.irq		= IRQ_NS_TIMER0,
};

#define SEQ_printf(m, x...)			\
 do {						\
	if (m)					\
		seq_printf(m, x);		\
	else					\
		printk(x);			\
 } while (0)

extern int elite_rtc_read_time(struct device *dev, struct rtc_time *tm);
extern int tick_do_timer_cpu;
extern ktime_t tick_next_period;
extern ktime_t tick_period;


static ssize_t timer_test_show(struct seq_file *m, void *v)
{

	static u64 jiffies64_last = 0;
	static uint pmu_last = 0;
	static u64  sched_clk_last = 0;
	static u64 arm_last = 0;
	static struct rtc_time rtc_last;
	static ktime_t mono_last, real_last, boot_last;

	u64 jiff_new;
	uint pmu_new;
	u64  sched_clk_new;
	u64 arm_new;
	struct rtc_time rtc_new;
	ulong rtc_tm1, rtc_tm2;
	ktime_t mono_new, real_new, boot_new;

	jiff_new = jiffies;
	pmu_new = timer_readl(PMU_TIMER1 + PMU_TIMER_TC);
	sched_clk_new= sched_clock();
	arm_new = gt_counter_read();
	elite_rtc_read_time(NULL, &rtc_new);
	rtc_tm_to_time(&rtc_last, &rtc_tm1);
	rtc_tm_to_time(&rtc_new, &rtc_tm2);
	mono_new = ktime_get();
	real_new = ktime_get_real();
	boot_new = ktime_get_boottime();


	SEQ_printf(m, "tick_do_timer_cpu %d, tick_next_period %Lu ns, tick_period %Lu ns\n",
	           tick_do_timer_cpu,
	           ktime_to_ns(tick_next_period),
	           ktime_to_ns(tick_period));

	SEQ_printf(m, "Interval jiffies: %Ld HZ, pmu: %Ld ns, arm: %Ld ns, sched clock: %Ld ns, rtc: %d s\n",
	           (s64)(jiff_new - jiffies64_last),
	           PMUIMER_TO_NS((s64)(pmu_new - pmu_last)),
	           ARMTIMER_TO_NS((s64)(arm_new - arm_last)),
	           (s64)(sched_clk_new - sched_clk_last),
	           (s32)(rtc_tm2 - rtc_tm1));

	SEQ_printf(m, "Current  jiffies: %Lu HZ, pmu: %Lu ns, arm: %Lu ns, sched clock: %Lu ns, rtc: %u s(%4d-%02d-%02d %02d:%02d:%02d)\n",
	           jiff_new,
	           PMUIMER_TO_NS((u64)pmu_new),
	           ARMTIMER_TO_NS(arm_new),
	           sched_clk_new,
	           (uint)rtc_tm2,
	           rtc_new.tm_year + 1900 , rtc_new.tm_mon + 1, rtc_new.tm_mday, rtc_new.tm_hour, rtc_new.tm_min, rtc_new.tm_sec);

	SEQ_printf(m , "Interval monotonic time %Ld ns, real time %Ld ns, boot time %Ld ns\n",
	           (s64)(ktime_to_ns(mono_new) - ktime_to_ns(mono_last)),
	           (s64)(ktime_to_ns(real_new) - ktime_to_ns(real_last)),
	           (s64)(ktime_to_ns(boot_new)- ktime_to_ns(boot_last)));

	SEQ_printf(m , "Current  monotonic time %Lu ns, real time %Lu ns, boot time %Lu ns\n",
	           (u64)(ktime_to_ns(mono_new)),
	           (u64)(ktime_to_ns(real_new)),
	           (u64)(ktime_to_ns(boot_new)));

	jiffies64_last = jiff_new;
	pmu_last = pmu_new;
	sched_clk_last = sched_clk_new;
	arm_last = arm_new;
	rtc_last = rtc_new;

	real_last = real_new;
	mono_last = mono_new;
	boot_last = boot_new;
	return 0;
}

static int proc_timer_test_open(struct inode *inode, struct file *file)
{
	return single_open(file, timer_test_show,NULL);
}
static struct proc_dir_entry *timer_test_proc_entry = NULL;
static const struct file_operations timer_test_proc_ops = {
	.owner = THIS_MODULE,
	.open = proc_timer_test_open,
	.read = seq_read,
	.release = single_release,
	.write = NULL,
};

static void elite_timer_init(void)
{
	struct clk *clk;
	unsigned long rate;
	int ret;

	clk = clk_get_sys(NULL, "timer");

	if (IS_ERR(clk)) {
		printk("unable to get timer clock, assuming 1Mhz input clock\n");
		rate = 1000000;
	} else {
		clk_enable(clk);
		rate = clk_get_rate(clk);
	}

	/* tiemr 1 as free running timer */
	timer_writel(0x2, PMU_TIMER1 + PMU_TIMER_TCR);
	timer_writel(0x0, PMU_TIMER1 + PMU_TIMER_PR);
	timer_writel(0x1, PMU_TIMER1 + PMU_TIMER_TCR);

//	setup_sched_clock(elite_read_sched_clock, 32, 1000000);
#ifdef ARM_GLOBAL_TIME
	gt_clocksource_init();
#else
	if (clocksource_mmio_init(timer_reg_base + PMU_TIMER1 + PMU_TIMER_TC,
	                          "elite_timer1", 1000000, 300, 32, clocksource_mmio_readl_up)) {
		printk(KERN_ERR "Failed to register clocksource\n");
		BUG();
	}
#endif

	timer_writel(0x2, PMU_TIMER0 + PMU_TIMER_TCR);
	timer_writel(0x0, PMU_TIMER0 + PMU_TIMER_PR);
	timer_writel(10000, PMU_TIMER0 + PMU_TIMER_MR0);
	timer_writel(0x1, PMU_TIMER0 + PMU_TIMER_TCR);

	ret = setup_irq(elite_timer_irq.irq, &elite_timer_irq);

	if (ret) {
		printk(KERN_ERR "Failed to register timer IRQ: %d\n", ret);
		BUG();
	}

	clockevents_calc_mult_shift(&elite_clkevt_device, 1000000, 5);
	elite_clkevt_device.max_delta_ns = clockevent_delta2ns(0x1fffffff, &elite_clkevt_device);
	elite_clkevt_device.min_delta_ns = clockevent_delta2ns(0x1, &elite_clkevt_device);
	elite_clkevt_device.cpumask = cpu_all_mask;
	elite_clkevt_device.irq = elite_timer_irq.irq;
	clockevents_register_device(&elite_clkevt_device);
	elite_twd_init();

	timer_test_proc_entry = proc_create_data("timer_test", 0444, NULL, &timer_test_proc_ops, NULL);
}

#define elite_timer_suspend NULL
#define elite_timer_resume NULL

#endif


struct sys_timer elite_timer = {
	.init		= elite_timer_init,
	.suspend	= elite_timer_suspend,
	.resume	= elite_timer_resume,
};


