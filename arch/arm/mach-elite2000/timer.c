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

#include <asm/div64.h>
#include <asm/mach/irq.h>
#include <asm/localtimer.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include "board.h"
#include <asm/smp_twd.h>
#include <asm/mach/time.h>
#include <asm/sched_clock.h>
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

static void __init elite_twd_init(void)
{
	int err = twd_local_timer_register(&twd_local_timer);
	if (err)
		pr_err("twd_local_timer_register failed %d\n", err);
}
#else
#define elite_twd_init()	do {} while(0)
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
	return timer_readl(PMU_TIMER1 + PMU_TIMER_TC);
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
	.name 		= "elite_timer0",
	.flags	 	= IRQF_DISABLED | IRQF_TIMER | IRQF_TRIGGER_HIGH,
	.handler	= elite_timer_interrupt,
	.dev_id		= &elite_clkevt_device,
	.irq		= IRQ_NS_TIMER0,
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

	setup_sched_clock(elite_read_sched_clock, 32, 1000000);

	if (clocksource_mmio_init(timer_reg_base + PMU_TIMER1 + PMU_TIMER_TC, 
		"elite_timer1", 1000000, 300, 32, clocksource_mmio_readl_up)) {
		printk(KERN_ERR "Failed to register clocksource\n");
		BUG();
	}
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
}

#define elite_timer_suspend NULL
#define elite_timer_resume NULL



#endif


struct sys_timer elite_timer = {
	.init		= elite_timer_init,
	.suspend	= elite_timer_suspend,
	.resume		= elite_timer_resume,
};


