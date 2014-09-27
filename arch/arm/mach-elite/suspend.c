#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/suspend.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/hardware/gic.h>
#include <asm/pgalloc.h>
#include <asm/tlbflush.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include "cpu-elite.h"
#include "elite-pg.h"

//;elite1k-520016c-YSW-05 - start
#include <linux/interrupt.h>			
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/mm.h>

#include <linux/input.h>
#define KPAD_POWER_FUNCTION_NUM  1
#define power_button_timeout 10
static struct input_dev *kpadPower_dev;
static unsigned int kpadPower_codes[KPAD_POWER_FUNCTION_NUM] = {
        [0] = KEY_POWER
};
static unsigned int powerKey_is_pressed;
static unsigned int pressed_jiffies;
static struct timer_list   kpadPower_timer;
static spinlock_t kpadPower_lock;
//;elite1k-520016c-YSW-05 - end

//unsigned long elite_pgd_phys;
//static pgd_t *elite_pgd;
extern void __cortex_a9_restore(void);
extern void elite_pg_startup(void);
extern void elite_set_pg_timer(unsigned int, int);
extern void elite_turn_on_pg_timer(unsigned int, int);
extern void elite_pcie_ports_suspend(void);
extern void elite_pcie_ports_resume();
extern int elite_dma_suspend(void);
extern int elite_dma_resume(void);
//void *elite_context_area = NULL;

void elite_cpu_wfi_cfg(int cpu, int mode)
{
        char *cpu_wfi_cfg_addr;

        cpu_wfi_cfg_addr = (char*)ELITE_PMU_BASE;

        switch(cpu) {
        case 0:
            cpu_wfi_cfg_addr += CORE0_WFI_CFG;
            break;
        case 1:
            cpu_wfi_cfg_addr += CORE1_WFI_CFG;
            break;
        case 2:
            cpu_wfi_cfg_addr += CORE2_WFI_CFG;
            break;
        case 3:
            cpu_wfi_cfg_addr += CORE3_WFI_CFG;
            break;
        default:
            printk("elite_cpu_wfi_cfg: unsupported cpu!\n");
            BUG();
        }
    
        writel(mode, (void*)cpu_wfi_cfg_addr);
}

void elite_sys_lp_cfg0(union sys_lp_cfg0 cfg)
{
        void *sys_lp_cfg0_addr;

        sys_lp_cfg0_addr = (void*)((char*)ELITE_PMU_BASE + SYS_LP_CFG0);

        writel(cfg.val, sys_lp_cfg0_addr);
}

void elite_sys_lp_cfg1(union sys_lp_cfg1 cfg)
{
        void *sys_lp_cfg1_addr;

        sys_lp_cfg1_addr = (void*)((char*)ELITE_PMU_BASE + SYS_LP_CFG1);

        writel(cfg.val, sys_lp_cfg1_addr);
}

void elite_stm_cfg(union stm_cfg cfg)
{
        void *stm_cfg_addr;

        stm_cfg_addr = (void*)((char*)ELITE_PMU_BASE + STM_CFG);

        writel(cfg.val, stm_cfg_addr);
}

void elite_sys_power_gating(void)
{
        unsigned long orig = 0;

        void *pg_mcboot_addr;

        pg_mcboot_addr = (void*)ELITE_PMU_BASE;
        
        pg_mcboot_addr += MCSBOOT0;
        
        orig = readl(pg_mcboot_addr);
        
        writel(virt_to_phys(elite_pg_startup), pg_mcboot_addr);

        stop_critical_timings();
        flush_cache_all();
        barrier();
  
        __cortex_a9_save(virt_to_phys(elite_pg_startup),6);

        barrier();
		
        start_critical_timings();

        writel(orig, pg_mcboot_addr);
}

#if 0
static void elite_suspend_lps4(void)
{
	int i;
	unsigned int pmu_cfg;

	for(i = 0; i < NR_CPUS; i++)
	{
		elite_cpu_wfi_cfg(i, WFI_CFG_ENTER_SYS_PWR_GATING_MODE);
	}

	pmu_cfg = readl((void*)(ELITE_PMC_BASE + PMU_CFG));

	pmu_cfg |= 0x4;

	writel(pmu_cfg, (void*)(ELITE_PMC_BASE + PMU_CFG));

	elite_sys_power_gating();

	for(i = 0; i < NR_CPUS; i++)
	{
		elite_cpu_wfi_cfg(i, WFI_CFG_KEEP_ORIGINAL_ARM_BEHAVIOR);
	}

}
#endif
static void elite_suspend_dram(void)
{
	unsigned int irq_status;
	int i;
	unsigned int pmu_cfg;
        union stm_cfg cfg;

	for(i = 0; i < NR_CPUS; i++)
	{
		elite_cpu_wfi_cfg(i, WFI_CFG_ENTER_STM_MODE);
	}

        /*stm config */
        cfg.val = 1;
        elite_stm_cfg(cfg);

	/* power group mask enable and pg sync enable */
	pmu_cfg = readl((void*)((char*)ELITE_PMU_BASE + PMU_CFG));
	pmu_cfg |= (1<<10) | (1<<27);
	writel(pmu_cfg, (void*)((char*)ELITE_PMU_BASE + PMU_CFG));

	writel(0, (void*)((char*)ELITE_PMU_BASE + PWR_GROUP_MASK0));
	writel(0, (void*)((char*)ELITE_PMU_BASE + PWR_GROUP_MASK1));

	/* make sure the interrupt is cleared*/
	irq_status = readl((void*)((char*)ELITE_PMU_BASE + EINT0_WAKEUP_IRQ_STATUS));
	writel(irq_status , (void*)((char*)ELITE_PMU_BASE + EINT0_WAKEUP_IRQ_STATUS));
 
	elite_sys_power_gating();

	pmu_cfg = readl((void*)((char*)ELITE_PMU_BASE + PMU_CFG));
	pmu_cfg &= ~((1<<10) | (1<<27));
	writel(pmu_cfg, (void*)((char*)ELITE_PMU_BASE + PMU_CFG));

	for(i = 0; i < NR_CPUS; i++)
	{
		elite_cpu_wfi_cfg(i, WFI_CFG_KEEP_ORIGINAL_ARM_BEHAVIOR);
	}
}

void elite_enable_pmu()
{
	unsigned int pmu_cfg;

	pmu_cfg = readl((void*)((char*)ELITE_PMU_BASE + PMU_CFG));

	pmu_cfg |= 0x4;

	writel(pmu_cfg, (void*)((char*)ELITE_PMU_BASE + PMU_CFG));

}
//;elite1k-520016c-YSW-13 extern spinlock_t elite_cpu_lock;
//;elite1k-520016c-YSW-13 extern int elite_cpu_set_voltage(struct clk *c, unsigned int voltage_idx);
static int elite_suspend_begin(suspend_state_t state)
{

//;elite1k-520016c-YSW-13	int i,ret;
//;elite1k-520016c-YSW-13	mutex_lock(&elite_cpu_lock);
//;elite1k-520016c-YSW-13	for(i=0;i<3;i++)
//;elite1k-520016c-YSW-13	{
//;elite1k-520016c-YSW-13	printk("begin to set voltage in suspend:%d\n",i);
//;elite1k-520016c-YSW-13	ret=elite_cpu_set_voltage(NULL,9);
//;elite1k-520016c-YSW-13	if(0==ret)
//;elite1k-520016c-YSW-13	break;
//;elite1k-520016c-YSW-13	}
//;elite1k-520016c-YSW-13	mutex_unlock(&elite_cpu_lock);

        return 0;
        //return regulator_suspend_prepare(state);
}

static int elite_suspend_prepare_late(void)
{
	elite_pcie_ports_suspend();
	elite_dma_suspend();
	return 0;
}

static void elite_suspend_wake(void)
{   
	elite_pcie_ports_resume();
	elite_dma_resume();
	
	//;elite1k-520016e-GLU-01-start
   	unsigned long flags;
	spin_lock_irqsave(&kpadPower_lock, flags);

        input_report_key(kpadPower_dev, KEY_POWER, 1);
   	input_sync(kpadPower_dev);
	input_report_key(kpadPower_dev, KEY_POWER, 0);
	input_sync(kpadPower_dev);

	spin_unlock_irqrestore(&kpadPower_lock, flags);
	//;elite1k-520016e-GLU-01-end
}


extern void wakeupcfg_pre_suspend(void);
extern void wakeupcfg_post_resume(void);
static int elite_suspend_enter(suspend_state_t state)
{
	
	switch (state) {
	case PM_SUSPEND_MEM:
		wakeupcfg_pre_suspend();
		break;
	}
	//elite_suspend_lps4();
        elite_suspend_dram();
	/* Zzz.. wakeup from suspend to RAM */
	wakeupcfg_post_resume();

#if 0
	/* for test purpose */
        elite_cpu_wfi_cfg(0, WFI_CFG_ENTER_MULTICORE_LP_MODE);

	writel(0xa, (void*)((char*)ELITE_PMU_BASE+CPU_LP_CFG_CORE0));

	elite_set_pg_timer(10000, 0);
	elite_turn_on_pg_timer(0, 0);

	barrier();

	elite_sys_power_gating();

        elite_cpu_wfi_cfg(0, 0);

	writel(0x0, (void*)((char*)ELITE_PMU_BASE+CPU_LP_CFG_CORE0));
#endif

        return 0;
}
/*this function is moved to elite800.c  
static int elite_create_suspend_pgtable(void)
{
	int i;
	pmd_t *pmd;

	unsigned long addr_v[] = {
		(unsigned long)elite_context_area,
		(unsigned long)virt_to_phys(elite_pg_startup),
		(unsigned long)__cortex_a9_restore,
	};
	unsigned long addr_p[] = {
		(unsigned long)virt_to_phys(elite_context_area),
		(unsigned long)virt_to_phys(elite_pg_startup),
		(unsigned long)virt_to_phys(__cortex_a9_restore),
	};
	unsigned int flags = PMD_TYPE_SECT | PMD_SECT_AP_WRITE |
		PMD_SECT_WBWA | PMD_SECT_S;

	elite_pgd = pgd_alloc(&init_mm);
	if (!elite_pgd)
		return -ENOMEM;

	for (i=0; i<ARRAY_SIZE(addr_p); i++) {
		unsigned long v = addr_v[i];
		pmd = pmd_offset(elite_pgd + pgd_index(v), v);
		*pmd = __pmd((addr_p[i] & PGDIR_MASK) | flags);
		flush_pmd_entry(pmd);
		outer_clean_range(__pa(pmd), __pa(pmd + 1));
	}

	elite_pgd_phys = virt_to_phys(elite_pgd);
	__cpuc_flush_dcache_area(&elite_pgd_phys,
		sizeof(elite_pgd_phys));
	outer_clean_range(__pa(&elite_pgd_phys),
		__pa(&elite_pgd_phys+1));

	__cpuc_flush_dcache_area(&elite_context_area,
		sizeof(elite_context_area));
	outer_clean_range(__pa(&elite_context_area),
		__pa(&elite_context_area+1));

	return 0;
}
*/

//;elite1k-520016c-YSW-05 - start
/*===========================================================================*/
/*  EINTG_irq_handler*/
/**/
/*  return: 0*/
/*===========================================================================*/
static irqreturn_t
EINTG_irq_handler(int irq, void *dev_id)
{	

	unsigned int pmu_cfg;
	unsigned long flags;

//	printk("ysw_debug: %s :irq no.%d\n", __func__, irq);

	spin_lock_irqsave(&kpadPower_lock, flags);

	//read PWRBTN INT Status
	pmu_cfg = readl((void*)((char*)ELITE_PMU_BASE + 0xC6CC));
//;elite1k-520016e-YSW-01	printk("\n[%s] pmu_cfg = %x\n", __func__, pmu_cfg);

	if((pmu_cfg & 0x030000) == 0x010000) {	//Short Press		
		input_report_key(kpadPower_dev, KEY_POWER, 1);
		input_sync(kpadPower_dev);

		input_report_key(kpadPower_dev, KEY_POWER, 0);
		input_sync(kpadPower_dev);		

		printk("\n[%s]power key short press --> entering STM\n",__func__);

//;elite1k-520016e-YSW-01		pmu_cfg |= 0x010000;
//;elite1k-520016e-YSW-01-start
			pmu_cfg = 0x10000;
//;elite1k-520016e-YSW-01-end

	} else if ((pmu_cfg & 0x030000) == 0x020000){	//Long Press
		input_event(kpadPower_dev, EV_KEY, KEY_POWER, 2); // power key repeat
		input_sync(kpadPower_dev);

		printk("\n[%s]power key long press --> entering Power Menu\n",__func__);

//;elite1k-520016e-YSW-01		pmu_cfg |= 0x020000;
//;elite1k-520016e-YSW-01-start
			pmu_cfg = 0x20000;
//;elite1k-520016e-YSW-01-end
		
	}
//;elite1k-520016e-YSW-01-start
	else{
//		printk("\n[%s]Not power key interrupt, skip handling\n", __func__);
//		writel(pmu_cfg, (void*)((char*)ELITE_PMU_BASE + 0xC6CC));
		spin_unlock_irqrestore(&kpadPower_lock, flags);
		return IRQ_NONE;
	}
//;elite1k-520016e-YSW-01-end


	//disable_irq(IRQ_PMC_WAKEUP);
//;elite1k-520016e-YSW-01	spin_unlock_irqrestore(&kpadPower_lock, flags);	

	//clear PWRBTN INT Status
	writel(pmu_cfg, (void*)((char*)ELITE_PMU_BASE + 0xC6CC));

	spin_unlock_irqrestore(&kpadPower_lock, flags);		//;elite1k-520016e-YSW-01

	return IRQ_HANDLED;
}

//;elite1k-520016c-YSW-05 - end

static struct platform_suspend_ops elite_suspend_ops = {

        .valid        = suspend_valid_only_mem,
        .begin        = elite_suspend_begin,
        .prepare_late = elite_suspend_prepare_late,
        .wake         = elite_suspend_wake,
        .enter        = elite_suspend_enter
};

static int __init elite_init_suspend(void)
{
	printk(KERN_CRIT "%s: start \n", __func__);

//;elite1k-520016c-YSW-05 - start

	int i;

	kpadPower_dev = input_allocate_device();
	if(kpadPower_dev) {
		//Initial the static variable
		spin_lock_init(&kpadPower_lock);

		// Register an input event device. //
		set_bit(EV_KEY,kpadPower_dev->evbit);
		for (i = 0; i < KPAD_POWER_FUNCTION_NUM; i++)
		set_bit(kpadPower_codes[i], kpadPower_dev->keybit);

		kpadPower_dev->name = "kpadPower",
		kpadPower_dev->phys = "kpadPower",


		kpadPower_dev->keycode = kpadPower_codes;
		kpadPower_dev->keycodesize = sizeof(unsigned int);
		kpadPower_dev->keycodemax = KPAD_POWER_FUNCTION_NUM;

		//
		// For better view of /proc/bus/input/devices
		//
		kpadPower_dev->id.bustype = 0;
		kpadPower_dev->id.vendor  = 0;
		kpadPower_dev->id.product = 0;
		kpadPower_dev->id.version = 0;
		input_register_device(kpadPower_dev);
	} else
		printk("[wmt_pm_init]Error: No memory for registering Kpad Power\n");

//;elite1k-520016c-YSW-05 - end

	//elite_context_area = kzalloc(CONTEXT_SIZE_BYTES * MAXCPUS, GFP_KERNEL);

	//elite_create_suspend_pgtable();

	suspend_set_ops(&elite_suspend_ops);

	//elite_enable_pmu();

//;elite1k-520016c-YSW-05 - start
	request_irq(IRQ_EINTG, EINTG_irq_handler, IRQF_SHARED, "EINTG", kpadPower_dev);
//;elite1k-520016c-YSW-05 - end

	return 0;
} 

late_initcall(elite_init_suspend);
