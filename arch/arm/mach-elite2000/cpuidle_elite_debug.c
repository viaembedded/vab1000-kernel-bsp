#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <mach/io.h>

extern  unsigned long g_pg_state;
//=========================================================================================================
static struct kobject *parent;
static struct kobject *child_kset_cpu0,*child_kset_cpu1;
#define PG_CPU0_ADDR  (void __iomem*)IO_ADDRESS(0xD839C740)
#define PG_CPU1_ADDR   (void __iomem*)IO_ADDRESS(0xD839C750)

#define CG_CPU0_ADDR   (void __iomem*)IO_ADDRESS(0xD8391110)
#define CG_CPU1_ADDR   (void __iomem*)IO_ADDRESS(0xD839C710)
static ssize_t att_show(struct kobject *kobj,struct attribute *attr ,char *buf)
{
        unsigned long long count;
        if(!strcmp("cpu0",kobj->name))
        {
                if(!strcmp("cpu_pg_times",attr->name))
                        sprintf(buf,"%lld",*(unsigned long long*)PG_CPU0_ADDR);
                if(!strcmp("cpu_cg_times",attr->name))
                        sprintf(buf,"%lld",*(unsigned long long*)CG_CPU0_ADDR);
                count=*(unsigned long long*)CG_CPU0_ADDR;
                //printk("~~~~~~~~~~~~~~~~~~~~%x\n",readl(CG_CPU0_ADDR));
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

int power_hardware_debug_init(void)
{
        int err;

        parent = kobject_create_and_add("cpu_power_info",NULL);
        parent->ktype = &pg_state_ktype;
        err=sysfs_create_file(parent,&pg_state_att);
        if(err)
                return err;

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

        return 0;
}

