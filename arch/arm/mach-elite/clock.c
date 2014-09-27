/*
 * Copyright (C) 2012 S3 Graphics, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include "clock.h"

/* clock information */
static DEFINE_MUTEX(clock_list_lock);
static LIST_HEAD(clocks);

struct clk *elite_get_clock_by_name(const char *name)
{
	struct clk *c;
	struct clk *ret = NULL;
	mutex_lock(&clock_list_lock);
	list_for_each_entry(c, &clocks, node) {
		if (strcmp(c->name, name) == 0) {
			ret = c;
			break;
		}
	}
	mutex_unlock(&clock_list_lock);
	return ret;
}

static unsigned long elite_calc_rate_from_parent(struct clk *c, struct clk *p)
{
	unsigned long rate;

	rate = clk_get_rate(p);
	if (c->div != 0) {
		rate += c->div - 1; /* round up */
		//do_div(rate, c->div);
		rate /= c->div; 
	}

	return rate;
}

unsigned long elite_get_rate(struct clk *c)
{
	unsigned long rate;

	if (c->parent)
		rate = elite_calc_rate_from_parent(c, c->parent);
	else
		rate = c->rate;

	return rate;
}

unsigned long clk_get_rate(struct clk *c)
{
	unsigned long flags;
	unsigned long rate;

	spin_lock_irqsave(&c->spinlock, flags);

	rate = elite_get_rate(c);

	spin_unlock_irqrestore(&c->spinlock, flags);

	return rate;
}
EXPORT_SYMBOL(clk_get_rate);

int clk_enable(struct clk *c)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&c->spinlock, flags);

	if (c->parent) {
		ret = clk_enable(c->parent);
		if (ret)
			goto out;
	}

	if (c->ops && c->ops->enable) {
		ret = c->ops->enable(c);
#if 0
		if (ret) {
			if (c->parent)
				clk_disable(c->parent);
			goto out;
		}
#endif
	}
out:
	spin_unlock_irqrestore(&c->spinlock, flags);
	return ret;
}
EXPORT_SYMBOL(clk_enable);


void clk_disable(struct clk *c)
{
	unsigned long flags;

	spin_lock_irqsave(&c->spinlock, flags);

	if (c->ops && c->ops->disable)
		c->ops->disable(c);
#if 0
	if (c->parent)
		clk_disable(c->parent);
#endif

	spin_unlock_irqrestore(&c->spinlock, flags);
}
EXPORT_SYMBOL(clk_disable);

int elite_clk_reparent(struct clk *c, struct clk *parent)
{
	c->parent = parent;
	return 0;
}

int clk_set_parent(struct clk *c, struct clk *parent)
{
	int ret;
	unsigned long flags;
	unsigned long new_rate;
	unsigned long old_rate;

	spin_lock_irqsave(&c->spinlock, flags);

	if (!c->ops || !c->ops->set_parent) {
		ret = -ENOSYS;
		goto out;
	}

	new_rate = elite_calc_rate_from_parent(c, parent);
	old_rate = elite_get_rate(c);

	ret = c->ops->set_parent(c, parent);
	if (ret)
		goto out;

out:
	spin_unlock_irqrestore(&c->spinlock, flags);
	return ret;
}
EXPORT_SYMBOL(clk_set_parent);

void elite_clk_init(struct clk *c)
{
	spin_lock_init(&c->spinlock);

	if (c->ops && c->ops->init)
		c->ops->init(c);

	mutex_lock(&clock_list_lock);
	list_add(&c->node, &clocks);
	mutex_unlock(&clock_list_lock);
}

struct clk *clk_get_parent(struct clk *c)
{
	return c->parent;
}
EXPORT_SYMBOL(clk_get_parent);

int elite_set_rate(struct clk *c, unsigned long rate)
{
	long new_rate;

	if (!c->ops || !c->ops->set_rate)
		return -ENOSYS;

	if (rate > c->max_rate)
		rate = c->max_rate;

	if (c->ops && c->ops->round_rate) {
		new_rate = c->ops->round_rate(c, rate);

		if (new_rate < 0)
			return new_rate;

		rate = new_rate;
	}

	return c->ops->set_rate(c, rate);
}

int clk_set_rate(struct clk *c, unsigned long rate)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&c->spinlock, flags);

	ret = elite_set_rate(c, rate);

	spin_unlock_irqrestore(&c->spinlock, flags);

	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

long clk_round_rate(struct clk *c, unsigned long rate)
{
	unsigned long flags;
	long ret;

	spin_lock_irqsave(&c->spinlock, flags);

	if (!c->ops || !c->ops->round_rate) {
		ret = -ENOSYS;
		goto out;
	}
#if 0
	if (rate > c->max_rate)
		rate = c->max_rate;
#endif
	ret = c->ops->round_rate(c, rate);

out:
	spin_unlock_irqrestore(&c->spinlock, flags);
	return ret;
}
EXPORT_SYMBOL(clk_round_rate);



#ifdef CONFIG_DEBUG_FS

static int __clk_lock_all_spinlocks(void)
{
	struct clk *c;

	list_for_each_entry(c, &clocks, node)
		if (!spin_trylock(&c->spinlock))
			goto unlock_spinlocks;

	return 0;

unlock_spinlocks:
	list_for_each_entry_continue_reverse(c, &clocks, node)
		spin_unlock(&c->spinlock);

	return -EAGAIN;
}

static void __clk_unlock_all_spinlocks(void)
{
	struct clk *c;

	list_for_each_entry_reverse(c, &clocks, node)
		spin_unlock(&c->spinlock);
}

/*
 * This function retries until it can take all locks, and may take
 * an arbitrarily long time to complete.
 * Must be called with irqs enabled, returns with irqs disabled
 * Must be called with clock_list_lock held
 */
static void clk_lock_all(void)
{
	int ret;
retry:
	local_irq_disable();

	ret = __clk_lock_all_spinlocks();
	if (ret)
		goto failed_spinlocks;

	/* All locks taken successfully, return */
	return;

failed_spinlocks:
	local_irq_enable();
	yield();
	goto retry;
}

/*
 * Unlocks all clocks after a clk_lock_all
 * Must be called with irqs disabled, returns with irqs enabled
 * Must be called with clock_list_lock held
 */
static void clk_unlock_all(void)
{
	__clk_unlock_all_spinlocks();

	local_irq_enable();
}

static struct dentry *clk_debugfs_root;


static void clock_tree_show_one(struct seq_file *s, struct clk *c, int level)
{
	struct clk *child;
	char div[8] = {0};


	seq_printf(s, "%s - div: %u\n", c->name, c->div);

	list_for_each_entry(child, &clocks, node) {
		if (child->parent != c)
			continue;

		clock_tree_show_one(s, child, level + 1);
	}
}

static int clock_tree_show(struct seq_file *s, void *data)
{
	struct clk *c;
	seq_printf(s, "   clock                          state  ref div      rate\n");
	seq_printf(s, "--------------------------------------------------------------\n");

	mutex_lock(&clock_list_lock);

	clk_lock_all();

	list_for_each_entry(c, &clocks, node)
		if (c->parent == NULL)
			clock_tree_show_one(s, c, 0);

	clk_unlock_all();

	mutex_unlock(&clock_list_lock);
	return 0;
}

static int clock_tree_open(struct inode *inode, struct file *file)
{
	return single_open(file, clock_tree_show, inode->i_private);
}

static const struct file_operations clock_tree_fops = {
	.open		= clock_tree_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int possible_parents_show(struct seq_file *s, void *data)
{
/*
	struct clk *c = s->private;
	int i;

	for (i = 0; c->inputs[i].input; i++) {
		char *first = (i == 0) ? "" : " ";
		seq_printf(s, "%s%s", first, c->inputs[i].input->name);
	}
	seq_printf(s, "\n");
*/
	return 0;
}

static int possible_parents_open(struct inode *inode, struct file *file)
{
	return single_open(file, possible_parents_show, inode->i_private);
}

static const struct file_operations possible_parents_fops = {
	.open		= possible_parents_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int clk_debugfs_register_one(struct clk *c)
{
	struct dentry *d;

	d = debugfs_create_dir(c->name, clk_debugfs_root);
	if (!d)
		return -ENOMEM;
	c->dent = d;

	d = debugfs_create_u32("rate", S_IRUGO, c->dent, (u32 *)&c->rate);
	if (!d)
		goto err_out;

	d = debugfs_create_x32("voltage", S_IRUGO, c->dent, (u32 *)&c->voltage);
	if (!d)
		goto err_out;
/*
	if (c->inputs) {
		d = debugfs_create_file("possible_parents", S_IRUGO, c->dent,
			c, &possible_parents_fops);
		if (!d)
			goto err_out;
	}
*/

	return 0;

err_out:
	debugfs_remove_recursive(c->dent);
	return -ENOMEM;
}

static int clk_debugfs_register(struct clk *c)
{
	int err;
	struct clk *pa = c->parent;

	if (pa && !pa->dent) {
		err = clk_debugfs_register(pa);
		if (err)
			return err;
	}

	if (!c->dent) {
		err = clk_debugfs_register_one(c);
		if (err)
			return err;
	}
	return 0;
}

static int __init clk_debugfs_init(void)
{
	struct clk *c;
	struct dentry *d;
	int err = -ENOMEM;

	d = debugfs_create_dir("clock", NULL);
	if (!d)
		return -ENOMEM;
	clk_debugfs_root = d;

	d = debugfs_create_file("clock_tree", S_IRUGO, clk_debugfs_root, NULL,
		&clock_tree_fops);
	if (!d)
		goto err_out;

	list_for_each_entry(c, &clocks, node) {
		err = clk_debugfs_register(c);
		if (err)
			goto err_out;
	}
	return 0;
err_out:
	debugfs_remove_recursive(clk_debugfs_root);
	return err;
}

late_initcall(clk_debugfs_init);
#endif

