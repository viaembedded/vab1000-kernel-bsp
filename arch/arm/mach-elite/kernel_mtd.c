/* Copyright 2006 S3G
 *
 * Implementation by S3 Graphics Co.,Ltd.
 *
 * (C) Copyright 2008
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */
#include <asm/div64.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/mtd/mtd.h>
#include <linux/slab.h>
#include <linux/sched.h>
MODULE_LICENSE("Dual BSD/GPL");  
static int erase_eraseblock(struct mtd_info * mtd,loff_t offset,int size)
{
	int err;
	struct erase_info ei;
	//loff_t addr = ebnum * mtd->erasesize;
	memset(&ei, 0, sizeof(struct erase_info));
	ei.mtd  = mtd;
	ei.addr = offset;
	//ei.len  = mtd->erasesize;
	ei.len  = (size+mtd->erasesize-1)&(~(mtd->erasesize-1));
	err = mtd_erase(mtd, &ei);
	if (err) {
		printk(KERN_ERR "error %d while erasing EB 0x%llx\n", err, (long long)offset);
		return err;
	}
	if (ei.state == MTD_ERASE_FAILED) {
		printk(KERN_ERR "some erase error occurred at EB 0x%llx\n",(long long)offset);
		return -EIO;
	}
	return 0;
}

static int is_block_bad(struct mtd_info * mtd,loff_t addr)
{
	int ret;
	ret = mtd_block_isbad(mtd, addr);
	if (ret)
		printk(KERN_ERR "block 0x%llx is bad\n", addr);
	return ret;
}

int partition_read(u32 partition_num,loff_t offset,u32 len,unsigned char *readbuf)
{
	int err;
	size_t read;
	struct mtd_info * mtd=NULL;
	mtd = get_mtd_device(NULL, partition_num);
	if (IS_ERR(mtd)) {
		err = PTR_ERR(mtd);
		printk(KERN_ERR"error: cannot get MTD device\n");
		return err;
	}
	if (mtd->writesize == 1) {
		printk(KERN_ERR"not NAND flash, assume page size is 512 bytes.\n");
		BUG();
	}
	while(offset <mtd->size)
	{
		if(is_block_bad(mtd,offset))
		{
			offset+=mtd->erasesize;
		}
		else
		{
			break;
		}
	}
	if(offset >=mtd->size)
	{
		printk(KERN_ERR"this mtd is full of bad blocks.\n");	
		return -EIO;
	}
	
	err = mtd_read(mtd, offset, len, &read, readbuf);
	if (mtd_is_bitflip(err))
		err = 0;
	if (unlikely(err || read != len)) {
		printk(KERN_ERR"error: read failed at 0x%llx\n", (long long)offset);
		if (!err)
			err = -EINVAL;
		return err;
	}
	return 0;
}

int partition_write(u32 partition_num,loff_t offset,u32 len,unsigned char *writebuf)
{
	int err;
	size_t read,written;
	struct mtd_info * mtd=NULL;
	mtd = get_mtd_device(NULL, partition_num);
	if (IS_ERR(mtd)) {
		err = PTR_ERR(mtd);
		printk(KERN_ERR "error: cannot get MTD device\n");
		return err;
	}
	if (mtd->writesize == 1) {
		printk(KERN_ERR "not NAND flash, assume page size is 512 bytes.\n");
		BUG();
	}
	while(offset <mtd->size)
	{
		if(is_block_bad(mtd,offset))
		{
			offset+=mtd->erasesize;
		}
		else
		{
			break;
		}
	}
	if(offset >=mtd->size)
	{
		printk(KERN_ERR"this mtd is full of bad blocks.\n");	
		return -EIO;
	}
	err = erase_eraseblock(mtd,offset,len);
	if (unlikely(err)) {
		return err;
	}
	len  = (len+mtd->writesize-1)&(~(mtd->writesize-1));
	err = mtd_write(mtd, offset, len, &written, writebuf);
	if (unlikely(err || written != len)) {
		printk(KERN_ERR "error: write failed at %#llx\n",(long long)offset);
		return err ? err : -1;
	}
	return 0;
}

EXPORT_SYMBOL(partition_read);
EXPORT_SYMBOL(partition_write);
