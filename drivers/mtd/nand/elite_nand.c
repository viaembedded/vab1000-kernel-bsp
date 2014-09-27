/*
 * NAND Controller Driver for S3 Elite SoC Chips
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/bbm.h>
#include <linux/reboot.h> 
#include <asm/io.h>
#include <asm/dma.h>
#include <asm/sizes.h>
#include <linux/clk.h>
#include <mach/irqs.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/memory.h>
#include <asm/highmem.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/sizes.h>
#include "elite_nand.h"

/*
* NAND_PAGE_SIZE = 2048 or 4096:
* support Harming ECC and BCH ECC
*
* NAND_PAGE_SIZE = 512:
* Only support Harming ECC according to the demand of filesystem
* so need open macro:
* #define NAND_HARMING_ECC
*
*/
//#define  NAND_DEBUG
#ifdef NAND_DEBUG
#define NAND_PRINTF(fmt, args...)   printk(KERN_INFO"%s: " fmt, __func__ , ## args)      
#define ENTER() printk(KERN_INFO"Enter %s, file:%s line:%d\n", __func__, __FILE__, __LINE__)    
#define LEAVE() printk(KERN_INFO"Exit %s, file:%s line:%d\n", __func__, __FILE__, __LINE__)   
#else
#define NAND_PRINTF(fmt, args...)   
#define ENTER() 
#define LEAVE()   
#endif
#define CONFIG_MTD_PARTITIONS
#ifndef memzero
#define memzero(s, n)     memset ((s), 0, (n))
#endif

#define LP_OPTIONS      (NAND_SAMSUNG_LP_OPTIONS | NAND_NO_READRDY | NAND_NO_AUTOINCR)
#define LP_OPTIONS16  (LP_OPTIONS | NAND_BUSWIDTH_16)

#define GPIO_BASE_ADDR                                 (0xD8110000)  /* 64K  */
#define PM_CTRL_BASE_ADDR                        (0xD8130000 )  /* 64K  */
#define SYSTEM_CFG_CTRL_BASE_ADDR    (0xD8120000 ) 
#define NF_CTRL_CFG_BASE_ADDR               (0xD8009000 )
#define PMPMA_ADDR       (PM_CTRL_BASE_ADDR + 0x0200)
#define PMPMB_ADDR       (PM_CTRL_BASE_ADDR + 0x0204)
#define PMPMC_ADDR       (PM_CTRL_BASE_ADDR + 0x0208)
#define PMPMD_ADDR       (PM_CTRL_BASE_ADDR + 0x020C)
#define PMPME_ADDR       (PM_CTRL_BASE_ADDR + 0x0210)
#define PMPMF_ADDR       (PM_CTRL_BASE_ADDR + 0x0214)     /* PLL Audio(I2S) Control Register */
#define PMNAND_ADDR      (PM_CTRL_BASE_ADDR + 0x0330)     /* NAND */
#define PMNANDH_ADDR     (PM_CTRL_BASE_ADDR + 0x0319)
#define PMCS_ADDR        (PM_CTRL_BASE_ADDR + 0x0000)
#define PMCEU_ADDR       (PM_CTRL_BASE_ADDR + 0x0254)

/* controller and mtd information */
struct elite_nand_set 
{
	int	nr_chips;
	int	nr_partitions;
	char *name;
	int *nr_map;
	struct mtd_partition *partitions;
};

struct elite_nand_info;

struct elite_nand_mtd 
{
	struct mtd_info	mtd;
	struct nand_chip chip;
	struct elite_nand_info *info;
	int	scan_res;
};

/* overview of the elite nand state */
struct elite_nand_info 
{
	/* mtd info */
	struct nand_hw_control controller;
	struct elite_nand_mtd *mtds;

	/* device info */
	struct device *device;
	struct resource *area;
	void __iomem *reg;
	int cpu_type;
	int datalen;
	int nr_data;
	int data_pos;
	int page_addr;
	dma_addr_t dmaaddr;
	unsigned char *dmabuf;
};

#ifdef CONFIG_MTD_NAND_ELITE_HWECC
static int MAX_CHIP = CONFIG_MTD_NAND_CHIP_NUM;
static int hardware_ecc = 1;

#if (CONFIG_MTD_NAND_HM_ECC == 1)
#define NAND_HARMING_ECC
#else
#undef NAND_HARMING_ECC
#endif

#else /*else CONFIG_MTD_NAND_ELITE_HWECC */
#define NAND_HARMING_ECC
#define MAX_CHIP 1
static int hardware_ecc = 0;
#endif /*end CONFIG_MTD_NAND_ELITE_HWECC */

/*
* ecc_type = 0 : Harming ECC
* ecc_type = 1 : BCH ECC
*/
#ifndef NAND_HARMING_ECC
static int ecc_type  = 1;
#else
static int ecc_type  = 0;
#endif

#if  CONFIG_MTD_NAND_PAGE_SIZE  > 4096
#define NAND_BANK_SIZE    1024
#else
#define NAND_BANK_SIZE    512
#endif
static uint32_t nfc_rw_timming = 0x1212;
static uint32_t ecc_8bit_engine = 0;
static uint32_t ecc_12bit_engine = 0;
static uint32_t ecc_24bit_engine = 0;
static uint32_t ecc_40bit_engine = 0;
static uint32_t redunt_err_mark = 0;
static uint32_t chip_swap = 0;
static int test_page=0;	 
static struct elite_nand_flash_dev elite_nand_flash_ids[] = {
	{0x2C64444B, 4096, 8192,  218, 0x200000, 5, 0, 255, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC,  24, 0x140A0C46, "MT29F64G08CBACA",LP_OPTIONS},
	{0x2C48044A, 2048, 4096,  218, 0x100000, 5, 0, 255, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC,  12, 0x140A0C46, "MT29F16G08CBACA",LP_OPTIONS},
	{0x2C68044A, 4096, 4096,  218, 0x100000, 5, 0, 255, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC,  12, 0x140A0C46, "MT29F32G08CBACA",LP_OPTIONS},
	{0xADD314A5, 4096, 2048,  64, 0x40000, 5, 0, 1, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 4, 0x140A0C46, "HY27UT088G2M-T(P)",LP_OPTIONS},
	{0xADF1801D, 1024, 2048,  64, 0x20000, 4, 0,  1, 0, WIDTH_8, 4, 0, 1, NAND_TYPE_SLC, 4, 0x140A0F64, "HY27UF081G2A", LP_OPTIONS},
	{0xADF1001D, 1024, 2048,  64, 0x20000, 4, 0,  1, 0, WIDTH_8, 4, 0, 1, NAND_TYPE_SLC, 4, 0x140A0C46, "H27U1G8F2BFR", LP_OPTIONS},
	{0xADD59425, 4096, 4096, 218,0x80000, 5, 125, 127, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC,12, 0x140A0F64, "HY27UAG8T2A", LP_OPTIONS},
	{0xADD7949A, 2048, 8192, 218,0x200000, 5, 0, 255, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC,12, 0x140A0C64, "HY27UBG8T2ATR", LP_OPTIONS},
	{0xECD314A5, 4096, 2048,  64, 0x40000, 5, 127, 0, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 4, 0x140A0C64, "K9G8G08X0A", LP_OPTIONS},
	{0xECD59429, 4096, 4096, 218,0x80000, 5, 127, 0, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 12, 0x140A0F64, "K9GAG08UXD", LP_OPTIONS},
	{0xECF10095, 1024, 2048,  64, 0x20000, 4, 0, 1, 0, WIDTH_8, 4, 0, 1, NAND_TYPE_SLC,4, 0x140a1464, "K9F1G08U0B", LP_OPTIONS},
	{0xEC75A5BD, 2048,  512,  16, 0x4000, 4, 0, 1, 5, WIDTH_8, 1, 1, 0, NAND_TYPE_SLC, 1, 0x230F1964, "K95608U0D", 0},
	{0xECD514B6, 4096, 4096, 128, 0x80000, 5, 127, 0, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC,4, 0x140A0C64, "K9GAG08U0M", LP_OPTIONS},
	{0xECD755B6, 8192, 4096, 128, 0x80000, 5, 127, 0, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC,4, 0x140A0C64, "K9LBG08U0M", LP_OPTIONS},
	{0xECD58472, 2048, 8192, 218,0x100000, 5, 0, 127, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 12/*24*/, 0x190A0FFF, "K9GAG08U0E", LP_OPTIONS},
	{0x98D594BA, 4096, 4096, 218, 0x80000, 5, 0, 127, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 12, 0x190F0F70, "TC58NVG4D1DTG0", LP_OPTIONS},
	{0x98D19015, 1024, 2048,  64, 0x20000, 4, 0, 1, 0, WIDTH_8, 4, 0, 1, NAND_TYPE_SLC, 4, 0x140A0C11, "TC58NVG0S3ETA00", LP_OPTIONS},
	{0x98D59432, 2048, 8192, 218,0x100000, 5, 0, 127, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC,12/*24*/, 0x140A0C84, "TC58NVG4D2FTA00", LP_OPTIONS},
	{0,}
	/*add new product item here.*/
};

#ifdef CONFIG_MTD_PARTITIONS
#include <linux/mtd/partitions.h>

struct mtd_partition nand_partitions[] = {
	{
		.name	= "secureboot",
		.offset	= 0x00000000,
		.size	= 0x400000,
		.mask_flags =MTD_WRITEABLE
	},
	{
		.name	= "secureos",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x400000,
		.mask_flags =MTD_WRITEABLE
	},
	{
		.name	= "audiofirmware",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x400000,
		.mask_flags =0
	},
	{
		.name	= "bdib_ro",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x200000,
		.mask_flags =MTD_WRITEABLE
	},
	{
		.name	= "uboot_env",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x200000,
		.mask_flags =0
	},
	{
		.name	= "uboot",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x400000,
		.mask_flags =0
	},
	{
		.name	= "uboot_logo",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x800000,
		.mask_flags =MTD_WRITEABLE
	},
	{
		.name	= "bdib_rw",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x300000,
		.mask_flags =0
	},
	{
		.name	= "nvram",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x400000,
		.mask_flags =0
	},
	{
		.name	= "otaloader",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x2000000 ,
		.mask_flags =0
	},
	{
		.name	= "iploader",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x2000000,
		.mask_flags =0
	},
	{
		.name	= "irdeto_nvram",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x400000,
		.mask_flags =0
	},
	{
		.name	= "boot",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x1000000 ,
		.mask_flags =0
	},
	{
		.name	= "minidvb",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x2000000   ,
		.mask_flags =0
	},
	{
		.name	= "recovery",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x2000000 ,
		.mask_flags =0
	},
	{
		.name	= "system",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x20000000  ,
		.mask_flags =0
	},
	{
		.name	= "package",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x8000000 ,
		.mask_flags =0
	},
	{
		.name	= "cache",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x20000000,
		.mask_flags =0
	},
	{
		.name	= "backup",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x40000000,
		.mask_flags =0
	},
	{
		.name	= "data",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x6c500000,
		.mask_flags =0
	},
};

#define MAX_ELITE_MTD_PARTS  sizeof(nand_partitions)/sizeof(struct mtd_partition)

EXPORT_SYMBOL(nand_partitions);
#else
#include <linux/mtd/partitions.h>
#define MAX_ELITE_MTD_PARTS  0
struct mtd_partition nand_partitions[0] = {};
#endif

/*hardware specific Out Of Band information*/
/*new oob placement block for use with hardware ecc generation*/
static struct nand_ecclayout elite_oobinfo_2048 = {
	/*nand flash new structure and BCH ECC oob info */
	.eccbytes = 7,
	.eccpos = { 24, 25, 26, 27, 28, 29, 30},
	.oobavail = 24,
	.oobfree = {{0, 24} }
};
static struct nand_ecclayout elite_nand_8bit_oobinfo_4096 = {
	.eccbytes = 16,
	.eccpos = {24, 25, 26, 27, 28, 29, 30,
			31, 32, 33, 34, 35, 36},
	.oobavail = 24,
	.oobfree = {{0, 24}}
};

static struct nand_ecclayout elite_12bit_oobinfo_4096 = {
	/*nand flash old structure and Harming ECC oob info */
	.eccbytes = 20,
	.eccpos = { 24, 25, 26, 27, 28, 29, 30,
			31, 32, 33, 34, 35, 36, 37,
			38, 39, 40, 41, 42, 43},
	.oobavail = 24,
	.oobfree = {{0, 24} }
};

static struct nand_ecclayout elite_oobinfo_8192 = {
	/*nand flash new structure and Harming ECC oob info */
	.eccbytes = 42,
	.eccpos = { 24, 25, 26, 27, 28, 29, 30,
			31, 32, 33, 34, 35, 36, 37,
			38, 39, 40, 41, 42, 43, 44,
			45, 46, 47, 48, 49, 50, 51,
			52, 53, 54, 55, 56, 57, 58,
			59, 60, 61, 62, 63, 64, 65},
	.oobavail = 24,
	.oobfree = {{0, 24} }
};

static struct nand_ecclayout elite_oobinfo_4096 = {
	/*nand flash old structure and Harming ECC oob info */
	.eccbytes = 7,
	.eccpos = { 24, 25, 26, 27, 28, 29, 30},
	.oobavail = 24,
	.oobfree = {{0, 24} }
};

static struct nand_ecclayout elite_hm_oobinfo_2048 = {
	/*nand flash old structure and Harming ECC oob info */
	.eccbytes = 14,
	.eccpos = { 32, 33, 34, 36, 37, 38, 40, 41, 42, 44, 45, 46, 48, 49},
	.oobavail = 32,
	.oobfree = {{0, 32} }
};

static struct nand_ecclayout elite_hm_oobinfo_4096 = {
	/*nand flash old structure and Harming ECC oob info */
	.eccbytes = 27,
	.eccpos = {64, 65, 66, 68, 69, 70, 72, 73, 74, 76, 77, 78,
			  80, 81, 82, 84, 85, 86, 88, 89, 90, 92, 93, 94,
			  96, 97, 98},
	.oobavail = 64,
	.oobfree = {{0, 64} }
};

static struct nand_ecclayout elite_oobinfo_512 = {
	/*  nand flash old structure and Harming ECC oob info */
	.eccbytes = 8,
	.eccpos = { 4, 5, 6, 8, 9, 10, 12, 13},
	.oobfree = {{0, 4},{7, 1},{11,1},{14,2}}
};
static struct nand_ecclayout elite_nand_24bit_oobinfo_8192 = {
	.eccbytes = 42,
	.eccpos = { 24, 25, 26, 27, 28, 29, 30,31, 
	                 32, 33, 34, 35, 36,37,38, 39, 
	                 40, 41, 42, 43,44,45,46, 47, 
	                },
	.oobavail = 24,
	.oobfree = {{0, 24}}
};
static struct nand_ecclayout elite_nand_40bit_oobinfo_8192 = {
	.eccbytes = 70,
	.eccpos = {24, 25, 26, 27, 28, 29, 30,
			31, 32, 33, 34, 35, 36},
	.oobavail = 24,
	.oobfree = {{0, 24}}
};

/*The BBT code really ought to be able to work this bit out for itself from the above, at least for the 2KiB case*/
static uint8_t elite_bbt_pattern_2048[] = { 'B', 'b', 't', '0' };
static uint8_t elite_mirror_pattern_2048[] = { '1', 't', 'b', 'B' };

static struct nand_bbt_descr elite_bbt_main_descr_2048 = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
			|NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs =4,
	.len = 4,
	.veroffs = 0,
	.maxblocks = 4,
	.pattern = elite_bbt_pattern_2048,
};

static struct nand_bbt_descr elite_bbt_mirror_descr_2048 = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
			| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs =4,
	.len = 4,
	.veroffs = 0,
	.maxblocks = 4,
	.pattern = elite_mirror_pattern_2048,
};

static struct nand_bbt_descr elite_bbt_main_descr_512 = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
			|NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs =0,
	.len = 4,
	.veroffs = 14,
	.maxblocks = 4,
	.pattern = elite_bbt_pattern_2048,
};

static struct nand_bbt_descr elite_bbt_mirror_descr_512 = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
			|NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs =0,
	.len = 4,
	.veroffs = 14,
	.maxblocks = 4,
	.pattern = elite_mirror_pattern_2048,
};

static spinlock_t elite_nand_lock;
static int dump_count =0;

extern int add_mtd_partitions(struct mtd_info *, const struct mtd_partition *, int);

/* conversion functions */
static struct elite_nand_mtd *elite_nand_mtd_toours(struct mtd_info *mtd)
{
	return container_of(mtd, struct elite_nand_mtd, mtd);
}

static struct elite_nand_info *elite_nand_mtd_toinfo(struct mtd_info *mtd)
{
	return elite_nand_mtd_toours(mtd)->info;
}

/*type : HW or SW ECC*/
static void elite_nfc_ecc_set(struct elite_nand_info *info, int type)
{
	if (type == hardware_ecc)
		writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) & (~USE_SW_ECC), (volatile char *)info->reg + ELITE_NFC_REG_12);
	else
		writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) | (USE_SW_ECC), (volatile char *)info->reg + ELITE_NFC_REG_12);
}

int elite_get_sys_para(char *var_name, unsigned char *varval, int *var_len)
{
	return -1;
}

int elite_set_sys_para(char *var_name, char *varval)
{
	return -1;
}

void elite_copy_filename(char *dst, char *src, int size)
{
	if (*src && (*src == '"')) 
	{
		++src;
		--size;
	}

	while ((--size > 0) && *src && (*src != '"')) 
	{
		*dst++ = *src++;
	}
	*dst = '\0';
}

/*Get the flash and manufacturer id and lookup if the type is supported*/
static struct elite_nand_flash_dev *get_flash_type(struct mtd_info *mtd,struct nand_chip *chip,
					     int busw, int *maf_id, int ce, unsigned int *spec_clk)
{
	struct elite_nand_flash_dev *type = NULL, type_env;
	int i, dev_id, maf_idx, ret = 0;
	unsigned int id = 0;
	char varval[200], *s = NULL, *tmp, varname[] = "elite.io.nand";
	unsigned int varlen = 200, value;
	ENTER();
	
	/* select the device */
	chip->select_chip(mtd, ce);

	/*  reset test */
	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	/* send the command for reading device ID */
	chip->cmdfunc(mtd, NAND_CMD_READID, 0x00, -1);

	/* Read manufacturer and device IDs */
	*maf_id = chip->read_byte(mtd);
	for (i = 0; i < 3; i++) 
	{
		dev_id = chip->read_byte(mtd);
		id += ((unsigned char)dev_id) <<((2-i)*8);
	}

	NAND_PRINTF("nand chip device maf_id is %x, and dev_id is %x\n",*maf_id,dev_id);

	/* Lookup the flash id */
	for (i = 0; elite_nand_flash_ids[i].flash_id != 0; i++) 
	{
		if (((unsigned int)id + ((*maf_id)<<24)) == elite_nand_flash_ids[i].flash_id) 
		{
			type =  &elite_nand_flash_ids[i];
			NAND_PRINTF("find nand chip device id i=%d\n", i);
			break;
		}
	}
list:
	if (!type) 
	{
		LEAVE();
		return ERR_PTR(-ENODEV);
	}
	if (!mtd->name)
		mtd->name = "nand";//"elite.nand";

	chip->chipsize = (uint64_t)type->block_count * (uint64_t)type->block_size;

	/* get all information from table */
	chip->cellinfo = type->nand_type << 2;
	chip->cellinfo = 0;
	mtd->writesize = type->page_size;
	mtd->oobsize = type->spare_size;
	mtd->erasesize = type->block_size;
	mtd->size=chip->chipsize ;
	busw = type->data_width ? NAND_BUSWIDTH_16 : 0;

	*spec_clk = type->rw_timming;

	printk("writesize:0x%x,oobsize:0x%x,erasesize:0x%x,size:0x%x,busw :0x%x",
		mtd->writesize,mtd->oobsize,mtd->erasesize,mtd->size,busw);

	/* Try to identify manufacturer */
	for (maf_idx = 0; nand_manuf_ids[maf_idx].id != 0x0; maf_idx++) 
	{
		if (nand_manuf_ids[maf_idx].id == *maf_id)
			break;
	}

	/*check, if buswidth is correct. Hardware drivers should set chip correct*/
	if (busw != (chip->options & NAND_BUSWIDTH_16)) {
		NAND_PRINTF("NAND device: Manufacturer ID:0x%02x, Chip ID: 0x%02x (%s %s)\n", 
					*maf_id,id, nand_manuf_ids[maf_idx].name, mtd->name);
		NAND_PRINTF("NAND bus width %d instead %d bit\n",
					  (chip->options & NAND_BUSWIDTH_16) ? 16 : 8,busw ? 16 : 8);
		LEAVE();
		return ERR_PTR(-EINVAL);
	}

	/* Calculate the address shift from the page size */
	chip->page_shift = ffs(mtd->writesize) - 1;

	/* Convert chipsize to number of pages per chip -1. */
	chip->pagemask = (chip->chipsize >> chip->page_shift) - 1;

	chip->bbt_erase_shift = chip->phys_erase_shift =ffs(mtd->erasesize) - 1;
	chip->chip_shift = ffs(chip->chipsize) - 1;

	/* Set the bad block position */
	chip->badblockpos = mtd->writesize > 512 ?NAND_LARGE_BADBLOCK_POS : NAND_SMALL_BADBLOCK_POS;

	/* Get chip options, preserve non chip based options */
	chip->options &= ~NAND_CHIPOPTIONS_MSK;
	chip->options |= type->options & NAND_CHIPOPTIONS_MSK;

	/* Set chip as a default. Board drivers can override it, if necessary */
	chip->options |= NAND_NO_AUTOINCR;

	/* Check if chip is a not a samsung device. Do not clear the
	* options for chips which are not having an extended id.
	*/
	if (*maf_id != NAND_MFR_SAMSUNG && type->page_size > 512)
		chip->options &= ~NAND_SAMSUNG_LP_OPTIONS;

	NAND_PRINTF("NAND device: Manufacturer ID: 0x%02x, Chip ID: 0x%02x (%s %s),size:0x%x\n", 
				*maf_id, id,nand_manuf_ids[maf_idx].name, type->product_name,mtd->size);

	LEAVE();
	return type;
}

struct device *nand_device;

static int elite_nfc_init(struct elite_nand_info *info, struct mtd_info *mtd)
{
	int busw, nand_maf_id, page_per_block, ce = 0;
	unsigned int spec_clk, t, thold, divisor, status = 0;
	struct nand_chip *chip = mtd->priv;
	struct elite_nand_flash_dev *type;
	struct clk * nand_clk=NULL;
	ENTER();

	/* get buswidth to select the correct functions */
	busw = chip->options & NAND_BUSWIDTH_16;
	writel(0x0,(volatile char *)info->reg + ELITE_NFC_REG_22);
	
	writel(0x0,(volatile char *)info->reg + ELITE_NFC_REG_D);
	writeb(0, (volatile char *)info->reg + ELITE_NFC_REG_13);
	writeb(0, (volatile char *)info->reg + ELITE_NFC_REG_9);
	writel(0xF<<4, (volatile char *)info->reg  + ELITE_NFC_REG_D);

	/* enable chip select */
	/* new structure  */
	writel(readl((volatile char *)info->reg + ELITE_NFC_REG_14) & 0xffff0000,(volatile char *)info->reg + ELITE_NFC_REG_14);
	writel(readl((volatile char *)info->reg + ELITE_NFC_REG_14) | nfc_rw_timming,(volatile char *)info->reg + ELITE_NFC_REG_14);
	
	writel(B2R,(volatile char *)info->reg + ELITE_NFC_REG_B);

	printk(KERN_ERR"nand_clk is name %s \n",dev_name(nand_device));
	nand_clk=clk_get(nand_device, "nand");
	if (IS_ERR(nand_clk))
	{
		printk(KERN_ERR"nand_clk is NULL\n");
		return -EIO;
	}
	else
	{
		clk_set_rate(nand_clk, 85700*1000);
	}
	writel(0x10,(volatile char *)info->reg + ELITE_NFC_REG_18);
	
	/* enable chip 0 */
	writel(0x800, (volatile char *)info->reg  + ELITE_NFC_REG_10);
	if(CONFIG_MTD_NAND_HM_ECC==8)
	{
		writel(0x1042, (volatile char *)info->reg  + ELITE_NFC_REG_10);
	}
	else if(CONFIG_MTD_NAND_HM_ECC==12)
	{
		writel(0x1422, (volatile char *)info->reg  + ELITE_NFC_REG_10);
	}
	else if(CONFIG_MTD_NAND_HM_ECC==24)
	{
		writel(0x2A2E, (volatile char *)info->reg  + ELITE_NFC_REG_10);
	}
	else if(CONFIG_MTD_NAND_HM_ECC==40)
	{
		writel(0x2A0E, (volatile char *)info->reg  + ELITE_NFC_REG_10);
	}
	
	/* add to identify the page size and block size */
	if (chip_swap == 1) 
	{
		NAND_PRINTF("resume: set chip_swap to 0!\n");
		chip_swap = 0;
	}
	type = get_flash_type(mtd, chip, busw, &nand_maf_id, ce, &spec_clk);
	if (IS_ERR(type)) 
	{
		ce++;
		type = get_flash_type(mtd, chip, busw, &nand_maf_id, ce, &spec_clk);
		if (IS_ERR(type)) 
		{
			printk(KERN_WARNING "NO NAND device found!!!\n");
			chip->select_chip(mtd, -1);
			if (nand_maf_id) 
			{
				printk(KERN_WARNING "nand flash identify fail(unknow id)\n");
				//while(1);
			}
			return -ENODEV;
		}
		else 
		{
			NAND_PRINTF("chip swap to chip 1!!!\n");
			chip_swap = 1;
		}
	}

	page_per_block = chip->phys_erase_shift - chip->page_shift;
	NAND_PRINTF("page per block = 2^%x \n", page_per_block);
	switch (page_per_block) 
	{
		case 4:
			/*32 page per block*/
			writel(0x1F, (volatile char *)info->reg + ELITE_NFC_REG_17); 
			break;
		case 5:/*32 page per block*/
			writel((1 << 5) | 0x1F, (volatile char *)info->reg + ELITE_NFC_REG_17);
			break;
		case 6:/*64 page per block*/
			writel((2 << 5) | 0x1F, (volatile char *)info->reg + ELITE_NFC_REG_17);
			break;
		case 7:/*128 page per block*/
			writel((3 << 5) | 0x1F, (volatile char *)info->reg + ELITE_NFC_REG_17);
			break;
		case 8:/*256 page per block*/
			writel((4 << 5) | 0x1F, (volatile char *)info->reg + ELITE_NFC_REG_17);
			break;
		case 9:/*512 page per block*/
			writel((5 << 5) | 0x1F, (volatile char *)info->reg + ELITE_NFC_REG_17);
			break;
		default:
			printk(KERN_ERR "page per block not support\n");
			break;
	}

	NAND_PRINTF("mtd->writesize = %x \n", mtd->writesize);
	if (mtd->writesize == 2048) 
	{
#ifndef NAND_HARMING_ECC
		writel((PAGE_2K|WIDTH_8|WP_DISABLE)&(~OLDDATA_EN),(volatile char *)info->reg + ELITE_NFC_REG_12);
#else
		if (ecc_type == 1)
			writel((PAGE_2K|WIDTH_8|WP_DISABLE) & (~OLDDATA_EN),(volatile char *)info->reg + ELITE_NFC_REG_12);
		else /*old structure*/
			writel(PAGE_2K|WIDTH_8|WP_DISABLE|OLDDATA_EN,(volatile char *)info->reg + ELITE_NFC_REG_12);
#endif
	} 
	else if (mtd->writesize == 4096 ) 
	{
#ifndef NAND_HARMING_ECC
		writel((PAGE_4K|WIDTH_8|WP_DISABLE)&(~OLDDATA_EN),(volatile char *)info->reg + ELITE_NFC_REG_12);
#else
		if (ecc_type == 1)
			writel((PAGE_4K|WIDTH_8|WP_DISABLE) & (~OLDDATA_EN),(volatile char *)info->reg + ELITE_NFC_REG_12);
		else
			writel(PAGE_4K|WIDTH_8|WP_DISABLE|OLDDATA_EN,(volatile char *)info->reg + ELITE_NFC_REG_12);
#endif
	} 
	else if (mtd->writesize == 8192) 
	{
#ifndef NAND_HARMING_ECC
		writel((PAGE_8K|WIDTH_8|WP_DISABLE)&(~OLDDATA_EN),(volatile char *)info->reg + ELITE_NFC_REG_12);
#else
		if (ecc_type == 1)
			writel((PAGE_8K|WIDTH_8|WP_DISABLE) & (~OLDDATA_EN),(volatile char *)info->reg + ELITE_NFC_REG_12);
		else
			writel(PAGE_8K|WIDTH_8|WP_DISABLE|OLDDATA_EN,(volatile char *)info->reg + ELITE_NFC_REG_12);
#endif
	} 
	else 
	{
		/*new structure*/
		writel((PAGE_512|WIDTH_8|WP_DISABLE)&(~OLDDATA_EN),(volatile char *)info->reg + ELITE_NFC_REG_12); 
	}
	chip->select_chip(mtd, -1);
	writel(readl((volatile char *)info->reg + ELITE_NFC_REG_14) & 0xffff0000,(volatile char *)info->reg + ELITE_NFC_REG_14);
	writel(readl((volatile char *)info->reg + ELITE_NFC_REG_14) | nfc_rw_timming,(volatile char *)info->reg + ELITE_NFC_REG_14);
	LEAVE();
	return 0;
}

static void disable_redunt_out_bch_ctrl(struct elite_nand_info *info, int flag)
{
	ENTER();
	printk("disable_redunt_out_bch_ctrl flag:%d\n",flag);
	if (flag == 1)
		writeb(readb((volatile char *)info->reg + ELITE_NFC_REG_D)|0x02, (volatile char *)info->reg + ELITE_NFC_REG_D);
	else
		writeb(readb((volatile char *)info->reg + ELITE_NFC_REG_D)&0xfd, (volatile char *)info->reg + ELITE_NFC_REG_D);
	LEAVE();
}

static void set_ecc_engine(struct elite_nand_info *info, int type)
{
	ENTER();
	printk("ECC_MODE:%d,ecc_type_local:%d\n",CONFIG_MTD_NAND_HM_ECC,type);
	writel(readl((volatile char *)info->reg + ELITE_NFC_REG_23) & 0xfffffff8, (volatile char *)info->reg + ELITE_NFC_REG_23);
	writel(readl((volatile char *)info->reg + ELITE_NFC_REG_23) | type, (volatile char *)info->reg + ELITE_NFC_REG_23);

	/* enable BCH ecc interrupt and new structure */
	if (type == ECC16BIT)
	{
		printk("set_ecc_engine for bch 16 bit\n");
		writew(ECC_BCH_INTERRUPT_ENABLE, (volatile char *)info->reg + ELITE_NFC_REG_24);
		writel(readl((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,(volatile char *)info->reg + ELITE_NFC_REG_23);
		if (ecc_type == 1)
			writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) & (~OLDDATA_EN),(volatile char *)info->reg + ELITE_NFC_REG_12);
		else
			writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) | OLDDATA_EN,(volatile char *)info->reg + ELITE_NFC_REG_12);
	}
	else if (type == ECC12BIT)
	{
		NAND_PRINTF("set_ecc_engine for bch 12 bit\n");
		writew(ECC_BCH_INTERRUPT_ENABLE, (volatile char *)info->reg + ELITE_NFC_ECC_BCH_INT_MASK);
		writel(readl((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,(volatile char *)info->reg + ELITE_NFC_REG_23);
		if (ecc_type == 1)
			writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) & (~OLDDATA_EN),(volatile char *)info->reg + ELITE_NFC_REG_12);
		else
			writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) | OLDDATA_EN,(volatile char *)info->reg + ELITE_NFC_REG_12);
	}
	else if (type == ECC8BIT) 
	{
		NAND_PRINTF("set_ecc_engine for bch 8 bit\n");
		writew(ECC_BCH_INTERRUPT_ENABLE, (volatile char *)info->reg + ELITE_NFC_ECC_BCH_INT_MASK);
		writel(readl((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,(volatile char *)info->reg + ELITE_NFC_REG_23);
		if (ecc_type == 1)
			writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) & (~OLDDATA_EN),(volatile char *)info->reg + ELITE_NFC_REG_12);
		else
			writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) | OLDDATA_EN,(volatile char *)info->reg + ELITE_NFC_REG_12);
	}
	else if (type == ECC4BIT)
	{
		NAND_PRINTF("set_ecc_engine for bch 4 bit\n");
		writew(ECC_BCH_INTERRUPT_ENABLE, (volatile char *)info->reg + ELITE_NFC_ECC_BCH_INT_MASK);
		writel(readl((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,(volatile char *)info->reg + ELITE_NFC_REG_23);
		if (ecc_type == 1)
			writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) & (~OLDDATA_EN),(volatile char *)info->reg + ELITE_NFC_REG_12);
		else
			writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) | OLDDATA_EN,(volatile char *)info->reg + ELITE_NFC_REG_12);
	}	
	else if (type == ECC24BIT)
	{
		printk("set_ecc_engine for bch 24 bit\n");
		writew(ECC_BCH_INTERRUPT_ENABLE, (volatile char *)info->reg + ELITE_NFC_ECC_BCH_INT_MASK);
		writel(readl((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,(volatile char *)info->reg + ELITE_NFC_REG_23);
		if (ecc_type == 1)
			writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) & (~OLDDATA_EN),(volatile char *)info->reg + ELITE_NFC_REG_12);
		else
			writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) | OLDDATA_EN,(volatile char *)info->reg + ELITE_NFC_REG_12);
	}	
	else if (type == ECC40BIT)
	{
		printk("set_ecc_engine for bch 40 bit\n");
		writew(ECC_BCH_INTERRUPT_ENABLE, (volatile char *)info->reg + ELITE_NFC_ECC_BCH_INT_MASK);
		writel(readl((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,(volatile char *)info->reg + ELITE_NFC_REG_23);
		if (ecc_type == 1)
			writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) & (~OLDDATA_EN),(volatile char *)info->reg + ELITE_NFC_REG_12);
		else
			writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) | OLDDATA_EN,(volatile char *)info->reg + ELITE_NFC_REG_12);
	}	
	else 
	{
		printk("set_ecc_engine for  1 bit\n");
	      /*disable 4bit ecc interrupt and old structure*/
		writew(ECC_BCH_INTERRUPT_DISABLE, (volatile char *)info->reg + ELITE_NFC_ECC_BCH_INT_MASK);
		writel(readl((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,(volatile char *)info->reg + ELITE_NFC_REG_23);
		NAND_PRINTF("set_ecc_engine for harmming\n");
		if (ecc_type == 1)
			writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) & (~OLDDATA_EN),(volatile char *)info->reg + ELITE_NFC_REG_12);
		else
			writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) | OLDDATA_EN,(volatile char *)info->reg + ELITE_NFC_REG_12);
	}
	LEAVE();
}


static int elite_nand_ready(struct mtd_info *mtd)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	unsigned int b2r_stat;
	int i = 0;
	ENTER();
	while (1)	
	{
		if (readb((volatile char *)info->reg + ELITE_NFC_REG_B) & B2R)
			break;
		if ((++i>>21)) 
		{
			printk(KERN_ERR "nand flash is not ready\n");
			LEAVE();
			//while(1);
			return -1;
		}
	}
	b2r_stat = readb((volatile char *)info->reg + ELITE_NFC_REG_B);
	writeb(B2R|b2r_stat, (volatile char *)info->reg + ELITE_NFC_REG_B);
	if (readb((volatile char *)info->reg + ELITE_NFC_REG_B) & B2R)	
	{
		printk(KERN_ERR "NFC err : B2R status not clean\n");
		LEAVE();
		//while(1);
		return -2;
	}
	LEAVE();
	return 0;
}

static int elite_nfc_transfer_ready(struct mtd_info *mtd)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	int i = 0;
	ENTER();
	while (1)	
	{
		if (!(readb((volatile char *)info->reg + ELITE_NFC_REG_A) & NFC_BUSY))
			break;

		if (++i>>21)
		{
			printk(KERN_ERR "NFC err : transfer status not ready\n");
			LEAVE();
			//while(1);
			return -3;
		}
	}
	LEAVE();
	return 0;
}

static int elite_wait_chip_ready(struct mtd_info *mtd)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	int i = 0;
	ENTER();
	while (1) 
	{
		if ((readb((volatile char *)info->reg + ELITE_NFC_REG_A) & FLASH_RDY))
			break;
		if (++i>>21)
		{
			printk(KERN_ERR "NFC err : chip status not ready\n");
			LEAVE();
			//while(1);
			return -3;
		}
	}
	LEAVE();
	return 0;
}
static int elite_wait_cmd_ready(struct mtd_info *mtd)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	int i = 0;
	ENTER();
	while (1)	
	{
		if (!(readb((volatile char *)info->reg + ELITE_NFC_REG_A) & NFC_CMD_RDY))
			break;
		if (++i>>21)
		{
			printk(KERN_ERR "NFC err : cmd status not ready\n");
			LEAVE();
			//while(1);
			return -3;
		}
	}
	LEAVE();
	return 0;
}

static int elite_wait_nfc_ready(struct elite_nand_info *info)
{

	unsigned int bank_stat1, i = 0;
	ENTER();
	while (1) 
	{
		bank_stat1 = readw((volatile char *)info->reg + ELITE_NFC_REG_25);
		if (!(readb((volatile char *)info->reg + ELITE_NFC_REG_A) & NFC_BUSY))
			break;
		else if ((bank_stat1 & (ERR_CORRECT | BCH_ERR)) == (ERR_CORRECT | BCH_ERR))
			break;

		if (++i>>21)
		{
			printk(KERN_ERR "NFC err : nfc status not ready\n");
			LEAVE();
			return -1;
		}
	}
	LEAVE();
	return 0;

}

static void bit_correct(uint8_t *c, uint8_t pos)
{
	c[0] = (((c[0] ^ (0x01<<pos)) & (0x01<<pos)) | (c[0] & (~(0x01<<pos))));
}

/*****************************************************
* flag = 0, need check BCH ECC
* flag = 1, don't check ECC
* flag = 2, need check Harming ECC
*****************************************************/
static int elite_nfc_wait_idle(struct mtd_info *mtd, unsigned int flag, int command,
							int column, unsigned int page_addr, int phase)
{
	int i = 0, k;
	int page_step;
	unsigned int bank_stat1, bank_stat2;
	unsigned int data_redunt_flag;
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	page_step = 1 + mtd->writesize/NAND_BANK_SIZE;
	ENTER();
	while (1)
	{
		if (readb((volatile char *)info->reg + ELITE_NFC_REG_1D) & NFC_IDLE)
			break;

		if ((flag != 1) && (flag != 2))
		{
			if ((command == NAND_CMD_READ0))
			{
				unsigned int failed =mtd->ecc_stats.failed;
				for (k = 0; k < page_step; k++)
				{
					elite_wait_nfc_ready(info);
					bank_stat1 = readw((volatile char *)info->reg + ELITE_NFC_REG_25);
				   	NAND_PRINTF("bank_stat1 READ0 :0x%x\n",bank_stat1);
					if ((bank_stat1 & (ERR_CORRECT | BCH_ERR)) == (ERR_CORRECT | BCH_ERR))
					{
						elite_bch4bit_data_ecc_correct(mtd, column, page_addr);	
					}
				}
				if ((mtd->ecc_stats.failed !=failed)&&!(
					(((*((volatile char *)info->reg+ECC_FIFO_0+60)) != 0xFF)||
					((*((volatile char *)info->reg+ECC_FIFO_0+61)) != 0xFF))&&
					(((*((volatile char *)info->reg+ECC_FIFO_0+62)) != 0xFF)||
					((*((volatile char *)info->reg+ECC_FIFO_0+63)) != 0xFF))))
				{
					mtd->ecc_stats.failed=failed;
					writew(readw((volatile char *)info->reg  + ELITE_NFC_REG_25) | (ERR_CORRECT | BCH_ERR), (volatile char *)info->reg  + ELITE_NFC_REG_25);
					writew(readw((volatile char *)info->reg  + ELITE_NFC_REG_23) | READ_RESUME, (volatile char *)info->reg  + ELITE_NFC_REG_23);
				}
				else if(mtd->ecc_stats.failed !=failed)
				{    
					
					printk("\tdump:%d page:0x%x,column:0x%x\n",dump_count,page_addr,column);
					writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME, (volatile char *)info->reg  + ELITE_NFC_REG_23);
					return -2;
				       	
				}
			}
			else if (command == NAND_CMD_READOOB)
			{
				unsigned int failed =mtd->ecc_stats.failed;
				for (k = 0; k < 2; k++)
				{
                        elite_wait_nfc_ready(info);
						bank_stat1 = readw((volatile char *)info->reg + ELITE_NFC_REG_25);
						NAND_PRINTF("bank_stat1 READOOB :0x%x\n",bank_stat1);

						if ((bank_stat1 & (ERR_CORRECT | BCH_ERR)) == (ERR_CORRECT | BCH_ERR))
						{
							elite_bch4bit_redunt_ecc_correct(mtd);
						}
				}
				if ((mtd->ecc_stats.failed !=failed)&&!(
					(((*((volatile char *)info->reg+ECC_FIFO_0+60)) != 0xFF)||
					((*((volatile char *)info->reg+ECC_FIFO_0+61)) != 0xFF))&&
					(((*((volatile char *)info->reg+ECC_FIFO_0+62)) != 0xFF)||
					((*((volatile char *)info->reg+ECC_FIFO_0+63)) != 0xFF))))
				{
					mtd->ecc_stats.failed=failed;
					writew(readw((volatile char *)info->reg  + ELITE_NFC_REG_25) | (ERR_CORRECT | BCH_ERR), (volatile char *)info->reg  + ELITE_NFC_REG_25);
					writew(readw((volatile char *)info->reg  + ELITE_NFC_REG_23) | READ_RESUME, (volatile char *)info->reg  + ELITE_NFC_REG_23);
				}
				else if(mtd->ecc_stats.failed !=failed)
				{    				
					printk("\toob dump:%d page:0x%x,column:0x%x\n",dump_count,page_addr,column);
					writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME, (volatile char *)info->reg  + ELITE_NFC_REG_23);		
					return -2;
				    	
				}
			}
			writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME, (volatile char *)info->reg  + ELITE_NFC_REG_23);
		}

		if (i >> 21)
		{
			printk(KERN_ERR"NFC err : nfc status not idle\n");
			LEAVE();
			return -1;
		}

		i++;
	}

	LEAVE();
	return 0;
	
}

void elite_bch4bit_data_ecc_correct(struct mtd_info *mtd,int column, unsigned int page_addr)
{
	int i;
	unsigned int bch_err_pos[8], bank_stat1, bank_stat2, bch_ecc_idx, bank, ecc_engine;
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	ENTER();
	bank_stat1 = readw((volatile char *)info->reg  + ELITE_NFC_REG_25);
	if ((bank_stat1 & (ERR_CORRECT | BCH_ERR)) == (ERR_CORRECT | BCH_ERR))
	{
		bank_stat2 = readw((volatile char *)info->reg  + ELITE_NFC_REG_26);
		bch_ecc_idx = bank_stat2 & BCH_ERR_CNT;
		/* bank number of current decoder */
		bank = (bank_stat2 & 0x700) >> 8; 
		ecc_engine = readl((volatile char *)info->reg  + ELITE_NFC_REG_23) & ECC_MODE;
		NAND_PRINTF("bch_ecc_idx:%d,ecc_engine:%d\n",bch_ecc_idx,ecc_engine);
		if (((ecc_engine <5 )&&(bch_ecc_idx > (ecc_engine*4))) || ((ecc_engine == 5) && (bch_ecc_idx > 24))|| ((ecc_engine == 6) && (bch_ecc_idx > 40)))
		{
		
			NAND_PRINTF("data errors exceed the ability of ECC correct\n");
			writew(readw((volatile char *)info->reg  + ELITE_NFC_REG_25) | (ERR_CORRECT | BCH_ERR), (volatile char *)info->reg  + ELITE_NFC_REG_25);
			writew(readw((volatile char *)info->reg  + ELITE_NFC_REG_23) | READ_RESUME, (volatile char *)info->reg  + ELITE_NFC_REG_23);
			mtd->ecc_stats.failed++;
			LEAVE();
			return;
		}

		/* BCH ecc correct */
		for (i = 0; i < bch_ecc_idx; i++)
		{
			bch_err_pos[i] = (readw((volatile char *)info->reg+ ELITE_NFC_REG_27 + 2*i) & 0x3fff);
			if ((bch_err_pos[i] >> 3) < NAND_BANK_SIZE)
				bit_correct((unsigned char *)info->dmabuf + (NAND_BANK_SIZE*bank + (bch_err_pos[i] >> 3)), bch_err_pos[i] & 0x07);
			else if(bank==7)
			{
				bit_correct(((volatile char *)info->reg+ECC_FIFO_0 +(bch_err_pos[i] >> 3)-NAND_BANK_SIZE),(bch_err_pos[i] & 0x07));						 
			}

		}
	}

	writew(readw((volatile char *)info->reg  + ELITE_NFC_REG_25) | (ERR_CORRECT | BCH_ERR), (volatile char *)info->reg  + ELITE_NFC_REG_25);
	writew(readw((volatile char *)info->reg  + ELITE_NFC_REG_23) | READ_RESUME, (volatile char *)info->reg  + ELITE_NFC_REG_23);
	LEAVE();

}

void elite_bch4bit_redunt_ecc_correct(struct mtd_info *mtd)
{
	int i;
	unsigned int bch_err_pos[12], bank_stat1, bank_stat2, bch_ecc_idx, ecc_engine;
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	ENTER();
	bank_stat2 = readw((volatile char *)info->reg + ELITE_NFC_REG_26);
	bch_ecc_idx = bank_stat2 & BCH_ERR_CNT;
	bank_stat1 = readw((volatile char *)info->reg + ELITE_NFC_REG_25);
	if ((bank_stat1 & (ERR_CORRECT | BCH_ERR)) == (ERR_CORRECT | BCH_ERR)) /* has ECC error and BCH decoder finish */
	{
		/* ECC mode select: 1 bit Hamming, 4bits BCH, 8bits BCH, 12bits BCH, 16bits BCH per 512bytes */
		ecc_engine = readl((volatile char *)info->reg + ELITE_NFC_REG_23) & ECC_MODE;
 		if (((ecc_engine <5 )&&(bch_ecc_idx > (ecc_engine*4))) || ((ecc_engine == 5) && (bch_ecc_idx > 24))|| ((ecc_engine == 6) && (bch_ecc_idx > 40)))
		/* ERROR bit number is beyond the capacity of error correct */
		{	
			NAND_PRINTF("redunt errors exceed the ability of ECC correct\n");
			/* 
			* for example, if ECC mode is 4bits BCH/512bytes (bch_ecc_idx = 1), it has been beyond 
			* the capacity of correct error when greater than 4 bits error
			*/
			writew(readw((volatile char *)info->reg + ELITE_NFC_REG_25) | (0x100 | 0x01), (volatile char *)info->reg + ELITE_NFC_REG_25);
			writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | 0x100, (volatile char *)info->reg + ELITE_NFC_REG_23);

			mtd->ecc_stats.failed++;
			LEAVE();
			return;
		}

		/* OK, error is within the capacity of correction BCH ecc correct */
		for (i = 0; i < bch_ecc_idx; i++)
		{
			bch_err_pos[i] = (readw((volatile char *)info->reg + ELITE_NFC_REG_27 + 2*i) & 0x3fff);

			if((bch_err_pos[i] >> 3 )>=NAND_BANK_SIZE)
				bit_correct(((volatile char *)info->reg +ECC_FIFO_0 +(bch_err_pos[i] >> 3)-NAND_BANK_SIZE),(bch_err_pos[i] & 0x07));

		}
	}

	writew(readw((volatile char *)info->reg + ELITE_NFC_REG_25) | (ERR_CORRECT | BCH_ERR), (volatile char *)info->reg + ELITE_NFC_REG_25);
	writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME, (volatile char *)info->reg + ELITE_NFC_REG_23);
	LEAVE();

}

void elite_nand_hamming_ecc_1bit_correct(struct mtd_info *mtd)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	unsigned int ecc_err_pos, bank_stat, redunt_stat;
	ENTER();
	/* use HAMMING ECC but page not 512 and read data area */
	NAND_PRINTF("in elite_nand_cmdfunc(): Read data \n");

	bank_stat = readl((volatile char *)info->reg +ELITE_NFC_REG_20);
	redunt_stat = readb((volatile char *)info->reg + ELITE_NFC_REG_1F);

	if (bank_stat & 0x5555) 
	{
		printk(KERN_ERR "There are uncorrected ecc error in data area--\n");
		mtd->ecc_stats.failed++;
		LEAVE();
		return;
	} 
	else if (redunt_stat & 0x05) 
	{
		printk(KERN_ERR "There are uncorrected ecc error in reduntant area--\n");
		mtd->ecc_stats.failed++;
		LEAVE();
		return;
	}
	else if (((mtd->writesize == 2048) && (bank_stat & 0x2222)) ||
		((mtd->writesize == 4096) && (bank_stat & 0x22222222)) ||
		((mtd->writesize == 512) && (bank_stat & 0x22))) 
	{
	
		NAND_PRINTF("enter ther 11111\n");
		writeb(readb((volatile char *)info->reg + ELITE_NFC_REG_15) & 0xfc, (volatile char *)info->reg + ELITE_NFC_REG_15);
		if (bank_stat & 0x02) 
		{
			ecc_err_pos = readw((volatile char *)info->reg + ELITE_NFC_REG_1A);
			printk(KERN_NOTICE "bank 1 error BYTE: %x bit:%x\n",ecc_err_pos & 0x1ff, (ecc_err_pos >> 9) & 0x07);
			printk(KERN_NOTICE "error value :%x\n",*(uint8_t *)&info->dmabuf[ecc_err_pos & 0x1ff]);
			bit_correct(&info->dmabuf[ecc_err_pos & 0x1ff], (ecc_err_pos >> 9) & 0x07);
			printk(KERN_NOTICE "correct value :%x\n", *(uint8_t *)&info->dmabuf[ecc_err_pos & 0x1ff]);
		}
		else if (bank_stat & 0x20) 
		{
			ecc_err_pos = readw((volatile char *)info->reg + ELITE_NFC_REG_1B);
			printk(KERN_NOTICE "bank 2 error BYTE: %x bit:%x\n",(ecc_err_pos & 0x1ff)+512, (ecc_err_pos >> 9) & 0x07);
			printk(KERN_NOTICE "error value :%x\n",*(uint8_t *)&info->dmabuf[512 + (ecc_err_pos & 0x1ff)]);
			bit_correct(&info->dmabuf[512 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
			printk(KERN_NOTICE "correct value :%x\n",*(uint8_t *)&info->dmabuf[512 + (ecc_err_pos & 0x1ff)]);

		}
		if (mtd->writesize == 2048) 
		{
			writeb(readb((volatile char *)info->reg + ELITE_NFC_REG_15) | 0x1, (volatile char *)info->reg +  ELITE_NFC_REG_15);
			if (bank_stat & 0x0200) 
			{
				ecc_err_pos = readw((volatile char *)info->reg +  ELITE_NFC_REG_1A);
				printk(KERN_NOTICE "bank 3 error BYTE: %x bit:%x\n",(ecc_err_pos & 0x1ff)+1024, (ecc_err_pos >> 9) & 0x07);
				printk(KERN_NOTICE "error value :%x\n",*(uint8_t *)&info->dmabuf[1024 + (ecc_err_pos & 0x1ff)]);
				bit_correct(&info->dmabuf[1024 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
				printk(KERN_NOTICE "correct value :%x\n",*(uint8_t *)&info->dmabuf[1024 + (ecc_err_pos & 0x1ff)]);
			} 
			else if (bank_stat & 0x2000)
			{
				ecc_err_pos = readw((volatile char *)info->reg +  ELITE_NFC_REG_1B);
				printk(KERN_NOTICE "bank 4 error BYTE: %x bit:%x\n",(ecc_err_pos & 0x1ff)+1536, (ecc_err_pos >> 9) & 0x07);
				printk(KERN_NOTICE "error value :%x\n",*(uint8_t *)&info->dmabuf[1536 + (ecc_err_pos & 0x1ff)]);
				bit_correct(&info->dmabuf[1536 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
				printk(KERN_NOTICE "correct value :%x\n",*(uint8_t *)&info->dmabuf[1536 + (ecc_err_pos & 0x1ff)]);
			}
		}
		else if (mtd->writesize == 4096) 
		{
			writeb(readb((volatile char *)info->reg +  ELITE_NFC_REG_15) | 0x1 , (volatile char *)info->reg +  ELITE_NFC_REG_15);
			if (bank_stat & 0x0200) 
			{
				ecc_err_pos = readw((volatile char *)info->reg +  ELITE_NFC_REG_1A);
				printk(KERN_NOTICE "bank 3 error BYTE: %x bit:%x\n",(ecc_err_pos & 0x1ff)+1024, (ecc_err_pos >> 9) & 0x07);
				printk(KERN_NOTICE "error value :%x\n",*(uint8_t *)&info->dmabuf[1024 + (ecc_err_pos & 0x1ff)]);
				bit_correct(&info->dmabuf[1024 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
				printk(KERN_NOTICE "correct value :%x\n",*(uint8_t *)&info->dmabuf[1024 + (ecc_err_pos & 0x1ff)]);
			}
			else if (bank_stat & 0x2000) 
			{
				ecc_err_pos = readw((volatile char *)info->reg +  ELITE_NFC_REG_1B);
				printk(KERN_NOTICE "bank 4 error BYTE: %x bit:%x\n",(ecc_err_pos & 0x1ff)+1536, (ecc_err_pos >> 9) & 0x7);
				printk(KERN_NOTICE "error value :%x\n",*(uint8_t *)&info->dmabuf[1536 + (ecc_err_pos & 0x1ff)]);
				bit_correct(&info->dmabuf[1536 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
				printk(KERN_NOTICE "correct value :%x\n",*(uint8_t *)&info->dmabuf[1536 + (ecc_err_pos & 0x1ff)]);
			} 
			else if (bank_stat & 0x020000) 
			{
				writeb((readb((volatile char *)info->reg +  ELITE_NFC_REG_15) & 0xFC) | 0x2,(volatile char *)info->reg +  ELITE_NFC_REG_15);
				ecc_err_pos = readw((volatile char *)info->reg +  ELITE_NFC_REG_1A);
				printk(KERN_NOTICE "bank 5 error BYTE: %x bit:%x\n",(ecc_err_pos & 0x1ff)+2048, (ecc_err_pos >> 9) & 0x07);
				printk(KERN_NOTICE "error value :%x\n",*(uint8_t *)&info->dmabuf[2048 + (ecc_err_pos & 0x1ff)]);
				bit_correct(&info->dmabuf[2048 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
				printk(KERN_NOTICE "correct value :%x\n",*(uint8_t *)&info->dmabuf[2048 + (ecc_err_pos & 0x1ff)]);
			}
			else if (bank_stat & 0x200000) 
			{
				writeb((readb((volatile char *)info->reg +  ELITE_NFC_REG_15) & 0xFC) | 0x2,(volatile char *)info->reg +  ELITE_NFC_REG_15);
				ecc_err_pos = readw((volatile char *)info->reg +  ELITE_NFC_REG_1B);
				printk(KERN_NOTICE "bank 6 error BYTE: %x bit:%x\n",(ecc_err_pos & 0x1ff)+2560, (ecc_err_pos >> 9) & 0x07);
				printk(KERN_NOTICE "error value :%x\n",*(uint8_t *)&info->dmabuf[2560 + (ecc_err_pos & 0x1ff)]);
				bit_correct(&info->dmabuf[2560 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
				printk(KERN_NOTICE "correct value :%x\n",*(uint8_t *)&info->dmabuf[2560 + (ecc_err_pos & 0x1ff)]);
			}
			else if (bank_stat & 0x02000000) 
			{
				writeb(readb((volatile char *)info->reg +  ELITE_NFC_REG_15) | 0x3, (volatile char *)info->reg +  ELITE_NFC_REG_15);
				ecc_err_pos = readw((volatile char *)info->reg +  ELITE_NFC_REG_1A);
				printk(KERN_NOTICE "bank 7 error BYTE: %x bit:%x\n",(ecc_err_pos & 0x1ff)+3072, (ecc_err_pos >> 9) & 0x07);
				printk(KERN_NOTICE "error value :%x\n",*(uint8_t *)&info->dmabuf[3072 + (ecc_err_pos & 0x1ff)]);
				bit_correct(&info->dmabuf[3072 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
				printk(KERN_NOTICE "correct value :%x\n",*(uint8_t *)&info->dmabuf[3072 + (ecc_err_pos & 0x1ff)]);
			} 
			else if (bank_stat & 0x20000000) 
			{
				writeb(readb((volatile char *)info->reg +  ELITE_NFC_REG_15) | 0x3, (volatile char *)info->reg +  ELITE_NFC_REG_15);
				ecc_err_pos = readw((volatile char *)info->reg +  ELITE_NFC_REG_1B);
				printk(KERN_NOTICE "bank 8 error BYTE: %x bit:%x\n",(ecc_err_pos & 0x1ff)+3584, (ecc_err_pos >> 9) & 0x07);
				printk(KERN_NOTICE "error value :%x\n",*(uint8_t *)&info->dmabuf[3584 + (ecc_err_pos & 0x1ff)]);
				bit_correct(&info->dmabuf[3584 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
				printk(KERN_NOTICE "correct value :%x\n",*(uint8_t *)&info->dmabuf[3584 + (ecc_err_pos & 0x1ff)]);
			}
		}
	} 
	else if (redunt_stat & 0x02) 
	{
		ecc_err_pos = readw((volatile char *)info->reg +  ELITE_NFC_REG_1C);
		printk(KERN_NOTICE "oob area error BYTE: %x bit:%x\n",ecc_err_pos & 0x3f, (ecc_err_pos >> 8) & 0x07);
		printk(KERN_NOTICE "error value :%x\n",*((unsigned char *)info->reg+ECC_FIFO_0 + (ecc_err_pos & 0x3f)));
		bit_correct((unsigned char *)(info->dmabuf + mtd->writesize) + (ecc_err_pos & 0x3f),(ecc_err_pos >> 8) & 0x07);
		printk(KERN_NOTICE "error value :%x\n",*((unsigned char *)info->reg+ECC_FIFO_0 + (ecc_err_pos & 0x3f)));
	}
	LEAVE();
}

/*********************************************************
* [Routine Description]
*	read status
* [Arguments]
*	cmd : nand read status command
* [Return]
*	the result of command
*********************************************************/
static int elite_read_nand_status(struct mtd_info *mtd, unsigned char cmd)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	int cfg = 0, status = -1;
	unsigned int b2r_stat;
	ENTER();
	writeb(cmd, (volatile char *)info->reg + ELITE_NFC_REG_2);
	cfg = SP_CMD_EN|DPAHSE_DISABLE|NFC2NAND|(1<<1);
	b2r_stat = readl((volatile char *)info->reg + ELITE_NFC_REG_B);
	writel(B2R|b2r_stat, (volatile char *)info->reg + ELITE_NFC_REG_B);
	writel(cfg|NFC_TRIGGER, (volatile char *)info->reg + ELITE_NFC_REG_1);

	status = elite_wait_nfc_ready(info);
	if (status) 
	{
		printk(KERN_ERR "NFC command transfer1 is not ready\n");
		writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,(volatile char *)info->reg + ELITE_NFC_REG_23);
		LEAVE();
		return status;
	}
	b2r_stat = readl((volatile char *)info->reg + ELITE_NFC_REG_B);
	writel(B2R|b2r_stat, (volatile char *)info->reg + ELITE_NFC_REG_B);

	cfg = SING_RW|NAND2NFC;
	writel(cfg|NFC_TRIGGER, (volatile char *)info->reg + ELITE_NFC_REG_1);
	status = elite_wait_nfc_ready(info);
	if (status) 
	{
		printk(KERN_ERR "NFC command transfer2 is not ready\n");
		writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,
		(volatile char *)info->reg + ELITE_NFC_REG_23);
		LEAVE();
		return status;
	}
	status = elite_nfc_wait_idle(mtd, -1, -1, -1, -1, 0);
	if (status) 
	{
		printk(KERN_ERR "NFC IO transfer is not idle\n");
		LEAVE();
		return status;
	}
	NAND_PRINTF("read status is %x\n", readb((volatile char *)info->reg + ELITE_NFC_REG_0) & 0xff);
	info->datalen = 0;
	info->dmabuf[0] = readb((volatile char *)info->reg + ELITE_NFC_REG_0) & 0xff;
	status = info->dmabuf[0];
	LEAVE();
	return status;
}

/* data_flag = 0:  set data ecc fifo */
static int elite_nfc_dma_cfg(struct mtd_info *mtd, unsigned int len, unsigned int wr,
							int data_flag, int nbank)
{
	int i, status;
	unsigned long *dma_addr;
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	unsigned long *read_desc, *write_desc;
	ENTER();

	read_desc = (unsigned long *)(info->dmabuf + mtd->writesize + 0x100);
	write_desc = (unsigned long *)(info->dmabuf + mtd->writesize + 0x200);
	if (len == 0)
	{
		LEAVE();
		return 1;
	}
	writel(len - 1, (volatile char *)info->reg + ELITE_NFC_REG_8);
	if (readl((volatile char *)info->reg  + ELITE_NFC_DMA_ISR) & NAND_PDMA_IER_INT_STS)
		writel(NAND_PDMA_IER_INT_STS, (volatile char *)info->reg  + ELITE_NFC_DMA_ISR);
	if ((readl((volatile char *)info->reg  + ELITE_NFC_DMA_ISR) )& NAND_PDMA_IER_INT_STS) 
	{
		NAND_PRINTF("PDMA interrupt status can't be clear ");
		NAND_PRINTF("pNand_PDma_Reg->DMA_ISR = 0x%8.8x \n", (volatile char *)info->reg  + ELITE_NFC_DMA_ISR);
	}

	status = elite_nand_init_pdma(mtd);
	if (status)
	{
		printk(KERN_ERR "ERROR : DMA controller cannot be enabled!\n");
	}

	memset((wr)?write_desc:read_desc, 0, 0x100);
	elite_nand_init_long_desc((wr)?write_desc:read_desc, len, (unsigned long *)info->dmaaddr, 0, 1);

	dma_addr = (wr) ? (unsigned long *)(info->dmaaddr + mtd->writesize + 0x200) \
				: (unsigned long *)(info->dmaaddr + mtd->writesize + 0x100);

	elite_nand_config_pdma(mtd,dma_addr, wr);
    dmac_flush_range(info->dmabuf,info->dmabuf+(mtd->writesize +0x300));
    outer_flush_range(info->dmaaddr,info->dmaaddr+(mtd->writesize+0x300));

	LEAVE();
	return 0;

}

int elite_nand_init_pdma(struct mtd_info *mtd)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	ENTER();

	writel(NAND_PDMA_GCR_SOFTRESET, (volatile char *)info->reg + ELITE_NFC_DMA_GCR);
	writel(NAND_PDMA_GCR_DMA_EN, (volatile char *)info->reg + ELITE_NFC_DMA_GCR);
	if (readl((volatile char *)info->reg + ELITE_NFC_DMA_GCR) & NAND_PDMA_GCR_DMA_EN)
	{
		LEAVE();
		return 0;
	}
	else
	{
		LEAVE();
		return 1;
	}
}

int elite_nand_free_pdma(struct mtd_info *mtd)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	ENTER();
	writel(0, (volatile char *)info->reg + ELITE_NFC_DMA_DESPR);
	writel(0,(volatile char *) info->reg + ELITE_NFC_DMA_GCR);
	LEAVE();
	return 0;
}


int elite_nand_alloc_desc_pool(unsigned long *desc_addr)
{
	ENTER();
	memset(desc_addr, 0x00, 0x100);
	LEAVE();
	return 0;
}

int elite_nand_init_short_desc(unsigned long *desc_addr, unsigned int req_count, unsigned long *buffer_addr)
{
	nand_pdma_desc_short *cur_des_short;
	ENTER();
	cur_des_short = (nand_pdma_desc_short *) desc_addr;
	cur_des_short->req_count = req_count;
	cur_des_short->i = 1;
	cur_des_short->end = 1;
	cur_des_short->format = 0;
	cur_des_short->data_buffer_addr = (unsigned long)buffer_addr;
	LEAVE();
	return 0;
}

int elite_nand_init_long_desc(unsigned long *desc_addr, unsigned int req_count, unsigned long *buffer_addr,
							unsigned long *branch_addr, int end)
{
	nand_pdma_desc_long *cur_des_long;
	ENTER();
	cur_des_long = (nand_pdma_desc_long *) desc_addr;
	cur_des_long->req_count = req_count;
	cur_des_long->i = 0;
	cur_des_long->format = 1;
	cur_des_long->data_buffer_addr = (unsigned long)buffer_addr;
	cur_des_long->branch_addr = (unsigned long)branch_addr;
	if (end) 
	{
		cur_des_long->end = 1;
		cur_des_long->i = 1;
	}
	LEAVE();
	return 0;
}
int elite_nand_config_pdma(struct mtd_info *mtd, unsigned long *desc_addr, unsigned int dir)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	ENTER();
	
	writel((unsigned long)NAND_PDMA_IER_INT_EN, (volatile char *)info->reg + ELITE_NFC_DMA_IER);
	writel((unsigned long)desc_addr, (volatile char *)info->reg + ELITE_NFC_DMA_DESPR);
	if (dir == NAND_PDMA_READ)
		writel(readl((volatile char *)info->reg + ELITE_NFC_DMA_CCR)|NAND_PDMA_CCR_PERIPHERAL_TO_IF,
				(volatile char *)info->reg + ELITE_NFC_DMA_CCR);
	else
		writel(readl((volatile char *)info->reg + ELITE_NFC_DMA_CCR)&(~NAND_PDMA_CCR_IF_TO_PERIPHERAL),
				(volatile char *)info->reg + ELITE_NFC_DMA_CCR);

	/*mask_interrupt(IRQ_NFC_DMA);*/
	writel(readl((volatile char *)info->reg + ELITE_NFC_DMA_CCR)|NAND_PDMA_CCR_RUN, (volatile char *)info->reg + ELITE_NFC_DMA_CCR);

	LEAVE();
	return 0;
}

int elite_nand_pdma_handler(struct mtd_info *mtd)
{
	unsigned long status = 0;
	unsigned long count = 0;
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	ENTER();
	count = 0x100000;
	/*polling CSR TC status	*/
	do {
		count--;
		if (readl((volatile char *)info->reg + ELITE_NFC_DMA_ISR) & NAND_PDMA_IER_INT_STS) 
		{
			status = readl((volatile char *)info->reg + ELITE_NFC_DMA_CCR) & NAND_PDMA_CCR_EVT_CODE;
			writel(readl((volatile char *)info->reg + ELITE_NFC_DMA_ISR)&NAND_PDMA_IER_INT_STS,(volatile char *)info->reg + ELITE_NFC_DMA_ISR);
			break;
		}
		if (count == 0) 
		{
			printk(KERN_ERR "PDMA Time Out!\n");
			printk(KERN_ERR "NFC_DMA_CCR = 0x%8.8x\r\n",(unsigned int)readl((volatile char *)info->reg + ELITE_NFC_DMA_CCR));
			count = 0x100000;
		}
	} while (1);

	if (status == NAND_PDMA_CCR_EVT_FF_UNDERRUN)
		printk(KERN_ERR "PDMA Buffer under run!\n");

	if (status == NAND_PDMA_CCR_EVT_FF_OVERRUN)
		printk(KERN_ERR "PDMA Buffer over run!\n");

	if (status == NAND_PDMA_CCR_EVT_DESP_READ)
		printk(KERN_ERR "PDMA read Descriptor error!\n");

	if (status == NAND_PDMA_CCR_EVT_DATA_RW)
		printk(KERN_ERR "PDMA read/write memory descriptor error!\n");

	if (status == NAND_PDMA_CCR_EVT_EARLY_END)
		printk(KERN_ERR "PDMA read early end!\n");

	if (count == 0) 
	{
		printk(KERN_ERR "PDMA TimeOut!\n");
		//while (1);
	}
	
	LEAVE();
	return 0;
}

static int elite_nand_read_id(struct mtd_info *mtd)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	unsigned int cfg = 0, i = 0;
	int status = -1;
	ENTER();
	writeb(NAND_CMD_READID, (volatile char *)info->reg + ELITE_NFC_REG_2);
	writel(0xff00, (volatile char *)info->reg + ELITE_NFC_REG_3);
	writel(0x4, (volatile char *)info->reg + ELITE_NFC_REG_8);

	cfg = DPAHSE_DISABLE|(0x02<<1);
	writel(cfg|NFC_TRIGGER|SP_CMD_EN, (volatile char *)info->reg + ELITE_NFC_REG_1);

	status = elite_wait_cmd_ready(mtd);
	if (status) 
	{
		printk(KERN_ERR "in elite_nand_readID(): wait cmd is not ready\n");
		writew(readw((volatile char *)info->reg +ELITE_NFC_REG_23) | READ_RESUME,(volatile char *)info->reg + ELITE_NFC_REG_23);
		LEAVE();
		return status;
	}
	cfg = NAND2NFC|SING_RW;
	for (i = 0; i < 5; i++) 
	{
		writel(cfg|NFC_TRIGGER|SP_CMD_EN, (volatile char *)info->reg + ELITE_NFC_REG_1);
		status = elite_wait_cmd_ready(mtd);
		if (status)
		{
			printk(KERN_ERR "in elite_nand_readID(): wait cmd is not ready\n");
			LEAVE();
			return status;
		}
		status =elite_nfc_transfer_ready(mtd);
		if (status) 
		{
			printk(KERN_ERR "in elite_nand_readID(): wait transfer cmd is not ready\n");
			LEAVE();
			return status;
		}
		info->dmabuf[i] = readb((volatile char *)info->reg + ELITE_NFC_REG_0) & 0xff;
		NAND_PRINTF(KERN_NOTICE "readID is %x\n", readb((volatile char *)info->reg +ELITE_NFC_REG_0));
	}
	info->datalen = 0;
	LEAVE();
	return 0;
}

static int elite_device_ready(struct mtd_info *mtd)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	return readb((volatile char *)info->reg + ELITE_NFC_REG_A) & 0x01;
}

static void elite_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	ENTER();
	if (mode == hardware_ecc)
		writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) & (~USE_SW_ECC),(volatile char *)info->reg + ELITE_NFC_REG_12);
	else
		writel(readl((volatile char *)info->reg + ELITE_NFC_REG_12) |USE_SW_ECC, (volatile char *)info->reg + ELITE_NFC_REG_12);
	LEAVE();


}
static unsigned int  elite_nand_address(struct mtd_info *mtd,unsigned int addr_cycle,int column, int page_addr)
{
	struct nand_chip *chip = mtd->priv;
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	ENTER();
	NAND_PRINTF(KERN_NOTICE "column:0x%x,page_addr:0x%x\n", column,page_addr);

	if (column != -1)
	{
		writeb(column, (volatile char *)info->reg + ELITE_NFC_REG_3);
		addr_cycle++;

		if (mtd->writesize != 512)
		{
			writeb(column >> 8, (unsigned char *)((volatile char *)info->reg + ELITE_NFC_REG_3) + 1);
			addr_cycle++;
		}

		if (page_addr != -1)
		{
			if (mtd->writesize != 512)
			{
				writeb(page_addr, (volatile char *)info->reg + ELITE_NFC_REG_4);
				page_addr >>= 8;
				writeb(page_addr, (unsigned char *)((volatile char *)info->reg + ELITE_NFC_REG_4) + 1);
				addr_cycle += 2;
			}
			else
			{
				writeb(page_addr, (unsigned char *)((volatile char *)info->reg + ELITE_NFC_REG_3) + 1);
				page_addr >>= 8;
				writeb(page_addr, (volatile char *)info->reg + ELITE_NFC_REG_4);
				addr_cycle += 2;
			}

			if (mtd->writesize == 2048)
			{
				/* 2048 bytes/per page, one more address cycle needed for 128Mb-larger devices */
				if (chip->chipsize > (128 << 20))
				{
					page_addr >>= 8;
					writeb(page_addr, (volatile char *)info->reg + ELITE_NFC_REG_5);
					addr_cycle++;
				}
			}
			else if (mtd->writesize == 4096)
			{
				if (chip->chipsize > (256 << 20))
				{
					page_addr >>= 8;
					writeb(page_addr, (volatile char *)info->reg + ELITE_NFC_REG_5);
					addr_cycle++;
				}
			}
			else
			{
				if (chip->chipsize > (32 << 20))
				{
					page_addr >>= 8;
					writeb(page_addr, (volatile char *)info->reg + ELITE_NFC_REG_5);
					addr_cycle++;
				}
			}
		}
	}
	else if ((page_addr != -1) && (column == -1))
	{
		writeb(page_addr & 0xff, (volatile char *)info->reg + ELITE_NFC_REG_3);
		page_addr >>= 8;
		writeb(page_addr, (unsigned char *)((volatile char *)info->reg + ELITE_NFC_REG_3) + 1);
		addr_cycle += 2;

		if (mtd->writesize == 2048)
		{
			/* 2048 bytes/per page, one more address cycle needed for 128Mb-larger devices */
			if (chip->chipsize > (128 << 20))
			{
				page_addr >>= 8;
				writeb(page_addr, (volatile char *)info->reg + ELITE_NFC_REG_4);
				addr_cycle++;
			}
		}
		else if (mtd->writesize == 4096)
		{
			if (chip->chipsize > (256 << 20))
			{
				page_addr >>= 8;
				writeb(page_addr, (volatile char *)info->reg + ELITE_NFC_REG_4);
				addr_cycle++;
			}
		}
		else
		{
			if (chip->chipsize > (32 << 20))
			{
				page_addr >>= 8;
				writeb(page_addr, (volatile char *)info->reg + ELITE_NFC_REG_4);
				addr_cycle++;
			}
		}

	}
	LEAVE();
	return addr_cycle;
}

/**************************************************************
* elite_nand_cmdfunc - Send command to NAND large page device
* @mtd:	MTD device structure
* @command:	the command to be sent
* @column: the column address for this command, -1 if none
* @page_addr: the page address for this command, -1 if none
* 
* Send command to NAND device. This is the version for the new large page devices.
* We dont have the separate regions as we have in the small page devices.
* We must emulate NAND_CMD_READOOB to keep the code compatible.
***************************************************************/
static void elite_nand_cmdfunc(struct mtd_info *mtd, unsigned command, int column, int page_addr)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	struct nand_chip *chip = mtd->priv;
	unsigned int addr_cycle = 0, b2r_stat;
	int status = -1;
	unsigned int ecc_err_pos, bank_stat, redunt_stat, bank_stat1, read_phase =FIRST_4K_218;
	int readcmd;
	int mycolumn = column, mypage_addr = page_addr; 
	ENTER();
	NAND_PRINTF("enter in elite_nand_cmdfunc() command: %x column:%x, page_addr:%x\n",command, column, page_addr);
	test_page=page_addr;
	spin_lock(&elite_nand_lock);

	switch (command) 
	{

		case NAND_CMD_READOOB:
		{
			int failed=mtd->ecc_stats.failed;
			mycolumn = column=(NAND_BANK_SIZE+chip->ecc.bytes)*(chip->ecc.steps-1);
			writel(readl((volatile char *)info->reg + ELITE_NFC_REG_23) |0x10, (volatile char *)info->reg + ELITE_NFC_REG_23);
			dump_count=0;
read_oob_dump_again:			
			dump_count++;
			addr_cycle = 0;
			/* nand cmd read data area */
			NAND_PRINTF(KERN_NOTICE "config dma: read page_addr:%x\n", page_addr);
			/* 1: read, 0:data, -1:  */
			NAND_PRINTF(KERN_NOTICE "read data area column %x writesize %x\n", column, mtd->writesize);
			elite_nfc_dma_cfg(mtd,chip->ecc.size, 0, -1, -1);

			info->datalen = 0;
			/* write to clear B2R */
			b2r_stat = readb((volatile char *)info->reg + ELITE_NFC_REG_B);
			writeb(B2R|b2r_stat, (volatile char *)info->reg + ELITE_NFC_REG_B);
			NAND_PRINTF("RB is %d\n", b2r_stat & 0x02);

			addr_cycle= elite_nand_address(mtd, addr_cycle, column,  page_addr);
			bank_stat1 = readw((volatile char *)info->reg + ELITE_NFC_REG_25);
			writew(bank_stat1|(ERR_CORRECT | BCH_ERR), (volatile char *)info->reg + ELITE_NFC_REG_25);
			writeb(readb((volatile char *)info->reg + ELITE_NFC_REG_9)&0xD,(volatile char *)info->reg + ELITE_NFC_REG_9);

			b2r_stat = readb((volatile char *)info->reg + ELITE_NFC_REG_11);
			if ((b2r_stat&3) == 3) 
			{
				NAND_PRINTF("chip 0, or 1, is not select chip_sel=%x\n", b2r_stat);
				writeb(0xfe, (volatile char *)info->reg + ELITE_NFC_REG_11);
			}
			status = elite_wait_chip_ready(mtd); 
			if (status)
				printk(KERN_ERR "The chip is not ready\n");
			writeb(NAND_CMD_READ0, (volatile char *)info->reg + ELITE_NFC_REG_2);
			if (addr_cycle == 4)
				writeb(NAND_CMD_READSTART, (volatile char *)info->reg + ELITE_NFC_REG_5);
			else if (addr_cycle == 5)
				writeb(NAND_CMD_READSTART, (unsigned char *)(info->reg + ELITE_NFC_REG_5) + 1);

			writel(NAND2NFC|MUL_CMDS|((addr_cycle + 2)<<1)|SP_CMD_EN|NFC_TRIGGER, (unsigned char *)info->reg + ELITE_NFC_REG_1);

			status = elite_nand_ready(mtd);
			
			if (status) 
			{
				printk(KERN_ERR "readstart: nand flash is not ready\n");
				writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,(volatile char *)info->reg + ELITE_NFC_REG_23);
			}

			elite_wait_nfc_ready(info);
			status = elite_nfc_wait_idle(mtd, 0, command, mycolumn, mypage_addr, read_phase);
			
			writel(readl((volatile char *)info->reg + ELITE_NFC_REG_23) & ((~0x10)&0xFFFF), (volatile char *)info->reg + ELITE_NFC_REG_23);
			if (status) 
			{
			
				printk(KERN_ERR "elite_nfc_wait_idle status =%d\n", status);
				printk(KERN_ERR "command =0x%x\n", command);
				printk(KERN_ERR "Read ERR ,NFC is not idle\n");

				writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,(volatile char *)info->reg + ELITE_NFC_REG_23);
				if((status ==-2 )&&(dump_count <=3))
				{
					goto read_oob_dump_again;
				}
				spin_unlock(&elite_nand_lock);
				LEAVE();
				return;
			}

			if (!(readw((volatile char *)info->reg + ELITE_NFC_REG_9)&2)) 
			{
				status = elite_nand_pdma_handler(mtd);
				elite_nand_free_pdma(mtd);
				if (status)
					printk(KERN_ERR "dma transfer data time out: %x\n",
				readb((volatile char *)info->reg + ELITE_NFC_REG_A));
			}

			{
				uint8_t *buf = info->dmabuf + mtd->writesize;
				memcpy(buf, (volatile char *)info->reg+ECC_FIFO_0, mtd->oobsize);
				memcpy(chip->oob_poi,buf, mtd->oobsize);
				buf = info->dmabuf;
				memcpy(buf, (volatile char *)info->reg+ECC_FIFO_0, 64);

			}

			if(dump_count >1)
			{	 
				mtd->ecc_stats.failed=failed;
				printk("\toob dump:%d finished page:0x%x,column:0x%x\n",dump_count,page_addr,column);				   
			}
			spin_unlock(&elite_nand_lock);
			LEAVE();
			return;
		}
	case NAND_CMD_READ0:
		{
			int failed=mtd->ecc_stats.failed;
			dump_count=0;
read_dump_again:			
			dump_count++;
			addr_cycle = 0;
			/* nand cmd read data area */
			if (mtd->writesize == 512) 
			{
				/* write to clear B2R */
				b2r_stat = readb((volatile char *)info->reg + ELITE_NFC_REG_B);
				writeb(B2R|b2r_stat, (volatile char *)info->reg + ELITE_NFC_REG_B);
				/* chip enable:  enable CE0*/
				writeb(0x00, (volatile char *)info->reg + ELITE_NFC_REG_11);
			}

			NAND_PRINTF(KERN_NOTICE "config dma: read page_addr:%x\n", page_addr);
			/* 1: read, 0:data, -1:  */
			NAND_PRINTF(KERN_NOTICE "read data area column %x writesize %x\n", column, mtd->writesize);
			elite_nfc_dma_cfg(mtd, ((mtd->writesize <= 8192)? mtd->writesize : mtd->writesize/2), 0, -1, -1);

			info->datalen = 0;
			/* write to clear B2R */
			b2r_stat = readb((volatile char *)info->reg + ELITE_NFC_REG_B);
			writeb(B2R|b2r_stat, (volatile char *)info->reg + ELITE_NFC_REG_B);
			NAND_PRINTF("RB is %d\n", b2r_stat & 0x02);
	
			addr_cycle= elite_nand_address(mtd, addr_cycle, column,  page_addr);
				 
#ifdef NAND_HARMING_ECC /* HAMMing ECC */
			writeb(0x07, (volatile char *)info->reg + ELITE_NFC_REG_1F);
			writel(0xffffffff, (volatile char *)info->reg + ELITE_NFC_REG_20);
#else  
			bank_stat1 = readw((volatile char *)info->reg + ELITE_NFC_REG_25);
			writew(bank_stat1|(ERR_CORRECT | BCH_ERR), (volatile char *)info->reg + ELITE_NFC_REG_25);
#endif
	
			writeb(readb((volatile char *)info->reg + ELITE_NFC_REG_9)&0xD,(volatile char *)info->reg + ELITE_NFC_REG_9);
			if (mtd->writesize == 512) 
			{
				NAND_PRINTF("pagesize=512 command mode\n");
				NAND_PRINTF( "command is %x\n", command);
				writeb(command, (volatile char *)info->reg + ELITE_NFC_REG_2);
				writel(SP_CMD_EN|NAND2NFC | ((addr_cycle + 1)<<1)|NFC_TRIGGER,(unsigned char *)info->reg + ELITE_NFC_REG_1);
	
				status = elite_nand_ready(mtd);
				if (status) 
				{
					printk(KERN_ERR "nand flash is not ready : %x\n",
					readb((volatile char *)info->reg + ELITE_NFC_REG_A));
				}
				status =elite_nfc_transfer_ready(mtd);
				if (status) {
					printk(KERN_ERR "wait transfer command and data is not finished : %x\n",
					readb((volatile char *)info->reg + ELITE_NFC_REG_A));
					writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,
					(volatile char *)info->reg + ELITE_NFC_REG_23);
				}
				/*	chip disable:  disable CE*/
				status = elite_nfc_wait_idle(mtd, 1, -1, -1, -1, read_phase);
				if (status) 
				{

				        
					printk(KERN_ERR "wait transfer data and nfc is not idle : %x\n",
					readb((volatile char *)info->reg + ELITE_NFC_REG_A));
					/*print_register(mtd);*/
					writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,
					(volatile char *)info->reg + ELITE_NFC_REG_23);
                                        
					goto read_dump_again;
				}
				if (!(readw((volatile char *)info->reg + ELITE_NFC_REG_9)&2))
				{
					status = elite_nand_pdma_handler(mtd);
					elite_nand_free_pdma(mtd);
					if (status)
						printk(KERN_ERR "dma transfer data time out: %x\n",
						readb((volatile char *)info->reg + ELITE_NFC_REG_A));
				}	
				else
					printk(KERN_NOTICE "ELITE_NFC_REG_9 = 2\n");
				/* disable CE0 */
				writeb(0x0, (volatile char *)info->reg + ELITE_NFC_REG_11);
			}
			else 
			{ 
	
#ifdef NAND_HARMING_ECC
				NAND_PRINTF( "page2k/4k, command =0x%x\n", command);
				status = elite_wait_chip_ready(mtd);
				if (status)
					printk(KERN_ERR "The chip is not ready\n");
	
				writeb(NAND_CMD_READ0, (volatile char *)info->reg + ELITE_NFC_REG_2);
				writel(SP_CMD_EN|DPAHSE_DISABLE|((addr_cycle + 1)<<1)|NFC_TRIGGER,(volatile char *)info->reg + ELITE_NFC_REG_1);
				/* wait all command + address sequence finish status */
				status = elite_wait_cmd_ready(mtd);
				if (status) 
				{
					printk(KERN_NOTICE "dma transfer is not ready 2k or 4k page\n");
				}
				/* wait device idle 1: don't check ecc*/
				status =elite_nfc_wait_idle(mtd, 1, -1, -1, -1, read_phase);
	
				if (status) 
				{
					printk(KERN_ERR "wait transfer command and nfc is not idle : %x\n",
					readb((volatile char *)info->reg + ELITE_NFC_REG_A));
					writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,
							(volatile char *)info->reg + ELITE_NFC_REG_23);
					goto read_dump_again;
				}
	
				/* write to clear B2R */
				b2r_stat = readb((volatile char *)info->reg + ELITE_NFC_REG_B);
				writeb(B2R|b2r_stat, (volatile char *)info->reg + ELITE_NFC_REG_B);
	
				writeb(NAND_CMD_READSTART, (volatile char *)info->reg + ELITE_NFC_REG_2);
				writel(SP_CMD_EN|NAND2NFC|(1<<1)|NFC_TRIGGER, (volatile char *)info->reg + ELITE_NFC_REG_1);
				/*wait busy to ready int status*/
				status = elite_nand_ready(mtd);
				if (status) 
				{
					printk(KERN_ERR "readstart: nand flash is not ready\n");
					writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,
							(volatile char *)info->reg + ELITE_NFC_REG_23);
				}
				elite_wait_nfc_ready(info); 
	
#ifdef NAND_BBT_BCH_ECC
				if (ecc_type == 0) 
				{
#endif
					/* use HAMMING ECC */
					status = elite_nfc_wait_idle(mtd, 2, command, mycolumn, mypage_addr, read_phase);
#ifdef NAND_BBT_BCH_ECC
				}	
				else 
				{
					/* use BCH ECC */
					NAND_PRINTF("use BCH to read command = %x \n", command);
					status = elite_nfc_wait_idle(mtd, 0, command, mycolumn, mypage_addr, read_phase);

				}
#endif
	
				if (status) 
				{
					printk(KERN_ERR "elite_nfc_wait_idle status =%d\n", status);
					printk(KERN_ERR "command =0x%x\n", command);
					printk(KERN_ERR "Read ERR ,NFC is not idle\n");
					writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,
					(volatile char *)info->reg + ELITE_NFC_REG_23);
					if (status == -2)
					{
						if(dump_count <=3)
						{
							goto read_dump_again;
						}
					} 
					spin_unlock(&elite_nand_lock);
					LEAVE();
					return;
				}
	
				if (!(readw((volatile char *)info->reg +ELITE_NFC_REG_9)&2)) 
				{
					status = elite_nand_pdma_handler(mtd);
					printk(KERN_ERR "check status pdma handler status= %x \n", status);
					elite_nand_free_pdma(mtd);
					if (status)
					{
						spin_unlock(&elite_nand_lock);
						LEAVE();
						return;
					}
				}
	
#else /* #ifdef NAND_HARMING_ECC */
				b2r_stat = readb((volatile char *)info->reg + ELITE_NFC_REG_11);
				if ((b2r_stat&3) == 3) 
				{
					NAND_PRINTF("chip 0, or 1, is not select chip_sel=%x\n", b2r_stat);
					writeb(0xfe, (volatile char *)info->reg + ELITE_NFC_REG_11);
				}
				status = elite_wait_chip_ready(mtd); 
				if (status)
					printk(KERN_ERR "The chip is not ready\n");
				writeb(NAND_CMD_READ0, (volatile char *)info->reg + ELITE_NFC_REG_2);
				if (addr_cycle == 4)
					writeb(NAND_CMD_READSTART, (volatile char *)info->reg + ELITE_NFC_REG_5);
				else if (addr_cycle == 5)
					writeb(NAND_CMD_READSTART, (unsigned char *)(info->reg + ELITE_NFC_REG_5) + 1);
	
				writel(NAND2NFC|MUL_CMDS|((addr_cycle + 2)<<1)|SP_CMD_EN|NFC_TRIGGER, (unsigned char *)info->reg + ELITE_NFC_REG_1);
	
				status = elite_nand_ready(mtd);
				if (status) 
				{
					printk(KERN_ERR "readstart: nand flash is not ready\n");
					writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,(volatile char *)info->reg + ELITE_NFC_REG_23);
				}
				elite_wait_nfc_ready(info);
	
				status = elite_nfc_wait_idle(mtd, 0, command, mycolumn, mypage_addr, read_phase);
				
				if (status) 
				{
	
					printk(KERN_ERR "elite_nfc_wait_idle status =%d\n", status);
					printk(KERN_ERR "command =0x%x\n", command);
					printk(KERN_ERR "Read ERR ,NFC is not idle\n");
	
					writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,(volatile char *)info->reg + ELITE_NFC_REG_23);
					if (status == -2)
					{
						if(dump_count <=3)
						{
							goto read_dump_again;
						}
					} 
					spin_unlock(&elite_nand_lock);
					LEAVE();
					return;
				}
	
				if (!(readw((volatile char *)info->reg + ELITE_NFC_REG_9)&2)) 
				{
					status = elite_nand_pdma_handler(mtd);
					elite_nand_free_pdma(mtd);
					if (status)
						printk(KERN_ERR "dma transfer data time out: %x\n",
					readb((volatile char *)info->reg + ELITE_NFC_REG_A));
				}
#endif
			} /* end of else of page size = 512 */
	
#ifdef NAND_BBT_BCH_ECC
			if (ecc_type == 0) 
			{
#endif
				if (mtd->writesize == 512) 
				{
					/* use HAMMING ECC but page not 512 and read oob area */
					NAND_PRINTF( "in elite_nand_cmdfunc(): Read oob data \n");
					redunt_stat = readb((volatile char *)info->reg + ELITE_NFC_REG_1F);
					if (redunt_stat) 
					{
						NAND_PRINTF(" Read OOB redundant ecc eror command: %x column:%x, page_addr:%x\n",
										command, mycolumn, mypage_addr);
						NAND_PRINTF("redunt_stat:%x\n", redunt_stat);
					}
					if (redunt_stat & 0x05) 
					{
						printk(KERN_ERR "There are uncorrected ecc error in reduntant area--\n");
						mtd->ecc_stats.failed++;
						spin_unlock(&elite_nand_lock);
						LEAVE();
						return;
					} 
					else if (redunt_stat & 0x02) 
					{
						NAND_PRINTF("There are 1 bit ecc error in reduntant data area--\n");
						ecc_err_pos = readw((volatile char *)info->reg + ELITE_NFC_REG_1C);
						bit_correct((unsigned char *)info->reg+ECC_FIFO_0 + (ecc_err_pos & 0x3f),
									(ecc_err_pos >> 8) & 0x07);
					}
	
					/* read data area with hamming ecc correct */
					/* use HAMMING ECC but page not 512 and read data area */
					NAND_PRINTF("in elite_nand_cmdfunc(): Read data \n");
	
					bank_stat = readl((volatile char *)info->reg + ELITE_NFC_REG_20);
					redunt_stat = readb((volatile char *)info->reg + ELITE_NFC_REG_1F);
					if ((bank_stat&0x77) || redunt_stat) 
					{
						NAND_PRINTF(" Read data ecc eror command: %x column:%x, page_addr:%x\n",
									command, column, mypage_addr);
						NAND_PRINTF("error block addr: 0x%x page_addr:0x%x\n",
						mypage_addr>>(chip->phys_erase_shift - chip->page_shift), mypage_addr&0x3F);
						NAND_PRINTF(" bank_stat:0x%x, redunt_stat:0x%x\n",bank_stat, redunt_stat);
					}
					elite_nand_hamming_ecc_1bit_correct(mtd);
				}
				else 
				{
					/* read data area with hamming ecc correct */
					/* use HAMMING ECC but page not 512 and read data area */
					NAND_PRINTF(KERN_NOTICE "in elite_nand_cmdfunc(): Read data \n");
					bank_stat = readl((volatile char *)info->reg + ELITE_NFC_REG_20);
					redunt_stat = readb((volatile char *)info->reg + ELITE_NFC_REG_1F);
					if ((bank_stat&0x77777777) || redunt_stat) 
					{
						NAND_PRINTF(" Read data ecc eror command: %x column:%x, page_addr:%x\n",
										command, column, mypage_addr);
						NAND_PRINTF("error block addr: 0x%x page_addr:0x%x\n",
							mypage_addr>>(chip->phys_erase_shift - chip->page_shift), mypage_addr&0x3F);
						NAND_PRINTF(" bank_stat:0x%x, redunt_stat:0x%x\n",bank_stat, redunt_stat);
					}
					elite_nand_hamming_ecc_1bit_correct(mtd);	
				}
#ifdef NAND_BBT_BCH_ECC
			}
#endif
			{
				uint8_t *buf = info->dmabuf + mtd->writesize;
				memcpy(buf, (volatile char *)info->reg+ECC_FIFO_0, mtd->oobsize);
				memcpy(chip->oob_poi,buf, mtd->oobsize);
			}
		
			if(dump_count >1)
			{   
				mtd->ecc_stats.failed=failed;
				printk("\tdump:%d finished page:0x%x,column:0x%x\n",dump_count,page_addr,column);
			}
			spin_unlock(&elite_nand_lock);
			LEAVE();
			return;
		}		
		case NAND_CMD_SEQIN:
			if (mtd->writesize  == 512) 
			{
				if (column >= mtd->writesize) 
				{
					/* OOB area */
					column -= mtd->writesize;
					readcmd = NAND_CMD_READOOB;
				} 
				else if (column < 256) 
				{
					/* First 256 bytes --> READ0 */
					readcmd = NAND_CMD_READ0;
				}
				else 
				{
					column -= 256;
					readcmd = NAND_CMD_READ1;
				}
				writeb(readcmd, (volatile char *)info->reg + ELITE_NFC_REG_2);
				writel(SP_CMD_EN|DPAHSE_DISABLE | (1<<1) | NFC_TRIGGER, (volatile char *)info->reg + ELITE_NFC_REG_1);
				elite_wait_nfc_ready(info);
			}

		case NAND_CMD_ERASE1:
			NAND_PRINTF("enter ERASE1\n");	
			addr_cycle=elite_nand_address(mtd, addr_cycle, column,  page_addr);
	
			/* set command 1 cycle */
			writeb(command, (volatile char *)info->reg + ELITE_NFC_REG_2);
			if (command == NAND_CMD_SEQIN)
				writel(((addr_cycle + 1)<<1)|SP_CMD_EN|NFC_TRIGGER, (volatile char *)info->reg + ELITE_NFC_REG_1);
			else 
			{
				writel(SP_CMD_EN|DPAHSE_DISABLE|((addr_cycle + 1)<<1)|NFC_TRIGGER,(volatile char *)info->reg + ELITE_NFC_REG_1);
			}

			if (command == NAND_CMD_ERASE1) 
			{
				status = elite_wait_cmd_ready(mtd);
				if (status)
					printk(KERN_ERR "command is not ready\n");
				writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,
				(volatile char *)info->reg + ELITE_NFC_REG_23);
			}	
			else 
			{
#ifdef NAND_HARMING_ECC
				status = 0;
				status = elite_nfc_transfer_ready(mtd);
#else 
				elite_wait_nfc_ready(info);
				status = elite_nfc_transfer_ready(mtd);
#endif
				if (status)	
				{
					printk(KERN_ERR "dma transfer data is not ready: %x\n",
					readb((volatile char *)info->reg + ELITE_NFC_REG_A));
					writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,
							(volatile char *)info->reg + ELITE_NFC_REG_23);
				}

			}	
			spin_unlock(&elite_nand_lock);
			LEAVE();
			return;

		case NAND_CMD_PAGEPROG:
		case NAND_CMD_ERASE2:
			writeb(command, (volatile char *)info->reg + ELITE_NFC_REG_2);
			b2r_stat = readb((volatile char *)info->reg + ELITE_NFC_REG_B);
			writeb(B2R|b2r_stat,(volatile char *) info->reg + ELITE_NFC_REG_B);
			status = elite_wait_chip_ready(mtd);
			if (status)
				printk(KERN_NOTICE"The chip is not ready\n");
			writel(SP_CMD_EN|DPAHSE_DISABLE|((addr_cycle+1)<<1)|NFC_TRIGGER, (volatile char *)info->reg + ELITE_NFC_REG_1);

			info->datalen = 0;
			status = elite_nand_ready(mtd);
			if (status) 
			{
				printk(KERN_ERR "program or erase: nand flash is not ready\n");
				writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,
						(volatile char *)info->reg + ELITE_NFC_REG_23);
			}
			/* write page, don't check ecc */
			status = elite_nfc_wait_idle(mtd, 1, 1, -1, -1, read_phase);
			if (status < 0) 
			{
				printk(KERN_ERR "page program or erase err, nand controller is not idle\n");
				spin_unlock(&elite_nand_lock);
				LEAVE();
				return;
			}	
			spin_unlock(&elite_nand_lock);
			LEAVE();
			return;

		case NAND_CMD_RESET:

			if (!chip->dev_ready)
				break;
			udelay(chip->chip_delay);
			writel(command, (volatile char *)info->reg + ELITE_NFC_REG_2);
			/* write to clear B2R */
			b2r_stat = readb((volatile char *)info->reg + ELITE_NFC_REG_B);
			writeb(B2R|b2r_stat, (volatile char *)info->reg + ELITE_NFC_REG_B);

			writel(SP_CMD_EN|DPAHSE_DISABLE|(0x01<<1)|NFC_TRIGGER, (volatile char *)info->reg + ELITE_NFC_REG_1);
			status = elite_nand_ready(mtd);
			if (status) 
			{
				printk(KERN_ERR "Reset err, nand device is not ready\n");
				writew(readw((volatile char *)info->reg + ELITE_NFC_REG_23) | READ_RESUME,(unsigned char *)info->reg + ELITE_NFC_REG_23);
			}

			elite_read_nand_status(mtd, NAND_CMD_STATUS);
			while (!((readb((volatile char *)info->reg + ELITE_NFC_REG_0) & 0xff) & NAND_STATUS_READY))
						;

			NAND_PRINTF( "Reset status is ok\n");
			spin_unlock(&elite_nand_lock);
			LEAVE();
			return;

		case NAND_CMD_READID:
			status = elite_nand_read_id(mtd);
			NAND_PRINTF("readID status is %d\n", status);	
			spin_unlock(&elite_nand_lock);
			LEAVE();
			return;

		case NAND_CMD_STATUS:
			elite_read_nand_status(mtd, command);	
			spin_unlock(&elite_nand_lock);
			LEAVE();
			return;

		case NAND_CMD_RNDIN:
			if (column != -1) 
			{
				writeb(column, (volatile char *)info->reg + ELITE_NFC_REG_3);
				addr_cycle++;
				if (mtd->writesize != 512) 
				{
					writeb(column >> 8, (unsigned char *)((volatile char *)info->reg + ELITE_NFC_REG_3) + 1);
					addr_cycle++;
				}
			}

			/* set command 1 cycle */
			writeb(command, (volatile char *)info->reg + ELITE_NFC_REG_2);
			writel(((addr_cycle + 1)<<1)|NFC_TRIGGER,(volatile char *) info->reg + ELITE_NFC_REG_1);
			status = elite_nfc_wait_idle(mtd, 1, -1, -1, -1, read_phase); /* don't check ecc, wait nfc idle */
			if (status)
				printk(KERN_ERR "Ramdom input err: nfc is not idle\n");
			spin_unlock(&elite_nand_lock);
			LEAVE();
			return;

		case NAND_CMD_RNDOUT:

			if (column != -1) 
			{
				writeb(column, (volatile char *)info->reg + ELITE_NFC_REG_3);
				writeb(column, (volatile char *)info->reg + ELITE_NFC_REG_3 + 1);
				addr_cycle += 2;
			}
			/* CLEAR ECC BIT */
			writeb(0x07, (volatile char *)info->reg + ELITE_NFC_REG_1F);
			writel(0xffffffff, (volatile char *)info->reg + ELITE_NFC_REG_20);
			/* write to clear B2R */
			b2r_stat = readb((volatile char *)info->reg + ELITE_NFC_REG_B);
			writeb(B2R|b2r_stat, (volatile char *)info->reg + ELITE_NFC_REG_B);
			/* set command 1 cycle */
			writeb(command, (volatile char *)info->reg + ELITE_NFC_REG_2);
			writel(SP_CMD_EN|DPAHSE_DISABLE|((addr_cycle + 1)<<1)|NFC_TRIGGER, (volatile char *)info->reg + ELITE_NFC_REG_1);
			status = elite_wait_cmd_ready(mtd);
			if (status) 
			{
				printk(KERN_ERR "Ramdom output err: nfc command is not ready\n");
			}

			writeb(NAND_CMD_RNDOUTSTART, (volatile char *)info->reg + ELITE_NFC_REG_2);
			/* write to clear B2R */
			b2r_stat = readb((volatile char *)info->reg + ELITE_NFC_REG_B);
			writeb(B2R|b2r_stat, (volatile char *)info->reg +ELITE_NFC_REG_B);
			writel(SP_CMD_EN|NAND2NFC|(1<<1)|NFC_TRIGGER, (volatile char *)info->reg + ELITE_NFC_REG_1);
			status = elite_wait_cmd_ready(mtd);
			if (status) 
			{
				printk(KERN_ERR "Ramdom output err: nfc io transfer is not finished\n");
			}
			/* reduntant aera check ecc, wait nfc idle */
			status = elite_nfc_wait_idle(mtd, 0, -1, -1, -1, read_phase);
			if (status)
				printk(KERN_ERR "Ramdom output err: nfc is not idle\n");
			spin_unlock(&elite_nand_lock);
			LEAVE();
			return;


		case NAND_CMD_STATUS_ERROR:
		case NAND_CMD_STATUS_ERROR0:
			udelay(chip->chip_delay);		
			spin_unlock(&elite_nand_lock);
			LEAVE();
			return;


		default:
			/* If we don't have access to the busy pin, we apply the given command delay */
			/* trigger command and addrress cycle */
			if (!chip->dev_ready) 
			{
				udelay(chip->chip_delay);				
				spin_unlock(&elite_nand_lock);
				LEAVE();
				return;
			}
	}
	
	/* Apply this short delay always to ensure that we do wait tWB in */
	/* any case on any machine.*/
	elite_device_ready(mtd);
	spin_unlock(&elite_nand_lock);
	LEAVE();
}


static void elite_nand_select_chip(struct mtd_info *mtd, int chipnr)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	unsigned int b2r_stat;
	ENTER();

	NAND_PRINTF(KERN_NOTICE "\r enter in elite_nand_select_chip() chipnr:%d\n",chipnr);

	if (chipnr > 1)
		printk(KERN_WARNING "There are only support two chip sets\n");

	b2r_stat = readb((volatile char *)info->reg + ELITE_NFC_REG_B);
	writeb(B2R|b2r_stat, (volatile char *)info->reg +  ELITE_NFC_REG_B);

	if (chipnr >= 0 && chipnr < 2)		
		writeb(chipnr&0xf,  (unsigned char *)info->reg + ELITE_NFC_REG_11);	 
	else if (chipnr < 0)		
		writeb(~0,  (unsigned char *)info->reg + ELITE_NFC_REG_11);	 
	else		
		 printk(KERN_WARNING "There are only support two chip sets. chipnr = %d\n", chipnr);

	//writeb(0, (volatile char *)info->reg + ELITE_NFC_REG_11);
	LEAVE();
}

static void elite_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	ENTER();
	NAND_PRINTF("enter in elite_nand_write_buf()\n");
	memcpy(info->dmabuf + info->datalen, buf, len);
	info->datalen += len;
	LEAVE();
}

static void elite_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	ENTER();
	NAND_PRINTF("enter in elite_nand_read_buf() len: %x infoDatalen :%x\n", len, info->datalen);
	memcpy(buf, info->dmabuf + info->datalen, len);
	info->datalen += len;
	LEAVE();
}

static uint8_t elite_read_byte(struct mtd_info *mtd)
{
	uint8_t d;
	ENTER();
	NAND_PRINTF("enter in elite_nand_read_byte()\n");
	elite_nand_read_buf(mtd, &d, 1);
	LEAVE();
	return d;
}

static int elite_nand_read_oob(struct mtd_info *mtd, struct nand_chip *chip, int page, int sndcmd)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	uint8_t *buf = chip->oob_poi;
	uint8_t *bufpoi = buf;
#ifndef NAND_HARMING_ECC
	int pos;   /* toread, sndrnd = 1;*/
	int eccsize = chip->ecc.size;
#endif
	ENTER();
	NAND_PRINTF("\r enter in elite_nand_read_oob()\n");
#ifdef NAND_HARMING_ECC
	chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
	chip->read_buf(mtd, bufpoi, 64);
#endif
#ifdef NAND_HARMING_ECC
	chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
	chip->read_buf(mtd,bufpoi,64);
#else
	writeb(readb((volatile char *)info->reg + ELITE_NFC_REG_9) | 0x2,(volatile char *)info->reg + ELITE_NFC_REG_9);
	if (mtd->writesize <= 8192)
		pos = eccsize*chip->ecc.steps + chip->ecc.bytes * (chip->ecc.steps-1);
	else
		pos = 8192 + 218 + 0xA0;//or 4096 + 0xA0
	chip->cmdfunc(mtd, NAND_CMD_READOOB, pos, page);
        
	NAND_PRINTF("pos:0x%x,page:0x%x\n",pos, page);
	memcpy(bufpoi, (volatile char *)info->reg+ECC_FIFO_0, 64);
	memcpy(info->dmabuf, bufpoi, 64);
    {
		int i=0;
		for(;i<64;i++)
		{
			NAND_PRINTF("nand_read_oob:0x%x\n",bufpoi[i]);
		}
	}
	writeb(readb((volatile char *)info->reg + ELITE_NFC_REG_9) &0XD,(volatile char *)info->reg + ELITE_NFC_REG_9);
#endif
	LEAVE();
	return 1;
}

/****************************************************
* elite_nand_read_bb_oob - OOB data read function
* @mtd:	mtd info structure
* @chip:	nand chip info structure
* @page:	page number to read
* @sndcmd:flag whether to issue read command or not
****************************************************/
static int elite_nand_read_bb_oob(struct mtd_info *mtd, struct nand_chip *chip,
									int page, int sndcmd)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	ENTER();
	NAND_PRINTF(KERN_NOTICE "\r enter in elite_nand_read_bb_oob()\n");

	/* disable hardware ECC  */
	writeb(readb((volatile char *)info->reg + ELITE_NFC_REG_12) | (USE_SW_ECC), (volatile char *)info->reg + ELITE_NFC_REG_12);
	ecc_type = 0;
	 /* off hardware ecc */
	elite_nfc_ecc_set(info, 0);  
	 /* harming ECC structure for bad block check*/
	set_ecc_engine(info, 0);  
	if (sndcmd) 
	{
		chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
		sndcmd = 0;
	}
	chip->read_buf(mtd, chip->oob_poi, 64);
	 /* on hardware ecc  */
	elite_nfc_ecc_set(info, ECC4BIT);  
#ifndef NAND_HARMING_ECC
	ecc_type = 1;
	if (CONFIG_MTD_NAND_HM_ECC == 12)
		/* BCH ECC structure 12bit ecc engine*/
		set_ecc_engine(info, ECC12BIT);  
	else if (CONFIG_MTD_NAND_HM_ECC == 8)
		/* BCH ECC structure 8bit ecc engine*/
		set_ecc_engine(info, ECC8BIT);  
	else if (CONFIG_MTD_NAND_HM_ECC == 24)
		/* BCH ECC structure 8bit ecc engine*/
		set_ecc_engine(info, ECC24BIT);  
	else if (CONFIG_MTD_NAND_HM_ECC == 40)
		/* BCH ECC structure 8bit ecc engine*/
		set_ecc_engine(info,ECC40BIT);  
	else
		 /* BCH ECC structure 4bit ecc engine*/
		set_ecc_engine(info, ECC4BIT); 
#else
	if (mtd->writesize != 512)
		 /* harming ECC structure for bad block check*/
		set_ecc_engine(info, ECC1BIT); 
#endif
	LEAVE();
	return sndcmd;
}

static int elite_nand_write_oob(struct mtd_info *mtd, struct nand_chip *chip, int page)
{

	int pos, status;
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	ENTER();
	info->datalen = 0;
	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);
	elite_nfc_dma_cfg(mtd, mtd->oobsize, 1, -1, -1);
	printk("ECC.SIZE:%d\n",chip->ecc.size);
	printk("ECC.STEP:%d\n",chip->ecc.steps);
	printk("ECC.bytes:%d\n",chip->ecc.bytes);
	pos=(chip->ecc.steps-1)*chip->ecc.bytes+mtd->writesize;

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, pos,page);
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	status = chip->waitfunc(mtd, chip);
	if (status & NAND_STATUS_FAIL)
	{
		LEAVE();
		//while(1);
		return -EIO;
	}
	LEAVE();
	return 0;

}


/*********************************************************
* elite_nand_read_page - hardware ecc syndrom based page read
* @mtd:	mtd info structure
* @chip:	nand chip info structure
* @buf:	buffer to store read data
*
* The hw generator calculates the error syndrome automatically. Therefor
* we need a special oob layout and handling.
*********************************************************/
static int elite_nand_read_page(struct mtd_info *mtd, struct nand_chip *chip,
						 uint8_t *buf, int page)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	ENTER();
	NAND_PRINTF("\r enter in elite_nand_read_page()\n");
	info->datalen = 0;
	chip->read_buf(mtd, buf, mtd->writesize);

	NAND_PRINTF("\r enter in nand_read_page(): mtd->writesize is %d and oobsize is %d\n",
				mtd->writesize, mtd->oobsize);
	if (mtd->writesize == 2048) 
	{
		memcpy(chip->oob_poi, (volatile char *)info->reg+ECC_FIFO_0, mtd->oobsize);
	} 
	else if (mtd->writesize == 4096 || mtd->writesize == 8192) 
	{
		memcpy(chip->oob_poi, (volatile char *)info->reg+ECC_FIFO_0, 64);
#ifdef NAND_HARMING_ECC
		memcpy(chip->oob_poi+64, (volatile char *)info->reg+ECC_FIFO_0, 64);
#endif
	} 
	else 
	{  
		/* pagesize = 512 */
		/* only reduntant area read enable */
		memcpy(chip->oob_poi,(volatile char *) info->reg+ECC_FIFO_0, mtd->oobsize);
	}
	{
		int i=0;
		for(;i<64;i++)
		{
			NAND_PRINTF("and_read_page _oob:0x%x\n",chip->oob_poi[i]);
		}
	}
	LEAVE();
	return 0;
}

/**
*  elite_nand_write_page_lowlevel - hardware ecc syndrom based page write
*  @mtd:    mtd info structure
*  @chip:  nand chip info structure
*  @buf:  data buffer
*
*  The hw generator calculates the error syndrome automatically. Therefor
*  we need a special oob layout and handling.
*/
static void elite_nand_write_page_lowlevel(struct mtd_info *mtd, struct nand_chip *chip, const uint8_t *buf)
{
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	ENTER();
	NAND_PRINTF("enter in elite_nand_page_write_lowlevel() writesize %x,oobsize %x\n", mtd->writesize,mtd->oobsize);
	{
		int i=0;
		for(;i<64;i++)
		{
			NAND_PRINTF("nand_write_oob:%x\n",chip->oob_poi[i]);
		}
	}

	info->datalen = 0;
	chip->write_buf(mtd, buf, mtd->writesize);
	/*  2048bytes  */
	elite_nfc_dma_cfg(mtd, mtd->writesize, 1, 0, -1);  

	if (mtd->writesize == 2048) 
	{
		memcpy((volatile char *)info->reg+ECC_FIFO_0, chip->oob_poi, mtd->oobsize);
		/* solve a hardware bug --- bank 3, byte 7, bit 7 error  */
	} 
	else if (mtd->writesize == 4096) 
	{
		memcpy((volatile char *)info->reg+ECC_FIFO_0, chip->oob_poi, 64);
#ifdef NAND_HARMING_ECC
		memcpy((volatile char *)info->reg+ECC_FIFO_0, chip->oob_poi+64, 64);
#endif
		/* solve a hardware bug --- bank 7, byte 7, bit 7 error  */
	} 
	else if (mtd->writesize == 8192) 
	{
		memcpy((volatile char *)info->reg+ECC_FIFO_0, chip->oob_poi, 64);
#ifdef NAND_HARMING_ECC
		memcpy((volatile char *)info->reg+ECC_FIFO_0, chip->oob_poi+64, 64);
#endif
		writel(readl((volatile char *)info->reg + ELITE_NFC_REG_D) | 0x08, (volatile char *)info->reg + ELITE_NFC_REG_D);
		memset((volatile char *)info->reg+ECC_FIFO_0, 0xff, 64);
		writel(readl((volatile char *)info->reg + ELITE_NFC_REG_D) & 0xF7, (volatile char *)info->reg + ELITE_NFC_REG_D);
		/* solve a hardware bug --- bank 7, byte 7, bit 7 error  */
	} 
	else 
	{
		memcpy((volatile char *)info->reg+ECC_FIFO_0, chip->oob_poi, mtd->oobsize);
	}

	*((volatile char *)info->reg+ECC_FIFO_0+63)=0x00;
	*((volatile char *)info->reg+ECC_FIFO_0+62)=0x00;
	*((volatile char *)info->reg+ECC_FIFO_0+61)=0x00;
	*((volatile char *)info->reg+ECC_FIFO_0+60)=0x00;

	LEAVE();

}

static int elite_nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
									const uint8_t *buf, int page, int cached, int raw)
{
	int status;
	struct elite_nand_info *info =elite_nand_mtd_toinfo(mtd);
	ENTER();

	NAND_PRINTF("enter in elite_nand_write_page()\n");
	NAND_PRINTF("raw = %d, and ecc_type = %d\n", raw, ecc_type);
#ifdef NAND_BBT_BCH_ECC
	if (raw == 1 && ecc_type == 1) 
	{ 
		/* nand old structure  */
		printk("old structure: enter in elite_nand_write_page()\n");
		ecc_type = 0;
		set_ecc_engine(info, ECC1BIT);  /* old structure for file system write*/
		if (mtd->writesize == 2048)
			chip->ecc.layout = &elite_hm_oobinfo_2048;
		else
			chip->ecc.layout = &elite_hm_oobinfo_2048;
	}	
	else if (raw == 0 && ecc_type == 0) 
	{  
		/* nand new structure  */
		NAND_PRINTF("new structure: enter in elite_nand_write_page()\n");
		ecc_type = 1;
		if (CONFIG_MTD_NAND_HM_ECC == 12)
			/* new structure for bad block check*/
			set_ecc_engine(info, ECC12BIT); 
		else if (CONFIG_MTD_NAND_HM_ECC == 8)
			/* new structure for bad block check*/
			set_ecc_engine(info, ECC8BIT); 
		else if (CONFIG_MTD_NAND_HM_ECC == 24)
			/* new structure for bad block check*/
			set_ecc_engine(info, ECC24BIT); 
		else if (CONFIG_MTD_NAND_HM_ECC == 40)
			/* new structure for bad block check*/
			set_ecc_engine(info, ECC40BIT); 
		else
			/* new structure for bad block check*/
			set_ecc_engine(info, ECC4BIT); 

		if (mtd->writesize == 2048)
			chip->ecc.layout = &elite_oobinfo_2048;
		else 
		{
		      if (CONFIG_MTD_NAND_HM_ECC == 12)
				chip->ecc.layout = &elite_12bit_oobinfo_4096;
			else if (CONFIG_MTD_NAND_HM_ECC == 8)
				chip->ecc.layout = &elite_nand_8bit_oobinfo_4096;
			else if (CONFIG_MTD_NAND_HM_ECC == 24)
				chip->ecc.layout = &elite_nand_24bit_oobinfo_8192;
			else if (CONFIG_MTD_NAND_HM_ECC == 40)
				chip->ecc.layout = &elite_nand_40bit_oobinfo_8192;
			else
				chip->ecc.layout = &elite_oobinfo_4096;
		}
	}
#endif
	info->datalen = 0;
	chip->ecc.write_page(mtd, chip, buf);
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);

	status =elite_nand_pdma_handler(mtd);
	elite_nand_free_pdma(mtd);
	if (status)
		printk(KERN_ERR "check write pdma handler status= %x \n", status);

	/*
	* Cached progamming disabled for now, Not sure if its worth the
	* trouble. The speed gain is not very impressive. (2.3->2.6Mib/s)
	*/
	cached = 0;
	if (!cached || !(chip->options & NAND_CACHEPRG)) 
	{
		chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
		
		/* See if operation failed and additional status checks are available*/
		if ((status & NAND_STATUS_FAIL) && (chip->errstat))
			status = chip->errstat(mtd, chip, FL_WRITING, status,page);

		if (status & NAND_STATUS_FAIL)
		{
			LEAVE();
			//while(1);
			return -EIO;
		}
	} 
	else
	{
		chip->cmdfunc(mtd, NAND_CMD_CACHEDPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
	}

#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	/* Send command to read back the data */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);
	if (chip->verify_buf(mtd, buf, mtd->writesize))
	{
		LEAVE();
		return -EIO;
	}
#endif
	LEAVE();
	return 0;
}


/*elite_nand_init_chip init a single instance of an chip*/
static void elite_nand_init_chip(struct elite_nand_info *info, struct elite_nand_mtd *nmtd)
{
	struct nand_chip *chip = &nmtd->chip;
	struct mtd_info *mtd = &nmtd->mtd;
	ENTER();
	if (hardware_ecc) 
	{
		if (mtd->writesize == 2048) 
		{
			chip->ecc.size      = 512;
			chip->ecc.bytes     = 8;
			chip->ecc.steps     = 4;
			chip->ecc.prepad    = 1;
			chip->ecc.postpad   = 8;
		} 
		else if (mtd->writesize == 4096 ) 
		{
			chip->ecc.size      = 512;
			if (CONFIG_MTD_NAND_HM_ECC == 12)
				chip->ecc.bytes     = 20;
			else if (CONFIG_MTD_NAND_HM_ECC == 8)
				chip->ecc.bytes     = 16;
			else
				chip->ecc.bytes     = 8;

			chip->ecc.steps     = 8;
			chip->ecc.prepad    = 1;
			chip->ecc.postpad   = 8;

		} 
		else if (mtd->writesize == 8192)
		{
			chip->ecc.size = 1024;
			if (CONFIG_MTD_NAND_HM_ECC == 24)
			{
				chip->ecc.bytes = 42;
				chip->ecc.layout = &elite_nand_24bit_oobinfo_8192;
			}
			else if (CONFIG_MTD_NAND_HM_ECC == 40)
			{
				chip->ecc.bytes = 70;
				chip->ecc.layout = &elite_nand_40bit_oobinfo_8192;
			}
			else
			{
				BUG_ON(1);
			}
			chip->ecc.steps = 8;
			chip->ecc.prepad = 1;
			chip->ecc.postpad = 8;
			chip->bbt_td = &elite_bbt_main_descr_2048;
			chip->bbt_md = &elite_bbt_mirror_descr_2048;
		}	
		else 
		{   
			/*  512 page   */
			chip->ecc.size    = 512;
			chip->ecc.bytes   = 3;
			chip->ecc.steps     = 1;
			chip->ecc.prepad    = 4;
			chip->ecc.postpad   = 9;
		}

		chip->write_page = elite_nand_write_page;
		chip->ecc.write_page = elite_nand_write_page_lowlevel;
		chip->ecc.write_oob = elite_nand_write_oob;
		chip->ecc.read_page = elite_nand_read_page;
		chip->ecc.read_oob = elite_nand_read_oob;
		chip->ecc.hwctl  = elite_nand_enable_hwecc;

	} 
	else
		chip->ecc.mode    = NAND_ECC_SOFT;
	LEAVE();
}

static int elite_nand_remove(struct platform_device *pdev)
{
	struct elite_nand_info *info = dev_get_drvdata(&pdev->dev);
	ENTER();
	dev_set_drvdata(&pdev->dev, NULL);
	if (info == NULL)
	{
		LEAVE();
		return 0;
	}

	/* first thing we need to do is release all our mtds
	* and their partitions, then go through freeing the
	* resources used
	*/
	if (info->mtds != NULL) 
	{
		struct elite_nand_mtd *ptr = info->mtds;
		nand_release(&ptr->mtd);
		kfree(info->mtds);
	}

	/* free the common resources */
	if ((volatile char *)info->reg != NULL) 
	{
		iounmap((volatile char *)info->reg);
		info->reg = NULL;
	}

	if (info->area != NULL) 
	{
		release_resource(info->area);
		kfree(info->area);
		info->area = NULL;
	}
	kfree(info);
	LEAVE();
	return 0;
}

#ifdef CONFIG_MTD_CMDLINE_PARTS
extern int mtdpart_setup(char *);
extern int parse_mtd_partitions(struct mtd_info *master, const char **types,
				struct mtd_partition **pparts,
				struct mtd_part_parser_data *data);
static int __init add_dynamic_parts(struct mtd_info *mtd)
{
	static const char *part_parsers[] = { "cmdlinepart", NULL };
	struct mtd_partition *parts;
	int c;
	ENTER();
	c = parse_mtd_partitions(mtd, /*part_parsers*/NULL, &parts, NULL);

	if (c <= 0)
	{
		LEAVE();
		return -1;
	}
	add_mtd_partitions(mtd, parts, c);
	LEAVE();
	return 0;
}

#else

static inline int add_dynamic_parts(struct mtd_info *mtd)
{
	ENTER();
	LEAVE();
	return -1;
}

#endif

int search_mtd_table(char *string, char *ret)
{
	int i, err = 0;
	ENTER();
	for (i = 0; i < MAX_ELITE_MTD_PARTS; i++) 
	{
		if (!&nand_partitions[i]) 
		{
			err = 1;
			break;
		}
		NAND_PRINTF("MTD dev%d size: %8.8llx \"%s\"\n",i, nand_partitions[i].size, nand_partitions[i].name);
		if (strcmp(string, nand_partitions[i].name) == 0)
		{
			*ret = i;
			break;
		}
	}
	LEAVE();
	return err;
}
	
/*Lch */
static int elite_recovery_call(struct notifier_block *nb, unsigned long code, void *_cmd)
{
	ENTER();
	LEAVE();
	return NOTIFY_DONE;
}

static int elite_nand_probe(struct platform_device *pdev)
{
	struct elite_nand_info *info;
	struct elite_nand_mtd *nmtd;
	struct mtd_info *mtd;
	struct resource *res;
	unsigned int varlen;
	char ret1;
	int err = 0, ret = 0, status = 0;
	unsigned char varval[100], tmp[100];
	int size;
	ENTER();
	
	pr_debug("elite_nand_probe(%p)\n", pdev);

	nand_device = &pdev->dev;

	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		dev_err(&pdev->dev, "no memory for flash info\n");
		err = -ENOMEM;
		goto exit_error;
	}

	memzero(info, sizeof(*info));
	dev_set_drvdata(&pdev->dev, info);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res) {
		return -ENXIO;
	}

	spin_lock_init(&info->controller.lock);
	init_waitqueue_head(&info->controller.wq);
	spin_lock_init(&elite_nand_lock);

	/* allocate and map the resource */
	/* currently we assume we have the one resource */
	info->area = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!info->area) {
		dev_err(&pdev->dev, "cannot reserve register region\n");
		err = -ENOENT;
		goto exit_error;
	}

	info->device  = &pdev->dev;
	info->reg = (void __iomem *)ioremap(res->start, resource_size(res));
	if (!info->reg) {
		dev_err(&pdev->dev, "cannot reserve register region\n");
		err = -EIO;
		goto exit_error;
	}

	size = sizeof(*info->mtds);
	info->mtds = kmalloc(size, GFP_KERNEL);
	if (info->mtds == NULL) {
		dev_err(&pdev->dev, "failed to allocate mtd storage\n");
		err = -ENOMEM;
		goto exit_error;
	}

	memzero(info->mtds, size);
	nmtd = info->mtds;
	elite_nfc_ecc_set(info, 1);  /* on hw ecc */
	mtd = &nmtd->mtd;
	info->dmabuf = dma_alloc_coherent(&pdev->dev, 8640 + 0x300, &info->dmaaddr, GFP_KERNEL);

	if (!info->dmabuf || (info->dmaaddr & 0x0f)) 
	{
        BUG_ON(1);
		err = -ENOMEM;
		goto out_free_dma;
	}

	nmtd->chip.cmdfunc      = elite_nand_cmdfunc;
	nmtd->chip.dev_ready    = elite_device_ready;
	nmtd->chip.read_byte    = elite_read_byte;
	nmtd->chip.write_buf    = elite_nand_write_buf;
	nmtd->chip.read_buf     = elite_nand_read_buf;
	nmtd->chip.select_chip  = elite_nand_select_chip;
	nmtd->chip.chip_delay   = 20;
	nmtd->chip.priv	   = nmtd;
	nmtd->chip.bbt_options	   = NAND_BBT_LASTBLOCK | NAND_BBT_USE_FLASH | NAND_BBT_PERCHIP;
	nmtd->chip.ecc.mode	    = NAND_ECC_HW;

	NAND_PRINTF("initialising (%p, info %p)\n", nmtd, info);	
	if (mtd->writesize == 512) {
		printk(KERN_NOTICE "elite_oobinfo_512 \n");
		nmtd->chip.ecc.layout = &elite_oobinfo_512;
		nmtd->chip.bbt_td = &elite_bbt_main_descr_512;
		nmtd->chip.bbt_md = &elite_bbt_mirror_descr_512;
	} else { 
		nmtd->chip.bbt_td = &elite_bbt_main_descr_2048;
		nmtd->chip.bbt_md = &elite_bbt_mirror_descr_2048;
	}

	nmtd->info   = info;
	nmtd->mtd.priv  = &nmtd->chip;
	nmtd->mtd.owner   = THIS_MODULE;
	nmtd->mtd.reboot_notifier.notifier_call =elite_recovery_call;

	set_ecc_engine(info, ECC4BIT);
	info->datalen = 0;
	if(elite_nfc_init(info, &nmtd->mtd))
	{
		dev_err(&pdev->dev, "failed to init nand\n");
		err = -EIO;
		goto exit_error;
	}
	writeb(0xff, (volatile char *)info->reg + ELITE_NFC_REG_11); //chip disable 

	elite_nand_init_chip(info, nmtd);
#ifndef NAND_HARMING_ECC
	if (CONFIG_MTD_NAND_HM_ECC == 12) 
	{
		printk(KERN_NOTICE "BCH ECC 12BIT \n");
		set_ecc_engine(info, ECC12BIT);  /* BCH ECC new structure */
	}
	else  if (CONFIG_MTD_NAND_HM_ECC == 8) 
	{
		printk(KERN_NOTICE "BCH ECC 8BIT \n");
		set_ecc_engine(info, ECC8BIT);  /* BCH ECC new structure */
	}
	else  if (CONFIG_MTD_NAND_HM_ECC == 24) 
	{
		printk(KERN_NOTICE "BCH ECC 24BIT \n");
		set_ecc_engine(info, ECC24BIT);  /* BCH ECC new structure */
	}
	else  if (CONFIG_MTD_NAND_HM_ECC == 40) 
	{
		printk(KERN_NOTICE "BCH ECC 40BIT \n");
		set_ecc_engine(info, ECC40BIT);  /* BCH ECC new structure */
	}
	else
		set_ecc_engine(info, ECC4BIT);  /* BCH ECC new structure */

	#else
#ifdef NAND_BBT_BCH_ECC
	ecc_type = 1;
	if (CONFIG_MTD_NAND_HM_ECC == 12) 
	{
		printk(KERN_NOTICE "BCH ECC 12BIT \n");
		set_ecc_engine(info, ECC12BIT);  /* BCH ECC new structure */
	}
	else   if (CONFIG_MTD_NAND_HM_ECC == 8) 
	{
		printk(KERN_NOTICE "BBT BCH ECC 8BIT \n");
		set_ecc_engine(info, ECC8BIT); /* write bbt with BCH ECC new structure */
	} 
	else  if (CONFIG_MTD_NAND_HM_ECC == 24) 
	{
		printk(KERN_NOTICE "BCH ECC 24BIT \n");
		set_ecc_engine(info, ECC24BIT);  /* BCH ECC new structure */
	}
	else  if (CONFIG_MTD_NAND_HM_ECC == 40) 
	{
		printk(KERN_NOTICE "BCH ECC 40BIT \n");
		set_ecc_engine(info, ECC40BIT);  /* BCH ECC new structure */
	}
	else
		set_ecc_engine(info, ECC4BIT); /* write bbt with BCH ECC new structure */

#else
	set_ecc_engine(info, ECC1BIT);  /* Harming ECC  */
#endif
#endif
	if (mtd->writesize == 2048) 
	{
		nmtd->chip.ecc.size = 512;
		nmtd->chip.ecc.bytes = 8;
		nmtd->chip.ecc.steps = 4;
		nmtd->chip.ecc.prepad = 1;
		nmtd->chip.ecc.postpad = 8;

#ifndef NAND_HARMING_ECC
		nmtd->chip.ecc.layout = &elite_oobinfo_2048;
#else
#ifdef NAND_BBT_BCH_ECC
		nmtd->chip.ecc.layout = &elite_oobinfo_2048;
#else
		nmtd->chip.ecc.layout = &elite_hm_oobinfo_2048;
#endif
#endif
		nmtd->chip.bbt_td = &elite_bbt_main_descr_2048;
		nmtd->chip.bbt_md = &elite_bbt_mirror_descr_2048;

	} 
	else if (mtd->writesize == 4096 ) 
	{
	
		nmtd->chip.ecc.size = 512;
		nmtd->chip.ecc.steps = 8;
		nmtd->chip.ecc.prepad = 1;
		nmtd->chip.ecc.postpad = 8;
#ifndef NAND_HARMING_ECC
		if(CONFIG_MTD_NAND_HM_ECC == 12)
		{
			nmtd->chip.ecc.bytes = 20;
			nmtd->chip.ecc.layout = &elite_12bit_oobinfo_4096;
		}
		else if(CONFIG_MTD_NAND_HM_ECC == 8)
		{
			nmtd->chip.ecc.bytes = 16;
			nmtd->chip.ecc.layout = &elite_nand_8bit_oobinfo_4096;
		}
		else
		{
			nmtd->chip.ecc.bytes = 8;
			nmtd->chip.ecc.layout = &elite_oobinfo_4096;
		}
#else
#ifdef NAND_BBT_BCH_ECC
		if(CONFIG_MTD_NAND_HM_ECC == 12)
		{
			nmtd->chip.ecc.bytes = 20;
			nmtd->chip.ecc.layout = &elite_12bit_oobinfo_4096;
		}
		else if(CONFIG_MTD_NAND_HM_ECC == 8)
		{
			nmtd->chip.ecc.bytes = 16;
			nmtd->chip.ecc.layout = &elite_nand_8bit_oobinfo_4096;
		}
		else
		{
			nmtd->chip.ecc.bytes = 8;
			nmtd->chip.ecc.layout = &elite_oobinfo_4096;
		}
#else
		nmtd->chip.ecc.layout = &elite_hm_oobinfo_4096;

#endif
#endif
		nmtd->chip.bbt_td = &elite_bbt_main_descr_2048;
		nmtd->chip.bbt_md = &elite_bbt_mirror_descr_2048;
	} 
	else if (mtd->writesize == 8192)
	{
	   
		nmtd->chip.ecc.size = 1024;
		if (CONFIG_MTD_NAND_HM_ECC == 24)
		{
			nmtd->chip.ecc.bytes = 42;
			nmtd->chip.ecc.layout = &elite_nand_24bit_oobinfo_8192;
		}
		else if (CONFIG_MTD_NAND_HM_ECC == 40)
		{
			nmtd->chip.ecc.bytes = 70;
			nmtd->chip.ecc.layout = &elite_nand_40bit_oobinfo_8192;
		}
		else
		{
		    	BUG_ON(1);
		}
		nmtd->chip.ecc.steps = 8;
		nmtd->chip.ecc.prepad = 1;
		nmtd->chip.ecc.postpad = 8;
		nmtd->chip.bbt_td = &elite_bbt_main_descr_2048;
		nmtd->chip.bbt_md = &elite_bbt_mirror_descr_2048;

	}
	
	else 
	{
		nmtd->chip.ecc.size = 512;
		nmtd->chip.ecc.bytes = 3;
		nmtd->chip.ecc.steps = 1;
		nmtd->chip.ecc.prepad = 4;
		nmtd->chip.ecc.postpad = 9;
		nmtd->chip.ecc.layout = &elite_oobinfo_512;
		nmtd->chip.bbt_td = &elite_bbt_main_descr_512;
		nmtd->chip.bbt_md = &elite_bbt_mirror_descr_512;
	}
	nmtd->chip.ecc.mode        = NAND_ECC_HW;
	nmtd->chip.badblock_pattern =NULL;

	nmtd->scan_res = nand_scan(&nmtd->mtd, MAX_CHIP);
	printk("scan_res:0x%x\n",nmtd->scan_res);

	info->datalen = 0;
	if (nmtd->scan_res == 0) 
	{
		/* elite_nand_add_partition(info, nmtd, sets);*/
#ifdef NAND_HARMING_ECC
		ecc_type = 0;
		if (mtd->writesize == 2048) 
		{
			nmtd->chip.ecc.layout = &elite_hm_oobinfo_2048;
		} 
		else if (mtd->writesize == 4096 || mtd->writesize == 8192)
			nmtd->chip.ecc.layout = &elite_hm_oobinfo_4096;

		set_ecc_engine(info, ECC1BIT);  /* Harming ECC  */
#endif


#ifdef CONFIG_MTD_PARTITIONS
#ifdef CONFIG_MTD_CMDLINE_PARTS
		err = add_dynamic_parts(&nmtd->mtd);
		if (err < 0) {
			dev_err(&pdev->dev, "elite_nand: uboot no dynamic partitions defined, use default static\n");
			add_mtd_partitions(&nmtd->mtd, nand_partitions, ARRAY_SIZE(nand_partitions));
		}
#else
		add_mtd_partitions(&nmtd->mtd, nand_partitions, ARRAY_SIZE(nand_partitions));
#endif
#else
		add_mtd_device(&nmtd->mtd);
#endif
	}

	register_reboot_notifier(&mtd->reboot_notifier);
	dev_info(&pdev->dev, "nand initialised ok\n");
	LEAVE();
	return 0;

out_free_dma:
	dma_free_coherent(&pdev->dev, 8640 + 0x300, info->dmabuf, info->dmaaddr);

exit_error:
	elite_nand_remove(pdev);

	if (err == 0)
		err = -EINVAL;
	LEAVE();
	return err;
}

#ifdef CONFIG_PM_SLEEP
int elite_nand_suspend(struct device *dev)
{
	ENTER();
	if (((*(volatile unsigned long *)((void *)(IO_ADDRESS(GPIO_BASE_ADDR)) + 0x100))&6) == 2) 
	{
		*(volatile unsigned long *)((void *)(IO_ADDRESS(PMCEU_ADDR))) &= ~(0x0010000);/*add by vincent*/

		*(volatile unsigned long *)((void *)(IO_ADDRESS(NF_CTRL_CFG_BASE_ADDR)) + 0x88) |= (1<<5);
		printk("reset nand boot register NF_CTRL_CFG_BASE_ADDR + 0x88\n");
		*(volatile unsigned long *)((void *)(IO_ADDRESS(NF_CTRL_CFG_BASE_ADDR)) + 0x88) &= ~(1<<5);
	}
	NAND_PRINTF("elite_nand_suspend\n");
	LEAVE();
	return 0;
}

int elite_nand_resume(struct device *dev)
{
	struct elite_nand_info *info = dev_get_drvdata(dev);
	struct elite_nand_mtd *nmtd;
	ENTER();

	*(volatile unsigned long *)(IO_ADDRESS(PMCEU_ADDR)) |= (0x0010000);/*add by vincent*/
	if (info) 
	{
		nmtd = info->mtds;
		if (((*(volatile unsigned long *)((void *)(IO_ADDRESS(GPIO_BASE_ADDR)) + 0x100))&6) == 2)
			writel(0x0, (volatile char *)info->reg + ELITE_NFC_NANDFLASH_BOOT);
		/* initialise the hardware */
		if(elite_nfc_init(info, &nmtd->mtd))
		{
			printk("failed to resume nand\n");
			return  -EIO;
		}
		elite_nfc_ecc_set(info, 1);  /* on hw ecc */

		if (ecc_type == 1) 
		{	 /* nand new structure  */
		    if (CONFIG_MTD_NAND_HM_ECC == 12)
				set_ecc_engine(info, ECC12BIT); /* BCH ECC */
			else if (CONFIG_MTD_NAND_HM_ECC == 8)
				set_ecc_engine(info, ECC8BIT); /* BCH ECC */
			else if (CONFIG_MTD_NAND_HM_ECC == 40)
				set_ecc_engine(info,ECC40BIT); /* BCH ECC */
			else if (CONFIG_MTD_NAND_HM_ECC == 24)
				set_ecc_engine(info,ECC24BIT); /* BCH ECC */
			else
				set_ecc_engine(info, ECC4BIT); /* BCH ECC */
		} 
		else
			set_ecc_engine(info, 0);  /* Harmming ECC */

		printk("elite_nand_resume OK\n");
	} 
	else
		printk("elite_nand_resume error\n");

	LEAVE();
	return 0;
}
#endif //CONFIG_PM_SLEEP

#ifdef CONFIG_PM_RUNTIME
static int elite_nand_runtime_suspend(struct device *dev)
{
	return 0;
}
static int elite_nand_runtime_resume(struct device *dev)
{
	return 0;
}
#endif //CONFIG_PM_RUNTIME

static struct platform_device_id elite_nand_driver_ids[] = {
	{
		.name  = "elite-nand",
		.driver_data = (kernel_ulong_t)NULL,
	}, {
		.name  = "elite-nand.0",
		.driver_data = (kernel_ulong_t)NULL,
	},
	{ /* sentinel */ },
};


static const struct dev_pm_ops elite_nand_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(elite_nand_suspend, elite_nand_resume)
	SET_RUNTIME_PM_OPS(elite_nand_runtime_suspend,
			elite_nand_runtime_resume, NULL)
};

#ifdef CONFIG_OF
static const struct of_device_id elite_nand_match[] = {
	{ .compatible = "s3graphics,elite1000-nand" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, elite_nand_match);
#endif


struct platform_driver elite_nand_driver = {
	.driver = {
		.name = "s3graphics-elite-nand",
		.owner = THIS_MODULE,
		.pm = &elite_nand_dev_pm_ops,
		.of_match_table = of_match_ptr(elite_nand_match),
	},
	.id_table  = elite_nand_driver_ids,
	.probe = elite_nand_probe,
	.remove = elite_nand_remove,
};

static int __init elite_nand_init(void)
{
	return platform_driver_register(&elite_nand_driver);
}

static void __exit elite_nand_exit(void)
{	
	platform_driver_unregister(&elite_nand_driver);
}

module_init(elite_nand_init);
module_exit(elite_nand_exit);

MODULE_AUTHOR("VIA S3 Inc.");
MODULE_DESCRIPTION("ELITE [Nand Flash Interface] driver");
MODULE_LICENSE("GPL");

