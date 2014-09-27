/*
  * Cryptographic API.
  *
  * Support for ELITE cryptographic HW acceleration.
  *
  * Copyright (c) 2011 S3 Graphics co., Ltd.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as published
  * by the Free Software Foundation.
  *
  */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/crypto.h>
#include <linux/cryptohash.h>
#include <linux/interrupt.h>
#include <crypto/algapi.h>
#include <crypto/internal/hash.h>
#include <crypto/scatterwalk.h>
#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/hash.h>
#include <crypto/sha.h>
#include <crypto/ctr.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/atomic.h>
#include <linux/workqueue.h>

#include <mach/irqs.h>
#include <mach/iomap.h>

#define ELT_CTRL_CIPH_MODE_IDX		8
#define ELT_CTRL_DES_ALG_IDX		16	

/* Algorithm type mask. */
#define ELITE_CRYPTO_ALG_MASK		0x7
#define ELITE_CRYPTO_MODE_MASK		(0xf << ELT_CTRL_CIPH_MODE_IDX)
#define ELITE_CRYPTO_DES_MASK		(0xf << ELT_CTRL_DES_ALG_IDX)

#define ELT_CTRL_DES_ALG		(0x01 << ELT_CTRL_DES_ALG_IDX)
#define ELT_CTRL_3DES_ALG		(0x02 << ELT_CTRL_DES_ALG_IDX)

#define ELT_CTRL_CIPH_MODE_ECB		(0x00 << ELT_CTRL_CIPH_MODE_IDX)
#define ELT_CTRL_CIPH_MODE_CBC		(0x01 << ELT_CTRL_CIPH_MODE_IDX)
#define ELT_CTRL_CIPH_MODE_CTR		(0x02 << ELT_CTRL_CIPH_MODE_IDX)
#define ELT_CTRL_CIPH_MODE_OFB		(0x03 << ELT_CTRL_CIPH_MODE_IDX)
#define ELT_CTRL_CIPH_MODE_PCBC		(0x04 << ELT_CTRL_CIPH_MODE_IDX)
#define ELT_CTRL_CIPH_MODE_CFB		(0x05 << ELT_CTRL_CIPH_MODE_IDX)
#define ELT_CTRL_CIPH_MODE_CFB_dec	(0x05 << ELT_CTRL_CIPH_MODE_IDX)
#define ELT_CTRL_CIPH_MODE_CFB_enc	(0x06 << ELT_CTRL_CIPH_MODE_IDX)
#define ELT_CTRL_CIPH_MODE_ECB_CTS	(0x08 << ELT_CTRL_CIPH_MODE_IDX)
#define ELT_CTRL_CIPH_MODE_CBC_CTS	(0x09 << ELT_CTRL_CIPH_MODE_IDX)
#define ELT_CTRL_CIPH_MODE_PCBC_CTS	(0x0c << ELT_CTRL_CIPH_MODE_IDX)

//#define MAX_DDT_LEN			16

/* The priority to register each algorithm with. */
#define ELITE_CRYPTO_ALG_PRIORITY	300

#define MAX_HW_SHA_SIZE 		0xffffffffUL
#define MAX_DAM_BUFFER_SIZE		(SZ_64K - 1)
#define RESV_BUFF_SIZE			SZ_4M

/* DMA buffer length is limited to 64KBytes */
#define MAX_DIV_BUFF_SIZE		SZ_32K
#define MAX_NUM_SG			(RESV_BUFF_SIZE/MAX_DIV_BUFF_SIZE)

#define ROUNDUP(x,y)			((x + ((y)-1)) & ~((y)-1))

#define AES_CTRL_ECB			0x00
#define AES_CTRL_CBC			0x01
#define AES_CTRL_CTR			0x02
#define AES_CTRL_OFB			0x03

#define AES_CTRL_ENC			0x01
#define AES_CTRL_DEC			0x00

/* SHA1 Registers definition */
#define SHA1_REG_CTRL			0x0c
#define SHA1_MAX_HW_LEN			0x3ff

#define FLAGS_ALIGNED			1
#define FLAGS_UNALIGNED			2

/* AES Registers definition */
#define AES_REG_KEY1            	0x00 //AES key register base address (key1~key4)
#define AES_REG_KEY2            	0x04
#define AES_REG_KEY3            	0x08
#define AES_REG_KEY4            	0x0c
#define AES_REG_KEY5            	0x40
#define AES_REG_KEY6            	0x44
#define AES_REG_KEY7            	0x48
#define AES_REG_KEY8            	0x4c
#define AES_REG_KEY_CALCU       	0x10 //Start AES key calculation
#define AES_REG_DEC_ENC         	0x14 //AES Decrypt/Encrypt mode, AES key mode (128/192/256)
#define AES_REG_MODE            	0x18 //AES cipher operation mode
#define AES_REG_IV1             	0x1c //AES initial vector base register
#define AES_REG_IV2             	0x20
#define AES_REG_IV3             	0x24
#define AES_REG_IV4             	0x28
#define AES_REG_CTR_I	        	0x2c //AES CTR mode increment I value
#define AES_REG_STATUS_FULL     	0x30 //AES full status
#define AES_REG_STATUS_EMPTY    	0x34 //AES empty status
#define AES_REG_AKE             	0x38


/* SHA(SHA1/SHA224/SHA256) Registers definition */
#define SHA_REG_CTRL			0x50
#define SHA_REG_INPUT_LEN		0x54


/* DES registers definition */
#define DES_REG_CALCU           	0x60
#define DES_REG_KEY3_SET1       	0x64
#define DES_REG_KEY3_SET2       	0x68

#define DES3_REG_KEY3_SET1      	0x64
#define DES3_REG_KEY3_SET2      	0x68
#define DES3_REG_KEY2_SET1      	0x6c
#define DES3_REG_KEY2_SET2      	0x70
#define DES3_REG_KEY1_SET1      	0x74
#define DES3_REG_KEY1_SET2      	0x78

#define DES_REG_DEC_NEC         	0x7c
#define DES_REG_MODE            	0x80
#define DES3_REG_MODE           	0x84
#define DES_REG_IV_SET1         	0x88
#define DES_REG_IV_SET2         	0x8c
#define DES_REG_CTR_MODE        	0x90
#define DES_REG_STATUS_FULL     	0x94

/* RC4 registers definition */
#define RC4_REG_START_KSA_CALCU		0xA0 //Start RC4 KSA Key Calculation
#define RC4_REG_START_PRGA_CALCU	0xA4 //Start RC4 PRGA step Data Calculation
#define RC4_REG_KEY_LEN			0x2C //RC4 key length
#define RC4_REG_KEY1    		0xA8 //RC4 KEY Set1
#define RC4_REG_KEY2    		0xAC //RC4 KEY Set2

#define RC4_REG_KEY3    		0x64 //RC4 KEY Set3
#define RC4_REG_KEY4    		0x68 //RC4 KEY Set4
#define RC4_REG_KEY5    		0x6C //RC4 KEY Set5
#define RC4_REG_KEY6    		0x70 //RC4 KEY Set6
#define RC4_REG_KEY7    		0x74 //RC4 KEY Set7
#define RC4_REG_KEY8    		0x78 //RC4 KEY Set8
#define RC4_REG_SRAM_DATA		0xA8 //SW write SRAM data
#define RC4_REG_SRAM_ADDR		0xAC //SW write SRAM address [5:0]
#define RC4_REG_SRAM_EN			0xB0 //SW write SRAM enable [0:0]

/* MISC registers definition */
#define MISC_REG_MODE			0xC0
#define MISC_REG_RESET			0xC4
#define MISC_REG_ENABLE_CLOCK		0xC8
#define MISC_REG_ENABLE_INT		0xCC

/* DMA Registers definition */
#define DMA_REG_START			0xd0
#define DMA_REG_STATUS			0xd4
#define DMA_REG_IN_PRDADR		0xe0
#define DMA_REG_IN_RD			0xe4
#define DMA_REG_IN_CNT			0xe8
#define DMA_REG_OUT_PRDADR		0xf0
#define DMA_REG_OUT_RD			0xf4
#define DMA_REG_OUT_CNT			0xf8

/* efuse */
#define S3G_EFUSE_CTRL			0x250

enum cipher_mode {
	ELT_CTRL_CIPH_ALG_AES = 0,
	ELT_CTRL_CIPH_ALG_DES = 1,
	ELT_CTRL_CIPH_ALG_RC4 = 2,
	ELT_CTRL_CIPH_ALG_SHA = 4,
	ELT_CTRL_CIPH_ALG_EXT = 7,
	ELT_CTRL_CIPH_ALG_END,
};

enum sha_mode {
	ELT_CTRL_CIPH_ALG_SHA1 = ELT_CTRL_CIPH_ALG_END,
	ELT_CTRL_CIPH_ALG_SHA224,
	ELT_CTRL_CIPH_ALG_SHA256,
};

/* flags for security engine status  */
enum {
	FLAGS_SE_IDLE = 0,
	FLAGS_SE_DEQUE,
	FLAGS_SE_BUSY,
};


static void __iomem *reg_se_base;
static void __iomem *reg_pmu_base = (void *)IO_ADDRESS(ELITE_PMU_BASE);
static void __iomem *reg_tzpc_base = (void *)IO_ADDRESS(0xd8016000);

#define se_writel(value, reg) \
	__raw_writel(value, (reg_se_base + (reg)))
#define se_readl(reg) \
	__raw_readl(reg_se_base + (reg))

#define pmu_writel(value, reg) \
	__raw_writel(value, (reg_pmu_base + (reg)))
#define pmu_readl(reg) \
	__raw_readl(reg_pmu_base + (reg))

#define tzpc_writel(value, reg) \
	__raw_writel(value, (reg_tzpc_base + (reg)))
#define tzpc_readl(reg) \
	__raw_readl(reg_tzpc_base + (reg))

/*
 * DMA Physical Region Descriptor Table
 */
struct elite_dma_desc_tbl
{
	volatile unsigned long *databuffer_addr;	/* bit 0-31  Memory Region Physical Base Address */
	volatile unsigned long count:16;		/* bit 32-47 - Byte Count */
	volatile unsigned long reserved:15;		/* bit 48-62 - Reserved for future used */
	volatile unsigned long edt:1;			/* bit 63 - End flag of descriptor table */
} __packed __aligned(4);

struct elite_crypto_se;

struct elite_hash_reqctx {
	struct elite_crypto_se	*se;
	unsigned long sha_type;
	struct crypto_async_request	*req;
	
	unsigned char *result;

	int error;

	dma_addr_t	src_addr, dst_addr;
	struct elite_dma_desc_tbl *src_ddt, *dst_ddt;
	struct scatterlist sg;

	struct sg_mapping_iter src_sg_it;
	struct sg_mapping_iter dst_sg_it;
	
	unsigned int offset;	/* offset in current sg */
	unsigned int total;	/* total request */
	void (*complete)(struct elite_hash_reqctx *req, int err);
};

struct elite_hash_ctx {
	struct elite_crypto_se	*se;
	/* fallback stuff */
	struct crypto_shash	*fallback;
};


/* elite definition of a crypto algorithm. */
struct elite_alg {
	unsigned long ctrl_default;
	unsigned long type;
	union {
		struct crypto_alg alg;
		struct ahash_alg halg;
	} alg_u;
	struct elite_crypto_se	*se;
	struct list_head entry;
};

/* Block cipher context. */
struct elite_ablk_ctx {
	struct elite_crypto_se	*se;
	union {
		u8	aes[AES_MAX_KEY_SIZE];
		u8	des[DES_KEY_SIZE];
		u8	des3[3 * DES_KEY_SIZE];
		u8	arc4[258]; /* S-box, X, Y */
	} key;
	u8 nonce[CTR_RFC3686_NONCE_SIZE];
	u8 key_len;
	/*
	 * The fallback cipher. If the operation can't be done in hardware,
	 * fallback to a software version.
	 */
	struct crypto_ablkcipher *sw_cipher;
};


/************DES and Triple DES*********************/
struct elite_des_ctx {
	u32 expkey[DES_EXPKEY_WORDS];
};

struct elite_des3_ede_ctx {
	u32 expkey[DES3_EDE_EXPKEY_WORDS];
};
/***********************************************/


/*********************RC4***********************/
#define RC4_MIN_KEY_SIZE	1
#define RC4_MAX_KEY_SIZE	256
#define RC4_BLOCK_SIZE		1
/***********************************************/

#define ELITE_CRYPTO_QUEUE_LENGTH	10
 
/*
 * Asynchronous crypto request structure.
 *
 * This structure defines a request that is either queued for processing or
 * being processed.
 */
struct elite_ablk_reqctx {
	struct list_head list;
	struct elite_crypto_se	*se;
	struct crypto_async_request	*req;
	int	error;
	bool is_encrypt;
	unsigned ctx_id;
	
	unsigned long flags;
	/* for ciphertext stealing mode */
	unsigned int cts_num_bits;
	unsigned int cts_blk_num;

	dma_addr_t	src_addr, dst_addr;
	struct elite_dma_desc_tbl *src_ddt, *dst_ddt;
	void (*complete)(struct elite_ablk_reqctx *req, int err);
};

struct elite_crypto_se {
	unsigned long phys_base;
	void __iomem  *io_base;
	struct clk *clk;
	int irq;
	struct device *dev;
	atomic_t se_status_flags;
	int	err;

	u32 clock;

	spinlock_t queue_lock;

	struct list_head list_algs;

	struct elite_alg *algs;
	unsigned long num_algs;

	unsigned long	flags;

	struct completion op_comp;	

	const char *name;
	struct dma_pool	*req_pool;
	enum cipher_mode mode;
	
	//struct task_struct *queue_thread;
	struct workqueue_struct *workqueue;
	struct work_struct req_queue_work;
	
	struct crypto_queue	queue;

	struct crypto_async_request *req;
	unsigned int total;
	/* in_sg and out_sg could be the same scatterlist */
	struct scatterlist *in_sg; /* for crypto alg*/
	size_t in_offset;
	struct scatterlist *out_sg; /* for crypto alg*/
	size_t out_offset;

	size_t buflen;
	void *buf_in;
	size_t dma_size;
	dma_addr_t dma_addr_in;
	void *buf_out;
	dma_addr_t dma_addr_out;

	unsigned long *desc_vir_in;
	dma_addr_t desc_phy_in;
	
	unsigned long *desc_vir_out;
	dma_addr_t desc_phy_out;
	void *resv_buf_virt;
};

struct elite_crypto_se *engine;

/* keep registered devices data here */

/* function protypes  */
static int elite_ablk_hw_submit(struct elite_ablk_reqctx *req);
static int elite_sha_hw_submit(struct elite_hash_reqctx *req);
int elite_crypto_handle_req(struct crypto_async_request *req);
static void elite_ablk_complete(struct elite_ablk_reqctx *reqctx, int err);


static inline struct elite_alg *to_elite_alg(struct crypto_alg *alg)
{
	return alg ? container_of(alg, struct elite_alg, alg_u.alg) : NULL;
}

static inline struct elite_alg *to_elite_alg_ahash(struct ahash_alg *alg)
{
	return alg ? container_of(alg, struct elite_alg, alg_u.halg) : NULL;
}

/* ********************** DMA API ************************************ */
/* last sg of CTS-XXX-AES maybe unaligned */
static bool is_sg_aligned(struct scatterlist *sg_list, int aligned)
{
	struct scatterlist *sg = sg_list;

	while (!sg_is_last(sg)) {
		if (!IS_ALIGNED(sg->length, aligned))
			return false;
		sg = sg_next(sg);
	}

	/* no need to check the last sg */
	return true;
}

/* check to see if scatterlist buffer exceeds this maxinum size 64kbytes - 1 */
static bool is_sg_exceed_dmabuf_limit(struct scatterlist *sg_list)
{
	struct scatterlist *sg = sg_list;

	while (!sg_is_last(sg)) {
		if (sg->length > MAX_DAM_BUFFER_SIZE)
			return true;
		sg = sg_next(sg);
	}
 
	/* check the last sg */
	if (sg_dma_len(sg) > MAX_DAM_BUFFER_SIZE)
		return true;

	return false;
}

/* check to if these two scatterlist buffers are symmetrical */
static bool is_sg_symmetrical(struct scatterlist *sg_list1, struct scatterlist *sg_list2)
{
	struct scatterlist *sg1 = sg_list1;
	struct scatterlist *sg2 = sg_list2;

	for (;; sg1 = sg_next(sg1), sg2 = sg_next(sg2)) {
                if (sg1->offset != sg2->offset)
                        return false;
                if (sg1->length != sg2->length)
                        return false;
                if (sg_page(sg1) != sg_page(sg2))
                        return false;

		if (sg_is_last(sg1) ||  sg_is_last(sg2)) {
			if (!sg_is_last(sg1))
				return false;
			if (!sg_is_last(sg2))
				return false;
			return true;
		}
	}
	
	return true;
}

/* Count the number of scatterlist entries in a scatterlist. */
static int sg_count(struct scatterlist *sg_list, int nbytes)
{
	struct scatterlist *sg = sg_list;
	int sg_nents = 0;

	while (nbytes > 0) {
		++sg_nents;
		nbytes -= sg->length;
		sg = sg_next(sg);
	}

	return sg_nents;
}

void desc_tbl_set(struct elite_dma_desc_tbl *link_tbl_ptr, 
			dma_addr_t phys, size_t len, int end, int aligned)
{
	link_tbl_ptr->databuffer_addr = (unsigned long *)phys;
	link_tbl_ptr->count = ROUNDUP(len, aligned);
	link_tbl_ptr->reserved = 0;
	link_tbl_ptr->edt = end;
	
	if(end) {
		link_tbl_ptr->edt = 1;
	}
}

static void hexdump(void *buf, unsigned int len)
{
	print_hex_dump(KERN_CONT, "", DUMP_PREFIX_OFFSET,
			16, 1,
			buf, len, true);
}

static struct elite_dma_desc_tbl  *elite_sg_to_desc_tlb(struct elite_crypto_se *se,
				struct scatterlist *sg_list,
				unsigned nbytes,
				enum dma_data_direction dir,
				dma_addr_t *ddt_phys,
				int aligned)
{
	unsigned nents, mapped_ents;
	struct scatterlist *cur;
	struct elite_dma_desc_tbl *ddt;
	int i;

	nents = sg_count(sg_list, nbytes);
	mapped_ents = dma_map_sg(se->dev, sg_list, nents, dir);
	//if (mapped_ents + 1 > MAX_DDT_LEN)
	//	goto out;

	pr_debug("nents=%u, mapped_ents=%u, len=%u\n", nents, mapped_ents, nbytes);
	if (!mapped_ents) {
		pr_err("Fatal error: No entries mapped!\n");
		goto out;
	}
	//ddt = dma_pool_alloc(se->req_pool, GFP_ATOMIC, ddt_phys);
	ddt = dma_alloc_coherent(se->dev, mapped_ents * sizeof(struct elite_dma_desc_tbl), ddt_phys, GFP_KERNEL);
	if (!ddt)
		goto out;

	for_each_sg(sg_list, cur, mapped_ents, i) {
		BUG_ON(sg_dma_len(cur) > MAX_DAM_BUFFER_SIZE);
		desc_tbl_set(&ddt[i], sg_dma_address(cur), sg_dma_len(cur), sg_is_last(cur) ? 1:0, aligned);
	}
#if 0
	hexdump((void *)ddt, mapped_ents * sizeof(struct elite_dma_desc_tbl));
#endif
	return ddt;

out:
	dma_unmap_sg(se->dev, sg_list, nents, dir);
	return NULL;
}

static void elite_free_desc_tlb(struct elite_crypto_se	*se, struct elite_dma_desc_tbl *ddt,
			   dma_addr_t ddt_addr, struct scatterlist *sg_list,
			   unsigned nbytes, enum dma_data_direction dir)
{
	unsigned nents = sg_count(sg_list, nbytes);
	pr_debug("nents=%u, len=%u\n", nents, nbytes);
	dma_unmap_sg(se->dev, sg_list, nents, dir);

	//dma_pool_free(se->req_pool, ddt, ddt_addr);
	dma_free_coherent(se->dev, nents*sizeof(struct elite_dma_desc_tbl), ddt, ddt_addr);
}

#if 0
static void sg_copy_buf(void *buf, struct scatterlist *sg, 
						unsigned int start, unsigned int nbytes, int out)
{
	struct scatter_walk walk;

	if (!nbytes)
		return;
	scatterwalk_start(&walk, sg);
	scatterwalk_advance(&walk, start);
	scatterwalk_copychunks(buf, &walk, nbytes, out);
	scatterwalk_done(&walk, out, 0);
}
#endif

/* ********************** End of DMA API ************************************ */

irqreturn_t crypto_interrupt(int irq, void *data)
{
	struct elite_crypto_se *se = (struct elite_crypto_se *)data;

	if(!(se_readl(DMA_REG_STATUS) & 0x1))
		return IRQ_NONE;

	/* clear DMA status */
	se_writel(0x1, DMA_REG_STATUS);

	complete(&se->op_comp);

	return IRQ_HANDLED;
}

static int elite_crypto_handle_req_queue(struct elite_crypto_se *se)
{
	struct elite_ablk_reqctx *ablk_reqctx = NULL;
	struct elite_hash_reqctx *hash_reqctx = NULL;
	struct crypto_async_request *async_req;
	struct crypto_async_request *backlog;
	int ret, err = 0;
	u32 val;

	spin_lock_irq(&se->queue_lock);
	backlog = crypto_get_backlog(&se->queue);
	async_req = crypto_dequeue_request(&se->queue);
	if (async_req) {
		atomic_set(&se->se_status_flags, FLAGS_SE_BUSY);
	} else
		atomic_set(&se->se_status_flags, FLAGS_SE_IDLE);
		
	spin_unlock_irq(&se->queue_lock);
	if (backlog) {
		backlog->complete(backlog, -EINPROGRESS);
		backlog = NULL;
	}
	if (async_req) {
		if (async_req->tfm->__crt_alg->cra_type != &crypto_ahash_type) {
			struct ablkcipher_request *ablk_req = ablkcipher_request_cast(async_req); 
			ablk_reqctx = ablkcipher_request_ctx(ablk_req);

			elite_ablk_hw_submit(ablk_reqctx);
		} else {
			struct ahash_request *hash_req = ahash_request_cast(async_req);
			hash_reqctx = ahash_request_ctx(hash_req);

			hash_reqctx->req = async_req;
			elite_sha_hw_submit(hash_reqctx);
		}
	} else
		return -ENODATA;	

	INIT_COMPLETION(se->op_comp);

	ret = wait_for_completion_timeout(&se->op_comp,
				  msecs_to_jiffies(60));
	if (ret == 0) {
		dev_err(se->dev, "timed out (0x%x)\n",
			se_readl(DMA_REG_STATUS));
		err = -ETIMEDOUT;
	}

	val = se_readl(MISC_REG_MODE);

	switch(val) {
	case ELT_CTRL_CIPH_ALG_AES:
	case ELT_CTRL_CIPH_ALG_DES:
	case ELT_CTRL_CIPH_ALG_RC4:
		if(ablk_reqctx)
			ablk_reqctx->complete(ablk_reqctx, err);
		break;
	case ELT_CTRL_CIPH_ALG_SHA:
		if(hash_reqctx)
			hash_reqctx->complete(hash_reqctx, err);
		break;
	default:
		BUG();
		break;
	}
	
	return 0;
}

static void elite_crypto_workqueue_handler(struct work_struct *work)
{
	struct elite_crypto_se *se = container_of(work, struct elite_crypto_se, req_queue_work);
	int ret = 0;

	ret = clk_prepare_enable(se->clk);
	if (ret)
		BUG_ON("failed to enable security engine clock");
	/* handle every already enqueued request */
	do {
		ret = elite_crypto_handle_req_queue(se);
	} while (!ret);

	clk_disable_unprepare(se->clk);
}


static int elite_hash_cra_init(struct crypto_tfm *tfm)
{
	struct elite_hash_ctx *tctx = crypto_tfm_ctx(tfm);
	const char *alg_name = crypto_tfm_alg_name(tfm);
	struct crypto_alg *alg = tfm->__crt_alg;
	struct elite_alg *elite_alg = to_elite_alg(alg);

	tctx->se = elite_alg->se;
	/* Allocate a fallback and abort if it failed. */
	tctx->fallback = crypto_alloc_shash(alg_name, 0,
				CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(tctx->fallback)) {
		pr_err("elite-hash: fallback driver %s could not be loaded\n", alg_name);
		return PTR_ERR(tctx->fallback);
	}

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				sizeof(struct elite_hash_reqctx));

	return 0;
}


static void elite_hash_cra_exit(struct crypto_tfm *tfm)
{
	struct elite_hash_ctx *tctx = crypto_tfm_ctx(tfm);

	crypto_free_shash(tctx->fallback);
	tctx->fallback = NULL;
}

static int elite_hash_final_fallback(struct ahash_request *req)
{
	const struct elite_hash_ctx *tfm_ctx = crypto_tfm_ctx(req->base.tfm);
	struct elite_hash_reqctx *req_ctx = ahash_request_ctx(req);
	struct {
		struct shash_desc shash;
		char ctx[crypto_shash_descsize(tfm_ctx->fallback)];
	} desc;
	int rc, ret, num_sgs, copy_len, len = req_ctx->total;

	
	desc.shash.tfm = tfm_ctx->fallback;
	desc.shash.flags = CRYPTO_TFM_REQ_MAY_SLEEP;
	crypto_shash_init(&desc.shash);
	
	num_sgs = sg_count(req->src, req_ctx->total);
	sg_miter_start(&req_ctx->src_sg_it, req->src, num_sgs, SG_MITER_FROM_SG);
	while(len) {
		ret = sg_miter_next(&req_ctx->src_sg_it);
		BUG_ON(!ret);
		copy_len = min(req_ctx->src_sg_it.length, (size_t)len);
		crypto_shash_update(&desc.shash, req_ctx->src_sg_it.addr,
				    copy_len);
		len -= copy_len;
	}
	
	sg_miter_stop(&req_ctx->src_sg_it);
	rc = crypto_shash_final(&desc.shash, req->result);

	return rc;
}


static void elite_hash_complete(struct elite_hash_reqctx *reqctx, int err)
{
	struct ahash_request *ahash_req = ahash_request_cast(reqctx->req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(ahash_req);

	elite_free_desc_tlb(reqctx->se, reqctx->src_ddt, reqctx->src_addr, ahash_req->src,
			       ahash_req->nbytes, DMA_TO_DEVICE);
	elite_free_desc_tlb(reqctx->se, reqctx->dst_ddt, reqctx->dst_addr, &reqctx->sg,
			       crypto_ahash_digestsize(tfm), DMA_FROM_DEVICE);

	if (ahash_req->base.complete)
		ahash_req->base.complete(&ahash_req->base, err);

}

static int elite_sha_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct elite_hash_ctx *tctx = crypto_ahash_ctx(tfm);
	struct elite_hash_reqctx *reqctx = ahash_request_ctx(req);

	reqctx->complete = elite_hash_complete;
	reqctx->se = engine;
	tctx->se = engine;

	pr_debug("Hash digest size: %d\n", crypto_ahash_digestsize(tfm));

	if (crypto_ahash_digestsize(tfm) == SHA224_DIGEST_SIZE)
		reqctx->sha_type = ELT_CTRL_CIPH_ALG_SHA224;
	else if (crypto_ahash_digestsize(tfm) == SHA256_DIGEST_SIZE)
		reqctx->sha_type = ELT_CTRL_CIPH_ALG_SHA256;
	else if (crypto_ahash_digestsize(tfm) == SHA1_DIGEST_SIZE)
		reqctx->sha_type = ELT_CTRL_CIPH_ALG_SHA1;
	else
		return -EINVAL;

	reqctx->total = 0;

	/* create a scatter list for digest output */
	sg_init_one(&reqctx->sg, req->result, crypto_ahash_digestsize(tfm));

	return 0;
}

static int elite_sha_update(struct ahash_request *req)
{
	struct elite_hash_reqctx *reqctx = ahash_request_ctx(req);

	if (!req->nbytes)
		return 0;

	reqctx->total = req->nbytes;
	reqctx->offset = 0;


	return 0;
}

static int elite_sha_final(struct ahash_request *req)
{
	int rc = 0;
	bool next_sg;
	unsigned int offset = 0;
	struct sg_mapping_iter miter;
	static struct scatterlist sg[MAX_NUM_SG]; /* this array is too big */
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct elite_hash_reqctx *reqctx = ahash_request_ctx(req);

	if(reqctx->total > MAX_HW_SHA_SIZE) {
		rc = elite_hash_final_fallback(req);
		if (req->base.complete)
			req->base.complete(&req->base, rc);
		
		atomic_set(&reqctx->se->se_status_flags, FLAGS_SE_IDLE);

		return rc;
	}

	if(is_sg_aligned(req->src, 8)) {
		set_bit(FLAGS_ALIGNED, &reqctx->se->flags);
		reqctx->src_ddt = elite_sg_to_desc_tlb(reqctx->se, req->src, reqctx->total,
			DMA_TO_DEVICE, &reqctx->src_addr, 8);
	} else {
		/* should fall back to sw if request bigger than RESV_BUFF_SIZE */
		BUG_ON(reqctx->total > RESV_BUFF_SIZE);
		INIT_COMPLETION(reqctx->se->op_comp);
		sg_miter_start(&miter, req->src, sg_count(req->src, reqctx->total), SG_MITER_FROM_SG);
		while(offset < reqctx->total) {
			unsigned int len;

			next_sg = sg_miter_next(&miter);
			BUG_ON(!next_sg);

			len = min_t(unsigned int, (unsigned int)miter.length, (unsigned int)(RESV_BUFF_SIZE - offset));
			memcpy(reqctx->se->resv_buf_virt + offset, miter.addr, len);

			offset += len;

			if (offset == reqctx->total) {
				unsigned int i, count, remainder;

				count = offset / MAX_DIV_BUFF_SIZE;
				remainder = offset % MAX_DIV_BUFF_SIZE;

				if ((count + 1 < MAX_NUM_SG) && remainder > 0)
					count += 1;
				/* clear sg prior to using it */
				memset(sg, 0, sizeof(struct scatterlist) * MAX_NUM_SG);
				sg_init_table(sg, count);
				for (i = 0; i < count; i++) {
					sg_set_buf(&sg[i], reqctx->se->resv_buf_virt + MAX_DIV_BUFF_SIZE * i, 
							MAX_DIV_BUFF_SIZE);
				}
				if (remainder > 0) {
					sg_set_buf(&sg[i - 1], reqctx->se->resv_buf_virt + 
							MAX_DIV_BUFF_SIZE * (i - 1), remainder);
				}

				reqctx->src_ddt = elite_sg_to_desc_tlb(reqctx->se, sg, reqctx->total,
					DMA_TO_DEVICE, &reqctx->src_addr, 8);
				reqctx->dst_ddt = elite_sg_to_desc_tlb(reqctx->se, &reqctx->sg, crypto_ahash_digestsize(tfm),
					DMA_FROM_DEVICE, &reqctx->dst_addr, 1);

				elite_sha_hw_submit(reqctx);

				rc = wait_for_completion_timeout(&reqctx->se->op_comp,
						  msecs_to_jiffies(150));
				if (rc == 0) {
					dev_err(reqctx->se->dev, "timed out waiting for interrupt\n");
					WARN_ON_ONCE(1);
					rc = -ETIMEDOUT;
					goto out;
				}

				elite_free_desc_tlb(reqctx->se, reqctx->src_ddt, reqctx->src_addr, sg,
						       reqctx->total, DMA_TO_DEVICE);
				elite_free_desc_tlb(reqctx->se, reqctx->dst_ddt, reqctx->dst_addr, &reqctx->sg,
						       crypto_ahash_digestsize(tfm), DMA_FROM_DEVICE);
				rc = 0;	
			}
		}
out:
		sg_miter_stop(&miter);
		if (req->base.complete)
			req->base.complete(&req->base, rc);

		return rc;
	}
	
	/* hash digest can't be aligned with 8 */
	reqctx->dst_ddt = elite_sg_to_desc_tlb(reqctx->se, &reqctx->sg, crypto_ahash_digestsize(tfm),
		DMA_FROM_DEVICE, &reqctx->dst_addr, 1);

	return elite_crypto_handle_req(&req->base);
}

/* finup = update + final */
static int elite_sha_finup(struct ahash_request *req)
{
	int err1, err2;
	//struct elite_hash_reqctx *reqctx = ahash_request_ctx(req);
	
	if (!req->nbytes)
		return elite_sha_final(req);

	err1 = elite_sha_update(req);
	if ((err1 == -EINPROGRESS) || (err1 == -EBUSY))
		return err1;
	/*
	 * final() has to be always called to cleanup resources
	 * even if udpate() failed, except EINPROGRESS
	 */
	err2 = elite_sha_final(req);

	return err1 ?: err2;
}

/* digest = init + finup */
static int elite_sha_digest(struct ahash_request *req)
{
	return elite_sha_init(req) ?: elite_sha_finup(req);
}


int elite_crypto_handle_req(struct crypto_async_request *req)
{
	struct elite_crypto_se *se = engine;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&se->queue_lock, flags);
	ret = crypto_enqueue_request(&se->queue, req);
	spin_unlock_irqrestore(&se->queue_lock, flags);

	if (FLAGS_SE_IDLE == atomic_read(&se->se_status_flags))	
		queue_work(se->workqueue, &se->req_queue_work);

	return ret;
}

#if 0
/* Check if we need do a software fallback */
static int elite_ablk_need_fallback(struct elite_ablk_reqctx *reqctx)
{
	struct elite_ablk_ctx *ctx;
	struct crypto_tfm *tfm = reqctx->req->tfm;
	struct crypto_alg *alg = reqctx->req->tfm->__crt_alg;
	struct elite_alg *elite_alg = to_elite_alg(alg);

	ctx = crypto_tfm_ctx(tfm);

	return (elite_alg->ctrl_default & ELITE_CRYPTO_ALG_MASK) ==
			ELT_CTRL_CIPH_ALG_AES &&
			ctx->key_len != AES_KEYSIZE_128;

	return 0;
}
#endif

static void elite_ablk_complete(struct elite_ablk_reqctx *reqctx, int err)
{
	struct ablkcipher_request *ablk_req = container_of(reqctx->req, struct ablkcipher_request, base);

	if (ablk_req->src != ablk_req->dst) {
		elite_free_desc_tlb(reqctx->se, reqctx->src_ddt, reqctx->src_addr, ablk_req->src,
			       ablk_req->nbytes, DMA_TO_DEVICE);
		elite_free_desc_tlb(reqctx->se, reqctx->dst_ddt, reqctx->dst_addr, ablk_req->dst,
			       ablk_req->nbytes, DMA_FROM_DEVICE);
	} else {
		elite_free_desc_tlb(reqctx->se, reqctx->dst_ddt, reqctx->dst_addr, ablk_req->dst,
			       ablk_req->nbytes, DMA_BIDIRECTIONAL);
	}
	if (ablk_req->base.complete)
		ablk_req->base.complete(&ablk_req->base, err);

	//atomic_set(&reqctx->se->se_status_flags, FLAGS_SE_IDLE);
}

static inline void memcpy_toio32(u32 __iomem *dst, const void *src,
				 unsigned count)
{
	const u32 *src32 = (const u32 *) src;

	while (count--)
		writel(*src32++, dst++);
}

static int elite_sha_hw_submit(struct elite_hash_reqctx *reqctx)
{
	u32 ctrl = reqctx->sha_type, ctrl_val = 0;

	/* choose sha mode */
	if(ctrl == ELT_CTRL_CIPH_ALG_SHA224) {
		ctrl_val |= 1<<2;
		pr_debug("use sha224 hash agorithm\n");
	} else if(ctrl == ELT_CTRL_CIPH_ALG_SHA256) {
		ctrl_val |= 2<<2;
		pr_debug("use sha256 hash agorithm\n");
	} else if(ctrl == ELT_CTRL_CIPH_ALG_SHA1) {
		ctrl_val |= 0<<2;
		pr_debug("use sha1 hash agorithm\n");
	} else
		return -EINVAL;

	/* to do SHA mode */
	se_writel(ELT_CTRL_CIPH_ALG_SHA, MISC_REG_MODE);
	/* Set SHA mode clock enable */
	se_writel(0x2001, MISC_REG_ENABLE_CLOCK);
	/* enable SE DMA interrupt*/
	se_writel(0x1, MISC_REG_ENABLE_INT);
	/* setup input data length */
	se_writel(reqctx->total, SHA_REG_INPUT_LEN);
	/* clear DMA status */
	se_writel(0x1, DMA_REG_STATUS);
	/* specify the byte address of a physical memory region */
	se_writel((u32)reqctx->src_addr, DMA_REG_IN_PRDADR);
	se_writel((u32)reqctx->dst_addr, DMA_REG_OUT_PRDADR);

	/* enable 32-bit data bus byte-swap */
	ctrl_val &= ~0x02;
	ctrl_val |= 0x01; //start calculation
	se_writel(ctrl_val, SHA_REG_CTRL);
	/* Start security engine DMA */
	se_writel(0x00010001, DMA_REG_START);

	return 0;
}

void set_keys_ivs(u32 *set,const u8 *key, int len)
{
	int i, j;

	for (i=0; i<len; i+=4) {
		j = len/4-1-(i>>2);
		set[j] = (key[i+0]<<24) + (key[i+1]<<16) + (key[i+2]<<8) + key[i+3];
	}
}

void aes_set_keys(const u8 *key, int len)
{
	u32 keyset[8] = {0};

	set_keys_ivs(keyset, key, len);

	se_writel(keyset[0], AES_REG_KEY1);
	se_writel(keyset[1], AES_REG_KEY2);
	se_writel(keyset[2], AES_REG_KEY3);
	se_writel(keyset[3], AES_REG_KEY4);
	se_writel(keyset[4], AES_REG_KEY5);
	se_writel(keyset[5], AES_REG_KEY6);
	se_writel(keyset[6], AES_REG_KEY7);
	se_writel(keyset[7], AES_REG_KEY8);
}

void aes_set_ivs(const u8 *iv, int len)
{
	u32 ivset[4] = {0};

	set_keys_ivs(ivset, iv, len);

	se_writel(ivset[0], AES_REG_IV1);
	se_writel(ivset[1], AES_REG_IV2);
	se_writel(ivset[2], AES_REG_IV3);
	se_writel(ivset[3], AES_REG_IV4);
}

void des_set_keys(const u8 *key, int len)
{
	u32 keyset[2] = {0};

	set_keys_ivs(keyset, key, len);

	se_writel(keyset[0], DES_REG_KEY3_SET1);
	se_writel(keyset[1], DES_REG_KEY3_SET2);
}

void des3_set_keys(const u8 *key, int len)
{
	u32 keyset[6] = {0};

	set_keys_ivs(keyset, key, len);

	se_writel(keyset[0], DES3_REG_KEY3_SET1);
	se_writel(keyset[1], DES3_REG_KEY3_SET2);
	se_writel(keyset[2], DES3_REG_KEY2_SET1);
	se_writel(keyset[3], DES3_REG_KEY2_SET2);
	se_writel(keyset[4], DES3_REG_KEY1_SET1);
	se_writel(keyset[5], DES3_REG_KEY1_SET2);
}

void des3_des_set_ivs(const u8 *iv, int len)
{
	u32 ivset[2] = {0};

	set_keys_ivs(ivset, iv, len);

	se_writel(ivset[0], DES_REG_IV_SET1);
	se_writel(ivset[1], DES_REG_IV_SET2);
}

void rc4_set_keys(const u8 *key, int len)
{
	u32 keyset[8] = {0};

	set_keys_ivs(keyset, key, len);

	se_writel(keyset[0], RC4_REG_KEY1);
	se_writel(keyset[1], RC4_REG_KEY2);
	se_writel(keyset[2], RC4_REG_KEY3);
	se_writel(keyset[3], RC4_REG_KEY4);
	se_writel(keyset[4], RC4_REG_KEY5);
	se_writel(keyset[5], RC4_REG_KEY6);
	se_writel(keyset[6], RC4_REG_KEY7);
	se_writel(keyset[7], RC4_REG_KEY8);
}

/**
 * Only bit 0 of ar/awprot is valid
 * for encryption mode: source data in secure mode (range), output data in non-secure mode (range)
 * for decryption mode: source data in non-secure mode, output data in non-secure mode
 * arprot bit 0: 1 do encrypt; 0 do decrypt
 * awprot bit 0: 0 do encrypt; 1 do decrypt
 */
void aes_init_EfuseReg(u32 _testMode)
{
	#define CIPHER_ENCRYPT 0x1

	u8 decryptFlag = 0;
	u8 awprot = 0;
	u8 arprot = 0;
	u32 regval = se_readl(AES_REG_DEC_ENC);

	decryptFlag = (regval & (0x1 == CIPHER_ENCRYPT)) ? 0 : 1;//0: secure access; 1: non-secure access;
	arprot |= 1;//decryptFlag;//0: normal mode; 1: priviledge mode;
	arprot |= (1<<1);//0: secure range; 1: non secure range;
	arprot |= (0<<2);//0: data; 1: instruction

	awprot |= 1;//decryptFlag;//0: normal mode; 1: priviledge mode;
	awprot |= (0<<1);//0: secure range; 1: non secure range;
	awprot |= (1<<2);//0: data; 1: instruction

	regval = awprot << 8 | arprot << 4;
	if (_testMode)
		regval |= 1; //Force to use outside eFuseKey[127:0]
	//force to 128-bit key mode when run AES ,control values of output ports SE_ARPROT[2:0] and SE_AWPROT[2:0].
	else 
		regval &= ~0x1; //use register-configured sw-key

	se_writel(regval, S3G_EFUSE_CTRL);
}


void debug_print(const u8 *key, int len)
{
	int i;

	for(i=0; i<len; i++)
		printk("%02x ", key[i]);
	printk("\n");
}

static int elite_ablk_hw_submit(struct elite_ablk_reqctx *reqctx)
{
	struct crypto_tfm *tfm = reqctx->req->tfm;
	struct elite_ablk_ctx *ctx = crypto_tfm_ctx(tfm);
	struct ablkcipher_request *ablk_req = ablkcipher_request_cast(reqctx->req);
	struct crypto_alg *alg = reqctx->req->tfm->__crt_alg;
	struct elite_alg *elite_alg = to_elite_alg(alg);
	struct elite_crypto_se *se = ctx->se;
	//struct crypto_ablkcipher *tfm2 = crypto_ablkcipher_reqtfm(ablk_req);
	//unsigned int ivsize = crypto_ablkcipher_ivsize(tfm2);
	unsigned int ctrl, mode, des;
	u32 aes_ctrl = 0;

	ctrl = elite_alg->ctrl_default & ELITE_CRYPTO_ALG_MASK;
	mode = elite_alg->ctrl_default & ELITE_CRYPTO_MODE_MASK;
	des  = elite_alg->ctrl_default & ELITE_CRYPTO_DES_MASK;

	if (ELT_CTRL_CIPH_MODE_CFB == mode) {
		if (reqctx->is_encrypt)
			mode = ELT_CTRL_CIPH_MODE_CFB_enc;
		else
			mode = ELT_CTRL_CIPH_MODE_CFB_dec;
	}
	
	switch(ctrl) {
	case ELT_CTRL_CIPH_ALG_AES:
		/* to do AES mode */
		se_writel(ELT_CTRL_CIPH_ALG_AES, MISC_REG_MODE);
		/* reset cipher */
		se_writel(0, MISC_REG_RESET);			
		/* Set AES, SRAM, DMA clock enable */
		se_writel(0x805, MISC_REG_ENABLE_CLOCK);			
		/* enable SE DMA interrupt*/
		se_writel(0x1, MISC_REG_ENABLE_INT);			
		/* clear DMA status */
		se_writel(0x1, DMA_REG_STATUS);			
		/* specify the byte address of a physical memory region */
		se_writel((u32)reqctx->src_addr, DMA_REG_IN_PRDADR);
		se_writel((u32)reqctx->dst_addr, DMA_REG_OUT_PRDADR);
			
		if (reqctx->is_encrypt)
			pr_debug("AES encryptation mode\n");
		else
			pr_debug("AES decryptation mode\n");

		if(mode == ELT_CTRL_CIPH_MODE_CTR
		 || mode == ELT_CTRL_CIPH_MODE_OFB
		 || mode == ELT_CTRL_CIPH_MODE_CFB_dec)
			aes_ctrl |= 1;
		else
			aes_ctrl |= reqctx->is_encrypt ? 1:0;
		
		if(ctx->key_len == AES_KEYSIZE_128)
			aes_ctrl |= 0<<4;
		else if(ctx->key_len == AES_KEYSIZE_192)
			aes_ctrl |= 1<<4;
		else if(ctx->key_len == AES_KEYSIZE_256)
			aes_ctrl |= 2<<4;
		else {
			return -EINVAL;
			pr_err("Invalid AES key length!\n");
		}
		/* AES Decrypt or Encrypt mode setting */
		se_writel(aes_ctrl, AES_REG_DEC_ENC);
		pr_debug("register AES_REG_DEC_ENC: 0x%x\n", se_readl(AES_REG_DEC_ENC));
		
		/* AES cipher mode setting */
		if ((ELT_CTRL_CIPH_MODE_CBC_CTS == mode)
		  || (ELT_CTRL_CIPH_MODE_ECB_CTS == mode)
		  || (ELT_CTRL_CIPH_MODE_PCBC_CTS == mode)) {
			se_writel((reqctx->cts_num_bits << 8) | (mode >> ELT_CTRL_CIPH_MODE_IDX), AES_REG_MODE);
			se_writel(reqctx->cts_blk_num, AES_REG_CTR_I);
			pr_debug("CTS number of blocks: %u\n", reqctx->cts_blk_num);
			pr_debug("CTS number of bits: %u\n", reqctx->cts_num_bits);
		} else
			se_writel(mode >> ELT_CTRL_CIPH_MODE_IDX, AES_REG_MODE);
		
		pr_debug("register AES_REG_MODE: 0x%x\n", se_readl(AES_REG_MODE));
		
		aes_set_keys(ctx->key.aes, ctx->key_len);
		
		/* CBC mode IV or CTR mode ctr value setting */
		if ((ELT_CTRL_CIPH_MODE_CBC == mode)
		  || (ELT_CTRL_CIPH_MODE_OFB == mode)
		  || (ELT_CTRL_CIPH_MODE_PCBC == mode)
		  || (ELT_CTRL_CIPH_MODE_CFB_enc == mode)
		  || (ELT_CTRL_CIPH_MODE_CFB_dec == mode)
		  || (ELT_CTRL_CIPH_MODE_CBC_CTS == mode)
		  || (ELT_CTRL_CIPH_MODE_PCBC_CTS == mode)) {
			/* CBC mode IV setting */
			aes_set_ivs(ablk_req->info, /*ivsize*/AES_BLOCK_SIZE);
		} else if(ELT_CTRL_CIPH_MODE_CTR == mode ) {
			/* Write CTR mode ctr value */
			aes_set_ivs(ablk_req->info, /*ivsize*/AES_BLOCK_SIZE);
			/* CTR mode increment I value */
			se_writel(1, AES_REG_CTR_I);
		}
		
		se_writel(0x0, AES_REG_AKE); //redundant

		/* special setup for se if using efuse as aes-ecb key */
		if ((ELT_CTRL_CIPH_MODE_ECB == mode) && (ctx->key_len == AES_KEYSIZE_128) 
		 && !strncmp(ctx->key.aes, "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff", 
		 AES_KEYSIZE_128)) {
			/* PMU config: enable use efuse key bit */
			pmu_writel(pmu_readl(0xe320) | 0x1, 0xe320);
			/* TZPC */
			tzpc_writel(tzpc_readl(0x14) | (0x1 << 20), 0x14);
			/* PMU config: set secure memory range0 
			 * driver user must make sure that DMA buffers are consecutive
			 */
			pmu_writel(sg_dma_address(ablk_req->src), 0xd7e0);
			pmu_writel(sg_dma_address(sg_last(ablk_req->src, sg_count(ablk_req->src, ablk_req->nbytes))), 0xd7e4);
			aes_init_EfuseReg(1);
		} else {
			se_writel(0x0, S3G_EFUSE_CTRL);
			/* PMU config: clear use efuse key bit */
			pmu_writel(pmu_readl(0xe320) & (~0x1), 0xe320);
			/* TZPC */
			tzpc_writel(tzpc_readl(0x14) & (~(0x1 << 20)), 0x14);
			/* PMU config: secure memory range0 */
			pmu_writel(0, 0xd7e0);
			pmu_writel(0, 0xd7e4);
		}
		
		/* start the AES KEY calculation */
		se_writel(0x1, AES_REG_KEY_CALCU);
		/* wait for calculation finished */
		while(0x1 & se_readl(AES_REG_KEY_CALCU))
			cpu_relax();
		
		/* Start security engine DMA */
		se_writel(0x00010001, DMA_REG_START);
		
		pr_debug("AES operation mode: 0x%x\n", mode >> ELT_CTRL_CIPH_MODE_IDX);
		break;
	case ELT_CTRL_CIPH_ALG_DES:
		/* to do AES mode */
		se_writel(ELT_CTRL_CIPH_ALG_DES, MISC_REG_MODE);
		/* reset cipher */
		se_writel(0, MISC_REG_RESET);
		/* Set DES mode clock enable */
		se_writel(0x809, MISC_REG_ENABLE_CLOCK);
		/* enable SE DMA interrupt */
		se_writel(0x1, MISC_REG_ENABLE_INT);
		/* clear DMA status */
		se_writel(0x1, DMA_REG_STATUS);
		
		/* specify the byte address of a physical memory region */
		se_writel((u32)reqctx->src_addr, DMA_REG_IN_PRDADR);
		se_writel((u32)reqctx->dst_addr, DMA_REG_OUT_PRDADR);
		
		if (reqctx->is_encrypt)
			pr_debug("DES/TDES encryptation mode\n");
		else
			pr_debug("DES/TDES decryptation mode\n");
		/* DES Decrypt or Encrypt mode setting */
		if (ELT_CTRL_CIPH_MODE_OFB == mode || ELT_CTRL_CIPH_MODE_CTR == mode)
			se_writel(1, DES_REG_DEC_NEC);
		else
			se_writel(reqctx->is_encrypt ? 1 : 0, DES_REG_DEC_NEC);
		
		/* DES cipher mode setting */
		se_writel(mode >> ELT_CTRL_CIPH_MODE_IDX, DES_REG_MODE);
		
		/* DES or 3DES mode setting */
		if (des == ELT_CTRL_DES_ALG) {
			se_writel(0, DES3_REG_MODE);
		} else if (des == ELT_CTRL_3DES_ALG) {
			se_writel(1, DES3_REG_MODE);
		} else
			return -EINVAL;
		
		if((ELT_CTRL_CIPH_MODE_CBC == mode) || (ELT_CTRL_CIPH_MODE_OFB == mode)) {
			/* CBC mode IV or CTR mode ctr value setting */
			des3_des_set_ivs(ablk_req->info, /*ivsize*/DES_BLOCK_SIZE);
		} else if (ELT_CTRL_CIPH_MODE_CTR == mode) {/* CTR mode increment I value  */
			des3_des_set_ivs(ablk_req->info, /*ivsize*/DES_BLOCK_SIZE);
			se_writel(1, DES_REG_CTR_MODE);
		}
		
		/* DES 64 bits KEY */
		if(des == ELT_CTRL_DES_ALG)
			des_set_keys(ctx->key.des, ctx->key_len);
		else if(des == ELT_CTRL_3DES_ALG)
			des3_set_keys(ctx->key.des3, ctx->key_len);
		
		if (ELT_CTRL_CIPH_MODE_CBC == mode)
			se_writel(1, DES_REG_CTR_MODE);
		
		/* start the DES KEY calculation  */
		se_writel(0x1, DES_REG_CALCU);
		/* wait for calculation finished */
		while(0x1 & se_readl(DES_REG_CALCU))
			cpu_relax();
		
		/* Start security engine DMA */
		se_writel(0x00010001, DMA_REG_START);
		
		pr_debug("DES3/DES mode:0x%x\n", mode >> ELT_CTRL_CIPH_MODE_IDX);
		break;
	case ELT_CTRL_CIPH_ALG_RC4:
		/* to do RC4 mode */
		if(ctx->key_len <= 32)
			se_writel(ELT_CTRL_CIPH_ALG_RC4, MISC_REG_MODE);
		else
			se_writel(ELT_CTRL_CIPH_ALG_EXT, MISC_REG_MODE);
		pr_debug("MISC_REG_MODE: 0x%x\n", se_readl(MISC_REG_MODE));
		/* reset cipher */
		se_writel(0, MISC_REG_RESET);
		/* Set RC4 mode clock enable */
		se_writel(0x811, MISC_REG_ENABLE_CLOCK);
		/* enable SE DMA interrupt*/
		se_writel(0x1, MISC_REG_ENABLE_INT);
		/* clear DMA status */
		se_writel(0x1, DMA_REG_STATUS);
		
		/* specify the byte address of a physical memory region */
		se_writel((u32)reqctx->src_addr, DMA_REG_IN_PRDADR);
		se_writel((u32)reqctx->dst_addr, DMA_REG_OUT_PRDADR);
		
		if(ctx->key_len <= 32) { //KSA + PRGA by HW
			pr_debug("KSA + PRGA by HW\n");
			/* RC4 Key Length setting */
			se_writel(ctx->key_len - 1, RC4_REG_KEY_LEN);
			/* RC4 64 bits KEY setting (MSB first and LSB alignment) */
			rc4_set_keys(ctx->key.arc4, ctx->key_len);
		
			/* To start the KSA KEY calculation */
			se_writel(0x1, RC4_REG_START_KSA_CALCU);
			while(0x1 & se_readl(RC4_REG_START_KSA_CALCU))
				cpu_relax();
			pr_debug("KSA finished\n");
			/* start the PRGA (The pseudo-random generation algorithm), the DATA mode */
			se_writel(0x1, RC4_REG_START_PRGA_CALCU);
		} else { //Only PRGA by HW
			u32 addr;
		
			pr_debug("Only PRGA by HW\n");
		
			for(addr = 0; addr < 64; addr++) {
				se_writel(addr, RC4_REG_SRAM_ADDR);
				se_writel(*((u32 *)(ctx->key.arc4 + addr*4)), RC4_REG_SRAM_DATA);
				se_writel(0x1, RC4_REG_SRAM_EN);
			}
		
			/* re-select CIPHER mode as RC4 mode */
			se_writel(ELT_CTRL_CIPH_ALG_RC4, MISC_REG_MODE);
			/* start the PRGA (The pseudo-random generation algorithm), the DATA mode */
			se_writel(0x1, RC4_REG_START_PRGA_CALCU);
		}
			
		/* Start security engine DMA */
		se_writel(0x00010001, DMA_REG_START);
		break;
	default:
		dev_err(se->dev, "Algorithm is not supported in this driver\n");
		return -EINVAL;
	}

	return 0;
}

#if 0
static int elite_ablk_do_fallback(struct ablkcipher_request *req,
				  unsigned alg_type, bool is_encrypt)
{
	struct crypto_tfm *old_tfm =
	    crypto_ablkcipher_tfm(crypto_ablkcipher_reqtfm(req));
	struct elite_ablk_ctx *ctx = crypto_tfm_ctx(old_tfm);
	int err;

	if (!ctx->sw_cipher)
		return -EINVAL;

	/*
	 * Change the request to use the software fallback transform, and once
	 * the ciphering has completed, put the old transform back into the
	 * request.
	 */
	ablkcipher_request_set_tfm(req, ctx->sw_cipher);
	err = is_encrypt ? crypto_ablkcipher_encrypt(req) :
		crypto_ablkcipher_decrypt(req);
	ablkcipher_request_set_tfm(req, __crypto_ablkcipher_cast(old_tfm));

	return err;
}
#endif

/* Elite Security engine DMA buffers must be 16 bytes aligned */
static int elite_ablk_handle_unaliged_req(struct ablkcipher_request *req, bool is_encrypt)	
{
	struct crypto_alg *alg = req->base.tfm->__crt_alg;
	struct elite_crypto_se *se = to_elite_alg(alg)->se;
	struct elite_ablk_reqctx *reqctx = ablkcipher_request_ctx(req);
	struct sg_mapping_iter i_miter, o_miter;
	static struct scatterlist sg[MAX_NUM_SG]; /* this array is too big */
	unsigned int count, remainder, nents_src, nents_dst;
	unsigned int offset = 0, tmp = 0;
	int ret = 0, i;
	bool next_sg;

	reqctx->req = &req->base;
	reqctx->is_encrypt = is_encrypt;
	reqctx->se = se;
	reqctx->complete = NULL;
	reqctx->error = -EINPROGRESS;

	INIT_COMPLETION(se->op_comp);
	nents_src = sg_count(req->src, req->nbytes);
	nents_dst = sg_count(req->dst, req->nbytes);
	sg_miter_start(&i_miter, req->src, nents_src, SG_MITER_FROM_SG);
	sg_miter_start(&o_miter, req->dst, nents_dst, SG_MITER_TO_SG);

	/* input & output sg should share the same buffer length & size in each sg */
	while (offset < req->nbytes) {
		unsigned int len, leftover = 0, unfilled_len = 0;//, tmp = 0;

		if (leftover > 0) {
			memcpy(se->resv_buf_virt + leftover, i_miter.addr + 
					(i_miter.length - leftover), leftover);
			offset += leftover;
			tmp += leftover;
			leftover = 0;
		}
		next_sg = sg_miter_next(&i_miter);
		BUG_ON(!next_sg);

		len = min(i_miter.length, RESV_BUFF_SIZE - tmp);
		memcpy(se->resv_buf_virt + tmp, i_miter.addr, len);

		offset += len;
		tmp += len;
		if ((tmp == RESV_BUFF_SIZE) || (tmp == req->nbytes) || ((offset == req->nbytes))) {
			unsigned int position = 0;
			//hexdump(se->resv_buf_virt, req->nbytes);
			if (i_miter.length > len)
				leftover = i_miter.length - len;
			else
				leftover = 0;

			count = tmp / MAX_DIV_BUFF_SIZE;
			remainder = tmp % MAX_DIV_BUFF_SIZE;

			if ((count + 1 < MAX_NUM_SG) && remainder > 0)
				count += 1;
			/* clear sg prior to using it */
			memset(sg, 0, sizeof(struct scatterlist) * MAX_NUM_SG);
			sg_init_table(sg, count);
			for (i = 0; i < count; i++) {
				sg_set_buf(&sg[i], se->resv_buf_virt + MAX_DIV_BUFF_SIZE * i, 
						MAX_DIV_BUFF_SIZE);
			}
			if (remainder > 0)
				sg_set_buf(&sg[i - 1], se->resv_buf_virt + 
						MAX_DIV_BUFF_SIZE * (i - 1), remainder);

			reqctx->dst_ddt = elite_sg_to_desc_tlb(se, sg, tmp, 
				DMA_BIDIRECTIONAL, &reqctx->dst_addr, 16);
			if (!reqctx->dst_ddt)
				goto out;

			reqctx->src_ddt = NULL;
			reqctx->src_addr = reqctx->dst_addr;
			
			if(strstr(alg->cra_name, "cts")) {
				reqctx->cts_num_bits = (req->nbytes % 16) * 8;
				if (!reqctx->cts_num_bits)
					reqctx->cts_num_bits = 128;
				reqctx->cts_blk_num = ROUNDUP(req->nbytes, 16) / 16;
			}
			//set_bit(FLAGS_UNALIGNED, &se->flags);
			elite_ablk_hw_submit(reqctx);

			ret = wait_for_completion_timeout(&se->op_comp,
					  msecs_to_jiffies(60));
			if (ret == 0) {
				dev_err(se->dev, "timed out waiting for interrupt\n");
				//WARN_ON_ONCE(1);
				ret = -ETIMEDOUT;
				goto out;
			}

			elite_free_desc_tlb(se, reqctx->dst_ddt, reqctx->dst_addr, sg,
			       tmp, DMA_BIDIRECTIONAL);

			if (unfilled_len > 0) {
				memcpy(o_miter.addr + (i_miter.length - unfilled_len), 
						se->resv_buf_virt + position, unfilled_len);
				position += unfilled_len;
				unfilled_len = 0;
			}
			while (position < tmp) {
				next_sg = sg_miter_next(&o_miter);
				BUG_ON(!next_sg);
				
				len = min(o_miter.length, tmp - position);

				memcpy(o_miter.addr, se->resv_buf_virt + position, len);
				position += len;
			}
			if (i_miter.length > len)
				unfilled_len = i_miter.length - len;
			else
				unfilled_len = 0;

			tmp = 0;
		}
	}
	if (req->base.complete)
		req->base.complete(&req->base, 0);
	ret = 0;
out:
	sg_miter_stop(&i_miter);
	sg_miter_stop(&o_miter);
	
	return ret;
}

static int elite_ablk_setup(struct ablkcipher_request *req, unsigned alg_type,
			    bool is_encrypt)
{
	struct crypto_alg *alg = req->base.tfm->__crt_alg;
	struct elite_crypto_se *se = to_elite_alg(alg)->se;
	struct elite_ablk_reqctx *dev_req = ablkcipher_request_ctx(req);
	//unsigned long flags;
	//bool is_rc4 = strstr(alg->cra_name, "rc4") ? true: false;
	int ret = -ENOMEM;

	dev_req->req		= &req->base;
	dev_req->is_encrypt	= is_encrypt;
	dev_req->se		= se;
	dev_req->complete	= &elite_ablk_complete;
	dev_req->error		= -EINPROGRESS;
#if 0
	if (unlikely(elite_ablk_need_fallback(dev_req))
		return elite_ablk_do_fallback(req, alg_type, is_encrypt);
#endif
	/*
	 * Create the DDT's for the engine. If we share the same source and
	 * destination then we can optimize by reusing the DDT's.
	 */
	if (req->src != req->dst) {
		dev_req->src_ddt = elite_sg_to_desc_tlb(se, req->src,
			req->nbytes, DMA_TO_DEVICE, &dev_req->src_addr, 16);
		if (!dev_req->src_ddt)
			goto out;

		dev_req->dst_ddt = elite_sg_to_desc_tlb(se, req->dst,
			req->nbytes, DMA_FROM_DEVICE, &dev_req->dst_addr, 16);
		if (!dev_req->dst_ddt)
			goto out_free_src;
	} else {
		dev_req->dst_ddt = elite_sg_to_desc_tlb(se, req->dst,
			req->nbytes, DMA_BIDIRECTIONAL, &dev_req->dst_addr, 16);
		if (!dev_req->dst_ddt)
			goto out;

		dev_req->src_ddt = NULL;
		dev_req->src_addr = dev_req->dst_addr;
	}

	dev_req->cts_num_bits = 0;
	if(strstr(alg->cra_name, "cts")) {
		//int bsize = crypto_ablkcipher_blocksize(crypto_ablkcipher_reqtfm(req));
		dev_req->cts_num_bits = (req->nbytes % 16) * 8;
		if (!dev_req->cts_num_bits)
			dev_req->cts_num_bits = 128;
		dev_req->cts_blk_num = ROUNDUP(req->nbytes, 16) / 16;
	}

	/* inform the high level API to wait for completion */	
	//ret = -EINPROGRESS;

	ret = elite_crypto_handle_req(&req->base) ;
	if(!ret)
		goto out_free_ddts;

	return ret;

out_free_ddts:
	elite_free_desc_tlb(se, dev_req->dst_ddt, dev_req->dst_addr, req->dst,
		       req->nbytes, req->src == req->dst ?
		       DMA_BIDIRECTIONAL : DMA_FROM_DEVICE);
out_free_src:
	if (req->src != req->dst)
		elite_free_desc_tlb(se, dev_req->src_ddt, dev_req->src_addr,
			       req->src, req->nbytes, DMA_TO_DEVICE);
out:
	return ret;
}

/* ********************** ALG API ************************************ */

/*
 * Set the key for an AES block cipher. 
 */
static int elite_aes_setkey(struct crypto_ablkcipher *cipher, const u8 *key,
			    unsigned int keylen)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	//struct crypto_alg *alg = tfm->__crt_alg;
	struct elite_ablk_ctx *ctx = crypto_tfm_ctx(tfm);
	int err = 0;

	switch(keylen) {
	case AES_KEYSIZE_128:
	case AES_KEYSIZE_192:
	case AES_KEYSIZE_256:
		break;
	default:
		crypto_ablkcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

#if 0
	/*
	 * Elite security engine only supports 128 bit AES keys. If we get a
	 * request for any other size (192 and 256 bits) then we need to do a software
	 * fallback.
	 */
	if ((len != AES_KEYSIZE_128) && ctx->sw_cipher) {
		/*
		 * Set the fallback transform to use the same request flags as
		 * the hardware transform.
		 */
		ctx->sw_cipher->base.crt_flags &= ~CRYPTO_TFM_REQ_MASK;
		ctx->sw_cipher->base.crt_flags |=
			cipher->base.crt_flags & CRYPTO_TFM_REQ_MASK;

		err = crypto_ablkcipher_setkey(ctx->sw_cipher, key, len);
		if (err)
			goto sw_setkey_failed;
	} else if ((len != AES_KEYSIZE_128) && !ctx->sw_cipher)
		err = -EINVAL;
#endif
	memcpy(ctx->key.aes, key, keylen);
	ctx->key_len = keylen;
#if 0
sw_setkey_failed:
	if (err && ctx->sw_cipher) {
		tfm->crt_flags &= ~CRYPTO_TFM_RES_MASK;
		tfm->crt_flags |=
			ctx->sw_cipher->base.crt_flags & CRYPTO_TFM_RES_MASK;
	}
#endif
	return err;
}

static int elite_aes_rfc3686_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
		unsigned int key_len)
{
	struct elite_ablk_ctx *ctx = crypto_ablkcipher_ctx(tfm);

	/* the nonce is stored in bytes at end of key */
	if (key_len < CTR_RFC3686_NONCE_SIZE)
		return -EINVAL;

	memcpy(ctx->nonce, key + (key_len - CTR_RFC3686_NONCE_SIZE),
			CTR_RFC3686_NONCE_SIZE);

	key_len -= CTR_RFC3686_NONCE_SIZE;
	return elite_aes_setkey(tfm, key, key_len);
}


/*
 * Set the DES key for a block cipher transform. This also performs weak key
 * checking if the transform has requested it.
 */
static int elite_des_setkey(struct crypto_ablkcipher *cipher, const u8 *key,
			    unsigned int len)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct elite_ablk_ctx *ctx = crypto_tfm_ctx(tfm);
	u32 tmp[DES_EXPKEY_WORDS];

	if (len != DES_KEY_SIZE) {
		crypto_ablkcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	if (unlikely(!des_ekey(tmp, key)) &&
	    (crypto_ablkcipher_get_flags(cipher) & CRYPTO_TFM_REQ_WEAK_KEY)) {
		tfm->crt_flags |= CRYPTO_TFM_RES_WEAK_KEY;
		return -EINVAL;
	}

	memcpy(ctx->key.des, key, len);
	ctx->key_len = len;

	return 0;
}

static int elite_3des_setkey(struct crypto_ablkcipher *cipher, const u8 *key,
			    unsigned int len)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct elite_ablk_ctx *ctx = crypto_tfm_ctx(tfm);
	//u32 tmp[DES_EXPKEY_WORDS];

	if (len != DES3_EDE_KEY_SIZE) {
		crypto_ablkcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}
/*
	if (unlikely(!des_ekey(tmp, key)) &&
	    (crypto_ablkcipher_get_flags(cipher) & CRYPTO_TFM_REQ_WEAK_KEY)) {
		tfm->crt_flags |= CRYPTO_TFM_RES_WEAK_KEY;
		return -EINVAL;
	}
*/
	memcpy(ctx->key.des3, key, len);
	ctx->key_len = len;

	return 0;
}



/*
 * implement the key-scheduling algorithm (KSA)
 */
/* expand a key (makes a rc4_key) */
void rc4_expand_key(struct crypto_ablkcipher *cipher, const u8 *keydata, 
					unsigned len)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct elite_ablk_ctx *ctx = crypto_tfm_ctx(tfm);
	unsigned index1, index2, counter;
	unsigned char *state, *x, *y;
	
	state = ctx->key.arc4;
	x = state + 256;
	y = x + 1;

	for (counter = 0; counter < 256; counter++)
        	state[counter] = counter;

	*x = *y = index1 = index2 = 0;

	for (counter = 0; counter < 256; counter++) {
		index2 = (keydata[index1] + state[counter] + index2) & 255;

		/* swap */
		state[counter] ^= state[index2];
		state[index2]  ^= state[counter];
		state[counter] ^= state[index2];

		index1 = (index1 + 1) % len;
	}
}

static int elite_arc4_setkey(struct crypto_ablkcipher *cipher, const u8 *key, 
	unsigned int keylen)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct elite_ablk_ctx *ctx = crypto_tfm_ctx(tfm);

	if(keylen <= 32) {
		memcpy(ctx->key.arc4, key, keylen);
		ctx->key_len = keylen;
	} else {
		rc4_expand_key(cipher, key, keylen);
	}

	return 0;
}

static int elite_ablk_cra_init(struct crypto_tfm *tfm)
{
	struct elite_ablk_ctx *ctx = crypto_tfm_ctx(tfm);
	struct crypto_alg *alg = tfm->__crt_alg;
	struct elite_alg *elite_alg = to_elite_alg(alg);

	ctx->se = elite_alg->se;
	if (alg->cra_flags & CRYPTO_ALG_NEED_FALLBACK) {
		ctx->sw_cipher = crypto_alloc_ablkcipher(alg->cra_name, 0,
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK);
		if (IS_ERR(ctx->sw_cipher)) {
			dev_warn(ctx->se->dev, "failed to allocate fallback for %s\n",
				 alg->cra_name);
			ctx->sw_cipher = NULL;
		}
	}
	/* appoint the size of request context for the core layer to alloc memory for it */
	tfm->crt_ablkcipher.reqsize = sizeof(struct elite_ablk_reqctx);

	return 0;
}


static void elite_ablk_cra_exit(struct crypto_tfm *tfm)
{
	struct elite_ablk_ctx *ctx = crypto_tfm_ctx(tfm);

	if (ctx->sw_cipher)
		crypto_free_ablkcipher(ctx->sw_cipher);
	ctx->sw_cipher = NULL;
}

static int elite_ablk_encrypt(struct ablkcipher_request *req)
{
	struct crypto_ablkcipher *cipher = crypto_ablkcipher_reqtfm(req);
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct elite_alg *elt_alg = to_elite_alg(tfm->__crt_alg);

	if(is_sg_aligned(req->src, 16) && is_sg_aligned(req->dst, 16) &&
	   (!is_sg_exceed_dmabuf_limit(req->src)) && (!is_sg_exceed_dmabuf_limit(req->dst))
	   && is_sg_symmetrical(req->src, req->dst)) {
		return elite_ablk_setup(req, elt_alg->type, 1);
	} else {
		return elite_ablk_handle_unaliged_req(req, 1);
	}
}

static int elite_ablk_decrypt(struct ablkcipher_request *req)
{
	struct crypto_ablkcipher *cipher = crypto_ablkcipher_reqtfm(req);
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct elite_alg *elt_alg = to_elite_alg(tfm->__crt_alg);

	if(is_sg_aligned(req->src, 16) && is_sg_aligned(req->dst, 16) &&
	   (!is_sg_exceed_dmabuf_limit(req->src)) && (!is_sg_exceed_dmabuf_limit(req->dst))
	   && is_sg_symmetrical(req->src, req->dst)) {
		return elite_ablk_setup(req, elt_alg->type, 0);
	} else {
		return elite_ablk_handle_unaliged_req(req, 0);
	}
}

static int elite_ablk_rfc3686_encrypt(struct ablkcipher_request *req)
{
	struct crypto_ablkcipher *tfm = crypto_ablkcipher_reqtfm(req);
	struct elite_ablk_ctx *ctx = crypto_ablkcipher_ctx(tfm);
	u8 iv[CTR_RFC3686_BLOCK_SIZE];
	u8 *info = req->info;
	int ret;

	/* set up counter block */
	memcpy(iv, ctx->nonce, CTR_RFC3686_NONCE_SIZE);
	memcpy(iv + CTR_RFC3686_NONCE_SIZE, info, CTR_RFC3686_IV_SIZE);

	/* initialize counter portion of counter block */
	*(__be32 *)(iv + CTR_RFC3686_NONCE_SIZE + CTR_RFC3686_IV_SIZE) =
		cpu_to_be32(1);

	req->info = iv;
	ret = elite_ablk_encrypt(req);
	req->info = info;
	return ret;
}

static int elite_ablk_rfc3686_decrypt(struct ablkcipher_request *req)
{
	struct crypto_ablkcipher *tfm = crypto_ablkcipher_reqtfm(req);
	struct elite_ablk_ctx *ctx = crypto_ablkcipher_ctx(tfm);
	u8 iv[CTR_RFC3686_BLOCK_SIZE];
	u8 *info = req->info;
	int ret;

	/* set up counter block */
	memcpy(iv, ctx->nonce, CTR_RFC3686_NONCE_SIZE);
	memcpy(iv + CTR_RFC3686_NONCE_SIZE, info, CTR_RFC3686_IV_SIZE);

	/* initialize counter portion of counter block */
	*(__be32 *)(iv + CTR_RFC3686_NONCE_SIZE + CTR_RFC3686_IV_SIZE) =
		cpu_to_be32(1);

	req->info = iv;
	ret = elite_ablk_decrypt(req);
	req->info = info;
	return ret;
}


/* ********************** ALGS ************************************ */
static struct elite_alg security_engine_algs[] = {
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_AES | ELT_CTRL_CIPH_MODE_CBC,
		.alg_u.alg = {
			.cra_name = "cbc(aes)",
			.cra_driver_name = "cbc-aes-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
					/*CRYPTO_ALG_KERN_DRIVER_ONLY |*/
					CRYPTO_ALG_ASYNC /*|
					CRYPTO_ALG_NEED_FALLBACK*/,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx), //size of memory to be allocated
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_aes_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = AES_MIN_KEY_SIZE,
				.max_keysize = AES_MAX_KEY_SIZE,
#if 0
				.ivsize = AES_BLOCK_SIZE,
#endif
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_AES | ELT_CTRL_CIPH_MODE_ECB,
		.alg_u.alg = {
			.cra_name = "ecb(aes)",
			.cra_driver_name = "ecb-aes-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
				CRYPTO_ALG_KERN_DRIVER_ONLY |
				CRYPTO_ALG_ASYNC /*| CRYPTO_ALG_NEED_FALLBACK*/,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_aes_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = AES_MIN_KEY_SIZE,
				.max_keysize = AES_MAX_KEY_SIZE,
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_AES | ELT_CTRL_CIPH_MODE_OFB,
		.alg_u.alg = {
			.cra_name = "ofb(aes)",
			.cra_driver_name = "ofb-aes-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
				CRYPTO_ALG_KERN_DRIVER_ONLY |
				CRYPTO_ALG_ASYNC /*| CRYPTO_ALG_NEED_FALLBACK*/,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_aes_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = AES_MIN_KEY_SIZE,
				.max_keysize = AES_MAX_KEY_SIZE,
#if 0
				.ivsize = AES_BLOCK_SIZE,
				 /* Needed to be affirmed (Choose an IV Generator)
				  * <default>, <built-in>, chainiv (Chain IV Generator), seqiv, eseqiv 
				  */
				.geniv = "default",
#endif		
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_AES | ELT_CTRL_CIPH_MODE_CTR,
		.alg_u.alg = {
			.cra_name = "ctr(aes)",
			.cra_driver_name = "ctr-aes-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
				CRYPTO_ALG_KERN_DRIVER_ONLY |
				CRYPTO_ALG_ASYNC /*| CRYPTO_ALG_NEED_FALLBACK*/,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_aes_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = AES_MIN_KEY_SIZE,
				.max_keysize = AES_MAX_KEY_SIZE,
#if 0
				.ivsize = AES_BLOCK_SIZE,
				 /* Needed to be affirmed (Choose an IV Generator)
				  * <default>, <built-in>, chainiv (Chain IV Generator), seqiv, eseqiv 
				  */
				.geniv = "default",
#endif		
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
#if 0
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_AES | ELT_CTRL_CIPH_MODE_CTR,
		.alg_u.alg = {
			/* May be rfc4106, need to be affirmed */
			.cra_name = "rfc3686(ctr(aes))",
			.cra_driver_name = "rfc3686-ctr-aes-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
				CRYPTO_ALG_KERN_DRIVER_ONLY |
				CRYPTO_ALG_ASYNC /*| CRYPTO_ALG_NEED_FALLBACK*/,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_aes_rfc3686_setkey,
				.encrypt = elite_ablk_rfc3686_encrypt,
				.decrypt = elite_ablk_rfc3686_decrypt,
				.min_keysize = AES_MIN_KEY_SIZE,
				.max_keysize = AES_MAX_KEY_SIZE,
#if 0
				.ivsize = AES_BLOCK_SIZE,
				 /* Needed to be affirmed (Choose an IV Generator)
				  * <default>, <built-in>, chainiv (Chain IV Generator), seqiv, eseqiv 
				  */
				.geniv = "eseqiv",		
#endif
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
#endif
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_AES | ELT_CTRL_CIPH_MODE_PCBC,
		.alg_u.alg = {
			.cra_name = "pcbc(aes)",
			.cra_driver_name = "pcbc-aes-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
				CRYPTO_ALG_KERN_DRIVER_ONLY |
				CRYPTO_ALG_ASYNC /*| CRYPTO_ALG_NEED_FALLBACK*/,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_aes_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = AES_MIN_KEY_SIZE,
				.max_keysize = AES_MAX_KEY_SIZE,
#if 0
				.ivsize = AES_BLOCK_SIZE,
				 /* Needed to be affirmed (Choose an IV Generator)
				  * <default>, <built-in>, chainiv (Chain IV Generator), seqiv, eseqiv 
				  */
				.geniv = "<default>",	
#endif	
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_AES | ELT_CTRL_CIPH_MODE_CFB,
		.alg_u.alg = {
			.cra_name = "cfb(aes)",
			.cra_driver_name = "cfb-aes-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
				CRYPTO_ALG_KERN_DRIVER_ONLY |
				CRYPTO_ALG_ASYNC /*| CRYPTO_ALG_NEED_FALLBACK*/,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_aes_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = AES_MIN_KEY_SIZE,
				.max_keysize = AES_MAX_KEY_SIZE,
#if 0
				.ivsize = AES_BLOCK_SIZE,
				/* Needed to be affirmed (Choose an IV Generator)
				  * <default>, <built-in>, chainiv (Chain IV Generator), seqiv, eseqiv 
				  */
				.geniv = "<default>",
#endif		
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_AES | ELT_CTRL_CIPH_MODE_ECB_CTS,
		.alg_u.alg = {
			.cra_name = "cts(ecb(aes))",
			.cra_driver_name = "cts-ecb-aes-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
				CRYPTO_ALG_KERN_DRIVER_ONLY |
				CRYPTO_ALG_ASYNC /*| CRYPTO_ALG_NEED_FALLBACK*/,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_aes_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = AES_MIN_KEY_SIZE,
				.max_keysize = AES_MAX_KEY_SIZE,
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_AES | ELT_CTRL_CIPH_MODE_CBC_CTS,
		.alg_u.alg = {
			.cra_name = "cts(cbc(aes))",
			.cra_driver_name = "cts-cbc-aes-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
				CRYPTO_ALG_KERN_DRIVER_ONLY |
				CRYPTO_ALG_ASYNC /*| CRYPTO_ALG_NEED_FALLBACK*/,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_aes_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = AES_MIN_KEY_SIZE,
				.max_keysize = AES_MAX_KEY_SIZE,
#if 0
				.ivsize = AES_BLOCK_SIZE,
				/* Needed to be affirmed (Choose an IV Generator)
				  * <default>, <built-in>, chainiv (Chain IV Generator), seqiv, eseqiv 
				  */
				.geniv = "<default>",
#endif	
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_AES | ELT_CTRL_CIPH_MODE_PCBC_CTS,
		.alg_u.alg = {
			.cra_name = "cts(pcbc(aes))",
			.cra_driver_name = "cts-pcbc-aes-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
				CRYPTO_ALG_KERN_DRIVER_ONLY |
				CRYPTO_ALG_ASYNC /*| CRYPTO_ALG_NEED_FALLBACK*/,
			.cra_blocksize = AES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_aes_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = AES_MIN_KEY_SIZE,
				.max_keysize = AES_MAX_KEY_SIZE,
#if 0
				.ivsize = AES_BLOCK_SIZE,
				/* Needed to be affirmed (Choose an IV Generator)
				  * <default>, <built-in>, chainiv (Chain IV Generator), seqiv, eseqiv 
				  */
				.geniv = "<default>",
#endif		
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_DES | ELT_CTRL_CIPH_MODE_CBC | ELT_CTRL_DES_ALG,
		.alg_u.alg = {
			.cra_name = "cbc(des)",
			.cra_driver_name = "cbc-des-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
			.cra_blocksize = DES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_des_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = DES_KEY_SIZE,
				.max_keysize = DES_KEY_SIZE,
#if 0
				.ivsize = DES_BLOCK_SIZE,
				/* Needed to be affirmed (Choose an IV Generator)
				  * <default>, <built-in>, chainiv (Chain IV Generator), seqiv, eseqiv 
				  */
				.geniv = "<default>",		
#endif
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_DES | ELT_CTRL_CIPH_MODE_ECB | ELT_CTRL_DES_ALG,
		.alg_u.alg = {
			.cra_name = "ecb(des)",
			.cra_driver_name = "ecb-des-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
			.cra_blocksize = DES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_des_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = DES_KEY_SIZE,
				.max_keysize = DES_KEY_SIZE,
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_DES | ELT_CTRL_CIPH_MODE_OFB | ELT_CTRL_DES_ALG,
		.alg_u.alg = {
			.cra_name = "ofb(des)",
			.cra_driver_name = "ofb-des-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
			.cra_blocksize = DES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_des_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = DES_KEY_SIZE,
				.max_keysize = DES_KEY_SIZE,
#if 0
				.ivsize = DES_BLOCK_SIZE,
				/* Needed to be affirmed (Choose an IV Generator)
				  * <default>, <built-in>, chainiv (Chain IV Generator), seqiv, eseqiv 
				  */
				.geniv = "<default>",
#endif		
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_DES | ELT_CTRL_CIPH_MODE_CTR | ELT_CTRL_DES_ALG,
		.alg_u.alg = {
			.cra_name = "ctr(des)",
			.cra_driver_name = "ctr-des-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
			.cra_blocksize = DES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_des_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = DES_KEY_SIZE,
				.max_keysize = DES_KEY_SIZE,
#if 0
				.ivsize = DES_BLOCK_SIZE,
				/* Needed to be affirmed (Choose an IV Generator)
				  * <default>, <built-in>, chainiv (Chain IV Generator), seqiv, eseqiv 
				  */
				.geniv = "<default>",
#endif
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
#if 0 //TDES only support ECB mode
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_DES | ELT_CTRL_CIPH_MODE_CBC | ELT_CTRL_3DES_ALG,
		.alg_u.alg = {
			.cra_name = "cbc(des3_ede)",
			.cra_driver_name = "cbc-des3-ede-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_3des_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = DES3_EDE_KEY_SIZE,
				.max_keysize = DES3_EDE_KEY_SIZE,
#if 0
				.ivsize = DES_BLOCK_SIZE,
				/* Needed to be affirmed (Choose an IV Generator)
				  * <default>, <built-in>, chainiv (Chain IV Generator), seqiv, eseqiv 
				  */
				.geniv = "<default>",
#endif		
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_DES | ELT_CTRL_CIPH_MODE_ECB | ELT_CTRL_3DES_ALG,
		.alg_u.alg = {
			.cra_name = "ecb(des3_ede)",
			.cra_driver_name = "ecb-des3-ede-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_3des_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = DES3_EDE_KEY_SIZE,
				.max_keysize = DES3_EDE_KEY_SIZE,
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_DES | ELT_CTRL_CIPH_MODE_OFB | ELT_CTRL_3DES_ALG,
		.alg_u.alg = {
			.cra_name = "ofb(des3_ede)",
			.cra_driver_name = "ofb-des3-ede-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_3des_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = DES3_EDE_KEY_SIZE,
				.max_keysize = DES3_EDE_KEY_SIZE,
#if 0
				.ivsize = DES_BLOCK_SIZE,
				/* Needed to be affirmed (Choose an IV Generator)
				  * <default>, <built-in>, chainiv (Chain IV Generator), seqiv, eseqiv 
				  */
				.geniv = "<default>",
#endif		
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_DES | ELT_CTRL_CIPH_MODE_CTR | ELT_CTRL_3DES_ALG,
		.alg_u.alg = {
			.cra_name = "ctr(des3_ede)",
			.cra_driver_name = "ctr-des3-ede-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
			.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_3des_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = DES3_EDE_KEY_SIZE,
				.max_keysize = DES3_EDE_KEY_SIZE,
#if 0
				.ivsize = DES_BLOCK_SIZE,
				/* Needed to be affirmed (Choose an IV Generator)
				  * <default>, <built-in>, chainiv (Chain IV Generator), seqiv, eseqiv 
				  */
				.geniv = "<default>",
#endif		
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
#endif
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_RC4,
		.alg_u.alg = {
			/*
			 * These modes only apply to block ciphers, as they are designed to function using each block output
			 */
			.cra_name = "ecb(arc4)",
			.cra_driver_name = "ecb-arc4-elite",
			.cra_priority = ELITE_CRYPTO_ALG_PRIORITY,
			.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_KERN_DRIVER_ONLY |
						CRYPTO_ALG_ASYNC,
			.cra_blocksize = RC4_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct elite_ablk_ctx),
			.cra_type = &crypto_ablkcipher_type,
			.cra_module = THIS_MODULE,
			.cra_ablkcipher = {
				.setkey = elite_arc4_setkey,
				.encrypt = elite_ablk_encrypt,
				.decrypt = elite_ablk_decrypt,
				.min_keysize = RC4_MIN_KEY_SIZE,
				.max_keysize = RC4_MAX_KEY_SIZE,
			},
			.cra_init = elite_ablk_cra_init,
			.cra_exit = elite_ablk_cra_exit,
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_SHA1,
		.alg_u.halg = {
			.init		= elite_sha_init,
			.update		= elite_sha_update,
			.final		= elite_sha_final,
			.finup		= elite_sha_finup,  //update+final
			.digest		= elite_sha_digest, //init+finup
			.halg.digestsize	= SHA1_DIGEST_SIZE,
			.halg.base	= {
				.cra_name		= "sha1",
				.cra_driver_name	= "elite-sha1",
				.cra_priority		= 100,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
								CRYPTO_ALG_KERN_DRIVER_ONLY |
								CRYPTO_ALG_ASYNC |
								CRYPTO_ALG_NEED_FALLBACK,
				.cra_blocksize		= SHA1_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct elite_hash_ctx),
				.cra_alignmask		= 0,
				.cra_module		= THIS_MODULE,
				.cra_init		= elite_hash_cra_init,
				.cra_exit		= elite_hash_cra_exit,
			}
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_SHA224,
		.alg_u.halg = {
			.init		= elite_sha_init,
			.update		= elite_sha_update,
			.final		= elite_sha_final,
			.finup		= elite_sha_finup,
			.digest		= elite_sha_digest,
			.halg.digestsize	= SHA224_DIGEST_SIZE,
			.halg.base	= {
				.cra_name		= "sha224",
				.cra_driver_name	= "elite-sha224",
				.cra_priority		= 100,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
								CRYPTO_ALG_KERN_DRIVER_ONLY |
								CRYPTO_ALG_ASYNC |
								CRYPTO_ALG_NEED_FALLBACK,
				.cra_blocksize		= SHA224_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct elite_hash_ctx),
				.cra_alignmask		= 0,
				.cra_module		= THIS_MODULE,
				.cra_init		= elite_hash_cra_init,
				.cra_exit		= elite_hash_cra_exit,
			}
		},
	},
	{
		.ctrl_default = ELT_CTRL_CIPH_ALG_SHA256,
		.alg_u.halg = {
			.init		= elite_sha_init,
			.update		= elite_sha_update,
			.final		= elite_sha_final,
			.finup		= elite_sha_finup,
			.digest		= elite_sha_digest,
			.halg.digestsize	= SHA256_DIGEST_SIZE,
			.halg.base	= {
				.cra_name		= "sha256",
				.cra_driver_name	= "elite-sha256",
				.cra_priority		= 100,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
								CRYPTO_ALG_KERN_DRIVER_ONLY |
								CRYPTO_ALG_ASYNC |
								CRYPTO_ALG_NEED_FALLBACK,
				.cra_blocksize		= SHA256_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct elite_hash_ctx),
				.cra_alignmask		= 0,
				.cra_module		= THIS_MODULE,
				.cra_init		= elite_hash_cra_init,
				.cra_exit		= elite_hash_cra_exit,
			}
		},
	}
};


/**
 * due to the limitation of security engine itself, we actually can only
 * register one of the four algorithms (AES/(T)DES/RC4/SHA) to the system.
 * 
 */
static int elite_crypto_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct elite_crypto_se *se;
	struct resource *mem, *irq;
	int err = 0, i, j;

	se = devm_kzalloc(&pdev->dev, sizeof(*se), GFP_KERNEL);
	if(se == NULL) {
		dev_err(dev, "unable to alloc data struct.\n");
		return -ENOMEM;
	}
#if 0
	reg_se_base = of_ioremap(&pdev->resource[0], 0, 2, "elite_crypto");
	if (!reg_se_base)
		goto err_of;
#endif
	se->resv_buf_virt = (void *)__get_free_pages(GFP_KERNEL | GFP_DMA,
						get_order(RESV_BUFF_SIZE));
	if (!se->resv_buf_virt) {
		dev_err(dev, "unable to alloc memory for reseved buffer.\n");
		return -ENOMEM;
	}
	engine = se;
	se->dev = dev;
	se->algs = security_engine_algs;
	se->num_algs = ARRAY_SIZE(security_engine_algs);
	se->name = dev_name(&pdev->dev);
	atomic_set(&se->se_status_flags, FLAGS_SE_IDLE);
	platform_set_drvdata(pdev, se);

	init_completion(&se->op_comp);
	
	crypto_init_queue(&se->queue, ELITE_CRYPTO_QUEUE_LENGTH);

	/* Get the base address */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if(!mem || ! irq ) {
		dev_err(dev, "No io memory/irq resource for security engine\n");
		err = -ENODEV;
		return err;
	}

	if(!devm_request_mem_region(dev, mem->start, resource_size(mem),
			 dev_name(&pdev->dev)))
		return -ENOMEM;

	se->phys_base = mem->start;

	se->io_base = devm_ioremap(dev, se->phys_base, resource_size(mem));
	if(!se->io_base) {
		dev_err(dev, "can't devm_ioremap\n");
		err = -ENOMEM;
		return err;
	}

	reg_se_base = se->io_base;

	se->irq = irq->start;
	err = devm_request_irq(dev, irq->start, crypto_interrupt, IRQF_DISABLED, 
		dev_name(&pdev->dev), se);
	if (err) {
		dev_err(dev, "failed to request irq\n");
		return err;
	}

#if 0
	se->req_pool = dmam_pool_create(se->name, se->dev,
			MAX_DDT_LEN * sizeof(struct elite_dma_desc_tbl), 8, SZ_64K);	
	if(!se->req_pool)
		return -ENOMEM;
#endif
	spin_lock_init(&se->queue_lock);

        se->clk = clk_get(&pdev->dev, "se");
        if (IS_ERR(engine->clk)) {
                dev_info(&pdev->dev, "clk unavailable\n");
                return PTR_ERR(se->clk);
        }

        if (clk_enable(engine->clk)) {
                dev_info(&pdev->dev, "unable to enable clk\n");
                clk_put(engine->clk);
                return -EIO;
        }

	se->workqueue = create_singlethread_workqueue("elite_crypto_se");
	if (!se->workqueue) {
		dev_err(&pdev->dev, "fail to create workqueue for elite SE driver\n");
		return -ENOMEM;
	}

	INIT_WORK(&se->req_queue_work, elite_crypto_workqueue_handler);

	INIT_LIST_HEAD(&se->list_algs);	

	for(i=0; i<se->num_algs; i++) {
		se->algs[i].se = se;
		if((se->algs[i].ctrl_default == ELT_CTRL_CIPH_ALG_SHA1) ||
			(se->algs[i].ctrl_default == ELT_CTRL_CIPH_ALG_SHA224) ||
			(se->algs[i].ctrl_default == ELT_CTRL_CIPH_ALG_SHA256)) {
			dev_info(dev, "register hash driver: %s\n", 
				se->algs[i].alg_u.halg.halg.base.cra_driver_name);
			err = crypto_register_ahash(&se->algs[i].alg_u.halg);
		} else {
			dev_info(dev, "register block cipher driver: %s\n", 
				se->algs[i].alg_u.alg.cra_driver_name);
			err = crypto_register_alg(&se->algs[i].alg_u.alg);
		}
		if(!err) {
			list_add_tail(&se->algs[i].entry, &se->list_algs);
		}
		if(err){
			if((se->algs[i].ctrl_default == ELT_CTRL_CIPH_ALG_SHA1) ||
				(se->algs[i].ctrl_default == ELT_CTRL_CIPH_ALG_SHA224) ||
				(se->algs[i].ctrl_default == ELT_CTRL_CIPH_ALG_SHA256))
				dev_err(dev, "failed to register hash alg \"%s\"\n",
					se->algs[i].alg_u.halg.halg.base.cra_driver_name);
			else
				dev_err(dev, "failed to register alg \"%s\"\n",
					se->algs[i].alg_u.alg.cra_name);
			goto err_algs;
		}
	}

	pr_info("security engine probed done\n");
	return 0;

err_algs:
	for(j=0; j<i; j++){
		if((se->algs[j].ctrl_default == ELT_CTRL_CIPH_ALG_SHA1) ||
			(se->algs[j].ctrl_default == ELT_CTRL_CIPH_ALG_SHA224) ||
			(se->algs[j].ctrl_default == ELT_CTRL_CIPH_ALG_SHA256))
			crypto_unregister_ahash(&se->algs[j].alg_u.halg);
		else
			crypto_unregister_alg(&se->algs[j].alg_u.alg);
	}

        clk_disable(se->clk);
        clk_put(se->clk);
	destroy_workqueue(se->workqueue);

	return err;
#if 0
	err_of:
		of_iounmap(&pdev->resource[0], reg_se_base, 1);
#endif

}

static int elite_crypto_remove(struct platform_device *pdev)
{
	struct elite_alg *alg, *next;
	struct elite_crypto_se *se = platform_get_drvdata(pdev);
 
 	if (!se)
 		return -ENODEV;

	flush_workqueue(se->workqueue);
	destroy_workqueue(se->workqueue);
	list_for_each_entry_safe(alg, next, &se->list_algs, entry) {
		list_del(&alg->entry);
		if((alg->ctrl_default == ELT_CTRL_CIPH_ALG_SHA1) ||
			(alg->ctrl_default == ELT_CTRL_CIPH_ALG_SHA224) ||
			(alg->ctrl_default == ELT_CTRL_CIPH_ALG_SHA256))
			crypto_unregister_ahash(&alg->alg_u.halg);
		else
			crypto_unregister_alg(&alg->alg_u.alg);
	}
#if 0
	dma_pool_destroy(se->req_pool); 
#endif
	free_pages((unsigned long)se->resv_buf_virt, get_order(RESV_BUFF_SIZE));
        clk_disable(se->clk);
        clk_put(se->clk);

 	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int elite_crypto_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct elite_crypto_se *se = platform_get_drvdata(pdev);

	/* gate to clock to security engine */
        clk_disable(se->clk);

	return 0;
}
static int elite_crypto_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct elite_crypto_se *se = platform_get_drvdata(pdev);

	return clk_enable(se->clk);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM_RUNTIME
static int elite_crypto_runtime_suspend(struct device *dev)
{
	return 0;
}
static int elite_crypto_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

//static SIMPLE_DEV_PM_OPS(elite_crypto_pm_ops,
//		elite_crypto_suspend, elite_crypto_resume);

static const struct dev_pm_ops elite_crypto_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(elite_crypto_suspend, elite_crypto_resume)
	SET_RUNTIME_PM_OPS(elite_crypto_runtime_suspend,
				elite_crypto_runtime_resume, NULL)
};

#ifdef CONFIG_OF
static const struct of_device_id elite_crypto_match[] = {
	{
		.compatible = "s3graphics, elite1000-crypto",
	},
	{},
};
MODULE_DEVICE_TABLE(of, elite_crypto_match);
#endif

static struct platform_device_id elite_crypto_driver_ids[] = {
	{
		.name  = "elite-se",
		.driver_data	= (kernel_ulong_t)NULL,
	}, {
		.name  = "elite-se.0",
		.driver_data	= (kernel_ulong_t)NULL,
	},
	{ /* sentinel */ },
};

static struct platform_driver elite_crypto_driver = {
	.driver = {
		.name = "s3graphics-elite-se",
		.owner = THIS_MODULE,
		.pm = &elite_crypto_pm_ops,
		.of_match_table = of_match_ptr(elite_crypto_match),
	},
	.id_table  = elite_crypto_driver_ids,
	.probe = elite_crypto_probe,
	.remove = __devexit_p(elite_crypto_remove),
};

module_platform_driver(elite_crypto_driver);

MODULE_DESCRIPTION("ELITE1k SOC security engine support.");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:elite_crypto");
MODULE_AUTHOR("S3 Graphics, Inc.");


