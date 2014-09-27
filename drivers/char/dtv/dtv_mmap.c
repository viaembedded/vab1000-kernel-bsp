#include <linux/module.h>	
#include <linux/kernel.h>
#include <linux/init.h>			
#include <linux/fs.h>			
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/mm_types.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/mempool.h>
#include <linux/ioprio.h>
#include <asm/io.h>
#include <linux/scatterlist.h>

#include <linux/crypto.h>

#define KSTR_DEF "HELLO WORLD from kernel space"
#define order 9  //2M
#define max_length  32*1024
static struct cdev *pcdev;
static dev_t ndev;
static struct page *pg; 
unsigned char *pvr_str;
struct class *pvr_mmap_class;
struct crypto_ablkcipher *tfm;

#define ENCRYPT 1
#define DECRYPT 0

#define PVR_ENCRYPT 1
#define PVR_DECRYPT 3


#define pfn_to_virt(pfn)	__va((pfn) << PAGE_SHIFT)
#define page_to_virt(page)	pfn_to_virt(page_to_pfn(page))


#define CRYPTO_TFM_REQ_MAY_BACKLOG	0x00000400

#if 1
char edtv_key[16] =
	"\x06\xa9\x21\x40\x36\xb8\xa1\x5b"
	"\x51\x2e\x03\xd5\x34\x12\x00\x06";
#endif
#if 0
char edtv_key[16] =
	"\x00\x00\x00\x00\x00\x00\x00\x00"
	"\x00\x00\x00\x00\x00\x00\x00\x00";
#endif
struct cipher_testvec {
	char *key;
	char *iv;
	char *input;
	char *result;
	unsigned short tap[8];
	int np;
	unsigned char fail;
	unsigned char wk; /* weak key flag */
	unsigned char klen;
	unsigned short ilen;
	unsigned short rlen;
};

struct pvr_tcrypt_result {
	struct completion completion;
	int err;
};

//==========================================================
void pvr_tcrypt_complete(struct crypto_async_request *req, int err)
{
	struct pvr_tcrypt_result *res = req->data;

	if (err == -EINPROGRESS)
		return;

	res->err = err;
	complete(&res->completion);
}


int pvr_crypto_32k(unsigned int offset,unsigned int crypto_length,int enc)
{
  const char *algo = crypto_tfm_alg_driver_name(crypto_ablkcipher_tfm(tfm));
  const char *e;
  unsigned char *crypto_str;
  struct ablkcipher_request *req;
  struct scatterlist sg[8];
  unsigned int i, sg_count,sg_offset,n;
 
  int ret = -ENOMEM;
  struct pvr_tcrypt_result result;


  if (enc == ENCRYPT)
   {
     e = "encryption";
     crypto_str=pvr_str;
   }
  else
   {
	e = "decryption";
	crypto_str=pvr_str+1024*1024;
   }

  init_completion(&result.completion);

  req = ablkcipher_request_alloc(tfm, GFP_KERNEL);
  if (!req) {
	printk(KERN_ERR "alg: skcipher: Failed to allocate request for %s\n", algo);
	goto out;
  }

  ablkcipher_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,pvr_tcrypt_complete, &result);
  
 
	ret = -EINVAL;
	if (WARN_ON(crypto_length > PAGE_SIZE*32))
	goto out;

	crypto_ablkcipher_clear_flags(tfm, ~0);

	ret = crypto_ablkcipher_setkey(tfm,edtv_key,16);
			

	sg_count=crypto_length/PAGE_SIZE;
	sg_offset=crypto_length%PAGE_SIZE;
        n=sg_count;
        if(sg_offset!=0)
        n++;
	sg_init_table(sg, n);
        for(i=0;i<sg_count;i++)
	sg_set_buf(&sg[i], crypto_str+i*PAGE_SIZE, PAGE_SIZE);
        
        if(sg_offset!=0)
	sg_set_buf(&sg[sg_count], crypto_str+sg_count*PAGE_SIZE, sg_offset);

	ablkcipher_request_set_crypt(req, sg, sg,crypto_length, NULL);
	ret = enc ?crypto_ablkcipher_encrypt(req) :crypto_ablkcipher_decrypt(req);

	switch (ret) {
	case 0:
	break;
	case -EINPROGRESS:
	case -EBUSY:
	ret = wait_for_completion_interruptible(
	&result.completion);
	if (!ret && !((ret = result.err))) {
	INIT_COMPLETION(result.completion);
	break;
	}
	/* fall through */
	default:
	//printk(KERN_ERR "alg: skcipher: %s failed on test %d for %s: ret=%d\n", e, j, algo,-ret);
	goto out;
	}


        ret=0;
out:
	ablkcipher_request_free(req);

	return ret;
}

static int demo_release(struct inode *inode,struct file *filp)
{

 return 0;
}

static int pvr_mmap(struct file *filp,struct vm_area_struct *vma)
{
  int err=0;
  unsigned long start=vma->vm_start;
  unsigned long size=vma->vm_end-vma->vm_start;
  err=remap_pfn_range(vma,start,page_to_phys(pg)>>12,size,vma->vm_page_prot);
  return err;
}

static int demo_open(struct inode *inode,struct file *filp)
{
return 0;
}



void pvr_crypto(unsigned int data_length,int enc)
{
 unsigned int loop_times,length_offset,i;
 loop_times=data_length/(max_length);
 length_offset=data_length%(max_length);
 for(i=0;i<loop_times;i++)
  pvr_crypto_32k(max_length*i,max_length,enc);
 if(0!=length_offset)
  pvr_crypto_32k(loop_times*max_length,length_offset,enc);
}


long pvr_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned int data_length;
	int reval = 0;
	switch (cmd)
	{
	  case PVR_ENCRYPT:
	  reval = copy_from_user(&data_length, (void __user *)arg, sizeof(unsigned int));
	  pvr_crypto(data_length,ENCRYPT);

	  break;

	  case PVR_DECRYPT:
	  reval = copy_from_user(&data_length, (void __user *)arg, sizeof(unsigned int));
	  pvr_crypto(data_length,DECRYPT);

	  break;

	  default:
	  reval=-1;
	  printk("This cmd(%d) is not support!\n",cmd);
	  break;
	}
	return reval;
}

static struct file_operations mmap_fops=
{
 .owner=THIS_MODULE,
 .open=demo_open,
 .unlocked_ioctl=pvr_ioctl,
 .release=demo_release,
 .mmap=pvr_mmap,
 
};


static int __init edtv_mmap_init(void)
{
  unsigned int i,count;

  pg=alloc_pages(GFP_KERNEL,order);//2M
  count=1<<order;
  for(i=0;i<count;i++)
  SetPageReserved(pg+i);
  
   pvr_str=page_to_virt(pg);

  strcpy(pvr_str,KSTR_DEF);
  printk("kpa=0x%x,kernel string=%s\n",page_to_phys(pg),pvr_str);


  pcdev = cdev_alloc();
  cdev_init(pcdev,&mmap_fops);
  alloc_chrdev_region(&ndev,0,1,"mmap_dev");
  printk("major=%d,minor=%d\n",MAJOR(ndev),MINOR(ndev));
  pcdev->owner=THIS_MODULE;
  cdev_add(pcdev,ndev,1);

/* create your own class under /sysfs */
  pvr_mmap_class = class_create(THIS_MODULE, "security_edtv");
  if(IS_ERR(pvr_mmap_class)) 
  {
	printk("Err: failed in creating class.\n");
	return -1; 
  }

/* register your own device in sysfs, and this will cause udev to create corresponding device node */
  device_create(pvr_mmap_class, NULL,ndev, NULL, "crypto");

//==============================================================================================
//crypto
  tfm = crypto_alloc_ablkcipher("ecb-aes-elite", 0, 0);
  if (IS_ERR(tfm)) {
     printk(KERN_ERR "alg: skcipher: Failed to load transform for ""%s: %ld\n", "ecb-aes-elite", PTR_ERR(tfm));
     return PTR_ERR(tfm);
  }

  return 0;
}

static void __exit edtv_mmap_exit(void)
{
 unsigned int i,count;
 count=1<<order;

 device_destroy(pvr_mmap_class, ndev);
 class_destroy(pvr_mmap_class);                              

 cdev_del(pcdev);
 unregister_chrdev_region(ndev,1);
 for(i=0;i<count;i++)
 {
 ClearPageReserved(pg+i);
  }
 __free_pages(pg,order);

 crypto_free_ablkcipher(tfm);
}

late_initcall(edtv_mmap_init);
module_exit(edtv_mmap_exit);
MODULE_AUTHOR("S3");
MODULE_DESCRIPTION("mmap Driver Module");
MODULE_LICENSE("GPL");
