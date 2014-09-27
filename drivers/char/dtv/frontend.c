#define CONFIG_I2C_MODULE
#include <linux/module.h>

#include <linux/kernel.h>

#include <linux/init.h>

#include <linux/delay.h>

#include <linux/pm.h>

#include <linux/slab.h>

#include <linux/cpufreq.h>

#include <linux/ioport.h>

#include <linux/platform_device.h>

#include <linux/i2c.h>

#include <linux/power_supply.h>

#include <linux/pinctrl/machine.h>

#include <linux/pinctrl/consumer.h>

#include <linux/pinctrl/pinconf-generic.h>



#include <asm/mach-types.h>

#include <asm/system.h>

#include <asm/pgtable.h>

//#include <asm/mach/map.h>

#include <asm/irq.h>

#include <asm/sizes.h>

#include <asm/io.h>

#include <asm/mach/arch.h>

#include <asm/setup.h>

#include <asm/hardware/gic.h>

#include <asm/pgalloc.h>



#include <mach/iomap.h>

#include <mach/pinconf-elite.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/cpufreq.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/consumer.h>
#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>
#include <asm/sizes.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/pgalloc.h>
#include <mach/iomap.h>
#include <mach/pinconf-elite.h>
//#include "board.h"
//#include "cpu-elite.h"
//#include "elite-pg.h"
#include <linux/module.h>	
#include <linux/kernel.h>					
#include <linux/cdev.h>
#include <linux/device.h>					
#include <linux/slab.h>					
#include <linux/poll.h>
#include <linux/i2c.h>
#include "frontend.h"

static int tuner_major = 237;
static int tuner_minor = 0;
static int number_of_devices = 1;
struct class *tuner_class;
struct device *tuner_device;



static struct i2c_board_info tuner_i2c_devs[] __initdata = {

        {I2C_BOARD_INFO("tuner", 0x60), },

};


//#define _DEBUG_
struct edtv_tuner_device *tuner;

static int elite_get_chip_version()
{
    int iVersion = 0;
    volatile unsigned int * pChipVersionNumber = (unsigned int *)ioremap(0xD80A8500, 4);

    if(pChipVersionNumber)
    {
        iVersion = (*pChipVersionNumber)&0x0F;

        iounmap(pChipVersionNumber);
    }

    return iVersion;
}

int edtv_demod_read(unsigned char addr, unsigned char *buf_data, unsigned char len)
{
  struct i2c_msg xfer[2];
  struct i2c_client *client = tuner->client;
  int ret;
  
  if((NULL==client)||(NULL==buf_data)||(NULL==tuner))
 {
  printk("bug,null point!\n");
  return -1;
 }
  xfer[0].addr = 0x1c;//client->addr;
  xfer[0].flags = 0;
  xfer[0].len = 1;
  xfer[0].buf = &addr;
  
  xfer[1].addr = 0x1c;//client->addr;
  xfer[1].flags = I2C_M_RD;
  xfer[1].len = len;
  xfer[1].buf = buf_data;

  ret = i2c_transfer(client->adapter, xfer, 2);
  
  return ret;
}

int edtv_demod_write(unsigned char addr, unsigned char *buf_data, unsigned char len)
{
  struct i2c_msg xfer[2];
  struct i2c_client *client = tuner->client;
  unsigned char data[4];
  int ret;

  if((NULL==client)||(NULL==buf_data)||(NULL==tuner))
  {
	  printk("bug,null point!\n");
	  return -1;
  }
  data[0] = addr;
  data[1] = buf_data[0];
  xfer[0].addr = 0x1c;//client->addr;
  xfer[0].flags = 0;
  xfer[0].len = 2;
  xfer[0].buf = data;
/*  
  xfer[1].addr = 0x1c;//client->addr;
  xfer[1].flags = 0;
  xfer[1].len = len;
  xfer[1].buf = buf_data;
*/
  ret = i2c_transfer(client->adapter, xfer, 1);
 
  return ret;
}

int edtv_tuner_read(unsigned char *buf_data, unsigned char len)
{
  struct i2c_msg xfer[1];
  struct i2c_client *client = tuner->client;
  int ret;
  unsigned char data;
  
	if((NULL==client)||(NULL==buf_data)||(NULL==tuner))
	{
		printk("bug,null point!\n");
		return -1;
	}
 
  if(elite_get_chip_version() < 2)
  {

    edtv_demod_read(0x86, &data, 1);
    printk("repeat value is %02x\n", data);
    data = 0x80 | data;
    edtv_demod_write(0x86, &data, 1);
  }

  xfer[0].addr = 0x60;//client->addr;
  xfer[0].flags = I2C_M_RD;
  xfer[0].len = len;
  xfer[0].buf = buf_data;

  ret = i2c_transfer(client->adapter, xfer, 1);

  return ret;
}

int edtv_tuner_write(unsigned char *buf_data, unsigned char len)
{
  struct i2c_msg xfer[1];
  struct i2c_client *client = tuner->client;
  int ret;
  unsigned char data;

  if((NULL==client)||(NULL==buf_data)||(NULL==tuner))
  {
  printk("bug,null point!\n");
  return -1;
  }

  if(elite_get_chip_version() < 2)
  {
    edtv_demod_read(0x86, &data, 1);
    printk("repeat value is %02x\n", data);
    data = 0x80 | data;
    edtv_demod_write(0x86, &data, 1);
  }
	
  xfer[0].addr = 0x60;//client->addr;
  xfer[0].flags = 0;
  xfer[0].len = len;
  xfer[0].buf = buf_data;

  ret = i2c_transfer(client->adapter, xfer, 1);
 
  return ret;
}

#ifndef _DEBUG_

#endif

static long tuner_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int reval = 0;
	int err = 0;
	unsigned int kmdvalue = 0;
	elite_nim_data param;
	elite_nim_data* pparam;
	//printk(KERN_EMERG "command: %d \n", cmd);
	udelay(100);		
	//printk(KERN_EMERG "switch begin\n");
	switch (cmd)
	{
		case IOCTL_READ_DEMOD:
			pparam = (elite_nim_data*)arg;
			if (copy_from_user(&param, pparam, sizeof(elite_nim_data)))
			{
				return -EACCES;
			}
//			printk(KERN_INFO "%02x %02x %02x\n", param.addr, param.data[0], param.len);
			edtv_demod_read(param.addr, param.data, param.len);
			if (copy_to_user(pparam, &param, sizeof(elite_nim_data)))
			{
				return -EACCES;
			}
			break;
		case IOCTL_READ_TUNER:
			pparam = (elite_nim_data*)arg;
			if (copy_from_user(&param, pparam, sizeof(elite_nim_data)))
			{
				return -EACCES;
			}
//			printk(KERN_INFO"%02x %02x \n", param.data[0], param.len);			
			err = edtv_tuner_read(param.data, param.len);
			
			if(err < 0)
			{
				printk(KERN_INFO"edtv_tuner_read fail, err:%d\n", err);
				return -EACCES;
			}
			
			if (copy_to_user(pparam, &param, sizeof(elite_nim_data)))
			{
				return -EACCES;
			}			
			break;
		case IOCTL_WRITE_DEMOD:
			pparam = (elite_nim_data*)arg;
			if (copy_from_user(&param, pparam, sizeof(elite_nim_data)))
			{
				return -EACCES;
			}
//			printk(KERN_INFO "%02x %02x %02x\n", param.addr, param.data[0], param.len);
			edtv_demod_write(param.addr, param.data, param.len);
			break;
		case IOCTL_WRITE_TUNER:
			pparam = (elite_nim_data*)arg;
			if (copy_from_user(&param, pparam, sizeof(elite_nim_data)))
			{
				return -EACCES;
			}
//			printk(KERN_INFO "%02x %02x %02x %02x %02x %02x \n", param.data[0], param.data[1], param.data[2], param.data[3], param.data[4], param.len);
			printk(KERN_INFO"Start to lock %d MHZ\n", (((((unsigned int)((param.data[0])<<8|param.data[1]))*62500)- 36160000 - 31250)+10000)/1000000);
			err = edtv_tuner_write(param.data, param.len);
			
			if(err < 0)
			{
				printk(KERN_INFO"edtv_tuner_write fail, err:%d\n", err);
				return -EACCES;
			}  
			break;		
		default:
			reval = -EFAULT;
			break;
	}
	return reval;
}

struct file_operations tuner_fops = 
{
	.owner			=	THIS_MODULE,
	.unlocked_ioctl 	=	tuner_ioctl,
};

static int init_char_device(void)
{
	int ret, result,error;
	
	dev_t devno = MKDEV(tuner_major, tuner_minor);
	
	printk(KERN_EMERG "front end module init\n");
	if (NULL==tuner) 
	{
		return -1;
	}
	result = register_chrdev_region (devno, number_of_devices, "tuner");
	if (result < 0)
	{
		printk (KERN_WARNING "tuner: can't get major number %d\n", tuner_major);
		return result;
	}

	tuner->cdev = kmalloc(sizeof(struct cdev), GFP_KERNEL);
	if(!tuner->cdev)
	{
		ret = -ENOMEM;
		printk("create device failed.\n");
		return ret;
	}
	else
	{
		cdev_init(tuner->cdev, &tuner_fops);
		tuner->cdev->owner = THIS_MODULE;
		tuner->cdev->ops = &tuner_fops;
		error = cdev_add(tuner->cdev, devno, 1);
		if (error)
		printk(KERN_NOTICE "Error %d adding demux device", error);
	}

	/* create your own class under /sysfs */
	tuner_class = class_create(THIS_MODULE, DEVNAME);
	if(IS_ERR(tuner_class)) 
	{
		printk("Err: failed in creating class.\n");
		return -1; 
	}

	/* register your own device in sysfs, and this will cause udev to create corresponding device node */
	device_create(tuner_class, NULL, MKDEV(tuner_major, tuner_minor), NULL, "TUNER");

	// add code

	return 0;
}

static int __devinit edtv_tuner_probe(struct i2c_client *client,
				   const struct i2c_device_id *idp)
{
	
	int ret;
	printk("edtv_tuner_probe\n");

	tuner = kzalloc(sizeof(*tuner), GFP_KERNEL);
	if (!tuner) 
	{
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -1;
	}
  if(NULL==client)
  {
	  printk("client is null\n");
	  return -1;
  }
  else
  	printk("client is find,address is:%x\n",client->addr);
  tuner->client = client;
   
  ret=init_char_device();
  return ret;
}

static int __devexit  edtv_tuner_remove(struct i2c_client *client)
{
  dev_t devno = MKDEV (tuner_major, tuner_minor);

	cdev_del (tuner->cdev);

	device_destroy(tuner_class, MKDEV(tuner_major, tuner_minor));
	class_destroy(tuner_class);                              

	unregister_chrdev_region (devno, number_of_devices);
	kfree(tuner->cdev);
	printk(KERN_EMERG "front end module exit\n");

	kfree(tuner);

	return 0;
}


static const struct i2c_device_id edtv_tuner_id[] = {
	{ "tuner", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, edtv_tuner_id);

static struct i2c_driver edtv_tuner_driver = {
	.driver = {
		.name = "tuner",
	},
	.probe = edtv_tuner_probe,
	.remove = __devexit_p(edtv_tuner_remove),
	.suspend = NULL,
	.resume = NULL,
	.id_table = edtv_tuner_id,
};

static int __init edtv_tuner_init(void)
{
  int ret;
//  i2c_register_board_info(0, tuner_i2c_devs, ARRAY_SIZE(tuner_i2c_devs));
  ret=i2c_add_driver(&edtv_tuner_driver);
//  i2c_register_board_info(0, tuner_i2c_devs, ARRAY_SIZE(tuner_i2c_devs));
	printk("edtv_tuner_init:%x\n",ret);
	return ret;
}

static void __exit edtv_tuner_exit(void)
{
	i2c_del_driver(&edtv_tuner_driver);
}

MODULE_DESCRIPTION("edtv front end driver");
MODULE_AUTHOR("lj");
MODULE_LICENSE("GPL");

module_init(edtv_tuner_init);
module_exit(edtv_tuner_exit);

