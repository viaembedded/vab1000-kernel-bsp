#include <linux/module.h>	
#include <linux/init.h>
#include <linux/kernel.h>	
#include <linux/ioport.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/slab.h>					
#include <linux/poll.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/sched.h>
#include <linux/kthread.h>

#define DEVNAME	  "frontpad"
#define KEYCODES  7

struct elite_frontpad_device {
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct task_struct *task;
    unsigned short keycode[KEYCODES];
};

static struct elite_fronted_keymap {
    u16 key;
    u16 code;
}keymap[] = {
{KEY_UP,        0x8062},
{KEY_DOWN,      0x4062},
{KEY_LEFT,      0x0462},
{KEY_RIGHT,     0x0862},
{KEY_MENU,      0x2062},
{KEY_OK,        0x1062},
{KEY_POWER,     0x0262},
};

// this struct should be add in BSP. not here
// register it in /arch/arm/math-elite/generic.c
/*
static struct i2c_board_info frontpad_i2c_devs[] __initdata = {
    {I2C_BOARD_INFO("frontpad", 0x10), },
};
i2c_register_board_info(0, frontpad_i2c_devs, ARRAY_SIZE(frontpad_i2c_devs));
*/

//thread function
static void frontpad_thread(struct elite_frontpad_device *dev)
{
    struct elite_frontpad_device *frontpad = dev;
    u8 rdbuf[2];
    int ret,i,value;

    if(!frontpad || !frontpad->input_dev || !frontpad->client)
    {
        printk(KERN_ERR "frontpad_thread: frontpad is empty.\n");
        return;
    }
    /*
    ret = i2c_master_send(frontpad->client, wrbuf, sizeof(wrbuf));
    if (ret != sizeof(wrbuf)) {
    	dev_err(&frontpad->client->dev,
    		"%s: i2c_master_send failed(), ret=%d\n",
    		__func__, ret);
    	//return;
    }*/
    
    while(!kthread_should_stop())
    {
        ret = i2c_master_recv(frontpad->client, rdbuf, sizeof(rdbuf));
        if (ret != sizeof(rdbuf)) {
            //dev_err(&frontpad->client->dev,"%s: i2c_master_recv failed(), ret=%d\n", __func__, ret);
        }
        value = (rdbuf[1]<<8) | rdbuf[0];
        //printk("read i2c data: %x ret=%d\n",value,ret);
        
        for(i=0; i< KEYCODES; i++)
        {
            if(value == keymap[i].code)
            {  
                input_event(frontpad->input_dev, EV_MSC, MSC_SCAN, keymap[i].code);
                input_event(frontpad->input_dev, EV_KEY, keymap[i].key, 1);
                input_sync(frontpad->input_dev);
                input_event(frontpad->input_dev, EV_MSC, MSC_SCAN, keymap[i].code);
                input_event(frontpad->input_dev, EV_KEY, keymap[i].key, 0);
                input_sync(frontpad->input_dev);
                break;
            }
        }

        msleep(200);
    }

}


static int __devinit frontpad_probe(struct i2c_client *client,const struct i2c_device_id *idp)
{
    int i, ret = 0;
    struct elite_frontpad_device *frontpad;
    struct input_dev *input;
    
    frontpad = kzalloc(sizeof(*frontpad), GFP_KERNEL);
    input = input_allocate_device();
    if(!frontpad || !input)
    {
        printk(KERN_ERR "failed allocate driver data\n");
        ret = -ENOMEM;
        goto err_free_mem;
    }
    
    frontpad->client = client;
    frontpad->input_dev = input;
    
    if (!(frontpad->input_dev))
    {
        printk(KERN_ERR "input_dev: not enough memory for input device\n");
        return -ENOMEM;
    }

    for(i=0; i<KEYCODES; i++)
    {
        frontpad->keycode[i] = keymap[i].key;
    }
    input->keycode = frontpad->keycode;
    input->keycodesize = sizeof(u16);
    input->keycodemax = KEYCODES;
    input->name = "Elite Frontpad Driver";
    input->id.bustype = BUS_I2C;
    input->dev.parent = &client->dev;
    //input_dev->open = frontpad_open;
    //input_dev->close = frontpad_close;
    input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP) | BIT_MASK(EV_MSC) ;
    input->mscbit[0] = BIT_MASK(MSC_RAW) | BIT_MASK(MSC_SCAN) ;
    for (i = 0; i < KEYCODES; i++)
    {
        __set_bit(frontpad->keycode[i], input->keybit);
    }
    input_set_drvdata(input, frontpad);
    ret = input_register_device(input);
    if(ret)
    {
        printk(KERN_ERR "input_register_device failed!\n");  
        goto err_free_mem;
    }
    
    i2c_set_clientdata(client, frontpad);
    device_init_wakeup(&client->dev, 1);   
    
    frontpad->task = kthread_run((int (*)(void *))frontpad_thread, frontpad, "frontpad");
    if(IS_ERR(frontpad->task))
    {
        printk(KERN_ERR "create kthread failed!\n");  
        goto err_free_mem;
    }
    return 0;
    
err_free_mem:
	input_free_device(input);
	kfree(frontpad);
	return ret;
}

static int __devexit  frontpad_remove(struct i2c_client *client)
{
    struct elite_frontpad_device *frontpad = i2c_get_clientdata(client);
    device_init_wakeup(&client->dev, 0);
    mb();
    input_unregister_device(frontpad->input_dev);
    if(!IS_ERR(frontpad->task))
    {
        int ret = kthread_stop(frontpad->task);  
        printk(KERN_ERR "thread function has run %ds\n", ret);  
    }
    kfree(frontpad);
	return 0;
}

static const struct i2c_device_id frontpad_id[] = {
	{ "frontpad", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, frontpad_id);

static struct i2c_driver frontpad_driver = {
	.driver = {
        .owner	= THIS_MODULE,
		.name = "frontpad",
	},
	.probe = frontpad_probe,
	.remove = __devexit_p(frontpad_remove),
	.suspend = NULL,
	.resume = NULL,
	.id_table = frontpad_id,
};

static int __init frontpad_init(void)
{
	return i2c_add_driver(&frontpad_driver);
}

static void __exit frontpad_exit(void)
{   
	i2c_del_driver(&frontpad_driver);
}

MODULE_DESCRIPTION("elite frontpad driver");
MODULE_AUTHOR("S3");
MODULE_LICENSE("GPL");

module_init(frontpad_init);
module_exit(frontpad_exit);

