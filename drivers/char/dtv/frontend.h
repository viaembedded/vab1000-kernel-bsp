#ifndef _FRONT_END_H
#define _FRONT_END_H

typedef struct 
{
	unsigned char addr;
	unsigned char len;
	unsigned char data[16];
}elite_nim_data;

#define TUNER_IOC_MAGIC 'k'
#define IOCTL_CHECK_LOCK _IOR(TUNER_IOC_MAGIC,0,int)
#define IOCTL_SET_TUNER _IOW(TUNER_IOC_MAGIC,1,unsigned int)
typedef enum
{
	IOCTL_READ_DEMOD = 0x10,
	IOCTL_READ_TUNER,
	IOCTL_WRITE_DEMOD, 
	IOCTL_WRITE_TUNER,
}FE_IO_CMD;
#define IOCTL_MAXNR 6


#define DEVNAME					"frontend"


struct edtv_tuner_device {
	struct i2c_client *client;
 	struct cdev* cdev;
};

#endif
