/*
 * ASoC Platform Driver for S3 Elite Chips
 * Part 1:
 *     Elite I2S Controller Operations
 *
 * This part of driver is just dependent on elite chips, no matter which
 * kind of codec is used and how development board is designed. Any change on it
 * should maintain the independence of this driver from codecs and development
 * board.
 *
 * Copyright (c) 2011 S3 Graphics,Co.,Ltd
 *
 */
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/asoundef.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <mach/io.h>
#include <mach/iomap.h>

#include "elite-pcm.h"
#include "elite-i2s.h"

#define ELITE_I2S_RATES    \
        (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
        SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
        SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define NUM_LINKS                1
#define NULL_DMA                 ((dmach_t)(-1))

extern unsigned int system_rev;
static int elite_i2s_dev_init(struct elite_i2s_dev *i2s_dev);

unsigned char clk_reg_36864[29] = {
    0x00,
    0x1b,
    0x11,
    0x7d,
    0x00,
    0x68,
    0x01,
    0x01,
    0x80,
    0x01,
    0x01,
    0x20,
    0x00,
    0x00,
    0x00,
    0x06,
    0x01,
    0x01,
    0x01,
    0x01,
    0x01,
    0x38,
    0x31,
    0x31,
    0x31,
    0x31,
    0x31,
    0x00,
    0x1b
};

extern int elite_i2c_xfer_by_dev_id(unsigned int dev_id, struct i2c_msg *msg, int num);
static int elite_i2s_ext_clk_set(unsigned int value)
{
    struct i2c_msg msg;
    
    memset(&msg, 0, sizeof(msg));

    switch(value) {
        case 36864:
            msg.len = 28;
            msg.addr = 0x69;
            msg.flags = 0;
            msg.buf = clk_reg_36864;
            elite_i2c_xfer_by_dev_id(0, &msg, 1);   
            break;
		default:
			break;
    }
    return 0;
}


int elite_i2s_set_channel_mmap(unsigned int spdif)
{
    int channels = 0;
    unsigned int samp_cfg_val = 0;
    
    if(spdif)
        channels = 4;
    else
        channels = 2;

    samp_cfg_val = readl(ASMPFCFG_ADDR);
    samp_cfg_val &= ~ASMPF_ENABLE;
    writel(samp_cfg_val, ASMPFCFG_ADDR);
    
    //update channels
    samp_cfg_val &= 0xfffffff0;
    switch(channels) {
        case 1:
            samp_cfg_val |= 0x1;
            break;
        case 2:
            samp_cfg_val |= 0x2;
            break;
        case 4:
            samp_cfg_val |= 0x4;
            break;
        default:
            break;
    }
    writel(samp_cfg_val, ASMPFCFG_ADDR);

    //update the channel router
    switch(channels) {
        case 2:
            writel(0x76541010, ASMPFCHCFG_ADDR);
            break;
        case 4:
            writel(0x76541032, ASMPFCHCFG_ADDR);
            break;
        default:
            break;
    }
    
    samp_cfg_val |= ASMPF_ENABLE;
    writel(samp_cfg_val, ASMPFCFG_ADDR);
    
    return 0;
}
EXPORT_SYMBOL_GPL(elite_i2s_set_channel_mmap);


int elite_i2s_set_iec958(int audio)
{
    unsigned int statusa = 0;
    statusa = readl(DGOCS0A_ADDR);

    if(!audio) {
        statusa |= IEC958_AES0_NONAUDIO; 
        writel(statusa, DGOCS0A_ADDR);
        writel(statusa, DGOCS1A_ADDR);
    } else {
        statusa &= ~IEC958_AES0_NONAUDIO; 
        writel(statusa, DGOCS0A_ADDR);
        writel(statusa, DGOCS1A_ADDR);
    }
    return 0;
}
EXPORT_SYMBOL_GPL(elite_i2s_set_iec958);

static int elite_i2s_pm_suspend(struct device *dev)
{
    struct elite_i2s_dev *i2s_dev = dev_get_drvdata(dev);
    
    if (i2s_dev && i2s_dev->clk)
       clk_disable_unprepare(i2s_dev->clk);

    return 0;
}

static int elite_i2s_pm_resume(struct device *dev)
{
    int ret;
    struct elite_i2s_dev *i2s_dev = dev_get_drvdata(dev);
    
    if (i2s_dev && i2s_dev->clk) {
       ret = clk_prepare_enable(i2s_dev->clk);
       if (ret) {
           dev_err(dev, "clk_enable failed: %d\n", ret);
           return ret;
       }
    }
    

    return 0;    
}

static int elite_i2s_dai_trigger(struct snd_pcm_substream *substream, 
                              int cmd, struct snd_soc_dai *dai)
{
    struct elite_i2s_dev *i2s_dev = snd_soc_dai_get_drvdata(dai);
    int err = 0;
    int stream_id = substream->pstr->stream;
    u32 tmp;

    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
    case SNDRV_PCM_TRIGGER_RESUME:
    case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
            if(i2s_dev->clk_count == 0){
                tmp = readl(DACCFG_ADDR);
                tmp |= DACITF_ENABLE | (1 << 20);
                writel(tmp, DACCFG_ADDR);
            }
            tmp = readl(ASMPFCFG_ADDR);
            tmp |= ASMPF_ENABLE;
            writel(tmp, ASMPFCFG_ADDR);
#ifdef AUD_SPDIF_ENABLE
            tmp = readl(DGOCFG_ADDR);
            tmp |= DGOITF_ENABLE;
            writel(tmp, DGOCFG_ADDR);
#endif
            writel(0x1,DZDRQ8_CFG_ADDR);
            i2s_dev->clk_count ++;
        } else if (stream_id == SNDRV_PCM_STREAM_CAPTURE) {
            if(i2s_dev->clk_count == 0){
                tmp = readl(DACCFG_ADDR);
                tmp |= DACITF_ENABLE | (1 << 20);
                writel(tmp, DACCFG_ADDR);
            }
            tmp = readl(AADCFEN_ADDR);
            tmp |= AADCF_ENABLE;
            tmp |= 1<<2;
            writel(tmp,AADCFEN_ADDR);

            writel(0x0,DZDRQ9_CFG_ADDR);
            i2s_dev->clk_count ++;
        }
        break;
    
    case SNDRV_PCM_TRIGGER_STOP:
    case SNDRV_PCM_TRIGGER_SUSPEND:
    case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
            if(i2s_dev->clk_count == 0){
                tmp = readl(DACCFG_ADDR);
                tmp &= ~DACITF_ENABLE;
                writel(tmp, DACCFG_ADDR);
            }else
            {
                i2s_dev->clk_count --;
            }
            tmp = readl(ASMPFCFG_ADDR);
            tmp &= ~ASMPF_ENABLE;
            writel(tmp, ASMPFCFG_ADDR);
#ifdef AUD_SPDIF_ENABLE
            tmp = readl(DGOCFG_ADDR);
            tmp &= ~DGOITF_ENABLE;
            writel(tmp, DGOCFG_ADDR);
#endif
            writel(0x8,DZDRQ8_CFG_ADDR);
        } else if (stream_id == SNDRV_PCM_STREAM_CAPTURE) {
            if(i2s_dev->clk_count == 0){    
                tmp = readl(DACCFG_ADDR);
                tmp &= ~DACITF_ENABLE;
                writel(tmp, DACCFG_ADDR);
            }else
                i2s_dev->clk_count --;
            tmp = readl(AADCFEN_ADDR);
            tmp &= ~AADCF_ENABLE;
            tmp &= ~(1<<2);
            writel(tmp,AADCFEN_ADDR);
            writel(0x9,DZDRQ9_CFG_ADDR);
        }
        break;
    default:
        err = -EINVAL;
        break;
    }

    return err;
}
static int elite_i2s_dai_hw_params(struct snd_pcm_substream *substream, 
                           struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
    int samp_size, chan_num;
    unsigned int samp_cfg_val;

    u32 tmp;
    int stream_id = substream->pstr->stream;
    samp_size = params_format(params);


    if (stream_id == SNDRV_PCM_STREAM_PLAYBACK)
    {

        samp_cfg_val = readl(ASMPFCFG_ADDR);
        
        //set bit sample
        samp_cfg_val &= ~(ASMPF_16BIT_SMP | ASMPF_32BIT_SMP);
        switch (samp_size) 
        {
            case SNDRV_PCM_FORMAT_S8:
            case SNDRV_PCM_FORMAT_U8:
            {
                samp_cfg_val |= ASMPF_8BIT_SMP;
                break;
            }
            case SNDRV_PCM_FORMAT_S16_BE:
            case SNDRV_PCM_FORMAT_U16_BE:
            case SNDRV_PCM_FORMAT_S16_LE:
            case SNDRV_PCM_FORMAT_U16_LE:
            {
                samp_cfg_val |= ASMPF_16BIT_SMP;
                break;
            }
            case SNDRV_PCM_FORMAT_S24_BE:
            case SNDRV_PCM_FORMAT_U24_BE:
            case SNDRV_PCM_FORMAT_S32_BE:
            case SNDRV_PCM_FORMAT_U32_BE:    
            case SNDRV_PCM_FORMAT_S24_LE:
            case SNDRV_PCM_FORMAT_U24_LE:
            case SNDRV_PCM_FORMAT_S32_LE:
            case SNDRV_PCM_FORMAT_U32_LE:
            {
                samp_cfg_val |= ASMPF_32BIT_SMP;  
                break;
            }
            default:
            {
                return -EINVAL;
            }
        }
        
        //set endian
        switch (samp_size) 
        {
            case SNDRV_PCM_FORMAT_S16_BE:
            case SNDRV_PCM_FORMAT_U16_BE:
            case SNDRV_PCM_FORMAT_S24_BE:
            case SNDRV_PCM_FORMAT_U24_BE:
            case SNDRV_PCM_FORMAT_S32_BE:
            case SNDRV_PCM_FORMAT_U32_BE:
            {
                samp_cfg_val |= ASMPF_EXCH_ENDIAN;
                break;
            }
            case SNDRV_PCM_FORMAT_S16_LE:
            case SNDRV_PCM_FORMAT_U16_LE:
            case SNDRV_PCM_FORMAT_S24_LE:
            case SNDRV_PCM_FORMAT_U24_LE:
            case SNDRV_PCM_FORMAT_S32_LE:
            case SNDRV_PCM_FORMAT_U32_LE:
            {
                samp_cfg_val &= ~(ASMPF_EXCH_ENDIAN);
                break;
            }
            default:
            {
                return -EINVAL;
            }
        }
        
        //set fmt
        
        switch (samp_size) 
        {
            case SNDRV_PCM_FORMAT_U8:
            case SNDRV_PCM_FORMAT_U16_BE:
            case SNDRV_PCM_FORMAT_U16_LE:
            case SNDRV_PCM_FORMAT_U24_LE:
            case SNDRV_PCM_FORMAT_U24_BE:
            case SNDRV_PCM_FORMAT_U32_BE:
            case SNDRV_PCM_FORMAT_U32_LE:
            {
                samp_cfg_val |= ASMPF_EXCH_FMT;
                break;
            }
            case SNDRV_PCM_FORMAT_S8:
            case SNDRV_PCM_FORMAT_S16_BE:
            case SNDRV_PCM_FORMAT_S16_LE:
            case SNDRV_PCM_FORMAT_S24_BE:
            case SNDRV_PCM_FORMAT_S24_LE:
            case SNDRV_PCM_FORMAT_S32_LE:
            case SNDRV_PCM_FORMAT_S32_BE:
            {
                samp_cfg_val &= ~(ASMPF_EXCH_FMT);
                break;
            }
            default:
            {
                return -EINVAL;
            }
        }

        chan_num = params_channels(params);
        if (chan_num == 1)
            samp_cfg_val |= 0x1;
        else if (chan_num == 2)
            samp_cfg_val |= 0x2;
        else if (chan_num == 4)
            samp_cfg_val |= 0x4;
        writel(samp_cfg_val, ASMPFCFG_ADDR);
    } else if (stream_id == SNDRV_PCM_STREAM_CAPTURE) 
    {       
        tmp = readl(IO_ADDRESS(0xD8110200));  //gpio pinshare 
        tmp &=~ 0x2;                   //set 0: PCM/I2S DAC
        tmp |= 0x1;                    //set 1: PCM
        writel(tmp,IO_ADDRESS(0xD8110200)); 
        tmp = readl(ADCCFG_ADDR);
        tmp &=~ ADCITF_MASTER;  //set slave mode 
        tmp |= 0x1;             // Padding config  
        writel(tmp,ADCCFG_ADDR);

        samp_cfg_val = readl(AADCFEN_ADDR);
        samp_cfg_val |= BIT1;   //set AADCFIFO 16-bit mode
	writel(samp_cfg_val, AADCFEN_ADDR);
        tmp = readl(AADCFSTATE_ADDR);
    }
    return 0;
}


/*
 * This must be called before _set_clkdiv and _set_sysclk since McBSP register
 * cache is initialized here
 */
static int elite_i2s_dai_set_fmt(struct snd_soc_dai *cpu_dai,
                      unsigned int fmt)
{
    switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
    case SND_SOC_DAIFMT_I2S:
        break;
    default:
        /* Unsupported data format */
        return -EINVAL;
    }

    switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
    case SND_SOC_DAIFMT_CBS_CFS:
        break;
    case SND_SOC_DAIFMT_CBM_CFM:
        break;
    default:
        return -EINVAL;
    }

    /* Set bit clock (CLKX/CLKR) and FS polarities */
    switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
    case SND_SOC_DAIFMT_NB_NF:
        /*
         * Normal BCLK + FS.
         * FS active low. TX data driven on falling edge of bit clock
         * and RX data sampled on rising edge of bit clock.
         */
        break;
    case SND_SOC_DAIFMT_NB_IF:
        break;
    case SND_SOC_DAIFMT_IB_NF:
        break;
    case SND_SOC_DAIFMT_IB_IF:
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

/*
 * I2S operations though which actual audio data is
 * transmitted.
 */
static struct snd_soc_dai_ops elite_i2s_dai_ops = {
    .trigger     = elite_i2s_dai_trigger,
    .hw_params   = elite_i2s_dai_hw_params,
    .set_fmt     = elite_i2s_dai_set_fmt,
};

static int elite_i2s_dai_probe(struct snd_soc_dai *dai)
{
    struct elite_i2s_dev *i2s_dev = snd_soc_dai_get_drvdata(dai);

    dai->playback_dma_data = &i2s_dev->playback_dma_params;
    dai->capture_dma_data = &i2s_dev->capture_dma_params;
    return 0;
}

#ifdef CONFIG_PM
static int elite_i2s_dai_suspend(struct snd_soc_dai *dai)
{
    struct elite_i2s_dev *i2s_dev = snd_soc_dai_get_drvdata(dai);
    struct elite_i2s_regs *regs = &i2s_dev->regs;
    regs->ADCCFG = readl(ADCCFG_ADDR);
    regs->AADCFEN = readl(AADCFEN_ADDR);
 
    regs->DGOCFG = readl(DGOCFG_ADDR);
    regs->DGOCS0A = readl(DGOCS0A_ADDR);
    regs->DGOCS1A = readl(DGOCS1A_ADDR);
    regs->ADGICFG = readl(ADGICFG_ADDR);
    regs->ASMPFCFG = readl(ASMPFCFG_ADDR);
    regs->DACCFG = readl(DACCFG_ADDR);

    return 0;
}

static int elite_i2s_dai_resume(struct snd_soc_dai *dai)
{
    struct elite_i2s_dev *i2s_dev = snd_soc_dai_get_drvdata(dai);
    struct elite_i2s_regs *regs = &i2s_dev->regs;

    writel(regs->ADCCFG,ADCCFG_ADDR);
    writel(regs->AADCFEN,AADCFEN_ADDR);
    writel(0x0,AUDPRFRST_ADDR);
    writel(regs->DGOCFG, DGOCFG_ADDR);

    writel(regs->DGOCS0A, DGOCS0A_ADDR);
    writel(regs->DGOCS1A, DGOCS1A_ADDR);
    writel(0x1,DGOCSRDY_ADDR);

    writel(regs->ADGICFG, ADGICFG_ADDR);
    writel(regs->ASMPFCFG, ASMPFCFG_ADDR);
    writel(regs->DACCFG, DACCFG_ADDR);
    writel(0x7f,AUDPRFRST_ADDR);

    writel(0x1,DZDRQ8_CFG_ADDR);
    writel(0x0,DZDRQ9_CFG_ADDR);

    //set clk
    elite_i2s_dev_init(i2s_dev);

    return 0;
}

#else
#define elite_i2s_dai_suspend NULL
#define elite_i2s_dai_resume NULL
#endif

struct snd_soc_dai_driver elite_i2s_dai = {
    .probe    = elite_i2s_dai_probe,
    .suspend = elite_i2s_dai_suspend,
    .resume = elite_i2s_dai_resume,
    .playback = {
                .channels_min = 2,
                .channels_max = 2,
                .rates        = ELITE_I2S_RATES,
                .formats      = SNDRV_PCM_FMTBIT_S16_LE | 
                                SNDRV_PCM_FMTBIT_U8     |
                                SNDRV_PCM_FMTBIT_S24_LE |
                                SNDRV_PCM_FMTBIT_FLOAT,
                },
    .capture  = {
                .channels_min = 2,
                .channels_max = 2,
                .rates        = ELITE_I2S_RATES,
                .formats      = SNDRV_PCM_FMTBIT_S16_LE |
                                SNDRV_PCM_FMTBIT_U8,
                },
    .ops     = &elite_i2s_dai_ops,
};

EXPORT_SYMBOL_GPL(elite_i2s_dai);

static int elite_i2s_dev_init(struct elite_i2s_dev *i2s_dev)
{
    if ((system_rev == 0x71a1) || (system_rev == 0x90a1)) {
        void __iomem *reg_gpio_base = (void *)IO_ADDRESS(ELITE_GPIO_BASE);
    
        //hard code clock.
        unsigned int value = 0;
        unsigned int reg = 0x200;
        //use external clk
        __raw_writeb(value, reg_gpio_base + reg); 
    
        __raw_writeb(0x10, reg_gpio_base + 0x4d); 
        __raw_writeb(0x10, reg_gpio_base + 0x8d); 
        __raw_writeb(0x10, reg_gpio_base + 0xcd);

        //set clock value
       elite_i2s_ext_clk_set(36864);
      
       i2s_dev->clk = NULL;
    }  else {
        i2s_dev->clk = clk_get_sys("elite-i2s.0", NULL);
        if(IS_ERR(i2s_dev->clk)) {
            printk(" clk_get i2s error. n");
            return -ENODEV;
        }

        i2s_dev->clk_count = 0;
        if((system_rev == 0x90a2) || (system_rev == 0xa0a2)) {
            void __iomem *reg_gpio_base = (void *)IO_ADDRESS(ELITE_GPIO_BASE);
            __raw_writeb(0x10, reg_gpio_base + 0x4d); 
            __raw_writeb(0x10, reg_gpio_base + 0x8d); 
            __raw_writeb(0x10, reg_gpio_base + 0xcd);
        }
    }


    return 0;
}

static int __devinit elite_i2s_probe(struct platform_device *pdev)
{
    int ret;
    struct elite_i2s_dev *i2s_dev;
    
    i2s_dev = kzalloc(sizeof(struct elite_i2s_dev), GFP_KERNEL);
    if(!i2s_dev) {
        ret = -ENOMEM;
        goto err;
    }
 
    dev_set_drvdata(&pdev->dev, i2s_dev);
   
    ret = elite_i2s_dev_init(i2s_dev);
    if (ret) {
        dev_err(&pdev->dev, "elite i2s clk init failed: %d\n", ret);
        goto clk_init_err;
    }

    i2s_dev->playback_dma_params.id = "Elite_I2S_Out";
    i2s_dev->playback_dma_params.dmach = NULL_DMA;
    i2s_dev->playback_dma_params.dma_dev = AHB1_AUD_DMA_REQ_1;
    spin_lock_init(&i2s_dev->playback_dma_params.dma_lock);
    i2s_dev->capture_dma_params.id = "Elite_I2S_In";
    i2s_dev->capture_dma_params.dmach = NULL_DMA;
    i2s_dev->capture_dma_params.dma_dev = AHB1_AUD_DMA_REQ_0;
    spin_lock_init(&i2s_dev->capture_dma_params.dma_lock);

    pm_runtime_enable(&pdev->dev);
    if (!pm_runtime_enabled(&pdev->dev)) {
        ret = elite_i2s_pm_resume(&pdev->dev);
        if (ret)
            goto pm_resume_err;
    }

    /* register with the ASoC layers */
    ret = snd_soc_register_dai(&pdev->dev, &elite_i2s_dai);
    if(ret != 0) {
        dev_err(&pdev->dev, "could not register DAI: %d\n", ret);
        goto register_dai_err;
    }
	dev_info(&pdev->dev, "i2s probed done\n", pdev->id);

    return 0;

register_dai_err:
   if (!pm_runtime_status_suspended(&pdev->dev))
       elite_i2s_pm_suspend(&pdev->dev);
pm_resume_err:
    if (i2s_dev->clk)
        clk_put(i2s_dev->clk);
clk_init_err:
    kfree(i2s_dev);
err:
    return ret;
}

static int __devexit elite_i2s_remove(struct platform_device *pdev)
{
    struct elite_i2s_dev *i2s_dev = platform_get_drvdata(pdev);

    pm_runtime_disable(&pdev->dev);
    if (!pm_runtime_status_suspended(&pdev->dev))
        elite_i2s_pm_suspend(&pdev->dev);

    snd_soc_unregister_dai(&pdev->dev);

    if (i2s_dev->clk)
        clk_put(i2s_dev->clk);
    kfree(i2s_dev);

    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id elite_i2s_of_match[] = {
	{ .compatible = "s3graphics,elite1000-i2s" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, elite_i2s_of_match);
#endif

static const struct dev_pm_ops elite_i2s_pm_ops = {
    SET_RUNTIME_PM_OPS(elite_i2s_pm_suspend,
                        elite_i2s_pm_resume, NULL)
};

static struct platform_device_id elite_i2s_driver_ids[] = {
	{
		.name  = "elite-i2s",
		.driver_data	= (kernel_ulong_t)NULL,
	}, {
		.name  = "elite-i2s.0",
		.driver_data	= (kernel_ulong_t)NULL,
    },
	{ /* sentinel */ },
};

static struct platform_driver elite_i2s_driver = {
    .probe  = elite_i2s_probe,
    .remove = __devexit_p(elite_i2s_remove),
	.id_table  = elite_i2s_driver_ids,
    .driver = {
        .name = "s3graphics-elite-i2s",
        .owner = THIS_MODULE,
        .pm = &elite_i2s_pm_ops,
		.of_match_table = of_match_ptr(elite_i2s_of_match),
    },
};

module_platform_driver(elite_i2s_driver);

MODULE_AUTHOR("S3 Graphics Co.,Ltd");
MODULE_DESCRIPTION("ELite Alsa SoC Driver");
MODULE_LICENSE("GPL");
