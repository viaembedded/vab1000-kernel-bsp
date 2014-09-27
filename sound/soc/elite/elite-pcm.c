/*
 * ASoC Platform Driver for S3 Elite Chips
 * Part 2:
 *       Elite Audio DMA Management
 *
 * This part of driver is just dependent on elite chips, no matter which
 * kind of codec is used and how development board is designed. Any change on it
 * should maintain the independence of this driver from codecs and development
 * board.
 *
 * Copyright (c) 2011 S3 Graphics,Co.,Ltd
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/asound.h>
#include <asm/dma.h>

#include "elite-pcm.h"
#include "elite-i2s.h"

#define NULL_DMA                ((dmach_t)(-1))

static struct snd_dma_buffer dump_buf[1];/*Used to dump mono data*/

static const struct snd_pcm_hardware elite_pcm_hardware = {
    .info            = SNDRV_PCM_INFO_MMAP |
                       SNDRV_PCM_INFO_MMAP_VALID |
                       SNDRV_PCM_INFO_INTERLEAVED |
                       SNDRV_PCM_INFO_PAUSE |
                       SNDRV_PCM_INFO_RESUME,
    .formats         = SNDRV_PCM_FMTBIT_S16_LE |
                       SNDRV_PCM_FMTBIT_U16_LE |
                       SNDRV_PCM_FMTBIT_S24_LE |
                       SNDRV_PCM_FMTBIT_U24_LE |
                       SNDRV_PCM_FMTBIT_U8 |
                       SNDRV_PCM_FMTBIT_S8,
    .period_bytes_min    = 64 * 64,
    .period_bytes_max    = 4 * 64 * 64,
    .periods_min         = 1,
    .periods_max         = 8,
    .buffer_bytes_max    = 64 * 1024,
    .fifo_size           = 32,
};

static struct dma_req_s i2s_dma_req = {
    .addr_wrp_bnd = ELITE_DMA_1_BURST,
    .bst_len = ELITE_DMA_BURST_LEN_INCR8,
    .trans_size = ELITE_DMA_RANS_SIZE_32BIT,
    .if12_addr_mode = ELITE_DMA_ADDR_MODE_WRAP,
    .sw_req = ELITE_DMA_REQ_HW,
    .chunk_size = 1,
};

unsigned char elite_hw_mute = 1;

static void elite_pcm_fmt_trans(int fmt, int channel, char *src_buf, char *dst_buf, unsigned int chunksize)
{
    unsigned int index = 0;
    float_data_t f_data;
    unsigned short data;

    if ((fmt == SNDRV_PCM_FORMAT_FLOAT) && (channel == 2)) {
        /* transfer from 2ch float(8 bytes) to 2ch s16le(4 bytes) */
        for (index = 0; index < (chunksize / 4); ++index) {
            f_data = *((float_data_t *)src_buf + index);
            
            if (!f_data.sign)  
                data = (f_data.frac + 0x800000) >> (8 - (f_data.exp - 127));
            else
                data = ~((f_data.frac + 0x800000) >> (8 - (f_data.exp - 127))) + 1;

            *((unsigned short *)dst_buf + index) = data;
        }
    } else if ((fmt == SNDRV_PCM_FORMAT_FLOAT) && (channel == 1)) {
        /* transfer from 1ch float(4 bytes) to 2ch s16le(4 bytes) */
        for (index = 0; index < (chunksize / 4); ++index) {
            f_data = *((float_data_t *)src_buf + index);
            
            if (!f_data.sign)
                data = (unsigned short)((f_data.frac + 0x800000) >> (8 - (f_data.exp - 127)));
            else
                data = (unsigned short)(~((f_data.frac + 0x800000) >> (8 - (f_data.exp - 127))) + 1);

            *((unsigned int *)dst_buf + index) = (data << 16) | data;
        }
    }    
}

/* this may get called several times by oss emulation */
static int elite_pcm_hw_params(struct snd_pcm_substream *substream,
                  struct snd_pcm_hw_params *params)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_pcm_runtime *runtime = substream->runtime;
    
    snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
    runtime->dma_bytes = params_buffer_bytes(params);
    runtime->private_data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
    
    return 0;
}

static int elite_pcm_hw_free(struct snd_pcm_substream *substream)
{   
    snd_pcm_set_runtime_buffer(substream, NULL);

    return 0;
}

/*
 *  Main dma routine, requests dma according where you are in main alsa buffer
 */
static void elite_pcm_dma_process(struct elite_pcm_dma_params *s)
{
    struct snd_pcm_substream *substream = s->stream;
    struct snd_pcm_runtime *runtime;
    unsigned int dma_size;
    unsigned int offset;
    dma_addr_t dma_base;
    int ret = 0;
    int stream_id = substream->pstr->stream;
    
    if (s->active) {
        substream = s->stream;
        runtime = substream->runtime;
        dma_size = frames_to_bytes(runtime, runtime->period_size);

        if (dma_size > MAX_DMA_SIZE)
            dma_size = CUT_DMA_SIZE;
        offset = dma_size * s->period;

        dma_base = __virt_to_phys((dma_addr_t)runtime->dma_area);

        if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
            if ((runtime->channels == 2) && (runtime->format == SNDRV_PCM_FORMAT_FLOAT)) {
                elite_pcm_fmt_trans(runtime->format, runtime->channels,
                    (unsigned char *)runtime->dma_buffer_p->area + offset,
                    (unsigned char *)dump_buf[stream_id].area + (offset / 2),
                    dma_size);
                ret = elite_start_dma(s->dmach, dump_buf[stream_id].addr + (offset / 2), 0, (dma_size / 2));
            } else if ((runtime->channels == 1) && (runtime->format == SNDRV_PCM_FORMAT_FLOAT)) {
                elite_pcm_fmt_trans(runtime->format, runtime->channels,
                    (unsigned char *)runtime->dma_buffer_p->area + offset,
                    (unsigned char *)dump_buf[stream_id].area + offset,
                    dma_size);
                ret = elite_start_dma(s->dmach, dump_buf[stream_id].addr + offset, 0, dma_size);
            } else {
                ret = elite_start_dma(s->dmach, runtime->dma_addr + offset, 0, dma_size);
            }
        } else {
            ret = elite_start_dma(s->dmach, runtime->dma_addr + offset, 0, dma_size);
        }

        if (ret) {
            printk(KERN_ERR "audio_process_dma: cannot queue DMA buffer (%i)\n", ret);
            return;
        }

        s->period++;
        s->period %= runtime->periods;
        s->periods++;
        s->offset = offset;
    }
}

/* 
 *  This is called when dma IRQ occurs at the end of each transmited block
 */
static void elite_pcm_dma_callback(void *data)
{
    struct elite_pcm_dma_params *s = data;
    
    /* 
     * If we are getting a callback for an active stream then we inform
     * the PCM middle layer we've finished a period
     */
    if (s->active)
        snd_pcm_period_elapsed(s->stream);

    spin_lock(&s->dma_lock);
    if (s->periods > 0) 
        s->periods--;
    
    elite_pcm_dma_process(s);
    spin_unlock(&s->dma_lock);
}

static int elite_pcm_dma_request(struct elite_pcm_dma_params *s, void (*callback) (void *))
{
    struct snd_pcm_substream *substream = s->stream;
    int stream_id = substream->stream;
    int err;
    err = 0;

    i2s_dma_req.device_id = s->id;
    i2s_dma_req.device = s->dma_dev;
    i2s_dma_req.callback = callback; 
    i2s_dma_req.callback_data = s;

    if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
        i2s_dma_req.device = 21;
        i2s_dma_req.trans_dir = ELITE_DMA_TRANS_IF0_TO_IF12;
        i2s_dma_req.fifo_addr = I2S_TX_FIFO; 
        i2s_dma_req.bst_len = ELITE_DMA_BURST_LEN_INCR8;
    } else {
        i2s_dma_req.device = 22;
        i2s_dma_req.trans_dir = ELITE_DMA_TRANS_IF12_TO_IF0;
        i2s_dma_req.fifo_addr = I2S_RX_FIFO; 
        i2s_dma_req.bst_len = ELITE_DMA_BURST_LEN_SINGLE;
    }

    err = elite_request_dma(&i2s_dma_req);
    s->dmach = i2s_dma_req.channel;

    if (err < 0)
        printk(KERN_ERR "unable to grab audio DMA 0x%0x\n", s->dmach);

    return err;
}

static int elite_pcm_dma_free(struct elite_pcm_dma_params *s)
{
    int err = 0;
    
    elite_free_dma(s->dmach);
    s->dmach = NULL_DMA;
    
    return err;
}

static int elite_pcm_prepare(struct snd_pcm_substream *substream)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct elite_pcm_dma_params *s = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

    s->period = 0;
    s->periods = 0;
    s->offset = 0;

    return 0;
}

/*
 * this stops the dma and clears the dma ptrs
 */
static void audio_stop_dma(struct elite_pcm_dma_params *s)
{
    unsigned long flags;
    
    local_irq_save(flags);
    s->active = 0;
    s->period = 0;
    s->periods = 0;
    s->offset = 0;
    elite_stop_dma(s->dmach);
    elite_clear_dma(s->dmach);
    local_irq_restore(flags);
}

static int elite_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
    int stream_id = substream->pstr->stream;
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct elite_pcm_dma_params *s = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
    int ret = 0;
    
    spin_lock(&s->dma_lock);
    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
    case SNDRV_PCM_TRIGGER_RESUME:
    case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        s->active = 1;
        if ((stream_id == SNDRV_PCM_STREAM_PLAYBACK) && (elite_hw_mute)) {
            /* mute disable */
            //GPIO_OD_GP2_WAKEUP_SUS_BYTE_VAL |= BIT5;
            elite_hw_mute = 0;
            mdelay(10);
        }
        elite_pcm_dma_process(s);
        break;

    case SNDRV_PCM_TRIGGER_STOP:
    case SNDRV_PCM_TRIGGER_SUSPEND:
    case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        s->active = 0;
        audio_stop_dma(s);
        break;
    default:
        ret = -EINVAL;
    }
    spin_unlock(&s->dma_lock);

    return ret;
}

static snd_pcm_uframes_t elite_pcm_pointer(struct snd_pcm_substream *substream)
{
    int stream_id = substream->pstr->stream;
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct elite_pcm_dma_params *s = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
    dma_addr_t ptr;
    snd_pcm_uframes_t offset = 0;
    
    ptr = elite_get_dma_pos(s->dmach);

    if ((runtime->channels == 2) && (runtime->format == SNDRV_PCM_FORMAT_FLOAT)) {
        offset = bytes_to_frames(runtime, (ptr - dump_buf[stream_id].addr) << 1);
    } else if ((runtime->channels == 1) && (runtime->format == SNDRV_PCM_FORMAT_FLOAT)) {
        offset = bytes_to_frames(runtime, ptr - dump_buf[stream_id].addr);
    } else {
        offset = bytes_to_frames(runtime, ptr - runtime->dma_addr);
    }

    if (offset >= runtime->buffer_size)
        offset = 0;

    spin_lock(&s->dma_lock);

    if (s->periods > 0 && s->periods < 2) {
        if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
            if (snd_pcm_playback_hw_avail(runtime) >= 2*runtime->period_size)
                elite_pcm_dma_process(s);
        } else {
            if (snd_pcm_capture_hw_avail(runtime) >= 2*runtime->period_size)
                elite_pcm_dma_process(s);
        }
        
    }
    spin_unlock(&s->dma_lock);

    return offset;
}

static int elite_pcm_open(struct snd_pcm_substream *substream)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct elite_pcm_dma_params *s = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
    int ret;
    
    if (!s)
    {
        printk("elite_pcm_open failed at 1\n");
        return -1;
    }
    s->stream = substream;

    if (!s->active) {
        elite_pcm_dma_request(s, elite_pcm_dma_callback);
    }

    snd_soc_set_runtime_hwparams(substream, &elite_pcm_hardware);

    /* Ensure that buffer size is a multiple of period size */
    ret = snd_pcm_hw_constraint_integer(runtime,
                        SNDRV_PCM_HW_PARAM_PERIODS);
    if (ret < 0)
    {
        printk("elite_pcm_open failed at 2\n");
        goto out;
    }
    return 0;

out:
    return ret;
}

static int elite_pcm_close(struct snd_pcm_substream *substream)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
    struct elite_pcm_dma_params *s = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

    if (!s->active) {
        elite_pcm_dma_free(s);
    }

    return 0;
}

static int elite_pcm_mmap(struct snd_pcm_substream *substream,
    struct vm_area_struct *vma)
{
    struct snd_pcm_runtime *runtime = substream->runtime;

    return dma_mmap_writecombine(substream->pcm->card->dev, vma,
                     runtime->dma_area,
                     runtime->dma_addr,
                     runtime->dma_bytes);
}

struct snd_pcm_ops elite_pcm_ops = {
    .open        = elite_pcm_open,
    .close        = elite_pcm_close,
    .ioctl        = snd_pcm_lib_ioctl,
    .hw_params    = elite_pcm_hw_params,
    .hw_free    = elite_pcm_hw_free,
    .prepare    = elite_pcm_prepare,
    .trigger    = elite_pcm_trigger,
    .pointer    = elite_pcm_pointer,
    .mmap        = elite_pcm_mmap,
};

static u64 elite_pcm_dmamask = DMA_BIT_MASK(32);

static int elite_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
    struct snd_pcm_substream *substream = pcm->streams[stream].substream;
    struct snd_dma_buffer *buf = &substream->dma_buffer;
    size_t size = elite_pcm_hardware.buffer_bytes_max;
    
    buf->dev.type = SNDRV_DMA_TYPE_DEV;
    buf->dev.dev = pcm->card->dev;
    buf->private_data = NULL;
    buf->area = dma_alloc_writecombine(pcm->card->dev, size,
                       &buf->addr, GFP_KERNEL);
    
    if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
        dump_buf[0].area = dma_alloc_writecombine(pcm->card->dev, size,
                       &(dump_buf[stream].addr), GFP_KERNEL);
    }

    if (!buf->area)
        return -ENOMEM;

    buf->bytes = size;
    
    return 0;
}

static void elite_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
    struct snd_pcm_substream *substream;
    struct snd_dma_buffer *buf;
    int stream;
    
    for (stream = 0; stream < 2; stream++) {
        substream = pcm->streams[stream].substream;
        if (!substream)
            continue;

        buf = &substream->dma_buffer;
        if (!buf->area)
            continue;

        dma_free_writecombine(pcm->card->dev, buf->bytes,
                      buf->area, buf->addr);
        
        if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
            dma_free_writecombine(pcm->card->dev, 4 * buf->bytes,
                      dump_buf[stream].area, dump_buf[stream].addr);
        }
        
        buf->area = NULL;
    }
}

static int elite_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
    struct snd_card *card = rtd->card->snd_card;
    struct snd_pcm *pcm = rtd->pcm;
    int ret = 0;

    if (!card->dev->dma_mask)
        card->dev->dma_mask = &elite_pcm_dmamask;
    if (!card->dev->coherent_dma_mask)
        card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

    if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
        ret = elite_pcm_preallocate_dma_buffer(pcm,
            SNDRV_PCM_STREAM_PLAYBACK);
        if (ret)
            goto out;
    }

    if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
        ret = elite_pcm_preallocate_dma_buffer(pcm,
            SNDRV_PCM_STREAM_CAPTURE);
        if (ret)
            goto out;
    }

out:
    return ret;
}

#ifdef CONFIG_PM
static int elite_pcm_suspend(struct snd_soc_dai *dai)
{
    struct snd_pcm_runtime *runtime = dai->runtime;
    struct elite_pcm_dma_params *s;

    if (!runtime)
        return 0;

    /* If not null, it is the current DMA channel */
    s = runtime->private_data;

    if (s && s->active) {
        udelay(5);
        elite_stop_dma(s->dmach);
    }

    return 0;
}

static int elite_pcm_resume(struct snd_soc_dai *dai)
{
    struct snd_pcm_runtime *runtime = dai->runtime;
    struct elite_pcm_dma_params *s;

    *(unsigned int*)0xfe130254 |= 0x30; /*enalbe dma clock*/
    udelay(1);

    if (!runtime)
        return 0;
    
    s = runtime->private_data;
    if (s && s->active) {
        elite_resume_dma(s->dmach) ;
    }

    elite_hw_mute = 1;
    return 0;
}
#else
#define elite_pcm_suspend       NULL
#define elite_pcm_resume        NULL
#endif


struct snd_soc_platform_driver elite_snd_platform = {
    .ops     = &elite_pcm_ops,
    .pcm_new     = elite_pcm_new,
    .pcm_free    = elite_pcm_free_dma_buffers,
    .suspend     = elite_pcm_suspend,
    .resume      = elite_pcm_resume,
};
EXPORT_SYMBOL_GPL(elite_snd_platform);

static int __devinit elite_i2s_soc_platform_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "sound-pmc driver start probing\n");
    return snd_soc_register_platform(&pdev->dev, &elite_snd_platform);
}

static int __devexit elite_i2s_soc_platform_remove(struct platform_device *pdev)
{
    snd_soc_unregister_platform(&pdev->dev);
    return 0;
}

static struct platform_driver elite_i2s_pcm_driver = {
    .driver = {
        .name = "elite-i2s-pcm-audio",
        .owner = THIS_MODULE,
    },
    .probe = elite_i2s_soc_platform_probe,
    .remove = __devexit_p(elite_i2s_soc_platform_remove),
};

module_platform_driver(elite_i2s_pcm_driver);

MODULE_AUTHOR("Elite Graphics Co.,Ltd");
MODULE_DESCRIPTION("Elite PCM DMA Module");
MODULE_LICENSE("GPL");
