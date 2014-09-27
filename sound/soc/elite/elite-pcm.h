#ifndef _ELITE_PCM_H
#define _ELITE_PCM_H

#include <asm/dma.h>
#include <mach/dma.h>

typedef struct float_data {
    unsigned long frac : 23;
    unsigned long exp : 8;
    unsigned long sign : 1;
} float_data_t;

struct  elite_pcm_dma_params {
    char  *id;               /* identification string */
    dmach_t dmach;          /* DMA channel number */
    struct dma_device_cfg_s dma_cfg;   /* DMA device config */
    int dma_dev;            /* dma number of that device */
    int dma_q_head;         /* DMA Channel Q Head */
    int dma_q_tail;         /* DMA Channel Q Tail */
    char dma_q_count;       /* DMA Channel Q Count */
    int active:1;           /* we are using this stream for transfer now */
    int period;             /* current transfer period */
    int periods;            /* current count of periods registerd in the DMA engine */
    spinlock_t dma_lock;    /* for locking in DMA operations */
    struct snd_pcm_substream *stream;       /* the pcm stream */
    unsigned linked:1;      /* dma channels linked */
    int offset;             /* store start position of the last period in the alsa buffer */
};

#endif
