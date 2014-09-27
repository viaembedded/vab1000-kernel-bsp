/*
 * ASoC Machine (board) Driver for WM8900 Codec On S3 Elite-based Development Kit
 *
 * This part driver glues platform driver and codec driver together. It also provides
 * board specific functionalities. It should maintain updated when codec chip is changed
 * or development board design is revised.
 *
 * Copyright (c) 2011 S3 Graphics Co.,Ltd
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>
#include <linux/module.h>
#include <sound/pcm_params.h>
#include <asm/io.h>
#include <linux/slab.h>
#include <mach/io.h>
#include "elite-pcm.h"
#include "elite-i2s.h"

extern struct snd_soc_dai elite_i2s_dai;
extern struct snd_soc_platform elite_snd_platform;
extern unsigned int system_rev;

static struct i2s_wm8960_clk {
    int  rate;
    int  dac_div;
    unsigned int mclk;   
} wm8960_clks[] = {
    {
       48000, 0, 12288000, 
    },
    {
       32000, 1, 12288000, 
    },
    {
       24000, 2, 12288000, 
    },
    {
       16000, 3, 12288000, 
    },
    {
       12000, 4, 12288000, 
    },
    {
       8000, 6, 12288000, 
    },
    {
       44100, 0, 11289600, 
    },
    {
       22050, 1, 11289600, 
    },
    {
       11025, 2, 11289600, 
    }
};

static int spdif_clk_div[] = {1, 2, 4, 8, 16, 0, 0, 0, 0, 3, 6, 12, 24};
static int i2s_clk_divA[] = {1, 2, 4, 8, 16, 0, 0, 0, 0, 3, 6, 12, 24};
static int i2s_clk_divB[] = {2, 4, 8, 0, 3, 6, 12};
static int i2s_clk_divC[] = {64, 32, 48};

static unsigned int sys_clks[] = {
    16384000, 24576000, 22597200,
    33868800, 36864000
};

static int elite_i2s_clk_set(struct clk *clk, unsigned int value)
{
    int ret = 0;
    struct clk *pclk = NULL;
    pclk = clk_get_parent(clk);
    if(!pclk) {
        return -1;
    }

    ret = clk_set_rate(pclk, value);
    if(ret) {
        return -1;
    }

    ret = clk_set_rate(clk, value);
    if(ret) {
        return -1;
    }

    return 0;
}

struct clk_params {
    int a;
    int b;
    int c;
    int d;
    int dac_div;
};

static struct clk_params *determine_clk_params(int samp_rate, int samp_bits, 
           int chan_num, int spdif_bclk, unsigned int *sys_clk)
{
    int i, j, z, c;
    unsigned int mclk, bit_clk;
    struct clk_params *params = kmalloc(sizeof(struct clk_params), GFP_KERNEL);
    if (!params)
        return NULL;
    
    for (i = 0; i < sizeof(wm8960_clks)/sizeof(wm8960_clks[0]); i++)
        if (wm8960_clks[i].rate == samp_rate)
            break;

    if (i == sizeof(wm8960_clks)/sizeof(wm8960_clks[0]))
        goto err;
 
    mclk = wm8960_clks[i].mclk;
    params->dac_div = wm8960_clks[i].dac_div;

    c = samp_bits*chan_num;
    for (i = 0; i < sizeof(i2s_clk_divC)/sizeof(i2s_clk_divC[0]); i++)
        if (i2s_clk_divC[i] == c)
            break;

    if (i == sizeof(i2s_clk_divC)/sizeof(i2s_clk_divC[0]))
        goto err;

    params->c = i;

    bit_clk = samp_rate*c;
    for (i = 0; i < sizeof(i2s_clk_divB)/sizeof(i2s_clk_divB[0]); i++)
        if (bit_clk*i2s_clk_divB[i] == mclk)
            break;

    if (i == sizeof(i2s_clk_divB)/sizeof(i2s_clk_divB[0]))
        goto err;

    params->b = i;

    for (i = 0; i < sizeof(sys_clks)/sizeof(sys_clks[0]); i++) {
        for (j = 0; j < sizeof(i2s_clk_divA)/sizeof(i2s_clk_divA[0]); j++)
            if (mclk*i2s_clk_divA[j] == sys_clks[i])
                break;

        if (j == sizeof(i2s_clk_divA)/sizeof(i2s_clk_divA[0]))
            continue;
        else { 
            /* now the i2s system clock is ok for I2S DAC, check it for SPDIF */
            if (spdif_bclk == 0) {
                params->a = j;
                break;
            } else  {
                for (z = 0; z < sizeof(spdif_clk_div)/sizeof(spdif_clk_div[0]); z++)
                    if (spdif_clk_div[z]*spdif_bclk == sys_clks[i])
                        break;

                if (z == sizeof(spdif_clk_div)/sizeof(spdif_clk_div[0]))
                    continue;
                else {
                    params->a = j;
                    params->d = z;
                    break;
                }
            }
        }
    }

    if (i == sizeof(sys_clks)/sizeof(sys_clks[0]))
        goto err;

    *sys_clk = sys_clks[i];
    return params;

err:
    kfree(params);
    return NULL;
}

static int elite_snd_hw_params(struct snd_pcm_substream *substream,
                                       struct snd_pcm_hw_params *params)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *codec_dai = rtd->codec_dai;
    struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
    struct elite_i2s_dev *i2s_dev = snd_soc_dai_get_drvdata(cpu_dai);
    int samp_rate, samp_bits, chan_num;
    int a, b, c;
    unsigned int sys_clk;
    int stream_id = substream->pstr->stream;
    unsigned int tmp;
    unsigned int padding = 0;
    unsigned int format = 0;

#ifdef AUD_SPDIF_ENABLE
    int d;
    int spdif_bclk;
    unsigned int status;
#endif
    writel(0x0,AUDPRFRST_ADDR);
    format = params_format(params);

    switch(format)
    {
    case SNDRV_PCM_FORMAT_U8:
    case SNDRV_PCM_FORMAT_S8:
		samp_bits = 8;
		break;
    case SNDRV_PCM_FORMAT_U16_LE:
    case SNDRV_PCM_FORMAT_S16_LE:
    case SNDRV_PCM_FORMAT_U16_BE:
    case SNDRV_PCM_FORMAT_S16_BE:
		samp_bits = 16;
		break;
    case SNDRV_PCM_FORMAT_U24_LE:
    case SNDRV_PCM_FORMAT_S24_LE:
    case SNDRV_PCM_FORMAT_U24_BE:
    case SNDRV_PCM_FORMAT_S24_BE:
		samp_bits = 32;
		break;
    case SNDRV_PCM_FORMAT_U32_LE:
    case SNDRV_PCM_FORMAT_S32_LE:
    case SNDRV_PCM_FORMAT_U32_BE:
    case SNDRV_PCM_FORMAT_S32_BE:
		samp_bits = 32;
		break;
    default:
		return -EINVAL;
    }
    chan_num = params_channels(params);
    samp_rate = params_rate(params);

#ifdef AUD_SPDIF_ENABLE
    spdif_bclk = samp_rate*((samp_bits*chan_num + 31) & ~31)*2*2;
#else
    spdif_bclk = 0;    
#endif

    if (system_rev == 0x73a2) {
        struct clk_params *params = determine_clk_params(samp_rate, \
                    samp_bits, chan_num, spdif_bclk, &sys_clk);
        if (!params)
            return -EINVAL;

        a = params->a;
        b = params->b;
        c = params->c;
#ifdef AUD_SPDIF_ENABLE
        d = params->d;
#endif
	if(stream_id == SNDRV_PCM_STREAM_CAPTURE){
            tmp = readl(AUDPRFRST_ADDR);
            tmp &= ~0x8; 
            writel(tmp, AUDPRFRST_ADDR);

            snd_soc_update_bits(codec_dai->codec,0x19,0x22,0x22); //SET: WM8960POWER1 AINC MICB
            snd_soc_update_bits(codec_dai->codec,0x9,0x40,0x40);  // SET: AUDIO INTERFACE ALRGPIO

        }else if(stream_id == SNDRV_PCM_STREAM_PLAYBACK){
            tmp = readl(AUDPRFRST_ADDR);
            tmp &= ~0x2; 
            writel(tmp, AUDPRFRST_ADDR);
            snd_soc_dai_set_clkdiv(codec_dai, 1, params->dac_div); //WM8960_DACDIV
        }
        kfree(params);
        //S24_LE, low three bytes, so need to shift
        if(format == SNDRV_PCM_FORMAT_S24_LE)
        {
            padding = 0x19; // 25bits shift
        }
        else
        {
            padding = 0x1;
        }
    }
    else //for wm8524, it's only support 24bit
    {
        a = 1;
        b = 2;
        c = 2;
#ifdef AUD_SPDIF_ENABLE
        d = 0xa;
#endif
        sys_clk = 36864000;
        padding = 0x0;
    }
 
    if(i2s_dev->clk_count == 0)
    writel((a << 16) | (b << 12) | (c << 8)|(padding), DACCFG_ADDR);

#ifdef AUD_SPDIF_ENABLE
    writel(readl(DGOCFG_ADDR) | (d << 0), DGOCFG_ADDR);

    status = 0;
    switch(samp_rate) {
    case 44100:
	    status = (0x0 << 24);
	    break;
    case 48000:
	    status = (0x2 << 24);
	    break;
    case 32000:
	    status = (0x3 << 24);
	    break;
    case 96000:
	    status = (0xa << 24);
        break;
    default:
	    break;
    }

    writel(status, DGOCS0A_ADDR);
    writel(status, DGOCS1A_ADDR);
    writel(0x1, DGOCSRDY_ADDR);
#endif

    if (i2s_dev->clk && i2s_dev->clk_count == 0) {
        if (elite_i2s_clk_set(i2s_dev->clk, sys_clk))
	    return -1;
    }

    writel(0x7f,AUDPRFRST_ADDR);
    return 0;
}

static struct snd_soc_ops elite_snd_ops = {
    .hw_params   = elite_snd_hw_params,
};

static struct snd_soc_dai_link elite_dais_link = {
    .ops         = &elite_snd_ops,
    .cpu_dai_name     = "elite-i2s",
    .platform_name    = "elite-i2s-pcm-audio",
    .name     = "i2s-dac",
    .stream_name = "I2S PCM",
    .codec_name = "dummy-codec-wm",
    .codec_dai_name = "dummy-dai-wm",
};

static struct snd_soc_dai_link elite_dais_link_wm8960 = {
    .ops         = &elite_snd_ops,
    .cpu_dai_name     = "elite-i2s",
    .platform_name    = "elite-i2s-pcm-audio",
    .name     = "i2s-dac",
    .stream_name = "I2S PCM",
    .codec_name = "wm8960.1-001a",
    .codec_dai_name = "wm8960-hifi",
};


static struct snd_soc_card elite_snd_card = {
    .name         = "elite-audio",
    .dai_link     = &elite_dais_link,
    .num_links    = 1,
};


struct platform_device *pcm_dev;
static __devinit int elite_snd_driver_probe(struct platform_device *pdev)
{
    struct snd_soc_card *card = &elite_snd_card;
    int ret;
    if (system_rev == 0x73a2) {
        card->dai_link = &elite_dais_link_wm8960;
    }
	
    if (!pdev->dev.platform_data && !pdev->dev.of_node) {
		dev_err(&pdev->dev, "No platform data supplied\n");
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);

	if (pdev->dev.of_node) {
		ret = snd_soc_of_parse_card_name(card, "s3g,model");
		if (ret)
			return ret;
#if 0
		ret = snd_soc_of_parse_audio_routing(card,
						     "s3g,audio-routing");
		if (ret)
			goto err;
#endif
		elite_dais_link_wm8960.codec_name = NULL;
		elite_dais_link_wm8960.codec_of_node = of_parse_phandle(
				pdev->dev.of_node, "s3g,audio-codec", 0);
		if (!elite_dais_link_wm8960.codec_of_node) {
			dev_err(&pdev->dev,
				"Property 's3g,audio-codec' missing or invalid\n");
			return -EINVAL;
		}

		elite_dais_link_wm8960.cpu_dai_name = NULL;
		elite_dais_link_wm8960.cpu_dai_of_node = of_parse_phandle(
				pdev->dev.of_node, "s3g,i2s-controller", 0);
		if (!elite_dais_link_wm8960.cpu_dai_of_node) {
			dev_err(&pdev->dev,
				"Property 's3g,i2s-controller' missing or invalid\n");
			return -EINVAL;
		}

		pcm_dev = platform_device_register_simple(
					"elite-i2s-pcm-audio", -1, NULL, 0);
		if (IS_ERR(pcm_dev)) {
			dev_err(&pdev->dev,
				"Can't instantiate elite-i2s-pcm-audio\n");
			ret = PTR_ERR(pcm_dev);
			return ret;
		}
	}
	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		return ret;
	}

	return 0;
}

static int __devexit elite_snd_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	if (!IS_ERR(pcm_dev))
		platform_device_unregister(pcm_dev);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id elite_snd_of_match[] __devinitconst = {
	{ .compatible = "s3graphics,elite1000-audio", },
	{},
};
MODULE_DEVICE_TABLE(of, elite_snd_of_match);
#endif

static struct platform_device_id elite_snd_driver_ids[] = {
    {
		.name  = "elite1000-audio",
		.driver_data = (kernel_ulong_t)NULL,
    },
};

static struct platform_driver elite_snd_driver = {
	.driver = {
		.name = "elite-audio",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(elite_snd_of_match),
	},
	.id_table  = elite_snd_driver_ids,
	.probe = elite_snd_driver_probe,
	.remove = __devexit_p(elite_snd_driver_remove),
};
module_platform_driver(elite_snd_driver);

MODULE_AUTHOR("S3 Graphics");
MODULE_DESCRIPTION("Elite1000 machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform: elite1000-snd");

