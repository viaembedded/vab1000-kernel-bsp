#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/regulator/consumer.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>


#define DUMMY_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		      SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
		      SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)

#define DUMMY_PCM_FORMATS \
	(SNDRV_PCM_FORMAT_S16_LE | SNDRV_PCM_FORMAT_S20_3LE | \
	 SNDRV_PCM_FORMAT_S24_LE)

static struct snd_soc_dai_driver dummy_dai = {
	.name = "dummy-dai-wm",
	.playback = {
		.stream_name = "dummy Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = DUMMY_RATES,
		.formats = DUMMY_PCM_FORMATS,
	},
};

static struct snd_soc_codec_driver soc_codec_dev_dummy; 

static int __devinit dummy_codec_probe(struct platform_device *pdev)
{
	int ret;
	ret =  snd_soc_register_codec(&pdev->dev,
			&soc_codec_dev_dummy, &dummy_dai, 1);
	if (ret < 0)
		printk("register dummy codec dai failed!\n");
	return ret;
}
static int __devexit dummy_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver dummy_codec_driver = {
	.driver	= {
		.name	= "dummy-codec-wm",
		.owner	= THIS_MODULE,
	},
	.probe	= dummy_codec_probe,
	.remove	= __devexit_p(dummy_codec_remove),
};

module_platform_driver(dummy_codec_driver);
MODULE_DESCRIPTION("ASoC Dummy codec driver");
MODULE_AUTHOR("S3 Graphics");
MODULE_LICENSE("GPL");
