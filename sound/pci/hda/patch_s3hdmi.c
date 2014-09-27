/*
 * 
 *universal Interface for Intel High Definition Audio Codec
 *
 * HD audio interface patch for S3 HDMI codecs
 *
 * Copyright (c) 2006 S3 Technologies Inc.
 *
 *
 *  This driver is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This driver is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <sound/core.h>
#include "hda_codec.h"
#include "hda_local.h"
#include <linux/module.h>

struct s3hdmi_spec {
	struct hda_multi_out multiout;

	struct hda_pcm pcm_rec;
};

#define CVT_NID		0x02	/* audio converter */
#define PIN_NID		0x03	/* HDMI output pin */

static struct hda_verb s3hdmi_basic_init[] = {
	/* enable digital output on pin widget */
	{ 0x03, AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT },
	{} /* terminator */
};

/*
 * Controls
 */
static int s3hdmi_build_controls(struct hda_codec *codec)
{
	struct s3hdmi_spec *spec = codec->spec;
	int err;

	err = snd_hda_create_spdif_out_ctls(codec, 0, spec->multiout.dig_out_nid);
	if (err < 0)
		return err;

	return 0;
}

static int s3hdmi_init(struct hda_codec *codec)
{
	snd_hda_sequence_write(codec, s3hdmi_basic_init);
	/* SI codec requires to unmute the pin */
	if (get_wcaps(codec, PIN_NID) & AC_WCAP_OUT_AMP)
		snd_hda_codec_write(codec, PIN_NID, 0,
				    AC_VERB_SET_AMP_GAIN_MUTE,
				    AMP_OUT_UNMUTE);
	return 0;
}

/*
 * Digital out
 */
static int s3hdmi_dig_playback_pcm_open(struct hda_pcm_stream *hinfo,
				     struct hda_codec *codec,
				     struct snd_pcm_substream *substream)
{
	struct s3hdmi_spec *spec = codec->spec;
	return snd_hda_multi_out_dig_open(codec, &spec->multiout);
}

static int s3hdmi_dig_playback_pcm_close(struct hda_pcm_stream *hinfo,
				      struct hda_codec *codec,
				      struct snd_pcm_substream *substream)
{
	struct s3hdmi_spec *spec = codec->spec;
	return snd_hda_multi_out_dig_close(codec, &spec->multiout);
}

static int s3hdmi_dig_playback_pcm_prepare(struct hda_pcm_stream *hinfo,
					    struct hda_codec *codec,
					    unsigned int stream_tag,
					    unsigned int format,
					    struct snd_pcm_substream *substream)
{
	struct s3hdmi_spec *spec = codec->spec;
	int chans = substream->runtime->channels;
	int i, err;

	err = snd_hda_multi_out_dig_prepare(codec, &spec->multiout, stream_tag,
					    format, substream);
	if (err < 0)
		return err;
	snd_hda_codec_write(codec, CVT_NID, 0, AC_VERB_SET_CVT_CHAN_COUNT,
			    chans - 1);
	/* FIXME: XXX */
	for (i = 0; i < chans; i++) {
		snd_hda_codec_write(codec, CVT_NID, 0,
				    AC_VERB_SET_HDMI_CHAN_SLOT,
				    (i << 4) | i);
	}
	return 0;
}

static struct hda_pcm_stream s3hdmi_pcm_digital_playback = {
	.substreams = 1,
	.channels_min = 2,
	.channels_max = 2,
	.nid = CVT_NID, /* NID to query formats and rates and setup streams */
	.ops = {
		.open = s3hdmi_dig_playback_pcm_open,
		.close = s3hdmi_dig_playback_pcm_close,
		.prepare = s3hdmi_dig_playback_pcm_prepare
	},
};

static int s3hdmi_build_pcms(struct hda_codec *codec)
{
	struct s3hdmi_spec *spec = codec->spec;
	struct hda_pcm *info = &spec->pcm_rec;
	unsigned int chans;
	static int s3_codec = 0;

	codec->num_pcms = 1;
	codec->pcm_info = info;

	if(s3_codec == 0)
	{
	    info->name = "S3 HDMI 1";
	    s3_codec++;
	}
	else
	{
	    info->name = "S3 HDMI 2";
	}
	info->pcm_type = HDA_PCM_TYPE_AUDIO;
	info->stream[SNDRV_PCM_STREAM_PLAYBACK] = s3hdmi_pcm_digital_playback;

	/* FIXME: we must check ELD and change the PCM parameters dynamically
	 */
	chans = get_wcaps(codec, CVT_NID);
	chans = (chans & AC_WCAP_CHAN_CNT_EXT) >> 13;
	chans = ((chans << 1) | 1) + 1;
	info->stream[SNDRV_PCM_STREAM_PLAYBACK].channels_max = chans;

	return 0;
}

static void s3hdmi_free(struct hda_codec *codec)
{
	kfree(codec->spec);
}

static struct hda_codec_ops s3hdmi_patch_ops = {
	.build_controls = s3hdmi_build_controls,
	.build_pcms = s3hdmi_build_pcms,
	.init = s3hdmi_init,
	.free = s3hdmi_free,
};

static int patch_s3hdmi(struct hda_codec *codec)
{
	struct s3hdmi_spec *spec;


	spec = kzalloc(sizeof(*spec), GFP_KERNEL);
	if (spec == NULL)
		return -ENOMEM;

	codec->spec = spec;

	spec->multiout.num_dacs = 0;	  /* no analog */
	spec->multiout.max_channels = 2;
	/* NID for copying analog to digital,
	 * seems to be unused in pure-digital
	 * case.
	 */
	spec->multiout.dig_out_nid = CVT_NID;

	codec->patch_ops = s3hdmi_patch_ops;

	return 0;
}

/*
 * patch entries
 */
struct hda_codec_preset snd_hda_preset_s3hdmi[] = {
	{ .id = 0x11069f84, .name = "S3 Elite1000 HDMI", .patch = patch_s3hdmi }, 
	{ .id = 0x53331111, .name = "S3 CHROME4XX HDMI", .patch = patch_s3hdmi },
	{ .id = 0x53331112, .name = "S3 CHROME5XX HDMI",  .patch = patch_s3hdmi },
	{ .id = 0x53339f82, .name = "S3 419 HDMI",  .patch = patch_s3hdmi },
	{} /* terminator */
};

MODULE_ALIAS("snd-hda-codec-id:53331111");
MODULE_ALIAS("snd-hda-codec-id:53331112");
MODULE_ALIAS("snd-hda-codec-id:53339f82");


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("S3 HDMI HD-audio codec");

static struct hda_codec_preset_list s3hdmi_list = {
	.preset = snd_hda_preset_s3hdmi,
	.owner = THIS_MODULE,
};

static int __init patch_s3hdmi_init(void)
{
	return snd_hda_add_codec_preset(&s3hdmi_list);
}

static void __exit patch_s3hdmi_exit(void)
{
	snd_hda_delete_codec_preset(&s3hdmi_list);
}

module_init(patch_s3hdmi_init)
module_exit(patch_s3hdmi_exit)
