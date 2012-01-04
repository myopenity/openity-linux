/*
 * cm-t3517.c  -- ALSA SoC support for CM-T3517
 *
 * Copyright 2010 CompuLab, Ltd.
 * Author Mike Rapoport <mike@compulab.co.il>
 *
 * Based on am3517evm.c 
 * Author: Anuj Aggarwal <anuj.aggarwal@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"

#include "../codecs/tlv320aic23.h"

#define CODEC_CLOCK 	12000000

static int cm_t3517_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_DSP_B |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_DSP_B |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
			CODEC_CLOCK, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops cm_t3517_ops = {
	.hw_params = cm_t3517_hw_params,
};

/* cm_t3517 machine dapm widgets */
static const struct snd_soc_dapm_widget tlv320aic23_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"Headphone Jack", NULL, "LHPOUT"},
	{"Headphone Jack", NULL, "RHPOUT"},

	{"LLINEIN", NULL, "Line In"},
	{"RLINEIN", NULL, "Line In"},

	{"MICIN", NULL, "Mic Jack"},
};

static int cm_t3517_aic23_init(struct snd_soc_codec *codec)
{
	/* Add am3517-evm specific widgets */
	snd_soc_dapm_new_controls(codec, tlv320aic23_dapm_widgets,
				  ARRAY_SIZE(tlv320aic23_dapm_widgets));

	/* Set up davinci-evm specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* always connected */
	snd_soc_dapm_enable_pin(codec, "Line Out");
	snd_soc_dapm_enable_pin(codec, "Line In");
	snd_soc_dapm_enable_pin(codec, "Mic In");

	snd_soc_dapm_sync(codec);

	return 0;
}

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link cm_t3517_dai = {
	.name = "TLV320AIC23",
	.stream_name = "AIC23",
	.cpu_dai = &omap_mcbsp_dai[0],
	.codec_dai = &tlv320aic23_dai,
	.init = cm_t3517_aic23_init,
	.ops = &cm_t3517_ops,
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_cm_t3517 = {
	.name = "cm_t3517",
	.platform = &omap_soc_platform,
	.dai_link = &cm_t3517_dai,
	.num_links = 1,
};

/* Audio subsystem */
static struct snd_soc_device cm_t3517_snd_devdata = {
	.card = &snd_soc_cm_t3517,
	.codec_dev = &soc_codec_dev_tlv320aic23,
};

static struct platform_device *cm_t3517_snd_device;

static int __init cm_t3517_soc_init(void)
{
	int ret;

	if (!machine_is_cm_t3517()) {
		pr_err("Not CM-T3517\n");
		return -ENODEV;
	}

	cm_t3517_snd_device = platform_device_alloc("soc-audio", -1);
	if (!cm_t3517_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(cm_t3517_snd_device, &cm_t3517_snd_devdata);
	cm_t3517_snd_devdata.dev = &cm_t3517_snd_device->dev;
	*(unsigned int *)cm_t3517_dai.cpu_dai->private_data = 1; /* McBSP2 */

	ret = platform_device_add(cm_t3517_snd_device);
	if (ret)
		goto err1;

	/* enable audio amplifier on SB-T35 */
	ret = gpio_request(61, "AMP SHDN");
	if (ret)
		goto err1;

	gpio_direction_output(61, 0);
	gpio_export(61, 0);

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(cm_t3517_snd_device);

	return ret;
}

static void __exit cm_t3517_soc_exit(void)
{
	gpio_free(61);
	platform_device_unregister(cm_t3517_snd_device);
}

module_init(cm_t3517_soc_init);
module_exit(cm_t3517_soc_exit);

MODULE_AUTHOR("Mike Rapoport <mike@compulab.co.il");
MODULE_DESCRIPTION("ALSA SoC CM-T3517");
MODULE_LICENSE("GPL v2");
