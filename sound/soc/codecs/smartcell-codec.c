/*
 * smartcell-codec.c -- SOC codec driver for modem
 *
 * Author: Cliff Brake <cbrake@bec-systems.com
 * Author: Nathan Eggan <nathan@ccc-i.com>
 *
 * based on spdif_transciever.c by Steve Chen <schen@mvista.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#define DRV_NAME "smartcell-codec"


/***********************************************************************
 * This device really does not need a CODEC. There definitely is no "encoding"
 * or "decoding" being done here. All that is desired is a passthrough for the raw
 * PCM. Likewise, there is no interconnect to the cell modem (i2c, SPI, etc) 
 * and it is not configurable. This file is really just a stub.
 ***********************************************************************/

static struct snd_soc_codec_driver soc_codec_smartcell_codec = {
};

static struct snd_soc_dai_driver mc55_dai = {
	.name		= "smartcell-codec-dai",
	.playback 	= {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
//		.sig_bits = 16,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
//		.sig_bits = 16,
	},
};

static int __devinit smartcell_codec_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_smartcell_codec,
			&mc55_dai, 1);
}

static int smartcell_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

MODULE_ALIAS("platform:" DRV_NAME);

static struct platform_driver smartcell_codec_driver = {
	.probe		= smartcell_codec_probe,
	.remove		= __devexit_p(smartcell_codec_remove),
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};


module_platform_driver(smartcell_codec_driver);


MODULE_AUTHOR("SmarTcell Tech Support <?>@smartcell.com");
MODULE_DESCRIPTION("SmarTcell Cellular Digital 'CODEC' Driver");
MODULE_LICENSE("GPL");
