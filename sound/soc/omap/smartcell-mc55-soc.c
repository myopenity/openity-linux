/*
 * smartcell-soc.c  --  ALSA SoC audio pcm for OMAP3/AM35xx
 *
 * Author: Cliff Brake <cbrake@bec-systems.com
 * Author: Nathan Eggan <nathan@ccc-i.com>
 *
 * Based on sound/soc/omap/overo.c by Steve Sakoman
 *   and am3517evm.c by Anuj Aggarwal <anuj.aggarwal@ti.com>
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



//#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
/*** includes here down, needed??? ***/
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"

static struct platform_device *sc_mc55_snd_device;
static struct platform_device *sc_mc55_codec_device;

static int sc_mc55_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	/* we really do not have a "codec", it's an external device with no control & a fixed configuration */
	//struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

/*** This stanza replaced by the insertion of ".dai_fmt" below; it's a fixed setup? ***/
	unsigned int fmt;
	
	switch (params_channels(params)) {
	case 1: /* voice only - 8kHz, S16_LE, mono */
		fmt = SND_SOC_DAIFMT_RIGHT_J |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		break;
	default:
		return -EINVAL;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

#if 0	/*** stuff preserved for posterity ***/

/*** This driver cannot configure anything regarding the cell modem from the kernel ***/
	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}
	
	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
					    SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}
#endif

/*** these next 2 are valid only for mcbsp1 (0 to the driver) as other mcbsp's lack separate CLKR/FSR lines ***/
	/* set cpu CLKR & FSR as inputs (unused) */
	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_CLKR_SRC_CLKX, 0,
				SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set CPU system clock OMAP_MCBSP_CLKR_SRC_CLKX\n");
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_FSR_SRC_FSX, 0,
				SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set CPU system clock OMAP_MCBSP_FSR_SRC_FSX\n");
		return ret;
	}

#if 0
/*** The SRG is not needed since McBSP is slave! ***/
	/* Set McBSP clock to external */
	// note, final parameter appears ignored.
	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKX_EXT, 
					256 * params_rate(params), SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI clock source: OMAP_MCBSP_SYSCLK_CLKX_EXT\n");  // was 0
		return ret;
	}

	/* Set cpu DAI master clock divisor */
	ret =	snd_soc_dai_set_clkdiv(cpu_dai, OMAP_MCBSP_CLKGDV, 8);  // was 1
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI clock divider: OMAP_MCBSP_CLKGDV\n");
		return ret;
	}

#endif

	return 0;
}

static struct snd_soc_ops sc_mc55_ops = {
	.hw_params = sc_mc55_hw_params,
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link sc_mc55_dai[] = {
	{
		.name = "SmarTcell-MC55_0",
		.stream_name = "SmarTcell-MC55_0",
		.cpu_dai_name = "omap-mcbsp.1",
		.platform_name = "omap-pcm-audio",
		.codec_dai_name = "smartcell-codec-dai",
		.codec_name = "smartcell-codec",
		.ops = &sc_mc55_ops,
	},
	{
		.name = "SmarTcell-MC55_A",
		.stream_name = "SmarTcell-MC55_A",
		.cpu_dai_name = "omap-mcbsp.2",
		.platform_name = "omap-pcm-audio",
		.codec_dai_name = "smartcell-codec-dai",
		.codec_name = "smartcell-codec",
		.ops = &sc_mc55_ops,
	},
	{
		.name = "SmarTcell-MC55_B",
		.stream_name = "SmarTcell-MC55_B",
		.cpu_dai_name = "omap-mcbsp.3",
		.platform_name = "omap-pcm-audio",
		.codec_dai_name = "smartcell-codec-dai",
		.codec_name = "smartcell-codec",
		.ops = &sc_mc55_ops,
	},
	{
		.name = "SmarTcell-MC55_C",
		.stream_name = "SmarTcell-MC55_C",
		.cpu_dai_name = "omap-mcbsp.4",
		.platform_name = "omap-pcm-audio",
		.codec_dai_name = "smartcell-codec-dai",
		.codec_name = "smartcell-codec",
		.ops = &sc_mc55_ops,
	}
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_sc_mc55 = {
	.name = "sc-mc55",
	.owner = THIS_MODULE,
	.dai_link = sc_mc55_dai,
	.num_links = ARRAY_SIZE(sc_mc55_dai),
};

static int __init sc_mc55_soc_init(void)
{
	int ret;
	printk(KERN_DEBUG "entering sc_mc55_soc_init...\n");

	sc_mc55_codec_device = platform_device_alloc("smartcell-codec", -1);
	if (!sc_mc55_codec_device)
		return -ENOMEM;

	ret = platform_device_add(sc_mc55_codec_device);
	if (ret)
		goto err1;

	sc_mc55_snd_device = platform_device_alloc("soc-audio", -1);
	if (!sc_mc55_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		ret = -ENOMEM;
		goto err2;
	}

	platform_set_drvdata(sc_mc55_snd_device, &snd_soc_sc_mc55);

	ret = platform_device_add(sc_mc55_snd_device);
	if (ret)
		goto err3;

	printk(KERN_INFO "SmarTcell/MC55 SoC init\n");
	
	return 0;

err3:
	printk(KERN_DEBUG "platform_device_del & _put (soc)\n");
	platform_device_del(sc_mc55_snd_device);
	platform_device_put(sc_mc55_snd_device);
err2:
	printk(KERN_DEBUG "platform_device_del (codec)\n");
	platform_device_del(sc_mc55_codec_device);
err1:
	printk(KERN_DEBUG "platform_device_put (codec)\n");
	platform_device_put(sc_mc55_codec_device);

	return ret;
}

static void __exit sc_mc55_soc_exit(void)
{
	printk(KERN_DEBUG "sc_mc55_soc_exit\n");
	platform_device_unregister(sc_mc55_snd_device);
	platform_device_unregister(sc_mc55_codec_device);
}

module_init(sc_mc55_soc_init);
module_exit(sc_mc55_soc_exit);

MODULE_AUTHOR("SmarTcell Tech Support <?>@smartcell.com");
MODULE_DESCRIPTION("ALSA SoC - SmarTcell/MC55 DAI");
MODULE_LICENSE("GPL v2");


