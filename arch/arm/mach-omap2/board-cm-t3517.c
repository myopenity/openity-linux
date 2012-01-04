/*
 * board-cm-t3517.c (CompuLab CM-T3517 module)
 *
 * Copyright (C) 2010 CompuLab, Ltd.
 * Author: Igor Grinberg <grinberg@compulab.co.il>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/davinci_emac.h>
#include <linux/rtc-v3020.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/can/platform/ti_hecc.h>

#include <linux/i2c/at24.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>

#include <linux/spi/spi.h>
#include <linux/spi/tdo24m.h>
#include <linux/spi/ads7846.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <mach/am35xx.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/nand.h>
#include <plat/gpmc.h>
#include <plat/usb.h>
#include <plat/display.h>
#include <plat/mcspi.h>

#include "mux.h"
#include "sdram-micron-mt46h32m32lf-6.h"
#include "mmc-am3517evm.h"

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
#include <linux/smsc911x.h>

#define SB_T35_SMSC911X_CS		(4)
#define SB_T35_SMSC911X_IRQ_GPIO	(65)

static struct smsc911x_platform_config cm_t3517_smsc911x_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags		= SMSC911X_USE_16BIT | SMSC911X_SAVE_MAC_ADDRESS,
	.phy_interface	= PHY_INTERFACE_MODE_MII,
};

static struct resource sb_t35_smsc911x_resources[] = {
	{
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= OMAP_GPIO_IRQ(SB_T35_SMSC911X_IRQ_GPIO),
		.end	= OMAP_GPIO_IRQ(SB_T35_SMSC911X_IRQ_GPIO),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct platform_device sb_t35_smsc911x_device = {
	.name		= "smsc911x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(sb_t35_smsc911x_resources),
	.resource	= sb_t35_smsc911x_resources,
	.dev		= {
		.platform_data = &cm_t3517_smsc911x_config,
	},
};

static void __init cm_t3517_init_smsc911x(struct platform_device *dev,
					  int cs, int irq_gpio)
{
	unsigned long cs_mem_base;

	if (gpmc_cs_request(cs, SZ_16M, &cs_mem_base) < 0) {
		pr_err("CM-T3517: Failed request for GPMC mem for smsc911x\n");
		return;
	}

	dev->resource[0].start = cs_mem_base + 0x0;
	dev->resource[0].end   = cs_mem_base + 0xff;

	if ((gpio_request(irq_gpio, "SMSC911X IRQ") == 0) &&
	    (gpio_direction_input(irq_gpio) == 0)) {
		gpio_export(irq_gpio, 0);
	} else {
		pr_err("CM-T3517: could not obtain gpio for SMSC911X IRQ\n");
		return;
	}

	platform_device_register(dev);
}

static void __init cm_t3517_init_ethernet(void)
{
	cm_t3517_init_smsc911x(&sb_t35_smsc911x_device,
			       SB_T35_SMSC911X_CS, SB_T35_SMSC911X_IRQ_GPIO);
}
#else
static inline void __init cm_t3517_init_ethernet(void) { return; }
#endif

#define AM35XX_PHY_MASK		(0xF)
#define AM35XX_MDIO_FREQUENCY	(1000000)

static struct emac_platform_data cm_t3517_emac_pdata = {
	.phy_mask	= AM35XX_PHY_MASK,
	.mdio_max_freq	= AM35XX_MDIO_FREQUENCY,
	.rmii_en	= 1,
};

static struct resource cm_t3517_emac_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE,
		.end    = AM35XX_IPSS_EMAC_BASE + 0x3FFFF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.end    = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device cm_t3517_emac_device = {
	.name		= "davinci_emac",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(cm_t3517_emac_resources),
	.resource	= cm_t3517_emac_resources,
};

static void cm_t3517_enable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		  AM35XX_CPGMAC_C0_TX_PULSE_CLR |
		  AM35XX_CPGMAC_C0_MISC_PULSE_CLR |
		  AM35XX_CPGMAC_C0_RX_THRESH_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void cm_t3517_disable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		  AM35XX_CPGMAC_C0_TX_PULSE_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

void cm_t3517_emac_init(struct emac_platform_data *pdata)
{
	unsigned int regval;

	pdata->ctrl_reg_offset		= AM35XX_EMAC_CNTRL_OFFSET;
	pdata->ctrl_mod_reg_offset	= AM35XX_EMAC_CNTRL_MOD_OFFSET;
	pdata->ctrl_ram_offset		= AM35XX_EMAC_CNTRL_RAM_OFFSET;
	pdata->mdio_reg_offset		= AM35XX_EMAC_MDIO_OFFSET;
	pdata->ctrl_ram_size		= AM35XX_EMAC_CNTRL_RAM_SIZE;
	pdata->version			= EMAC_VERSION_2;
	pdata->hw_ram_addr		= AM35XX_EMAC_HW_RAM_ADDR;
	pdata->interrupt_enable		= cm_t3517_enable_ethernet_int;
	pdata->interrupt_disable	= cm_t3517_disable_ethernet_int;
	cm_t3517_emac_device.dev.platform_data	= pdata;
	platform_device_register(&cm_t3517_emac_device);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	regval = regval & (~(AM35XX_CPGMACSS_SW_RST));
	omap_ctrl_writel(regval, AM35XX_CONTROL_IP_SW_RESET);
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
}

#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
#include <linux/leds.h>

static struct gpio_led cm_t3517_leds[] = {
	[0] = {
		.gpio			= 186,
		.name			= "cm-t3517:green",
		.default_trigger	= "heartbeat",
		.active_low		= 0,
	},
};

static struct gpio_led_platform_data cm_t3517_led_pdata = {
	.num_leds	= ARRAY_SIZE(cm_t3517_leds),
	.leds		= cm_t3517_leds,
};

static struct platform_device cm_t3517_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &cm_t3517_led_pdata,
	},
};

static void __init cm_t3517_init_led(void)
{
	platform_device_register(&cm_t3517_led_device);
}
#else
static inline void cm_t3517_init_led(void) {}
#endif

#if defined(CONFIG_CAN_TI_HECC) || defined(CONFIG_CAN_TI_HECC_MODULE)
static struct resource cm_t3517_hecc_resources[] = {
	{
		.start	= AM35XX_IPSS_HECC_BASE,
		.end	= AM35XX_IPSS_HECC_BASE + 0x3FFF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_35XX_HECC0_IRQ,
		.end	= INT_35XX_HECC0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct ti_hecc_platform_data cm_t3517_hecc_pdata = {
	.scc_hecc_offset	= AM35XX_HECC_SCC_HECC_OFFSET,
	.scc_ram_offset		= AM35XX_HECC_SCC_RAM_OFFSET,
	.hecc_ram_offset	= AM35XX_HECC_RAM_OFFSET,
	.mbx_offset		= AM35XX_HECC_MBOX_OFFSET,
	.int_line		= AM35XX_HECC_INT_LINE,
	.version		= AM35XX_HECC_VERSION,
};

static struct platform_device cm_t3517_hecc_device = {
	.name		= "ti_hecc",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(cm_t3517_hecc_resources),
	.resource	= cm_t3517_hecc_resources,
	.dev		= {
		.platform_data = &cm_t3517_hecc_pdata,
	},
};

static void cm_t3517_hecc_init(void)
{
	platform_device_register(&cm_t3517_hecc_device);
}
#else
static void cm_t3517_hecc_init(void) {}
#endif

#if defined(CONFIG_MTD_NAND_OMAP2) || defined(CONFIG_MTD_NAND_OMAP2_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#define NAND_BLOCK_SIZE		SZ_128K
#define GPMC_CS0_BASE		0x60
#define GPMC_CS0_BASE_ADDR	(OMAP34XX_GPMC_VIRT + GPMC_CS0_BASE)

static struct mtd_partition cm_t3517_nand_partitions[] = {
	{
		.name           = "xloader",
		.offset         = 0,			/* Offset = 0x00000 */
		.size           = 4 * NAND_BLOCK_SIZE,
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "uboot",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size           = 15 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "uboot environment",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size           = 2 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "linux",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size           = 32 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "rootfs",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size           = MTDPART_SIZ_FULL,
	},
};

static struct omap_nand_platform_data cm_t3517_nand_data = {
	.parts			= cm_t3517_nand_partitions,
	.nr_parts		= ARRAY_SIZE(cm_t3517_nand_partitions),
	.dma_channel		= -1,	/* disable DMA in OMAP NAND driver */
	.cs			= 0,
	.gpmc_cs_baseaddr	= (void __iomem *) GPMC_CS0_BASE_ADDR,
	.gpmc_baseaddr		= (void __iomem *) OMAP34XX_GPMC_VIRT,

};

static struct resource cm_t3517_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device cm_t3517_nand_device = {
	.name		= "omap2-nand",
	.id		= -1,
	.num_resources	= 1,
	.resource	= &cm_t3517_nand_resource,
	.dev		= {
		.platform_data	= &cm_t3517_nand_data,
	},
};

static void __init cm_t3517_init_nand(void)
{
	if (platform_device_register(&cm_t3517_nand_device) < 0)
		pr_err("CM-T3517: Unable to register NAND device\n");
}
#else
static inline void cm_t3517_init_nand(void) {}
#endif

#if defined(CONFIG_TOUCHSCREEN_ADS7846) || \
	defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
#define CM_T3517_GPIO_PENDOWN		(57)

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(CM_T3517_GPIO_PENDOWN);
}

static struct ads7846_platform_data ads7846_config = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
	.x_plate_ohms		= 180,
	.pressure_max		= 255,
	.debounce_max		= 10,
	.debounce_tol		= 3,
	.debounce_rep		= 1,
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
	.irq_flags		= IRQF_TRIGGER_FALLING,
};

static struct spi_board_info cm_t3517_spi_board_info[] __initdata = {
	{
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &ads7846_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(CM_T3517_GPIO_PENDOWN),
		.platform_data		= &ads7846_config,
	},
};

static void __init cm_t3517_init_ads7846(void)
{
	if ((gpio_request(CM_T3517_GPIO_PENDOWN, "ADS7846_PENDOWN") == 0) &&
	    (gpio_direction_input(CM_T3517_GPIO_PENDOWN) == 0)) {
		gpio_export(CM_T3517_GPIO_PENDOWN, 0);
	} else {
		pr_err("CM-T3517: could not obtain gpio for ADS7846_PENDOWN\n");
		return;
	}

	spi_register_board_info(cm_t3517_spi_board_info,
				ARRAY_SIZE(cm_t3517_spi_board_info));
}
#else
static inline void cm_t3517_init_ads7846(void) {}
#endif

#if defined(CONFIG_OMAP2_DSS) || defined(CONFIG_OMAP2_DSS_MODULE)
#define SB_T35_LCD_EN_GPIO 157
#define SB_T35_LCD_BL_GPIO 58
#define SB_T35_DVI_EN_GPIO 54

static int lcd_enabled;
static int dvi_enabled;

static int cm_t3517_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	if (dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}

	gpio_set_value(SB_T35_LCD_EN_GPIO, 1);
	gpio_set_value(SB_T35_LCD_BL_GPIO, 1);

	lcd_enabled = 1;

	return 0;
}

static void cm_t3517_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	lcd_enabled = 0;

	gpio_set_value(SB_T35_LCD_BL_GPIO, 0);
	gpio_set_value(SB_T35_LCD_EN_GPIO, 0);
}

static int cm_t3517_panel_enable_dvi(struct omap_dss_device *dssdev)
{
	if (lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}

	gpio_set_value(SB_T35_DVI_EN_GPIO, 0);
	dvi_enabled = 1;

	return 0;
}

static void cm_t3517_panel_disable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(SB_T35_DVI_EN_GPIO, 1);
	dvi_enabled = 0;
}

static int cm_t3517_panel_enable_tv(struct omap_dss_device *dssdev)
{
	return 0;
}

static void cm_t3517_panel_disable_tv(struct omap_dss_device *dssdev)
{
}

static struct omap_dss_device cm_t3517_lcd_device = {
	.name			= "lcd",
	.driver_name		= "toppoly_tdo35s_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 18,
	.platform_enable	= cm_t3517_panel_enable_lcd,
	.platform_disable	= cm_t3517_panel_disable_lcd,
};

static struct omap_dss_device cm_t3517_dvi_device = {
	.name			= "dvi",
	.driver_name		= "generic_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.panel.config		= OMAP_DSS_LCD_IPC | OMAP_DSS_LCD_ONOFF,
	.platform_enable	= cm_t3517_panel_enable_dvi,
	.platform_disable	= cm_t3517_panel_disable_dvi,
};

static struct omap_dss_device cm_t3517_tv_device = {
	.name			= "tv",
	.driver_name		= "venc",
	.type			= OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_SVIDEO,
	.platform_enable	= cm_t3517_panel_enable_tv,
	.platform_disable	= cm_t3517_panel_disable_tv,
};

static struct omap_dss_device *cm_t3517_dss_devices[] = {
	&cm_t3517_lcd_device,
	&cm_t3517_dvi_device,
	&cm_t3517_tv_device,
};

static struct omap_dss_board_info cm_t3517_dss_data = {
	.num_devices	= ARRAY_SIZE(cm_t3517_dss_devices),
	.devices	= cm_t3517_dss_devices,
	.default_device	= &cm_t3517_dvi_device,
};

static struct platform_device cm_t3517_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data = &cm_t3517_dss_data,
	},
};

static struct omap2_mcspi_device_config tdo24m_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct tdo24m_platform_data tdo24m_config = {
	.model = TDO35S,
};

static struct spi_board_info cm_t3517_lcd_spi_board_info[] __initdata = {
	{
		.modalias		= "tdo24m",
		.bus_num		= 4,
		.chip_select		= 0,
		.max_speed_hz		= 1000000,
		.controller_data	= &tdo24m_mcspi_config,
		.platform_data		= &tdo24m_config,
	},
};

static void __init cm_t3517_init_display(void)
{
	int err;

	spi_register_board_info(cm_t3517_lcd_spi_board_info,
				ARRAY_SIZE(cm_t3517_lcd_spi_board_info));

	err = gpio_request(SB_T35_LCD_EN_GPIO, "LCD RST");
	if (err) {
		pr_err("CM-T3517: failed to get LCD reset GPIO\n");
		goto out;
	}

	err = gpio_request(SB_T35_LCD_BL_GPIO, "LCD BL");
	if (err) {
		pr_err("CM-T3517: failed to get LCD backlight control GPIO\n");
		goto err_lcd_bl;
	}

	err = gpio_request(SB_T35_DVI_EN_GPIO, "DVI EN");
	if (err) {
		pr_err("CM-T3517: failed to get DVI reset GPIO\n");
		goto err_dvi_en;
	}

	gpio_export(SB_T35_LCD_EN_GPIO, 0);
	gpio_export(SB_T35_LCD_BL_GPIO, 0);
	gpio_export(SB_T35_DVI_EN_GPIO, 0);
	gpio_direction_output(SB_T35_LCD_EN_GPIO, 0);
	gpio_direction_output(SB_T35_LCD_BL_GPIO, 0);
	gpio_direction_output(SB_T35_DVI_EN_GPIO, 1);

	msleep(50);
	gpio_set_value(SB_T35_LCD_EN_GPIO, 1);

	err = platform_device_register(&cm_t3517_dss_device);
	if (err) {
		pr_err("CM-T3517: failed to register DSS device\n");
		goto err_dev_reg;
	}

	return;

err_dev_reg:
	gpio_free(SB_T35_DVI_EN_GPIO);
err_dvi_en:
	gpio_free(SB_T35_LCD_BL_GPIO);
err_lcd_bl:
	gpio_free(SB_T35_LCD_EN_GPIO);
out:

	return;
}
#else
static inline void cm_t3517_init_display(void) {}
#endif /* CONFIG_OMAP2_DSS */

#if defined(CONFIG_RTC_DRV_V3020) || defined(CONFIG_RTC_DRV_V3020_MODULE)
#define RTC_IO_GPIO		(153)
#define RTC_WR_GPIO		(154)
#define RTC_RD_GPIO		(53)
#define RTC_CS_GPIO		(163)
#define RTC_CS_EN_GPIO		(160)

struct v3020_platform_data cm_t3517_v3020_pdata = {
	.use_gpio	= 1,
	.gpio_cs	= RTC_CS_GPIO,
	.gpio_wr	= RTC_WR_GPIO,
	.gpio_rd	= RTC_RD_GPIO,
	.gpio_io	= RTC_IO_GPIO,
};

static struct platform_device cm_t3517_rtc_device = {
	.name		= "v3020",
	.id		= -1,
	.dev		= {
		.platform_data = &cm_t3517_v3020_pdata,
	}
};

static void __init cm_t3517_init_rtc(void)
{
	int err;

	err = gpio_request(RTC_CS_EN_GPIO, "RTC CS EN");
	if (err) {
		pr_err("CM-T3517: failed to get RTC CS EN GPIO\n");
		return;
	}

	gpio_direction_output(RTC_CS_EN_GPIO, 1);
	platform_device_register(&cm_t3517_rtc_device);
}
#else
static inline void cm_t3517_init_rtc(void) {}
#endif

#if defined(CONFIG_MMC) || defined(CONFIG_MMC_MODULE)
static struct am3517_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 4,
		.gpio_cd	= 144,
		.gpio_wp	= 59,

	},
	{
		.mmc		= 2,
		.wires		= 4,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{}	/* Terminator */
};

static void __init cm_t3517_mmc_init(void)
{
	/* MMC init function */
	am3517_mmc_init(mmc);
}
#else
static void __init cm_t3517_mmc_init(void) {}
#endif

#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
#define HSUSB1_RESET_GPIO       (146)
#define HSUSB2_RESET_GPIO       (147)
#define USB_HUB_RESET_GPIO      (152)
#define SB_T35_USB_HUB_RST_GPIO	(98)

static struct ehci_hcd_omap_platform_data cm_t3517_ehci_pdata __initdata = {
	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0] = HSUSB1_RESET_GPIO,
	.reset_gpio_port[1] = HSUSB2_RESET_GPIO,
	.reset_gpio_port[2] = -EINVAL,
};

static int cm_t3517_init_usb(void)
{
	int err;

	err = gpio_request(USB_HUB_RESET_GPIO, "usb hub rst");
	if (err) {
		pr_err("CM-T3517: usb hub rst gpio request failed: %d\n", err);
	} else {
		gpio_direction_output(USB_HUB_RESET_GPIO, 0);
		udelay(10);
		gpio_set_value(USB_HUB_RESET_GPIO, 1);
		msleep(1);
	}

	err = gpio_request(SB_T35_USB_HUB_RST_GPIO, "sb-t35 usb hub rst");
	if (err) {
		pr_err("SB-T35: usb hub rst gpio request failed: %d\n", err);
	} else {
		gpio_direction_output(SB_T35_USB_HUB_RST_GPIO, 0);
		udelay(10);
		gpio_set_value(SB_T35_USB_HUB_RST_GPIO, 1);
		msleep(1);
	}

	usb_ehci_init(&cm_t3517_ehci_pdata);

	return 0;
}
#else
static inline int cm_t3517_init_usb(void)
{
	return 0;
}
#endif

static struct i2c_board_info __initdata cm_t3517_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("tlv320aic23", 0x1a),
	},
};

static void __init cm_t3517_init_i2c(void)
{
	omap_register_i2c_bus(1, 400, cm_t3517_i2c1_boardinfo,
			      ARRAY_SIZE(cm_t3517_i2c1_boardinfo));
}

#define CM_T3517_WLAN_RST_GPIO		145

static void __init cm_t3517_init_wifi(void)
{
	int err;

	err = gpio_request(CM_T3517_WLAN_RST_GPIO, "WLAN RST");
	if (err) {
		pr_err("CM-T3517: failed to request wlan rst gpio: %d\n", err);
		return;
	}

	gpio_export(CM_T3517_WLAN_RST_GPIO, 1);
	gpio_direction_output(CM_T3517_WLAN_RST_GPIO, 0);
	msleep(10);
	gpio_set_value(CM_T3517_WLAN_RST_GPIO, 1);
	msleep(10);
}

static struct omap_board_config_kernel cm_t3517_config[] __initdata = {
};

static void __init cm_t3517_init_irq(void)
{
	omap_board_config = cm_t3517_config;
	omap_board_config_size = ARRAY_SIZE(cm_t3517_config);

	omap2_init_common_hw(NULL, NULL, NULL, NULL, NULL);
	omap_init_irq();
	omap_gpio_init();
}

static void __init cm_t3517_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

static struct omap_board_mux board_mux[] __initdata = {
	/* RTC GPIOs */
	/* IO - GPIO153 */
	OMAP3_MUX(MCBSP4_DR, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	/* WR# - GPIO154 */
	OMAP3_MUX(MCBSP4_DX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	/* RD# - GPIO53 */
	OMAP3_MUX(GPMC_NCS2, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	/* CS# EN - GPIO160 */
	OMAP3_MUX(MCBSP_CLKS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	/* CS# - GPIO163 */
	OMAP3_MUX(UART3_CTS_RCTX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	
	/* nCS, IRQ and nRESET for SB-T35 ethernet */
	OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE0),
	OMAP3_MUX(GPMC_WAIT3, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(UART3_RTS_SD, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* GPIO186 is a Green LED */
	OMAP3_MUX(SYS_CLKOUT2, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* PENDOWN GPIO */
	OMAP3_MUX(GPMC_NCS6, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),

	/* HSUSB1 RESET */
	OMAP3_MUX(UART2_TX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	/* HSUSB2 RESET */
	OMAP3_MUX(UART2_RX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	/* CM-T3517 USB HUB nRESET */
	OMAP3_MUX(MCBSP4_CLKX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	/* SB-T35 USB HUB nRESET */
	OMAP3_MUX(SAD2D_MCAD4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),

	/* MMC 1 */
	/* CD */
	OMAP3_MUX(UART2_CTS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	/* WP */
	OMAP3_MUX(GPMC_CLK, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	/* MMC1 PWR EN */
	OMAP3_MUX(GPMC_WAIT2, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),

	/* WLAN nRESET */
	OMAP3_MUX(UART2_RTS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),

	/* McSPI 1 */
	OMAP3_MUX(MCSPI1_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_SIMO, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_SOMI, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_CS0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),

	/* McSPI 4 */
	OMAP3_MUX(MCBSP1_CLKR, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP1_DX, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP1_DR, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP1_FSX, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),

	/* McBSP 2 */
	OMAP3_MUX(MCBSP2_FSX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_CLKX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_DR, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_DX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_NBE1, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* UART1 */
	OMAP3_MUX(UART1_TX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART1_RX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* UART2 */
	OMAP3_MUX(MCBSP3_CLKX, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(MCBSP3_FSX, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),

	/* DSS */
	OMAP3_MUX(DSS_PCLK, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_HSYNC, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_VSYNC, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_ACBIAS, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA0, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA1, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA2, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA3, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA4, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA5, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA6, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA7, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA8, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA9, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA10, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA11, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA12, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA13, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA14, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA15, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA16, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA17, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA18, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA19, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA20, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA21, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA22, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA23, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),

	/* display controls */
	OMAP3_MUX(MCBSP1_FSR, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_NCS7, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_NCS3, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

static void __init cm_t3517_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CUS);
	omap_serial_init();
	cm_t3517_init_i2c();
	cm_t3517_init_nand();
	cm_t3517_init_ads7846();
	cm_t3517_init_led();
	cm_t3517_init_display();

	cm_t3517_init_wifi();
	cm_t3517_mmc_init();

	/*Ethernet*/
	cm_t3517_emac_init(&cm_t3517_emac_pdata);
	/* SB-T35 Ethernet */
	cm_t3517_init_ethernet();

	cm_t3517_init_usb();
	usb_musb_init();

	cm_t3517_hecc_init();
	cm_t3517_init_rtc();
}

MACHINE_START(CM_T3517, "Compulab CM-T3517")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= cm_t3517_map_io,
	.init_irq	= cm_t3517_init_irq,
	.init_machine	= cm_t3517_init,
	.timer		= &omap_timer,
MACHINE_END
