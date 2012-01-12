/*
 * linux/arch/arm/mach-omap2/board-cm-t3517.c
 *
 * Support for the CompuLab CM-T3517 modules
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
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/rtc-v3020.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/can/platform/ti_hecc.h>
#include <linux/mmc/host.h>
#include <linux/davinci_emac.h>

#include <mach/hardware.h>
#include <mach/am35xx.h>

#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/nand.h>
#include <plat/gpmc.h>


#include "mux.h"
#include "control.h"
#include "common-board-devices.h"
#include "hsmmc.h"

/*******************************************************
 * SMSC 911X Ethernet
 *******************************************************/

#if 0 && ( defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE) )
#include <linux/smsc911x.h>

#define CM_T35_SMSC911X_CS	5
#define CM_T35_SMSC911X_GPIO	163
#define SB_T35_SMSC911X_CS		4
#define SB_T35_SMSC911X_GPIO	65
#define SB_T35_SMSC911X_RESET_GPIO	164

static struct smsc911x_platform_config cm_t3517_smsc911x_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags		= SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS,
	.phy_interface	= PHY_INTERFACE_MODE_MII,
};

static struct resource cm_t3517_smsc911x_resources[] = {
	{
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= OMAP_GPIO_IRQ(CM_T35_SMSC911X_GPIO),
		.end	= OMAP_GPIO_IRQ(CM_T35_SMSC911X_GPIO),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct platform_device cm_t3517_smsc911x_device = {
	.name		= "smsc911x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(cm_t3517_smsc911x_resources),
	.resource	= cm_t3517_smsc911x_resources,
	.dev		= {
		.platform_data = &cm_t3517_smsc911x_config,
	},
};

static struct resource sb_t35_smsc911x_resources[] = {
	{
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= OMAP_GPIO_IRQ(SB_T35_SMSC911X_GPIO),
		.end	= OMAP_GPIO_IRQ(SB_T35_SMSC911X_GPIO),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct platform_device sb_t35_smsc911x_device = {
	.name		= "smsc911x",
	.id		= 1,
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
		printk(KERN_ERR "Failed request for GPMC mem for smsc911x\n");
		return;
	}

	dev->resource[0].start = cs_mem_base + 0x0;
	dev->resource[0].end   = cs_mem_base + 0xff;

	if ((gpio_request(irq_gpio, "SMSC911X IRQ") == 0) &&
	    (gpio_direction_input(irq_gpio) == 0)) {
		gpio_export(irq_gpio, 0);
	} else {
		printk(KERN_ERR "could not obtain gpio for SMSC911X IRQ\n");
		return;
	}

	platform_device_register(dev);
}

static void __init cm_t3517_init_ethernet(void)
{
	cm_t3517_init_smsc911x(&cm_t3517_smsc911x_device,
			     CM_T35_SMSC911X_CS, CM_T35_SMSC911X_GPIO);
	cm_t3517_init_smsc911x(&sb_t35_smsc911x_device,
			     SB_T35_SMSC911X_CS, SB_T35_SMSC911X_GPIO);
}
#else
static inline void __init cm_t3517_init_ethernet(void) { return; }
#endif

/*******************************************************
 * DaVinci EMAC Ethernet
 *******************************************************/

#define AM35XX_MDIO_FREQUENCY	(1000000)

static struct mdio_platform_data cm_t3517_mdio_pdata = {
	.bus_freq	= AM35XX_MDIO_FREQUENCY,
};

static struct resource cm_t3517_mdio_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET,
		.end    = AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET +
			  SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device cm_t3517_mdio_device = {
	.name		= "davinci_mdio",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(cm_t3517_mdio_resources),
	.resource	= cm_t3517_mdio_resources,
	.dev.platform_data = &cm_t3517_mdio_pdata,
};

static struct emac_platform_data cm_t3517_emac_pdata = {
	.rmii_en	= 1,
};

static struct resource cm_t3517_emac_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE,
		.end    = AM35XX_IPSS_EMAC_BASE + 0x2FFFF,
		.flags  = IORESOURCE_MEM,
	}, {
		.start  = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.end    = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.flags  = IORESOURCE_IRQ,
	}, {
		.start  = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	}, {
		.start  = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	}, {
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

static void cm_t3517_init_emac(struct emac_platform_data *pdata)
{
    u32 regval, mac_lo, mac_hi;
	mac_lo = omap_ctrl_readl(AM35XX_CONTROL_FUSE_EMAC_LSB);
	mac_hi = omap_ctrl_readl(AM35XX_CONTROL_FUSE_EMAC_MSB);

	pdata->mac_addr[0] = (u_int8_t)((mac_hi & 0xFF0000) >> 16);
	pdata->mac_addr[1] = (u_int8_t)((mac_hi & 0xFF00) >> 8);
	pdata->mac_addr[2] = (u_int8_t)((mac_hi & 0xFF) >> 0);
	pdata->mac_addr[3] = (u_int8_t)((mac_lo & 0xFF0000) >> 16);
	pdata->mac_addr[4] = (u_int8_t)((mac_lo & 0xFF00) >> 8);
	pdata->mac_addr[5] = (u_int8_t)((mac_lo & 0xFF) >> 0);
	pdata->ctrl_reg_offset		= AM35XX_EMAC_CNTRL_OFFSET;
	pdata->ctrl_mod_reg_offset	= AM35XX_EMAC_CNTRL_MOD_OFFSET;
	pdata->ctrl_ram_offset		= AM35XX_EMAC_CNTRL_RAM_OFFSET;
	pdata->ctrl_ram_size		= AM35XX_EMAC_CNTRL_RAM_SIZE;
	pdata->version			= EMAC_VERSION_2;
	pdata->hw_ram_addr		= AM35XX_EMAC_HW_RAM_ADDR;
	pdata->interrupt_enable		= cm_t3517_enable_ethernet_int;
	pdata->interrupt_disable	= cm_t3517_disable_ethernet_int;
	cm_t3517_emac_device.dev.platform_data	= pdata;
	platform_device_register(&cm_t3517_emac_device);
	platform_device_register(&cm_t3517_mdio_device);
	clk_add_alias(NULL, dev_name(&cm_t3517_mdio_device.dev),
		      NULL, &cm_t3517_emac_device.dev);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	regval = regval & (~(AM35XX_CPGMACSS_SW_RST));
	omap_ctrl_writel(regval, AM35XX_CONTROL_IP_SW_RESET);
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
}

/*******************************************************
 * MMC0
 *******************************************************/

#if defined(CONFIG_MMC) || defined(CONFIG_MMC_MODULE)
static struct regulator_consumer_supply cm_t3517_vmmc_supplies[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.0"),
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.1")
};

/* static 3.3V regulator */
static struct regulator_init_data cm_t3517_v33 = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= cm_t3517_vmmc_supplies,
};

static struct fixed_voltage_config cm_t3517_v33_pdata = {
	.supply_name	= "board-3V3",
	.microvolts	= 3300000,
	.gpio		= -EINVAL,
	.init_data	= &cm_t3517_v33,
};

static struct platform_device cm_t3517_v33_platform_device = {
	.name		= "reg-fixed-voltage",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &cm_t3517_v33_pdata,
	},
};

static struct omap2_hsmmc_info cm_t3517_mmc[] = {
       {
               .mmc            = 1,
               .caps           = MMC_CAP_4_BIT_DATA,
               .gpio_cd        = 144,
               .gpio_wp        = 59,

       },
       {
               .mmc            = 2,
               .caps           = MMC_CAP_4_BIT_DATA,
               .gpio_cd        = -EINVAL,
               .gpio_wp        = -EINVAL,
       },
       {}      /* Terminator */
};

static void __init cm_t3517_mmc_init(void)
{
       /* MMC init function */
       omap2_hsmmc_init(cm_t3517_mmc);
       platform_device_register(&cm_t3517_v33_platform_device);
}
#else
static void __init cm_t3517_mmc_init(void) {}
#endif


/*******************************************************
 * LEDS GPIO
 *******************************************************/
 
#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
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

static void __init cm_t3517_init_leds(void)
{
	platform_device_register(&cm_t3517_led_device);
}
#else
static inline void cm_t3517_init_leds(void) {}
#endif

/****************************************************************************
 * HECC (High-End CAN Controller, for CAN-bus)
 ****************************************************************************/

#if defined(CONFIG_CAN_TI_HECC) || defined(CONFIG_CAN_TI_HECC_MODULE)
static struct resource cm_t3517_hecc_resources[] = {
	{
		.start	= AM35XX_IPSS_HECC_BASE,
		.end	= AM35XX_IPSS_HECC_BASE + SZ_16K - 1,
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
		.platform_data	= &cm_t3517_hecc_pdata,
	},
};

static void cm_t3517_init_hecc(void)
{
	platform_device_register(&cm_t3517_hecc_device);
}
#else
static inline void cm_t3517_init_hecc(void) {}
#endif

/*******************************************************
 * RTC
 *******************************************************/
 
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

	err = gpio_request_one(RTC_CS_EN_GPIO, GPIOF_OUT_INIT_HIGH,
			       "rtc cs en");
	if (err) {
		pr_err("CM-T3517: rtc cs en gpio request failed: %d\n", err);
		return;
	}

	platform_device_register(&cm_t3517_rtc_device);
}
#else
static inline void cm_t3517_init_rtc(void) {}
#endif

/*******************************************************
 * USB EHCI
 *******************************************************/
 
#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
#define HSUSB1_RESET_GPIO	(146)
#define HSUSB2_RESET_GPIO	(147)
#define USB_HUB_RESET_GPIO	(152)

static struct usbhs_omap_board_data cm_t3517_ehci_pdata __initdata = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

	.phy_reset  = true,
	.reset_gpio_port[0]  = HSUSB1_RESET_GPIO,
	.reset_gpio_port[1]  = HSUSB2_RESET_GPIO,
	.reset_gpio_port[2]  = -EINVAL,
};

static int __init cm_t3517_init_usbh(void)
{
	int err;

	err = gpio_request_one(USB_HUB_RESET_GPIO, GPIOF_OUT_INIT_LOW,
			       "usb hub rst");
	if (err) {
		pr_err("CM-T3517: usb hub rst gpio request failed: %d\n", err);
	} else {
		udelay(10);
		gpio_set_value(USB_HUB_RESET_GPIO, 1);
		msleep(1);
	}

	usbhs_init(&cm_t3517_ehci_pdata);

	return 0;
}
#else
static inline int cm_t3517_init_usbh(void)
{
	return 0;
}
#endif

/*******************************************************
 * MTD NAND
 *******************************************************/
 
#if defined(CONFIG_MTD_NAND_OMAP2) || defined(CONFIG_MTD_NAND_OMAP2_MODULE)
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
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x2A0000 */
		.size           = 32 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "rootfs",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x6A0000 */
		.size           = MTDPART_SIZ_FULL,
	},
};

static struct omap_nand_platform_data cm_t3517_nand_data = {
	.parts			= cm_t3517_nand_partitions,
	.nr_parts		= ARRAY_SIZE(cm_t3517_nand_partitions),
	.cs			= 0,
};

static void __init cm_t3517_init_nand(void)
{
	if (gpmc_nand_init(&cm_t3517_nand_data) < 0)
		pr_err("CM-T3517: NAND initialization failed\n");
}
#else
static inline void cm_t3517_init_nand(void) {}
#endif



static struct omap_board_config_kernel cm_t3517_config[] __initdata = {
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* RTC GPIOs: */
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

	/* GPIO186 - Green LED */
	OMAP3_MUX(SYS_CLKOUT2, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),



	/* HSUSB1 RESET */
	OMAP3_MUX(UART2_TX, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	/* HSUSB2 RESET */
	OMAP3_MUX(UART2_RX, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	/* CM-T3517 USB HUB nRESET */
	OMAP3_MUX(MCBSP4_CLKX, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),


	/* WLAN nRESET */
	OMAP3_MUX(UART2_RTS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),

	/* McBSP 2 */
	OMAP3_MUX(MCBSP2_FSX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_CLKX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_DR, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_DX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_NBE1, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static void __init cm_t3517_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap_serial_init();
	omap_sdrc_init(NULL, NULL);
	omap_board_config = cm_t3517_config;
	omap_board_config_size = ARRAY_SIZE(cm_t3517_config);
	cm_t3517_init_leds();
	cm_t3517_init_nand();
	cm_t3517_init_rtc();

	/*Ethernet*/
	cm_t3517_init_emac(&cm_t3517_emac_pdata);
	/* SB-T35 Ethernet */
	cm_t3517_init_ethernet();

	cm_t3517_init_usbh();
	cm_t3517_init_hecc();
	cm_t3517_mmc_init();
}

MACHINE_START(CM_T3517, "Compulab CM-T3517")
	.atag_offset	= 0x100,
	.reserve        = omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= am35xx_init_early,
	.init_irq	= omap3_init_irq,
	.init_machine	= cm_t3517_init,
	.timer		= &omap3_timer,
MACHINE_END
