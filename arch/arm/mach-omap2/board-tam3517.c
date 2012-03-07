/*
 * linux/arch/arm/mach-omap2/board-tam3517.c
 *
 * Copyright (C) 2012 Technexion and friends
 * Author: Technexion + others
 *
 * Based on mach-omap2/board-tam3517.c from Technexion BSP release
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as  published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/can/platform/ti_hecc.h>

#include <linux/mmc/host.h>

#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>

#include <mach/hardware.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include "common.h"
#include <plat/usb.h>
#include <plat/nand.h>
#include <plat/gpmc.h>

#include <mach/am35xx.h>

#include "mux.h"
#include "control.h"
#include "hsmmc.h"

/* custom settings */
#define ENABLE_EMAC_ETH	1 // this messes with the SMSC right now
#define USE_ALT__EMAC_ETH	1

#define ENABLE_SMSC_ETH	1 // this messes with the EMAC right now
#define USE_ALT__SMSC_ETH	0

/****************************************************************************
 *
 * Display SubSystem
 *
 ****************************************************************************/

/* -- CANNOT BE DISABLED or board dies!!! -- */
//#if ( defined(CONFIG_FB_OMAP2) || defined(CONFIG_FB_OMAP2_MODULE) )

#include <video/omapdss.h>

#define TAM3517_LCD_ENVDD_GPIO			138
#define TAM3517_LCD_BKLIGHT_PON_GPIO		53
#define TAM3517_LCD_PON_GPIO			139
#define	TAM3517_DVI_PON_GPIO			24

#define TAM3517_DEFAULT_BACKLIGHT_LEVEL 99

static int tam3517_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(TAM3517_DVI_PON_GPIO, 0);
	gpio_set_value(TAM3517_LCD_ENVDD_GPIO, 0);
	gpio_set_value(TAM3517_LCD_PON_GPIO, 1);
        printk("LCD voltage on\n");
	return 0;
}

static void tam3517_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(TAM3517_LCD_ENVDD_GPIO, 1);
	gpio_set_value(TAM3517_LCD_PON_GPIO, 0);
}

/* TODO? Set display timings here from bootargs, and get rid of the tnlcd / panel tao-series driver? */
static struct omap_dss_device tam3517_lcd_device = {
	.type                   = OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd",
	.driver_name		= "tnlcd",
        .platform_enable        = tam3517_panel_enable_lcd,
        .platform_disable       = tam3517_panel_disable_lcd,
        .max_backlight_level    = 100,
//#ifdef CONFIG_TAM3517_THB_CARRIER
	.phy.dpi.data_lines	= 24,
//#else
//	.phy.dpi.data_lines	= 18,
//#endif

};


static struct omap_dss_device *tam3517_dss_devices[] = {
	&tam3517_lcd_device,
};

static struct omap_dss_board_info tam3517_dss_data = {
	.num_devices	= ARRAY_SIZE(tam3517_dss_devices),
	.devices	= tam3517_dss_devices,
	.default_device	= &tam3517_lcd_device,
};

struct platform_device tam3517_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data	= &tam3517_dss_data,
	},
};

//#endif // CANNOT BE DISABLED OR BOARD DOES NOT BOOT!



/****************************************************************************
 *
 * SOUND
 *
 ****************************************************************************/

/* The sound driver had to be moved to sound/soc/omap/ due to initialization
order. Loading it too early hangs kernel, too late crashes */


/****************************************************************************
 *
 *  NAND Flash setup
 *
 ****************************************************************************/

#define NAND_BLOCK_SIZE SZ_128K

/*OAMP3 Teh Flash Init*/
static struct mtd_partition tam3517_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 15 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 32 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct omap_nand_platform_data tam3517_nand_data = {
        .cs = GPMC_CS_NUM+1,
        .parts = tam3517_nand_partitions,
        .nr_parts = ARRAY_SIZE(tam3517_nand_partitions),
        .xfer_type = NAND_OMAP_PREFETCH_DMA,
        .ecc_opt = OMAP_ECC_HAMMING_CODE_DEFAULT,
        .devsize = NAND_BUSWIDTH_16,
};

void tam3517_nand_init(void) {
	u8 nandcs;

        for (nandcs=0; nandcs < GPMC_CS_NUM; nandcs++) {
                /* Detect NAND chipselect */
		if ((gpmc_cs_read_reg(nandcs, GPMC_CS_CONFIG1) & 0xC00) == 0x800) {
                        tam3517_nand_data.cs = nandcs;
                        if (gpmc_nand_init(&tam3517_nand_data) < 0)
                                printk(KERN_ERR "Unable to register NAND device\n");                        
                        return;
                }
	}

        printk(KERN_INFO "NAND: Unable to find configuration in GPMC\n");        
}


/****************************************************************************
 *
 * MMC bus
 *
 ****************************************************************************/

static struct omap2_hsmmc_info mmc[] = {
        /* SD-card */
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	/* WIFI */
        {
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.transceiver    = true,
		.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34, /* 3.2-3.4V */
	},
	{}	/* Terminator */
};

/****************************************************************************
 *
 * SMSC LAN
 *
 ****************************************************************************/

#if ENABLE_SMSC_ETH && ( defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE) )

#include <linux/smsc911x.h>
#include <plat/gpmc-smsc911x.h>

#define SMSC911X_GPIO_IRQ 153
#define SMSC911X_GPIO_RESET 142
#define SMSC911X_GPIO_CS 5

#if USE_ALT__SMSC_ETH // gpmc-smsc911x style

static struct omap_smsc911x_platform_data tam3517_smsc911x_cfg = {
	.id		= 0,
	.cs             = SMSC911X_GPIO_CS,
	.gpio_irq       = SMSC911X_GPIO_IRQ,
	.gpio_reset     = -EINVAL,
	.flags		= SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS,
};

static void __init tam3517_init_smsc911x(void)
{
	gpmc_smsc911x_init(&tam3517_smsc911x_cfg);
}

#else // use older style


static struct resource tam3517_smsc911x_resources[] = {
	{
		.name	= "smsc911x-memory",
		.flags	= IORESOURCE_MEM,
	},
	{
		.start  = OMAP_GPIO_IRQ(SMSC911X_GPIO_IRQ),
		.end    = OMAP_GPIO_IRQ(SMSC911X_GPIO_IRQ),
		.flags	=  (IORESOURCE_IRQ | IRQF_TRIGGER_LOW),
	},
};

static struct smsc911x_platform_config smsc911x_config = {
	.phy_interface	= PHY_INTERFACE_MODE_MII,
    .irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
    .irq_type       = SMSC911X_IRQ_TYPE_OPEN_DRAIN,
    .flags          = SMSC911X_USE_16BIT | SMSC911X_SAVE_MAC_ADDRESS,
};

static struct platform_device tam3517_smsc911x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(tam3517_smsc911x_resources),
	.resource	= tam3517_smsc911x_resources,
	.dev		= {
		.platform_data = &smsc911x_config,
	},
};

static void __init tam3517_init_smsc911x(void)
{
	unsigned long cs_mem_base;

	if (gpmc_cs_request(SMSC911X_GPIO_CS, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed request for GPMC mem for smsc911x\n");
		return;
	}

	tam3517_smsc911x_resources[0].start = cs_mem_base + 0x0;
	tam3517_smsc911x_resources[0].end   = cs_mem_base + 0xFF;

	if ((gpio_request(SMSC911X_GPIO_IRQ, "smsc911x irq") == 0) &&
	    (gpio_direction_input(SMSC911X_GPIO_IRQ) == 0)) {
		gpio_export(SMSC911X_GPIO_IRQ, 0);
	} else {
		printk(KERN_ERR "could not obtain gpio for SMSC911X IRQ\n");
		return;
	}

	omap_mux_init_gpio(SMSC911X_GPIO_IRQ, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE4);
	gpio_direction_input(SMSC911X_GPIO_IRQ);

	// next 2 lines redundant?
	tam3517_smsc911x_resources[1].start = OMAP_GPIO_IRQ(SMSC911X_GPIO_IRQ);
	tam3517_smsc911x_resources[1].end  = OMAP_GPIO_IRQ(SMSC911X_GPIO_IRQ);
	omap_mux_init_gpio(SMSC911X_GPIO_RESET, OMAP_PIN_INPUT_PULLUP|OMAP_MUX_MODE4);

	if (gpio_request(SMSC911X_GPIO_RESET, "smsc911x reset") < 0)
	{
		printk(KERN_ERR "can't get smsc911x reset GPIO\n");
		return;
	}
	
	gpio_direction_output(SMSC911X_GPIO_RESET, 0);
	mdelay(1);
	gpio_direction_output(SMSC911X_GPIO_RESET, 1);

}

#endif // USE_ALT__SMSC_ETH

#else
static inline void __init tam3517_init_smsc911x(void) { return; }
#endif

/****************************************************************************
 *
 * EMAC LAN
 *
 ****************************************************************************/
#if ENABLE_EMAC_ETH

#include <linux/davinci_emac.h>

#if USE_ALT__EMAC_ETH // Use new standalone EMAC code for generic AM35xx?

#include "am35xx-emac.h"

#else // Use original Davinci EMAC code

static struct resource tam3517_mdio_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET,
		.end    = AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET +
			  SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct mdio_platform_data tam3517_mdio_pdata = {
	.bus_freq	= 1000000,
};

static struct platform_device tam3517_mdio_device = {
	.name		= "davinci_mdio",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(tam3517_mdio_resources),
	.resource	= tam3517_mdio_resources,
	.dev.platform_data = &tam3517_mdio_pdata,
};

static struct emac_platform_data tam3517_emac_pdata = {
	.rmii_en	= 1,
};

static struct resource tam3517_emac_resources[] = {
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

static struct platform_device tam3517_emac_device = {
	.name		= "davinci_emac",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(tam3517_emac_resources),
	.resource	= tam3517_emac_resources,
};

static void tam3517_enable_emac_int(void)
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

static void tam3517_disable_emac_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void tam3517_emac_ethernet_init(void) {
	u32 regval, mac_lo, mac_hi;

	mac_lo = omap_ctrl_readl(AM35XX_CONTROL_FUSE_EMAC_LSB);
	mac_hi = omap_ctrl_readl(AM35XX_CONTROL_FUSE_EMAC_MSB);

	tam3517_emac_pdata.mac_addr[0] = (u_int8_t)((mac_hi & 0xFF0000) >> 16);
	tam3517_emac_pdata.mac_addr[1] = (u_int8_t)((mac_hi & 0xFF00) >> 8);
	tam3517_emac_pdata.mac_addr[2] = (u_int8_t)((mac_hi & 0xFF) >> 0);
	tam3517_emac_pdata.mac_addr[3] = (u_int8_t)((mac_lo & 0xFF0000) >> 16);
	tam3517_emac_pdata.mac_addr[4] = (u_int8_t)((mac_lo & 0xFF00) >> 8);
	tam3517_emac_pdata.mac_addr[5] = (u_int8_t)((mac_lo & 0xFF) >> 0);

	tam3517_emac_pdata.ctrl_reg_offset		= AM35XX_EMAC_CNTRL_OFFSET;
	tam3517_emac_pdata.ctrl_mod_reg_offset		= AM35XX_EMAC_CNTRL_MOD_OFFSET;
	tam3517_emac_pdata.ctrl_ram_offset		= AM35XX_EMAC_CNTRL_RAM_OFFSET;
	tam3517_emac_pdata.ctrl_ram_size		= AM35XX_EMAC_CNTRL_RAM_SIZE;
	tam3517_emac_pdata.version			= EMAC_VERSION_2;
	tam3517_emac_pdata.hw_ram_addr			= AM35XX_EMAC_HW_RAM_ADDR;
	tam3517_emac_pdata.interrupt_enable		= tam3517_enable_emac_int;
	tam3517_emac_pdata.interrupt_disable		= tam3517_disable_emac_int;
	tam3517_emac_device.dev.platform_data		= &tam3517_emac_pdata;
/* taken care of with platform_add_devices() below
	platform_device_register(&tam3517_emac_device);
	platform_device_register(&tam3517_mdio_device);
*/
	clk_add_alias(NULL, dev_name(&tam3517_mdio_device.dev),
		      NULL, &tam3517_emac_device.dev);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	regval = regval & (~(AM35XX_CPGMACSS_SW_RST));
	omap_ctrl_writel(regval, AM35XX_CONTROL_IP_SW_RESET);
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
}

#endif  // USE_ALT__EMAC_ETH
#endif  // ENABLE_EMAC_ETH

/****************************************************************************
 *
 * TPS65023 voltage regulator
 *
 ****************************************************************************/

/* VDCDC1 -> VDD_CORE */
static struct regulator_consumer_supply tam3517_vdcdc1_supplies[] = {
	{
		.supply = "vdd_core",
	},
};

/* VDCDC2 -> VDDSHV */
static struct regulator_consumer_supply tam3517_vdcdc2_supplies[] = {
	{
		.supply = "vddshv",
	},
	REGULATOR_SUPPLY("vdds_dsi", "omapdss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};

/* VDCDC3 |-> VDDS
	   |-> VDDS_SRAM_CORE_BG
	   |-> VDDS_SRAM_MPU */
static struct regulator_consumer_supply tam3517_vdcdc3_supplies[] = {
	{
		.supply = "vdds",
	},
	{
		.supply = "vdds_sram_core_bg",
	},
	{
		.supply = "vdds_sram_mpu",
	},
};

/* LDO1 |-> VDDA1P8V_USBPHY
		|-> VDDA_DAC */
static struct regulator_consumer_supply tam3517_ldo1_supplies[] = {
	{
		.supply = "vdda1p8v_usbphy",
	},
	{
		.supply = "vdda_dac",
                .dev	= &tam3517_dss_device.dev,
	},
};

/* LDO2 -> VDDA3P3V_USBPHY */
static struct regulator_consumer_supply tam3517_ldo2_supplies[] = {
	{
		.supply = "vdda3p3v_usbphy",
	},
};


static struct regulator_init_data tam3517_regulators[] = {
	/* DCDC1 */
	{
		.constraints = {
			.min_uV = 1200000,
			.max_uV = 1600000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(tam3517_vdcdc1_supplies),
		.consumer_supplies = tam3517_vdcdc1_supplies,
	},
	/* DCDC2 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000, /* should be 1V8 -- 3V3 */
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(tam3517_vdcdc2_supplies),
		.consumer_supplies = tam3517_vdcdc2_supplies,
	},
	/* DCDC3 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(tam3517_vdcdc3_supplies),
		.consumer_supplies = tam3517_vdcdc3_supplies,
	},
	/* LDO1 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(tam3517_ldo1_supplies),
		.consumer_supplies = tam3517_ldo1_supplies,
	},
	/* LDO2 */
	{
		.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(tam3517_ldo2_supplies),
		.consumer_supplies = tam3517_ldo2_supplies,
	},
};

/****************************************************************************
 *
 * HECC (High-End CAN Controller, for CAN-bus)
 *
 ****************************************************************************/

#if 0 && ( defined(CONFIG_CAN_TI_HECC) || defined(CONFIG_CAN_TI_HECC_MODULE) )
/*
 * HECC information
 */

static struct resource tam3517_hecc_resources[] = {
        {
                .start  = AM35XX_IPSS_HECC_BASE,
                .end    = AM35XX_IPSS_HECC_BASE + 0x3FFF,
                .flags  = IORESOURCE_MEM,
        },
        {
                .start  = INT_35XX_HECC0_IRQ,
                .end    = INT_35XX_HECC0_IRQ,
                .flags  = IORESOURCE_IRQ,
        },
};

static struct ti_hecc_platform_data tam3517_hecc_pdata = {
        .scc_hecc_offset        = AM35XX_HECC_SCC_HECC_OFFSET,
        .scc_ram_offset         = AM35XX_HECC_SCC_RAM_OFFSET,
        .hecc_ram_offset        = AM35XX_HECC_RAM_OFFSET,
        .mbx_offset            = AM35XX_HECC_MBOX_OFFSET,
        .int_line               = AM35XX_HECC_INT_LINE,
        .version                = AM35XX_HECC_VERSION,
};

static struct platform_device tam3517_hecc_device = {
        .name           = "ti_hecc",
        .id             = 1,
        .num_resources  = ARRAY_SIZE(tam3517_hecc_resources),
        .resource       = tam3517_hecc_resources,
	.dev.platform_data = &tam3517_hecc_pdata,
};


#endif

/****************************************************************************
 *
 * I2C
 *
 ****************************************************************************/

static struct i2c_board_info __initdata tam3517_i2c1_boardinfo[] = {
        {
                I2C_BOARD_INFO("tps65023", 0x48),
                .flags = I2C_CLIENT_WAKE,
                .platform_data = tam3517_regulators,
        }, {
                I2C_BOARD_INFO("24c02", 0x50),
        }, {
                I2C_BOARD_INFO("tlv320aic23", 0x1a),
        },
};

static struct i2c_board_info __initdata tam3517_i2c2_boardinfo[] = {
/*
        {
                I2C_BOARD_INFO("24c02", 0x50),
        },
*/
};

static struct i2c_board_info __initdata tam3517_i2c3_boardinfo[] = {
        {
                I2C_BOARD_INFO("s35390a", 0x30),
                .type           = "s35390a",
        }, {
                I2C_BOARD_INFO("ds1307", 0x68),
        },
};


static int __init tam3517_i2c_init(void)
{
// comment top line?
	omap_register_i2c_bus(1, 400, tam3517_i2c1_boardinfo,
			ARRAY_SIZE(tam3517_i2c1_boardinfo));
	omap_register_i2c_bus(2, 400, tam3517_i2c2_boardinfo,
			ARRAY_SIZE(tam3517_i2c2_boardinfo));
	omap_register_i2c_bus(3, 400, tam3517_i2c3_boardinfo,
                        ARRAY_SIZE(tam3517_i2c3_boardinfo));
	return 0;
}

/****************************************************************************
 *
 * MUX
 *
 ****************************************************************************/

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux tam3517_mux[] __initdata = {
	/* USB OTG DRVVBUS offset = 0x212 */
	OMAP3_MUX(CHASSIS_DMAREQ3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN), /* spi */
	OMAP3_MUX(MCBSP_CLKS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
        OMAP3_MUX(MCSPI1_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(MCSPI1_SIMO, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
        OMAP3_MUX(MCSPI1_SOMI, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(MCSPI1_CS0, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
          
        OMAP3_MUX(SDMMC2_DAT4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP), /* Touchscreen irq */
        OMAP3_MUX(SDMMC2_DAT6, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* LCD */
        OMAP3_MUX(SDMMC2_DAT7, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* LCD */
        OMAP3_MUX(ETK_D10, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN), /* DVI */
	OMAP3_MUX(GPMC_NCS2, OMAP_MUX_MODE4 | OMAP_PULL_UP), /* BL */
	OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(GPMC_NCS6, OMAP_MUX_MODE4 ),
	OMAP3_MUX(GPMC_NCS7, OMAP_MUX_MODE4 | OMAP_PULL_UP),
        OMAP3_MUX(MCBSP3_CLKX, OMAP_MUX_MODE4), /* GPIO 142 for keypad */

#if 1
	/* MCBSP2 config */
	OMAP3_MUX(MCBSP2_CLKX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_FSX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_DR, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_DX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
#endif

 { .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

/****************************************************************************
 *
 * USB
 *
 ****************************************************************************/

#define TAM3517_EHCI_RESET_PIN	25

static struct omap_musb_board_data tam3517_musb_data = {
	.interface_type         = MUSB_INTERFACE_ULPI,
	.mode                   = MUSB_OTG,
	.power                  = 500,
};

static struct usbhs_omap_board_data tam3517_ehci_pdata __initdata = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

	.phy_reset  = true,
	.reset_gpio_port[0]  = TAM3517_EHCI_RESET_PIN,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};


static __init void tam3517_usb_init(void) {
	u32 devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	/* Set USB2.0 PHY reference clock to 13 MHz */
	devconf2 &= ~(CONF2_REFFREQ | CONF2_OTGMODE | CONF2_PHY_GPIOMODE);
	devconf2 |=  CONF2_REFFREQ_13MHZ | CONF2_SESENDEN | CONF2_VBDTCTEN | CONF2_DATPOL;
        
	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);

	usb_musb_init(&tam3517_musb_data);
	omap_mux_init_gpio(TAM3517_EHCI_RESET_PIN, OMAP_PIN_OUTPUT);
        usbhs_init(&tam3517_ehci_pdata);
}

/****************************************************************************
 *
 * GPIO Keypad (THB HMI only)
 *
 ****************************************************************************/

#if 0 && ( defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE) )
#include <linux/gpio_keys.h>

static struct gpio_keys_button tam3517_gpio_buttons[] = { 
        {
                .code                   = KEY_HOME,
                .gpio                   = 65,
                .desc                   = "home",
                .wakeup                 = 1,
		.active_low		= 1,
        }, {
                .code                   = KEY_ENTER,
                .gpio                   = 64,
                .desc                   = "enter",
                .wakeup                 = 1,
		.active_low		= 1,
        }, {
                .code                   = KEY_BACK,
                .gpio                   = 63,
                .desc                   = "back",
                .wakeup                 = 1,
		.active_low		= 1,
        }, {
                .code                   = KEY_MENU,
                .gpio                   = 150,
                .desc                   = "menu",
                .wakeup                 = 1,
		.active_low		= 1,
        }, {        
                .code                   = KEY_BRIGHTNESSUP,
                .gpio                   = 143,
                .desc                   = "brightness up",
                .wakeup                 = 1,
                .active_low             = 1,
        }, {
                .code                   = KEY_BRIGHTNESSDOWN,
                .gpio                   = 142,
                .desc                   = "brightness down",
                .wakeup                 = 1,
                .active_low             = 1,
        }, {
                .code                   = KEY_VOLUMEUP,
                .gpio                   = 141,
                .desc                   = "volume up",
                .wakeup                 = 1,
		.active_low		= 1,
        }, {
                .code                   = KEY_VOLUMEDOWN,
                .gpio                   = 140,
                .desc                   = "volume down",
                .wakeup                 = 1,
		.active_low		= 1,
        },
};

static struct gpio_keys_platform_data tam3517_gpio_key_info = {
        .buttons        = tam3517_gpio_buttons,
        .nbuttons       = ARRAY_SIZE(tam3517_gpio_buttons),
};

struct platform_device tam3517_keys_gpio = {
        .name   = "gpio-keys",
        .id     = -1,
        .dev    = {
                .platform_data  = &tam3517_gpio_key_info,
        },
};
#endif

/* --------------------------------------------------------- */

static struct omap_board_config_kernel tam3517_config[] = {};

/* --------------------------------------------------------- */
static struct platform_device *tam3517_devices[] __initdata = {
#if ENABLE_SMSC_ETH && !(USE_ALT__SMSC_ETH) && ( defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE) )
	&tam3517_smsc911x_device,
#endif
#if 0 && ( defined(CONFIG_CAN_TI_HECC) || defined(CONFIG_CAN_TI_HECC_MODULE) )
	&tam3517_hecc_device,
#endif
	&tam3517_dss_device,
#if ENABLE_EMAC_ETH && !(USE_ALT__EMAC_ETH)
	&tam3517_mdio_device,
	&tam3517_emac_device,
#endif
#if 0 && ( defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE) )
	&tam3517_keys_gpio,
#endif
};

/* ------------------------------------------------------------------- */

static void __init tam3517_init(void) {
	platform_add_devices(tam3517_devices, ARRAY_SIZE(tam3517_devices));
	omap_board_config = tam3517_config;
	omap_board_config_size = ARRAY_SIZE(tam3517_config);
	omap3_mux_init(tam3517_mux, OMAP_PACKAGE_CBC);
	omap_serial_init();
	tam3517_i2c_init();
        
	omap_hsmmc_init(mmc);
        
	tam3517_usb_init();        
	tam3517_nand_init();

	/*Ethernet:  SMSC911x */
#if ENABLE_SMSC_ETH
	tam3517_init_smsc911x();
#endif

	/*Ethernet:  DaVinci EMAC */
#if ENABLE_EMAC_ETH
#if USE_ALT__EMAC_ETH
	am35xx_emac_init(AM35XX_DEFAULT_MDIO_FREQUENCY, 1);
#else
	tam3517_emac_ethernet_init();
#endif  // USE_ALT__EMAC_ETH
#endif  // ENABLE_EMAC_ETH
	
}

MACHINE_START(TAM3517, "Technexion TAM3517")
	.atag_offset	= 0x100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= am35xx_init_early,
	.init_irq	= omap3_init_irq,
	.handle_irq	= omap3_intc_handle_irq,
	.init_machine	= tam3517_init,
	.timer		= &omap3_timer,
	.restart	= omap_prcm_restart,
MACHINE_END
