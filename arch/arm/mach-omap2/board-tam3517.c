/*
 * linux/arch/arm/mach-omap2/board-tam3517.c
 *
 * Copyright (C) 2012 Technexion and friends
 * Author: Technexion + LOTS of help!
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
#include <linux/can/platform/ti_hecc.h>
#include <linux/serial_8250.h>

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

/* SC custom settings */
#define ENABLE_EMAC_ETH				1
#define USE_EXTERNAL_INIT__EMAC_ETH	1

#define ENABLE_SMSC_ETH				0
#define USE_EXTERNAL_INIT__SMSC_ETH	1

#define ENABLE_HECC					0
#define ENABLE_I2C_TPS65023			1
#define ENABLE_I2C_TLV320AIC23		0
#define ENABLE_I2C_DS1307			0
#define ENABLE_I2C_M41T65			1
#define TAM3517_ENABLE_DUAL_UART		0

#if (TAM3517_ENABLE_DUAL_UART)

static struct plat_serial8250_port serial_platform_data[] = {
	{
		.mapbase	= 0x21000000,
		.flags		= UPF_BOOT_AUTOCONF|UPF_IOREMAP|UPF_SHARE_IRQ,
		.irqflags	= IRQF_SHARED | IRQF_TRIGGER_RISING,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= 3072000,
	}, 
	{
		.mapbase	= 0x22000000,
		.flags		= UPF_BOOT_AUTOCONF|UPF_IOREMAP|UPF_SHARE_IRQ,
		.irqflags	= IRQF_SHARED | IRQF_TRIGGER_RISING,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= 3072000,
	},
	{}
};

static struct platform_device tam3517_serial_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= serial_platform_data,
	},
};

#define TAM3517_UART_IRQ_A_GPIO  127
#define TAM3517_UART_IRQ_B_GPIO  128

static inline void __init tam3517_init_dualuart(void)
{
	unsigned long cs_mem_base;

	if (gpmc_cs_request(4, SZ_1M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem CS4"
				"for Dual UART(TL16CP752C)\n");
		return;
	}

	if (gpmc_cs_request(5, SZ_1M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem CS5"
				"for Dual UART(TL16CP752C)\n");
		return;
	}

	if (gpio_request_one(TAM3517_UART_IRQ_A_GPIO, GPIOF_IN, "TL16CP7542 IRQ A") < 0)
		printk(KERN_ERR "Failed to request GPIO%d for TL16CP752C IRQ\n",
								TAM3517_UART_IRQ_A_GPIO);

	serial_platform_data[0].irq = gpio_to_irq(TAM3517_UART_IRQ_A_GPIO);

	if (gpio_request_one(TAM3517_UART_IRQ_B_GPIO, GPIOF_IN, "TL16CP7542 IRQ B") < 0)
		printk(KERN_ERR "Failed to request GPIO%d for TL16CP752C IRQ\n",
								TAM3517_UART_IRQ_A_GPIO);

	serial_platform_data[0].irq = gpio_to_irq(TAM3517_UART_IRQ_A_GPIO);
	serial_platform_data[1].irq = gpio_to_irq(TAM3517_UART_IRQ_B_GPIO);
}

#endif

/****************************************************************************
 *
 *  NAND Flash setup
 *
 ****************************************************************************/

#if defined(CONFIG_MTD_NAND_OMAP2) || defined(CONFIG_MTD_NAND_OMAP2_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#define NAND_BLOCK_SIZE SZ_128K

/*
 * [let's try the 1st one]
 * 0x00000000 - 0x0007FFFF  Booting Image (X-Loader, 4 copies)	// 512k
 * 0x00080000 - 0x0023FFFF  U-Boot Image	// 1835k
 * 0x00240000 - 0x0027FFFF  U-Boot Env Data (X-loader doesn't care)	// 256k
 * 0x00280000 - 0x0077FFFF  Kernel Image // 5120k
 * 0x00780000 - 0x0CF80000	Root FS (read-only) // 200MB
 * 0x0CF80000 - 0x1FB80000	Root FS (writable overlay) // 300MB
 * 0x1FB80000 - 0x20000000  DATA // ~4.5MB
 */

static struct mtd_partition tam3517_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,  /* -> 0x00080000 (size: 0x80000, 512k) */
		.mask_flags	= MTD_WRITEABLE,        /* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,   /* Offset = 0x00080000 */
		.size		= 14 * NAND_BLOCK_SIZE, /* -> 0x00240000 (size: 0x1C0000, 1835k) */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,   /* Offset = 0x00240000 */
		.size		= 2 * NAND_BLOCK_SIZE,  /* -> 0x00280000 (size: 0x40000, 256k) */
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,   /* Offset = 0x00280000 */
		.size		= 60 * NAND_BLOCK_SIZE, /* -> 0x00780000 (size: 0x500000, 5120k) */
	},
	{
		.name		= "Root Filesystem (read-only)",
		.offset		= MTDPART_OFS_APPEND,    /* Offset = 0x00780000 */
		.size		= 1600 * NAND_BLOCK_SIZE, /* -> 0x0CF80000 (size: 0xC800000, 200M) */
	},
	{
		.name		= "Root Filesystem (writeable overlay)",
		.offset		= MTDPART_OFS_APPEND,    /* Offset = 0x0CF80000 */
		.size		= 800 * NAND_BLOCK_SIZE, /* -> 0x1FB80000 (size: 0x12C00000, 100M) */
	},
	{
		.name		= "DATA",
		.offset		= MTDPART_OFS_APPEND,    /* Offset = 0x1FB80000 */
		.size		= MTDPART_SIZ_FULL,      /* -> 0x20000000 (size: 0x480000, ~4.5MB) */
	},
};

#if 0 // older TAM-style initialization
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
#else  // newer CL-style initialization

static struct omap_nand_platform_data tam3517_nand_data = {
	.parts			= tam3517_nand_partitions,
	.nr_parts		= ARRAY_SIZE(tam3517_nand_partitions),
	.cs				= 0,
};

static void __init tam3517_nand_init(void)
{
	if (gpmc_nand_init(&tam3517_nand_data) < 0)
		pr_err("TAM3517: NAND initialization failed\n");
}

#endif

#else
static inline void tam3517_nand_init(void) {}
#endif



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

#if (ENABLE_SMSC_ETH) && ( defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE) )

#include <linux/smsc911x.h>

#define SMSC911X_GPIO_IRQ		65 // Twister: 153
#define SMSC911X_GPIO_RESET		26 // Twister: 142
#define SMSC911X_GPIO_CS		4  // Twister: 5

#if (USE_EXTERNAL_INIT__SMSC_ETH) // gpmc-smsc911x style

#include <plat/gpmc-smsc911x.h>

static struct omap_smsc911x_platform_data tam3517_smsc911x_cfg = {
	.id		= 0, // removed by Igor's CL patches, but would likely be 0 anyway
	.cs             = SMSC911X_GPIO_CS,
	.gpio_irq       = SMSC911X_GPIO_IRQ,
	.gpio_reset     = SMSC911X_GPIO_RESET,
	.flags		= SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS | SMSC911X_FORCE_INTERNAL_PHY, // SMSC911X_USE_16BIT,
};

static struct regulator_consumer_supply tam3517_smsc911x_supplies[] = {
	REGULATOR_SUPPLY("vddvario", "smsc911x.0"),
	REGULATOR_SUPPLY("vdd33a", "smsc911x.0"),
};

static void __init tam3517_init_smsc911x(void)
{
	regulator_register_fixed(0, tam3517_smsc911x_supplies,
				 ARRAY_SIZE(tam3517_smsc911x_supplies));

	gpmc_smsc911x_init(&tam3517_smsc911x_cfg);
}

#else // use non-gpmc-smsc911x style; modeled after the old Overo


static struct resource tam3517_smsc911x_resources[] = {
	{
		.name	= "smsc911x-memory",
		.flags	= IORESOURCE_MEM,
	},
	{
		.flags	=  IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL, // IRQF_TRIGGER_LOW, // (same value: 8)
	},
};

static struct smsc911x_platform_config smsc911x_config = {
	.irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type       = SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags			= SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS | SMSC911X_FORCE_INTERNAL_PHY, // SMSC911X_USE_16BIT,
	.phy_interface	= PHY_INTERFACE_MODE_MII,
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
	tam3517_smsc911x_resources[0].end   = cs_mem_base + 0xff;

	if ((gpio_request(SMSC911X_GPIO_IRQ, "smsc911x irq") == 0) &&
	    (gpio_direction_input(SMSC911X_GPIO_IRQ) == 0)) {
		gpio_export(SMSC911X_GPIO_IRQ, 0);
	} else {
		printk(KERN_ERR "could not obtain gpio for SMSC911X IRQ\n");
		return;
	}

	tam3517_smsc911x_resources[1].start = OMAP_GPIO_IRQ(SMSC911X_GPIO_IRQ);
	tam3517_smsc911x_resources[1].end   = 0; // OMAP_GPIO_IRQ(SMSC911X_GPIO_IRQ);
	
	if (gpio_request(SMSC911X_GPIO_RESET, "smsc911x reset") < 0)
	{
		printk(KERN_ERR "could not obtain gpio for SMSC911X RESET");
		return;
	}
	
	gpio_direction_output(SMSC911X_GPIO_RESET, 0);
	/* Use value from gpmc (100) instead of original 1 from this legacy driver
	 *  datasheet says minimum 30ms
	 */  
	mdelay(100);
	gpio_direction_output(SMSC911X_GPIO_RESET, 1);

}

#endif // USE_EXTERNAL_INIT__SMSC_ETH

#else
static inline void __init tam3517_init_smsc911x(void) { return; }
#endif

/****************************************************************************
 *
 * EMAC LAN
 *
 ****************************************************************************/
#if (ENABLE_EMAC_ETH)

#include <linux/davinci_emac.h>

#if (USE_EXTERNAL_INIT__EMAC_ETH) /* Use new standalone EMAC code for generic AM35xx? */

#include "am35xx-emac.h"

#else // Use original Davinci EMAC code

/* 
 * To use this code, configured for mdio "2", you MUST update the line below in
 *  arch/arm/mach-omap2/clock3xxx_data.c like this!:
-	CLK("davinci_mdio.0",	NULL,	&emac_fck,	CK_AM35XX),
+	CLK("davinci_mdio.2",	NULL,	&emac_fck,	CK_AM35XX),
 */
 
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
	.id		= 2,
	.num_resources	= ARRAY_SIZE(tam3517_mdio_resources),
	.resource	= tam3517_mdio_resources,
	.dev.platform_data = &tam3517_mdio_pdata,
};

static struct emac_platform_data tam3517_emac_pdata = {
	.phy_id		= "2:00",
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

#endif  // USE_EXTERNAL_INIT__EMAC_ETH
#endif  // ENABLE_EMAC_ETH

/****************************************************************************
 *
 * TPS65023 voltage regulator
 *
 ****************************************************************************/

#if (ENABLE_I2C_TPS65023) /* Unsure about this code from TN */

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
			.valid_modes_mask = REGULATOR_MODE_NORMAL
//					| REGULATOR_MODE_STANDBY
					,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS
//					| REGULATOR_CHANGE_MODE
//					| REGULATOR_CHANGE_VOLTAGE
					,
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
			.valid_modes_mask = REGULATOR_MODE_NORMAL
//					| REGULATOR_MODE_STANDBY
					,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS
//					| REGULATOR_CHANGE_MODE
//					| REGULATOR_CHANGE_VOLTAGE
					,
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
			.valid_modes_mask = REGULATOR_MODE_NORMAL
//					| REGULATOR_MODE_STANDBY
					,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS
//					| REGULATOR_CHANGE_MODE
//					| REGULATOR_CHANGE_VOLTAGE
					,
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
			.valid_modes_mask = REGULATOR_MODE_NORMAL
//					| REGULATOR_MODE_STANDBY
					,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS
//					| REGULATOR_CHANGE_MODE
//					| REGULATOR_CHANGE_VOLTAGE
					,
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
			.valid_modes_mask = REGULATOR_MODE_NORMAL
//					| REGULATOR_MODE_STANDBY
					,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS
//					| REGULATOR_CHANGE_MODE
//					| REGULATOR_CHANGE_VOLTAGE
					,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(tam3517_ldo2_supplies),
		.consumer_supplies = tam3517_ldo2_supplies,
	},
};

#endif /* ENABLE_I2C_TPS65023 */

/****************************************************************************
 *
 * HECC (High-End CAN Controller, for CAN-bus)
 *
 ****************************************************************************/

#if (ENABLE_HECC) && ( defined(CONFIG_CAN_TI_HECC) || defined(CONFIG_CAN_TI_HECC_MODULE) )
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
#if (ENABLE_I2C_TPS65023)
        {
                I2C_BOARD_INFO("tps65023", 0x48),
                .flags = I2C_CLIENT_WAKE,
                .platform_data = tam3517_regulators,
        },
#endif  /* ENABLE_I2C_TPS65023 */
		{
                I2C_BOARD_INFO("24c02", 0x50),
        },
#if (ENABLE_I2C_TLV320AIC23)
		{
                I2C_BOARD_INFO("tlv320aic23", 0x1a),
        },
#endif /* ENABLE_I2C_TLV320AIC23 */
};

static struct i2c_board_info __initdata tam3517_i2c2_boardinfo[] = {
	/*[TN comment]
        {
                I2C_BOARD_INFO("24c02", 0x50),
        },
	*/
};

static struct i2c_board_info __initdata tam3517_i2c3_boardinfo[] = {
	/*[TN comment]
        {
                I2C_BOARD_INFO("s35390a", 0x30),
                .type           = "s35390a",
        },
	*/
#if (ENABLE_I2C_DS1307)
		{
                I2C_BOARD_INFO("ds1307", 0x68),
        },
#endif /* ENABLE_I2C_DS1307 */
#if (ENABLE_I2C_M41T65)
		{
                I2C_BOARD_INFO("m41t65", 0x68),
        },
#endif /* ENABLE_I2C_M41T80 */
};


static int __init tam3517_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, tam3517_i2c1_boardinfo,
			ARRAY_SIZE(tam3517_i2c1_boardinfo));
	omap_register_i2c_bus(2, 400, tam3517_i2c2_boardinfo,
			ARRAY_SIZE(tam3517_i2c2_boardinfo));
	omap_register_i2c_bus(3, 100, tam3517_i2c3_boardinfo,
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
//	OMAP3_MUX(CHASSIS_DMAREQ3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN), /* spi */
//	OMAP3_MUX(MCBSP_CLKS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
//	OMAP3_MUX(MCSPI1_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
//	OMAP3_MUX(MCSPI1_SIMO, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
//	OMAP3_MUX(MCSPI1_SOMI, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
//	OMAP3_MUX(MCSPI1_CS0, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
          
//	OMAP3_MUX(SDMMC2_DAT4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP), /* Touchscreen irq */
//	OMAP3_MUX(SDMMC2_DAT6, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* LCD */
//	OMAP3_MUX(SDMMC2_DAT7, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* LCD */
//	OMAP3_MUX(ETK_D10, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN), /* DVI */
//	OMAP3_MUX(GPMC_NCS2, OMAP_MUX_MODE4 | OMAP_PULL_UP), /* BL */
//	OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),
//	OMAP3_MUX(GPMC_NCS6, OMAP_MUX_MODE4 ),
//	OMAP3_MUX(GPMC_NCS7, OMAP_MUX_MODE4 | OMAP_PULL_UP),
//	OMAP3_MUX(MCBSP3_CLKX, OMAP_MUX_MODE4), /* GPIO 142 for keypad */

	/* ensure nCS and IRQ properly set for SMSC ethernet */
//	OMAP3_MUX(GPMC_NCS5, OMAP_MUX_MODE0),	// SMSC CS = 5
//	OMAP3_MUX(MCBSP3_CLKX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),  // SMSC RESET = 142

	/* MCBSP2 config */
//	OMAP3_MUX(MCBSP2_CLKX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
//	OMAP3_MUX(MCBSP2_FSX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
//	OMAP3_MUX(MCBSP2_DR, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
//	OMAP3_MUX(MCBSP2_DX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),

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

#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)

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
	.reset_gpio_port[2]  = -EINVAL,
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
#else
static __init void tam3517_usb_init(void)
{
	return;
}
#endif

/****************************************************************************
 *
 * HD01 Custom GPIO configuration
 *
 *  GPIO definitions: if name ends in '_', active LOW!
 ****************************************************************************/

// reset & power lines 
#define HD01__O_B_RESET_				171
#define HD01__O_C_RESET_				172
#define HD01__O_D_RESET_				173
#define HD01__O_E_RESET_				177
#define HD01__O_F_RESET_				176
#define HD01__O_MASTER_PWR_EN			175
#define HD01__I_MASTER_PWR_OK			174

// led outputs
#define HD01__O_POWER_STATUS_LED		93
#define HD01__O_TAM_STATUS_LED			92
#define HD01__O_RM1_LED					89
#define HD01__O_RM2_LED					90
#define HD01__O_RM3_LED					91

// unit identification switch inputs
#define HD01__I_SW1_1_BIT6				86
#define HD01__I_SW1_2_BIT5				85
#define HD01__I_SW1_3_BIT4				84
#define HD01__I_SW1_4_BIT3				83
#define HD01__I_FIXED_BIT2				82
#define HD01__I_FIXED_BIT1				81
#define HD01__I_FIXED_BIT0				80

// radio control gpios
#define HD01__O_ALL_RADIOS_DISABLE		79

#define HD01__I_RM1_STATUS_				66
#define HD01__I_RM2_STATUS_				67
#define HD01__I_RM3_STATUS_				68

#define HD01__O_RM1_IGNITION			106
#define HD01__O_RM1_RESET				105
#define HD01__I_RM1_INCOMING_CALL		104

#define HD01__O_RM2_IGNITION			78
#define HD01__O_RM2_RESET				73
#define HD01__I_RM2_INCOMING_CALL		72

#define HD01__O_RM3_IGNITION			103
#define HD01__O_RM3_RESET				102
#define HD01__I_RM3_INCOMING_CALL		101


static struct gpio hd01_gpios[] __initdata = {
	{ HD01__O_B_RESET_, GPIOF_OUT_INIT_HIGH, "HD01__O_B_RESET_" },
	{ HD01__O_C_RESET_, GPIOF_OUT_INIT_HIGH, "HD01__O_C_RESET_" },
	{ HD01__O_D_RESET_, GPIOF_OUT_INIT_HIGH, "HD01__O_D_RESET_" },
	{ HD01__O_E_RESET_, GPIOF_OUT_INIT_HIGH, "HD01__O_E_RESET_" },
	{ HD01__O_F_RESET_, GPIOF_OUT_INIT_HIGH, "HD01__O_F_RESET_" },
	{ HD01__O_MASTER_PWR_EN, GPIOF_OUT_INIT_HIGH, "HD01__O_MASTER_PWR_EN" },
	{ HD01__I_MASTER_PWR_OK, GPIOF_IN, "HD01__I_MASTER_PWR_OK" },
	\
	{ HD01__O_POWER_STATUS_LED, GPIOF_OUT_INIT_LOW, "HD01__O_POWER_STATUS_LED" },
	{ HD01__O_TAM_STATUS_LED, GPIOF_OUT_INIT_LOW, "HD01__O_TAM_STATUS_LED" },
	{ HD01__O_RM1_LED, GPIOF_OUT_INIT_LOW, "HD01__O_RM1_LED" },
	{ HD01__O_RM2_LED, GPIOF_OUT_INIT_LOW, "HD01__O_RM2_LED" },
	{ HD01__O_RM3_LED, GPIOF_OUT_INIT_LOW, "HD01__O_RM3_LED" },
	\
	{ HD01__I_SW1_1_BIT6, GPIOF_IN, "HD01__I_SW1_1_BIT6" },
	{ HD01__I_SW1_2_BIT5, GPIOF_IN, "HD01__I_SW1_2_BIT5" },
	{ HD01__I_SW1_3_BIT4, GPIOF_IN, "HD01__I_SW1_3_BIT4" },
	{ HD01__I_SW1_4_BIT3, GPIOF_IN, "HD01__I_SW1_4_BIT3" },
	{ HD01__I_FIXED_BIT2, GPIOF_IN, "HD01__I_FIXED_BIT2" },
	{ HD01__I_FIXED_BIT1, GPIOF_IN, "HD01__I_FIXED_BIT1" },
	{ HD01__I_FIXED_BIT0, GPIOF_IN, "HD01__I_FIXED_BIT0" },
	\
	{ HD01__O_ALL_RADIOS_DISABLE, GPIOF_OUT_INIT_LOW, "HD01__O_ALL_RADIOS_DISABLE" },
	{ HD01__I_RM1_STATUS_, GPIOF_IN, "HD01__I_RM1_STATUS_" },
	{ HD01__I_RM2_STATUS_, GPIOF_IN, "HD01__I_RM2_STATUS_" },
	{ HD01__I_RM3_STATUS_, GPIOF_IN, "HD01__I_RM3_STATUS_" },
	\
	{ HD01__O_RM1_IGNITION, GPIOF_OUT_INIT_LOW, "HD01__O_RM1_IGNITION" },
	{ HD01__O_RM1_RESET, GPIOF_OUT_INIT_LOW, "HD01__O_RM1_RESET" },
	{ HD01__I_RM1_INCOMING_CALL, GPIOF_IN, "HD01__I_RM1_INCOMING_CALL" },
	\
	{ HD01__O_RM2_IGNITION, GPIOF_OUT_INIT_LOW, "HD01__O_RM2_IGNITION" },
	{ HD01__O_RM2_RESET, GPIOF_OUT_INIT_LOW, "HD01__O_RM2_RESET" },
	{ HD01__I_RM2_INCOMING_CALL, GPIOF_IN, "HD01__I_RM2_INCOMING_CALL" },
	\
	{ HD01__O_RM3_IGNITION, GPIOF_OUT_INIT_LOW, "HD01__O_RM3_IGNITION" },
	{ HD01__O_RM3_RESET, GPIOF_OUT_INIT_LOW, "HD01__O_RM3_RESET" },
	{ HD01__I_RM3_INCOMING_CALL, GPIOF_IN, "HD01__I_RM3_INCOMING_CALL" },
};

static void __init hd01_gpios_init(void)
{
	if (gpio_request_array(hd01_gpios, ARRAY_SIZE(hd01_gpios))) {
		printk(KERN_ERR "failed to obtain HD01 control/display GPIOs\n");
		return;
	}

	// reset & power lines 
	if ( gpio_export(HD01__O_B_RESET_, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_B_RESET_'\n");
		return;
	}
	if ( gpio_export(HD01__O_C_RESET_, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_C_RESET_'\n");
		return;
	}
	if ( gpio_export(HD01__O_D_RESET_, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_D_RESET_'\n");
		return;
	}
	if ( gpio_export(HD01__O_E_RESET_, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_E_RESET_'\n");
		return;
	}
	if ( gpio_export(HD01__O_F_RESET_, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_F_RESET_'\n");
		return;
	}
	if ( gpio_export(HD01__O_MASTER_PWR_EN, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_MASTER_PWR_EN'\n");
		return;
	}
	if ( gpio_export(HD01__I_MASTER_PWR_OK, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__I_MASTER_PWR_OK'\n");
		return;
	}
	// led outputs
	if ( gpio_export(HD01__O_POWER_STATUS_LED, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_POWER_STATUS_LED'\n");
		return;
	}
	if ( gpio_export(HD01__O_TAM_STATUS_LED, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_TAM_STATUS_LED'\n");
		return;
	}
	if ( gpio_export(HD01__O_RM1_LED, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_RM1_LED'\n");
		return;
	}
	if ( gpio_export(HD01__O_RM2_LED, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_RM2_LED'\n");
		return;
	}
	if ( gpio_export(HD01__O_RM3_LED, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_RM3_LED'\n");
		return;
	}
	// unit identification switch inputs
	if ( gpio_export(HD01__I_SW1_1_BIT6, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__I_SW1_1_BIT6'\n");
		return;
	}
	if ( gpio_export(HD01__I_SW1_2_BIT5, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__I_SW1_2_BIT5'\n");
		return;
	}
	if ( gpio_export(HD01__I_SW1_3_BIT4, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__I_SW1_3_BIT4'\n");
		return;
	}
	if ( gpio_export(HD01__I_SW1_4_BIT3, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__I_SW1_4_BIT3'\n");
		return;
	}
	if ( gpio_export(HD01__I_FIXED_BIT2, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__I_FIXED_BIT2'\n");
		return;
	}
	if ( gpio_export(HD01__I_FIXED_BIT1, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__I_FIXED_BIT1'\n");
		return;
	}
	if ( gpio_export(HD01__I_FIXED_BIT0, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__I_FIXED_BIT0'\n");
		return;
	}
	// radio control gpios
	if ( gpio_export(HD01__O_ALL_RADIOS_DISABLE, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_ALL_RADIOS_DISABLE'\n");
		return;
	}
	if ( gpio_export(HD01__I_RM1_STATUS_, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__I_RM1_STATUS_'\n");
		return;
	}
	if ( gpio_export(HD01__I_RM2_STATUS_, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__I_RM2_STATUS_'\n");
		return;
	}
	if ( gpio_export(HD01__I_RM3_STATUS_, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__I_RM3_STATUS_'\n");
		return;
	}

	if ( gpio_export(HD01__O_RM1_IGNITION, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_RM1_IGNITION'\n");
		return;
	}
	if ( gpio_export(HD01__O_RM1_RESET, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_RM1_RESET'\n");
		return;
	}
	if ( gpio_export(HD01__I_RM1_INCOMING_CALL, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__I_RM1_INCOMING_CALL'\n");
		return;
	}

	if ( gpio_export(HD01__O_RM2_IGNITION, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_RM2_IGNITION'\n");
		return;
	}
	if ( gpio_export(HD01__O_RM2_RESET, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_RM2_RESET'\n");
		return;
	}
	if ( gpio_export(HD01__I_RM2_INCOMING_CALL, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__I_RM2_INCOMING_CALL'\n");
		return;
	}
	
	if ( gpio_export(HD01__O_RM3_IGNITION, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_RM3_IGNITION'\n");
		return;
	}
	if ( gpio_export(HD01__O_RM3_RESET, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__O_RM3_RESET'\n");
		return;
	}
	if ( gpio_export(HD01__I_RM3_INCOMING_CALL, 0) < 0 )
	{
		printk(KERN_ERR "gpio failed to export 'HD01__I_RM3_INCOMING_CALL'\n");
		return;
	}
	
	printk(KERN_INFO "HD01 control/display GPIOs initialized\n");
}


/* --------------------------------------------------------- */

static struct omap_board_config_kernel tam3517_config[] = {};

/* --------------------------------------------------------- */
static struct platform_device *tam3517_devices[] __initdata = {
	/*Ethernet:  SMSC911x */
#if (ENABLE_SMSC_ETH) && !(USE_EXTERNAL_INIT__SMSC_ETH) && ( defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE) )
	&tam3517_smsc911x_device,
#endif
#if (ENABLE_HECC) && ( defined(CONFIG_CAN_TI_HECC) || defined(CONFIG_CAN_TI_HECC_MODULE) )
	&tam3517_hecc_device,
#endif
	/*Ethernet:  DaVinci EMAC */
#if (ENABLE_EMAC_ETH) && !(USE_EXTERNAL_INIT__EMAC_ETH)
	&tam3517_mdio_device,
	&tam3517_emac_device,
#endif

#if (TAM3517_ENABLE_DUAL_UART)
	&tam3517_serial_device
#endif
};

/* ------------------------------------------------------------------- */

static void __init tam3517_init(void) {

#if (TAM3517_ENABLE_DUAL_UART)
	tam3517_init_dualuart(); // this must be called before platform_add_devices
#endif

	platform_add_devices(tam3517_devices, ARRAY_SIZE(tam3517_devices));
	omap_board_config = tam3517_config;
	omap_board_config_size = ARRAY_SIZE(tam3517_config);
	
	omap3_mux_init(tam3517_mux, OMAP_PACKAGE_CBB);	// [CL switched to OMAP_PACKAGE_CBB or CUS?]
	omap_serial_init();
/*	omap_sdrc_init(NULL, NULL);  // [CL & am3517evm adds this] */
	tam3517_i2c_init();
        
	omap_hsmmc_init(mmc);
        
	tam3517_usb_init();        
	tam3517_nand_init();

	/*Ethernet:  SMSC911x */
#if (ENABLE_SMSC_ETH)
	tam3517_init_smsc911x();
#endif

	/*Ethernet:  DaVinci EMAC */
#if (ENABLE_EMAC_ETH)
#if (USE_EXTERNAL_INIT__EMAC_ETH)
	am35xx_emac_init(AM35XX_DEFAULT_MDIO_FREQUENCY, 1);
#else
	tam3517_emac_ethernet_init();
#endif
#endif
	hd01_gpios_init();
}

MACHINE_START(TAM3517, "Technexion TAM3517")
	.atag_offset	= 0x100,
	.reserve		= omap_reserve,
	.map_io			= omap3_map_io,
	.init_early		= am35xx_init_early,
	.init_irq		= omap3_init_irq,
	.handle_irq		= omap3_intc_handle_irq,
	.init_machine	= tam3517_init,
	.timer			= &omap3_timer,
	.restart		= omap_prcm_restart,
MACHINE_END
