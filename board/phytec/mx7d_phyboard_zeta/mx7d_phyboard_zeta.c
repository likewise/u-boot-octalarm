/*
 * Copyright (C) 2015-2017 PHYTEC America, LLC
 *
 * Based on board/freescale/mx7dsabresd/mx7dsabresd.c
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx7-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/io.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>
#include <power/pmic.h>
#include <power/pfuze3000_pmic.h>
#include "../../freescale/common/pfuze.h"
#include <asm/arch/crm_regs.h>
#include <asm/imx-common/video.h>

#define NUM_SUPPORTED_VARIATIONS 3
static const char * const board_variations[NUM_SUPPORTED_VARIATIONS] = {
	"failsafe",						/* 0: Minimal */
	"PBA-C-09 Plotech display",		/* 1: Development kit */
	"ADS4500A",				/* 2: ADS4500A */
};
DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_DSE_3P3V_49OHM | \
	PAD_CTL_PUS_PU100KOHM | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_PU100KOHM | PAD_CTL_DSE_3P3V_49OHM)
#define ENET_PAD_CTRL_MII  (PAD_CTL_DSE_3P3V_32OHM)

#define ENET_RX_PAD_CTRL  (PAD_CTL_PUS_PU100KOHM | PAD_CTL_DSE_3P3V_49OHM)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_PU100KOHM | \
	PAD_CTL_DSE_3P3V_49OHM)

#define QSPI_PAD_CTRL	\
	(PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_PUE | PAD_CTL_PUS_PU47KOHM)

#define NAND_PAD_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_SRE_SLOW | PAD_CTL_HYS)

#define NAND_PAD_READY0_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_PUS_PU5KOHM)


int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
}

static iomux_v3_cfg_t const wdog_pads[] = {
	MX7D_PAD_GPIO1_IO00__WDOG1_WDOG_B | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const uart_console_pads[] = {
#if (CONFIG_CONS_INDEX == 1)
	MX7D_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
#elif (CONFIG_CONS_INDEX == 5)
	MX7D_PAD_GPIO1_IO07__UART5_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_GPIO1_IO06__UART5_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_I2C3_SDA__UART5_DCE_RTS | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_I2C3_SCL__UART5_DCE_CTS | MUX_PAD_CTRL(UART_PAD_CTRL),
#else
#warning Please set CONFIG_CONS_INDEX to a supported value
#endif
};

#ifdef CONFIG_NAND_MXS
static iomux_v3_cfg_t const gpmi_pads[] = {
	MX7D_PAD_SD3_DATA0__NAND_DATA00 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA1__NAND_DATA01 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA2__NAND_DATA02 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA3__NAND_DATA03 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA4__NAND_DATA04 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA5__NAND_DATA05 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA6__NAND_DATA06 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA7__NAND_DATA07 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_CLK__NAND_CLE	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_CMD__NAND_ALE	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_STROBE__NAND_RE_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_RESET_B__NAND_WE_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_MCLK__NAND_WP_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_RX_BCLK__NAND_CE3_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_RX_SYNC__NAND_CE2_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_RX_DATA__NAND_CE1_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_TX_BCLK__NAND_CE0_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_TX_SYNC__NAND_DQS	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_TX_DATA__NAND_READY_B	| MUX_PAD_CTRL(NAND_PAD_READY0_CTRL),
};

static void setup_gpmi_nand(void)
{
	imx_iomux_v3_setup_multiple_pads(gpmi_pads, ARRAY_SIZE(gpmi_pads));

	/*
	 * NAND_USDHC_BUS_CLK is set in rom
	 */

	set_clk_nand();

	/*
	 * APBH clock root is set in init_esdhc, USDHC3_CLK.
	 * There is no clk gate for APBHDMA.
	 * No touch here.
	 */
}
#endif /* CONFIG_NAND_MXS */

/* @TODO WIP meld freescale/mx7dsabresd/mx7dsabresd.c phytec/mx7d_phyboard_zeta/mx7d_phyboard_zeta.c */
#ifdef CONFIG_VIDEO_MXS
static iomux_v3_cfg_t const lcd_pads[] = {
	MX7D_PAD_LCD_CLK__LCD_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_ENABLE__LCD_ENABLE | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_HSYNC__LCD_HSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_VSYNC__LCD_VSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA00__LCD_DATA0 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA01__LCD_DATA1 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA02__LCD_DATA2 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA03__LCD_DATA3 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA04__LCD_DATA4 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA05__LCD_DATA5 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA06__LCD_DATA6 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA07__LCD_DATA7 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA08__LCD_DATA8 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA09__LCD_DATA9 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA10__LCD_DATA10 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA11__LCD_DATA11 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA12__LCD_DATA12 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA13__LCD_DATA13 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA14__LCD_DATA14 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA15__LCD_DATA15 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA16__LCD_DATA16 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA17__LCD_DATA17 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA18__LCD_DATA18 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA19__LCD_DATA19 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA20__LCD_DATA20 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA21__LCD_DATA21 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA22__LCD_DATA22 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA23__LCD_DATA23 | MUX_PAD_CTRL(LCD_PAD_CTRL),

	MX7D_PAD_LCD_RESET__GPIO3_IO4	| MUX_PAD_CTRL(LCD_PAD_CTRL),
};

static iomux_v3_cfg_t const pwm_pads[] = {
	/* Use GPIO for Brightness adjustment, duty cycle = period */
	MX7D_PAD_GPIO1_IO03__PWM3_OUT | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX7D_PAD_EPDC_DATA15__GPIO2_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

void do_enable_parallel_lcd(struct display_info_t const *dev)
{
	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

	imx_iomux_v3_setup_multiple_pads(pwm_pads, ARRAY_SIZE(pwm_pads));

	/* Reset LCD */
	gpio_request(IMX_GPIO_NR(3, 4), "lcd reset");
	gpio_direction_output(IMX_GPIO_NR(3, 4) , 0);
	udelay(500);
	gpio_direction_output(IMX_GPIO_NR(3, 4) , 1);

	/* De-assert Display Force-Off */
	gpio_request(IMX_GPIO_NR(2, 15), "lcd disable");
	gpio_direction_output(IMX_GPIO_NR(2, 15) , 1);

	/* Set Brightness to high */
	gpio_request(IMX_GPIO_NR(1, 3), "lcd backlight");
	gpio_direction_output(IMX_GPIO_NR(1, 3) , 1);
}

struct display_info_t const displays[] = {{
	.bus = ELCDIF1_IPS_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = NULL,
	.enable	= do_enable_parallel_lcd,
	.mode	= {
		.name			= "KWH070KQ20F01",
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 30006,
		.left_margin    = 26,
		.right_margin   = 210,
		.upper_margin   = 17,
		.lower_margin   = 23,
		.hsync_len      = 20,
		.vsync_len      = 6,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);
#endif

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart_console_pads, ARRAY_SIZE(uart_console_pads));
}

#ifdef CONFIG_FSL_QSPI
#ifndef CONFIG_DM_SPI
static iomux_v3_cfg_t const quadspi_pads[] = {
	MX7D_PAD_EPDC_DATA00__QSPI_A_DATA0 | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	MX7D_PAD_EPDC_DATA01__QSPI_A_DATA1 | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	MX7D_PAD_EPDC_DATA02__QSPI_A_DATA2 | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	MX7D_PAD_EPDC_DATA03__QSPI_A_DATA3 | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	MX7D_PAD_EPDC_DATA05__QSPI_A_SCLK  | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	MX7D_PAD_EPDC_DATA06__QSPI_A_SS0_B | MUX_PAD_CTRL(QSPI_PAD_CTRL),
};
#endif

int board_qspi_init(void)
{
#ifndef CONFIG_DM_SPI
	/* Set the iomux */
	imx_iomux_v3_setup_multiple_pads(quadspi_pads, ARRAY_SIZE(quadspi_pads));
#endif

	/* Set the clock */
	set_clk_qspi();

	return 0;
}
#endif

#ifdef CONFIG_FSL_ESDHC
int board_mmc_get_env_dev(int devno)
{
	if (devno == 2)
		devno--;

	return devno;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	if (1 == dev_no)
		dev_no++;

	return dev_no;
}

#endif

#ifdef CONFIG_FEC_MXC
static int setup_fec(int fec_id)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;

	if (0 == fec_id) {
		/* Use 125M anatop REF_CLK1 for ENET1, clear gpr1[13], gpr1[17]*/
		clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
			(IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK |
			 IOMUXC_GPR_GPR1_GPR_ENET1_CLK_DIR_MASK), 0);

	} else {
		/* Use 125M anatop REF_CLK2 for ENET2, clear gpr1[14], gpr1[18]*/
		clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
			(IOMUXC_GPR_GPR1_GPR_ENET2_TX_CLK_SEL_MASK |
			 IOMUXC_GPR_GPR1_GPR_ENET2_CLK_DIR_MASK), 0);
	}

	return set_clk_enet(ENET_125MHz);
}


int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_FEC_MXC
	setup_fec(CONFIG_FEC_ENET_DEV);
#endif

#ifdef CONFIG_NAND_MXS
	setup_gpmi_nand();
#endif

#ifdef CONFIG_FSL_QSPI
	board_qspi_init();
#endif
	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd1", MAKE_CFGVAL(0x10, 0x10, 0x00, 0x00)},
	{"emmc", MAKE_CFGVAL(0x10, 0x2a, 0x00, 0x00)},
	/* TODO: Nand */
	{"qspi", MAKE_CFGVAL(0x00, 0x40, 0x00, 0x00)},
	{NULL,   0},
};
#endif

#ifdef CONFIG_DM_PMIC
int power_init_board(void)
{
	struct udevice *dev;
	int ret, dev_id, rev_id;

	ret = pmic_get("PFUZE3000", &dev);
	if (ret == -ENODEV)
		return 0;
	if (ret != 0)
		return ret;

	dev_id = pmic_reg_read(dev, PFUZE3000_DEVICEID);
	rev_id = pmic_reg_read(dev, PFUZE3000_REVID);
	printf("PMIC: PFUZE3000 DEV_ID=0x%x REV_ID=0x%x\n", dev_id, rev_id);

	pmic_clrsetbits(dev, PFUZE3000_LDOGCTL, 0, 1);

	return 0;
}
#endif

int board_late_init(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));
	set_wdog_reset(wdog);

	return 0;
}

int misc_init_r(void)
{

	if (!getenv("ethaddr")) {
		uchar buf[6];
		uchar ifm_oui[3] = { 0, 2, 1, };
		int ret;

		/* I2C-0 for on-board eeprom */
		i2c_set_bus_num(CONFIG_SYS_I2C_EEPROM_BUS_NUM);

		/* Read ethaddr from EEPROM */
		ret = i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR,
			       CONFIG_SYS_I2C_EEPROM_MAC_OFFSET,
			       CONFIG_SYS_I2C_EEPROM_ADDR_LEN, buf, 6);
		if (ret != 0) {
			printf("Error: Unable to read MAC from I2C"
				" EEPROM at address %02X:%02X\n",
				CONFIG_SYS_I2C_EEPROM_ADDR,
				CONFIG_SYS_I2C_EEPROM_MAC_OFFSET);
			return 1;
		}

		eth_setenv_enetaddr("ethaddr", buf);
	}

	return 0;
}

u32 get_board_rev(void)
{
	uchar buf[1];
	/* I2C-0 for on-board eeprom */
	i2c_set_bus_num(CONFIG_SYS_I2C_EEPROM_BUS_NUM);
	if(i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR,
			   CONFIG_SYS_I2C_EEPROM_MAC_OFFSET,
			   CONFIG_SYS_I2C_EEPROM_ADDR_LEN, buf, 1) != 0)
	{
		printf("Error: Unable to read board revision from I2C"
			" EEPROM at address %02X:%02X\n",
				CONFIG_SYS_I2C_EEPROM_ADDR,
				CONFIG_SYS_I2C_EEPROM_BOARD_REV_OFFSET);
		return get_cpu_rev();
	} else {
		return buf[0];
	}
}

int checkboard(void)
{
	char *mode;
	/* Failsafe board */
	uchar boardID = 0;
	uchar buf[4];

	if (IS_ENABLED(CONFIG_ARMV7_BOOT_SEC_DEFAULT))
		mode = "secure";
	else
		mode = "non-secure";

	i2c_set_bus_num(CONFIG_SYS_I2C_EEPROM_BUS_NUM);
	if(i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR,
			   CONFIG_SYS_I2C_EEPROM_MAGIC_OFFSET,
			   CONFIG_SYS_I2C_EEPROM_ADDR_LEN, buf, 4) != 0)
	{
		printf("Error: Unable to read board variation from I2C"
			" EEPROM at address %02X\n",
				CONFIG_SYS_I2C_EEPROM_ADDR);
	} else {
		if((buf[CONFIG_SYS_I2C_EEPROM_MAGIC_OFFSET] != CONFIG_SYS_I2C_EEPROM_MAGIC) ||
			(buf[CONFIG_SYS_I2C_EEPROM_REVISION_OFFSET] != CONFIG_SYS_I2C_EEPROM_REVISION) ||
			(buf[CONFIG_SYS_I2C_EEPROM_BOARD_ID_OFFSET] >= NUM_SUPPORTED_VARIATIONS))
		{
			printf("Error: Incompatible board variation info from I2C "
				" EEPROM at address %02X\n",
				CONFIG_SYS_I2C_EEPROM_ADDR);
		} else {
			boardID = buf[CONFIG_SYS_I2C_EEPROM_BOARD_ID_OFFSET];
		}
	}
	setenv_hex("boardID", boardID);

	printf("Board: PHYTEC phyBOARD-Zeta i.MX7%c in %s mode on %s\n",
		is_cpu_type(MXC_CPU_MX7D) ? 'D' : 'S', mode, board_variations[boardID]);

	return 0;
}
