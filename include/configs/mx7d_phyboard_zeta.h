/*
 * Copyright (C) 2017 PHYTEC America, LLC
 *
 * Configuration settings for the PHYTEC i.MX7D phyBOARD-Zeta.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX7D_PHYBOARD_ZETA_CONFIG_H
#define __MX7D_PHYBOARD_ZETA_CONFIG_H

#include "mx7_common.h"

/*
#define CONFIG_ROM_UNIFIED_SECTIONS
#define CONFIG_SYS_GENERIC_BOARD
*/

#define CONFIG_DBG_MONITOR
/* uncomment for PLUGIN mode support */
/* #define CONFIG_USE_PLUGIN */

/* uncomment for SECURE mode support */
/* #define CONFIG_SECURE_BOOT */

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(32 * SZ_1M)

#ifdef CONFIG_SECURE_BOOT
#ifndef CONFIG_CSF_SIZE
#define CONFIG_CSF_SIZE 0x4000
#endif
#endif

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#define CONFIG_MXC_UART

#undef CONFIG_MXC_UART_BASE
#define CONFIG_MXC_UART_BASE		UART1_IPS_BASE_ADDR

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#undef CONFIG_CONS_INDEX
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

#ifdef CONFIG_PHYTEC_MT41K64M16TW107IT
#define PHYS_SDRAM_SIZE			SZ_256M
#elif defined CONFIG_PHYTEC_MT41K256M16TW107IT
#define PHYS_SDRAM_SIZE			SZ_1G
#else
#define PHYS_SDRAM_SIZE			SZ_1G
#endif

/* Network */
#ifdef CONFIG_DM_ETH
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define CONFIG_FEC_XCV_TYPE             RGMII
#define CONFIG_FEC_ENET_DEV             0

#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL

/* ENET1 */
#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE			ENET_IPS_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR		1
#ifdef CONFIG_DM_ETH
#define CONFIG_ETHPRIME			"eth0"
#else
#define CONFIG_ETHPRIME			"FEC0"
#endif
#elif (CONFIG_FEC_ENET_DEV == 1)
#define IMX_FEC_BASE			ENET2_IPS_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR		2
#ifdef CONFIG_DM_ETH
#define CONFIG_ETHPRIME			"eth1"
#else
#define CONFIG_ETHPRIME			"FEC1"
#endif
#endif

#define CONFIG_FEC_MXC_MDIO_BASE	ENET_IPS_BASE_ADDR
#endif

#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_PLAN9
#undef CONFIG_BOOTM_RTEMS

#define CONFIG_CMD_IMPORTENV

#define CONFIG_MISC_INIT_R		/* Call misc_init_r		*/

#ifdef CONFIG_DISPLAY_BOARDINFO
#undef CONFIG_DISPLAY_BOARDINF
#endif
#define CONFIG_DISPLAY_BOARDINFO_LATE

/* I2C configs */
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_DM_I2C_COMPAT

#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN  2
#define CONFIG_SYS_I2C_EEPROM_ADDR      0x50
#define CONFIG_SYS_I2C_EEPROM_BUS_NUM   0

#define CONFIG_SYS_I2C_EEPROM_MAGIC_OFFSET		0x00
#	define CONFIG_SYS_I2C_EEPROM_MAGIC			0xAD
#define CONFIG_SYS_I2C_EEPROM_REVISION_OFFSET	0x01
#	define CONFIG_SYS_I2C_EEPROM_REVISION		0x01
#define CONFIG_SYS_I2C_EEPROM_BOARD_ID_OFFSET	0x02
#define CONFIG_SYS_I2C_EEPROM_BOARD_REV_OFFSET	0x03
#define CONFIG_SYS_I2C_EEPROM_MAC_OFFSET		0x10

/* Implement bootcounting in the (unused) rtc to support fallback */
#define CONFIG_BOOTCOUNT_LIMIT
#define CONFIG_BOOTCOUNT_I2C
#define CONFIG_BOOTCOUNT_ALEN		1
#define CONFIG_SYS_I2C_RTC_ADDR		0x68
#define CONFIG_SYS_BOOTCOUNT_ADDR	0x0D

#define CONFIG_BOOTLIMIT_ENV \
	"bootlimit=3\0" \
	"altbootcmd=echo Boot failed to often... trying fallback; setenv fallback true; run bootcmd\0"

#ifdef CONFIG_FSL_QSPI
#define CONFIG_SYS_AUXCORE_BOOTDATA 0x60100000 /* Set to QSPI1 A flash at default */
#else
#define CONFIG_SYS_AUXCORE_BOOTDATA 0x7F8000 /* Set to TCML address */
#endif

#ifdef CONFIG_IMX_BOOTAUX
#define UPDATE_M4_ENV \
	"m4image=m4_qspi.bin\0" \
	"loadm4image=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${m4image}\0" \
	"update_m4_from_sd=" \
		"if sf probe 0:0; then " \
			"if run loadm4image; then " \
				"setexpr fw_sz ${filesize} + 0xffff; " \
				"setexpr fw_sz ${fw_sz} / 0x10000; "	\
				"setexpr fw_sz ${fw_sz} * 0x10000; "	\
				"sf erase 0x0 ${fw_sz}; " \
				"sf write ${loadaddr} 0x0 ${filesize}; " \
			"fi; " \
		"fi\0" \
	"m4boot=sf probe 0:0; bootaux "__stringify(CONFIG_SYS_AUXCORE_BOOTDATA)"\0"
#else
#define UPDATE_M4_ENV ""
#endif

#define CONFIG_SUPPORT_EMMC_BOOT		/* eMMC specific */
#define CONFIG_SYS_MMC_IMG_LOAD_PART	1

/*
 * boardID and boardRev should be overriden from boot code
 */
#define CONFIG_OSTREE_ENV_SETTINGS \
	"kernel_addr_r=0x81000000\0" \
	"ostree_partition=1\0" \
	"ostree_device=/dev/mmcblk0p\0" \
	"boardID=0\0" \
	"boardRev=0\0" \
	"bootcmd_otenv=ext4load mmc ${mmcdev}:${ostree_partition} $loadaddr /boot/loader/uEnv.txt; env import -t $loadaddr $filesize\0" \
	"bootcmd_args=setenv ostree_root ${ostree_device}${ostree_partition}; " \
		"setenv bootargs $bootargs $bootargs_fdt ostree_root=${ostree_root} root=${ostree_root} rw rootwait rootdelay=2 console=$console,$baudrate\0" \
	"bootcmd_load=if test '${fallback}' = true; then " \
		"ext2load mmc ${mmcdev}:${ostree_partition} $kernel_addr_r /boot${kernel_image2}; " \
	"else " \
		"ext2load mmc ${mmcdev}:${ostree_partition} $kernel_addr_r /boot${kernel_image}; " \
	"fi;\0" \
	"bootcmd_run=bootm ${kernel_addr_r}#conf@imx7d-octalarm-${boardID}.${boardRev}.dtb; " \
		"bootm ${kernel_addr_r}#conf@imx7d-octalarm-${boardID}.dtb; " \
		"bootm ${kernel_addr_r}#conf@imx7d-octalarm-0.dtb;\0" \
	"bootcmd_ostree=mmc dev ${mmcdev}; mmc rescan; run bootcmd_otenv; run bootcmd_splash; run bootcmd_args; run bootcmd_load; run bootcmd_run\0" \
	"bootcmd_extract=setexpr rootdir gsub \"ostree=([^ ]*)(.*)\" \"\\\\\\\\1\" \"${bootargs}\"\0" \
	"bootcmd_splash=run bootcmd_extract; ext4load mmc ${mmcdev}:${ostree_partition} $loadaddr ${rootdir}/usr/share/boot-splash-images/splash.bmp; bmp display ${loadaddr}\0"

/* setexpr rootdir gsub "ostree=([^ ]*)(.*)" "\1" "${bootargs}" */


/* @TODO compare checksums after read (store in mem, do compare) */
#define TFTP_PROGRAM_EMMC_ENV \
	"serverip=192.168.2.17\0" \
	"tftp_prog_emmc=dhcp ${loadaddr} ${serverip}:imx7/emmc.img; " \
	"mmc dev 1; mmc rescan; setexpr blocks ${filesize} + 1ff; setexpr blocks ${blocks} / 200; " \
	"mmc write ${loadaddr} 0 ${blocks}; mw.b ${loadaddr} a5 ${filesize}; " \
	"mmc read ${loadaddr} 0 ${blocks}; crc32 ${loadaddr} ${filesize};\0" \
	"tftp_prog_boot=dhcp ${loadaddr} ${serverip}:imx7/u-boot.imx; " \
	"mmc dev 1; mmc rescan; setexpr blocks ${filesize} + 1ff; setexpr blocks ${blocks} / 200; " \
	"mmc write ${loadaddr} 2 ${blocks}; mw.b ${loadaddr} a5 ${filesize}; " \
	"mmc read ${loadaddr} 2 ${blocks}; crc32 ${loadaddr} ${filesize};\0"

#define CONFIG_MFG_ENV_SETTINGS \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
		"rdinit=/linuxrc " \
		"g_mass_storage.stall=0 g_mass_storage.removable=1 " \
		"g_mass_storage.idVendor=0x066F g_mass_storage.idProduct=0x37FF "\
		"g_mass_storage.iSerialNumber=\"\" "\
		"clk_ignore_unused "\
		"\0" \
	"initrd_addr=0x83800000\0" \
	"initrd_high=0xffffffff\0" \
	"bootcmd_mfg=run mfgtool_args;bootz ${loadaddr} ${initrd_addr} ${fdt_addr};\0"

#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS \
	TFTP_PROGRAM_EMMC_ENV \
	CONFIG_OSTREE_ENV_SETTINGS \
	UPDATE_M4_ENV \
	CONFIG_BOOTLIMIT_ENV \
	"script=boot.scr\0" \
	"image=zImage\0" \
	"console=ttymxc0\0" \
	"ip_dyn=yes\0" \
	"mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0" \
	"mmcpart=" __stringify(CONFIG_SYS_MMC_IMG_LOAD_PART) "\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0" \
	"mmcautodetect=yes\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"root=${mmcroot}\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"mmc dev ${mmcdev}; if mmc rescan; then " \
			"run mmcbootcmd; " \
		"fi;\0" \
	"mmcbootcmd=run loadimage; "\
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0"

#define CONFIG_BOOTCOMMAND 			"run bootcmd_ostree"

#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x20000000)

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

#define CONFIG_STACKSIZE		SZ_128K

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			SZ_8K

#ifdef CONFIG_QSPI_BOOT
#define CONFIG_FSL_QSPI
#define CONFIG_ENV_IS_IN_SPI_FLASH
#else
#define CONFIG_ENV_IS_IN_MMC
#endif

#ifdef CONFIG_FSL_QSPI
#define CONFIG_SYS_FSL_QSPI_AHB
#define QSPI0_BASE_ADDR		QSPI1_IPS_BASE_ADDR
#define QSPI0_AMBA_BASE		QSPI0_ARB_BASE_ADDR

#define	CONFIG_SPI_FLASH_STMICRO
#define	CONFIG_SPI_FLASH_BAR
#define	CONFIG_SF_DEFAULT_BUS		1
#define	CONFIG_SF_DEFAULT_CS		0
#define	CONFIG_SF_DEFAULT_SPEED		40000000
#define	CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#define FSL_QSPI_FLASH_NUM		1
#define FSL_QSPI_FLASH_SIZE		SZ_16M
#endif

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(14 * SZ_64K)
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		(896 * 1024)
#define CONFIG_ENV_SECT_SIZE		(64 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#endif

#define CONFIG_SYS_FSL_USDHC_NUM	2

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR	0
#define CONFIG_SYS_MMC_ENV_DEV		0   /* USDHC1 */
#define CONFIG_SYS_MMC_ENV_PART		0	/* user area */
#define CONFIG_MMCROOT			"/dev/mmcblk0p1"  /* USDHC1 */

#define CONFIG_CMD_BMODE

/* USB Configs */
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS   0
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2

#define CONFIG_IMX_THERMAL

#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_MXS
/*#define CONFIG_VIDEO_LOGO*/
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_CMD_BMP
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_VIDEO_SKIP
#endif

#endif	/* __CONFIG_H */
