/*
 * watchdog.c - driver for i.mx on-chip watchdog
 *
 * Licensed under the GPL-2 or later.
 */

#include <common.h>
#include <asm/io.h>
#include <watchdog.h>
#include <asm/arch/imx-regs.h>
#include <fsl_wdog.h>

#ifdef CONFIG_IMX_WATCHDOG
void hw_watchdog_reset(void)
{
	struct watchdog_regs *wdog = (struct watchdog_regs *)WDOG1_BASE_ADDR;
	writew(0x5555, &wdog->wsr);
	writew(0xaaaa, &wdog->wsr);
}

void hw_watchdog_init(void)
{
	struct watchdog_regs *wdog = (struct watchdog_regs *)WDOG1_BASE_ADDR;
	u16 timeout;

	/*
	 * The timer watchdog can be set between
	 * 0.5 and 128 seconds. If not defined
	 * in configuration file, sets 128 seconds.
	 */
#ifndef CONFIG_WATCHDOG_TIMEOUT_MSECS
#define CONFIG_WATCHDOG_TIMEOUT_MSECS 128000
#endif
	timeout = (CONFIG_WATCHDOG_TIMEOUT_MSECS / 500) - 1;
	writew(WCR_WDZST | WCR_WDBG | WCR_WDE | WCR_WDT | WCR_SRS |
		WCR_WDA | SET_WCR_WT(timeout), &wdog->wcr);
	hw_watchdog_reset();

	/* as we are now starting to pet the watchdog ourselves, we must
         * disable the watchdog auto powerdown timer, as it kicks in 16
	 * seconds after reset; we assume we are here within 16 seconds! */
	imx_set_wdog_powerdown();
}
#endif

void __attribute__((weak)) reset_cpu(ulong addr)
{
	struct watchdog_regs *wdog = (struct watchdog_regs *)WDOG1_BASE_ADDR;

	/* load minimum 1/2 second timeout */
	clrsetbits_le16(&wdog->wcr, WCR_WT_MSK/*0.5s*/, WCR_WDE);

	writew(0x5555, &wdog->wsr);
	writew(0xaaaa, &wdog->wsr);
        /* spin for 1/2 seconds before reset */
	while (1) {
		/*
		 */
	}
}
