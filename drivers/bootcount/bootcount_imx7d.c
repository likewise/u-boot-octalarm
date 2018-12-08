/*
 * (C) Copyright 2018
 * Leon Woestenberg, Sidebranch, leon@sidebranch.com
 *
 * A bootcount driver for the i.MX7D SNVS LP block. Note that the register may
 * not be reset on hard resets, depending on board design.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <bootcount.h>

#define SNVS_LPLR   0x30370034U
#define SNVS_LPCR   0x30370038U
#define SNVS_LPGPR0 0x30370090U

void bootcount_store(ulong a)
{
	unsigned long val;
	/* prevent zero of GPR upon tamper event */
	val = readl(SNVS_LPCR);
	val |= (1 << 24);
	writel(val, SNVS_LPCR);
	/* unlock GPR */
	val = readl(SNVS_LPLR);
	val &= ~(1 << 5);
	writel(val, SNVS_LPLR);
	/* write bootcount in LPGPR0 register */
	raw_bootcount_store(SNVS_LPGPR0,
			    (BOOTCOUNT_MAGIC & 0xffff0000) | (a & 0x0000ffff));
}

ulong bootcount_load(void)
{
	unsigned long val;
	/* read bootcount from LPGPR0 register */
	val = raw_bootcount_load(SNVS_LPGPR0);
	if ((val & 0xffff0000) != (BOOTCOUNT_MAGIC & 0xffff0000))
		return 0;
	else
		return val & 0x0000ffff;
}
