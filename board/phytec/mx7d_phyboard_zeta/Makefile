# (C) Copyright 2015 PHYTEC America, LLC
#
# Based on board/freescale/mx7dsabresd/Makefile
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y  := mx7d_phyboard_zeta.o

extra-$(CONFIG_USE_PLUGIN) :=  plugin.bin
$(obj)/plugin.bin: $(obj)/plugin.o
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@
