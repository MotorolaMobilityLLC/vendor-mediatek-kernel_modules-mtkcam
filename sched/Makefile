# SPDX-License-Identifier: GPL-2.0
#
# Copyright (C) 2023 MediaTek Inc.
#

include $(wildcard $(KERNEL_SRC)/$(DEVICE_MODULES_REL_DIR)/Makefile.include)

EXTRA_SYMBOLS += $(wildcard $(OUT_DIR)/../vendor/mediatek/kernel_modules/mtkcam/sched/c2ps_ioctl/*Module.symvers)

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)
