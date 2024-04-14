# SPDX-License-Identifier: GPL-2.0
#
# Copyright (c) 2021 MediaTek Inc.
#

KVER = @KVER@

ccflags-y += -I$(src)/
ccflags-y += -I$(srctree)/arch/arm/include/
ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/aee/mrdump
obj-$(CONFIG_MTK_MKP_TEST) += test_mkp.o
test_mkp-$(CONFIG_MTK_MKP_TEST) += mkp_test.o
#test_mkp-$(CONFIG_MTK_MKP_TEST) += dummy_ksym.o
test_mkp-$(CONFIG_MTK_MKP_TEST) += aee_ksym.o

