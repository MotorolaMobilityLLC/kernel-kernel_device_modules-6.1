# SPDX-License-Identifier: GPL-2.0
# UFSHCD makefile
ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include/
ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/blocktag/
ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/power_throttling/
ifeq ($(CONFIG_UFSHID_V3),y)
ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/ufs/v3
endif
ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/ufs/vendor
ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/ufs/
ccflags-y += -I$(srctree)/drivers/ufs/core/
ccflags-y += -I$(srctree)/drivers/devfreq/
ccflags-y += -I$(srctree)/kernel/

obj-$(CONFIG_DEVICE_MODULES_SCSI_UFS_MEDIATEK) += vendor/
obj-$(CONFIG_SCSI_UFS_MEDIATEK_DBG) += ufs-mediatek-dbg.o
