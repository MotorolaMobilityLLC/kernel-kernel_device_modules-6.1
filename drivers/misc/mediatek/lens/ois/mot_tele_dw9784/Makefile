#
# Makefile for misc devices that really don't fit anywhere else.
#

OIS_DRIVER_PATH  := $(DEVICE_MODULES_PATH)/drivers/misc/mediatek

subdir-ccflags-y += -I$(OIS_DRIVER_PATH)/lens/ois/mot_tele_dw9784/

ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/sensor/2.0/core

obj-$(CONFIG_MTK_V4L2_LENS) += mot_tele_dw9784.o
mot_tele_dw9784-objs :=
mot_tele_dw9784-objs += mot_tele_dw9784_ois.o
mot_tele_dw9784-objs += mot_tele_dw9784_i2c.o

$(info mot_tele_dw9784-objs : $(mot_tele_dw9784-objs))
#$(info dw9784 subdir-ccflags-y : $(subdir-ccflags-y))
