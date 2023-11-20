#
# Makefile for misc devices that really don't fit anywhere else.
#

OIS_DRIVER_PATH  := $(DEVICE_MODULES_PATH)/drivers/misc/mediatek

subdir-ccflags-y += -I$(OIS_DRIVER_PATH)/lens/ois/dw9784/

obj-$(CONFIG_MTK_V4L2_LENS) += dw9784.o
dw9784-objs :=
dw9784-objs += dw9784_ois.o
dw9784-objs += dw9784_i2c.o

$(info dw9784-objs : $(dw9784-objs))
#$(info dw9784 subdir-ccflags-y : $(subdir-ccflags-y))
