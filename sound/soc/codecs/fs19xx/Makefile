# MTK include path
subdir-ccflags-y += -I$(srctree)/sound/soc/mediatek/audio_dsp
subdir-ccflags-y += -I$(srctree)/sound/soc/mediatek/common

# debugging support (also enables trace_printk)
ccflags-y += -DDEBUG -DTFA_NON_DSP_SOLUTION
ccflags-y += -Werror
ccflags-y += $(call cc-option,-Wno-date-time,)

snd-soc-fs19xx-objs := fs18yn.o fs1958.o fsm_core.o fsm_i2c.o
obj-$(CONFIG_SND_SOC_FS1XXX) += snd-soc-fs19xx.o
