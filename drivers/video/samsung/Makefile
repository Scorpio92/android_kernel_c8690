#
# Makefile for the s3c framebuffer driver
#
ifeq ($(CONFIG_FB_S5P),y)
obj-$(CONFIG_FB_S5P_DUMMY_MIPI_LCD)	+= s3cfb_dummymipilcd.o
obj-$(CONFIG_FB_S5P_MIPI_DSIM)	+= s5p_dsim_lowlevel.o
obj-$(CONFIG_FB_S5P_MIPI_DSIM)	+= s5p-dsim.o

obj-y				+= s3cfb.o
s3cfb-y				:= s3cfb_main.o s3cfb_ops.o lcdfreq.o

obj-$(CONFIG_ARCH_S5PV210)	+= s3cfb_fimd6x.o
obj-$(CONFIG_ARCH_EXYNOS4)	+= s3cfb_fimd6x.o

obj-$(CONFIG_FB_S5P_MDNIE)      += s3cfb_mdnie.o s3cfb_ielcd.o mdnie.o mdnie_tunning.o
obj-$(CONFIG_FB_S5P_MDNIE_CONTROL)      += mdnie_control.o
endif

ifeq ($(CONFIG_FB_S5P_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif


