#
# Makefile for the s3c framebuffer driver
#
ifeq ($(CONFIG_FB_S5P),y)
obj-$(CONFIG_FB_S5P_DUMMY_MIPI_LCD)	+= s3cfb_dummymipilcd.o
obj-$(CONFIG_FB_S5P_MIPI_DSIM)	+= s5p_dsim_lowlevel.o
obj-$(CONFIG_FB_S5P_MIPI_DSIM)	+= s5p-dsim.o

obj-y				+= s3cfb.o
s3cfb-y				:= s3cfb_main.o s3cfb_ops.o

obj-$(CONFIG_ARCH_S5PV210)	+= s3cfb_fimd6x.o
obj-$(CONFIG_ARCH_EXYNOS4)	+= s3cfb_fimd6x.o

obj-$(CONFIG_FB_S5P_LTE480WV)	+= s3cfb_lte480wv.o
obj-$(CONFIG_FB_S5P_HT101HD1)	+= s3cfb_ht101hd1.o
obj-$(CONFIG_FB_S5P_WA101S)	+= s3cfb_wa101s.o
obj-$(CONFIG_FB_S5P_AMS369FG06)	+= s3cfb_ams369fg06.o
obj-$(CONFIG_FB_S5P_LMS501KF03)	+= s3cfb_lms501kf03.o
endif

ifeq ($(CONFIG_FB_S5P_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif

ifeq ($(CONFIG_FB_S5P_V2_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
