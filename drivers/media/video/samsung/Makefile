ifeq ($(CONFIG_VIDEO_S5K3H7),y)
obj-$(CONFIG_VIDEO_FIMC)	+= fimc/
else
obj-$(CONFIG_VIDEO_FIMC)	+= fimc_external_isp/
endif

obj-$(CONFIG_VIDEO_JPEG)	+= jpeg/

#костыль для сохранения фоток на хайскрине
ifeq ($(CONFIG_VIDEO_S5K3H7),y)
obj-$(CONFIG_VIDEO_JPEG_V2X)	+= jpeg_v2x-old/
else
obj-$(CONFIG_VIDEO_JPEG_V2X)	+= jpeg_v2x/
endif
#end

obj-$(CONFIG_VIDEO_MFC5X)	+= mfc5x/

ifeq ($(CONFIG_ARCH_EXYNOS4), y)
obj-$(CONFIG_VIDEO_FIMG2D3X)	+= fimg2d3x-exynos4/
obj-$(CONFIG_VIDEO_FIMG2D4X)	+= fimg2d4x-exynos4/
else
obj-$(CONFIG_VIDEO_FIMG2D3X)	+= fimg2d3x/
obj-$(CONFIG_VIDEO_FIMG2D4X)	+= fimg2d4x/
endif

obj-$(CONFIG_VIDEO_UMP)		+= ump/
obj-$(CONFIG_VIDEO_MALI400MP)	+= mali/

EXTRA_CFLAGS += -Idrivers/media/video
