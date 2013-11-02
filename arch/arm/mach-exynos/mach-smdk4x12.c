/* linux/arch/arm/mach-exynos/mach-smdk4x12.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/clk.h>
#include <linux/lcd.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/i2c.h>
#ifdef CONFIG_SNFC
#include <linux/nfc/s3fhrn2.h>
#include <linux/i2c-gpio.h>
#include <linux/pn544.h>
#endif
#include <linux/pwm_backlight.h>
#include <linux/input.h>
#include <linux/mmc/host.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/max8649.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/max8997.h>
#include <linux/mfd/max77686.h>
#include <linux/v4l2-mediabus.h>
#include <linux/memblock.h>
#include <linux/delay.h>
#if defined(CONFIG_S5P_MEM_CMA)
#include <linux/cma.h>
#endif
#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif
#ifdef CONFIG_BATTERY_MAX17040
#include <linux/max17040_battery.h>
#endif
#ifdef CONFIG_AAT3635
#include <linux/power/aat3635_battery.h>
#endif
#include <linux/smsc911x.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <plat/exynos4.h>
#include <plat/cpu.h>
#include <plat/clock.h>
#include <plat/keypad.h>
#include <plat/devs.h>
#include <plat/fb.h>
#include <plat/fb-s5p.h>
#include <plat/fb-core.h>
#include <plat/regs-fb-v4.h>
#include <plat/backlight.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-adc.h>
#include <plat/adc.h>
#include <plat/iic.h>
#include <plat/pd.h>
#include <plat/sdhci.h>
#include <plat/mshci.h>
#include <plat/ehci.h>
#include <plat/usbgadget.h>
#include <plat/s3c64xx-spi.h>
#if defined(CONFIG_VIDEO_FIMC)
#include <plat/fimc.h>
#elif defined(CONFIG_VIDEO_SAMSUNG_S5P_FIMC)
#include <plat/fimc-core.h>
#include <media/s5p_fimc.h>
#endif
#if defined(CONFIG_VIDEO_FIMC_MIPI)
#include <plat/csis.h>
#elif defined(CONFIG_VIDEO_S5P_MIPI_CSIS)
#include <plat/mipi_csis.h>
#endif
#include <plat/tvout.h>
#include <plat/media.h>
#include <plat/regs-srom.h>
#include <plat/tv-core.h>
#include <media/s5k4ba_platform.h>
#include <media/s5k4ea_platform.h>
#include <media/ov2675_platform.h>
#include <media/exynos_flite.h>
#include <media/exynos_fimc_is.h>
#include <video/platform_lcd.h>
#include <media/m5mo_platform.h>
#include <media/m5mols.h>
#include <mach/board_rev.h>
#include <mach/map.h>
#include <mach/spi-clocks.h>
#include <mach/exynos-ion.h>
#include <mach/regs-pmu.h>
#ifdef CONFIG_EXYNOS4_DEV_DWMCI
#include <mach/dwmci.h>
#endif
#ifdef CONFIG_SMM6260_MODEM
#include <mach/modem.h>
#endif
#include <mach/map.h>
#include <mach/regs-pmu.h>
#ifdef CONFIG_SOC_CAMERA_MT9D115
#include <media/mt9d115_platform.h>
#endif
#ifdef CONFIG_VIDEO_S5K4ECGX
#include <media/s5k4ecgx.h>
#define temp_width 640
#define temp_height 480
#endif

#include <mach/max8997.h>
#ifdef CONFIG_BATTERY_MAX8997
#include <linux/fg8997_battery.h>
#endif
#ifdef CONFIG_RTC_MAX8997
#include <linux/rtc-max8997.h>
#endif
#ifdef CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION
#include <mach/secmem.h>
#endif
#include <mach/dev.h>
#include <mach/ppmu.h>
#ifdef CONFIG_EXYNOS_C2C
#include <mach/c2c.h>
#endif
#if defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC) || defined(CONFIG_VIDEO_MFC5X)
#include <plat/s5p-mfc.h>
#endif

#ifdef CONFIG_FB_S5P_MIPI_DSIM
#include <mach/mipi_ddi.h>
#include <mach/dsim.h>
#include <../../../drivers/video/samsung/s3cfb.h>
#endif
#include <plat/fimg2d.h>
#ifdef CONFIG_TC4_GB
#include <mach/sysmmu.h>
#else
#include <mach/dev-sysmmu.h>
#include <plat/sysmmu.h>

#endif
#if defined(CONFIG_SENSOR_ST_LSM303DLHC)	// G/M sensor
#include <linux/i2c/lsm303dlhc.h>
#endif

#if defined(CONFIG_SENSOR_ST_L3G4200D)		// Gyroscope sensor
#include <linux/i2c/l3g4200d.h>
#endif

#if defined(CONFIG_SENSOR_ST_LSM330D)
#include <linux/i2c/lsm330dlc.h>
#endif

#if defined(CONFIG_SENSORS_AK8963)
#include <linux/akm8963.h>	
#endif

#if defined(CONFIG_SENSOR_ROHM_BH1721)		// Light sensor
#include <linux/i2c/rohm_bh1721.h>
#endif

#if defined(CONFIG_KERNEL_PANIC_DUMP)		//panic-dump
#include <mach/panic-dump.h>
#endif

#include <linux/i2c/max8997.h>
#include <mach/regs-clock.h>//kaixian@cellon for xclkout regsiter define
 // Cellon add start, ZePeng Wu, 2012/08/04, for TP 
#if defined(CONFIG_RMI4_BUS)
#include <linux/rmi.h>
#endif
// Cellon add end, ZePeng Wu, 2012/08/04, for TP 
extern  int ov2675_power_down2(void);
extern  int ov2675_power_down(void);

//Cellon add begin, charles hu, 2012/08/21 , for front camera
#define GPIO_CAM_MCLK    	EXYNOS4212_GPJ1(3)
#define GPIO_CAM_MEGA_EN	EXYNOS4212_GPM0(3)		//EXYNOS4_GPC0(1)	//EXYNOS4_GPX0(0)
#define GPIO_CAM_MEGA_nRST	EXYNOS4212_GPM0(2)	//EXYNOS4_GPX0(3)	//EXYNOS4_GPX1(6)
//#define GPIO_CAM_PCLK    EXYNOS4212_GPJ0(0)
//Cellon and end , charles hu, 2012/08/21 , for front camera

#ifdef CONFIG_SNFC
#define SNFC_I2C_ADDR		0x2a
#define SNFC_EINT_NUM		9
#endif

// Cellon add start, Ted Shi, 2012/08/06, for porting bcm4330 bt 
#ifdef CONFIG_BCMDHD_DEBUG
#define BCM_DBG(fmt, ...) printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define BCM_DBG(fmt, ...)
#endif
// Cellon add end, Ted Shi, 2012/08/06
#ifdef CONFIG_TC4_GB// liang
#include <linux/mpu.h>
#if defined(CONFIG_MPU_SENSORS_MPU3050) || defined(CONFIG_MPU_SENSORS_MPU3050_MODULE)

#define SENSOR_MPU_NAME "mpu3050"
#define MPUGPIO (EXYNOS4_GPX3(3))

static struct mpu3050_platform_data mpu_data = {
	.int_config  = 0x10,
#ifdef CONFIG_TC4_PORTRAIT_MODE
	.orientation = {  0,  1,  0, 
		1,  0,  0, 
		0,  0, -1 },
#else
	.orientation = {  -1,  0,  0, 
		0,  1,  0, 
		0,  0, -1 },
#endif
	/* accel */
	.accel = {
#ifdef CONFIG_MPU_SENSORS_MPU3050_MODULE
		.get_slave_descr = NULL,
#else
		.get_slave_descr = get_accel_slave_descr,
#endif
		.adapt_num   = 5,
		.bus         = EXT_SLAVE_BUS_SECONDARY,
		.address     = (0x30>>1),//0x0F,
#ifdef CONFIG_TC4_PORTRAIT_MODE
		.orientation = {  1,  0,  0, 
			0,  -1,  0, 
			0,  0, -1 },
	},
#else
		.orientation = {  0,  1,  0, 
			1,  0,  0, 
			0,  0, -1 },
	},
#endif
	/* compass */
	.compass = {
#ifdef CONFIG_MPU_SENSORS_MPU3050_MODULE
		.get_slave_descr = NULL,
#else
		.get_slave_descr = get_compass_slave_descr,
#endif
		.adapt_num   = 5,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
		.address     = (0x3c>>1),//0x0E,
#ifdef CONFIG_TC4_PORTRAIT_MODE
		.orientation = { -1, 0, 0, 
			0, 1, 0, 
			0, 0, 1 },
	},
#else
		.orientation = { 0, -1, 0, 
			-1, 0, 0, 
			0, 0, -1 },
	},
#endif
	/* pressure */
	.pressure = {
#ifdef CONFIG_MPU_SENSORS_MPU3050_MODULE
		.get_slave_descr = NULL,
#else
		.get_slave_descr = get_pressure_slave_descr,
#endif
		.adapt_num   = 2,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
		.address     = 0x77,
		.orientation = { 1, 0, 0, 
			0, 1, 0, 
			0, 0, 1 },
	},
};
#endif

#if defined(CONFIG_MPU_SENSORS_MPU6000) || defined(CONFIG_MPU_SENSORS_MPU6000_MODULE)

#define SENSOR_MPU_NAME "mpu6000"

static struct mpu3050_platform_data mpu_data = {
	.int_config  = 0x10,
	.orientation = {  -1,  0,  0,
		0,  1,  0,
		0,  0, -1 },
	/* accel */
	.accel = {
#ifdef CONFIG_MPU_SENSORS_MPU6000_MODULE
		.get_slave_descr = NULL,
#else
		.get_slave_descr = get_accel_slave_descr,
#endif
		.adapt_num   = 2,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
		.address     = 0x68,
		.orientation = {  -1,  0,  0,
			0,  1,  0,
			0,  0, -1 },
	},
	/* compass */
	.compass = {
#ifdef CONFIG_MPU_SENSORS_MPU6000_MODULE
		.get_slave_descr = NULL,
#else
		.get_slave_descr = get_compass_slave_descr,
#endif
		.adapt_num   = 2,
		.bus         = EXT_SLAVE_BUS_SECONDARY,
		.address     = 0x0E,
		.orientation = { 1, 0, 0,
			0, 1, 0,
			0, 0, 1 },
	},
	/* pressure */
	.pressure = {
#ifdef CONFIG_MPU_SENSORS_MPU6000_MODULE
		.get_slave_descr = NULL,
#else
		.get_slave_descr = get_pressure_slave_descr,
#endif
		.adapt_num   = 2,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
		.address     = 0x77,
		.orientation = { 1, 0, 0, 
			0, 1, 0, 
			0, 0, 1 },
	},

};
#endif
#endif

#ifdef CONFIG_TC4_ICS
#include <linux/mpu.h>

static struct mpu_platform_data mpu3050_data = {
	.int_config  = 0x10,
#ifdef CONFIG_TC4_PORTRAIT_MODE
	.orientation = {  0,  1,  0, 
		1,  0,  0, 
		0,  0, -1 },
#else
	.orientation = {  -1,  0,  0,
			   0,  1,  0,
			   0,  0, -1 },
#endif
};

/* accel */
static struct ext_slave_platform_data inv_mpu_bma250_data = {
	.bus         = EXT_SLAVE_BUS_SECONDARY,
#ifdef CONFIG_TC4_PORTRAIT_MODE
	.orientation = {  1,  0,  0, 
		0,  -1,  0, 
		0,  0, -1 },
#else
	.orientation = {  0,  1,  0,
			  1,  0,  0,
			  0,  0, -1 },
#endif
};

/* compass */
static struct ext_slave_platform_data inv_mpu_hmc5883_data = {
	.bus         = EXT_SLAVE_BUS_PRIMARY,
#ifdef CONFIG_TC4_PORTRAIT_MODE
	.orientation = { -1, 0, 0, 
		0, 1, 0, 
		0, 0, 1 },
#else
	.orientation = { 0, -1, 0,
			 -1, 0, 0,
			 0, 0, -1 },
#endif
};
#endif

#if(defined(CONFIG_SENSORS_AK8963) || defined(CONFIG_SENSORS_AK8963_MODULE))
static struct akm8963_platform_data akm_platform_data_8963 = {
	.gpio_DRDY = EXYNOS4_GPX0(5),
	.gpio_RST = EXYNOS4_GPL0(6),//EXYNOS4_GPL0(6),
	.layout = 3,
	.outbit = 1,
};
#endif
#if defined(CONFIG_TOUCHSCREEN_EGALAX)	// Egalax I2C TS
#include <linux/i2c/egalax.h>	// touch
#endif
#include <linux/i2c-gpio.h>	

#ifdef CONFIG_VIDEO_SAMSUNG_S5P_FIMC
#include <plat/fimc-core.h>
#include <media/s5p_fimc.h>
#endif

#if defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
#include <plat/s5p-mfc.h>
#endif

#if defined (CONFIG_VIDEO_JPEG_V2X) || defined(CONFIG_VIDEO_JPEG)
#include <plat/jpeg.h>
#endif
#ifdef CONFIG_REGULATOR_S5M8767
#include <linux/mfd/s5m87xx/s5m-core.h>
#include <linux/mfd/s5m87xx/s5m-pmic.h>
#endif

#if defined(CONFIG_EXYNOS_SETUP_THERMAL) 
#include <plat/s5p-tmu.h> 
#endif
#define REG_INFORM4            (S5P_INFORM4)

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDK4X12_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDK4X12_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDK4X12_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg smdk4x12_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
};

static struct resource smdk4x12_smsc911x_resources[] = {
	[0] = {
		.start	= EXYNOS4_PA_SROM_BANK(1),
		.end	= EXYNOS4_PA_SROM_BANK(1) + SZ_64K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_EINT(5),
		.end	= IRQ_EINT(5),
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_LOW,
	},
};

static struct smsc911x_platform_config smsc9215_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_16BIT | SMSC911X_FORCE_INTERNAL_PHY,
	.phy_interface	= PHY_INTERFACE_MODE_MII,
	.mac		= {0x00, 0x80, 0x00, 0x23, 0x45, 0x67},
};

static struct platform_device smdk4x12_smsc911x = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(smdk4x12_smsc911x_resources),
	.resource	= smdk4x12_smsc911x_resources,
	.dev		= {
		.platform_data	= &smsc9215_config,
	},
};

#ifdef CONFIG_EXYNOS_MEDIA_DEVICE
struct platform_device exynos_device_md0 = {
	.name = "exynos-mdev",
	.id = -1,
};
#endif

#define WRITEBACK_ENABLED

#if defined(CONFIG_VIDEO_FIMC) || defined(CONFIG_VIDEO_SAMSUNG_S5P_FIMC)
/*
 * External camera reset
 * Because the most of cameras take i2c bus signal, so that
 * you have to reset at the boot time for other i2c slave devices.
 * This function also called at fimc_init_camera()
 * Do optimization for cameras on your platform.
*/
#if defined(CONFIG_ITU_A) || defined(CONFIG_CSI_C) \
	|| defined(CONFIG_S5K3H2_CSI_C) || defined(CONFIG_S5K3H7_CSI_C) \
	|| defined(CONFIG_S5K4E5_CSI_C) || defined(CONFIG_S5K6A3_CSI_C)
static int smdk4x12_cam0_reset(int dummy)
{
	int err;

	return 0;
}

static int S5K3H2_cam0_reset(int dummy)
{
	int err;
	/* Camera A */

	printk(KERN_ERR "#### S5K3H2_cam0_reset ####\n");
	err = gpio_request(EXYNOS4_GPF1(4), "GPF1_4");
	if (err)
		printk(KERN_ERR "#### failed to request GPF1_4 ####\n");

	s3c_gpio_setpull(EXYNOS4_GPF1(4), S3C_GPIO_PULL_NONE);
	gpio_direction_output(EXYNOS4_GPF1(4), 0);
	gpio_direction_output(EXYNOS4_GPF1(4), 1);
	gpio_free(EXYNOS4_GPF1(4));


	err = gpio_request(EXYNOS4_GPF1(5), "GPF1_5");
	if (err)
		printk(KERN_ERR "#### failed to request GPF1_5 ####\n");

	s3c_gpio_setpull(EXYNOS4_GPF1(5), S3C_GPIO_PULL_NONE);
	gpio_direction_output(EXYNOS4_GPF1(5), 1);
	gpio_free(EXYNOS4_GPF1(5));

	return 0;
}
#endif
#if defined(CONFIG_ITU_B) || defined(CONFIG_CSI_D) \
	|| defined(CONFIG_S5K3H2_CSI_D) || defined(CONFIG_S5K3H7_CSI_D) \
	|| defined(CONFIG_S5K4E5_CSI_D) || defined(CONFIG_S5K6A3_CSI_D)
static int smdk4x12_cam1_reset(int dummy)
{
	int err;

	return 0;
}
#if 1//ndef CONFIG_TC4_EVT
static struct regulator *vdd18_5m_cam_regulator = NULL;	
static struct regulator *vdd28_5m_cam_regulator = NULL;
static struct regulator *vddaf_cam_regulator = NULL;	
static struct regulator *vdd5m_cam_regulator = NULL;

extern struct regulator *tv_regulator_vdd18 ;	//added by yulu for controlling mipi voltage 	
extern struct regulator *tv_regulator_vdd10 ;	
extern bool cam_mipi_en;
#endif

static int S5K3H2_cam1_reset(int dummy)
{
	int err;
	int ret = -ENODEV;
#ifndef CONFIG_TC4_EVT
	vdd28_5m_cam_regulator = regulator_get(NULL, "vdd28_cam");
	
	if (IS_ERR(vdd28_5m_cam_regulator)) {
		printk("%s: failed to get %s\n", __func__, "vdd28_cam");
		ret = -ENODEV;
		goto err_regulator;
	}
	vddaf_cam_regulator = regulator_get(NULL, "vdd28_af");
	if (IS_ERR(vdd28_5m_cam_regulator)) {
		printk("%s: failed to get %s\n", __func__, "vdd28_af");
		ret = -ENODEV;
		goto err_regulator;
	}
	vdd18_5m_cam_regulator = regulator_get(NULL, "vdd18_cam");
	
	if (IS_ERR(vdd18_5m_cam_regulator)) {
		printk("%s: failed to get %s\n", __func__, "vdd18_cam");
		ret = -ENODEV;
		goto err_regulator;
	}
	vdd5m_cam_regulator = regulator_get(NULL, "vdd12_5m");
	if (IS_ERR(vdd5m_cam_regulator)) {
		printk("%s: failed to get %s\n", __func__, "vdd12_5m");
		ret = -ENODEV;
		goto err_regulator;
	}
#endif
	printk("S5K3H2_cam1_reset  dummy = %d\n",dummy);
       /* Camera B */
	if(dummy == 1)// power on
	{	
		if (gpio_request(GPIO_CAM_MEGA_nRST, "GPF1_4"/*"GPJ1"*/) < 0)
			pr_err("failed gpio_request(GPF1_4) for camera control\n");

		gpio_direction_output(GPIO_CAM_MEGA_nRST, 0);
		s3c_gpio_setpull(GPIO_CAM_MEGA_nRST, S3C_GPIO_PULL_NONE);
	
		if (gpio_request(GPIO_CAM_MEGA_EN, "GPF1_5"/*"GPJ0"*/) < 0)
			pr_err("failed gpio_request(GPF1_5) for camera control\n");

		gpio_direction_output(GPIO_CAM_MEGA_EN, 0);
		s3c_gpio_setpull(GPIO_CAM_MEGA_EN, S3C_GPIO_PULL_NONE);
#ifndef CONFIG_TC4_EVT
		regulator_enable(vdd18_5m_cam_regulator); 
	       udelay(10);
		regulator_enable(vdd28_5m_cam_regulator); 
	       udelay(10);
         	regulator_enable(vddaf_cam_regulator); 
	       udelay(10);
		regulator_enable(vdd5m_cam_regulator); 
	       udelay(10);

		if (!regulator_is_enabled(tv_regulator_vdd18) ||
			!regulator_is_enabled(tv_regulator_vdd10))
		{
			regulator_enable(tv_regulator_vdd18);
			udelay(10);
			regulator_enable(tv_regulator_vdd10);
			udelay(10);
		}
		cam_mipi_en = true;
#endif
		
		s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(2));//MCLK
		mdelay(1);

		// STBYN high
		gpio_direction_output(GPIO_CAM_MEGA_EN, 1);	
		mdelay(1);

		// RSTN high
		gpio_direction_output(GPIO_CAM_MEGA_nRST, 1);	
		mdelay(1);

		// PCLK high
		s3c_gpio_cfgpin(GPIO_CAM_PCLK, S3C_GPIO_SFN(2));//PCLK

		gpio_free(GPIO_CAM_MEGA_nRST);
		gpio_free(GPIO_CAM_MEGA_EN);

	}else{
		if (gpio_request(GPIO_CAM_MEGA_nRST, "GPF1_4"/*"GPJ1"*/) < 0)
			pr_err("failed gpio_request(GPF1_4) for camera control\n");
	
		gpio_direction_output(GPIO_CAM_MEGA_nRST, 0);	
		mdelay(1);
		// STBYN high
		if (gpio_request(GPIO_CAM_MEGA_EN, "GPF1_5"/*"GPJ0"*/) < 0)
			pr_err("failed gpio_request(GPF1_5) for camera control\n");
		gpio_direction_output(GPIO_CAM_MEGA_EN, 0); 
		mdelay(1);

		gpio_free(GPIO_CAM_MEGA_nRST);
		gpio_free(GPIO_CAM_MEGA_EN);
#ifndef CONFIG_TC4_EVT
    	      // cam_mipi_en = false;
    	      // if((!tv_mipi_en )&& (!cam_mipi_en)){
			regulator_disable(tv_regulator_vdd10); 
		       udelay(10);
			regulator_disable(tv_regulator_vdd18); 
		       udelay(10);
    	       //}
		regulator_disable(vdd18_5m_cam_regulator); 
	       udelay(10);
		regulator_disable(vdd28_5m_cam_regulator); 
	       udelay(10);
		regulator_disable(vddaf_cam_regulator); 
	       udelay(10);
		regulator_disable(vdd5m_cam_regulator); 
	       udelay(10);
#endif		
	}
#ifndef CONFIG_TC4_EVT
err_regulator:
	regulator_put(vdd18_5m_cam_regulator);	
	regulator_put(vdd28_5m_cam_regulator);
	regulator_put(vddaf_cam_regulator);	
	regulator_put(vdd5m_cam_regulator);
#endif
       return 0;
}

#endif
#endif

#ifdef CONFIG_VIDEO_FIMC
#ifdef CONFIG_VIDEO_S5K4BA
static struct s5k4ba_platform_data s5k4ba_plat = {
	.default_width = 800,
	.default_height = 600,
	.pixelformat = V4L2_PIX_FMT_YUYV,
	.freq = 24000000,
	.is_mipi = 0,
};

static struct i2c_board_info s5k4ba_i2c_info = {
	I2C_BOARD_INFO("S5K4BA", 0x2d),
	.platform_data = &s5k4ba_plat,
};

static struct s3c_platform_camera s5k4ba = {
#ifdef CONFIG_ITU_A
	.id		= CAMERA_PAR_A,
	.clk_name	= "sclk_cam0",
	.i2c_busnum	= 4,
	.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_ITU_B
	.id		= CAMERA_PAR_B,
	.clk_name	= "sclk_cam1",
	.i2c_busnum	= 5,
	.cam_power	= smdk4x12_cam1_reset,
#endif
	.type		= CAM_TYPE_ITU,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.info		= &s5k4ba_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_YUYV,
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.line_length	= 1920,
	.width		= 1600,
	.height		= 1200,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1600,
		.height	= 1200,
	},

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.reset_camera	= 1,
	.initialized	= 0,
};
#endif
#ifdef CONFIG_SOC_CAMERA_MT9D115

static struct mt9d115_platform_data mt9d115_plat = {
	.default_width = 640,
	.default_height = 480,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 0,
};

static struct i2c_board_info  mt9d115_i2c_info = {
	I2C_BOARD_INFO("MT9D115", 0x3c),
	.platform_data = &mt9d115_plat,
};

static struct s3c_platform_camera mt9d115 = {
//#ifdef CONFIG_CSI_C //added yqf, remove
			.id 	= CAMERA_PAR_A,
			.clk_name	= "sclk_cam0",
			.i2c_busnum = 7,
			.cam_power	= smdk4x12_cam1_reset,
//#endif

		.type		= CAM_TYPE_ITU,
		.fmt		= ITU_601_YCBCR422_8BIT,
		.order422	= CAM_ORDER422_8BIT_CBYCRY,
		.info		= &mt9d115_i2c_info,
		.pixelformat	= V4L2_PIX_FMT_UYVY,
		.srclk_name = "xusbxti",
		.clk_rate	= 24000000,
		.line_length	= 1920,
		.width		= 640,
		.height 	= 480,
		.window 	= {
			.left	= 0,
			.top	= 0,
			.width	= 640,
			.height = 480,
		},
	
		/* Polarity */
		.inv_pclk	= 0,
		.inv_vsync	= 1,
		.inv_href	= 0,
		.inv_hsync	= 0,
		.reset_camera	= 1,
		.initialized	= 0,
              .layout_rotate = 180, 
};
#endif

/*
 * Guide for Camera Configuration for Crespo board
 * ITU CAM CH A: LSI s5k4ecgx
 */

#ifdef CONFIG_VIDEO_S5K4ECGX
static struct s5k4ecgx_platform_data s5k4ecgx_plat = {
	.default_width = temp_width,
	.default_height = temp_height,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,

	.is_mipi = 1,
};

static struct i2c_board_info  s5k4ecgx_i2c_info = {
	I2C_BOARD_INFO("S5K4ECGX", 0xAC>>1),
	.platform_data = &s5k4ecgx_plat,
};

static struct s3c_platform_camera s5k4ecgx = {
	.id = CAMERA_CSI_D,      
	.type = CAM_TYPE_MIPI, 
	.fmt = MIPI_CSI_YCBCR422_8BIT, 
	.order422 = CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum = 6,
	.info = &s5k4ecgx_i2c_info,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.srclk_name ="xusbxti",
	.clk_name = "sclk_cam0",
	.clk_rate = 24000000,
	.line_length = 1920,
	.width = temp_width,
	.height = temp_height,
	.window = {
		.left = 0,
		.top = 0,
		.width = temp_width,
		.height = temp_height,
	},

	.mipi_lanes = 2, //add
	.mipi_settle = 12, //add
	.mipi_align = 32, //add

	/* Polarity */
	.inv_pclk = 0,
	.inv_vsync = 1,
	.inv_href = 0,
	.inv_hsync = 0,

	.initialized = 0,
       .layout_rotate = 180, 
	.cam_power = smdk4x12_cam1_reset,

};
#endif



/* 2 MIPI Cameras */
#ifdef CONFIG_VIDEO_S5K4EA
static struct s5k4ea_platform_data s5k4ea_plat = {
	.default_width = 1920,
	.default_height = 1080,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 1,
};

static struct i2c_board_info s5k4ea_i2c_info = {
	I2C_BOARD_INFO("S5K4EA", 0x2d),
	.platform_data = &s5k4ea_plat,
};

static struct s3c_platform_camera s5k4ea = {
#ifdef CONFIG_CSI_C
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.i2c_busnum	= 4,
	.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_CSI_D
	.id		= CAMERA_CSI_D,
	.clk_name	= "sclk_cam1",
	.i2c_busnum	= 5,
	.cam_power	= smdk4x12_cam1_reset,
#endif
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.info		= &s5k4ea_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.line_length	= 1920,
	.width		= 1920,
	.height		= 1080,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1920,
		.height	= 1080,
	},

	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 32,

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,

	.initialized	= 0,
};
#endif

#ifdef WRITEBACK_ENABLED
static struct i2c_board_info writeback_i2c_info = {
	I2C_BOARD_INFO("WriteBack", 0x0),
};

static struct s3c_platform_camera writeback = {
	.id		= CAMERA_WB,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum	= 0,
	.info		= &writeback_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_YUV444,
	.line_length	= 800,
	.width		= 480,
	.height		= 800,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 480,
		.height	= 800,
	},

	.initialized	= 0,
};
#endif

#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
#ifdef CONFIG_VIDEO_S5K3H2
static struct i2c_board_info s5k3h2_sensor_info = {
	.type = "S5K3H2",
};

static struct s3c_platform_camera s5k3h2 = {
#ifdef CONFIG_S5K3H2_CSI_C
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.cam_power	= S5K3H2_cam0_reset,
#endif
#ifdef CONFIG_S5K3H2_CSI_D
	.id		= CAMERA_CSI_D,
	.clk_name	= "sclk_cam0",//zhuxuezhen
	.cam_power	= S5K3H2_cam1_reset,
#endif
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_RAW10,
	.info		= &s5k3h2_sensor_info,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.line_length	= 1920,
	.width		= 1920,
	.height		= 1080,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1920,
		.height	= 1080,
	},
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 24,

	.initialized	= 0,
#ifdef CONFIG_S5K3H2_CSI_C
	.flite_id	= FLITE_IDX_A,
#endif
#ifdef CONFIG_S5K3H2_CSI_D
	.flite_id	= FLITE_IDX_B,
#endif
	.use_isp	= true,
#ifdef CONFIG_S5K3H2_CSI_C
	.sensor_index	= 1,
#endif
#ifdef CONFIG_S5K3H2_CSI_D
	.sensor_index	= 101,
#endif
};
#endif

#ifdef CONFIG_VIDEO_S5K3H7
static struct s3c_platform_camera s5k3h7 = {
#ifdef CONFIG_S5K3H7_CSI_C
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_S5K3H7_CSI_D
	.id		= CAMERA_CSI_D,
	.clk_name	= "sclk_cam1",
	.cam_power	= smdk4x12_cam1_reset,
#endif
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_RAW10,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.line_length	= 1920,
	.width		= 1920,
	.height		= 1080,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1920,
		.height	= 1080,
	},
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 24,

	.initialized	= 0,
#ifdef CONFIG_S5K3H7_CSI_C
	.flite_id	= FLITE_IDX_A,
#endif
#ifdef CONFIG_S5K3H7_CSI_D
	.flite_id	= FLITE_IDX_B,
#endif
	.use_isp	= true,
#ifdef CONFIG_S5K3H7_CSI_C
	.sensor_index	= 4,
#endif
#ifdef CONFIG_S5K3H7_CSI_D
	.sensor_index	= 104,
#endif
};
#endif

#ifdef CONFIG_VIDEO_S5K4E5
static struct s3c_platform_camera s5k4e5 = {
#ifdef CONFIG_S5K4E5_CSI_C
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_S5K4E5_CSI_D
	.id		= CAMERA_CSI_D,
	.clk_name	= "sclk_cam1",
	.cam_power	= smdk4x12_cam1_reset,
#endif
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_RAW10,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.line_length	= 1920,
	.width		= 1920,
	.height		= 1080,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1920,
		.height	= 1080,
	},
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 24,

	.initialized	= 0,
#ifdef CONFIG_S5K4E5_CSI_C
	.flite_id	= FLITE_IDX_A,
#endif
#ifdef CONFIG_S5K4E5_CSI_D
	.flite_id	= FLITE_IDX_B,
#endif
	.use_isp	= true,
#ifdef CONFIG_S5K4E5_CSI_C
	.sensor_index	= 3,
#endif
#ifdef CONFIG_S5K4E5_CSI_D
	.sensor_index	= 103,
#endif
};
#endif


#ifdef CONFIG_VIDEO_S5K6A3
static struct s3c_platform_camera s5k6a3 = {
#ifdef CONFIG_S5K6A3_CSI_C
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_S5K6A3_CSI_D
	.id		= CAMERA_CSI_D,
	.clk_name	= "sclk_cam1",
	.cam_power	= smdk4x12_cam1_reset,
#endif
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_RAW10,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.line_length	= 1920,
	.width		= 1920,
	.height		= 1080,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1920,
		.height	= 1080,
	},
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.mipi_lanes	= 1,
	.mipi_settle	= 12,
	.mipi_align	= 24,

	.initialized	= 0,
#ifdef CONFIG_S5K6A3_CSI_C
	.flite_id	= FLITE_IDX_A,
#endif
#ifdef CONFIG_S5K6A3_CSI_D
	.flite_id	= FLITE_IDX_B,
#endif
	.use_isp	= true,
#ifdef CONFIG_S5K6A3_CSI_C
	.sensor_index	= 2,
#endif
#ifdef CONFIG_S5K6A3_CSI_D
	.sensor_index	= 102,
#endif
};
#endif

#endif

/* legacy M5MOLS Camera driver configuration */
#ifdef CONFIG_VIDEO_M5MO
#define CAM_CHECK_ERR_RET(x, msg)	\
	if (unlikely((x) < 0)) { \
		printk(KERN_ERR "\nfail to %s: err = %d\n", msg, x); \
		return x; \
	}
#define CAM_CHECK_ERR(x, msg)	\
		if (unlikely((x) < 0)) { \
			printk(KERN_ERR "\nfail to %s: err = %d\n", msg, x); \
		}
//add charles.hu begin 2012-07-26

static int m5mo_power_on(void)
{
	//struct regulator *regulator;
	int ret = 0;

	printk(KERN_DEBUG "%s: in\n", __func__);

	/* ISP_RESET low */
	ret = gpio_request(EXYNOS4_GPL0(1), "GPL0_1");	//ISP RESET GPIO
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_ISP_RESET)\n");
		return ret;
	}

	gpio_direction_output(EXYNOS4_GPL0(1), 0);
	CAM_CHECK_ERR_RET(ret, "output ISP_nRST");

//Cellon add begin , charles.hu 2012/10/17
#if 1			
	///POWER UP   charles.hu add 	
	struct regulator *camrear_2v8_regulator;   //24	
	struct regulator *camrear_1v8_regulator;   //26	
	struct regulator *camrear_1v2_regularor;   //buck6	

	camrear_1v2_regularor = regulator_get(NULL, "vdd12_5m");	
	if (IS_ERR(camrear_1v2_regularor)) {		
		printk("%s: failed to get %s\n", __func__, "vdd12_5m");		
		return -1;			
	}	
	if (!regulator_is_enabled(camrear_1v2_regularor)){	
		regulator_enable(camrear_1v2_regularor);	
	}	
	regulator_put(camrear_1v2_regularor);	
	mdelay(1);	

	camrear_1v8_regulator = regulator_get(NULL, "vdd18_a31");	
	if (IS_ERR(camrear_1v8_regulator)) {		
		printk("%s: failed to get %s\n", __func__, "camrear_1v8_regulator");
		return -1;		
	}	

	if (!regulator_is_enabled(camrear_1v8_regulator)){	
		regulator_enable(camrear_1v8_regulator);	
	}	
	regulator_put(camrear_1v8_regulator);	
	mdelay(1);

	camrear_2v8_regulator = regulator_get(NULL, "vdd33_a31"); ///vdd33_a31	
	if (IS_ERR(camrear_2v8_regulator)) {		
		printk("%s: failed to get %s\n", __func__, "camrear_2v8_regulator");
		return -1;			
	}	
	if (!regulator_is_enabled(camrear_2v8_regulator)){	
		regulator_enable(camrear_2v8_regulator);
	}	
	regulator_put(camrear_2v8_regulator);	
	mdelay(1);

#endif
//Cellon add begin , charles.hu 2012/10/17

	/* MCLK */
	ret = gpio_request(EXYNOS4212_GPJ1(3), "GPJ1_3");	//ISP MCLK GPIO
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_ISP_MCLK)\n");
		return ret;
	}

	ret = s3c_gpio_cfgpin(EXYNOS4212_GPJ1(3), S3C_GPIO_SFN(2));
	CAM_CHECK_ERR_RET(ret, "cfg mclk");
	s3c_gpio_setpull(EXYNOS4212_GPJ1(3), S3C_GPIO_PULL_NONE);
	mdelay(2);


	/* ISP_RESET */
	ret = gpio_direction_output(EXYNOS4_GPL0(1), 1);
	CAM_CHECK_ERR_RET(ret, "output reset");
	mdelay(10);

	gpio_free(EXYNOS4_GPL0(1));
	gpio_free(EXYNOS4212_GPJ1(3));

	return ret;
}

static int m5mo_power_down(void)
{
//	struct regulator *regulator;
	int ret = 0;
	
	printk(KERN_DEBUG "%s: in\n", __func__);

	ret = gpio_request(EXYNOS4_GPL0(1), "GPL0_1");	//ISP RESET GPIO
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_ISP_RESET)\n");
		return ret;
	}

	/* ISP_RESET */
	ret = gpio_direction_output(EXYNOS4_GPL0(1), 0);
	CAM_CHECK_ERR(ret, "output reset");
	gpio_free(EXYNOS4_GPL0(1));
	mdelay(2);

#if 0
	ret = gpio_request(EXYNOS4212_GPJ1(3), "GPJ1_3");	//ISP MCLK GPIO
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_ISP_CLOCK)\n");
		return ret;
	}

	/* MCLK */
	ret = s3c_gpio_cfgpin(EXYNOS4212_GPJ1(3), S3C_GPIO_INPUT);
	s3c_gpio_setpull(EXYNOS4212_GPJ1(3), S3C_GPIO_PULL_DOWN);
	CAM_CHECK_ERR(ret, "cfg mclk");
	mdelay(2);
	gpio_free(EXYNOS4212_GPJ1(3));
#endif 


//Cellon add begin , charles.hu 2012/10/17
#if 1			
	///POWER DOWN   charles.hu add 	
	struct regulator *camrear_2v8_regulator;   //24	
	struct regulator *camrear_1v8_regulator;   //26	
	struct regulator *camrear_1v2_regularor;   //buck6	
	
	camrear_2v8_regulator = regulator_get(NULL, "vdd33_a31"); ///vdd33_a31	
	if (IS_ERR(camrear_2v8_regulator)) {		
		printk("%s: failed to get %s\n", __func__, "camrear_2v8_regulator");
		return -1;			
	}		
	regulator_disable(camrear_2v8_regulator);	
	regulator_put(camrear_2v8_regulator);	
	mdelay(1);

	camrear_1v8_regulator = regulator_get(NULL, "vdd18_a31");	
	if (IS_ERR(camrear_1v8_regulator)) {		
		printk("%s: failed to get %s\n", __func__, "camrear_1v8_regulator");
		return -1;		
	}	
	
	regulator_disable(camrear_1v8_regulator);	
	regulator_put(camrear_1v8_regulator);
	mdelay(1);

	camrear_1v2_regularor = regulator_get(NULL, "vdd12_5m");	
	if (IS_ERR(camrear_1v2_regularor)) {		
		printk("%s: failed to get %s\n", __func__, "vdd12_5m");		
		return -1;			
	}		
	regulator_disable(camrear_1v2_regularor);	
	regulator_put(camrear_1v2_regularor);	
	mdelay(1);
#endif
//Cellon add begin , charles.hu 2012/10/17	

	return ret;
}

static int m5mo_power(int enable)
{
	int ret = 0;

	printk(KERN_ERR "%s %s\n", __func__, enable ? "on" : "down");
	if (enable) {
		ret = m5mo_power_on();
		if (unlikely(ret))
			goto error_out;
	} else
		ret = m5mo_power_down();

error_out:
	return ret;
}		

//add charles.hu end 2012-07-26
static int m5mo_config_isp_irq(void)
{
	s3c_gpio_cfgpin(EXYNOS4_GPX3(3), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS4_GPX3(3), S3C_GPIO_PULL_DOWN);
	
	return 0;
}

static struct m5mo_platform_data m5mo_plat = {
	.default_width = 640, /* 1920 */
	.default_height = 480, /* 1080 */
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 1,
	.config_isp_irq = m5mo_config_isp_irq,
	.irq = IRQ_EINT(27),
};

static struct i2c_board_info m5mo_i2c_info = {
	I2C_BOARD_INFO("M5MO", 0x1F),
	.platform_data = &m5mo_plat,
	.irq = IRQ_EINT(27),
};

static struct s3c_platform_camera m5mo = {
	.id		    = CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.i2c_busnum	= 7,
	.cam_power	= m5mo_power,
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.info		= &m5mo_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "xusbxti", /* "mout_mpll" */
	.clk_rate	= 24000000, /* 48000000 */
	.line_length	= 1920,
	.width		= 640,
	.height		= 480,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 640,
		.height	= 480,
	},

	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 32,

	/* Polarity */
	.inv_pclk	= 1,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.reset_camera	= 0,
	.initialized	= 0,
        .layout_rotate = 180,
	.use_isp	= false,	//add charles.hu 2012/08/07
};
#endif

/* 5CA MIPI Cameras */
#ifdef CONFIG_VIDEO_S5K5CA

static int s5k5ca_power_on(void)
{
	printk("%s: Entering...\n",__func__);

	//nRST
	if (gpio_request(GPIO_CAM_MEGA_nRST, "GPM0_2"/*"GPM0_2"*/) < 0)
		pr_err("failed gpio_request(GPM0_2) for camera control\n");

	gpio_direction_output(GPIO_CAM_MEGA_nRST, 0);
	s3c_gpio_setpull(GPIO_CAM_MEGA_nRST, S3C_GPIO_PULL_NONE);

	//POWER DOWN
	if (gpio_request(GPIO_CAM_MEGA_EN, "GPM0_3"/*"GPM0_3"*/) < 0)
		pr_err("failed gpio_request(GPM0_3) for camera control\n");

	gpio_direction_output(GPIO_CAM_MEGA_EN, 1);
	s3c_gpio_setpull(GPIO_CAM_MEGA_EN, S3C_GPIO_PULL_NONE);

	//MCLK
	//if (gpio_request(GPIO_CAM_MCLK,"GPJ1_3") < 0)
	//	pr_err("failed gpio_request(GPJ1_3) for camera control\n");
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(2));
	mdelay(1);
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_NONE);
	
	// STBYN high
	gpio_direction_output(GPIO_CAM_MEGA_EN, 0);	
	mdelay(15);

	// RSTN high
	gpio_direction_output(GPIO_CAM_MEGA_nRST, 1);	
	mdelay(100);

	gpio_free(GPIO_CAM_MEGA_nRST);
	gpio_free(GPIO_CAM_MEGA_EN);
	//gpio_free(GPIO_CAM_MCLK);

}

static int s5k5ca_power_down(void)
{

	printk("%s:Entering....\n",__func__);

	//nRST
	if (gpio_request(GPIO_CAM_MEGA_nRST, "GPX1_6"/*"GPJ1"*/) < 0)
		pr_err("failed gpio_request(GPX1_6) for camera control\n");

	gpio_direction_output(GPIO_CAM_MEGA_nRST, 0);	
	mdelay(1);
	
	// STBYN high
	if (gpio_request(GPIO_CAM_MEGA_EN, "GX0_0"/*"GPJ0"*/) < 0)
		pr_err("failed gpio_request(GX0_0) for camera control\n");
	gpio_direction_output(GPIO_CAM_MEGA_EN, 1); 
	mdelay(1);

	//MCLK
	//if (gpio_request(GPIO_CAM_MCLK,"GPJ1_3") < 0)
	//	pr_err("failed gpio_request(GPJ1_3) for camera control\n");

	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_DOWN);
	

	gpio_free(GPIO_CAM_MEGA_nRST);
	gpio_free(GPIO_CAM_MEGA_EN);
	//gpio_free(GPIO_CAM_MCLK);

}


static int s5k5ca_power(int enable)
{
	int ret = 0;

	printk(KERN_ERR "%s %s\n", __func__, enable ? "on" : "down");
	if (enable) {
		ret = s5k5ca_power_on();
		if (unlikely(ret))
			goto error_out;
	} else
		ret = s5k5ca_power_down();

error_out:
	return ret;
}
static struct s5k4ea_platform_data s5k5ca_plat = {	
	.default_width = 640,	
	.default_height = 480,	
	.pixelformat = V4L2_PIX_FMT_UYVY,	
	.freq = 24000000,	
	.is_mipi = 1,
};
static struct i2c_board_info s5k5ca_i2c_info = {	
	I2C_BOARD_INFO("S5K5CA", 0x5a>>1),	
	.platform_data = &s5k5ca_plat,
};
static struct s3c_platform_camera s5k5ca = {
	.id		= CAMERA_CSI_D,	
	.clk_name	= "sclk_cam0",	
	.i2c_busnum	= 7,	
	.cam_power	= s5k5ca_power,
	.type		= CAM_TYPE_MIPI,	
	.fmt		= MIPI_CSI_YCBCR422_8BIT,	
	.order422	= CAM_ORDER422_8BIT_YCBYCR,	
	.info		= &s5k5ca_i2c_info,	
	.pixelformat	= V4L2_PIX_FMT_UYVY,	
	.srclk_name	= "xusbxti",	
	.clk_rate	=24000000,	
	.line_length	= 1920,	
	.width		= 640,	
	.height		= 480,	
	.window		= {		
		.left	= 0,		
		.top	= 0,		
		.width	= 460,		
		.height	= 480,	
	},	
	.mipi_lanes	= 1,	
	.mipi_settle = 12,	
	.mipi_align	= 32,	
	/* Polarity */	
	.inv_pclk	= 0,	
	.inv_vsync	= 1,	
	.inv_href	= 0,	
	.inv_hsync	= 0,	
	.initialized = 0,	
};
#endif

//Cellon add begin, crystal wang, 2012/9/13 , for front camera
///#ifdef CONFIG_VIDEO_OV2675 		//crystal add,2012-9-13
extern 	ov2675_pwr_up();


int 	ov2675_power_on(void)
{
	printk("===%s  %d \n",__func__,__LINE__);
	
		//MCLK
	if (gpio_request(GPIO_CAM_MCLK,"GPJ1_3") < 0)
		printk("failedcry gpio_request(GPJ1_3) for camera control\n");
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(2));
	mdelay(1);
	///s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_NONE);
	gpio_free(GPIO_CAM_MCLK);
	
	
	if (gpio_request(GPIO_CAM_MEGA_nRST, "GPM0_2"/*"GPM0_2"*/) < 0)
		printk("failedcry gpio_request(GPM0_2) for camera control\n");
	s3c_gpio_setpull(GPIO_CAM_MEGA_nRST, S3C_GPIO_PULL_NONE);
	gpio_direction_output(GPIO_CAM_MEGA_nRST,0); mdelay(1);	
///	gpio_direction_output(GPIO_CAM_MEGA_nRST,1); mdelay(1);	
///		gpio_direction_output(GPIO_CAM_MEGA_nRST,0); mdelay(1);		
//		gpio_direction_output(GPIO_CAM_MEGA_nRST,1); mdelay(1);	
	//POWER DOWN
	if (gpio_request(GPIO_CAM_MEGA_EN, "GPM0_3"/*"GPM0_3"*/) < 0)
		printk("failedcry gpio_request(GPM0_3) for camera control\n");
	s3c_gpio_setpull(GPIO_CAM_MEGA_EN, S3C_GPIO_PULL_NONE);
	gpio_direction_output(GPIO_CAM_MEGA_EN, 1);

	mdelay(10);


	struct regulator *camffont_2v8_regulatorcha;//24
	struct regulator *camffont_1v8_regulatorcha;  //26

	camffont_2v8_regulatorcha = regulator_get(NULL, "vdd33_a31"); ///vdd33_a31
	if (IS_ERR(camffont_2v8_regulatorcha)) {
		printk("%s: failedcry to get %s\n", __func__, "camffont_2v8_regulator");
		//ret = -ENODEV;
		///goto out1;
	}
	regulator_enable(camffont_2v8_regulatorcha);
	regulator_put(camffont_2v8_regulatorcha);
	
	msleep(10);
	gpio_direction_output(GPIO_CAM_MEGA_EN, 0);
	 mdelay(10);	
	gpio_direction_output(GPIO_CAM_MEGA_nRST,1);
		
	mdelay(3);
	
	camffont_1v8_regulatorcha = regulator_get(NULL, "vdd18_a31"); ///vdd33_a31
	if (IS_ERR(camffont_1v8_regulatorcha)) {
		printk("%s: failedcry to get %s\n", __func__, "vddioperi_18");
		//ret = -ENODEV;
		///goto out1;
	}
	
	regulator_enable(camffont_1v8_regulatorcha);
	regulator_put(camffont_1v8_regulatorcha);
	mdelay(10); 

	
	///s3c_gpio_setpull(GPIO_CAM_MEGA_EN, S3C_GPIO_PULL_DOWN);
	///s3c_gpio_setpull(GPIO_CAM_MEGA_EN, S3C_GPIO_PULL_UP);
	//nRST

	//// ov2675_pwr_up();
///	gpio_direction_output(GPIO_CAM_MEGA_EN,0);
	
	//MCLK
/*	if (gpio_request(GPIO_CAM_MCLK,"GPJ1_3") < 0)
		pr_err("failed gpio_request(GPJ1_3) for camera control\n");
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(2));
	*/
	
///	mdelay(2); ///bigger than 1
	
	///gpio_direction_output(GPIO_CAM_MEGA_nRST,0); mdelay(1);	
	
////	gpio_direction_output(GPIO_CAM_MEGA_nRST,1);mdelay(10);
////	gpio_direction_output(GPIO_CAM_MEGA_nRST,0);mdelay(10);
////	gpio_direction_output(GPIO_CAM_MEGA_nRST,1);mdelay(10);
		
	///s3c_gpio_setpull(GPIO_CAM_MEGA_EN, S3C_GPIO_PULL_UP);
	///s3c_gpio_setpull(GPIO_CAM_MEGA_EN, S3C_GPIO_PULL_DOWN);
	/*msleep(20);///mdelay(20);
	gpio_direction_output(GPIO_CAM_MEGA_nRST,1);
	msleep(20);
	msleep(20);
	*/	
	///mdelay(10);
	
	gpio_free(GPIO_CAM_MEGA_nRST);
	gpio_free(GPIO_CAM_MEGA_EN);
	gpio_free(GPIO_CAM_MCLK);
	
}

 int ov2675_power_down(void)
{
	printk("===%s  %d \n",__func__,__LINE__);
	//printk("%s:Entering....\n",__func__);

	//nRST
	if (gpio_request(GPIO_CAM_MEGA_nRST, "GPX1_6"/*"GPJ1"*/) < 0)
		pr_err("failed gpio_request(GPX1_6) for camera control\n");

	gpio_direction_output(GPIO_CAM_MEGA_nRST, 0);	
	mdelay(1);	
	
	// STBYN high
	if (gpio_request(GPIO_CAM_MEGA_EN, "GX0_0"/*"GPJ0"*/) < 0)
		pr_err("failed gpio_request(GX0_0) for camera control\n");
	gpio_direction_output(GPIO_CAM_MEGA_EN, 1); 
	mdelay(1);

	//MCLK
	if (gpio_request(GPIO_CAM_MCLK,"GPJ1_3") < 0)
		pr_err("failed gpio_request(GPJ1_3) for camera control\n");

	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_DOWN);	


	struct regulator *camffont_2v8_regulatorcha;//24
	struct regulator *camffont_1v8_regulatorcha;  //26	
	
	camffont_2v8_regulatorcha = regulator_get(NULL, "vdd33_a31");
	///camffont_2v8_regulatorcha = regulator_get(NULL, "vdd33_a31"); ///vdd33_a31
	if (IS_ERR(camffont_2v8_regulatorcha)) {
		pr_err("%s: failed to get %s\n", __func__, "camffont_2v8_regulator");
		//ret = -ENODEV;
		///goto out1;
	}
	////if (!regulator_is_enabled(camffont_2v8_regulator))
	regulator_disable(camffont_2v8_regulatorcha);
	regulator_put(camffont_2v8_regulatorcha);
	
	msleep(10);
	camffont_1v8_regulatorcha = regulator_get(NULL, "vdd18_a31");
	///camffont_1v8_regulatorcha = regulator_get(NULL, "vdd18_a31"); ///vdd33_a31
	if (IS_ERR(camffont_1v8_regulatorcha)) {
		pr_err("%s: failed to get %s\n", __func__, "vddioperi_18");
		//ret = -ENODEV;
		///goto out1;
	}
	///if (!regulator_is_enabled(camffont_1v8_regulator))
	regulator_disable(camffont_1v8_regulatorcha);
	regulator_put(camffont_1v8_regulatorcha);






	gpio_free(GPIO_CAM_MEGA_nRST);
	gpio_free(GPIO_CAM_MEGA_EN);
	gpio_free(GPIO_CAM_MCLK);
	
}
EXPORT_SYMBOL(ov2675_power_down);
 int ov2675_power_down2(void)
{
	printk("%s:Entering....\n",__func__);
	struct regulator *camffont_2v8_regulatorcha;//24
	struct regulator *camffont_1v8_regulatorcha;  //26	
	
	camffont_2v8_regulatorcha = regulator_get(NULL, "vdd33_a31");
	if (IS_ERR(camffont_2v8_regulatorcha)) {
		pr_err("%s: failed to get %s\n", __func__, "camffont_2v8_regulator");
		//ret = -ENODEV;
		///goto out1;
	}
	regulator_enable(camffont_2v8_regulatorcha);
	regulator_disable(camffont_2v8_regulatorcha);
	regulator_put(camffont_2v8_regulatorcha);
	
	msleep(10);
	camffont_1v8_regulatorcha = regulator_get(NULL, "vdd18_a31");
	if (IS_ERR(camffont_1v8_regulatorcha)) {
		pr_err("%s: failed to get %s\n", __func__, "vddioperi_18");
		//ret = -ENODEV;
		///goto out1;
	}
	///if (!regulator_is_enabled(camffont_1v8_regulator))
	regulator_enable(camffont_1v8_regulatorcha);
	regulator_disable(camffont_1v8_regulatorcha);
	regulator_put(camffont_1v8_regulatorcha);

	//nRST
	if (gpio_request(GPIO_CAM_MEGA_nRST, "GPX1_6"/*"GPJ1"*/) < 0)
		pr_err("failed gpio_request(GPX1_6) for camera control\n");

	gpio_direction_output(GPIO_CAM_MEGA_nRST, 0);	
	mdelay(1);
		
	// STBYN high
	if (gpio_request(GPIO_CAM_MEGA_EN, "GX0_0"/*"GPJ0"*/) < 0)
		pr_err("failed gpio_request(GX0_0) for camera control\n");
	gpio_direction_output(GPIO_CAM_MEGA_EN, 1); 
	mdelay(1);

	//MCLK
	if (gpio_request(GPIO_CAM_MCLK,"GPJ1_3") < 0)
		pr_err("failed gpio_request(GPJ1_3) for camera control\n");

	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_DOWN);
	

	gpio_free(GPIO_CAM_MEGA_nRST);
	gpio_free(GPIO_CAM_MEGA_EN);
	gpio_free(GPIO_CAM_MCLK);
	
}
EXPORT_SYMBOL(ov2675_power_down2);





static int ov2675_power(int enable)
{
	int ret = 0;

	printk(KERN_ERR "=========%s %s\n", __func__, enable ? "on" : "down");
	if (enable) {
		ret = ov2675_power_on();
		if (unlikely(ret))
			goto error_out;
	} else
		ret = ov2675_power_down();

error_out:
	return ret;
}
static struct ov2675_platform_data ov2675_plat = {	
	.default_width = 800,//640,///1600,	
	.default_height =600,//480,//1200,
	.pixelformat = V4L2_PIX_FMT_UYVY,	
	.freq = 24000000,	
	.is_mipi = 1,
};
static struct i2c_board_info ov2675_i2c_info = {	
	I2C_BOARD_INFO("ov2675", 0x60>>1),	
	.platform_data = &ov2675_plat,
};

static struct s3c_platform_camera ov2675 = {
	///.id		= CAMERA_CSI_D,	
	///.clk_name	= "sclk_cam1",	 //NO
	
	//.id		= CAMERA_CSI_C,	
	//.clk_name	= "sclk_cam0",	 //NO
	
	.id		= CAMERA_CSI_D,	
	.clk_name	= "sclk_cam0",	
	 
	.i2c_busnum	= 7,	
	.cam_power	= ov2675_power,
	.type		= CAM_TYPE_MIPI,	
	.fmt		= MIPI_CSI_YCBCR422_8BIT,	
	.order422	= CAM_ORDER422_8BIT_YCBYCR,	
	.info		= &ov2675_i2c_info,	
	.pixelformat	= V4L2_PIX_FMT_UYVY,	
	.srclk_name	= "xusbxti",	
	.clk_rate	=24000000,	
	.line_length	= 1920,	
	.width		= 1600,//1600,//640,	
	.height		= 1200,//1200,//480,//
	.window		= {		
		.left	= 0,		
		.top	= 0,		
		.width	= 1600,//640,//1600,//640,		
		.height	= 1200,//480,//1200,//480,	
	},	
	.mipi_lanes	= 1,	
	.mipi_settle = 12,	
	.mipi_align	= 32,///24//WILL REEN,///32SVNRAW,	
	/* Polarity */	
	.inv_pclk	= 1,	////
	.inv_vsync	= 1,	
	.inv_href	= 0,	
	.inv_hsync	= 0,	
	.initialized = 0,	
};
///#endif 

//Cellon add end, crystal wang, 2012/09/21 , for front camera

/* Interface setting */
static struct s3c_platform_fimc fimc_plat = {
#ifdef CONFIG_ITU_A
	.default_cam	= CAMERA_PAR_A,
#endif
#ifdef CONFIG_ITU_B
	.default_cam	= CAMERA_PAR_B,
#endif
#ifdef CONFIG_CSI_C
	.default_cam	= CAMERA_CSI_C,
#endif
#ifdef CONFIG_CSI_D
	.default_cam	= CAMERA_CSI_D,
#endif
#ifdef WRITEBACK_ENABLED
	.default_cam	= CAMERA_WB,
#endif
	.camera		= {
#ifdef CONFIG_VIDEO_S5K4BA
		&s5k4ba,
#endif
#ifdef CONFIG_VIDEO_S5K4EA
		&s5k4ea,
#endif

#ifdef CONFIG_VIDEO_S5K3H2
		&s5k3h2,
#endif
#ifdef CONFIG_VIDEO_S5K3H7
		&s5k3h7,
#endif
#ifdef CONFIG_VIDEO_M5MO
		&m5mo,
#endif
#ifdef CONFIG_VIDEO_S5K5CA
		&s5k5ca,
#endif
///#ifdef CONFIG_VIDEO_OV2675  //crystal add,2012-9-13
		&ov2675,
///#endif

#if 0 //added yqf, adjust for middleware request
#ifdef CONFIG_SOC_CAMERA_MT9D115
                &mt9d115,
#endif
#ifdef CONFIG_VIDEO_S5K4ECGX
                &s5k4ecgx,
#endif
#else
//for S5K4EC back
#ifdef CONFIG_VIDEO_S5K4ECGX
		&s5k4ecgx,
#endif
//front
#ifdef CONFIG_SOC_CAMERA_MT9D115
		&mt9d115,
#endif
#endif

#ifdef CONFIG_VIDEO_S5K4E5
		&s5k4e5,
#endif
#ifdef CONFIG_VIDEO_S5K6A3
		&s5k6a3,
#endif
#ifdef WRITEBACK_ENABLED
		&writeback,
#endif
	},
	.hw_ver		= 0x51,
};
#endif /* CONFIG_VIDEO_FIMC */

/* for mainline fimc interface */
#ifdef CONFIG_VIDEO_SAMSUNG_S5P_FIMC
#ifdef WRITEBACK_ENABLED
struct writeback_mbus_platform_data {
	int id;
	struct v4l2_mbus_framefmt fmt;
};

static struct i2c_board_info __initdata writeback_info = {
	I2C_BOARD_INFO("writeback", 0x0),
};
#endif

#ifdef CONFIG_FB_S3C
#ifdef CONFIG_VIDEO_S5K4ECGX		//yulu
static struct s5k4ecgx_platform_data s5k4ecgx_plat = {
	.default_width = temp_width,
	.default_height = temp_height,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,

	.is_mipi = 1,
};
static struct i2c_board_info  s5k4ecgx_i2c_info = {
	I2C_BOARD_INFO("S5K4ECGX", 0xAC>>1),
	.platform_data = &s5k4ecgx_plat,
};
/* This is for platdata of fimc-lite */
static struct s3c_platform_camera s5k4ecgx = {
	.type		= CAM_TYPE_MIPI,
	.use_isp	= false,
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
};
#endif
#ifdef CONFIG_SOC_CAMERA_MT9D115

static struct mt9d115_platform_data mt9d115_plat = {
	.default_width = 640,
	.default_height = 480,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 0,
};

static struct i2c_board_info  mt9d115_i2c_info = {
	I2C_BOARD_INFO("MT9D115", 0x3c),
	.platform_data = &mt9d115_plat,
};
static struct s3c_platform_camera mt9d115 = {
	.type		= CAM_TYPE_ITU,
	.use_isp	= false,
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
};
#endif
#endif

#ifdef CONFIG_VIDEO_S5K4BA
static struct s5k4ba_mbus_platform_data s5k4ba_mbus_plat = {
	.id		= 0,
	.fmt = {
		.width	= 1600,
		.height	= 1200,
		/*.code	= V4L2_MBUS_FMT_UYVY8_2X8, */
		.code	= V4L2_MBUS_FMT_VYUY8_2X8,
	},
	.clk_rate	= 24000000UL,
#ifdef CONFIG_ITU_A
	.set_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_ITU_B
	.set_power	= smdk4x12_cam1_reset,
#endif
};

static struct i2c_board_info s5k4ba_info = {
	I2C_BOARD_INFO("S5K4BA", 0x2d),
	.platform_data = &s5k4ba_mbus_plat,
};
#endif

/* 2 MIPI Cameras */
#ifdef CONFIG_VIDEO_S5K4EA
static struct s5k4ea_mbus_platform_data s5k4ea_mbus_plat = {
#ifdef CONFIG_CSI_C
	.id		= 0,
	.set_power = smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_CSI_D
	.id		= 1,
	.set_power = smdk4x12_cam1_reset,
#endif
	.fmt = {
		.width	= 1920,
		.height	= 1080,
		.code	= V4L2_MBUS_FMT_VYUY8_2X8,
	},
	.clk_rate	= 24000000UL,
};

static struct i2c_board_info s5k4ea_info = {
	I2C_BOARD_INFO("S5K4EA", 0x2d),
	.platform_data = &s5k4ea_mbus_plat,
};
#endif

#ifdef CONFIG_VIDEO_M5MOLS
static struct m5mols_platform_data m5mols_platdata = {
#ifdef CONFIG_CSI_C
	.gpio_rst = EXYNOS4_GPX1(2), /* ISP_RESET */
#endif
#ifdef CONFIG_CSI_D
	.gpio_rst = EXYNOS4_GPX1(0), /* ISP_RESET */
#endif
	.enable_rst = true, /* positive reset */
	.irq = IRQ_EINT(27),
};

static struct i2c_board_info m5mols_board_info = {
	I2C_BOARD_INFO("M5MOLS", 0x1F),
	.platform_data = &m5mols_platdata,
};

#endif

#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
#ifdef CONFIG_VIDEO_S5K3H2
static struct i2c_board_info s5k3h2_sensor_info = {
	.type = "S5K3H2",
};
#endif
#ifdef CONFIG_VIDEO_S5K3H7
static struct i2c_board_info s5k3h7_sensor_info = {
	.type = "S5K3H7",
};
#endif
#ifdef CONFIG_VIDEO_S5K4E5
static struct i2c_board_info s5k4e5_sensor_info = {
	.type = "S5K4E5",
};
#endif
#ifdef CONFIG_VIDEO_S5K6A3
static struct i2c_board_info s5k6a3_sensor_info = {
	.type = "S5K6A3",
};
#endif
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
/* This is for platdata of fimc-lite */
#ifdef CONFIG_VIDEO_S5K3H2
static struct s3c_platform_camera s5k3h2 = {
	.type  = CAM_TYPE_MIPI,
	.use_isp = true,
	.inv_pclk = 0,
	.inv_vsync = 0,
	.inv_href = 0,
	.inv_hsync = 0,
};
#endif

#ifdef CONFIG_VIDEO_S5K3H7
static struct s3c_platform_camera s5k3h7 = {
	.type  = CAM_TYPE_MIPI,
	.use_isp = true,
	.inv_pclk = 0,
	.inv_vsync = 0,
	.inv_href = 0,
	.inv_hsync = 0,
};
#endif

#ifdef CONFIG_VIDEO_S5K4E5
static struct s3c_platform_camera s5k4e5 = {
	.type  = CAM_TYPE_MIPI,
	.use_isp = true,
	.inv_pclk = 0,
	.inv_vsync = 0,
	.inv_href = 0,
	.inv_hsync = 0,
};
#endif


#ifdef CONFIG_VIDEO_S5K6A3
static struct s3c_platform_camera s5k6a3 = {
	.type  = CAM_TYPE_MIPI,
	.use_isp = true,
	.inv_pclk = 0,
	.inv_vsync = 0,
	.inv_href = 0,
	.inv_hsync = 0,
};
#endif
#endif
#endif /* CONFIG_VIDEO_SAMSUNG_S5P_FIMC */

#ifdef CONFIG_S3C64XX_DEV_SPI
static struct s3c64xx_spi_csinfo spi0_csi[] = {
	[0] = {
		.line = EXYNOS4_GPB(1),
		.set_level = gpio_set_value,
		.fb_delay = 0x2,
	},
};

static struct spi_board_info spi0_board_info[] __initdata = {
	{
		.modalias = "spidev",
		.platform_data = NULL,
		.max_speed_hz = 10*1000*1000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data = &spi0_csi[0],
	}
};

#ifndef CONFIG_FB_S5P_LMS501KF03
static struct s3c64xx_spi_csinfo spi1_csi[] = {
	[0] = {
		.line = EXYNOS4_GPB(5),
		.set_level = gpio_set_value,
		.fb_delay = 0x2,
	},
};

static struct spi_board_info spi1_board_info[] __initdata = {
	{
		.modalias = "spidev",
		.platform_data = NULL,
		.max_speed_hz = 10*1000*1000,
		.bus_num = 1,
		.chip_select = 0,
		.mode = SPI_MODE_3,
		.controller_data = &spi1_csi[0],
	}
};
#endif

static struct s3c64xx_spi_csinfo spi2_csi[] = {
	[0] = {
		.line = EXYNOS4_GPC1(2),
		.set_level = gpio_set_value,
		.fb_delay = 0x2,
	},
};

static struct spi_board_info spi2_board_info[] __initdata = {
	{
		.modalias = "spidev",
		.platform_data = NULL,
		.max_speed_hz = 10*1000*1000,
		.bus_num = 2,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data = &spi2_csi[0],
	}
};
#endif

#ifdef CONFIG_FB_S3C
#if defined(CONFIG_LCD_AMS369FG06)
static int lcd_power_on(struct lcd_device *ld, int enable)
{
	return 1;
}

static int reset_lcd(struct lcd_device *ld)
{
/*	int err = 0;

	err = gpio_request_one(EXYNOS4_GPX0(6), GPIOF_OUT_INIT_HIGH, "GPX0");
	if (err) {
		printk(KERN_ERR "failed to request GPX0 for "
				"lcd reset control\n");
		return err;
	}
	gpio_set_value(EXYNOS4_GPX0(6), 0);
	mdelay(1);

	gpio_set_value(EXYNOS4_GPX0(6), 1);

	gpio_free(EXYNOS4_GPX0(6));
*/
	return 1;
}

static struct lcd_platform_data ams369fg06_platform_data = {
	.reset			= reset_lcd,
	.power_on		= lcd_power_on,
	.lcd_enabled		= 0,
	.reset_delay		= 100,	/* 100ms */
};
/*
#define		LCD_BUS_NUM	3
#define		DISPLAY_CS	EXYNOS4_GPB(5)
#define		DISPLAY_CLK	EXYNOS4_GPB(4)
#define		DISPLAY_SI	EXYNOS4_GPB(7)
*/
static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias		= "ams369fg06",
		.platform_data		= (void *)&ams369fg06_platform_data,
		.max_speed_hz		= 1200000,
		.bus_num		= LCD_BUS_NUM,
		.chip_select		= 0,
		.mode			= SPI_MODE_3,
		.controller_data	= (void *)DISPLAY_CS,
	}
};

static struct spi_gpio_platform_data ams369fg06_spi_gpio_data = {
	.sck	= DISPLAY_CLK,
	.mosi	= DISPLAY_SI,
	.miso	= -1,
	.num_chipselect = 1,
};

static struct platform_device s3c_device_spi_gpio = {
	.name	= "spi_gpio",
	.id	= LCD_BUS_NUM,
	.dev	= {
		.parent		= &s5p_device_fimd0.dev,
		.platform_data	= &ams369fg06_spi_gpio_data,
	},
};

static struct s3c_fb_pd_win smdk4x12_fb_win0 = {
	.win_mode = {
		.left_margin	= 9,
		.right_margin	= 9,
		.upper_margin	= 5,
		.lower_margin	= 5,
		.hsync_len	= 2,
		.vsync_len	= 2,
		.xres		= 480,
		.yres		= 800,
	},
	.virtual_x		= 480,
	.virtual_y		= 1600,
	.width			= 48,
	.height			= 80,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk4x12_fb_win1 = {
	.win_mode = {
		.left_margin	= 9,
		.right_margin	= 9,
		.upper_margin	= 5,
		.lower_margin	= 5,
		.hsync_len	= 2,
		.vsync_len	= 2,
		.xres		= 480,
		.yres		= 800,
	},
	.virtual_x		= 480,
	.virtual_y		= 1600,
	.width			= 48,
	.height			= 80,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk4x12_fb_win2 = {
	.win_mode = {
		.left_margin	= 9,
		.right_margin	= 9,
		.upper_margin	= 5,
		.lower_margin	= 5,
		.hsync_len	= 2,
		.vsync_len	= 2,
		.xres		= 480,
		.yres		= 800,
	},
	.virtual_x		= 480,
	.virtual_y		= 1600,
	.width			= 48,
	.height			= 80,
	.max_bpp		= 32,
	.default_bpp		= 24,
};
#elif defined (CONFIG_LCD_LMS501KF03)
static int lcd_power_on(struct lcd_device *ld, int enable)
{
	return 1;
}

static int reset_lcd(struct lcd_device *ld)
{
/*	int err = 0;

	if (samsung_board_rev_is_0_1()) {
		err = gpio_request_one(EXYNOS4212_GPM3(6),
				GPIOF_OUT_INIT_HIGH, "GPM3");
		if (err) {
			printk(KERN_ERR "failed to request GPM3 for "
					"lcd reset control\n");
			return err;
		}
		gpio_set_value(EXYNOS4212_GPM3(6), 0);
		mdelay(1);

		gpio_set_value(EXYNOS4212_GPM3(6), 1);

		gpio_free(EXYNOS4212_GPM3(6));
	} else {
		err = gpio_request_one(EXYNOS4_GPX1(5),
				GPIOF_OUT_INIT_HIGH, "GPX1");
		if (err) {
			printk(KERN_ERR "failed to request GPX1 for "
					"lcd reset control\n");
			return err;
		}
		gpio_set_value(EXYNOS4_GPX1(5), 0);
		mdelay(1);

		gpio_set_value(EXYNOS4_GPX1(5), 1);

		gpio_free(EXYNOS4_GPX1(5));
	}
*/
	return 1;
}

static struct lcd_platform_data lms501kf03_platform_data = {
	.reset			= reset_lcd,
	.power_on		= lcd_power_on,
	.lcd_enabled		= 0,
	.reset_delay		= 100,	/* 100ms */
};
/*
#define		LCD_BUS_NUM	3
#define		DISPLAY_CS	EXYNOS4_GPB(5)
#define		DISPLAY_CLK	EXYNOS4_GPB(4)
#define		DISPLAY_SI	EXYNOS4_GPB(7)
*/
static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias		= "lms501kf03",
		.platform_data		= (void *)&lms501kf03_platform_data,
		.max_speed_hz		= 1200000,
		.bus_num		= LCD_BUS_NUM,
		.chip_select		= 0,
		.mode			= SPI_MODE_3,
		.controller_data	= (void *)DISPLAY_CS,
	}
};

static struct spi_gpio_platform_data lms501kf03_spi_gpio_data = {
	.sck	= DISPLAY_CLK,
	.mosi	= DISPLAY_SI,
	.miso	= -1,
	.num_chipselect = 1,
};

static struct platform_device s3c_device_spi_gpio = {
	.name	= "spi_gpio",
	.id	= LCD_BUS_NUM,
	.dev	= {
		.parent		= &s5p_device_fimd0.dev,
		.platform_data	= &lms501kf03_spi_gpio_data,
	},
};

static struct s3c_fb_pd_win smdk4x12_fb_win0 = {
	.win_mode = {
		.left_margin	= 8,		/* HBPD */
		.right_margin	= 8,		/* HFPD */
		.upper_margin	= 6,	/* VBPD */
		.lower_margin	= 6,		/* VFPD */
		.hsync_len	= 6,		/* HSPW */
		.vsync_len	= 4,		/* VSPW */
		.xres		= 480,
		.yres		= 800,
	},
	.virtual_x		= 480,
	.virtual_y		= 1600,
	.width			= 48,
	.height			= 80,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk4x12_fb_win1 = {
	.win_mode = {
		.left_margin	= 8,		/* HBPD */
		.right_margin	= 8,		/* HFPD */
		.upper_margin	= 6,	/* VBPD */
		.lower_margin	= 6,		/* VFPD */
		.hsync_len	= 6,		/* HSPW */
		.vsync_len	= 4,		/* VSPW */
		.xres		= 480,
		.yres		= 800,
	},
	.virtual_x		= 480,
	.virtual_y		= 1600,
	.width			= 48,
	.height			= 80,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk4x12_fb_win2 = {
	.win_mode = {
		.left_margin	= 8,		/* HBPD */
		.right_margin	= 8,		/* HFPD */
		.upper_margin	= 6,	/* VBPD */
		.lower_margin	= 6,		/* VFPD */
		.hsync_len	= 6,		/* HSPW */
		.vsync_len	= 4,		/* VSPW */
		.xres		= 480,
		.yres		= 800,
	},
	.virtual_x		= 480,
	.virtual_y		= 1600,
	.width			= 48,
	.height			= 80,
	.max_bpp		= 32,
	.default_bpp		= 24,
};
#elif defined(CONFIG_LCD_WA101S)
static void lcd_wa101s_set_power(struct plat_lcd_data *pd,
				   unsigned int power)
{
/*	if (power) {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request_one(EXYNOS4_GPD0(1), GPIOF_OUT_INIT_HIGH, "GPD0");
		gpio_free(EXYNOS4_GPD0(1));
#endif
	} else {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request_one(EXYNOS4_GPD0(1), GPIOF_OUT_INIT_LOW, "GPD0");
		gpio_free(EXYNOS4_GPD0(1));
#endif
	}*/
}

static struct plat_lcd_data smdk4x12_lcd_wa101s_data = {
	.set_power		= lcd_wa101s_set_power,
};

static struct platform_device smdk4x12_lcd_wa101s = {
	.name			= "platform-lcd",
	.dev.parent		= &s5p_device_fimd0.dev,
	.dev.platform_data      = &smdk4x12_lcd_wa101s_data,
};
#ifndef CONFIG_FB_S3C
static struct s3c_fb_pd_win smdk4x12_fb_win0 = {
	.win_mode = {
		.left_margin	= 80,
		.right_margin	= 48,
		.upper_margin	= 14,
		.lower_margin	= 3,
		.hsync_len	= 32,
		.vsync_len	= 5,
		.xres		= 1360, /* real size : 1366 */
		.yres		= 768,
	},
	.virtual_x		= 1360, /* real size : 1366 */
	.virtual_y		= 768 * 2,
	.width			= 223,
	.height			= 125,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk4x12_fb_win1 = {
	.win_mode = {
		.left_margin	= 80,
		.right_margin	= 48,
		.upper_margin	= 14,
		.lower_margin	= 3,
		.hsync_len	= 32,
		.vsync_len	= 5,
		.xres		= 1360, /* real size : 1366 */
		.yres		= 768,
	},
	.virtual_x		= 1360, /* real size : 1366 */
	.virtual_y		= 768 * 2,
	.width			= 223,
	.height			= 125,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk4x12_fb_win2 = {
	.win_mode = {
		.left_margin	= 80,
		.right_margin	= 48,
		.upper_margin	= 14,
		.lower_margin	= 3,
		.hsync_len	= 32,
		.vsync_len	= 5,
		.xres		= 1360, /* real size : 1366 */
		.yres		= 768,
	},
	.virtual_x		= 1360, /* real size : 1366 */
	.virtual_y		= 768 * 2,
	.width			= 223,
	.height			= 125,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

#else

static struct s3c_fb_pd_win smdk4x12_fb_win0 = {
	.win_mode = {
		.left_margin	= 30,
		.right_margin	= 30,
		.upper_margin	= 10,
		.lower_margin	= 6,
		.hsync_len	= 9,
		.vsync_len	= 7,
		.xres		= 1280, /* real size : 1366 */
		.yres		= 800,
	},
	.virtual_x		= 1280, /* real size : 1366 */
	.virtual_y		= 800 * 2,
	.width			= 223,
	.height			= 125,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk4x12_fb_win1 = {
	.win_mode = {
		.left_margin	= 30,
		.right_margin	= 30,
		.upper_margin	= 10,
		.lower_margin	= 6,
		.hsync_len	= 9,
		.vsync_len	= 7,
		.xres		= 1280, /* real size : 1366 */
		.yres		= 800,
	},
	.virtual_x		= 1280, /* real size : 1366 */
	.virtual_y		= 800 * 2,
	.width			= 223,
	.height			= 125,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk4x12_fb_win2 = {
	.win_mode = {
		.left_margin	= 30,
		.right_margin	= 30,
		.upper_margin	= 10,
		.lower_margin	= 6,
		.hsync_len	= 9,
		.vsync_len	= 7,
		.xres		= 1280, /* real size : 1366 */
		.yres		= 800,
	},
	.virtual_x		= 1280, /* real size : 1366 */
	.virtual_y		= 800 * 2,
	.width			= 223,
	.height			= 125,
	.max_bpp		= 32,
	.default_bpp		= 24,
};


#endif


#elif defined(CONFIG_LCD_LTE480WV)
static void lcd_lte480wv_set_power(struct plat_lcd_data *pd,
				   unsigned int power)
{/*
	if (power) {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request_one(EXYNOS4_GPD0(1), GPIOF_OUT_INIT_HIGH, "GPD0");
		gpio_free(EXYNOS4_GPD0(1));
#endif
*/		/* fire nRESET on power up */
/*		gpio_request_one(EXYNOS4_GPX0(6), GPIOF_OUT_INIT_HIGH, "GPX0");
		mdelay(100);

		gpio_set_value(EXYNOS4_GPX0(6), 0);
		mdelay(10);

		gpio_set_value(EXYNOS4_GPX0(6), 1);
		mdelay(10);

		gpio_free(EXYNOS4_GPX0(6));
	} else {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request_one(EXYNOS4_GPD0(1), GPIOF_OUT_INIT_LOW, "GPD0");
		gpio_free(EXYNOS4_GPD0(1));
#endif
	}
*/
}

static struct plat_lcd_data smdk4x12_lcd_lte480wv_data = {
	.set_power		= lcd_lte480wv_set_power,
};

static struct platform_device smdk4x12_lcd_lte480wv = {
	.name			= "platform-lcd",
	.dev.parent		= &s5p_device_fimd0.dev,
	.dev.platform_data      = &smdk4x12_lcd_lte480wv_data,
};

static struct s3c_fb_pd_win smdk4x12_fb_win0 = {
	.win_mode = {
		.left_margin	= 13,
		.right_margin	= 8,
		.upper_margin	= 7,
		.lower_margin	= 5,
		.hsync_len	= 3,
		.vsync_len	= 1,
		.xres		= 800,
		.yres		= 480,
	},
	.virtual_x		= 800,
	.virtual_y		= 960,
	.width			= 104,
	.height			= 62,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk4x12_fb_win1 = {
	.win_mode = {
		.left_margin	= 13,
		.right_margin	= 8,
		.upper_margin	= 7,
		.lower_margin	= 5,
		.hsync_len	= 3,
		.vsync_len	= 1,
		.xres		= 800,
		.yres		= 480,
	},
	.virtual_x		= 800,
	.virtual_y		= 960,
	.width			= 104,
	.height			= 62,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk4x12_fb_win2 = {
	.win_mode = {
		.left_margin	= 13,
		.right_margin	= 8,
		.upper_margin	= 7,
		.lower_margin	= 5,
		.hsync_len	= 3,
		.vsync_len	= 1,
		.xres		= 800,
		.yres		= 480,
	},
	.virtual_x		= 800,
	.virtual_y		= 960,
	.width			= 104,
	.height			= 62,
	.max_bpp		= 32,
	.default_bpp		= 24,
};
#elif defined(CONFIG_LCD_MIPI_S6E63M0)
static void mipi_lcd_set_power(struct plat_lcd_data *pd,
				unsigned int power)
{/*
	gpio_request_one(EXYNOS4_GPX2(7), GPIOF_OUT_INIT_HIGH, "GPX2");

	mdelay(100);
	if (power) {
*/		/* fire nRESET on power up */
/*		gpio_set_value(EXYNOS4_GPX2(7), 0);
		mdelay(100);
		gpio_set_value(EXYNOS4_GPX2(7), 1);
		mdelay(100);
		gpio_free(EXYNOS4_GPX2(7));
	} else {
*/		/* fire nRESET on power off */
/*		gpio_set_value(EXYNOS4_GPX2(7), 0);
		mdelay(100);
		gpio_set_value(EXYNOS4_GPX2(7), 1);
		mdelay(100);
		gpio_free(EXYNOS4_GPX2(7));
	}
*/
}

static struct plat_lcd_data smdk4x12_mipi_lcd_data = {
	.set_power	= mipi_lcd_set_power,
};

static struct platform_device smdk4x12_mipi_lcd = {
	.name			= "platform-lcd",
	.dev.parent		= &s5p_device_fimd0.dev,
	.dev.platform_data	= &smdk4x12_mipi_lcd_data,
};

static struct s3c_fb_pd_win smdk4x12_fb_win0 = {
	.win_mode = {
		.left_margin	= 0x16,
		.right_margin	= 0x16,
		.upper_margin	= 0x1,
		.lower_margin	= 0x28,
		.hsync_len	= 0x2,
		.vsync_len	= 0x3,
		.xres		= 480,
		.yres		= 800,
	},
	.virtual_x		= 480,
	.virtual_y		= 1600,
	.width			= 48,
	.height			= 80,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk4x12_fb_win1 = {
	.win_mode = {
		.left_margin	= 0x16,
		.right_margin	= 0x16,
		.upper_margin	= 0x1,
		.lower_margin	= 0x28,
		.hsync_len	= 0x2,
		.vsync_len	= 0x3,
		.xres		= 480,
		.yres		= 800,
	},
	.virtual_x		= 480,
	.virtual_y		= 1600,
	.width			= 48,
	.height			= 80,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk4x12_fb_win2 = {
	.win_mode = {
		.left_margin	= 0x16,
		.right_margin	= 0x16,
		.upper_margin	= 0x1,
		.lower_margin	= 0x28,
		.hsync_len	= 0x2,
		.vsync_len	= 0x3,
		.xres		= 480,
		.yres		= 800,
	},
	.virtual_x		= 480,
	.virtual_y		= 1600,
	.width			= 48,
	.height			= 80,
	.max_bpp		= 32,
	.default_bpp		= 24,
};
#endif

static struct s3c_fb_platdata smdk4x12_lcd0_pdata __initdata = {
#if defined(CONFIG_LCD_AMS369FG06) || defined(CONFIG_LCD_WA101S) || \
	defined(CONFIG_LCD_LTE480WV) || defined(CONFIG_LCD_LMS501KF03) || \
	defined(CONFIG_LCD_MIPI_S6E63M0)
	.win[0]		= &smdk4x12_fb_win0,
	.win[1]		= &smdk4x12_fb_win1,
	.win[2]		= &smdk4x12_fb_win2,
#endif
	.default_win	= 2,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
#if defined(CONFIG_LCD_AMS369FG06)
	.vidcon1	= VIDCON1_INV_VCLK | VIDCON1_INV_VDEN |
			  VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
#elif defined(CONFIG_LCD_LMS501KF03)
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
#elif defined(CONFIG_LCD_WA101S)
	.vidcon1	= VIDCON1_INV_VCLK | VIDCON1_INV_HSYNC |
			  VIDCON1_INV_VSYNC,
#elif defined(CONFIG_LCD_LTE480WV)
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
#endif
/*
	.setup_gpio	= exynos4_fimd0_gpio_setup_24bpp,
*/
};
#endif

#ifdef CONFIG_FB_S5P
#ifdef CONFIG_FB_S5P_LMS501KF03
static struct s3c_platform_fb lms501kf03_data __initdata = {
	.hw_ver = 0x70,
	.clk_name = "sclk_lcd",
	.nr_wins = 5,
	.default_win = CONFIG_FB_S5P_DEFAULT_WINDOW,
	.swap = FB_SWAP_HWORD | FB_SWAP_WORD,
};
/*
#define		LCD_BUS_NUM	3
#define		DISPLAY_CS	EXYNOS4_GPB(5)
#define		DISPLAY_CLK	EXYNOS4_GPB(4)
#define		DISPLAY_SI	EXYNOS4_GPB(7)
*/
static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias	= "lms501kf03",
		.platform_data	= NULL,
		.max_speed_hz	= 1200000,
		.bus_num	= LCD_BUS_NUM,
		.chip_select	= 0,
		.mode		= SPI_MODE_3,
		.controller_data = (void *)DISPLAY_CS,
	}
};
static struct spi_gpio_platform_data lms501kf03_spi_gpio_data = {
	.sck	= DISPLAY_CLK,
	.mosi	= DISPLAY_SI,
	.miso	= -1,
	.num_chipselect = 1,
};

static struct platform_device s3c_device_spi_gpio = {
	.name	= "spi_gpio",
	.id	= LCD_BUS_NUM,
	.dev	= {
		.parent		= &s3c_device_fb.dev,
		.platform_data	= &lms501kf03_spi_gpio_data,
	},
};
#elif defined(CONFIG_FB_S5P_DUMMY_MIPI_LCD)
#define		LCD_BUS_NUM	3

static struct s3cfb_lcd dummy_mipi_lcd = {
//Cellon modify start, Jacob, 2012/07/29, for resolution and timing issue
	.width = 720,
	.height = 1280,
	.bpp = 24,

	.freq = 60,

	.timing = {
		.h_fp = 0x30,
		.h_bp = 0x30,
		.h_sw = 0x2,
		.v_fp = 0x6,
		.v_fpe = 3,
		.v_bp = 0x0,
		.v_bpe = 0,
		.v_sw = 2,
		.cmd_allow_len = 0x4,
	},
// Cellon modify end, Jacob, 2012/07/29
	.polarity = {
		.rise_vclk = 0,
		.inv_hsync = 0,
		.inv_vsync = 0,
		.inv_vden = 0,
	},
};

static struct s3c_platform_fb fb_platform_data __initdata = {
	.hw_ver		= 0x70,
	.clk_name	= "sclk_lcd",
	.nr_wins	= 5,
	.default_win	= CONFIG_FB_S5P_DEFAULT_WINDOW,
	.swap		= FB_SWAP_HWORD | FB_SWAP_WORD,
};

static void lcd_cfg_gpio(void)
{
	return;
}

static int reset_lcd(void)
{
	int err = -1;
	int level_value = -1;

#ifdef CONFIG_MIPI_DSI_RST_N
	err = gpio_request(EXYNOS4_GPX3(2), "GPX3(2)");
	if (err) {
		printk(KERN_ERR "failed to request GPX3(2) for lcd reset control\n");
		return err;
	}

	err = -1;
	err = gpio_request(EXYNOS4_GPF1(2), "GPF1(2)");
	if (err) {
		printk(KERN_ERR "failed to request GPF1(2) for lcd ID\n");
		return err;
	}

	s3c_gpio_cfgpin(EXYNOS4_GPF1(2),S3C_GPIO_INPUT);
	s3c_gpio_setpull(EXYNOS4_GPF1(2), S3C_GPIO_PULL_UP);
	msleep(2);
	level_value = gpio_get_value(EXYNOS4_GPF1(2));
	gpio_free(EXYNOS4_GPF1(2));
	
//	printk("DSI_RST_N level_value = %d\n",level_value);

	if(level_value == 1){
	gpio_direction_output(EXYNOS4_GPX3(2), 0);

	usleep_range(20000, 20000);
	gpio_set_value(EXYNOS4_GPX3(2), 1);
	usleep_range(10000, 10000);
/*
	usleep_range(13000, 13000);
	gpio_set_value(EXYNOS4_GPX3(2), 0);

	usleep_range(1000, 1000);
	gpio_set_value(EXYNOS4_GPX3(2), 1);

	usleep_range(20000, 20000);
*/
	gpio_free(EXYNOS4_GPX3(2));
	}else{

	gpio_direction_output(EXYNOS4_GPX3(2), 1);
	gpio_set_value(EXYNOS4_GPX3(2), 0);
	msleep(20);

	gpio_set_value(EXYNOS4_GPX3(2), 1);

	msleep(1);
	gpio_set_value(EXYNOS4_GPX3(2), 0);
	msleep(1);
	gpio_set_value(EXYNOS4_GPX3(2), 1);

	msleep(20);
	gpio_free(EXYNOS4_GPX3(2));

	}
#else

#endif


#ifdef CONFIG_CPU_EXYNOS4212

#else

#endif
// Cellon modify end, Jacob, 2012/08/09
	return 0;
}

static int lcd_power_on(void *pdev, int enable)
{

	return 1;
}

static void __init mipi_fb_init(void)
{
	struct s5p_platform_dsim *dsim_pd = NULL;
	struct mipi_ddi_platform_data *mipi_ddi_pd = NULL;
	struct dsim_lcd_config *dsim_lcd_info = NULL;

	/* gpio pad configuration for rgb and spi interface. */
	lcd_cfg_gpio();

	/*
	 * register lcd panel data.
	 */
	dsim_pd = (struct s5p_platform_dsim *)
		s5p_device_dsim.dev.platform_data;

	strcpy(dsim_pd->lcd_panel_name, "dummy_mipi_lcd");

	dsim_lcd_info = dsim_pd->dsim_lcd_info;
	dsim_lcd_info->lcd_panel_info = (void *)&dummy_mipi_lcd;

	mipi_ddi_pd = (struct mipi_ddi_platform_data *)
		dsim_lcd_info->mipi_ddi_pd;
	mipi_ddi_pd->lcd_reset = reset_lcd;
	mipi_ddi_pd->lcd_power_on = lcd_power_on;

	platform_device_register(&s5p_device_dsim);

	s3cfb_set_platdata(&fb_platform_data);

	printk(KERN_INFO "platform data of %s lcd panel has been registered.\n",
			dsim_pd->lcd_panel_name);
}
#endif
#endif

static int exynos4_notifier_call(struct notifier_block *this,
					unsigned long code, void *_cmd)
{
	int mode = 0;

	if ((code == SYS_RESTART) && _cmd)
		if (!strcmp((char *)_cmd, "recovery"))
			mode = 0xf;

	__raw_writel(mode, REG_INFORM4);

	return NOTIFY_DONE;
}

static struct notifier_block exynos4_reboot_notifier = {
	.notifier_call = exynos4_notifier_call,
};

#ifdef CONFIG_EXYNOS4_DEV_DWMCI
static void exynos_dwmci_cfg_gpio(int width)
{
	unsigned int gpio;

	for (gpio = EXYNOS4_GPK0(0); gpio < EXYNOS4_GPK0(2); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	}

	switch (width) {
	case 8:
		for (gpio = EXYNOS4_GPK1(3); gpio <= EXYNOS4_GPK1(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(4));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
	case 4:
		for (gpio = EXYNOS4_GPK0(3); gpio <= EXYNOS4_GPK0(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
		break;
	case 1:
		gpio = EXYNOS4_GPK0(3);
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	default:
		break;
	}
}

static struct dw_mci_board exynos_dwmci_pdata __initdata = {
	.num_slots		= 1,
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION | DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 100 * 1000 * 1000,
	.caps			= MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR |
				MMC_CAP_8_BIT_DATA | MMC_CAP_CMD23,
	.fifo_depth		= 0x80,
	.detect_delay_ms	= 200,
	.hclk_name		= "dwmci",
	.cclk_name		= "sclk_dwmci",
	.cfg_gpio		= exynos_dwmci_cfg_gpio,
};
#endif
// Cellon add start, Ted Shi, 2012/08/09, for porting bcm4330 wifi 
#ifdef CONFIG_BCMDHD_WIFI
static DEFINE_MUTEX(notify_lock);

static void (*hsmmc3_notify_func)(struct platform_device *, int state);
static int ext_cd_init_hsmmc3(void (*notify_func)( 
			struct platform_device *, int state)) 
{
	mutex_lock(&notify_lock); 
	WARN_ON(hsmmc3_notify_func); 
	hsmmc3_notify_func = notify_func; 
	mutex_unlock(&notify_lock); 
	return 0; 
}
static int ext_cd_cleanup_hsmmc3(void (*notify_func)(
			struct platform_device *, int state))
{
	mutex_lock(&notify_lock);
	WARN_ON(hsmmc3_notify_func != notify_func);
	hsmmc3_notify_func = NULL;
	mutex_unlock(&notify_lock);
	return 0;
}

/*
 * call this when you need sd stack to recognize insertion or removal of card
 * that can't be told by SDHCI regs
 */
void mmc_force_presence_change(struct platform_device *pdev)
{
	void (*notify_func)(struct platform_device *, int state) = NULL;
	mutex_lock(&notify_lock);
#ifdef CONFIG_S3C_DEV_HSMMC3
	if (pdev == &s3c_device_hsmmc3)
		notify_func = hsmmc3_notify_func;
#endif

	if (notify_func)
		notify_func(pdev, 1);
	else
		pr_warn("%s: called for device with no notifier\n", __func__);
	mutex_unlock(&notify_lock);
}
EXPORT_SYMBOL_GPL(mmc_force_presence_change);
#endif
// Cellon add end, Ted Shi, 2012/08/09

#ifdef CONFIG_S3C_DEV_HSMMC
static struct s3c_sdhci_platdata smdk4x12_hsmmc0_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH0_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC1
static struct s3c_sdhci_platdata smdk4x12_hsmmc1_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC2
static struct s3c_sdhci_platdata smdk4x12_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_GPIO,//lisw sd    S3C_SDHCI_CD_INTERNAL,
	.ext_cd_gpio            =EXYNOS4_GPX0(7), //lisw sd
    	.ext_cd_gpio_invert     = 1,//lisw sd
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH2_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC3
static struct s3c_sdhci_platdata smdk4x12_hsmmc3_pdata __initdata = {
// Cellon add end, Ted Shi, 2012/08/09, for porting bcm4330 wifi
	// SEMCO
	#ifdef CONFIG_BCMDHD_WIFI
	//.cd_type		= S3C_SDHCI_CD_INTERNAL,
	//.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
	.cd_type		= S3C_SDHCI_CD_EXTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
	.pm_flags = S3C_SDHCI_PM_IGNORE_SUSPEND_RESUME,
	.ext_cd_init = ext_cd_init_hsmmc3,
	.ext_cd_cleanup = ext_cd_cleanup_hsmmc3,
// Cellon add end, Ted Shi, 2012/08/09
	#else
	.cd_type		= S3C_SDHCI_CD_PERMANENT,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
	#endif
};
#endif

#ifdef CONFIG_S5P_DEV_MSHC
static struct s3c_mshci_platdata exynos4_mshc_pdata __initdata = {
	.cd_type		= S3C_MSHCI_CD_PERMANENT,
	.clk_type		= S3C_MSHCI_CLK_DIV_EXTERNAL, //lisw ms
	.has_wp_gpio		= true,
	.wp_gpio		= 0xffffffff,
#if defined(CONFIG_EXYNOS4_MSHC_8BIT) && \
	defined(CONFIG_EXYNOS4_MSHC_DDR)
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA | MMC_CAP_1_8V_DDR |
				  MMC_CAP_UHS_DDR50,
#elif defined(CONFIG_EXYNOS4_MSHC_8BIT)
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#elif defined(CONFIG_EXYNOS4_MSHC_DDR)
	.host_caps		= MMC_CAP_1_8V_DDR | MMC_CAP_UHS_DDR50,
#endif
};
#endif

#ifdef CONFIG_USB_EHCI_S5P
static struct s5p_ehci_platdata smdk4x12_ehci_pdata;

static void __init smdk4x12_ehci_init(void)
{
	struct s5p_ehci_platdata *pdata = &smdk4x12_ehci_pdata;

	s5p_ehci_set_platdata(pdata);
}
// USB3503A, HSIC1 -> USB Host
#define GPIO_HUB_RESET EXYNOS4_GPL2(2)
#define GPIO_HUB_CONNECT EXYNOS4_GPK3(2)
#ifndef CONFIG_TC4_DVT
#define I2C_SDA6 EXYNOS4_GPC1(3)
#define I2C_SCL6 EXYNOS4_GPC1(4)
#endif
void usb_hub_gpio_init()
{
#if 0
	//printk("shengliang set HUB_RESET & HUB_CONNECT\n");
	// HUB_RESET
	gpio_request(GPIO_HUB_RESET, "GPIO_HUB_RESET");
	gpio_direction_output(GPIO_HUB_RESET, 1);
	s3c_gpio_setpull(GPIO_HUB_RESET, S3C_GPIO_PULL_NONE);
	gpio_free(GPIO_HUB_RESET);

	// HUB_CONNECT
	gpio_request(GPIO_HUB_CONNECT, "GPIO_HUB_CONNECT");
	gpio_direction_output(GPIO_HUB_CONNECT, 1);
	s3c_gpio_setpull(GPIO_HUB_CONNECT, S3C_GPIO_PULL_NONE);
	gpio_free(GPIO_HUB_CONNECT);

#ifndef CONFIG_TC4_DVT
	// I2C_SDA6
	gpio_request(I2C_SDA6, "I2C_SDA6");
	gpio_direction_output(I2C_SDA6,1);
	s3c_gpio_setpull(I2C_SDA6, S3C_GPIO_PULL_NONE);
	gpio_free(I2C_SDA6);

	// I2C_SCL6
	gpio_request(I2C_SCL6, "I2C_SCL6");
	gpio_direction_output(I2C_SCL6,1);
	s3c_gpio_setpull(I2C_SCL6, S3C_GPIO_PULL_NONE);
	gpio_free(I2C_SCL6);
#endif
#endif
}
#endif

#ifdef CONFIG_USB_OHCI_S5P
static struct s5p_ohci_platdata smdk4x12_ohci_pdata;

static void __init smdk4x12_ohci_init(void)
{
	struct s5p_ohci_platdata *pdata = &smdk4x12_ohci_pdata;

	s5p_ohci_set_platdata(pdata);
}
#endif

/* USB GADGET */
#ifdef CONFIG_USB_GADGET
static struct s5p_usbgadget_platdata smdk4x12_usbgadget_pdata;

static void __init smdk4x12_usbgadget_init(void)
{
	struct s5p_usbgadget_platdata *pdata = &smdk4x12_usbgadget_pdata;

	s5p_usbgadget_set_platdata(pdata);
}
#endif

/* NFC */
#ifdef	CONFIG_S3FHRN2
#ifdef	CONFIG_S3FHRN2_I2C_GPIO
#define GPIO_NFC_SDA	EXYNOS4_GPL2(4)
#define GPIO_NFC_SCL	EXYNOS4_GPL2(3)
#else
#define GPIO_NFC_I2C	EXYNOS4_GPB(0)
#endif
#define GPIO_NFC_IRQ	EXYNOS4_GPX1(1)
#define GPIO_NFC_VEN	EXYNOS4_GPL0(3)
#define GPIO_NFC_FIRM	EXYNOS4_GPL2(5)

static void exynos4_init_nfc_gpio(struct platform_device *dev)
{
#ifdef CONFIG_S3FHRN2_I2C
#ifdef CONFIG_S3FHRN2_I2C_GPIO
	s3c_gpio_cfgpin(GPIO_NFC_SDA, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_NFC_SDA, S3C_GPIO_PULL_NONE);
	s5p_gpio_set_drvstr(GPIO_NFC_SDA, S5P_GPIO_DRVSTR_LV1);

	s3c_gpio_cfgpin(GPIO_NFC_SCL, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_NFC_SCL, S3C_GPIO_PULL_NONE);
	s5p_gpio_set_drvstr(GPIO_NFC_SCL, S5P_GPIO_DRVSTR_LV1);
#else
	s3c_gpio_cfgall_range(GPIO_NFC_I2C, 2,
		S3C_GPIO_SFN(3), S3C_GPIO_PULL_UP);
#endif
	s3c_gpio_cfgpin(GPIO_NFC_IRQ, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(GPIO_NFC_IRQ, S3C_GPIO_PULL_NONE);
	s5p_gpio_set_drvstr(GPIO_NFC_IRQ, S5P_GPIO_DRVSTR_LV1);
#endif
	s3c_gpio_cfgpin(GPIO_NFC_VEN, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_NFC_VEN, S3C_GPIO_PULL_NONE);
	s5p_gpio_set_drvstr(GPIO_NFC_VEN, S5P_GPIO_DRVSTR_LV1);
	s5p_gpio_set_pd_cfg(GPIO_NFC_VEN, S5P_GPIO_PD_PREV_STATE);

	s3c_gpio_cfgpin(GPIO_NFC_FIRM, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_NFC_FIRM, S3C_GPIO_PULL_NONE);
	s5p_gpio_set_drvstr(GPIO_NFC_FIRM, S5P_GPIO_DRVSTR_LV1);
	s5p_gpio_set_pd_cfg(GPIO_NFC_FIRM, S5P_GPIO_PD_PREV_STATE);
}
#endif

#ifdef CONFIG_S3FHRN2_I2C
static struct s3fhrn2_platform_data exynos4_nfc_info = {
	.irq = GPIO_NFC_IRQ,
	.ven = GPIO_NFC_VEN,
	.firm = GPIO_NFC_FIRM,
	.cfg_gpio = exynos4_init_nfc_gpio,
};
#endif

#ifdef CONFIG_S3FHRN2_I2C_GPIO
static struct i2c_gpio_platform_data i2c_nfc_platdata = {
	.sda_pin	= GPIO_NFC_SDA,
	.scl_pin	= GPIO_NFC_SCL,
	.udelay		= 2,
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device s3c_device_i2c_nfc_gpio = {
	.name		=  "i2c-gpio",
	//.id		= 4,
	.id	= 2,
	.dev.platform_data	= &i2c_nfc_platdata,
};
#endif

#ifdef CONFIG_S3FHRN2_UART
static struct platform_device s3c_device_nfc_uart = {
	.name		= S3FHRN2_DRIVER_NAME,
	.dev.platform_data	= &exynos4_nfc_info,
};
#endif
static struct regulator_consumer_supply max8952_supply =
	REGULATOR_SUPPLY("vdd_mif", NULL);

static struct regulator_init_data max8952_init_data = {
	.constraints	= {
		.name		= "vdd_mif range",
		.min_uV		= 850000,
		.max_uV		= 1050000,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.uV		= 1100000,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8952_supply,
};

static struct max8649_platform_data exynos4_max8952_info = {
	.mode		= 1,	/* VID1 = 0, VID0 = 1 */
	.extclk		= 0,
	.ramp_timing	= MAX8649_RAMP_32MV,
	.regulator	= &max8952_init_data,
};

/* max8997 -- modified by yulu@111122*/

static struct regulator_consumer_supply max8997_buck1 =
	REGULATOR_SUPPLY("vdd_arm", NULL);

static struct regulator_consumer_supply max8997_buck2 =
	REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply max8997_buck3 =
	REGULATOR_SUPPLY("vdd_g3d", NULL);

static struct regulator_consumer_supply max8997_buck4 =
	REGULATOR_SUPPLY("vdd12_5m", NULL);

static struct regulator_consumer_supply max8997_buck5 =
	REGULATOR_SUPPLY("vdd_m12", NULL);

static struct regulator_consumer_supply max8997_buck6 =
	REGULATOR_SUPPLY("vdd33_lcd", NULL);

static struct regulator_consumer_supply max8997_buck7 =
	REGULATOR_SUPPLY("vdd_buck7", NULL);

static struct regulator_consumer_supply max8997_boost =
	REGULATOR_SUPPLY("vdd_50", NULL);

static struct regulator_consumer_supply __initdata ldo1_consumer =
	REGULATOR_SUPPLY("vdd18_hsic", NULL);

static struct regulator_consumer_supply __initdata ldo2_consumer =
	REGULATOR_SUPPLY("vdd_alive", NULL);

static struct regulator_consumer_supply __initdata ldo3_consumer =
	REGULATOR_SUPPLY("vdd_ldo3", NULL);

static struct regulator_consumer_supply __initdata ldo4_consumer =
	REGULATOR_SUPPLY("vdd_ldo4", NULL);

static struct regulator_consumer_supply __initdata ldo5_consumer =
	REGULATOR_SUPPLY("vdd18_abb", NULL);

static struct regulator_consumer_supply __initdata ldo6_consumer =
	REGULATOR_SUPPLY("vddioap_18", NULL);

static struct regulator_consumer_supply __initdata ldo7_consumer =
	REGULATOR_SUPPLY("vddioperi_18", NULL);

static struct regulator_consumer_supply __initdata ldo8_consumer =
	REGULATOR_SUPPLY("vdd33_uotg", NULL);

static struct regulator_consumer_supply __initdata ldo9_consumer =
	REGULATOR_SUPPLY("vdd_sys", NULL);

static struct regulator_consumer_supply __initdata ldo10_consumer =
	REGULATOR_SUPPLY("vdd_pll", NULL);

static struct regulator_consumer_supply __initdata ldo11_consumer =
	REGULATOR_SUPPLY("vdd10_ush", NULL);

static struct regulator_consumer_supply __initdata ldo12_consumer =
	REGULATOR_SUPPLY("vdd18_cam", NULL);

static struct regulator_consumer_supply __initdata ldo13_consumer =
	REGULATOR_SUPPLY("vdd10_mipi", NULL);

static struct regulator_consumer_supply __initdata ldo14_consumer =
	REGULATOR_SUPPLY("vdd18_mipi", NULL);

static struct regulator_consumer_supply __initdata ldo15_consumer =
	REGULATOR_SUPPLY("vdd28_af", NULL);

static struct regulator_consumer_supply __initdata ldo16_consumer =
	REGULATOR_SUPPLY("vdd28_cam", NULL);

static struct regulator_consumer_supply __initdata ldo17_consumer =
	REGULATOR_SUPPLY("vdd33_a31", NULL);

static struct regulator_consumer_supply __initdata ldo18_consumer =
	REGULATOR_SUPPLY("vdd18_a31", NULL);

static struct regulator_consumer_supply __initdata ldo21_consumer =
	REGULATOR_SUPPLY("vddq_m12", NULL);


/*#define REGULATOR_INIT(_ldo, _name, _min_uV, _max_uV, _always_on, _ops_mask,\
		_disabled) \
	static struct regulator_init_data _ldo##_init_data = {		\
		.constraints = {					\
			.name	= _name,				\
			.min_uV = _min_uV,				\
			.max_uV = _max_uV,				\
			.always_on	= _always_on,			\
			.boot_on	= _always_on,			\
			.apply_uV	= 1,				\
			.valid_ops_mask = _ops_mask,			\
			.state_mem	= {				\
				.disabled	= _disabled,		\
				.enabled	= !(_disabled),		\
			}						\
		},							\
		.num_consumer_supplies = ARRAY_SIZE(_ldo##_supply),	\
		.consumer_supplies = &_ldo##_supply[0],			\
	};
*/

static struct regulator_init_data __initdata max8997_ldo1_data = {
	.constraints	= {
		.name		= "vdd18_hsic",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo1_consumer,
};

static struct regulator_init_data __initdata max8997_ldo2_data = {
	.constraints	= {
		.name		= "vdd_alive",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 0,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo2_consumer,
};

static struct regulator_init_data __initdata max8997_ldo3_data = {
	.constraints	= {
		.name		= "vdd_ldo3",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.apply_uV	= 1,
		.always_on	= 0,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo3_consumer,
};

static struct regulator_init_data __initdata max8997_ldo4_data = {
	.constraints	= {
		.name		= "vdd_ldo4",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 0,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo4_consumer,
};

static struct regulator_init_data __initdata max8997_ldo5_data = {
	.constraints	= {
		.name		= "vdd18_abb",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo5_consumer,
};

static struct regulator_init_data __initdata max8997_ldo6_data = {
	.constraints	= {
		.name		= "vddioap_18",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 0,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo6_consumer,
};

static struct regulator_init_data __initdata max8997_ldo7_data = {
	.constraints	= {
		.name		= "vddioperi_18",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 0,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo7_consumer,
};

static struct regulator_init_data __initdata max8997_ldo8_data = {
	.constraints	= {
		.name		= "vdd33_uotg",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo8_consumer,
};

static struct regulator_init_data __initdata max8997_ldo9_data = {
	.constraints	= {
		.name		= "vdd_sys",
		.min_uV		= 3000000,
		.max_uV		= 3000000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE|REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 0,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo9_consumer,
};

static struct regulator_init_data __initdata max8997_ldo10_data = {
	.constraints	= {
		.name		= "vdd_pll",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo10_consumer,
};

static struct regulator_init_data __initdata max8997_ldo11_data = {
	.constraints	= {
		.name		= "vdd10_ush",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo11_consumer,
};

static struct regulator_init_data __initdata max8997_ldo12_data = {
	.constraints	= {
		.name		= "vdd18_cam",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo12_consumer,
};

static struct regulator_init_data __initdata max8997_ldo13_data = {
	.constraints	= {
		.name		= "vdd10_mipi",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo13_consumer,
};


static struct regulator_init_data __initdata max8997_ldo14_data = {
	.constraints	= {
		.name		= "vdd18_mipi",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo14_consumer,
};

static struct regulator_init_data __initdata max8997_ldo15_data = {
	.constraints	= {
		.name		= "vdd28_af",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo15_consumer,
};

static struct regulator_init_data __initdata max8997_ldo16_data = {
	.constraints	= {
		.name		= "vdd28_cam",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo16_consumer,
};

static struct regulator_init_data __initdata max8997_ldo17_data = {
	.constraints	= {
		.name		= "vdd33_a31",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo17_consumer,
};

static struct regulator_init_data __initdata max8997_ldo18_data = {
	.constraints	= {
		.name		= "vdd18_a31",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo18_consumer,
};

static struct regulator_init_data __initdata max8997_ldo21_data = {
	.constraints	= {
		.name		= "vddq_m12",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo21_consumer,
};

static struct regulator_init_data __initdata max8997_buck1_data = {
	.constraints	= {
		.name		= "vdd_arm range",
		.min_uV		= 925000,
		.max_uV		= 1400000,
		.always_on	= 1,
		.boot_on		= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE|
					REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.mode = REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8997_buck1,
};

static struct regulator_init_data __initdata max8997_buck2_data = {
	.constraints	= {
		.name		= "vdd_int range",
		.min_uV		= 950000,
		.max_uV		= 1200000,
		.always_on	= 1,
		.boot_on		= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE|
					REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.mode = REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8997_buck2,
};

static struct regulator_init_data __initdata max8997_buck3_data = {
	.constraints	= {
		.name		= "vdd_g3d",
		.min_uV		= 1150000,			//1150000
		.max_uV		= 1150000,
		.always_on	= 1,
		.boot_on		= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.mode = REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8997_buck3,
};

static struct regulator_init_data __initdata max8997_buck4_data = {
	.constraints	= {
		.name		= "vdd12_5m",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.always_on	= 1,
		.boot_on		= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8997_buck4,
};

static struct regulator_init_data __initdata max8997_buck5_data = {
	.constraints	= {
		.name		= "vdd_m12",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.always_on	= 1,
		.boot_on		= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.mode = REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8997_buck5,
};

static struct regulator_init_data __initdata max8997_buck6_data = {
	.constraints	= {
		.name		= "vdd33_lcd",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.always_on	= 1,
		.boot_on		= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.mode = REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8997_buck6,
};

static struct regulator_init_data __initdata max8997_buck7_data = {
	.constraints	= {
		.name		= "vdd_buck7",
		.min_uV		= 2000000,
		.max_uV		= 2000000,
		.always_on	= 1,
		.boot_on		= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8997_buck7,
};

static struct regulator_init_data __initdata max8997_boost_data = {
	.constraints	= {
		.name		= "vdd_50",
		.min_uA		= 23440,
		.max_uA		= 750000,
		.valid_ops_mask	= REGULATOR_CHANGE_CURRENT |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8997_boost,
};



static struct max8997_regulator_data __initdata max8997_regulators[] = {
	{ MAX8997_LDO1, &max8997_ldo1_data, },	
	{ MAX8997_LDO2, &max8997_ldo2_data, },
	//{ MAX8997_LDO3, &max8997_ldo3_data, },
	//{ MAX8997_LDO4, &max8997_ldo4_data, },
	{ MAX8997_LDO5, &max8997_ldo5_data, },
	{ MAX8997_LDO6, &max8997_ldo6_data, },	
	{ MAX8997_LDO7, &max8997_ldo7_data, },
	{ MAX8997_LDO8, &max8997_ldo8_data, },
	//{ MAX8997_LDO9, &max8997_ldo9_data, },
	{ MAX8997_LDO10, &max8997_ldo10_data, },
	{ MAX8997_LDO11, &max8997_ldo11_data, },
	{ MAX8997_LDO12, &max8997_ldo12_data, },
	{ MAX8997_LDO13, &max8997_ldo13_data, },
	{ MAX8997_LDO14, &max8997_ldo14_data, },
	{ MAX8997_LDO15, &max8997_ldo15_data, },
	{ MAX8997_LDO16, &max8997_ldo16_data, },
	{ MAX8997_LDO17, &max8997_ldo17_data, },
	{ MAX8997_LDO18, &max8997_ldo18_data, },
	{ MAX8997_LDO21, &max8997_ldo21_data, },
	{ MAX8997_BUCK1, &max8997_buck1_data, },
	{ MAX8997_BUCK2, &max8997_buck2_data, },
	{ MAX8997_BUCK3, &max8997_buck3_data, },
	{ MAX8997_BUCK4, &max8997_buck4_data, },
	{ MAX8997_BUCK5, &max8997_buck5_data, },
	{ MAX8997_BUCK6, &max8997_buck6_data, },
	{ MAX8997_BUCK7, &max8997_buck7_data, },	
	{ MAX8997_BOOST, &max8997_boost_data, },		
};

static struct max8997_platform_data __initdata exynos4_max8997_info = {
	.num_regulators = ARRAY_SIZE(max8997_regulators),
	.regulators     = max8997_regulators,
	//.irq_base	= IRQ_BOARD_START,	
	//.wakeup		= 1,
	#if 1
	.buck1_voltage[0] = 1200000, /* 1.1V */
	.buck1_voltage[1] = 1150000, /* 1.1V */
	.buck1_voltage[2] = 1125000, /* 1.1V */
	.buck1_voltage[3] = 1100000, /* 1.1V */
	.buck1_voltage[4] = 1050000, /* 1.1V */
	.buck1_voltage[5] = 1000000, /* 1.1V */
	.buck1_voltage[6] = 950000, /* 1.0V */
	.buck1_voltage[7] = 950000, /* 0.95V */

	.buck2_voltage[0] = 1100000, /* 1.1V */
	.buck2_voltage[1] = 1100000, /* 1.0V */
	.buck2_voltage[2] = 1100000, /* 0.95V */
	.buck2_voltage[3] = 1100000, /* 0.9V */
	.buck2_voltage[4] = 1000000, /* 1.1V */
	.buck2_voltage[5] = 1000000, /* 1.0V */
	.buck2_voltage[6] = 1000000, /* 0.95V */
	.buck2_voltage[7] = 1000000, /* 0.9V */

	.buck5_voltage[0] = 1200000, /* 1.1V */
	.buck5_voltage[1] = 1200000, /* 1.1V */
	.buck5_voltage[2] = 1200000, /* 1.1V */
	.buck5_voltage[3] = 1200000, /* 1.1V */
	.buck5_voltage[4] = 1100000, /* 1.1V */
	.buck5_voltage[5] = 1100000, /* 1.1V */
	.buck5_voltage[6] = 1100000, /* 1.1V */
	.buck5_voltage[7] = 1100000, /* 1.1V */
	.buck1_gpiodvs = false,
	.buck2_gpiodvs = false,
	.buck5_gpiodvs = false,
	.buck125_gpios[0] = EXYNOS4_GPB(5),
	.buck125_gpios[1] = EXYNOS4_GPB(6),
	.buck125_gpios[2] = EXYNOS4_GPB(7),
#endif

};
static struct platform_device tc4_regulator_consumer = 
	{	.name = "tc4-regulator-consumer",	
		.id = -1,
	};

#ifdef CONFIG_REGULATOR_S5M8767
/* S5M8767 Regulator */
static int s5m_cfg_irq(void)
{
	/* AP_PMIC_IRQ: EINT15 */
	s3c_gpio_cfgpin(EXYNOS4_GPX1(7), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS4_GPX1(7), S3C_GPIO_PULL_UP);
	return 0;
}
static struct regulator_consumer_supply s5m8767_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_alive", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo2_supply[] = {
	REGULATOR_SUPPLY("vddq_m12", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo3_supply[] = {
	REGULATOR_SUPPLY("vddioap_18", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo4_supply[] = {
	REGULATOR_SUPPLY("vddq_pre", NULL),
};
//Cellon delete start, Jacob, 2012/08/04, for S5M8767 issue

static struct regulator_consumer_supply s5m8767_ldo5_supply[] = {
	REGULATOR_SUPPLY("vdd18_2m", NULL),
};

//Cellon delete end, Jacob, 2012/08/04
static struct regulator_consumer_supply s5m8767_ldo6_supply[] = {
	REGULATOR_SUPPLY("vdd10_mpll", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo7_supply[] = {
	REGULATOR_SUPPLY("vdd10_xpll", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo8_supply[] = {
	REGULATOR_SUPPLY("vdd10_mipi", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo9_supply[] = {
	REGULATOR_SUPPLY("vdd33_lcd", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo10_supply[] = {
	REGULATOR_SUPPLY("vdd18_mipi", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo11_supply[] = {
	REGULATOR_SUPPLY("vdd18_abb1", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo12_supply[] = {
	REGULATOR_SUPPLY("vdd33_uotg", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo13_supply[] = {
	REGULATOR_SUPPLY("vddioperi_18", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo14_supply[] = {
	REGULATOR_SUPPLY("vdd18_abb02", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo15_supply[] = {
	REGULATOR_SUPPLY("vdd10_ush", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo16_supply[] = {
	REGULATOR_SUPPLY("vdd18_hsic", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo17_supply[] = {
	REGULATOR_SUPPLY("vddioap_mmc012_28", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo18_supply[] = {
	REGULATOR_SUPPLY("vddioperi_28", NULL),
};
//Cellon delete start, Jacob, 2012/08/04, for S5M8767 issue

static struct regulator_consumer_supply s5m8767_ldo19_supply[] = {
	REGULATOR_SUPPLY("dvdd25", NULL),
};

//Cellon delete end, Jacob, 2012/08/04
static struct regulator_consumer_supply s5m8767_ldo20_supply[] = {
	REGULATOR_SUPPLY("vdd28_cam", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo21_supply[] = {
	REGULATOR_SUPPLY("vdd28_af", NULL),
};
//Cellon delete start, Jacob, 2012/08/04, for S5M8767 issue

static struct regulator_consumer_supply s5m8767_ldo22_supply[] = {
	REGULATOR_SUPPLY("vdda28_2m", NULL),
};

//Cellon delete end, Jacob, 2012/08/04
static struct regulator_consumer_supply s5m8767_ldo23_supply[] = {
	REGULATOR_SUPPLY("vdd_tf", NULL),
};


static struct regulator_consumer_supply s5m8767_ldo24_supply[] = {
	REGULATOR_SUPPLY("vdd33_a31", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo25_supply[] = {
	REGULATOR_SUPPLY("vdd18_cam", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo26_supply[] = {
	REGULATOR_SUPPLY("vdd18_a31", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo27_supply[] = {
	REGULATOR_SUPPLY("gps_1v8", NULL),
};
//Cellon delete start, Jacob, 2012/08/04, for S5M8767 issue

static struct regulator_consumer_supply s5m8767_ldo28_supply[] = {
	REGULATOR_SUPPLY("dvdd12", NULL),
};

//Cellon delete end, Jacob, 2012/08/04
static struct regulator_consumer_supply s5m8767_buck1_consumer =
	REGULATOR_SUPPLY("vdd_mif", NULL);

static struct regulator_consumer_supply s5m8767_buck2_consumer =
	REGULATOR_SUPPLY("vdd_arm", NULL);

static struct regulator_consumer_supply s5m8767_buck3_consumer =
	REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply s5m8767_buck4_consumer =
	REGULATOR_SUPPLY("vdd_g3d", NULL);

static struct regulator_consumer_supply s5m8767_buck5_consumer =
	REGULATOR_SUPPLY("vdd_m12", NULL);
	
static struct regulator_consumer_supply s5m8767_buck6_consumer =
	REGULATOR_SUPPLY("vdd12_5m", NULL);
	
static struct regulator_consumer_supply s5m8767_buck7_consumer =
	REGULATOR_SUPPLY("vdd12_in123", NULL);
	
static struct regulator_consumer_supply s5m8767_buck8_consumer =
	REGULATOR_SUPPLY("vdd12_in89", NULL);

static struct regulator_consumer_supply s5m8767_buck9_consumer =
	REGULATOR_SUPPLY("vddf28_emmc", NULL);



#define REGULATOR_INIT(_ldo, _name, _min_uV, _max_uV, _always_on, _ops_mask,\
		_disabled) \
	static struct regulator_init_data s5m8767_##_ldo##_init_data = {		\
		.constraints = {					\
			.name	= _name,				\
			.min_uV = _min_uV,				\
			.max_uV = _max_uV,				\
			.always_on	= _always_on,			\
			.boot_on	= _always_on,			\
			.apply_uV	= 1,				\
			.valid_ops_mask = _ops_mask,			\
			.state_mem	= {				\
				.disabled	= _disabled,		\
				.enabled	= !(_disabled),		\
			}						\
		},							\
		.num_consumer_supplies = ARRAY_SIZE(s5m8767_##_ldo##_supply),	\
		.consumer_supplies = &s5m8767_##_ldo##_supply[0],			\
	};
//Cellon modify start, Jacob, 2012/08/04, for S5M8767 issue

/*
REGULATOR_INIT(ldo1, "VDD_ALIVE", 1100000, 1100000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo2, "VDDQ_M12", 1200000, 1200000, 1,
		REGULATOR_CHANGE_STATUS, 1);//sleep controlled by pwren
REGULATOR_INIT(ldo3, "VDDIOAP_18", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo4, "VDDQ_PRE", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 1); //sleep controlled by pwren

REGULATOR_INIT(ldo5, "VDD18_2M", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo6, "VDD10_MPLL", 1000000, 1000000, 1,
		REGULATOR_CHANGE_STATUS, 1);//sleep controlled by pwren
REGULATOR_INIT(ldo7, "VDD10_XPLL", 1000000, 1000000, 1,
		REGULATOR_CHANGE_STATUS, 1);//sleep controlled by pwren
REGULATOR_INIT(ldo8, "VDD10_MIPI", 1000000, 1000000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo9, "VDD33_LCD", 3300000, 3300000, 1,
		REGULATOR_CHANGE_STATUS, 1);


REGULATOR_INIT(ldo10, "VDD18_MIPI", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo11, "VDD18_ABB1", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 0); //???
REGULATOR_INIT(ldo12, "VDD33_UOTG", 3300000, 3300000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo13, "VDDIOPERI_18", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 0);//???
REGULATOR_INIT(ldo14, "VDD18_ABB02", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 0); //???
REGULATOR_INIT(ldo15, "VDD10_USH", 1000000, 1000000, 1,
		REGULATOR_CHANGE_STATUS, 1);

//liang, VDD18_HSIC must be 1.8V, otherwise USB HUB 3503A can't be recognized
REGULATOR_INIT(ldo16, "VDD18_HSIC", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo17, "VDDIOAP_MMC012_28", 2800000, 2800000, 1,
		REGULATOR_CHANGE_STATUS, 0); //???
REGULATOR_INIT(ldo18, "VDDIOPERI_28", 2800000, 2800000, 1,
		REGULATOR_CHANGE_STATUS, 0);//???
REGULATOR_INIT(ldo19, "DVDD25", 2500000, 2500000, 0,
		REGULATOR_CHANGE_STATUS, 1); //??
REGULATOR_INIT(ldo20, "VDD28_CAM", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS, 1);

REGULATOR_INIT(ldo21, "VDD28_AF", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo22, "VDDA28_2M", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo23, "VDD28_TF", 2800000, 2800000, 1,
		REGULATOR_CHANGE_STATUS, 1);//sleep controlled by pwren
REGULATOR_INIT(ldo24, "VDD33_A31", 3300000, 3300000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo25, "VDD18_CAM", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo26, "VDD18_A31", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo27, "GPS_1V8", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo28, "DVDD12", 1200000, 1200000, 0,
		REGULATOR_CHANGE_STATUS, 1);
*/

REGULATOR_INIT(ldo1, "VDD_ALIVE", 1000000, 1000000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo2, "VDDQ_M12", 1200000, 1200000, 1,
		REGULATOR_CHANGE_STATUS, 1);//sleep controlled by pwren
REGULATOR_INIT(ldo3, "VDDIOAP_18", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo4, "VDDQ_PRE", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 1); //sleep controlled by pwren
REGULATOR_INIT(ldo5, "VDD18_2M", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo6, "VDD10_MPLL", 1000000, 1000000, 1,
		REGULATOR_CHANGE_STATUS, 1);//sleep controlled by pwren
REGULATOR_INIT(ldo7, "VDD10_XPLL", 1000000, 1000000, 1,
		REGULATOR_CHANGE_STATUS, 1);//sleep controlled by pwren
REGULATOR_INIT(ldo8, "VDD10_MIPI", 1000000, 1000000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo9, "VDD33_LCD", 3000000, 3000000, 1,
		REGULATOR_CHANGE_STATUS, 1);


REGULATOR_INIT(ldo10, "VDD18_MIPI", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo11, "VDD18_ABB1", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 0); //???
REGULATOR_INIT(ldo12, "VDD33_UOTG", 3000000, 3000000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo13, "VDDIOPERI_18", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 0);//???
REGULATOR_INIT(ldo14, "VDD18_ABB02", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 0); //???
REGULATOR_INIT(ldo15, "VDD10_USH", 1000000, 1000000, 1,
		REGULATOR_CHANGE_STATUS, 1);

//liang, VDD18_HSIC must be 1.8V, otherwise USB HUB 3503A can't be recognized
REGULATOR_INIT(ldo16, "VDD18_HSIC", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo17, "VDDIOAP_MMC012_28", 2800000, 2800000, 1,
		REGULATOR_CHANGE_STATUS, 0); //???
REGULATOR_INIT(ldo18, "VDDIOPERI_28", 3000000, 3000000, 1,
		REGULATOR_CHANGE_STATUS, 0);//???
REGULATOR_INIT(ldo19, "DVDD25", 2500000, 2500000, 0,
		REGULATOR_CHANGE_STATUS, 1); //??
REGULATOR_INIT(ldo20, "VDD28_CAM", 3000000, 3000000, 1,
		REGULATOR_CHANGE_STATUS, 0);

REGULATOR_INIT(ldo21, "VDD28_AF", 3000000, 3000000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo22, "VDDA28_2M", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo23, "VDD28_TF", 2800000, 2800000, 1,
		REGULATOR_CHANGE_STATUS, 0);//sleep controlled by pwren
REGULATOR_INIT(ldo24, "VDD33_A31", 2800000, 2800000, 0, //crys
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo25, "VDD18_CAM", 1200000, 1200000, 0,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo26, "VDD18_A31", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo27, "GPS_1V8", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 0); // cellon qiu.li change 1 to 0 20120821
REGULATOR_INIT(ldo28, "DVDD12", 1200000, 1200000, 0,
		REGULATOR_CHANGE_STATUS, 1);

//Cellon modify end, Jacob, 2012/08/04

//Cellon modify start, Jacob, 2012/08/04, for S5M8767 issue
/*
static struct regulator_init_data s5m8767_buck1_data = {
	.constraints	= {
		.name		= "vdd_mif range",
		.min_uV		= 900000,
		.max_uV		= 1100000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck1_consumer,
};

static struct regulator_init_data s5m8767_buck2_data = {
	.constraints	= {
		.name		= "vdd_arm range",
		.min_uV		=  850000,
		.max_uV		= 1450000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck2_consumer,
};

static struct regulator_init_data s5m8767_buck3_data = {
	.constraints	= {
		.name		= "vdd_int range",
		.min_uV		=  875000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			//.uV		= 1100000,
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck3_consumer,
};

static struct regulator_init_data s5m8767_buck4_data = {
	.constraints	= {
		.name		= "vdd_g3d range",
		.min_uV		= 750000,
		.max_uV		= 1500000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck4_consumer,
};

static struct regulator_init_data s5m8767_buck5_data = {
	.constraints	= {
		.name		= "vdd_m12 range",
		.min_uV		= 750000,
		.max_uV		= 1500000,
		.apply_uV	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck5_consumer,
};
static struct regulator_init_data s5m8767_buck6_data = {
	.constraints	= {
		.name		= "vdd12_5m range",
		.min_uV		= 750000,
		.max_uV		= 1500000,
		.boot_on	= 0,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck6_consumer,
};
static struct regulator_init_data s5m8767_buck9_data = {
	.constraints	= {
		.name		= "vddf28_emmc range",
		.min_uV		= 750000,
		.max_uV		= 3000000,
		.boot_on	= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck9_consumer,
};
*/

static struct regulator_init_data s5m8767_buck1_data = {
	.constraints	= {
		.name		= "vdd_mif range",
		.min_uV		= 650000,
		.max_uV		= 2225000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck1_consumer,
};

static struct regulator_init_data s5m8767_buck2_data = {
	.constraints	= {
		.name		= "vdd_arm range",
		.min_uV		= 600000,
		.max_uV		= 1600000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck2_consumer,
};

static struct regulator_init_data s5m8767_buck3_data = {
	.constraints	= {
		.name		= "vdd_int range",
		.min_uV		= 600000,
		.max_uV		= 1600000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			//.uV		= 1100000,
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck3_consumer,
};

static struct regulator_init_data s5m8767_buck4_data = {
	.constraints	= {
		.name		= "vdd_g3d range",
		.min_uV		= 600000,
		.max_uV		= 1600000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck4_consumer,
};

static struct regulator_init_data s5m8767_buck5_data = {
	.constraints	= {
		.name		= "vdd_m12 range",
		.min_uV		= 650000,
		.max_uV		= 2225000,
		.apply_uV	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck5_consumer,
};
static struct regulator_init_data s5m8767_buck6_data = {
	.constraints	= {
		.name		= "vdd12_5m range",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		//Cellon modify begin , 2012/10/30 charles.hu
		.boot_on	= 0,	
		//Cellon modify end , 2012/10/30 charles.hu
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
//Cellon modify begin , charles.hu 2012/10/30			
			.disabled = 1,
//Cellon modify end   , charles.hu 2012/10/30			
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck6_consumer,
};

static struct regulator_init_data s5m8767_buck7_data = {
	.constraints	= {
		.name		= "vdd12_in123 range",
		.min_uV		= 750000,
		.max_uV		= 3000000,
		.boot_on	= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck7_consumer,
};

static struct regulator_init_data s5m8767_buck8_data = {
	.constraints	= {
		.name		= "vdd12_in89 range",
		.min_uV		= 750000,
		.max_uV		= 3000000,
		.boot_on	= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck8_consumer,
};

static struct regulator_init_data s5m8767_buck9_data = {
	.constraints	= {
		.name		= "vddf28_emmc range",
		.min_uV		= 750000,
		.max_uV		= 3300000,
//		.boot_on	= 1,
		.boot_on	= 0,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck9_consumer,
};
//Cellon modify end, Jacob, 2012/08/04

static struct s5m_regulator_data pegasus_regulators[] = {
	{ S5M8767_BUCK1, &s5m8767_buck1_data },
	{ S5M8767_BUCK2, &s5m8767_buck2_data },
	{ S5M8767_BUCK3, &s5m8767_buck3_data },
	{ S5M8767_BUCK4, &s5m8767_buck4_data },
	{ S5M8767_BUCK5, &s5m8767_buck5_data },
	{ S5M8767_BUCK6, &s5m8767_buck6_data },
	{ S5M8767_BUCK7, &s5m8767_buck7_data },
	{ S5M8767_BUCK8, &s5m8767_buck8_data },
	{ S5M8767_BUCK9, &s5m8767_buck9_data },

	{ S5M8767_LDO1, &s5m8767_ldo1_init_data },
	{ S5M8767_LDO2, &s5m8767_ldo2_init_data },
	{ S5M8767_LDO3, &s5m8767_ldo3_init_data },
	{ S5M8767_LDO4, &s5m8767_ldo4_init_data },
//Cellon delete start, Jacob, 2012/08/04, for S5M8767 issue
	{ S5M8767_LDO5, &s5m8767_ldo5_init_data },
//Cellon delete end, Jacob, 2012/08/04	
	{ S5M8767_LDO6, &s5m8767_ldo6_init_data },
	{ S5M8767_LDO7, &s5m8767_ldo7_init_data },
	{ S5M8767_LDO8, &s5m8767_ldo8_init_data },
	{ S5M8767_LDO9, &s5m8767_ldo9_init_data },
	{ S5M8767_LDO10, &s5m8767_ldo10_init_data },

	{ S5M8767_LDO11, &s5m8767_ldo11_init_data },
	{ S5M8767_LDO12, &s5m8767_ldo12_init_data },
	{ S5M8767_LDO13, &s5m8767_ldo13_init_data },
	{ S5M8767_LDO14, &s5m8767_ldo14_init_data },
	{ S5M8767_LDO15, &s5m8767_ldo15_init_data },
	{ S5M8767_LDO16, &s5m8767_ldo16_init_data },
	{ S5M8767_LDO17, &s5m8767_ldo17_init_data },
	{ S5M8767_LDO18, &s5m8767_ldo18_init_data },
//Cellon delete start, Jacob, 2012/08/04, for S5M8767 issue
	{ S5M8767_LDO19, &s5m8767_ldo19_init_data },
//Cellon delete end, Jacob, 2012/08/04	
	{ S5M8767_LDO20, &s5m8767_ldo20_init_data },

	{ S5M8767_LDO21, &s5m8767_ldo21_init_data },
//Cellon delete start, Jacob, 2012/08/04, for S5M8767 issue
	{ S5M8767_LDO22, &s5m8767_ldo22_init_data },
//Cellon delete end, Jacob, 2012/08/04	
	{ S5M8767_LDO23, &s5m8767_ldo23_init_data },
	{ S5M8767_LDO24, &s5m8767_ldo24_init_data },
	{ S5M8767_LDO25, &s5m8767_ldo25_init_data },
	{ S5M8767_LDO26, &s5m8767_ldo26_init_data },
	{ S5M8767_LDO27, &s5m8767_ldo27_init_data },
//Cellon delete start, Jacob, 2012/08/04, for S5M8767 issue
	{ S5M8767_LDO28, &s5m8767_ldo28_init_data },
//Cellon delete end, Jacob, 2012/08/04	
	
};

static struct s5m_platform_data exynos4_s5m8767_pdata = {
	.device_type		= S5M8767X,
	.irq_base		= IRQ_BOARD_START,
	.num_regulators		= ARRAY_SIZE(pegasus_regulators),
	.regulators		= pegasus_regulators,
	.cfg_pmic_irq		= s5m_cfg_irq,
	.ono				= EXYNOS4_GPX0(2),

	.buck2_voltage[0]	= 1250000,
	.buck2_voltage[1]	= 1200000,
	.buck2_voltage[2]	= 1200000,
	.buck2_voltage[3]	= 1200000,
	.buck2_voltage[4]	= 1200000,
	.buck2_voltage[5]	= 1200000,
	.buck2_voltage[6]	=  1200000,
	.buck2_voltage[7]	=  1200000,

	.buck3_voltage[0]	= 1100000,
	.buck3_voltage[1]	= 1100000,
	.buck3_voltage[2]	= 1100000,
	.buck3_voltage[3]	= 1100000,
	.buck3_voltage[4]	= 1100000,
	.buck3_voltage[5]	= 1100000,
	.buck3_voltage[6]	= 1100000,
	.buck3_voltage[7]	= 1100000,

	.buck4_voltage[0]	= 1200000,
	.buck4_voltage[1]	= 1200000,
	.buck4_voltage[2]	= 1200000,
	.buck4_voltage[3]	= 1200000,
	.buck4_voltage[4]	= 1200000,
	.buck4_voltage[5]	= 1200000,
	.buck4_voltage[6]	= 1200000,
	.buck4_voltage[7]	= 1200000,

	.buck_default_idx	= 3,
	.buck_gpios[0]		= EXYNOS4_GPB(5),
	.buck_gpios[1]		= EXYNOS4_GPB(6),
	.buck_gpios[2]		= EXYNOS4_GPB(7),

	.buck_ramp_delay        = 10,
	.buck2_ramp_enable      = true,
	.buck3_ramp_enable      = true,
	.buck4_ramp_enable      = true,
};
/* End of S5M8767 */
#endif


#ifdef CONFIG_VIDEO_S5P_MIPI_CSIS
static struct regulator_consumer_supply mipi_csi_fixed_voltage_supplies[] = {
	REGULATOR_SUPPLY("mipi_csi", "s5p-mipi-csis.0"),
	REGULATOR_SUPPLY("mipi_csi", "s5p-mipi-csis.1"),
};

static struct regulator_init_data mipi_csi_fixed_voltage_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(mipi_csi_fixed_voltage_supplies),
	.consumer_supplies	= mipi_csi_fixed_voltage_supplies,
};

static struct fixed_voltage_config mipi_csi_fixed_voltage_config = {
	.supply_name	= "DC_5V",
	.microvolts	= 5000000,
	.gpio		= -EINVAL,
	.init_data	= &mipi_csi_fixed_voltage_init_data,
};

static struct platform_device mipi_csi_fixed_voltage = {
	.name		= "reg-fixed-voltage",
	.id		= 3,
	.dev		= {
		.platform_data	= &mipi_csi_fixed_voltage_config,
	},
};
#endif

#ifdef CONFIG_VIDEO_M5MOLS
static struct regulator_consumer_supply m5mols_fixed_voltage_supplies[] = {
	REGULATOR_SUPPLY("core", NULL),
	REGULATOR_SUPPLY("dig_18", NULL),
	REGULATOR_SUPPLY("d_sensor", NULL),
	REGULATOR_SUPPLY("dig_28", NULL),
	REGULATOR_SUPPLY("a_sensor", NULL),
	REGULATOR_SUPPLY("dig_12", NULL),
};

static struct regulator_init_data m5mols_fixed_voltage_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(m5mols_fixed_voltage_supplies),
	.consumer_supplies	= m5mols_fixed_voltage_supplies,
};

static struct fixed_voltage_config m5mols_fixed_voltage_config = {
	.supply_name	= "CAM_SENSOR",
	.microvolts	= 1800000,
	.gpio		= -EINVAL,
	.init_data	= &m5mols_fixed_voltage_init_data,
};

static struct platform_device m5mols_fixed_voltage = {
	.name		= "reg-fixed-voltage",
	.id		= 4,
	.dev		= {
		.platform_data	= &m5mols_fixed_voltage_config,
	},
};
#endif

static struct regulator_consumer_supply wm8994_fixed_voltage0_supplies[] = {
	REGULATOR_SUPPLY("AVDD2", "4-001a"),
	REGULATOR_SUPPLY("CPVDD", "4-001a"),
};

static struct regulator_consumer_supply wm8994_fixed_voltage1_supplies[] = {
	REGULATOR_SUPPLY("SPKVDD1", "4-001a"),
	REGULATOR_SUPPLY("SPKVDD2", "4-001a"),
};

static struct regulator_consumer_supply wm8994_fixed_voltage2_supplies =
	REGULATOR_SUPPLY("DBVDD", "4-001a");

static struct regulator_init_data wm8994_fixed_voltage0_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(wm8994_fixed_voltage0_supplies),
	.consumer_supplies	= wm8994_fixed_voltage0_supplies,
};

static struct regulator_init_data wm8994_fixed_voltage1_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(wm8994_fixed_voltage1_supplies),
	.consumer_supplies	= wm8994_fixed_voltage1_supplies,
};

static struct regulator_init_data wm8994_fixed_voltage2_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &wm8994_fixed_voltage2_supplies,
};

static struct fixed_voltage_config wm8994_fixed_voltage0_config = {
	.supply_name	= "VDD_1.8V",
	.microvolts	= 1800000,
	.gpio		= -EINVAL,
	.init_data	= &wm8994_fixed_voltage0_init_data,
};

static struct fixed_voltage_config wm8994_fixed_voltage1_config = {
	.supply_name	= "DC_5V",
	.microvolts	= 5000000,
	.gpio		= -EINVAL,
	.init_data	= &wm8994_fixed_voltage1_init_data,
};

static struct fixed_voltage_config wm8994_fixed_voltage2_config = {
	.supply_name	= "VDD_3.3V",
	.microvolts	= 3300000,
	.gpio		= -EINVAL,
	.init_data	= &wm8994_fixed_voltage2_init_data,
};

static struct platform_device wm8994_fixed_voltage0 = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev		= {
		.platform_data	= &wm8994_fixed_voltage0_config,
	},
};

static struct platform_device wm8994_fixed_voltage1 = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev		= {
		.platform_data	= &wm8994_fixed_voltage1_config,
	},
};

static struct platform_device wm8994_fixed_voltage2 = {
	.name		= "reg-fixed-voltage",
	.id		= 2,
	.dev		= {
		.platform_data	= &wm8994_fixed_voltage2_config,
	},
};

static struct regulator_consumer_supply wm8994_avdd1_supply =
	REGULATOR_SUPPLY("AVDD1", "4-001a");

static struct regulator_consumer_supply wm8994_dcvdd_supply =
	REGULATOR_SUPPLY("DCVDD", "4-001a");

static struct regulator_init_data wm8994_ldo1_data = {
	.constraints	= {
		.name		= "AVDD1",
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &wm8994_avdd1_supply,
};

static struct regulator_init_data wm8994_ldo2_data = {
	.constraints	= {
		.name		= "DCVDD",
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &wm8994_dcvdd_supply,
};

struct wm8994_drc_cfg wm8994_drc_cfg[]={
	{
        .name = "aif1_drc1_media",
        .regs={0xb0d4,0x0204,0x4499,0x02ac,0x00cc},
	},
	{
        .name = "aif1_drc2_input",
        .regs={0x0098,0x0000,0x0000,0x0000,0x0000},
	},
	{
        .name = "aif2_drc_output",
        .regs={0x0098,0x0000,0x0000,0x0000,0x0000},
	},
};


static struct wm8994_pdata wm8994_platform_data = {
	/* configure gpio1 function: 0x0001(Logic level input/output) */
	.gpio_defaults[0] = 0x0001,
	/* configure gpio3/4/5/7 function for AIF2 voice */
	.gpio_defaults[2] = 0x8100,/* BCLK2 in */
	.gpio_defaults[3] = 0x8100,/* LRCLK2 in */
	.gpio_defaults[4] = 0x8100,/* DACDAT2 in */
	/* configure gpio6 function: 0x0001(Logic level input/output) */
	.gpio_defaults[5] = 0x0001,
	.gpio_defaults[6] = 0x0100,/* ADCDAT2 out */
	.ldo[0] = { 0, NULL, &wm8994_ldo1_data },
	.ldo[1] = { 0, NULL, &wm8994_ldo2_data },

       .num_drc_cfgs = 3,
       .drc_cfgs = &wm8994_drc_cfg,

	
};

/* ly 20111102 : i2c devs mapping: 
  * i2c0 : HDMI
  * i2c1 : max8997: PMIC & RTC & motor
  * i2c2 : not used
  * i2c3 : touch
  * i2c4 : max8997 fuel gauge & wm8994
  * i2c5 : sensor: MPU3050
  * i2c6 : camera & HSIC
  * i2c7 : light sensor
  */
static struct i2c_board_info i2c_devs0[] __initdata = {
#ifdef CONFIG_VIDEO_TVOUT
	{
	//cellon qiu.li modify start 20120821
	#ifdef CONFIG_MHL_Sii8334
		I2C_BOARD_INFO("s5p_ddc", (0xA0 >> 1)),    
	#else
		I2C_BOARD_INFO("s5p_ddc", (0x74 >> 1)),    
	#endif
	//cellon qiu.li modify end 20120821
		//.platform_data=&hdmi_i2c_en_pdata,            //yqf 20110608, move to s5p_hdmi_ctrl_init_private()
	},
#endif
};

static struct i2c_board_info i2c_devs1[] __initdata = {
	{
#ifdef CONFIG_REGULATOR_S5M8767

		I2C_BOARD_INFO("s5m87xx", 0xCC >> 1),
		.platform_data = &exynos4_s5m8767_pdata,
		.irq		= IRQ_EINT(15),

#else
		I2C_BOARD_INFO(MAX8997_I2C_NAME, MAX8997_ADDR),
#ifdef CONFIG_REGULATOR_MAX8997
		.platform_data = &exynos4_max8997_info,
#endif
#endif
	},
#ifdef CONFIG_VIBRATOR

#ifndef CONFIG_REGULATOR_S5M8767

	{
		I2C_BOARD_INFO(MOTOR8997_I2C_NAME, MOTOR8997_ADDR),
	},
#endif

#endif

#ifdef CONFIG_RTC_MAX8997
	{
		I2C_BOARD_INFO(RTC8997_I2C_NAME, RTC8997_I2C_ADDR),
	},
#endif

#ifdef CONFIG_BATTERY_MAX17040
	{
		I2C_BOARD_INFO(MAX17040_I2C_NAME, MAX17040_I2C_ADDR),
	},
#endif

};
#if 1 //iic 2 : nfc
static struct i2c_board_info i2c_devs2[] __initdata = {

#ifdef CONFIG_S3FHRN2_I2C
	{
		I2C_BOARD_INFO(S3FHRN2_DRIVER_NAME, SNFC_I2C_ADDR),
		.platform_data  = &exynos4_nfc_info,
		.irq		= IRQ_EINT(SNFC_EINT_NUM),
	},
#endif
};
#endif
//kaixian@cellon

//cellon qiu.li add start 20120821
#ifdef CONFIG_MHL_Sii8334
//#define CI2CA  true  // CI2CA depend on the CI2CA pin's level
#ifdef CI2CA 
#define SII8334_plus 0x02  //Define sii8334's I2c Address of all pages by the status of CI2CA.
#else
#define SII8334_plus 0x00  //Define sii8334's I2c Address of all pages by the status of CI2CA.
#endif
#define SII8334_INT_PIN  EXYNOS4_GPX3(6)
#define SII8334_RESET_PIN  EXYNOS4_GPX3(1)

struct sii_mhl_platform_data {
	void (*reset) (void);
};
static void Sii8334_reset(void)
{	
	int err = 0;
	err=gpio_request(SII8334_INT_PIN, "mhl INT");
	if (err) {
		printk(KERN_ERR "failed to request sii8334 mhl INT\n");
		return err;
	}
	gpio_direction_input(SII8334_INT_PIN);
	s3c_gpio_cfgpin(SII8334_INT_PIN, S3C_GPIO_SFN(0x3));//
	s3c_gpio_setpull(SII8334_INT_PIN, S3C_GPIO_PULL_NONE);
	
	gpio_request(SII8334_RESET_PIN, "mhl reset");
	if (err) {
		printk(KERN_ERR "failed to request sii8334 mhl reset\n");
		return err;
	}
	gpio_direction_output(SII8334_RESET_PIN, 1);
	
	gpio_set_value(SII8334_RESET_PIN, 0);
	msleep(5);
	gpio_set_value(SII8334_RESET_PIN, 1);

}

static struct sii_mhl_platform_data Sii8334_data = {
	.reset = Sii8334_reset,
};
#endif
//cellon qiu.li add end 20120821

#ifdef CONFIG_ANC_ES305
struct es305_platform_data {
	uint32_t gpio_es305_reset;
	void (* es305_mclk_init )(void);
	void (* es305_mclk_enable )(bool enable);
	uint32_t gpio_es305_wakeup;
};

static void smdk4x12_xclkout_init(void)
{
	unsigned int tmp;

	printk("%s\n",__func__);
	tmp = __raw_readl(EXYNOS4_CLKOUT_CMU_CPU);
	tmp &= ~0xffff;
	tmp |= 0x1904;
	__raw_writel(tmp, EXYNOS4_CLKOUT_CMU_CPU);

	tmp = __raw_readl(S5P_PMU_DEBUG);
	tmp &= ~0xf00;
	tmp |= 0x900;  

	tmp |= 0x0001; //disable clock out

	__raw_writel(tmp, S5P_PMU_DEBUG);
}

static void smdk4x12_xclkout_config(bool enable)
{
	unsigned int tmp;

	printk("%s\n",__func__);
	tmp = __raw_readl(S5P_PMU_DEBUG);

	if(enable)
		tmp &= ~0x0001;	
	else
		tmp |= 0x0001;  
	
	__raw_writel(tmp, S5P_PMU_DEBUG);
}

static struct es305_platform_data es305_platform_data = {
	.gpio_es305_reset = EXYNOS4212_GPM0(5),//EXYNOS4_GPX1(6),
	.es305_mclk_init = smdk4x12_xclkout_init,
	.es305_mclk_enable = smdk4x12_xclkout_config,
	.gpio_es305_wakeup = EXYNOS4_GPA1(5),
};
#endif /* CONFIG_ANC_ES305 */


// For S5K4EC(using i2c6)
static struct i2c_board_info i2c_devs6[] __initdata = {
#ifdef CONFIG_AAT3635	
	{
		I2C_BOARD_INFO(AAT3635_I2C_NAME, AAT3635_I2C_ADDR),
		 //.irq = IRQ_EINT(23),	
		//.platform_data = &aat3635_priv,			
	},
#endif
#ifdef CONFIG_AAT3635_P0
	{
		I2C_BOARD_INFO("aat3635_p0", 0x6a),
			
	},
#endif
#ifdef CONFIG_BATTERY_BQ27425
{
	I2C_BOARD_INFO("bq27425", 0x55),
},
#endif

//cellon qiu.li add start 20120821
#ifdef CONFIG_MHL_Sii8334
	{
		I2C_BOARD_INFO("sii8334_PAGE_TPI",   (0x39 + SII8334_plus)),
		.irq = IRQ_EINT(30),  
		.platform_data = &Sii8334_data,
	},
	{
		I2C_BOARD_INFO("sii8334_PAGE_TX_L1", (0x3d + SII8334_plus)), 
	},
	{
		I2C_BOARD_INFO("sii8334_PAGE_TX_2", (0x49 + SII8334_plus)), 
	},
	{
		I2C_BOARD_INFO( "sii8334_PAGE_TX_3", (0x4d + SII8334_plus)), 
	},
	{
		I2C_BOARD_INFO("sii8334_PAGE_CBUS", (0x64 + SII8334_plus)), 
	}
#endif
//cellon qiu.li add end 20120821
};

#if defined(CONFIG_TOUCHSCREEN_EGALAX)	// Egalax I2C TS
struct egalax_i2c_platform_data egalax_eeti_pdata = {
	.gpio_int = EXYNOS4_GPX0(4),
	.gpio_en = EXYNOS4_GPL0(2),
	//.gpio_rst = S5PV310_GPA1(4),
};
#endif

static struct i2c_board_info i2c_devs3[] __initdata = {
#if defined(CONFIG_TOUCHSCREEN_EGALAX)	// Egalax I2C TS
	{
		I2C_BOARD_INFO(EGALAX_I2C_NAME, EGALAX_I2C_ADDR),
		.irq = EGALAX_IRQ,
		//.irq = gpio_to_irq(egalax_eeti_pdata.gpio),
		.platform_data = &egalax_eeti_pdata,
	},
#endif
};

/* I2C4 */
static struct i2c_board_info i2c_devs4[] __initdata = {
/*Fri Aug 26 18:26:55 CST 2011 add by cwp:move codec i2c from controller 6 to 4*/
#if defined(CONFIG_SND_SOC_WM8994) || defined(CONFIG_SND_SOC_WM8994_MODULE)
	{
		I2C_BOARD_INFO("wm8994", 0x34>>1),
		.platform_data	= &wm8994_platform_data,
	},
#endif
//kaixian@cellon
#ifdef CONFIG_ANC_ES305
	{
		I2C_BOARD_INFO("audience_es305", 0x3e),//0x3e
		.platform_data = &es305_platform_data,
	},
#endif /*CONFIG_ANC_ES305*/

#ifdef CONFIG_BATTERY_MAX8997//ly
	{
		I2C_BOARD_INFO(FG8997_I2C_NAME, FG8997_I2C_ADDR),
	},
#endif

};

#ifdef CONFIG_S3FHRN2_I2C
static struct s3c2410_platform_i2c i2c_data4 __initdata = {
	.flags		= 0,
	.slave_addr	= SNFC_I2C_ADDR,
	.frequency	= 200*1000,
	.sda_delay	= 50,
	.bus_num 	= 4,
	.cfg_gpio	= exynos4_init_nfc_gpio,
};
#endif

#if defined(CONFIG_SENSOR_ST_LSM303DLHC)	// G/M sensor
struct lsm303dlhc_acc_platform_data lsm303dlhc_g_pdata = {
	.poll_interval = 10,
	.min_interval = 0,
	.g_range = LSM303DLHC_ACC_G_2G,
	.axis_map_x = GM_AXIS_MAP_X,
	.axis_map_y = GM_AXIS_MAP_Y,
	.axis_map_z = GM_AXIS_MAP_Z,
	.negate_x = GM_NEGATE_X,
	.negate_y = GM_NEGATE_Y,
	.negate_z = GM_NEGATE_Z,
	.gpio_int1 = EXYNOS4_GPX3(0),
	.gpio_int2 = EXYNOS4_GPX3(1),
};
struct lsm303dlhc_mag_platform_data lsm303dlhc_m_pdata = {
	.poll_interval = 13,
	.min_interval = 0,
	.h_range = LSM303DLHC_H_8_1G,
	.axis_map_x = GM_AXIS_MAP_X,
	.axis_map_y = GM_AXIS_MAP_Y,
	.axis_map_z = GM_AXIS_MAP_Z,
	.negate_x = GM_NEGATE_X,
	.negate_y = GM_NEGATE_Y,
	.negate_z = GM_NEGATE_Z,
};
#endif

#if defined(CONFIG_SENSOR_ST_L3G4200D)	// Gyroscope sensor
struct l3g4200d_gyr_platform_data l3g4200d_gyro_pdata = {
	.poll_interval = 10,
	.min_interval = 0,
	.fs_range = L3G4200D_FS_2000DPS,
	.axis_map_x = GYRO_AXIS_MAP_X,
	.axis_map_y = GYRO_AXIS_MAP_Y,
	.axis_map_z = GYRO_AXIS_MAP_Z,
	.negate_x = GYRO_NEGATE_X,
	.negate_y = GYRO_NEGATE_Y,
	.negate_z = GYRO_NEGATE_Z,
};
#endif
#if defined(CONFIG_SENSOR_ROHM_BH1721)	// Light sensor
struct ROHM_I2C_platform_data rohm_bh1721_pdata = {
};
#endif

 // Cellon add start, ZePeng Wu, 2012/08/04, for TP 
#if defined(CONFIG_RMI4_BUS)
struct syna_gpio_data {
	u16 gpio_number;
	char* gpio_name;
};

static int s3202_touchpad_gpio_setup(void *gpio_data, bool configure)
{
	int retval=0;
	struct syna_gpio_data *data = gpio_data;
#if(0)	
	struct regulator *vdd28_af_regulator;
	struct regulator *vddioperi_18_regulator;


	vdd28_af_regulator = regulator_get(NULL, "vdd28_af");
	if (IS_ERR(vdd28_af_regulator)) {
		pr_err("%s: failed to get %s\n", __func__, "vdd28_af");
		//ret = -ENODEV;
		goto out2;
	}

	vddioperi_18_regulator = regulator_get(NULL, "vddioperi_18");
	if (IS_ERR(vddioperi_18_regulator)) {
		pr_err("%s: failed to get %s\n", __func__, "vddioperi_18");
		//ret = -ENODEV;
		goto out1;
	}

	if (!regulator_is_enabled(vdd28_af_regulator))
			regulator_enable(vdd28_af_regulator);
	if (!regulator_is_enabled(vddioperi_18_regulator))
			regulator_enable(vddioperi_18_regulator);
	
	regulator_put(vdd28_af_regulator);
	regulator_put(vddioperi_18_regulator);
	goto out2;
	
out1:
	regulator_put(vdd28_af_regulator);
out2:
 	msleep(20);
#endif

	if (configure) {
		retval = gpio_request(EXYNOS4_GPX0(3),"rmi4_rst");
		if (retval) {
			pr_err("%s: Failed to get rst gpio %d. Code: %d.",
			       __func__, data->gpio_number, retval);
			return retval;
		}

		gpio_direction_output(EXYNOS4_GPX0(3), 0);
		s3c_gpio_cfgpin(EXYNOS4_GPX0(3), S3C_GPIO_SFN(1));
		s3c_gpio_setpull(EXYNOS4_GPX0(3), S3C_GPIO_PULL_UP);
		
		gpio_set_value(EXYNOS4_GPX0(3), 1);

		retval = gpio_request(data->gpio_number, "rmi4_attn");
		if (retval) {
			pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			       __func__, data->gpio_number, retval);
			return retval;
		}

		gpio_direction_output(data->gpio_number, 0);
		s3c_gpio_cfgpin(data->gpio_number, S3C_GPIO_SFN(0xf));
		s3c_gpio_setpull(data->gpio_number, S3C_GPIO_PULL_UP);
	
		gpio_set_value(data->gpio_number, 1);

	} else {
		pr_warn("%s: No way to deconfigure gpio %d.",
		       __func__, data->gpio_number);
	}
	msleep(20);
	return retval;
}


#if(1)

#define MAX_LEN		100
static ssize_t s3202_virtual_keys_register(struct kobject *kobj,
		     struct kobj_attribute *attr, char *buf)
{

   char *virtual_keys = __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":64:1330:120:80"  \
                     ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":251:1330:120:80" \
                     ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":516:1330:120:80" \
                     ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH)":645:1330:120:80" "\n";

	return snprintf(buf, strnlen(virtual_keys, MAX_LEN) + 1 , "%s",
			virtual_keys);

}

static struct kobj_attribute s3202_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.sensor00fn1a",
		.mode = S_IRUGO,
	},
	.show = &s3202_virtual_keys_register,
};

static struct attribute *virtual_key_properties_attrs[] = {
	&s3202_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group virtual_key_properties_attr_group = {
	.attrs = virtual_key_properties_attrs,
};

static int s3202_virtual_key_properties(void)
{
	int retval = 0;
	
	struct kobject *virtual_key_properties_kobj;
	virtual_key_properties_kobj =
		kobject_create_and_add("board_properties", NULL);
	if (virtual_key_properties_kobj)
		retval = sysfs_create_group(virtual_key_properties_kobj,
				&virtual_key_properties_attr_group);
	if (!virtual_key_properties_kobj || retval)
		pr_err("failed to create ft5202 board_properties\n");

	return retval;
}
#endif

#define S3202_ADDR	0x20
#define S3202_ATTN	EXYNOS4_GPX0(1)

static unsigned char s3202_f1a_button_codes[] = {KEY_HOME, KEY_BACK,KEY_MENU,KEY_SEARCH};

static struct rmi_f1a_button_map s3202_f1a_button_map = {
    .nbuttons = ARRAY_SIZE(s3202_f1a_button_codes),
    .map = s3202_f1a_button_codes,
};



static struct syna_gpio_data s3202_gpiodata = {
	.gpio_number = S3202_ATTN,
	.gpio_name = "s3202_irq",
};


static struct rmi_device_platform_data s3202_platformdata = {
	.sensor_name = "s3202_ts",
	.driver_name = "rmi_generic",
	.attn_gpio = S3202_ATTN,
	.level_triggered = 1,
	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
	.gpio_data = &s3202_gpiodata,
	.gpio_config = s3202_touchpad_gpio_setup,
	//.axis_align = AXIS_ALIGNMENT,
	.f1a_button_map = &s3202_f1a_button_map,
	//.virtualbutton_map = &s3202_virtualbutton_map,
};

#endif

 // Cellon add end, ZePeng Wu, 2012/08/04, for TP 
 // Cellon add start, Ted Shi, 2012/10/07, for ar1000 driver porting 
#ifdef CONFIG_RADIO_AR1000
 struct radio_info {
         const char *radio_name;
         int radio_reset;
         struct resource *resource;
         uint8_t num_resources;
 };

 static struct radio_info radio_ar1000_data = {
         .radio_name="radio-ar1000",
 };
#define AR1000_IRQ IRQ_EINT(28)
static struct resource  ar1000_resources[] = {
	{
		.name   = "ar1000_interrupt",
		.start = AR1000_IRQ,
		.end = AR1000_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};


static struct platform_device radio_ar1000 = {
	.name      = "radio-ar1000",
	.num_resources = ARRAY_SIZE(ar1000_resources),
	.resource = ar1000_resources,
       .dev      	= {
                 .platform_data = &radio_ar1000_data,
       },
};
#endif
 // Cellon add end, Ted Shi, 2012/10/07
/* I2C5 */
static struct i2c_board_info i2c_devs5[] __initdata = {
#if defined(CONFIG_SENSOR_ST_LSM303DLHC)	// G/M sensor
	{
		I2C_BOARD_INFO(LSM303DLHC_ACC_DEV_NAME, LSM303DLHC_ACC_I2C_ADDR),
		.platform_data  = &lsm303dlhc_g_pdata,
	},
	{
		I2C_BOARD_INFO(LSM303DLHC_MAG_DEV_NAME, LSM303DLHC_MAG_I2C_ADDR),
		.platform_data  = &lsm303dlhc_m_pdata,
	},
#endif
#if defined(CONFIG_LTR558)
	{
		I2C_BOARD_INFO("ltr558",0x23),
		.irq = IRQ_EINT(20),
	},
#endif
#if defined(CONFIG_SENSOR_ST_LSM330D) && defined(CONFIG_LSM330D_USE_I2C5)
	{
		I2C_BOARD_INFO(LSM330DLC_ACC_DEV_NAME, LSM330DLC_ACC_I2C_SAD_L),
	},
	{
		I2C_BOARD_INFO(LSM330DLC_GYR_DEV_NAME, LSM330DLC_GYR_I2C_SAD_H),
	},
#endif
#if defined(CONFIG_SENSORS_AK8963) && defined(CONFIG_AKM8963C_USE_I2C5)
	{
		I2C_BOARD_INFO(AKM8963_I2C_NAME, 0x0C),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &akm_platform_data_8963,
		.irq = IRQ_EINT(5),
	},
#endif
#ifdef CONFIG_TC4_GB// liang
#if defined(CONFIG_MPU_SENSORS_MPU3050) || defined(CONFIG_MPU_SENSORS_MPU3050_MODULE)
	// liang
	{
		I2C_BOARD_INFO(SENSOR_MPU_NAME, 0x68),
		//.irq = gpio_to_irq(MPUGPIO),
		.irq = IRQ_EINT(27),
		.platform_data = &mpu_data,
	},
#endif  //yulu for test sleep
#endif
#ifdef CONFIG_TC4_ICS
//deleted by D.Z 2012-10-23 follow sensor were not be used
/*
	// gyro
	{
		I2C_BOARD_INFO(MPU_NAME, 0x68),
		.irq = IRQ_EINT(27),
		.platform_data = &mpu3050_data,
	},
	// compass
	{
		I2C_BOARD_INFO("hmc5883", (0x3c>>1)),
		//.irq = IRQ_EINT(28),
		.platform_data = &inv_mpu_hmc5883_data,
	},
	// accel
	{
		I2C_BOARD_INFO("bma250", (0x30>>1)),
		//.irq = IRQ_EINT(24),// 25?
		.platform_data = &inv_mpu_bma250_data,
	},
*/
//deleted end D.Z 2012-10-23
#endif

 // Cellon add start, ZePeng Wu, 2012/08/04, for TP 
#if defined(CONFIG_RMI4_BUS)
	{
        I2C_BOARD_INFO("rmi_i2c", S3202_ADDR),
        .platform_data = &s3202_platformdata,
    },
#endif
 // Cellon add end, ZePeng Wu, 2012/08/04, for TP 
  // Cellon add start, Ted Shi, 2012/10/07, for ar1000 driver porting 
 #if defined(CONFIG_RADIO_AR1000)
	{
		I2C_BOARD_INFO("ar1000",0x10),
	},
#endif
  // Cellon add end, Ted Shi, 2012/10/07
};


/* I2C7 */
static struct i2c_board_info i2c_devs7[] __initdata = {
#if defined(CONFIG_SENSOR_ST_L3G4200D)	// Gyroscope sensor
	{
		I2C_BOARD_INFO(L3G4200D_DEV_NAME, L3G4200D_I2C_ADDR),
		.platform_data  = &l3g4200d_gyro_pdata,
	},
#endif

#if defined(CONFIG_SENSOR_ROHM_BH1721)	// Light sensor
	{
		I2C_BOARD_INFO(ROHM_I2C_NAME, ROHM_I2C_ADDR),
		.platform_data = &rohm_bh1721_pdata,
	},
#endif
};


#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data pmem_pdata = {
	.name		= "pmem",
	.no_allocator	= 1,
	.cached		= 0,
	.start		= 0,
	.size		= 0
};

static struct android_pmem_platform_data pmem_gpu1_pdata = {
	.name		= "pmem_gpu1",
	.no_allocator	= 1,
	.cached		= 0,
	.start		= 0,
	.size		= 0,
};

static struct platform_device pmem_device = {
	.name	= "android_pmem",
	.id	= 0,
	.dev	= {
		.platform_data = &pmem_pdata
	},
};

static struct platform_device pmem_gpu1_device = {
	.name	= "android_pmem",
	.id	= 1,
	.dev	= {
		.platform_data = &pmem_gpu1_pdata
	},
};

static void __init android_pmem_set_platdata(void)
{
#if defined(CONFIG_S5P_MEM_CMA)
	pmem_pdata.size = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM * SZ_1K;
	pmem_gpu1_pdata.size = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM_GPU1 * SZ_1K;
#endif
}
#endif

#ifdef CONFIG_BATTERY_SAMSUNG
static struct platform_device samsung_device_battery = {
	.name	= "samsung-fake-battery",
	.id	= -1,
};
#endif

/* s5p-pmic interface */
static struct resource s5p_pmic_resource[] = {

};


struct platform_device s5p_device_pmic = {
  .name             = "s5p-pmic",
  .id               = -1,
  .num_resources    = ARRAY_SIZE(s5p_pmic_resource),
  .resource         = s5p_pmic_resource,


  
};

EXPORT_SYMBOL(s5p_device_pmic);

#ifdef CONFIG_SWITCH_GPIO
#include <linux/switch.h>
static struct gpio_switch_platform_data headset_switch_data = {
       .name = "h2w",
       .gpio = EXYNOS4_GPX2(2), // "GPX2"
};

static struct resource switch_gpio_resource[] = {
        [0] = {
                .start  = IRQ_EINT(18), // WAKEUP_INT2[2]
                .end    = IRQ_EINT(18),
                .flags  = IORESOURCE_IRQ,
        },
};

static struct platform_device headset_switch_device = {
        .name             = "switch-gpio",
        .dev = {
                .platform_data    = &headset_switch_data,
        },
        .num_resources  = ARRAY_SIZE(switch_gpio_resource),
        .resource = switch_gpio_resource,
};
#endif

#if 0
static struct gpio_event_direct_entry smdk4x12_keypad_key_map[] = {
	{
		.gpio   = EXYNOS4_GPX0(0),
		.code   = KEY_POWER,
	}
};

static struct gpio_event_input_info smdk4x12_keypad_key_info = {
	.info.func              = gpio_event_input_func,
	.info.no_suspend        = true,
	.debounce_time.tv64	= 5 * NSEC_PER_MSEC,
	.type                   = EV_KEY,
	.keymap                 = smdk4x12_keypad_key_map,
	.keymap_size            = ARRAY_SIZE(smdk4x12_keypad_key_map)
};

static struct gpio_event_info *smdk4x12_input_info[] = {
	&smdk4x12_keypad_key_info.info,
};

static struct gpio_event_platform_data smdk4x12_input_data = {
	.names  = {
		"smdk4x12-keypad",
		NULL,
	},
	.info           = smdk4x12_input_info,
	.info_count     = ARRAY_SIZE(smdk4x12_input_info),
};

static struct platform_device smdk4x12_input_device = {
	.name   = GPIO_EVENT_DEV_NAME,
	.id     = 0,
	.dev    = {
		.platform_data = &smdk4x12_input_data,
	},
};
#endif
static void __init smdk4x12_gpio_power_init(void)
{
	int err = 0;

	err = gpio_request_one(EXYNOS4_GPX0(0), 0, "GPX0");
	if (err) {
		printk(KERN_ERR "failed to request GPX0 for "
				"suspend/resume control\n");
		return;
	}
	s3c_gpio_setpull(EXYNOS4_GPX0(0), S3C_GPIO_PULL_NONE);

	gpio_free(EXYNOS4_GPX0(0));
}

static uint32_t smdk4x12_keymap[] __initdata = {
	/* KEY(row, col, keycode) */
	//KEY(0, 0, KEY_VOLUMEDOWN), KEY(0, 1, KEY_VOLUMEUP), KEY(0, 2, KEY_HOME), KEY(0, 3, KEY_MENU), KEY(0, 4, KEY_BACK),KEY(0, 5, KEY_POWER)	//volume up ---volume down
/*add the key for C8690 by zhao.jin 20120820*/
KEY(0, 0, KEY_VOLUMEUP), KEY(0, 1, KEY_CAMERA_FOCUS), KEY(1, 0, KEY_VOLUMEDOWN), KEY(1, 1, KEY_CAMERA),KEY(0, 5, KEY_POWER)
};

static struct matrix_keymap_data smdk4x12_keymap_data __initdata = {
	.keymap		= smdk4x12_keymap,
	.keymap_size	= ARRAY_SIZE(smdk4x12_keymap),
};

static struct samsung_keypad_platdata smdk4x12_keypad_data __initdata = {
	.keymap_data	= &smdk4x12_keymap_data,
	.rows		= 2,
	.cols		= 2,  //modify from 1 to 2 for C8690 
};

#ifdef CONFIG_WAKEUP_ASSIST
static struct platform_device wakeup_assist_device = {
	.name   = "wakeup_assist",
};
#endif

#ifdef CONFIG_VIDEO_FIMG2D
static struct fimg2d_platdata fimg2d_data __initdata = {
	.hw_ver = 0x41,
	.parent_clkname = "mout_g2d0",
	.clkname = "sclk_fimg2d",
	.gate_clkname = "fimg2d",
	.clkrate = 267 * 1000000,	/* 266 Mhz */
};
#endif

#ifdef CONFIG_EXYNOS_C2C
struct exynos_c2c_platdata smdk4x12_c2c_pdata = {
	.setup_gpio	= NULL,
	.shdmem_addr	= C2C_SHAREDMEM_BASE,
	.shdmem_size	= C2C_MEMSIZE_64,
	.ap_sscm_addr	= NULL,
	.cp_sscm_addr	= NULL,
	.rx_width	= C2C_BUSWIDTH_16,
	.tx_width	= C2C_BUSWIDTH_16,
	.clk_opp100	= 400,
	.clk_opp50	= 266,
	.clk_opp25	= 0,
	.default_opp_mode	= C2C_OPP50,
	.get_c2c_state	= NULL,
	.c2c_sysreg	= S5P_VA_CMU + 0x12000,
};
#endif

#ifdef CONFIG_USB_EXYNOS_SWITCH
static struct s5p_usbswitch_platdata smdk4x12_usbswitch_pdata;

static void __init smdk4x12_usbswitch_init(void)
{
	struct s5p_usbswitch_platdata *pdata = &smdk4x12_usbswitch_pdata;
	int err;

	pdata->gpio_host_detect = EXYNOS4_GPX3(5); /* low active */
	err = gpio_request_one(pdata->gpio_host_detect, GPIOF_IN, "HOST_DETECT");
	if (err) {
		printk(KERN_ERR "failed to request gpio_host_detect\n");
		return;
	}

	s3c_gpio_cfgpin(pdata->gpio_host_detect, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(pdata->gpio_host_detect, S3C_GPIO_PULL_NONE);
	gpio_free(pdata->gpio_host_detect);

// Cellon delete start, Ted Shi, 2012/12/13, for C8690 FM INT GPIO
//	pdata->gpio_device_detect = EXYNOS4_GPX3(4); /* high active */
// Cellon delete end, Ted Shi, 2012/12/13
	err = gpio_request_one(pdata->gpio_device_detect, GPIOF_IN, "DEVICE_DETECT");
	if (err) {
		printk(KERN_ERR "failed to request gpio_host_detect for\n");
		return;
	}

	s3c_gpio_cfgpin(pdata->gpio_device_detect, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(pdata->gpio_device_detect, S3C_GPIO_PULL_NONE);
	gpio_free(pdata->gpio_device_detect);

	if (samsung_board_rev_is_0_0())
		pdata->gpio_host_vbus = 0;
	else {
		pdata->gpio_host_vbus = EXYNOS4_GPL2(0);
		err = gpio_request_one(pdata->gpio_host_vbus, GPIOF_OUT_INIT_LOW, "HOST_VBUS_CONTROL");
		if (err) {
			printk(KERN_ERR "failed to request gpio_host_vbus\n");
			return;
		}

		s3c_gpio_setpull(pdata->gpio_host_vbus, S3C_GPIO_PULL_NONE);
		gpio_free(pdata->gpio_host_vbus);
	}
	s5p_usbswitch_set_platdata(pdata);
}
#endif

#ifdef CONFIG_BUSFREQ_OPP
/* BUSFREQ to control memory/bus*/
static struct device_domain busfreq;
#endif

static struct platform_device exynos4_busfreq = {
	.id = -1,
	.name = "exynos-busfreq",
};
// Cellon delete start, Ted Shi, 2012/07/30, for porting bcm4330 wifi 
// SEMCO
/* The sdhci_s3c_sdio_card_detect function is used for detecting
   the WiFi/BT module when the menu for enabling the WiFi is
   selected.
   The semco_a31_detection function is called by ar6000's probe function.

   The call sequence is

   ar6000_pm_probe() -> plat_setup_power_for_onoff() -> detect_semco_wlan_for_onoff()
   -> setup_semco_wlan_power_onoff() -> semco_a31_detection()

   The mmc_semco_a31_sdio_remove function is used for removing the mmc driver
   when the menu for disabling the WiFi is selected.
   The semco_a31_removal function is called by ar6000's remove function.

   The call sequence is

   ar6000_pm_remove() -> plat_setup_power_for_onoff() -> detect_semco_wlan_for_onoff()
   -> setup_semco_wlan_power_onoff() -> semco_a31_removal()

   The setup_semco_wlan_power function is only used for sleep/wakeup. It controls only 
   the power of A31 module only(Do not card detection/removal function)
*/
/*   
extern void sdhci_s3c_sdio_card_detect(struct platform_device *pdev);
void semco_a31_detection(void)
{
	sdhci_s3c_sdio_card_detect(&s3c_device_hsmmc3);
}
EXPORT_SYMBOL(semco_a31_detection);


extern void mmc_semco_a31_sdio_remove(void);
void semco_a31_removal(void)
{
	mmc_semco_a31_sdio_remove();
}
EXPORT_SYMBOL(semco_a31_removal);

static struct platform_device s3c_wlan_ar6000_pm_device = {
        .name           = "wlan_ar6000_pm_dev",
        .id             = 1,
        .num_resources  = 0,
        .resource       = NULL,
};

static struct platform_device bt_sysfs = {
        .name = "bt-sysfs",
        .id = -1,
};
*/
// END SEMCO
// Cellon delete end, Ted Shi, 2012/07/30 
// Cellon add start, Ted Shi, 2012/09/24, for porting bcm4330 bt 
#ifdef CONFIG_BCM_BT
/* Bluetooth */
static struct platform_device bcm4330_bluetooth_device = {
	.name = "bcm4330_bluetooth",
	.id = -1,
};

static struct resource bluetooth_sleep_resource[] = {
        [0] = {
                .start  = IRQ_EINT(13), // WAKEUP_INT2[2]
                .end    = IRQ_EINT(13),
                .name = "wake_host_irq",
                .flags  = IORESOURCE_IRQ,
        },
        [1] = {
                .start  = EXYNOS4_GPX1(5), 
                .end    = EXYNOS4_GPX1(5),
                .name = "bt_wake_host",
                .flags  = IORESOURCE_IO,
        },
        [2] = {
                .start  = EXYNOS4_GPC1(1), 
                .end    = EXYNOS4_GPC1(1),
                .name = "host_wake_bt",
                .flags  = IORESOURCE_IO,
        },
};

static struct platform_device bcm4330_btsleep_device = {
	.name = "bluesleep",
	.id = -1,
       .num_resources  = ARRAY_SIZE(bluetooth_sleep_resource),
       .resource = bluetooth_sleep_resource,
};
#endif
// Cellon add end, Ted Shi, 2012/09/24

//Cellon delete start,Fengying Zhang,2012/08/21
/*struct platform_device s3c_device_gps = {
        .name   = "si_gps",
        .id             = -1,
};*/
//Cellon delete end,Fengying Zhang,2012/08/21
static  struct  i2c_gpio_platform_data  i2c0_platdata = {
        .sda_pin                = EXYNOS4_GPD1(0),
        .scl_pin                = EXYNOS4_GPD1(1),
        .udelay                 = 1 ,  
        .sda_is_open_drain      = 0,
        .scl_is_open_drain      = 0,
        .scl_is_output_only     = 0,
//      .scl_is_output_only     = 1,
      };

static struct platform_device s3c_device_i2c0_gpio = {
        .name                           = "i2c-gpio",
        .id                                     = 0,
        .dev.platform_data      = &i2c0_platdata,
};

#ifdef CONFIG_GPIO_SMM6260
//shengliang
struct platform_device smm6260_device = {
	.name	= "smm6260-gpio",
	.id	= -1,
};
#endif

#ifdef CONFIG_VIBRATOR
struct platform_device s5p_vib_dev = {
	.name	= "s5p-vib",
	.id	= -1,
};
#endif

#if defined (CONFIG_S5P_SYSTEM_MMU) && defined(CONFIG_TC4_GB)

extern struct platform_device s5p_device_sysmmu;
#endif
static struct platform_device *smdk4412_devices[] __initdata = {
	&s3c_device_adc,
};

static struct platform_device *smdk4x12_devices[] __initdata = {
#ifdef CONFIG_ANDROID_PMEM
	&pmem_device,
	&pmem_gpu1_device,
#endif
	/* Samsung Power Domain */
	&exynos4_device_pd[PD_MFC],
	&exynos4_device_pd[PD_G3D],
	&exynos4_device_pd[PD_LCD0],
	&exynos4_device_pd[PD_CAM],
	&exynos4_device_pd[PD_TV],
	&exynos4_device_pd[PD_GPS],
	&exynos4_device_pd[PD_GPS_ALIVE],
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
	&exynos4_device_pd[PD_ISP],
#endif
#ifdef CONFIG_FB_MIPI_DSIM
	&s5p_device_mipi_dsim,
#endif
/* mainline fimd */
#ifdef CONFIG_FB_S3C
	&s5p_device_fimd0,
#if defined(CONFIG_LCD_AMS369FG06) || defined(CONFIG_LCD_LMS501KF03)
	&s3c_device_spi_gpio,
#elif defined(CONFIG_LCD_WA101S)
	&smdk4x12_lcd_wa101s,
#elif defined(CONFIG_LCD_LTE480WV)
	&smdk4x12_lcd_lte480wv,
#elif defined(CONFIG_LCD_MIPI_S6E63M0)
	&smdk4x12_mipi_lcd,
#endif
#endif
	/* legacy fimd */
#ifdef CONFIG_FB_S5P
	&s3c_device_fb,
#ifdef CONFIG_FB_S5P_LMS501KF03
	&s3c_device_spi_gpio,
#endif
#endif
	&s3c_device_wdt,
	&s3c_device_rtc,
	//&s3c_device_i2c0,
	&s3c_device_i2c0_gpio, //added yqf 
	&s3c_device_i2c1,
	//&s3c_device_i2c2,
	&s3c_device_i2c3,
	&s3c_device_i2c4,
	&s3c_device_i2c5,
	&s3c_device_i2c6,//For S5K4EC
	&s3c_device_i2c7,

#if !defined(CONFIG_REGULATOR_MAX8997)	
	&s5p_device_pmic,
#endif	

	&tc4_regulator_consumer,

	//robin, no need&s3c_device_adc,//wenpin.cui
#ifdef CONFIG_S3FHRN2_I2C_GPIO
	&s3c_device_i2c_nfc_gpio,
#endif
#ifdef CONFIG_S3FHRN2_UART
	&s3c_device_nfc_uart,
#endif

#ifdef CONFIG_USB_EHCI_S5P
	//&s5p_device_ehci, //move the device register to ehci_hcd.c  from samsung patch 20121126 by justy.yang.
#endif
#ifdef CONFIG_USB_OHCI_S5P
	&s5p_device_ohci,
#endif
#ifdef CONFIG_USB_GADGET
	&s3c_device_usbgadget,
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	&s3c_device_rndis,
#endif
#ifdef CONFIG_USB_ANDROID
	&s3c_device_android_usb,
	&s3c_device_usb_mass_storage,
#endif
// Cellon delete start, Ted Shi, 2012/07/30, for porting bcm4330 wifi 
// SEMCO
//    &s3c_wlan_ar6000_pm_device,
//    &bt_sysfs,
// END SEMCO
// Cellon delete end, Ted Shi, 2012/07/30 
// Cellon add start, Ted Shi, 2012/09/24, for porting bcm4330 wifi 
#ifdef CONFIG_BCM_BT
	&bcm4330_bluetooth_device,
	&bcm4330_btsleep_device,
#endif
// Cellon add end, Ted Shi, 2012/09/24 
// Cellon add start, Ted Shi, 2012/10/07, for ar1000 driver porting
#ifdef CONFIG_RADIO_AR1000
	&radio_ar1000,
#endif
// Cellon add end, Ted Shi, 2012/10/07 
#ifdef CONFIG_S5P_DEV_MSHC
	&s3c_device_mshci,//lisw sd mshci should be probe before hsmmc
#endif
#ifdef CONFIG_S3C_DEV_HSMMC
//	&s3c_device_hsmmc0,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
//	&s3c_device_hsmmc1,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	&s3c_device_hsmmc2,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	&s3c_device_hsmmc3,
#endif
#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	&exynos_device_dwmci,
#endif
#ifdef CONFIG_SND_SAMSUNG_AC97
	&exynos_device_ac97,
#endif
#ifdef CONFIG_SND_SAMSUNG_I2S
	&exynos_device_i2s0,
#endif
#ifdef CONFIG_SND_SAMSUNG_PCM
	&exynos_device_pcm0,
#endif
#ifdef CONFIG_SND_SAMSUNG_SPDIF
	&exynos_device_spdif,
#endif
#if defined(CONFIG_SND_SAMSUNG_RP) || defined(CONFIG_SND_SAMSUNG_ALP)
	&exynos_device_srp,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
	&exynos4_device_fimc_is,
#endif
#ifdef CONFIG_VIDEO_TVOUT
	&s5p_device_tvout,
	&s5p_device_cec,
	&s5p_device_hpd,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_TV
	&s5p_device_i2c_hdmiphy,
	&s5p_device_hdmi,
	&s5p_device_sdo,
	&s5p_device_mixer,
	&s5p_device_cec,
#endif
#if defined(CONFIG_VIDEO_FIMC)
	&s3c_device_fimc0,
	&s3c_device_fimc1,
	&s3c_device_fimc2,
	&s3c_device_fimc3,
/* CONFIG_VIDEO_SAMSUNG_S5P_FIMC is the feature for mainline */
#elif defined(CONFIG_VIDEO_SAMSUNG_S5P_FIMC)
	&s5p_device_fimc0,
	&s5p_device_fimc1,
	&s5p_device_fimc2,
	&s5p_device_fimc3,
#endif
#if defined(CONFIG_VIDEO_FIMC_MIPI)
	&s3c_device_csis0,
	&s3c_device_csis1,
#elif defined(CONFIG_VIDEO_S5P_MIPI_CSIS)
	&s5p_device_mipi_csis0,
	&s5p_device_mipi_csis1,
#endif
#ifdef CONFIG_VIDEO_S5P_MIPI_CSIS
	&mipi_csi_fixed_voltage,
#endif
#ifdef CONFIG_VIDEO_M5MOLS
	&m5mols_fixed_voltage,
#endif

#if defined(CONFIG_VIDEO_MFC5X) || defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
	&s5p_device_mfc,
#endif
#ifdef CONFIG_S5P_SYSTEM_MMU
#ifdef CONFIG_TC4_GB
	&s5p_device_sysmmu,
#else
	&SYSMMU_PLATDEV(g2d_acp),
	&SYSMMU_PLATDEV(fimc0),
	&SYSMMU_PLATDEV(fimc1),
	&SYSMMU_PLATDEV(fimc2),
	&SYSMMU_PLATDEV(fimc3),
	&SYSMMU_PLATDEV(jpeg),
	&SYSMMU_PLATDEV(mfc_l),
	&SYSMMU_PLATDEV(mfc_r),
	&SYSMMU_PLATDEV(tv),
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
	&SYSMMU_PLATDEV(is_isp),
	&SYSMMU_PLATDEV(is_drc),
	&SYSMMU_PLATDEV(is_fd),
	&SYSMMU_PLATDEV(is_cpu),
#endif
#endif //CONIG_TC4_GB
#endif /* CONFIG_S5P_SYSTEM_MMU */

#ifdef CONFIG_ION_EXYNOS
	&exynos_device_ion,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
	&exynos_device_flite0,
	&exynos_device_flite1,
#endif
#ifdef CONFIG_VIDEO_FIMG2D
	&s5p_device_fimg2d,
#endif
#ifdef CONFIG_EXYNOS_MEDIA_DEVICE
	&exynos_device_md0,
#endif
#if	defined(CONFIG_VIDEO_JPEG_V2X) || defined(CONFIG_VIDEO_JPEG)

	&s5p_device_jpeg,
#endif
	&wm8994_fixed_voltage0,
	&wm8994_fixed_voltage1,
	&wm8994_fixed_voltage2,
	&samsung_asoc_dma,
	&samsung_asoc_idma,
#ifdef CONFIG_BATTERY_SAMSUNG
	&samsung_device_battery,
#endif
	&samsung_device_keypad,
#ifdef CONFIG_WAKEUP_ASSIST
	&wakeup_assist_device,
#endif
#ifdef CONFIG_EXYNOS_C2C
	&exynos_device_c2c,
#endif
	//&smdk4x12_input_device, yulu
	&smdk4x12_smsc911x,
#ifdef CONFIG_S3C64XX_DEV_SPI
	&exynos_device_spi0,
#ifndef CONFIG_FB_S5P_LMS501KF03
	&exynos_device_spi1,
#endif
	&exynos_device_spi2,
#endif
#ifdef CONFIG_EXYNOS_SETUP_THERMAL
	//&exynos_device_tmu,
	&s5p_device_tmu,
#endif
#ifdef CONFIG_S5P_DEV_ACE
	&s5p_device_ace,
#endif
	&exynos4_busfreq,
#ifdef CONFIG_SWITCH_GPIO
        &headset_switch_device,
#endif
      //Cellon delete start,Fengying Zhang,2012/08/21
	//&s3c_device_gps,
	//Cellon delete end,Fengying Zhang,2012/08/21
	//&smm6260_modem,
#ifdef CONFIG_GPIO_SMM6260
	&smm6260_device,//liang
#endif
#ifdef CONFIG_SMM6260_MODEM
	&smm6260_modem,
#endif	
#ifdef CONFIG_VIBRATOR
	&s5p_vib_dev,
#endif
};
#ifdef CONFIG_EXYNOS_SETUP_THERMAL
#if 0
/* below temperature base on the celcius degree */
struct tmu_data exynos_tmu_data __initdata = {
	.ts = {
		.stop_throttle  = 82,
		.start_throttle = 85,
		.stop_warning  = 95,
		.start_warning = 103,
		.start_tripping = 110, /* temp to do tripping */
	},
	.efuse_value = 55,
	.slope = 0x10008802,
	.mode = 0,
};
#else
/* below temperature base on the celcius degree */
struct s5p_platform_tmu exynos_tmu_data __initdata = {
        .ts = {
                .stop_1st_throttle  = 78,
                .start_1st_throttle = 80,
                .stop_2nd_throttle  = 87,
                .start_2nd_throttle = 103,
                .start_tripping     = 110, /* temp to do tripping */
                .start_emergency    = 120, /* To protect chip,forcely kernel panic */
                .stop_mem_throttle  = 80,
                .start_mem_throttle = 85,
                .stop_tc  = 13,
                .start_tc = 10,
        },
        .cpufreq = {
                .limit_1st_throttle  = 800000, /* 800MHz in KHz order */
                .limit_2nd_throttle  = 200000, /* 200MHz in KHz order */
        },
        .temp_compensate = {
                .arm_volt = 925000, /* vdd_arm in uV for temperature compensation */
                .bus_volt = 900000, /* vdd_bus in uV for temperature compensation */
                .g3d_volt = 900000, /* vdd_g3d in uV for temperature compensation */
        },
};
#endif
#endif


#if defined(CONFIG_VIDEO_TVOUT)
static struct s5p_platform_hpd hdmi_hpd_data __initdata = {

};
static struct s5p_platform_cec hdmi_cec_data __initdata = {

};
#endif
#ifdef CONFIG_VIDEO_EXYNOS_HDMI_CEC
static struct s5p_platform_cec hdmi_cec_data __initdata = {

};
#endif

#ifdef CONFIG_VIDEO_SAMSUNG_S5P_FIMC
static struct s5p_fimc_isp_info isp_info[] = {

#if defined(CONFIG_SOC_CAMERA_MT9D115)
	{
		.board_info	= &mt9d115_i2c_info,
		.clk_frequency  = 24000000UL,
		.bus_type	= FIMC_ITU_601,//CAM_TYPE_ITU,
		.i2c_bus_num	= 7,
		.mux_id	= 0, /* A-Port : 0, B-Port : 1 */
		.flags		= FIMC_CLK_INV_VSYNC,
		.csi_data_align = 32,
	},
#endif


#if defined(CONFIG_VIDEO_S5K4ECGX)	//yulu
	{
		.board_info	= &s5k4ecgx_i2c_info,
		.clk_frequency  = 24000000UL,
		.bus_type	= FIMC_MIPI_CSI2,//CAM_TYPE_MIPI,
		.i2c_bus_num	= 6,
		.mux_id	= 0, /* A-Port : 0, B-Port : 1 */
		.flags		= FIMC_CLK_INV_VSYNC,
		.csi_data_align = 32,
		/*.cam_power = ,*/
	},
#endif





#if defined(CONFIG_VIDEO_S5K4BA)
	{
		.board_info	= &s5k4ba_info,
		.clk_frequency  = 24000000UL,
		.bus_type	= FIMC_ITU_601,
#ifdef CONFIG_ITU_A
		.i2c_bus_num	= 0,
		.mux_id		= 0, /* A-Port : 0, B-Port : 1 */
#endif
#ifdef CONFIG_ITU_B
		.i2c_bus_num	= 1,
		.mux_id		= 1, /* A-Port : 0, B-Port : 1 */
#endif
		.flags		= FIMC_CLK_INV_VSYNC,
	},
#endif
#if defined(CONFIG_VIDEO_S5K4EA)
	{
		.board_info	= &s5k4ea_info,
		.clk_frequency  = 24000000UL,
		.bus_type	= FIMC_MIPI_CSI2,
#ifdef CONFIG_CSI_C
		.i2c_bus_num	= 0,
		.mux_id		= 0, /* A-Port : 0, B-Port : 1 */
#endif
#ifdef CONFIG_CSI_D
		.i2c_bus_num	= 1,
		.mux_id		= 1, /* A-Port : 0, B-Port : 1 */
#endif
		.flags		= FIMC_CLK_INV_VSYNC,
		.csi_data_align = 32,
	},
#endif
#if defined(CONFIG_VIDEO_M5MOLS)
	{
		.board_info	= &m5mols_board_info,
		.clk_frequency  = 24000000UL,
		.bus_type	= FIMC_MIPI_CSI2,
#ifdef CONFIG_CSI_C
		.i2c_bus_num	= 4,
		.mux_id		= 0, /* A-Port : 0, B-Port : 1 */
#endif
#ifdef CONFIG_CSI_D
		.i2c_bus_num	= 5,
		.mux_id		= 1, /* A-Port : 0, B-Port : 1 */
#endif
		.flags		= FIMC_CLK_INV_PCLK | FIMC_CLK_INV_VSYNC,
		.csi_data_align = 32,
	},
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
#if defined(CONFIG_VIDEO_S5K3H2)
	{
		.board_info	= &s5k3h2_sensor_info,
		.clk_frequency  = 24000000UL,
		.bus_type	= FIMC_MIPI_CSI2,
#ifdef CONFIG_S5K3H2_CSI_C
		.i2c_bus_num	= 0,
		.mux_id		= 0, /* A-Port : 0, B-Port : 1 */
		.flite_id	= FLITE_IDX_A,
		.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_S5K3H2_CSI_D
		.i2c_bus_num	= 1,
		.mux_id		= 1, /* A-Port : 0, B-Port : 1 */
		.flite_id	= FLITE_IDX_B,
		.cam_power	= smdk4x12_cam1_reset,
#endif
		.flags		= 0,
		.csi_data_align = 24,
		.use_isp	= true,
	},
#endif
#if defined(CONFIG_VIDEO_S5K3H7)
	{
		.board_info	= &s5k3h7_sensor_info,
		.clk_frequency  = 24000000UL,
		.bus_type	= FIMC_MIPI_CSI2,
#ifdef CONFIG_S5K3H7_CSI_C
		.i2c_bus_num	= 0,
		.mux_id		= 0, /* A-Port : 0, B-Port : 1 */
		.flite_id	= FLITE_IDX_A,
		.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_S5K3H7_CSI_D
		.i2c_bus_num	= 1,
		.mux_id		= 1, /* A-Port : 0, B-Port : 1 */
		.flite_id	= FLITE_IDX_B,
		.cam_power	= smdk4x12_cam1_reset,
#endif
		.csi_data_align = 24,
		.use_isp	= true,
	},
#endif
#if defined(CONFIG_VIDEO_S5K4E5)
	{
		.board_info	= &s5k4e5_sensor_info,
		.clk_frequency  = 24000000UL,
		.bus_type	= FIMC_MIPI_CSI2,
#ifdef CONFIG_S5K4E5_CSI_C
		.i2c_bus_num	= 0,
		.mux_id		= 0, /* A-Port : 0, B-Port : 1 */
		.flite_id	= FLITE_IDX_A,
		.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_S5K4E5_CSI_D
		.i2c_bus_num	= 1,
		.mux_id		= 1, /* A-Port : 0, B-Port : 1 */
		.flite_id	= FLITE_IDX_B,
		.cam_power	= smdk4x12_cam1_reset,
#endif
		.csi_data_align = 24,
		.use_isp	= true,
	},
#endif
#if defined(CONFIG_VIDEO_S5K6A3)
	{
		.board_info	= &s5k6a3_sensor_info,
		.clk_frequency  = 12000000UL,
		.bus_type	= FIMC_MIPI_CSI2,
#ifdef CONFIG_S5K6A3_CSI_C
		.i2c_bus_num	= 0,
		.mux_id		= 0, /* A-Port : 0, B-Port : 1 */
		.flite_id	= FLITE_IDX_A,
		.cam_power	= smdk4x12_cam0_reset,
#endif
#ifdef CONFIG_S5K6A3_CSI_D
		.i2c_bus_num	= 1,
		.mux_id		= 1, /* A-Port : 0, B-Port : 1 */
		.flite_id	= FLITE_IDX_B,
		.cam_power	= smdk4x12_cam1_reset,
#endif
		.flags		= 0,
		.csi_data_align = 12,
		.use_isp	= true,
	},
#endif
#endif
#if defined(WRITEBACK_ENABLED)
	{
		.board_info	= &writeback_info,
		.bus_type	= FIMC_LCD_WB,
		.i2c_bus_num	= 0,
		.mux_id		= 0, /* A-Port : 0, B-Port : 1 */
		.flags		= FIMC_CLK_INV_VSYNC,
	},
#endif
};

static void __init smdk4x12_subdev_config(void)
{

#if 0
#if defined(CONFIG_VIDEO_S5K4ECGX)
	s3c_fimc0_default_data.isp_info[0] = &isp_info[0];//&s5k4ec;
	s3c_fimc0_default_data.isp_info[0]->use_cam = true;
	//s3c_fimc0_default_data.cam[0] = &s5k4ecgx;
	//exynos_flite0_default_data.cam[flite0_cam_index] = &s5k4ecgx;
	//exynos_flite0_default_data.isp_info[flite0_cam_index] = &isp_info[0];//&s5k4ec;
	//flite0_cam_index++;
#endif

#if defined(CONFIG_SOC_CAMERA_MT9D115)
	s3c_fimc0_default_data.isp_info[1] = &isp_info[1];
	s3c_fimc0_default_data.isp_info[1]->use_cam = true;
	//exynos_flite1_default_data.cam[flite1_cam_index] = &mt9d115;
	//exynos_flite1_default_data.isp_info[flite1_cam_index] = &isp_info[1];
	//flite0_cam_index++;
#endif

#else

#if defined(CONFIG_SOC_CAMERA_MT9D115)
	s3c_fimc0_default_data.isp_info[0] = &isp_info[0];
	s3c_fimc0_default_data.isp_info[0]->use_cam = true;
	s3c_fimc1_default_data.isp_info[0] = &isp_info[0];
	s3c_fimc1_default_data.isp_info[0]->use_cam = false;
	//exynos_flite1_default_data.cam[flite1_cam_index] = &mt9d115;
	//exynos_flite1_default_data.isp_info[flite1_cam_index] = &isp_info[1];
	//flite0_cam_index++;
#endif


#if defined(CONFIG_VIDEO_S5K4ECGX)
	s3c_fimc0_default_data.isp_info[1] = &isp_info[1];//&s5k4ec;
	s3c_fimc0_default_data.isp_info[1]->use_cam = true;
	s3c_fimc1_default_data.isp_info[1] = &isp_info[1];
	s3c_fimc1_default_data.isp_info[1]->use_cam = false;
	//s3c_fimc0_default_data.cam[0] = &s5k4ecgx;
	//exynos_flite0_default_data.cam[flite0_cam_index] = &s5k4ecgx;
	//exynos_flite0_default_data.isp_info[flite0_cam_index] = &isp_info[0];//&s5k4ec;
	//flite0_cam_index++;
#endif
#endif


#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
#ifdef CONFIG_VIDEO_S5K3H2
#ifdef CONFIG_S5K3H2_CSI_C
	s5p_mipi_csis0_default_data.clk_rate	= 160000000;
	s5p_mipi_csis0_default_data.lanes	= 2;
	s5p_mipi_csis0_default_data.alignment	= 24;
	s5p_mipi_csis0_default_data.hs_settle	= 12;
#endif
#ifdef CONFIG_S5K3H2_CSI_D
	s5p_mipi_csis1_default_data.clk_rate	= 160000000;
	s5p_mipi_csis1_default_data.lanes	= 2;
	s5p_mipi_csis1_default_data.alignment	= 24;
	s5p_mipi_csis1_default_data.hs_settle	= 12;
#endif
#endif
#ifdef CONFIG_VIDEO_S5K3H7
#ifdef CONFIG_S5K3H7_CSI_C
	s5p_mipi_csis0_default_data.clk_rate	= 160000000;
	s5p_mipi_csis0_default_data.lanes	= 2;
	s5p_mipi_csis0_default_data.alignment	= 24;
	s5p_mipi_csis0_default_data.hs_settle	= 12;
#endif
#ifdef CONFIG_S5K3H7_CSI_D
	s5p_mipi_csis1_default_data.clk_rate	= 160000000;
	s5p_mipi_csis1_default_data.lanes	= 2;
	s5p_mipi_csis1_default_data.alignment	= 24;
	s5p_mipi_csis1_default_data.hs_settle	= 12;
#endif
#endif
#ifdef CONFIG_VIDEO_S5K4E5
#ifdef CONFIG_S5K4E5_CSI_C
	s5p_mipi_csis0_default_data.clk_rate	= 160000000;
	s5p_mipi_csis0_default_data.lanes	= 2;
	s5p_mipi_csis0_default_data.alignment	= 24;
	s5p_mipi_csis0_default_data.hs_settle	= 12;
#endif
#ifdef CONFIG_S5K4E5_CSI_D
	s5p_mipi_csis1_default_data.clk_rate	= 160000000;
	s5p_mipi_csis1_default_data.lanes	= 2;
	s5p_mipi_csis1_default_data.alignment	= 24;
	s5p_mipi_csis1_default_data.hs_settle	= 12;
#endif
#endif
#ifdef CONFIG_VIDEO_S5K6A3
#ifdef CONFIG_S5K6A3_CSI_C
	s5p_mipi_csis0_default_data.clk_rate	= 160000000;
	s5p_mipi_csis0_default_data.lanes 	= 1;
	s5p_mipi_csis0_default_data.alignment	= 24;
	s5p_mipi_csis0_default_data.hs_settle	= 12;
#endif
#ifdef CONFIG_S5K6A3_CSI_D
	s5p_mipi_csis1_default_data.clk_rate	= 160000000;
	s5p_mipi_csis1_default_data.lanes 	= 1;
	s5p_mipi_csis1_default_data.alignment	= 24;
	s5p_mipi_csis1_default_data.hs_settle	= 12;
#endif
#endif
#endif
}

static void __init smdk4x12_camera_config(void)
{
	/* CAM A port(b0010) : PCLK, VSYNC, HREF, DATA[0-4] */
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPJ0(0), 8, S3C_GPIO_SFN(2));
	/* CAM A port(b0010) : DATA[5-7], CLKOUT(MIPI CAM also), FIELD */
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPJ1(0), 5, S3C_GPIO_SFN(2));
	/* CAM B port(b0011) : PCLK, DATA[0-6] */
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPM0(0), 8, S3C_GPIO_SFN(3));
	/* CAM B port(b0011) : FIELD, DATA[7]*/
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPM1(0), 2, S3C_GPIO_SFN(3));
	/* CAM B port(b0011) : VSYNC, HREF, CLKOUT*/
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPM2(0), 3, S3C_GPIO_SFN(3));

	/* note : driver strength to max is unnecessary */
#ifdef CONFIG_VIDEO_M5MOLS
	s3c_gpio_cfgpin(EXYNOS4_GPX2(6), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS4_GPX2(6), S3C_GPIO_PULL_NONE);
#endif
}
#endif /* CONFIG_VIDEO_SAMSUNG_S5P_FIMC */

#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
static void __set_flite_camera_config(struct exynos_platform_flite *data,
					u32 active_index, u32 max_cam)
{
	data->active_cam_index = active_index;
	data->num_clients = max_cam;
}

static void __init smdk4x12_set_camera_flite_platdata(void)
{
	int flite0_cam_index = 0;
	int flite1_cam_index = 0;
#ifdef CONFIG_VIDEO_S5K3H2
#ifdef CONFIG_S5K3H2_CSI_C
	exynos_flite0_default_data.cam[flite0_cam_index++] = &s5k3h2;
#endif
#ifdef CONFIG_S5K3H2_CSI_D
	exynos_flite1_default_data.cam[flite1_cam_index++] = &s5k3h2;
#endif
#endif
#ifdef CONFIG_VIDEO_S5K3H7
#ifdef CONFIG_S5K3H7_CSI_C
	exynos_flite0_default_data.cam[flite0_cam_index++] = &s5k3h7;
#endif
#ifdef CONFIG_S5K3H7_CSI_D
	exynos_flite1_default_data.cam[flite1_cam_index++] = &s5k3h7;
#endif
#endif
#ifdef CONFIG_VIDEO_S5K4E5
#ifdef CONFIG_S5K4E5_CSI_C
	exynos_flite0_default_data.cam[flite0_cam_index++] = &s5k4e5;
#endif
#ifdef CONFIG_S5K4E5_CSI_D
	exynos_flite1_default_data.cam[flite1_cam_index++] = &s5k4e5;
#endif
#endif

#ifdef CONFIG_VIDEO_S5K6A3
#ifdef CONFIG_S5K6A3_CSI_C
	exynos_flite0_default_data.cam[flite0_cam_index++] = &s5k6a3;
#endif
#ifdef CONFIG_S5K6A3_CSI_D
	exynos_flite1_default_data.cam[flite1_cam_index++] = &s5k6a3;
#endif
#endif
	__set_flite_camera_config(&exynos_flite0_default_data, 0, flite0_cam_index);
	__set_flite_camera_config(&exynos_flite1_default_data, 0, flite1_cam_index);
}
#endif

#if defined(CONFIG_S5P_MEM_CMA)
static void __init exynos4_cma_region_reserve(
			struct cma_region *regions_normal,
			struct cma_region *regions_secure)
{
	struct cma_region *reg;
	phys_addr_t paddr_last = 0xFFFFFFFF;

	for (reg = regions_normal; reg->size != 0; reg++) {
		phys_addr_t paddr;

		if (!IS_ALIGNED(reg->size, PAGE_SIZE)) {
			pr_err("S5P/CMA: size of '%s' is NOT page-aligned\n",
								reg->name);
			reg->size = PAGE_ALIGN(reg->size);
		}


		if (reg->reserved) {
			pr_err("S5P/CMA: '%s' alread reserved\n", reg->name);
			continue;
		}

		if (reg->alignment) {
			if ((reg->alignment & ~PAGE_MASK) ||
				(reg->alignment & ~reg->alignment)) {
				pr_err("S5P/CMA: Failed to reserve '%s': "
						"incorrect alignment 0x%08x.\n",
						reg->name, reg->alignment);
				continue;
			}
		} else {
			reg->alignment = PAGE_SIZE;
		}

		if (reg->start) {
			if (!memblock_is_region_reserved(reg->start, reg->size)
			    && (memblock_reserve(reg->start, reg->size) == 0))
				reg->reserved = 1;
			else
				pr_err("S5P/CMA: Failed to reserve '%s'\n",
								reg->name);

			continue;
		}

		paddr = memblock_find_in_range(0, MEMBLOCK_ALLOC_ACCESSIBLE,
						reg->size, reg->alignment);
		if (paddr != MEMBLOCK_ERROR) {
			if (memblock_reserve(paddr, reg->size)) {
				pr_err("S5P/CMA: Failed to reserve '%s'\n",
								reg->name);
				continue;
			}

			reg->start = paddr;
			reg->reserved = 1;
		} else {
			pr_err("S5P/CMA: No free space in memory for '%s'\n",
								reg->name);
		}

		if (cma_early_region_register(reg)) {
			pr_err("S5P/CMA: Failed to register '%s'\n",
								reg->name);
			memblock_free(reg->start, reg->size);
		} else {
			paddr_last = min(paddr, paddr_last);
		}
	}

	if (regions_secure && regions_secure->size) {
		size_t size_secure = 0;
		size_t align_secure, size_region2, aug_size, order_region2;

		for (reg = regions_secure; reg->size != 0; reg++)
			size_secure += reg->size;

		reg--;

		/* Entire secure regions will be merged into 2
		 * consecutive regions. */
		align_secure = 1 <<
			(get_order((size_secure + 1) / 2) + PAGE_SHIFT);
		/* Calculation of a subregion size */
		size_region2 = size_secure - align_secure;
		order_region2 = get_order(size_region2) + PAGE_SHIFT;
		if (order_region2 < 20)
			order_region2 = 20; /* 1MB */
		order_region2 -= 3; /* divide by 8 */
		size_region2 = ALIGN(size_region2, 1 << order_region2);

		aug_size = align_secure + size_region2 - size_secure;
		if (aug_size > 0)
			reg->size += aug_size;

		size_secure = ALIGN(size_secure, align_secure);

		if (paddr_last >= memblock.current_limit) {
			paddr_last = memblock_find_in_range(0,
					MEMBLOCK_ALLOC_ACCESSIBLE,
					size_secure, reg->alignment);
		} else {
			paddr_last -= size_secure;
			paddr_last = round_down(paddr_last, align_secure);
		}

		if (paddr_last) {
			while (memblock_reserve(paddr_last, size_secure))
				paddr_last -= align_secure;

			do {
				reg->start = paddr_last;
				reg->reserved = 1;
				paddr_last += reg->size;

				if (cma_early_region_register(reg)) {
					memblock_free(reg->start, reg->size);
					pr_err("S5P/CMA: "
					"Failed to register secure region "
					"'%s'\n", reg->name);
				} else {
					size_secure -= reg->size;
				}
			} while (reg-- != regions_secure);

			if (size_secure > 0)
				memblock_free(paddr_last, size_secure);
		} else {
			pr_err("S5P/CMA: Failed to reserve secure regions\n");
		}
	}
}

static void __init exynos4_reserve_mem(void)
{
	static struct cma_region regions[] = {
#ifdef CONFIG_ANDROID_PMEM_MEMSIZE_PMEM
		{
			.name = "pmem",
			.size = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_ANDROID_PMEM_MEMSIZE_PMEM_GPU1
		{
			.name = "pmem_gpu1",
			.size = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM_GPU1 * SZ_1K,
			.start = 0,
		},
#endif
#ifndef CONFIG_VIDEOBUF2_ION
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_TV
		{
			.name = "tv",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_TV * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_JPEG
		{
			.name = "jpeg",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_JPEG * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_AUDIO_SAMSUNG_MEMSIZE_SRP
		{
			.name = "srp",
			.size = CONFIG_AUDIO_SAMSUNG_MEMSIZE_SRP * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMG2D
		{
			.name = "fimg2d",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMG2D * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD
		{
			.name = "fimd",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC0
		{
			.name = "fimc0",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC0 * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC2
		{
			.name = "fimc2",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC2 * SZ_1K,
			.start = 0
		},
#endif
#if !defined(CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION) && \
	defined(CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3)
		{
			.name = "fimc3",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3 * SZ_1K,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1
		{
			.name = "fimc1",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1 * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC1
		{
#ifdef CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION
			.name = "mfc-normal",
#else
			.name = "mfc1",
#endif
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC1 * SZ_1K,
			{ .alignment = 1 << 17 },
		},
#endif
#if !defined(CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION) && \
	defined(CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0)
		{
			.name = "mfc0",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0 * SZ_1K,
			{ .alignment = 1 << 17 },
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC
		{
			.name = "mfc",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC * SZ_1K,
			{ .alignment = 1 << 17 },
		},
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
		{
			.name = "fimc_is",
			.size = CONFIG_VIDEO_EXYNOS_MEMSIZE_FIMC_IS * SZ_1K,
			{
				.alignment = 1 << 26,
			},
			.start = 0
		},
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS_BAYER
		{
			.name = "fimc_is_isp",
			.size = CONFIG_VIDEO_EXYNOS_MEMSIZE_FIMC_IS_ISP * SZ_1K,
			.start = 0
		},
#endif
#endif
#if !defined(CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION) && \
	defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
		{
			.name		= "b2",
			.size		= 32 << 20,
			{ .alignment	= 128 << 10 },
		},
		{
			.name		= "b1",
			.size		= 32 << 20,
			{ .alignment	= 128 << 10 },
		},
		{
			.name		= "fw",
			.size		= 1 << 20,
			{ .alignment	= 128 << 10 },
		},
#endif
#else /* !CONFIG_VIDEOBUF2_ION */
#ifdef CONFIG_FB_S5P
#error CONFIG_FB_S5P is defined. Select CONFIG_FB_S3C, instead
#endif
		{
			.name	= "ion",
			.size	= CONFIG_ION_EXYNOS_CONTIGHEAP_SIZE * SZ_1K,
		},
#endif /* !CONFIG_VIDEOBUF2_ION */
		{
			.size = 0
		},
	};
#ifdef CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION
	static struct cma_region regions_secure[] = {
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD_VIDEO
		{
			.name = "video",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD_VIDEO * SZ_1K,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3
		{
			.name = "fimc3",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3 * SZ_1K,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0
		{
			.name = "mfc-secure",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0 * SZ_1K,
		},
#endif
		{
			.name = "sectbl",
			.size = SZ_1M,
			{
				.alignment = SZ_64M,
			},
		},
		{
			.size = 0
		},
	};
#else /* !CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION */
	struct cma_region *regions_secure = NULL;
#endif
	static const char map[] __initconst =
#ifdef CONFIG_EXYNOS_C2C
		"samsung-c2c=c2c_shdmem;"
#endif
		"android_pmem.0=pmem;android_pmem.1=pmem_gpu1;"
		"s3cfb.0/fimd=fimd;exynos4-fb.0/fimd=fimd;"
#ifdef CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION
		"s3cfb.0/video=video;exynos4-fb.0/video=video;"
#endif
		"s3c-fimc.0=fimc0;s3c-fimc.1=fimc1;s3c-fimc.2=fimc2;s3c-fimc.3=fimc3;"
		"exynos4210-fimc.0=fimc0;exynos4210-fimc.1=fimc1;exynos4210-fimc.2=fimc2;exynos4210-fimc.3=fimc3;"
#ifdef CONFIG_VIDEO_MFC5X
		"s3c-mfc/A=mfc0,mfc-secure;"
		"s3c-mfc/B=mfc1,mfc-normal;"
		"s3c-mfc/AB=mfc;"
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_S5P_MFC
		"s5p-mfc/f=fw;"
		"s5p-mfc/a=b1;"
		"s5p-mfc/b=b2;"
#endif
		"samsung-rp=srp;"
		"s5p-jpeg=jpeg;"
		"exynos4-fimc-is/f=fimc_is;"
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS_BAYER
		"exynos4-fimc-is/i=fimc_is_isp;"
#endif
		"s5p-mixer=tv;"
		"s5p-fimg2d=fimg2d;"
		"ion-exynos=ion,fimd,fimc0,fimc1,fimc2,fimc3,fw,b1,b2;"
#ifdef CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION
		"s5p-smem/video=video;"
		"s5p-smem/sectbl=sectbl;"
#endif
		"s5p-smem/mfc=mfc0,mfc-secure;"
		"s5p-smem/fimc=fimc3;"
		"s5p-smem/mfc-shm=mfc1,mfc-normal;";

	cma_set_defaults(NULL, map);

	exynos4_cma_region_reserve(regions, regions_secure);
}
#endif

/* LCD Backlight data */
static struct samsung_bl_gpio_info smdk4x12_bl_gpio_info = {
//Cellon modify start, Jacob, 2012/08/30, for set LCD backLight GPIO issue
	.no = EXYNOS4_GPD0(1),
	.func = S3C_GPIO_SFN(2),
//Cellon modify end, Jacob, 2012/08/30
};

static struct platform_pwm_backlight_data smdk4x12_bl_data = {
	.pwm_id = 1,
#ifdef CONFIG_FB_S5P_LMS501KF03
	.pwm_period_ns  = 1000,
#endif
};
// add leds' platform parameters by zhaojin 20120820
#ifdef CONFIG_LEDS_KP
static struct gpio_led smdk4x12_kp_leds[] = {
	{
		.name = "KP_backlight",
		.gpio = EXYNOS4_GPL2(4),
		.default_state = LEDS_GPIO_DEFSTATE_ON,
	},
	{
		.name = "BLUE_LED",
		.gpio = EXYNOS4_GPL2(3),
		.default_state = LEDS_GPIO_DEFSTATE_ON,
	},
	{
		.name = "YELLOW_LED",
		.gpio = EXYNOS4_GPF0(0),
		.default_state = LEDS_GPIO_DEFSTATE_ON,
	},
	{
		.name = "RED_LED",
		//.gpio = EXYNOS4_GPF0(0),
		.default_state = LEDS_GPIO_DEFSTATE_ON,
	}	
};

static struct gpio_led_platform_data smdk4x12_kp_led = {
	.num_leds = ARRAY_SIZE(smdk4x12_kp_leds),
	.leds = smdk4x12_kp_leds,
};

static struct platform_device smdk4x12_kp_led_dev = {
	.name          = "kp_leds-gpio",
	.id            = -1,
	.dev = {
		.platform_data = &smdk4x12_kp_led,
	},
};
#endif
// add end
static void __init smdk4x12_map_io(void)
{
	clk_xusbxti.rate = 24000000;
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(smdk4x12_uartcfgs, ARRAY_SIZE(smdk4x12_uartcfgs));

#if defined(CONFIG_S5P_MEM_CMA)
	exynos4_reserve_mem();
#endif
}

static void __init smdk4x12_smsc911x_init(void)
{
	u32 cs1;

	/* configure nCS1 width to 16 bits */
	cs1 = __raw_readl(S5P_SROM_BW) &
		~(S5P_SROM_BW__CS_MASK << S5P_SROM_BW__NCS1__SHIFT);
	cs1 |= ((1 << S5P_SROM_BW__DATAWIDTH__SHIFT) |
		(1 << S5P_SROM_BW__WAITENABLE__SHIFT) |
		(1 << S5P_SROM_BW__BYTEENABLE__SHIFT)) <<
		S5P_SROM_BW__NCS1__SHIFT;
	__raw_writel(cs1, S5P_SROM_BW);

	/* set timing for nCS1 suitable for ethernet chip */
	__raw_writel((0x1 << S5P_SROM_BCX__PMC__SHIFT) |
		     (0x9 << S5P_SROM_BCX__TACP__SHIFT) |
		     (0xc << S5P_SROM_BCX__TCAH__SHIFT) |
		     (0x1 << S5P_SROM_BCX__TCOH__SHIFT) |
		     (0x6 << S5P_SROM_BCX__TACC__SHIFT) |
		     (0x1 << S5P_SROM_BCX__TCOS__SHIFT) |
		     (0x1 << S5P_SROM_BCX__TACS__SHIFT), S5P_SROM_BC1);
}

#ifndef CONFIG_TC4_GB
static void __init exynos_sysmmu_init(void)
{
	ASSIGN_SYSMMU_POWERDOMAIN(fimc0, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(fimc1, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(fimc2, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(fimc3, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(jpeg, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(mfc_l, &exynos4_device_pd[PD_MFC].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(mfc_r, &exynos4_device_pd[PD_MFC].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(tv, &exynos4_device_pd[PD_TV].dev);
#ifdef CONFIG_VIDEO_FIMG2D
	sysmmu_set_owner(&SYSMMU_PLATDEV(g2d_acp).dev, &s5p_device_fimg2d.dev);
#endif
#if defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC) || defined(CONFIG_VIDEO_MFC5X)
	sysmmu_set_owner(&SYSMMU_PLATDEV(mfc_l).dev, &s5p_device_mfc.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(mfc_r).dev, &s5p_device_mfc.dev);
#endif
#if defined(CONFIG_VIDEO_FIMC)
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc0).dev, &s3c_device_fimc0.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc1).dev, &s3c_device_fimc1.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc2).dev, &s3c_device_fimc2.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc3).dev, &s3c_device_fimc3.dev);
#elif defined(CONFIG_VIDEO_SAMSUNG_S5P_FIMC)
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc0).dev, &s5p_device_fimc0.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc1).dev, &s5p_device_fimc1.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc2).dev, &s5p_device_fimc2.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc3).dev, &s5p_device_fimc3.dev);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_TV
	sysmmu_set_owner(&SYSMMU_PLATDEV(tv).dev, &s5p_device_mixer.dev);
#endif
#ifdef CONFIG_VIDEO_TVOUT
	sysmmu_set_owner(&SYSMMU_PLATDEV(tv).dev, &s5p_device_tvout.dev);
#endif
#ifdef CONFIG_VIDEO_JPEG_V2X
	sysmmu_set_owner(&SYSMMU_PLATDEV(jpeg).dev, &s5p_device_jpeg.dev);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
	ASSIGN_SYSMMU_POWERDOMAIN(is_isp, &exynos4_device_pd[PD_ISP].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(is_drc, &exynos4_device_pd[PD_ISP].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(is_fd, &exynos4_device_pd[PD_ISP].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(is_cpu, &exynos4_device_pd[PD_ISP].dev);

	sysmmu_set_owner(&SYSMMU_PLATDEV(is_isp).dev,
						&exynos4_device_fimc_is.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(is_drc).dev,
						&exynos4_device_fimc_is.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(is_fd).dev,
						&exynos4_device_fimc_is.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(is_cpu).dev,
						&exynos4_device_fimc_is.dev);
#endif
}

#endif
// zsb: wait for i2c5 bus idle before software reset
extern int wait_for_i2c_idle(struct platform_device *pdev);
#if 1 //def CONFIG_TC4_EVT //mj
extern int is_charger_online();
#endif
static void smdk4x12_power_off(void)
{
	int ret = 0;
	
//shengliang
	gpio_set_value(EXYNOS4_GPC0(0), 0);// MD_PWON low
	msleep(10);
	gpio_set_value(EXYNOS4_GPC0(2), 1);// MD_RSTN low
	//gpio_set_value(EXYNOS4_GPL2(1), 0);// MD_RESETBB low   mask by samsung 20121211
	msleep(10);
	#ifdef CONFIG_TC4_EVT
	if (is_charger_online())
	#else
	if(0) //Robin, For TC4 DVT, when usb plug-in, the pmic still can power off...
	#endif
	{    
		//Turn off some LDO&BUCKs, Will Implment later.
		//max8997_pmic_off();//TBD,Robin Wang
		ret = wait_for_i2c_idle(&s3c_device_i2c5);
		if (ret != 0) 
			printk(KERN_EMERG "%s : i2c5 bus is busy.\n", __func__);

		writel(1,S5P_SWRESET);
	}    
	else 
	{    
		writel(0,S5P_INFORM7);
		if (is_charger_online())//mj for charging while power off
		{
			printk(KERN_EMERG "%s : charging..\n", __func__);
			//writel(0xc2, S5P_INFORM5); //mj : power off-charging mode 
			writel(1,S5P_SWRESET);
		}
		else
		{
			/* PS_HOLD --> Output Low */
			printk(KERN_EMERG "%s : setting GPIO_PDA_PS_HOLD low.\n", __func__);
			/* PS_HOLD output High --> Low  PS_HOLD_CONTROL, R/W, 0xE010_E81C */
			writel(0x5200, S5P_PS_HOLD_CONTROL);
		}
	}    
	while(1);

	printk(KERN_EMERG "%s : should not reach here!\n", __func__);
}

#if 1
extern void (*s3c_config_sleep_gpio_table)(void);
#include <plat/gpio-core.h>

int s3c_gpio_slp_cfgpin(unsigned int pin, unsigned int config)
{
	struct s3c_gpio_chip *chip = s3c_gpiolib_getchip(pin);
	void __iomem *reg;
	unsigned long flags;
	int offset;
	u32 con;
	int shift;

	if (!chip)
		return -EINVAL;

	if ((pin >= EXYNOS4_GPX0(0)) && (pin <= EXYNOS4_GPX3(7)))
		return -EINVAL;

	if (config > S3C_GPIO_SLP_PREV)
		return -EINVAL;

	reg = chip->base + 0x10;

	offset = pin - chip->chip.base;
	shift = offset * 2;

	local_irq_save(flags);

	con = __raw_readl(reg);
	con &= ~(3 << shift);
	con |= config << shift;
	__raw_writel(con, reg);

	local_irq_restore(flags);
	return 0;
}

int s3c_gpio_slp_setpull_updown(unsigned int pin, unsigned int config)
{
	struct s3c_gpio_chip *chip = s3c_gpiolib_getchip(pin);
	void __iomem *reg;
	unsigned long flags;
	int offset;
	u32 con;
	int shift;

	if (!chip)
		return -EINVAL;

	if ((pin >= EXYNOS4_GPX0(0)) && (pin <= EXYNOS4_GPX3(7)))
		return -EINVAL;

	if (config > S3C_GPIO_PULL_UP)
		return -EINVAL;

	reg = chip->base + 0x14;

	offset = pin - chip->chip.base;
	shift = offset * 2;

	local_irq_save(flags);

	con = __raw_readl(reg);
	con &= ~(3 << shift);
	con |= config << shift;
	__raw_writel(con, reg);

	local_irq_restore(flags);

	return 0;
}
//Cellon add start, Jacob, 2012/08/07, for GPIO init setting issue
struct gpio_init_data {
	uint num;
	uint cfg;
	uint val;
	uint pud;
	uint drv;
};

static struct gpio_init_data tc4_init_gpio[] = {
	//GPA
	{EXYNOS4_GPA0(0),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_UP},					//UART_BT_RX
	{EXYNOS4_GPA0(1),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//UART_BT_TX
	{EXYNOS4_GPA0(2),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//UART_BT_RTS
	{EXYNOS4_GPA0(3),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//UART_BT_CTS
	{EXYNOS4_GPA0(4),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_UP},					//XuRXD1
	{EXYNOS4_GPA0(5),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XuTXD1
	{EXYNOS4_GPA0(6),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XuCTS1
	{EXYNOS4_GPA0(7),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XuRTS1
	{EXYNOS4_GPA1(0),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_UP},					//XuRXD2
	{EXYNOS4_GPA1(1),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XuTXD2
	{EXYNOS4_GPA1(2),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XuCTS2
	{EXYNOS4_GPA1(3),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XuRTS2
	{EXYNOS4_GPA1(4),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_UP},					//XuRXD3
	{EXYNOS4_GPA1(5),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XuTXD3
/*
	//GPB
	{EXYNOS4_GPB(0),S3C_GPIO_SFN(3),2, S3C_GPIO_PULL_NONE},						//XI2C_4_SDA
	{EXYNOS4_GPB(1),S3C_GPIO_SFN(3),2, S3C_GPIO_PULL_NONE},						//XI2C_4_SCL
	{EXYNOS4_GPB(2),S3C_GPIO_SFN(3),2, S3C_GPIO_PULL_NONE},						//XI2C_5_SDA
	{EXYNOS4_GPB(3),S3C_GPIO_SFN(3),2, S3C_GPIO_PULL_NONE},						//XI2C_5_SCL
	{EXYNOS4_GPB(4),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},						//GPS_RST
	{EXYNOS4_GPB(5),S3C_GPIO_OUTPUT,1, S3C_GPIO_PULL_NONE},						//PMIC_SET1
	{EXYNOS4_GPB(6),S3C_GPIO_OUTPUT,1, S3C_GPIO_PULL_NONE},						//PMIC_SET2
	{EXYNOS4_GPB(7),S3C_GPIO_OUTPUT,1, S3C_GPIO_PULL_NONE},						//PMIC_SET3

	//GPC
	{EXYNOS4_GPC0(0),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_UP},					//MD_PWON
	{EXYNOS4_GPC0(1),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_DOWN},					//LCD_BL_EN
	{EXYNOS4_GPC0(2),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_DOWN},					//MD_RSTN
	{EXYNOS4_GPC0(3),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},					//AP_SLEEP
	{EXYNOS4_GPC0(4),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},					//AP_WAKEUP_MD
	{EXYNOS4_GPC1(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//
	{EXYNOS4_GPC1(1),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},					//AP_WAKEUP_BT
	{EXYNOS4_GPC1(2),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},					//GPS_EN
	{EXYNOS4_GPC1(3),S3C_GPIO_SFN(4),2, S3C_GPIO_PULL_NONE},					//XI2C_6_SDA
	{EXYNOS4_GPC1(4),S3C_GPIO_SFN(4),2, S3C_GPIO_PULL_NONE},					//XI2C_6_SCL

	//GPD
	{EXYNOS4_GPD0(0),S3C_GPIO_INPUT,2, S3C_GPIO_PULL_NONE},	//NC
	{EXYNOS4_GPD0(1),S3C_GPIO_INPUT,2, S3C_GPIO_PULL_NONE},	//NC
	{EXYNOS4_GPD0(2),S3C_GPIO_SFN(3),2, S3C_GPIO_PULL_NONE},					//XI2C_7_SDA
	{EXYNOS4_GPD0(3),S3C_GPIO_SFN(3),2, S3C_GPIO_PULL_NONE},					//XI2C_7_SCL
	{EXYNOS4_GPD1(0),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XI2C_0_SDA_MHL_D
	{EXYNOS4_GPD1(1),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XI2C_0_SCL_MHL_D
	{EXYNOS4_GPD1(2),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XI2C_1_SDA
	{EXYNOS4_GPD1(3),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XI2C_1_SCL

	//GPF
	{EXYNOS4_GPF0(0),S3C_GPIO_OUTPUT,0,S3C_GPIO_PULL_DOWN},					//YELLOW_LED
	{EXYNOS4_GPF0(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF0(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF0(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF0(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF0(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF0(6),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF0(7),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF1(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF1(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF1(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF1(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF1(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF1(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF1(6),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF1(7),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF2(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF2(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF2(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF2(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF2(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF2(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF2(6),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF2(7),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF3(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF3(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF3(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF3(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF3(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPF3(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC

	//GPJ
	{EXYNOS4212_GPJ0(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPJ0(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPJ0(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPJ0(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPJ0(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPJ0(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPJ0(6),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPJ0(7),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPJ1(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPJ1(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPJ1(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPJ1(3),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					 	//CAM_MCLK
	{EXYNOS4212_GPJ1(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPK0(0),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					 	//XMMC_0_CLK
	{EXYNOS4_GPK0(1),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					 	//XMMC_0_CMD
	{EXYNOS4_GPK0(2),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					 	//XMMC_0_CDN
	{EXYNOS4_GPK0(3),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					 	//XMMC_0_DATA0
	{EXYNOS4_GPK0(4),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					 	//XMMC_0_DATA1
	{EXYNOS4_GPK0(5),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					 	//XMMC_0_DATA2
	{EXYNOS4_GPK0(6),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					 	//XMMC_0_DATA3
	{EXYNOS4_GPK1(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	 	//NC
	{EXYNOS4_GPK1(1),S3C_GPIO_OUTPUT,1,S3C_GPIO_PULL_DOWN},					 	//TORCH_EN
	{EXYNOS4_GPK1(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},		 //NC
	{EXYNOS4_GPK1(3),S3C_GPIO_SFN(3),2, S3C_GPIO_PULL_NONE},					 	//XMMC_0_DATA4

	//GPK
	{EXYNOS4_GPK1(4),S3C_GPIO_SFN(3),2, S3C_GPIO_PULL_NONE},					//XMMC_0_DATA5
	{EXYNOS4_GPK1(5),S3C_GPIO_SFN(3),2, S3C_GPIO_PULL_NONE},					//XMMC_0_DATA6
	{EXYNOS4_GPK1(6),S3C_GPIO_SFN(3),2, S3C_GPIO_PULL_NONE},					//XMMC_0_DATA7
	{EXYNOS4_GPK2(0),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XMMC_2_CLK
	{EXYNOS4_GPK2(1),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XMMC_2_CMD
	{EXYNOS4_GPK2(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPK2(3),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XMMC_2_DATA0
	{EXYNOS4_GPK2(4),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XMMC_2_DATA1
	{EXYNOS4_GPK2(5),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XMMC_2_DATA2
	{EXYNOS4_GPK2(6),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XMMC_2_DATA3
	{EXYNOS4_GPK3(0),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//Xmmc3CLK
	{EXYNOS4_GPK3(1),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//Xmmc3CMD
	{EXYNOS4_GPK3(2),S3C_GPIO_SFN(2),2,S3C_GPIO_PULL_DOWN},					//USB_SEL
	{EXYNOS4_GPK3(3),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//Xmmc3DATA0
	{EXYNOS4_GPK3(4),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//Xmmc3DATA1
	{EXYNOS4_GPK3(5),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//Xmmc3DATA2
	{EXYNOS4_GPK3(6),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//Xmmc3DATA3

	//GPL
	{EXYNOS4_GPL0(0),S3C_GPIO_OUTPUT,1, S3C_GPIO_PULL_NONE},					//BUCK6_EN
	{EXYNOS4_GPL0(1),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},					//ISP_RST_N
	{EXYNOS4_GPL0(2),S3C_GPIO_INPUT,2, S3C_GPIO_PULL_NONE},						//CAM1_ID
	{EXYNOS4_GPL0(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPL0(4),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},					//DYRO_EN
	{EXYNOS4_GPL0(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPL0(6),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},					//COMPASS_RSTN
	{EXYNOS4_GPL1(0),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//AAT3635BL_EN
	{EXYNOS4_GPL1(1),S3C_GPIO_INPUT,2, S3C_GPIO_PULL_NONE},						//CAM2_ID
	{EXYNOS4_GPL2(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},				//NC
	{EXYNOS4_GPL2(1),S3C_GPIO_OUTPUT,1, S3C_GPIO_PULL_UP},					//MD_RESETBB
	{EXYNOS4_GPL2(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},				//NC
	{EXYNOS4_GPL2(3),S3C_GPIO_OUTPUT,0,S3C_GPIO_PULL_DOWN},					//BLUE_LED
	{EXYNOS4_GPL2(4),S3C_GPIO_OUTPUT,0,S3C_GPIO_PULL_DOWN},					//KYPD_DRV_N
	{EXYNOS4_GPL2(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},				//NC
	{EXYNOS4_GPL2(6),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},				//NC
	{EXYNOS4_GPL2(7),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},				//NC

	//GPY
	{EXYNOS4_GPY0(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY0(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY0(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY0(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY0(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY0(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY1(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY1(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY1(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY1(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY2(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY2(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY2(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY2(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY2(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY2(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY3(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY3(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY3(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY3(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY3(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY3(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY3(6),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY3(7),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY4(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY4(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY4(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY4(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY4(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY4(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY4(6),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY4(7),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY5(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY5(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY5(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY5(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY5(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY5(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY5(6),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY5(7),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY6(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY6(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY6(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY6(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY6(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY6(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY6(6),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4_GPY6(7),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC

	//GPM
	{EXYNOS4212_GPM0(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM0(1),S3C_GPIO_OUTPUT,0,S3C_GPIO_PULL_DOWN},						//VIB_ON
	{EXYNOS4212_GPM0(2),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},						//SEN2_RST
	{EXYNOS4212_GPM0(3),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},						//SEN2_PWDN
	{EXYNOS4212_GPM0(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM0(5),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},						//ES305_RST
	{EXYNOS4212_GPM0(6),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM0(7),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM1(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM1(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM1(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM1(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM1(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM1(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM1(6),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM2(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM2(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM2(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM2(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM2(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM3(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM3(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM3(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM3(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM3(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM3(5),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},						//PMIC_DS2
	{EXYNOS4212_GPM3(6),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},						//PMIC_DS3
	{EXYNOS4212_GPM3(7),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},						//PMIC_DS4
  	{EXYNOS4212_GPM4(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM4(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM4(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM4(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM4(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM4(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM4(6),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPM4(7),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC

	//GPX
	{EXYNOS4_GPX0(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},		//NC
	{EXYNOS4_GPX0(1),S3C_GPIO_SFN(0xF),2, S3C_GPIO_PULL_NONE},				//TS_PENIRQ_N
	{EXYNOS4_GPX0(2),S3C_GPIO_SFN(0xF),2, S3C_GPIO_PULL_NONE},				//ONO
	{EXYNOS4_GPX0(3),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},				//TP_RST
	{EXYNOS4_GPX0(4),S3C_GPIO_INPUT,2, S3C_GPIO_PULL_NONE},					//GPS_INT
	{EXYNOS4_GPX0(5),S3C_GPIO_SFN(0xF),2, S3C_GPIO_PULL_NONE},				//COMPASS_INT
	{EXYNOS4_GPX0(6),S3C_GPIO_INPUT,2, S3C_GPIO_PULL_NONE},					//CTP_ID
	{EXYNOS4_GPX0(7),S3C_GPIO_SFN(0xF),2, S3C_GPIO_PULL_NONE},				//SD_CARD_DET_N
	{EXYNOS4_GPX1(0),S3C_GPIO_SFN(3),2, S3C_GPIO_PULL_UP},				//KP_COL0
	{EXYNOS4_GPX1(1),S3C_GPIO_SFN(3),2, S3C_GPIO_PULL_UP},				//KP_COL1
	{EXYNOS4_GPX1(2),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},				//WLAN_INT
	{EXYNOS4_GPX1(3),S3C_GPIO_SFN(0xF),2, S3C_GPIO_PULL_NONE},				//NFC_IRQ
	{EXYNOS4_GPX1(4),S3C_GPIO_SFN(0xF),2, S3C_GPIO_PULL_NONE},				//3635_INT
	{EXYNOS4_GPX1(5),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},				//BT_WAKEUP_AP
	{EXYNOS4_GPX1(6),S3C_GPIO_SFN(0xF),2, S3C_GPIO_PULL_NONE},				//MD_SLEEP_REQUEST
	{EXYNOS4_GPX1(7),S3C_GPIO_SFN(0xF),2, S3C_GPIO_PULL_NONE},				//PMU_IRQ
	{EXYNOS4_GPX2(0),S3C_GPIO_SFN(3),2, S3C_GPIO_PULL_NONE},				//KP_ROW0
	{EXYNOS4_GPX2(1),S3C_GPIO_SFN(3),2, S3C_GPIO_PULL_NONE},				//KP_ROW1
	{EXYNOS4_GPX2(2),S3C_GPIO_SFN(0xF),2, S3C_GPIO_PULL_NONE},				//HS_PLUG_DET
	{EXYNOS4_GPX2(3),S3C_GPIO_SFN(0xF),2, S3C_GPIO_PULL_NONE},				//GYRO_INT_N
	{EXYNOS4_GPX2(4),S3C_GPIO_SFN(0xF),2, S3C_GPIO_PULL_NONE},				//PROXIMITY_INT_N
	{EXYNOS4_GPX2(5),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},				//MD_WAKEUP_AP
	{EXYNOS4_GPX2(6),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},				//NFC_FW_DL
	{EXYNOS4_GPX2(7),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},			//NC
	{EXYNOS4_GPX3(0),S3C_GPIO_SFN(0xF),2, S3C_GPIO_PULL_NONE},				//ACCEL_INT1
	{EXYNOS4_GPX3(1),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},				//MHL_RESET_N
	{EXYNOS4_GPX3(2),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},				//MIPI_DSI_RST_N
	{EXYNOS4_GPX3(3),S3C_GPIO_SFN(0xF),2, S3C_GPIO_PULL_NONE},				//ISP_INT
	{EXYNOS4_GPX3(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},			//NC
	{EXYNOS4_GPX3(5),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},				//NFC_EXT_EN
	{EXYNOS4_GPX3(6),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},				//MHL_INT
	{EXYNOS4_GPX3(7),S3C_GPIO_SFN(0xF),2, S3C_GPIO_PULL_NONE},				//HDMI_HOTPLUG_DET

	//GPZ
	{EXYNOS4_GPZ(0),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XI2S0SCLK
	{EXYNOS4_GPZ(1),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XI2S0CDCLK
	{EXYNOS4_GPZ(2),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XI2S0LRCK
	{EXYNOS4_GPZ(3),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XI2S0SDI
	{EXYNOS4_GPZ(4),S3C_GPIO_SFN(2),2, S3C_GPIO_PULL_NONE},					//XI2S0SDO
	{EXYNOS4_GPZ(5),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},					//WLAN_PWDN
	{EXYNOS4_GPZ(6),S3C_GPIO_OUTPUT,0, S3C_GPIO_PULL_NONE},					//BT_RESET

	//GPV
	{EXYNOS4212_GPV0(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV0(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV0(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV0(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV0(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV0(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV0(6),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV0(7),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV1(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV1(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV1(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV1(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV1(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV1(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV1(6),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV1(7),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV2(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV2(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV2(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV2(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV2(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV2(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV2(6),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV2(7),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV3(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV3(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV3(2),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV3(3),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV3(4),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV3(5),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV3(6),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV3(7),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV4(0),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	{EXYNOS4212_GPV4(1),S3C_GPIO_INPUT,2,S3C_GPIO_PULL_DOWN},	//NC
	*/
};

void tc4_config_init_gpio(void)
{
	u32 i, gpio;

	for (i = 0; i < ARRAY_SIZE(tc4_init_gpio); i++) {
		gpio = tc4_init_gpio[i].num;
		s3c_gpio_cfgpin(gpio, tc4_init_gpio[i].cfg);
		s3c_gpio_setpull(gpio, tc4_init_gpio[i].pud);

		if (tc4_init_gpio[i].val != S3C_GPIO_SETPIN_NONE)
			gpio_set_value(gpio, tc4_init_gpio[i].val);
//			s5p_gpio_set_drvstr(gpio, tc4_init_gpio[i].drv);
	}
}
//Cellon add end, Jacob, 2012/08/07

static void config_sleep_gpio_table(int array_size, unsigned int (*gpio_table)[3])
{
        u32 i, gpio;

        for (i = 0; i < array_size; i++) {
                gpio = gpio_table[i][0];
                s3c_gpio_slp_cfgpin(gpio, gpio_table[i][1]);
                s3c_gpio_slp_setpull_updown(gpio, gpio_table[i][2]);
        }
}

/*sleep gpio table for TC4*/
static unsigned int tc4_sleep_gpio_table[][3] = {
#if 1 //zhangdong reduce sleep current
	{ EXYNOS4_GPA0(0),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE}, //BT_TXD
	{ EXYNOS4_GPA0(1),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE}, //BT_RXD
	{ EXYNOS4_GPA0(2),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE}, //BT_RTS
	{ EXYNOS4_GPA0(3),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE}, //BT_CTS
	
	{ EXYNOS4_GPA0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //AC100_TXD,SMM6260
	{ EXYNOS4_GPA0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //AC100_RXD
	{ EXYNOS4_GPA0(6),	S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, //AC100_RTS
	{ EXYNOS4_GPA0(7),	S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, //AC100_CTS


	{ EXYNOS4_GPA1(0),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE}, //DEBUG
	{ EXYNOS4_GPA1(1),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE}, //DEBUG
	{ EXYNOS4_GPA1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},  //I2C_SDA3
	{ EXYNOS4_GPA1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C_SCL3
#ifdef CONFIG_TC4_EVT	
	{ EXYNOS4_GPA1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //TP1_RST
	{ EXYNOS4_GPA1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //TestPoint
#endif
#ifdef CONFIG_TC4_DVT
	{ EXYNOS4_GPA1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //GPS_TXD
	{ EXYNOS4_GPA1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //GPS_RXD
#endif

	{ EXYNOS4_GPB(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C_SDA4
	{ EXYNOS4_GPB(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C_SCL4
	{ EXYNOS4_GPB(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C_SDA5
	{ EXYNOS4_GPB(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C_SCL5
	{ EXYNOS4_GPB(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //GPS_RST
	{ EXYNOS4_GPB(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //PMIC_SET1
	{ EXYNOS4_GPB(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //PMIC_SET2
	{ EXYNOS4_GPB(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //PMIC_SET3


#ifdef CONFIG_SMM6260_MODEM
	{ EXYNOS4_GPC0(0),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE}, //MD_PWON
#else
	{ EXYNOS4_GPC0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //MD_PWON
#endif
	{ EXYNOS4_GPC0(1),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE}, //VLED_ON	//modified by terry for wm8994 power enable pin in sleep mode
	{ EXYNOS4_GPC0(2),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE}, //MD_RSTN
#ifdef CONFIG_SMM6260_MODEM
	{ EXYNOS4_GPC0(3),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE}, //AP_SLEEP
	{ EXYNOS4_GPC0(4),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE}, //AP_WAKEUP_MD
	{ EXYNOS4_GPX2(5),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE}, //MD_WAKEUP_AP
#else
	{ EXYNOS4_GPC0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //AP_SLEEP
	{ EXYNOS4_GPC0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //AP_WAKEUP_MD
#endif
	
	{ EXYNOS4_GPC1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //UART_SW  config as hp  out1??
	{ EXYNOS4_GPC1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //LED_EN18
	{ EXYNOS4_GPC1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //VLED_EN
	{ EXYNOS4_GPC1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C6_SDA
	{ EXYNOS4_GPC1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C6_SCL

	{ EXYNOS4_GPD0(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, // MOTOR-PWM
	{ EXYNOS4_GPD0(1),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, //XPWMOUT1
	{ EXYNOS4_GPD0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},  //I2C7_SDA
	{ EXYNOS4_GPD0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C7_SCL

	{ EXYNOS4_GPD1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C0_SDA
	{ EXYNOS4_GPD1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C0_SCL
	{ EXYNOS4_GPD1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C1_SDA
	{ EXYNOS4_GPD1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C1_SCL

	{ EXYNOS4_GPF0(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//LCD_HSYNC
	{ EXYNOS4_GPF0(1),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//LCD_VSYNC
	{ EXYNOS4_GPF0(2),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//LCD_VDEN
	{ EXYNOS4_GPF0(3),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//LCD_VCLK
	{ EXYNOS4_GPF0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//CAM2M_RST
	{ EXYNOS4_GPF0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//CAM2M_PWDN
	{ EXYNOS4_GPF0(6),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//LCD_D2
	{ EXYNOS4_GPF0(7),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//LCD_D3

	{ EXYNOS4_GPF1(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//LCD_D4
	{ EXYNOS4_GPF1(1),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//LCD_D5
	{ EXYNOS4_GPF1(2),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D6
	{ EXYNOS4_GPF1(3),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D7
	{ EXYNOS4_GPF1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//CAM5M_RST
	{ EXYNOS4_GPF1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//CAM5M_PWDN
	{ EXYNOS4_GPF1(6),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D10
	{ EXYNOS4_GPF1(7),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D11

	{ EXYNOS4_GPF2(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D12
	{ EXYNOS4_GPF2(1),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D13
	{ EXYNOS4_GPF2(2),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D14
	{ EXYNOS4_GPF2(3),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D15
	{ EXYNOS4_GPF2(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	{ EXYNOS4_GPF2(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	{ EXYNOS4_GPF2(6),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D18
	{ EXYNOS4_GPF2(7),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D19
	
	{ EXYNOS4_GPF3(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D20
	{ EXYNOS4_GPF3(1),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D21
	{ EXYNOS4_GPF3(2),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D22
	{ EXYNOS4_GPF3(3),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D23
	{ EXYNOS4_GPF3(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	{ EXYNOS4_GPF3(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//MD_G15

	{ EXYNOS4212_GPJ0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//CAM_CLK
	{ EXYNOS4212_GPJ0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//CAM_VSYNC
	{ EXYNOS4212_GPJ0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//CAM_HREF
	{ EXYNOS4212_GPJ0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//DATA0
	{ EXYNOS4212_GPJ0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//DATA1
	{ EXYNOS4212_GPJ0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//DATA2
	{ EXYNOS4212_GPJ0(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//DATA3
	{ EXYNOS4212_GPJ0(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//DATA4

	{ EXYNOS4212_GPJ1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//DATA5
	{ EXYNOS4212_GPJ1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//DATA6
	{ EXYNOS4212_GPJ1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//DATA7
	{ EXYNOS4212_GPJ1(3),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//CAM_CLK_OUT
	{ EXYNOS4212_GPJ1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC

	{ EXYNOS4_GPK0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_CLK
	{ EXYNOS4_GPK0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_CMD
	{ EXYNOS4_GPK0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_CDn
	{ EXYNOS4_GPK0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_DATA0
	{ EXYNOS4_GPK0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_DATA1
	{ EXYNOS4_GPK0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_DATA2
	{ EXYNOS4_GPK0(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_DATA3
	#ifdef CONFIG_TC4_EVT
	{ EXYNOS4_GPK1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	{ EXYNOS4_GPK1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	#endif
	#ifdef CONFIG_TC4_DVT
	{ EXYNOS4_GPK1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//ANX7805_PD
	{ EXYNOS4_GPK1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//VDD50_EN
	#endif
	
	{ EXYNOS4_GPK1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	{ EXYNOS4_GPK1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_DATA4
	{ EXYNOS4_GPK1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_DATA5
	{ EXYNOS4_GPK1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_DATA6
	{ EXYNOS4_GPK1(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_DATA7

	{ EXYNOS4_GPK2(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//TF_CLK
	{ EXYNOS4_GPK2(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//TF_CMD
	{ EXYNOS4_GPK2(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//ANX7805_RSTN
	{ EXYNOS4_GPK2(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//TF_DATA0
	{ EXYNOS4_GPK2(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//TF_DATA1
	{ EXYNOS4_GPK2(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//TF_DATA2
	{ EXYNOS4_GPK2(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//TF_DATA3

	{ EXYNOS4_GPK3(0),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},	//WIFI_CLK
	{ EXYNOS4_GPK3(1),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},	//WIFI_CMD
// Cellon modify start, Ted Shi, 2012/12/13, for optimize usb switch func
//	{ EXYNOS4_GPK3(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//HUB_CONNECT
	{ EXYNOS4_GPK3(2),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},	//HUB_CONNECT
// Cellon modify end, Ted Shi, 2012/12/13
	{ EXYNOS4_GPK3(3),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},	//WIFI_DATA0
	{ EXYNOS4_GPK3(4),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},	//WIFI_DATA1
	{ EXYNOS4_GPK3(5),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},	//WIFI_DATA2
	{ EXYNOS4_GPK3(6),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},	//WIFI_DATA3

	{ EXYNOS4_GPL0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//BUCK6_EN
	#ifdef CONFIG_TC4_EVT
	{ EXYNOS4_GPL0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//VDD50_EN
	#endif
	#ifdef CONFIG_TC4_DVT
	{ EXYNOS4_GPL0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//6260_GPIO3
	#endif
	
	{ EXYNOS4_GPL0(2), /* S3C_GPIO_SLP_PREV*/S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//TP1_EN
	{ EXYNOS4_GPL0(3),  S3C_GPIO_SLP_OUT1,	S3C_GPIO_PULL_NONE},	//NFC_EN1  out1
	
	{ EXYNOS4_GPL0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//CHG_EN
	{ EXYNOS4_GPL0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NO THIS PIN
	{ EXYNOS4_GPL0(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//HDMI_IIC_EN
	{ EXYNOS4_GPL0(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC

	{ EXYNOS4_GPL1(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 	//LVDS_PWDN  out0
	{ EXYNOS4_GPL1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	{ EXYNOS4_GPL1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC

	{ EXYNOS4_GPL2(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//KP_COL0
#ifdef CONFIG_SMM6260_MODEM
	{ EXYNOS4_GPL2(1),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},	//MD_RESETBB
#else
	{ EXYNOS4_GPL2(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//MD_RESETBB
#endif
	{ EXYNOS4_GPL2(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//HUB_RESET
// Cellon modify start, Ted Shi, 2012/12/11, for blue led when sleep
//	{ EXYNOS4_GPL2(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NFC_SCL
	{ EXYNOS4_GPL2(3),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_DOWN},	//BLUE_LED
// Cellon modify end, Ted Shi, 2012/12/11
	{ EXYNOS4_GPL2(4),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_DOWN},	//NFC_SDA
	{ EXYNOS4_GPL2(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NFC_GPIO4
	#ifdef CONFIG_TC4_EVT
	{ EXYNOS4_GPL2(6),	S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//ANX7805_PWON
	{ EXYNOS4_GPL2(7),	S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//TP
	#endif
 	{ EXYNOS4212_GPM0(5),  S3C_GPIO_SLP_PREV,  S3C_GPIO_PULL_NONE},	//add by terry huang for es305 reset pin

	{ EXYNOS4212_GPM4(0),  S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_NONE},	//XISP_I2C_0_SCL sleep mode
	{ EXYNOS4212_GPM4(1),  S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_NONE},	//XISP_I2C_0_SDA sleep mode

	
	#ifdef CONFIG_TC4_DVT
	//GPM4(2) --ISP_SCL1
	//GPM4(3)--ISP_SDA1
	//GPM3(5)--PMIC_DS2
	//GPM3(6)--PMIC_DS3
	//GPM3(7)--PMIC_DS4
	#endif
	

#if 1
	{ EXYNOS4_GPY0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	{ EXYNOS4_GPY0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC

	{ EXYNOS4_GPY0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */

	{ EXYNOS4_GPY1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */

	{ EXYNOS4_GPY2(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY2(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY2(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY2(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY2(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY2(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */

	{ EXYNOS4_GPY3(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
       { EXYNOS4_GPY3(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	{ EXYNOS4_GPY3(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* MHL_SCL_1.8V */
	{ EXYNOS4_GPY3(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY3(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY3(5),  S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY3(6),  S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY3(7),  S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},

	{ EXYNOS4_GPY4(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY4(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY4(2),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY4(3),  S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY4(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY4(5),  S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY4(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY4(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ EXYNOS4_GPY5(0),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY5(1),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY5(2),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY5(3),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY5(4),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY5(5),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY5(6),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY5(7),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */

	{ EXYNOS4_GPY6(0),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY6(1),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY6(2),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY6(3),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY6(4),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY6(5),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY6(6),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY6(7),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
#endif 
	{ EXYNOS4_GPZ(0),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	//I2S0_SCLK
	{ EXYNOS4_GPZ(1),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	//I2S0_CDCLK
	{ EXYNOS4_GPZ(2),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, //I2S0_LRCK
	{ EXYNOS4_GPZ(3),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, //I2S0_SDI
	{ EXYNOS4_GPZ(4),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, //I2S0_SDO0
// Cellon modify start, Ted Shi, 2012/10/17, for wifi can not work after system sleep
//	{ EXYNOS4_GPZ(5),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, //WIFI_PWDN
	{ EXYNOS4_GPZ(5),  S3C_GPIO_SLP_PREV, 	S3C_GPIO_PULL_NONE}, //WIFI_PWDN
// Cellon modify end, Ted Shi, 2012/10/17
	{ EXYNOS4_GPZ(6),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},	//BT_RST

#else
	{ EXYNOS4_GPA0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //BT_TXD
	{ EXYNOS4_GPA0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //BT_RXD
	{ EXYNOS4_GPA0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //BT_RTS
	{ EXYNOS4_GPA0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //BT_CTS
	
	{ EXYNOS4_GPA0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //AC100_TXD,SMM6260
	{ EXYNOS4_GPA0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //AC100_RXD
	{ EXYNOS4_GPA0(6),	S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, //AC100_RTS
	{ EXYNOS4_GPA0(7),	S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, //AC100_CTS


	{ EXYNOS4_GPA1(0),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE}, //DEBUG
	{ EXYNOS4_GPA1(1),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE}, //DEBUG
	{ EXYNOS4_GPA1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},  //I2C_SDA3
	{ EXYNOS4_GPA1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C_SCL3
#ifdef CONFIG_TC4_EVT	
	{ EXYNOS4_GPA1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //TP1_RST
	{ EXYNOS4_GPA1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //TestPoint
#endif
#ifdef CONFIG_TC4_DVT
	{ EXYNOS4_GPA1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //GPS_TXD
	{ EXYNOS4_GPA1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //GPS_RXD
#endif

	{ EXYNOS4_GPB(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C_SDA4
	{ EXYNOS4_GPB(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C_SCL4
	{ EXYNOS4_GPB(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C_SDA5
	{ EXYNOS4_GPB(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C_SCL5
	{ EXYNOS4_GPB(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //GPS_RST
	{ EXYNOS4_GPB(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //PMIC_SET1
	{ EXYNOS4_GPB(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //PMIC_SET2
	{ EXYNOS4_GPB(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //PMIC_SET3


	{ EXYNOS4_GPC0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //MD_PWON
	{ EXYNOS4_GPC0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //VLED_ON
#ifdef CONFIG_SMM6260_MODEM
	{ EXYNOS4_GPC0(2),  S3C_GPIO_SLP_PREV,  S3C_GPIO_PULL_NONE}, //MD_RSTN  //lisw_2012.029 for Modem do not go into L3
#else
	{ EXYNOS4_GPC0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //MD_RSTN
#endif
	{ EXYNOS4_GPC0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //AP_SLEEP
	{ EXYNOS4_GPC0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //AP_WAKEUP_MD
	
	{ EXYNOS4_GPC1(0),  S3C_GPIO_SLP_PREV,  S3C_GPIO_PULL_NONE}, //UART_SW  config as hp  out1??
	{ EXYNOS4_GPC1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //LED_EN18
	{ EXYNOS4_GPC1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, //VLED_EN
	{ EXYNOS4_GPC1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C6_SDA
	{ EXYNOS4_GPC1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C6_SCL

	{ EXYNOS4_GPD0(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, // MOTOR-PWM
	{ EXYNOS4_GPD0(1),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, //XPWMOUT1
	{ EXYNOS4_GPD0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},  //I2C7_SDA
	{ EXYNOS4_GPD0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C7_SCL

	{ EXYNOS4_GPD1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C0_SDA
	{ EXYNOS4_GPD1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C0_SCL
	{ EXYNOS4_GPD1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C1_SDA
	{ EXYNOS4_GPD1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, //I2C1_SCL

	{ EXYNOS4_GPF0(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//LCD_HSYNC
	{ EXYNOS4_GPF0(1),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//LCD_VSYNC
	{ EXYNOS4_GPF0(2),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//LCD_VDEN
	{ EXYNOS4_GPF0(3),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//LCD_VCLK
	{ EXYNOS4_GPF0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//CAM2M_RST
	{ EXYNOS4_GPF0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//CAM2M_PWDN
	{ EXYNOS4_GPF0(6),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//LCD_D2
	{ EXYNOS4_GPF0(7),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//LCD_D3

	{ EXYNOS4_GPF1(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//LCD_D4
	{ EXYNOS4_GPF1(1),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//LCD_D5
	{ EXYNOS4_GPF1(2),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D6
	{ EXYNOS4_GPF1(3),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D7
	{ EXYNOS4_GPF1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//CAM5M_RST
	{ EXYNOS4_GPF1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//CAM5M_PWDN
	{ EXYNOS4_GPF1(6),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D10
	{ EXYNOS4_GPF1(7),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D11

	{ EXYNOS4_GPF2(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D12
	{ EXYNOS4_GPF2(1),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D13
	{ EXYNOS4_GPF2(2),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D14
	{ EXYNOS4_GPF2(3),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D15
	{ EXYNOS4_GPF2(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	{ EXYNOS4_GPF2(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	{ EXYNOS4_GPF2(6),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D18
	{ EXYNOS4_GPF2(7),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D19
	
	{ EXYNOS4_GPF3(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D20
	{ EXYNOS4_GPF3(1),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D21
	{ EXYNOS4_GPF3(2),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D22
	{ EXYNOS4_GPF3(3),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//D23
	{ EXYNOS4_GPF3(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	{ EXYNOS4_GPF3(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//MD_G15

	{ EXYNOS4212_GPJ0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//CAM_CLK
	{ EXYNOS4212_GPJ0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//CAM_VSYNC
	{ EXYNOS4212_GPJ0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//CAM_HREF
	{ EXYNOS4212_GPJ0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//DATA0
	{ EXYNOS4212_GPJ0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//DATA1
	{ EXYNOS4212_GPJ0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//DATA2
	{ EXYNOS4212_GPJ0(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//DATA3
	{ EXYNOS4212_GPJ0(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//DATA4

	{ EXYNOS4212_GPJ1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//DATA5
	{ EXYNOS4212_GPJ1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//DATA6
	{ EXYNOS4212_GPJ1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//DATA7
	{ EXYNOS4212_GPJ1(3),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	//CAM_CLK_OUT
	{ EXYNOS4212_GPJ1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC

	{ EXYNOS4_GPK0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_CLK
	{ EXYNOS4_GPK0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_CMD
	{ EXYNOS4_GPK0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_CDn
	{ EXYNOS4_GPK0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_DATA0
	{ EXYNOS4_GPK0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_DATA1
	{ EXYNOS4_GPK0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_DATA2
	{ EXYNOS4_GPK0(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_DATA3
	#ifdef CONFIG_TC4_EVT
	{ EXYNOS4_GPK1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	{ EXYNOS4_GPK1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	#endif
	#ifdef CONFIG_TC4_DVT
	{ EXYNOS4_GPK1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//ANX7805_PD
	{ EXYNOS4_GPK1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//VDD50_EN
	#endif
	
	{ EXYNOS4_GPK1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	{ EXYNOS4_GPK1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_DATA4
	{ EXYNOS4_GPK1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_DATA5
	{ EXYNOS4_GPK1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_DATA6
	{ EXYNOS4_GPK1(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//eMMC_DATA7

	{ EXYNOS4_GPK2(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//TF_CLK
	{ EXYNOS4_GPK2(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//TF_CMD
	{ EXYNOS4_GPK2(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//ANX7805_RSTN
	{ EXYNOS4_GPK2(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//TF_DATA0
	{ EXYNOS4_GPK2(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//TF_DATA1
	{ EXYNOS4_GPK2(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//TF_DATA2
	{ EXYNOS4_GPK2(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//TF_DATA3

	{ EXYNOS4_GPK3(0),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},	//WIFI_CLK
	{ EXYNOS4_GPK3(1),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},	//WIFI_CMD
	{ EXYNOS4_GPK3(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//HUB_CONNECT
	{ EXYNOS4_GPK3(3),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},	//WIFI_DATA0
	{ EXYNOS4_GPK3(4),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},	//WIFI_DATA1
	{ EXYNOS4_GPK3(5),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},	//WIFI_DATA2
	{ EXYNOS4_GPK3(6),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},	//WIFI_DATA3

	{ EXYNOS4_GPL0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//BUCK6_EN
	#ifdef CONFIG_TC4_EVT
	{ EXYNOS4_GPL0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//VDD50_EN
	#endif
	#ifdef CONFIG_TC4_DVT
	{ EXYNOS4_GPL0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//6260_GPIO3
	#endif
	
	{ EXYNOS4_GPL0(2),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},	//TP1_EN
	{ EXYNOS4_GPL0(3),  S3C_GPIO_SLP_OUT1,	S3C_GPIO_PULL_NONE},	//NFC_EN1  out1
	
	{ EXYNOS4_GPL0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//CHG_EN
	{ EXYNOS4_GPL0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//NO THIS PIN
	{ EXYNOS4_GPL0(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//HDMI_IIC_EN
	{ EXYNOS4_GPL0(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//NC

	{ EXYNOS4_GPL1(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 	//LVDS_PWDN  out0
	{ EXYNOS4_GPL1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//NC
	{ EXYNOS4_GPL1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//NC

	{ EXYNOS4_GPL2(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//KP_COL0
	{ EXYNOS4_GPL2(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//MD_RESETBB
	{ EXYNOS4_GPL2(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//HUB_RESET
	{ EXYNOS4_GPL2(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//NFC_SCL
	{ EXYNOS4_GPL2(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	//NFC_SDA
	{ EXYNOS4_GPL2(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NFC_GPIO4
	#ifdef CONFIG_TC4_EVT
	{ EXYNOS4_GPL2(6),	S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//ANX7805_PWON
	{ EXYNOS4_GPL2(7),	S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//TP
	#endif

	#ifdef CONFIG_TC4_DVT
	//GPM4(2) --ISP_SCL1
	//GPM4(3)--ISP_SDA1
	//GPM3(5)--PMIC_DS2
	//GPM3(6)--PMIC_DS3
	//GPM3(7)--PMIC_DS4
	#endif
	

#if 1
	{ EXYNOS4_GPY0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	{ EXYNOS4_GPY0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC

	{ EXYNOS4_GPY0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */

	{ EXYNOS4_GPY1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */

	{ EXYNOS4_GPY2(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY2(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY2(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY2(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY2(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY2(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */

	{ EXYNOS4_GPY3(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
       { EXYNOS4_GPY3(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	//NC
	{ EXYNOS4_GPY3(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* MHL_SCL_1.8V */
	{ EXYNOS4_GPY3(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY3(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY3(5),  S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY3(6),  S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY3(7),  S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},

	{ EXYNOS4_GPY4(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY4(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY4(2),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY4(3),  S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY4(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY4(5),  S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY4(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS4_GPY4(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ EXYNOS4_GPY5(0),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY5(1),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY5(2),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY5(3),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY5(4),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY5(5),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY5(6),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY5(7),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */

	{ EXYNOS4_GPY6(0),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY6(1),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY6(2),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY6(3),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY6(4),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY6(5),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY6(6),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
	{ EXYNOS4_GPY6(7),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	/* NC */
#endif 
	{ EXYNOS4_GPZ(0),  S3C_GPIO_SLP_PREV,  S3C_GPIO_PULL_NONE},	//I2S0_SCLK
	{ EXYNOS4_GPZ(1),  S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},	//I2S0_CDCLK
	{ EXYNOS4_GPZ(2),  S3C_GPIO_SLP_PREV,  S3C_GPIO_PULL_NONE}, //I2S0_LRCK
	{ EXYNOS4_GPZ(3),  S3C_GPIO_SLP_PREV,  S3C_GPIO_PULL_NONE}, //I2S0_SDI
	{ EXYNOS4_GPZ(4),  S3C_GPIO_SLP_PREV,  S3C_GPIO_PULL_NONE}, //I2S0_SDO0
	{ EXYNOS4_GPZ(5),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, //WIFI_PWDN
	{ EXYNOS4_GPZ(6),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},	//BT_RST
#endif
///crystal add 
	{ GPIO_CAM_MEGA_EN,  S3C_GPIO_SLP_OUT1,	S3C_GPIO_PULL_NONE},
	{ GPIO_CAM_MEGA_nRST,  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ GPIO_CAM_MCLK,  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ EXYNOS4212_GPJ1(3),  S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_UP},	//ID
};
#ifdef CONFIG_TC4_DVT
	//GPX0(0) ---NC
	//GPX2(6) --6260_GPIO1
	//GPX3(2) --6260_GPIO2
#endif
static unsigned int tc4_sleep_alive_gpio_table[][4] =
{//ly 20111118 modified it for eint wakeup
	{EXYNOS4_GPX0(0), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //ANX7805_INIT
	{EXYNOS4_GPX0(1), S3C_GPIO_SLP_OUT1,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //ANX7805_PD
	//{EXYNOS4_GPX0(2), S3C_GPIO_OUTPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //ONO
	{EXYNOS4_GPX0(3), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE},	//CABLE_DET
	{EXYNOS4_GPX0(4), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE},	//TP1_INT
	{EXYNOS4_GPX0(5), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE},	//PIX_SDA
	{EXYNOS4_GPX0(6), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //PIX_SCL
	//{EXYNOS4_GPX0(7), S3C_GPIO_SFN(0xf),	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //TF_CDN

	{EXYNOS4_GPX1(0), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE},  //CHG_FLT
	{EXYNOS4_GPX1(1), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE},	//NFC_INT
	{EXYNOS4_GPX1(2), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE},	//KP_LED
	{EXYNOS4_GPX1(3), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //HOOK_DET
	{EXYNOS4_GPX1(4), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE},	//WIFI_WOW
	//{EXYNOS4_GPX1(5), S3C_GPIO_SFN(0xf),	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE},	//CHG_UOK
	//{EXYNOS4_GPX1(6), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE},	//MD_SLEEP_REQUEST
	{EXYNOS4_GPX1(7), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //IRQ_PMIC


	{EXYNOS4_GPX2(0), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE},	//KP_ROW0
	{EXYNOS4_GPX2(1), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE},	//KP_ROW1
	{EXYNOS4_GPX2(2), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE},	//HP_DET
	{EXYNOS4_GPX2(3), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE},	//HUB_INIT_N
	{EXYNOS4_GPX2(4), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //MD_B14
	//{EXYNOS4_GPX2(5), S3C_GPIO_SFN(0xf),	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //MD_WAKEUP_AP
	//{EXYNOS4_GPX2(6), S3C_GPIO_SFN(0xf),	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //CHG_DOK
	//{EXYNOS4_GPX2(7), S3C_GPIO_SFN(0xf),	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //CHG_COK

	{EXYNOS4_GPX3(0), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE},	//GM_INT1
	{EXYNOS4_GPX3(1), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //GM_INI2
	//{EXYNOS4_GPX3(2), S3C_GPIO_SFN(0xf),	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //FUEL_ALRT
	{EXYNOS4_GPX3(3), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE},	//GYRO_INT
// Cellon modify start, Ted Shi, 2012/12/13, for modify notation
//	{EXYNOS4_GPX3(4), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //COMPASS_RDY
	{EXYNOS4_GPX3(4), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //FM_INT
// Cellon modify end, Ted Shi, 2012/12/13
	{EXYNOS4_GPX3(5), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //MD_L15
	{EXYNOS4_GPX3(6), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE}, //HDMI_CEC
	{EXYNOS4_GPX3(7), S3C_GPIO_INPUT,	S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE},	//HDMI_HPD

};


//added by yulu
static void tc4_config_sleep_gpio_table(void)
{
	int i,gpio;
/*
	[Fix Me] Below codes are sample GPIO initialization and review codes
	for target platform if needed.
*/
#if 0
	for (i = 0; i < ARRAY_SIZE(tc4_sleep_alive_gpio_table); i++)
	  {
		  gpio = tc4_sleep_alive_gpio_table[i][0];
	
		  s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(tc4_sleep_alive_gpio_table[i][1]));
		  if (tc4_sleep_alive_gpio_table[i][2] != S3C_GPIO_SETPIN_NONE)
		  {
			  gpio_set_value(gpio, tc4_sleep_alive_gpio_table[i][2]);
		  }
		  s3c_gpio_setpull(gpio, tc4_sleep_alive_gpio_table[i][3]);
	  }

	
#endif
	config_sleep_gpio_table(ARRAY_SIZE(tc4_sleep_gpio_table),
			tc4_sleep_gpio_table); ///yulu
}

#endif
#define SMDK4412_REV_0_0_ADC_VALUE 0
#define SMDK4412_REV_0_1_ADC_VALUE 443
int samsung_board_rev;

static int get_samsung_board_rev(void)
{
	int		ret = 0;
#if 0 //It's only for smdk
	int 		adc_val = 0;
	struct clk	*adc_clk;
	struct resource	*res;
	void __iomem	*adc_regs;
	unsigned int	con;
	int		ret;

	if ((soc_is_exynos4412() && samsung_rev() < EXYNOS4412_REV_1_0) ||
		(soc_is_exynos4212() && samsung_rev() < EXYNOS4212_REV_1_0))
		return SAMSUNG_BOARD_REV_0_0;

	adc_clk = clk_get(NULL, "adc");
	if (unlikely(IS_ERR(adc_clk)))
		return SAMSUNG_BOARD_REV_0_0;

	clk_enable(adc_clk);

	res = platform_get_resource(&s3c_device_adc, IORESOURCE_MEM, 0);
	if (unlikely(!res))
		goto err_clk;

	adc_regs = ioremap(res->start, resource_size(res));
	if (unlikely(!adc_regs))
		goto err_clk;

	writel(S5PV210_ADCCON_SELMUX(3), adc_regs + S5PV210_ADCMUX);

	con = readl(adc_regs + S3C2410_ADCCON);
	con &= ~S3C2410_ADCCON_MUXMASK;
	con &= ~S3C2410_ADCCON_STDBM;
	con &= ~S3C2410_ADCCON_STARTMASK;
	con |=  S3C2410_ADCCON_PRSCEN;

	con |= S3C2410_ADCCON_ENABLE_START;
	writel(con, adc_regs + S3C2410_ADCCON);

	udelay (50);

	adc_val = readl(adc_regs + S3C2410_ADCDAT0) & 0xFFF;
	writel(0, adc_regs + S3C64XX_ADCCLRINT);

	iounmap(adc_regs);
err_clk:
	clk_disable(adc_clk);
	clk_put(adc_clk);

	ret = (adc_val < SMDK4412_REV_0_1_ADC_VALUE/2) ?
			SAMSUNG_BOARD_REV_0_0 : SAMSUNG_BOARD_REV_0_1;

	pr_info ("SMDK MAIN Board Rev 0.%d (ADC value:%d)\n", ret, adc_val);
#endif
	return ret;
}

static void __init smdk4x12_machine_init(void)
{
#ifdef CONFIG_S3C64XX_DEV_SPI
	unsigned int gpio;
	struct clk *sclk = NULL;
	struct clk *prnt = NULL;
	struct device *spi0_dev = &exynos_device_spi0.dev;
#ifndef CONFIG_FB_S5P_LMS501KF03
	struct device *spi1_dev = &exynos_device_spi1.dev;
#endif
	struct device *spi2_dev = &exynos_device_spi2.dev;
#endif
	int err;
//	tc4_config_init_gpio();
#if 0
	u32 gpio;
	gpio_request(EXYNOS4_GPL2(1), "GPL2(1)");
	gpio = EXYNOS4_GPL2(1);
	gpio_direction_output(EXYNOS4_GPL2(1), 1);
	s3c_gpio_cfgpin(gpio, S3C_GPIO_OUTPUT);

	gpio_set_value(gpio, 1);
#endif

	pm_power_off = smdk4x12_power_off;
	s3c_config_sleep_gpio_table = tc4_config_sleep_gpio_table;
	samsung_board_rev = get_samsung_board_rev();

#if defined(CONFIG_EXYNOS_DEV_PD) && defined(CONFIG_PM_RUNTIME)
	exynos_pd_disable(&exynos4_device_pd[PD_MFC].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_G3D].dev);
//	exynos_pd_disable(&exynos4_device_pd[PD_LCD0].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_CAM].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_TV].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_GPS].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_GPS_ALIVE].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_ISP].dev);
#elif defined(CONFIG_EXYNOS_DEV_PD)
	/*
	 * These power domains should be always on
	 * without runtime pm support.
	 */
	exynos_pd_enable(&exynos4_device_pd[PD_MFC].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_G3D].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_LCD0].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_CAM].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_TV].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_GPS].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_GPS_ALIVE].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_ISP].dev);
#endif
	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));

	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));

	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(2, i2c_devs2, ARRAY_SIZE(i2c_devs2));

	s3c_i2c3_set_platdata(NULL);
	i2c_register_board_info(3, i2c_devs3, ARRAY_SIZE(i2c_devs3));

	s3c_i2c4_set_platdata(NULL);
	i2c_register_board_info(4, i2c_devs4, ARRAY_SIZE(i2c_devs4));
	s3c_i2c5_set_platdata(NULL);
	i2c_register_board_info(5, i2c_devs5, ARRAY_SIZE(i2c_devs5));

	//For S5K4EC (using i2c6)
	#ifdef CONFIG_AAT3635
	//aat3635_gpio_init();
	#endif
	//cellon qiu.li modify 20120821
	#if defined(CONFIG_AAT3635) ||defined(CONFIG_MHL_Sii8334)
	s3c_i2c6_set_platdata(NULL);
	i2c_register_board_info(6, i2c_devs6, ARRAY_SIZE(i2c_devs6));

	#endif
	s3c_i2c7_set_platdata(NULL);
	i2c_register_board_info(7, i2c_devs7, ARRAY_SIZE(i2c_devs7));

#ifdef CONFIG_ANDROID_PMEM
	android_pmem_set_platdata();
#endif
#if defined(CONFIG_FB_S5P_MIPI_DSIM)
	mipi_fb_init();
#endif
#ifdef CONFIG_FB_S3C
	dev_set_name(&s5p_device_fimd0.dev, "s3cfb.0");
	clk_add_alias("lcd", "exynos4-fb.0", "lcd", &s5p_device_fimd0.dev);
	clk_add_alias("sclk_fimd", "exynos4-fb.0", "sclk_fimd", &s5p_device_fimd0.dev);
	s5p_fb_setname(0, "exynos4-fb");
#if defined(CONFIG_LCD_AMS369FG06) || defined(CONFIG_LCD_LMS501KF03)
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif
	s5p_fimd0_set_platdata(&smdk4x12_lcd0_pdata);
#ifdef CONFIG_FB_MIPI_DSIM
	s5p_device_mipi_dsim.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
#endif
#ifdef CONFIG_EXYNOS_DEV_PD
	s5p_device_fimd0.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
#endif
#endif
#ifdef CONFIG_FB_S5P
#ifdef CONFIG_FB_S5P_LMS501KF03
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
	s3cfb_set_platdata(&lms501kf03_data);
#else
	s3cfb_set_platdata(NULL);
#endif
#ifdef CONFIG_FB_S5P_MIPI_DSIM
	s5p_device_dsim.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
#endif
#ifdef CONFIG_EXYNOS_DEV_PD
	s3c_device_fb.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
#endif
#endif
#ifdef CONFIG_USB_EHCI_S5P
	smdk4x12_ehci_init();
#endif
#ifdef CONFIG_USB_OHCI_S5P
	smdk4x12_ohci_init();
#endif
#ifdef CONFIG_USB_GADGET
	smdk4x12_usbgadget_init();
#endif
#ifdef CONFIG_USB_EXYNOS_SWITCH
	smdk4x12_usbswitch_init();
#endif

	samsung_bl_set(&smdk4x12_bl_gpio_info, &smdk4x12_bl_data);
// add leds' platform parameters by zhaojin 20120820
#ifdef CONFIG_LEDS_KP
	platform_device_register(&smdk4x12_kp_led_dev);
#endif
//add end
#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	exynos_dwmci_set_platdata(&exynos_dwmci_pdata);
#endif

#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
	exynos4_fimc_is_set_platdata(NULL);
#ifdef CONFIG_EXYNOS_DEV_PD
	exynos4_device_fimc_is.dev.parent = &exynos4_device_pd[PD_ISP].dev;
#endif
#endif
#ifdef CONFIG_S3C_DEV_HSMMC
	s3c_sdhci0_set_platdata(&smdk4x12_hsmmc0_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	s3c_sdhci1_set_platdata(&smdk4x12_hsmmc1_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	s3c_sdhci2_set_platdata(&smdk4x12_hsmmc2_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	s3c_sdhci3_set_platdata(&smdk4x12_hsmmc3_pdata);
#endif
#ifdef CONFIG_S5P_DEV_MSHC
	s3c_mshci_set_platdata(&exynos4_mshc_pdata);
#endif
#if defined(CONFIG_VIDEO_EXYNOS_TV) && defined(CONFIG_VIDEO_EXYNOS_HDMI)
	dev_set_name(&s5p_device_hdmi.dev, "exynos4-hdmi");
	clk_add_alias("hdmi", "s5p-hdmi", "hdmi", &s5p_device_hdmi.dev);
	clk_add_alias("hdmiphy", "s5p-hdmi", "hdmiphy", &s5p_device_hdmi.dev);
	s5p_tv_setup();

	/* setup dependencies between TV devices */
	s5p_device_hdmi.dev.parent = &exynos4_device_pd[PD_TV].dev;
	s5p_device_mixer.dev.parent = &exynos4_device_pd[PD_TV].dev;

	s5p_i2c_hdmiphy_set_platdata(NULL);
#ifdef CONFIG_VIDEO_EXYNOS_HDMI_CEC
	s5p_hdmi_cec_set_platdata(&hdmi_cec_data);
#endif
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
	smdk4x12_set_camera_flite_platdata();
	s3c_set_platdata(&exynos_flite0_default_data,
			sizeof(exynos_flite0_default_data), &exynos_device_flite0);
	s3c_set_platdata(&exynos_flite1_default_data,
			sizeof(exynos_flite1_default_data), &exynos_device_flite1);
#ifdef CONFIG_EXYNOS_DEV_PD
	exynos_device_flite0.dev.parent = &exynos4_device_pd[PD_ISP].dev;
	exynos_device_flite1.dev.parent = &exynos4_device_pd[PD_ISP].dev;
#endif
#endif
#ifdef CONFIG_EXYNOS_SETUP_THERMAL
	s5p_tmu_set_platdata(&exynos_tmu_data);
#endif
#ifdef CONFIG_VIDEO_FIMC
	s3c_fimc0_set_platdata(&fimc_plat);
#ifdef CONFIG_TC4_GB
	s3c_fimc1_set_platdata(NULL);
	s3c_fimc2_set_platdata(&fimc_plat);
#else
	s3c_fimc1_set_platdata(&fimc_plat);
        s3c_fimc2_set_platdata(NULL);
#endif
	s3c_fimc3_set_platdata(NULL);
#ifdef CONFIG_EXYNOS_DEV_PD
	s3c_device_fimc0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc1.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc2.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc3.dev.parent = &exynos4_device_pd[PD_CAM].dev;
#ifdef CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION
	secmem.parent = &exynos4_device_pd[PD_CAM].dev;
#endif
#endif
#ifdef CONFIG_VIDEO_FIMC_MIPI
	s3c_csis0_set_platdata(NULL);
	s3c_csis1_set_platdata(NULL);
#ifdef CONFIG_EXYNOS_DEV_PD
	s3c_device_csis0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_csis1.dev.parent = &exynos4_device_pd[PD_CAM].dev;
#endif
#endif

#if defined(CONFIG_ITU_A) || defined(CONFIG_CSI_C) \
	|| defined(CONFIG_S5K3H1_CSI_C) || defined(CONFIG_S5K3H2_CSI_C) \
	|| defined(CONFIG_S5K6A3_CSI_C)
	smdk4x12_cam0_reset(1);
#endif
#if defined(CONFIG_ITU_B) || defined(CONFIG_CSI_D) \
	|| defined(CONFIG_S5K3H1_CSI_D) || defined(CONFIG_S5K3H2_CSI_D) \
	|| defined(CONFIG_S5K6A3_CSI_D)
	smdk4x12_cam1_reset(1);
#endif
#endif /* CONFIG_VIDEO_FIMC */

#ifdef CONFIG_VIDEO_SAMSUNG_S5P_FIMC
	smdk4x12_camera_config();
	smdk4x12_subdev_config();

	dev_set_name(&s5p_device_fimc0.dev, "s3c-fimc.0");
	dev_set_name(&s5p_device_fimc1.dev, "s3c-fimc.1");
	dev_set_name(&s5p_device_fimc2.dev, "s3c-fimc.2");
	dev_set_name(&s5p_device_fimc3.dev, "s3c-fimc.3");

	clk_add_alias("fimc", "exynos4210-fimc.0", "fimc", &s5p_device_fimc0.dev);
	clk_add_alias("sclk_fimc", "exynos4210-fimc.0", "sclk_fimc",
			&s5p_device_fimc0.dev);
	clk_add_alias("fimc", "exynos4210-fimc.1", "fimc", &s5p_device_fimc1.dev);
	clk_add_alias("sclk_fimc", "exynos4210-fimc.1", "sclk_fimc",
			&s5p_device_fimc1.dev);
	clk_add_alias("fimc", "exynos4210-fimc.2", "fimc", &s5p_device_fimc2.dev);
	clk_add_alias("sclk_fimc", "exynos4210-fimc.2", "sclk_fimc",
			&s5p_device_fimc2.dev);
	clk_add_alias("fimc", "exynos4210-fimc.3", "fimc", &s5p_device_fimc3.dev);
	clk_add_alias("sclk_fimc", "exynos4210-fimc.3", "sclk_fimc",
			&s5p_device_fimc3.dev);

	s3c_fimc_setname(0, "exynos4210-fimc");
	s3c_fimc_setname(1, "exynos4210-fimc");
	s3c_fimc_setname(2, "exynos4210-fimc");
	s3c_fimc_setname(3, "exynos4210-fimc");
	/* FIMC */
	s3c_set_platdata(&s3c_fimc0_default_data,
			 sizeof(s3c_fimc0_default_data), &s5p_device_fimc0);
	s3c_set_platdata(&s3c_fimc1_default_data,
			 sizeof(s3c_fimc1_default_data), &s5p_device_fimc1);
	s3c_set_platdata(&s3c_fimc2_default_data,
			 sizeof(s3c_fimc2_default_data), &s5p_device_fimc2);
	s3c_set_platdata(&s3c_fimc3_default_data,
			 sizeof(s3c_fimc3_default_data), &s5p_device_fimc3);
#ifdef CONFIG_EXYNOS_DEV_PD
	s5p_device_fimc0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s5p_device_fimc1.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s5p_device_fimc2.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s5p_device_fimc3.dev.parent = &exynos4_device_pd[PD_CAM].dev;
#endif
#ifdef CONFIG_VIDEO_S5P_MIPI_CSIS
	dev_set_name(&s5p_device_mipi_csis0.dev, "s3c-csis.0");
	dev_set_name(&s5p_device_mipi_csis1.dev, "s3c-csis.1");
	clk_add_alias("csis", "s5p-mipi-csis.0", "csis",
			&s5p_device_mipi_csis0.dev);
	clk_add_alias("sclk_csis", "s5p-mipi-csis.0", "sclk_csis",
			&s5p_device_mipi_csis0.dev);
	clk_add_alias("csis", "s5p-mipi-csis.1", "csis",
			&s5p_device_mipi_csis1.dev);
	clk_add_alias("sclk_csis", "s5p-mipi-csis.1", "sclk_csis",
			&s5p_device_mipi_csis1.dev);
	dev_set_name(&s5p_device_mipi_csis0.dev, "s5p-mipi-csis.0");
	dev_set_name(&s5p_device_mipi_csis1.dev, "s5p-mipi-csis.1");

	s3c_set_platdata(&s5p_mipi_csis0_default_data,
			sizeof(s5p_mipi_csis0_default_data), &s5p_device_mipi_csis0);
	s3c_set_platdata(&s5p_mipi_csis1_default_data,
			sizeof(s5p_mipi_csis1_default_data), &s5p_device_mipi_csis1);
#ifdef CONFIG_EXYNOS_DEV_PD
	s5p_device_mipi_csis0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s5p_device_mipi_csis1.dev.parent = &exynos4_device_pd[PD_CAM].dev;
#endif
#endif
#if defined(CONFIG_ITU_A) || defined(CONFIG_CSI_C) \
	|| defined(CONFIG_S5K3H1_CSI_C) || defined(CONFIG_S5K3H2_CSI_C) \
	|| defined(CONFIG_S5K6A3_CSI_C)
	smdk4x12_cam0_reset(1);
#endif
#if defined(CONFIG_ITU_B) || defined(CONFIG_CSI_D) \
	|| defined(CONFIG_S5K3H1_CSI_D) || defined(CONFIG_S5K3H2_CSI_D) \
	|| defined(CONFIG_S5K6A3_CSI_D)
	smdk4x12_cam1_reset(1);
#endif
#endif

#if defined(CONFIG_VIDEO_TVOUT)
	s5p_hdmi_hpd_set_platdata(&hdmi_hpd_data);
	s5p_hdmi_cec_set_platdata(&hdmi_cec_data);
#ifdef CONFIG_EXYNOS_DEV_PD
	s5p_device_tvout.dev.parent = &exynos4_device_pd[PD_TV].dev;
	exynos4_device_pd[PD_TV].dev.parent= &exynos4_device_pd[PD_LCD0].dev;
#endif
#endif

#if	defined(CONFIG_VIDEO_JPEG_V2X) || defined(CONFIG_VIDEO_JPEG)
#ifdef CONFIG_EXYNOS_DEV_PD
	s5p_device_jpeg.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	exynos4_jpeg_setup_clock(&s5p_device_jpeg.dev, 160000000);
#endif
#endif

#ifdef CONFIG_ION_EXYNOS
	exynos_ion_set_platdata();
#endif

#if defined(CONFIG_VIDEO_MFC5X) || defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
#ifdef CONFIG_EXYNOS_DEV_PD
	s5p_device_mfc.dev.parent = &exynos4_device_pd[PD_MFC].dev;
#endif
	if (soc_is_exynos4412() && samsung_rev() >= EXYNOS4412_REV_1_0)
		exynos4_mfc_setup_clock(&s5p_device_mfc.dev, 200 * MHZ);
	else
		exynos4_mfc_setup_clock(&s5p_device_mfc.dev, 267 * MHZ);
#endif

#if defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
	dev_set_name(&s5p_device_mfc.dev, "s3c-mfc");
	clk_add_alias("mfc", "s5p-mfc", "mfc", &s5p_device_mfc.dev);
	s5p_mfc_setname(&s5p_device_mfc, "s5p-mfc");
#endif

#ifdef CONFIG_VIDEO_FIMG2D
	s5p_fimg2d_set_platdata(&fimg2d_data);
#endif
	samsung_keypad_set_platdata(&smdk4x12_keypad_data);
	//smdk4x12_smsc911x_init();
#ifdef CONFIG_EXYNOS_C2C
	exynos_c2c_set_platdata(&smdk4x12_c2c_pdata);
#endif
#ifndef CONFIG_TC4_GB
	exynos_sysmmu_init();
#endif
	smdk4x12_gpio_power_init();

// Cellon add start, ZePeng Wu, 2012/08/04, for TP 
#if defined(CONFIG_RMI4_BUS)
	s3202_virtual_key_properties();
#endif
// Cellon end start, ZePeng Wu, 2012/08/04, for TP 
 
	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
	if (soc_is_exynos4412())
		platform_add_devices(smdk4412_devices, ARRAY_SIZE(smdk4412_devices));

#ifdef CONFIG_FB_S3C
	exynos4_fimd0_setup_clock(&s5p_device_fimd0.dev, "mout_mpll_user",
				800 * MHZ);
#endif
#ifdef CONFIG_S3C64XX_DEV_SPI
	sclk = clk_get(spi0_dev, "dout_spi0");
	if (IS_ERR(sclk))
		dev_err(spi0_dev, "failed to get sclk for SPI-0\n");
	prnt = clk_get(spi0_dev, "mout_mpll_user");
	if (IS_ERR(prnt))
		dev_err(spi0_dev, "failed to get prnt\n");
	if (clk_set_parent(sclk, prnt))
		printk(KERN_ERR "Unable to set parent %s of clock %s.\n",
				prnt->name, sclk->name);

	clk_set_rate(sclk, 800 * 1000 * 1000);
	clk_put(sclk);
	clk_put(prnt);

	if (!gpio_request(EXYNOS4_GPB(1), "SPI_CS0")) {
		gpio_direction_output(EXYNOS4_GPB(1), 1);
		s3c_gpio_cfgpin(EXYNOS4_GPB(1), S3C_GPIO_SFN(1));
		s3c_gpio_setpull(EXYNOS4_GPB(1), S3C_GPIO_PULL_UP);
		exynos_spi_set_info(0, EXYNOS_SPI_SRCCLK_SCLK,
			ARRAY_SIZE(spi0_csi));
	}

	for (gpio = EXYNOS4_GPB(0); gpio < EXYNOS4_GPB(4); gpio++)
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV3);

	spi_register_board_info(spi0_board_info, ARRAY_SIZE(spi0_board_info));

#ifndef CONFIG_FB_S5P_LMS501KF03
	sclk = clk_get(spi1_dev, "dout_spi1");
	if (IS_ERR(sclk))
		dev_err(spi1_dev, "failed to get sclk for SPI-1\n");
	prnt = clk_get(spi1_dev, "mout_mpll_user");
	if (IS_ERR(prnt))
		dev_err(spi1_dev, "failed to get prnt\n");
	if (clk_set_parent(sclk, prnt))
		printk(KERN_ERR "Unable to set parent %s of clock %s.\n",
				prnt->name, sclk->name);

	clk_set_rate(sclk, 800 * 1000 * 1000);
	clk_put(sclk);
	clk_put(prnt);

	if (!gpio_request(EXYNOS4_GPB(5), "SPI_CS1")) {
		gpio_direction_output(EXYNOS4_GPB(5), 1);
		s3c_gpio_cfgpin(EXYNOS4_GPB(5), S3C_GPIO_SFN(1));
		s3c_gpio_setpull(EXYNOS4_GPB(5), S3C_GPIO_PULL_UP);
		exynos_spi_set_info(1, EXYNOS_SPI_SRCCLK_SCLK,
			ARRAY_SIZE(spi1_csi));
	}

	for (gpio = EXYNOS4_GPB(4); gpio < EXYNOS4_GPB(8); gpio++)
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV3);

	spi_register_board_info(spi1_board_info, ARRAY_SIZE(spi1_board_info));
#endif

	sclk = clk_get(spi2_dev, "dout_spi2");
	if (IS_ERR(sclk))
		dev_err(spi2_dev, "failed to get sclk for SPI-2\n");
	prnt = clk_get(spi2_dev, "mout_mpll_user");
	if (IS_ERR(prnt))
		dev_err(spi2_dev, "failed to get prnt\n");
	if (clk_set_parent(sclk, prnt))
		printk(KERN_ERR "Unable to set parent %s of clock %s.\n",
				prnt->name, sclk->name);

	clk_set_rate(sclk, 800 * 1000 * 1000);
	clk_put(sclk);
	clk_put(prnt);

	if (!gpio_request(EXYNOS4_GPC1(2), "SPI_CS2")) {
		gpio_direction_output(EXYNOS4_GPC1(2), 1);
		s3c_gpio_cfgpin(EXYNOS4_GPC1(2), S3C_GPIO_SFN(1));
		s3c_gpio_setpull(EXYNOS4_GPC1(2), S3C_GPIO_PULL_UP);
		exynos_spi_set_info(2, EXYNOS_SPI_SRCCLK_SCLK,
			ARRAY_SIZE(spi2_csi));
	}

	for (gpio = EXYNOS4_GPC1(1); gpio < EXYNOS4_GPC1(5); gpio++)
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV3);

	spi_register_board_info(spi2_board_info, ARRAY_SIZE(spi2_board_info));
#endif
#ifdef CONFIG_BUSFREQ_OPP
	dev_add(&busfreq, &exynos4_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_DMC0], &exynos4_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_DMC1], &exynos4_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_CPU], &exynos4_busfreq.dev);
#endif
	register_reboot_notifier(&exynos4_reboot_notifier);
	
	//set pwrdn always 1 for ov2675,crystal,2012-10-7
	if (gpio_request(GPIO_CAM_MEGA_EN, "GX0_0"/*"GPJ0"*/) < 0)
	{
		pr_err("failed gpio_request(GX0_0) for camera control\n");
	}
	gpio_direction_output(GPIO_CAM_MEGA_EN, 1); 
	gpio_free(GPIO_CAM_MEGA_EN); 
	printk("========== to set pwrdn always 1\n");
	//set for sleep-power-consuption for ov2675,crystal,2012-10-17
	///ov2675_power_down();	
	//Qiu.Li add mhl gpio config
#ifndef CONFIG_MHL_Sii8334
	if (!gpio_request(EXYNOS4_GPX3(1), "mhl rst")) {
		s3c_gpio_cfgpin(EXYNOS4_GPX3(1), S3C_GPIO_INPUT);	
		s3c_gpio_setpull(EXYNOS4_GPX3(1), S3C_GPIO_PULL_DOWN);
		gpio_free(EXYNOS4_GPX3(1));
	}
	if (!gpio_request(EXYNOS4_GPX3(6), "mhl int")) {
		s3c_gpio_cfgpin(EXYNOS4_GPX3(6), S3C_GPIO_INPUT);	
		s3c_gpio_setpull(EXYNOS4_GPX3(6), S3C_GPIO_PULL_DOWN);
		gpio_free(EXYNOS4_GPX3(6));
	}
	if (!gpio_request(EXYNOS4_GPX3(7), "mhl plug")) {
		s3c_gpio_cfgpin(EXYNOS4_GPX3(7), S3C_GPIO_INPUT);
		s3c_gpio_setpull(EXYNOS4_GPX3(7), S3C_GPIO_PULL_DOWN);
		gpio_free(EXYNOS4_GPX3(7));
	}
#endif
      //Qiu.Li add end
}

#ifdef CONFIG_EXYNOS_C2C
static void __init exynos_c2c_reserve(void)
{
	static struct cma_region region = {
			.name = "c2c_shdmem",
			.size = 64 * SZ_1M,
			{ .alignment	= 64 * SZ_1M },
			.start = C2C_SHAREDMEM_BASE
	};

	BUG_ON(cma_early_region_register(&region));
	BUG_ON(cma_early_region_reserve(&region));
}
#endif
#ifdef CONFIG_TC4_GB
MACHINE_START(SMDK4212, "SMDK4212")
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= smdk4x12_map_io,
	.init_machine	= smdk4x12_machine_init,
	.timer		= &exynos4_timer,
	#if defined(CONFIG_KERNEL_PANIC_DUMP)		//mj for panic-dump
	.reserve		= reserve_panic_dump_area,
	#endif

#ifdef CONFIG_EXYNOS_C2C
	.reserve	= &exynos_c2c_reserve,
#endif
MACHINE_END

MACHINE_START(SMDK4412, "SMDK4212")
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= smdk4x12_map_io,
	.init_machine	= smdk4x12_machine_init,
	.timer		= &exynos4_timer,

	#if defined(CONFIG_KERNEL_PANIC_DUMP)		//mj for panic-dump
	.reserve		= reserve_panic_dump_area,
	#endif

#ifdef CONFIG_EXYNOS_C2C
	.reserve	= &exynos_c2c_reserve,
#endif
MACHINE_END
#endif

#ifdef CONFIG_TC4_ICS
MACHINE_START(SMDK4212, "SMDK4X12")
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= smdk4x12_map_io,
	.init_machine	= smdk4x12_machine_init,
	.timer		= &exynos4_timer,
	#if defined(CONFIG_KERNEL_PANIC_DUMP)		//mj for panic-dump
	.reserve		= reserve_panic_dump_area,
	#endif

#ifdef CONFIG_EXYNOS_C2C
	.reserve	= &exynos_c2c_reserve,
#endif
MACHINE_END

MACHINE_START(SMDK4412, "SMDK4X12")
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= smdk4x12_map_io,
	.init_machine	= smdk4x12_machine_init,
	.timer		= &exynos4_timer,

	#if defined(CONFIG_KERNEL_PANIC_DUMP)		//mj for panic-dump
	.reserve		= reserve_panic_dump_area,
	#endif

#ifdef CONFIG_EXYNOS_C2C
	.reserve	= &exynos_c2c_reserve,
#endif
MACHINE_END
#endif


