/* linux/arch/arm/plat-samsung/dev-i2c3.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * S5P series device definition for i2c device 3
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/map.h>

#include <plat/regs-iic.h>
#include <plat/iic.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/gpio-cfg.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#define RELEASE_I2C5_PWRON_AT_KERNEL
//#undef RELEASE_I2C5_PWRON_AT_KERNEL

static struct resource s3c_i2c_resource[] = {
	[0] = {
		.start	= S3C_PA_IIC5,
		.end	= S3C_PA_IIC5 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_IIC5,
		.end	= IRQ_IIC5,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device s3c_device_i2c5 = {
	.name		= "s3c2440-i2c",
	.id		= 5,
	.num_resources	= ARRAY_SIZE(s3c_i2c_resource),
	.resource	= s3c_i2c_resource,
};

#ifdef RELEASE_I2C5_PWRON_AT_KERNEL
//check if IIC-5 is hold or not, if yes, send some clk release the bus first
static void release_i2c5(void)
{
	//add by D.Z start 2012/12/15 
	int cycle = 0;
	printk("[start]***************%s*****************\n", __func__);
		
	s3c_gpio_cfgall_range(EXYNOS4_GPB(2), 2,S3C_GPIO_OUTPUT, S3C_GPIO_PULL_UP);
	
	gpio_set_value(EXYNOS4_GPB(2), 1);
	gpio_set_value(EXYNOS4_GPB(3), 1);

	s3c_gpio_cfgpin(EXYNOS4_GPB(2),S3C_GPIO_INPUT);
		
	for(cycle = 0; cycle < 300; cycle ++)
	{
		if(gpio_get_value(EXYNOS4_GPB(2)) != 0){
			break;
		}
		else{
			printk("DAT ==0\n");
		}
		
		udelay(4);
		gpio_set_value(EXYNOS4_GPB(3), 0);
		udelay(4);
		gpio_set_value(EXYNOS4_GPB(3), 1);
		

	}

	if(cycle >= 299){
//		panic("IIC-5 wrong, restart......\n");
	 		printk("[error] : i2c bus had been hold by hardware!!!");
	}

	s3c_gpio_cfgall_range(EXYNOS4_GPB(2), 2,S3C_GPIO_INPUT, S3C_GPIO_PULL_UP);//set input
	printk("%s:GPB_2 %d,GPB_3 %d\n", __func__,gpio_get_value(EXYNOS4_GPB(2)),gpio_get_value(EXYNOS4_GPB(3)));	
	printk("[end]******************%s****************11\n", __func__);

	s3c_gpio_cfgall_range(EXYNOS4_GPB(2), 2,S3C_GPIO_SFN(3), S3C_GPIO_PULL_UP);//set as iic
//		//add by D.Z end
}
#endif

void __init s3c_i2c5_set_platdata(struct s3c2410_platform_i2c *pd)
{
	struct s3c2410_platform_i2c *npd;

	if (!pd) {
		pd = &default_i2c_data;
		pd->bus_num = 5;
	}

#ifdef RELEASE_I2C5_PWRON_AT_KERNEL
	release_i2c5();
#endif

	npd = s3c_set_platdata(pd, sizeof(struct s3c2410_platform_i2c),
			       &s3c_device_i2c5);

	if (!npd->cfg_gpio)
		npd->cfg_gpio = s3c_i2c5_cfg_gpio;
}
