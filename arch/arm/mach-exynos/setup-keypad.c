/* linux/arch/arm/mach-exynos/setup-keypad.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * GPIO configuration for Exynos4 KeyPad device
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gpio.h>
#include <plat/gpio-cfg.h>

void samsung_keypad_cfg_gpio(unsigned int rows, unsigned int cols)
{
	/* Keypads can be of various combinations, Just making sure */

	/* Set all the necessary GPX2 pins: KP_ROW[x] */
	s3c_gpio_cfgall_range(EXYNOS4_GPX2(0), 2,
					 S3C_GPIO_SFN(3), S3C_GPIO_PULL_UP);

	/*GPL2: KP_COL[x] */
	//s3c_gpio_cfgrange_nopull(EXYNOS4_GPL2(0), 1, S3C_GPIO_SFN(3));
	s3c_gpio_cfgrange_nopull(EXYNOS4_GPX1(0), 2, S3C_GPIO_SFN(3));
}
