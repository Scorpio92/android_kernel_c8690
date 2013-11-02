/*=========================================================
Additions and modifications made by Cellon Communications
==========================================================*/
/* arch/arm/mach-exynos/dev-broadcom-gps.c
 *
 * Copyright (c) 2012-2012, Cellon Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/mach-types.h>
#include <linux/switch.h>
#include <linux/gpio.h>
#include <plat/gpio-cfg.h>

//gpio control  switch
#define GPIO_EN_SWITCH_GPIO EXYNOS4_GPC1(2)
#define GPIO_RST_SWITCH_GPIO EXYNOS4_GPB(4)

#define GPS_SWITCH_DEBUG

#ifdef GPS_SWITCH_DEBUG
#define GPS_TRACE(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
#else
#define GPS_TRACE(fmt, ...)
#endif

#ifdef CONFIG_CELLON_ENG_LOG
#define BCM4751_LOG_ON 1
#else 
#define BCM4751_LOG_ON 0
#endif

static unsigned int gps_uart_on_table[][4] = {
	{EXYNOS4_GPA0(4), S3C_GPIO_SFN(2), 2, S3C_GPIO_PULL_NONE},
	{EXYNOS4_GPA0(5), S3C_GPIO_SFN(2), 2, S3C_GPIO_PULL_NONE},
	{EXYNOS4_GPA0(6), S3C_GPIO_SFN(2), 2, S3C_GPIO_PULL_NONE},
	{EXYNOS4_GPA0(7), S3C_GPIO_SFN(2), 2, S3C_GPIO_PULL_NONE},
	{EXYNOS4_GPB(4),   S3C_GPIO_OUTPUT, 1, S3C_GPIO_PULL_NONE},
	{EXYNOS4_GPC1(2), S3C_GPIO_OUTPUT, 0, S3C_GPIO_PULL_NONE},
};
void gps_config_gpio_table(int array_size, unsigned int (*gpio_table)[4])
{
	u32 i, gpio;

	for (i = 0; i < array_size; i++) {
		gpio = gpio_table[i][0];
		s3c_gpio_cfgpin(gpio, gpio_table[i][1]);
		gpio_set_value(gpio, gpio_table[i][2]);
		s3c_gpio_setpull(gpio, gpio_table[i][3]);
	}
}

static ssize_t gps_switch_print_state(struct switch_dev *sdev, char *buf);
static ssize_t gps_switch_print_name(struct switch_dev *sdev, char *buf);

static struct switch_dev gps_switch_dev = {
	.name = "gps_en_rst_switch",
//	.print_state = gps_switch_print_state, 
	.print_name = gps_switch_print_name, 
};

unsigned int gps_en_show_state(char *buf)
{
	int value;
	int err;
	err = gpio_request(GPIO_EN_SWITCH_GPIO,"GPC1_2");
	if(err){
		printk("%s: failed to request for GPIO %d \n",__func__,GPIO_EN_SWITCH_GPIO);	
		goto failed;
	}

	value = gpio_get_value(GPIO_EN_SWITCH_GPIO);
	#if BCM4751_LOG_ON
	printk("get: GPIO_EN_SWITCH_GPIO = %x \n",value);
	#endif
	gpio_free(GPIO_EN_SWITCH_GPIO);

	sprintf(buf,"%d",value);	
	return 0;
	
failed:
	printk("%s: failed to request for GPIO \n",__func__);	
	return -EINVAL;
}
EXPORT_SYMBOL(gps_en_show_state);
unsigned int gps_rst_show_state(char *buf)
{
	int value;
	int err;	
	err = gpio_request(GPIO_RST_SWITCH_GPIO,"GPB4");
	if(err){
		printk("%s: failed to request for GPIO %d \n",__func__,GPIO_RST_SWITCH_GPIO);	
		goto failed;
	}

	value = gpio_get_value(GPIO_RST_SWITCH_GPIO);
	#if BCM4751_LOG_ON
	printk("get:GPIO_RST_SWITCH_GPIO= %x \n",value);
	#endif
	gpio_free(GPIO_RST_SWITCH_GPIO);

	sprintf(buf,"%d ",value);	
	return 0;
	
failed: 
	printk("%s: failed to request for GPIO \n",__func__);	
	return -EINVAL;
}
EXPORT_SYMBOL(gps_rst_show_state);
static ssize_t gps_switch_print_state(struct switch_dev *sdev, char *buf)
{
	int value1;
	int value2;
	
	int err;
	err = gpio_request(GPIO_EN_SWITCH_GPIO,"GPC1[2]");
	if(err){
		printk("%s: failed to request for GPIO %d \n",__func__,GPIO_EN_SWITCH_GPIO);
		goto failed;
	}
	err = s3c_gpio_cfgpin(GPIO_EN_SWITCH_GPIO, S3C_GPIO_OUTPUT);
	if(err){
		printk("%s: failed to set direction for GPIO %d \n",__func__,GPIO_EN_SWITCH_GPIO);
		goto failed;
	}
	value1 = gpio_get_value(GPIO_EN_SWITCH_GPIO);
	gpio_free(GPIO_EN_SWITCH_GPIO);

	err = gpio_request(GPIO_RST_SWITCH_GPIO,"GPB[4]");
	if(err){
		printk("%s: failed to request for GPIO %d \n",__func__,GPIO_RST_SWITCH_GPIO);	
		goto failed;
	}
	err = s3c_gpio_cfgpin(GPIO_RST_SWITCH_GPIO, S3C_GPIO_OUTPUT);
	if(err){
		printk("%s: failed to set direction for GPIO %d \n",__func__,GPIO_RST_SWITCH_GPIO);	
		goto failed;
	}
	value2 = gpio_get_value(GPIO_RST_SWITCH_GPIO);
	gpio_free(GPIO_RST_SWITCH_GPIO);

	sprintf(buf,"GPS_EN: %x , GPS_RST: %x ",value1,value2);	

	return 0;
failed: 
	printk("%s: failed to request for GPIO \n",__func__);	
	return -EINVAL;
}

int gps_en_switch_gpio(unsigned int onoff)
{
	int err;

	GPS_TRACE("%s: onoff = %d \n",__func__,onoff);
	err = gpio_request(GPIO_EN_SWITCH_GPIO,"GPC1_2");
	if(err){
		printk("%s: failed to request for GPIO %d \n",__func__,GPIO_EN_SWITCH_GPIO);	
		goto failed;
	}

	err = s3c_gpio_cfgpin(GPIO_EN_SWITCH_GPIO, S3C_GPIO_OUTPUT);
	if(err){
		printk("%s: failed to set direction for GPIO %d \n",__func__,GPIO_EN_SWITCH_GPIO);	
		goto failed;
	}

	gpio_set_value(GPIO_EN_SWITCH_GPIO, onoff?1:0);
	#if BCM4751_LOG_ON
	printk("set:GPIO_EN_SWITCH_GPIO = %x \n",onoff);
	#endif
	gpio_free(GPIO_EN_SWITCH_GPIO);

	return 0;
failed: 
	printk("%s: failed to request for GPIO %d \n",__func__,GPIO_EN_SWITCH_GPIO);	

	return -EINVAL;
}
EXPORT_SYMBOL(gps_en_switch_gpio);

int gps_rst_switch_gpio(unsigned int onoff)
{
	int err;

	GPS_TRACE("%s: onoff = %d \n",__func__,onoff);
	err = gpio_request(GPIO_RST_SWITCH_GPIO,"GPB4");
	if(err){
		printk("%s: failed to request for GPIO %d \n",__func__,GPIO_RST_SWITCH_GPIO);	
		goto failed;
	}

	err = s3c_gpio_cfgpin(GPIO_RST_SWITCH_GPIO, S3C_GPIO_OUTPUT);
	if(err){
		printk("%s: failed to set direction for GPIO %d \n",__func__,GPIO_RST_SWITCH_GPIO);	
		goto failed;
	}

	gpio_set_value(GPIO_RST_SWITCH_GPIO, onoff?1:0);
	#if BCM4751_LOG_ON
	printk("set:GPIO_RST_SWITCH_GPIO = %x \n",onoff);
	#endif
	gpio_free(GPIO_RST_SWITCH_GPIO);

	return 0;
failed: 
	printk("%s: failed to request for GPIO %d \n",__func__,GPIO_RST_SWITCH_GPIO);	

	return -EINVAL;
}
EXPORT_SYMBOL(gps_rst_switch_gpio);

static ssize_t gps_switch_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "gps_en_rst_switch \n");
}

static int __init gps_switch_init(void)
{
	int err;
	
	GPS_TRACE("%s: \n",__func__);
	
	gps_config_gpio_table(ARRAY_SIZE(gps_uart_on_table),
					gps_uart_on_table);
	err = switch_dev_register(&gps_switch_dev);
	
	if(err){
		goto err_switch_dev_register;
	}
	
	return err;
err_switch_dev_register:
	switch_dev_unregister(&gps_switch_dev);

	return err;
}
late_initcall(gps_switch_init);

static void __exit gps_switch_exit(void)
{
	switch_dev_unregister(&gps_switch_dev);
}
module_exit(gps_switch_exit);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("cellon gps switch");
