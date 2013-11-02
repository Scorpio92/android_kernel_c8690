/*=========================================================
Additions and modifications made by Cellon Communications
==========================================================*/
/* arch/arm/mach-exynos/dev-usb-ma-switch.c
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

//gpio control USB switch
#define USB_MA_SWITCH_GPIO EXYNOS4_GPK3(2)

#define USB_MA_SWITCH_DEBUG

#ifdef USB_MA_SWITCH_DEBUG
#define USB_MA_TRACE(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
#else
#define USB_MA_TRACE(fmt, ...)
#endif

// Cellon add start, Ted Shi, 2013/01/07, for prevent write/read race
static spinlock_t usb_switch_lock;
// Cellon add end, Ted Shi, 2013/01/07

static ssize_t usb_ma_switch_print_state(struct switch_dev *sdev, char *buf);
static ssize_t usb_ma_switch_print_name(struct switch_dev *sdev, char *buf);

static struct switch_dev usb_ma_switch_dev = {
	.name = "usb_ma_switch",
	.print_state = usb_ma_switch_print_state, 
	.print_name = usb_ma_switch_print_name, 
};
static int usb_ma_gpio_init()
{
	int err;
	err = gpio_request(USB_MA_SWITCH_GPIO,"USB_SEL");
	if(err){
		pr_err("%s: failed to request for GPIO %d \n",__func__,USB_MA_SWITCH_GPIO);	
		goto failed;
	}
	err = s3c_gpio_cfgpin(USB_MA_SWITCH_GPIO, S3C_GPIO_SFN(1));
	if(err){
		pr_err("%s: failed to set direction for GPIO %d \n",__func__,USB_MA_SWITCH_GPIO);	
		goto failed;
	}
	gpio_free(USB_MA_SWITCH_GPIO);
	return err;
failed: 
	printk("%s: failed to request for GPIO %d \n",__func__,USB_MA_SWITCH_GPIO);	
	return -EINVAL;
}
static ssize_t usb_ma_switch_print_state(struct switch_dev *sdev, char *buf)
{
	int value,err;
// Cellon add start, Ted Shi, 2013/01/07, for prevent write/read race
	unsigned long flags;

	spin_lock_irqsave(&usb_switch_lock, flags);
// Cellon add end, Ted Shi, 2013/01/07
	err = gpio_request(USB_MA_SWITCH_GPIO,"USB_SEL");
	if(err){
		pr_err("%s: failed to request for GPIO %d \n",__func__,USB_MA_SWITCH_GPIO);	
		goto failed;
	}
	
	value = gpio_get_value(USB_MA_SWITCH_GPIO);
	printk("get: USB_MA_SWITCH_GPIO %x \n",value);
	gpio_free(USB_MA_SWITCH_GPIO);

// Cellon add start, Ted Shi, 2013/01/07, for prevent write/read race
	spin_unlock_irqrestore(&usb_switch_lock, flags);
// Cellon add end, Ted Shi, 2013/01/07

	return sprintf(buf,"%s\n",value?"1":"0");;
failed: 
	printk("%s: failed to request for GPIO %d \n",__func__,USB_MA_SWITCH_GPIO);	
	return -EINVAL;
}
/*
*func desc: switch usb port to modem or ap side
*onoff : 1 means usb on ap side,0 means usb on modem side
*/
int usb_ma_switch_gpio(unsigned int onoff)
{
	int value,err;
// Cellon add start, Ted Shi, 2013/01/07, for prevent write/read race
	unsigned long flags;

	spin_lock_irqsave(&usb_switch_lock, flags);
// Cellon add end, Ted Shi, 2013/01/07
	err = gpio_request(USB_MA_SWITCH_GPIO,"USB_SEL");
	if(err){
		pr_err("%s: failed to request for GPIO %d \n",__func__,USB_MA_SWITCH_GPIO);	
		goto failed;
	}
	
	USB_MA_TRACE("%s: onoff = %d \n",__func__,onoff);
	gpio_set_value(USB_MA_SWITCH_GPIO, onoff?1:0);
	printk("set: USB_MA_SWITCH_GPIO %x \n",onoff);
	gpio_free(USB_MA_SWITCH_GPIO);
	
// Cellon add start, Ted Shi, 2013/01/07, for prevent write/read race
	spin_unlock_irqrestore(&usb_switch_lock, flags);
// Cellon add end, Ted Shi, 2013/01/07

	return 0;
failed: 
	printk("%s: failed to request for GPIO %d \n",__func__,USB_MA_SWITCH_GPIO);	

	return -EINVAL;
}
EXPORT_SYMBOL(usb_ma_switch_gpio);

static ssize_t usb_ma_switch_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "usb_mb_switch \n");
}

static int __init usb_ma_switch_init(void)
{
	int err;
	
	USB_MA_TRACE("%s: \n",__func__);
	//usb_ma_gpio_init();
// Cellon add start, Ted Shi, 2013/01/07, for prevent write/read race
	spin_lock_init(&usb_switch_lock);
// Cellon add end, Ted Shi, 2013/01/07
	err = switch_dev_register(&usb_ma_switch_dev);
	
	if(err){
		goto err_switch_dev_register;
	}
	return err;
err_switch_dev_register:
	switch_dev_unregister(&usb_ma_switch_dev);

	return err;
}
late_initcall(usb_ma_switch_init);

static void __exit usb_ma_switch_exit(void)
{
	switch_dev_unregister(&usb_ma_switch_dev);
}
module_exit(usb_ma_switch_exit);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("cellon USB Modem/Ap switch");
