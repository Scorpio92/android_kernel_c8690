/*
 *
 * Charger driver for Max8903
 *
 *  Copyright (C) 2008 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * Author: yang81.li@samsung.com
 * Date: 2011.09.05
 */

#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <mach/gpio.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <plat/gpio-cfg.h>
//#define MAX8903_DBG
#ifdef MAX8903_DBG
#define max8903_dbg(fmt,...)   \
	printk("max8903:"fmt, ##__VA_ARGS__)
#else
#define max8903_dbg(fmt,...)
#endif


struct max8903_platform_data{
	unsigned int  cok;
	unsigned int  uok;
	unsigned int  cen;
};

//need ldo7 1.8V for cok do uok io power
static struct max8903_platform_data max8903_priv=
{
	.cok=EXYNOS4_GPX2(7),//dok
	.uok=EXYNOS4_GPX1(5),//uok
	.cen=EXYNOS4_GPL0(4),//cen
};

static struct workqueue_struct *max8903_workqueue;
static struct work_struct max8903_work;

extern void s5p_cable_check_status(int flag);

static irqreturn_t irq_handler(int irqno, void *param)
{

	max8903_dbg("irq_handler\n");

	queue_work(max8903_workqueue, &max8903_work);

	return IRQ_HANDLED; 

}


void changer_work_func(void)
{
	u32 cok;

	cok=gpio_get_value(max8903_priv.cok);
	max8903_dbg("***cok is %d ***\n",cok);
	if(!cok)
		s5p_cable_check_status(1);//ac
	else 
		s5p_cable_check_status(0);//battery

}
/*
init max8903 gpio and irq
we don't register a platform driver for max8903,so don't clear DOK and UOK 
pending in pm.c when system suspend/resume.
*/
static int __init max8903_init(void)
{
	int ret = 0;
	
	max8903_dbg("charger driver initialize+++\n");
	
	ret=gpio_is_valid(max8903_priv.cen) && gpio_is_valid(max8903_priv.cok);
	if(!ret){
		printk("gpio is not valid for max8903");
		goto done;
		}
	//enable charger set low 
 	ret=gpio_request(max8903_priv.cen, "changer enable pin");
	if(ret<0){
			printk("gpio requst fail for max8903: S5PV310_GPL0(4) (cen) pin!! ");
		goto done;
		}
	gpio_direction_output(max8903_priv.cen,0);

	//uok 
 	 ret=gpio_request(max8903_priv.cok, "cok pin");
	if(ret<0){
		printk("gpio requst fail for max8903: cok pin!! ");
		goto done;
		}
	s3c_gpio_setpull(max8903_priv.cok, S3C_GPIO_PULL_NONE);
	gpio_direction_input(max8903_priv.cok);
	//init woekqueue 
	 INIT_WORK(&max8903_work, changer_work_func);
	 max8903_workqueue = create_singlethread_workqueue("max8903_workqueue");

	//scan before enable irq	 
	queue_work(max8903_workqueue, &max8903_work);

	
	//set  irq
	ret = request_irq(gpio_to_irq(max8903_priv.cok), irq_handler,
			IRQF_SAMPLE_RANDOM|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
			"uok_int",NULL);
	if (ret < 0)
	{
		printk("request_irq(uok_irq_handler) failed due to %d !!!\n",ret);
		free_irq(gpio_to_irq(max8903_priv.cok),NULL);
		goto done;
	}
	enable_irq_wake(gpio_to_irq(max8903_priv.cok));
	
	max8903_dbg("charger driver initialize---\n");
	return 0;

done:
	return ret;

}

static void __exit max8903_exit(void)
{
	max8903_dbg("charger driver release\n");

	free_irq(max8903_priv.cok,NULL);
	//free_irq(max8903_priv.dok,NULL);

	gpio_free(max8903_priv.cen);
	gpio_free(max8903_priv.cok);
	//gpio_free(max8903_priv.cok);
}

module_init(max8903_init);
module_exit(max8903_exit);



