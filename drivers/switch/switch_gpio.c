/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/types.h>

#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/slab.h>

#include <linux/err.h>
#include <linux/delay.h>

#include <plat/adc.h>
#include <linux/timer.h>
#include <linux/input.h>
#include "../../sound/soc/codecs/wm8994.h"
#include <linux/earlysuspend.h>
#include <linux/input.h>
/*
 * GPIO pin description for Switch
 *
 * - HP_DET   : GPX2_2 (WAKEUP_INT2[2])
 * - HOOK_DET : GPX1_3 (WAKEUP_INT1[3])
 * - UART_SW  : GPC1_0
 */


/*
 * HeadSet type definition
 */

//#define DEBUG
#define ADC_HOOK_DET  /* just read the gpio value */
#define IRQ_HOOK_DET
#define ADC_BAT_TEST
static int Use_irq = 0;
#define Hook_Key_Interval 30

int bat_temp=25;
static int  bat_temp_value=3645;
int bat_ID_value=2048;

#ifdef ADC_HOOK_DET
#define HOOK_KEY1_VAL 30
#endif

static bool is_suspend = false;

#define HOOK_IRQ IRQ_EINT(23)

#ifdef DEBUG
#define hook_printk( fmt, arg...) printk(fmt, ##arg)
#else
#define hook_printk( fmt, arg...)
#endif

#define BIT_HEADSET             (1<<0)  // Speaker and Mic
#define BIT_HEADSET_NO_MIC      (1<<1)  // Only Speaker

#define STABLE_TIME (2*HZ)

static int switch_gpio_active = 0;
/*wenpin.cui: because initial detecion code has been moved to wm8994 driver, 
so initial state 0 will bring problem*/
static volatile int switch_current_state = -1;

static volatile u32 last_jiffies;
static volatile bool bias_flag = false;
extern bool wm8994_suspend_flag;
extern volatile int incall_suspend_flag;
static long int adc_value[19]=
{
3795,
3691,
3561,
3405,
3222,
3014,
2787,
2544,
2296,
2048,
1808,
1583,
1375,
1187,
1021,
875,
748,
640,
547,
};

static int adc_temp[19]=
{
-20,
-15,
-10,
-5,
0,
5,
10,
15,
20,
25,
30,
35,
40,
45,
50,
55,
60,
65,
70,
};

struct gpio_switch_data {
        struct switch_dev sdev;
        unsigned gpio;
        const char *name_on;
        const char *name_off;
        const char *state_on;
        const char *state_off;
#ifdef ADC_HOOK_DET
	struct s3c_adc_client	*hk_adc_client;//hs_hook_adc client
	struct delayed_work hk_adc_work;	
	struct timer_list tx_timer;
	struct input_dev *input_adc_dev;
#endif
#ifdef IRQ_HOOK_DET
	struct input_dev *input_irq_dev;
#endif
#ifdef ADC_BAT_TEST
	struct s3c_adc_client	*bat_adc_client;//hs_hook_adc client
	struct delayed_work bat_adc_work;	
	struct timer_list bat_adc_timer;
	//struct input_dev *input_dev;
#endif	   

        int irq;
        struct delayed_work work;
};

static int debug2hs_show(char *val, struct kernel_param *kp)
{
        int value = 0;
        int ret = param_set_int(val, kp);
        if(ret < 0) {
                printk(KERN_ERR"%s: Error param_set_int.\n", __FUNCTION__);
                return -EINVAL;
        }

        value = *((int*)kp->arg);
        printk(KERN_ALERT"\r\nxx value(%d) arg(%d)\r\n\r\n", value, switch_gpio_active);
        return 0;
}

/*
 * on:
 *      true: Headset on
 *      false: UART on
 */
void debug_switch_to_hs(const unsigned int on)
{
	int ret = 0;

	ret = gpio_request(EXYNOS4_GPC1(0), "GPC1");
	if (ret < 0) {
		printk(KERN_ERR"fail to request S5PV310_GPC1(0) for UART_SW\n ");
		return;
	}

	s3c_gpio_cfgpin(EXYNOS4_GPC1(0), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPC1(0), S3C_GPIO_PULL_NONE);
	gpio_set_value(EXYNOS4_GPC1(0), ((!!on)? 1: 0));
	gpio_free(EXYNOS4_GPC1(0));
	return;
}

static int debug2hs_store(const char *val, struct kernel_param *kp)
{
	int value;
	int ret = param_set_int(val, kp);

	if(ret < 0) {
		printk(KERN_ERR"%s: Error param_set_int.\n", __FUNCTION__);
		return -EINVAL;
	}

	value = *((int*)kp->arg);
	printk(KERN_ALERT"xx value = %d\n",value);
	debug_switch_to_hs(value);
	return 0; 
}

static void gpio_switch_work(struct work_struct *work)
{
	int state = 0;
	struct delayed_work *dwork =  to_delayed_work(work);
	struct gpio_switch_data	*data =
		container_of(dwork, struct gpio_switch_data, work);

	last_jiffies = jiffies;
	// Get "physical" level(0: HS inserted, 1: HS removed)
	state = gpio_get_value(data->gpio);
	mdelay(10); 	// Wait before 2nd sampling to detect the changed status of HP_DET.
	if (state != gpio_get_value(data->gpio))
		return;

	state = !state;	// Get the state from "physical" level(0: HS removed, 1: HS inserted)

	debug_switch_to_hs(state? true: false);	/* switch between uart/hp */ 
	if (switch_current_state != state)
	{
		// Set the state and make the uevent to platfrom(HeadsetObserver).
		switch_set_state(&data->sdev, state);
		switch_current_state = state;
	}

	#ifdef IRQ_HOOK_DET
	if(state == 1){
		wm8994_set_micb2(1);
		bias_flag = true;
		printk("headset insert \n");
	}
	else{
		wm8994_set_micb2(0);
		bias_flag = false;
		printk("headset unplug \n");
	}
	#endif

	#ifdef ADC_HOOK_DET
	if(state == 1){
		hook_printk("headset insert \n");
		wm8994_set_micb2(1);
		mod_timer(&data->tx_timer, jiffies + (2*HZ));
	}
	else{
		hook_printk("headset unplug \n");	
		wm8994_set_micb2(0);		
		del_timer(&data->tx_timer);
	}
	#endif	
	
	return;
}

#ifdef ADC_HOOK_DET
static int hk_adc_read_ch(struct s3c_adc_client *client,struct gpio_switch_data *switch_data)
{
	int adc_val = 0;
	static int press = 0;
	static int key_val = 0;
	static int last_key_val = 0;

//	ret = mutex_lock_interruptible(&hwmon->lock);
//	if (ret < 0)
//		return ret;
	adc_val = s3c_adc_read(client, 1);
	hook_printk("s3c_adc_read 1  %d\n",adc_val);

	if(adc_val < HOOK_KEY1_VAL){
	//	hook_printk("input_report_key  0 do press =%d\n",press);
		press +=1;
		if(press == 2){
			key_val = 1;
//			input_report_key(switch_data->input_dev, KEY_MEDIA, 0);
			press = 0;
			hook_printk("1 last_key_val  %d    key_val  %d press %d\n",last_key_val,key_val,press);
		}
	}
	else{
	//	hook_printk("input_report_key  1 not press  %d\n",press);
		key_val = 0;
//		input_report_key(switch_data->input_dev, KEY_MEDIA, 1);	
		press = 0;
		hook_printk("2 last_key_val  %d  key_val  %d press %d\n",last_key_val,key_val,press);
	}

	if(last_key_val != key_val){
		printk("??input_report_keylast_key_val %d   key_val  %d\n",last_key_val,key_val);
		input_report_key(switch_data->input_adc_dev, KEY_MEDIA, key_val);
		input_sync(switch_data->input_adc_dev);
		last_key_val = key_val;
	}
//	mutex_unlock(&hwmon->lock);
	return adc_val;
}

static void adc_hk_detect_timer_expire(unsigned long data)
{
//	unsigned long irq_flags;

	struct gpio_switch_data *switch_data = (struct gpio_switch_data *)data;
	if(Use_irq> 1)/*if in p2b board we use irq ,return adc timer*/
		return ;
	//printk("!! adc_hk_detect_timer_expire\n");

	/* modify by devin for unplug detect */
	mod_timer(&switch_data->tx_timer, jiffies + (HZ/7));

	if(is_suspend == true){
		return ;
	}	
	hk_adc_read_ch(switch_data->hk_adc_client,switch_data);

}
#endif

#ifdef ADC_BAT_TEST
static int get_a(int x1,int x2,int y1,int y2 )
{
	return (y1-y2)/(x1-x2);
}
static int get_b(int x1,int x2,int y1,int y2 )
{
	return (x1*y2-x2*y1)/(x1-x2);
}
static int get_temp(int a, int b,int adc_value)
{
	return (adc_value - b)/a;
}
	
static int bat_adc_read_ch(struct s3c_adc_client *client,struct gpio_switch_data *switch_data)
{
	int adc0_val,adc2_val;
	int a,b;
	int temp;
	int i=0;
	
	adc0_val = s3c_adc_read(client, 0);
	adc2_val = s3c_adc_read(client, 2);
	hook_printk("\n*********s3c_adc_read 2 -- %d********\n",adc2_val);
	if(adc0_val>(bat_temp_value+500))
		adc0_val =bat_temp_value ;
	
	hook_printk("\ns3c_adc_read 0 -- %d\n",adc0_val);
	hook_printk("\n########s3c_adc_read 2 -- %d#########\n",adc2_val);
	bat_ID_value = adc2_val;
	bat_temp_value = adc0_val;
	if(adc0_val>adc_value[0])
		adc0_val = adc_value[0];
	if(adc0_val < adc_value[18])
		adc0_val = adc_value[18];
	while(i<18)
	{
		if((adc0_val>=adc_value[i+1]) && (adc0_val<=adc_value[i]))
		{
			a = get_a(adc_temp[i],adc_temp[i+1],adc_value[i],adc_value[i+1]);
			hook_printk("gat a=%d\n",a);
			b = get_b(adc_temp[i],adc_temp[i+1],adc_value[i],adc_value[i+1]);
			hook_printk("gat b=%d\n",b);
			temp = get_temp(a,b,adc0_val);
			hook_printk("gat temp=%d\n",temp);
			bat_temp = temp;
			return temp;
		}
		else 
		{
			hook_printk("count = %d \n",i);
			i++;
		}
	}
	
	return 0;
}

static void adc_bat_detect_timer_expire(unsigned long data)
{
//	unsigned long irq_flags;

	struct gpio_switch_data *switch_data = (struct gpio_switch_data *)data;

	//printk("!!adc_bat_detect_timer_expire\n");
	mod_timer(&switch_data->bat_adc_timer, jiffies + msecs_to_jiffies(3*1000));

	if(is_suspend == true){
		return ;
	}

	bat_adc_read_ch(switch_data->bat_adc_client,switch_data);

}
#endif

extern unsigned int pm_eint_pend2, pm_wakeup_stat;

#ifdef IRQ_HOOK_DET
static irqreturn_t hook_det_isr(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data 
						= (struct gpio_switch_data *)dev_id;
	int val, i, hs_state, interval;
	Use_irq += 1;/*if Use_irq > 1 ,we use irq*/
	
	if(wm8994_suspend_flag && !incall_suspend_flag){
			goto out;
	}
	if (abs(jiffies - last_jiffies) < STABLE_TIME)
		goto out;
	hs_state = gpio_get_value(switch_data->gpio);
	val = gpio_get_value(EXYNOS4_GPX2(7));
	
	for (i = 0; i < 3; i++) {
		msleep(Hook_Key_Interval/3);
		if ((val != gpio_get_value(EXYNOS4_GPX2(7)))|| (hs_state != gpio_get_value(switch_data->gpio)))
			goto out;
	}

	if (bias_flag) {
		if (val == 0) {
			input_report_key(switch_data->input_irq_dev, KEY_MEDIA, 1);
		} else {
			input_report_key(switch_data->input_irq_dev, KEY_MEDIA, 0);
		}
	}

	input_sync(switch_data->input_irq_dev);

out:
	return IRQ_HANDLED;
}
#endif

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data =
		(struct gpio_switch_data *)dev_id;

//	hook_printk("-gpio_irq_handler ----------->");
//	hk_adc_read_ch(switch_data->hk_adc_client,switch_data);

	if (switch_current_state) 
		;
	else /* noise coming */
		/* Turn off uart first to avoid noise made by trace */
		debug_switch_to_hs(true);

	/* wait for a long time to make sure voltage is stable*/
	schedule_delayed_work(&switch_data->work, HZ/100);
	return IRQ_HANDLED;
}

static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct gpio_switch_data	*switch_data =
		container_of(sdev, struct gpio_switch_data, sdev);
	const char *state;

	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

#if defined(ADC_HOOK_DET) || defined(IRQ_HOOK_DET)
#ifdef CONFIG_HAS_EARLYSUSPEND
static void switch_gpio_early_suspend(struct early_suspend *handler)
{
	is_suspend = true;
}

static void switch_gpio_early_resume(struct early_suspend *handler)
{
	is_suspend = false;
}
#endif
#endif

static int gpio_switch_probe(struct platform_device *pdev)
{
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_switch_data *switch_data;
	int ret = 0;
#ifdef IRQ_HOOK_DET
	struct input_dev *irq_dev;
	static struct early_suspend early_suspend;
#endif
#ifdef ADC_HOOK_DET
	struct input_dev *adc_dev;
	//static struct early_suspend early_suspend;
#endif

	if (!pdata)
		return -EBUSY;

	switch_data = kzalloc(sizeof(struct gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = pdata->name;
	switch_data->gpio = pdata->gpio;
	switch_data->name_on = pdata->name_on;
	switch_data->name_off = pdata->name_off;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
	switch_data->sdev.print_state = switch_gpio_print_state;
#ifdef ADC_HOOK_DET
	switch_data->hk_adc_client = s3c_adc_register(pdev, NULL, NULL, 0);
	if (IS_ERR(switch_data->hk_adc_client)) {
		dev_err(&pdev->dev, "cannot register adc\n");
		ret = PTR_ERR(switch_data->hk_adc_client);
		goto err;
	}
	
	/* Initialize timer */
	init_timer(&switch_data->tx_timer);
	switch_data->tx_timer.function = adc_hk_detect_timer_expire;
	switch_data->tx_timer.data = switch_data;

	adc_dev = input_allocate_device();
	if (!adc_dev) {
		ret = -ENOMEM;
		goto err;
	}
	switch_data->input_adc_dev = adc_dev;
	adc_dev->name	= "samsung_adc_hs";

	input_set_capability(adc_dev, EV_KEY, KEY_MEDIA);

	ret = input_register_device(adc_dev);
	if (ret) {
		dev_err(&adc_dev->dev,
				"hs_probe: input_register_device rc=%d\n", ret);
		goto err_reg_input_dev;
	}
	
#endif

#ifdef IRQ_HOOK_DET
	irq_dev = input_allocate_device();
	if (!irq_dev) {
		ret = -ENOMEM;
		goto err;
	}
	switch_data->input_irq_dev = irq_dev;
	irq_dev->name	= "hook_detection";

	input_set_capability(irq_dev, EV_KEY, KEY_MEDIA);

	ret = input_register_device(irq_dev);
	if (ret) {
		dev_err(&irq_dev->dev,
				"hs_probe: input_register_device rc=%d\n", ret);
		goto err_input_dev;
	}

	s3c_gpio_cfgpin(EXYNOS4_GPX2(7), S3C_GPIO_SFN(0xf)); /* GPIO we use */
	ret = gpio_request(EXYNOS4_GPX2(7), "hook_det");
	if (ret) {
		printk("failed to request gpio\n");
		return ret;
	}
	
	ret = request_threaded_irq(HOOK_IRQ, NULL, hook_det_isr,
				(IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
					"hook-interrupt", switch_data);
	if (ret < 0)
		panic("failed to request hook irq\n");
#endif

#ifdef ADC_BAT_TEST
	switch_data->bat_adc_client = s3c_adc_register(pdev, NULL, NULL, 0);
	if (IS_ERR(switch_data->bat_adc_client)) {
		dev_err(&pdev->dev, "cannot register bat adc\n");
		ret = PTR_ERR(switch_data->bat_adc_client);
		goto err;
	}
	
	/* Initialize timer */
	init_timer(&switch_data->bat_adc_timer);
	switch_data->bat_adc_timer.function = adc_bat_detect_timer_expire;
	switch_data->bat_adc_timer.data = (void *)switch_data;
	mod_timer(&switch_data->bat_adc_timer, jiffies + (2*HZ));
#endif


	ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	ret = gpio_request(switch_data->gpio, "GPX2");
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(switch_data->gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	s3c_gpio_setpull(switch_data->gpio, S3C_GPIO_PULL_NONE);

	INIT_DELAYED_WORK(&switch_data->work, gpio_switch_work);

	switch_data->irq = platform_get_irq(pdev, 0);   // WAKEUP_INT2[2]
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;
	}

	ret = request_irq(switch_data->irq, gpio_irq_handler,
		(IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING), pdev->name, switch_data);

	if (ret < 0)
		goto err_request_irq;
	/*detect the headphone after the whole system started up*/
	schedule_delayed_work(&switch_data->work, HZ*10);/*wenpin.cui: init check*/
	debug_switch_to_hs(true);

/* 
 * for some reason, we must make use of 
 * this flag which set during earlysuspend.
 */
#if defined(ADC_HOOK_DET) || defined(IRQ_HOOK_DET)
#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend.suspend = switch_gpio_early_suspend;
	early_suspend.resume = switch_gpio_early_resume;
	register_early_suspend(&early_suspend);
#endif
#endif

	return 0;

err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->gpio);
err_request_gpio:
	switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:
	kfree(switch_data);
err:
        return ret;

#ifdef IRQ_HOOK_DET
err_input_dev:
	input_free_device(irq_dev);
#endif		
#ifdef ADC_HOOK_DET		
err_reg_input_dev:
	input_free_device(adc_dev);
#endif
	return ret;
}

static int __devexit gpio_switch_remove(struct platform_device *pdev)
{
	struct gpio_switch_data *switch_data = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&switch_data->work);
	gpio_free(switch_data->gpio);
	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

static struct platform_driver gpio_switch_driver = {
	.probe		= gpio_switch_probe,
	.remove		= __devexit_p(gpio_switch_remove),
	.driver		= {
		.name	= "switch-gpio",
		.owner	= THIS_MODULE,
	},
};

static int __init gpio_switch_init(void)
{
	return platform_driver_register(&gpio_switch_driver);
}

static void __exit gpio_switch_exit(void)
{
	platform_driver_unregister(&gpio_switch_driver);
}


module_param_call(switch_gpio_active, debug2hs_store, debug2hs_show, &switch_gpio_active, (S_IRUSR | S_IWUSR));

module_init(gpio_switch_init);
module_exit(gpio_switch_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");
