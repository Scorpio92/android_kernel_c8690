/*
 * Modem control driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Suchang Woo <suchang.woo@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
#include <mach/modem.h>
#include <linux/modemctl.h>
#include <linux/wait.h>
#include <linux/sched.h>

#include <linux/pm_runtime.h>

#ifdef CONFIG_CELLON_ENG_LOG
#define HSIC_DEBUG
#endif

static struct wake_lock cp_reset_wake_lock;
extern struct device *s5p_dev;
extern u8 suspending_flag;
extern u8 suspended_flag;
extern void usb_host_phy_init(void);
extern void usb_host_phy_off(void);
extern void usb_host_phy_suspend(void);
extern int usb_host_phy_resume(void);
enum {
	HOST_WAKEUP_LOW = 1,
	HOST_WAKEUP_WAIT_RESET,
} HOST_WAKEUP_STATE;

enum {
	MODEM_EVENT_RESET,
	MODEM_EVENT_CRASH,
	MODEM_EVENT_DUMP,
	MODEM_EVENT_CONNECT,
	MODEM_EVENT_RESET_DONE,
} MODEM_EVENT_TYPE;

/* FIXME: Don't use this except pm */
struct modemctl *global_mc;
extern int acm_init(void);
extern int smd_init(void);
extern void  acm_exit(void);
extern void smd_exit(void);
extern int acm_request_resume(void);
extern int  s5pv210_ehci_power(int value);
static int modem_boot_enumeration(struct modemctl *mc);
static int modem_main_enumeration(struct modemctl *mc);
u8 under_reset=0;
#ifdef CONFIG_USB_EHCI_S5P	
extern int s5p_ehci_power(int value);
#endif
int mc_reconnect_gpio(void)
{
	struct modemctl *mc = global_mc;

	if (!mc)
		return -EFAULT;
	printk("TRY Reconnection.................................\n");

	gpio_set_value(mc->gpio_active_state, 0);
	msleep(10);
	gpio_set_value(mc->gpio_ipc_slave_wakeup, 1);
	msleep(10);
	gpio_set_value(mc->gpio_ipc_slave_wakeup, 0);
	msleep(10);
	gpio_set_value(mc->gpio_active_state, 1);

	return 0;
}
EXPORT_SYMBOL_GPL(mc_reconnect_gpio);
int mc_is_suspend_request(void)
{

	printk(KERN_DEBUG "%s:suspend requset val=%d\n", __func__,
		gpio_get_value(global_mc->gpio_suspend_request));
	return gpio_get_value(global_mc->gpio_suspend_request);
}
EXPORT_SYMBOL_GPL(mc_is_suspend_request);
int mc_prepare_resume(int ms_time)
{
	int val;
	int count =0;
	struct completion done;
	struct modemctl *mc = global_mc;
	if (!mc)
		return -EFAULT;
	
	if((global_mc->in_l3_state == 1)){//in L3 state
			if(smm6260_is_host_wakeup()&&(gpio_get_value(mc->gpio_ipc_slave_wakeup)==0)){  //CP request from L3 to L0
				printk("waiting for process from L3 to L0 by cp, %s()\n",__func__);	
				msleep(30);
				return MC_HOST_HIGH;
			}
			else{
				if(suspended_flag==0){   //hsic suspended but devices not finished
					printk("system suspend not finished, waiting!, %s()\n",__func__);	
					msleep(50);
					return MC_CP_RESET;
				}
			}
		}
	else {//in L2 state
		if(smm6260_is_host_wakeup()&&(gpio_get_value(mc->gpio_ipc_slave_wakeup)))  //AP request from L2 to L0
			{
				printk("already in L2 to L0 process by AP, %s()\n",__func__);	
				msleep(20);
				return MC_HOST_HIGH;
			}
	}

		if(suspending_flag){   //AP change from L2 to L3 ,go to L3 first
			printk("system suspending,waiting! %s()",__func__);
			msleep(20);
			return MC_CP_RESET;
		}

	val = gpio_get_value(mc->gpio_ipc_slave_wakeup);
	if (val) {
		gpio_set_value(mc->gpio_ipc_slave_wakeup, 0);
		dev_info(mc->dev, "svn SLAV_WUP:reset\n");
	}
	val = gpio_get_value(mc->gpio_ipc_host_wakeup);
	if (val == HOST_WUP_LEVEL) {
#ifdef HSIC_DEBUG
		dev_info(mc->dev, "svn HOST_WUP:high!\n");
#endif
		return MC_HOST_HIGH;
	}

	init_completion(&done);
	mc->l2_done = &done;

	gpio_set_value(mc->gpio_ipc_slave_wakeup, 1);
#ifdef HSIC_DEBUG
	dev_info(mc->dev, "AP>>CP:  SLAV_WUP:1,%d\n",
		gpio_get_value(mc->gpio_ipc_slave_wakeup));
#endif

	if (!wait_for_completion_timeout(&done, ms_time)) {
		val = gpio_get_value(mc->gpio_ipc_host_wakeup);
		if (val == HOST_WUP_LEVEL) {
			dev_err(mc->dev, "maybe complete late.. %d\n", ms_time);
			mc->l2_done = NULL;
			return MC_SUCCESS;
		}
		dev_err(mc->dev, "Modem wakeup timeout %d\n", ms_time);
		gpio_set_value(mc->gpio_ipc_slave_wakeup, 0);
		dev_info(mc->dev, "AP>>CP:  SLAV_WUP:0,%d\n",
			gpio_get_value(mc->gpio_ipc_slave_wakeup));
		mc->l2_done = NULL;
		return MC_HOST_TIMEOUT;
	}
	return MC_SUCCESS;
}
EXPORT_SYMBOL_GPL(mc_prepare_resume);

//#ifdef CONFIG_SEC_DEBUG
/*
 * HSIC CP uploas scenario -
 * 1. CP send Crash message
 * 2. Rild save the ram data to file via HSIC
 * 3. Rild call the kernel_upload() for AP ram dump
 */
static void enumeration(struct modemctl *mc)
{
	 gpio_set_value(mc->gpio_active_state, 0);
}
//#else
//static void enumeration(struct modemctl *mc) {}
//#endif

static int modem_on(struct modemctl *mc)
{
	//dev_info(mc->dev, "%s\n", __func__);
	if (!mc->ops || !mc->ops->modem_on)
		return -ENXIO;
	mc->ops->modem_on();
	return 0;
}

static int modem_off(struct modemctl *mc)
{
	dev_info(mc->dev, "%s\n", __func__);
	if (!mc->ops || !mc->ops->modem_off)
		return -ENXIO;
	msleep(10);
	mc->ops->modem_off();
	return 0;
}

static int modem_reset(struct modemctl *mc)
{
	dev_info(mc->dev, "%s\n", __func__);
	if (!mc->ops || !mc->ops->modem_reset)
		return -ENXIO;
	mc->ops->modem_reset();
	return 0;
}

static int modem_boot(struct modemctl *mc)
{
	dev_info(mc->dev, "%s\n", __func__);
	if (!mc->ops || !mc->ops->modem_boot)
		return -ENXIO;

	mc->ops->modem_boot();

	return 0;
}

static int modem_get_active(struct modemctl *mc)
{
	dev_info(mc->dev, "%s\n", __func__);
	if (!mc->gpio_active_state || !mc->gpio_cp_reset)
		return -ENXIO;

	dev_info(mc->dev, "cp %d phone %d\n",
			gpio_get_value(mc->gpio_cp_reset),
			gpio_get_value(mc->gpio_active_state));

	if (gpio_get_value(mc->gpio_cp_reset))
		return gpio_get_value(mc->gpio_active_state);

	return 0;
}

static ssize_t show_control(struct device *d,
		struct device_attribute *attr, char *buf)
{
	char *p = buf;
	struct modemctl *mc = dev_get_drvdata(d);
	struct modem_ops *ops = mc->ops;

	if (ops) {
		if (ops->modem_on)
			p += sprintf(p, "on ");
		if (ops->modem_off)
			p += sprintf(p, "off ");
		if (ops->modem_reset)
			p += sprintf(p, "reset ");
		if (ops->modem_boot)
			p += sprintf(p, "boot ");
		/*add for modem download by justy.yang 20120830*/
		    p += sprintf(p, "download ");
	} else {
		p += sprintf(p, "(No ops)");
	}

	p += sprintf(p, "\n");
	return p - buf;
}
/*add for modem download by justy.yang 20120830  start*/
static void modem_reset_download()
{
		printk("debug : start modem_reset_download\n");
	
		gpio_set_value(GPIO_CP_PMU_RST, 1);
		//gpio_set_value(GPIO_CP_RST, 0); //mask by samsung 20121211
		gpio_set_value(GPIO_PHONE_ON, 0);
		msleep(100);
		
		//gpio_set_value(GPIO_CP_RST, 1); //mask by samsung 20121211
		gpio_set_value(GPIO_CP_PMU_RST, 0);
		mdelay(2);
		gpio_set_value(GPIO_PHONE_ON, 1);
		mdelay(1);
		gpio_set_value(GPIO_PHONE_ON, 0);		

}
/*add for modem download by justy.yang 20120830  end*/

// add by samsung 20121211
static void modem_reset_gpio()
{
		//printk("debug : start modem_reset_gpio\n");
		gpio_set_value(GPIO_CP_PMU_RST, 1);
		//gpio_set_value(GPIO_CP_RST, 0);
		gpio_set_value(GPIO_PHONE_ON, 0);
		msleep(2000);
		under_reset=0;
		//gpio_set_value(GPIO_CP_RST, 1);
		gpio_set_value(GPIO_CP_PMU_RST, 0);
		mdelay(2);
		gpio_set_value(GPIO_PHONE_ON, 1);
		mdelay(1);
		gpio_set_value(GPIO_PHONE_ON, 0);	
		
		printk("debug : end modem_reset_gpio\n");
}
static void modem_reset_fota(struct modemctl *mc)
{
	struct completion done;
	unsigned long timeout;
	under_reset=1;
	//wake_lock(&mc->reset_lock);
	mc->in_l3_state =0;
	mc->ready_to_boot=1;
	mc->boot_done =0;
	mc->cpcrash_flag =0;
	mc->cpreset_flag =0;
	mc->cpdump_flag =0 ;
	mc->reset_flag = 1;
	mc->reset_count = 0;
	
	printk("%s() start!\n",__FUNCTION__);
	
#ifdef CONFIG_USB_EHCI_S5P	
	s5p_ehci_power(0);//power off hsic and remove modem devices for skip bootrom flash program
#endif
	msleep(100);
		gpio_set_value(GPIO_CP_PMU_RST, 1);
		//gpio_set_value(GPIO_CP_RST, 0);
		gpio_set_value(GPIO_PHONE_ON, 0);
		msleep(2000);
		under_reset=0;
#ifdef CONFIG_USB_EHCI_S5P	
	s5p_ehci_power(1);//power off hsic and remove modem devices for skip bootrom flash program
#endif
		msleep(500);
		//gpio_set_value(GPIO_CP_RST, 1);
		gpio_set_value(GPIO_CP_PMU_RST, 0);
		mdelay(2);
		gpio_set_value(GPIO_PHONE_ON, 1);
		mdelay(1);
		gpio_set_value(GPIO_PHONE_ON, 0);	
		
		printk("debug : end modem_reset_fota\n");

}
extern void smm6260_reset(void);

static ssize_t store_control(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct modemctl *mc = dev_get_drvdata(d);
printk("---%s():%s\n",__FUNCTION__,buf);
	if (!strncmp(buf, "testreset", 9)) {
		printk("---%s():case for test reset\n",__FUNCTION__);
		mc->boot_done = 0;
		crash_event(MODEM_EVENT_RESET); //report MODEM_EVENT_RESET 
		return count;
	}

	if (!strncmp(buf, "on", 2)) {
		modem_on(mc);
		return count;
	}

	if (!strncmp(buf, "off", 3)) {
		modem_off(mc);
		return count;
	}

	if (!strncmp(buf, "reset", 5)) {
		modem_reset_fota(mc);
		return count;
	}
/*add for modem download by justy.yang 20120830  start*/
	if (!strncmp(buf, "download", 8)) {
		printk("---%s():reset_pmu so start download modem\n",__FUNCTION__);
		mdelay(500);
		/*reset pmu should after uswitch usb to modem*/
		modem_reset_download();
		return count;
	}
/*add for modem download by justy.yang 20120830  end*/


	if (!strncmp(buf, "resettest", 9)) {
		printk("---%s():direct reset in kernel\n",__FUNCTION__);
		modem_reset_gpio();
		return count;
	}

	if (!strncmp(buf, "boot", 4)) {
		modem_boot(mc);
		return count;
	}

	if (!strncmp(buf, "renum", 6)) {
		enumeration(mc);
		return count;
	}
	if (!strncmp(buf, "phon", 4)) {
		gpio_set_value(mc->gpio_phone_on, 0);
		mdelay(1);
		gpio_set_value(mc->gpio_phone_on, 1);
		return count;
	}
	if (!strncmp(buf, "gsw=0", 5)) {
		gpio_set_value(mc->gpio_ipc_slave_wakeup, 0);
		return count;
	}
	if (!strncmp(buf, "gsw=1", 5)) {
		gpio_set_value(mc->gpio_ipc_slave_wakeup, 1);
		return count;
	}
	if (!strncmp(buf, "gat=0", 5)) {
		gpio_set_value(mc->gpio_active_state, 0);
		return count;
	}
	if (!strncmp(buf, "gat=1", 5)) {
		gpio_set_value(mc->gpio_active_state, 1);
		return count;
	}	
	return count;
}

static ssize_t show_status(struct device *d,
		struct device_attribute *attr, char *buf)
{
	char *p = buf;
	struct modemctl *mc = dev_get_drvdata(d);

	p += sprintf(p, "%d\n", modem_get_active(mc));

	return p - buf;
}

static ssize_t show_wakeup(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct modemctl *mc = dev_get_drvdata(d);
	int count = 0;

	if (!mc->gpio_ipc_host_wakeup)
		return -ENXIO;

	count += sprintf(buf + count, "%d\n",
			mc->wakeup_flag);

	return count;
}

static ssize_t store_wakeup(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct modemctl *mc = dev_get_drvdata(d);

	if (!strncmp(buf, "reset", 5)) {
		mc->wakeup_flag = HOST_WAKEUP_WAIT_RESET;
		dev_info(mc->dev, "%s: wakup_flag %d\n",
			__func__, mc->wakeup_flag);
		return count;
	}
	return 0;

}

static ssize_t show_debug(struct device *d,
		struct device_attribute *attr, char *buf)
{
	char *p = buf;
	int i;
	struct modemctl *mc = dev_get_drvdata(d);

	for (i = 0; i < ARRAY_SIZE(mc->irq); i++) {
		if (mc->irq[i])
			p += sprintf(p, "Irq %d: %d\n", i, mc->irq[i]);
	}

	p += sprintf(p, "GPIO ----\n");

	if (mc->gpio_phone_on)
		p += sprintf(p, "\t%3d %d : phone on\n", mc->gpio_phone_on,
				gpio_get_value(mc->gpio_phone_on));
	if (mc->gpio_phone_active)
		p += sprintf(p, "\t%3d %d : phone active\n",
				mc->gpio_phone_active,
				gpio_get_value(mc->gpio_phone_active));
	if (mc->gpio_pda_active)
		p += sprintf(p, "\t%3d %d : pda active\n", mc->gpio_pda_active,
				gpio_get_value(mc->gpio_pda_active));
	if (mc->gpio_cp_reset)
		p += sprintf(p, "\t%3d %d : CP reset\n", mc->gpio_cp_reset,
				gpio_get_value(mc->gpio_cp_reset));
	if (mc->gpio_usim_boot)
		p += sprintf(p, "\t%3d %d : USIM boot\n", mc->gpio_usim_boot,
				gpio_get_value(mc->gpio_usim_boot));
	if (mc->gpio_flm_sel)
		p += sprintf(p, "\t%3d %d : FLM sel\n", mc->gpio_flm_sel,
				gpio_get_value(mc->gpio_flm_sel));

	p += sprintf(p, "Support types ---\n");

	return p - buf;
}
static DEVICE_ATTR(control, 0664, show_control, store_control);
static DEVICE_ATTR(status, S_IRUGO, show_status, NULL);
static DEVICE_ATTR(wakeup, 0664, show_wakeup, store_wakeup);
static DEVICE_ATTR(debug, S_IRUGO, show_debug, NULL);

static struct attribute *modem_attributes[] = {
	&dev_attr_control.attr,
	&dev_attr_status.attr,
	&dev_attr_wakeup.attr,
	&dev_attr_debug.attr,
	NULL
};

static const struct attribute_group modem_group = {
	.attrs = modem_attributes,
};

void crash_event(int type)
{
	char *envs[2] = { NULL, NULL };

	if (!global_mc)
		return;

	switch (type)
	{

	case MODEM_EVENT_RESET:
		envs[0] = "MAILBOX=cp_reset";
		kobject_uevent_env(&global_mc->dev->kobj, KOBJ_OFFLINE, envs);
		wake_up_interruptible(&global_mc->wq);
		printk("%s: MODEM_EVENT_RESET\n", __func__);
		break;
	case MODEM_EVENT_DUMP:	
		envs[0] = "MAILBOX=cp_dump";
		kobject_uevent_env(&global_mc->dev->kobj, KOBJ_OFFLINE, envs);
		wake_up_interruptible(&global_mc->wq);
		printk("%s: MODEM_EVENT_DUMP\n", __func__);
		break;
	case MODEM_EVENT_CONNECT:
		envs[0] = "MAILBOX=cp_connect";
		kobject_uevent_env(&global_mc->dev->kobj, KOBJ_ONLINE, envs);
		wake_up_interruptible(&global_mc->wq);
		printk("%s: MODEM_EVENT_CONNECT\n", __func__);
		break;
	case MODEM_EVENT_CRASH:
		envs[0] = "MAILBOX=cp_crash";
		kobject_uevent_env(&global_mc->dev->kobj, KOBJ_OFFLINE, envs);
		wake_up_interruptible(&global_mc->wq);
		printk("%s: MODEM_EVENT_CRASH\n", __func__);
		break;
	case MODEM_EVENT_RESET_DONE:
		msleep(10000);
		wake_unlock(&global_mc->reset_lock);
		envs[0] = "MAILBOX=cp_reset_done";
		kobject_uevent_env(&global_mc->dev->kobj, KOBJ_OFFLINE, envs);
		wake_up_interruptible(&global_mc->wq);
		printk("%s: MODEM_EVENT_RESET_DONE\n", __func__);
		break;
	}
}


#ifdef CONFIG_USB_EHCI_S5P	
extern int s5p_ehci_power(int value);
#endif

extern u8 suspending_flag;
static irqreturn_t modem_resume_thread(int irq, void *dev_id)
{
	struct modemctl *mc = (struct modemctl *)dev_id;
	int val = gpio_get_value(mc->gpio_ipc_host_wakeup);
	int err;
	if(under_reset){
		printk("when modem reset ,skip interrupt before power on modem --------\n");
		return IRQ_HANDLED;
		}
#ifdef HSIC_DEBUG
	dev_info(mc->dev, "CP>>AP:  HOST_WUP:%d\n", val);
#endif
		if(suspending_flag == 1)
		{
			printk("IRQ handle skip: suspending!\n");
			return IRQ_HANDLED;
		}
	if((mc->ready_to_boot==1)&&(mc->boot_done == 0) && (val == HOST_WUP_LEVEL)){
			printk("\n ---%s():power on modem success\n",__FUNCTION__);
			mc->ready_to_boot=0;
			mc->boot_done =1;
			#ifdef CONFIG_USB_EHCI_S5P	
				s5p_ehci_power(1);//power on hsic
			#endif
			//printk("---%s():prepare to simple enumeration!\n",__FUNCTION__);

			return IRQ_HANDLED;
		}

	if (val != HOST_WUP_LEVEL) {
		if(mc->in_l3_state==1){
			if(gpio_get_value(mc->gpio_ipc_slave_wakeup)) {
				gpio_set_value(mc->gpio_ipc_slave_wakeup, 0);
#ifdef HSIC_DEBUG
				dev_info(mc->dev, "AP>>CP:	SLAV_WUP:0,%d\n",
				gpio_get_value(mc->gpio_ipc_slave_wakeup));
#endif
			}
			mc->in_l3_state=0;
#ifdef HSIC_DEBUG
			printk("---HOST_WUP_LEVEL is 1 means L3 to L0 transfer over\n ");
#endif

			return IRQ_HANDLED;
		}
		else{
			if (mc->l2_done) {
				complete(mc->l2_done);
				mc->l2_done = NULL;
			}
			if(gpio_get_value(mc->gpio_ipc_slave_wakeup)) {
				gpio_set_value(mc->gpio_ipc_slave_wakeup, 0);
#ifdef HSIC_DEBUG
				dev_info(mc->dev, "AP>>CP:	SLAV_WUP:0,%d\n",
				gpio_get_value(mc->gpio_ipc_slave_wakeup));
#endif
			}
			mc->debug_cnt = 0;
			return IRQ_HANDLED;
		}
	}
	
	if((mc)&&(mc->in_l3_state==1)&&(val == HOST_WUP_LEVEL)){
		if ((mc->in_l3_state==1)) {			//xujie temp
		#if (0) 
		{
			if (mc->l3_done) {                   
				complete(mc->l3_done);
				mc->l3_done = NULL;
			}
			printk("---use l3_done for L3 to L0 transfer\n ");
		}
		#endif
			return IRQ_HANDLED;
		}
	}

	if (val == HOST_WUP_LEVEL) {
		err = acm_request_resume();
		if (err < 0)
			dev_err(mc->dev, "request resume failed: %d\n", err);
			mc->debug_cnt++;
	}
	
	if (mc->debug_cnt > 30) {
		dev_err(mc->dev, "Abnormal Host wakeup -- over 30times");
		//disable_irq(irq);
		mc->debug_cnt = 0;
		if(mc->boot_done)
		{
			mc->cpcrash_flag = 1;
			crash_event(MODEM_EVENT_CRASH);
		}		
	}

	if (!val
		&& mc->wakeup_flag == HOST_WAKEUP_WAIT_RESET) {
		mc->wakeup_flag = HOST_WAKEUP_LOW;
		dev_info(mc->dev, "%s: wakeup flag (%d)\n",
			__func__, mc->wakeup_flag);
	}

	return IRQ_HANDLED;
}
static irqreturn_t modem_cpreset_irq(int irq, void *dev_id)
{
	struct modemctl *mc = (struct modemctl *)dev_id;
	if(mc->boot_done) {
		wake_lock_timeout(&mc->reset_lock, HZ*30);
		}
	else
		return IRQ_HANDLED;
	if (!work_pending(&mc->cpreset_work))
		schedule_work(&mc->cpreset_work);

	return IRQ_HANDLED;
}

static void _free_all(struct modemctl *mc)
{
	int i;

	if (mc) {
		if (mc->ops)
			mc->ops = NULL;

		if (mc->group)
			sysfs_remove_group(&mc->dev->kobj, mc->group);

		for (i = 0; i < ARRAY_SIZE(mc->irq); i++) {
			if (mc->irq[i])
				free_irq(mc->irq[i], mc);
		}

		kfree(mc);
	}
}
int modem_open (struct inode *inode, struct file *file)
{
	return 0;
}
int modem_close (struct inode *inode, struct file *file)
{
	return 0;
}

#ifdef CONFIG_USB_SERIAL_IMC
extern int g_uart_enabled;
#endif

int modem_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	printk("%s, cmd: 0x%x\n",__FUNCTION__, cmd);
	if(!global_mc)
		{
			printk("[XJ] no global_mc!\n");
			return -1;
		}
	switch(cmd)
	{
		case MODEM_POWER_MAIN_CMD:
			modem_main_enumeration(global_mc);
			break;
		case MODEM_POWER_FLASH_CMD:
			modem_boot_enumeration(global_mc);
			break;
		case MODEM_POWER_OFF_CMD:
			modem_off(global_mc);
			break;
		case MODEM_POWER_ON_CMD:
			modem_on(global_mc);
			break;
		case MODEM_POWER_RESET_CMD:
		#ifdef CONFIG_USB_SERIAL_IMC
			if (!g_uart_enabled){
				return 0;
			}
		#endif
			wake_lock(&cp_reset_wake_lock);// add by justy.yang 20121205
			modem_reset(global_mc);
			wake_lock_timeout(&cp_reset_wake_lock,10*HZ);// add by justy.yang 20121205
			
			break;	
		case MODEM_POWER_DOWNLOAD_CMD:
			modem_reset_fota(global_mc);
			break;		
	}
	return 0;
}

static ssize_t modem_read (struct file *filp, char __user * buffer, size_t count, loff_t * offset)
{
	int flag = 0;
	
	printk("%s: call\n", __func__);
	if(!global_mc)
		return -EFAULT;
	
	wait_event_interruptible(global_mc->wq, (global_mc->cpcrash_flag || global_mc->cpreset_flag || global_mc->cpdump_flag));
	
	flag = (global_mc->cpcrash_flag << CRASH_STATE_OFFSET) |\
		(global_mc->cpreset_flag << RESET_STATE_OFFSET) |\
		(global_mc->cpdump_flag << DUMP_STATE_OFFSET);
	printk("%s: modem event = 0x%x   1\n", __func__, flag);
	if(copy_to_user(buffer, &flag, sizeof(flag)))
		return -EFAULT;
	global_mc->boot_done =0;
	global_mc->cpcrash_flag =0;
	global_mc->cpreset_flag =0;
	global_mc->cpdump_flag =0 ;	
	printk("%s: modem event = 0x%x   2\n", __func__, flag);
	return 1;
}
static ssize_t modem_write (struct file *filp, const char __user *buffer, size_t count, loff_t *offset)
{
	if(!global_mc 
#ifdef CONFIG_USB_SERIAL_IMC
		|| !g_uart_enabled
#endif
		)
		return -1;

	if(count >= 4 && !strncmp(buffer, "main", 4))
	{
		modem_main_enumeration(global_mc);
	}
	if(count >= 5 && !strncmp(buffer, "flash", 5))
	{
		modem_boot_enumeration(global_mc);
	}
	if(count >= 3 && !strncmp(buffer, "off", 3))
	{
		modem_off(global_mc);
	}
	if(count >= 2 && !strncmp(buffer, "on", 2))
	{
		modem_on(global_mc);
	}	
	if(count >= 5 && !strncmp(buffer, "reset", 5))
	{
		modem_reset(global_mc);
	}	
	return count;
}

static struct file_operations modem_file_ops = {
	.owner = THIS_MODULE,
	.open = modem_open,
	.release = modem_close,
	.read = modem_read,
	.write = modem_write,
	.unlocked_ioctl = modem_ioctl,
};

static struct miscdevice modem_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "modemctl",
	.fops = &modem_file_ops
};
static int modem_boot_enumeration(struct modemctl *mc)
{
//	struct completion done;

	wake_lock(&mc->reset_lock);	
	mc->boot_done =0;
	mc->cpcrash_flag =0;
	mc->cpreset_flag =0;
	mc->cpdump_flag =0 ;
	mc->boot_done =1;
	wake_unlock(&mc->reset_lock);	
	return 0;	
}

static int modem_main_enumeration(struct modemctl *mc)
{
	struct completion done;
	unsigned long timeout;
	//wake_lock(&mc->reset_lock);
	mc->in_l3_state =0;
	mc->ready_to_boot =0;
	mc->boot_done =0;
	mc->cpcrash_flag =0;
	mc->cpreset_flag =0;
	mc->cpdump_flag =0 ;

	#ifdef CONFIG_USB_EHCI_S5P	
		s5p_ehci_power(0);//power off hsic and remove modem devices for skip bootrom flash program
		gpio_set_value(mc->gpio_ipc_slave_wakeup, 0);
#ifdef HSIC_DEBUG
		dev_info(mc->dev, "AP>>CP:	SLAV_WUP:0,%d\n",
		gpio_get_value(mc->gpio_ipc_slave_wakeup));
#endif
//	smm6260_set_active_state(1);
/*modify by justy.yang 20121129. change modem_on from  ril ioctol cmd to here*/
	modem_on(mc);//power on modem
	#endif
		//printk("---the first time simple enumeration for c2c\n");
	
		mc->ready_to_boot=1;
		
	//wake_unlock(&mc->reset_lock);
	return 0;
}

static int modem_reset_reintinalize(struct modemctl *mc)
{
	struct completion done;
	unsigned long timeout;
	under_reset=1;
	pm_runtime_set_active(s5p_dev);
	pm_runtime_disable(s5p_dev);
	//wake_lock(&mc->reset_lock);
	mc->in_l3_state =0;
	mc->ready_to_boot=1;
	mc->boot_done =0;
	mc->cpcrash_flag =0;
	mc->cpreset_flag =0;
	mc->cpdump_flag =0 ;
	mc->reset_flag = 1;
	mc->reset_count = 0;
	
	//printk("[XJ] %s() start!\n",__FUNCTION__);
	
#ifdef CONFIG_USB_EHCI_S5P	
	s5p_ehci_power(0);//power off hsic and remove modem devices for skip bootrom flash program
#endif
	modem_reset_gpio();
	return 0;
	
}

void do_modem_reset(){
	printk("---%s()\n",__func__);
	modem_reset_reintinalize(global_mc);
}
static int __devinit modem_probe(struct platform_device *pdev)
{
	struct modem_platform_data *pdata = pdev->dev.platform_data;
	struct device *dev = &pdev->dev;
	struct modemctl *mc;
	int irq;
	int error;
	if (!pdata) {
		dev_err(dev, "No platform data\n");
		return -EINVAL;
	}

	mc = kzalloc(sizeof(struct modemctl), GFP_KERNEL);
	if (!mc) {
		dev_err(dev, "Failed to allocate device\n");
		return -ENOMEM;
	}
	/////Added by lisw for modem reset function //////
	mc->gModemPowerState=0;
	mc->gCdcAcmSimpleEnumeratinoState=0;
	init_timer(&mc->reset_judge_timer);
	//mc->reset_judge_timer.function = reset_judge_timer_func;
	mc->reset_judge_timer.data = (unsigned long) mc;
	///////////////END//////////////////////////
	mc->gpio_phone_on = pdata->gpio_phone_on;
	mc->gpio_phone_active = pdata->gpio_phone_active;
	mc->gpio_pda_active = pdata->gpio_pda_active;
	mc->gpio_cp_reset = pdata->gpio_cp_reset;
	mc->gpio_cp_req_reset = pdata->gpio_cp_req_reset;
	mc->gpio_ipc_slave_wakeup = pdata->gpio_ipc_slave_wakeup;
	mc->gpio_ipc_host_wakeup = pdata->gpio_ipc_host_wakeup;
	mc->gpio_suspend_request = pdata->gpio_suspend_request;
	mc->gpio_active_state = pdata->gpio_active_state;
	mc->gpio_usim_boot = pdata->gpio_usim_boot;
	mc->gpio_flm_sel = pdata->gpio_flm_sel;
	//mc->gpio_cp_reset_int = pdata->gpio_cp_reset_int;
		
	//mc->gpio_cp_dump_int = pdata->gpio_cp_dump_int;
	mc->ops = &pdata->ops;
	mc->dev = dev;
	dev_set_drvdata(mc->dev, mc);

	error = sysfs_create_group(&mc->dev->kobj, &modem_group);
	if (error) {
		dev_err(dev, "Failed to create sysfs files\n");
		goto fail;
	}
	mc->group = &modem_group;

	//INIT_DELAYED_WORK(&mc->work, mc_work);
	//INIT_WORK(&mc->cpreset_work, mc_cpreset_worker);
	//INIT_WORK(&mc->do_reset_work, do_reset_worker);
	wake_lock_init(&mc->reset_lock, WAKE_LOCK_SUSPEND, "modemctl");
	init_waitqueue_head(&mc->wq);

//add wake_lock by justy 20121205 for when modem reset system can not enter to sleep
	wake_lock_init(&cp_reset_wake_lock, WAKE_LOCK_SUSPEND, "cp_reset_wake_lock");
	
//mask by justy 20121116 for power on phone too long  start
//#ifdef CONFIG_USB_EHCI_S5P	
	//s5p_ehci_power(0);
//#endif
//mask by justy 20121116 for power on phone too long  end
	mc->ops->modem_cfg();
#if 0
	irq = gpio_to_irq(pdata->gpio_suspend_request);

	error = request_irq(irq, modem_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"phone_request_suspend", mc);
	if (error) {
		dev_err(dev, "Active Failed to allocate an interrupt(%d)\n", irq);
		goto fail;
	}
	mc->irq[0] = irq;
	enable_irq_wake(irq);
#endif
	irq = gpio_to_irq(pdata->gpio_ipc_host_wakeup);

	error = request_threaded_irq(irq, NULL, modem_resume_thread,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"IPC_HOST_WAKEUP", mc);

	if (error) {
		dev_err(dev, "Resume thread Failed to allocate an interrupt(%d)\n", irq);
		goto fail;
	}
	mc->irq[1] = irq;
	enable_irq_wake(irq);
	
#if 0
	irq = gpio_to_irq(pdata->gpio_cp_reset_int);
	error = request_threaded_irq(irq, NULL, modem_cpreset_irq,
			IRQF_TRIGGER_RISING,
			"CP_RESET_INT", mc);

	if (error) {
		dev_err(dev, "CP reset report Failed to allocate an interrupt(%d)\n", irq);
		goto fail;
	}
	mc->irq[2] = irq;
	#endif
	//enable_irq_wake(irq);	

	mc->debug_cnt = 0;

	device_init_wakeup(&pdev->dev, pdata->wakeup);
	platform_set_drvdata(pdev, mc);
	global_mc = mc;

	error = misc_register(&modem_miscdev);
	if(error)
	{
		dev_err(dev, "Failed to register modem control device\n");
		goto fail;
	}
#ifdef CONFIG_MODEM_BOOT_IN_UBOOT
mc->boot_done =1;
printk("--- modem boot done in uboot \n");
#else
	modem_main_enumeration(mc);//enumerate modem devices
#endif

	return 0;

fail:
	_free_all(mc);
	return error;
}

static int __devexit modem_remove(struct platform_device *pdev)
{
	struct modemctl *mc = platform_get_drvdata(pdev);

	flush_work(&mc->work.work);
	flush_work(&mc->cpreset_work);
	platform_set_drvdata(pdev, NULL);
	wake_lock_destroy(&mc->reset_lock);

	misc_deregister(&modem_miscdev);
	_free_all(mc);
	return 0;
}

#ifdef CONFIG_PM
static int modem_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct modemctl *mc = platform_get_drvdata(pdev);

	if (mc->ops && mc->ops->modem_suspend)
		mc->ops->modem_suspend();

	if (device_may_wakeup(dev) && smm6260_is_on())
		enable_irq_wake(mc->irq[1]);

	return 0;
}

static int modem_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct modemctl *mc = platform_get_drvdata(pdev);

	if (device_may_wakeup(dev) && smm6260_is_on())
		disable_irq_wake(mc->irq[1]);

	if (mc->ops && mc->ops->modem_resume)
		mc->ops->modem_resume();

	return 0;
}

static const struct dev_pm_ops modem_pm_ops = {
	.suspend	= modem_suspend,
	.resume		= modem_resume,
};
#endif

static struct platform_driver modem_driver = {
	.probe		= modem_probe,
	.remove		= __devexit_p(modem_remove),
	.driver		= {
		.name	= "smm_modem",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &modem_pm_ops,
#endif
	},
};

static int __init modem_init(void)
{
	platform_driver_register(&modem_driver);
	return 0; 
}

static void __exit modem_exit(void)
{
	platform_driver_unregister(&modem_driver);
}
//module_init(modem_init);
late_initcall(modem_init);

module_exit(modem_exit);
