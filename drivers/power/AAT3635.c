/* drivers/i2c/chips/aat3635.c
 *
 * Copyright (C) 2009 HTC Corporation
 * Author: Josh Hsiao <Josh_Hsiao@htc.com>
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/mach-types.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux//power/aat3635_battery.h>
#include <linux/power_supply.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>
#include <linux/android_alarm.h>
#include <linux/interrupt.h>
#include <plat/gpio-cfg.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/kdev_t.h>
#include <linux/input.h>

struct mutex lcd_on_or_off;

//backlight
#define MAX_BL_LEVEL 26
#define MAX_BRIGHTNESS		(0xff)
#define MIN_BRIGHTNESS		(0)

#define pr_aat3635_fmt(fmt) "[BATT][aat3635] " fmt
#define pr_aat3635_err_fmt(fmt) "[BATT][aat3635] err:" fmt
#define pr_aat3635_warn_fmt(fmt) "[BATT][aat3635] warn:" fmt
#define pr_aat3635_info(fmt, ...) \
	printk(KERN_INFO pr_aat3635_fmt(fmt), ##__VA_ARGS__)
#define pr_aat3635_err(fmt, ...) \
	printk(KERN_ERR pr_aat3635_err_fmt(fmt), ##__VA_ARGS__)
#define pr_aat3635_warn(fmt, ...) \
	printk(KERN_WARNING pr_aat3635_warn_fmt(fmt), ##__VA_ARGS__)

/* used for debug if function called */
#define FUNC_CALL_CHECK 0

#ifdef CONFIG_CELLON_ENG_LOG
#define AAT_LOG_ON 1
#else 
#define AAT_LOG_ON 0
#endif

#if FUNC_CALL_CHECK
#define CHECK_LOG() pr_aat3635_info("%s\n", __func__)
#else
#define CHECK_LOG() (void)0
#endif

#define POLLING_TIME_MSEC 5000

static struct aat3635_chg_int_data *chg_int_data;

static struct aat3635_chg_int_data *chg_status;

//static struct workqueue_struct *aat3635_wq;

//static struct work_struct aat3635_work;

static unsigned int chg_stat_enabled;
//static spinlock_t chg_stat_lock;
static struct wake_lock chg_wake_lock;
static struct timer_list temp_dect_timer;
static struct delayed_work chg_otp_work;
struct class *aat3635_class;

//static int system_suspended = 0; /*indicate whether the system still is in the suspended status*/
static bool timer_out=0;
//static struct work_struct state_wq;
//static struct timer_list polling_timer;
 struct timer_list chg_dect_timer;

static struct aat3635_chip *aat_this_chip = NULL;
static bool first_time_on =0;

//static int aat3635_dump_register(void);
static int aat3635_i2c_rx_data(char *buf, int len);
static int aat3635_i2c_tx_data(char *buf, int len);

//static void aat3635_update_status(void);
static int aat3635_cable_status_update(int status);
static void aat3635_charger_work_func(void);
static int aat3635_set_charger_ctrl(u32 ctl);
//void WLED_ctrl(bool ctrl,u32 level);

static int aat3635_initial = -1;
static int aat3635_low_chg;
int charger_type;
u8 	charger_status;
static int  temp_fault;
extern struct aat3635_platform_data aat3635_priv;

extern struct platform_device s3c_device_usbgadget;

extern void lcd_off(void);
extern void lcd_on(void);
static bool bl_on=1;

/*set the charger parameters for differnent mode*/

static void update_aat3635_charger(charger_type_t type)
{
	#if AAT_LOG_ON
	printk("update_aat3635_charger###%d\n",type);
	#endif
	switch(type)
	{
		case CHARGER_AC:
			aat3635_set_charger_ctrl(CHARGER_AC);
			break;
		case CHARGER_USB:
			aat3635_set_charger_ctrl(CHARGER_USB);
			break;
		case CHARGER_BATTERY:
			aat3635_set_charger_ctrl(CHARGER_BATTERY);
			break;
		default:
			break;
	}	
}

static void aat3635_charger_type_work_func(void)
{
	charger_type_t status = 0;	
	int usb_sel_value,gpio_value=0;

	gpio_value = gpio_get_value(EXYNOS4_GPX0(4));
	usb_sel_value = gpio_get_value(EXYNOS4_GPK3(2));
	#if AAT_LOG_ON
	printk("gpio_value@@@==%d,usb_sel_value=%d\n",gpio_value,usb_sel_value);
	#endif
	if(gpio_value>0)
	{
		if((charger_type ==1)||(usb_sel_value==1))
			status = CHARGER_USB;
		else
			status = CHARGER_AC;
	}
	else
	{
		status = CHARGER_BATTERY;
	}

	//printk("status=%d--\n",status);
	if((status == aat_this_chip->battery_info.charging_source)&&(timer_out==0))
		return;
	//aat_this_chip->battery_info.charging_source = status;

	update_aat3635_charger(status);
	aat3635_cable_status_update(status);		
}

extern int bat_temp;

static void chg_otp_work_func(void)
{
	charger_type_t type; 	
	int gpio_value=0;
	
       //printk("chg_otp_work_func********\n");
	if(bat_temp<-10 ||bat_temp>55)	
	{		
		aat_this_chip->battery_info.health = POWER_SUPPLY_HEALTH_OVERHEAT;		   	
		type = CHARGER_BATTERY;		
		temp_fault =1;	
	}	
	else if(temp_fault)	
	{	   
		temp_fault=0;
		gpio_value = gpio_get_value(EXYNOS4_GPX0(4));
		if(gpio_value==1)
		{
			if(charger_type ==1)
				type = CHARGER_USB;
			else 
				type = CHARGER_AC;
		}
		else 
			type = CHARGER_BATTERY;	
		aat_this_chip->battery_info.health =POWER_SUPPLY_HEALTH_GOOD;	   		
		//type = old_type;			
	}	
	else		
		goto out;	
	
	update_aat3635_charger(type);	
	aat3635_cable_status_update(type);      
out:
	mod_timer(&temp_dect_timer,jiffies + (60*HZ));	
}

static void aat_otp_timer_func(unsigned long data)
{	
	//printk("aat_temp_timer_func******************\n");
	
	schedule_delayed_work(&chg_otp_work, msecs_to_jiffies(10));
 		  
}

extern  void exynos_usb_mux_change(struct platform_device *pdev, int val);
//extern void SimulateKey(int keycode,int DownFlag);
extern int bat_temp;
static void aat3635_charger_work_func(void)
{
	charger_type_t status = 0;
	int gpio_value=0;
       //first_time_on = 0;
       
	gpio_value = gpio_get_value(EXYNOS4_GPX0(4));
	   #if AAT_LOG_ON
	printk("gpio_value==%d\n",gpio_value);
	   #endif
	if(gpio_value>0 && (bat_temp>-10 && bat_temp<55))
	{
		wake_lock(&chg_wake_lock);
		schedule_delayed_work(&chg_status->chg_type_work, msecs_to_jiffies(5000));
		status = CHARGER_USB;
		
		exynos_usb_mux_change(&s3c_device_usbgadget, 0);
		
		update_aat3635_charger(status);
	       aat3635_cable_status_update(status);	
	}
	else
	{
		charger_type =0;
		status = CHARGER_BATTERY;
		gpio_set_value(EXYNOS4_GPL2(3),0);
		//mdelay(500);
		msleep(500);	//modify by Samsung AVP Envi for break of A2DP playback in plugging off usb
		exynos_usb_mux_change(&s3c_device_usbgadget, 1);
			
		update_aat3635_charger(status);
		aat3635_cable_status_update(status);	
		wake_unlock(&chg_wake_lock);
	}

	//if(status == aat_this_chip->battery_info.charging_source)
	//	return;
	
}


 extern int irq_set_irq_type(unsigned int irq, unsigned int type);
irqreturn_t aat3635_irq_handler(int irqno, void *param)
{
	//int gpio_value;
	#if AAT_LOG_ON
	printk("*********aat3635_irq_handler\n");
	#endif
	/*
	if(gpio_value)
	{
		irq_set_irq_type(gpio_to_irq(EXYNOS4_GPX0(4)),IRQF_TRIGGER_LOW);
	}
	else 
	{
		irq_set_irq_type(gpio_to_irq(EXYNOS4_GPX0(4)),IRQF_TRIGGER_HIGH);
	}
	*/
	
	schedule_delayed_work(&chg_status->chg_int_work, msecs_to_jiffies(0));
	
	return IRQ_HANDLED; 

}

static enum power_supply_property aat3635_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

static DEFINE_MUTEX(work_lock);

static int aat3635_power_get_usb_property(struct power_supply *bat_ps, 
		enum power_supply_property psp, 
		union power_supply_propval *val)
{
	charger_type_t charger = aat_this_chip->battery_info.charging_source;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if(charger==CHARGER_USB)
			val->intval = 1;//(charger==CHARGER_USB);
		else
			val->intval = 0;
		//printk("aat3635_power_get_usb_property--%d\n",val->intval);
		break;
	default:
		return -EINVAL;
	}
	
	return 0;
}

static int aat3635_power_get_ac_property(struct power_supply *bat_ps, 
		enum power_supply_property psp, 
		union power_supply_propval *val)
{
	charger_type_t charger = aat_this_chip->battery_info.charging_source;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if(charger==CHARGER_AC)
			val->intval = 1;
		else
			val->intval = 0;
		//printk("aat3635_power_get_ac_property--%d\n",val->intval);
		break;
	default:
		return -EINVAL;
	}
	
	return 0;
}
static struct power_supply aat3635_power_supplies[] = {
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = aat3635_power_properties,
		.num_properties = ARRAY_SIZE(aat3635_power_properties),
		.get_property = aat3635_power_get_usb_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = aat3635_power_properties,
		.num_properties = ARRAY_SIZE(aat3635_power_properties),
		.get_property = aat3635_power_get_ac_property,
	},
};

static int aat3635_cable_status_update(int status)
{
	int ret = 0;

	if(!aat3635_initial)
		return -EPERM;

	switch(status) {
	case CHARGER_BATTERY:
		aat_this_chip->battery_info.charging_source = CHARGER_BATTERY;
		aat_this_chip->battery_info.status= POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case CHARGER_USB:
		aat_this_chip->battery_info.charging_source = CHARGER_USB;
		aat_this_chip->battery_info.status= POWER_SUPPLY_STATUS_CHARGING;
		break;
	case CHARGER_AC:
		aat_this_chip->battery_info.charging_source = CHARGER_AC;
		aat_this_chip->battery_info.status= POWER_SUPPLY_STATUS_CHARGING;
		break;
	case CHARGER_DISCHARGE:	
		aat_this_chip->battery_info.charging_source = CHARGER_DISCHARGE;
		aat_this_chip->battery_info.status= POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		ret = -EINVAL;
	}


	/* if the power source changes, all power supplies may change state */
	power_supply_changed(&aat3635_power_supplies[0]);
	power_supply_changed(&aat3635_power_supplies[1]);
	//power_supply_changed(&aat3635_power_supplies[2]);
	return ret;
}


/**
 * Insmod parameters
 */
/* I2C_CLIENT_INSMOD_1(aat3635); */

static int aat3635_probe(struct i2c_client *client,
			const struct i2c_device_id *id);
static int aat3635_remove(struct i2c_client *client);

//static struct aat3635_i2c_client aat3635_i2c_module;
int  is_charger_online(void)//mj for charging detect.
{
	//struct fg8997_chip *chip = aat_this_chip;
	return  (aat_this_chip->battery_info.charging_source == CHARGER_USB||aat_this_chip->battery_info.charging_source == CHARGER_AC);
}

static int aat3635_i2c_rx_data(char *buf, int len)
{	
	struct i2c_client *aat_client = aat_this_chip->client;	
	struct i2c_msg msgs[] = {		
		{	.addr	= aat_client->addr,		
			.flags	= 0,			
			.len	= 1,			
			.buf	= buf,		},		
		{			
			.addr	= aat_client->addr,			
			.flags	= I2C_M_RD,			
			.len	= len,			
			.buf	= buf,		
		}	
	};	
	if(NULL == aat_client || NULL == aat_client->addr)		
		return -EIO;
	i2c_transfer(aat_client->adapter, msgs, 2);
	
		return 0;
}
static int aat3635_i2c_tx_data(char *buf, int len)
{	
	
	struct i2c_client *aat_client = aat_this_chip->client;	
	struct i2c_msg msg[] = {		
		{			
			.addr	= aat_client->addr,			
			.flags	= 0,			
			.len	= len,			
			.buf	= buf,		
		}	
	};	
	//printk("aat3635_i2c_tx_data------start -------,times = %lx \n",jiffies);	
	if(NULL == aat_client || NULL == aat_client->addr)		
		return -EIO;	
		i2c_transfer(aat_client->adapter, msg, 1) ;
					
		//printk("aat3635_i2c_tx_data------end -------,times = %lx \n",jiffies);		
	return 0;
}

void aat3635_LED_ctrl(bool ctl)
{
	u8 reg[2];
	u8 led_status;
	reg[0]=0x00;
	aat3635_i2c_rx_data(reg,1);
	led_status = reg[0];
	//printk("aat3635_LED_ctrl --%d,LED_status--%x\n",ctl,led_status);
	if(ctl)
	{
		led_status = led_status & 0x7F;
		led_status = led_status | 0x40;
		//printk("LED_status ON--%x\n",led_status);
		reg[0]=0x00;
		reg[1]=led_status;
		aat3635_i2c_tx_data(reg,2);
	}
	else
	{
		led_status = led_status | 0x80;
		led_status = led_status &  0xBF;
		//printk("LED_status OFF--%x\n",led_status);
		reg[0]=0x00;
		reg[1]=led_status;
		aat3635_i2c_tx_data(reg,2);
	}
}

void WLED_ctrl(bool ctrl,u8 level)
{
	u8 reg[2];
	u8 wled_status;
	//printk("WLED_ctrl---%d\n",ctrl);
	reg[0]=0x00;
	aat3635_i2c_rx_data(reg,1);
	wled_status = reg[0];
	//printk("WLED_status--%x\n",wled_status);	
	if(ctrl)
	{
		wled_status = wled_status | 0x10;
		reg[0]=0x00;
		reg[1]=wled_status;
		aat3635_i2c_tx_data(reg,2);
		//printk("PWM level--%x\n",level);
		reg[0]=0x05;
		reg[1]=level;
		aat3635_i2c_tx_data(reg,2);
	}
	else
	{
		wled_status = wled_status & 0xEF;
		reg[0]=0x00;
		reg[1]=wled_status;
		aat3635_i2c_tx_data(reg,2);
		reg[0]=0x05;
		reg[1]=level;
		aat3635_i2c_tx_data(reg,2);
	}
/*	
	reg[0]=0x00;
	aat3635_i2c_rx_data(reg,1);
	wled_status = reg[0];
	printk(" check WLED_ctrl --%d,WLED_status--%x\n",ctrl,wled_status);	
*/
}

static u8 aat3635_get_id(void)
{
	u8 nRegValue;
	u8 int_reg[2];
	int_reg[0]=0x0E;
	 aat3635_i2c_rx_data(int_reg,1);
	nRegValue = int_reg[0];	
	//printk("aat3635_get_id---%x\n",nRegValue);
	return nRegValue;
}

static void aat3635_mask_DPM(void)
{
	u8 nRegValue;
	u8 int_reg[2];
	int_reg[0]=0x0C;
	 aat3635_i2c_rx_data(int_reg,1);
	nRegValue = int_reg[0];
	nRegValue = nRegValue |0xD1;
	int_reg[0]=0x0C;
	int_reg[1]=nRegValue;
	aat3635_i2c_tx_data(int_reg,2);
	
}

extern int bat_ID_value;
static int aat3635_set_charger_ctrl(u32 ctl)
{
	int result = 0;
	u8 nRegValue;
	u8 reg_dis[2],reg_fast[2],reg_slow[2],test[2];

	CHECK_LOG();
	//aat3635_mask_timerout();
	if (aat3635_initial < 0)
		return 0;
	#if AAT_LOG_ON
      printk("aat3635_set_charger_ctrl--%d\n",ctl);
	#endif
	switch (ctl) {
	case POWER_SUPPLY_DISABLE_CHARGE:
		//pr_aat3635_info("Switch charger OFF\n");
		reg_dis[0]=0x00;
		//aat3635_i2c_tx_data(&reg_dis,1);
		aat3635_LED_ctrl(0);
		aat3635_i2c_rx_data(reg_dis,1);
		nRegValue = reg_dis[0];
		#if AAT_LOG_ON
		printk("nRegValue--%x\n",nRegValue);
		#endif
		nRegValue &= ~(0x03);//CH_EN = 0
		reg_dis[0]=0x00;
		reg_dis[1]=nRegValue;
		aat3635_i2c_tx_data(reg_dis,2);
		break;
	case POWER_SUPPLY_ENABLE_SLOW_CHARGE:
		reg_slow[0]=0x01;
		reg_slow[1]=0x93;
		aat3635_i2c_tx_data(reg_slow,2);
		reg_slow[0]=0x02;
		if((bat_ID_value>1820 && bat_ID_value<2275)||(bat_ID_value>683 && bat_ID_value<1365))
			reg_slow[1]=0x6C;
		else 
			reg_slow[1]=0x64;
		aat3635_i2c_tx_data(reg_slow,2);
		reg_slow[0]=0x03;
		reg_slow[1]=0x14;
		aat3635_i2c_tx_data(reg_slow,2);
		reg_slow[0]=0x04;
		if((bat_ID_value>1820 && bat_ID_value<2275)||(bat_ID_value>683 && bat_ID_value<1365))
			reg_slow[1]=0x28;
		else 
			reg_slow[1]=0x20;
		aat3635_i2c_tx_data(reg_slow,2);

		if(timer_out)
		{
			test[0]=0x00;
			aat3635_i2c_rx_data(test,1);
			//printk("test of 0x00 is %x\n",test[0]);
			test[0] = test[0]|0x03;
			#if AAT_LOG_ON
			printk("***test of 0x00 is %x\n",test[0]);
			#endif
			reg_slow[0]=0x00;
			reg_slow[1]=test[0];
			aat3635_i2c_tx_data(reg_slow,2);	
		}
		else
		{
			reg_slow[0]=0x00;
			reg_slow[1]=0x8B;
			aat3635_i2c_tx_data(reg_slow,2);
		}
		#if AAT_LOG_ON
		reg_slow[0]=0x01;
		aat3635_i2c_rx_data(reg_slow,1);
		printk("reg_slow[1]=%x\n",reg_slow[0]);
		reg_slow[0]=0x02;
		aat3635_i2c_rx_data(reg_slow,1);
		printk("reg_slow[2]=%x\n",reg_slow[0]);
		reg_slow[0]=0x03;
		aat3635_i2c_rx_data(reg_slow,1);
		printk("reg_slow[3]=%x\n",reg_slow[0]);		
		reg_slow[0]=0x04;
		aat3635_i2c_rx_data(reg_slow,1);
		printk("reg_slow[4]=%x\n",reg_slow[0]);	
		reg_slow[0]=0x00;
		aat3635_i2c_rx_data(reg_slow,1);
		printk("reg_slow[0]=%x\n",reg_slow[0]);		
		reg_slow[0]=0x05;
		aat3635_i2c_rx_data(reg_slow,1);
		printk("reg_slow[5]=%x\n",reg_slow[0]);	
		reg_slow[0]=0x06;
		aat3635_i2c_rx_data(reg_slow,1);
		printk("reg_slow[6]=%x\n",reg_slow[0]);	
		reg_slow[0]=0x07;
		aat3635_i2c_rx_data(reg_slow,1);
		printk("reg_slow[7]=%x\n",reg_slow[0]);	
		reg_slow[0]=0x08;
		aat3635_i2c_rx_data(reg_slow,1);
		printk("reg_slow[8]=%x\n",reg_slow[0]);	
		reg_slow[0]=0x09;
		aat3635_i2c_rx_data(reg_slow,1);
		printk("reg_slow[9]=%x\n",reg_slow[0]);	
		reg_slow[0]=0x0A;
		aat3635_i2c_rx_data(reg_slow,1);
		printk("reg_slow[A]=%x\n",reg_slow[0]);	
		reg_slow[0]=0x0B;
		aat3635_i2c_rx_data(reg_slow,1);
		printk("reg_slow[B]=%x\n",reg_slow[0]);	
		reg_slow[0]=0x0C;
		aat3635_i2c_rx_data(reg_slow,1);
		printk("reg_slow[C]=%x\n",reg_slow[0]);	
		reg_slow[0]=0x0D;
		aat3635_i2c_rx_data(reg_slow,1);
		printk("reg_slow[D]=%x\n",reg_slow[0]);	
		reg_slow[0]=0x0E;
		aat3635_i2c_rx_data(reg_slow,1);
		printk("reg_slow[E]=%x\n",reg_slow[0]);			
		//aat3635_i2c_write_byte(0x81, 0x01);
		//aat3635_i2c_write_byte(0x63, 0x02);
		//aat3635_i2c_write_byte(0x03, 0x03);
		//aat3635_i2c_write_byte(0x00, 0x04);
		//aat3635_i2c_write_byte(0x0A, 0x00);
		pr_aat3635_info("Switch charger ON (SLOW):");
		#endif
		break;
	case POWER_SUPPLY_ENABLE_FAST_CHARGE:
		reg_fast[0]=0x01;
		reg_fast[1]=0xBB;
		aat3635_i2c_tx_data(reg_fast,2);
		reg_fast[0]=0x02;
		if((bat_ID_value>1820 && bat_ID_value<2275)||(bat_ID_value>683 && bat_ID_value<1365))
			reg_fast[1]=0xAC;
		else
			reg_fast[1]=0xA4;
		aat3635_i2c_tx_data(reg_fast,2);
		reg_fast[0]=0x04;
		if((bat_ID_value>1820 && bat_ID_value<2275)||(bat_ID_value>683 && bat_ID_value<1365))
			reg_fast[1]=0x78;
		else
			reg_fast[1]=0x70;
		aat3635_i2c_tx_data(reg_fast,2);
		reg_fast[0]=0x03;
		reg_fast[1]=0x14;
		aat3635_i2c_tx_data(reg_fast,2);
		test[0]=0x00;
		aat3635_i2c_rx_data(test,1);
		test[0] = test[0]|0x03;
		//printk("***reg_fast of 0x00 is %x\n",test[0]);
		reg_fast[0]=0x00;
		reg_fast[1]=test[0];
		aat3635_i2c_tx_data(reg_fast,2);

		#if AAT_LOG_ON
		reg_fast[0]=0x01;
		aat3635_i2c_rx_data(reg_fast,1);
		printk("reg_fast[1]=%x\n",reg_fast[0]);
		reg_fast[0]=0x02;
		aat3635_i2c_rx_data(reg_fast,1);
		printk("reg_fast[2]=%x\n",reg_fast[0]);
		reg_fast[0]=0x03;
		aat3635_i2c_rx_data(reg_fast,1);
		printk("reg_fast[3]=%x\n",reg_fast[0]);		
		reg_fast[0]=0x04;
		aat3635_i2c_rx_data(reg_fast,1);
		printk("reg_fast[4]=%x\n",reg_fast[0]);	
		reg_fast[0]=0x00;
		aat3635_i2c_rx_data(reg_fast,1);
		printk("reg_fast[0]=%x\n",reg_fast[0]);	
		reg_fast[0]=0x05;
		aat3635_i2c_rx_data(reg_fast,1);
		printk("reg_fast[5]=%x\n",reg_fast[0]);	
		reg_fast[0]=0x06;
		aat3635_i2c_rx_data(reg_fast,1);
		printk("reg_fast[6]=%x\n",reg_fast[0]);	
		reg_fast[0]=0x07;
		aat3635_i2c_rx_data(reg_fast,1);
		printk("reg_fast[7]=%x\n",reg_fast[0]);	
		reg_fast[0]=0x08;
		aat3635_i2c_rx_data(reg_fast,1);
		printk("reg_fast[8]=%x\n",reg_fast[0]);	
		reg_fast[0]=0x09;
		aat3635_i2c_rx_data(reg_fast,1);
		printk("reg_fast[9]=%x\n",reg_fast[0]);	
		reg_fast[0]=0x0A;
		aat3635_i2c_rx_data(reg_fast,1);
		printk("reg_fast[A]=%x\n",reg_fast[0]);	
		reg_fast[0]=0x0B;
		aat3635_i2c_rx_data(reg_fast,1);
		printk("reg_fast[B]=%x\n",reg_fast[0]);	
		reg_fast[0]=0x0C;
		aat3635_i2c_rx_data(reg_fast,1);
		printk("reg_fast[C]=%x\n",reg_fast[0]);	
		reg_fast[0]=0x0D;
		aat3635_i2c_rx_data(reg_fast,1);
		printk("reg_fast[D]=%x\n",reg_fast[0]);	
		reg_fast[0]=0x0E;
		aat3635_i2c_rx_data(reg_fast,1);
		printk("reg_slow[E]=%x\n",reg_fast[0]);			
		//aat3635_i2c_write_byte(0xB1, 0x01);
		//aat3635_i2c_write_byte(0xE3, 0x02);
		//aat3635_i2c_write_byte(0x60, 0x04);
		//aat3635_i2c_write_byte(0x03, 0x03);
		//aat3635_i2c_write_byte(0x0A, 0x00);
		pr_aat3635_info("Switch charger ON (FAST):");
		#endif
		break;	
	default:
		pr_aat3635_info("%s: Not supported battery ctr called.!\n", __func__);
		result = -EINVAL;
		break;
	}

	return result;
}
EXPORT_SYMBOL(aat3635_set_charger_ctrl);

#if 1
static irqreturn_t chg_int_handler(int irq, void *data)
{
	struct aat3635_chip *chip = aat_this_chip;
	CHECK_LOG();
	#if AAT_LOG_ON
	pr_aat3635_info("interrupt chg_int is triggered.\n");
	#endif
	chip->battery_info.health = POWER_SUPPLY_HEALTH_GOOD;	
	if(!first_time_on)
		schedule_delayed_work(&chg_int_data->aat_int_work, msecs_to_jiffies(10));
	first_time_on = 0;
	return IRQ_HANDLED;
}

static void aat3635_int_func(struct work_struct *work)
{
	u8 reg_int[2];
	u8 int1_fault_bit,int2_fault_bit,int3_fault_bit;
	struct aat3635_chip *chip = aat_this_chip;
	charger_type_t type;

	reg_int[0]=0x08;
	aat3635_i2c_rx_data(reg_int,1);
	int1_fault_bit = reg_int[0];
	printk("reg_int[8]=%x\n",reg_int[0]);\
	reg_int[0]=0x09;
	aat3635_i2c_rx_data(reg_int,1);
	int2_fault_bit = reg_int[0];
	printk("reg_int[9]=%x\n",reg_int[0]);
	reg_int[0]=0x0A;
	aat3635_i2c_rx_data(reg_int,1);
	int3_fault_bit = reg_int[0];
	printk("reg_int[A]=%x\n",reg_int[0]);	
	if(int2_fault_bit & 0x04)
	{
		timer_out =1;
		chip->battery_info.health = POWER_SUPPLY_HEALTH_GOOD;		
	}
	else if(int1_fault_bit & 0x10)
		chip->battery_info.health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if(int1_fault_bit & 0x40)
		chip->battery_info.health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if(int2_fault_bit & 0x20)
		chip->battery_info.health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if(int2_fault_bit & 0x10)
		chip->battery_info.health = POWER_SUPPLY_HEALTH_UNDERVOLTAGE;	
	else if(int2_fault_bit & 0x08)
	{
		//timer_out =1;
		chip->battery_info.health = POWER_SUPPLY_HEALTH_GOOD;		
	}	
	else if((int1_fault_bit == 0)&&(int2_fault_bit == 0))
		chip->battery_info.health = POWER_SUPPLY_HEALTH_GOOD;	
	else
		chip->battery_info.health = POWER_SUPPLY_HEALTH_UNKNOWN;

	   if(chip->battery_info.health !=POWER_SUPPLY_HEALTH_GOOD)
	   		type = CHARGER_BATTERY;
	   		//aat3635_set_charger_ctrl(POWER_SUPPLY_DISABLE_CHARGE);
	   else
	   	   type = aat_this_chip->battery_info.charging_source;

	update_aat3635_charger(type);
	aat3635_cable_status_update(type);	
	//wake_lock_timeout(&fault_wake_lock, 500);
	//power_supply_changed(&aat3635_power_supplies[0]);
	//power_supply_changed(&aat3635_power_supplies[1]);
}
#endif

static int  aat3635_gpio_init(void)
{
	int ret = 0;
	#if AAT_LOG_ON
	printk("aat3635_gpio_init---\n");
	#endif
	ret=gpio_is_valid(EXYNOS4_GPX0(4));
	if(!ret){
		printk("gpio is not valid for aat3635");
		return ret;
		}
	//enable charger_int  pin.
 	ret=gpio_request(EXYNOS4_GPX0(4), "charger int pin");
	if(ret<0){
			printk("gpio requst fail for charger: EXYNOS4_GPX0(4) (cen) pin!! ");
		return ret;
		}
	s3c_gpio_cfgpin(EXYNOS4_GPX0(4), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS4_GPX0(4), S3C_GPIO_PULL_NONE);

	ret=gpio_is_valid(EXYNOS4_GPX1(4));
	if(!ret){
		printk("gpio is not valid for aat3635 INT");
		return ret;
		}
	//enable charger_int  pin.
 	ret=gpio_request(EXYNOS4_GPX1(4), "AAT int pin");
	if(ret<0){
			printk("gpio requst fail for charger: S5PV310_GPX1(4) (cen) pin!! ");
		return ret;
		}
	s3c_gpio_cfgpin(EXYNOS4_GPX1(4), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS4_GPX1(4), S3C_GPIO_PULL_NONE);	

	return ret;

}


void att3635_set_backlight(int level,int max,int min)
{
	//unsigned long flags;
	//int i;
	
	if(level == 0){

		if(bl_on == 1){
		lcd_off();
		bl_on = 0;
		}
	}
	else
	{
		if(bl_on==0)
		{
			lcd_on();
			bl_on = 1;
		}
	}

	level/=8;
	
	//printk("%s, level = %d, max = %d, min = %d\n", __func__, level,max,min);
	if (level < min)
		level = min;
	if (level > MAX_BL_LEVEL)
		level = MAX_BL_LEVEL;

	WLED_ctrl(0,level);

	return;
}

static int aat3635_backlight_update_status(struct backlight_device *bd)
{
	//struct pwm_bl_data *pb = dev_get_drvdata(&bd->dev);
	int brightness = bd->props.brightness;
	int max = bd->props.max_brightness;
	
	//printk("%s\n",__func__);

	if (brightness == 0) {
		att3635_set_backlight(0,max,0);
	} 
	else 
	{
		att3635_set_backlight(brightness,max,0);
	}
	return 0;
}

static int aat3635_backlight_get_brightness(struct backlight_device *bd)
{
	#if AAT_LOG_ON
	printk("%s\n",__func__);
	#endif
	return bd->props.brightness;
}

static int aat3635_backlight_check_fb(struct backlight_device *bl,
			  struct fb_info *info)
{
	#if AAT_LOG_ON
	printk("%s\n",__func__);	
	#endif
	return 0;
}

static const struct backlight_ops aat3635_backlight_ops = {
	.update_status	= aat3635_backlight_update_status,
	.get_brightness	= aat3635_backlight_get_brightness,
	.check_fb	= aat3635_backlight_check_fb,
};

static ssize_t charging_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	unsigned int status = 0;
	status = is_charger_online();
	return sprintf(buf, " %s \n", status? "enabled":"disabled");
}
static ssize_t charging_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned int state;
	charger_type_t type;
	static  charger_type_t old_type;

	sscanf(buf, "%d", &state);
	printk("aat3635 :update status = %s \n",state?"enable":"disable");
	if(state)
		type = old_type;
	else
	{
		old_type = aat_this_chip->battery_info.charging_source;
		type = CHARGER_BATTERY;
	}

	update_aat3635_charger(type);
	aat3635_cable_status_update(type);
	return count;
}

static DEVICE_ATTR(charging, S_IRUGO | S_IWUSR, charging_show, charging_store);


static int aat3635_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int i,rc = 0;
	struct aat3635_chip *chip;
	//struct aat3635_i2c_client   *data = &aat3635_i2c_module;
	//struct aat3635_platform_data *pdata =client->dev.platform_data;
	struct backlight_properties props;
	struct backlight_device *bl;
	struct device *aat3635_dev;
	int gpio_value;

	mutex_init(&lcd_on_or_off);
       wake_lock_init(&chg_wake_lock, WAKE_LOCK_SUSPEND, "chg_lock");
	INIT_DELAYED_WORK(&chg_otp_work,
				chg_otp_work_func);
	
	init_timer(&temp_dect_timer);	
	temp_dect_timer.function = aat_otp_timer_func;
	//temp_dect_timer.expires = jiffies + (2*HZ);
	//add_timer(&temp_dect_timer);
	mod_timer(&temp_dect_timer,jiffies + (2*HZ));

	first_time_on =1;
	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	aat_this_chip = chip;
	chip->client = client;
	//chip->chg_info= client->dev.platform_data;
	i2c_set_clientdata(client, chip);

	CHECK_LOG();
	//printk("aat3635_probe-----\n");
	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C) == 0) {
		pr_aat3635_err("I2C fail\n");
		return -EIO;
	}

	if(aat3635_get_id()!=0x43)
		return -1;

	aat3635_gpio_init();
	
	/* init power supplier framework */
	for (i = 0; i < ARRAY_SIZE(aat3635_power_supplies); i++) {
		rc = power_supply_register(&client->dev, 
				&aat3635_power_supplies[i]);
		if (rc) {
			pr_aat3635_err("power_supply_register fail\n");
			return rc;
		}
	}
	#if 1
	//init woekqueue 
	aat3635_mask_DPM();
	//printk("power_supply_register OK-----\n");
	chg_status = (struct aat3635_chg_int_data *)
					kmalloc(sizeof(struct aat3635_chg_int_data),
						GFP_KERNEL);
	if (!chg_status) {
		pr_aat3635_err("No memory for chg_status!\n");
		return -1;
	}
	
	INIT_DELAYED_WORK(&chg_status->chg_int_work,
				aat3635_charger_work_func);

	INIT_DELAYED_WORK(&chg_status->chg_type_work,
				aat3635_charger_type_work_func);
	//set  irq
	rc = request_irq(gpio_to_irq(EXYNOS4_GPX0(4)), aat3635_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"ss_charger_int",NULL);
	if (rc < 0)
	{
		printk("aat3635 request_irq(cok_irq_handler) failed due to %d !!!\n",rc);
		free_irq(gpio_to_irq(EXYNOS4_GPX0(4)),NULL);
		return -1;
	}
	enable_irq_wake(gpio_to_irq(EXYNOS4_GPX0(4)));	
#endif
#if 1
	/*  For chg_int interrupt initialization. */
	
		chg_int_data = (struct aat3635_chg_int_data *)
				kmalloc(sizeof(struct aat3635_chg_int_data),
					GFP_KERNEL);
		if (!chg_int_data) {
			pr_aat3635_err("No memory for chg_int_data!\n");
			return -1;
		}

		//chg_int_data->gpio_chg_int = 0;
		INIT_DELAYED_WORK(&chg_int_data->aat_int_work,
				aat3635_int_func);

		rc = request_irq(
				gpio_to_irq(EXYNOS4_GPX1(4)),
				chg_int_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"chg_int", NULL);
		if (rc < 0)
		{
			printk("aat3635 request_irq(uok_irq_handler) failed due to %d !!!\n",rc);
			free_irq(gpio_to_irq(EXYNOS4_GPX1(4)),NULL);
			return -1;
		}
		enable_irq_wake(gpio_to_irq(EXYNOS4_GPX1(4)));	
		
	 gpio_value = gpio_get_value(EXYNOS4_GPX0(4));
	 if(gpio_value)  
	 	aat3635_charger_work_func();
#endif
	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = MAX_BRIGHTNESS;
	//init backlight
	bl = backlight_device_register("pwm-backlight.0", &client->dev, NULL,
				       &aat3635_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&client->dev, "failed to register backlight\n");
		PTR_ERR(bl);
	}

	aat3635_class = class_create(THIS_MODULE, "charger_IC");
	if (IS_ERR((void *)aat3635_class))
		return PTR_ERR((void *)aat3635_class);
	aat3635_dev = device_create(aat3635_class, NULL,
		MKDEV(0, 0), NULL, "aat3635");
	if (IS_ERR((void *)aat3635_dev))
		return PTR_ERR((void *)aat3635_dev);

	rc = device_create_file(aat3635_dev, &dev_attr_charging);
	if (rc < 0)
		goto err_create_file_1;

	aat3635_initial = 1;
	pr_aat3635_info("[aat3635]: Driver registration done\n");

		return 0;
		
err_create_file_1:
	device_destroy(aat3635_class, MKDEV(0, 0));
	printk(KERN_ERR "switch: Failed to register driver %s\n", "aat3635");
}

static int aat3635_remove(struct i2c_client *client)
{
	int i;
	struct aat3635_chip *chip = i2c_get_clientdata(client);

	CHECK_LOG();	

	for (i = 0; i < ARRAY_SIZE(aat3635_power_supplies); i++)
		power_supply_unregister(&aat3635_power_supplies[i]);
	i2c_set_clientdata(client, NULL);
	kfree(chip);	
	return 0;
}

#if 1
static int aat3635_suspend(struct platform_device *pdev, 
		pm_message_t state)
{
	//printk("+++%s+++\n", __func__);

	return 0;
}

static int aat3635_resume(struct platform_device *pdev)
{
#if AAT_LOG_ON
	printk("+++%s+++\n", __func__);
#endif
	aat3635_get_id();
	return 0;
}
#endif
static const struct i2c_device_id aat3635_id[] = {
	{ "aat3635", 0 },
	{  },
};
static struct i2c_driver aat3635_driver = {
	.driver.name    = "aat3635",
	.id_table   = aat3635_id,
	.probe      = aat3635_probe,
	.remove     = aat3635_remove,
	//.shutdown   = aat3635_shutdown,
	.suspend   =   aat3635_suspend,
	.resume    =   aat3635_resume,
};

static int __init sensors_aat3635_init(void)
{
	int res;

	CHECK_LOG();
	#if AAT_LOG_ON
       printk("sensors_aat3635_init");
	#endif
	aat3635_low_chg = 0;
	chg_stat_enabled = 0;
	//spin_lock_init(&chg_stat_lock);
	res = i2c_add_driver(&aat3635_driver);
	if (res)
		pr_aat3635_err("[aat3635]: Driver registration failed \n");

	return res;
}

static void __exit sensors_aat3635_exit(void)
{
	free_irq(gpio_to_irq(EXYNOS4_GPX1(4)),NULL);	
	gpio_free(EXYNOS4_GPX1(4));
	free_irq(gpio_to_irq(EXYNOS4_GPX0(4)),NULL);	
	gpio_free(EXYNOS4_GPX0(4));	
	i2c_del_driver(&aat3635_driver);
}

MODULE_AUTHOR("Josh Hsiao <Josh_Hsiao@htc.com>");
MODULE_DESCRIPTION("aat3635 driver");
MODULE_LICENSE("GPL");

module_init(sensors_aat3635_init);
module_exit(sensors_aat3635_exit);

