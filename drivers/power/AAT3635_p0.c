/* drivers/i2c/chips/aat3635_p0.c
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


#define pr_aat3635_p0_fmt(fmt) "[BATT][aat3635_p0] " fmt
#define pr_aat3635_p0_err_fmt(fmt) "[BATT][aat3635_p0] err:" fmt
#define pr_aat3635_p0_warn_fmt(fmt) "[BATT][aat3635_p0] warn:" fmt
#define pr_aat3635_p0_info(fmt, ...) \
	printk(KERN_INFO pr_aat3635_p0_fmt(fmt), ##__VA_ARGS__)
#define pr_aat3635_p0_err(fmt, ...) \
	printk(KERN_ERR pr_aat3635_p0_err_fmt(fmt), ##__VA_ARGS__)
#define pr_aat3635_p0_warn(fmt, ...) \
	printk(KERN_WARNING pr_aat3635_p0_warn_fmt(fmt), ##__VA_ARGS__)

/* used for debug if function called */
#define FUNC_CALL_CHECK 0

#if FUNC_CALL_CHECK
#define CHECK_LOG() pr_aat3635_p0_info("%s\n", __func__)
#else
#define CHECK_LOG() (void)0
#endif

#define P0_POLLING_TIME_MSEC 1000

static struct aat3635_chg_int_data *chg_int_data_p0;

static struct workqueue_struct *aat3635_p0_wq;

static struct work_struct aat3635_p0_work;

static unsigned int chg_stat_enabled;
//static spinlock_t chg_stat_lock;

static int system_suspended = 0; /*indicate whether the system still is in the suspended status*/
static struct work_struct state_wq;
static struct timer_list polling_timer;

static struct aat3635_chip *aat_p0_this_chip = NULL;

static int aat3635_p0_i2c_write_byte(u8 value, u8 reg);
static int aat3635_p0_i2c_read_byte(u8 *value, u8 reg);

static void aat3635_p0_update_status(void);
static int aat3635_p0_cable_status_update(int status);
static int aat3635_p0_set_charger_ctrl(u32 ctl);

static int aat3635_p0_initial = -1;
//static int aat3635_p0_low_chg;
int charger_type;
u8 	charger_status;
extern struct aat3635_platform_data aat3635_p0_priv;

static void update_aat3635_p0_charger(charger_type_t type)
{
	//printk("2) aat3635_p0 update_aat3635_p0_charger  type = %d\n", type);
	
	switch(type)
	{
		case CHARGER_AC:
			aat3635_p0_set_charger_ctrl(CHARGER_AC);
			break;
		case CHARGER_USB:
			aat3635_p0_set_charger_ctrl(CHARGER_USB);
			break;
		case CHARGER_BATTERY:
			aat3635_p0_set_charger_ctrl(CHARGER_BATTERY);
		default:
			break;
	}	
}

void aat3635_p0_charger_work_func(void)
{
	charger_type_t status = 0;

	printk("***charger_status is 0x:%x ***\n",charger_status);
	if(charger_status)
	{
		printk("***charger_type is %d ***\n",charger_type);
		if(charger_type==1)
			status = CHARGER_USB;
		else if(charger_type==2)
		{
			status = CHARGER_AC;
		}
	}
	else
	{
		charger_type =0;
		status = CHARGER_BATTERY;
	}

	if(status == aat_p0_this_chip->battery_info.charging_source)
		return;
	//aat_p0_this_chip->battery_info.charging_source = status;

	update_aat3635_p0_charger(status);
	aat3635_p0_cable_status_update(status);	
}


static enum power_supply_property aat3635_p0_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

static DEFINE_MUTEX(work_lock);

static void polling_timer_func(void)
{
	if(system_suspended)
		return;
	//queue_work(aat3635_p0_wq, &state_wq);
    	schedule_work(&state_wq);    
    	mod_timer(&polling_timer,
        jiffies + msecs_to_jiffies(P0_POLLING_TIME_MSEC));

}

static int aat3635_p0_power_get_property(struct power_supply *bat_ps, 
		enum power_supply_property psp, 
		union power_supply_propval *val)
{
	charger_type_t charger = aat_p0_this_chip->battery_info.charging_source;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (charger==CHARGER_USB ||charger==CHARGER_AC);
		break;
	default:
		return -EINVAL;
	}
	
	return 0;
}

static int  aat3635_gpio_init(void)
{
	int ret = 0;
	
	printk("aat3635_gpio_init---\n");

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

static void  aat3635_p0_charger_init(void)
{
	printk("aat3635_p0_charger_init\n");
	aat3635_gpio_init();
	INIT_WORK(&state_wq, aat3635_p0_update_status);
	
	setup_timer(&polling_timer, polling_timer_func, 0);
	mod_timer(&polling_timer,
		  jiffies + msecs_to_jiffies(0));
}

static struct power_supply aat3635_p0_power_supplies[] = {
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = aat3635_p0_power_properties,
		.num_properties = ARRAY_SIZE(aat3635_p0_power_properties),
		.get_property = aat3635_p0_power_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = aat3635_p0_power_properties,
		.num_properties = ARRAY_SIZE(aat3635_p0_power_properties),
		.get_property = aat3635_p0_power_get_property,
	},
};

static int aat3635_p0_cable_status_update(int status)
{
	int ret = 0;

	if(!aat3635_p0_initial)
		return -EPERM;

	switch(status) {
	case CHARGER_BATTERY:
		aat_p0_this_chip->battery_info.charging_source = CHARGER_BATTERY;
		aat_p0_this_chip->battery_info.status= POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case CHARGER_USB:
		aat_p0_this_chip->battery_info.charging_source = CHARGER_USB;
		aat_p0_this_chip->battery_info.status= POWER_SUPPLY_STATUS_CHARGING;
		break;
	case CHARGER_AC:
		aat_p0_this_chip->battery_info.charging_source = CHARGER_AC;
		aat_p0_this_chip->battery_info.status= POWER_SUPPLY_STATUS_CHARGING;
		break;
	case CHARGER_DISCHARGE:
		aat_p0_this_chip->battery_info.charging_source = CHARGER_DISCHARGE;
		aat_p0_this_chip->battery_info.status= POWER_SUPPLY_STATUS_NOT_CHARGING;		
		break;
	default:
		ret = -EINVAL;
	}

	/* if the power source changes, all power supplies may change state */
	power_supply_changed(&aat3635_p0_power_supplies[0]);
	power_supply_changed(&aat3635_p0_power_supplies[1]);
	//power_supply_changed(&aat3635_p0_power_supplies[2]);
	return ret;
}

static void aat3635_p0_update_status(void)
{
	if(system_suspended)
		return;
    	aat3635_p0_charger_work_func();
}

/**
 * Insmod parameters
 */
/* I2C_CLIENT_INSMOD_1(aat3635_p0); */

static int aat3635_p0_probe(struct i2c_client *client,
			const struct i2c_device_id *id);
static int aat3635_p0_remove(struct i2c_client *client);

int  is_charger_online(void)//mj for charging detect.
{
	//struct fg8997_chip *chip = aat_aat_p0_this_chip;
	return  (aat_p0_this_chip->battery_info.charging_source == CHARGER_USB||aat_p0_this_chip->battery_info.charging_source == CHARGER_AC);
}

static int aat3635_p0_i2c_rx_data(char *buf, int len)
{	uint8_t i;	
	struct i2c_client *aat_client = aat_p0_this_chip->client;	
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
	for (i = 0; i < 3; i++) {		
		if (i2c_transfer(aat_client->adapter, msgs, 2) > 0) 
		{	
			break;		
		}		
		mdelay(10);	}	
		if (i >= 3) {		
			pr_err("%s: retry over 3\n", __FUNCTION__);		
			return -EIO;	
		}	
		return 0;
}
static int aat3635_p0_i2c_tx_data(char *buf, int len)
{	
	uint8_t i;	
	struct i2c_client *aat_client = aat_p0_this_chip->client;	
	struct i2c_msg msg[] = {		
		{			
			.addr	= aat_client->addr,			
			.flags	= 0,			
			.len	= len,			
			.buf	= buf,		
		}	
	};	
	//printk("aat3635_p0_i2c_tx_data------start -------,times = %lx \n",jiffies);	
	if(NULL == aat_client || NULL == aat_client->addr)		
		return -EIO;
	for (i = 0; i < 3; i++) {		
		if (i2c_transfer(aat_client->adapter, msg, 1) > 0) 
		{			
			break;		
		}		
		mdelay(10);	}	
		//printk("aat3635_p0_i2c_tx_data------end -------,times = %lx \n",jiffies);	
	if (i >= 3) {		
		pr_err("%s: retry over 3\n", __FUNCTION__);		
		return -EIO;	
	}	
	return 0;
}

void aat3635_p0_LED_ctrl(bool ctl)
{
	u8 reg[2];
	u8 led_status;
	reg[0]=0x00;
	aat3635_p0_i2c_rx_data(reg,1);
	led_status = reg[0];
	printk("aat3635_LED_ctrl --%d,LED_status--%x\n",ctl,led_status);
	if(ctl)
	{
		led_status = led_status & 0x7F;
		led_status = led_status | 0x40;
		printk("LED_status ON--%x\n",led_status);
		reg[0]=0x00;
		reg[1]=led_status;
		aat3635_p0_i2c_tx_data(reg,2);
	}
	else
	{
		led_status = led_status | 0x80;
		led_status = led_status &  0xBF;
		printk("LED_status OFF--%x\n",led_status);
		reg[0]=0x00;
		reg[1]=led_status;
		aat3635_p0_i2c_tx_data(reg,2);
	}
}

static int aat3635_p0_set_charger_ctrl(u32 ctl)
{
	int result = 0;
	u8 nRegValue;
	u8 reg_dis[2],reg_fast[2],reg_slow[2];

	CHECK_LOG();

	if (aat3635_p0_initial < 0)
		return 0;

	switch (ctl) {
	case POWER_SUPPLY_DISABLE_CHARGE:
		pr_aat3635_p0_info("Switch charger OFF\n");
		reg_dis[0]=0x00;
		//aat3635_p0_i2c_tx_data(&reg_dis,1);
		aat3635_p0_i2c_rx_data(reg_dis,1);
		nRegValue = reg_dis[0];
		printk("nRegValue--%x\n",nRegValue);
		nRegValue &= ~(0x03);//CH_EN = 0
		reg_dis[0]=0x00;
		reg_dis[1]=nRegValue;
		aat3635_p0_i2c_tx_data(reg_dis,2);
         	//aat3635_p0_i2c_read_byte(&nRegValue, 0x00);//Get register 0's value
         	//nRegValue &= ~(0x03);//CH_EN = 0
         	//aat3635_p0_i2c_write_byte(nRegValue, 0x00);
		break;
	case POWER_SUPPLY_ENABLE_SLOW_CHARGE:
		reg_slow[0]=0x01;
		reg_slow[1]=0x81;
		aat3635_p0_i2c_tx_data(reg_slow,2);
		reg_slow[0]=0x02;
		reg_slow[1]=0x63;
		aat3635_p0_i2c_tx_data(reg_slow,2);
		reg_slow[0]=0x03;
		reg_slow[1]=0x03;
		aat3635_p0_i2c_tx_data(reg_slow,2);
		reg_slow[0]=0x04;
		reg_slow[1]=0x00;
		aat3635_p0_i2c_tx_data(reg_slow,2);
		reg_slow[0]=0x00;
		reg_slow[1]=0x0A;
		aat3635_p0_i2c_tx_data(reg_slow,2);
		reg_slow[0]=0x01;
		aat3635_p0_i2c_rx_data(reg_slow,1);
		printk("reg_slow[1]=%x\n",reg_slow[0]);
		reg_slow[0]=0x02;
		aat3635_p0_i2c_rx_data(reg_slow,1);
		printk("reg_slow[2]=%x\n",reg_slow[0]);
		reg_slow[0]=0x03;
		aat3635_p0_i2c_rx_data(reg_slow,1);
		printk("reg_slow[3]=%x\n",reg_slow[0]);		
		reg_slow[0]=0x04;
		aat3635_p0_i2c_rx_data(reg_slow,1);
		printk("reg_slow[4]=%x\n",reg_slow[0]);	
		reg_slow[0]=0x00;
		aat3635_p0_i2c_rx_data(reg_slow,1);
		printk("reg_slow[0]=%x\n",reg_slow[0]);		
		//aat3635_p0_i2c_write_byte(0x81, 0x01);
		//aat3635_p0_i2c_write_byte(0x63, 0x02);
		//aat3635_p0_i2c_write_byte(0x03, 0x03);
		//aat3635_p0_i2c_write_byte(0x00, 0x04);
		//aat3635_p0_i2c_write_byte(0x0A, 0x00);
		pr_aat3635_p0_info("Switch charger ON (SLOW):");
		break;
	case POWER_SUPPLY_ENABLE_FAST_CHARGE:
		reg_fast[0]=0x01;
		reg_fast[1]=0xB1;
		aat3635_p0_i2c_tx_data(reg_fast,2);
		reg_fast[0]=0x02;
		reg_fast[1]=0xB3;
		aat3635_p0_i2c_tx_data(reg_fast,2);
		reg_fast[0]=0x04;
		reg_fast[1]=0x60;
		aat3635_p0_i2c_tx_data(reg_fast,2);
		reg_fast[0]=0x03;
		reg_fast[1]=0x03;
		aat3635_p0_i2c_tx_data(reg_fast,2);
		reg_fast[0]=0x00;
		reg_fast[1]=0x0A;		
		aat3635_p0_i2c_tx_data(reg_fast,2);
		reg_fast[0]=0x01;
		aat3635_p0_i2c_rx_data(reg_fast,1);
		printk("reg_fast[0]=%x\n",reg_fast[0]);
		reg_fast[0]=0x02;
		aat3635_p0_i2c_rx_data(reg_fast,1);
		printk("reg_fast[2]=%x\n",reg_fast[0]);
		reg_fast[0]=0x03;
		aat3635_p0_i2c_rx_data(reg_fast,1);
		printk("reg_fast[3]=%x\n",reg_fast[0]);		
		reg_fast[0]=0x04;
		aat3635_p0_i2c_rx_data(reg_fast,1);
		printk("reg_fast[4]=%x\n",reg_fast[0]);	
		reg_fast[0]=0x00;
		aat3635_p0_i2c_rx_data(reg_fast,1);
		printk("reg_fast[0]=%x\n",reg_fast[0]);		
		//aat3635_p0_i2c_write_byte(0xB1, 0x01);
		//aat3635_p0_i2c_write_byte(0xE3, 0x02);
		//aat3635_p0_i2c_write_byte(0x60, 0x04);
		//aat3635_p0_i2c_write_byte(0x03, 0x03);
		//aat3635_p0_i2c_write_byte(0x0A, 0x00);
		pr_aat3635_p0_info("Switch charger ON (FAST):");

		break;	
	default:
		pr_aat3635_p0_info("%s: Not supported battery ctr called.!\n", __func__);
		result = -EINVAL;
		break;
	}

	return result;
}
EXPORT_SYMBOL(aat3635_p0_set_charger_ctrl);

#if 1
static irqreturn_t p0_chg_int_handler(int irq, void *data)
{
	struct aat3635_chip *chip = aat_p0_this_chip;
	CHECK_LOG();

	printk("interrupt chg_int is triggered.\n");
	chip->battery_info.health = POWER_SUPPLY_HEALTH_GOOD;	
	schedule_delayed_work(&chg_int_data_p0->aat_int_work, msecs_to_jiffies(10));
	return IRQ_HANDLED;
}

static void aat3635_p0_int_func(struct work_struct *work)
{
	u8 reg_int[2];
	u8 int1_fault_bit,int2_fault_bit,int3_fault_bit;
	struct aat3635_chip *chip = aat_p0_this_chip;

	reg_int[0]=0x08;
	aat3635_p0_i2c_rx_data(reg_int,1);
	int1_fault_bit = reg_int[0];
	printk("reg_int[8]=%x\n",reg_int[0]);\
	reg_int[0]=0x09;
	aat3635_p0_i2c_rx_data(reg_int,1);
	int2_fault_bit = reg_int[0];
	printk("reg_int[9]=%x\n",reg_int[0]);
	reg_int[0]=0x0A;
	aat3635_p0_i2c_rx_data(reg_int,1);
	int3_fault_bit = reg_int[0];
	printk("reg_int[A]=%x\n",reg_int[0]);	
	if(int1_fault_bit & 0x10)
		chip->battery_info.health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if(int1_fault_bit & 0x40)
		chip->battery_info.health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if(int2_fault_bit & 0x20)
		chip->battery_info.health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if(int2_fault_bit & 0x10)
		chip->battery_info.health = POWER_SUPPLY_HEALTH_UNDERVOLTAGE;	
	else if((int1_fault_bit == 0)&&(int2_fault_bit == 0))
		chip->battery_info.health = POWER_SUPPLY_HEALTH_GOOD;	

	   if(chip->battery_info.health !=POWER_SUPPLY_HEALTH_GOOD)
	   	{
	   		aat3635_p0_set_charger_ctrl(POWER_SUPPLY_DISABLE_CHARGE);
	   		power_supply_changed(&aat3635_p0_power_supplies[0]);
			power_supply_changed(&aat3635_p0_power_supplies[1]);
	   	}
	   else 
	   		aat3635_p0_charger_work_func();
	   	
	//wake_lock_timeout(&fault_wake_lock, 500);
}
#endif


static int aat3635_p0_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int i,rc = 0;
	struct aat3635_chip *chip;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	aat_p0_this_chip = chip;
	chip->client = client;
	//chip->chg_info= client->dev.platform_data;
	i2c_set_clientdata(client, chip);
	
	CHECK_LOG();
	printk("aat3635_p0_probe-----\n");
	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C) == 0) {
		pr_aat3635_p0_err("I2C fail\n");
		return -EIO;
	}
	
	aat3635_p0_charger_init();

	/* init power supplier framework */
	for (i = 0; i < ARRAY_SIZE(aat3635_p0_power_supplies); i++) {
		rc = power_supply_register(&client->dev, 
				&aat3635_p0_power_supplies[i]);
		if (rc) {
			pr_aat3635_p0_err("power_supply_register fail\n");
			return rc;
		}
	}
#if 1
	/*  For chg_int interrupt initialization. */
	
		chg_int_data_p0 = (struct aat3635_chg_int_data *)
				kmalloc(sizeof(struct aat3635_chg_int_data),
					GFP_KERNEL);
		if (!chg_int_data_p0) {
			pr_aat3635_p0_err("No memory for chg_int_data_p0!\n");
			return -1;
		}

		//chg_int_data_p0->gpio_chg_int = 0;
		INIT_DELAYED_WORK(&chg_int_data_p0->aat_int_work,
				aat3635_p0_int_func);

		rc = request_irq(
				gpio_to_irq(EXYNOS4_GPX1(4)),
				p0_chg_int_handler,
				IRQF_TRIGGER_FALLING,
				"chg_int", NULL);
		if (rc < 0)
		{
			printk("aat3635 request_irq(uok_irq_handler) failed due to %d !!!\n",rc);
			free_irq(gpio_to_irq(EXYNOS4_GPX1(4)),NULL);
			return -1;
		}
		enable_irq_wake(gpio_to_irq(EXYNOS4_GPX1(4)));	
		
	
#endif

	aat3635_p0_initial = 1;
	pr_aat3635_p0_info("[aat3635_p0]: Driver registration done\n");

	return 0;
}

static int aat3635_p0_remove(struct i2c_client *client)
{
	int i;
	struct aat3635_chip *chip = i2c_get_clientdata(client);

	CHECK_LOG();	

	for (i = 0; i < ARRAY_SIZE(aat3635_p0_power_supplies); i++)
		power_supply_unregister(&aat3635_p0_power_supplies[i]);
	i2c_set_clientdata(client, NULL);
	kfree(chip);	
	return 0;
}

static int aat3635_p0_bat_suspend(struct platform_device *pdev, 
		pm_message_t state)
{
	printk("+++%s+++\n", __func__);
	
	system_suspended = 1;

	del_timer_sync(&polling_timer);

	/*cancel and flash related work_queue*/
	flush_work(&state_wq);
	return 0;
}

static int aat3635_p0_bat_resume(struct platform_device *pdev)
{
	printk("+++%s+++\n", __func__);
	setup_timer(&polling_timer, polling_timer_func, 0);
	mod_timer(&polling_timer, jiffies + msecs_to_jiffies(P0_POLLING_TIME_MSEC));
	system_suspended = 0;
	
	return 0;
}

static const struct i2c_device_id aat3635_p0_id[] = {
	{ "aat3635_p0", 0 },
	{  },
};
static struct i2c_driver aat3635_p0_driver = {
	.driver.name    = "aat3635_p0",
	.id_table   = aat3635_p0_id,
	.probe      = aat3635_p0_probe,
	.remove     = aat3635_p0_remove,
	//.shutdown   = aat3635_p0_shutdown,
	.suspend   =   aat3635_p0_bat_suspend,
	.resume    =   aat3635_p0_bat_resume,
};

static int __init sensors_aat3635_p0_init(void)
{
	int res;

	CHECK_LOG();
	chg_stat_enabled = 0;
	//spin_lock_init(&chg_stat_lock);
	res = i2c_add_driver(&aat3635_p0_driver);
	if (res)
		pr_aat3635_p0_err("[aat3635_p0]: Driver registration failed \n");

	return res;
}

static void __exit sensors_aat3635_p0_exit(void)
{
	CHECK_LOG();
	
	free_irq(gpio_to_irq(EXYNOS4_GPX1(4)),NULL);
	gpio_free(EXYNOS4_GPX1(4));

	//free_irq(aat3635_p0_priv.charger_int,NULL);
	//gpio_free(aat3635_p0_priv.charger_int);

	i2c_del_driver(&aat3635_p0_driver);
}

MODULE_AUTHOR("Josh Hsiao <Josh_Hsiao@htc.com>");
MODULE_DESCRIPTION("aat3635_p0 driver");
MODULE_LICENSE("GPL");

module_init(sensors_aat3635_p0_init);
module_exit(sensors_aat3635_p0_exit);

