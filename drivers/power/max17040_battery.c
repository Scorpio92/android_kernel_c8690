/*
 *  max17040_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/max17040_battery.h>
#include <linux/slab.h>
#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>
#include <linux/regulator/consumer.h>

#define MAX17040_DBG_MSG	0
#define MAX17040_LOWVOL_TEST	0

#define MAX17040_VCELL_MSB	0x02
#define MAX17040_VCELL_LSB	0x03
#define MAX17040_SOC_MSB	0x04
#define MAX17040_SOC_LSB	0x05
#define MAX17040_MODE_MSB	0x06
#define MAX17040_MODE_LSB	0x07
#define MAX17040_VER_MSB	0x08
#define MAX17040_VER_LSB	0x09
#define MAX17040_RCOMP_MSB	0x0C
#define MAX17040_RCOMP_LSB	0x0D
#define MAX17040_CMD_MSB	0xFE
#define MAX17040_CMD_LSB	0xFF

#define MAX17040_RETRY_COUNT	3
#define MAX17040_DELAY		1000
#define MAX17040_BATTERY_FULL	98


#if 0

/* LED
 * #define GPIO_LED_RED		S5PV210_GPJ0(1)
 * #define GPIO_LED_GREEN	S5PV210_GPJ0(2)
 */
#define CFG_GPIO_RED do {\
	s3c_gpio_cfgpin(GPIO_LED_RED,S3C_GPIO_OUTPUT);\
	s3c_gpio_setpull(GPIO_LED_RED,S3C_GPIO_PULL_NONE);\
} while(0)
	/*
	s3c_gpio_cfgpin_slp(GPIO_LED_RED,S3C_GPIO_SLP_PREV);\
	s3c_gpio_setpull_slp(GPIO_LED_RED,S3C_GPIO_PULL_NONE);\
	*/
#define LED_RED_ON s3c_gpio_setpin(GPIO_LED_RED,1)
#define LED_RED_OFF s3c_gpio_setpin(GPIO_LED_RED,0)

#define CFG_GPIO_GREEN do {\
	s3c_gpio_cfgpin(GPIO_LED_GREEN,S3C_GPIO_OUTPUT);\
	s3c_gpio_setpull(GPIO_LED_GREEN,S3C_GPIO_PULL_NONE);\
} while(0)
	/*
	s3c_gpio_cfgpin_slp(GPIO_LED_GREEN,S3C_GPIO_SLP_PREV);\
	s3c_gpio_setpull_slp(GPIO_LED_GREEN,S3C_GPIO_PULL_NONE);\
	*/
#define LED_GREEN_ON s3c_gpio_setpin(GPIO_LED_GREEN,1)
#define LED_GREEN_OFF s3c_gpio_setpin(GPIO_LED_GREEN,0)


#endif


struct max17040_battery_info {
	int tech;	// always = POWER_SUPPLY_TECHNOLOGY_LION
	//int adc;	// ADC value of voltage, not used
	int vcell;	// battery voltage * 1000, unit mV
	int soc;	// battery capacity
	int temp10;	// temperature read from akm8973, then *10
	int charging_source;	// battery? usb? ac?
	int health;	// always = POWER_SUPPLY_HEALTH_GOOD
	int status;	// charging?
};

struct max17040_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct power_supply		battery[3];//0 for battery, 1 for USB, 2 for AC
	struct max17040_battery_info	bat_info;
	//int (*charger_enable)(void);
};
static struct max17040_chip *this_chip = NULL;
static int s5p_battery_initial = 0;

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB_AC,
	//CHARGER_AC,
	CHARGER_DISCHARGE
} charger_type_t;

#if 0
int batt_is_real_full = 0;
EXPORT_SYSMBOL(batt_is_real_full);
#endif

//extern int battery_is_full(void);
static void max17040_get_status()
{
	struct max17040_chip *chip = this_chip;

	if (chip->bat_info.charging_source == CHARGER_USB_AC) {
		if(chip->bat_info.soc >= MAX17040_BATTERY_FULL) {
			chip->bat_info.status = POWER_SUPPLY_STATUS_FULL;
			chip->bat_info.soc = 100;
		}
		else
			
			chip->bat_info.status = POWER_SUPPLY_STATUS_CHARGING;
	}
	else
		chip->bat_info.status = POWER_SUPPLY_STATUS_NOT_CHARGING;

#if 0
	int charging_flag = 0;
	if (!chip->charger_enable) {
		chip->bat_info.status = POWER_SUPPLY_STATUS_UNKNOWN;
		return;
	}

	if (chip->bat_info.charging_source == CHARGER_USB
			|| chip->bat_info.charging_source == CHARGER_AC) {
		charging_flag = chip->charger_enable();
		if (charging_flag == 1)
		{
			printk("It's still in charging\n");
			batt_is_real_full = 0;
			chip->bat_info.status = POWER_SUPPLY_STATUS_CHARGING;
		}
		else if (charging_flag == 2)
		{
			if(batt_is_real_full >= BATT_REAL_FULL_COUNT )
			{
				printk("battery is real full\n");
				if(batt_is_real_full == BATT_REAL_FULL_COUNT)
					battery_is_full();
				//chip->bat_info.soc = 100;
				if(chip->bat_info.soc >= BATT_FULL_LEVEL)
				{	chip->bat_info.soc = 100;
					chip->bat_info.status = POWER_SUPPLY_STATUS_NOT_CHARGING;//POWER_SUPPLY_STATUS_FULL;
				} 
				else
				{
					printk("It's still not full ????\n");
				}
				//chip->bat_info.status = POWER_SUPPLY_STATUS_FULL;//POWER_SUPPLY_STATUS_FULL;
			}
			else
			{
				printk("charger full or not ?\n");
				if(chip->bat_info.soc >= BATT_FULL_LEVEL)
				{	chip->bat_info.soc = 100;
					chip->bat_info.status = POWER_SUPPLY_STATUS_NOT_CHARGING;//POWER_SUPPLY_STATUS_FULL;
				}
				else
				{
					printk("It's still not full\n");
				}
			}
			batt_is_real_full++;

		}
		else
		{
			printk("In current design,should not go here...\n");
			if(chip->bat_info.soc >= BATT_FULL_LEVEL)
			{	chip->bat_info.soc = 100;
				chip->bat_info.status = POWER_SUPPLY_STATUS_NOT_CHARGING;//POWER_SUPPLY_STATUS_FULL;
			}

		}
	} else {
		printk("It's not charging\n");
		batt_is_real_full = 0;
		chip->bat_info.status = POWER_SUPPLY_STATUS_NOT_CHARGING;//POWER_SUPPLY_STATUS_DISCHARGING;
	}
#endif
}
int  is_charger_online(void)//mj for charging detect.
{
	//struct fg8997_chip *chip = this_chip;
	return  (this_chip->bat_info.charging_source == CHARGER_USB_AC);
}

//int max17040_get_soc();
static int s5p_cable_status_update(int source)
{
	int ret = 0;

	if(!s5p_battery_initial)
		return -EPERM;

	switch(source) {
		case CHARGER_BATTERY:
			this_chip->bat_info.charging_source = CHARGER_BATTERY;
			break;
		case CHARGER_USB_AC:
			this_chip->bat_info.charging_source = CHARGER_USB_AC;
			break;
		case CHARGER_DISCHARGE:
			this_chip->bat_info.charging_source = CHARGER_DISCHARGE;
			break;
		default:
			printk("%s : Nat supported status\n", __func__);
			ret = -EINVAL;
	}
#if 0
	source = this_chip->bat_info.charging_source;

	if (source == CHARGER_USB || source == CHARGER_AC) {
		wake_lock(&vbus_wake_lock);
	} else {
		/* give userspace some time to see the uevent and update
		 * LED state or whatnot...
		 */
		wake_lock_timeout(&vbus_wake_lock, HZ / 2);
	}
#endif

	if(!ret)
	{
		//this_chip->bat_info.soc = max17040_get_soc();
		max17040_get_status();
	}

	/* if the power source changes, all power supplies may change state */
	power_supply_changed(&(this_chip->battery[CHARGER_BATTERY]));
	/*
	   power_supply_changed(&(this_chip->battery[CHARGER_USB]));
	   power_supply_changed(&(this_chip->battery[CHARGER_AC]));
	   */
	return ret;
}

void s5p_cable_check_status(int flag)
{
	charger_type_t status = 0;

	if (flag ==0)  // Battery
		status = CHARGER_BATTERY;
	else if (flag == 1) 
		status = CHARGER_USB_AC;

	s5p_cable_status_update(status);
}
EXPORT_SYMBOL(s5p_cable_check_status);

static int max17040_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct max17040_chip *chip = container_of(psy,
			struct max17040_chip, battery);

	//printk("%s(),%d\n",__FUNCTION__,psp);
	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = chip->bat_info.status;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = chip->bat_info.vcell;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = chip->bat_info.soc;
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = chip->bat_info.health;// POWER_SUPPLY_HEALTH_GOOD;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = chip->bat_info.tech;// POWER_SUPPLY_TECHNOLOGY_LION;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = 1;
			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = chip->bat_info.temp10;// get10Temp();
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

static int s5p_power_get_property(struct power_supply *bat_ps, 
		enum power_supply_property psp, 
		union power_supply_propval *val)
{
	charger_type_t charger = this_chip->bat_info.charging_source;

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			if (bat_ps->type == POWER_SUPPLY_TYPE_MAINS)
				val->intval = (charger == CHARGER_USB_AC ? 1 : 0);
			else
				val->intval = 0;
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

#if 0
static int max17040_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int max17040_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}
#endif

static int max17040_i2c_rx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_client *max17040_client = this_chip->client;
	struct i2c_msg msgs[] = {
		{
			.addr	= max17040_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= max17040_client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		}
	};
	if(NULL == this_chip || NULL == max17040_client || NULL == max17040_client->addr)
		return -EIO;

	for (i = 0; i < MAX17040_RETRY_COUNT; i++) {
		if (i2c_transfer(max17040_client->adapter, msgs, 2) > 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= MAX17040_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, MAX17040_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int max17040_i2c_tx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_client *max17040_client = this_chip->client;
	struct i2c_msg msg[] = {
		{
			.addr	= max17040_client->addr,
			.flags	= 0,
			.len	= len,
			.buf	= buf,
		}
	};
	if(NULL == this_chip || NULL == max17040_client || NULL == max17040_client->addr)
		return -EIO;

	for (i = 0; i < MAX17040_RETRY_COUNT; i++) {
		if (i2c_transfer(max17040_client->adapter, msg, 1) > 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= MAX17040_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, MAX17040_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int max17040_update_rcomp(int temp)
{
	unsigned char data[4] = {0,0,0,0};
	const int StartingRCOMP = 92;// 20 C = 92, liang, 2011-3-29
	int NewRCOMP;
	int ret;

	if(temp > 20)
		NewRCOMP = StartingRCOMP - (temp-20)*7/8;// -0.875
	else if(temp < 20)
		NewRCOMP = StartingRCOMP + (20-temp)*267/40;// -6.675
	else
		NewRCOMP = StartingRCOMP;

	if(NewRCOMP > 255)
		NewRCOMP = 255;
	else if(NewRCOMP < 0)
		NewRCOMP = 0;

	data[0] = MAX17040_RCOMP_MSB;
	data[1] = NewRCOMP;
	data[2] = 0x00;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error\n",__FUNCTION__);
	}
	return ret;
}

struct module_config {
	unsigned char addr;
	unsigned char content[16];
};

struct module_config configs[] = { // for v8
#if 1// 2011.3.24 robin
	{0x40,{0xA8,0x00,0xB7,0xF0,0xB9,0x70,0xBB,0x60,0xBC,0x20,0xBD,0x30,0xBD,0xE0,0xBE,0x90}},
	{0x50,{0xC1,0x70,0xC3,0x50,0xC5,0xB0,0xC7,0xE0,0xCA,0x20,0xCD,0x70,0xCF,0x80,0xD2,0xA0}},
	{0x60,{0x02,0xF0,0x2D,0x20,0x25,0x30,0x46,0x00,0x40,0x50,0x44,0xF0,0x44,0xF0,0x20,0xF0}},
	{0x70,{0x17,0xB0,0x17,0xA0,0x1B,0xF0,0x0D,0xC0,0x0E,0x50,0x1D,0x60,0x00,0xF0,0x00,0xF0}},
#endif
#if 0
	{0x40,{0x9A,0xC0,0xB7,0xA0,0xB8,0xC0,0xB9,0xD0,0xBA,0xF0,0xBB,0xD0,0xBC,0xE0,0xBD,0xD0}},
	{0x50,{0xBF,0x10,0xBF,0xE0,0xC1,0x70,0xC3,0x70,0xC5,0x50,0xC8,0xE0,0xCC,0xE0,0xD0,0x90}},
	{0x60,{0x00,0x90,0x0F,0x60,0x12,0x40,0x13,0x00,0x21,0x00,0x1C,0x00,0x20,0x00,0x16,0x00}},
	{0x70,{0x20,0x80,0x10,0xE0,0x0B,0xE0,0x0A,0xF0,0x0B,0xF0,0x07,0xE0,0x09,0xF0,0x09,0xF0}},
#endif
#if 0
	{0x40,{0x7F,0xE0,0xB6,0x70,0xB9,0x20,0xBA,0x20,0xBB,0x80,0xBC,0x90,0xBD,0x90,0xBE,0x80}},
	{0x50,{0xBF,0x80,0xC0,0xA0,0xC2,0xB0,0xC5,0x40,0xC6,0xE0,0xC9,0x00,0xCD,0x20,0xD1,0xE0}},
	{0x60,{0x00,0x20,0x0B,0x00,0x17,0x20,0x19,0x00,0x22,0x00,0x2B,0x00,0x16,0x50,0x0D,0xE0}},
	{0x70,{0x1A,0x70,0x0B,0xF0,0x0B,0xE0,0x0B,0x80,0x0B,0xF0,0x08,0x60,0x07,0x00,0x07,0x00}},
#endif
};

static int max17040_load_model()
{
	unsigned char data[20];
	u8 OriginalRCOMP1,OriginalRCOMP2,OriginalOCV1,OriginalOCV2;
	int ret;
	unsigned char *config;

	memset(data,0,sizeof(data));

	// Unlock Model Access
	data[0] = 0x3E;// OCV
	data[1] = 0x4A;
	data[2] = 0x57;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error1\n",__FUNCTION__);
		return ret;
	}

	// Read RCOMP and OCV
	data[0] = MAX17040_RCOMP_MSB;
	ret = max17040_i2c_rx_data(data,4);
	if(ret)
	{
		printk("%s() error2\n",__FUNCTION__);
		return ret;
	}
	OriginalRCOMP1 = data[0];
	OriginalRCOMP2 = data[1];
	OriginalOCV1 = data[2];
	OriginalOCV2 = data[3];

	// Write OCV
	data[0] = 0x0E;
	data[1] = 0xDC;
	data[2] = 0xA0;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error3\n",__FUNCTION__);
		return ret;
	}

	// Write RCOMP to a Maximum value of 0xFF00h
	data[0] = MAX17040_RCOMP_MSB;
	data[1] = 0xFF;
	data[2] = 0x00;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error4\n",__FUNCTION__);
		return ret;
	}

	// Write the Model
	config = &(configs[0].addr);// 0x40
	ret = max17040_i2c_tx_data(config,17);
	if(ret)
	{
		printk("%s() error5\n",__FUNCTION__);
		return ret;
	}
	config = &(configs[1].addr);// 0x50
	ret = max17040_i2c_tx_data(config,17);
	if(ret)
	{
		printk("%s() error6\n",__FUNCTION__);
		return ret;
	}
	config = &(configs[2].addr);// 0x60
	ret = max17040_i2c_tx_data(config,17);
	if(ret)
	{
		printk("%s() error7\n",__FUNCTION__);
		return ret;
	}
	config = &(configs[3].addr);// 0x70
	ret = max17040_i2c_tx_data(config,17);
	if(ret)
	{
		printk("%s() error8\n",__FUNCTION__);
		return ret;
	}

	msleep(150);

	// Write OCV
	data[0] = 0x0E;
	data[1] = 0xDC;
	data[2] = 0xA0;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error9\n",__FUNCTION__);
		return ret;
	}

	msleep(200);

	// Read SOC and Compare to expected result
	data[0] = MAX17040_SOC_MSB;
	ret = max17040_i2c_rx_data(data,2);
	if(ret)
	{
		printk("%s() error10\n",__FUNCTION__);
		return ret;
	}
	if(data[0] >= 0xC9 && data[0] <=0xCB)
	{
		//printk("%s success, 0x%x\n",__FUNCTION__,data[0]);
		NULL;
	}
	else
	{
		printk("%s fail, 0x%X\n",__FUNCTION__,data[0]);
		return -1;
	}

	// Restore RCOMP and OCV
	data[0] = MAX17040_RCOMP_MSB;
	data[1] = OriginalRCOMP1;
	data[2] = OriginalRCOMP2;
	data[3] = OriginalOCV1;
	data[4] = OriginalOCV2;
	ret = max17040_i2c_tx_data(data,5);
	if(ret)
	{
		printk("%s() error11\n",__FUNCTION__);
		return ret;
	}

	// Lock Model Access
	data[0] = 0x3E;
	data[1] = 0x00;
	data[2] = 0x00;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error12\n",__FUNCTION__);
		return ret;
	}

	return ret;
}

static int max17040_reset()
{
	unsigned char data[4] = {0,0,0,0};
	int ret;

	data[0] = MAX17040_CMD_MSB;
	data[1] = 0x54;
	data[2] = 0x00;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
		printk("%s() error\n",__FUNCTION__);
	return ret;
}

static int max17040_quick_start()
{
	unsigned char data[4] = {0,0,0,0};
	int ret;

	data[0] = MAX17040_MODE_MSB;
	data[1] = 0x40;
	data[2] = 0x00;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
		printk("%s() error\n",__FUNCTION__);
	return ret;
}

static int max17040_get_vcell()
{
	unsigned char data[4] = {0,0,0,0};
	u8 msb;
	u8 lsb;
	int unit;
	int ret;

	data[0] = MAX17040_VCELL_MSB;
	ret = max17040_i2c_rx_data(data,2);
	if(ret)
	{
		printk("%s() error\n",__FUNCTION__);
		return ret;
	}
	msb = data[0];
	lsb = data[1];

	//--------------------------------------------------
	//     MSB--ADDRESS 02h      |   LSB--ADDRESS 03H  |
	// ^11 ^10 ^9 ^8 ^7 ^6 ^5 ^4 | ^3 ^2 ^1 ^0 0 0 0 0 |
	//--------------------------------------------------
	unit = (msb << 4) + (lsb >> 4);
#if 1// 1.25mV for MAX17040
	ret = (5*unit)>>2;
#else// 2.5mV for MAX17041
	ret = (5*unit)>>1;
#endif
	//this_chip->bat_info.vcell = (ret+0)*1000;// bias
#if MAX17040_DBG_MSG
	printk("%s(): vcell: %dmV\n",__FUNCTION__,ret);
#endif
	return ret;

}

int max17040_get_soc()
{
	unsigned char data[4] = {0,0,0,0};
	u8 msb;
	u8 lsb;
	int ret;
	int timeout=0;

	data[0] = MAX17040_SOC_MSB;
	ret = max17040_i2c_rx_data(data,2);
	if(ret)
	{
		printk("%s() error\n",__FUNCTION__);
		return ret;
	}
	msb = data[0];
	lsb = data[1];

	//------------------------------------------------------------
	//                  DATESHEET of MAX17040                    |
	//------------------------------------------------------------
	//     MSB--ADDRESS 04h    |        LSB--ADDRESS 05H         |
	// ^7 ^6 ^5 ^4 ^3 ^2 ^1 ^0 | ^-1 ^-2 ^-3 ^-4 ^-5 ^-6 ^-7 ^-8 |
	//------------------------------------------------------------
#if 1 // 2011 -03-24 
	/* The custom model for this cell requires a change in the LSB value
	 * from the datasheet specification. Nominally, the LSB has a value
	 * of 2^-8, but for this custom model, the LSB has been shifted by one
	 * bit so that it now has a value of 2^-9.
	 * SOCValue = ((SOC1 * 256) + SOC2) * 0.001953125
	 */
	ret = msb>>1;
#else// for ZTE V8
	ret = msb;
#endif
	//this_chip->bat_info.soc = ret;
#if MAX17040_DBG_MSG
	printk("%s(): soc: %d.%02d%%\n",__FUNCTION__,ret,((((msb & 0x1)<<8)+lsb)*100)>>9);
#endif
	if (ret < 0)
	{
#if MAX17040_DBG_MSG
		printk("percent:%d%%\n",ret);
#endif
		ret = 5;
	}
	else if (ret > 100)
	{
		while(ret >100 && timeout <15)
		{

			data[0] = MAX17040_SOC_MSB;
			ret = max17040_i2c_rx_data(data,2);
			if(ret)
			{
				printk("%s() error\n",__FUNCTION__);
				return ret;
			}
			msb = data[0];
			lsb = data[1];
			ret = msb>>1;
			timeout ++;
		}

		if(timeout >= 15)
			ret = 99;
	}
	return ret;
}
EXPORT_SYMBOL(max17040_get_soc);

static int max17040_get_version()
{
	unsigned char data[4] = {0,0,0,0};
	u8 msb;
	u8 lsb;
	int ret;

	data[0] = MAX17040_VER_MSB;
	ret = max17040_i2c_rx_data(data,2);
	if(ret)
	{
		printk("%s() error\n",__FUNCTION__);
		return ret;
	}
	msb = data[0];
	lsb = data[1];

	printk("MAX17040 Fuel-Gauge Ver %d%d\n", msb, lsb);
	return ret;
}

static void max17040_work(struct work_struct *work)
{
	struct max17040_chip *chip = this_chip;
	int ret;
	int someerror = 0;
#if MAX17040_LOWVOL_TEST
	static int flag = 0;
#endif

#if MAX17040_DBG_MSG
	printk("%s()\n",__FUNCTION__);
#endif
	if(NULL == chip)
		printk("Error in %s(), chip is NULL!!!\n",__FUNCTION__);

	chip->bat_info.temp10 = 20*10;

	ret = max17040_get_vcell();
	if (ret<0)
		chip->bat_info.vcell = 3800*1000;// error
	else
		chip->bat_info.vcell = ret*1000;
	ret = max17040_get_soc();
	if (ret<0)
	{
		chip->bat_info.soc = 80;// error
		someerror = 1;
	}
	else
		chip->bat_info.soc = ret;
#if 1 //modify soc data to trigger proper shutdown action
	if (chip->bat_info.soc <= 10 && chip->bat_info.vcell >= 3500000)
		chip->bat_info.soc = 15;
	if (chip->bat_info.vcell < 3500000)
		chip->bat_info.soc = 4;
#endif
#if MAX17040_LOWVOL_TEST
	flag++;
	if (flag > 4)
		chip->bat_info.soc = 5;
	else if(flag > 3)
		chip->bat_info.soc = 10;
	else if(flag > 2)
		chip->bat_info.soc = 15;
#endif
#if 0
	if (chip->bat_info.soc <= 15)
		LED_RED_ON;
#endif

	max17040_get_status();

	//chip->bat_info.soc=80; // report 80 ... shengliang 

	power_supply_changed(&(chip->battery[CHARGER_BATTERY]));

	if (someerror)
		schedule_delayed_work(&chip->work, msecs_to_jiffies(1000));
	else
		schedule_delayed_work(&chip->work, msecs_to_jiffies(30000));
}

static enum power_supply_property max17040_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
};

static enum power_supply_property s5p_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

#ifdef CONFIG_CHARGER_MAX8903
extern int is_charging(void);
#endif
static int __devinit max17040_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17040_chip *chip;
	int ret;
	int i;

	printk("%s: addr=0x%x @ IIC%d, irq=%d\n",client->name,client->addr,client->adapter->nr,client->irq);
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))//I2C_FUNC_SMBUS_BYTE
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	this_chip = chip;
	chip->client = client;
#if 0//def CONFIG_CHARGER_MAX8903
	chip->charger_enable = is_charging;
#endif

	i2c_set_clientdata(client, chip);

	chip->battery[CHARGER_BATTERY].name		= "battery";
	chip->battery[CHARGER_BATTERY].type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery[CHARGER_BATTERY].get_property	= max17040_get_property;
	chip->battery[CHARGER_BATTERY].properties	= max17040_battery_props;
	chip->battery[CHARGER_BATTERY].num_properties	= ARRAY_SIZE(max17040_battery_props);

	chip->battery[CHARGER_USB_AC].name		= "ac";
	chip->battery[CHARGER_USB_AC].type		= POWER_SUPPLY_TYPE_MAINS;
	chip->battery[CHARGER_USB_AC].supplied_to	= supply_list;
	chip->battery[CHARGER_USB_AC].num_supplicants = ARRAY_SIZE(supply_list);
	chip->battery[CHARGER_USB_AC].get_property	= s5p_power_get_property;
	chip->battery[CHARGER_USB_AC].properties	= s5p_power_props;
	chip->battery[CHARGER_USB_AC].num_properties	= ARRAY_SIZE(s5p_power_props);

	chip->bat_info.charging_source = CHARGER_BATTERY;
	chip->bat_info.health = POWER_SUPPLY_HEALTH_GOOD;
	chip->bat_info.tech = POWER_SUPPLY_TECHNOLOGY_LION;

	for(i=0;i<ARRAY_SIZE(chip->battery);i++)
	{
		ret = power_supply_register(&client->dev, &(chip->battery[i]));
		if (ret) {
			dev_err(&client->dev, "failed to register power supply %d\n",i);
			i2c_set_clientdata(client, NULL);
			kfree(chip);
			return ret;
		}
	}

	ret = max17040_reset();
	if (ret)
		return ret;
	ret = max17040_load_model();
	if (ret)
		return ret;
	ret = max17040_quick_start();
	if (ret)
		return ret;
	ret = max17040_get_version();
	if (ret)
		return ret;

	chip->bat_info.temp10 = 20*10;
/*
	ret = max17040_update_rcomp((chip->bat_info.temp10)/10);
	if(ret)
		return ret;
*/
	s5p_battery_initial = 1;

	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17040_work);
	schedule_delayed_work(&chip->work, 0);

	return 0;
}

static int __devexit max17040_remove(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	int i;

	for(i=0;i<ARRAY_SIZE(chip->battery);i++)
		power_supply_unregister(&(chip->battery[i]));
	cancel_delayed_work(&chip->work);
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int max17040_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work(&chip->work);
	return 0;
}

static int max17040_resume(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	schedule_delayed_work(&chip->work, MAX17040_DELAY);
	return 0;
}

#else

#define max17040_suspend NULL
#define max17040_resume NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id max17040_id[] = {
	{ MAX17040_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17040_id);

static struct i2c_driver max17040_i2c_driver = {
	.driver	= {
		.name	= MAX17040_I2C_NAME,
	},
	.probe		= max17040_probe,
	.remove		= __devexit_p(max17040_remove),
	.suspend	= max17040_suspend,
	.resume		= max17040_resume,
	.id_table	= max17040_id,
};

static int __init max17040_init(void)
{
	printk("MAX17040 Fuel Gauge driver: initialize\n");
	return i2c_add_driver(&max17040_i2c_driver);
}
module_init(max17040_init);

static void __exit max17040_exit(void)
{
	i2c_del_driver(&max17040_i2c_driver);
}
module_exit(max17040_exit);

MODULE_AUTHOR("Minkyu Kang <mk7.kang@samsung.com>");
MODULE_DESCRIPTION("MAX17040 Fuel Gauge");
MODULE_LICENSE("GPL");
