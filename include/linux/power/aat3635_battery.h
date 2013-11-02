/*
 * Fuel gauge driver for Maxim 17042 / 8966 / 8997
 *  Note that Maxim 8966 and 8997 are mfd and this is its subdevice.
 *
 * Copyright (C) 2011 Samsung Electronics
 * MyungJoo Ham <myungjoo.ham@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __AAT3635_BATTERY_H_
#define __AAT3635_BATTERY_H_
#include <linux/notifier.h>
#include <linux/power_supply.h>
#define AAT3635_I2C_NAME	"aat3635"
#define AAT3635_I2C_ADDR	(0x6a)

#define ENABLE_LIMITED_CHG      0x10
#define CLEAR_LIMITED_CHG       0x11
#define CHECK_CHG		0X64
#define SET_ICL500		0X65
#define SET_ICL100		0X66
#define CHECK_INT2		0X67
#define OVERTEMP_VREG		0XC8
#define NORMALTEMP_VREG		0XC9
#define CHECK_INT1		0XCA
#define CHECK_CONTROL		0xCB
#define NORMALTEMP_VREG_HV	0xCC
#define CHECK_INT3		0XD1


enum wled_ctl_t {
	WLED_DISABLE = 0,
	WLED_ENABLE,
	WLED_STATUS
};

struct aat3635_chg_int_data {
	int gpio_chg_int;
	int gpio_aat_int;
	int aat3635_reg;
	struct delayed_work chg_int_work;
	struct delayed_work aat_int_work;
	struct delayed_work chg_type_work;
};

struct aat3635_platform_data{
	unsigned int  charger_int;
	unsigned int  aat3635_int;
};

struct aat3635_chg_int_notifier {
	struct list_head notifier_link;
	const char *name;
	void (*func)(int int_reg, int value);
};

struct aat3635_battery_info {
	int tech;	// always = POWER_SUPPLY_TECHNOLOGY_LION
	int batt_vol;
	int batt_lev;
	int max_current;	// ADC value of voltage, not used
	int vcell;	// battery voltage * 1000, unit mV
	int soc;	// battery capacity
	int temp10;	// temperature read from akm8973, then *10
	int charging_source;	// battery? usb? ac?
	int health;	// always = POWER_SUPPLY_HEALTH_GOOD
	int batt_is_full;       /* 0 : Not full 1: Full */
	int status;	// charging?
};

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC,
	CHARGER_DISCHARGE
} charger_type_t;

struct aat3635_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct power_supply		charger;//0 for battery, 1 for USB, 2 for AC
	struct aat3635_platform_data	*chg_info;
	struct aat3635_battery_info   battery_info;
	//int (*charger_enable)(void);
};
#endif /* __MAX17042_BATTERY_H_ */
