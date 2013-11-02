
/* Lite-On LTR-501ALS Linux Driver
 *
 * Copyright (C) 2011 Cellon Technology Corp (shenzhen)
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <mach/io.h>
#include <linux/slab.h>

#include <plat/gpio-cfg.h>

//add charles.hu
#ifdef 	CONFIG_SENSORS_LTR501ALS_FUNCTION_CONTROL
#include <linux/lsensor.h>
int tp_class = 0;
#endif

#define DRIVER_VERSION			"1.0"
#define LTR558_DEVICE_NAME 		"ltr558"

#define LTR558_DEBUG 			0
#define LTR558_ERR				1
#if LTR558_DEBUG
#define LTRDBG(format, ...)	\
		printk(KERN_INFO "LTR558_DEBUG line[%d] " format, __LINE__, ## __VA_ARGS__)
#else
#define LTRDBG(format, ...)
#endif

#if LTR558_ERR
#define LTRERR(format, ...)	\
		printk(KERN_INFO "LTR558_ERR line[%d] " format, __LINE__, ## __VA_ARGS__)
#else
#define LTRERR(format, ...)
#endif

/* LTR-558 Registers */
#define LTR558_ALS_CONTR		0x80
#define LTR558_PS_CONTR			0x81
#define LTR558_PS_LED			0x82
#define LTR558_PS_N_PULSES		0x83
#define LTR558_PS_MEAS_RATE		0x84
#define LTR558_ALS_MEAS_RATE	0x85
#define LTR558_PART_ID			0x86
#define LTR558_MANUFACTURER_ID	0x87

#define LTR558_INTERRUPT		0x8F
#define LTR558_PS_THRES_UP_0	0x90
#define LTR558_PS_THRES_UP_1	0x91
#define LTR558_PS_THRES_LOW_0	0x92
#define LTR558_PS_THRES_LOW_1	0x93

#define LTR558_ALS_THRES_UP_0	0x97
#define LTR558_ALS_THRES_UP_1	0x98
#define LTR558_ALS_THRES_LOW_0	0x99
#define LTR558_ALS_THRES_LOW_1	0x9A

#define LTR558_INTERRUPT_PERSIST 0x9E

/* 558's Read Only Registers */
#define LTR558_ALS_DATA_CH1_0	0x88
#define LTR558_ALS_DATA_CH1_1	0x89
#define LTR558_ALS_DATA_CH0_0	0x8A
#define LTR558_ALS_DATA_CH0_1	0x8B
#define LTR558_ALS_PS_STATUS	0x8C
#define LTR558_PS_DATA_0		0x8D
#define LTR558_PS_DATA_1		0x8E

/* Basic Operating Modes */
#define MODE_ALS_ON_Range1		0x0B
#define MODE_ALS_ON_Range2		0x03
#define MODE_ALS_StdBy			0x00

#define MODE_PS_ON_Gain1		0x03
#define MODE_PS_ON_Gain4		0x07
#define MODE_PS_ON_Gain8		0x0B
#define MODE_PS_ON_Gain16		0x0F
#define MODE_PS_StdBy			0x00

#define PS_RANGE1 				1
#define PS_RANGE4				4
#define PS_RANGE8 				8
#define PS_RANGE16				16

#define ALS_RANGE1_320			1
#define ALS_RANGE2_64K 			2

/* Basicn Sensor Setting */
#define LED_PULSE_FREQUENCY		(0b11) << 5		//60kHZ	
#define LED_DUTY_CYCLE			(0b11) << 3		//100%
#define LED_PEAK_CURRENT			(0b11)			//50mA
#define PS_LED_SETTING			(LED_PULSE_FREQUENCY) | (LED_DUTY_CYCLE) | (LED_PEAK_CURRENT)

#define LED_PULSE_COUNT			(0b1111)			//16
#define PS_N_PULSE_SETTING		LED_PULSE_COUNT

#define PS_MEAS_REPEAT_RATE		(0b0)			//50ms
#define PS_MEAS_RATE_SETING		PS_MEAS_REPEAT_RATE

#define ALS_INTEGRATION_TIME		(0b11) << 3		//100ms
#define ALS_MEAS_REPEAT_RATE	(0b11)			//500ms
#define ALS_MEAS_RATE_SETTING	(ALS_INTEGRATION_TIME) | (ALS_MEAS_REPEAT_RATE)

#define OUTPUT_MODE				(0b0) << 3		//INT output pin keep in triggered state
#define INTERRUPT_POLARITY		(0b0) << 2		//INT output pin active when it is a logic 0
#define INTERRUPT_MODE			(0b11)			//Both ALS and PS measurement can trigger interrupt
#define INTERRUPT_SETTING		(OUTPUT_MODE) | (INTERRUPT_POLARITY) | (INTERRUPT_MODE)

#define PS_PERSIST				(0b10) << 4
#define ALS_PERSIST				(0b00)
#define INTE_PERSIST_SETTING		(PS_PERSIST) | (ALS_PERSIST)

#define PS_GAINRANGE				PS_RANGE16
#define ALS_GAINRANGE				ALS_RANGE2_64K//ALS_RANGE1_320//ALS_RANGE2_64K //crystal changed ,ehanced Sensitivity,2012-10-23

#define P_RANGE_NULL				0
#define P_RANGE_1				1
#define P_RANGE_2				2

#define L_RANGE_NULL				0
#define L_RANGE_1				1

#define OBJECT_IS_DETECTED		128
#define OBJECT_IS_NOT_DETECTED	0

#define PS_ENABLE					1
#define PS_DISABLE				0

#define ALS_ENABLE				1
#define ALS_DISABLE				0

/*  
 * Magic Number
 * ============
 * Refer to file ioctl-number.txt for allocation
 */
#define LIGHT_IOM		 'i'

#define LTR558_IOC_GET_PFLAG		_IOR(LIGHT_IOM, 0x00, short)
#define LTR558_IOC_SET_PFLAG		_IOW(LIGHT_IOM, 0x01, short)

#define LTR558_IOC_GET_LFLAG		_IOR(LIGHT_IOM, 0x10, short)
#define LTR558_IOC_SET_LFLAG		_IOW(LIGHT_IOM, 0x11, short)

/* Power On response time in ms */
#define PON_DELAY				100
#define WAKEUP_DELAY			10

/* Interrupt vector number to use when probing IRQ number.
 * User changeable depending on sys interrupt.
 * For IRQ numbers used, see /proc/interrupts.
 */
#define GPIO_INT_NO				28


#ifdef CONFIG_CELLON_ENG_LOG
#define P_L_LOG_ON 1
#else 
#define P_L_LOG_ON 0
#endif

#define P_L_printk(a...) \
	do { \
	if (P_L_LOG_ON) \
		printk(KERN_INFO a); \
	} while (0)

/*****************************************
Notice:do not use any "atomic" operation in this driver,
	there is no need in single core.
*******************************************/
struct ltr558_data {
	struct work_struct irq_workqueue;
	struct input_dev *ltr558_input;
	struct i2c_client *client;
	int ltr558_irq;
	short p_enable;
	short l_enable;
};

static struct ltr558_data *ltr558_data = NULL;
bool ltr558_is_good = false;
#if defined(CONFIG_SENSORS_TSL2550)
bool already_have_p_sensor = false;	//check in tsl2550.c
#endif

/*******************************************
function:I2C read protocol
parameter: 
	@regnum: the register want to read
return:
	This executes the SMBus "read byte" protocol, returning 
	negative errno, else a data byte received from the device.
********************************************/

static int ltr558_i2c_read_reg(u8 regnum)
{
	int readdata;
	
	readdata = i2c_smbus_read_byte_data(ltr558_data->client, regnum);	
	
	return readdata;

}


/*******************************************
function:I2C write protocol
parameter: 
	@regnum: the register want to read
	@value: the value want to write to regum
return:
	This executes the SMBus "write byte" protocol, returning
	negative errno else zero on success.
********************************************/
static int ltr558_i2c_write_reg(u8 regnum, u8 value)
{
	int writeerror;
	
	writeerror = i2c_smbus_write_byte_data(ltr558_data->client, regnum, value);
	
	return writeerror;

}
int ltr558_i2c_read_reg_test(u8 regnum)
{
	int readdata;
	
	readdata = i2c_smbus_read_byte_data(ltr558_data->client, regnum);	
	
	return readdata;

}
EXPORT_SYMBOL(ltr558_i2c_read_reg_test);

/*******************************************
function:I2C write protocol
parameter: 
	@regnum: the register want to read
	@value: the value want to write to regum
return:
	This executes the SMBus "write byte" protocol, returning
	negative errno else zero on success.
********************************************/
int ltr558_i2c_write_reg_test(u8 regnum, u8 value)
{
	int writeerror;
	
	writeerror = i2c_smbus_write_byte_data(ltr558_data->client, regnum, value);
	
	return writeerror;

}
EXPORT_SYMBOL(ltr558_i2c_write_reg_test);
/*******************************************
function:enable P-sensor
parameter: 
	@gainrange: select the gain range
return:
	This executes the SMBus "write byte" protocol, returning
	negative errno else zero on success.
inportant:
	Other settings like timing and threshold to be set BEFORE here, 
	if required. Not set and kept as device default for now.
********************************************/
static int ltr558_ps_enable(int gainrange){
	int error;
	int setgain;

	switch (gainrange) {
		case PS_RANGE1:
			setgain = MODE_PS_ON_Gain1;
			break;

		case PS_RANGE4:
			setgain = MODE_PS_ON_Gain4;
			break;

		case PS_RANGE8:
			setgain = MODE_PS_ON_Gain8;
			break;

		case PS_RANGE16:
			setgain = MODE_PS_ON_Gain16;
			break;

		default:
			setgain = MODE_PS_ON_Gain1;
			break;
	}

	error = ltr558_i2c_write_reg(LTR558_PS_CONTR, setgain); 
	ltr558_i2c_read_reg(0x81);
	LTRDBG("0x81 = [%x]\n", ltr558_i2c_read_reg(0x81));
	mdelay(WAKEUP_DELAY);
	
	return error;
}


/*******************************************
function:Put P-sensor into Standby mode
parameter: 
	none
return:
	This executes the SMBus "write byte" protocol, returning
	negative errno else zero on success.
********************************************/
static int ltr558_ps_disable(void){
	int error;
	
	error = ltr558_i2c_write_reg(LTR558_PS_CONTR, MODE_PS_StdBy); 
	LTRDBG("0x81 = [%x]\n", ltr558_i2c_read_reg(0x81));
	
	return error;
}

/*******************************************
function:read P-sensor data
parameter: 
	none
return:
	This executes the SMBus "read byte" protocol, returning 
	negative errno, else return the P-sensor data .
********************************************/
static int ltr558_ps_read(void){
	int psval_lo, psval_hi, psdata;

	psval_lo = ltr558_i2c_read_reg(LTR558_PS_DATA_0);
	if (psval_lo < 0){
		LTRERR("read LTR558_PS_DATA_0 fail\n");
		psdata = psval_lo;
		goto out;
	}
		
	psval_hi = ltr558_i2c_read_reg(LTR558_PS_DATA_1);
	if (psval_hi < 0){
		LTRERR("read LTR558_PS_DATA_1 fail\n");
		psdata = psval_hi;
		goto out;
	}

	//printk("data_lo == %d data_hi == %d \n",psval_lo,psval_hi);
	psdata = ((psval_hi & 7) << 8) + psval_lo;

	out:
	return psdata;
}

/*******************************************
function:enable L-sensor
parameter: 
	@gainrange: select the gain range
return:
	This executes the SMBus "write byte" protocol, returning
	negative errno else zero on success.
inportant:
	Other settings like timing and threshold to be set BEFORE here, 
	if required. Not set and kept as device default for now.
********************************************/
static int ltr558_als_enable(int gainrange){
	int error;

	if (gainrange == ALS_RANGE1_320)
		error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_ON_Range1);
	else if (gainrange == ALS_RANGE2_64K)
		error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_ON_Range2);
	else
		error = -1;

	ltr558_i2c_read_reg(0x80);
	LTRDBG("0x80 = [%x]\n", ltr558_i2c_read_reg(0x80));
	mdelay(WAKEUP_DELAY);

	return error;
}

/*******************************************
function:Put L-sensor into Standby mode
parameter: 
	none
return:
	This executes the SMBus "write byte" protocol, returning
	negative errno else zero on success.
********************************************/
static int ltr558_als_disable(void){
	int error;
	
	error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_StdBy); 
	LTRDBG("0x80 = [%x]\n", ltr558_i2c_read_reg(0x80));
	
	return error;
}

/*******************************************
function:read L-sensor data
parameter: 
	none
return:
	This executes the SMBus "read byte" protocol, returning 
	negative errno, else return the P-sensor data .
inportant:
	must read by sequence 0x88->0x89->0x8A->0x8C
********************************************/
static int ltr558_als_read(void){
	int alsval_ch0_lo, alsval_ch0_hi;
	int alsval_ch1_lo, alsval_ch1_hi;
	int luxdata;
	int ratio;
	int alsval_ch0, alsval_ch1;
	int ch0_coeff, ch1_coeff;
	static int old_luxdata = 1;
	static int time_num1 = 0;
	static int time_num2 = 0;
	int light_value = 0;
	int report_flag = false;
	static int luxdata_out = 200;
#ifdef 	CONFIG_SENSORS_LTR501ALS_FUNCTION_CONTROL
	int differ_tp_light = 0;
#endif
	static int old_range_count = -1;
	static int variable1 = 3;
	static int array[4]  = {0};
	int range_count, i;

	alsval_ch1_lo = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH1_0);
	if (alsval_ch1_lo < 0){
		LTRERR("read LTR558_ALS_DATA_CH1_0 fail\n");
		luxdata = alsval_ch1_lo;
		goto out;
	}
	alsval_ch1_hi = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH1_1);
	if (alsval_ch1_hi < 0){
		LTRERR("read LTR558_ALS_DATA_CH1_1 fail\n");
		luxdata = alsval_ch1_hi;
		goto out;
	}
	alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;

	alsval_ch0_lo = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH0_0);
	if (alsval_ch0_lo < 0){
		LTRERR("read LTR501_ALS_DATA_CH0_0 fail\n");
		luxdata = alsval_ch0_lo;
		goto out;
	}
	alsval_ch0_hi = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH0_1);
	if (alsval_ch0_hi < 0){
		LTRERR("read LTR558_ALS_DATA_CH0_1 fail\n");
		luxdata = alsval_ch0_hi;
		goto out;
	}
	alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;
//	printk("alsval_ch0_hi[%d],  alsval_ch0_lo[%d]\n", alsval_ch0_hi, alsval_ch0_lo);
//	printk("alsval_ch1_hi[%d],  alsval_ch1_lo[%d]\n", alsval_ch1_hi, alsval_ch1_lo);
//	printk("alsval_ch0[%d],  alsval_ch1[%d]\n", alsval_ch0, alsval_ch1);
	
	LTRDBG("alsval_ch0[%d],  alsval_ch1[%d]\n", alsval_ch0, alsval_ch1);

	if ((0 == alsval_ch0) && ( 0 == alsval_ch1)){
		luxdata = 0;
		goto out;
	}
	
	ratio = (100 * alsval_ch1)/(alsval_ch1 + alsval_ch0);
//	printk("ratio[%d]\n", ratio);
	if (ratio < 45)
	{
		ch0_coeff = 17743;
		ch1_coeff = -11059;
	}
	else if ((ratio >= 45) && (ratio < 64))
	{
		ch0_coeff = 37725;
		ch1_coeff = 13363;
	}
	else if ((ratio >= 64) && (ratio < 85))
	{
		ch0_coeff = 16900;
		ch1_coeff = 1690;
	}
	else if ((ratio >= 85) && (ratio < 99))
	{
		ch0_coeff = 0;
		ch1_coeff = 0;
	}
	else if(ratio == 100)
	{
		if(alsval_ch1 < 1000)
		{
			ch0_coeff = 0;
			ch1_coeff = 0;
		}
		else
		{
//			ch0_coeff = 0;
//			ch1_coeff = 100000;
			luxdata = 100000;
			goto out;
			
		}
	}
	luxdata = ((alsval_ch0 * ch0_coeff) - (alsval_ch1 * ch1_coeff))/10000;
	///luxdata = ((alsval_ch0 * ch0_coeff) - (alsval_ch1 * ch1_coeff))/900000;///600000; //crystal changed ,ehanced Sensitivity,2012-10-23
out:
/*	if (0 == luxdata)
	{
		luxdata = 1;
	}
*/	
//	printk("luxdata[%d]\n", luxdata);
#ifdef CONFIG_SENSORS_LTR501ALS_FUNCTION_CONTROL	
	if ( (tp_class == B_Y_TOUCH) && (luxdata > 1) )		//compatible different Tp light value
	{

		if ( luxdata > (LEVEL_1_LUXDATA_DIV + 2 ) && luxdata < (LEVEL_2_LUXDATA_DIV + 2) ){
			differ_tp_light = DIFFER_TP_LIGHT_1 ;
		}
		else if ( luxdata >= (LEVEL_2_LUXDATA_DIV + 2) && luxdata < (LEVEL_3_LUXDATA_DIV + 2) ){
			differ_tp_light = DIFFER_TP_LIGHT_2 ;
		}
		else if ( luxdata >= (LEVEL_3_LUXDATA_DIV + 2) && luxdata < (LEVEL_4_LUXDATA_DIV + 5) ){
			differ_tp_light = DIFFER_TP_LIGHT_3 ;
		}
		else if ( luxdata >= (LEVEL_4_LUXDATA_DIV + 5) ){
			differ_tp_light = DIFFER_TP_LIGHT_4 ;
		}
	
		if ( POLARITY_FLAG )
		{
			luxdata += differ_tp_light ;
		}
		else
		{
			if( luxdata > differ_tp_light )
			{
				luxdata -= differ_tp_light ;
			}
		}
	}

	if ( luxdata > (LEVEL_4_LUXDATA_DIV + 5) ){
		return luxdata * MULTIPLE_VALUE;			//LEVEL5 -> LEVEL1,2,3,4  immediately return
	}

	
	if (luxdata == 1)		//counter for "avoid level changed too quickly"
	{
		time_num1++;
		time_num2 = 0;
	}
	else
	{
		time_num1 = 0;
		if (luxdata >= (LEVEL_2_LUXDATA_DIV + 2) && luxdata < (LEVEL_3_LUXDATA_DIV - 2))
		{
			time_num2++;
		}
		else
		{
			time_num2 = 0;
		}
	}

	if ((time_num1 < 3 && luxdata < 2) || 
		(luxdata > (LEVEL_1_LUXDATA_DIV - 2) && luxdata < (LEVEL_1_LUXDATA_DIV + 2)) ||
		(luxdata > (LEVEL_2_LUXDATA_DIV - 3) && luxdata < (LEVEL_2_LUXDATA_DIV + 3)) || 
		(luxdata > (LEVEL_3_LUXDATA_DIV - 2) && luxdata < (LEVEL_3_LUXDATA_DIV + 2)) || 
		(time_num2 < 3 && (luxdata > (LEVEL_2_LUXDATA_DIV + 2) && luxdata < (LEVEL_3_LUXDATA_DIV -2))) ||
		(luxdata > (LEVEL_4_LUXDATA_DIV - 5) && luxdata < (LEVEL_4_LUXDATA_DIV + 5)))
		
	{
		luxdata = old_luxdata ;
	}else{
		old_luxdata = luxdata ;
	}

	luxdata =  luxdata * MULTIPLE_VALUE;
#else
	if(luxdata >= 0 && luxdata < variable1)
		range_count = 0;
	else if(luxdata >= variable1 && luxdata < 150)
		range_count = 1;
	else if(luxdata >= 150 && luxdata < 3000)
		range_count = 2;
	else if(luxdata >= 3000)
		range_count = 3;

	if(range_count == old_range_count)
	{
		array[range_count] ++;
		if(array[range_count] == 3)
		{
			for(i = 0; i <= 3; i ++)
			{
				array[i] = 0;
			}
			light_value = range_count;
			report_flag = true;
		}
	}
	else
	{
		for(i = 0; i <= 3; i ++)
		{
			array[i] = 0;
		}
		array[range_count] ++;
	}
	old_range_count = range_count;

	if(report_flag == true)
	{
		switch(light_value)
			{
			case 0 :
				luxdata_out = (luxdata - 3 < 0) ? luxdata : 2;//10;
				variable1 = 7;
				break;
			case 1:
				luxdata_out = luxdata;//200;
				variable1 = 3;
				break;
			case 2:
				luxdata_out = luxdata;//400;
				variable1 = 3;
				break;
			case 3:
				luxdata_out = luxdata;//1000;
				variable1 = 3;
				break;
			default:
				break;
			}
		report_flag = false;
	}
/*	if (luxdata == 1)
	{
		time_num1++;
		time_num2 = 0;
	}
	else
	{
		time_num1 = 0;
		if (luxdata >= 9)
		{
			time_num2++;
		}
		else
		{
			time_num2 = 0;
		}
	}


	if ((luxdata < 0) || 
		(old_luxdata < 2 && ((time_num2 < 2) || luxdata < 9)) || 
		((old_luxdata >= 2) && ((time_num1 < 2 && luxdata < 2) || (luxdata > 80 && luxdata < 90)))) 
	{
		luxdata = old_luxdata;
	}
	else
	{
		old_luxdata = luxdata;
	}
	
	if (time_num1 > 100)
	{
		time_num1 = 0;
	}
	if (time_num2 > 100)
	{
		time_num2 = 0;
	}
	
	luxdata =  luxdata * 7;

	if (luxdata > 7 && luxdata <= 1000)
		luxdata = (luxdata / 100) * 100 + 50;
	else if (luxdata > 1000)
		luxdata = (luxdata / 1000) * 1000 + 50; ///?500
*/		
#endif	

	return luxdata_out;
}

/*******************************************
function:choose the range for P-sensor to detect interrupt
parameter: 
	@p_range:the range we want to set.
		P_RANGE_1: detect range 200~max
		P_RANGE_2: detect range 0~128
		P_RANGE_NULL: do not detect
return:
	-1:wrong parameter
********************************************/
static int ltr558_set_p_range(int p_range){
	int result = 0;
	
	switch(p_range){
		case P_RANGE_1:
			result = ltr558_i2c_write_reg(LTR558_PS_THRES_UP_0, 0x14); ///d4		//0x2C		
			result = ltr558_i2c_write_reg(LTR558_PS_THRES_UP_1, 0x02);		//0x01
			LTRDBG("0x90 = [%x], 0x91 = [%x]\n", 
			ltr558_i2c_read_reg(LTR558_PS_THRES_UP_0), 
				ltr558_i2c_read_reg(LTR558_PS_THRES_UP_1));

			result = ltr558_i2c_write_reg(LTR558_PS_THRES_LOW_0, 0x00);
			result = ltr558_i2c_write_reg(LTR558_PS_THRES_LOW_1, 0x00);
			LTRDBG("0x92 = [%x], 0x93 = [%x]\n", 
				ltr558_i2c_read_reg(LTR558_PS_THRES_LOW_0),
				ltr558_i2c_read_reg(LTR558_PS_THRES_LOW_1));
			break;
		case P_RANGE_2:
			result = ltr558_i2c_write_reg(LTR558_PS_THRES_UP_0, 0xff);
			result = ltr558_i2c_write_reg(LTR558_PS_THRES_UP_1, 0x07);
			LTRDBG("0x90 = [%x], 0x91 = [%x]\n", 
				ltr558_i2c_read_reg(LTR558_PS_THRES_UP_0), 
				ltr558_i2c_read_reg(LTR558_PS_THRES_UP_1));

			result = ltr558_i2c_write_reg(LTR558_PS_THRES_LOW_0, 0x14);
			result = ltr558_i2c_write_reg(LTR558_PS_THRES_LOW_1, 0x02);
			LTRDBG("0x92 = [%x], 0x93 = [%x]\n", 
				ltr558_i2c_read_reg(LTR558_PS_THRES_LOW_0), 
				ltr558_i2c_read_reg(LTR558_PS_THRES_LOW_1));
			break;
		case P_RANGE_NULL:
			result = ltr558_i2c_write_reg(LTR558_PS_THRES_UP_0, 0xff);		
			result = ltr558_i2c_write_reg(LTR558_PS_THRES_UP_1, 0x07);
			LTRDBG("0x90 = [%x], 0x91 = [%x]\n", 
				ltr558_i2c_read_reg(LTR558_PS_THRES_UP_0), 
				ltr558_i2c_read_reg(LTR558_PS_THRES_UP_1));

			result = ltr558_i2c_write_reg(LTR558_PS_THRES_LOW_0, 0x00);
			result = ltr558_i2c_write_reg(LTR558_PS_THRES_LOW_1, 0x00);
			LTRDBG("0x92 = [%x], 0x93 = [%x]\n", 
				ltr558_i2c_read_reg(LTR558_PS_THRES_LOW_0), 
				ltr558_i2c_read_reg(LTR558_PS_THRES_LOW_1));
			break;
		default:
			return -1;
	}
	return result;
}


/*******************************************
function:choose the range for L-sensor to detect interrupt
parameter: 
	@l_range:the range we want to set.
		L_RANGE_1: detect ch0 range 0~3000
		L_RANGE_NULL: do not detect
return:
	-1:wrong parameter
********************************************/
static int ltr558_set_l_range(int l_range){
	int result =0;

	switch(l_range){
		case L_RANGE_1:
			result = ltr558_i2c_write_reg(LTR558_ALS_THRES_UP_0, 0xB8);		
			result = ltr558_i2c_write_reg(LTR558_ALS_THRES_UP_1, 0x0B);
			LTRDBG("0x97 = [%x], 0x98 = [%x]\n", 
				ltr558_i2c_read_reg(LTR558_ALS_THRES_UP_0), 
				ltr558_i2c_read_reg(LTR558_ALS_THRES_UP_1));
			result = ltr558_i2c_write_reg(LTR558_ALS_THRES_LOW_0, 0xff);
			result = ltr558_i2c_write_reg(LTR558_ALS_THRES_LOW_1, 0xff);
			LTRDBG("0x99 = [%x], 0x9a = [%x]\n", 
				ltr558_i2c_read_reg(LTR558_ALS_THRES_LOW_0), 
				ltr558_i2c_read_reg(LTR558_ALS_THRES_LOW_1));
			break;
		case L_RANGE_NULL:
			result = ltr558_i2c_write_reg(LTR558_ALS_THRES_UP_0, 0xff);		
			result = ltr558_i2c_write_reg(LTR558_ALS_THRES_UP_1, 0xff);
			LTRDBG("0x97 = [%x], 0x98 = [%x]\n", 
				ltr558_i2c_read_reg(LTR558_ALS_THRES_UP_0), 
				ltr558_i2c_read_reg(LTR558_ALS_THRES_UP_1));

			result = ltr558_i2c_write_reg(LTR558_ALS_THRES_LOW_0, 0x00);
			result = ltr558_i2c_write_reg(LTR558_ALS_THRES_LOW_1, 0x00);
			LTRDBG("0x99 = [%x], 0x9a = [%x]\n", 
				ltr558_i2c_read_reg(LTR558_ALS_THRES_LOW_0), 
				ltr558_i2c_read_reg(LTR558_ALS_THRES_LOW_1));
			break;
		default:
			return -1;
	}
	return result;
}

/*******************************************
function:enable or disable the P-sensor
parameter: 
	@p_flag:the flag that we want to disblae or enable P-sensor
		PS_ENABLE: enable P-sensor
		PS_DISABLE: disable P-sensor
return:
	return none zero means fail 
********************************************/
static int ltr558_updata_ps_status(short p_flag){
	int result = 0;
		
	if((PS_ENABLE == p_flag) && (PS_DISABLE == ltr558_data->p_enable)){
		P_L_printk("eanble P-sensor\n");
		result = ltr558_ps_enable(PS_GAINRANGE);
		if (0 == result){
			ltr558_data->p_enable = PS_ENABLE;
		}
	}else if((PS_DISABLE == p_flag) && (PS_ENABLE == ltr558_data->p_enable)){
		P_L_printk("disable P-sensor\n");
		result =  ltr558_ps_disable();
		if(0 == result){
			ltr558_data->p_enable = PS_DISABLE;
		}
	}else{
		LTRDBG("P-sensor already %s\n", (p_flag) ? "enable" : "disable");
		result = 0;
	}

	return result;
}

/*******************************************
function:enable or disable the L-sensor
parameter: 
	@l_flag:the flag that we want to disblae or enable L-sensor
		LS_ENABLE: enable L-sensor
		LS_DISABLE: disable L-sensor
return:
	return none zero means fail 
********************************************/
static int ltr558_updata_als_status(short l_flag){
	int result = 0;
		
	if((ALS_ENABLE == l_flag) && (ALS_DISABLE == ltr558_data->l_enable)){
		P_L_printk("enable L-sensor\n");
		result = ltr558_als_enable(ALS_GAINRANGE);
		if (0 == result){
			ltr558_data->l_enable = ALS_ENABLE;
		}
	}else if((ALS_DISABLE == l_flag) && (ALS_ENABLE == ltr558_data->l_enable)){
		P_L_printk("disable L-sensor\n");
		result =  ltr558_als_disable();
		if(0 == result){
			ltr558_data->l_enable = ALS_DISABLE;
		}
	}else{
		LTRDBG("L-sensor already %s\n", (l_flag) ? "enable" : "disable");
		result = 0;
	}

	return result;
}

static long ltr558_ioctl(struct file *file, unsigned int cmd, unsigned long arg){
	void __user *pa = (void __user *)arg;
	short flag = 0;

	switch (cmd) {
		case LTR558_IOC_SET_PFLAG:
			if (copy_from_user(&flag, pa, sizeof(flag)))
				return -EFAULT;

			if (flag < 0 || flag > 1){
				return -EINVAL;
			}
			
			if (ltr558_updata_ps_status(flag)){
				LTRERR("updata P-sensor status fail\n");
				return -EFAULT;
			}
			break;
		case LTR558_IOC_GET_PFLAG:
			flag = ltr558_data->p_enable;
			if (copy_to_user(pa, &flag, sizeof(flag)))
				return -EFAULT;
			break;
		case LTR558_IOC_SET_LFLAG:
			if (copy_from_user(&flag, pa, sizeof(flag)))
				return -EFAULT;
			
			if (flag < 0 || flag > 1)
				return -EINVAL;
			
			if (ltr558_updata_als_status(flag)){
				LTRERR("updata L-sensor status fail\n");
				return -EFAULT;
			}
			break;
		case LTR558_IOC_GET_LFLAG:
			flag = ltr558_data->l_enable;
			if (copy_to_user(pa, &flag, sizeof(flag)))
				return -EFAULT;
			break;
		default:
			return -EINVAL;
			break;
	}

	return 0;
}


static struct file_operations ltr558_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ltr558_ioctl,
};

static struct miscdevice ltr558_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &ltr558_fops,
};


static void ltr558_schedwork(struct work_struct *work){
	int als_ps_status;
	int interrupt, newdata;
	int final_prox_val;
	int final_lux_val;

	als_ps_status = ltr558_i2c_read_reg(LTR558_ALS_PS_STATUS);
	interrupt = als_ps_status & 10;
	newdata   = als_ps_status & 5 ;
	LTRDBG("interrupt [%d]  newdata  [%d]\n", interrupt, newdata);
	switch (interrupt){
		case 2:
			// PS interrupt
			if ((newdata == 1) | (newdata == 5)){
				
				final_prox_val = ltr558_ps_read();
				if(final_prox_val < 0){
					LTRERR("read P-sensor data fail\n");
					goto out;
				}
				
				LTRDBG("final_prox_val [%d]\n", final_prox_val);
				if (final_prox_val >= 0x214){
					P_L_printk("ltr558 OBJECT_IS_DETECTED\n");
					input_report_abs(ltr558_data->ltr558_input,
									ABS_DISTANCE, OBJECT_IS_DETECTED);
					if (ltr558_set_p_range(P_RANGE_2)){
						LTRERR("set P-sensor to range 2 fail\n");
					}	
				}else if (final_prox_val < 0x214){
					P_L_printk("ltr558 OBJECT_IS_NOT_DETECTED\n");
					input_report_abs(ltr558_data->ltr558_input,
									ABS_DISTANCE, OBJECT_IS_NOT_DETECTED);
					if (ltr558_set_p_range(P_RANGE_1)){
						LTRERR("set P-sensor to range 1 fail\n");
					}	
				}
			}
			break;

		case 8:
			// ALS interrupt
			if ((newdata == 4) | (newdata == 5)){
				
				final_lux_val = ltr558_als_read();
				if(final_lux_val < 0){
					LTRERR("read L-sensor data fail\n");
					goto out;
				}
				
				input_report_abs(ltr558_data->ltr558_input, ABS_MISC, final_lux_val);
				LTRDBG("final_lux_val [%d]\n", final_lux_val);
			}
			break;

		case 10:
			// Both interrupt
			if ((newdata == 1) | (newdata == 5)){
				
				final_prox_val = ltr558_ps_read();
				if(final_prox_val < 0){
					LTRERR("read P-sensor data fail\n");
					goto read_als;
				}
				
				LTRDBG("final_prox_val [%d]\n", final_prox_val);
				if (final_prox_val >= 0x214){
					input_report_abs(ltr558_data->ltr558_input,
									ABS_DISTANCE, OBJECT_IS_DETECTED);
					if (ltr558_set_p_range(P_RANGE_2)){
						LTRERR("set P-sensor to range 2 fail\n");
					}	
				}else if (final_prox_val <0x214){
					input_report_abs(ltr558_data->ltr558_input,
									ABS_DISTANCE, OBJECT_IS_NOT_DETECTED);
					if (ltr558_set_p_range(P_RANGE_1)){
						LTRERR("set P-sensor to range 2 fail\n");
					}	
				}
			}
read_als:
			if ((newdata == 4) | (newdata == 5)){
				
				final_lux_val = ltr558_als_read();
				if(final_lux_val < 0){
					LTRERR("read L-sensor data fail\n");
					goto out;
				}
				
				input_report_abs(ltr558_data->ltr558_input, ABS_MISC, final_lux_val);
				LTRDBG("final_lux_val [%d]\n", final_lux_val);
			}
			break;
	}
out:
	input_sync(ltr558_data->ltr558_input);	
	enable_irq(ltr558_data->ltr558_irq);

}


static irqreturn_t ltr558_irq_handler(int irq, void *dev_id){
	/* disable an irq without waiting */
	disable_irq_nosync(ltr558_data->ltr558_irq);

	/* schedule_work - put work task in global workqueue
	 * @work: job to be done
	 *
	 * Returns zero if @work was already on the kernel-global workqueue and
	 * non-zero otherwise.
	 *
	 * This puts a job in the kernel-global workqueue if it was not already
	 * queued and leaves it in the same position on the kernel-global
	 * workqueue otherwise.
	 */

	//printk(KERN_DEBUG "ltr501_irq_handler\n");
	schedule_work(&(ltr558_data->irq_workqueue));
	
	return IRQ_HANDLED;
}


static int ltr558_gpio_irq(void)
{
	int ret = 0;

	//config irq
	s3c_gpio_cfgpin(EXYNOS4_GPX2(4), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS4_GPX2(4), S3C_GPIO_PULL_UP);
	
	ret = request_irq(ltr558_data->ltr558_irq, ltr558_irq_handler,
					IRQF_TRIGGER_FALLING, LTR558_DEVICE_NAME, NULL);
	if (ret) {
		LTRERR(KERN_ALERT "%s: LTR-558ALS request irq failed.\n", __func__);
		return ret;
	}
	P_L_printk("LTR558 Request irq successfully!\n");
	
	return ret;
}

static bool ltr558_read_id(void){
	if ((0x80 == ltr558_i2c_read_reg(LTR558_PART_ID)) &&
		(0x05 == ltr558_i2c_read_reg(LTR558_MANUFACTURER_ID))){
		ltr558_is_good = true;
	}

	return ltr558_is_good;
}

bool ltr558_sensor_id(void)
{
	P_L_printk("ltr558_sensor_id %d\n", ltr558_is_good);
	return ltr558_is_good;
}
 EXPORT_SYMBOL(ltr558_sensor_id);
 
static int ltr558_config(void){
	int error = 0;
	
	error = ltr558_i2c_write_reg(LTR558_PS_LED, PS_LED_SETTING);				//0x7B
	if (error){
		LTRERR("write LTR558_PS_LED fail\n");
		return error;
	}
	
	error = ltr558_i2c_write_reg(LTR558_PS_N_PULSES, PS_N_PULSE_SETTING);		//0x0F
	if (error){
		LTRERR("write LTR558_PS_N_PULSES fail\n");
		return error;
	}
	
	error = ltr558_i2c_write_reg(LTR558_PS_MEAS_RATE, PS_MEAS_RATE_SETING);		//0x00
	if (error){
		LTRERR("write LTR558_PS_MEAS_RATE fail\n");
		return error;
	}
	
	error = ltr558_i2c_write_reg(LTR558_ALS_MEAS_RATE, ALS_MEAS_RATE_SETTING);	//0x03
	if (error){
		LTRERR("write LTR558_ALS_MEAS_RATE fail\n");
		return error;
	}
	
	error = ltr558_i2c_write_reg(LTR558_INTERRUPT, INTERRUPT_SETTING);			//0x03
	if (error){
		LTRERR("write LTR558_INTERRUPT fail\n");
		return error;
	}
	
	error = ltr558_i2c_write_reg(LTR558_INTERRUPT_PERSIST, INTE_PERSIST_SETTING);	//0x00
	if (error){
		LTRERR("write LTR558_INTERRUPT_PERSIST fail\n");
		return error;
	}

	return 0;
}

static int ltr558_devinit(void){
	int error;	
	int ret=0;

	//mdelay(PON_DELAY);
	ltr558_read_id();
	if (false == ltr558_read_id()){
		LTRERR("ltr558 id is wrong\n");
		return -1;
	}
	P_L_printk("%s: Can read ltr558 id successfully.\n",__func__);

	error = ltr558_als_disable();
	if (error < 0)
		goto exit_disable_sensor_fail;
	error = ltr558_ps_disable();
	if (error < 0)
		goto exit_disable_sensor_fail;

	error = ltr558_config();
	if (error < 0)
		goto exit_config_sensor_fail;

	error = ltr558_set_p_range(P_RANGE_1);
	if (error < 0)
		goto exit_set_sensor_range_fail;
	
	error = ltr558_set_l_range(L_RANGE_1);
	if (error < 0)
		goto exit_set_sensor_range_fail;

	mdelay(WAKEUP_DELAY); 	
	
#if 0
	LTRDBG("0x80 = [%x]\n", ltr558_i2c_read_reg(0x80));
	LTRDBG("0x81 = [%x]\n", ltr558_i2c_read_reg(0x81));
	LTRDBG("0x82 = [%x]\n", ltr558_i2c_read_reg(0x82));
	LTRDBG("0x83 = [%x]\n", ltr558_i2c_read_reg(0x83));
	LTRDBG("0x84 = [%x]\n", ltr558_i2c_read_reg(0x84));
	LTRDBG("0x85 = [%x]\n", ltr558_i2c_read_reg(0x85));
	LTRDBG("0x86 = [%x]\n", ltr558_i2c_read_reg(0x86));
	LTRDBG("0x87 = [%x]\n", ltr558_i2c_read_reg(0x87));
	LTRDBG("0x88 = [%x]\n", ltr558_i2c_read_reg(0x88));
	LTRDBG("0x89 = [%x]\n", ltr558_i2c_read_reg(0x89));
	LTRDBG("0x8a = [%x]\n", ltr558_i2c_read_reg(0x8a));
	LTRDBG("0x8b = [%x]\n", ltr558_i2c_read_reg(0x8b));
	LTRDBG("0x8c = [%x]\n", ltr558_i2c_read_reg(0x8c));
	LTRDBG("0x8d = [%x]\n", ltr558_i2c_read_reg(0x8d));
	LTRDBG("0x8e = [%x]\n", ltr558_i2c_read_reg(0x8e));
	LTRDBG("0x8f  = [%x]\n",ltr558_i2c_read_reg(0x8f));
	LTRDBG("0x90 = [%x]\n", ltr558_i2c_read_reg(0x90));
	LTRDBG("0x91 = [%x]\n", ltr558_i2c_read_reg(0x91));
	LTRDBG("0x92 = [%x]\n", ltr558_i2c_read_reg(0x92));
	LTRDBG("0x93 = [%x]\n", ltr558_i2c_read_reg(0x93));
	LTRDBG("0x97 = [%x]\n", ltr558_i2c_read_reg(0x97));
	LTRDBG("0x98 = [%x]\n", ltr558_i2c_read_reg(0x98));
	LTRDBG("0x99 = [%x]\n", ltr558_i2c_read_reg(0x99));
	LTRDBG("0x9a = [%x]\n", ltr558_i2c_read_reg(0x9a));
	LTRDBG("0x9e = [%x]\n", ltr558_i2c_read_reg(0x9e));
#endif

	error = 0;

exit_set_sensor_range_fail:
exit_config_sensor_fail:
exit_disable_sensor_fail:
	return error;
}

static int ltr558_probe(struct i2c_client *client, const struct i2c_device_id *id){
	int ret = 0;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	/* Return 1 if adapter supports everything we need, 0 if not. */
	if (!i2c_check_functionality(adapter, 
		I2C_FUNC_SMBUS_WRITE_BYTE |I2C_FUNC_SMBUS_READ_BYTE_DATA)){
		LTRERR(KERN_ALERT "%s: LTR-558ALS functionality check failed.\n", __func__);
		ret = -EIO;
		goto exit_check_functionality_failed;
	}

	/* data memory allocation */
	ltr558_data = kzalloc(sizeof(struct ltr558_data), GFP_KERNEL);
	if (ltr558_data == NULL) {
		LTRERR(KERN_ALERT "%s: LTR-558ALS kzalloc failed.\n", __func__);
		ret = -ENOMEM;
		goto exit_alloc_data_failed;
	}
	ltr558_data->ltr558_irq = client->irq;
	//printk("%s: Irq number is == %d\n",client->irq);
	
	ltr558_data->p_enable = 0;
	ltr558_data->l_enable = 0;
	ltr558_data->ltr558_input = NULL;
	ltr558_data->client = client;
	INIT_WORK(&(ltr558_data->irq_workqueue),ltr558_schedwork);
	i2c_set_clientdata(client, ltr558_data);
	
	/*init device by send i2c command*/
	ret = ltr558_devinit();
	if (ret) {
		LTRERR(KERN_ALERT "%s: LTR-558ALS device init failed.\n", __func__);
		goto exit_device_init_failed;
	}

	/*init  & register input dev*/
	ltr558_data->ltr558_input = input_allocate_device();
	if (ltr558_data->ltr558_input == NULL) {
		LTRERR(KERN_ALERT "%s: LTR-558ALS cllocate input device fail.\n", __func__);
		ret = -ENOMEM;
		goto exit_input_dev_alloc_failed;
	}
	
	ltr558_data->ltr558_input->name = "lightsensor";
	set_bit(EV_ABS, ltr558_data->ltr558_input->evbit);
	input_set_abs_params(ltr558_data->ltr558_input, ABS_MISC, 0, 100000, 0, 0);
	input_set_abs_params(ltr558_data->ltr558_input, ABS_DISTANCE, 0, 128, 0, 0);	

	ret = input_register_device(ltr558_data->ltr558_input);
	if (ret) {
		LTRERR(KERN_ALERT "%s: LTR-558ALS failed to register input device.\n", __func__);
		goto exit_input_register_device_failed;
	}	

	/*register misc device*/
	ret = misc_register(&ltr558_device);
	if (ret) {
		LTRERR(KERN_ALERT "%s: LTR-558ALS misc_register als failed.\n", __func__);
		goto exit_misc_device_register_failed;
	}

	/*init irq*/
	ret = ltr558_gpio_irq();
	if (ret) {
		LTRERR(KERN_ALERT "%s: LTR-558ALS gpio_irq failed.\n", __func__);
		goto exit_irq_request_failed;
	}

#if 0
//add charles.hu  just for test
	mdelay(600);
	ltr558_als_enable(ALS_RANGE2_64K);
	ltr558_ps_enable(PS_GAINRANGE);
	mdelay(600);
	P_L_printk("Ltr558 probe ok!!!\n");
//add end	
#endif	

	return 0;

exit_irq_request_failed:
	misc_deregister(&ltr558_device);
exit_misc_device_register_failed:
exit_input_register_device_failed:
	input_free_device(ltr558_data->ltr558_input);
exit_input_dev_alloc_failed:
exit_device_init_failed:
	kfree(ltr558_data);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return ret;
}


static int ltr558_remove(struct i2c_client *client){
	ltr558_ps_disable();
	ltr558_als_disable();
	
	disable_irq(ltr558_data->ltr558_irq);
	free_irq(gpio_to_irq(GPIO_INT_NO), NULL);
	gpio_free(GPIO_INT_NO);
	
	input_free_device(ltr558_data->ltr558_input);
	misc_deregister(&ltr558_device);	
	kfree(i2c_get_clientdata(client));
	return 0;
}

/*******************************************
we do not need to disable the sensor in suspend because 
it will do in function ltr501_ioctl. 
but we need to call enable_irq_wake, so  the irq will wake 
up the phone when we are in call(use P-sensor).
********************************************/
extern int tc4_get_call_flg();
static int ltr558_suspend(struct i2c_client *client, pm_message_t mesg){
	if (ltr558_data->p_enable){
		disable_irq(ltr558_data->ltr558_irq);
		enable_irq_wake(ltr558_data->ltr558_irq);
	}
	if(tc4_get_call_flg() && (ltr558_ps_read() >= 0x214))
	{
		ltr558_set_p_range(P_RANGE_2);
		ltr558_ps_enable(PS_GAINRANGE);
	}
	LTRDBG("ltr558 -----flag=%d--------%s\n",ltr558_data->p_enable, __func__);
	return 0;
}

static int ltr558_resume(struct i2c_client *client){
	if(ltr558_data->p_enable){
		disable_irq_wake(ltr558_data->ltr558_irq);
		enable_irq(ltr558_data->ltr558_irq);
	}
	LTRDBG("ltr558 -----flag=%d--------%s\n", ltr558_data->p_enable, __func__);
	return 0;
}

void ltr558_shutdown(struct i2c_client *client)
{
	ltr558_als_disable();
	ltr558_ps_disable();
}

static const struct i2c_device_id ltr558_id[] = {
	{ LTR558_DEVICE_NAME, 0 },
	{}
};


static struct i2c_driver ltr558_driver = {
	.probe = ltr558_probe,
	.remove = ltr558_remove,
	.id_table = ltr558_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = LTR558_DEVICE_NAME,
	},
	.suspend  = ltr558_suspend,
	.resume   = ltr558_resume,
	.shutdown = ltr558_shutdown,
};

static int __init ltr558_driverinit(void){
	return i2c_add_driver(&ltr558_driver);
}

static void __exit ltr558_driverexit(void){
	i2c_del_driver(&ltr558_driver);
}


module_init(ltr558_driverinit);
module_exit(ltr558_driverexit);

MODULE_AUTHOR("Cellon Technology Corp");
MODULE_DESCRIPTION("LTR-558ALS Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);

