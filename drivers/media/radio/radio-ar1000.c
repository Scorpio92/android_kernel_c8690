/*=========================================================
Additions and modifications made by Cellon Communications
==========================================================*/
/*
 * driver/media/radio/radio-ar1000.c
 *
 * Driver for AR100 radio chip for linux 2.6.
 * This driver is for AR100 chip from NXP, used in EZX phones from Motorola.
 * The I2C protocol is used for communicate with chip.
 *
 * Based in radio-tea5761.c Copyright (C) 2005 Nokia Corporation
 *
 *  Copyright (c) 2008 Fabio Belavenuto <belavenuto@gmail.com>
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * History:
 * 2008-12-06   Fabio Belavenuto <belavenuto@gmail.com>
 *              initial code
 *
 * TODO:
 *  add platform_data support for IRQs platform dependencies
 *  add RDS support
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>			/* Initdata			*/
#include <linux/delay.h>
#include <linux/videodev2.h>		/* kernel radio structs		*/
#include <linux/i2c.h>			/* I2C				*/
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <mach/gpio.h>
#include <asm/mach-types.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <linux/version.h>      	/* for KERNEL_VERSION MACRO     */
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/err.h>

#include <media/rds.h>

//#include <mach/camera.h>
//#include <media/msm_camera.h>
#include <linux/earlysuspend.h>

//#define DEBUG

 //START: according android_hardware_FmRadio.cpp
#define V4L2_CID_PRIVATE_TAVARUA_STATE    (V4L2_CID_AUDIO_BALANCE + 0)
#define V4L2_CID_PRIVATE_TAVARUA_SRHCMODE (V4L2_CID_AUDIO_BALANCE + 1)
#define V4L2_CID_PRIVATE_TAVARUA_SRCHON   (V4L2_CID_AUDIO_BALANCE + 2)

#define MODE_SEEK 0 
#define MODE_SCAN 1
#define STATE_DISABLE 0
#define STATE_ENABLE  1
 //END: according android_hardware_FmRadio.cpp

static irqreturn_t ar1000_interrupt(int irq, void *dev_id);
bool isRdsOn = 0;
static int ar1000_irq;

// Cellon delete start, Ted Shi, 2012/11/20, for roll back FM wrong check in
// Cellon add start, Ted Shi, 2012/11/16, for add FM LPA mode
/*
int fm_ar1000_is_running;
EXPORT_SYMBOL(fm_ar1000_is_running);
*/
// Cellon add end, Ted Shi, 2012/11/16
// Cellon delete end, Ted Shi, 2012/11/20

#define AR1000_INT_GPIO 	EXYNOS4_GPX3(4)
#if 1
//--  for rds
//++lucha rds
/* insmod options */
static unsigned int debug;
static unsigned int mmbs;
static unsigned int bufblocks = 100;

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "enable debug messages");
module_param(mmbs, int, 0);
MODULE_PARM_DESC(mmbs, "enable MMBS mode: 0=off (default), 1=on");
module_param(bufblocks, int, 64);
MODULE_PARM_DESC(bufblocks, "number of buffered blocks, default 100");

/* ---------------------------------------------------------------------- */
#define PREFIX   "ar1000"
#define dprintk     if (debug) printk
//--lucha rds
#endif

// Cellon modify start, Ted Shi, 2013/01/23, for add eng log switch
//#define AR1000_DEBUG
#ifdef CONFIG_CELLON_ENG_LOG
#define AR1000_DEBUG
#endif
// Cellon modify end, Ted Shi, 2013/01/23
#ifndef AR1000_DEBUG
#undef pr_info(fmt, ...)
#define pr_info(fmt, ...)
#endif

#define DRIVER_VERSION	"v0.01"
#define RADIO_VERSION	KERNEL_VERSION(0, 0, 1)

#define DRIVER_AUTHOR	"Fabio Belavenuto <belavenuto@gmail.com>"
#define DRIVER_DESC	"A driver for the AR100 radio chip for EZX Phones."

#define PINFO(format, ...)\
	printk(KERN_INFO KBUILD_MODNAME ": "\
		DRIVER_VERSION ": " format "\n", ## __VA_ARGS__)
#define PWARN(format, ...)\
	printk(KERN_WARNING KBUILD_MODNAME ": "\
		DRIVER_VERSION ": " format "\n", ## __VA_ARGS__)
# define PDEBUG(format, ...)\
	printk(KERN_DEBUG KBUILD_MODNAME ": "\
		DRIVER_VERSION ": " format "\n", ## __VA_ARGS__)

/* Frequency limits in MHz -- these are European values.  For Japanese
devices, that would be 76000 and 91000.  */
#define ADDR_DEVID	0x1B // the address of DEVID register
#define ADDR_CHIPID 0x1C // the address of CHIPID register 

#define DEVID_VerF	   	0x65B1 // VERSION+MFID
#define CHIPNO_AR1010	0x1010 // 16'b0001 0000 0000 0000
#define CHIPNO_AR1000	0x1000 // 16'b0001 0000 0000 0000
#define VER_F			0x6

#define ON  1
#define OFF 0

#define UEBAND	0x0000 // US & Europe BAND 87.5MHz to 108MHz
#define JBAND	0x1000 // Japen BAND 76MHz to 90MHz
#define JWBAND	0x1800 // Japen BAND (wide) 76 MHz to 108MHz

#define SPACE100K	1
#define SPACE200K	0
#define SEEKUP	1
#define SEEKDOWN 0

#define ADDR_STATUS	0x13 // the address of status register
#define MASK_STC 0x0020  // Seek/Tune/PowerOn complete  D5 in  adress 13H 
#define MASK_SF	 0x0010  //  Seek Fail D4 in address 13H
#define MASK_ST	 0x0008  //  Stereo  D3 in address 13H
#define MASK_READCHAN 0xFF80 // D7~D15 in address 13H
#define SHIFT_READCHAN 7

#define ADDR_RSSI	0x12 
#define MASK_RSSI 	0xFE00
#define SHIFT_RSSI  9


#define ADDR_SEEK_SETTING	0x11
#define SEEK_SETTING	0x2000
#define SEEK_MASK		0xC3FF

#define FREQ_MIN  87500
#define FREQ_MAX 108000
#define FREQ_MUL 16

u8  AR1000VOL[16]= { 
	 0x0,
	 0x5,// step 0
	 0x7,
	 0x9,// step 1
	 0xA,
	 0xA,// step 2
	 0xB,
	  0xB,// 3
	 0xC,
	 0xC,// 4
	 0xD,
	  0xD,// 5
	 0xE,
	 0xE,// 6
	 0xF,
	  0xF// 7
};

static unsigned int AR1000reg[18]=
{
	0xFF7B,		// R0 -- the first writable register .  (disable xo_en)
	0x5B15,		// R1.
	0xD0B9,		// R2.
	0xA006,		//0xA010,// R3   seekTHD = 16
    0x0780,		// R4
    0x28AB,		// R5
	0x6400,		// R6
	0x1EE7,		// R7
    0x7141,		// R8
	0x007D,		// R9
    0x82C6,		// R10  disable wrap
	0x4F55,		// R11. <--- (disable xo_output)
	0x970C,		// R12.
	0xB845,		// R13
	0xFC2D,		// R14
	0x8097,		// R15
	0x04A1,		// R16
    0xDF6A		// R17
};
typedef union DATA_TYPE_S {
	unsigned int i;	
	unsigned char c[2];
	struct { unsigned char
				B8:1,  //LSBit  C[0]
				B9:1, 
				B10:1, 
				B11:1, 
				B12:1, 
				B13:1, 
				B14:1, 
				B15:1,  //MSBit
				B0:1,
				B1:1,
				B2:1,
				B3:1,
				B4:1,
				B5:1,
				B6:1,
				B7:1;
	} BIT;
}DATA_TYPE_S;

static union DATA_TYPE_S Reg_Data[18];
#define AR1000_MUTE_ON { Reg_Data[1].BIT.B9 = ON; }
#define AR1000_MUTE_OFF { Reg_Data[1].BIT.B9 = OFF;}

#define AR1000_TUNE_ON 	{ Reg_Data[2].BIT.B1 = ON;}
#define AR1000_TUNE_OFF { Reg_Data[2].BIT.B1 = OFF;}

#define AR1000_SEEK_ON 	{ Reg_Data[3].BIT.B6 = ON;}
#define AR1000_SEEK_OFF { Reg_Data[3].BIT.B6 = OFF;}

#define AR1000_RDSInt_ON 	{ Reg_Data[1].BIT.B14 = ON;}
#define AR1000_RDSInt_OFF   { Reg_Data[1].BIT.B14 = OFF;}

struct ar1000_device {
	struct i2c_client		*i2c_client;
	struct video_device		*videodev;
	struct mutex			mutex;
	int				users;
	#if 1
       //++ for rds
	struct delayed_work work;
	
	spinlock_t lock;
	unsigned char   *buffer;
	unsigned int buf_size;
	unsigned int rd_index;
	unsigned int wr_index;
	unsigned int block_count;
	unsigned char last_blocknum;
	wait_queue_head_t read_queue;
	int data_available_for_read;
	u8 sync;
       //-- for rds	
    
	   #endif
};

static struct ar1000_device *Temp_radio;
static struct early_suspend early_suspend;
static int radio_nr = -1;


static int srhcmode;
static unsigned int fm_id;
void RDS_Eint_Handler(struct ar1000_device *radio);
void RDS_Enable(struct ar1000_device *radio);
void RDS_Disable(struct ar1000_device *radio);
static int ar1000_i2c_write_reg(struct ar1000_device *radio,unsigned char reg)
{
	unsigned char buf[4];
	struct i2c_msg msg[1]; 
//	memset(buf, 0, sizeof(buf));
	buf[0] = reg;
	buf[1]=Reg_Data[reg].c[1];
	buf[2]=Reg_Data[reg].c[0];

        msg->addr = radio->i2c_client->addr;
	msg->flags = 0;
	msg->len = 3;
	msg->buf = buf;
/*
	struct i2c_msg msg[1] = {
		{ radio->i2c_client->addr, 0, 3,buf},
	};
*/
	if (i2c_transfer(radio->i2c_client->adapter, msg, 1) < 0)
		return -EIO;
	return 0;
}
static int ar1000_i2c_read_reg(struct ar1000_device *radio,unsigned char reg)
{

	DATA_TYPE_S value;
	unsigned char buff[4];
	memset(buff, 0, sizeof(buff));
	
	buff[0] = reg;
	buff[1] = (reg & 0x00);
	struct i2c_msg msgs[] = {
	{
		.addr   = radio->i2c_client->addr,
		.flags = 0,
		.len   = 1,
		.buf   = buff,
	},
	{
		.addr   = radio->i2c_client->addr,
		.flags = I2C_M_RD,
		.len   = 2,
		.buf   = buff,
	},
	};
	if (i2c_transfer(radio->i2c_client->adapter, msgs, 2) < 0)
		return -EIO;
	value.i = buff[0] << 8 | buff[1];
	return value.i;
}
// Cellon add start, Ted Shi, 2012/10/09, for add dead loop times
//static void AR100_init(struct ar1000_device *radio)
static int AR100_init(struct ar1000_device *radio)
// Cellon add end, Ted Shi, 2012/10/09
{
	unsigned int Cnt1; // init R1, R2, ....R17 then R0
	unsigned int status;
// Cellon add start, Ted Shi, 2012/10/09, for add dead loop times
	unsigned int nReadTimes = 0;
// Cellon add end, Ted Shi, 2012/10/09
	Reg_Data[0].i=AR1000reg[0]&0xFFFE;
	pr_info("%s: enter \n",__func__);
	ar1000_i2c_write_reg(radio,0);
	 
	status = ar1000_i2c_read_reg(radio,2);
	//pr_info("%s: R2 %x \n",__func__,status);
	status = ar1000_i2c_read_reg(radio,ADDR_STATUS);
	//pr_info("%s: ADDR_STATUS %x \n",__func__,status);
	 
	for(Cnt1=1;Cnt1<18;Cnt1++)
	{
		Reg_Data[Cnt1].i=AR1000reg[Cnt1];
		ar1000_i2c_write_reg(radio,Cnt1);
	}
    	Reg_Data[0].i=AR1000reg[0];
	ar1000_i2c_write_reg(radio,0);
	
	//Power-On Calibration begins
	// then wait for STC flag
	// maybe you need to delay for a while
	// delay ( 100 ms )
	status = ar1000_i2c_read_reg(radio,ADDR_STATUS);
	status &=MASK_STC; // check STC flag 
// Cellon modify start, Ted Shi, 2012/11/16, for optimize system boot time
// Cellon modify start, Ted Shi, 2012/10/09, for add dead loop times
//	while( status == 0 )
//	while((status == 0)&&(nReadTimes < 3))
	while((status == 0)&&(nReadTimes < 5))
	{
		// maybe you can delay for a while
		// delay ( 100 ms )
		nReadTimes++;
		status = ar1000_i2c_read_reg(radio,ADDR_STATUS);
//		printk("AR1000 STATUS register %x \n",status);	
		pr_info("%s: AR1000 STATUS register %x \n",__func__ ,status);	
		status &=MASK_STC; // check STC flag 
		msleep(100);
	}
	if(!status){
		printk("%s: read STATUS register failed \n",__func__);	
		return -1;
	}
//  	return;
  	return 0;
// Cellon add end, Ted Shi, 2012/10/09
// Cellon modify end, Ted Shi, 2012/11/16
}


/* V4L2 code related */
static struct v4l2_queryctrl radio_qctrl[] = {
	{
		.id            = V4L2_CID_AUDIO_MUTE,
		.name          = "Mute",
		.minimum       = 0,
		.maximum       = 1,
		.default_value = 1,
		.type          = V4L2_CTRL_TYPE_BOOLEAN,
	}
};
static void ar1000_set_freq(struct ar1000_device *radio,int freq)
{
  unsigned int CHAN =0x0000; //actually it's 9-bit
  int val;
  AR1000_TUNE_OFF  //clear TUNE
  ar1000_i2c_write_reg(radio,2);

  val=ar1000_i2c_read_reg(radio,2);
  pr_info("ar1000_set_freq R2 :0x%x , freq : %d = 0x%x \n", val,freq,freq);
  CHAN = freq-690;
  pr_info("%s: set CHAN= 0x%x = %d \n",__func__,CHAN,CHAN);
  Reg_Data[2].i &=0xfe00;
  Reg_Data[2].i|=CHAN;
  AR1000_TUNE_ON
  ar1000_i2c_write_reg(radio,2);
//  val=ar1000_i2c_read_reg(radio,2);
//  pr_info("ar1000_set_freq verify val:0x%x\n", val);
}



static void ar1000_tune_hilo(struct ar1000_device *radio,unsigned int FreqKHz)// unsigned int band, unsigned char space)
{
  unsigned int status;
  unsigned int flag;
  unsigned int rssi;
  
  AR1000_MUTE_ON   // Set Muto ON before TUNE
  ar1000_i2c_write_reg(radio,1);
  
  AR1000_TUNE_OFF  //clear TUNE
  ar1000_i2c_write_reg(radio,2);

  AR1000_SEEK_OFF;	//clear SEEK 
  //Reg_Data[3].BIT.B13=space;  // set SPACE 
  //Reg_Data[3].i = (Reg_Data[3].i & 0xE7FF) | band;  // Set BAND
  ar1000_i2c_write_reg(radio,3);

  //Read Low-Side LO Injection
  //R11 --> clear  D15,  clear D0/D2,  D3 is the same as default
  Reg_Data[11].i = Reg_Data[11].i&0x7FFA; 
  ar1000_i2c_write_reg(radio,11);
  
  //TUNE to FreqKHz with current setting
  ar1000_set_freq(radio,FreqKHz); // this function will turn on TUNE 
 
  // TUNE  begins
  // then wait for STC flag
  
  // maybe you need to delay for a while
  // delay ( 100 ms )
  status = ar1000_i2c_read_reg(radio,ADDR_STATUS);
  flag = status & MASK_STC; // check STC flag 
  while( flag == 0)
  {
		// maybe you can delay for a while
		// delay ( 100 ms )
		status = ar1000_i2c_read_reg(radio,ADDR_STATUS);
		flag = status & MASK_STC; // check STC flag 
  }

  //Low-side TUNE Ends 
  
  status = ar1000_i2c_read_reg(radio,ADDR_RSSI);
  rssi = (status & MASK_RSSI);
  //Read Hi-Side LO Injection
  // R11-->set D15, set D0/D2,  D3 is the same as default
  Reg_Data[11].i = Reg_Data[11].i|0x8005;
  ar1000_i2c_write_reg(radio,11);
  //TUNE to FreqKHz with current setting
  ar1000_set_freq(radio,FreqKHz); // this function will turn on TUNE 
 
  // TUNE  begins
  // then wait for STC flag
  
  // maybe you need to delay for a while
  // delay ( 100 ms )
  status = ar1000_i2c_read_reg(radio,ADDR_STATUS);
  flag = status & MASK_STC; // check STC flag 
  while( flag == 0)
  {
		// maybe you can delay for a while
		// delay ( 100 ms )
		status = ar1000_i2c_read_reg(radio,ADDR_STATUS);
		flag = status & MASK_STC; // check STC flag 
  }
  //High-side TUNE Ends 
  
  status = ar1000_i2c_read_reg(radio,ADDR_RSSI);
  rssi = rssi- (status & MASK_RSSI);	
  if (rssi < 0) //errata in 0.82
  { 	
	// LO
	// R11--> clear D15, set D0/D2, D3 is the same as default
	Reg_Data[11].i = (Reg_Data[11].i&0x7FFF)|0x0005; 
	ar1000_i2c_write_reg(radio,11);
  }else{ 
	//HI
	//R11-->  set D15, clear D0/D2, D3 is the same as default
	Reg_Data[11].i = (Reg_Data[11].i|0x8000)&0xFFFA; 
	ar1000_i2c_write_reg(radio,11);
  }
  
  
  //fine-tune !!
  //TUNE to FreqKHz with current setting
  ar1000_set_freq(radio,FreqKHz); // this function will turn on TUNE 
 
  // TUNE  begins
  // then wait for STC flag
  
  // maybe you need to delay for a while
  // delay ( 100 ms )
  status = ar1000_i2c_read_reg(radio,ADDR_STATUS);
  flag = status & MASK_STC; // check STC flag 
  while( flag == 0)
  {
		// maybe you can delay for a while
		// delay ( 100 ms )
		status = ar1000_i2c_read_reg(radio,ADDR_STATUS);
		flag = status & MASK_STC; // check STC flag 
  }
  pr_info("setfrequence status:0x%x\n",status);
  //TUNE Ends 
  
  
  AR1000_MUTE_OFF     // <---   we dont mute off now, bcz this function may be called by other functions 
ar1000_i2c_write_reg(radio,1);
  
  return;
}

static void ar1000_i2c_seek(struct ar1000_device *radio,unsigned char updown, unsigned int band, unsigned char space)
{
  unsigned int status;
  unsigned int flag;
  unsigned int FreqKHz;
  
  AR1000_MUTE_ON   // Set Muto ON before SEEK
  ar1000_i2c_write_reg(radio,1);
  
  AR1000_TUNE_OFF  //clear TUNE
  ar1000_i2c_write_reg(radio,2);

  AR1000_SEEK_OFF;	//clear SEEK 
  ar1000_i2c_write_reg(radio,3);

  // Setting before seek
  Reg_Data[17].i = (Reg_Data[17].i & SEEK_MASK)|SEEK_SETTING;
  ar1000_i2c_write_reg(radio,17);
  //
  AR1000_SEEK_ON	 
  Reg_Data[3].BIT.B5=space;  // set SPACE 
  Reg_Data[3].BIT.B7=updown; // Seek up or down
  Reg_Data[3].i = (Reg_Data[3].i & 0xE7FF) | band;  // Set BAND
  ar1000_i2c_write_reg(radio,3); // set  and seek
  
  status = ar1000_i2c_read_reg(radio,ADDR_STATUS);
  flag = status & MASK_STC; // check STC flag 
  while( flag == 0)
  {
		// maybe you can delay for a while
		// delay ( 100 ms )
		status = ar1000_i2c_read_reg(radio,ADDR_STATUS);
		flag = status & MASK_STC; // check STC flag 
  }
  pr_info("seek status value 0x%x\n",status);
  pr_info("seek flag value 0x%x\n",flag);

  // Seek Ends 
  // check SF if seek fail ?
  flag = status & MASK_SF;
  if ( flag )
  {
	//seek fail 
	return;
  }
   // seek success, get READCHAN  and fine-tune now !
  FreqKHz = 690 + ((status & MASK_READCHAN )>> SHIFT_READCHAN );

  // Restore setting after seek
  Reg_Data[17].i = AR1000reg[17];
  ar1000_i2c_write_reg(radio,17);
  //

  //fine-tune with auto hilo rejection
  ar1000_tune_hilo(radio,FreqKHz);// band, space);	
  
  AR1000_MUTE_OFF      
  ar1000_i2c_write_reg(radio,1);
  
  return;
}
/*
static unsigned int ar1000_i2c_scan(struct ar1000_device *radio,unsigned char updown, unsigned int band, unsigned char space)
{	// use the native seek capibility of AR1000 to accomplish  scan function 
  unsigned int status;
  unsigned int flag;
  unsigned int FreqKHz;
  unsigned int found;

  unsigned int start; // starting freuqncy (KHz) for scan
  unsigned int end;	// ending frequency (KHz) for scan
  
  AR1000_MUTE_ON   // Set Muto ON before SCAN
  ar1000_i2c_write_reg(radio,1);
  
  AR1000_TUNE_OFF  //clear TUNE
  ar1000_i2c_write_reg(radio,2);
  
  // Setting before seek
  Reg_Data[17].i = (Reg_Data[17].i & SEEK_MASK)|SEEK_SETTING;
  ar1000_i2c_write_reg(radio,17);
  // 
 
  start = 875; //87.5MHz is just an example 
  end = 1080;  //108.0MHz is just an example 
  
  AR1000_TUNE_OFF  //clear TUNE, just make sure tune if OFF
  Reg_Data[2].i &=0xfe00;
  Reg_Data[2].i|=(start-690); // set 87.5 MHz as starting point for this scan  
  ar1000_i2c_write_reg(radio,2);
 
  flag = 0;
  found = 0;
  
  while( flag == 0 )
  { //scan begin
	AR1000_SEEK_OFF;	//clear SEEK 
	ar1000_i2c_write_reg(radio,3);      
 

	AR1000_SEEK_ON	 
    Reg_Data[3].BIT.B5=space;  // set SPACE 
	Reg_Data[3].BIT.B7=updown; // Seek up or down
	Reg_Data[3].i = (Reg_Data[3].i & 0xE7FF) | band;  // Set BAND
	ar1000_i2c_write_reg(radio,3); // set  and seek
  
	status = ar1000_i2c_read_reg(radio,ADDR_STATUS);
	flag = status & MASK_STC; // check STC flag 
	while( flag == 0)
	{
		// maybe you can delay for a while
		// delay ( 100 ms )
		status = ar1000_i2c_read_reg(radio,ADDR_STATUS);
		flag = status & MASK_STC; // check STC flag 
	}
	// Seek Ends 
	// check SF if seek fail ?   0 means successful seek
	flag = status & MASK_SF;
	if( flag ==0 )
	{
		// seek success, get READCHAN , you may record this FreqKHz into your station list !
		FreqKHz = 690 + ((status & MASK_READCHAN )>> SHIFT_READCHAN );

		// update seek result to CHAN for next seek
		AR1000_TUNE_OFF // just make sure tune bit is off
		Reg_Data[2].i &=0xfe00;
		Reg_Data[2].i|=((status & MASK_READCHAN )>> SHIFT_READCHAN );
		ar1000_i2c_write_reg(radio,2);
		found++;
		if( FreqKHz == end )
		{ // scan to the end !!
			flag = 1; // for quit the while loop
		}

	}
}	

  // Restore setting after seek
  Reg_Data[17].i = AR1000reg[17];
  ar1000_i2c_write_reg(radio,17);
  //
  
  AR1000_MUTE_OFF      
  ar1000_i2c_write_reg(radio,1);
  
  return found;
}
*/

static void ar1000_i2c_write_enable(struct ar1000_device *radio)
{
	unsigned int status;
// Cellon modify start, Ted Shi, 2012/10/09, for add dead loop times
	unsigned int ret;
	
	Reg_Data[0].BIT.B8 = 1;
	ar1000_i2c_write_reg(radio,0);

//	AR100_init(radio);
	ret = AR100_init(radio);
	if(ret < 0){
		printk(" AR1000 enable failed \n");
		return;
	}
// Cellon modify end, Ted Shi, 2012/10/09
	//Power-On Calibration begins
	// then wait for STC flag
	// maybe you need to delay for a while
	// delay ( 100 ms )
	status = ar1000_i2c_read_reg(radio,ADDR_STATUS);
	
	status &=MASK_STC; // check STC flag 
	while( status == 0)
	{
		// maybe you can delay for a while
		// delay ( 100 ms )
	    status = ar1000_i2c_read_reg(radio,ADDR_STATUS);
		status &=MASK_STC; // check STC flag 
	}
	//Power-On Calibration Ends	
	status = ar1000_i2c_read_reg(radio,ADDR_STATUS);
	pr_info("%s: register ADDR_STATUS %x \n",__func__,status);
	//fine-tune with auto hilo rejection
//	ar1000_tune_hilo(FreqKHz);	
	
}

static void ar1000_i2c_write_disable(struct ar1000_device *radio)
{
	Reg_Data[0].BIT.B8 = 0;
	ar1000_i2c_write_reg(radio,0);
}

/* tune an frequency, freq is defined by v4l's TUNER_LOW, i.e. 1/16th kHz */
static void ar1000_tune(struct ar1000_device *radio,int freq)
{
      //add for Rds
      unsigned char flag=0;
	   if(isRdsOn)  
	   	  flag=1;
	   else
	   	flag=0;
         //add for Rds

	AR1000_MUTE_ON   // Set Muto ON before TUNE
	ar1000_i2c_write_reg(radio,1);
	if(flag)
	RDS_Disable(radio);
	ar1000_set_freq(radio,freq);
	if(flag)
	RDS_Enable(radio);
	AR1000_MUTE_OFF   // Set Muto OFF before TUNE
	ar1000_i2c_write_reg(radio,1);
}

static void ar1000_SetVolumeLevel(struct ar1000_device *radio,u8 level)
{
	u16 dataRead;
	dataRead=ar1000_i2c_read_reg(radio,3);
    Reg_Data[3].i=dataRead&0xf87f;
	ar1000_i2c_write_reg(radio,3);
	if(level==0)
	{
        Reg_Data[3].i=dataRead|0x0780;
		ar1000_i2c_write_reg(radio,3);
	}
	dataRead=ar1000_i2c_read_reg(radio,14);
    Reg_Data[14].i=((dataRead&0x0fff)|(AR1000VOL[level]<<12));
	ar1000_i2c_write_reg(radio,14);
}
static void ar1000_set_audout_mode(struct ar1000_device *radio,bool mono)
{
//   u16 dataRead; 
   #if 0
   dataRead=ar1000_i2c_read_reg(radio,3);
    Reg_Data[1].i=(dataRead|0xfff7);
   ar1000_i2c_write_reg(radio,1);
    Reg_Data[1].i=(dataRead|(mono<<3));
   ar1000_i2c_write_reg(radio,1);
#endif
}

static char ar1000_get_audout_mode(struct ar1000_device*radio)
{
   u8 mono;
//   u16 dataRead;
   #if 0
   dataRead=ar1000_i2c_read_reg(radio,1);
   mono=(dataRead&0x0008)>>3;
#endif
   mono = 0;
   return mono;
}

static char tea57674_get_rssi(struct ar1000_device*radio)
{

      unsigned int read_rssi;
	
	read_rssi=ar1000_i2c_read_reg(radio,ADDR_RSSI);
	pr_info("read_rssi=<%d>\n",((read_rssi & MASK_RSSI )>> SHIFT_RSSI ));
	return(((read_rssi & MASK_RSSI )>> SHIFT_RSSI ));

}

static char tea57674_get_rds_ability(struct ar1000_device*radio)
{

      unsigned int chip_id,flag=-1;
	
	chip_id=ar1000_i2c_read_reg(radio,ADDR_CHIPID);
	if(chip_id==CHIPNO_AR1000)
		flag=0;
	return(flag);

}
static void ar1000_mute(struct ar1000_device *radio,int on)
{
	if (on)
	{
	     AR1000_MUTE_ON
	     ar1000_i2c_write_reg(radio,1);
	     //ar1000_i2c_write_disable(radio);
	}
	else
	{
             AR1000_MUTE_OFF
             ar1000_i2c_write_reg(radio,1);
	     //ar1000_i2c_write_enable(radio);
	}
}

/* V4L2 vidioc */
static int vidioc_querycap(struct file *file, void  *priv,
					struct v4l2_capability *v)
{
//	struct ar1000_device *radio = video_drvdata(file);
//	struct video_device *dev = radio->videodev;

//	strlcpy(v->driver, dev->dev.driver->name, sizeof(v->driver));
//	strlcpy(v->card, dev->name, sizeof(v->card));
//	snprintf(v->bus_info, sizeof(v->bus_info), "I2C:%s", dev->dev.bus_id);
	v->version = RADIO_VERSION;
	v->capabilities = V4L2_CAP_TUNER | V4L2_CAP_RADIO;
	return 0;
}

static int vidioc_g_tuner(struct file *file, void *priv,
				struct v4l2_tuner *v)
{
	struct ar1000_device *radio = video_drvdata(file);

	if (v->index > 0)
		return -EINVAL;
	//pr_info("(ted.shi)%s: enter \n",__func__);
	memset(v, 0, sizeof(v));
	strcpy(v->name, "FM");
	v->type = V4L2_TUNER_RADIO;
	v->rangelow   = FREQ_MIN * FREQ_MUL;
	v->rangehigh  = FREQ_MAX * FREQ_MUL;
	v->capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO;
	v->audmode = ar1000_get_audout_mode(radio);


	return 0;
}

static int vidioc_s_tuner(struct file *file, void *priv,
				struct v4l2_tuner *v)
{
	struct ar1000_device *radio = video_drvdata(file);

	if (v->index > 0)
		return -EINVAL;

	ar1000_set_audout_mode(radio, v->audmode);
	return 0;
}

static int vidioc_s_frequency(struct file *file, void *priv,
				struct v4l2_frequency *f)
{
	struct ar1000_device *radio = video_drvdata(file);
	if (f->frequency == 0) {
		/* We special case this as a power down control. */
		//ar1000_i2c_write_disable(radio);
	}
	//ar1000_i2c_write_enable(radio);
	//AR1000_RDSInt_OFF
	//AR1000_MUTE_OFF 
    	pr_info("set frequency is 0x%x\n",f->frequency);
	//ar1000_tune_hilo(radio,f->frequency);
	ar1000_tune(radio,f->frequency);
	//Reg_Data[1].i=0x5B15;
	//ar1000_i2c_write_reg(radio,1);

#if 0
	val =ar1000_i2c_read_reg(Temp_radio,ADDR_CHIPID);	
	pr_info("chip id is 0x%x,chip isnot ar1000\n",val);
	for(i=0;i<18;i++)
	{
		val =ar1000_i2c_read_reg(Temp_radio,i);	
		pr_info("chip reg[%d] == 0x%x,\n",i,val);
	}
#endif
	return 0;
}

static int vidioc_g_frequency(struct file *file, void *priv,
				struct v4l2_frequency *f)
{
	struct ar1000_device *radio = video_drvdata(file);
	unsigned int status;
	unsigned int FreqKHz;
	//ar1000_i2c_write_enable(radio);
	status=ar1000_i2c_read_reg(radio,0x13);
	FreqKHz = 690 + ((status & MASK_READCHAN )>> SHIFT_READCHAN );
	//pr_info("%s: status = %x freqKHZ = %x \n",__func__,status,FreqKHz);
	memset(f, 0, sizeof(f));
	f->type = V4L2_TUNER_RADIO;
	if(FreqKHz>690)
		f->frequency =FreqKHz ;
	else
		f->frequency = 0;

	return 0;
}

static int vidioc_queryctrl(struct file *file, void *priv,
			    struct v4l2_queryctrl *qc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(radio_qctrl); i++) {
		if (qc->id && qc->id == radio_qctrl[i].id) {
			memcpy(qc, &(radio_qctrl[i]), sizeof(*qc));
			return 0;
		}
	}
	return -EINVAL;
}

static int vidioc_g_ctrl(struct file *file, void *priv,
			    struct v4l2_control *ctrl)
{
	struct ar1000_device *radio = video_drvdata(file);

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_MUTE:
		ctrl->value = Reg_Data[1].BIT.B9;
		return 0;
	case V4L2_CID_GAIN:
		ctrl->value = tea57674_get_rssi(radio);;
		return 0;
	case V4L2_CID_AUTOGAIN:
		ctrl->value = tea57674_get_rds_ability(radio);;
		return 0;
	}
	return -EINVAL;
}

static int vidioc_s_ctrl(struct file *file, void *priv,
			    struct v4l2_control *ctrl)
{
	struct ar1000_device *radio = video_drvdata(file);
	//pr_info("%s: ctrl->id = %x ctrl->value= %d \n",__func__,ctrl->id,ctrl->value);
	switch (ctrl->id) {
	case V4L2_CID_AUDIO_MUTE:
		ar1000_mute(radio,ctrl->value);
		return 0;
	case V4L2_CID_AUDIO_VOLUME:
		pr_info("Set Volume level %d \n",ctrl->value);
		ar1000_SetVolumeLevel(radio,ctrl->value);
		return 0;
	case V4L2_CID_PRIVATE_TAVARUA_SRHCMODE:
                srhcmode = ctrl->value; 
                return 0;
	case V4L2_CID_PRIVATE_TAVARUA_STATE:
		switch (ctrl->value) {
		case STATE_ENABLE:
			ar1000_i2c_write_enable(radio);
			return 0;
		case STATE_DISABLE:	
			ar1000_i2c_write_disable(radio);	
			return 0;
		}
	}
	return -EINVAL;
}

static int vidioc_s_hw_freq_seek(struct file *file, void *priv,struct v4l2_hw_freq_seek *a)
{
	struct ar1000_device *radio = video_drvdata(file);
	unsigned char up_ward;
	//pr_info("%s: srhcmode=%d \n",__func__,srhcmode);
	switch (srhcmode) {
	case MODE_SEEK:
		//ar1000_i2c_write_enable(radio);
		up_ward = (unsigned char)a->seek_upward;
		//AR1000_RDSInt_OFF
		ar1000_i2c_seek(radio,up_ward,0,1);
		//AR1000_RDSInt_ON
		AR1000_MUTE_OFF 
		ar1000_i2c_write_reg(radio,1);
		return 0;
	case MODE_SCAN:
		//TODO
		//ar1000_i2c_scan
		return 0;
	}

	return -EINVAL;
}

static int vidioc_g_input(struct file *filp, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int vidioc_s_input(struct file *filp, void *priv, unsigned int i)
{
	if (i != 0)
		return -EINVAL;
	return 0;
}

static int vidioc_g_audio(struct file *file, void *priv,
			   struct v4l2_audio *a)
{
	if (a->index > 1)
		return -EINVAL;

	strcpy(a->name, "Radio");
	a->capability = V4L2_AUDCAP_STEREO;
	return 0;
}

static int vidioc_s_audio(struct file *file, void *priv,
			   struct v4l2_audio *a)
{
	if (a->index != 0)
		return -EINVAL;

	return 0;
}

static int ar1000_open(struct file *file)
{
	/* Currently we support only one device */
	int minor = video_devdata(file)->minor;
	struct ar1000_device *radio = video_drvdata(file);

	if (radio->videodev->minor != minor)
		return -ENODEV;

	mutex_lock(&radio->mutex);
	/* Only exclusive access */
	radio->users++;
	Temp_radio->users++;
	mutex_unlock(&radio->mutex);
	file->private_data = radio;
	ar1000_i2c_write_enable(radio);
// Cellon delete start, Ted Shi, 2012/11/20, for roll back FM wrong check in
// Cellon add start, Ted Shi, 2012/11/16, for add FM LPA mode
//	fm_ar1000_is_running = 1;
// Cellon add end, Ted Shi, 2012/11/16
// Cellon delete end, Ted Shi, 2012/11/20
	return 0;
}

static int ar1000_close(struct file *file)
{
	struct ar1000_device *radio = video_drvdata(file);

	if (!radio)
		return -ENODEV;
	mutex_lock(&radio->mutex);
	radio->users--;
	Temp_radio->users--;
	mutex_unlock(&radio->mutex);
	ar1000_i2c_write_disable(radio);
// Cellon delete start, Ted Shi, 2012/11/20, for roll back FM wrong check in
// Cellon add start, Ted Shi, 2012/11/16, for add FM LPA mode
//	fm_ar1000_is_running = 0;
// Cellon add end, Ted Shi, 2012/11/16
// Cellon delete end, Ted Shi, 2012/11/20
	return 0;
}
static void ar1000_early_suspend(struct early_suspend *h)
{

	//printk("enter ar1000_early_suspend\n");
	
	//enter sleep mode
   if(Temp_radio->users)
   {
  // ar1000_i2c_write_enable(Temp_radio);
	}
	else
   {
	//ar1000_i2c_write_disable(Temp_radio);
   }
// Cellon delete start, Ted Shi, 2012/11/02, for suspend/resume time too long
//	mdelay(10);
// Cellon delete end, Ted Shi, 2012/11/02
}

static void ar1000_late_resume(struct early_suspend *h)
{

	//printk("enter ar1000_late_resume\n");
	
	//exit sleep mode
	if(Temp_radio->users)
   {
  // ar1000_i2c_write_enable(Temp_radio);
	}
	else
   {
	//ar1000_i2c_write_disable(Temp_radio);
   }

// Cellon delete start, Ted Shi, 2012/11/02, for suspend/resume time too long
//	mdelay(10);
// Cellon delete end, Ted Shi, 2012/11/02
	
}


/**************************************************************************
 * File Operations Interface
 **************************************************************************/

/*
 * ar1000_fops_read - read RDS data
 */
static ssize_t ar1000_fops_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct ar1000_device *radio = video_drvdata(file);
	int retval = 0,ret;
	unsigned int block_count = 0;
	//unsigned char buffer[8]="TESTFRFM";
	/* switch on rds reception */
	RDS_Enable(radio);

	/* block if no new data available */
	while (radio->wr_index == radio->rd_index) {
		if (file->f_flags & O_NONBLOCK) {
			retval = -EWOULDBLOCK;
			goto done;
		}
		if (wait_event_interruptible(radio->read_queue,
			radio->wr_index != radio->rd_index) < 0) {
			retval = -EINTR;
			goto done;
		}
	}

	/* calculate block count from byte count */
	count /= 8;

			
	//mutex_lock(&radio->lock);
	while (block_count < count) {
		if (radio->rd_index == radio->wr_index)
			{
			break;
			}

		/* always transfer rds complete blocks */
		if ((ret=copy_to_user(buf, &radio->buffer[radio->rd_index], 8)))
		{
			/* retval = -EFAULT; */
			break;
		}

		/* increment and wrap read pointer */
		radio->rd_index += 8;
		if (radio->rd_index >= radio->buf_size)
			radio->rd_index = 0;

		/* increment counters */
		block_count++;
		buf += 8;
		retval += 8;
	}
	//mutex_unlock(&radio->lock);

done:
	return retval;
	
}
/*
 * ar1000_fops_poll - poll RDS data
 */
static unsigned int ar1000_fops_poll(struct file *file,
		struct poll_table_struct *pts)
{
	struct ar1000_device *radio = video_drvdata(file);
	int retval = 0;
	/* switch on rds reception */
	
		RDS_Enable(radio);

	poll_wait(file, &radio->read_queue, pts);

	if (radio->rd_index != radio->wr_index)
		retval = POLLIN | POLLRDNORM;

	return retval;
}
static long sdr_ioctl(struct file *file, void *fh, bool valid_prio, int cmd, void *arg)
{
	struct ar1000_device *s = video_drvdata(file);
	struct rds_command *a = arg;

	switch (cmd) {
		/* --- open() for /dev/radio --- */
	case RDS_CMD_OPEN:
		a->result = 0;	/* return error if chip doesn't work ??? */
              RDS_Enable(s);
		break;
		/* --- close() for /dev/radio --- */
	case RDS_CMD_CLOSE:
		s->data_available_for_read = 1;
		wake_up_interruptible(&s->read_queue);
		a->result = 0;
              RDS_Disable(s);
		break;
		
	default:
		/* nothing */
		return -ENOIOCTLCMD;
	}
	return 0;
}


static void sdr_work(struct work_struct *work)
{
	struct ar1000_device *s = container_of(work, struct ar1000_device, work.work);

       RDS_Eint_Handler(s);
}
static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
/*
	struct switch_dev *sdev = (struct switch_dev *)
		dev_get_drvdata(dev);

	if (sdev->print_state) {
		int ret = sdev->print_state(sdev, buf);
		if (ret >= 0)
			return ret;
	}
	return sprintf(buf, "%d\n", sdev->state);
*/
	unsigned int status = 0;
	unsigned int value = 0;
	
	value = ar1000_i2c_read_reg(Temp_radio,2);
	pr_info("%s: read register R2 %x \n",__func__,value);
	status = ar1000_i2c_read_reg(Temp_radio,ADDR_STATUS);
	pr_info("%s: read register STATUS %x \n",__func__,status);
	status &=MASK_STC; // check STC flag 
	//return sprintf(buf, "R2 0x%x status 0x%x \n",R2,status);
	return sprintf(buf, " %s \n", status? "enabled":"disabled");
}

static ssize_t enable_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned int state;

	sscanf(buf, "%d", &state);
	pr_info("AR1000 %s \n",state? "enable":"disable");
// Cellon modify start, Ted Shi, 2012/08/23, for usb modem/ap switch
	/*
	pr_info("%s: %s switch state from %d to %d \n",__func__,sdev->name,sdev->state,state);
	if (sdev->state != state) {
		sdev->state = state;
		mutex_lock(&usb_switch_mutex);
	*/
/*
	err = usb_ma_switch_gpio(state);
	if(err){
		printk("%s: switch usb ma gpio failed \n",__func__);
	}
		mutex_unlock(&usb_switch_mutex);
	}*/
// Cellon modify end, Ted Shi, 2012/08/23
	if(state)
		ar1000_i2c_write_enable(Temp_radio);
	else
		ar1000_i2c_write_disable(Temp_radio);
	
	ar1000_SetVolumeLevel(Temp_radio,AR1000VOL[15]);
	
	//pr_info("AR1000 %s successful \n",state? "enable":"disable");
	return  count;
}

static ssize_t freq_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	unsigned int status = 0;
	unsigned int value = 0;
	
	value=ar1000_i2c_read_reg(Temp_radio,2);
	pr_info("%s: read register R2 %x \n",__func__,value);
	status = ar1000_i2c_read_reg(Temp_radio,ADDR_STATUS);
	pr_info("%s: read register STATUS %x \n",__func__,status);
	status = ((status & MASK_READCHAN )>> SHIFT_READCHAN) + 690;
	
	return sprintf(buf, "current frep: %d \n", status);
}
static ssize_t freq_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned int state;

	sscanf(buf, "%d", &state);
	pr_info("(ted.shi) AR1000 : set freq = %d \n",state);
	//state = state-690;
	ar1000_tune(Temp_radio,state);
	//pr_info("(ted.shi) AR1000 : set freq successful  \n");
	return count;
}
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);
static DEVICE_ATTR(freq, S_IRUGO | S_IWUSR, freq_show, freq_store);
/* File system interface */
static const struct v4l2_file_operations ar1000_fops = {
	.owner		= THIS_MODULE,
	.open           = ar1000_open,
	.release        = ar1000_close,
	.read		= ar1000_fops_read,
	.poll		= ar1000_fops_poll,
	.ioctl		= video_ioctl2,
};

static const struct v4l2_ioctl_ops ar1000_ioctl_ops = {
	.vidioc_querycap    = vidioc_querycap,
	.vidioc_g_tuner     = vidioc_g_tuner,
	.vidioc_s_tuner     = vidioc_s_tuner,
	.vidioc_g_audio     = vidioc_g_audio,
	.vidioc_s_audio     = vidioc_s_audio,
	.vidioc_g_input     = vidioc_g_input,
	.vidioc_s_input     = vidioc_s_input,
	.vidioc_g_frequency = vidioc_g_frequency,
	.vidioc_s_frequency = vidioc_s_frequency,
	.vidioc_queryctrl   = vidioc_queryctrl,
	.vidioc_g_ctrl      = vidioc_g_ctrl,
	.vidioc_s_ctrl      = vidioc_s_ctrl,
	.vidioc_s_hw_freq_seek=vidioc_s_hw_freq_seek,
//	.vidioc_default =  sdr_ioctl,
	.vidioc_default =  sdr_ioctl,
};

/* V4L2 interface */
static struct video_device ar1000_radio_template = {
	.name		= "AR1000 FM-Radio",
	.fops           = &ar1000_fops,
	.ioctl_ops 	= &ar1000_ioctl_ops,
	.debug	= 0x01,
	.release	= video_device_release,
};
struct class *ar1000_class;
/* I2C probe: check if the device exists and register with v4l if it is */
static int ar1000_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
//	struct ar1000_device *radio;
	int ret;
       int result = 0;
	unsigned int val=0 ;//,i;
	bool  ar1000_flag=0;
	struct device *dev;
	// Get IC version
	pr_info("%s: enter \r\n", __func__);
	Temp_radio = kmalloc(sizeof(struct ar1000_device), GFP_KERNEL);
	if (!Temp_radio)
		return -ENOMEM;
#if 1
	Temp_radio->buf_size = bufblocks * 3;

	Temp_radio->buffer = kmalloc(Temp_radio->buf_size, GFP_KERNEL);
	pr_info("Temp_radio->buffer=<%s>\n",Temp_radio->buffer);
	if (Temp_radio->buffer == NULL) {
		kfree(Temp_radio);
		return -ENOMEM;
	}
#endif
	mutex_init(&Temp_radio->mutex);

    	Temp_radio->i2c_client=client;
	Temp_radio->videodev = video_device_alloc();
	if (!(Temp_radio->videodev)) {
		ret = -ENOMEM;
		goto errfr;
	}
	memcpy(Temp_radio->videodev, &ar1000_radio_template,
		sizeof(ar1000_radio_template));

	i2c_set_clientdata(client, Temp_radio);
	video_set_drvdata(Temp_radio->videodev, Temp_radio);

	ret = video_register_device(Temp_radio->videodev, VFL_TYPE_RADIO,radio_nr);
	if (ret < 0) {
		printk("Could not register video device!\n");
		goto errrel;
	}

	/* initialize and power off the chip */
	val =ar1000_i2c_read_reg(Temp_radio,ADDR_CHIPID);
	fm_id=val;
// Cellon modify start, Ted Shi, 2012/10/09, for add dead loop times
//	if( val != CHIPNO_AR1010&&val != CHIPNO_AR1000 ) {
	if((val != CHIPNO_AR1010)&&(val != CHIPNO_AR1000)) {
    		pr_info("chip id is 0x%x,chip isnot ar1000\n",val);
		return 0;
	}
// Cellon modify end, Ted Shi, 2012/10/09
	if( val == CHIPNO_AR1000 )
	{
		ar1000_flag=1;
	}
// Cellon modify start, Ted Shi, 2012/10/09, for add dead loop times
//	pr_info("chip id is 0x%x,chip is ar1000\n",val);
	pr_info("chip id is 0x%x \n",val);
	// Get IC version
//	AR100_init(Temp_radio);
// Cellon delete start, Ted Shi, 2012/11/16, for optimize system boot time
/*
	ret = AR100_init(Temp_radio);
	if (ret < 0) {
		printk("AR1000 probe failed , eixt !\n");
		goto errrel;
	}
*/
// Cellon delete end, Ted Shi, 2012/11/16
// Cellon modify end, Ted Shi, 2012/10/09
	
	//if FM ic is not ar1000,will not execute this
  if(ar1000_flag)
  	{
	/* start polling via eventd */
	INIT_DELAYED_WORK(&Temp_radio->work, sdr_work);
	// TODO start the work in rds interupt ....schedule_delayed_work(&s->work, 0);
     

	 gpio_request(AR1000_INT_GPIO, "interrupt");
	ar1000_irq = gpio_to_irq(AR1000_INT_GPIO);
	result = request_irq(ar1000_irq, ar1000_interrupt, 
						IRQF_TRIGGER_FALLING, "ar1000", Temp_radio);
  	}
	
   	//AR1000_RDSInt_OFF
//	ar1000_tune_hilo(radio,918);
//	ar1000_i2c_seek(radio,1,0,1);
// Cellon delete start, Ted Shi, 2012/11/16, for optimize system boot time
/*
	AR1000_MUTE_ON
	ar1000_i2c_write_reg(Temp_radio,1);
*/
// Cellon delete end, Ted Shi, 2012/11/16
#if 0
	AR1000_MUTE_OFF 
	ar1000_i2c_write_reg(Temp_radio,1);

	
	for(i=0;i<18;i++)
	{
		val =ar1000_i2c_read_reg(Temp_radio,i);	
		pr_info("chip reg[%d] == 0x%x,\n",i,val);
	}
#endif
// Cellon delete start, Ted Shi, 2012/11/16, for optimize system boot time
//	ar1000_i2c_write_disable(Temp_radio);
// Cellon delete end, Ted Shi, 2012/11/16

#if 1
       //++ lucha rds
	spin_lock_init(&Temp_radio->lock);
	Temp_radio->block_count = 0;
	Temp_radio->wr_index = 0;
	Temp_radio->rd_index = 0;
	Temp_radio->last_blocknum = 0xff;
	init_waitqueue_head(&Temp_radio->read_queue);
	Temp_radio->data_available_for_read = 0;
      // TODO init rds for radio device
#endif
	

#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	early_suspend.suspend = ar1000_early_suspend;
	early_suspend.resume = ar1000_late_resume;
	register_early_suspend(&early_suspend);
#endif
	PINFO("registered.");
	ar1000_class = class_create(THIS_MODULE, "ar1000");
	if (IS_ERR((void *)ar1000_class))
		return PTR_ERR((void *)ar1000_class);
	dev = device_create(ar1000_class, NULL,
		MKDEV(0, 0), NULL, "ar1000");
	if (IS_ERR((void *)dev))
		return PTR_ERR((void *)dev);

	ret = device_create_file(dev, &dev_attr_enable);
	if (ret < 0)
		goto err_create_file_1;
	ret = device_create_file(dev, &dev_attr_freq);
	if (ret < 0)
		goto err_create_file_2;
	
	return 0;
err_create_file_2:
	device_remove_file(dev, &dev_attr_enable);
err_create_file_1:
	device_destroy(ar1000_class, MKDEV(0, 0));
	printk(KERN_ERR "switch: Failed to register driver %s\n", "ar1000");
errrel:
	video_device_release(Temp_radio->videodev);
errfr:
	kfree(Temp_radio->buffer);
	kfree(Temp_radio);
	return ret;
}
#if 1

//////////////////////////////////////////////////////
//      CODE START
///////////////////////////////////////////////////////

void AR_RDS_ON_OFF(struct ar1000_device *radio,  unsigned char ON_OFF)  
{
	unsigned int  dataRead;
      pr_info("AR_RDS_ON_OFF\r\n");
	if(ON_OFF)
	{
		// RDS GPIO2 setting
		// Set GPIO2 as RDS interrupt
		dataRead=ar1000_i2c_read_reg(radio,13);
		dataRead=(dataRead&0xFFF3)|0x0004;
		Reg_Data[13].i=dataRead;
		ar1000_i2c_write_reg(radio,13);

		// RDS interrupt ON and RDS ON
		// Enable rds_int_en and rds_en
              dataRead=ar1000_i2c_read_reg(radio,1);
		dataRead= (dataRead&0xDFBF)|0x2040;
		Reg_Data[1].i=dataRead;
		ar1000_i2c_write_reg(radio,1);
		
		// RDS statistic on
		 // Enable rds_sta_en
		dataRead=ar1000_i2c_read_reg(radio,15);
		dataRead=(dataRead&0xFFDF)|0x0020;
		Reg_Data[15].i=dataRead;
		ar1000_i2c_write_reg(radio,15);
	}
	else
	{
		// RDS interrupt OFF and RDS OFF
		// Disable rds_int_en and rds_en
		dataRead=ar1000_i2c_read_reg(radio,1);
		dataRead=(dataRead&0xDFBF);
		Reg_Data[1].i=dataRead;
		ar1000_i2c_write_reg(radio,1);
	}
}

void AR_RDS_INT_ON_OFF(struct ar1000_device *radio,unsigned char ON_OFF)
{
	unsigned int  dataRead;
	 pr_info("******AR_RDS_INT_ON_OFF\r\n");
	if(ON_OFF)
	{
		 // Set GPIO2 as RDS interrupt
              dataRead=ar1000_i2c_read_reg(radio,13);
		dataRead=(dataRead&0xFFF3)|0x0004;
		Reg_Data[13].i=dataRead;
		ar1000_i2c_write_reg(radio,13);

	        // Enable rds_int_en
		dataRead=ar1000_i2c_read_reg(radio,1);
		dataRead=(dataRead&0xFFBF)|0x0040;
		Reg_Data[1].i=dataRead;
		ar1000_i2c_write_reg(radio,1);
	}else
	{
		 // Disable rds_int_en
		dataRead=ar1000_i2c_read_reg(radio,1);
		dataRead=(dataRead&0xFFBF);
		Reg_Data[1].i=dataRead;
		ar1000_i2c_write_reg(radio,1);
	}
	
}

void AR_RDS_Block_Counter_ON_OFF(struct ar1000_device *radio,unsigned char ON_OFF)	
{
	unsigned int  dataRead;	
	if(ON_OFF)
	{
		// RDS statistic on
		// Enable rds_sta_en	
		dataRead=ar1000_i2c_read_reg(radio,15);
		dataRead=(dataRead&0xFFDF)|0x0020;
		Reg_Data[15].i=dataRead;
		ar1000_i2c_write_reg(radio,15);
	}else
	{
		// RDS statistic OFF
		// Disable rds_sta_en
		dataRead=ar1000_i2c_read_reg(radio,15);
		dataRead=(dataRead&0xFFDF);
		Reg_Data[15].i=dataRead;
		ar1000_i2c_write_reg(radio,15);
	}
}

void   AR_Get_RDS_DATA(struct ar1000_device *radio)
{
	unsigned char  tmpbuf[8];
	unsigned int  dataRead;
        
	
	dataRead=ar1000_i2c_read_reg(radio,19);
      pr_info("register[13h]:%x\r\n", dataRead);
       
	if((dataRead & 0x0040))  // check if RDS data is Ready for read.
	{
                  pr_info("start read rds data!\r\n");
                 
		tmpbuf[0] = ar1000_i2c_read_reg(radio,21);
		tmpbuf[1] = (ar1000_i2c_read_reg(radio,21))>>8;
				   
		tmpbuf[2] = ar1000_i2c_read_reg(radio,22);
		tmpbuf[3] = (ar1000_i2c_read_reg(radio,22))>>8;
		
		tmpbuf[4] = ar1000_i2c_read_reg(radio,23);
		tmpbuf[5] = (ar1000_i2c_read_reg(radio,23))>>8;
		
		tmpbuf[6] =ar1000_i2c_read_reg(radio,24);
		tmpbuf[7] =(ar1000_i2c_read_reg(radio,24))>>8;
            //  for(i=0;i<8;i++)
	        // pr_info("tmpbuf[%d] == 0x%x,\n",i,tmpbuf[i]);
	        
                    /* copy RDS block to internal buffer */
		memcpy(&radio->buffer[radio->wr_index], tmpbuf, 8);
		pr_info("radio->buffer== <%s>\n",&radio->buffer[radio->wr_index]);	
				
			radio->wr_index += 8;

			/* wrap write pointer */
			if (radio->wr_index >= radio->buf_size)
				radio->wr_index = 0;

			/* check for overflow */
			if (radio->wr_index == radio->rd_index) {
				/* increment and wrap read pointer */
				radio->rd_index += 8;
				if (radio->rd_index >= radio->buf_size)
					radio->rd_index = 0;
			}
		
		if (radio->wr_index != radio->rd_index)
			wake_up_interruptible(&radio->read_queue);

		}
}


void ar1000_set_rds( struct ar1000_device *radio,unsigned char  vl_state)
{
     
       pr_info("ar1000_set_rds:state=%d\r\n", vl_state);
     
       if(vl_state)
       {     	     
	        AR_RDS_ON_OFF(radio,1);		   
       }
	else
	 {
	         AR_RDS_ON_OFF(radio,0);
	}
}
/**********************************************************************
*  RDS Enable
**********************************************************************/
void RDS_Enable(struct ar1000_device *radio)
{

	if(isRdsOn)
		return;
        ar1000_set_rds(radio,1);
        isRdsOn = 1;
	memset(radio->buffer,0,sizeof(radio->buffer));
}
/**********************************************************************
*  RDS Disable
**********************************************************************/
void RDS_Disable(struct ar1000_device *radio)
{
	if(!isRdsOn)
		return;
        ar1000_set_rds(radio,0);
       isRdsOn = 0;
      
}

void RDS_Eint_Handler(struct ar1000_device *radio)
{
	if(!isRdsOn)       
		return;
        AR_Get_RDS_DATA(radio);
        return;	
}
static irqreturn_t ar1000_interrupt(int irq, void *dev_id)
{	
	  
      struct ar1000_device *radio = (struct ar1000_device *) dev_id;
	  
      if(!isRdsOn)
	return IRQ_HANDLED;

	pr_info("ar1000_interrupt enter\n");
	schedule_delayed_work(&radio->work, 0);

	return IRQ_HANDLED;
}
 #endif
bool  fm_ar1000_id(void)
{
        if (fm_id==CHIPNO_AR1000)
        {
                return true;
        }
        else
        {
                return false;
        }
}
EXPORT_SYMBOL(fm_ar1000_id);
/*
static int  ar1000_i2c_remove(struct i2c_client *client)
{
	struct ar1000_device *radio = i2c_get_clientdata(client);

	if (radio) {
		//ar1000_i2c_write_disable(radio);
		video_unregister_device(radio->videodev);
		kfree(radio);
	}
	return 0;
}
*/
/* I2C subsystem interface */
static const struct i2c_device_id ar1000_id[] = {
	{ "ar1000", 0 },
	{ },
};

static struct i2c_driver ar1000_i2c_driver = {
	.id_table = ar1000_id,
	.probe = ar1000_i2c_probe,
	.remove =__exit_p(ar1000_i2c_remove),
	.driver = {
		.name = "ar1000",
	},

};

static int __ar1000_probe(struct platform_device *pdev)
{
	int ret;
	pr_info("%s: enter \r\n", __func__);
	ret = i2c_add_driver(&ar1000_i2c_driver);
	return ret;
}



static struct platform_driver ar1000_driver = {
	.probe = __ar1000_probe,
	.driver = {
		.name = "radio-ar1000",
		.owner = THIS_MODULE,
	},
};


/* init the driver */
static int __init ar1000_init(void)
{
	return platform_driver_register(&ar1000_driver);
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
/*
module_param(use_xtal, int, 1);
MODULE_PARM_DESC(use_xtal, "Chip have a xtal connected in board");
*/
module_param(radio_nr, int, 0);
MODULE_PARM_DESC(radio_nr, "video4linux device number to use");

module_init(ar1000_init);
