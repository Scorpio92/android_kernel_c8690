/* linux/drivers/media/video/ov2675.c
 * Driver for ov2675 (UXGA camera) from Samsung Electronics
 * 1/4" 2.0Mp CMOS Image Sensor SoC with an Embedded Image Processor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-i2c-drv.h>
#include <media/ov2675_platform.h> 
#include <linux/slab.h>
#include <plat/gpio-cfg.h> ///
#include <linux/gpio.h>
#ifdef CONFIG_VIDEO_SAMSUNG_V4L2
#include <linux/videodev2_samsung.h>
#endif

#include <plat/exynos4.h>
#include <linux/gpio.h>
#include <plat/gpio-cfg.h>
///
#include <linux/gpio.h>
#define GPIO_CAM_MCLK    	EXYNOS4212_GPJ1(3)
#define GPIO_CAM_MEGA_EN	EXYNOS4212_GPM0(3)		//EXYNOS4_GPC0(1)	//EXYNOS4_GPX0(0)
#define GPIO_CAM_MEGA_nRST	EXYNOS4212_GPM0(2)
struct v4l2_subdev *sd_g;
struct ov2675_jpeg_param {
	u32 enable;
	u32 quality;
	u32 main_size;		/* Main JPEG file size */
	u32 thumb_size;		/* Thumbnail file size */
	u32 main_offset;
	u32 thumb_offset;
	u32 postview_offset;
};

extern void s3c_csis_start(int csis_id, int lanes, int settle, int align, int width, \
				int height, int pixel_format);
extern  mipi_start;				
#define SENSOR_JPEG_SNAPSHOT_MEMSIZE	0x410580

#include "ov2675newman.h"
extern int mipid_start;//crystal ,2012-11-27
#define ov2675_DRIVER_NAME	"ov2675"
#define CONFIG_BOARD_ODROID_A4

/* Default resolution & pixelformat. plz ref ov2675_platform.h */
#define DEFAULT_RES		WVGA	/* Index of resoultion */
#define DEFAUT_FPS_INDEX	ov2675_15FPS
#define DEFAULT_FMT		V4L2_PIX_FMT_UYVY	/* YUV422 */

static int read_device_id(struct i2c_client *client);

/*
 * Specification
 * Parallel : ITU-R. 656/601 YUV422, RGB565, RGB888 (Up to VGA), RAW10
 * Serial : MIPI CSI2 (single lane) YUV422, RGB565, RGB888 (Up to VGA), RAW10
 * Resolution : 1280 (H) x 1024 (V)
 * Image control : Brightness, Contrast, Saturation, Sharpness, Glamour
 * Effect : Mono, Negative, Sepia, Aqua, Sketch
 * FPS : 15fps @full resolution, 30fps @VGA, 24fps @720p
 * Max. pixel clock frequency : 48MHz(upto)
 * Internal PLL (6MHz to 27MHz input frequency)
 */

/* Camera functional setting values configured by user concept */
struct ov2675_userset {
	signed int exposure_bias;	/* V4L2_CID_EXPOSURE */
	unsigned int ae_lock;
	unsigned int awb_lock;
	unsigned int auto_wb;	/* V4L2_CID_AUTO_WHITE_BALANCE */
	unsigned int manual_wb;	/* V4L2_CID_WHITE_BALANCE_PRESET */
	unsigned int wb_temp;	/* V4L2_CID_WHITE_BALANCE_TEMPERATURE */
	unsigned int effect;	/* Color FX (AKA Color tone) */
	unsigned int contrast;	/* V4L2_CID_CONTRAST */
	unsigned int saturation;	/* V4L2_CID_SATURATION */
	unsigned int sharpness;		/* V4L2_CID_SHARPNESS */
	unsigned int glamour;
};
static const struct ov2675_frmsizeenum preview_frmsizes[] = {
	{ ov2675_PREVIEW_QCIF,	176,	144,	0x05 },	/* 176 x 144 */
	{ ov2675_PREVIEW_QCIF2,	528,	432,	0x2C },	/* 176 x 144 */
	{ ov2675_PREVIEW_QVGA,	320,	240,	0x09 },
	{ ov2675_PREVIEW_VGA,	640,	480,	0x17 },
	{ ov2675_PREVIEW_D1,	720,	480,	0x18 },
	{ ov2675_PREVIEW_WVGA,	800,	480,	0x1A },
	{ ov2675_PREVIEW_720P,	1280,	720,	0x21 },
	{ ov2675_PREVIEW_1080P,	1920,	1080,	0x28 },
	{ ov2675_PREVIEW_HDR,	3264,	2448,	0x27 },
};

static const struct ov2675_frmsizeenum capture_frmsizes[] = {
	{ ov2675_CAPTURE_VGA,	640,	480,	0x09 },
	{ ov2675_CAPTURE_WVGA,	800,	480,	0x0A },
	{ ov2675_CAPTURE_W2MP,	2048,	1232,	0x2C },
	{ ov2675_CAPTURE_3MP,	2048,	1536,	0x1B },
	{ ov2675_CAPTURE_W7MP,	3264,	1968,	0x2D },
	{ ov2675_CAPTURE_8MP,	3264,	2448,	0x25 },
};
int 	g_alive3=0;  
int 	g_ov2675_width=800;///1600;///1600;///640;//1600;///640;
int 	g_ov2675_height=600;///1200;//1200;///480;//1200;///480;
int 	g_ov2675_old_width=0;
int 	g_ov2675_old_height=0;
int 	g_is_initial=0;
EXPORT_SYMBOL(g_ov2675_width);
//bool cam_mipi_en = false;
//EXPORT_SYMBOL(cam_mipi_en);

static struct ov2675_control ov2675_ctrls[] = {
	{
		.id = V4L2_CID_CAMERA_ISO,
		.minimum = ISO_AUTO,
		.maximum = ISO_800,
		.step = 1,
		.value = ISO_AUTO,
		.default_value = ISO_AUTO,
	}, {
		.id = V4L2_CID_CAMERA_BRIGHTNESS,
		.minimum = EV_MINUS_4,
		.maximum = EV_MAX - 1,
		.step = 1,
		.value = EV_DEFAULT,
		.default_value = EV_DEFAULT,
	}, {
		.id = V4L2_CID_CAMERA_SATURATION,
		.minimum = SATURATION_MINUS_2,
		.maximum = SATURATION_MAX - 1,
		.step = 1,
		.value = SATURATION_DEFAULT,
		.default_value = SATURATION_DEFAULT,
	}, {
		.id = V4L2_CID_CAMERA_SHARPNESS,
		.minimum = SHARPNESS_MINUS_2,
		.maximum = SHARPNESS_MAX - 1,
		.step = 1,
		.value = SHARPNESS_DEFAULT,
		.default_value = SHARPNESS_DEFAULT,
	}, {
		.id = V4L2_CID_CAMERA_ZOOM,
		.minimum = ZOOM_LEVEL_0,
		.maximum = ZOOM_LEVEL_MAX - 1,
		.step = 1,
		.value = ZOOM_LEVEL_0,
		.default_value = ZOOM_LEVEL_0,
	}, {
		.id = V4L2_CID_CAM_JPEG_QUALITY,
		.minimum = 1,
		.maximum = 100,
		.step = 1,
		.value = 100,
		.default_value = 100,
	},
};

static inline struct ov2675_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov2675_state, sd);
}

static unsigned short i2c_read_regraw(struct i2c_client *client, unsigned short reg_h, unsigned short reg)
{
	int ret;
	unsigned char i2c_data[10];

	//{0x00, 0x28, 0x70, 0x00},  
	i2c_data[0]= 0x00;
	i2c_data[1]= 0x2c;
	i2c_data[2]= reg_h >>8;		//0x70;
	i2c_data[3]= reg_h & 0xff;	//0x00;

	i2c_master_send(client,i2c_data,4);
	

	i2c_data[0]= 0x00;
	i2c_data[1]= 0x2e;
	i2c_data[2]= (unsigned char)((reg>>8) & 0xff);
	i2c_data[3]= (unsigned char)(reg & 0xff);	
	

	i2c_master_send(client,i2c_data,4);

	i2c_data[0]=0x0f;
	i2c_data[1]=0x12;
	i2c_master_send(client,i2c_data,2);			
	

	ret = i2c_master_recv(client,i2c_data,2);

#if 0
	for(i=0;i<2;i++)
	printk("retdata %d => %x \n",i,i2c_data[i]);
#endif

#if 0
		if (ret < 0)
			printk( "%s: err %d\n", __func__, ret);
#endif
	
		return i2c_data[0]<<8 | i2c_data[1];
}
static unsigned short i2c_read_reg(struct i2c_client *client, unsigned short reg_h, unsigned short reg)
{
	int ret;
	unsigned char i2c_data[10];


	
	if(0)
	{
		i2c_data[0]= 0x00;
		i2c_data[1]= 0x2e;
		i2c_data[2]= (unsigned char)((reg>>8) & 0xff);
		i2c_data[3]= (unsigned char)(reg & 0xff);	
	}

	i2c_data[0]= (unsigned char)((reg>>8) & 0xff);
	i2c_data[1]= (unsigned char)(reg & 0xff);	
	
	i2c_master_send(client,i2c_data,2);
/***
	i2c_data[0]=0x0f;
	i2c_data[1]=0x12;
	i2c_master_send(client,i2c_data,2);		*/	
	

	ret = i2c_master_recv(client,i2c_data,2);

#if 0
	for(i=0;i<2;i++)
	printk("retdata %d => %x \n",i,i2c_data[i]);
#endif

#if 0
		if (ret < 0)
			printk( "%s: err %d\n", __func__, ret);
#endif
	
		return i2c_data[0]<<8 | i2c_data[1];
}
/*
 * ov2675 register structure : 2bytes address, 2bytes value
 * retry on write failure up-to 5 times
 */
static inline int ov2675_write(struct v4l2_subdev *sd, u8 addr, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg[1];
	unsigned char reg[2];
	int err = 0;
	int retry = 0;


	if (!client->adapter)
		return -ENODEV;

again:
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = reg;

	reg[0] = addr & 0xff;
	reg[1] = val & 0xff;

	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return err;	/* Returns here on success */

	/* abnormal case: retry 5 times */
	if (retry < 5) {
		dev_err(&client->dev, "%s: address: 0x%02x%02x, " \
			"value: 0x%02x%02x\n", __func__, \
			reg[0], reg[1], reg[2], reg[3]);
		retry++;
		goto again;
	}

	return err;
}

static int ov2675_i2c_write(struct v4l2_subdev *sd, unsigned char i2c_data[],
				unsigned char length)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char buf[length], i;
	struct i2c_msg msg = {client->addr, 0, length, buf};

	for (i = 0; i < length; i++)
		buf[i] = i2c_data[i];

	return i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

static int ov2675_write_regs(struct v4l2_subdev *sd, unsigned char regs[],
				int size)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int i, err;

	for (i = 0; i < size; i++) {
		err = ov2675_i2c_write(sd, &regs[i], sizeof(regs[i]));
		if (err < 0)
			v4l_info(client, "%s: register set failed\n", \
			__func__);
	}

	return 0;	/* FIXME */
}

static int ov2675_i2c_write_4byte(struct v4l2_subdev *sd, unsigned char d0,unsigned char d1,unsigned char d2,unsigned char d3)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char buf[4], i;
	struct i2c_msg msg = {client->addr, 0, 4, buf};

	buf[0] = d0;
	buf[1] = d1;
	buf[2] = d2;
	buf[3] = d3;

	return i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

static int ov2675_i2c_write_2short(struct v4l2_subdev *sd, unsigned short d0,unsigned short d1)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char buf[4], i;
	struct i2c_msg msg = {client->addr, 0, 4, buf};

	buf[0] = d0>>8;
	buf[1] = d0 & 0xff;
	buf[2] = d1>>8;
	buf[3] = d1 & 0xff;

	return i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}


static int ov2675_write_array(struct v4l2_subdev *sd,unsigned  short * reg , int size)
{	
	unsigned char _tmp[4];
	int i,err=0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	for (i = 0; i < size ; i=i+2) {
		if(reg[i] == 0xffff){ //delay
			mdelay(reg[i+1]);
		}
		else {
///		printk("i:%d   %x %x. \n",i,reg[i],reg[i+1]);
//			printk(".");

			_tmp[0] =(unsigned char)( reg[i] >> 8) ;
			_tmp[1] =(unsigned char)( reg[i] & 0xff);

			///_tmp[2] =(unsigned char)( reg[i+1] >> 8) ;
			_tmp[2] =(unsigned char)( reg[i+1] & 0xff);
			err = ov2675_i2c_write(sd,_tmp , 3);
			if (err < 0){
				v4l_info(client, "%s: register set failed\n", \
					__func__);
				v4l_info(client,"err i=%d %02x %02x %02x %02x \n",\
					i, _tmp[0],_tmp[1],_tmp[2]); 
				return -1;
				
				}
		}
//	if(i%50 == 0) 	printk("\n");
	}
//	printk(" %d\n",i);
	
	return err;
}
static const char *ov2675_querymenu_wb_preset[] = {
	"WB Tungsten", "WB Fluorescent", "WB sunny", "WB cloudy", NULL
};

static const char *ov2675_querymenu_effect_mode[] = {
	"Effect FFSepia", "Effect FGAqua", "Effect Monochrome",
	"Effect Negative", "Effect FGSketch", NULL
};

static const char *ov2675_querymenu_ev_bias_mode[] = {
	"-3EV",	"-2,1/2EV", "-2EV", "-1,1/2EV",
	"-1EV", "-1/2EV", "0", "1/2EV",
	"1EV", "1,1/2EV", "2EV", "2,1/2EV",
	"3EV", NULL
};

static struct v4l2_queryctrl ov2675_controls[] = {
	{
		/*
		 * For now, we just support in preset type
		 * to be close to generic WB system,
		 * we define color temp range for each preset
		 */
		.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "White balance in kelvin",
		.minimum = 0,
		.maximum = 10000,
		.step = 1,
		.default_value = 0,	/* FIXME */
	},
	{
		.id = V4L2_CID_WHITE_BALANCE_PRESET,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "White balance preset",
		.minimum = 0,
		.maximum = ARRAY_SIZE(ov2675_querymenu_wb_preset) - 2,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_AUTO_WHITE_BALANCE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Auto white balance",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_EXPOSURE,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "Exposure bias",
		.minimum = 0,
		.maximum = ARRAY_SIZE(ov2675_querymenu_ev_bias_mode) - 2,
		.step = 1,
		.default_value = \
			(ARRAY_SIZE(ov2675_querymenu_ev_bias_mode) - 2) / 2,
			/* 0 EV */
	},
	{
		.id = V4L2_CID_COLORFX,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "Image Effect",
		.minimum = 0,
		.maximum = ARRAY_SIZE(ov2675_querymenu_effect_mode) - 2,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_CONTRAST,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Contrast",
		.minimum = 0,
		.maximum = 4,
		.step = 1,
		.default_value = 2,
	},
	{
		.id = V4L2_CID_SATURATION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Saturation",
		.minimum = 0,
		.maximum = 4,
		.step = 1,
		.default_value = 2,
	},
	{
		.id = V4L2_CID_SHARPNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Sharpness",
		.minimum = 0,
		.maximum = 4,
		.step = 1,
		.default_value = 2,
	},
};

const char **ov2675_ctrl_get_menu(u32 id)
{
	switch (id) {
	case V4L2_CID_WHITE_BALANCE_PRESET:
		return ov2675_querymenu_wb_preset;

	case V4L2_CID_COLORFX:
		return ov2675_querymenu_effect_mode;

	case V4L2_CID_EXPOSURE:
		return ov2675_querymenu_ev_bias_mode;

	default:
		return v4l2_ctrl_get_menu(id);
	}
}

static inline struct v4l2_queryctrl const *ov2675_find_qctrl(int id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov2675_controls); i++)
		if (ov2675_controls[i].id == id)
			return &ov2675_controls[i];

	return NULL;
}

static int ov2675_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov2675_controls); i++) {
		if (ov2675_controls[i].id == qc->id) {
			memcpy(qc, &ov2675_controls[i], \
				sizeof(struct v4l2_queryctrl));
			return 0;
		}
	}

	return -EINVAL;
}

static int ov2675_querymenu(struct v4l2_subdev *sd, struct v4l2_querymenu *qm)
{
	struct v4l2_queryctrl qctrl;

	qctrl.id = qm->id;
	ov2675_queryctrl(sd, &qctrl);

	return v4l2_ctrl_query_menu(qm, &qctrl, ov2675_ctrl_get_menu(qm->id));
}

/*
 * Clock configuration
 * Configure expected MCLK from host and return EINVAL if not supported clock
 * frequency is expected
 * 	freq : in Hz
 * 	flag : not supported for now
 */
static int ov2675_s_crystal_freq(struct v4l2_subdev *sd, u32 freq, u32 flags)
{
	int err = -EINVAL;

	return 0;///err;  ///crystal
}

static int ov2675_g_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int err = 0;
	printk("## ov2675_g_fmt \n");

	return err;
}

static int ov2675_set_frame_size(struct v4l2_subdev *sd,int width)
{
	int err = 0;
	if(1){
	if(width == 640)
	{		printk("\n =======CRYSTAL %s %d\n",__func__,__LINE__);
	 		err=ov2675_write_array(sd,ov2675_640x480_preset_0,ARRAY_SIZE(ov2675_640x480_preset_0));
	}
	else if(width == 800) //1280x720
	{		printk("\n =======CRYSTAL %s %d\n",__func__,__LINE__);
	 		err=ov2675_write_array(sd,ov2675_800x600_preset_0,ARRAY_SIZE(ov2675_800x600_preset_0));
	}
	else if(width == 1280) //1280x720
	{		printk("\n =======CRYSTAL %s %d\n",__func__,__LINE__);
	 		err=ov2675_write_array(sd,ov2675_1280x960_preset_0,ARRAY_SIZE(ov2675_1280x960_preset_0));
	}
	else if(width == 1600) //1280x720
	{		printk("\n =======CRYSTAL %s %d\n",__func__,__LINE__);
	 		err=ov2675_write_array(sd,ov2675_1600x1200_preset_0,ARRAY_SIZE(ov2675_1600x1200_preset_0));
	}
	 else if(width == 2048)//still capture
 	{		
 	}
	}
	return 0;///err;
}
static int ov2675_reg_init(struct v4l2_subdev *sd, int width,int height);
void cam0_odroid_reset(int power_up);

static int ov2675_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *ffmt)
{
	#define cam_err printk
	#define	cam_trace printk
	#define	cam_warn printk
	int err = 0;
	printk("%s  %d \n",__func__,__LINE__);
	struct ov2675_state *state =
		container_of(sd, struct ov2675_state, sd);
	const struct ov2675_frmsizeenum **frmsize;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	
	u32 width = ffmt->width;
	u32 height = ffmt->height;
	u32 tmp_width;
	u32 old_index;
	int i, num_entries;
	printk("\n\n === %s %d ,wxh: %d x %d\n\n",__func__,__LINE__,width,height);
/*if(0){
state->pix.width = fmt->width;
	state->pix.height = fmt->height;
	///state->pix.pixelformat = fmt->fmt.pix.pixelformat;

	if (fmt->colorspace == V4L2_COLORSPACE_JPEG) {
		state->oprmode = S5K4ECGX_OPRMODE_IMAGE;

		s5k4ecgx_set_framesize(sd, s5k4ecgx_capture_framesize_list,
				ARRAY_SIZE(s5k4ecgx_capture_framesize_list),
				false);

	} else {
		state->oprmode = S5K4ECGX_OPRMODE_VIDEO;
		
	
		s5k4ecgx_set_framesize(sd, s5k4ecgx_preview_framesize_list,
				ARRAY_SIZE(s5k4ecgx_preview_framesize_list),
				true);
	}

	state->jpeg.enable = state->pix.pixelformat == V4L2_PIX_FMT_JPEG
}
 */









/***	cam_trace("E\n");

	if (unlikely(state->isp.bad_fw)) {
		cam_err("\"Unknown\" state, please update F/W");
		return -ENOSYS;
	}*/
	if (ffmt->width < ffmt->height) {
		tmp_width = ffmt->height;
		height = ffmt->width;
		width = tmp_width;
	}

	if (ffmt->colorspace == V4L2_COLORSPACE_JPEG) {
		state->format_mode = V4L2_PIX_FMT_MODE_CAPTURE;
		frmsize = &state->capture;
	} else {
		state->format_mode = V4L2_PIX_FMT_MODE_PREVIEW;
		frmsize = &state->preview;
	}
	//printk("frmsize %d %d \n",state->preview->height,state->preview->width);
if(1){
			g_ov2675_width= width=800;
			g_ov2675_height= height=600;
			///ov2675_write_array(sd,ov2675_init_reg_short0,ov2675_INIT_REGS0);
			///ov2675_set_frame_size(sd,800);
			return 0;
}			
	old_index = *frmsize ? (*frmsize)->index : -1;
	*frmsize = NULL;

/***	if (state->format_mode != V4L2_PIX_FMT_MODE_CAPTURE) {
	} else {
	}*/
	
///	if (*frmsize == NULL) {
///		cam_warn("invalid frame size %dx%d\n", width, height);
///	}

///	if(g_ov2675_old_width != width)
	{
	
		if((width == 800) && ( height == 600)){
			printk("\n\n === %s %d ,wxh: %d x %d\n\n",__func__,__LINE__,width,height);
			g_ov2675_width= width=800;
			g_ov2675_height= height=600;
			///ov2675_write_array(sd,ov2675_init_reg_short0,ov2675_INIT_REGS0);
			ov2675_set_frame_size(sd,800);
			return 0;
		}
		 if((width == 1280) && ( height == 720)){
		 	printk("\n\n === %s %d ,wxh: %d x %d\n\n",__func__,__LINE__,width,height);
			g_ov2675_width= width=1280;
			g_ov2675_height= height=720;
		ov2675_set_frame_size(sd,1280);
		return 0;
		}
		if((width == 1600) && ( height == 1200)){
			printk("\n\n === %s %d ,wxh: %d x %d\n\n",__func__,__LINE__,width,height);
			g_ov2675_width= width=1600;
			g_ov2675_height= height=1200;
			ov2675_set_frame_size(sd,1600);
			return 0;
		}
		if((width == 640) && ( height == 480)){
			printk("\n\n === %s %d ,wxh: %d x %d\n\n",__func__,__LINE__,width,height);
			g_ov2675_width= width=640;
			g_ov2675_height= height=480;
			ov2675_set_frame_size(sd,640);
			return 0;
		}
		g_ov2675_old_width=width;
		g_ov2675_old_height=height;
		
	}   
	
	if(0){
		g_ov2675_width=width=640;
		g_ov2675_height=height=480;
		
	}
	else if(1)
	{	g_ov2675_width=width=800;
		g_ov2675_height=height=600;
		}
	else{
		g_ov2675_width=width=1600;
		g_ov2675_height=height=1200;
	} 
	return 0;
}

static int ov2675_enum_framesizes(struct v4l2_subdev *sd, \
					struct v4l2_frmsizeenum *fsize)
{	
	int err = 0;
	printk("%s  %d \n",__func__,__LINE__);
	fsize->discrete.width=g_ov2675_width;
	fsize->discrete.height=g_ov2675_height;
	
/*	
fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = state->pix.width;
	fsize->discrete.height = state->pix.height;*/
	return err;
}

static int ov2675_enum_frameintervals(struct v4l2_subdev *sd,
					struct v4l2_frmivalenum *fival)
{
	int err = 0;

	return err;
}

static int ov2675_enum_fmt(struct v4l2_subdev *sd, struct v4l2_fmtdesc *fmtdesc)
{
	int err = 0;
	printk("## ov2675_enum_fmt \n");

	return err;
}

static int ov2675_try_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int err = 0;
	return err;
}

static int ov2675_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0;

	dev_info(&client->dev, "%s\n", __func__);

	return err;
}

static int ov2675_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0;

	dev_info(&client->dev, "%s: numerator %d, denominator: %d\n", \
		__func__, param->parm.capture.timeperframe.numerator, \
		param->parm.capture.timeperframe.denominator);

	return err;
}
static int ov2675_set_jpeg_quality(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2675_state *state =
		container_of(sd, struct ov2675_state, sd);

	dev_dbg(&client->dev,
		"%s: jpeg.quality %d\n", __func__, state->jpeg.quality);
	if (state->jpeg.quality < 0)
		state->jpeg.quality = 0;
	if (state->jpeg.quality > 100)
		state->jpeg.quality = 100;

	switch (state->jpeg.quality) {
	case 90 ... 100:
		dev_dbg(&client->dev,
			"%s: setting to high jpeg quality\n", __func__);
		break;
	case 80 ... 89:
		dev_dbg(&client->dev,
			"%s: setting to normal jpeg quality\n", __func__);
		break;
	default:
		dev_dbg(&client->dev,
			"%s: setting to low jpeg quality\n", __func__);
		break;
	}
	return 0;
}
static int ov2675_set_focus_location(struct v4l2_subdev *sd,unsigned int location)
{
	static unsigned short ov2675_focus_location[]={	
		0x0F12, 0x7000,
		0x002A, 0x025A,

		0x0F12, 0x0100, //#REG_TC_AF_FstWinStartX //#2nd : 1200-900(OUT)_640-480(IN) window setting
		0x0F12, 0x00E3, //#REG_TC_AF_FstWinStartY
		0x0F12, 0x0200, //#REG_TC_AF_FstWinSizeX 512
		0x0F12, 0x0238, //#REG_TC_AF_FstWinSizeY
		
		0x0F12, 0x018C, //#REG_TC_AF_ScndWinStartX
		0x0F12, 0x0166, //#REG_TC_AF_ScndWinStartY
		0x0F12, 0x00E6, //#REG_TC_AF_ScndWinSizeX
		0x0F12, 0x0132, //#REG_TC_AF_ScndWinSizeY
		
		0x0F12, 0x0001, //#REG_TC_AF_WinSizesUpdated
	};
	
#define ov2675_FOCUS_LOCATION	\
			(sizeof(ov2675_focus_location) / sizeof(ov2675_focus_location[0]))


	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2675_state *state =
		container_of(sd, struct ov2675_state, sd);
	int err;
	int x1=  location >> 16;
	int y1= location & 0xffff;
	printk("location %d %d %d \n",location,x1,y1);
	if(0){
	ov2675_focus_location[5]=x1;
	ov2675_focus_location[7]=y1;

	ov2675_focus_location[9]=200;
	ov2675_focus_location[11]=200;

	ov2675_focus_location[13]=x1;
	ov2675_focus_location[15]=y1;

//	ov2675_focus_location[17]=100;
//	ov2675_focus_location[19]=100;
	

	err=ov2675_write_array(sd,ov2675_focus_location, \
				ov2675_FOCUS_LOCATION);
	}///
	return 0;
}

static int ov2675_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2675_state *state = to_state(sd);

	int err = 0;
	printk("===%s  %d \n",__func__,__LINE__);
	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:

		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:

		break;

	case V4L2_CID_WHITE_BALANCE_PRESET:

		break;

	case V4L2_CID_COLORFX:

		break;

	case V4L2_CID_CONTRAST:

		break;

	case V4L2_CID_SATURATION:

		break;

	case V4L2_CID_SHARPNESS:

		break;
	case V4L2_CID_CAM_JPEG_MEMSIZE:
		ctrl->value = SENSOR_JPEG_SNAPSHOT_MEMSIZE;
		break;
	case V4L2_CID_CAM_JPEG_MAIN_OFFSET:
		ctrl->value = state->jpeg.main_offset;
		break;			
	case V4L2_CID_CAM_JPEG_POSTVIEW_OFFSET:
			ctrl->value = state->jpeg.postview_offset;
		break;
	case V4L2_CID_CAM_JPEG_MAIN_SIZE:
			ctrl->value =SENSOR_JPEG_SNAPSHOT_MEMSIZE;// state->jpeg.main_size;
		break;
	case V4L2_CID_CAM_JPEG_QUALITY:
			ctrl->value = state->jpeg.quality;
		break;

	default:
		err= -ENOIOCTLCMD;
		dev_err(&client->dev, "%s: no such ctrl\n", __func__);
		break;
	}
	return 0;///err;
}

static int ov2675_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	unsigned char _tmp[4];
	int i,value;
	struct ov2675_state *state =
			container_of(sd, struct ov2675_state, sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = -EINVAL;
	printk("==%s  %d   %d  \n",__func__,__LINE__,ctrl->id);
	switch (ctrl->id) {
	case V4L2_CID_FOCUS_AUTO:	
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
		break;
				
	case V4L2_CID_CAMERA_SET_AUTO_FOCUS:
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
		break;
		
	case V4L2_CID_CAMERA_EXPOSURE:  //add  according to  8MP&2MP HAL
	case V4L2_CID_EXPOSURE:
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
	/***	ov2675_write_regs(sd, \
		ov2675_regs_ev_bias[ctrl->value+4], \
			12);  */
		ov2675_write_array(sd, \
		ov2675_regs_ev_bias[ctrl->value+4], \
			6);
		break;
			
	case V4L2_CID_AUTO_WHITE_BALANCE:
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
		
		dev_dbg(&client->dev, "%s: V4L2_CID_AUTO_WHITE_BALANCE\n", \
			__func__);
		err = ov2675_write_regs(sd, \
		(unsigned char *) ov2675_regs_awb_enable[ctrl->value], \
			sizeof(ov2675_regs_awb_enable[ctrl->value]));
		break;
	case V4L2_CID_CAMERA_WHITE_BALANCE:
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_WHITE_BALANCE\n", \
					__func__);
					if(ctrl->value==1)
					ov2675_write_array(sd,ov2675_regs_wb_preset[ctrl->value-1],\
							2);
							else 
				err=ov2675_write_array(sd,ov2675_regs_wb_preset[ctrl->value-1],\
							ov2675_WB_PRESET); 
		break;
		
	case V4L2_CID_COLORFX:
	case V4L2_CID_CAMERA_EFFECT:
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
		err=ov2675_write_array(sd,ov2675_regs_color_effect[ctrl->value]
			,ov2675_COLOR_FX);
		break;
	case V4L2_CID_CAMERA_CONTRAST:
	case V4L2_CID_CONTRAST:
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
		ov2675_write_array(sd,ov2675_regs_contrast_bias[ctrl->value+2]
			,6);
		
		break;
	case V4L2_CID_CAMERA_SATURATION:
	case V4L2_CID_SATURATION:
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
			ov2675_write_array(sd,ov2675_regs_saturation_bias[ctrl->value]
			,6);
		break;
		

	case V4L2_CID_SHARPNESS:
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
		break;
			
	case V4L2_CID_CAMERA_CAPTURE:
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
//		err=ov2675_write_array(sd,ov2675_capture_jpeg_2048x1536, \
//			ov2675_CAPTURE_JPEG_2048x1536);
		///mdelay(1500);
		break;
	case V4L2_CID_CAM_JPEG_QUALITY:
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
		state->jpeg.quality = ctrl->value;
			err = ov2675_set_jpeg_quality(sd);
		break;
#if 0
	case V4L2_CID_CAM_FOCUS_LOCATION:
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
	break;
		
#endif
	case V4L2_CID_CAMERA_ZOOM:
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
			break;
		
	case V4L2_CID_CAMERA_BRIGHTNESS:
	case V4L2_CID_IS_CAMERA_EXPOSURE:
	ov2675_write_array(sd, \
		///ov2675_regs_ev_bias[ctrl->value+4], 6);
		ov2675_regs_ev_bias[ctrl->value+1], 6); 
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
			break;
			
	case V4L2_CID_CAMERA_SCENE_MODE:
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
		break;
	case V4L2_CID_CAMERA_ANTI_BANDING: ///crystal  add	
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
	
	break;
	default:
		printk("\n =======CRYSTAL %s %d,%d \n",__func__,__LINE__,ctrl->value);
		dev_err(&client->dev, "%s: no such control: id:%d  val:%d \n", __func__,ctrl->id,ctrl->value);
		break;
	}

	if (err < 0)
		goto out;
	else 
		return 0;
out:
	dev_dbg(&client->dev, "%s: vidioc_s_ctrl failed\n", __func__);
	return 0;///err; //cyrstal
}
static int ov2675_connected_check(struct i2c_client *client)
{
	int id;
	// id check
	id=read_device_id(client);
	///if(id != 0x5ca) return -1;
	return 0;
}
static int ov2675_check_IPRM_ErrorInfo(struct i2c_client *client)
{
	///u16 ret=0;
	///ret=i2c_read_reg(client,0x7000,0x20a);
	///printk("%s  :%d \n",__func__,ret);  

}

static int ov2675_reg_init(struct v4l2_subdev *sd, int width,int height)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	printk("%s : %dx%d add csis on\n",__func__,width,height);
	if(mipid_start==0)
	{
		s3c_csis_start(1, 1, 12, 32, 800, 600, 825382478);//set 
		mipid_start=1;
	}
	mdelay(20);
	/// mipi_start=0;//NEWLY ADD ,for can be 2nd s3c_csis_start called in streamon_capture
///	if(g_is_initial == true) return 0;
	printk("=dels3c csis==%s  %d \n",__func__,__LINE__);
	v4l_info(client, "%s: camera initialization start\n", __func__);
	err=ov2675_write_array(sd,ov2675_init_reg_short0,ov2675_INIT_REGS0);
	mdelay(1);
///	ov2675_write_array(sd, ov2675_regs_ev_bias[1+4], 6); //exposure to p1 level 
///	v4l_info(client, "%s: camera initialization done\n", __func__);
	///v4l_info(client, " ==== %x  %x %x  %x %x \n",i2c_read_reg(client,0x308c,0x308c),i2c_read_reg(client,0x308c,0x308d),i2c_read_reg(client,0x308c,0x30b2),i2c_read_reg(client,0x34,0x300e),i2c_read_reg(client,0x64,0x331A));

	return 0;///err;
}

//#ifdef CONFIG_BOARD_ODROID_A4


#ifdef CONFIG_BOARD_ODROID_PC
void cam0_odroid_reset(int power_up)
{
	int err;
	printk(KERN_INFO "cam0 reset\n");

	err = gpio_request(EXYNOS4_GPC1(2), "GPC1");
	if (err)
		printk(KERN_ERR "#### failed to reset(GPC1_2) CAM0 \n");
	
	err = gpio_request(EXYNOS4210_GPJ1(4), "GPJ1");
	if (err)
		printk(KERN_ERR "#### failed to reset(GPJ1_4) CAM0 \n");

	s3c_gpio_setpull(EXYNOS4_GPC1(2), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(EXYNOS4210_GPJ1(4), S3C_GPIO_PULL_NONE);

	gpio_direction_output(EXYNOS4_GPC1(2), 0);
	gpio_direction_output(EXYNOS4_GPC1(2), 1);
	
	gpio_direction_output(EXYNOS4210_GPJ1(4), 1); //stnby

	if(power_up)
	{
		gpio_set_value(EXYNOS4210_GPJ1(4), 1);
		//reset  --> L 
		gpio_set_value(EXYNOS4_GPC1(2), 0);
		mdelay(10);
		//reset  --> H			
		gpio_set_value(EXYNOS4_GPC1(2), 1);
	//	mdelay(50);
	}
	else //power down
	{
		//reset  --> L 
		gpio_set_value(EXYNOS4_GPC1(2), 0);

	}
	gpio_free(EXYNOS4_GPC1(2));
	gpio_free(EXYNOS4210_GPJ1(4));
}
#endif

#ifdef CONFIG_BOARD_ODROID_A
void cam0_odroid_reset(int power_up)
{
	int err;
	printk(KERN_INFO "cam0 reset\n");

	err = gpio_request(EXYNOS4_GPC1(1), "GPC1");
	if (err)
		printk(KERN_ERR "#### failed to reset(GPC1_2) CAM0 \n");
	
	err = gpio_request(EXYNOS4210_GPJ1(4), "GPJ1");
	if (err)
		printk(KERN_ERR "#### failed to reset(GPJ1_4) CAM0 \n");

	s3c_gpio_setpull(EXYNOS4_GPC1(1), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(EXYNOS4210_GPJ1(4), S3C_GPIO_PULL_NONE);


	gpio_direction_output(EXYNOS4210_GPJ1(4), 1); //stnby

	if(power_up)
	{
		gpio_set_value(EXYNOS4210_GPJ1(4), 1);
		//reset  --> L 
		gpio_set_value(EXYNOS4_GPC1(1), 0);
		mdelay(50);
		//reset  --> H			
		gpio_set_value(EXYNOS4_GPC1(1), 1);
		mdelay(50);
	}
	else //power down
	{
		//reset  --> L 
		gpio_set_value(EXYNOS4_GPC1(1), 0);

	}
	gpio_free(EXYNOS4_GPC1(1));
	gpio_free(EXYNOS4210_GPJ1(4));

}

#endif
#ifdef CONFIG_BOARD_ETRI_RTU
void cam0_odroid_reset(int power_up)
{
	int err;
	printk(KERN_INFO "cam0 reset\n");

	err = gpio_request(EXYNOS4_GPC0(2), "GPC0");
	if (err)
		printk(KERN_ERR "#### failed to reset(GPX1_3) CAM0 \n");
	
	err = gpio_request(EXYNOS4210_GPJ1(4), "GPJ1");
	if (err)
		printk(KERN_ERR "#### failed to reset(GPJ1_4) CAM0 \n");

	s3c_gpio_setpull(EXYNOS4_GPC0(2), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(EXYNOS4210_GPJ1(4), S3C_GPIO_PULL_NONE);

	gpio_direction_output(EXYNOS4_GPC0(2), 0);
	gpio_direction_output(EXYNOS4210_GPJ1(4), 1); //stnby

	if(power_up)
	{
		gpio_set_value(EXYNOS4210_GPJ1(4), 1);

		//reset  --> L 
		gpio_set_value(EXYNOS4_GPC0(2), 0);
		mdelay(50);
		//reset  --> H			
		gpio_set_value(EXYNOS4_GPC0(2), 1);
		mdelay(50);
	}
	else //power down
	{
		//reset  --> L 
		gpio_set_value(EXYNOS4_GPC0(2), 0);
		gpio_set_value(EXYNOS4210_GPJ1(4), 0);
	}
	gpio_free(EXYNOS4_GPC0(2));
	gpio_free(EXYNOS4210_GPJ1(4));
}	
#endif
extern	int 	ov2675_power_on(void);
static int ov2675_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = -EINVAL, i;
	unsigned char _tmp[4];;
	printk("==%s  %d \n",__func__,__LINE__);
	//cam0_odroid_reset(1);
	///sd_g=sd;
	//init parameters
	///ov2675_power_on(); ////crystal new  add
/***	struct ov2675_state *state =
		container_of(sd, struct ov2675_state, sd);
	state->jpeg.quality = 100;
	state->jpeg.main_offset = 0;
	state->jpeg.main_size = 0;
	state->jpeg.thumb_offset = 0;
	state->jpeg.thumb_size = 0;
	state->jpeg.postview_offset = 0;*/
#if 1
	if(ov2675_connected_check(client)<0) {
		printk("Not Connected\n");
		v4l_err(client, "%s: camera initialization failed\n", __func__);
		g_alive3 = 0;
		return -1;
	}
#endif	
	printk("==%s  %d \n",__func__,__LINE__);
//	v4l_info(client, "%s: camera initialization start\n", __func__);
	err=ov2675_reg_init(sd,g_ov2675_width,g_ov2675_height);
	g_is_initial=true;
	//cam_mipi_en = true;
	g_alive3 = 3;
	return 0;
}


static int ov2675_s_stream(struct v4l2_subdev *sd, int enable)
{
///	struct ov2675_state *state = to_state(sd);
///	int err;
	printk("==%s  %d \n",__func__,__LINE__);
	return 0;
} 
//crystal added for reduce power comsuption when sleep,2012-10-17.
static unsigned short ov2675_poweron[]={
	0x3634,0x00 ,
};
static unsigned short ov2675_poweroff[]={
	0x3634,0x00 ,
};
unsigned short ov2675pwrdown[]={
	0x30ab, 0x00,
	0x30ad, 0x0a,
	0x30ae, 0x27,
	0x363b, 0x01,
};
unsigned short ov2675pwrup[]={		
	0x3012, 0x80
};	
void 	ov2675_poweron_from_wakeup(void)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd_g);
			///i2c_read_reg(client,0x3634,0x3634)&(~(1<<6)&(0xff))
	ov2675_poweron[1]=( i2c_read_reg(client,0x3634,0x3634)  >> 8 )  & (0x00bf);
	ov2675_write_array(sd_g,ov2675_poweron,2);
	printk("======%x on-crystal\n",i2c_read_reg(client,0x3634,0x3634));
}
void	ov2675_poweron_standby(void)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd_g);
	ov2675_poweroff[1]=( ( i2c_read_reg(client,0x3634,0x3634)  >> 8 ) & (0x00ff)  )  |(1<<6);	
	ov2675_write_array(sd_g,ov2675_poweroff,2); ///
	printk("======%x stanby-crystal\n",i2c_read_reg(client,0x3634,0x3634));
	
}
EXPORT_SYMBOL(ov2675_poweron_from_wakeup);
EXPORT_SYMBOL(ov2675_poweron_standby);

void	ov2675_pwr_up(void)
{
	printk("===%s  %d \n",__func__,__LINE__);
	ov2675_write_array(sd_g,ov2675pwrup,2);	
}
EXPORT_SYMBOL(ov2675_pwr_up);

static int ov2675_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2675_state *state = to_state(sd);
	struct ov2675_mbus_platform_data *pdata = state->pdata;
	int ret;
	int tmp=0;
	printk("===%s  %d \n",__func__,__LINE__);
	if(1){
		if(on)  ///bit[6]=0
		{	///i2c_read_reg(client,0x3634,0x3634)&(~(1<<6)&(0xff))
			printk(" %s  %d \n",__FUNCTION__,__LINE__);
			/**ov2675_poweron[1]=( i2c_read_reg(client,0x3634,0x3634)  >> 8 )  & (0x00bf);
			ov2675_write_array(sd,ov2675_poweron,2);
			printk("======%x on-crystal \n",i2c_read_reg(client,0x3634,0x3634));
		*/
		ov2675_write_array(sd,ov2675pwrup,2);	
		}
		else   ///bit[6]=1
		{		
			
			
			printk(" %s  %d \n",__FUNCTION__,__LINE__);
		/***	ov2675_poweroff[1]=( ( i2c_read_reg(client,0x3634,0x3634)  >> 8 ) & (0x00ff)  )  |(1<<6);	
			ov2675_write_array(sd,ov2675_poweroff,2); ///
			*/
			printk("======%x stanby-crystal\n",i2c_read_reg(client,0x3634,0x3634));
		
	
			ov2675_write_array(sd,ov2675pwrdown,8);
		
		
			if (gpio_request(GPIO_CAM_MEGA_EN, "GX0_0"/*"GPJ0"*/) < 0)
			pr_err("failed gpio_request(GX0_0) for camera control\n");
			gpio_direction_output(GPIO_CAM_MEGA_EN, 1);
			///s3c_gpio_setpull(GPIO_CAM_MEGA_EN, S3C_GPIO_PULL_UP); 
			mdelay(1);
			gpio_free(GPIO_CAM_MEGA_EN);
			
			udelay(100);
			
			if (gpio_request(GPIO_CAM_MCLK,"GPJ1_3") < 0)
			pr_err("failed gpio_request(GPJ1_3) for camera control\n");

			s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_INPUT);
			s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_DOWN);
			gpio_free(GPIO_CAM_MCLK);
		}
	}	
	return 0;
}

static const struct v4l2_subdev_core_ops ov2675_core_ops = {
	.init = ov2675_init,	/* initializing API */
//del	.s_config = ov2675_s_config,	/* Fetch platform data */
///	.s_power =ov2675_s_power,
	.queryctrl = ov2675_queryctrl,
	.querymenu = ov2675_querymenu,
	.g_ctrl = ov2675_g_ctrl,
	.s_ctrl = ov2675_s_ctrl,
};

static const struct v4l2_subdev_video_ops ov2675_video_ops = {
	.s_crystal_freq = ov2675_s_crystal_freq,
//del	.g_fmt = ov2675_g_fmt,
	.s_mbus_fmt = ov2675_s_fmt,
	.enum_framesizes = ov2675_enum_framesizes,
	.enum_frameintervals = ov2675_enum_frameintervals,
//del	.enum_fmt = ov2675_enum_fmt,
//del	.try_fmt = ov2675_try_fmt,
	.g_parm = ov2675_g_parm, ///nothing
	.s_parm = ov2675_s_parm,///nothing
	.s_stream = ov2675_s_stream,
	.enum_framesizes=ov2675_enum_framesizes,
};

static const struct v4l2_subdev_ops ov2675_ops = {
	.core = &ov2675_core_ops,
	.video = &ov2675_video_ops,
};

/*
 * ov2675_probe
 * Fetching platform data is being done with s_config subdev call.
 * In probe routine, we just register subdev device
 */
 extern  int ov2675_power_down(void);
 extern  int ov2675_power_down2(void);
 extern  int ov2675_power_on(void);
static int ov2675_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ov2675_state *state;
	struct v4l2_subdev *sd;
	printk(" %s  %d \n",__FUNCTION__,__LINE__);
	state = kzalloc(sizeof(struct ov2675_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;
	g_alive3=0;
	mipid_start=0;
	sd = &state->sd;
	sd_g=sd;
	strcpy(sd->name, ov2675_DRIVER_NAME);

	/* Registering subdev */
	v4l2_i2c_subdev_init(sd, client, &ov2675_ops);
	
	dev_info(&client->dev, "ov2675 has been probed\n");
	g_is_initial=false;
	//ov2675_power_on();
	//ov2675_power_down();
	ov2675_power_down2();
	return 0; 
}


static int ov2675_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	g_is_initial=false;
	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	g_alive3=0;
	//cam_mipi_en = false;
	printk("===%s  %d \n",__func__,__LINE__);
	return 0;
}

static const struct i2c_device_id ov2675_id[] = {
	{ ov2675_DRIVER_NAME, 0 },
	{ },
};

static int read_device_id(struct i2c_client *client)
{
	int id;
	id= i2c_read_reg(client, 0x300a, 0x300a);//26	
	v4l_info(client,"Chip ID 0x00000040 :0x%x \n", id);
	/***v4l_info(client,"Chip Revision  0x00000042 :0x%x \n",i2c_read_reg(client,0x0, 0x42));
	v4l_info(client,"FW version control revision  0x00000048 :%d \n",i2c_read_reg(client,0x0, 0x48));
	v4l_info(client,"FW compilation date(0xYMDD) 0x0000004e :0x%x \n",i2c_read_reg(client,0x0, 0x4e));*/
	return id;
}

MODULE_DEVICE_TABLE(i2c, ov2675_id);

int ov2675_suspend(struct i2c_client *client, pm_message_t state)
{
	printk(" %s  %d \n",__FUNCTION__,__LINE__);
	ov2675_poweron_standby();
}

int ov2675_resume(struct i2c_client *client)
{
	printk(" %s  %d \n",__FUNCTION__,__LINE__);
	ov2675_poweron_from_wakeup();
}
	
static struct v4l2_i2c_driver_data v4l2_i2c_data = {
	.name = ov2675_DRIVER_NAME,
	.probe = ov2675_probe,
	.remove = ov2675_remove,
	.suspend = ov2675_suspend,
	.resume =  ov2675_resume,
	
	.id_table = ov2675_id,
};

MODULE_DESCRIPTION("Samsung Electronics ov2675 UXGA camera driver");
MODULE_AUTHOR("Jinsung Yang <jsgood.yang@samsung.com>");
MODULE_LICENSE("GPL");

