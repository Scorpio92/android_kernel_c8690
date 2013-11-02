/* linux/drivers/media/video/ov2675.h
 *
 *
 * Driver for ov2675 (UXGA camera) from Samsung Electronics
 * 1/4" 2.0Mp CMOS Image Sensor SoC with an Embedded Image Processor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define ov2675_COMPLETE
#ifndef __ov2675_H__
#define __ov2675_H__
 
struct ov2675_reg {
	unsigned char addr;
	unsigned char val;
};

struct ov2675_regset_type {
	unsigned char *regset;
	int len;
};

/*
 * Macro
 */
#define REGSET_LENGTH(x)	(sizeof(x)/sizeof(ov2675_reg))

/*
 * User defined commands
 */
/* S/W defined features for tune */
#define REG_DELAY	0xFF00	/* in ms */
#define REG_CMD		0xFFFF	/* Followed by command */

/* Following order should not be changed */
enum image_size_ov2675 {
	/* This SoC supports upto UXGA (1600*1200) */
#if 0
	QQVGA,	/* 160*120*/
	QCIF,	/* 176*144 */
	QVGA,	/* 320*240 */
	CIF,	/* 352*288 */
	VGA,	/* 640*480 */
#endif
	SVGA,	/* 800*600 */
#if 0
	HD720P,	/* 1280*720 */
	SXGA,	/* 1280*1024 */
	UXGA,	/* 1600*1200 */
#endif
};
enum ov2675_prev_frmsize {
	ov2675_PREVIEW_QCIF,
	ov2675_PREVIEW_QCIF2,
	ov2675_PREVIEW_QVGA,
	ov2675_PREVIEW_VGA,
	ov2675_PREVIEW_D1,
	ov2675_PREVIEW_WVGA,
	ov2675_PREVIEW_720P,
	ov2675_PREVIEW_1080P,
	ov2675_PREVIEW_HDR,
};

enum ov2675_cap_frmsize {
	ov2675_CAPTURE_VGA,	/* 640 x 480 */
	ov2675_CAPTURE_WVGA,	/* 800 x 480 */
	ov2675_CAPTURE_W1MP,	/* 1600 x 960 */
	ov2675_CAPTURE_2MP,	/* UXGA - 1600 x 1200 */
	ov2675_CAPTURE_W2MP,	/* 2048 x 1232 */
	ov2675_CAPTURE_3MP,	/* QXGA - 2048 x 1536 */
	ov2675_CAPTURE_W4MP,	/* WQXGA - 2560 x 1536 */
	ov2675_CAPTURE_5MP,	/* 2560 x 1920 */
	ov2675_CAPTURE_W6MP,	/* 3072 x 1856 */
	ov2675_CAPTURE_7MP,	/* 3072 x 2304 */
	ov2675_CAPTURE_W7MP,	/* WQXGA - 2560 x 1536 */
	ov2675_CAPTURE_8MP,	/* 3264 x 2448 */
};

struct ov2675_control {
	u32 id;
	s32 value;
	s32 minimum;		/* Note signedness */
	s32 maximum;
	s32 step;
	s32 default_value;
};

struct ov2675_frmsizeenum {
	unsigned int index;
	unsigned int width;
	unsigned int height;
	u8 reg_val;		/* a value for category parameter */
};

struct ov2675_isp {
	wait_queue_head_t wait;
	unsigned int irq;	/* irq issued by ISP */
	unsigned int issued;
	unsigned int int_factor;
	unsigned int bad_fw:1;
};

struct ov2675_jpeg {
	int quality;
	unsigned int main_size;	/* Main JPEG file size */
	unsigned int thumb_size;	/* Thumbnail file size */
	unsigned int main_offset;
	unsigned int thumb_offset;
	unsigned int postview_offset;
};

struct ov2675_focus {
	unsigned int mode;
	unsigned int lock;
	unsigned int status;
	unsigned int touch;
	unsigned int pos_x;
	unsigned int pos_y;
};

struct ov2675_exif {
	char unique_id[7];
	u32 exptime;		/* us */
	u16 flash;
	u16 iso;
	int tv;			/* shutter speed */
	int bv;			/* brightness */
	int ebv;		/* exposure bias */
};

struct ov2675_state {
	struct ov2675_platform_data *pdata;
	struct v4l2_subdev sd;

	struct ov2675_isp isp;

	const struct ov2675_frmsizeenum *preview;
	const struct ov2675_frmsizeenum *capture;

	enum v4l2_pix_format_mode format_mode;
	enum v4l2_sensor_mode sensor_mode;
	enum v4l2_flash_mode flash_mode;
	int vt_mode;
	int beauty_mode;
	int zoom;

	unsigned int fps;
	struct ov2675_focus focus;

	struct ov2675_jpeg jpeg;
	struct ov2675_exif exif;

	int check_dataline;
	char *fw_version;

#ifdef CONFIG_CAM_DEBUG
	u8 dbg_level;
#endif

};
static unsigned short ov2675_640x480_preset_0[]={
    0x3012,0x00 ,  
    0x302a,0x05 ,
    0x302b,0xCB ,
    0x306f,0x54 ,
    0x3362,0x80 ,

    0x3070,0x5d ,
    0x3072,0x5d ,
    0x301c,0x0f ,
    0x301d,0x0f ,

    0x3020,0x01 ,
    0x3021,0x18 ,
    0x3022,0x00 ,
    0x3023,0x0A ,
    0x3024,0x06 ,
    0x3025,0x58 ,
    0x3026,0x04 ,
    0x3027,0xbc ,
    0x3088,0x02 ,
    0x3089,0x80 ,
    0x308A,0x01 ,
    0x308B,0xe0 ,
    0x3316,0x64 ,
    0x3317,0x4B ,
    0x3318,0x00 ,
    0x3319,0x6C ,
    0x331A,0x28 ,
    0x331B,0x1e ,
    0x331C,0x00 ,
    0x331D,0x6C ,
    0x3302,0x11 ,
};
static unsigned short ov2675_800x600_preset_0[]={
 0x3012 ,0x00 ,  
 0x302a ,0x05 ,  
 0x302b ,0xCB ,  
 0x306f ,0x54 ,  
 0x3362 ,0x80 ,  
    
 0x3070 ,0x5d ,  
 0x3072 ,0x5d ,  
 0x301c ,0x0f ,  
 0x301d ,0x0f ,  

 0x3020 ,0x01 ,  
 0x3021 ,0x18 ,  
 0x3022 ,0x00 ,
 0x3023 ,0x0A ,
 0x3024 ,0x06 ,
 0x3025 ,0x58 ,
 0x3026 ,0x04 ,
 0x3027 ,0xbc ,
 0x3088 ,0x03 ,
 0x3089 ,0x20 ,
 0x308A ,0x02 ,
 0x308B ,0x58 ,
 0x3316 ,0x64 ,
 0x3317 ,0x4B ,
 0x3318 ,0x00 ,
 0x3319 ,0x6C ,
 0x331A ,0x32 ,
 0x331B ,0x25 ,
 0x331C ,0x08 ,
 0x331D ,0x6C ,
 0x3302 ,0x11 ,
 };
 static unsigned short ov2675_1280x960_preset_0[]={
                           0x3012 ,0x00,  
                          0x302a ,0x05,
                          0x302b ,0xCB,
                          0x306f ,0x54,
                          0x3362 ,0x80,
                   
                          0x3070 ,0x5d,
                          0x3072 ,0x5d,
                          0x301c ,0x0f,
                          0x301d ,0x0f,
          
                          0x3020 ,0x01,
                          0x3021 ,0x18,
                          0x3022 ,0x00,
                          0x3023 ,0x0A,
                          0x3024 ,0x06,
                          0x3025 ,0x58,
                          0x3026 ,0x04,
                          0x3027 ,0xbc,
                          0x3088 ,0x05,
                          0x3089 ,0x00,
                          0x308A ,0x03,
                          0x308B ,0xc0,
                          0x3316 ,0x64,
                          0x3317 ,0x4B,
                          0x3318 ,0x00,
                          0x3319 ,0x6C,
                          0x331A ,0x50,
                          0x331B ,0x3c,
                          0x331C ,0x00,
                          0x331D ,0x6C,
                          0x3302 ,0x11,
 };
static unsigned short ov2675_1600x1200_preset_0[]={
0x03013, 0xF0 ,	
0x03012, 0x00 ,
0x03014, 0x84 ,
0x0302a, 0x04 ,
0x0302b, 0xd4 ,
0x0306f, 0x54 ,
0x03362, 0x80 ,
0x03070, 0x5d ,
0x03072, 0x5d ,
0x0301c, 0x0f ,
0x0301d, 0x0f ,
0x03020, 0x01 ,
0x03021, 0x18 ,
0x03022, 0x00 ,
0x03023, 0x0A ,
0x03024, 0x06 ,
0x03025, 0x58 ,
0x03026, 0x04 ,
0x03027, 0xbc ,
0x03088, 0x06 ,
0x03089, 0x40 ,
0x0308A, 0x04 ,
0x0308B, 0xB0 ,
0x03316, 0x64 ,
0x03317, 0x4B ,
0x03318, 0x00 ,
0x03319, 0x6C ,
0x0331A, 0x64 ,
0x0331B, 0x4B ,
0x0331C, 0x00 ,
0x0331D, 0x6C ,
0x03302, 0x01 ,
};


/*
 * Following values describe controls of camera
 * in user aspect and must be match with index of ov2675_regset[]
 * These values indicates each controls and should be used
 * to control each control
 */

#define ov2675_REGSET(x)	{	\
	.regset = x,			\
	.len = sizeof(x)/sizeof(ov2675_reg),}
	
	static unsigned short ov2675_init_reg_short0[]={	//day 1116
0x3012 ,0x80, 
0x3086 ,0x0f,
0x308c ,0x80, 
0x308d ,0x0e, 
0x360b ,0x00, 
0x30b0 ,0xff, 
0x30b1 ,0xff, 
0x30b2 ,0x24, 
0x300e ,0x3a, 
0x300f ,0xa6, 
0x3010 ,0x80, 
0x3011 ,0x00, 
0x3082 ,0x01, 
0x30f4 ,0x01, 
0x3091 ,0xc0, 
0x30ac ,0x42, 
0x30d1 ,0x08, 
0x30a8 ,0x54, 
0x3093 ,0x00, 
0x307e ,0xe5, 
0x3079 ,0x00, 
0x30aa ,0x82, 
0x3017 ,0x40, 
0x30f3 ,0x83, 
0x306a ,0x0c, 
0x306d ,0x00, 
0x336a ,0x3c, 
0x3076 ,0x6a, 
0x30d9 ,0x95, 
0x3016 ,0x52, 
0x3601 ,0x30, 
0x304e ,0x88, 
0x30f1 ,0x82, 
0x306f ,0x14, 
0x3012 ,0x10, 
0x3391 ,0x06, 
0x3394 ,0x40, 
0x3395 ,0x40, 
0x3015 ,0x12, 
0x3013 ,0xf7, 
0x3018 ,0x80, 
0x3019 ,0x70, 
0x301a ,0xc4, 
0x3030 ,0x55, 
0x3031 ,0x7d, 
0x3032 ,0x7d, 
0x3033 ,0x55, 
0x30af ,0x00, 
0x3048 ,0x1f, 
0x3049 ,0x4e, 
0x304a ,0x40, 
0x304f ,0x40, 
0x304b ,0x02, 
0x304c ,0x00, 
0x304d ,0x42, 
0x304f ,0x40, 
0x30a3 ,0x91, 
0x3013 ,0xf7, 
0x3014 ,0x8c, 
0x3071 ,0x00, 
0x3070 ,0xb9, 
0x3073 ,0x00, 
0x3072 ,0xb9, 
0x301c ,0x03, 
0x301d ,0x03, 
0x304d ,0x42, 
0x304a ,0x40, 
0x304f ,0x40, 
0x3095 ,0x07, 
0x3096 ,0x16, 
0x3097 ,0x1d, 
0x3020 ,0x01, 
0x3021 ,0x1a, 
0x3022 ,0x00, 
0x3023 ,0x06, 
0x3024 ,0x06, 
0x3025 ,0x58, 
0x3026 ,0x02, 
0x3027 ,0x5e, 
0x3088 ,0x03, 
0x3089 ,0x20, 
0x308a ,0x02, 
0x308b ,0x58, 
0x3316 ,0x64, 
0x3317 ,0x25, 
0x3318 ,0x80, 
0x3319 ,0x08, 
0x331a ,0x64, 
0x331b ,0x4b, 
0x331c ,0x00, 
0x331d ,0x38, 
0x3028 ,0x07, 
0x3029 ,0x93, 
0x302a ,0x02, 
0x302b ,0xe6, 
0x302c ,0x00, 
0x302d ,0x00, 
0x302e ,0x00, 
0x3100 ,0x00, 
0x3320 ,0xf8, 
0x3321 ,0x11, 
0x3322 ,0x92, 
0x3323 ,0x1 , 
0x3324 ,0x97, 
0x3325 ,0x2 , 
0x3326 ,0xff, 
0x3327 ,0x10, 
0x3328 ,0x10, 
0x3329 ,0x11, 
0x332a ,0x5c, 
0x332b ,0x54, 
0x332c ,0xca, 
0x332d ,0xc4, 
0x332e ,0x30, 
0x332f ,0x30, 
0x3330 ,0x48, 
0x3331 ,0x43, 
0x3332 ,0xf0, 
0x3333 ,0xa , 
0x3334 ,0xf0, 
0x3335 ,0xf0, 
0x3336 ,0xf0, 
0x3337 ,0x40, 
0x3338 ,0x40, 
0x3339 ,0x40, 
0x333a ,0x0 , 
0x333b ,0x0 , 
0x3380 ,0x28, 
0x3381 ,0x48, 
0x3382 ,0x14, 
0x3383 ,0x17, 
0x3384 ,0x90, 
0x3385 ,0xa7, 
0x3386 ,0xbf, 
0x3387 ,0xb7, 
0x3388 ,0x08, 
0x3389 ,0x98, 
0x338a ,0x01, 
0x3340 ,0x04, 
0x3341 ,0x08, 
0x3342 ,0x16, 
0x3343 ,0x30, 
0x3344 ,0x4e, 
0x3345 ,0x5f, 
0x3346 ,0x6d, 
0x3347 ,0x78, 
0x3348 ,0x84, 
0x3349 ,0x95, 
0x334a ,0xa5, 
0x334b ,0xb4, 
0x334c ,0xc5, 
0x334d ,0xd2, 
0x334e ,0xe0, 
0x334f ,0x2a, 
0x307c ,0x10, 
0x3090 ,0x03, 
0x3350 ,0x30, 
0x3351 ,0x26, 
0x3352 ,0x80, 
0x3353 ,0x23, 
0x3354 ,0x00, 
0x3355 ,0x85, 
0x3356 ,0x31, 
0x3357 ,0x26, 
0x3358 ,0x88, 
0x3359 ,0x1e, 
0x335a ,0x00, 
0x335b ,0x85, 
0x335c ,0x31, 
0x335d ,0x26, 
0x335e ,0x80, 
0x335f ,0x1b, 
0x3360 ,0x00, 
0x3361 ,0x85, 
0x3363 ,0x70, 
0x3364 ,0x7f, 
0x3365 ,0x00, 
0x3366 ,0x00, 
0x3362 ,0x90, 
0x3301 ,0xff, 
0x338B ,0x10, 
0x338c ,0x10, 
0x338d ,0x40, 
0x3370 ,0xd0, 
0x3371 ,0x00, 
0x3372 ,0x00, 
0x3373 ,0x90, 
0x3374 ,0x10, 
0x3375 ,0x10, 
0x3376 ,0x08, 
0x3377 ,0x00, 
0x3378 ,0x04, 
0x3379 ,0x50, 
0x3069 ,0x86, 
0x3087 ,0x02, 
0x3300 ,0xfc, 
0x3302 ,0x11, 
0x3400 ,0x02, 
0x3606 ,0x20, 
0x3601 ,0x30, 
0x30f3 ,0x83, 
0x304e ,0x88, 
0x30a8 ,0x54, 
0x30aa ,0x82, 
0x30a3 ,0x91, 
0x30a1 ,0x41, 
0x363b ,0x01, 
0x309e ,0x08, 
0x3606 ,0x00, 
0x3630 ,0x35, 
0x3640 ,0x1B, 
0x304e ,0x04, 
0x363b ,0x01, 
0x309e ,0x08, 
0x3606 ,0x00, 
0x3084 ,0x01, 
0x3634 ,0x26,  

                                               
0x3013, 0xf7,              
0x3012, 0x10,              
0x3014, 0x8c,              
0x3640, 0x1B,              
0x3302, 0x11,              
0x3362, 0x90,              
0x3069, 0x86,              
0x306f, 0x14,              
0x3020, 0x01,              
0x3021, 0x1a,              
0x3022, 0x00,              
0x3023, 0x06,              
0x3024, 0x06,              
0x3025, 0x58,              
0x3026, 0x02,              
0x3027, 0x5e,              
0x3088, 0x03,              
0x3089, 0x20,              
0x308a, 0x02,              
0x308b, 0x58,              
0x3316, 0x64,              
0x3317, 0x25,              
0x3318, 0x80,              
0x3319, 0x08,              
0x331a, 0x64,              
0x331b, 0x4b,              
0x331c, 0x00,              
0x331d, 0x38,              
0x3028, 0x07,              
0x3029, 0x93,              
0x302a, 0x02,              
0x302b, 0xe6,              
0x302c, 0x00,              
0x302d, 0x00,              
0x302e, 0x00,  

0x3086 ,0x00,             
};


#define ov2675_INIT_REGS0	\
		(sizeof(ov2675_init_reg_short0) / sizeof(ov2675_init_reg_short0[0]))

#define ov2675_INIT_REGS1	\
		(sizeof(ov2675_init_reg_short1) / sizeof(ov2675_init_reg_short1[0]))

#define ov2675_INIT_REGS2	\
		(sizeof(ov2675_init_reg_short2) / sizeof(ov2675_init_reg_short2[0]))

#define ov2675_INIT_REGS3	\
		(sizeof(ov2675_init_reg_short3) / sizeof(ov2675_init_reg_short3[0]))

#define ov2675_INIT_REGS4	\
		(sizeof(ov2675_init_reg_short4) / sizeof(ov2675_init_reg_short4[0]))

#define ov2675_INIT_REGS4_2	\
		(sizeof(ov2675_init_reg_short4_2) / sizeof(ov2675_init_reg_short4_2[0]))


#define ov2675_INIT_REGS5	\
		(sizeof(ov2675_init_reg_short5) / sizeof(ov2675_init_reg_short5[0]))

#define ov2675_INIT_REGS6	\
		(sizeof(ov2675_init_reg_short6) / sizeof(ov2675_init_reg_short6[0]))

#define ov2675_INIT_REGS7	\
		(sizeof(ov2675_init_reg_short7) / sizeof(ov2675_init_reg_short7[0]))

#define ov2675_INIT_REGS8	\
		(sizeof(ov2675_init_reg_short8) / sizeof(ov2675_init_reg_short8[0]))

#define ov2675_INIT_REGS8_1	\
		(sizeof(ov2675_init_reg_short8_1_2048x1536) / sizeof(ov2675_init_reg_short8_1_2048x1536[0]))

#define ov2675_INIT_REGS8_640x480	\
			(sizeof(ov2675_init_reg_short8_640x480) / sizeof(ov2675_init_reg_short8_640x480[0]))

#define ov2675_INIT_REGS8_1280x720	\
			(sizeof(ov2675_init_reg_short8_1280x720) / sizeof(ov2675_init_reg_short8_1280x720[0]))

#define ov2675_INIT_REGS8_1000x562	\
			(sizeof(ov2675_preview_preset_1000x562) / sizeof(ov2675_preview_preset_1000x562[0]))

#define ov2675_INIT_REGS9	\
			(sizeof(ov2675_init_reg_short9) / sizeof(ov2675_init_reg_short9[0]))

	
	

#define ov2675_CAPTURE_PRESET_0	\
		(sizeof(ov2675_capture_preset_0) / sizeof(ov2675_capture_preset_0[0]))

#define ov2675_PREVIEW_PRESET_0	\
		(sizeof(ov2675_preview_preset_0) / sizeof(ov2675_preview_preset_0[0]))
		
#define ov2675_PREVIEW_PRESET_800x480	\
				(sizeof(ov2675_preview_preset_800x480) / sizeof(ov2675_preview_preset_800x480[0]))

#define ov2675_MOVIE_PRESET_0	\
			(sizeof(ov2675_movie_preset_0) / sizeof(ov2675_movie_preset_0[0]))

#define ov2675_CAPTURE_JPEG_2048x1536	\
		(sizeof(ov2675_capture_jpeg_2048x1536) / sizeof(ov2675_capture_jpeg_2048x1536[0]))

#define ov2675_PREVIEW_ON	\
		(sizeof(ov2675_preview_on) / sizeof(ov2675_preview_on[0]))

#define ov2675_PREVIEW_OFF	\
		(sizeof(ov2675_preview_off) / sizeof(ov2675_preview_off[0]))


/*
 * EV bias
 */

static const struct ov2675_reg ov2675_ev_m6[] = {
};

static const struct ov2675_reg ov2675_ev_m5[] = {
};

static  unsigned short ov2675_ev_m4[] = {
	         	0x3391 ,0x06  , 
                   	0x3390, 0x49  ,
                   	0x339A, 0x40  ,
};

static  unsigned short ov2675_ev_m3[] = {
	           	0x3391, 0x06  ,
           		0x3390, 0x49  ,
           		0x339A, 0x30  ,
           	/*	0x3391 ,0x06  , 
                   	0x3390, 0x49  ,
                   	0x339A, 0x40  ,*/
};

static  unsigned short  ov2675_ev_m2[] = {
	               0x3391, 0x06  ,
                    	0x3390, 0x49  ,
                     	0x339A, 0x20  ,
                   /*   	0x3391, 0x06  ,
           		0x3390, 0x49  ,
           		0x339A, 0x30  ,*/
};

static  unsigned short  ov2675_ev_m1[] = { ///
	               0x3391, 0x06  ,
                     	0x3390, 0x49  ,
                     	0x339A, 0x10  ,
                    /*  	0x3391, 0x06  ,
                    	0x3390, 0x49  ,
                     	0x339A, 0x20  ,*/
};

static  unsigned short  ov2675_ev_default[] = {
	               0x3391, 0x06  ,
                     	0x3390, 0x41  ,
                     	0x339A, 0x00  ,
                    /*  	0x3391, 0x06  ,
                     	0x3390, 0x49  ,
                     	0x339A, 0x10  ,*/
};

static  unsigned short  ov2675_ev_p1[] = {
	                0x3391, 0x06  ,
                     	0x3390, 0x41  ,
                     	0x339A, 0x10  ,
};

static  unsigned short  ov2675_ev_p2[] = {
	                0x3391, 0x06  ,
                     	0x3390, 0x41  ,
                     	0x339A, 0x20  ,
};

static  unsigned short  ov2675_ev_p3[] = {
	                0x3391, 0x06  ,
                     	0x3390, 0x41  ,
                     	0x339A, 0x30  ,
};

static  unsigned short  ov2675_ev_p4[] = {
	                0x3391, 0x06  ,
                     	0x3390, 0x41  ,
                     	0x339A, 0x40  ,
};

static const struct ov2675_reg ov2675_ev_p5[] = {
};

static const struct ov2675_reg ov2675_ev_p6[] = {
};

#ifdef ov2675_COMPLETE
/* Order of this array should be following the querymenu data */
static const  unsigned short *ov2675_regs_ev_bias[] = {

	(unsigned short *)ov2675_ev_m4, (unsigned short *)ov2675_ev_m3,
	(unsigned short *)ov2675_ev_m2, (unsigned short *)ov2675_ev_m1,
	(unsigned short *)ov2675_ev_default, (unsigned short *)ov2675_ev_p1,
	(unsigned short *)ov2675_ev_p2, (unsigned short *)ov2675_ev_p3,
	(unsigned short *)ov2675_ev_p4,
};

/* auto focus */

unsigned short ov2675_focus_auto[] = {
		0x0028, 0x7000,
		0x002A, 0x0254, //REG_TC_AF_AfCmdParam
		0x0F12, 0x0000, //write lens position from 0000 to 00FF. 
				  //0000 means infinity and 00FF means macro
	//	P133	  //Delay 133ms
		0xffff, 133,
		0x002A, 0x0252, //REG_TC_AF_AfCmd
		0x0F12, 0x0004, //0004 - Manual AF
		
		//P200	  //Delay 200ms
		0xffff, 200,
		0x002A, 0x0252, //REG_TC_AF_AfCmd
		0x0F12, 0x0006, //0005 - Single AF

};

#define ov2675_FOCUS	(sizeof(ov2675_focus_auto) / sizeof(ov2675_focus_auto[0]))


/*
 * Auto White Balance configure
 */
static const struct ov2675_reg ov2675_awb_off[] = {
};

static const struct ov2675_reg ov2675_awb_on[] = {
};

static const unsigned char *ov2675_regs_awb_enable[] = {
	(unsigned char *)ov2675_awb_off,
	(unsigned char *)ov2675_awb_on,
};

/*
temp = Read_i2c(0x3306);


@@auto
60 3306 temp_reg&~0x2

@@sunny
60 3306 temp_reg|0x2
60 3337 60 
60 3338 40 
60 3339 44 

@@clody
60 3306 temp_reg|0x2
60 3337 5e 
60 3338 40 
60 3339 46 

@@tun
60 3306 temp_reg|0x2
60 3337 42 
60 3338 40 
60 3339 63 

@@fluo
60 3306 temp_reg|0x2
60 3337 53 
60 3338 40 
60 3339 58 

@@daylight
60 3306 temp_reg|0x2
60 3337 60 
60 3338 40 
60 3339 44  */
/*
 * Manual White Balance (presets)
 */

static unsigned short ov2675_wb_tungsten[] = {
	   /*    0x3306, 0x02,
               0x3337, 0x42,
               0x3338, 0x40,
               0x3339 , 0x43,///0x70,*/
               
             0x3306 ,0x02,
            //   0x3337 ,0x65,
            //   0x3338 ,0x40,
            //   0x3339 ,0x41,
                    0x3337 ,0x53,
               0x3338 ,0x40,
               0x3339 ,0x68,  //rechanged by cystal 12-8£¬only changed this
		
};

///{{0x3306, 0x02}, {0x3337, 0x65}, {0x3338, 0x40},	{0x3339,0x41},}, //FLOURESECT NOT SUPPORTED 
static unsigned short ov2675_wb_fluorescent[] = {
	      0x3306 ,0x02,
            //   0x3337 ,0x65,
            //   0x3338 ,0x40,
            //   0x3339 ,0x41,
                    0x3337 ,0x53,
               0x3338 ,0x40,
               0x3339 ,0x58,
               


};

static unsigned short ov2675_wb_sunny[] = {
	       0x3306, 0x02,
        ///       0x3337, 0x5E,
         ///      0x3338, 0x40,
         //      0x3339 ,0x46,
               
                    0x3337, 0x60,
               0x3338, 0x40,
               0x3339 , 0x44,

};
	
//{{0x3306, 0x02}, {0x3337, 0x68}, {0x3338, 0x40},	{0x3339,0x4e},}, //CLOUDY
static unsigned short ov2675_wb_cloudy[] = {
		0x3306 ,0x02,
             // 	0x3337, 0x68,
            ///  	0x3338 ,0x40,
            ///   	0x3339 ,0x4E,
               		0x3337, 0x5e,
              	0x3338 ,0x40,
               	0x3339 ,0x46,


};
static unsigned short ov2675_wb_auto[] = {
		0x3306, 0x00
};

/* Order of this array should be following the querymenu data */
static const unsigned short *ov2675_regs_wb_preset[] = {
	(unsigned short *)ov2675_wb_auto,
	(unsigned short *)ov2675_wb_sunny,
	(unsigned short *)ov2675_wb_cloudy,
	(unsigned short *)ov2675_wb_tungsten,
	(unsigned short *)ov2675_wb_fluorescent,
};
#define ov2675_WB_PRESET	8///(sizeof(ov2675_wb_auto) / sizeof(ov2675_wb_auto[0]))

/*
 * Color Effect (COLORFX)
 */
 

static unsigned short ov2675_color_none[] = {
		0x3391 ,0x06,
              	0x3390 ,0x41,
   		0x3390 ,0x41,///
};

static unsigned short ov2675_color_sepia[] = {  ///FUGU LU
		0x3391 ,0x1E,
               	0x3396 ,0x60,
             	///0x3397 ,0x60,
             	0x3397 ,0xa6,


};

static unsigned short ov2675_color_aqua[] = {///FUFULAN
		0x3391  ,0x1E,
              	0x3396  ,0xA0,
             	0x3397  ,0x40,

};

static unsigned short ov2675_color_monochrome[] = {
		0x3391 ,0x26,
		0x3391 ,0x26,
		0x3391 ,0x26,///
};

static unsigned short ov2675_color_negative[] = {
		0x3391 ,0x46,
		0x3391 ,0x46,
		0x3391 ,0x46,///
};

static unsigned short ov2675_color_sketch[] = {//FUGU
		0x3391 ,0x1E,
             	0x3396 ,0x40,
               	0x3397, 0xA6,

};

/* Order of this array should be following the querymenu data */
/** 13mp-hal will use this 
static const unsigned short *ov2675_regs_color_effect[] = {
	(unsigned short *)ov2675_color_none,
	(unsigned short *)ov2675_color_sepia,
	(unsigned short *)ov2675_color_aqua,
	(unsigned short *)ov2675_color_monochrome,
	(unsigned short *)ov2675_color_negative,
	(unsigned short *)ov2675_color_sketch,
};*/
static const unsigned short *ov2675_regs_color_effect[] = {
	(unsigned short *)ov2675_color_none,
	(unsigned short *)ov2675_color_none,//1
	(unsigned short *)ov2675_color_monochrome,//2
	(unsigned short *)ov2675_color_sepia,//3	
	(unsigned short *)ov2675_color_aqua,
	(unsigned short *)ov2675_color_negative,
	(unsigned short *)ov2675_color_negative, //6
	(unsigned short *)ov2675_color_sketch,
};
#define ov2675_COLOR_FX	6////(sizeof(ov2675_color_none) / sizeof(ov2675_color_none[0]))


/*
 * Contrast bias
 */
static unsigned short ov2675_contrast_m2[] = {
	0x3391,0x06,
	0x3398,0x18,
	0x3399,0x18,
};

static unsigned short ov2675_contrast_m1[] = {
	0x3391,0x06,
	0x3398,0x1c,
	0x3399,0x1c,
};

static unsigned short ov2675_contrast_default[] = {
	0x3391,0x06,
	0x3398,0x20,
	0x3399,0x20,
/*	0x3391,0x06,
0x3394,0x40,
0x3395,0x40,*/
	
};

static unsigned short  ov2675_contrast_p1[] = {
	0x3398,0x24,
	0x3399,0x24,
	0x3399,0x24,
};

static unsigned short ov2675_contrast_p2[] = { ///-2
	0x3391,0x06,
	0x3398,0x28,
	0x3399,0x28,
};

static const  unsigned short * ov2675_regs_contrast_bias[] = {
	ov2675_contrast_m2,
	ov2675_contrast_m1,
	ov2675_contrast_default,
	ov2675_contrast_p1,
	ov2675_contrast_p2,
};

/*
 * Saturation bias
 */
static unsigned short  ov2675_saturation_m2[] = {  ///-2
			0x3391, 0x06,       
      			0x3394, 0x30,
      			0x3395, 0x30,
};

static unsigned short  ov2675_saturation_m1[] = {
	         	0x3391, 0x06,
             		0x3394, 0x38,
             		0x3395, 0x38,
};

static unsigned short  ov2675_saturation_default[] = {
	      		0x3391, 0x06,
      			0x3394, 0x40,
      			0x3395, 0x40,
};

static unsigned short  ov2675_saturation_p1[] = {
	         	0x3391, 0x06,
         		0x3394, 0x48,
         		0x3395, 0x48,
};

static unsigned short ov2675_saturation_p2[] = {
	     		0x3391, 0x06,
     			0x3394, 0x50,
     			0x3395, 0x50,
};

static const unsigned short  *ov2675_regs_saturation_bias[] = {
	ov2675_saturation_m2,
	ov2675_saturation_m1,
	ov2675_saturation_default,
	ov2675_saturation_p1,
	ov2675_saturation_p2,
};

/*
 * Sharpness bias
 */
static const struct ov2675_reg ov2675_sharpness_m2[] = {
};

static const struct ov2675_reg ov2675_sharpness_m1[] = {
};

static const struct ov2675_reg ov2675_sharpness_default[] = {
};

static const struct ov2675_reg ov2675_sharpness_p1[] = {
};

static const struct ov2675_reg ov2675_sharpness_p2[] = {
};
#endif /* ov2675_COMPLETE */

#endif
