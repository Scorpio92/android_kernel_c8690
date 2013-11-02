/* es305 voice processor driver
 *
 * Copyright (C) 
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
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/kernel.h>
#include <linux/serial_core.h>
#include <linux/gpio.h>
#include <plat/iic.h>
#include <plat/gpio-cfg.h>
#include <plat/cpu.h>

#include "es305.h"
#include "es305_iic_firmware.c"

#undef LOG_TAG
#define LOG_TAG "AUD"

#undef AUDIO_DEBUG
#define AUDIO_DEBUG 1


#ifdef CONFIG_CELLON_ENG_LOG
#define DDEBUG
#define DEBUG
#endif

#ifdef DDEBUG
#define dprintk(x...) 	printk(x)
#else
#define dprintk(x...)
#endif
//static struct mutex es305_lock;
//static int es305_opened;
static int es305_suspended = 0;
static int es305_current_config = ES305_PATH_SUSPEND;
//static int es305_param_ID;
static char *config_data;
static int es305_cmds_len;
static int es305_mic_mode = 1;

static bool ES305_SYNC_DONE = false;

/* support at most 1024 set of commands */
#define PARAM_MAX		sizeof(char) * 6 * 1024

static struct i2c_client *this_client;
static struct es305_platform_data *pdata;

static void es305_gpio_set_value(int gpio, int value);

static int es305_i2c_write(char *txData, int length);

static int es305_i2c_read(char *rxData, int length);
int execute_cmdmsg(unsigned int msg);
static int es305_device_init(void);
void es305_set_call_mode(enum es305_call_mode mode);

int tc4_get_call_flg(void);

struct vp_ctxt {
	unsigned char *data;
	unsigned int img_size;
};

struct vp_ctxt the_vp;

static void es305_i2c_sw_reset(unsigned int reset_cmd)
{
	int rc = 0;
	unsigned char msgbuf[4];

	msgbuf[0] = (reset_cmd >> 24) & 0xFF;
	msgbuf[1] = (reset_cmd >> 16) & 0xFF;
	msgbuf[2] = (reset_cmd >> 8) & 0xFF;
	msgbuf[3] = reset_cmd & 0xFF;

	printk("%s: %08x\n", __func__, reset_cmd);

	rc = es305_i2c_write(msgbuf, 4);
	if (!rc)
		msleep(30);
}


static void es305_hardreset(void)
{

	printk("es305_hardreset");
	/* Reset ES305 chip */
	es305_gpio_set_value(pdata->gpio_es305_reset, 0);

	mdelay(1);

	/* Take out of reset */
	es305_gpio_set_value(pdata->gpio_es305_reset, 1);

	mdelay(50);
}

enum sin_define{
	SIN_DEFINE_GPIO,
	SIN_DEFINE_UART,
};

//mode 0 gpio, 1 uart;
static void es305_wakeuppin_set_mode(enum sin_define mode)
{
	if(mode == SIN_DEFINE_GPIO){
		s3c_gpio_cfgpin(pdata->gpio_es305_wakeup,S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(pdata->gpio_es305_wakeup, S3C_GPIO_PULL_NONE);
	}
	else if(mode == SIN_DEFINE_UART){
		s3c_gpio_cfgpin(pdata->gpio_es305_wakeup,S3C_GPIO_SFN(2));//uart
		s3c_gpio_setpull(pdata->gpio_es305_wakeup, S3C_GPIO_PULL_NONE);
	}
	else{
		printk("undefine mode\n");
		panic("undefine mode\n");
	}
}

static void es305_wakeuppin_set_level(int level)
{
	es305_gpio_set_value(pdata->gpio_es305_wakeup, level);
}

static int es305_wakeup(void) 
{
	int rc = 0, retry = 3;
	dprintk("es305_wakeup\n");

	if (es305_suspended == 1) {
		/* Enable es305 clock */
		//es305_gpio_set_value(pdata->gpio_es305_clk, 1);
		pdata->es305_mclk_enable(true); 
		mdelay(2);
		es305_wakeuppin_set_mode(SIN_DEFINE_GPIO);

		es305_wakeuppin_set_level(1);
		mdelay(5);
		es305_wakeuppin_set_level(0);
		//msleep(120);	
		msleep(30);	
		
		do {
			rc = execute_cmdmsg(A200_msg_Sync_Interrupt_L);	
			dprintk("es305 sync ");
		} while ((rc < 0) && --retry);

		if (rc < 0) {
			printk("%s: failed (%d)\n", __func__, rc);
			goto wakeup_sync_err;
		}
		s3c_gpio_cfgall_range(pdata->gpio_es305_wakeup, 1,S3C_GPIO_SFN(2), S3C_GPIO_PULL_UP);
		es305_suspended = 0;
	}
	else{
		dprintk("es305 has been waken up , no need to wake it up!\n");
	}
wakeup_sync_err:
	return rc;
}

int es305_sleep(void ) 
{
	int rc = 0;
	int64_t t1, t2;
	if (es305_suspended == 0) {		
		dprintk("trying es305_sleep\n");
		t1 = ktime_to_ms(ktime_get());

		es305_wakeuppin_set_mode(SIN_DEFINE_GPIO);//gpio
		es305_wakeuppin_set_level(0);//

		/* Put ES305 into sleep mode */
		rc = execute_cmdmsg(A200_msg_SetPowerState_Sleep);
		if (rc < 0) {
			printk("%s: suspend error\n", __func__);
			return -1;
		}

		es305_suspended = 1;
		es305_current_config = ES305_PATH_SUSPEND;

		msleep(20);
	//	mdelay(120);
		/* Disable ES305 clock */
		pdata->es305_mclk_enable(false); 
		//pr_device_power_off();

		t2 = ktime_to_ms(ktime_get()) - t1;
		dprintk("power off es305 %lldms --\n", t2);
	}
	else{
		dprintk("es305 has been slept , no need to sleep it!\n");
	}
	return 0;
 }

int es305_request_wakeup_pin(void)
{
	int rc = 0;
	dprintk("%s\n", __func__);
	gpio_free(pdata->gpio_es305_wakeup);

	rc = gpio_request(pdata->gpio_es305_wakeup, "es305_wakeup_pin");
	if (rc < 0) {
		printk("%s: request wakeup gpio failed\n", __func__);
		return -1;
	}

	rc = gpio_direction_output(pdata->gpio_es305_wakeup, 1);
	if (rc < 0) {
		printk("%s: request wakeup gpio direction failed\n", __func__);
		goto err_free_gpio;
	}

	return 0;

err_free_gpio:
	gpio_free(pdata->gpio_es305_wakeup);
	return -1;

}

int es305_port_config(void)
{
	int i;
	
	dprintk("%s ES305 Configuration(Port A, C configuration and Path Routing )\n", __func__);

	for (i = 0; i < 28; i++) {
		es305_i2c_write(ES305_CMD_ROUTE[i], 4);
		mdelay(20);

	}
	return 0;
}

 int build_cmds(char *cmds, int newid)
{
	int i = 0;
	int offset = 0;

	for (i = 0; i < es305_cmds_len; i += 5) {

		 if (config_data[i] == newid) {
			cmds[offset++] = config_data[i + 1];
			cmds[offset++] = config_data[i + 2];
			cmds[offset++] = config_data[i + 3];
			cmds[offset++] = config_data[i + 4];
		}
	}

	return offset;
}

 static unsigned int DEBUG_STRESS_TEST_COUNT = 1;


/* don't use this function to set suspend mode, es305_sleep instead */
int es305_set_config(int newid, int mode)
{
	int block_size = 128;
	int rc = 0, size = 0;
	unsigned int sw_reset = 0;
	unsigned char *i2c_cmds = NULL;
	unsigned char ack_buf[ES305_CMD_FIFO_DEPTH * 4];
	unsigned char custom_cmds[800] = {0};
	int pass = size / block_size;
	int remainder = size % block_size;
	unsigned char* ptr = i2c_cmds;

	printk("%s\n", __func__);
		/* if es305 is suspended */
	if (es305_suspended) {
		dprintk("ES305 suspended, wakeup it");
		rc = es305_wakeup();
		if (rc < 0) {
			printk("ES305 set path fail, wakeup fail");
			return rc;
		}
	}

	sw_reset = ((A200_msg_Reset << 16) | RESET_IMMEDIATE);
	
	es305_current_config = newid;
	if (es305_cmds_len > 0) {
		int cmd_size = 0;
		cmd_size = build_cmds(custom_cmds, newid);
		if (cmd_size > 0)
			i2c_cmds = custom_cmds;
		size = cmd_size;
	}
	dprintk("%s: change to mode %d\n", __func__, newid);
	dprintk("%s: block write start (size = %d)\n", __func__, size);

		/* DEBUG: dump cmds */
#if 0
	{
		int i = 0;
		for (i = 1; i <= size; i++) {
			printk("%x ", *(i2c_cmds + i - 1));
			if (!(i % 4))
				printk("\n");
		}
	}
#endif

	es305_port_config();
	/* size of block write = 128, delay time = 20ms */
	/* TODO: do stress test */
   
	printk("ES305 set path, total cmd %d, pass %d, remainder %d", size, pass, remainder);
		
    while(pass) {
		/* es305 block write */
		dprintk("%s: block write %d byte start\n", __func__, block_size);
		rc = es305_i2c_write(ptr, block_size);
		if (rc < 0) {
			printk("ES305 CMD block write error!\n");
			es305_i2c_sw_reset(sw_reset);
			return rc;
		}
		dprintk("%s: block write %d byte end\n", __func__, block_size);
		ptr += block_size;
		pass--;
		
		mdelay(20);
		/* es305 block read */
		memset(ack_buf, 0, sizeof(ack_buf));
		dprintk("%s: CMD ACK block read start\n", __func__);
		rc = es305_i2c_read(ack_buf, block_size);
		if (rc < 0) {
			printk("%s: CMD ACK block read error\n", __func__);
			DEBUG_STRESS_TEST_COUNT++;
			es305_i2c_sw_reset(sw_reset);
			return rc;
		} else {
			printk("%s: CMD ACK block read end\n", __func__);
			/* DEBUG : dump ACK */
#if 0
	{
		int i = 0;
		for (i = 1; i <= size; i++) {
			printk("%x ", ack_buf[i-1]);
			if (!(i % 4))
				printk("\n");
		}
	}
#endif		
		if (*ack_buf != 0x80) {
			printk("%s: CMD ACK fail, ES305 may be died\n", __func__);
			DEBUG_STRESS_TEST_COUNT++;
			es305_i2c_sw_reset(sw_reset);
			return -1;
		}
	} // end else
    }
			
	if (remainder) {
		dprintk("%s: block write %d byte start\n", __func__, remainder);
		rc = es305_i2c_write(ptr, remainder);
		if (rc < 0) {
			printk("ES305 CMD block write error!\n");
			
			es305_i2c_sw_reset(sw_reset);
			return rc;
		}
		dprintk("%s: block write %d byte end\n", __func__, remainder);

		mdelay(20);
		/* es305 block read */
		memset(ack_buf, 0, sizeof(ack_buf));
		dprintk("%s: CMD ACK block read start\n", __func__);
		rc = es305_i2c_read(ack_buf, remainder);
		if (rc < 0) {
			printk("%s: CMD ACK block read error\n", __func__);
			DEBUG_STRESS_TEST_COUNT++;
			es305_i2c_sw_reset(sw_reset);
			return rc;
		} else {
			dprintk("%s: CMD ACK block read end\n", __func__);
			/* DEBUG : dump ACK */
#if 0
		{
			int i = 0;
			for (i = 1; i <= size; i++) {
				printk("%x ", ack_buf[i-1]);
				if (!(i % 4))
					printk("\n");
			}
		}
#endif		
			if (*ack_buf != 0x80) {
				printk("%s: CMD ACK fail, ES305 may be died\n", __func__);
				DEBUG_STRESS_TEST_COUNT++;
				es305_i2c_sw_reset(sw_reset);
				return -1;
			}
		} // end else
	}
	dprintk("%s: block write end\n", __func__);
	dprintk("ES305 set path(%d) ok\n", newid);

	return 0;
}

static int es305_do_msg_data(struct ES305_msg_data *data)
{
	int rc = 0;
	int i = 0;
	unsigned int count = 0, *msg = NULL;
	unsigned int cmd;
	
	dprintk("%s\n",__func__);

	count = data->count;
	msg = data->msg;

	for(i=0;i<count;i++){
		cmd = *(msg+i);
		dprintk("%d,%x\n",i,cmd);
		rc = execute_cmdmsg(cmd);
		if(rc < 0){
			printk("excute cmdmsg:%x error\n",cmd);
			return -ESRCH;
		}
	}
	return 0;
}

//set es305 path
static int es305_set_path(unsigned int pathid,bool sleep)
{
	int rc = 0;
	struct ES305_msg_data msg;
	unsigned int cmd[20];

	printk("%s,pathid = %d\n",__func__,pathid);
	switch(pathid){
		case ES305_PATH_BYPASS_PD2PB:
			cmd[0] = (ES305_Path_Through<<16)|ES305_PD2PB_data_clk;
			msg.count = 1;
			msg.msg = cmd;
			break;
			
		case ES305_PATH_BYPASS_PC2PA:
			cmd[0] = (ES305_Path_Through<<16)|ES305_PC2PA_data_clk;
			msg.count = 1;
			msg.msg = cmd;		
			break;
			
		case ES305_PATH_BYPASS_PD2PB_PC2PA:
			cmd[0] = (ES305_Path_Through<<16)|ES305_PC2PA_data_clk;			
			cmd[1] = (ES305_Path_Through<<16)|ES305_PD2PB_data_clk;
			msg.count = 2;
			msg.msg = cmd;		
			break;
			
		default:
			printk("no such path, please reconfigure");
			msg.count = 0;
			msg.msg = NULL;
			break;
	}

	
	es305_do_msg_data(&msg);

	//enter sleep mode
	if(sleep == true){
		cmd[0] = A200_msg_SetPowerState_Sleep;
		msg.count = 1;
		msg.msg = cmd;
		rc = es305_do_msg_data(&msg);
		if(rc < 0){
			printk("excute msg data error\n");
			return -ESRCH;
		}
	}
	return 0;
}


static long es305_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
//	struct es305img img;
	struct ES305_config_data cfg;
	int rc = 0;
	unsigned int pathid;
	char msg[4];
	unsigned int cmdmsg = 0;

	int rc1 = 0;
	
	switch (cmd) {
	case ES305_DEV_INIT:
		printk("%s -------------------------ES305_DEV_INIT \n", __func__);
		es305_device_init();
		execute_cmdmsg(ES305_PB2PD_BYPASS);
		es305_set_call_mode(CALL_MODE_RCV);
		execute_cmdmsg(CALL_MODE_DISABLE_PROCESSING);
		#if 0
		execute_cmdmsg(ES305_PA2PC_BYPASS);
		#endif
		break;

	case ES305_SET_MIC_MODE:
		if (copy_from_user(&es305_mic_mode, argp, sizeof(unsigned int))) {
			printk("%s: copy from user failed.\n", __func__);
			return -EFAULT;
		}
		printk("%s ES305_SET_MIC_MODE=%d \n", __func__,es305_mic_mode);
		if (es305_mic_mode < 0 || es305_mic_mode >= (unsigned int)ES305_PATH_MAX)
			return -EINVAL;

		break;

	case ES305_RESET_CMD:
		printk("%s ES305_RESET_CMD \n", __func__);
		es305_hardreset();
		break;

	case ES305_SLEEP_CMD:
		printk("%s ES305_SLEEP_CMD \n", __func__);
		es305_sleep();
		break;
	case ES305_WAKEUP_CMD:
		printk("%s ES305_WAKEUP_CMD \n", __func__);
		es305_wakeup();
		break;
	case ES305_SYNC_CMD:
		printk("%s ES305_SEND_SYNC \n", __func__);
		mdelay(100);


		/* Audience bug:
		symptom: alway return not ready ACK first time
		solution: write SYNC CMD at least two times
		write 80000000, return 00000000
		write 80000000, return 80000000
		*/
		rc = execute_cmdmsg(A200_msg_Sync_Polling);
		if (rc < 0) {
			printk("%s: sync command error %d\n", __func__, rc);
		} else {
			printk("%s: sync command ok\n", __func__);
			es305_request_wakeup_pin();
			if (es305_sleep() < 0) {
				printk("%s: sleep command fail\n", __func__);
			}
			else {
				ES305_SYNC_DONE = 1;
				printk("%s: sleep command ok\n", __func__);
			}
		}
		break;
	case ES305_READ_SYNC_DONE:
		printk("%s ES305_READ_SYNC_DONE\n", __func__);
		if (copy_to_user(argp, &ES305_SYNC_DONE, sizeof(ES305_SYNC_DONE))) {
			return -EFAULT;
		}
		break;

	case ES305_MDELAY:
		printk("%s ES305_MDELAY\n", __func__);
		#if 0
		unsigned int dtime;
		if (copy_from_user(&dtime, argp, sizeof(dtime))) {
			printk("%s: copy from user failed.\n", __func__);
			return -EFAULT;
		}
		mdelay(dtime);
		#endif
		break;
	case ES305_READ_FAIL_COUNT:
		printk("%s ES305_READ_FAIL_COUNT \n", __func__);

		if (copy_to_user(argp, &DEBUG_STRESS_TEST_COUNT, sizeof(DEBUG_STRESS_TEST_COUNT))) {
			return -EFAULT;
		}

		break;

	case ES305_SET_CONFIG:
		printk("%s ES305_SET_CONFIG \n", __func__);

		if (copy_from_user(&pathid, argp, sizeof(unsigned int))) {
			printk("%s: copy from user failed.\n", __func__);
			return -EFAULT;
		}

		if (pathid < 0 || pathid >= (unsigned int)ES305_PATH_MAX)
			return -EINVAL;

		rc = es305_set_config(pathid, ES305_CONFIG_FULL);
		if (rc < 0)
			printk("%s: ES305_SET_CONFIG (%d) error %d!\n",
				__func__, pathid, rc);

		break;

	case ES305_SET_PATH:
		printk("%s ES305_SET_PATH \n", __func__);
		if (copy_from_user(&pathid, argp, sizeof(unsigned int))) {
			printk("%s: copy from user failed.\n", __func__);
			return -EFAULT;
		}
		
		rc = es305_set_path(pathid,false);
		if(rc < 0){
			printk("%s: es305_set_path failed.\n", __func__);
			return -EFAULT;
		}
		break;
		
	case ES305_SET_PARAM:
		printk("%s ES305_SET_PARAM \n", __func__);
		es305_cmds_len = 0;
		cfg.cmd_data = 0;
		if (copy_from_user(&cfg, argp, sizeof(cfg))) {
			printk("%s: copy from user failed.\n", __func__);
			return -EFAULT;
		}

		if (cfg.data_len <= 0 || cfg.data_len > PARAM_MAX) {
				printk("%s: invalid data length %d\n", \
					__func__,  cfg.data_len);
				return -EINVAL;
		}

		if (cfg.cmd_data == NULL) {
			printk("%s: invalid data\n", __func__);
			return -EINVAL;
		}

		if (config_data == NULL)
			config_data = kmalloc(cfg.data_len, GFP_KERNEL);
		if (!config_data) {
			printk("%s: out of memory\n", __func__);
			return -ENOMEM;
		}
		if (copy_from_user(config_data, cfg.cmd_data, cfg.data_len)) {
			printk("%s: copy data from user failed.\n",\
				__func__);
			kfree(config_data);
			config_data = NULL;
			return -EFAULT;
		}
		es305_cmds_len = cfg.data_len;
		printk("%s: update ES305 mode commands success.\n",\
			__func__);
		rc = 0;
		break;

	case ES305_READ_DATA:
//		rc = chk_wakeup_a1026();
//		if (rc < 0)
//			return rc;
		rc = es305_i2c_read(msg, 4);
		if (copy_to_user(argp, &msg, 4))
			return -EFAULT;
		break;
	case ES305_WRITE_MSG:
//		rc = chk_wakeup_a1026();
//		if (rc < 0)
//			return rc;
		if (copy_from_user(msg, argp, sizeof(msg)))
			return -EFAULT;
//		printk("%s\n",msg)
		rc = es305_i2c_write(msg, 4);
		break;
		
	case ES305_SET_CMDMSG:
		printk("%s ES305_SET_CMDMSG \n", __func__);
		
		if (copy_from_user(&cmdmsg, argp, sizeof(unsigned int))) {
			printk("%s: copy from user failed.\n", __func__);
			return -EFAULT;
		}
		rc = execute_cmdmsg(cmdmsg);
		if(rc < 0){
			printk("excute cmdmsg:%x error\n",cmdmsg);
			return -ESRCH;
		}
		
		break;

	default:
		printk("%s: invalid command\n", __func__);
		rc = -EINVAL;
		break;
	}

	return rc;
}
 
static int es305_open(struct inode *inode, struct file *file)
{
	printk("%s\n", __func__);

	return 0;
}

int es305_release(struct inode *inode, struct file *file)
{
	printk("%s\n", __func__);
	return 0;
}
static ssize_t es305_write (struct file *file, const char __user *buf, size_t size, loff_t *offset)
{
	unsigned long rc;
//	unsigned int reg;
//	unsigned int dest;
	char *data = kmalloc(size,GFP_KERNEL);
	rc = copy_from_user((void *)data, buf, size);
	if(rc< 0){
		printk("copy_from_user error!!\n");
		kfree(data);
		return -ENOMEM;
	}
	
	printk("%s size %d\n",__func__,size);
	printk("%s, %s",__func__,data);
	if(data[0] == 'r'){
		if(data[1] == '0'){

			printk("gpio_es305_reset ::0\n");
			es305_gpio_set_value(pdata->gpio_es305_reset, 0);
		}
		else{
			printk("gpio_es305_reset ::1\n");
			es305_gpio_set_value(pdata->gpio_es305_reset, 1);
		}
	}
	else if(data[0] == 'c'){
		if(data[1]>='0' ||data[1]<='4'){
			printk("gpio_es305_reset ::2\n");
			es305_gpio_set_value(pdata->gpio_es305_reset, data[1]-'0');
		}
		else{
			printk("config error %d!!\n",data[1]);
		}	
	}
	else if(data[0] == 't'){
		char data = '8';
		es305_i2c_write(&data, 2);
	}
	else{
		printk("unsupport cmd\n");
	}
	kfree(data);
	return size;
}

static const struct file_operations es305_fileops = {
	.owner = THIS_MODULE,
	.open = es305_open,
	.unlocked_ioctl = es305_ioctl,
	.release = es305_release,
	.write = es305_write,
};


static struct miscdevice es305_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "audience_es305",
	.fops = &es305_fileops,
};

static void es305_gpio_set_value(int gpio, int value)
{
	printk("%s:gpio-%d,val-%d\n",__func__,gpio,value);
	
	if (gpio >  0)
		gpio_set_value(gpio, value);
}

static int es305_i2c_read(char *rxData, int length)
{
	int rc;
	struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};
#if 0
	{
			int i = 0;
			for (i = 0; i < length; i++)
				dprintk("%s: rx[%d] = %2x\n", __func__, \
					i, rxData[i]);
	}
#endif

	rc = i2c_transfer(this_client->adapter, msgs, 1);
	if (rc < 0) {
		printk("%s: transfer error %d\n", __func__, rc);
		return rc;
	}

	return 0;
}

static int es305_i2c_write(char *txData, int length)
{
	int rc;
	struct i2c_msg msg[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};
	//printk("%s,addr %x\n",__func__,this_client->addr); //open by terry
#if 0 //DEBUG_
	{
		int i = 0;
		for (i = 0; i < length; i++)
			dprintk("%s: tx[%d] = %2x\n", __func__, \
				i, txData[i]);
	}
#endif

	rc = i2c_transfer(this_client->adapter, msg, 1);
	if (rc < 0) {
		printk("%s: transfer error %d\n", __func__, rc);
		return rc;
	}
 
	return 0;
}
 
int execute_cmdmsg(unsigned int msg)
{
	int rc = 0;
	int retries, pass = 0;
	unsigned char msgbuf[4];
	unsigned char chkbuf[4];
	unsigned int sw_reset = 0;

	dprintk("%s:%x\n",__func__,msg);

	sw_reset = ((A200_msg_Reset << 16) | RESET_IMMEDIATE);

	msgbuf[0] = (msg >> 24) & 0xFF;
	msgbuf[1] = (msg >> 16) & 0xFF;
	msgbuf[2] = (msg >> 8) & 0xFF;
	msgbuf[3] = msg & 0xFF;
 
	retries = 3;//POLLING_RETRY_CNT;
	while (retries--) {
		rc = 0;

		mdelay(POLLING_TIMEOUT); /* use polling */
		rc = es305_i2c_write(msgbuf, 4);
		if (rc < 0) {
			printk("%s: error %d\n", __func__, rc);
			es305_i2c_sw_reset(sw_reset);
			return rc;
		}
		//printk("execute_cmdmsg %8x\n", msg);

		/* We don't need to get Ack after sending out a suspend command */
		if (msg == A200_msg_SetPowerState_Sleep) {
			return rc;
		}


		memset(chkbuf, 0xaa, sizeof(chkbuf));
		rc = es305_i2c_read(chkbuf, 4);
		if (rc < 0) {
			printk("%s: ack-read error %d (%d retries)\n",\
				__func__, rc, retries);
			continue;
		}

#ifdef DEBUG
		dprintk("%s: msgbuf[0] = %x\n", __func__, chkbuf[0]);
		dprintk("%s: msgbuf[1] = %x\n", __func__, chkbuf[1]);
		dprintk("%s: msgbuf[2] = %x\n", __func__, chkbuf[2]);
		dprintk("%s: msgbuf[3] = %x\n", __func__, chkbuf[3]);
#endif

		if (msgbuf[0] == chkbuf[0]  && msgbuf[1] == chkbuf[1] && msgbuf[2] == chkbuf[2] &&msgbuf[3] == chkbuf[3]) {
			pass = 1;
			printk("Execute_cmd OK, %08x\n", msg);
			break;
		} else if (msgbuf[2] == 0xff && msgbuf[3] == 0xff) {
			printk("%s: illegal cmd %08x, %x, %x, %x, %x\n", __func__, msg, chkbuf[0], chkbuf[1], chkbuf[2], chkbuf[3] );
			rc = -EINVAL;
			continue;
		} else if ( msgbuf[2] == 0x00 && msgbuf[3] == 0x00 ) {
			printk("%s: not ready (%d retries), %x, %x, %x, %x\n", __func__,
				retries, chkbuf[0], chkbuf[1], chkbuf[2], chkbuf[3]);
			rc = -EBUSY;
			continue;
		} else {
			printk("%s: cmd/ack mismatch: (%d retries left), %x, %x, %x, %x\n",
				__func__,
				retries, chkbuf[0], chkbuf[1], chkbuf[2], chkbuf[3]);
			rc = -EBUSY;
			continue;
		}
	}

	if (!pass) {
		printk("%s: failed execute cmd %08x (%d)\n", __func__,
			msg, rc);
		es305_i2c_sw_reset(sw_reset);
	}
	return rc;
}

int es305_downloadfirmware_IIC(struct es305img *p)
{
	int rc = 0;
	struct vp_ctxt ctxt_vp;
	struct vp_ctxt *vp = &ctxt_vp;
	int remaining;
	unsigned char *index;
	int time1 = 0;
	int time2 = 0;

	time1 = jiffies;
	if(p == NULL || p->buf== NULL || p->img_size <= 0)
	{		
		vp->data = (unsigned char *)es305_firmware_data;
		vp->img_size = sizeof(es305_firmware_data);
	}
	else
	{
		vp->data = p->buf;
		vp->img_size = p->img_size;
	}

	remaining = vp->img_size / 32;
	index = vp->data;	

	printk("%s: starting to load image (%d passes)...img_size = %d\n",__func__,
		remaining + !!(vp->img_size % 32),vp->img_size);

	for (; remaining; remaining--, index += 32) {
		rc = es305_i2c_write(index, 32);
		if (rc < 0)
			break;
	}

	if (rc >= 0 && vp->img_size % 32)
		rc = es305_i2c_write(index, vp->img_size % 32);

	if (rc < 0) {
		pr_err("%s: fw load error %d \n",__func__, rc);
		return rc;
	}

	mdelay(120); /* Delay time before issue a Sync Cmd */

	time2 = jiffies;
	printk("%s: firmware loaded successfully\n", __func__);
	printk("cust time %d ms\n", jiffies_to_msecs(time2-time1));

	return rc;
}

void es305_set_call_mode(enum es305_call_mode mode)
{
	switch(mode)
	{
		case CALL_MODE_OFF:
			execute_cmdmsg(CALL_MODE_DISABLE_PROCESSING);
			printk("%s-------CALL_MODE_DISABLE_PROCESSING\n", __func__);
			break;
		case CALL_MODE_SPK:
			execute_cmdmsg(CALL_MODE_HANDFREE);
			printk("%s-------CALL_MODE_HANDFREE\n", __func__);
			break;
		case CALL_MODE_RCV:
			if(es305_mic_mode == 1)
			{
				execute_cmdmsg(CALL_MODE_EARPIECE_DOUBLE_MIC);
				printk("%s-------CALL_MODE_EARPIECE_DOUBLE_MIC\n", __func__);
			}
			else
			{
				execute_cmdmsg(CALL_MODE_EARPIECE_SINGLE_MIC);
				printk("%s-------CALL_MODE_EARPIECE_SINGLE_MIC\n", __func__);
			}
			break;
		case CALL_MODE_HP:
			execute_cmdmsg(CALL_MODE_HEADSET);
			printk("%s-------CALL_MODE_HEADSET\n", __func__);
			break;
		case CALL_MODE_BT_NORMAL:
			execute_cmdmsg(CALL_MODE_BT);
			printk("%s-------CALL_MODE_BT\n", __func__);
			break;
		case CALL_MODE_MUTE:
			execute_cmdmsg(0x801B00A6);	//GAIN -90db = mute for switching mode between bt mode and the other mdoe
			execute_cmdmsg(0x801B01A6);	//GAIN -90db = mute for switching mode between bt mode and the other mdoe
			printk("%s-------MUTE uplink volume\n", __func__);
			break;
		case CALL_MODE_BT_VOIP:
			execute_cmdmsg(VOIP_MODE_BT);
			printk("%s-------VOIP_MODE_BT\n", __func__);
			break;
		case CALL_MODE_NORMAL_VOIP:
			execute_cmdmsg(ES305_PB2PD_BYPASS);
			printk("%s-------CALL_MODE_NORMAL_VOIP\n", __func__);
			break;
		default:
			printk("%s-------UNKNOW\n", __func__);
			break;
	}
}
EXPORT_SYMBOL(es305_set_call_mode);
//download firmware
static int es305_device_init(void)
{
	struct es305img img;
	int rc = 0;
	int retry = RETRY_CNT;
	char buf[2];

	printk("%s\n",__func__);
		
	img.buf= (unsigned char *)es305_firmware_data;
	img.img_size = sizeof(es305_firmware_data);

	while (retry--) {

		/* Boot Cmd to A1026 */
		buf[0] = ES305_msg_BOOT >> 8;
		buf[1] = ES305_msg_BOOT & 0xff;

		rc = es305_i2c_write(buf, 2);
		if (rc < 0) {
			pr_err("%s: set boot mode error (%d retries left)\n",
				__func__, retry);
			continue;
		}

		mdelay(1); /* use polling */
		rc = es305_i2c_read(buf, 1);
		if (rc < 0) {
			pr_err("%s: boot mode ack error (%d retries left)\n",
				__func__, retry);
			continue;
		}

		if (buf[0] != ES305_msg_BOOT_ACK) {
			pr_err("%s: not a boot-mode ack (%d retries left)\n",
				__func__, retry);
			continue;
		}

		es305_downloadfirmware_IIC(&img);//down firmware to deives by iic
		
		rc = execute_cmdmsg(A200_msg_Sync_Polling);
		if (rc < 0) {
			pr_err("%s: sync command error %d (%d retries left)\n",
				__func__, rc, retry);
			continue;
		}
		break;
	}
	
	return rc;
}
//download firmware
static int es305_suspend(struct i2c_client *client, pm_message_t mesg)
{

	/* vp suspend function will be dominated by in-call mode, not system state */
	if(tc4_get_call_flg ()!= 1){
		printk("es305 now suspended\n");
		//es305_sleep(); 
		execute_cmdmsg(ES305_PA2PC_BYPASS);
	}
	else{
		printk("es305 can't goto suspend because of calling now\n");
	}
	
	return 0;
}

static int es305_resume(struct i2c_client *client)
{
	//printk("%s\n", __func__);

	/* vp resume function will be dominated by in-call mode, not system state */
	if(tc4_get_call_flg ()!= 1){
		printk("es305 now es305_resume\n");
		//es305_wakeup(); 
		es305_set_call_mode(2);
		execute_cmdmsg(CALL_MODE_DISABLE_PROCESSING);
	}
	else{
		printk("es305 no need to resume because of calling now\n");
	}
	  
 	return 0;
}

static int es305_board_init(struct es305_platform_data *pdata,struct i2c_client *client)
{
	int rc=0;
	
	printk("%s\n",__func__);
	
	pdata->es305_mclk_init();
	pdata->es305_mclk_enable(false); //disable mclk first
	mdelay(5);//delay a whihe and than set mclk
	pdata->es305_mclk_enable(true); //enable mclk first

	gpio_free(pdata->gpio_es305_reset);
	rc = gpio_request(pdata->gpio_es305_reset, "es305 GPIO reset");
	if (rc < 0) {
		printk("%s: gpio request reset pin failed\n", __func__);
 	}

	s3c_gpio_cfgall_range(pdata->gpio_es305_reset, 1,S3C_GPIO_SFN(1), S3C_GPIO_PULL_UP);
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("%s: i2c check functionality error\n", __func__);

		rc = -ENODEV;
		goto err_free_gpio_all;
	}
		
	es305_gpio_set_value(pdata->gpio_es305_reset, 0);
	dprintk("***** 1. %s OK :%d *****\n", "AUD_ES305_RESET", gpio_get_value(pdata->gpio_es305_reset));

	mdelay(50);

	es305_gpio_set_value(pdata->gpio_es305_reset, 1);
	dprintk("***** 2. %s OK :%d *****\n", "AUD_ES305_RESET", gpio_get_value(pdata->gpio_es305_reset));
	return 0;
	
err_free_gpio_all:
	gpio_free(pdata->gpio_es305_reset);
	return -1;

}

static int es305_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	
	dprintk("%s \n", __func__);

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			rc = -ENOMEM;
			printk("%s: platform data is NULL\n", __func__);
			goto err_alloc_data_failed;
		}
	}
	this_client = client;
	
	rc = es305_board_init(pdata,client);
	if(rc < 0){
		return -1;
	}
	//es305_device_init();

	rc = misc_register(&es305_device);
	if (rc) {
		printk("%s: es305_device register failed\n", __func__);
//		goto err_free_gpio_all;
	}

	return 0;

err_alloc_data_failed:
	return rc;
}


static int es305_i2c_remove(struct i2c_client *client)
{
	struct es305_platform_data *pes305data = i2c_get_clientdata(client);
	dprintk("%s\n", __func__);

	kfree(pes305data);
	return 0;
}


/* i2c codec control layer */
static const struct i2c_device_id es305_i2c_id[] = {
       { "audience_es305", 0 },
       { }
};


MODULE_DEVICE_TABLE(i2c, es305_i2c_id);


static struct i2c_driver es305_i2c_driver = {
	.driver = {
		.name = "audience_es305",
		.owner = THIS_MODULE,
	},
	.probe    = es305_i2c_probe,
	.remove   = es305_i2c_remove,
	.suspend = es305_suspend,
	.resume	= es305_resume,
	.id_table = es305_i2c_id,
};


static int __init es305_init(void)
{
	int ret;
	dprintk("%s\n", __func__);

	ret = i2c_add_driver(&es305_i2c_driver);

	return ret;
}
module_init(es305_init);

static void __exit es305_deinit(void)
{
	dprintk("%s\n", __func__);

	i2c_del_driver(&es305_i2c_driver);
}
module_exit(es305_deinit);

MODULE_DESCRIPTION("Audience eS305 driver");
MODULE_AUTHOR(" Corp.");
MODULE_LICENSE("GPL");



