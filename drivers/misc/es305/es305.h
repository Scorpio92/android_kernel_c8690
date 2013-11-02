#ifndef __LINUX_es305_H
#define __LINUX_es305_H
/*-----------------API ---------------------*/

/* RE-DOWNLOAD FIRMWARE  */
#define A200_msg_BootloadInitiate 0x80030000


/* AFTER POWER UP OR SYSYTEM RESET*/
#define ES305_msg_BOOT		0x0001
#define ES305_msg_BOOT_ACK	0x01

/* sync */
#define A200_msg_Sync_Polling 0x80000000
#define A200_msg_Sync_Interrupt_L 0x80000001
#define SYNC_WAIT_TIME_BEFORE 10 // ms
/* S/W reset */
#define A200_msg_Reset 0x8002
#define RESET_IMMEDIATE 0x0000
#define RESET_DELAYED 0x0001

/*wake up */
#define A200_msg_Wakeup
#define WAKEUP_WAITTIME_AFTER 30 // ms

/* get/set device id and parameter 
* set :
1. es305_msg(SetDeviceParmID, Parameter ID)
   Parameter ID : 8 bit device + 8 bit device parameter
2. read 4 byte ack.
3. es305_msg(SetDeviceParm, value)
4. read 4 byte ack.
* get:
1. es305_msg(GetDeviceParm, Parameter ID)
   Parameter ID : 8 bit device + 8 bit device parameter

*/
#define A200_msg_GetDeviceParm		0x800B
#define A200_msg_SetDeviceParmID	0x800C
#define A200_msg_SetDeviceParm		0x800D

/* power state */
#define A200_msg_SetPowerState_Sleep 0x80100001
#define STOP_CLOCK_WAITTIME_AFTER 120 // ms

/* call mode */
#define CALL_MODE_EARPIECE_DOUBLE_MIC 0x80310000
#define CALL_MODE_EARPIECE_SINGLE_MIC 0x80310006
#define CALL_MODE_HANDFREE 0x80310002
#define CALL_MODE_HEADSET 0x80310003
#define CALL_MODE_BT 0x80310004
#define VOIP_MODE_BT 0x80310007
#define CALL_MODE_DISABLE_PROCESSING 0x801C0000

#define ES305_PB2PD_BYPASS 0x805200f7
#define ES305_PA2PC_BYPASS 0x805200e2

#define ES305_Path_Through 0x8052

#define ES305_PD2PB_data_clk 0x00f7
#define ES305_PD2PB_data 0x00f4
#define ES305_PC2PA_data_clk 0x00e2
#define ES305_PC2PA_data 0x0060


/* ------------- general definitions -------------*/
#define ES305_I2C_NAME "audience_es305"
#define ES305_I2S_SLAVE_ADDRESS (0x3E)
#define POLLING_TIMEOUT 20 // ms
#define RESET_TIMEOUT 50 // ms

// unconfirmed
#define ES305_MAX_FW_SIZE	(32*4096)

#define TIMEOUT			20 /* ms */
#define RETRY_CNT		5
#define POLLING_RETRY_CNT	10
#define ES305_ERROR_CODE	0xffff
#define ES305_SLEEP		0
#define ES305_ACTIVE		1
#define ES305_CMD_FIFO_DEPTH	32 /* 128 / 4 = 32 */
#define ERROR			0xffffffff

/* ---------------------Stucture -------------------*/

struct es305_platform_data {
	uint32_t gpio_es305_reset;
	void (* es305_mclk_init )(void);
	void (* es305_mclk_enable )(bool enable);
	uint32_t gpio_es305_wakeup;
};

struct ES305_config_data {
	unsigned int data_len;
	unsigned int mode_num;
	unsigned char *cmd_data;  /* [mode][cmd_len][cmds..] */
};

struct ES305_msg_data{
	unsigned int count;
	unsigned int *msg;
};

enum ES305_config_mode {
	ES305_CONFIG_FULL,
	ES305_CONFIG_VP
};

enum es305_call_mode {
	CALL_MODE_OFF=0,
	CALL_MODE_SPK,
	CALL_MODE_RCV,
	CALL_MODE_HP,
	CALL_MODE_BT_NORMAL,
	CALL_MODE_MUTE,
	CALL_MODE_BT_VOIP,
	CALL_MODE_NORMAL_VOIP
} ;

enum ES305_PathID {
 ES305_PATH_SUSPEND = -1,
 ES305_PATH_Initial = 0,
 ES305_PATH_Reciver_NB,
 ES305_PATH_Headphone_NB,
 ES305_PATH_Speaker_NB,
 ES305_PATH_Reciver_WB,
 ES305_PATH_Headphone_WB,
 ES305_PATH_Speaker_WB,
 ES305_PATH_HAC_NB,
 ES305_PATH_TTY_NB,
 ES305_PATH_BT,
 ES305_PATH_BYPASS_PD2PB ,//10
 ES305_PATH_BYPASS_PC2PA ,//11
 ES305_PATH_BYPASS_PD2PB_PC2PA,//12
 ES305_PATH_MAX
};


enum ES305_NS_states {
 ES305_NS_STATE_AUTO,
 ES305_NS_STATE_OFF,
 ES305_NS_STATE_CT,
 ES305_NS_STATE_FT,
 ES305_NS_NUM_STATES
};


struct es305img {
	unsigned char *buf;
	unsigned img_size;
};



static unsigned char ES305_CMD_ROUTE[28][4]  = {
{0x80, 0x0C, 0x0A, 0x00},
{0x80, 0x0D, 0x00, 0x0F},
// 0x800C:SetDeviceParmID, 0x0A:PCM0, 0x00:PCM WordLength, 0x800D:SetDeviceParm, 0x000F:16 Bits
{0x80, 0x0C, 0x0A, 0x02},
{0x80, 0x0D, 0x00, 0x00},
// 0x800C:SetDeviceParmID, 0x0A:PCM0, 0x02:PCM DelFromFsTx, 0x800D:SetDeviceParm, 0x0000:(0 clocks)
{0x80, 0x0C, 0x0A, 0x03},
{0x80, 0x0D, 0x00, 0x01},
// 0x800C:SetDeviceParmID, 0x0A:PCM0, 0x03:PCM DelFromFsRx, 0x800D:SetDeviceParm, 0x0001:(1 clock)
{0x80, 0x0C, 0x0A, 0x04},
{0x80, 0x0D, 0x00, 0x00},
// 0x800C:SetDeviceParmID, 0x0A:PCM0, 0x04:PCM Latch Edge, 0x800D:SetDeviceParm, 0x0000:TxFalling/RxRising
{0x80, 0x0C, 0x0A, 0x05},
{0x80, 0x0D, 0x00, 0x01},
// 0x800C:SetDeviceParmID, 0x0A:PCM0, 0x05:PCM Endianness, 0x800D:SetDeviceParm, 0x0001:Big Endian
{0x80, 0x0C, 0x0A, 0x06},
{0x80, 0x0D, 0x00, 0x01},
// 0x800C:SetDeviceParmID, 0x0A:PCM0, 0x06:PCM Tristate Enable, 0x800D:SetDeviceParm, 0x0001:Enable
{0x80, 0x0C, 0x0A, 0x07},
{0x80, 0x0D, 0x00, 0x01},
// 0x800C:SetDeviceParmID, 0x0A:PCM0, 0x07:PCM Audio Port Mode, 0x800D:SetDeviceParm, 0x0001:I2S
{0x80, 0x0C, 0x0C, 0x00},
{0x80, 0x0D, 0x00, 0x0F},
// 0x800C:SetDeviceParmID, 0x0C:PCM2, 0x00:PCM WordLength, 0x800D:SetDeviceParm, 0x000F:16 Bits
{0x80, 0x0C, 0x0C, 0x02},
{0x80, 0x0D, 0x00, 0x00},
// 0x800C:SetDeviceParmID, 0x0C:PCM2, 0x02:PCM DelFromFsTx, 0x800D:SetDeviceParm, 0x0000:(0 clocks)
{0x80, 0x0C, 0x0C, 0x03},
{0x80, 0x0D, 0x00, 0x01},
// 0x800C:SetDeviceParmID, 0x0C:PCM2, 0x03:PCM DelFromFsRx, 0x800D:SetDeviceParm, 0x0001:(1 clock)
{0x80, 0x0C, 0x0C, 0x04},
{0x80, 0x0D, 0x00, 0x00},
// 0x800C:SetDeviceParmID, 0x0C:PCM2, 0x04:PCM Latch Edge, 0x800D:SetDeviceParm, 0x0000:TxFalling/RxRising
{0x80, 0x0C, 0x0C, 0x05},
{0x80, 0x0D, 0x00, 0x01},
// 0x800C:SetDeviceParmID, 0x0C:PCM2, 0x05:PCM Endianness, 0x800D:SetDeviceParm, 0x0001:Big Endian
{0x80, 0x0C, 0x0C, 0x06},
{0x80, 0x0D, 0x00, 0x01},
// 0x800C:SetDeviceParmID, 0x0C:PCM2, 0x06:PCM Tristate Enable, 0x800D:SetDeviceParm, 0x0001:Enable
{0x80, 0x0C, 0x0C, 0x07},
{0x80, 0x0D, 0x00, 0x01}
// 0x800C:SetDeviceParmID, 0x0C:PCM2, 0x07:PCM Audio Port Mode, 0x800D:SetDeviceParm, 0x0001:I2S
};

int es305_set_config(int newid, int mode);


#define ES305_IOCTL_MAGIC ';'

#define ES305_BOOTUP_INIT _IOW(ES305_IOCTL_MAGIC, 1, struct es305img *)
#define ES305_SET_CONFIG _IOW(ES305_IOCTL_MAGIC, 2, unsigned int *)
#define ES305_SET_PARAM	   _IOW(ES305_IOCTL_MAGIC, 4, struct ES305_config_data *)
#define ES305_DEV_INIT _IO(ES305_IOCTL_MAGIC, 7)
#define ES305_CALL_OFF _IO(ES305_IOCTL_MAGIC, 8)
#define ES305_SYNC_CMD _IO(ES305_IOCTL_MAGIC, 9)
#define ES305_SET_MIC_MODE _IOW(ES305_IOCTL_MAGIC, 10, unsigned int *)
#define ES305_SLEEP_CMD _IO(ES305_IOCTL_MAGIC, 11)
#define ES305_RESET_CMD _IO(ES305_IOCTL_MAGIC, 12)
#define ES305_WAKEUP_CMD _IO(ES305_IOCTL_MAGIC, 13)
#define ES305_MDELAY _IOW(ES305_IOCTL_MAGIC, 14, unsigned int)
#define ES305_READ_FAIL_COUNT _IOR(ES305_IOCTL_MAGIC, 15, unsigned int *)
#define ES305_READ_SYNC_DONE _IOR(ES305_IOCTL_MAGIC, 16, bool *)
#define ES305_READ_DATA		_IOR(ES305_IOCTL_MAGIC, 17, unsigned)
#define ES305_WRITE_MSG		_IOW(ES305_IOCTL_MAGIC, 18, unsigned)
#define ES305_SET_PATH			_IOW(ES305_IOCTL_MAGIC,19,unsigned)
#define ES305_SET_CMDMSG _IOW(ES305_IOCTL_MAGIC,20,unsigned int)
#endif

int es305_sleep(void);

