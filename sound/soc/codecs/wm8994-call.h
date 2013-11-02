//#define max(a, b) ((a) > (b)? (a): (b))
//#define CALL_PATH_IN_BYPASS_MODE
//#define DEBUG_AUDIO_PARAMETERS

struct incall_reg {
	unsigned int reg;
	unsigned int val;
};

static unsigned int cache_size = 0;
static struct incall_reg *cache;
static int cache_path = -1;
volatile int calling_flag = 0;
EXPORT_SYMBOL_GPL(calling_flag);
static struct incall_reg *cur_incall_reg = NULL;
static struct incall_reg incall_spk_reg[] = {
//Clocking
{0x620 , 0x0000}, //     Write{0x34      //Select Low Power ADC/DMIC Oversample Rate,
/////////////////////////////////add for Keytone
{0x210 , 0x0073}, // AIF1 Sample Rate = 44.1 kHz, AIF1CLK/Fs ratio = 256
{0x300 , 0x4010}, // AIF1 Word Length = 16-bits, AIF1 Format = I2S
{0x302 , 0x0000}, // AIF1 Slave Mode (Default Register Value)
/////////////////////////////////
//Select Low Power DAC Oversample Rate (Default)
{0x211 , 0x0033}, //     Write{0x34      //AIF2 Sample Rate = 8 kHz, AIF2CLK/Fs ratio = 256
{0x310 , 0x4010}, //     Write{0x34      //AIF2 Word Length = 16-bits, AIF2 Format = DSP
{0x311 , 0x0000}, //     Write{0x34      //Enable AIF2 DSP Mono Mode
#ifdef CALL_PATH_IN_BYPASS_MODE
{0x312 , 0x0000}, //     Write{0x34      //AIF2 Slave Mode (Default Register Value)
#else
{0x312 , 0x4000}, //     Write{0x34      //AIF2 Slave Mode (Default Register Value)
#endif
{0x313 , 0x0070}, //     Write{0x34      //16 BCLK per mono
{0x314 , 0x0020}, //     Write{0x34      //16 BCLK per mono
		//{0x208 , 0x0007}, //     Write{0x34      //Enable the DSP processing clock for AIF2, 
{0x208 , 0x000E}, //     Write{0x34      //Enable the DSP processing clock for AIF2, 
{0x200 , 0x0001}, //     Write{0x34      //Enable the DSP processing clock for AIF2, 
                                                      //Enable the core clock,
                                                      //Set the core clock source to AIF2CLK
{0x204 , 0x0001}, //     Write{0x34      //Enable AIF2 Clock, AIF2 Clock Source = MCLK2 pin
/////////////////////////////////add for Keytone
{0x220 , 0x0000}, // FLL1 Control (1)(220H):  0000  FLL1_OSC_ENA=0, FLL1_ENA=0
{0x221 , 0x0700}, // FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=8, FLL1_FRATIO=1
{0x222 , 0x86C2}, // FLL1 Control (3)(222H):  3126  FLL1_K=0.19199
{0x223 , 0x00E0}, // FLL1 Control (4)(223H):  0100  FLL1_N=8
{0x224 , 0x0C88}, // FLL1 Control (5)(224H):  0C80  FLL1_BYP=FLL1, FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_REFCLK_DIV=MCLK / 1, FLL1_REFCLK_SRC=MCLK1
{0x220 , 0x0005}, // FLL1 Control (1)(220H):  0001  FLL1_OSC_ENA=0, FLL1_ENA=1
{0xffff,5},
{0x200 , 0x0011}, // AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=FLL1/BCLK1, AIF1CLK_INV=0, AIF1CLK_DIV=AIF1CLK, AIF1CLK_ENA=1
/////////////////////////////////
{0x240 , 0x0000}, //     Write{0x34      //FLL1 Control (1)(220H):  0000  FLL1_OSC_ENA=0, FLL1_ENA=0
{0x241 , 0x1700}, //     Write{0x34      //FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=8, FLL1_FRATIO=1
{0x242 , 0x3126}, //     Write{0x34      //FLL1 Control (3)(222H):  3126  FLL1_K=0.19199
{0x243 , 0x0100}, //     Write{0x34      //FLL1 Control (4)(223H):  0100  FLL1_N=8
{0x244 , 0x0C88}, //     Write{0x34      //FLL1 Control (5)(224H):  0C80  FLL1_BYP=FLL1, FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_REFCLK_DIV=MCLK / 1, FLL1_REFCLK_SRC=MCLK1
{0x240 , 0x0005}, //     Write{0x34      //FLL1 Control (1)(220H):  0001  FLL1_OSC_ENA=0, FLL1_ENA=1
{0x204 , 0x0019}, //     Write{0x34      //AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=FLL1/BCLK1, AIF1CLK_INV=0, AIF1CLK_DIV=AIF1CLK, AIF1CLK_ENA=1
	//Unmutes
{0x610 , 0x03C0}, // Unmute DAC 1 (Left)
{0x611 , 0x03C0}, // Unmute DAC 1 (Right)
{0x612 , 0x03C0}, // Unmute DAC 1 (Right)
{0x613 , 0x03C0}, // Unmute DAC 2 (Right) path
/////////////////////////////////add for Keytone
//{0x420 , 0x0000}, //
/////////////////////////////////
{0x520 , 0x0200}, // Unmute the AIF2 DAC path
{0x540 , 0x0098}, // close AIF2 DRC
{0x580 , 0x6318}, // default close EQ
{0x521 , 0x0000}, // default close 3D

//Audio Interface Input/Output (I/O) Configuration
{0x702 , 0xC100}, //     Write{0x34      *
{0x703 , 0xC100}, //     Write{0x34      *
{0x704 , 0xC100}, //     Write{0x34      *
{0x706 , 0x4100}, //     Write{0x34      *

//Analogue Configuration
{0x39 , 0x01E4}, //     Write{0x34      //Enable VMID soft start (fast), Start-up Bias Current Enabled
//{0x01 , 0x0013}, //     Write{0x34      //Enable bias generator, Enable VMID, Enable Microphone Bias 1
{0x01 , 0x0033}, //     Write{0x34      //Enable bias generator, Enable VMID, Enable Microphone Bias 1
 {0xffff , 50}, // //INSERT_DELAY_MS [50]
//Analogue Input Configuration
{0x02 , 0x6360}, //     Write{0x34      //Enable IN1L Input PGA, Enable Left Input Mixer (MIXINL)
#ifdef DEBUG_AUDIO_PARAMETERS
{0x18 , 0x010b}, //     Write{0x34      //Unmute IN1L Input PGA  main mic
{0x1A , 0x018b}, //     Write{0x34      //Unmute IN1L Input PGA  second mic
{0x1B , 0x018b}, //     Write{0x34      //Unmute IN1L Input PGA  hp mic
#else
{0x18 , 0x010b}, //     Write{0x34      //Unmute IN1L Input PGA  main mic
{0x1A , 0x018b}, //     Write{0x34      //Unmute IN1L Input PGA  second mic
{0x1B , 0x0108}, //     Write{0x34      //Unmute IN1L Input PGA  hp mic
#endif
{0x28 , 0x003C}, //     Write{0x34      //Connect IN1LN to IN1L PGA, Connect IN1LP to IN1L PGA
{0x29 , 0x0030}, //     Write{0x34      //Unmute IN1L PGA output to Left Input Mixer (MIXINL) Path
{0x2A , 0x0180}, //     Write{0x34      //Unmute IN1L PGA output to Left Input Mixer (MIXINL) Path

//Path Configuration
{0x04 , 0x3003}, //     Write{0x34      //Enable ADC (Left), Enable AIF2 ADC (Left) Path
	//{0x05 , 0x2002}, //     Write{0x34      //Enable DAC1 (Left), Enable AIF2 DAC (Left) Path
	{0x05 , 0x2202}, //     Write{0x34      //Enable DAC1 (Left), Enable AIF2 DAC (Left) Path
	//{0x601 , 0x0004}, //     Write{0x34      //Enable the AIF2 (Left) to DAC 1 (Left) mixer path
	{0x601 , 0x0005}, //     Write{0x34      //Enable the AIF2 (Left) to DAC 1 (Left) mixer path
{0x603 , 0x018C}, //     Write{0x34      //Set the volume on the STL to DAC2 mixer paths to 0dB
{0x604 , 0x0010}, //     Write{0x34      //Enable the STL to DAC 2 (Left) mixer path
{0x605 , 0x0020}, //     Write{0x34      //Enable the STL to DAC 2 (Left) mixer path
//modify for main mic default in intel mono channel
//{0x604 , 0x0020}, //     Write{0x34      //Enable the STL to DAC 2 (Left) mixer path
//{0x605 , 0x0010}, //     Write{0x34      //Enable the STL to DAC 2 (Left) mixer path

//Analogue Output Configuration
{0x03 , 0x0100}, //     Write{0x34      //Enable SPKLVOL PGA, Enable SPKMIXL
{0x22 , 0x0000}, //     Write{0x34      //Left Speaker Mixer Volume = 0dB
{0x23 , 0x0003}, //     Write{0x34      //Speaker output mode = Class D, Right Speaker Mixer Volume = Mute
{0x25 , 0x0176}, //     Write{0x34      //
{0x36 , 0x0002}, //     Write{0x34      //Unmute DAC1 (Left) to Left Speaker Mixer (SPKMIXL) path
{0x01 , 0x1033}, //     Write{0x34      //Enable bias generator, Enable VMID, Enable Microphone Bias 1, Enable SPKOUTL
	//{0x26 , 0x017c}, //     Write{0x34      * Speaker Volume Left(26H): 0179  SPKOUT_VU=1, SPKOUTL_ZC=0, SPKOUTL_MUTE_N=1, SPKOUTL_VOL=11_1001
//Unmutes
{0x610 , 0x00C0}, //     Write{0x34      //Unmute DAC 1 (Left)
{0x612 , 0x00C0}, //     Write{0x34      //Unmute DAC 2 (Left) path
{0x613 , 0x00C0}, //     Write{0x34      //Unmute DAC 2 (Left) path
/////////////////////////////////add for Keytone
{0x420 , 0x0000}, //
/////////////////////////////////
{0x520 , 0x0000}, //     Write{0x34      //Unmute the AIF2 DAC path

	{0x26 , 0x017b}, //     Write{0x34      * Speaker Volume Left(26H): 0179  SPKOUT_VU=1, SPKOUTL_ZC=0, SPKOUTL_MUTE_N=1, SPKOUTL_VOL=11_1001
#if 1
//for record in call
{0x700 , 0xA101}, //Write{0x34      * GPIO 1(700H):            A101  GP1_DIR=1, GP1_PU=0, GP1_PD=1, GP1_POL=0, GP1_OP_CFG=0, GP1_DB=1, GP1_LVL=0, GP1_FN=0_0001
 {0x04 , 0x3303}, //Write{0x34      * Power Management (4)(04H): 3303  AIF2ADCL_ENA=1, AIF2ADCR_ENA=1, AIF1ADC2L_ENA=0, AIF1ADC2R_ENA=0, AIF1ADC1L_ENA=1, AIF1ADC1R_ENA=1, DMIC2L_ENA=0, DMIC2R_ENA=0, DMIC1L_ENA=0, DMIC1R_ENA=0, ADCL_ENA=1, ADCR_ENA=1
{0x606 , 0x0003}, //Write{0x34      * AIF1 ADC1 Left Mixer Routing(606H): 0001  ADC1L_TO_AIF1ADC1L=0, AIF2DACL_TO_AIF1ADC1L=1
{0x607 , 0x0003}, //Write{0x34      * AIF1 ADC1 Right Mixer Routing(607H): 0001  ADC1R_TO_AIF1ADC1R=0, AIF2DACR_TO_AIF1ADC1R=1
/*
{0x220 , 0x0000}, //Write{0x34      * FLL1 Control (1)(220H):  0000  FLL1_FRACN_ENA=0, FLL1_OSC_ENA=0, FLL1_ENA=0
{0x221 , 0x0700}, //Write{0x34      * FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=00_0111, FLL1_CTRL_RATE=000, FLL1_FRATIO=000
{0x222 , 0x86C2}, //Write{0x34      * FLL1 Control (3)(222H):  86C2  FLL1_K=1000_0110_1100_0010
{0x223 , 0x00E0}, //Write{0x34      * FLL1 Control (4)(223H):  00E0  FLL1_N=00_0000_0111, FLL1_GAIN=0000
{0x224 , 0x0C88}, //Write{0x34      * FLL1 Control (5)(224H):  0C88  FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_CLK_REF_DIV=01, FLL1_CLK_REF_SRC=00
{0x220 , 0x0005}, //Write{0x34      * FLL1 Control (1)(220H):  0000  FLL1_FRACN_ENA=0, FLL1_OSC_ENA=0, FLL1_ENA=0
{0x210 , 0x0073}, //Write{0x34      * AIF1 Rate(210H):         0073  AIF1_SR=0111, AIF1CLK_RATE=0011
*/
{0x300 , 0x0010}, //Write{0x34      * AIF1 Control (1)(300H):  4010  AIF1ADCL_SRC=0, AIF1ADCR_SRC=1, AIF1ADC_TDM=0, AIF1_BCLK_INV=0, AIF1_LRCLK_INV=0, AIF1_WL=00, AIF1_FMT=10
/*
{0x302 , 0x0000}, //Write{0x34      * AIF1 Master/Slave(302H): 0000  AIF1_TRI=0, AIF1_MSTR=0, AIF1_CLK_FRC=0, AIF1_LRCLK_FRC=0
{0x208 , 0x000F}, //Write{0x34      * Clocking (1)(208H):      000F  TOCLK_ENA=0, DSP_FS1CLK_ENA=1, DSP_FS2CLK_ENA=1, DSP_FSINTCLK_ENA=1, SYSCLK_SRC=1
{0x200 , 0x0011}, //Write{0x34      * AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=10, AIF1CLK_INV=0, AIF1CLK_DIV=0, AIF1CLK_ENA=1
{0x208 , 0x000E}, //Write{0x34      * Clocking (1)(208H):      000E  TOCLK_ENA=0, DSP_FS1CLK_ENA=1, DSP_FS2CLK_ENA=1, DSP_FSINTCLK_ENA=1, SYSCLK_SRC=0
*/
#endif
};

static struct incall_reg incall_rcv_reg[] = {
//{0x0000 , 0x0000}, //     Write{0x34      *
//Clocking
{0x620 , 0x0000}, //     Write{0x34      //Select Low Power ADC/DMIC Oversample Rate,
                                                      //Select Low Power DAC Oversample Rate (Default)
/////////////////////////////////add for Keytone
{0x210 , 0x0073}, // AIF1 Sample Rate = 44.1 kHz, AIF1CLK/Fs ratio = 256
{0x300 , 0x4010}, // AIF1 Word Length = 16-bits, AIF1 Format = I2S
{0x302 , 0x0000}, // AIF1 Slave Mode (Default Register Value)
///////////////////////////////// 
{0x211 , 0x0033}, //     Write{0x34      //AIF2 Sample Rate = 8 kHz, AIF2CLK/Fs ratio = 256
{0x310 , 0x4010}, //     Write{0x34      //AIF2 Word Length = 16-bits, AIF2 Format = DSP
{0x311 , 0x0000}, //     Write{0x34      //Enable AIF2 DSP Mono Mode
#ifdef CALL_PATH_IN_BYPASS_MODE
{0x312 , 0x0000}, //     Write{0x34      //AIF2 Slave Mode (Default Register Value)
#else
{0x312 , 0x4000}, //     Write{0x34      //AIF2 Slave Mode (Default Register Value)
#endif

{0x313 , 0x0070}, //     Write{0x34      //16 BCLK per mono
{0x314 , 0x0020}, //     Write{0x34      //16 BCLK per mono
	
/////////////////////////////////modify and add for Keytone
{0x208 , 0x000E}, // Enable the DSP processing clock for AIF2, 
                                                      //Enable the core clock,
                                                      //Set the core clock source to AIF2CLK
{0x200 , 0x0001}, //

{0x204 , 0x0001}, //     Write{0x34      //Enable AIF2 Clock, AIF2 Clock Source = MCLK2 pin

/////////////////////////////////add for Keytone
{0x220 , 0x0000}, // FLL1 Control (1)(220H):  0000  FLL1_OSC_ENA=0, FLL1_ENA=0
{0x221 , 0x0700}, // FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=8, FLL1_FRATIO=1
{0x222 , 0x86C2}, // FLL1 Control (3)(222H):  3126  FLL1_K=0.19199
{0x223 , 0x00E0}, // FLL1 Control (4)(223H):  0100  FLL1_N=8
{0x224 , 0x0C88}, // FLL1 Control (5)(224H):  0C80  FLL1_BYP=FLL1, FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_REFCLK_DIV=MCLK / 1, FLL1_REFCLK_SRC=MCLK1
{0x220 , 0x0005}, // FLL1 Control (1)(220H):  0001  FLL1_OSC_ENA=0, FLL1_ENA=1
{0xffff,5},
{0x200 , 0x0011}, // AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=FLL1/BCLK1, AIF1CLK_INV=0, AIF1CLK_DIV=AIF1CLK, AIF1CLK_ENA=1
/////////////////////////////////
//set FLL
{0x240 , 0x0000}, //     Write{0x34      //FLL1 Control (1)(220H):  0000  FLL1_OSC_ENA=0, FLL1_ENA=0
{0x241 , 0x1700}, //     Write{0x34      //FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=8, FLL1_FRATIO=1
{0x242 , 0x3126}, //     Write{0x34      //FLL1 Control (3)(222H):  3126  FLL1_K=0.19199
{0x243 , 0x0100}, //     Write{0x34      //FLL1 Control (4)(223H):  0100  FLL1_N=8
{0x244 , 0x0C88}, //     Write{0x34      //FLL1 Control (5)(224H):  0C80  FLL1_BYP=FLL1, FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_REFCLK_DIV=MCLK / 1, FLL1_REFCLK_SRC=MCLK1
{0x240 , 0x0005}, //     Write{0x34      //FLL1 Control (1)(220H):  0001  FLL1_OSC_ENA=0, FLL1_ENA=1
{0x204 , 0x0019}, //     Write{0x34      //AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=FLL1/BCLK1, AIF1CLK_INV=0, AIF1CLK_DIV=AIF1CLK, AIF1CLK_ENA=1

//Unmutes
{0x610 , 0x03C0}, // Unmute DAC 1 (Left)
{0x611 , 0x03C0}, // Unmute DAC 1 (Right)
{0x612 , 0x03C0}, // Unmute DAC 1 (Right)
{0x613 , 0x03C0}, // Unmute DAC 2 (Right) path
/////////////////////////////////add for Keytone
//{0x420 , 0x0000}, //
/////////////////////////////////
{0x520 , 0x0200}, // Unmute the AIF2 DAC path
{0x540 , 0x0098}, // close AIF2 DRC
{0x580 , 0x6318}, // default close EQ
{0x521 , 0x0000}, // default close 3D
//Audio Interface Input/Output (I/O) Configuration
{0x702 , 0xC100}, //     Write{0x34      *
{0x703 , 0xC100}, //     Write{0x34      *
{0x704 , 0xC100}, //     Write{0x34      *
{0x706 , 0x4100}, //     Write{0x34      *

//Analogue Configuration
 {0x39 , 0x01E4}, //     Write{0x34      //Enable VMID soft start (fast), Start-up Bias Current Enabled
 //{0x01 , 0x0013}, //     Write{0x34      //Enable bias generator, Enable VMID, Enable Microphone Bias 1
 {0x01 , 0x0033}, //     Write{0x34      //Enable bias generator, Enable VMID, Enable Microphone Bias 1
{0xffff,50},//delay 50ms
//Analogue Input Configuration
 {0x02 , 0x6360}, //     Write{0x34      //Enable IN1L Input PGA, Enable Left Input Mixer (MIXINL)
#ifdef DEBUG_AUDIO_PARAMETERS
{0x18 , 0x010b}, //     Write{0x34      //Unmute IN1L Input PGA  main mic
{0x1A , 0x018b}, //     Write{0x34      //Unmute IN1L Input PGA  second mic
{0x1B , 0x018b}, //     Write{0x34      //Unmute IN1L Input PGA  hp mic
#else
{0x18 , 0x010b}, //     Write{0x34      //Unmute IN1L Input PGA  main mic
{0x1A , 0x018b}, //     Write{0x34      //Unmute IN2R Input PGA  hp mic
 {0x1B , 0x0108}, //     Write{0x34      //Unmute IN1R Input PGA  second mic
 #endif
 {0x28 , 0x003C}, //     Write{0x34      //Connect IN1LN to IN1L PGA, Connect IN1LP to IN1L PGA
 {0x29 , 0x0030}, //     Write{0x34      //Unmute IN1L PGA output to Left Input Mixer (MIXINL) Path
 {0x2A , 0x0180}, //     Write{0x34      //Unmute IN1L PGA output to Left Input Mixer (MIXINL) Path

//Analogue Output Configuration
 {0x03 , 0x00A0}, //     Write{0x34      //Enable Left Output Mixer (MIXOUTL), Enable Left Output Mixer Volume Control
 {0x1F , 0x0000}, //     Write{0x34      //Unmute HPOUT2 (Earpiece)
 {0x2D , 0x0001}, //     Write{0x34      //Unmute DAC1 (Left) to Left Output Mixer (MIXOUTL) path
 {0x33 , 0x0010}, //     Write{0x34      //Unmute Left Output Mixer to HPOUT2 (Earpiece) path
 {0x38 , 0x0040}, //     Write{0x34      //Enable HPOUT2 Mixer and Input Stage
 
 {0x01 , 0x0833}, //     Write{0x34      //Enable bias generator, Enable VMID, Enable Microphone Bias 1, Enable HPOUT2 (Earpiece)
 		//{0x20,0x01f5},  // receiver volume
//Path Configuration
 {0x04 , 0x3003}, //     Write{0x34      //Enable ADC (Left), Enable AIF2 ADC (Left) Path
 		//{0x05 , 0x2002}, //     Write{0x34      //Enable DAC1 (Left), Enable AIF2 DAC (Left) Path
 		{0x05 , 0x2202}, //     Write{0x34      //Enable DAC1 (Left), Enable AIF2 DAC (Left) Path
		//{0x601 , 0x0004}, //     Write{0x34      //Enable the AIF2 (Left) to DAC 1 (Left) mixer path
		{0x601 , 0x0005}, //     Write{0x34      //Enable the AIF2 (Left) to DAC 1 (Left) mixer path
{0x603 , 0x018C}, //     Write{0x34      //Set the volume on the STL to DAC2 mixer paths to 0dB
{0x604 , 0x0010}, //     Write{0x34      //Enable the STL to DAC 2 (Left) mixer path
{0x605 , 0x0020}, //     Write{0x34      //Enable the STL to DAC 2 (Left) mixer path
//modify for main mic default in intel mono channel
//{0x604 , 0x0020}, //     Write{0x34      //Enable the STL to DAC 2 (Left) mixer path
//{0x605 , 0x0010}, //     Write{0x34      //Enable the STL to DAC 2 (Left) mixer path
  
//Unmutes
{0x610 , 0x00C0}, //     Write{0x34      //Unmute DAC 1 (Left)
{0x612 , 0x00C0}, //     Write{0x34      //Unmute DAC 2 (Left) path
{0x613 , 0x00C0}, //     Write{0x34      //Unmute DAC 2 (Left) path
/////////////////////////////////add for Keytone
{0x420 , 0x0000}, //
/////////////////////////////////
{0x520 , 0x0000}, //     Write{0x34      //Unmute the AIF2 DAC path

 {0x20,0x01f8},  // receiver volume

#if 1
//for record in call
{0x700 , 0xA101}, //Write{0x34      * GPIO 1(700H):            A101  GP1_DIR=1, GP1_PU=0, GP1_PD=1, GP1_POL=0, GP1_OP_CFG=0, GP1_DB=1, GP1_LVL=0, GP1_FN=0_0001
{0x04 , 0x3303}, //Write{0x34      * Power Management (4)(04H): 3303  AIF2ADCL_ENA=1, AIF2ADCR_ENA=1, AIF1ADC2L_ENA=0, AIF1ADC2R_ENA=0, AIF1ADC1L_ENA=1, AIF1ADC1R_ENA=1, DMIC2L_ENA=0, DMIC2R_ENA=0, DMIC1L_ENA=0, DMIC1R_ENA=0, ADCL_ENA=1, ADCR_ENA=1
{0x606 , 0x0003}, //Write{0x34      * AIF1 ADC1 Left Mixer Routing(606H): 0001  ADC1L_TO_AIF1ADC1L=0, AIF2DACL_TO_AIF1ADC1L=1
{0x607 , 0x0003}, //Write{0x34      * AIF1 ADC1 Right Mixer Routing(607H): 0001  ADC1R_TO_AIF1ADC1R=0, AIF2DACR_TO_AIF1ADC1R=1
/*
{0x220 , 0x0000}, //Write{0x34      * FLL1 Control (1)(220H):  0000  FLL1_FRACN_ENA=0, FLL1_OSC_ENA=0, FLL1_ENA=0
{0x221 , 0x0700}, //Write{0x34      * FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=00_0111, FLL1_CTRL_RATE=000, FLL1_FRATIO=000
{0x222 , 0x86C2}, //Write{0x34      * FLL1 Control (3)(222H):  86C2  FLL1_K=1000_0110_1100_0010
{0x223 , 0x00E0}, //Write{0x34      * FLL1 Control (4)(223H):  00E0  FLL1_N=00_0000_0111, FLL1_GAIN=0000
{0x224 , 0x0C88}, //Write{0x34      * FLL1 Control (5)(224H):  0C88  FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_CLK_REF_DIV=01, FLL1_CLK_REF_SRC=00
{0x220 , 0x0005}, //Write{0x34      * FLL1 Control (1)(220H):  0000  FLL1_FRACN_ENA=0, FLL1_OSC_ENA=0, FLL1_ENA=0
{0x210 , 0x0073}, //Write{0x34      * AIF1 Rate(210H):         0073  AIF1_SR=0111, AIF1CLK_RATE=0011
*/
{0x300 , 0x0010}, //Write{0x34      * AIF1 Control (1)(300H):  4010  AIF1ADCL_SRC=0, AIF1ADCR_SRC=1, AIF1ADC_TDM=0, AIF1_BCLK_INV=0, AIF1_LRCLK_INV=0, AIF1_WL=00, AIF1_FMT=10
/*
{0x302 , 0x0000}, //Write{0x34      * AIF1 Master/Slave(302H): 0000  AIF1_TRI=0, AIF1_MSTR=0, AIF1_CLK_FRC=0, AIF1_LRCLK_FRC=0
{0x208 , 0x000F}, //Write{0x34      * Clocking (1)(208H):      000F  TOCLK_ENA=0, DSP_FS1CLK_ENA=1, DSP_FS2CLK_ENA=1, DSP_FSINTCLK_ENA=1, SYSCLK_SRC=1
{0x200 , 0x0011}, //Write{0x34      * AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=10, AIF1CLK_INV=0, AIF1CLK_DIV=0, AIF1CLK_ENA=1
{0x208 , 0x000E}, //Write{0x34      * Clocking (1)(208H):      000E  TOCLK_ENA=0, DSP_FS1CLK_ENA=1, DSP_FS2CLK_ENA=1, DSP_FSINTCLK_ENA=1, SYSCLK_SRC=0
*/
#endif
};

static struct incall_reg incall_hp_reg[] = {
//----- ------ -------------------- ------- --------- ------------------------------
// REG   DATA         ACCESS        READ OR  DEVICE
//INDEX  VALUE         TYPE          WRITE   ADDRESS  COMMENT (for information only)
//----- ------ -------------------- ------- --------- ------------------------------
//Clocking
{0x620 , 0x0000}, // Select Low Power ADC/DMIC Oversample Rate,
                                                      //Select Low Power DAC Oversample Rate (Default)
/////////////////////////////////add for Keytone
{0x210 , 0x0073}, // AIF1 Sample Rate = 44.1 kHz, AIF1CLK/Fs ratio = 256
{0x300 , 0x4010}, // AIF1 Word Length = 16-bits, AIF1 Format = I2S
{0x302 , 0x0000}, // AIF1 Slave Mode (Default Register Value)
/////////////////////////////////
{0x211 , 0x0033}, // AIF2 Sample Rate = 8 kHz, AIF2CLK/Fs ratio = 256
{0x310 , 0xC010}, // AIF2 Word Length = 16-bits, AIF2 Format = DSP
{0x311 , 0x0000}, // Enable AIF2 DSP Mono Mode
#ifdef CALL_PATH_IN_BYPASS_MODE
{0x312 , 0x0000}, //     Write{0x34      //AIF2 Slave Mode (Default Register Value)
#else
{0x312 , 0x4000}, //     Write{0x34      //AIF2 Slave Mode (Default Register Value)
#endif
//BCLK=32LRCK
{0x313 , 0x0070}, // AIF2 BCLK(313H):         0070  AIF2_BCLK_DIV=0111
{0x314 , 0x0020}, // AIF2ADC LRCLK(314H):     0020  AIF2ADC_LRCLK_DIR=0, AIF2ADC_RATE=000_0010_0000
/////////////////////////////////modify and add for Keytone
{0x208 , 0x000E}, // Enable the DSP processing clock for AIF2, 
                                                      //Enable the core clock,
                                                      //Set the core clock source to AIF2CLK
{0x200 , 0x0001}, //
/////////////////////////////////
{0x204 , 0x0001}, // Enable AIF2 Clock, AIF2 Clock Source = MCLK2 pin

/////////////////////////////////add for Keytone
{0x220 , 0x0000}, // FLL1 Control (1)(220H):  0000  FLL1_OSC_ENA=0, FLL1_ENA=0
{0x221 , 0x0700}, // FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=8, FLL1_FRATIO=1
{0x222 , 0x86C2}, // FLL1 Control (3)(222H):  3126  FLL1_K=0.19199
{0x223 , 0x00E0}, // FLL1 Control (4)(223H):  0100  FLL1_N=8
{0x224 , 0x0C88}, // FLL1 Control (5)(224H):  0C80  FLL1_BYP=FLL1, FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_REFCLK_DIV=MCLK / 1, FLL1_REFCLK_SRC=MCLK1
{0x220 , 0x0005}, // FLL1 Control (1)(220H):  0001  FLL1_OSC_ENA=0, FLL1_ENA=1
{0xffff,5},
{0x200 , 0x0011}, // AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=FLL1/BCLK1, AIF1CLK_INV=0, AIF1CLK_DIV=AIF1CLK, AIF1CLK_ENA=1
/////////////////////////////////

//FLL setting
{0x240 , 0x0000}, // FLL1 Control (1)(220H):  0000  FLL1_OSC_ENA=0, FLL1_ENA=0
{0x241 , 0x1700}, // FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=8, FLL1_FRATIO=1
{0x242 , 0x3126}, // FLL1 Control (3)(222H):  3126  FLL1_K=0.19199
{0x243 , 0x0100}, // FLL1 Control (4)(223H):  0100  FLL1_N=8
{0x244 , 0x0C88}, // FLL1 Control (5)(224H):  0C80  FLL1_BYP=FLL1, FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_REFCLK_DIV=MCLK / 1, FLL1_REFCLK_SRC=MCLK1
{0x240 , 0x0005}, // FLL1 Control (1)(220H):  0001  FLL1_OSC_ENA=0, FLL1_ENA=1
{0x204 , 0x0019}, // AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=FLL1/BCLK1, AIF1CLK_INV=0, AIF1CLK_DIV=AIF1CLK, AIF1CLK_ENA=1

//Unmutes
{0x610 , 0x03C0}, // Unmute DAC 1 (Left)
{0x611 , 0x03C0}, // Unmute DAC 1 (Right)
{0x612 , 0x03C0}, // Unmute DAC 1 (Right)
{0x613 , 0x03C0}, // Unmute DAC 2 (Right) path
/////////////////////////////////add for Keytone
//{0x420 , 0x0000}, //
/////////////////////////////////
{0x520 , 0x0200}, // Unmute the AIF2 DAC path
{0x540 , 0x0098}, // close AIF2 DRC
{0x580 , 0x6318}, // default close EQ
{0x521 , 0x0000}, // default close 3D
//Audio Interface Input/Output (I/O) Configuration
{0x702 , 0xC100}, //
{0x703 , 0xC100}, //
{0x704 , 0xC100}, //
{0x706 , 0x4100}, //
   
//Analogue Configuration
 {0x39 , 0x01EC}, // Enable VMID soft start (fast), Enable VMID Buffer, Start-up Bias Current Enabled
 {0xffff , 1},
 {0x01 , 0x0033}, // Enable bias generator, Enable VMID, Enable Microphone Bias 1
 {0xffff , 50}, // //INSERT_DELAY_MS [50]
   
//Analogue Input Configuration
 {0x02 , 0x6110}, // Enable IN1R Input PGA, Enable Right Input Mixer (MIXINR)
 {0x1A , 0x0109}, // Unmute IN1R Input PGA
 {0x28 , 0x0003}, // Connect IN1RN to IN1R PGA
 {0x2A , 0x0030}, // Unmute IN1R PGA output to Right Input Mixer (MIXINR) Path

//Path Configuration
 {0x04 , 0x1001}, // Enable ADC (Right), Enable AIF2 ADC (Right) Path
/////////////////////////////////modifiy for Keytone
{0x601 , 0x0005}, // Enable the AIF2 (Left) to DAC 1 (Left) mixer path
{0x602 , 0x0005}, // Enable the AIF2 (Right) to DAC 1 (Right) mixer path
/////////////////////////////////
{0x603 , 0x0180}, // Set the volume on the STR to DAC2 mixer paths to 0dB
{0x605 , 0x0020}, // Enable the STR to DAC 2 (Right) mixer path
  

//Headphone Enable
 {0x01 , 0x0333}, // Enable bias generator, Enable VMID, Enable Microphone Bias 1,
                                                      //Enable HPOUT1 (Left) and Enable HPOUT1 (Right) input stages
 {0x60 , 0x0022}, // Enable HPOUT1 (Left) and HPOUT1 (Right) intermediate stages
 {0x4C , 0x9F25}, // Enable Charge Pump
 {0xffff , 20}, //INSERT_DELAY_MS [15]
 {0x05 , 0x3303}, // Enable DAC1 (Left), Enable DAC1 (Right), 
                                                      //Enable AIF2 DAC (Left) Path, Enable AIF2 DAC (Right) Path
 {0x2D , 0x0001}, // Select DAC1 (Left) to Left Headphone Output PGA (HPOUT1LVOL) path
 {0x2E , 0x0001}, // Select DAC1 (Right) to Right Headphone Output PGA (HPOUT1RVOL) path
 {0x03 , 0x0030}, // Enable Left Output Mixer (MIXOUTL), Enable Right Output Mixer (MIXOUTR)
 {0xffff,1},
 {0x54 , 0x0033}, // Enable DC Servo and trigger start-up mode on left and right channels
   {0xffff , 258},//INSERT_DELAY_MS [250]
 {0x60 , 0x00EE}, // Enable HPOUT1 (Left) and HPOUT1 (Right) intermediate and output stages. Remove clamps
{0xffff,1},
//Unmutes
{0x610 , 0x00C0}, // Unmute DAC 1 (Left)
{0x611 , 0x00C0}, // Unmute DAC 1 (Right)
{0x613 , 0x00C0}, // Unmute DAC 2 (Right) path
/////////////////////////////////add for Keytone
{0x420 , 0x0000}, //
/////////////////////////////////
{0x520 , 0x0000}, // Unmute the AIF2 DAC path

//Volume
 {0x1C , 0x016a}, // Left Output Volume(1CH): 0179  HPOUT1_VU=1, HPOUT1L_ZC=0, HPOUT1L_MUTE_N=1, HPOUT1L_VOL=11_1001
 {0x1D , 0x016a}, // Right Output Volume(1DH): 0179  HPOUT1_VU=1, HPOUT1R_ZC=0, HPOUT1R_MUTE_N=1, HPOUT1R_VOL=11_1001
#if 1
//for record in call
{0x700 , 0xA101}, //Write{0x34      * GPIO 1(700H):            A101  GP1_DIR=1, GP1_PU=0, GP1_PD=1, GP1_POL=0, GP1_OP_CFG=0, GP1_DB=1, GP1_LVL=0, GP1_FN=0_0001
 {0x04 , 0x3303}, //Write{0x34      * Power Management (4)(04H): 3303  AIF2ADCL_ENA=1, AIF2ADCR_ENA=1, AIF1ADC2L_ENA=0, AIF1ADC2R_ENA=0, AIF1ADC1L_ENA=1, AIF1ADC1R_ENA=1, DMIC2L_ENA=0, DMIC2R_ENA=0, DMIC1L_ENA=0, DMIC1R_ENA=0, ADCL_ENA=1, ADCR_ENA=1
{0x606 , 0x0003}, //Write{0x34      * AIF1 ADC1 Left Mixer Routing(606H): 0001  ADC1L_TO_AIF1ADC1L=0, AIF2DACL_TO_AIF1ADC1L=1
{0x607 , 0x0003}, //Write{0x34      * AIF1 ADC1 Right Mixer Routing(607H): 0001  ADC1R_TO_AIF1ADC1R=0, AIF2DACR_TO_AIF1ADC1R=1
	
//{0x220 , 0x0000}, //Write{0x34      * FLL1 Control (1)(220H):  0000  FLL1_FRACN_ENA=0, FLL1_OSC_ENA=0, FLL1_ENA=0
//{0x221 , 0x0700}, //Write{0x34      * FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=00_0111, FLL1_CTRL_RATE=000, FLL1_FRATIO=000
//{0x222 , 0x86C2}, //Write{0x34      * FLL1 Control (3)(222H):  86C2  FLL1_K=1000_0110_1100_0010
//{0x223 , 0x00E0}, //Write{0x34      * FLL1 Control (4)(223H):  00E0  FLL1_N=00_0000_0111, FLL1_GAIN=0000
//{0x224 , 0x0C88}, //Write{0x34      * FLL1 Control (5)(224H):  0C88  FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_CLK_REF_DIV=01, FLL1_CLK_REF_SRC=00
//{0x220 , 0x0005}, //Write{0x34      * FLL1 Control (1)(220H):  0000  FLL1_FRACN_ENA=0, FLL1_OSC_ENA=0, FLL1_ENA=0
//{0x210 , 0x0073}, //Write{0x34      * AIF1 Rate(210H):         0073  AIF1_SR=0111, AIF1CLK_RATE=0011
{0x300 , 0xC010}, //Write{0x34      * AIF1 Control (1)(300H):  4010  AIF1ADCL_SRC=0, AIF1ADCR_SRC=1, AIF1ADC_TDM=0, AIF1_BCLK_INV=0, AIF1_LRCLK_INV=0, AIF1_WL=00, AIF1_FMT=10
//{0x302 , 0x0000}, //Write{0x34      * AIF1 Master/Slave(302H): 0000  AIF1_TRI=0, AIF1_MSTR=0, AIF1_CLK_FRC=0, AIF1_LRCLK_FRC=0
//{0x208 , 0x000F}, //Write{0x34      * Clocking (1)(208H):      000F  TOCLK_ENA=0, DSP_FS1CLK_ENA=1, DSP_FS2CLK_ENA=1, DSP_FSINTCLK_ENA=1, SYSCLK_SRC=1
//{0x200 , 0x0011}, //Write{0x34      * AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=10, AIF1CLK_INV=0, AIF1CLK_DIV=0, AIF1CLK_ENA=1
//{0x208 , 0x000E}, //Write{0x34      * Clocking (1)(208H):      000E  TOCLK_ENA=0, DSP_FS1CLK_ENA=1, DSP_FS2CLK_ENA=1, DSP_FSINTCLK_ENA=1, SYSCLK_SRC=0
#endif
};
//master:0x4000 slave:0x0000

static struct incall_reg normal_spk_reg[] = {
// {0x01 , 0x3003}, //     Write{0x34      * Power Management (1)(01H): 3003  SPKOUTR_ENA=1, SPKOUTL_ENA=1, HPOUT2_ENA=0, HPOUT1L_ENA=0, HPOUT1R_ENA=0, MICB2_ENA=0, MICB1_ENA=0, VMID_SEL=01, BIAS_ENA=1
 {0x01 , 0x3023}, //     Write{0x34      * Power Management (1)(01H): 3003  SPKOUTR_ENA=1, SPKOUTL_ENA=1, HPOUT2_ENA=0, HPOUT1L_ENA=0, HPOUT1R_ENA=0, MICB2_ENA=0, MICB1_ENA=0, VMID_SEL=01, BIAS_ENA=1
 {0x03 , 0x0300}, //     Write{0x34      * Power Management (3)(03H): 0300  LINEOUT1N_ENA=0, LINEOUT1P_ENA=0, LINEOUT2N_ENA=0, LINEOUT2P_ENA=0, SPKRVOL_ENA=1, SPKLVOL_ENA=1, MIXOUTLVOL_ENA=0, MIXOUTRVOL_ENA=0, MIXOUTL_ENA=0, MIXOUTR_ENA=0
 {0x05 , 0x0303}, //     Write{0x34      * Power Management (5)(05H): 3003  AIF2DACL_ENA=1, AIF2DACR_ENA=1, AIF1DAC2L_ENA=0, AIF1DAC2R_ENA=0, AIF1DAC1L_ENA=0, AIF1DAC1R_ENA=0, DAC2L_ENA=0, DAC2R_ENA=0, DAC1L_ENA=1, DAC1R_ENA=1
 {0x22 , 0x0000}, //     Write{0x34      * SPKMIXL Attenuation(22H): 0000  DAC2L_SPKMIXL_VOL=0, MIXINL_SPKMIXL_VOL=0, IN1LP_SPKMIXL_VOL=0, MIXOUTL_SPKMIXL_VOL=0, DAC1L_SPKMIXL_VOL=0, SPKMIXL_VOL=00
 {0x23 , 0x0000}, //     Write{0x34      * SPKMIXR Attenuation(23H): 0100  SPKOUT_CLASSAB=1, DAC2R_SPKMIXR_VOL=0, MIXINR_SPKMIXR_VOL=0, IN1RP_SPKMIXR_VOL=0, MIXOUTR_SPKMIXR_VOL=0, DAC1R_SPKMIXR_VOL=0, SPKMIXR_VOL=00
 {0x24 , 0x0018}, //     Write{0x34      *
 {0x25 , 0x015B}, //     Write{0x34      *
 {0x26 , 0x0179}, //     Write{0x34      * Speaker Volume Left(26H): 0179  SPKOUT_VU=1, SPKOUTL_ZC=0, SPKOUTL_MUTE_N=1, SPKOUTL_VOL=11_1001
 {0x27 , 0x0179}, //     Write{0x34      * Speaker Volume Right(27H): 0179  SPKOUT_VU=1, SPKOUTR_ZC=0, SPKOUTR_MUTE_N=1, SPKOUTR_VOL=11_1001
 {0x36 , 0x0003}, //     Write{0x34      * Speaker Mixer(36H):      0003  DAC2L_TO_SPKMIXL=0, DAC2R_TO_SPKMIXR=0, MIXINL_TO_SPKMIXL=0, MIXINR_TO_SPKMIXR=0, IN1LP_TO_SPKMIXL=0, IN1RP_TO_SPKMIXR=0, MIXOUTL_TO_SPKMIXL=0, MIXOUTR_TO_SPKMIXR=0, DAC1L_TO_SPKMIXL=1, DAC1R_TO_SPKMIXR=1
{0x200 , 0x0001}, //     Write{0x34      * AIF1 Clocking (1)(200H): 0001  AIF1CLK_SRC=00, AIF1CLK_INV=0, AIF1CLK_DIV=0, AIF1CLK_ENA=1
{0x208 , 0x000A}, //     Write{0x34      * Clocking (1)(208H):      000A  CLASSD_EDGE=00, TOCLK_ENA=0, DSP_FS1CLK_ENA=1, DSP_FS2CLK_ENA=0, DSP_FSINTCLK_ENA=1, SYSCLK_SRC=0
{0x210 , 0x0073}, //     Write{0x34      *
{0x420 , 0x0000}, //     Write{0x34      * AIF1 DAC1 Filters (1)(420H): 0000  AIF1DAC1_MUTE=0, AIF1DAC1_MONO=0, AIF1DAC1_MUTERATE=0, AIF1DAC1_UNMUTE_RAMP=0, AIF1DAC1_DEEMP=00
{0x601 , 0x0001}, //     Write{0x34      * DAC1 Left Mixer Routing(601H): 0001  ADC2_TO_DAC1L=0, ADC1_TO_DAC1L=0, AIF2DACL_TO_DAC1L=0, AIF1DAC2L_TO_DAC1L=0, AIF1DAC1L_TO_DAC1L=1
{0x602 , 0x0001}, //     Write{0x34      * DAC1 Right Mixer Routing(602H): 0001  ADC2_TO_DAC1R=0, ADC1_TO_DAC1R=0, AIF2DACR_TO_DAC1R=0, AIF1DAC2R_TO_DAC1R=0, AIF1DAC1R_TO_DAC1R=1
{0x610 , 0x01C0}, //     Write{0x34      * DAC1 Left Volume(610H):  01C0  DAC1L_MUTE=0, DAC1_VU=1, DAC1L_VOL=1100_0000
{0x611 , 0x01C0}, //     Write{0x34      * DAC1 Right Volume(611H): 01C0  DAC1R_MUTE=0, DAC1_VU=1, DAC1R_VOL=1100_0000

{0x220 , 0x0000}, //     Write{0x34      * FLL1 Control (1)(220H):  0000  FLL1_OSC_ENA=0, FLL1_ENA=0
{0x221 , 0x0700}, //     Write{0x34      * FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=8, FLL1_FRATIO=1
{0x222 , 0x86C2}, //     Write{0x34      * FLL1 Control (3)(222H):  3126  FLL1_K=0.19199
{0x223 , 0x00E0}, //     Write{0x34      * FLL1 Control (4)(223H):  0100  FLL1_N=8
{0x224 , 0x0C88}, //     Write{0x34      * FLL1 Control (5)(224H):  0C80  FLL1_BYP=FLL1, FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_REFCLK_DIV=MCLK / 1, FLL1_REFCLK_SRC=MCLK1
{0x220 , 0x0005}, //     Write{0x34      * FLL1 Control (1)(220H):  0001  FLL1_OSC_ENA=0, FLL1_ENA=1
{0x200 , 0x0011}, //     Write{0x34      * AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=FLL1/BCLK1, AIF1CLK_INV=0, AIF1CLK_DIV=AIF1CLK, AIF1CLK_ENA=1
};

static struct incall_reg normal_hp_reg[] = {
// {0x01 , 0x0303}, //     Write{0x34      * Power Management (1)(01H): 0303  SPKOUTR_ENA=0, SPKOUTL_ENA=0, HPOUT2_ENA=0, HPOUT1L_ENA=1, HPOUT1R_ENA=1, MICB2_ENA=0, MICB1_ENA=0, VMID_SEL=01, BIAS_ENA=1
 	{0x01 , 0x0323}, //     Write{0x34      * Power Management (1)(01H): 0303  SPKOUTR_ENA=0, SPKOUTL_ENA=0, HPOUT2_ENA=0, HPOUT1L_ENA=1, HPOUT1R_ENA=1, MICB2_ENA=0, MICB1_ENA=0, VMID_SEL=01, BIAS_ENA=1
 {0x05 , 0x0303}, //     Write{0x34      * Power Management (5)(05H): 0303  AIF2DACL_ENA=0, AIF2DACR_ENA=0, AIF1DAC2L_ENA=0, AIF1DAC2R_ENA=0, AIF1DAC1L_ENA=1, AIF1DAC1R_ENA=1, DAC2L_ENA=0, DAC2R_ENA=0, DAC1L_ENA=1, DAC1R_ENA=1
 {0x2D , 0x0100}, //     Write{0x34      * Output Mixer (1)(2DH):   0100  DAC1L_TO_HPOUT1L=1, MIXINR_TO_MIXOUTL=0, MIXINL_TO_MIXOUTL=0, IN2RN_TO_MIXOUTL=0, IN2LN_TO_MIXOUTL=0, IN1R_TO_MIXOUTL=0, IN1L_TO_MIXOUTL=0, IN2LP_TO_MIXOUTL=0, DAC1L_TO_MIXOUTL=0
 {0x2E , 0x0100}, //     Write{0x34      * Output Mixer (2)(2EH):   0100  DAC1R_TO_HPOUT1R=1, MIXINL_TO_MIXOUTR=0, MIXINR_TO_MIXOUTR=0, IN2LN_TO_MIXOUTR=0, IN2RN_TO_MIXOUTR=0, IN1L_TO_MIXOUTR=0, IN1R_TO_MIXOUTR=0, IN2RP_TO_MIXOUTR=0, DAC1R_TO_MIXOUTR=0
 {0x4C , 0x9F25}, //     Write{0x34      * Charge Pump (1)(4CH):    9F25  CP_ENA=1
 {0x60 , 0x00EE}, //     Write{0x34      * Analogue HP (1)(60H):    00EE  HPOUT1L_RMV_SHORT=1, HPOUT1L_OUTP=1, HPOUT1L_DLY=1, HPOUT1R_RMV_SHORT=1, HPOUT1R_OUTP=1, HPOUT1R_DLY=1
{0x200 , 0x0001}, //     Write{0x34      * AIF1 Clocking (1)(200H): 0001  AIF1CLK_SRC=00, AIF1CLK_INV=0, AIF1CLK_DIV=0, AIF1CLK_ENA=1
	{0x208 , 0x000A}, //     Write{0x34      * Clocking (1)(208H):      000A  CLASSD_EDGE=00, TOCLK_ENA=0, DSP_FS1CLK_ENA=1, DSP_FS2CLK_ENA=0, DSP_FSINTCLK_ENA=1, SYSCLK_SRC=0
{0x210 , 0x0073}, //     Write{0x34      *
	{0x300 , 0x4010}, //     Write{0x34      *
{0x420 , 0x0000}, //     Write{0x34      * AIF1 DAC1 Filters (1)(420H): 0000  AIF1DAC1_MUTE=0, AIF1DAC1_MONO=0, AIF1DAC1_MUTERATE=0, AIF1DAC1_UNMUTE_RAMP=0, AIF1DAC1_DEEMP=00
{0x601 , 0x0001}, //     Write{0x34      * DAC1 Left Mixer Routing(601H): 0001  ADC2_TO_DAC1L=0, ADC1_TO_DAC1L=0, AIF2DACL_TO_DAC1L=0, AIF1DAC2L_TO_DAC1L=0, AIF1DAC1L_TO_DAC1L=1
{0x602 , 0x0001}, //     Write{0x34      * DAC1 Right Mixer Routing(602H): 0001  ADC2_TO_DAC1R=0, ADC1_TO_DAC1R=0, AIF2DACR_TO_DAC1R=0, AIF1DAC2R_TO_DAC1R=0, AIF1DAC1R_TO_DAC1R=1
{0x610 , 0x01C0}, //     Write{0x34      * DAC1 Left Volume(610H):  01C0  DAC1L_MUTE=0, DAC1_VU=1, DAC1L_VOL=1100_0000
	{0x611 , 0x01C0}, //     Write{0x34      * DAC1 Right Volume(611H): 01C0  DAC1R_MUTE=0, DAC1_VU=1, DAC1R_VOL=1100_0000
	{0x1c,0x01ea},// hp volume
	{0x1d,0x01ea},// hp volume

{0x220 , 0x0000}, //     Write{0x34      * FLL1 Control (1)(220H):  0000  FLL1_OSC_ENA=0, FLL1_ENA=0
	{0x221 , 0x0700}, //     Write{0x34      * FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=8, FLL1_FRATIO=1
	{0x222 , 0x86C2}, //     Write{0x34      * FLL1 Control (3)(222H):  3126  FLL1_K=0.19199
	{0x223 , 0x00E0}, //     Write{0x34      * FLL1 Control (4)(223H):  0100  FLL1_N=8
	{0x224 , 0x0C88}, //     Write{0x34      * FLL1 Control (5)(224H):  0C80  FLL1_BYP=FLL1, FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_REFCLK_DIV=MCLK / 1, FLL1_REFCLK_SRC=MCLK1
	{0x220 , 0x0005}, //     Write{0x34      * FLL1 Control (1)(220H):  0001  FLL1_OSC_ENA=0, FLL1_ENA=1
	{0x200 , 0x0011}, //     Write{0x34      * AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=FLL1/BCLK1, AIF1CLK_INV=0, AIF1CLK_DIV=AIF1CLK, AIF1CLK_ENA=1
};

static struct incall_reg normal_spk_hp_reg[] = {
 //{0x01 , 0x3303}, //     Write{0x34      * Power Management (1)(01H): 0303  SPKOUTR_ENA=0, SPKOUTL_ENA=0, HPOUT2_ENA=0, HPOUT1L_ENA=1, HPOUT1R_ENA=1, MICB2_ENA=0, MICB1_ENA=0, VMID_SEL=01, BIAS_ENA=1
 {0x01 , 0x3323}, //     Write{0x34      * Power Management (1)(01H): 0303  SPKOUTR_ENA=0, SPKOUTL_ENA=0, HPOUT2_ENA=0, HPOUT1L_ENA=1, HPOUT1R_ENA=1, MICB2_ENA=0, MICB1_ENA=0, VMID_SEL=01, BIAS_ENA=1
 {0x03 , 0x0300}, //     Write{0x34      * Power Management (3)(03H): 0300  LINEOUT1N_ENA=0, LINEOUT1P_ENA=0, LINEOUT2N_ENA=0, LINEOUT2P_ENA=0, SPKRVOL_ENA=1, SPKLVOL_ENA=1, MIXOUTLVOL_ENA=0, MIXOUTRVOL_ENA=0, MIXOUTL_ENA=0, MIXOUTR_ENA=0
 {0x05 , 0x0303}, //     Write{0x34      * Power Management (5)(05H): 0303  AIF2DACL_ENA=0, AIF2DACR_ENA=0, AIF1DAC2L_ENA=0, AIF1DAC2R_ENA=0, AIF1DAC1L_ENA=1, AIF1DAC1R_ENA=1, DAC2L_ENA=0, DAC2R_ENA=0, DAC1L_ENA=1, DAC1R_ENA=1
//SPK
 {0x22 , 0x0000}, //     Write{0x34      * SPKMIXL Attenuation(22H): 0000  DAC2L_SPKMIXL_VOL=0, MIXINL_SPKMIXL_VOL=0, IN1LP_SPKMIXL_VOL=0, MIXOUTL_SPKMIXL_VOL=0, DAC1L_SPKMIXL_VOL=0, SPKMIXL_VOL=00
 {0x23 , 0x0000}, //     Write{0x34      * SPKMIXR Attenuation(23H): 0100  SPKOUT_CLASSAB=1, DAC2R_SPKMIXR_VOL=0, MIXINR_SPKMIXR_VOL=0, IN1RP_SPKMIXR_VOL=0, MIXOUTR_SPKMIXR_VOL=0, DAC1R_SPKMIXR_VOL=0, SPKMIXR_VOL=00
 {0x24 , 0x0018}, //     Write{0x34      *
 {0x25 , 0x015B}, //     Write{0x34      *
 {0x26 , 0x017d}, //     Write{0x34      * Speaker Volume Left(26H): 0179  SPKOUT_VU=1, SPKOUTL_ZC=0, SPKOUTL_MUTE_N=1, SPKOUTL_VOL=11_1001
 {0x27 , 0x0179}, //     Write{0x34      * Speaker Volume Right(27H): 0179  SPKOUT_VU=1, SPKOUTR_ZC=0, SPKOUTR_MUTE_N=1, SPKOUTR_VOL=11_1001
 {0x36 , 0x0003}, //     Write{0x34      * Speaker Mixer(36H):      0003  DAC2L_TO_SPKMIXL=0, DAC2R_TO_SPKMIXR=0, MIXINL_TO_SPKMIXL=0, MIXINR_TO_SPKMIXR=0, IN1LP_TO_SPKMIXL=0, IN1RP_TO_SPKMIXR=0, MIXOUTL_TO_SPKMIXL=0, MIXOUTR_TO_SPKMIXR=0, DAC1L_TO_SPKMIXL=1, DAC1R_TO_SPKMIXR=1
//HP
 {0x2D , 0x0100}, //     Write{0x34      * Output Mixer (1)(2DH):   0100  DAC1L_TO_HPOUT1L=1, MIXINR_TO_MIXOUTL=0, MIXINL_TO_MIXOUTL=0, IN2RN_TO_MIXOUTL=0, IN2LN_TO_MIXOUTL=0, IN1R_TO_MIXOUTL=0, IN1L_TO_MIXOUTL=0, IN2LP_TO_MIXOUTL=0, DAC1L_TO_MIXOUTL=0
 {0x2E , 0x0100}, //     Write{0x34      * Output Mixer (2)(2EH):   0100  DAC1R_TO_HPOUT1R=1, MIXINL_TO_MIXOUTR=0, MIXINR_TO_MIXOUTR=0, IN2LN_TO_MIXOUTR=0, IN2RN_TO_MIXOUTR=0, IN1L_TO_MIXOUTR=0, IN1R_TO_MIXOUTR=0, IN2RP_TO_MIXOUTR=0, DAC1R_TO_MIXOUTR=0
 {0x4C , 0x9F25}, //     Write{0x34      * Charge Pump (1)(4CH):    9F25  CP_ENA=1
 {0x60 , 0x00EE}, //     Write{0x34      * Analogue HP (1)(60H):    00EE  HPOUT1L_RMV_SHORT=1, HPOUT1L_OUTP=1, HPOUT1L_DLY=1, HPOUT1R_RMV_SHORT=1, HPOUT1R_OUTP=1, HPOUT1R_DLY=1

{0x200 , 0x0001}, //     Write{0x34      * AIF1 Clocking (1)(200H): 0001  AIF1CLK_SRC=00, AIF1CLK_INV=0, AIF1CLK_DIV=0, AIF1CLK_ENA=1
{0x208 , 0x000A}, //     Write{0x34      * Clocking (1)(208H):      000A  CLASSD_EDGE=00, TOCLK_ENA=0, DSP_FS1CLK_ENA=1, DSP_FS2CLK_ENA=0, DSP_FSINTCLK_ENA=1, SYSCLK_SRC=0
{0x210 , 0x0073}, //     Write{0x34      *
{0x300 , 0x4010}, //     Write{0x34      *
{0x302 , 0x0000}, //     Write{0x34      *
{0x420 , 0x0000}, //     Write{0x34      * AIF1 DAC1 Filters (1)(420H): 0000  AIF1DAC1_MUTE=0, AIF1DAC1_MONO=0, AIF1DAC1_MUTERATE=0, AIF1DAC1_UNMUTE_RAMP=0, AIF1DAC1_DEEMP=00
{0x601 , 0x0001}, //     Write{0x34      * DAC1 Left Mixer Routing(601H): 0001  ADC2_TO_DAC1L=0, ADC1_TO_DAC1L=0, AIF2DACL_TO_DAC1L=0, AIF1DAC2L_TO_DAC1L=0, AIF1DAC1L_TO_DAC1L=1
{0x602 , 0x0001}, //     Write{0x34      * DAC1 Right Mixer Routing(602H): 0001  ADC2_TO_DAC1R=0, ADC1_TO_DAC1R=0, AIF2DACR_TO_DAC1R=0, AIF1DAC2R_TO_DAC1R=0, AIF1DAC1R_TO_DAC1R=1
{0x610 , 0x01C0}, //     Write{0x34      * DAC1 Left Volume(610H):  01C0  DAC1L_MUTE=0, DAC1_VU=1, DAC1L_VOL=1100_0000
{0x611 , 0x01C0}, //     Write{0x34      * DAC1 Right Volume(611H): 01C0  DAC1R_MUTE=0, DAC1_VU=1, DAC1R_VOL=1100_0000

{0x220 , 0x0000}, //     Write{0x34      * FLL1 Control (1)(220H):  0000  FLL1_OSC_ENA=0, FLL1_ENA=0
{0x221 , 0x0700}, //     Write{0x34      * FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=8, FLL1_FRATIO=1
{0x222 , 0x86C2}, //     Write{0x34      * FLL1 Control (3)(222H):  3126  FLL1_K=0.19199
{0x223 , 0x00E0}, //     Write{0x34      * FLL1 Control (4)(223H):  0100  FLL1_N=8
{0x224 , 0x0C88}, //     Write{0x34      * FLL1 Control (5)(224H):  0C80  FLL1_BYP=FLL1, FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_REFCLK_DIV=MCLK / 1, FLL1_REFCLK_SRC=MCLK1
{0x220 , 0x0005}, //     Write{0x34      * FLL1 Control (1)(220H):  0001  FLL1_OSC_ENA=0, FLL1_ENA=1
{0x200 , 0x0011}, //     Write{0x34      * AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=FLL1/BCLK1, AIF1CLK_INV=0, AIF1CLK_DIV=AIF1CLK, AIF1CLK_ENA=1
};

static struct incall_reg incall_bt_reg[] = {
	//{0x00 , 0x0000}, //     Write{0x34      // Software reset
	// Audio Interface Input/Output (I/O) Configuration
	{0x702 , 0xC100}, //     Write{0x34      //
	{0x703 , 0xC100}, //     Write{0x34      //
	{0x704 , 0xC100}, //     Write{0x34      //
	{0x706 , 0x4100}, //     Write{0x34      //
	// Analogue Configuration
	//{0x01 , 0x0013}, //     Write{0x34      // Enable bias generator, Enable VMID, Enable Microphone Bias 1
	{0x01 , 0x0333}, //     Write{0x34      // Enable bias generator, Enable VMID, Enable Microphone Bias 1
	{0x04 , 0x3003}, //     Write{0x34      // Enable ADC (Left), Enable AIF2 ADC (Left) Path
	{0x05 , 0x2002}, //     Write{0x34      // Enable DAC1 (Left), Enable AIF2 DAC (Left) Path
	{0x620 , 0x0000}, //     Write{0x34      // Select Low Power ADC/DMIC Oversample Rate,
	{0x211 , 0x0033}, //     Write{0x34      // AIF2 Sample Rate = 8 kHz, AIF2CLK/Fs ratio = 256
	{0x310 , 0x4118}, //     Write{0x34      // AIF2 Word Length = 16-bits, AIF2 Format = DSP
	{0x312 , 0x4000}, //     Write{0x34      // AIF2 Slave Mode (Default Register Value)
	{0x313 , 0x0070}, //     Write{0x34      //
	{0x314 , 0x0020}, //     Write{0x34      //
	{0x208 , 0x0007}, //     Write{0x34      // Enable the DSP processing clock for AIF2, 
	{0x204 , 0x0001}, //     Write{0x34      // Enable AIF2 Clock, AIF2 Clock Source = MCLK2 pin
	{0x240 , 0x0000}, //     Write{0x34      // FLL1 Control (1)(220H):  0000  FLL1_OSC_ENA=0, FLL1_ENA=0
	{0x241 , 0x1700}, //     Write{0x34      // FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=8, FLL1_FRATIO=1
	{0x242 , 0x3126}, //     Write{0x34      // FLL1 Control (3)(222H):  3126  FLL1_K=0.19199
	{0x243 , 0x0100}, //     Write{0x34      // FLL1 Control (4)(223H):  0100  FLL1_N=8
	{0x244 , 0x0C88}, //     Write{0x34      // FLL1 Control (5)(224H):  0C80  FLL1_BYP=FLL1, FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_REFCLK_DIV=MCLK / 1, FLL1_REFCLK_SRC=MCLK1
	{0x240 , 0x0005}, //     Write{0x34      // FLL1 Control (1)(220H):  0001  FLL1_OSC_ENA=0, FLL1_ENA=1
	{0x204 , 0x0019}, //     Write{0x34      // AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=FLL1/BCLK1, AIF1CLK_INV=0, AIF1CLK_DIV=AIF1CLK, AIF1CLK_ENA=1
	/////AIF3
	{0x707 , 0xA100}, //     Write{0x34      // GPIO 8(707H):            A100  GP8_DIR=1, GP8_PU=0, GP8_PD=1, GP8_POL=0, GP8_OP_CFG=0, GP8_DB=1, GP8_LVL=0, GP8_FN=0_0000
	{0x708 , 0x4100}, //     Write{0x34      // GPIO 9(708H):            A100  GP9_DIR=1, GP9_PU=0, GP9_PD=1, GP9_POL=0, GP9_OP_CFG=0, GP9_DB=1, GP9_LVL=0, GP9_FN=0_0000
	{0x709 , 0xA100}, //     Write{0x34      // GPIO 10(709H):           A100  GP10_DIR=1, GP10_PU=0, GP10_PD=1, GP10_POL=0, GP10_OP_CFG=0, GP10_DB=1, GP10_LVL=0, GP10_FN=0_0000
	{0x70A , 0xA100}, //     Write{0x34      // GPIO 11(70AH):           A100  GP11_DIR=1, GP11_PU=0, GP11_PD=1, GP11_POL=0, GP11_OP_CFG=0, GP11_DB=1, GP11_LVL=0, GP11_FN=0_0000
	{0x06 , 0x0014}, //     Write{0x34      // Power Management (6)(06H): 0014  AIF3ADC_SRC=None, AIF2DAC_SRC=Left and Right inputs from AIF2, AIF3_TRI=0, AIF3_ADCDAT_SRC=GPIO5/DACDAT2, AIF2_ADCDAT_SRC=GPIO8/DACDAT3, AIF2_DACDAT_SRC=DACDAT2, AIF1_DACDAT_SRC=DACDAT1
//for record in call
{0x700 , 0xA101}, //Write{0x34      * GPIO 1(700H):            A101  GP1_DIR=1, GP1_PU=0, GP1_PD=1, GP1_POL=0, GP1_OP_CFG=0, GP1_DB=1, GP1_LVL=0, GP1_FN=0_0001
 {0x04 , 0x3303}, //Write{0x34      * Power Management (4)(04H): 3303  AIF2ADCL_ENA=1, AIF2ADCR_ENA=1, AIF1ADC2L_ENA=0, AIF1ADC2R_ENA=0, AIF1ADC1L_ENA=1, AIF1ADC1R_ENA=1, DMIC2L_ENA=0, DMIC2R_ENA=0, DMIC1L_ENA=0, DMIC1R_ENA=0, ADCL_ENA=1, ADCR_ENA=1
 {0x05 , 0x3003}, //Write{0x34      * Power Management (4)(04H): 3303  AIF2ADCL_ENA=1, AIF2ADCR_ENA=1, AIF1ADC2L_ENA=0, AIF1ADC2R_ENA=0, AIF1ADC1L_ENA=1, AIF1ADC1R_ENA=1, DMIC2L_ENA=0, DMIC2R_ENA=0, DMIC1L_ENA=0, DMIC1R_ENA=0, ADCL_ENA=1, ADCR_ENA=1
{0x606 , 0x0001}, //Write{0x34      * AIF1 ADC1 Left Mixer Routing(606H): 0001  ADC1L_TO_AIF1ADC1L=0, AIF2DACL_TO_AIF1ADC1L=1
{0x607 , 0x0001}, //Write{0x34      * AIF1 ADC1 Right Mixer Routing(607H): 0001  ADC1R_TO_AIF1ADC1R=0, AIF2DACR_TO_AIF1ADC1R=1
{0x520 , 0x0080}, //Write{0x34      * Unmute the AIF2 DAC path
{0x220 , 0x0000}, //Write{0x34      * FLL1 Control (1)(220H):  0000  FLL1_FRACN_ENA=0, FLL1_OSC_ENA=0, FLL1_ENA=0
{0x221 , 0x0700}, //Write{0x34      * FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=00_0111, FLL1_CTRL_RATE=000, FLL1_FRATIO=000
{0x222 , 0x86C2}, //Write{0x34      * FLL1 Control (3)(222H):  86C2  FLL1_K=1000_0110_1100_0010
{0x223 , 0x00E0}, //Write{0x34      * FLL1 Control (4)(223H):  00E0  FLL1_N=00_0000_0111, FLL1_GAIN=0000
{0x224 , 0x0C88}, //Write{0x34      * FLL1 Control (5)(224H):  0C88  FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_CLK_REF_DIV=01, FLL1_CLK_REF_SRC=00
{0x220 , 0x0005}, //Write{0x34      * FLL1 Control (1)(220H):  0000  FLL1_FRACN_ENA=0, FLL1_OSC_ENA=0, FLL1_ENA=0
{0x210 , 0x0073}, //Write{0x34      * AIF1 Rate(210H):         0073  AIF1_SR=0111, AIF1CLK_RATE=0011
{0x300 , 0x4010}, //Write{0x34      * AIF1 Control (1)(300H):  4010  AIF1ADCL_SRC=0, AIF1ADCR_SRC=1, AIF1ADC_TDM=0, AIF1_BCLK_INV=0, AIF1_LRCLK_INV=0, AIF1_WL=00, AIF1_FMT=10
{0x302 , 0x0000}, //Write{0x34      * AIF1 Master/Slave(302H): 0000  AIF1_TRI=0, AIF1_MSTR=0, AIF1_CLK_FRC=0, AIF1_LRCLK_FRC=0
//boost uplink volume
{0x311 , 0x4c00}, //Write{0x34      * AIF1 Master/Slave(302H): 0000  AIF1_TRI=0, AIF1_MSTR=0, AIF1_CLK_FRC=0, AIF1_LRCLK_FRC=0
{0x502 , 0x01c0}, //Write{0x34      * AIF1 Master/Slave(302H): 0000  AIF1_TRI=0, AIF1_MSTR=0, AIF1_CLK_FRC=0, AIF1_LRCLK_FRC=0

{0x208 , 0x000F}, //Write{0x34      * Clocking (1)(208H):      000F  TOCLK_ENA=0, DSP_FS1CLK_ENA=1, DSP_FS2CLK_ENA=1, DSP_FSINTCLK_ENA=1, SYSCLK_SRC=1
{0x200 , 0x0011}, //Write{0x34      * AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=10, AIF1CLK_INV=0, AIF1CLK_DIV=0, AIF1CLK_ENA=1
{0x208 , 0x000E}, //Write{0x34      * Clocking (1)(208H):      000E  TOCLK_ENA=0, DSP_FS1CLK_ENA=1, DSP_FS2CLK_ENA=1, DSP_FSINTCLK_ENA=1, SYSCLK_SRC=0
};

static struct incall_reg fm_hp_reg[] = {
	{0x39, 0x01E4},
	//{0x01, 0x0003},
	{0x01, 0x0023},
	{0x200, 0x0001},
	//{0x01, 0x0303},
	{0x01, 0x0323},
	{0x60, 0x0022},
	{0x4C, 0x9F25},
	{0x2D, 0x0002},
	{0x2E, 0x0020},
	{0x1C, 0x017e},
	{0x1D, 0x017e},
	{0x03, 0x0030},
	{0x54, 0x0033},
	{0x60, 0x00EE},
};
static struct incall_reg fm_spk_reg[] = {
	{0x39, 0x01EC},
	//{0x01, 0x0003},
	{0x01, 0x0023},
	{0x200, 0x0001},
	{0x03, 0x0330},
	{0x2D, 0x0002},
	{0x2E, 0x0020},
	{0x36, 0x000C},
	{0x22, 0x0000},
	{0x23, 0x0000},
	{0x24, 0x0018},
	{0x26, 0x017f},
	{0x25, 0x0128},
	//{0x01, 0x3003},
	{0x01, 0x3323},
};
static struct incall_reg main_mic_reg[] = {
	{0x700, 0xA101},
	{0x01, 0x0333},
	{0x02, 0x6240},
	{0x04, 0x0303},
	{0x18, 0x014B},
	{0x28, 0x0030},
	{0x29, 0x0030},
	{0x200, 0x0001},
	{0x208, 0x000A},
	{0x210, 0x0073},
	{0x300, 0x0010},
	{0x606, 0x0002},
	{0x607, 0x0002},
	{0x620, 0x0000},
	{0x220, 0x0000},
	{0x221, 0x0700},
	{0x222, 0x86C2},
	{0x223, 0x00E0},
	{0x224, 0x0C88},
	{0x220, 0x0001},
	{0x200, 0x0011},
};
static struct incall_reg hp_mic_reg[] = {
	{0x700, 0xA101},
	{0x01, 0x0333},
	{0x02 , 0x6110},
	{0x04 , 0x0303},
	{0x1A , 0x014B},
	{0x28 , 0x0003},
	{0x2A , 0x0030},
	{0x200 , 0x0001},
	{0x208, 0x000A},
	{0x210, 0x0073},
	{0x300 , 0xC010},
	{0x606 , 0x0002},
	{0x607 , 0x0002},
	{0x620, 0x0000},
	{0x220, 0x0000},
	{0x221, 0x0700},
	{0x222, 0x86C2},
	{0x223, 0x00E0},
	{0x224, 0x0C88},
	{0x220, 0x0001},
	{0x200, 0x0011},
};


static struct incall_reg aux_mic_rec_reg[] = {
	{0x700, 0xA101}, //  SMbus_16inx_16dat     Write  0x34      *

	{0x01, 0x0033}, //  SMbus_16inx_16dat     Write  0x34      *  
//INSERT_DELAY_MS [50]

	{0x02 ,0x6120}, //  SMbus_16inx_16dat     Write  0x34      * Power Management (2)(02H): 6110  TSHUT_ENA=1, TSHUT_OPDIS=1, OPCLK_ENA=0, MIXINL_ENA=0, MIXINR_ENA=1, IN2L_ENA=0, IN1L_ENA=0, IN2R_ENA=0, IN1R_ENA=1
	{0x04 ,0x0303}, //  SMbus_16inx_16dat     Write  0x34      * Power Management (4)(04H): 0303  AIF2ADCL_ENA=0, AIF2ADCR_ENA=0, AIF1ADC2L_ENA=0, AIF1ADC2R_ENA=0, AIF1ADC1L_ENA=1, AIF1ADC1R_ENA=1, DMIC2L_ENA=0, DMIC2R_ENA=0, DMIC1L_ENA=0, DMIC1R_ENA=0, ADCL_ENA=1, ADCR_ENA=1
	{0x1B ,0x014B}, //  SMbus_16inx_16dat     Write  0x34      * Right Line Input 1&2 Volume(1AH): 014B  IN1_VU=1, IN1R_MUTE=0, IN1R_ZC=1, IN1R_VOL=0_1011
	{0x28 ,0x000C}, //  SMbus_16inx_16dat     Write  0x34      * Input Mixer (2)(28H):    0003  IN2LP_TO_IN2L=0, IN2LN_TO_IN2L=0, IN1LP_TO_IN1L=0, IN1LN_TO_IN1L=0, IN2RP_TO_IN2R=0, IN2RN_TO_IN2R=0, IN1RP_TO_IN1R=1, IN1RN_TO_IN1R=1
	{0x2A ,0x0180}, //  SMbus_16inx_16dat     Write  0x34      * Input Mixer (4)(2AH):    0020  IN2R_TO_MIXINR=0, IN2R_MIXINR_VOL=0, IN1R_TO_MIXINR=1, IN1R_MIXINR_VOL=0, MIXOUTR_MIXINR_VOL=000
	{0x200 ,0x0001}, //  SMbus_16inx_16dat     Write  0x34      * AIF1 Clocking (1)(200H): 0001  AIF1CLK_SRC=00, AIF1CLK_INV=0, AIF1CLK_DIV=0, AIF1CLK_ENA=1
	{0x208, 0x000A}, //  SMbus_16inx_16dat     Write  0x34      * Clocking (1)(208H):      000A  TOCLK_ENA=0, DSP_FS1CLK_ENA=1, DSP_FS2CLK_ENA=0, DSP_FSINTCLK_ENA=1, SYSCLK_SRC=0
	{0x210, 0x0073}, //  SMbus_16inx_16dat     Write  0x34      * Clocking (1)(208H):      000A  TOCLK_ENA=0, DSP_FS1CLK_ENA=1, DSP_FS2CLK_ENA=0, DSP_FSINTCLK_ENA=1, SYSCLK_SRC=0
	{0x300, 0xC010}, //  SMbus_16inx_16dat     Write  0x34      * AIF1 Control (1)(300H):  C050  AIF1ADCL_SRC=1, AIF1ADCR_SRC=1, AIF1ADC_TDM=0, AIF1_BCLK_INV=0, AIF1_LRCLK_INV=0, AIF1_WL=10, AIF1_FMT=10
	{0x606 ,0x0002}, //  SMbus_16inx_16dat     Write  0x34      * AIF1 ADC1 Left Mixer Routing(606H): 0002  ADC1L_TO_AIF1ADC1L=1, AIF2DACL_TO_AIF1ADC1L=0
	{0x607 ,0x0002}, //  SMbus_16inx_16dat     Write  0x34      * AIF1 ADC1 Right Mixer Routing(607H): 0002  ADC1R_TO_AIF1ADC1R=1, AIF2DACR_TO_AIF1ADC1R=0
	{0x620, 0x0000}, //  SMbus_16inx_16dat     Write  0x34      * Oversampling(620H):      0000  ADC_OSR128=0, DAC_OSR128=0

	{0x220 ,0x0000}, //  SMbus_16inx_16dat     Write  0x34      * FLL1 Control (1)(220H):  0000  FLL1_OSC_ENA=0, FLL1_ENA=0
	{0x221, 0x0700}, //  SMbus_16inx_16dat     Write  0x34      * FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=8, FLL1_FRATIO=1
	{0x222, 0x86C2}, //  SMbus_16inx_16dat     Write  0x34      * FLL1 Control (3)(222H):  3126  FLL1_K=0.19199
	{0x223, 0x00E0}, //  SMbus_16inx_16dat     Write  0x34      * FLL1 Control (4)(223H):  0100  FLL1_N=8
	{0x224, 0x0C88}, //  SMbus_16inx_16dat     Write  0x34      * FLL1 Control (5)(224H):  0C80  FLL1_BYP=FLL1, FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_REFCLK_DIV=MCLK / 1, FLL1_REFCLK_SRC=MCLK1
	{0x220, 0x0001}, //  SMbus_16inx_16dat     Write  0x34      * FLL1 Control (1)(220H):  0001  FLL1_OSC_ENA=0, FLL1_ENA=1
	{0x200, 0x0011}, //  SMbus_16inx_16dat     Write  0x34      * AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=FLL1/BCLK1, AIF1CLK_INV=0, AIF1CLK_DIV=AIF1CLK, AIF1CLK_ENA=1
};


static struct incall_reg receiver_play_reg[] = {
	{0x01, 0x0803}, // SMbus_16inx_16dat     Write  0x34      * Power Management (1)(01H): 3003  SPKOUTR_ENA=1, SPKOUTL_ENA=1, HPOUT2_ENA=0, HPOUT1L_ENA=0, HPOUT1R_ENA=0, MICB2_ENA=0, MICB1_ENA=0, VMID_SEL=01, BIAS_ENA=1
	{0x03, 0x00F0}, // SMbus_16inx_16dat     Write  0x34      * Power Management (3)(03H): 0300  LINEOUT1N_ENA=0, LINEOUT1P_ENA=0, LINEOUT2N_ENA=0, LINEOUT2P_ENA=0, SPKRVOL_ENA=1, SPKLVOL_ENA=1, MIXOUTLVOL_ENA=0, MIXOUTRVOL_ENA=0, MIXOUTL_ENA=0, MIXOUTR_ENA=0
	{0x05, 0x0303}, // SMbus_16inx_16dat     Write  0x34      * Power Management (5)(05H): 3003  AIF2DACL_ENA=1, AIF2DACR_ENA=1, AIF1DAC2L_ENA=0, AIF1DAC2R_ENA=0, AIF1DAC1L_ENA=0, AIF1DAC1R_ENA=0, DAC2L_ENA=0, DAC2R_ENA=0, DAC1L_ENA=1, DAC1R_ENA=1
	{0x1F, 0x0010}, // SMbus_16inx_16dat     Write  0x34      *
	{0x2D, 0x0001}, // SMbus_16inx_16dat     Write  0x34      *
	{0x2E, 0x0001}, // SMbus_16inx_16dat     Write  0x34      *
	{0x33 ,0x0018}, // SMbus_16inx_16dat     Write  0x34      *

	{0x200, 0x0001}, // SMbus_16inx_16dat     Write  0x34      * AIF1 Clocking (1)(200H): 0001  AIF1CLK_SRC=00, AIF1CLK_INV=0, AIF1CLK_DIV=0, AIF1CLK_ENA=1
	{0x208, 0x000A}, // SMbus_16inx_16dat     Write  0x34      * Clocking (1)(208H):      000A  CLASSD_EDGE=00, TOCLK_ENA=0, DSP_FS1CLK_ENA=1, DSP_FS2CLK_ENA=0, DSP_FSINTCLK_ENA=1, SYSCLK_SRC=0
	{0x210, 0x0073}, // SMbus_16inx_16dat     Write  0x34      *
	{0x420, 0x0000}, // SMbus_16inx_16dat     Write  0x34      * AIF1 DAC1 Filters (1)(420H): 0000  AIF1DAC1_MUTE=0, AIF1DAC1_MONO=0, AIF1DAC1_MUTERATE=0, AIF1DAC1_UNMUTE_RAMP=0, AIF1DAC1_DEEMP=00
	{0x601, 0x0001}, // SMbus_16inx_16dat     Write  0x34      * DAC1 Left Mixer Routing(601H): 0001  ADC2_TO_DAC1L=0, ADC1_TO_DAC1L=0, AIF2DACL_TO_DAC1L=0, AIF1DAC2L_TO_DAC1L=0, AIF1DAC1L_TO_DAC1L=1
	{0x602, 0x0001}, // SMbus_16inx_16dat     Write  0x34      * DAC1 Right Mixer Routing(602H): 0001  ADC2_TO_DAC1R=0, ADC1_TO_DAC1R=0, AIF2DACR_TO_DAC1R=0, AIF1DAC2R_TO_DAC1R=0, AIF1DAC1R_TO_DAC1R=1
	{0x610, 0x01C0}, // SMbus_16inx_16dat     Write  0x34      * DAC1 Left Volume(610H):  01C0  DAC1L_MUTE=0, DAC1_VU=1, DAC1L_VOL=1100_0000
	{0x611, 0x01C0}, // SMbus_16inx_16dat     Write  0x34      * DAC1 Right Volume(611H): 01C0  DAC1R_MUTE=0, DAC1_VU=1, DAC1R_VOL=1100_0000

	{0x220, 0x0000}, // SMbus_16inx_16dat     Write  0x34      * FLL1 Control (1)(220H):  0000  FLL1_OSC_ENA=0, FLL1_ENA=0
	{0x221, 0x0700}, // SMbus_16inx_16dat     Write  0x34      * FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=8, FLL1_FRATIO=1
	{0x222, 0x86C2}, // SMbus_16inx_16dat     Write  0x34      * FLL1 Control (3)(222H):  3126  FLL1_K=0.19199
	{0x223, 0x00E0}, // SMbus_16inx_16dat     Write  0x34      * FLL1 Control (4)(223H):  0100  FLL1_N=8
	{0x224, 0x0C88}, // SMbus_16inx_16dat     Write  0x34      * FLL1 Control (5)(224H):  0C80  FLL1_BYP=FLL1, FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_REFCLK_DIV=MCLK / 1, FLL1_REFCLK_SRC=MCLK1
	{0x220, 0x0005}, // SMbus_16inx_16dat     Write  0x34      * FLL1 Control (1)(220H):  0001  FLL1_OSC_ENA=0, FLL1_ENA=1
	{0x200, 0x0011}, // SMbus_16inx_16dat     Write  0x34      * AIF1 Clocking (1)(200H): 0011  AIF1CLK_SRC=FLL1/BCLK1, AIF1CLK_INV=0, AIF1CLK_DIV=AIF1CLK, AIF1CLK_ENA=1
};

static struct incall_reg voip_bt[] = {
// ----- ------ -------------------- ------- --------- ------------------------------
//  REG   DATA         ACCESS        READ OR  DEVICE
// INDEX  VALUE         TYPE          WRITE   ADDRESS  COMMENT (for information only)
// ----- ------ -------------------- ------- --------- ------------------------------

 {0x00 , 0x0000},     // Software reset
   //INSERT_DELAY_MS [500]

// Audio Interface Input/Output (I/O) Configuration
//AIF1
{0x700 , 0xA101},     //

//AIF2
{0x702 , 0xC100},     //
{0x703 , 0xC100},     //
{0x704 , 0xC100},     //
{0x706 , 0x4100},     //
//AIF3
{0x707 , 0xA100},     // GPIO 8(707H):            A100  GP8_DIR=1, GP8_PU=0, GP8_PD=1, GP8_POL=0, GP8_OP_CFG=0, GP8_DB=1, GP8_LVL=0, GP8_FN=0_0000
{0x708 , 0x4100},     // GPIO 9(708H):            A100  GP9_DIR=1, GP9_PU=0, GP9_PD=1, GP9_POL=0, GP9_OP_CFG=0, GP9_DB=1, GP9_LVL=0, GP9_FN=0_0000
{0x709 , 0xA100},     // GPIO 10(709H):           A100  GP10_DIR=1, GP10_PU=0, GP10_PD=1, GP10_POL=0, GP10_OP_CFG=0, GP10_DB=1, GP10_LVL=0, GP10_FN=0_0000
{0x70A , 0xA100},     // GPIO 11(70AH):           A100  GP11_DIR=1, GP11_PU=0, GP11_PD=1, GP11_POL=0, GP11_OP_CFG=0, GP11_DB=1, GP11_LVL=0, GP11_FN=0_0000

// Analogue Configuration
// {0x39 , 0x01E4},     // Enable VMID soft start (fast), Start-up Bias Current Enabled
 {0x01 , 0x0003},     // Enable bias generator, Enable VMID, Enable Microphone Bias 1
//   INSERT_DELAY_MS [50]

// Analogue Input Configuration
// {0x02 , 0x6360},     // Enable IN1L Input PGA, Enable Left Input Mixer (MIXINL)
// {0x18 , 0x010B},     // Unmute IN1L Input PGA
// {0x1B , 0x010B},     // Unmute IN1L Input PGA
// {0x28 , 0x003C},     // Connect IN1LN to IN1L PGA, Connect IN1LP to IN1L PGA
// {0x29 , 0x0030},     // Unmute IN1L PGA output to Left Input Mixer (MIXINL) Path
// {0x2A , 0x0180},     // Unmute IN1L PGA output to Left Input Mixer (MIXINL) Path

// Analogue Output Configuration
// {0x03 , 0x00A0},     // Enable Left Output Mixer (MIXOUTL), Enable Left Output Mixer Volume Control
// {0x1F , 0x0000},     // Unmute HPOUT2 (Earpiece)
// {0x2D , 0x0001},     // Unmute DAC1 (Left) to Left Output Mixer (MIXOUTL) path
// {0x33 , 0x0010},     // Unmute Left Output Mixer to HPOUT2 (Earpiece) path
// {0x38 , 0x0040},     // Enable HPOUT2 Mixer and Input Stage
// {0x01 , 0x0833},     // Enable bias generator, Enable VMID, Enable Microphone Bias 1, Enable HPOUT2 (Earpiece)
   
// Path Configuration
 {0x04 , 0x3303},     // Enable ADC (Left), Enable AIF2 ADC (Left) Path
 {0x05 , 0x3303},     // Enable DAC1 (Left), Enable AIF2 DAC (Left) Path
//{0x601 , 0x0004},     // Enable the AIF2 (Left) to DAC 1 (Left) mixer path
//{0x603 , 0x018C},     // Set the volume on the STL to DAC2 mixer paths to 0dB
{0x604 , 0x0001},     // Enable the STL to DAC 2 (Left) mixer path
{0x605 , 0x0001},     // Enable the STL to DAC 2 (Left) mixer path
{0x606 , 0x0001},     // Enable the STL to DAC 2 (Left) mixer path
{0x607 , 0x0001},     // Enable the STL to DAC 2 (Left) mixer path

 {0x06 , 0x000A},     // Power Management (6)(06H): 0014  AIF3ADC_SRC=None, AIF2DAC_SRC=Left and Right inputs from AIF2, AIF3_TRI=0, AIF3_ADCDAT_SRC=GPIO5/DACDAT2, AIF2_ADCDAT_SRC=GPIO8/DACDAT3, AIF2_DACDAT_SRC=DACDAT2, AIF1_DACDAT_SRC=DACDAT1

// Clocking
{0x620 , 0x0000},     // Select Low Power ADC/DMIC Oversample Rate,

{0x220 , 0x0000},     // FLL1 Control (1)(220H):  0000  FLL1_OSC_ENA=0, FLL1_ENA=0
{0x221 , 0x1700},     // FLL1 Control (2)(221H):  0700  FLL1_OUTDIV=8, FLL1_FRATIO=1
{0x222 , 0x3126},     // FLL1 Control (3)(222H):  3126  FLL1_K=0.19199
{0x223 , 0x0100},     // FLL1 Control (4)(223H):  0100  FLL1_N=8
{0x224 , 0x0C88},     // FLL1 Control (5)(224H):  0C80  FLL1_BYP=FLL1, FLL1_FRC_NCO_VAL=01_1001, FLL1_FRC_NCO=0, FLL1_REFCLK_DIV=MCLK / 1, FLL1_REFCLK_SRC=MCLK1
{0x220 , 0x0005},     // FLL1 Control (1)(220H):  0001  FLL1_OSC_ENA=0, FLL1_ENA=1
                                                      // Select Low Power DAC Oversample Rate (Default)
//AIF1
{0x210 , 0x0033},     // AIF2 Sample Rate = 8 kHz, AIF2CLK/Fs ratio = 256
{0x300 , 0x4010},     // AIF1 Word Length = 16-bits, AIF2 Format = DSP //0x4118 -> 4010 by tab use i2s mode
//{0x311 , 0x4100},     // Enable AIF2 DSP Mono Mode
{0x302 , 0x4000},     // AIF2 Slave Mode (Default Register Value)
{0x303 , 0x0070},     //
{0x304 , 0x0040},     //

//AIF2 FLL2
{0x0240 , 0x0000},
{0x0241 , 0x1700},
{0x0242 , 0x3126},
{0x0243 , 0x0100},
{0x0244 , 0x0C88},
{0x0240 , 0x0005},

//AIF2
{0x211 , 0x0033},     // AIF2 Sample Rate = 8 kHz, AIF2CLK/Fs ratio = 256
{0x310 , 0x4118},     // AIF2 Word Length = 16-bits, AIF2 Format = DSP
{0x311 , 0x4100},     // Enable AIF2 DSP Mono Mode
{0x312 , 0x4000},     // AIF2 Slave Mode (Default Register Value)
{0x313 , 0x0070},     //
{0x314 , 0x0020},     //

{0x208 , 0x000E},     // Enable the DSP processing clock for AIF2, 
                                                      // Enable the core clock,
                                                      // Set the core clock source to AIF2CLK
{0x200 , 0x0011},     // Enable AIF2 Clock, AIF2 Clock Source = MCLK2 pin
{0x204 , 0x0011},     // Enable AIF2 Clock, AIF2 Clock Source = MCLK2 pin

// Unmutes
//{0x610 , 0x00C0},     // Unmute DAC 1 (Left)
{0x612 , 0x01C0},     // Unmute DAC 2 (Left) path
{0x613 , 0x01C0},     // Unmute DAC 2 (Left) path
{0x420 , 0x0000},     // Unmute the AIF2 DAC path
{0x520 , 0x0000},     // Unmute the AIF2 DAC path
};

