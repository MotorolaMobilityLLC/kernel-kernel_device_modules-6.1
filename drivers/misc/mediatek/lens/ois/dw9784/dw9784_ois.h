#ifndef __DW9784_OIS_H__
#define __DW9784_OIS_H__

#define EOK 0

#define OIS_SUCCESS						0
#define OIS_ERROR						-1

#define LOOP_A							200
#define	LOOP_B							LOOP_A-1
#define WAIT_TIME						100

#define FUNC_PASS						0
#define FUNC_FAIL						-1

#define MSC_SIZE_W                                              10240
#define PID_SIZE_W                                              256

/* gyro offset calibration */
#define GYRO_OFFSET_CAL_OK              0x00
#define GYRO_OFS_CAL_DONE_FAIL			0xFF
#define X_AXIS_GYRO_OFS_PASS			0x1
#define X_AXIS_GYRO_OFS_FAIL			0x1
#define Y_AXIS_GYRO_OFS_PASS			0x2
#define Y_AXIS_GYRO_OFS_FAIL			0x2
#define OIS_GYRO_CAL_TIME_OVER          0x08
#define X_AXIS_GYRO_OFS_OVER_MAX_LIMIT	0x10
#define Y_AXIS_GYRO_OFS_OVER_MAX_LIMIT	0x20
#define XY_AXIS_CHECK_GYRO_RAW_DATA		0x80
#define GYRO_OFST_CAL_OVERCNT           110

/* ioctl info */
#define OIS_HEA_MAX_TEST_POINTS 40
#define OIS_HEA_CHANNEL_NUM     2
#define OIS_EXT_DATA_MAX_LEN    (OIS_HEA_MAX_TEST_POINTS*OIS_HEA_CHANNEL_NUM*6*sizeof(u16)+4*sizeof(u16))

typedef enum {
	OIS_EXT_INVALID_CMD,
	//Action
	OIS_SART_FW_DL,
	OIS_START_HEA_TEST,
	OIS_START_GYRO_OFFSET_CALI,
	OIS_ACTION_MAX = OIS_START_GYRO_OFFSET_CALI,
	//Result
	OIS_QUERY_FW_INFO,
	OIS_QUERY_HEA_RESULT,
	OIS_QUERY_GYRO_OFFSET_RESULT,
	OIS_EXT_INTF_MAX,

	OIS_SET_GYRO_OFFSET = 100,
} motOISExtInfType;

typedef struct {
	u16 version;
	u16 date;
	u16 valid;
	u16 gOffset_valid;
} motOISFwInfo;

typedef struct {
	u16 radius;
	u16 accuracy;
	u16 steps_in_degree;
	u16 wait0;
	u16 wait1;
	u16 wait2;
	u16 ref_stroke;
} motOISHeaParam;

typedef struct {
	u16 points;
	u16 ng_points;
	u16 g_targetAdc[OIS_HEA_CHANNEL_NUM*2][OIS_HEA_MAX_TEST_POINTS];
	u16 g_diffs[OIS_HEA_CHANNEL_NUM][OIS_HEA_MAX_TEST_POINTS];
} motOISHeaResult;

typedef struct {
	u16 is_success;
	u16 x_offset;
	u16 y_offset;
} motOISGOffsetResult;

typedef union {
	motOISFwInfo fw_info;
	motOISHeaParam hea_param;
	motOISHeaResult hea_result;
	motOISGOffsetResult gyro_offset_result;
	u32 data[OIS_EXT_DATA_MAX_LEN/sizeof(u32)];
} motOISExtData;

typedef struct {
	motOISExtInfType cmd;
	motOISExtData    data;
} motOISExtIntf;

#define OIS_MAGIC 'O'
#define OISIOC_G_OISEXTINTF  _IOWR(OIS_MAGIC, 3, motOISExtIntf)

#define REF_ANGLE (1)
//#define REF_STROKE (130.56f)
#define REF_GYRO_RESULT (1000)	//Gyro 1 degree
#define GET_POSITION_COUNT (3)
#define AXIS_X (0)
#define AXIS_Y (1)
#define AXIS_ADC_X (2)
#define AXIS_ADC_Y (3)

static const int tab_sin[361] = {
	0,   17,   34,   52,   69,   87,  104,  121,  139,  156,  173,
	190,  207,  224,  241,  258,  275,  292,  309,  325,  342,
	358,  374,  390,  406,  422,  438,  453,  469,  484,  500,
	515,  529,  544,  559,  573,  587,  601,  615,  629,  642,
	656,  669,  681,  694,  707,  719,  731,  743,  754,  766,
	777,  788,  798,  809,  819,  829,  838,  848,  857,  866,
	874,  882,  891,  898,  906,  913,  920,  927,  933,  939,
	945,  951,  956,  961,  965,  970,  974,  978,  981,  984,
	987,  990,  992,  994,  996,  997,  998,  999,  999, 1000,
	999,  999,  998,  997,  996,  994,  992,  990,  987,  984,
	981,  978,  974,  970,  965,  961,  956,  951,  945,  939,
	933,  927,  920,  913,  906,  898,  891,  882,  874,  866,
	857,  848,  838,  829,  819,  809,  798,  788,  777,  766,
	754,  743,  731,  719,  707,  694,  681,  669,  656,  642,
	629,  615,  601,  587,  573,  559,  544,  529,  515,  500,
	484,  469,  453,  438,  422,  406,  390,  374,  358,  342,
	325,  309,  292,  275,  258,  241,  224,  207,  190,  173,
	156,  139,  121,  104,   87,   69,   52,   34,   17,    0,
	-17,  -34,  -52,  -69,  -87, -104, -121, -139, -156, -173,
	-190, -207, -224, -241, -258, -275, -292, -309, -325, -342,
	-358, -374, -390, -406, -422, -438, -453, -469, -484, -499,
	-515, -529, -544, -559, -573, -587, -601, -615, -629, -642,
	-656, -669, -681, -694, -707, -719, -731, -743, -754, -766,
	-777, -788, -798, -809, -819, -829, -838, -848, -857, -866,
	-874, -882, -891, -898, -906, -913, -920, -927, -933, -939,
	-945, -951, -956, -961, -965, -970, -974, -978, -981, -984,
	-987, -990, -992, -994, -996, -997, -998, -999, -999,-1000,
	-999, -999, -998, -997, -996, -994, -992, -990, -987, -984,
	-981, -978, -974, -970, -965, -961, -956, -951, -945, -939,
	-933, -927, -920, -913, -906, -898, -891, -882, -874, -866,
	-857, -848, -838, -829, -819, -809, -798, -788, -777, -766,
	-754, -743, -731, -719, -707, -694, -681, -669, -656, -642,
	-629, -615, -601, -587, -573, -559, -544, -529, -515, -500,
	-484, -469, -453, -438, -422, -406, -390, -374, -358, -342,
	-325, -309, -292, -275, -258, -241, -224, -207, -190, -173,
	-156, -139, -121, -104,  -87,  -69,  -52,  -34,  -17,    0
};

static const int tab_cos[361] = {
	1000,  999,  999,  998,  997,  996,  994,  992,  990,  987,  984,
	981,  978,  974,  970,  965,  961,  956,  951,  945,  939,
	933,  927,  920,  913,  906,  898,  891,  882,  874,  866,
	857,  848,  838,  829,  819,  809,  798,  788,  777,  766,
	754,  743,  731,  719,  707,  694,  681,  669,  656,  642,
	629,  615,  601,  587,  573,  559,  544,  529,  515,  499,
	484,  469,  453,  438,  422,  406,  390,  374,  358,  342,
	325,  309,  292,  275,  258,  241,  224,  207,  190,  173,
	156,  139,  121,  104,   87,   69,   52,   34,   17,    0,
	-17,  -34,  -52,  -69,  -87, -104, -121, -139, -156, -173,
	-190, -207, -224, -241, -258, -275, -292, -309, -325, -342,
	-358, -374, -390, -406, -422, -438, -453, -469, -484, -500,
	-515, -529, -544, -559, -573, -587, -601, -615, -629, -642,
	-656, -669, -681, -694, -707, -719, -731, -743, -754, -766,
	-777, -788, -798, -809, -819, -829, -838, -848, -857, -866,
	-874, -882, -891, -898, -906, -913, -920, -927, -933, -939,
	-945, -951, -956, -961, -965, -970, -974, -978, -981, -984,
	-987, -990, -992, -994, -996, -997, -998, -999, -999,-1000,
	-999, -999, -998, -997, -996, -994, -992, -990, -987, -984,
	-981, -978, -974, -970, -965, -961, -956, -951, -945, -939,
	-933, -927, -920, -913, -906, -898, -891, -882, -874, -866,
	-857, -848, -838, -829, -819, -809, -798, -788, -777, -766,
	-754, -743, -731, -719, -707, -694, -681, -669, -656, -642,
	-629, -615, -601, -587, -573, -559, -544, -529, -515, -499,
	-484, -469, -453, -438, -422, -406, -390, -374, -358, -342,
	-325, -309, -292, -275, -258, -241, -224, -207, -190, -173,
	-156, -139, -121, -104,  -87,  -69,  -52,  -34,  -17,    0,
	17,   34,   52,   69,   87,  104,  121,  139,  156,  173,
	190,  207,  224,  241,  258,  275,  292,  309,  325,  342,
	358,  374,  390,  406,  422,  438,  453,  469,  484,  499,
	515,  529,  544,  559,  573,  587,  601,  615,  629,  642,
	656,  669,  681,  694,  707,  719,  731,  743,  754,  766,
	777,  788,  798,  809,  819,  829,  838,  848,  857,  866,
	874,  882,  891,  898,  906,  913,  920,  927,  933,  939,
	945,  951,  956,  961,  965,  970,  974,  978,  981,  984,
	987,  990,  992,  994,  996,  997,  998,  999,  999, 1000
};

int gyro_offset_calibration(void);
int calibration_save(void);
void check_calibration_data(void);
int gyro_offset_check_update(void);
int circle_motion_test (int radius, int accuracy, int deg_step, int w_time0,
                               int w_time1, int w_time2, int ref_stroke, motOISExtData *pResult);

void ois_reset(void);
int dw9784_wait_check_register(unsigned short reg, unsigned short ref);
void erase_FW_flash_sector(void);
void dw9784_code_pt_off(void);
void dw9784_pid_erase(void);
void dw9784_flash_acess(void);
int dw9784_whoami_chk(void);
int dw9784_checksum_fw_chk(void);
int dw9784_erase_mtp_rewritefw(void);
int dw9784_prepare_fw_download(void);
int GenerateFirmwareContexts(void);
int dw9784_check_if_download(void);
void dw9784_post_firmware_download(void);
int dw9784_download_fw(void);
int dw9784_check_fw_download(void);
#endif
