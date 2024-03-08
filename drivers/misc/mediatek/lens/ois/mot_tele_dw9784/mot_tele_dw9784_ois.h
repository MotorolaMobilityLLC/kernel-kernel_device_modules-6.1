#ifndef __DW9784_OIS_H__
#define __DW9784_OIS_H__

//#include "ois_ext_cmd.h"

#define EOK 0

#define MSC_SIZE_W                                              10240
#define PID_SIZE_W                                              256

/* gyro offset calibration */
/*
#define GYRO_OFFSET_CAL_OK 0x00
#define GYRO_CAL_TIME_OVER 0xFF
#define X_GYRO_OFFSET_SPEC_OVER_NG 0x0001
#define X_GYRO_RAW_DATA_CHECK 0x0010
#define Y_GYRO_OFFSET_SPEC_OVER_NG 0x0002
#define Y_GYRO_RAW_DATA_CHECK 0x0020
#define GYRO_OFST_CAL_OVERCNT 110

int gyro_offset_calibrtion(void);
motOISGOffsetResult *dw9784_get_gyro_offset_result(void);
void calibration_save(void);
void check_calibration_data(void);
int gyro_offset_check_update(void);
int square_motion_test (int radius, int accuracy, int deg_step, int w_time0,
                               int w_time1, int w_time2, int ref_stroke, motOISExtData *pResult);
*/
void ois_reset(void);
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
