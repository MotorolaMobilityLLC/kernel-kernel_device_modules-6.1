// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <linux/firmware.h>
#include "dw9784_i2c.h"
#include "dw9784_ois.h"

#define DRIVER_NAME "dw9784"

#define LOG_INF(format, args...)                                               \
	pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)

#define DW9784_NAME				"dw9784"

#define DW9784_CTRL_DELAY_US			5000
#define DW9784_CHIP_ID_ADDRESS 0x7000
#define DW9784_CHECKSUM_ADRESS 0x700C
#define DW9784_CHIP_ID         0x9784
#define FW_VER_CURR_ADDR       0x7001
#define MTP_START_ADDRESS      0x8000

#define FW_VERSION_OFFSET   10493
#define FW_CHECKSUM_OFFSET  10495

extern void dw9784_set_i2c_client(struct i2c_client *i2c_client);

typedef struct
{
	unsigned int driverIc;
	unsigned int size;
	unsigned short *fwContentPtr;
	unsigned short version;
	unsigned short checksum;
}FirmwareContex;

FirmwareContex g_firmwareContext;
unsigned short g_downloadByForce;

const struct firmware *dw9784fw;

#define DW9784_OIS_I2C_SLAVE_ADDR 0xE4
#define DW9784_REG_CHIP_CTRL              0xD000
#define DW9784_REG_OIS_DSP_CTRL           0xD001
#define DW9784_REG_OIS_LOGIC_RESET        0xD002
#define DW9784_REG_OIS_USER_WRITE_PROTECT 0xEBF1
#define DW9784_REG_OIS_CTRL               0x7011
#define DW9784_REG_OIS_MODE               0x7012
#define OIS_ON                   0x0000   // OIS ON/SERVO ON
#define SERVO_ON                 0x0001   // OIS OFF/SERVO ON
#define SERVO_OFF                0x0002   // OIS OFF/SERVO OFF
#define ON                       0x0001
#define OFF                      0x0000

#define REGULATOR_MAXSIZE			16
#define PINCTRL_MAXSIZE				16
#define CLK_MAXSIZE				16

static const char * const ldo_names[] = {
	"vin",
	"vdd",
};

/* power  on stage : idx = 0, 2, 4, ... */
/* power off stage : idx = 1, 3, 5, ... */
static const char * const pio_names[] = {
	//"mclk_4mA",
	//"mclk_off",
	//"oisen_off",
	//"oisen_on",
	// "vdd_on",
	// "vdd_off",
	// "rst_low",
	// "rst_high",
};

/* set TG   : idx = 0, 2, 4, ... */
/* set Freq : idx = 1, 3, 5, ... */
static const char * const clk_names[] = {
	// "mclk",
	// "24",
};

/* dw9784 device structure */
struct dw9784_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *ldo[REGULATOR_MAXSIZE];
	struct pinctrl *pio;
	struct pinctrl_state *pio_state[PINCTRL_MAXSIZE];
	struct clk *clk[CLK_MAXSIZE];
};

#define OIS_DATA_NUMBER 32
struct OisInfo {
	int32_t is_ois_supported;
	int32_t data_mode;  /* ON/OFF */
	int32_t samples;
	int32_t x_shifts[OIS_DATA_NUMBER];
	int32_t y_shifts[OIS_DATA_NUMBER];
	int64_t timestamps[OIS_DATA_NUMBER];
};

struct mtk_ois_pos_info {
	struct OisInfo *p_ois_info;
};

static struct i2c_client *m_client;
/* Control commnad */
#define VIDIOC_MTK_S_OIS_MODE _IOW('V', BASE_VIDIOC_PRIVATE + 2, int32_t)

#define VIDIOC_MTK_G_OIS_POS_INFO _IOWR('V', BASE_VIDIOC_PRIVATE + 3, struct mtk_ois_pos_info)

static inline struct dw9784_device *to_dw9784_ois(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct dw9784_device, ctrls);
}

static inline struct dw9784_device *sd_to_dw9784_ois(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct dw9784_device, sd);
}


static void ois_mdelay(unsigned long ms)
{
        unsigned long us = ms*1000;
        usleep_range(us, us+2000);
}

void ois_reset(void)
{
	ois_i2c_wr_u16(m_client, 0xD002, 0x0001);
	ois_mdelay(4);

	ois_i2c_wr_u16(m_client, 0xD001, 0x0001);
	ois_mdelay(25);

	ois_i2c_wr_u16(m_client, 0xEBF1, 0x56FA);
}

static int dw9784_release(struct dw9784_device *dw9784)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9784->sd);

	ret = ois_i2c_wr_u16(client, DW9784_REG_OIS_CTRL, SERVO_OFF);
	I2C_OPERATION_CHECK(ret);
	ois_mdelay(4);
	ret = ois_i2c_wr_u16(client, DW9784_REG_CHIP_CTRL, OFF);
	I2C_OPERATION_CHECK(ret);
	ois_mdelay(10);

	return 0;
}

void erase_FW_flash_sector(void)
{
	LOG_INF("[dw9784_erase_mtp] start erasing firmware flash\r\n");
	/* 4k Sector_0 */
	ois_i2c_wr_u16(m_client, 0xde03, 0x0000);
	ois_mdelay(1);
	/* 4k Sector Erase */
	ois_i2c_wr_u16(m_client, 0xde04, 0x0002);
	ois_mdelay(10);
	/* 4k Sector_1 */
	ois_i2c_wr_u16(m_client, 0xde03, 0x0008);
	ois_mdelay(1);
	/* 4k Sector Erase */
	ois_i2c_wr_u16(m_client, 0xde04, 0x0002);
	ois_mdelay(10);
	/* 4k Sector_2 */
	ois_i2c_wr_u16(m_client, 0xde03, 0x0010);
	ois_mdelay(1);
	/* 4k Sector Erase */
	ois_i2c_wr_u16(m_client, 0xde04, 0x0002);
	ois_mdelay(10);
	/* 4k Sector_3 */
	ois_i2c_wr_u16(m_client, 0xde03, 0x0018);
	ois_mdelay(1);
	/* 4k Sector Erase */
	ois_i2c_wr_u16(m_client, 0xde04, 0x0002);
	ois_mdelay(10);
	/* 4k Sector_4 */
	ois_i2c_wr_u16(m_client, 0xde03, 0x0020);
	ois_mdelay(1);
	/* 4k Sector Erase */
	ois_i2c_wr_u16(m_client, 0xde04, 0x0002);
	ois_mdelay(10);
	LOG_INF("[dw9784_erase_mtp] complete erasing firmware flash\r\n");
}

void dw9784_code_pt_off(void)
{
	LOG_INF("[dw9784_code_pt_off] start");
	/* release all protection */
	ois_i2c_wr_u16(m_client, 0xFD00, 0x5252);
	ois_mdelay(1);
	LOG_INF("[dw9784_code_pt_off] finish");
}

void dw9784_pid_erase(void)
{
	int ret = 0;
	LOG_INF("[dw9784_pid_erase] start pid flash(IF) erase");
	ret = ois_i2c_wr_u16(m_client, 0xde03, 0x0000);			// page 0
	mdelay(1);
	ret = ois_i2c_wr_u16(m_client, 0xde04, 0x0008);			// page erase
	mdelay(10);											// need to delay after erase
	LOG_INF("[dw9784_pid_erase] finish");
}

void dw9784_flash_acess(void)
{
	int ret = 0;

	LOG_INF("[dw9784_flash_acess] execution");
	/* release all protection */
	ret = ois_i2c_wr_u16(m_client, 0xFAFA, 0x98AC);
	ois_mdelay(1);
	ret = ois_i2c_wr_u16(m_client, 0xF053, 0x70BD);
	ois_mdelay(1);
}

int dw9784_whoami_chk(void)
{
	uint16_t sec_chip_id;
	int ret = 0;
	ret = ois_i2c_wr_u16(m_client, 0xD000, 0x0001); /* chip enable */
	ois_mdelay(4);
	ret = ois_i2c_wr_u16(m_client, 0xD001, 0x0000); /* dsp off mode */
	ois_mdelay(1);

	dw9784_flash_acess(); /* All protection */

	ret = ois_i2c_rd_u16(m_client, 0xD060, &sec_chip_id); /* 2nd chip id */
	LOG_INF("[dw9784_ois_ready_check] sec_chip_id : 0x%04x", sec_chip_id);
	if(sec_chip_id != 0x0020)
	{
		LOG_INF("[dw9784] second_chip_id check fail : 0x%04X", sec_chip_id);
		LOG_INF("[dw9784] second_enter shutdown mode");
		ret = ois_i2c_wr_u16(m_client, 0xD000, 0x0000); /* ic */
		return -1;
	}

	ois_reset(); /* ois reset */
	return 0;
}

int dw9784_checksum_fw_chk(void)
{
	int ret = 0;
	/*
	Bit [0]: FW checksum error
	Bit [1]: Module cal. checksum error
	Bit [2]: Set cal. checksum error
	*/
	uint16_t reg_fw_checksum;			// 0x700C
	uint16_t reg_checksum_status;		// 0x700D

	ret = ois_i2c_rd_u16(m_client, 0x700C, &reg_fw_checksum);
	ret = ois_i2c_rd_u16(m_client, 0x700D, &reg_checksum_status);

	LOG_INF("[dw9784_checksum_fw_chk] reg_checksum_status : 0x%04X", reg_checksum_status);
	LOG_INF("[dw9784_checksum_fw_chk] reg_fw_checksum : 0x%04X", reg_fw_checksum);

	if( (reg_checksum_status & 0x0001) == 0)
	{
		LOG_INF("[dw9784_checksum_fw_chk] fw checksum pass");
		return 0;
	}else
	{
		LOG_INF("[dw9784_checksum_fw_chk] fw checksum error reg_fw_checksum : 0x%04X", reg_fw_checksum);
		return -1;
	}
}

int dw9784_erase_mtp_rewritefw(void)
{
	uint16_t FMC;
	int ret = 0;
	LOG_INF("dw9784 erase for rewritefw starting..");
	ret = ois_i2c_wr_u16(m_client, 0xd001, 0x0000);
	ois_mdelay(1);
	dw9784_flash_acess();

	ret = ois_i2c_wr_u16(m_client, 0xDE01, 0x0000); // FMC block FW select
	ois_mdelay(1);
	ret = ois_i2c_rd_u16(m_client, 0xDE01, &FMC);
	if (FMC != 0)
	{
		LOG_INF("[dw9784_download_fw] FMC register value 1st warning : %04x", FMC);
		ret = ois_i2c_wr_u16(m_client, 0xDE01, 0x0000);
		ois_mdelay(1);
		FMC = 0; // initialize FMC value

		ret = ois_i2c_rd_u16(m_client, 0xDE01, &FMC);
		if (FMC != 0)
		{
			LOG_INF("[dw9784_download_fw] 2nd FMC register value 2nd warning : %04x", FMC);
			LOG_INF("[dw9784_download_fw] stop f/w download");
			return -1;
		}
	}

	 /* code protection off */
	dw9784_code_pt_off();

	/* 512 byte page */
	ret = ois_i2c_wr_u16(m_client, 0xde03, 0x0027);
	/* page erase */
	ret = ois_i2c_wr_u16(m_client, 0xde04, 0x0008);
	ois_mdelay(10);

	ret = ois_i2c_wr_u16(m_client, 0xd000, 0x0000); /* Shut download mode */

	return 0;
}

int dw9784_prepare_fw_download(void)
{
	/* step 1: MTP Erase and DSP Disable for firmware 0x8000 write */
	/* step 2: MTP setup                                           */
	/* step 3. FMC register check                                  */
	/* step 4. code protection off                                 */
	/* step 5. erase flash fw data                                 */
	uint16_t FMC;
	int ret = 0;

	ret = ois_i2c_wr_u16(m_client, 0xd001, 0x0000);
	dw9784_flash_acess();

	ret = ois_i2c_wr_u16(m_client, 0xDE01, 0x0000); // FMC block FW select
	ois_mdelay(1);
	ret = ois_i2c_rd_u16(m_client, 0xDE01, &FMC);
	if (FMC != 0)
	{
		LOG_INF("[dw9784_download_fw] FMC register value 1st warning : %04x", FMC);
		ret = ois_i2c_wr_u16(m_client, 0xDE01, 0x0000);
		ois_mdelay(1);
		FMC = 0; // initialize FMC value

		ret = ois_i2c_rd_u16(m_client, 0xDE01, &FMC);
		if (FMC != 0)
		{
			LOG_INF("[dw9784_download_fw] 2nd FMC register value 2nd warning : %04x", FMC);
			LOG_INF("[dw9784_download_fw] stop f/w download");
			return -1;
		}
	}

	dw9784_code_pt_off();
	erase_FW_flash_sector();
	LOG_INF("[dw9784_download_fw] start firmware download");
	return 0;
}

int GenerateFirmwareContexts(void)
{
	int i4RetValue;
	g_firmwareContext.size = 10496; /* size: word */
	g_firmwareContext.driverIc = 0x9784;

	i4RetValue = request_firmware(&dw9784fw, "mot_dw9784.prog", NULL);

	if(i4RetValue<0) {
		LOG_INF("get dw9784 firmware failed!\n");
	} else {
		int i = 0;
		g_firmwareContext.fwContentPtr = kmalloc(20992,GFP_KERNEL);
		if(!g_firmwareContext.fwContentPtr )
		{
			LOG_INF("kmalloc fwContentPtr failed in GenerateFirmwareContexts\n");
			return -1;
		}
		while(i<10496) {
			(g_firmwareContext.fwContentPtr)[i] = (((dw9784fw->data)[2*i] << 8) + ((dw9784fw->data)[(2*i+1)]));
			i++;
		}
		LOG_INF("get dw9784 firmware sucess,and size = %zu\n",dw9784fw->size);

			g_firmwareContext.version = *(g_firmwareContext.fwContentPtr+FW_VERSION_OFFSET);
			g_firmwareContext.version = ((g_firmwareContext.version << 8) & 0xff00) |
	                                  ((g_firmwareContext.version >> 8) & 0xff);
			g_firmwareContext.checksum = *(g_firmwareContext.fwContentPtr+FW_CHECKSUM_OFFSET);
			g_firmwareContext.checksum = ((g_firmwareContext.checksum << 8) & 0xff00) |
	                                   ((g_firmwareContext.checksum >> 8) & 0xff);
	}
	g_downloadByForce = 0;
	return i4RetValue;
}

int dw9784_check_if_download(void)
{
	uint16_t FMC;
	int ret = 0;

	/* step 1. Writes 512Byte FW(PID) data to IF flash.	(FMC register check) */
	ret = ois_i2c_wr_u16(m_client, 0xDE01, 0x0000);
	ois_mdelay(1);
	ret = ois_i2c_rd_u16(m_client, 0xDE01, &FMC);
	ois_mdelay(1);
	if (FMC != 0x1000)
	{
		LOG_INF("[dw9784_download_fw] IF FMC register value 1st warning : %04x", FMC);
		ret = ois_i2c_wr_u16(m_client, 0xDE01, 0x0000);
		ois_mdelay(1);
		FMC = 0; // initialize FMC value

		ret = ois_i2c_rd_u16(m_client, 0xDE01, &FMC);
		if (FMC != 0x1000)
		{
			LOG_INF("[dw9784_download_fw] 2nd IF FMC register value 2nd fail : %04x", FMC);
			LOG_INF("[dw9784_download_fw] stop firmware download");
			return -1;
		}
	}

	/* step 2. erease IF(FW/PID) eFLASH  */
	dw9784_pid_erase();

	LOG_INF("[dw9784_download_fw] start firmware/pid download");
	return 0;
}

void dw9784_post_firmware_download(void)
{
	uint16_t fwchecksum = 0;

	ois_reset();
	fwchecksum = dw9784_checksum_fw_chk();
	if(fwchecksum != 0)
	{
		LOG_INF("[dw9784] firmware checksum error");
		dw9784_erase_mtp_rewritefw();
		LOG_INF("[dw9784] change dw9784 state to shutdown mode");
	}
	return;
}

int dw9784_download_fw(void)
{
	int ret = 0;
	unsigned short i;
	unsigned short addr;
	//unsigned short buf[g_firmwareContext.size];
	unsigned short* buf_temp = NULL;
	buf_temp = kmalloc(20992,GFP_KERNEL);
	if(buf_temp == NULL)
	{
		LOG_INF("kmalloc buf_temp failed in download_fw\n");
		return -1;
	}
	memset(buf_temp, 0, g_firmwareContext.size * sizeof(unsigned short));

	ret = dw9784_prepare_fw_download();
	LOG_INF("[dw9784] flash download::vendor_dw9784");
	if (ret != EOK) {
		dw9784_erase_mtp_rewritefw();
		LOG_INF("[dw9784] firmware download error, ret = 0x%x", ret);
		LOG_INF("[dw9784] change dw9784 state to shutdown mode");
	}

	for (i = 0; i < g_firmwareContext.size; i += DATPKT_SIZE)
	{
		addr = MTP_START_ADDRESS + i;
		i2c_block_write_reg(addr, g_firmwareContext.fwContentPtr + i, DATPKT_SIZE);

		if (i == g_firmwareContext.size / DATPKT_SIZE + 1) {
			ret = dw9784_check_if_download();
			if (ret < 0) {
				LOG_INF("dw9784 check if download fail");
					kfree(buf_temp);
					buf_temp = NULL;
			}
		}
	}
	LOG_INF("[dw9784_download_fw] write firmware to flash\n");
	/* step 4: firmware sequential read from flash */
	for (i = 0; i <  g_firmwareContext.size; i += DATPKT_SIZE)
	{
		addr = MTP_START_ADDRESS + i;
		i2c_block_read_reg(addr, buf_temp + i, DATPKT_SIZE);
	}
	LOG_INF("[dw9784_download_fw] read firmware from flash\n");
	/* step 5: firmware verify */
	for (i = 0; i < g_firmwareContext.size; i++)
	{
		buf_temp[i] = ((buf_temp[i] << 8) & 0xff00) | ((buf_temp[i] >> 8) & 0x00ff);
		if (g_firmwareContext.fwContentPtr[i] != buf_temp[i])
		{
			LOG_INF("[dw9784_download_fw] firmware verify NG!!! ADDR:%04X -- firmware:%04x -- READ:%04x\n", MTP_START_ADDRESS+i, g_firmwareContext.fwContentPtr[i], buf_temp[i]);
			kfree(buf_temp);
			buf_temp = NULL;
			return -1;
		}else
			ret = EOK;
	}

	dw9784_post_firmware_download();
	kfree(buf_temp);
	buf_temp = NULL;

	return ret;
}

int dw9784_check_fw_download(void)
{
	int ret = 0;
	uint16_t fwchecksum = 0;
	uint16_t first_chip_id = 0;
	uint16_t chip_checksum = 0;
	uint16_t fw_version_current = 0;
	uint16_t fw_version_latest = 0;

	if (m_client == NULL) {
		LOG_INF("FATAL: OIS CCI context error!!!");
		return -1;
	}

	ret = GenerateFirmwareContexts();
	if(ret < 0)
		return ret;

	if (dw9784_whoami_chk() != 0) {
		LOG_INF("[dw9784] second chip id check fail");
		return -1;
	}

	ret = ois_i2c_rd_u16(m_client, DW9784_CHIP_ID_ADDRESS, &first_chip_id);
	ret = ois_i2c_rd_u16(m_client, 0x7001, &chip_checksum);
	LOG_INF("[dw9784] FW_VER_PHONE_MAKER : 0x%x", chip_checksum);
	ret = ois_i2c_rd_u16(m_client, 0x7002, &chip_checksum);
	LOG_INF("[dw9784] FW_DATE_PHONE_MAKER : 0x%x", chip_checksum);

	LOG_INF("[dw9784] first_chip_id : 0x%x", first_chip_id);
	if (first_chip_id != DW9784_CHIP_ID) { /* first_chip_id verification failed */
		LOG_INF("[dw9784] start flash download:: size:%d, version:0x%x",
			g_firmwareContext.size, g_firmwareContext.version);
		g_downloadByForce = 1;
		ret = dw9784_download_fw(); /* Need to forced update OIS firmware again. */
	} else {
		fwchecksum = dw9784_checksum_fw_chk();
		if(fwchecksum != 0)
		{
			g_downloadByForce = 1;
			LOG_INF("[dw9784] firmware checksum error");
		}

		ret = ois_i2c_rd_u16(m_client, FW_VER_CURR_ADDR, &fw_version_current);
		fw_version_latest = g_firmwareContext.version; /*Firmware version read from file content.*/

		LOG_INF("[dw9784] fw_version_current = 0x%x, fw_version_latest = 0x%x",
			              fw_version_current, fw_version_latest);

		/* download firmware, check if need update, download firmware to flash */
		if (g_downloadByForce || ((fw_version_current & 0xFFFF) < (fw_version_latest & 0xFFFF))) {
			g_downloadByForce = 1;

			LOG_INF("[dw9784] start flash download:: size:%d, version:0x%x g_downloadByForce %d",
			                 g_firmwareContext.size, g_firmwareContext.version, g_downloadByForce);

			ret = dw9784_download_fw();
			LOG_INF("[dw9784] flash download::vendor_dw9784");
			if (ret != EOK) {
				dw9784_erase_mtp_rewritefw();
				LOG_INF("[dw9784] firmware download error, ret = 0x%x", ret);
				LOG_INF("[dw9784] change dw9784 state to shutdown mode");
				g_downloadByForce = 1;
			}
		} else {
			LOG_INF("[dw9784] ois firmware version is updated, skip download");
		}
	}
	kfree(g_firmwareContext.fwContentPtr);
	g_firmwareContext.fwContentPtr = NULL;

	return g_downloadByForce;
}


static int dw9784_init(struct dw9784_device *dw9784)
{
	// Initial parameters
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9784->sd);
	unsigned short lock_ois = 0;

	LOG_INF("+\n");

	m_client = client;

	client->addr = DW9784_OIS_I2C_SLAVE_ADDR >> 1;
	dw9784_check_fw_download();
	ois_reset();
	ret = ois_i2c_rd_u16(client, 0x7011, &lock_ois);
	LOG_INF("Check HW lock_ois: %x\n", lock_ois);

	return 0;
}

/* Power handling */
static int dw9784_power_off(struct dw9784_device *dw9784)
{
	int ret;
	int i;
	int hw_nums;

	LOG_INF("+\n");

	ret = dw9784_release(dw9784);
	if (ret)
		LOG_INF("dw9784 release failed!\n");

	hw_nums = ARRAY_SIZE(pio_names);
	if (hw_nums > PINCTRL_MAXSIZE)
		hw_nums = PINCTRL_MAXSIZE;
	for (i = 0; i < hw_nums; i += 2) {
		int idx = 1 + i;

		if (dw9784->pio && dw9784->pio_state[idx]) {
			ret = pinctrl_select_state(dw9784->pio,
						dw9784->pio_state[idx]);
			if (ret < 0)
				LOG_INF("cannot enable %d pintctrl\n", idx);
		}
	}
	usleep_range(1000, 1100);

	hw_nums = ARRAY_SIZE(ldo_names);
	if (hw_nums > REGULATOR_MAXSIZE)
		hw_nums = REGULATOR_MAXSIZE;
	for (i = 0; i < hw_nums; i++) {
		if (dw9784->ldo[i]) {
			ret = regulator_disable(dw9784->ldo[i]);
			if (ret < 0)
				LOG_INF("cannot enable %d regulator\n", i);
		}
	}

	hw_nums = ARRAY_SIZE(clk_names);
	if (hw_nums > CLK_MAXSIZE)
		hw_nums = CLK_MAXSIZE;
	for (i = 0; i < hw_nums; i += 2) {
		struct clk *mclk = dw9784->clk[i];
		struct clk *mclk_src = dw9784->clk[i + 1];

		if (mclk && mclk_src) {
			clk_disable_unprepare(mclk_src);
			clk_disable_unprepare(mclk);
		}
	}

	LOG_INF("-\n");

	return ret;
}

static int dw9784_power_on(struct dw9784_device *dw9784)
{
	int ret;
	int i;
	int hw_nums;

	LOG_INF("+\n");

	hw_nums = ARRAY_SIZE(clk_names);
	if (hw_nums > CLK_MAXSIZE)
		hw_nums = CLK_MAXSIZE;
	for (i = 0; i < hw_nums; i += 2) {
		struct clk *mclk = dw9784->clk[i];
		struct clk *mclk_src = dw9784->clk[i + 1];

		if (mclk && mclk_src) {
			ret = clk_prepare_enable(mclk);
			if (ret) {
				LOG_INF("cannot enable %d mclk\n", i);
			} else {
				ret = clk_prepare_enable(mclk_src);
				if (ret) {
					LOG_INF("cannot enable %d mclk_src\n", i);
				} else {
					ret = clk_set_parent(mclk, mclk_src);
					if (ret)
						LOG_INF("cannot set %d mclk's parent'\n", i);
				}
			}
		}
	}

	hw_nums = ARRAY_SIZE(ldo_names);
	if (hw_nums > REGULATOR_MAXSIZE)
		hw_nums = REGULATOR_MAXSIZE;
	for (i = 0; i < hw_nums; i++) {
		if (dw9784->ldo[i]) {
			ret = regulator_enable(dw9784->ldo[i]);
			if (ret < 0)
				LOG_INF("cannot enable %d regulator\n", i);
		}
	}
	usleep_range(1000, 1100);

	hw_nums = ARRAY_SIZE(pio_names);
	if (hw_nums > PINCTRL_MAXSIZE)
		hw_nums = PINCTRL_MAXSIZE;
	for (i = 0; i < hw_nums; i += 2) {
		int idx = i;

		if (dw9784->pio && dw9784->pio_state[idx]) {
			ret = pinctrl_select_state(dw9784->pio,
						dw9784->pio_state[idx]);
			if (ret < 0)
				LOG_INF("cannot enable %d pintctrl\n", idx);
		}
	}

	if (ret < 0)
		return ret;

	/*
	 * TODO(b/139784289): Confirm hardware requirements and adjust/remove
	 * the delay.
	 */
	usleep_range(DW9784_CTRL_DELAY_US, DW9784_CTRL_DELAY_US + 100);

	ret = dw9784_init(dw9784);
	if (ret < 0)
		goto fail;

	return 0;

fail:
	hw_nums = ARRAY_SIZE(pio_names);
	if (hw_nums > PINCTRL_MAXSIZE)
		hw_nums = PINCTRL_MAXSIZE;
	for (i = 0; i < hw_nums; i += 2) {
		int idx = 1 + i;

		if (dw9784->pio && dw9784->pio_state[idx]) {
			ret = pinctrl_select_state(dw9784->pio,
						dw9784->pio_state[idx]);
			if (ret < 0)
				LOG_INF("cannot enable %d pintctrl\n", idx);
		}
	}
	usleep_range(1000, 1100);

	hw_nums = ARRAY_SIZE(ldo_names);
	if (hw_nums > REGULATOR_MAXSIZE)
		hw_nums = REGULATOR_MAXSIZE;
	for (i = 0; i < hw_nums; i++) {
		if (dw9784->ldo[i]) {
			ret = regulator_disable(dw9784->ldo[i]);
			if (ret < 0)
				LOG_INF("cannot enable %d regulator\n", i);
		}
	}

	hw_nums = ARRAY_SIZE(clk_names);
	if (hw_nums > CLK_MAXSIZE)
		hw_nums = CLK_MAXSIZE;
	for (i = 0; i < hw_nums; i += 2) {
		struct clk *mclk = dw9784->clk[i];
		struct clk *mclk_src = dw9784->clk[i + 1];

		if (mclk && mclk_src) {
			clk_disable_unprepare(mclk_src);
			clk_disable_unprepare(mclk);
		}
	}

	LOG_INF("-\n");

	return ret;
}

static int dw9784_set_ctrl(struct v4l2_ctrl *ctrl)
{
	/* struct dw9784_device *dw9784 = to_dw9784_ois(ctrl); */

	return 0;
}

static const struct v4l2_ctrl_ops dw9784_ois_ctrl_ops = {
	.s_ctrl = dw9784_set_ctrl,
};

static int dw9784_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;

	LOG_INF("%s\n", __func__);

	ret = pm_runtime_get_sync(sd->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(sd->dev);
		return ret;
	}

	return 0;
}

static int dw9784_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	LOG_INF("%s\n", __func__);

	pm_runtime_put(sd->dev);

	return 0;
}

static long dw9784_ops_core_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct dw9784_device *dw9784 = sd_to_dw9784_ois(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&dw9784->sd);

	switch (cmd) {

	case VIDIOC_MTK_S_OIS_MODE:
	{
		int *ois_mode = arg;
		LOG_INF("VIDIOC_MTK_S_OIS_MODE mode = %d\n",*ois_mode);
		if (*ois_mode)
		{
			ois_i2c_wr_u16(client, 0x70D9, 0x0102);
			ois_i2c_wr_u16(client, 0x70DA, 0x0000);
			ois_i2c_wr_u16(client, 0x7012, 0x0001);
			ois_i2c_wr_u16(client, 0x7011, 0x0000);
			LOG_INF("VIDIOC_MTK_S_OIS_MODE Enable\n");
		} else {
			ois_i2c_wr_u16(client, 0x7012, 0x0001);
			ois_i2c_wr_u16(client, 0x7011, 0x0001);
			LOG_INF("VIDIOC_MTK_S_OIS_MODE Disable\n");
		}
	}
	break;

	case VIDIOC_MTK_G_OIS_POS_INFO:
	{
		struct mtk_ois_pos_info *info = arg;
		struct OisInfo pos_info;

		memset(&pos_info, 0, sizeof(struct OisInfo));

		/* To Do */

		if (copy_to_user((void *)info->p_ois_info, &pos_info, sizeof(pos_info)))
			ret = -EFAULT;
	}
	break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static const struct v4l2_subdev_internal_ops dw9784_int_ops = {
	.open = dw9784_open,
	.close = dw9784_close,
};

static const struct v4l2_subdev_core_ops dw9784_ops_core = {
	.ioctl = dw9784_ops_core_ioctl,
};

static const struct v4l2_subdev_ops dw9784_ops = {
	.core = &dw9784_ops_core,
};

static void dw9784_subdev_cleanup(struct dw9784_device *dw9784)
{
	v4l2_async_unregister_subdev(&dw9784->sd);
	v4l2_ctrl_handler_free(&dw9784->ctrls);
#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&dw9784->sd.entity);
#endif
}

static int dw9784_init_controls(struct dw9784_device *dw9784)
{
	struct v4l2_ctrl_handler *hdl = &dw9784->ctrls;
	/* const struct v4l2_ctrl_ops *ops = &dw9784_ois_ctrl_ops; */

	v4l2_ctrl_handler_init(hdl, 1);

	if (hdl->error)
		return hdl->error;

	dw9784->sd.ctrl_handler = hdl;

	return 0;
}

static int dw9784_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dw9784_device *dw9784;
	int ret;
	int i;
	int hw_nums;

	LOG_INF("+\n");

	dw9784 = devm_kzalloc(dev, sizeof(*dw9784), GFP_KERNEL);
	if (!dw9784)
		return -ENOMEM;

	hw_nums = ARRAY_SIZE(clk_names);
	if (hw_nums > CLK_MAXSIZE)
		hw_nums = CLK_MAXSIZE;
	for (i = 0; i < hw_nums; i++) {
		dw9784->clk[i] = devm_clk_get(dev, clk_names[i]);
		if (IS_ERR(dw9784->clk[i])) {
			dw9784->clk[i] = NULL;
			LOG_INF("cannot get %s clk\n", clk_names[i]);
		}
	}

	hw_nums = ARRAY_SIZE(ldo_names);
	if (hw_nums > REGULATOR_MAXSIZE)
		hw_nums = REGULATOR_MAXSIZE;
	for (i = 0; i < hw_nums; i++) {
		dw9784->ldo[i] = devm_regulator_get(dev, ldo_names[i]);
		if (IS_ERR(dw9784->ldo[i])) {
			LOG_INF("cannot get %s regulator\n", ldo_names[i]);
			dw9784->ldo[i] = NULL;
		}
	}

	hw_nums = ARRAY_SIZE(pio_names);
	if (hw_nums > PINCTRL_MAXSIZE)
		hw_nums = PINCTRL_MAXSIZE;
	dw9784->pio = devm_pinctrl_get(dev);
	if (IS_ERR(dw9784->pio)) {
		ret = PTR_ERR(dw9784->pio);
		dw9784->pio = NULL;
		LOG_INF("cannot get pinctrl\n");
	} else {
		for (i = 0; i < hw_nums; i++) {
			dw9784->pio_state[i] = pinctrl_lookup_state(
				dw9784->pio, pio_names[i]);

			if (IS_ERR(dw9784->pio_state[i])) {
				ret = PTR_ERR(dw9784->pio_state[i]);
				dw9784->pio_state[i] = NULL;
				LOG_INF("cannot get %s pinctrl\n", pio_names[i]);
			}
		}
	}

	v4l2_i2c_subdev_init(&dw9784->sd, client, &dw9784_ops);
	dw9784->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dw9784->sd.internal_ops = &dw9784_int_ops;
	dw9784_set_i2c_client(client);

	ret = dw9784_init_controls(dw9784);
	if (ret)
		goto err_cleanup;

#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&dw9784->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	dw9784->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&dw9784->sd);
	if (ret < 0)
		goto err_cleanup;

	pm_runtime_enable(dev);

	LOG_INF("-\n");

	return 0;

err_cleanup:
	dw9784_subdev_cleanup(dw9784);
	return ret;
}

static void dw9784_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9784_device *dw9784 = sd_to_dw9784_ois(sd);

	LOG_INF("+\n");

	dw9784_subdev_cleanup(dw9784);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		dw9784_power_off(dw9784);
	pm_runtime_set_suspended(&client->dev);

	LOG_INF("-\n");
}

static int __maybe_unused dw9784_ois_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9784_device *dw9784 = sd_to_dw9784_ois(sd);

	return dw9784_power_off(dw9784);
}

static int __maybe_unused dw9784_ois_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9784_device *dw9784 = sd_to_dw9784_ois(sd);

	return dw9784_power_on(dw9784);
}

static const struct i2c_device_id dw9784_id_table[] = {
	{ DW9784_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, dw9784_id_table);

static const struct of_device_id dw9784_of_table[] = {
	{ .compatible = "mediatek,dw9784" },
	{ },
};
MODULE_DEVICE_TABLE(of, dw9784_of_table);

static const struct dev_pm_ops dw9784_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(dw9784_ois_suspend, dw9784_ois_resume, NULL)
};

static struct i2c_driver dw9784_i2c_driver = {
	.driver = {
		.name = DW9784_NAME,
		.pm = &dw9784_pm_ops,
		.of_match_table = dw9784_of_table,
	},
	.probe_new  = dw9784_probe,
	.remove = dw9784_remove,
	.id_table = dw9784_id_table,
};

module_i2c_driver(dw9784_i2c_driver);

MODULE_AUTHOR("Po-Hao Huang <Po-Hao.Huang@mediatek.com>");
MODULE_DESCRIPTION("DW9784 OIS driver");
MODULE_LICENSE("GPL");
