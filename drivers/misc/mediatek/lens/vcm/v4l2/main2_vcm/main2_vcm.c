// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define DRIVER_NAME "main2_vcm"
#define MAIN2_VCM_I2C_SLAVE_ADDR 0x18

#define LOG_INF(format, args...)                                               \
	pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)
#define MAIN2_VCM_NAME				"main2_vcm"
#define MAIN2_VCM_MAX_FOCUS_POS			1023
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define MAIN2_VCM_FOCUS_STEPS			1

#define REGULATOR_MAXSIZE			16

static const char * const ldo_names[] = {
	"vin",
	"vdd",
};

/* main2_vcm device structure */
struct main2_vcm_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vin;
	struct regulator *vdd;
	struct pinctrl *vcamaf_pinctrl;
	struct pinctrl_state *vcamaf_on;
	struct pinctrl_state *vcamaf_off;
	struct regulator *ldo[REGULATOR_MAXSIZE];
};

#define I2CCOMM_MAXSIZE 4

/* I2C format */
struct stVCM_I2CFormat {
	/* Register address */
	uint8_t Addr[I2CCOMM_MAXSIZE];
	uint8_t AddrNum;

	/* Register Data */
	uint8_t CtrlData[I2CCOMM_MAXSIZE];
	uint8_t BitR[I2CCOMM_MAXSIZE];
	uint8_t Mask1[I2CCOMM_MAXSIZE];
	uint8_t BitL[I2CCOMM_MAXSIZE];
	uint8_t Mask2[I2CCOMM_MAXSIZE];
	uint8_t DataNum;
};

#define I2CSEND_MAXSIZE 4

struct VcmDriverConfig {
	// Init param
	unsigned int ctrl_delay_us;
	char wr_table[16][3];

	// Per-frame param
	unsigned int slave_addr;
	uint8_t I2CSendNum;
	struct stVCM_I2CFormat I2Cfmt[I2CSEND_MAXSIZE];

	// Uninit param
	unsigned int origin_focus_pos;
	unsigned int move_steps;
	unsigned int move_delay_us;
	char wr_rls_table[8][3];

	// Capacity
	int32_t vcm_bits;
	int32_t af_calib_bits;
};

struct mtk_vcm_info {
	struct VcmDriverConfig *p_vcm_info;
};

struct VcmDriverConfig g_vcmconfig = {.ctrl_delay_us=5000,};

/* Control commnad */
#define VIDIOC_MTK_S_LENS_INFO _IOWR('V', BASE_VIDIOC_PRIVATE + 3, struct mtk_vcm_info)

/* Control command for af & vib issue */
#ifdef CONFIG_AF_NOISE_ELIMINATION
#define AF_NOISE_TAG "AF_NOISE"
#define LOG_AFNE(format, args...)                                               \
	pr_err(AF_NOISE_TAG " [main2_vcm]-[%s] " format, __func__, ##args)

struct af_vib_lens_info {
  unsigned int holder;
  unsigned int lens_lock_pos;
};

struct af_vib_lens_info *ab_lens_info;
struct main2_vcm_device *ab_main2_vcm;

#define VIDIOC_MTK_S_SETOPENER  _IOWR('V', BASE_VIDIOC_PRIVATE + 21 , struct af_vib_lens_info)
#define VIDIOC_MTK_S_SETCLOSER _IOWR('V', BASE_VIDIOC_PRIVATE + 22 , struct af_vib_lens_info)
#define VIDIOC_MTK_S_SETVCMPOS  _IOWR('V', BASE_VIDIOC_PRIVATE + 23 , struct af_vib_lens_info)
#define VIDIOC_MTK_S_GETVCMSUBDEV  _IOWR('V', BASE_VIDIOC_PRIVATE + 24 , struct af_vib_lens_info)

#define NO_HOLD 0x0b00
#define CAM_HOLD 0x0b01
#define VIB_HOLD 0x0b10
#define ALL_HOLD 0x0b11
#define FIND_VCM 0x0b11
#define SUIT_POS 0x01ba

//Directly set len to Target Pos will cause noise
//Through several set_pos to Target Pos
#define SEGMENT_LENGTH 0x07

static unsigned short af_len = SUIT_POS;
static unsigned int Open_holder = NO_HOLD;
static unsigned int Close_holder = NO_HOLD;
static spinlock_t g_vcm_SpinLock;
#endif

static inline struct main2_vcm_device *to_main2_vcm_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct main2_vcm_device, ctrls);
}

static inline struct main2_vcm_device *sd_to_main2_vcm_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct main2_vcm_device, sd);
}

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};

static void register_setting(struct i2c_client *client, char table[][3], int table_size)
{
	int ret = 0, read_count = 0, i = 0, j = 0;

	for (i = 0; i < table_size; ++i) {

		LOG_INF("table[%d] = [0x%x 0x%x 0x%x]\n",
			i, table[i][0],
			table[i][1],
			table[i][2]);

		if (table[i][0] == 0)
			break;

		// write command
		if (table[i][0] == 0x1) {

			// write register
			ret = i2c_smbus_write_byte_data(client,
					table[i][1],
					table[i][2]);
			if (ret < 0) {
				LOG_INF(
					"i2c write fail: %d, table[%d] = [0x%x 0x%x 0x%x]\n",
					ret, i, table[i][0],
					table[i][1],
					table[i][2]);
			}

		// read command
		} else if (table[i][0] == 0x2) {
			read_count = 0;

			// read register and check value
			do {
				if (read_count > 10) {
					LOG_INF(
						"timeout, i2c read fail: %d, table[%d] = [0x%x 0x%x 0x%x]\n",
						ret, i, table[i][0],
						table[i][1],
						table[i][2]);
					break;
				}
				ret = i2c_smbus_read_byte_data(client,
					table[i][1]);
				read_count++;
			} while (ret != table[i][2]);

		// delay command
		} else if (table[i][0] == 0x3) {

			LOG_INF("delay time: %dms\n", table[i][1]);
			mdelay(table[i][1]);

		// read command w/o check result value
		} else if (table[i][0] == 0x4) {

			// read register
			for (j = 0; j < table[i][2]; ++j) {
				ret = i2c_smbus_read_byte_data(client,
					table[i][1]);
				if (ret < 0)
					LOG_INF(
						"i2c read fail: %d, table[%d] = [0x%x 0x%x 0x%x]\n",
						ret, i, table[i][0],
						table[i][1],
						table[i][2]);
				else
					LOG_INF(
						"table[%d] = [0x%x 0x%x 0x%x], result value is 0x%x",
						i, table[i][0], table[i][1],
						table[i][2], ret);
			}

		} else {
			// reserved
		}
		udelay(100);
	}
}

static int main2_vcm_set_position(struct main2_vcm_device *main2_vcm, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&main2_vcm->sd);
	int ret = -1;
	char puSendCmd[3] = {0};
	unsigned int nArrayIndex = 0;
	int i = 0, j = 0;
	int retry = 3;

	for (i = 0; i < g_vcmconfig.I2CSendNum; ++i, nArrayIndex = 0) {
		int nCommNum = g_vcmconfig.I2Cfmt[i].AddrNum +
			g_vcmconfig.I2Cfmt[i].DataNum;

		// Fill address
		for (j = 0; j < g_vcmconfig.I2Cfmt[i].AddrNum; ++j) {
			if (nArrayIndex >= ARRAY_SIZE(puSendCmd)) {
				LOG_INF("nArrayIndex(%d) exceeds the size of puSendCmd(%d)\n",
					nArrayIndex, (int)ARRAY_SIZE(puSendCmd));
				return -1;
			}
			puSendCmd[nArrayIndex] = g_vcmconfig.I2Cfmt[i].Addr[j];
			++nArrayIndex;
		}

		// Fill data
		for (j = 0; j < g_vcmconfig.I2Cfmt[i].DataNum; ++j) {
			if (nArrayIndex >= ARRAY_SIZE(puSendCmd)) {
				LOG_INF("nArrayIndex(%d) exceeds the size of puSendCmd(%d)\n",
					nArrayIndex, (int)ARRAY_SIZE(puSendCmd));
				return -1;
			}
			puSendCmd[nArrayIndex] =
				((((val >> g_vcmconfig.I2Cfmt[i].BitR[j]) &
				g_vcmconfig.I2Cfmt[i].Mask1[j]) <<
				g_vcmconfig.I2Cfmt[i].BitL[j]) &
				g_vcmconfig.I2Cfmt[i].Mask2[j]) |
				g_vcmconfig.I2Cfmt[i].CtrlData[j];
			++nArrayIndex;
		}

		while (retry-- > 0) {
			ret = i2c_master_send(client, puSendCmd, nCommNum);
			if (ret >= 0)
				break;
		}

		if (ret < 0) {
			LOG_INF(
				"puSendCmd I2C failure i2c_master_send: %d, I2Cfmt[%d].AddrNum/DataNum: %d/%d\n",
				ret, i, g_vcmconfig.I2Cfmt[i].AddrNum,
				g_vcmconfig.I2Cfmt[i].DataNum);
			return ret;
		}
	}

	return ret;
}

static int main2_vcm_release(struct main2_vcm_device *main2_vcm)
{
	int ret, val;
	int diff_dac = 0;
	int nStep_count = 0;
	int i = 0;

	struct i2c_client *client = v4l2_get_subdevdata(&main2_vcm->sd);

	diff_dac = g_vcmconfig.origin_focus_pos - main2_vcm->focus->val;
	nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
		g_vcmconfig.move_steps;

	val = main2_vcm->focus->val;

	for (i = 0; i < nStep_count; ++i) {
#ifdef CONFIG_AF_NOISE_ELIMINATION
	if(Open_holder == VIB_HOLD || Open_holder == CAM_HOLD)
	{
		LOG_AFNE("Skip Diff Reset Action for Open_holder=0x0%x, nStep_count=%d.\n",Open_holder, nStep_count);
		break;
	}
#endif
		val += (diff_dac < 0 ? (g_vcmconfig.move_steps*(-1)) :
			g_vcmconfig.move_steps);

		ret = main2_vcm_set_position(main2_vcm, val);
		if (ret < 0) {
			LOG_INF("%s I2C failure: %d\n",
				__func__, ret);
			return ret;
		}
		usleep_range(g_vcmconfig.move_delay_us,
			g_vcmconfig.move_delay_us + 1000);
	}

#ifdef CONFIG_AF_NOISE_ELIMINATION
	//Segmented set pos for VIB release VCM
	if(Open_holder == VIB_HOLD)
	{
		int i = 0;
		int af_step_count = 0;
		diff_dac = g_vcmconfig.origin_focus_pos - SUIT_POS;
		af_step_count = (af_len < 0 ? (af_len*(-1)) : diff_dac) / SEGMENT_LENGTH;
		af_len = SUIT_POS;
		LOG_AFNE("Reset pos to (%d) with %d Segments, echo segment long:0x0%x. Open_holder%x\n", g_vcmconfig.origin_focus_pos, af_step_count,\
			SEGMENT_LENGTH, Open_holder);

		for(i = 0;i < af_step_count; i++)
		{
			af_len += (diff_dac < 0 ? (SEGMENT_LENGTH*(-1)) : SEGMENT_LENGTH);
			if(af_len > g_vcmconfig.origin_focus_pos)
				af_len = g_vcmconfig.origin_focus_pos;

			ret = main2_vcm_set_position(ab_main2_vcm, af_len);
		}
		LOG_AFNE("MTK_S_SETVCMPOS, reset target pos=0x%x, ret=%d. Open_holder=0x0%x\n", af_len, ret, Open_holder);
	}
	else
	{
		// last step to origin
		ret = main2_vcm_set_position(main2_vcm, g_vcmconfig.origin_focus_pos);
	}
#else
	// last step to origin
	ret = main2_vcm_set_position(main2_vcm, g_vcmconfig.origin_focus_pos);
#endif
	if (ret < 0) {
		LOG_INF("%s I2C failure: %d\n",
			__func__, ret);
		return ret;
	}
	register_setting(client, g_vcmconfig.wr_rls_table, 8);

	return 0;
}

static int main2_vcm_init(struct main2_vcm_device *main2_vcm)
{
	struct i2c_client *client = v4l2_get_subdevdata(&main2_vcm->sd);
	int ret = 0;

	LOG_INF("+\n");

	client->addr = g_vcmconfig.slave_addr >> 1;

	register_setting(client, g_vcmconfig.wr_table, 16);

	LOG_INF("-\n");

	return ret;
}

/* Power handling */
static int main2_vcm_power_off(struct main2_vcm_device *main2_vcm)
{
	int ret = 0;
	int ldo_num = 0;
	int i = 0;

	LOG_INF("+\n");

	ret = main2_vcm_release(main2_vcm);
	if (ret)
		LOG_INF("main2_vcm release failed!\n");

	ldo_num = ARRAY_SIZE(ldo_names);
	if (ldo_num > REGULATOR_MAXSIZE)
		ldo_num = REGULATOR_MAXSIZE;
	for (i = 0; i < ldo_num; i++) {
		if (main2_vcm->ldo[i]) {
			ret = regulator_disable(main2_vcm->ldo[i]);
			if (ret < 0)
				LOG_INF("cannot disable %d regulator\n", i);
		}
	}

	if (main2_vcm->vcamaf_pinctrl && main2_vcm->vcamaf_off)
		ret = pinctrl_select_state(main2_vcm->vcamaf_pinctrl,
					main2_vcm->vcamaf_off);

	return ret;
}

static int main2_vcm_power_on(struct main2_vcm_device *main2_vcm)
{
	int ret = 0;
	int ldo_num = 0;
	int i = 0;

	LOG_INF("+\n");

	ldo_num = ARRAY_SIZE(ldo_names);
	if (ldo_num > REGULATOR_MAXSIZE)
		ldo_num = REGULATOR_MAXSIZE;
	for (i = 0; i < ldo_num; i++) {
		if (main2_vcm->ldo[i]) {
			ret = regulator_enable(main2_vcm->ldo[i]);
			if (ret < 0)
				LOG_INF("cannot enable %d regulator\n", i);
		}
	}

	if (main2_vcm->vcamaf_pinctrl && main2_vcm->vcamaf_on)
		ret = pinctrl_select_state(main2_vcm->vcamaf_pinctrl,
					main2_vcm->vcamaf_on);

	return ret;
}

static int main2_vcm_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct main2_vcm_device *main2_vcm = to_main2_vcm_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		LOG_INF("pos(%d)\n", ctrl->val);
		ret = main2_vcm_set_position(main2_vcm, ctrl->val);
		if (ret < 0) {
			LOG_INF("%s I2C failure: %d\n",
				__func__, ret);
			return ret;
		}
	}
	return 0;
}

static const struct v4l2_ctrl_ops main2_vcm_vcm_ctrl_ops = {
	.s_ctrl = main2_vcm_set_ctrl,
};

static int main2_vcm_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct main2_vcm_device *main2_vcm = sd_to_main2_vcm_vcm(sd);

	LOG_INF("+\n");

#ifdef CONFIG_AF_NOISE_ELIMINATION
	ab_main2_vcm = main2_vcm;
	LOG_AFNE("Open_holder=0x0%x\n", Open_holder);
	spin_lock(&g_vcm_SpinLock);
	if(Open_holder == CAM_HOLD)
	{
		//holder == CAM_HOLD while CAM working
		spin_unlock(&g_vcm_SpinLock);
		LOG_AFNE("open by VIB. Now VCM is ON!!!  open_hold=0x0%x, return -CAM_HOLD \n", Open_holder);
		return -CAM_HOLD;
	}else if(Open_holder == VIB_HOLD)
	{
		//holder == VIB_HOLD, VIB working && CAM start
		spin_unlock(&g_vcm_SpinLock);
		LOG_AFNE("open by CAM. Start to ON VCM, close VIB fdVCM first, Open_holder=0x0%x\n", Open_holder);
		//Close fdVCM by VIB, then power on for CAM
		main2_vcm_power_off(main2_vcm);
		spin_lock(&g_vcm_SpinLock);
		Open_holder = CAM_HOLD;
		spin_unlock(&g_vcm_SpinLock);
		ret = main2_vcm_power_on(main2_vcm);
		if (ret < 0) {
			LOG_INF("power on fail, ret = %d\n", ret);
			return ret;
		}
		LOG_AFNE("open by CAM. RepowerStart to ON VCM, close VIB fdVCM first, open_hold=0x0%x\n", Open_holder);
		return 0;
	}
	spin_unlock(&g_vcm_SpinLock);
#endif

	ret = main2_vcm_power_on(main2_vcm);
	if (ret < 0) {
		LOG_INF("power on fail, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int main2_vcm_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct main2_vcm_device *main2_vcm = sd_to_main2_vcm_vcm(sd);

	LOG_INF("+\n");
#ifdef CONFIG_AF_NOISE_ELIMINATION
	LOG_AFNE("Open_holder=0x0%x\n", Open_holder);
	if(Open_holder == CAM_HOLD && Close_holder != CAM_HOLD)
	{
		LOG_AFNE("Close VCM subdev by VIB, only close fd, not change Open_holder=0x0%x\n", Open_holder);
		return 0;
	}
	LOG_INF("Close vcm subdev, open_hold=0x0%x, set to NO_HOLD 0x0b00\n", Open_holder);
	main2_vcm_power_off(main2_vcm);
	spin_lock(&g_vcm_SpinLock);
	Open_holder = NO_HOLD;
	Close_holder = NO_HOLD;
	spin_unlock(&g_vcm_SpinLock);
#else
	main2_vcm_power_off(main2_vcm);
#endif

	return 0;
}

static long main2_vcm_ops_core_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct main2_vcm_device *main2_vcm = sd_to_main2_vcm_vcm(sd);

#ifdef CONFIG_AF_NOISE_ELIMINATION
	ab_main2_vcm = main2_vcm;
#endif
	switch (cmd) {

	case VIDIOC_MTK_S_LENS_INFO:
	{
		struct mtk_vcm_info *info = arg;

		if (copy_from_user(&g_vcmconfig,
				   (void *)info->p_vcm_info,
				   sizeof(struct VcmDriverConfig)) != 0) {
			LOG_INF("VIDIOC_MTK_S_LENS_INFO copy_from_user failed\n");
			ret = -EFAULT;
			break;
		}

		// Confirm hardware requirements and adjust/remove the delay.
		usleep_range(g_vcmconfig.ctrl_delay_us, g_vcmconfig.ctrl_delay_us + 100);

		LOG_INF("slave_addr: 0x%x\n", g_vcmconfig.slave_addr);

		ret = main2_vcm_init(main2_vcm);
		if (ret < 0)
			LOG_INF("init error\n");
	}
	break;
#ifdef CONFIG_AF_NOISE_ELIMINATION
	case VIDIOC_MTK_S_SETOPENER:
        {
		ab_lens_info = arg;
		spin_lock(&g_vcm_SpinLock);
		Open_holder = ab_lens_info->holder;
		spin_unlock(&g_vcm_SpinLock);
		LOG_AFNE("MTK_S_SETOPENER  Open_holder = 0x0%x\n",Open_holder);
	}
	break;
	case VIDIOC_MTK_S_SETCLOSER:
	{
		ab_lens_info = arg;
		spin_lock(&g_vcm_SpinLock);
		Close_holder = ab_lens_info->holder;
		spin_unlock(&g_vcm_SpinLock);
		LOG_AFNE("MTK_S_SETCLOSER, Close_holder = 0x0%x\n", Close_holder);
	}
	break;
	case VIDIOC_MTK_S_SETVCMPOS:
	{
		//set lens pos
		unsigned int i = 0;
		ab_lens_info = arg;
		spin_lock(&g_vcm_SpinLock);
		if(Open_holder != CAM_HOLD)
		Open_holder = ab_lens_info->holder;
		spin_unlock(&g_vcm_SpinLock);

		//Segmented set pos for VIB ioctl VCM
		af_len = g_vcmconfig.origin_focus_pos - SUIT_POS;
		i = af_len / SEGMENT_LENGTH;
		for(;i > 0; i--)
		{
			af_len = SUIT_POS + i * SEGMENT_LENGTH;
			if(af_len > g_vcmconfig.origin_focus_pos)
				af_len = g_vcmconfig.origin_focus_pos;

			ret = main2_vcm_set_position(ab_main2_vcm, af_len);
		}
		LOG_AFNE("MTK_S_SETVCMPOS, target pos=0x%x, ret=%d. Open_holder=0x0%x\n", af_len, ret, Close_holder);
		return ret;
	}
        break;
        case VIDIOC_MTK_S_GETVCMSUBDEV:
	{
                LOG_AFNE("Found main2 vcm dev \n");
		return 0;
        }
        break;

#endif
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static const struct v4l2_subdev_internal_ops main2_vcm_int_ops = {
	.open = main2_vcm_open,
	.close = main2_vcm_close,
};

static const struct v4l2_subdev_core_ops main2_vcm_ops_core = {
	.ioctl = main2_vcm_ops_core_ioctl,
};

static const struct v4l2_subdev_ops main2_vcm_ops = {
	.core = &main2_vcm_ops_core,
};

static void main2_vcm_subdev_cleanup(struct main2_vcm_device *main2_vcm)
{
	v4l2_async_unregister_subdev(&main2_vcm->sd);
	v4l2_ctrl_handler_free(&main2_vcm->ctrls);
#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&main2_vcm->sd.entity);
#endif
}

static int main2_vcm_init_controls(struct main2_vcm_device *main2_vcm)
{
	struct v4l2_ctrl_handler *hdl = &main2_vcm->ctrls;
	const struct v4l2_ctrl_ops *ops = &main2_vcm_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	main2_vcm->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, MAIN2_VCM_MAX_FOCUS_POS, MAIN2_VCM_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	main2_vcm->sd.ctrl_handler = hdl;

	return 0;
}

static int main2_vcm_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct main2_vcm_device *main2_vcm;
	int ret;
	int ldo_num;
	int i;

	LOG_INF("+\n");

	main2_vcm = devm_kzalloc(dev, sizeof(*main2_vcm), GFP_KERNEL);
	if (!main2_vcm)
		return -ENOMEM;

	ldo_num = ARRAY_SIZE(ldo_names);
	if (ldo_num > REGULATOR_MAXSIZE)
		ldo_num = REGULATOR_MAXSIZE;
	for (i = 0; i < ldo_num; i++) {
		main2_vcm->ldo[i] = devm_regulator_get(dev, ldo_names[i]);
		if (IS_ERR(main2_vcm->ldo[i])) {
			LOG_INF("cannot get %s regulator\n", ldo_names[i]);
			main2_vcm->ldo[i] = NULL;
		}
	}

	main2_vcm->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(main2_vcm->vcamaf_pinctrl)) {
		ret = PTR_ERR(main2_vcm->vcamaf_pinctrl);
		main2_vcm->vcamaf_pinctrl = NULL;
		LOG_INF("cannot get pinctrl\n");
	} else {
		main2_vcm->vcamaf_on = pinctrl_lookup_state(
			main2_vcm->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(main2_vcm->vcamaf_on)) {
			ret = PTR_ERR(main2_vcm->vcamaf_on);
			main2_vcm->vcamaf_on = NULL;
			LOG_INF("cannot get vcamaf_on pinctrl\n");
		}

		main2_vcm->vcamaf_off = pinctrl_lookup_state(
			main2_vcm->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(main2_vcm->vcamaf_off)) {
			ret = PTR_ERR(main2_vcm->vcamaf_off);
			main2_vcm->vcamaf_off = NULL;
			LOG_INF("cannot get vcamaf_off pinctrl\n");
		}
	}

	v4l2_i2c_subdev_init(&main2_vcm->sd, client, &main2_vcm_ops);
	main2_vcm->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	main2_vcm->sd.internal_ops = &main2_vcm_int_ops;

	ret = main2_vcm_init_controls(main2_vcm);
	if (ret)
		goto err_cleanup;

#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&main2_vcm->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	main2_vcm->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&main2_vcm->sd);
	if (ret < 0)
		goto err_cleanup;

	return 0;

err_cleanup:
	main2_vcm_subdev_cleanup(main2_vcm);
	return ret;
}

static void main2_vcm_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct main2_vcm_device *main2_vcm = sd_to_main2_vcm_vcm(sd);

	LOG_INF("+\n");

	main2_vcm_subdev_cleanup(main2_vcm);
}

static const struct i2c_device_id main2_vcm_id_table[] = {
	{ MAIN2_VCM_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, main2_vcm_id_table);

static const struct of_device_id main2_vcm_of_table[] = {
	{ .compatible = "mediatek,main2_vcm" },
	{ },
};
MODULE_DEVICE_TABLE(of, main2_vcm_of_table);

static struct i2c_driver main2_vcm_i2c_driver = {
	.driver = {
		.name = MAIN2_VCM_NAME,
		.of_match_table = main2_vcm_of_table,
	},
	.probe_new  = main2_vcm_probe,
	.remove = main2_vcm_remove,
	.id_table = main2_vcm_id_table,
};

module_i2c_driver(main2_vcm_i2c_driver);

MODULE_AUTHOR("Po-Hao Huang <Po-Hao.Huang@mediatek.com>");
MODULE_DESCRIPTION("MAIN2_VCM VCM driver");
MODULE_LICENSE("GPL");
