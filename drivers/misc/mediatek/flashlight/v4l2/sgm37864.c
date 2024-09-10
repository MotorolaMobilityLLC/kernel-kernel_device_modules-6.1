// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/videodev2.h>
#include <linux/pinctrl/consumer.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/pm_runtime.h>
#include <linux/thermal.h>

#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT)
#include "flashlight-core.h"

#include <linux/power_supply.h>
#endif

#define SGM37864_NAME	"sgm37864"
#define SGM37864_I2C_ADDR	(0x63)

#define SGM37864_DUAL_LED	FALSE

/* registers definitions */
#define REG_ENABLE		0x01
#define REG_IVFM		0x02
#define REG_LED0_FLASH_BR	0x03
#define REG_LED1_FLASH_BR	0x04
#define REG_LED0_TORCH_BR	0x05
#define REG_LED1_TORCH_BR	0x06
#define REG_BOOST_CFG   	0x07
#define REG_FLASH_TOUT		0x08
#define REG_TEMP		0x09
#define REG_FLAG1		0x0A
#define REG_FLAG2		0x0B
#define REG_DEVICE_ID		0x0C
#define REG_LAST_FLASH		0x0D

#define SGM37864_BIT_SOFT_RST_MASK       (0x80)
#define SGM37864_BIT_SOFT_RST_ENABLE     (1<<7)
#define SGM37864_BIT_SOFT_RST_DISABLE    (0<<7)

/* fault mask */
#define FAULT_TIMEOUT	(1<<0)
#define FAULT_THERMAL_SHUTDOWN	(1<<2)
#define FAULT_LED0_SHORT_CIRCUIT	(1<<5)

/* VALUE DEFINE*/
#define SGM37864_DEVICE_ID 				(0x11)
#define SGM37864_ENABLE_LED_TORCH 		(0x08)
#define SGM37864_ENABLE_LED_FLASH 		(0x0C)
#define SGM37864_FLASH_TIMEOUT 			(0x0A)
#define SGM37864_TORCH_RAMP_TIME   		(0x10)

/*  FLASH Brightness
 *	min 3910uA, step 7830uA, max 2000000uA
 */
#define SGM37864_FLASH_BRT_MIN 3910
#define SGM37864_FLASH_BRT_STEP 7830
#define SGM37864_FLASH_BRT_MAX 2000000
#define SGM37864_FLASH_BRT_uA_TO_REG(a)	\
	((a) < SGM37864_FLASH_BRT_MIN ? 0 :	\
	 (((a) - SGM37864_FLASH_BRT_MIN) / SGM37864_FLASH_BRT_STEP))
#define SGM37864_FLASH_BRT_REG_TO_uA(a)		\
	((a) * SGM37864_FLASH_BRT_STEP + SGM37864_FLASH_BRT_MIN)

/*  FLASH TIMEOUT DURATION
 *	min 40ms, step 40ms, max 1600ms
 */
#define SGM37864_FLASH_TOUT_MIN 40
#define SGM37864_FLASH_TOUT_STEP 40
#define SGM37864_FLASH_TOUT_MAX 280

/*  TORCH BRT
 *	min 980uA, step 1960uA, max 500000uA
 */
#define SGM37864_TORCH_BRT_MIN 980
#define SGM37864_TORCH_BRT_STEP 1960
#define SGM37864_TORCH_BRT_MAX 500000
#define SGM37864_TORCH_BRT_uA_TO_REG(a)	\
	((a) < SGM37864_TORCH_BRT_MIN ? 0 :	\
	 (((a) - SGM37864_TORCH_BRT_MIN) / SGM37864_TORCH_BRT_STEP))
#define SGM37864_TORCH_BRT_REG_TO_uA(a)		\
	((a) * SGM37864_TORCH_BRT_STEP + SGM37864_TORCH_BRT_MIN)

#define SGM37864_COOLER_MAX_STATE 5
static const int flash_state_to_current_limit[SGM37864_COOLER_MAX_STATE] = {
	100000, 80000, 60000, 40000, 20000
};

enum sgm37864_led_id {
	SGM37864_LED0 = 0,
#if SGM37864_DUAL_LED
	SGM37864_LED1,
#endif
	SGM37864_LED_MAX
};

static int sgm37864_chip_id = -1;

/* struct sgm37864_platform_data
 *
 * @max_flash_timeout: flash timeout
 * @max_flash_brt: flash mode led brightness
 * @max_torch_brt: torch mode led brightness
 */
struct sgm37864_platform_data {
	u32 max_flash_timeout;
	u32 max_flash_brt[SGM37864_LED_MAX];
	u32 max_torch_brt[SGM37864_LED_MAX];
};


enum led_enable {
	MODE_SHDN = 0x0,
	MODE_TORCH = SGM37864_ENABLE_LED_TORCH,
	MODE_FLASH = SGM37864_ENABLE_LED_FLASH,
};

/**
 * struct sgm37864_flash
 *
 * @dev: pointer to &struct device
 * @pdata: platform data
 * @regmap: reg. map for i2c
 * @lock: muxtex for serial access.
 * @led_mode: V4L2 LED mode
 * @ctrls_led: V4L2 controls
 * @subdev_led: V4L2 subdev
 */
struct sgm37864_flash {
	struct device *dev;
	struct sgm37864_platform_data *pdata;
	struct regmap *regmap;
	struct mutex lock;

	enum v4l2_flash_led_mode led_mode;
	struct v4l2_ctrl_handler ctrls_led[SGM37864_LED_MAX];
	struct v4l2_subdev subdev_led[SGM37864_LED_MAX];
	struct device_node *dnode[SGM37864_LED_MAX];
	struct pinctrl *sgm37864_hwen_pinctrl;
	struct pinctrl_state *sgm37864_hwen_high;
	struct pinctrl_state *sgm37864_hwen_low;
#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT)
	struct flashlight_device_id flash_dev_id[SGM37864_LED_MAX];
#endif
	struct thermal_cooling_device *cdev;
	int need_cooler;
	unsigned long max_state;
	unsigned long target_state;
	unsigned long target_current[SGM37864_LED_MAX];
	unsigned long ori_current[SGM37864_LED_MAX];
};

/* define usage count */
static int use_count;

static struct sgm37864_flash *sgm37864_flash_data;

#define to_sgm37864_flash(_ctrl, _no)	\
	container_of(_ctrl->handler, struct sgm37864_flash, ctrls_led[_no])

static int sgm37864_set_driver(int set);

/* define pinctrl */
#define SGM37864_PINCTRL_PIN_HWEN 0
#define SGM37864_PINCTRL_PINSTATE_LOW 0
#define SGM37864_PINCTRL_PINSTATE_HIGH 1
#define SGM37864_PINCTRL_STATE_HWEN_HIGH "hwen-high"
#define SGM37864_PINCTRL_STATE_HWEN_LOW  "hwen-low"
/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int sgm37864_pinctrl_init(struct sgm37864_flash *flash)
{
	int ret = 0;

	/* get pinctrl */
	flash->sgm37864_hwen_pinctrl = devm_pinctrl_get(flash->dev);
	if (IS_ERR(flash->sgm37864_hwen_pinctrl)) {
		pr_info("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(flash->sgm37864_hwen_pinctrl);
		return ret;
	}

	/* Flashlight HWEN pin initialization */
	flash->sgm37864_hwen_high = pinctrl_lookup_state(
			flash->sgm37864_hwen_pinctrl,
			SGM37864_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(flash->sgm37864_hwen_high)) {
		pr_info("Failed to init (%s)\n",
			SGM37864_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(flash->sgm37864_hwen_high);
	}
	flash->sgm37864_hwen_low = pinctrl_lookup_state(
			flash->sgm37864_hwen_pinctrl,
			SGM37864_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(flash->sgm37864_hwen_low)) {
		pr_info("Failed to init (%s)\n", SGM37864_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(flash->sgm37864_hwen_low);
	}

	return ret;
}

static int sgm37864_pinctrl_set(struct sgm37864_flash *flash, int pin, int state)
{
	int ret = 0;

	if (IS_ERR(flash->sgm37864_hwen_pinctrl)) {
		pr_info("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case SGM37864_PINCTRL_PIN_HWEN:
		if (state == SGM37864_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(flash->sgm37864_hwen_low))
			pinctrl_select_state(flash->sgm37864_hwen_pinctrl,
					flash->sgm37864_hwen_low);
		else if (state == SGM37864_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(flash->sgm37864_hwen_high))
			pinctrl_select_state(flash->sgm37864_hwen_pinctrl,
					flash->sgm37864_hwen_high);
		else
			pr_info("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_info("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}

	return ret;
}

/* enable mode control */
static int sgm37864_mode_ctrl(struct sgm37864_flash *flash)
{
	int rval = -EINVAL;

	pr_info_ratelimited("%s mode:%d", __func__, flash->led_mode);
	switch (flash->led_mode) {
	case V4L2_FLASH_LED_MODE_NONE:
		rval = regmap_update_bits(flash->regmap,
					  REG_ENABLE, 0x0C, MODE_SHDN);
		break;
	case V4L2_FLASH_LED_MODE_TORCH:
		rval = regmap_update_bits(flash->regmap,
					  REG_ENABLE, 0x0C, MODE_TORCH);
		break;
	case V4L2_FLASH_LED_MODE_FLASH:
		rval = regmap_update_bits(flash->regmap,
					  REG_ENABLE, 0x0C, MODE_FLASH);
		break;
	}
	return rval;
}

/* led1 enable/disable */
static int sgm37864_enable_ctrl(struct sgm37864_flash *flash,
			      enum sgm37864_led_id led_no, bool on)
{
	int rval;

	pr_info_ratelimited("%s led:%d enable:%d", __func__, led_no, on);

#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT_DLPT)
	flashlight_kicker_pbm(on);
	if (flashlight_pt_is_low()) {
		pr_info_ratelimited("pt is low\n");
		return 0;
	}
#endif

	if (led_no == SGM37864_LED0) {
		if (on)
			rval = regmap_update_bits(flash->regmap,
						  REG_ENABLE, 0x01, 0x01);
		else
			rval = regmap_update_bits(flash->regmap,
						  REG_ENABLE, 0x01, 0x00);
	} else {
		if (on)
			rval = regmap_update_bits(flash->regmap,
						  REG_ENABLE, 0x02, 0x02);
		else
			rval = regmap_update_bits(flash->regmap,
						  REG_ENABLE, 0x02, 0x00);
	}

	return rval;
}

/* torch1/2 brightness control */
static int sgm37864_torch_brt_ctrl(struct sgm37864_flash *flash,
				 enum sgm37864_led_id led_no, unsigned int brt)
{
	int rval;
	u8 br_bits;

	pr_info_ratelimited("%s %d brt:%u\n", __func__, led_no, brt);
	if (brt < SGM37864_TORCH_BRT_MIN)
		return sgm37864_enable_ctrl(flash, led_no, false);

	if (flash->need_cooler == 0) {
		flash->ori_current[led_no] = brt;
	} else {
		if (brt > flash->target_current[led_no]) {
			brt = flash->target_current[led_no];
			pr_info("thermal limit current:%d\n", brt);
		}
	}

	br_bits = SGM37864_TORCH_BRT_uA_TO_REG(brt);

#if SGM37864_DUAL_LED
	if (led_no == SGM37864_LED0)
		rval = regmap_update_bits(flash->regmap,
					  REG_LED0_TORCH_BR, 0x7f, br_bits);
	else
		rval = regmap_update_bits(flash->regmap,
					  REG_LED1_TORCH_BR, 0x7f, br_bits);
#else
	rval = regmap_update_bits(flash->regmap,
					  REG_LED0_TORCH_BR, 0xff, br_bits);
#endif

	return rval;
}

/* flash1/2 brightness control */
static int sgm37864_flash_brt_ctrl(struct sgm37864_flash *flash,
				 enum sgm37864_led_id led_no, unsigned int brt)
{
	int rval;
	u8 br_bits;

	pr_info_ratelimited("%s %d brt:%u", __func__, led_no, brt);
	if (brt < SGM37864_FLASH_BRT_MIN)
		return sgm37864_enable_ctrl(flash, led_no, false);

	if (flash->need_cooler == 1 && brt > flash->target_current[led_no]) {
		brt = flash->target_current[led_no];
		pr_info("thermal limit current:%d\n", brt);
	}

	br_bits = SGM37864_FLASH_BRT_uA_TO_REG(brt);

#if SGM37864_DUAL_LED
	if (led_no == SGM37864_LED0)
		rval = regmap_update_bits(flash->regmap,
					  REG_LED0_FLASH_BR, 0x7f, br_bits);
	else
		rval = regmap_update_bits(flash->regmap,
					  REG_LED1_FLASH_BR, 0x7f, br_bits);
#else
	rval = regmap_update_bits(flash->regmap,
				REG_LED0_FLASH_BR, 0xff, br_bits);
#endif

	return rval;
}

/* flash1 timeout control */
static int sgm37864_flash_tout_ctrl(struct sgm37864_flash *flash,
				unsigned int tout)
{
	int rval;
	u8 tout_bits;
	pr_info_ratelimited("%s timeout=%d", __func__, tout);
  if (tout <= SGM37864_FLASH_TOUT_MIN)
  {
		tout_bits = 0x00;   //The register value represents 40ms
	}
	else if (tout <= SGM37864_FLASH_TOUT_MAX)
	{
		tout_bits = (tout / SGM37864_FLASH_TOUT_STEP)-1;
	}
	else
	{
		tout_bits = 0x06;  //The register value represents 280ms
	}
	rval = regmap_update_bits(flash->regmap,
				  REG_FLASH_TOUT, 0x0f, tout_bits);

	return rval;
}

/* v4l2 controls  */
static int sgm37864_get_ctrl(struct v4l2_ctrl *ctrl, enum sgm37864_led_id led_no)
{
	struct sgm37864_flash *flash = to_sgm37864_flash(ctrl, led_no);
	int rval = -EINVAL;

	mutex_lock(&flash->lock);

	if (ctrl->id == V4L2_CID_FLASH_FAULT) {
		s32 fault = 0;
		unsigned int reg_val = 0;

		rval = regmap_read(flash->regmap, REG_FLAG1, &reg_val);
		if (rval < 0)
			goto out;
		if (reg_val & FAULT_LED0_SHORT_CIRCUIT)
			fault |= V4L2_FLASH_FAULT_SHORT_CIRCUIT;
		if (reg_val & FAULT_THERMAL_SHUTDOWN)
			fault |= V4L2_FLASH_FAULT_OVER_TEMPERATURE;
		if (reg_val & FAULT_TIMEOUT)
			fault |= V4L2_FLASH_FAULT_TIMEOUT;
		ctrl->cur.val = fault;
	}

out:
	mutex_unlock(&flash->lock);
	return rval;
}

static int sgm37864_set_ctrl(struct v4l2_ctrl *ctrl, enum sgm37864_led_id led_no)
{
	struct sgm37864_flash *flash = to_sgm37864_flash(ctrl, led_no);
	int rval = -EINVAL;

	pr_info("%s led:%d ID:%d", __func__, led_no, ctrl->id);
	mutex_lock(&flash->lock);

	switch (ctrl->id) {
	case V4L2_CID_FLASH_LED_MODE:
		flash->led_mode = ctrl->val;
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH)
			rval = sgm37864_mode_ctrl(flash);
		else
			rval = 0;
		if (flash->led_mode == V4L2_FLASH_LED_MODE_NONE)
			sgm37864_enable_ctrl(flash, led_no, false);
		else if (flash->led_mode == V4L2_FLASH_LED_MODE_TORCH)
			rval = sgm37864_enable_ctrl(flash, led_no, true);
		break;

	case V4L2_CID_FLASH_STROBE_SOURCE:
		if (ctrl->val == V4L2_FLASH_STROBE_SOURCE_SOFTWARE) {
			pr_info("sw ctrl\n");
			rval = regmap_update_bits(flash->regmap,
					REG_ENABLE, 0x2C, 0x00);
		} else if (ctrl->val == V4L2_FLASH_STROBE_SOURCE_EXTERNAL) {
			pr_info("hw trigger\n");
			rval = regmap_update_bits(flash->regmap,
					REG_ENABLE, 0x2C, 0x24);
			rval = sgm37864_enable_ctrl(flash, led_no, true);
		}
		if (rval < 0)
			goto err_out;
		break;

	case V4L2_CID_FLASH_STROBE:
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH) {
			rval = -EBUSY;
			goto err_out;
		}
		flash->led_mode = V4L2_FLASH_LED_MODE_FLASH;
		rval = sgm37864_mode_ctrl(flash);
		rval = sgm37864_enable_ctrl(flash, led_no, true);
		break;

	case V4L2_CID_FLASH_STROBE_STOP:
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH) {
			rval = -EBUSY;
			goto err_out;
		}
		sgm37864_enable_ctrl(flash, led_no, false);
		flash->led_mode = V4L2_FLASH_LED_MODE_NONE;
		rval = sgm37864_mode_ctrl(flash);
		break;

	case V4L2_CID_FLASH_TIMEOUT:
		rval = sgm37864_flash_tout_ctrl(flash, ctrl->val);
		break;

	case V4L2_CID_FLASH_INTENSITY:
		rval = sgm37864_flash_brt_ctrl(flash, led_no, ctrl->val);
		break;

	case V4L2_CID_FLASH_TORCH_INTENSITY:
		rval = sgm37864_torch_brt_ctrl(flash, led_no, ctrl->val);
		break;
	}

err_out:
	mutex_unlock(&flash->lock);
	return rval;
}

#if SGM37864_DUAL_LED
static int sgm37864_led1_get_ctrl(struct v4l2_ctrl *ctrl)
{
	return sgm37864_get_ctrl(ctrl, SGM37864_LED1);
}

static int sgm37864_led1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return sgm37864_set_ctrl(ctrl, SGM37864_LED1);
}
#endif

static int sgm37864_led0_get_ctrl(struct v4l2_ctrl *ctrl)
{
	return sgm37864_get_ctrl(ctrl, SGM37864_LED0);
}

static int sgm37864_led0_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return sgm37864_set_ctrl(ctrl, SGM37864_LED0);
}

static const struct v4l2_ctrl_ops sgm37864_led_ctrl_ops[SGM37864_LED_MAX] = {
	[SGM37864_LED0] = {
			.g_volatile_ctrl = sgm37864_led0_get_ctrl,
			.s_ctrl = sgm37864_led0_set_ctrl,
			},
#if SGM37864_DUAL_LED
	[SGM37864_LED1] = {
			.g_volatile_ctrl = sgm37864_led1_get_ctrl,
			.s_ctrl = sgm37864_led1_set_ctrl,
			},
#endif
};

static int sgm37864_init_controls(struct sgm37864_flash *flash,
				enum sgm37864_led_id led_no)
{
	struct v4l2_ctrl *fault;
	u32 max_flash_brt = flash->pdata->max_flash_brt[led_no];
	u32 max_torch_brt = flash->pdata->max_torch_brt[led_no];
	struct v4l2_ctrl_handler *hdl = &flash->ctrls_led[led_no];
	const struct v4l2_ctrl_ops *ops = &sgm37864_led_ctrl_ops[led_no];

	v4l2_ctrl_handler_init(hdl, 8);

	/* flash mode */
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_FLASH_LED_MODE,
			       V4L2_FLASH_LED_MODE_TORCH, ~0x7,
			       V4L2_FLASH_LED_MODE_NONE);
	flash->led_mode = V4L2_FLASH_LED_MODE_NONE;

	/* flash source */
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_FLASH_STROBE_SOURCE,
			       0x1, ~0x3, V4L2_FLASH_STROBE_SOURCE_SOFTWARE);

	/* flash strobe */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_STROBE, 0, 0, 0, 0);

	/* flash strobe stop */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_STROBE_STOP, 0, 0, 0, 0);

	/* flash strobe timeout */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TIMEOUT,
			  SGM37864_FLASH_TOUT_MIN,
			  flash->pdata->max_flash_timeout,
			  SGM37864_FLASH_TOUT_STEP,
			  flash->pdata->max_flash_timeout);

	/* flash brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_INTENSITY,
			  SGM37864_FLASH_BRT_MIN, max_flash_brt,
			  SGM37864_FLASH_BRT_STEP, max_flash_brt);

	/* torch brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TORCH_INTENSITY,
			  SGM37864_TORCH_BRT_MIN, max_torch_brt,
			  SGM37864_TORCH_BRT_STEP, max_torch_brt);

	/* fault */
	fault = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_FAULT, 0,
				  V4L2_FLASH_FAULT_OVER_VOLTAGE
				  | V4L2_FLASH_FAULT_OVER_TEMPERATURE
				  | V4L2_FLASH_FAULT_SHORT_CIRCUIT
				  | V4L2_FLASH_FAULT_TIMEOUT, 0, 0);
	if (fault != NULL)
		fault->flags |= V4L2_CTRL_FLAG_VOLATILE;

	if (hdl->error)
		return hdl->error;

	flash->subdev_led[led_no].ctrl_handler = hdl;
	return 0;
}

/* initialize device */
static const struct v4l2_subdev_ops sgm37864_ops = {
	.core = NULL,
};

static const struct regmap_config sgm37864_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

static void sgm37864_v4l2_i2c_subdev_init(struct v4l2_subdev *sd,
		struct i2c_client *client,
		const struct v4l2_subdev_ops *ops)
{
	v4l2_subdev_init(sd, ops);
	sd->flags |= V4L2_SUBDEV_FL_IS_I2C;
	/* the owner is the same as the i2c_client's driver owner */
	sd->owner = client->dev.driver->owner;
	sd->dev = &client->dev;
	/* i2c_client and v4l2_subdev point to one another */
	v4l2_set_subdevdata(sd, client);
	i2c_set_clientdata(client, sd);
	/* initialize name */
	snprintf(sd->name, sizeof(sd->name), "%s %d-%04x",
		client->dev.driver->name, i2c_adapter_id(client->adapter),
		client->addr);
}

static int sgm37864_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	pr_info("%s\n", __func__);

	ret = pm_runtime_get_sync(sd->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(sd->dev);
		return ret;
	}

	return 0;
}

static int sgm37864_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	regmap_update_bits(sgm37864_flash_data->regmap, REG_FLAG2, 0x0f, 0x00);
	pr_info("%s\n", __func__);

	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops sgm37864_int_ops = {
	.open = sgm37864_open,
	.close = sgm37864_close,
};

static int sgm37864_subdev_init(struct sgm37864_flash *flash,
			      enum sgm37864_led_id led_no, char *led_name)
{
	struct i2c_client *client = to_i2c_client(flash->dev);
	struct device_node *np = flash->dev->of_node, *child;
	const char *fled_name = "flash";
	int rval;

	// pr_info("%s %d", __func__, led_no);

	sgm37864_v4l2_i2c_subdev_init(&flash->subdev_led[led_no],
				client, &sgm37864_ops);
	flash->subdev_led[led_no].flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	flash->subdev_led[led_no].internal_ops = &sgm37864_int_ops;
	strscpy(flash->subdev_led[led_no].name, led_name,
		sizeof(flash->subdev_led[led_no].name));

	for (child = of_get_child_by_name(np, fled_name); child;
			child = of_find_node_by_name(child, fled_name)) {
		int rv;
		u32 reg = 0;

		rv = of_property_read_u32(child, "reg", &reg);
		if (rv)
			continue;

		if (reg == led_no) {
			flash->dnode[led_no] = child;
			flash->subdev_led[led_no].fwnode =
				of_fwnode_handle(flash->dnode[led_no]);
		}
	}

	rval = sgm37864_init_controls(flash, led_no);
	if (rval)
		goto err_out;
	rval = media_entity_pads_init(&flash->subdev_led[led_no].entity, 0, NULL);
	if (rval < 0)
		goto err_out;
	flash->subdev_led[led_no].entity.function = MEDIA_ENT_F_FLASH;

	rval = v4l2_async_register_subdev(&flash->subdev_led[led_no]);
	if (rval < 0)
		goto err_out;

	return rval;

err_out:
	v4l2_ctrl_handler_free(&flash->ctrls_led[led_no]);
	return rval;
}

/* flashlight init */
static int sgm37864_init(struct sgm37864_flash *flash)
{
	int rval = 0;
	unsigned int reg_val;

	sgm37864_pinctrl_set(flash, SGM37864_PINCTRL_PIN_HWEN, SGM37864_PINCTRL_PINSTATE_HIGH);
  mdelay(1);

	/* set timeout */
	rval = sgm37864_flash_tout_ctrl(flash, SGM37864_FLASH_TOUT_MAX);
	if (rval < 0)
		return rval;

	/* output disable */
	flash->led_mode = V4L2_FLASH_LED_MODE_NONE;
	rval = sgm37864_mode_ctrl(flash);
	if (rval < 0)
		return rval;

	sgm37864_torch_brt_ctrl(flash, SGM37864_LED0,
				flash->ori_current[SGM37864_LED0]);
	sgm37864_flash_brt_ctrl(flash, SGM37864_LED0,
				flash->ori_current[SGM37864_LED0]);
#if SGM37864_DUAL_LED
	sgm37864_flash_brt_ctrl(flash, SGM37864_LED1,
				flash->ori_current[SGM37864_LED1]);
	sgm37864_torch_brt_ctrl(flash, SGM37864_LED1,
				flash->ori_current[SGM37864_LED1]);
#endif

	/* reset faults */
	rval = regmap_read(flash->regmap, REG_FLAG1, &reg_val);
	return rval;
}

/* flashlight uninit */
static int sgm37864_uninit(struct sgm37864_flash *flash)
{
	sgm37864_pinctrl_set(flash, SGM37864_PINCTRL_PIN_HWEN, SGM37864_PINCTRL_PINSTATE_LOW);
	return 0;
}

static int sgm37864_flash_open(void)
{
	return 0;
}

static int sgm37864_flash_release(void)
{
	return 0;
}

static int sgm37864_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_ONOFF:
		pr_info_ratelimited("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if ((int)fl_arg->arg) {
			sgm37864_torch_brt_ctrl(sgm37864_flash_data, channel, 25000);
			sgm37864_flash_data->led_mode = V4L2_FLASH_LED_MODE_TORCH;
			sgm37864_mode_ctrl(sgm37864_flash_data);
			sgm37864_enable_ctrl(sgm37864_flash_data, channel, true);
		} else {
			if (sgm37864_flash_data->led_mode != V4L2_FLASH_LED_MODE_NONE) {
				sgm37864_flash_data->led_mode = V4L2_FLASH_LED_MODE_NONE;
				sgm37864_mode_ctrl(sgm37864_flash_data);
				sgm37864_enable_ctrl(sgm37864_flash_data, channel, false);
			}
		}
		break;
	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int sgm37864_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	//mutex_lock(&sgm37864_mutex);
	if (set) {
		if (!use_count)
			ret = sgm37864_init(sgm37864_flash_data);
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = sgm37864_uninit(sgm37864_flash_data);
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	//mutex_unlock(&sgm37864_mutex);

	return 0;
}

static ssize_t sgm37864_strobe_store(struct flashlight_arg arg)
{
	pr_info("%s:%d", __func__, __LINE__);
	sgm37864_set_driver(1);
	//sgm37864_set_level(arg.channel, arg.level);
	//sgm37864_timeout_ms[arg.channel] = 0;
	//sgm37864_enable(arg.channel);
	sgm37864_torch_brt_ctrl(sgm37864_flash_data, arg.channel,
				arg.level * 25000);
	sgm37864_enable_ctrl(sgm37864_flash_data, arg.channel, true);
	sgm37864_flash_data->led_mode = V4L2_FLASH_LED_MODE_TORCH;
	sgm37864_mode_ctrl(sgm37864_flash_data);
	msleep(arg.dur);
	//sgm37864_disable(arg.channel);
	sgm37864_flash_data->led_mode = V4L2_FLASH_LED_MODE_NONE;
	sgm37864_mode_ctrl(sgm37864_flash_data);
	sgm37864_enable_ctrl(sgm37864_flash_data, arg.channel, false);
	sgm37864_set_driver(0);
	pr_info("%s:%d", __func__, __LINE__);
	return 0;
}

static int sgm37864_cooling_get_max_state(struct thermal_cooling_device *cdev,
					unsigned long *state)
{
	struct sgm37864_flash *flash = cdev->devdata;

	*state = flash->max_state;

	return 0;
}

static int sgm37864_cooling_get_cur_state(struct thermal_cooling_device *cdev,
					unsigned long *state)
{
	struct sgm37864_flash *flash = cdev->devdata;

	*state = flash->target_state;

	return 0;
}

static int sgm37864_cooling_set_cur_state(struct thermal_cooling_device *cdev,
					unsigned long state)
{
	struct sgm37864_flash *flash = cdev->devdata;
	int ret = 0;

	/* Request state should be less than max_state */
	if (state > flash->max_state)
		state = flash->max_state;
	if (state < 0)
		state = 0;

	if (flash->target_state == state)
		return 0;

	flash->target_state = state;
//	pr_info("set thermal current:%d\n", flash->target_state);

	if (flash->target_state == 0) {
		flash->need_cooler = 0;
		flash->target_current[SGM37864_LED0] = SGM37864_FLASH_BRT_MAX;
#if SGM37864_DUAL_LED
		flash->target_current[SGM37864_LED1] = SGM37864_FLASH_BRT_MAX;
#endif
		ret = sgm37864_torch_brt_ctrl(flash, SGM37864_LED0,
						flash->ori_current[SGM37864_LED0]);
#if SGM37864_DUAL_LED
		ret = sgm37864_torch_brt_ctrl(flash, SGM37864_LED1,
						flash->ori_current[SGM37864_LED1]);
#endif
	} else {
		flash->need_cooler = 1;
		flash->target_current[SGM37864_LED0] =
			flash_state_to_current_limit[flash->target_state - 1];
#if SGM37864_DUAL_LED
		flash->target_current[SGM37864_LED1] =
			flash_state_to_current_limit[flash->target_state - 1];
#endif
		ret = sgm37864_torch_brt_ctrl(flash, SGM37864_LED0,
					flash->target_current[SGM37864_LED0]);
#if SGM37864_DUAL_LED
		ret = sgm37864_torch_brt_ctrl(flash, SGM37864_LED1,
					flash->target_current[SGM37864_LED1]);
#endif
	}
	return ret;
}

static struct thermal_cooling_device_ops sgm37864_cooling_ops = {
	.get_max_state		= sgm37864_cooling_get_max_state,
	.get_cur_state		= sgm37864_cooling_get_cur_state,
	.set_cur_state		= sgm37864_cooling_set_cur_state,
};

static struct flashlight_operations sgm37864_flash_ops = {
	sgm37864_flash_open,
	sgm37864_flash_release,
	sgm37864_ioctl,
	sgm37864_strobe_store,
	sgm37864_set_driver
};

static int sgm37864_parse_dt(struct sgm37864_flash *flash)
{
	struct device_node *np, *cnp;
	struct device *dev = flash->dev;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node)
		return -ENODEV;

	np = dev->of_node;
	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type",
					&flash->flash_dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp,
					"ct", &flash->flash_dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp,
					"part", &flash->flash_dev_id[i].part))
			goto err_node_put;
		snprintf(flash->flash_dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				flash->subdev_led[i].name);
		flash->flash_dev_id[i].channel = i;
		flash->flash_dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				flash->flash_dev_id[i].type,
				flash->flash_dev_id[i].ct,
				flash->flash_dev_id[i].part,
				flash->flash_dev_id[i].name,
				flash->flash_dev_id[i].channel,
				flash->flash_dev_id[i].decouple);
		if (flashlight_dev_register_by_device_id(&flash->flash_dev_id[i],
			&sgm37864_flash_ops))
			return -EFAULT;
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int sgm37864_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	struct sgm37864_flash *flash;
	struct sgm37864_platform_data *pdata = dev_get_platdata(&client->dev);
	int rval;

	pr_info("%s:%d", __func__, __LINE__);

	flash = devm_kzalloc(&client->dev, sizeof(*flash), GFP_KERNEL);
	if (flash == NULL)
		return -ENOMEM;

	/* if there is no platform data, use chip default value */
	if (pdata == NULL) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL)
			return -ENODEV;
		pdata->max_flash_timeout = SGM37864_FLASH_TOUT_MAX;
		/* led 1 */
		pdata->max_flash_brt[SGM37864_LED0] = SGM37864_FLASH_BRT_MAX;
		pdata->max_torch_brt[SGM37864_LED0] = SGM37864_TORCH_BRT_MAX;
#if SGM37864_DUAL_LED
		/* led 2 */
		pdata->max_flash_brt[SGM37864_LED1] = SGM37864_FLASH_BRT_MAX;
		pdata->max_torch_brt[SGM37864_LED1] = SGM37864_TORCH_BRT_MAX;
#endif
	}
	flash->pdata = pdata;
	flash->dev = &client->dev;

	rval = sgm37864_pinctrl_init(flash);
	if (rval < 0)
		return rval;

	flash->regmap = devm_regmap_init_i2c(client, &sgm37864_regmap);
	if (IS_ERR(flash->regmap)) {
		rval = PTR_ERR(flash->regmap);
		return rval;
	}

	sgm37864_pinctrl_set(flash, SGM37864_PINCTRL_PIN_HWEN, SGM37864_PINCTRL_PINSTATE_HIGH);
	mdelay(1);

  rval = regmap_read(flash->regmap, REG_DEVICE_ID, &sgm37864_chip_id);
  if (SGM37864_DEVICE_ID == sgm37864_chip_id) {
    dev_info(flash->dev, "SGM37864 chip id matched!\n");
  } else {
    dev_err(flash->dev, "SGM37864 read chip id error...chipid=:0x%x, ret:%d\n", sgm37864_chip_id, rval);
    return -1;
  }

	mutex_init(&flash->lock);
	sgm37864_flash_data = flash;

	rval = sgm37864_subdev_init(flash, SGM37864_LED0, "sgm37864-led0");
	if (rval < 0)
		return rval;

#if SGM37864_DUAL_LED
	rval = sgm37864_subdev_init(flash, SGM37864_LED1, "sgm37864-led1");
	if (rval < 0)
		return rval;
#endif

	pm_runtime_enable(flash->dev);

	rval = sgm37864_parse_dt(flash);

	i2c_set_clientdata(client, flash);

	flash->max_state = SGM37864_COOLER_MAX_STATE;
	flash->target_state = 0;
	flash->need_cooler = 0;
	flash->target_current[SGM37864_LED0] = SGM37864_FLASH_BRT_MAX;
#if SGM37864_DUAL_LED
	flash->target_current[SGM37864_LED1] = SGM37864_FLASH_BRT_MAX;
#endif
	flash->ori_current[SGM37864_LED0] = SGM37864_TORCH_BRT_MIN;
#if SGM37864_DUAL_LED
	flash->ori_current[SGM37864_LED1] = SGM37864_TORCH_BRT_MIN;
#endif
	flash->cdev = thermal_of_cooling_device_register(client->dev.of_node,
			"flashlight_cooler", flash, &sgm37864_cooling_ops);
	if (IS_ERR(flash->cdev))
		pr_info("register thermal failed\n");

	pr_info("%s:%d", __func__, __LINE__);
	return 0;
}

static void sgm37864_remove(struct i2c_client *client)
{
	struct sgm37864_flash *flash = i2c_get_clientdata(client);
	unsigned int i;

	thermal_cooling_device_unregister(flash->cdev);
	for (i = SGM37864_LED0; i < SGM37864_LED_MAX; i++) {
		v4l2_device_unregister_subdev(&flash->subdev_led[i]);
		v4l2_ctrl_handler_free(&flash->ctrls_led[i]);
		media_entity_cleanup(&flash->subdev_led[i].entity);
	}

	pm_runtime_disable(&client->dev);

	pm_runtime_set_suspended(&client->dev);
	return;
}
static void sgm37864_shutdown(struct i2c_client *client)
{
	regmap_update_bits(sgm37864_flash_data->regmap, REG_FLAG2, 0x1f, 0x00);
	pr_info_ratelimited("%s", __func__);
	return;
}

static int __maybe_unused sgm37864_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sgm37864_flash *flash = i2c_get_clientdata(client);

	pr_info("%s %d", __func__, __LINE__);
	return sgm37864_uninit(flash);
}

static int __maybe_unused sgm37864_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sgm37864_flash *flash = i2c_get_clientdata(client);

	pr_info("%s %d", __func__, __LINE__);
	return sgm37864_init(flash);
}

static const struct i2c_device_id sgm37864_id_table[] = {
	{SGM37864_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sgm37864_id_table);

static const struct of_device_id sgm37864_of_table[] = {
	{ .compatible = "mediatek,sgm37864" },
	{ },
};
MODULE_DEVICE_TABLE(of, sgm37864_of_table);

static const struct dev_pm_ops sgm37864_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(sgm37864_suspend, sgm37864_resume, NULL)
};

static struct i2c_driver sgm37864_i2c_driver = {
	.driver = {
		   .name = SGM37864_NAME,
		   .pm = &sgm37864_pm_ops,
		   .of_match_table = sgm37864_of_table,
		   },
	.probe = sgm37864_probe,
	.remove = sgm37864_remove,
	.shutdown = sgm37864_shutdown,
	.id_table = sgm37864_id_table,
};

module_i2c_driver(sgm37864_i2c_driver);

MODULE_AUTHOR("Chen Zhiming <chenzm8@lenovo.com>");
MODULE_DESCRIPTION("MOTO sgm37864 LED flash driver");
MODULE_LICENSE("GPL");