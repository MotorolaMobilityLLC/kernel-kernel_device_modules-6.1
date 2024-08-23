// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/usb/typec.h>
#include "fsa4480-i2c.h"
#include <linux/iio/consumer.h>

#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#if IS_ENABLED(CONFIG_MTK_USB_TYPEC_MUX)
#include "mux_switch.h"
#endif

#define FSA4480_I2C_NAME	"fsa4480-driver"

#define FSA4480_SWITCH_SETTINGS 0x04
#define FSA4480_SWITCH_CONTROL  0x05
#define FSA4480_SWITCH_STATUS1  0x07
#define FSA4480_SLOW_L          0x08
#define FSA4480_SLOW_R          0x09
#define FSA4480_SLOW_MIC        0x0A
#define FSA4480_SLOW_SENSE      0x0B
#define FSA4480_SLOW_GND        0x0C
#define FSA4480_DELAY_L_R       0x0D
#define FSA4480_DELAY_L_MIC     0x0E
#define FSA4480_DELAY_L_SENSE   0x0F
#define FSA4480_DELAY_L_AGND    0x10
#define FSA4480_RESET           0x1E

#define FSA4480_ENABLE_DEVICE	BIT(7)
#define FSA4480_ENABLE_SBU	GENMASK(6, 5)
#define FSA4480_ENABLE_USB	GENMASK(4, 3)

#define FSA4480_SEL_SBU_REVERSE	GENMASK(6, 5)
#define FSA4480_SEL_USB		GENMASK(4, 3)

struct fsa4480_priv {
	struct regmap *regmap;
	struct device *dev;
	struct power_supply *usb_psy;
	struct notifier_block nb;
	struct iio_channel *iio_ch;
	atomic_t usbc_mode;
	struct work_struct usbc_analog_work;
	struct blocking_notifier_head fsa4480_notifier;
	struct mutex fsa4480_lock;
	u32 use_powersupply;
	int switch_control;

	struct typec_switch_dev *sw;
	struct typec_mux_dev *mux;
	u8 cur_enable;
	u8 cur_select;
};

struct fsa4480_reg_val {
	u16 reg;
	u8 val;
};

static const struct regmap_config fsa4480_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FSA4480_RESET,
};

static const struct fsa4480_reg_val fsa_reg_i2c_defaults[] = {
	{FSA4480_SLOW_L, 0x00},
	{FSA4480_SLOW_R, 0x00},
	{FSA4480_SLOW_MIC, 0x00},
	{FSA4480_SLOW_SENSE, 0x00},
	{FSA4480_SLOW_GND, 0x00},
	{FSA4480_DELAY_L_R, 0x00},
	{FSA4480_DELAY_L_MIC, 0x00},
	{FSA4480_DELAY_L_SENSE, 0x00},
	{FSA4480_DELAY_L_AGND, 0x09},
#ifdef CONFIG_FSA4480_SENS_TO_GND
	{FSA4480_SWITCH_SETTINGS, 0x9D},
#else
	{FSA4480_SWITCH_SETTINGS, 0x98},
#endif
};

static struct fsa4480_priv *fsa_priv;

static void fsa4480_usbc_update_settings(struct fsa4480_priv *fsa_priv,
		u32 switch_control, u32 switch_enable)
{
	u32 prev_control, prev_enable;

	if (!fsa_priv->regmap) {
		dev_err(fsa_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}

	mutex_lock(&fsa_priv->fsa4480_lock);

	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, &prev_control);
	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, &prev_enable);

	if (prev_control == switch_control && prev_enable == switch_enable) {
		dev_dbg(fsa_priv->dev, "%s: settings unchanged\n", __func__);
		mutex_unlock(&fsa_priv->fsa4480_lock);
		return;
	}

	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x80);
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, switch_control);
	/* FSA4480 chip hardware requirement */
	usleep_range(50, 55);
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, switch_enable);

	fsa_priv->cur_select = switch_control;
	fsa_priv->cur_enable = switch_enable;
	dev_info(fsa_priv->dev, "%s: cur_enable %x, cur_select %x\n",
		__func__, fsa_priv->cur_enable, fsa_priv->cur_select);

	mutex_unlock(&fsa_priv->fsa4480_lock);
}

/*
 * fsa4480_switch_event - configure FSA switch position based on event
 *
 * @node - phandle node to fsa4480 device
 * @event - fsa_function enum
 *
 * Returns int on whether the switch happened or not
 */
int fsa4480_switch_event(enum fsa_function event)
{
	switch (event) {
	case FSA_MIC_GND_SWAP:
		regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL,
				&fsa_priv->switch_control);
		if ((fsa_priv->switch_control & 0x07) == 0x07)
			fsa_priv->switch_control = 0x0;
		else
			fsa_priv->switch_control = 0x7;
		fsa4480_usbc_update_settings(fsa_priv, fsa_priv->switch_control,
					     0x9F);
		break;
	case FSA_TYPEC_ACCESSORY_AUDIO:
		/* activate switches */
		fsa4480_usbc_update_settings(fsa_priv, 0x00, 0x9F);
		break;
	case FSA_TYPEC_ACCESSORY_NONE:
		/* deactivate switches */
#ifdef CONFIG_FSA4480_SENS_TO_GND
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x9D);
#else
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
#endif
		break;
	default:
		break;
	}
	return 0;
}
EXPORT_SYMBOL(fsa4480_switch_event);

static void fsa4480_update_reg_defaults(struct regmap *regmap)
{
	u8 i;

	//MTK plaform use battery as fsa4480 ldo, need sw reset
	regmap_write(regmap, FSA4480_RESET,0x01);
	msleep(5);

	for (i = 0; i < ARRAY_SIZE(fsa_reg_i2c_defaults); i++)
		regmap_write(regmap, fsa_reg_i2c_defaults[i].reg,
				   fsa_reg_i2c_defaults[i].val);
}


static int fsa4480_switch_set(struct typec_switch_dev *sw,
			      enum typec_orientation orientation)
{
	struct fsa4480_priv *fsa = typec_switch_get_drvdata(sw);
	u8 new_sel;
	u8 new_enable = fsa->cur_enable;

	mutex_lock(&fsa->fsa4480_lock);
	new_sel = FSA4480_SEL_USB;

	if (orientation == TYPEC_ORIENTATION_REVERSE)
		new_sel |= FSA4480_SEL_SBU_REVERSE;
	else if (orientation == TYPEC_ORIENTATION_NONE) {
		new_enable = FSA4480_ENABLE_DEVICE | FSA4480_ENABLE_USB;
		dev_info(fsa->dev, "%s: orientation %d, disable SBU, new_enable %x\n",
			__func__, orientation, new_enable);
	}

	dev_info(fsa->dev, "%s: orientation %d, new_enable %x, cur_enable %x, new_sel %x, cur_select %x\n",
		__func__, orientation, new_enable, fsa->cur_enable, new_sel, fsa->cur_select);

	if(new_sel == fsa->cur_select && new_enable == fsa->cur_enable)
		goto out_unlock;

	if (new_enable & FSA4480_ENABLE_SBU) {
		/* Disable SBU output while re-configuring the switch */
		regmap_write(fsa->regmap, FSA4480_SWITCH_SETTINGS,
			     new_enable & ~FSA4480_ENABLE_SBU);

		/* 35us to allow the SBU switch to turn off */
		usleep_range(35, 1000);
	}

	regmap_write(fsa->regmap, FSA4480_SWITCH_CONTROL, new_sel);
	fsa->cur_select = new_sel;

	if (new_enable & FSA4480_ENABLE_SBU || new_enable != fsa->cur_enable) {
		regmap_write(fsa->regmap, FSA4480_SWITCH_SETTINGS, new_enable);

		/* 15us to allow the SBU switch to turn on again */
		usleep_range(15, 1000);

		if(new_enable != fsa->cur_enable)
			fsa->cur_enable = new_enable;
	}

out_unlock:
	mutex_unlock(&fsa->fsa4480_lock);

	return 0;
}

static int fsa4480_mux_set(struct typec_mux_dev *mux, struct typec_mux_state *state)
{
	struct fsa4480_priv *fsa = typec_mux_get_drvdata(mux);
	struct typec_displayport_data *dp_data = state->data;
	u8 new_enable;

	mutex_lock(&fsa->fsa4480_lock);

	new_enable = FSA4480_ENABLE_DEVICE | FSA4480_ENABLE_USB;

	if (dp_data->conf) {
		new_enable |= FSA4480_ENABLE_SBU;

		dev_info(fsa->dev, "%s: conf %d, new_enable %x, cur_enable %x\n",
			__func__, dp_data->conf, new_enable, fsa->cur_enable);

		if (new_enable == fsa->cur_enable)
			goto out_unlock;

		regmap_write(fsa->regmap, FSA4480_SWITCH_SETTINGS, new_enable);
		fsa->cur_enable = new_enable;

		if (new_enable & FSA4480_ENABLE_SBU) {
			/* 15us to allow the SBU switch to turn off */
			usleep_range(15, 1000);
		}
	}
out_unlock:
	mutex_unlock(&fsa->fsa4480_lock);

	return 0;
}

static int fsa4480_probe(struct i2c_client *i2c,
			 const struct i2c_device_id *id)
{
	int rc = 0;
	struct device *dev = &i2c->dev;
	struct typec_switch_desc sw_desc = { };
	struct typec_mux_desc mux_desc = { };

	dev_info(dev, "%s: enter\n", __func__);

	fsa_priv = devm_kzalloc(&i2c->dev, sizeof(*fsa_priv),
				GFP_KERNEL);
	if (!fsa_priv)
		return -ENOMEM;

	memset(fsa_priv, 0, sizeof(struct fsa4480_priv));
	fsa_priv->dev = &i2c->dev;

	fsa_priv->regmap = devm_regmap_init_i2c(i2c, &fsa4480_regmap_config);
	if (IS_ERR_OR_NULL(fsa_priv->regmap)) {
		dev_err(fsa_priv->dev, "%s: Failed to initialize regmap: %d\n",
			__func__, rc);
		if (!fsa_priv->regmap) {
			rc = -EINVAL;
			goto err_data;
		}
		rc = PTR_ERR(fsa_priv->regmap);
		goto err_data;
	}

	fsa4480_update_reg_defaults(fsa_priv->regmap);

	mutex_init(&fsa_priv->fsa4480_lock);

	sw_desc.drvdata = fsa_priv;
	sw_desc.fwnode = dev->fwnode;
	sw_desc.set = fsa4480_switch_set;

#if IS_ENABLED(CONFIG_MTK_USB_TYPEC_MUX)
	fsa_priv->sw = mtk_typec_switch_register(dev, &sw_desc);
#else
	fsa_priv->sw = typec_switch_register(dev, &sw_desc);
#endif
	if (IS_ERR(fsa_priv->sw)) {
		dev_info(dev, "error registering typec switch: %ld\n",
			PTR_ERR(fsa_priv->sw));
		return PTR_ERR(fsa_priv->sw);
	}

	mux_desc.drvdata = fsa_priv;
	mux_desc.fwnode = dev->fwnode;
	mux_desc.set = fsa4480_mux_set;

#if IS_ENABLED(CONFIG_MTK_USB_TYPEC_MUX)
	fsa_priv->mux = mtk_typec_mux_register(dev, &mux_desc);
#else
	fsa_priv->mux = typec_mux_register(dev, &mux_desc);
#endif
	if (IS_ERR(fsa_priv->mux)) {
#if IS_ENABLED(CONFIG_MTK_USB_TYPEC_MUX)
		mtk_typec_switch_unregister(fsa_priv->sw);
#else
		mtk_typec_switch_unregister(fsa_priv->sw);
#endif
		dev_info(dev, "error registering typec mux: %ld\n",
			PTR_ERR(fsa_priv->mux));
		return PTR_ERR(fsa_priv->mux);
	}

	i2c_set_clientdata(i2c, fsa_priv);

	dev_info(dev, "%s: done\n", __func__);

	return 0;

err_data:
	devm_kfree(&i2c->dev, fsa_priv);
	return rc;
}

static void fsa4480_remove(struct i2c_client *i2c)
{
	struct fsa4480_priv *fsa_priv =
			(struct fsa4480_priv *)i2c_get_clientdata(i2c);

#ifdef CONFIG_FSA4480_SENS_TO_GND
	fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x9D);
#else
	fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
#endif
	pm_relax(fsa_priv->dev);
	mutex_destroy(&fsa_priv->fsa4480_lock);
	dev_set_drvdata(&i2c->dev, NULL);

}

static const struct of_device_id fsa4480_i2c_dt_match[] = {
	{
		.compatible = "fsa,fsa4480-i2c",
	},
	{}
};

static struct i2c_driver fsa4480_i2c_driver = {
	.driver = {
		.name = FSA4480_I2C_NAME,
		.of_match_table = fsa4480_i2c_dt_match,
	},
	.probe = fsa4480_probe,
	.remove = fsa4480_remove,
};

static int __init fsa4480_init(void)
{
	int rc;

	rc = i2c_add_driver(&fsa4480_i2c_driver);
	if (rc)
		pr_err("fsa4480: Failed to register I2C driver: %d\n", rc);

	return rc;
}
module_init(fsa4480_init);

static void __exit fsa4480_exit(void)
{
	i2c_del_driver(&fsa4480_i2c_driver);
}
module_exit(fsa4480_exit);

MODULE_DESCRIPTION("FSA4480 I2C driver");
MODULE_LICENSE("GPL v2");
