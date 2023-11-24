// SPDX-License-Identifier: GPL-2.0+
/*
 * HL7603, Boost Buck
 * Copyright (C) 2022  Motorola Mobility LLC,
 *
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

enum hl7603_registers {
	HL7603_REG_CONTROL = 1,
	HL7603_REG_VOUT_SEL,
	HL7603_REG_ILIMSET1,
	HL7603_REG_ILIMSET2,
	HL7603_REG_STATUS,
	HL7603_REG_MAX = HL7603_REG_STATUS,
};

enum hl7603_mode_cfg {
	LOW_IQ_FORCE_BYPASS_MODE = 0,
	NORMAL_FORCE_BYPASS_MODE,
	AUTO_BYPASS_MODE,
};

//HL7603_REG_CONTROL 0x01
#define HL7603_RESET_MASK               BIT(7)
#define HL7603_RESET_SHIFT              7
#define HL7603_DEV_EN_MASK             	BIT(6)
#define HL7603_DEV_EN_SHIFT             6
#define HL7603_MODE_CFG_MASK        	(BIT(5) |BIT(4))
#define HL7603_MODE_CFG_SHIFT       	4
#define HL7603_VOUT_DISCHG_MASK          BIT(3)
#define HL7603_VOUT_DISCHG_SHIFT         3
#define HL7603_EN_OOA_MASK              	BIT(2)
#define HL7603_EN_OOA_SHIFT             2
#define HL7603_FPWM_CFG_MASK              BIT(1)
#define HL7603_FPWM_CFG_SHIFT             1

//HL7603_REG_VOUT_SEL   0x02
#define HL7603_VOUT_REG_MASK               (BIT(5)|BIT(4)|BIT(3)|BIT(2)|BIT(1)|BIT(0))
#define HL7603_VOUT_REG_SHIFT              	0

// HL7603_REG_ILIMSET1   0x03
#define HL7603_ILIN1_SET_MASK        		(BIT(7) |BIT(6))
#define HL7603_ILIN1_SET_SHIFT       		6
#define HL7603_ILIM_OFF_MASK    		BIT(5)
#define HL7603_ILIM_OFF_SHIFT   		5
#define HL7603_SOFT_START_MASK   		BIT(4)
#define HL7603_SOFT_START_SHIFT   		4
#define HL7603_ILIM_MASK        		(BIT(3) |BIT(2)|BIT(0))
#define HL7603_ILIM_SHIFT       		0

//HL7603_REG_ILIMSET2     0x04
#define HL7603_T_ILIM_H_MASK        		(BIT(1) |BIT(0))
#define HL7603_T_ILIM_H_SHIFT       		0

// HL7603_REG_STATUS      0x05
#define HL7603_TSD_MASK               		BIT(7)
#define HL7603_TSD_SHIFT              		7
#define HL7603_HOTDIE_MASK             		BIT(6)
#define HL7603_HOTDIE_SHIFT              	6
#define HL7603_DCDCMODE_MASK        		BIT(5)
#define HL7603_DCDCMODE_SHIFT       		5
#define HL7603_OPMODE_MASK        		BIT(4)
#define HL7603_OPMODE_SHIFT       		4
#define HL7603_VIN_OVP_MASK         	 	BIT(3)
#define HL7603_VIN_OVP_SHIFT         		3
#define HL7603_VOUT_OVP_MASK             	BIT(2)
#define HL7603_VOUT_OVP_SHIFT             	2
#define HL7603_FAULT_MASK              		BIT(1)
#define HL7603_FAULT_SHIFT             		1
#define HL7603_PGOOD_MASK                	BIT(0)
#define HL7603_PGOOD_SHIFT             		0

struct hl7603_chip {
	struct device *dev;
	struct regmap *regmap;
	int chip_irq;
	int voltage_value;
};

static const struct regmap_config hl7603_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = HL7603_REG_MAX,
};

static int  hl7603_read_interface(struct hl7603_chip *chip, u8 RegNum,
				u8 *val, u8 MASK, u8 SHIFT)
{
	u32 reg_val = 0;
	int ret = 0;

	ret = regmap_read(chip->regmap, RegNum, &reg_val);
	if (ret < 0)
		dev_err(chip->dev, "[%s] regmap_read error, ret=%d\n", __func__, ret);
	else {
		reg_val &= MASK;
		*val = (reg_val >> SHIFT);
		dev_info(chip->dev, "[%s] MASK val=0x%x\n", __func__, *val);
	}

	return ret;
}

static int hl7603_config_interface(struct hl7603_chip *chip, u8 RegNum,
				u8 val, u8 MASK, u8 SHIFT)
{
	unsigned int reg_val = 0;
	int ret = 0;

	ret = regmap_read(chip->regmap, RegNum, &reg_val);
	if (ret < 0)
		dev_err(chip->dev, "[%s] regmap_read error, ret=%d\n", __func__, ret);
	else {
		reg_val &= ~(MASK);
		reg_val |= (val << SHIFT);

		ret = regmap_write(chip->regmap, RegNum, reg_val);
		if (ret < 0)
			dev_err(chip->dev, "[%s] regmap_write error, ret=%d\n", __func__, ret);
	}

	return ret;
}

unsigned char hl7603_get_reg_value(struct hl7603_chip *chip, unsigned char reg)
{
	unsigned char reg_val = 0;

	hl7603_read_interface(chip, reg, &reg_val, 0xFF, 0x0);

	return reg_val;
}


void hl7603_dump_register(struct hl7603_chip *chip)
{
	int i ;

	for (i = 0x01; i <= HL7603_REG_MAX; i++)
		dev_info(chip->dev, "%s [0x%02x]=0x%02x\n", __func__, i, hl7603_get_reg_value(chip, i));
}

int hl7603_enable(struct hl7603_chip *chip, unsigned char en)
{
	int ret = 0;

	ret = hl7603_config_interface(chip,HL7603_REG_CONTROL, en, HL7603_DEV_EN_MASK, HL7603_DEV_EN_SHIFT);
	if (ret < 0)
		dev_err(chip->dev, "[%s] error, ret=%d\n", __func__, ret);

	return ret;
}

int hl7603_set_mode(struct hl7603_chip *chip, unsigned char mode)
{
	int ret = 0;

	ret = hl7603_config_interface(chip, HL7603_REG_CONTROL, mode, HL7603_MODE_CFG_MASK, HL7603_MODE_CFG_SHIFT);
	if (ret < 0)
		dev_err(chip->dev, "[%s] error, ret=%d\n", __func__, ret);

	return ret;
}

int hl7603_hw_init(struct hl7603_chip *chip)
{
	int ret = 0;

	ret = hl7603_config_interface(chip, HL7603_REG_VOUT_SEL, chip->voltage_value, HL7603_VOUT_REG_MASK, HL7603_VOUT_REG_SHIFT);
	if (ret < 0)
		return -1;

	ret = hl7603_set_mode(chip, AUTO_BYPASS_MODE);
	if (ret < 0)
		return -1;

	ret = hl7603_enable(chip, 1);
	if (ret < 0)
		return -1;

	dev_err(chip->dev, "%s done\n", __func__);
	return ret;
}
static int hl7603_reg_reset(struct hl7603_chip *chip)
{
	int ret;
	unsigned int reg_val;

	hl7603_config_interface(chip, HL7603_REG_CONTROL, 0x01, HL7603_RESET_MASK,HL7603_RESET_SHIFT);
	mdelay(20); /* wait soft reset ready */
	ret = regmap_read(chip->regmap, HL7603_REG_CONTROL, &reg_val);
	if (ret < 0)
		return -EPERM;

	dev_info(chip->dev, "%s success,[%x]=0x%x\n", __func__, HL7603_REG_CONTROL, reg_val);
	return 0;
}

static ssize_t hl7603_attr_registers_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct hl7603_chip *hl7603_chip = i2c_get_clientdata(client);
	int ret = 0;
	unsigned int reg1 = 0;
	unsigned int reg2 = 0;
	unsigned int reg3 = 0;
	unsigned int reg4 = 0;
	unsigned int reg5 = 0;


	if (regmap_read(hl7603_chip->regmap, HL7603_REG_CONTROL, &reg1) < 0)
		dev_err(hl7603_chip->dev, "Failed to read HL7603_REG_CONTROL register\n");
	else if (regmap_read(hl7603_chip->regmap, HL7603_REG_VOUT_SEL, &reg2) < 0)
		dev_err(hl7603_chip->dev, "Failed to read HL7603_REG_VOUT_SEL register\n");
	else if (regmap_read(hl7603_chip->regmap, HL7603_REG_ILIMSET1, &reg3) < 0)
		dev_err(hl7603_chip->dev, "Failed to read HL7603_REG_ILIMSET1 register\n");
	else if (regmap_read(hl7603_chip->regmap, HL7603_REG_ILIMSET2, &reg4) < 0)
		dev_err(hl7603_chip->dev, "Failed to read HL7603_REG_ILIMSET2 register\n");
	else if (regmap_read(hl7603_chip->regmap, HL7603_REG_STATUS, &reg5) < 0)
		dev_err(hl7603_chip->dev, "Failed to read HL7603_REG_STATUS register\n");
	else
		ret = sprintf(buf, " 0x%02x \n 0x%02x \n 0x%02x \n 0x%02x \n 0x%02x \n",
			reg1, reg2, reg3, reg4, reg5);

	return ret;
}

static ssize_t hl7603_attr_dev_en_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct hl7603_chip *hl7603_chip = i2c_get_clientdata(client);
	unsigned char en = 0;
	int ret = 0;

	ret = hl7603_read_interface(hl7603_chip,HL7603_REG_CONTROL, &en,
					HL7603_DEV_EN_MASK, HL7603_DEV_EN_SHIFT);
	if(ret < 0)
		dev_err(hl7603_chip->dev, "%s Failed to read HL7603_REG_CONTROL register\n", __func__);
	else
		ret = sprintf(buf, "%d\n", en);

	return ret;
}

static ssize_t hl7603_attr_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct hl7603_chip *hl7603_chip = i2c_get_clientdata(client);
	unsigned char mode = 0;
	int ret = 0;

	ret = hl7603_read_interface(hl7603_chip, HL7603_REG_CONTROL, &mode,
					HL7603_MODE_CFG_MASK, HL7603_MODE_CFG_SHIFT);
	if(ret < 0)
		dev_err(hl7603_chip->dev, "%s Failed to read HL7603_REG_CONTROL register\n", __func__);
	else
		ret = sprintf(buf, "%d\n", mode);

	return ret;
}


static ssize_t hl7603_attr_vout_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct hl7603_chip *hl7603_chip = i2c_get_clientdata(client);
	int ret = 0;
	unsigned int reg1 = 0;

	if (regmap_read(hl7603_chip->regmap, HL7603_REG_VOUT_SEL, &reg1) < 0)
		dev_err(hl7603_chip->dev, "Failed to read HL7603_REG_VOUT_SEL register\n");
	else
		ret = sprintf(buf, "0x%02x\n", reg1);

	return ret;
}

static ssize_t hl7603_attr_vout_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct hl7603_chip *hl7603_chip = i2c_get_clientdata(client);
	int ret = 0;
	unsigned int val = 0;

	ret = kstrtouint(buf, 10, &val);
	if (ret) {
		dev_err(hl7603_chip->dev,"%s, %s not a valide buf(%d)\n", __func__, buf, ret);
		goto end;
	}
	ret = regmap_write(hl7603_chip->regmap, HL7603_REG_VOUT_SEL, val);
	if (ret < 0)
		dev_err(hl7603_chip->dev, "Failed to set HL7603_REG_VOUT_SEL value\n");
end:
	return count;
}

static DEVICE_ATTR(registers, S_IRUGO, hl7603_attr_registers_show, NULL);
static DEVICE_ATTR(vout,  S_IWUSR | S_IRUGO, hl7603_attr_vout_show, hl7603_attr_vout_store);
static DEVICE_ATTR(dev_en, S_IRUGO, hl7603_attr_dev_en_show, NULL);
static DEVICE_ATTR(mode, S_IRUGO, hl7603_attr_mode_show, NULL);

static struct attribute *hl7603_attributes[] = {
	&dev_attr_registers.attr,
	&dev_attr_vout.attr,
	&dev_attr_dev_en.attr,
	&dev_attr_mode.attr,
	NULL,
};

static const struct attribute_group hl7603_attr_group = {
	.attrs = hl7603_attributes,
};

static int hl7603_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct hl7603_chip *chip;
	struct device_node *np = NULL;
	int error = 0;
	int ret;

	chip = devm_kzalloc(dev, sizeof(struct hl7603_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(chip->dev, "hl7603_i2c_probe Memory error...\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, chip);
	chip->chip_irq = client->irq;
	chip->dev = dev;
	chip->regmap = devm_regmap_init_i2c(client, &hl7603_regmap_config);
	if (IS_ERR(chip->regmap)) {
		error = PTR_ERR(chip->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", error);
		return error;
	}

	ret=sysfs_create_group(&chip->dev->kobj, &hl7603_attr_group);
	if (ret < 0)
		dev_err(chip->dev, "Failed to sysfs_create_group (%d)\n",ret);


	np = chip->dev->of_node;
	chip->voltage_value = -EINVAL;
	ret = of_property_read_u32(np, "hl7603_vout_voltage", &chip->voltage_value);
	if (ret) {
		dev_err(chip->dev, "failed to read bat-ovp-threshold\n");
	}
	else {
		dev_info(chip->dev, "config voltage_value to(0x%02x)\n", chip->voltage_value);
		hl7603_reg_reset(chip);
		hl7603_hw_init(chip);
		hl7603_dump_register(chip);
	}

	dev_info(chip->dev, "hl7603_i2c_probe end\n");
	return 0;
}

static void hl7603_i2c_remove(struct i2c_client *client)
{
	struct hl7603_chip *chip = i2c_get_clientdata(client);

	sysfs_remove_group(&chip->dev->kobj, &hl7603_attr_group);
	return;
}

static int hl7603_suspend(struct device *device)
{

	return 0;
}

static int hl7603_resume(struct device *device)
{
	return 0;
}

static const struct dev_pm_ops hl7603_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hl7603_suspend, hl7603_resume)
};

static const struct i2c_device_id hl7603_i2c_id[] = {
	{"hl7603", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, hl7603_i2c_id);

static struct i2c_driver hl7603_driver = {
	.driver = {
		.name = "hl7603",
		.pm = &hl7603_pm_ops,
	},
	.probe = hl7603_i2c_probe,
	.remove = hl7603_i2c_remove,
	.id_table = hl7603_i2c_id,
};

module_i2c_driver(hl7603_driver);

MODULE_AUTHOR("motorola");
MODULE_DESCRIPTION("HL7603 regulator driver");
MODULE_LICENSE("GPL");
