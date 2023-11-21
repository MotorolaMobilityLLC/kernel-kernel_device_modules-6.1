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

struct hl7603_chip {
	struct device *dev;
	struct regmap *regmap;
	int chip_irq;
};

static const struct regmap_config hl7603_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = HL7603_REG_MAX,
};

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
	ret = regmap_write(hl7603_chip->regmap, HL7603_REG_CONTROL, val);
	if (ret < 0)
		dev_err(hl7603_chip->dev, "Failed to set HL7603_REG_VOUT_SEL value\n");
end:
	return count;
}

static DEVICE_ATTR(registers, S_IRUGO, hl7603_attr_registers_show, NULL);
static DEVICE_ATTR(vout,  S_IWUSR | S_IRUGO, hl7603_attr_vout_show, hl7603_attr_vout_store);

static struct attribute *hl7603_attributes[] = {
	&dev_attr_registers.attr,
	&dev_attr_vout.attr,
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
	int error = 0;
	int ret;
	unsigned int value = 0;
	pr_info("hl7603_i2c_probe start\n");

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
		dev_err(dev, "Failed to allocate register map: %d\n",
			error);
		return error;
	}

	ret=sysfs_create_group(&chip->dev->kobj, &hl7603_attr_group);
	if (ret < 0)
		dev_err(chip->dev, "Failed to sysfs_create_group (%d)\n",ret);

	ret = regmap_read(chip->regmap, HL7603_REG_CONTROL, &value);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read control register(%d)\n",
			ret);
		return ret;
	}
	else {
		dev_info(chip->dev, "read control register pass ret=(%d)\n",
			ret);
	}

	ret = regmap_read(chip->regmap, HL7603_REG_VOUT_SEL, &value);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read HL7603_REG_VOUT_SET register(%d)\n",
			ret);
		return ret;
	}
	else {
		dev_info(chip->dev, "read HL7603_REG_VOUT_SEL register pass ret=(%d), Value=0x%x\n",
			ret, value);
	}

	dev_info(chip->dev, "hl7603_i2c_probe end\n");
	return error;
}

static void hl7603_i2c_remove(struct i2c_client *client)
{
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
