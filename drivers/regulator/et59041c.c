/*
 * et59041c-regulator.c - Regulator device driver for ET59041C
 * Copyright (C) 2021  ETEK Semiconductor Ltd.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 */

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regmap.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/regulator/of_regulator.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include "et59041c.h"

// #define ET59041C_IRQ_EN
static int dbg_enable = 1;
module_param_named(dbg_level, dbg_enable, int, 0644);

#define ET_DBG(args...)                                                        \
	do {                                                                   \
		if (dbg_enable) {                                              \
			pr_info(args);                                         \
		}                                                              \
	} while (0)

// static struct class* hello_class = NULL;

static const struct regmap_config et59041c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = ET59041C_REG_CONTROL,
	// .cache_type	= REGCACHE_NONE,
	// .rd_table	= &et59041c_no_reg_table,
	// .wr_table	= &et59041c_no_reg_table,
};
static const int ldo12CurTable[]={
	1300000,
	1250000,
	1400000,
	1450000,
	1070000,
	1010000,
	1130000,
	1190000,
	1720000,
	1680000,
	1800000,
	1880000,
	1520000,
	1460000,
	1600000,
	1660000
};

static const int ldo34CurTable[]={
	450000,
	380000,
	590000,
	520000,
	230000,
	170000,
	380000,
	310000,
	860000,
	820000,
	1010000,
	940000,
	650000,
	600000,
	790000,
	740000

};



/******************************************************/
/*Funcation: Binary2Char                      	        				*/
/*Input:  	binary - 数值数组，输入要转换的数值；
	      len - binary数组里的数值个数（以字节为单位）；
	     buff - 字符数组，输出转换后的字符；
	      size - buff数组的空间（以字节为单位）。 		*/
/*Output: 	true or false									 */

/******************************************************/
int Binary2Char( unsigned char*  binary,  int len, char*  buff,  int size)
{
	    int  i, n;

	    n =0 ;

	    for (i = 0; i < len; i++)
	     n += sprintf(buff + i * 2, "%02X", binary[i]);

	   if(n > size){
			ET_DBG("Binary2Char data large than array size");
			return 0;
	    }
	    return  n;
}


/******************************************************/
/*Funcation: Char2Binary                      	        				*/
/*Input:  	binary - 数值数组，输入要转换的数值；
	      len - binary数组里的数值个数（以字节为单位）；
	     buff - 字符数组，输出转换后的字符；
	      size - buff数组的空间（以字节为单位）。 		*/
/*Output: 	true or false									 */

/******************************************************/
int Char2Binary(const char* token,  int len, unsigned char*  binary,  int size)
{
	    const char*     p;
	    int         i, n, m;
	    char        buf[3] = {0,0,0};


	    m = len % 2 ? (len - 1) / 2 : len / 2;


		if(m > size){
			ET_DBG("Char2Binary data large than array size");
			return 0;
	    }
		p = token;
	// 为了提高效率，先两个两个字符地转换：
		for (i = 0; i < m; i++)
		{
			p = token + i * 2;
			buf[0] = p[0];
			buf[1] = p[1];

			n = 0;
			sscanf(buf, "%X", &n);
			binary[i] = n;
		}

	    // 再转换最后一个字符（如果有）：
	    if(len % 2)
	    {
			buf[0] = p[2];
			buf[1] = 0;
			n = 0;
			sscanf(buf, "%X", &n);
			binary[i] = n;
	        i++;
	    }

	    return  i;
}

static int et59041c_regulator_dump_regs(struct et59041c_regulator *regular, u8 *regArr)
{
	//u8 regArr[0x1F];

	regmap_bulk_read(regular->regmap, 0x00, regArr, 0x10);
	ET_DBG("CHIPID: 0x%x\n", regArr[ET59041C_REG_CHIPID]);
	ET_DBG("LDO_ILIMIT: 0x%x\n", regArr[ET59041C_REG_ILIMIT]);
	ET_DBG("LDO_EN: 0x%x\n", regArr[ET59041C_REG_LDO_EN]);
	ET_DBG("RDIS: 0x%x\n", regArr[ET59041C_REG_RDIS]);
	ET_DBG("DVO1: 0x%x\n", regArr[ET59041C_REG_DVO1]);
	ET_DBG("DVO2: 0x%x\n", regArr[ET59041C_REG_DVO2]);
	ET_DBG("AVO1: 0x%x\n", regArr[ET59041C_REG_AVO1]);
	ET_DBG("AVO2: 0x%x\n", regArr[ET59041C_REG_AVO2]);
	ET_DBG("SEQ1: 0x%x\n", regArr[ET59041C_REG_SEQ1]);
	ET_DBG("SEQ2: 0x%x\n", regArr[ET59041C_REG_SEQ2]);
	ET_DBG("SEQ_C: 0x%x\n", regArr[ET59041C_REG_SEQ_C]);
	ET_DBG("ILIMT_COAR: 0x%x\n", regArr[ET59041C_REG_ILIMT_COAR]);


	return 0;
}

ssize_t et59041c_reg_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int len = 0;
	u8 regArr[0x10];
	struct et59041c_regulator *regular = dev_get_drvdata(dev);
	et59041c_regulator_dump_regs(regular, regArr);
	len = Binary2Char( regArr, 0x10,  buf,  200);
	if(len>0)	{
		ET_DBG("%s: buf = %s  \n", __func__,buf);
		return len;
	}
	return len;
}

/*写设备属性val*/
ssize_t et59041c_reg_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct et59041c_regulator *regular =
		(struct et59041c_regulator *)dev_get_drvdata(dev);
	int val = 0;
	int addr;
	int ret;

	ret = sscanf(buf, "%x:%x", &addr, &val);
	// val = simple_strtol(buf, NULL, 10);
	ET_DBG("REG%x: 0x%x\n", addr, val);
	if (ret != 2) {
		return -EINVAL;
	}
	ret = regmap_write(regular->regmap, addr, val);

	if (ret < 0)
		return ret;

	return count;
}

ssize_t et59041c_regulator_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct et59041c_regulator *regular = dev_get_drvdata(dev);
	int val = 0;
	int ret = 0;
	ret = regmap_read(regular->regmap, ET59041C_REG_LDO_EN, &val);

	if (ret < 0)
		return ret;

	return val;
}

/*写设备命令*/
ssize_t et59041c_regulator_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct et59041c_regulator *regular =
		(struct et59041c_regulator *)dev_get_drvdata(dev);
	int ret;
	char cmd;
	u32 input[2], addr, data;

	ret = sscanf(buf, "%c:", &cmd);
	switch (cmd) {
	case 'w':
		ret = sscanf(buf, "%c:%x:%x", &cmd, &input[0], &input[1]);
		if (ret != 3) {
			pr_err("erro! cmd format: echo w [addr] [value]\n");
			goto out;
		};
		addr = input[0] & 0xff;
		data = input[1] & 0xff;
		pr_info("cmd : %c %x %x\n\n", cmd, input[0], input[1]);
		regmap_write(regular->regmap, addr, data);
		regmap_read(regular->regmap, addr, &data);
		pr_info("new: %x %x\n", addr, data);
		break;
	case 'r':
		ret = sscanf(buf, "%c:%x", &cmd, &input[0]);
		if (ret != 2) {
			pr_err("erro! cmd format: echo r [addr]\n");
			goto out;
		};
		pr_info("cmd : %c %x\n\n", cmd, input[0]);
		addr = input[0] & 0xff;
		regmap_read(regular->regmap, addr, &data);
		pr_info("%x %x\n", input[0], data);
		break;
	case 'd':
		ret = sscanf(buf, "%c:%x:%x", &cmd, &input[0], &input[1]);
		if (ret != 3) {
			pr_err("erro! cmd format: echo d [addr]\n");
			goto out;
		};
		pr_info("cmd : %c %x %x\n\n", cmd, input[0], input[1]);
		addr = input[0] & 0xff;
		data = input[1] & 0xff;
		regmap_update_bits(regular->regmap, ET59041C_REG_SEQ_C, addr, data);
		regmap_read(regular->regmap, ET59041C_REG_SEQ_C, &data);
		pr_info("ET59041C_REG_SEQ_C: %x %x\n", ET59041C_REG_SEQ_C, data);
		break;
	default:
		pr_err("Unknown command\n");
		break;
	}

out:
	return count;
}

static struct device_attribute et59041c_regulator_attrs[] = {
	__ATTR(et59041c_regulator_info, 0664, et59041c_reg_show, et59041c_reg_store),
	__ATTR(et59041c_regulator_set, 0664, et59041c_regulator_show,
	       et59041c_regulator_store),
};

static void et59041c_init_sysfs(struct et59041c_regulator *regular)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(et59041c_regulator_attrs); i++) {
		ret = sysfs_create_file(&regular->dev->kobj,
					&et59041c_regulator_attrs[i].attr);
		if (ret)
			dev_err(regular->dev,
				"create et59041c regulator node(%s) error\n",
				et59041c_regulator_attrs[i].attr.name);
	}
}

static int et59041c_regulator_enable(struct regulator_dev *rdev)
{
	struct et59041c_regulator *info = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	pr_info("et59041c_regulator_enable  id= %d\n", id);
	if (info == NULL) {
		dev_err(rdev_get_dev(rdev), "regulator info null pointer\n");
		return -EINVAL;
	}


	return regmap_update_bits(rdev->regmap, ET59041C_REG_LDO_EN,
				 rdev->desc->vsel_mask, rdev->desc->vsel_mask);
}

static int et59041c_regulator_disable(struct regulator_dev *rdev)
{
	struct et59041c_regulator *info = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	pr_info("et59041c_regulator_disable  id= %d\n", id);
	if (info == NULL) {
		dev_err(rdev_get_dev(rdev), "regulator info null pointer\n");
		return -EINVAL;
	}

	return regmap_update_bits(rdev->regmap, ET59041C_REG_LDO_EN,
				 rdev->desc->vsel_mask, 0);
}

static int et59041c_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct et59041c_regulator *info = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int val;
	ET_DBG("et59041c_regulator_is_enabled id= %d", id);
	if (info == NULL) {
		dev_err(rdev_get_dev(rdev), "regulator info null pointer\n");
		return -EINVAL;
	}

	//Workaround: for some hw units 0x0B register unknown overwriting caused regulator enable disfunction.
	{
		int val = 0;
		regmap_read(rdev->regmap, ET59041C_REG_SEQ1, &val);
		if (val) {
			ET_DBG("Force correcting SEQ1 register from value %d\n", val);
			regmap_write(rdev->regmap, ET59041C_REG_SEQ1, 0);
		}
		regmap_read(rdev->regmap, ET59041C_REG_SEQ2, &val);
		if (val) {
			ET_DBG("Force correcting SEQ2 register from value %d\n", val);
			regmap_write(rdev->regmap, ET59041C_REG_SEQ2, 0);
		}
	}

	regmap_read(rdev->regmap, ET59041C_REG_LDO_EN, &val);

	ET_DBG("ET59041C_REG_LDO_EN(0x0E)  = 0x%x", val);
	pr_info("et59041c_vsel_mask %d\n", rdev->desc->vsel_mask);
	// ET_DBG("val= %d, vsel_mask=%d",val,rdev->desc->vsel_mask);
	// if(val && (1<<rdev->desc->vsel_mask)){

	// 	return 1;
	// }
	return 1;
}

__attribute__((unused)) static int
et59041c_get_voltage_regulator(struct regulator_dev *rdev)
{
	int ret;
	unsigned int val;
	struct et59041c_regulator *info = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	ET_DBG("et59041c_regulator_enable= %d", id);
	if (info == NULL) {
		dev_err(rdev_get_dev(rdev), "regulator info null pointer\n");
		return -EINVAL;
	}
	ret = regmap_read(rdev->regmap, ET59041C_LDO_REG_INDEX(id), &val);
	if (ret != 0)
		return ret;

	return val;
}

__attribute__((unused)) static int
et59041c_set_voltage_regulator(struct regulator_dev *rdev, unsigned int sel)
{
	struct et59041c_regulator *info = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	ET_DBG("et59041c_set_voltage_regulator id= %d sel : %d\n", id,sel);

	if (info == NULL) {
		dev_err(rdev_get_dev(rdev), "regulator info null pointer\n");
		return -EINVAL;
	}

	return regmap_write(rdev->regmap, ET59041C_LDO_REG_INDEX(id), sel);
}

static int et59041c_set_voltage(struct regulator_dev *rdev, int min_uV,
			      int max_uV, unsigned *selector)
{
	struct et59041c_regulator *info = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	u8 val = 0;

	int ret;
	int mask = rdev->desc->vsel_mask;
	ET_DBG("et59041c_reg_set_voltage id= %d min_uV=%d max_uV=%d mask=%d", id, min_uV,
	       max_uV,  mask);
	if (info == NULL) {
		dev_err(rdev_get_dev(rdev), "regulator info null pointer\n");
		return -EINVAL;
	}
	/*if (min_uV != max_uV) {
		dev_err(rdev_get_dev(rdev), "Wrong input value\n");
		return -EINVAL;
	}*/

	if (id == ET59041C_ID_REGULATOR1 || id == ET59041C_ID_REGULATOR2) {
		if (min_uV > 1800000 || min_uV < 600000) {
			dev_err(rdev_get_dev(rdev),
				"min_uV or max_uV out of range\n");
			return -EINVAL;
		}
		val = (min_uV - 600000) / 6000;
		dev_info(rdev_get_dev(rdev), "regulator val : 0x%x\n",val);

	} else {
		if (min_uV > 4300000 || min_uV < 1200000) {
			dev_err(rdev_get_dev(rdev),
				"min_uV or max_uV out of range\n");
			return -EINVAL;
		}
		val = (min_uV - 1200000) / 12500;
		dev_info(rdev_get_dev(rdev), "regulator val : 0x%x\n",val);
	}
	ret = regmap_write(rdev->regmap, ET59041C_LDO_REG_INDEX(id), val);
	if (ret < 0) {
		dev_err(info->dev, "Failed to et59041c_set_voltage: %d\n", ret);
		return ret;
	}

	return  regmap_update_bits(rdev->regmap, ET59041C_REG_LDO_EN,
				 mask, mask);


}


static int et59041c_voltage_list(struct regulator_dev *rdev, unsigned index)
{
	int id = rdev_get_id(rdev);

	ET_DBG("et59041c_voltage_list id= %d index=%d", id, index);

	if (id == ET59041C_ID_REGULATOR1 || id == ET59041C_ID_REGULATOR2) {
		return 200;

	} else {
		return 255;
	}
}




static int et59041c_voltage_get(struct regulator_dev *rdev)
{
	struct et59041c_regulator *info = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int err;
	int val = 0;
	err = regmap_read(rdev->regmap, ET59041C_LDO_REG_INDEX(id), &val);
	if(id == ET59041C_ID_REGULATOR2)
	{
		val = 0x4b;
	}
	if(id == ET59041C_ID_REGULATOR1)
	{
		val = 0x54;
	}
	if (err < 0) {
		dev_err(info->dev, "Failed to et59041c_voltage_get: %d\n", err);
		return -EINVAL;
	}
	if (id == ET59041C_ID_REGULATOR1 || id == ET59041C_ID_REGULATOR2) {
		return val * 6000 + 600000;

	} else {
		return val * 12500 + 1200000;
	}
}

static int et59041c_set_current_limit(struct regulator_dev *rdev, int min_uA, int max_uA)
{
	struct et59041c_regulator *info = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int err;
	int val = 0;
	int val2 = 0;
	int i = 0;
	int index = -1;

	if (id == ET59041C_ID_REGULATOR1 || id == ET59041C_ID_REGULATOR2) {
		for(i=0;i<16;i++){
			if(max_uA == ldo12CurTable[i]){
				index= i;
				break;
			}
		}
		if(index<0){
			dev_err(info->dev, "et59041c_set_current_limit value [%d] not in tabs\n", max_uA);
			return -EINVAL;
		}
		if(id == ET59041C_ID_REGULATOR1){
			val = index & 0x03;
			val2 = (index>>2) & 0x03;
		}else if(id == ET59041C_ID_REGULATOR2){
			val = index & 0x03;
			val2 = index & 0x0c;
		}

	} else {
		for(i=0;i<16;i++){
			if(max_uA == ldo34CurTable[i]){
				index= i;
				break;
			}
		}
		if(index<0){
			dev_err(info->dev, "et59041c_set_current_limit value [%d] not in tabs\n", max_uA);
			return -EINVAL;
		}

		if(id == ET59041C_ID_REGULATOR3){
			val = (index>>2) & 0x03;
			val2 = (index<<2) & 0x30;
		}else if(id == ET59041C_ID_REGULATOR4){
			val = (index>>2) & 0x03;
			val2 =  (index<<4) & 0xc0;
		}
	}
	err = regmap_update_bits(rdev->regmap, ET59041C_REG_ILIMIT,
				 val, val);
	if(err<0){
		dev_err(info->dev, "Failed to regmap_update_bits ET59041C_REG_ILIMIT: %d\n", err);
		return -EINVAL;
	}
	regmap_update_bits(rdev->regmap, ET59041C_REG_ILIMT_COAR,
			val2, val2);
	if(err<0){
		dev_err(info->dev, "Failed to regmap_update_bits ET59041C_REG_ILIMT_COAR: %d\n", err);
		return -EINVAL;
	}
	return 0;
}

static int et59041c_get_current_limit(struct regulator_dev *rdev)
{
	struct et59041c_regulator *info = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int ret;
	int val = 0;
	int val2 = 0;
	int cur = 0;
	ret = regmap_read(rdev->regmap, ET59041C_REG_ILIMIT, &val);
	if (ret < 0) {
		dev_err(info->dev, "Failed to ET59041C_REG_ILIMIT: %d\n", ret);
		return -EINVAL;
	}
	ret = regmap_read(rdev->regmap, ET59041C_REG_ILIMT_COAR, &val2);
	if (ret < 0) {
		dev_err(info->dev, "Failed to ET59041C_REG_ILIMT_COAR: %d\n", ret);
		return -EINVAL;
	}
	if (id == ET59041C_ID_REGULATOR1 || id == ET59041C_ID_REGULATOR2) {
		if(id == ET59041C_ID_REGULATOR1){
			val2 = (0x03 & val2)<<2;
			val2 = val2 | val;


		} else if(id == ET59041C_ID_REGULATOR2){
			val2 = 0x0c & val2;
			val2 = val2 | val;

		}
		if(val2>15){
				val2 = 0;
			}
		cur = ldo12CurTable[val2];
		ET_DBG("cur[%d] = %d\n", id, cur);
		return cur;

	} else {
		if(id == ET59041C_ID_REGULATOR3){
			val2 = (0x30 & val2)>>2;
			val2 = val2 | val;


		} else if(id == ET59041C_ID_REGULATOR4){
			val2 = (0xc0 & val2)>>4;
			val2 = val2 | val;

		}
		if(val2>15){
				val2 = 0;
			}
		cur = ldo34CurTable[val2];
		ET_DBG("cur[%d] = %d\n", id, cur);
		return cur;

	}
}


// .get_voltage_sel = et59041c_get_voltage_regulator,
// .set_voltage_sel = et59041c_set_voltage_regulator,
static const struct regulator_ops et59041c_regulator_ops = {
	.enable = et59041c_regulator_enable,
	.disable = et59041c_regulator_disable,
	.is_enabled = et59041c_regulator_is_enabled,
	.set_voltage = et59041c_set_voltage,
	.list_voltage = et59041c_voltage_list,
	.get_voltage = et59041c_voltage_get,
	.set_current_limit = et59041c_set_current_limit,
	.get_current_limit = et59041c_get_current_limit,
};

int getIdByReg(int reg, int reg_val)
{
	if (reg == ET59041C_REG_LDO_EN ) {
		if ((reg_val & ET59041C_BIT_0) == ET59041C_BIT_0) {
			return ET59041C_ID_REGULATOR1;
		}
		if ((reg_val & ET59041C_BIT_1) == ET59041C_BIT_1) {
			return ET59041C_ID_REGULATOR2;
		}
		if ((reg_val & ET59041C_BIT_2) == ET59041C_BIT_2) {
			return ET59041C_ID_REGULATOR3;
		}
		if ((reg_val & ET59041C_BIT_3) == ET59041C_BIT_3) {
			return ET59041C_ID_REGULATOR4;
		}

	}
	return -1;
}



#define REGULATOR_DESC
#ifdef REGULATOR_DESC
static struct regulator_desc et59041c_regulator_desc[ET59041C_MAX_REGULATORS] = {
	{
		.name = "et59041c_ldo1",
		.id = ET59041C_ID_REGULATOR1,
		.ops = &et59041c_regulator_ops,
		.vsel_reg = ET59041C_REG_DVO1,
		// .n_voltages = 1,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.vsel_mask = BIT(0),
	},
	{
		.name = "et59041c_ldo2",
		.id = ET59041C_ID_REGULATOR2,
		.ops = &et59041c_regulator_ops,
		.vsel_reg = ET59041C_REG_DVO2,
		// .n_voltages = 1,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.vsel_mask = BIT(1),
	},
	{
		.name = "et59041c_ldo3",
		.id = ET59041C_ID_REGULATOR3,
		.ops = &et59041c_regulator_ops,
		.vsel_reg = ET59041C_REG_AVO1,
		// .n_voltages = 1,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.vsel_mask = BIT(2),
	},
	{
		.name = "et59041c_ldo4",
		.id = ET59041C_ID_REGULATOR4,
		.ops = &et59041c_regulator_ops,
		.vsel_reg = ET59041C_REG_AVO2,
		// .n_voltages = 1,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.vsel_mask = BIT(3),
	}

};

static int et59041c_regulator_init(struct et59041c_regulator *chip)
{
	// struct regulator_init_data *init_data = dev_get_platdata(&i2c->dev);
	struct regulator_config config = {};
	// struct regulator_desc *desc ;
	int i, ret;

	// ET59041C SOFTRESET  disable ldos
	ret = regmap_write(chip->regmap, ET59041C_REG_LDO_EN, 0x80);
	if (ret < 0) {
		dev_err(chip->dev,
			"Failed to set ET59041C_REG_RESET reg: %d\n",
			ret);
		return ret;
	}
	// unsigned int data;
	/* Set up regulators */

	/* Register the regulators */
	for (i = 0; i < ET59041C_MAX_REGULATORS; i++) {
		config.init_data = chip->pdata->init_data[i];
		config.dev = chip->dev;
		config.driver_data = chip;
		config.regmap = chip->regmap;
		config.of_node = chip->pdata->reg_node[i];

		chip->rdev[i] = devm_regulator_register(
			chip->dev, &et59041c_regulator_desc[i], &config);
		if (IS_ERR(chip->rdev[i])) {
			ret = PTR_ERR(chip->rdev[i]);
			//	dev_err(chip->rdev[i], "error");
			dev_err(chip->dev,
				"Failed to register ET59041C regulator[ %s: %d]\n",
				et59041c_regulator_desc[i].name, ret);
			return ret;
		}
	}

	return 0;
}
#endif

#define ET59041C_REGULATOR_MATCH_EN

#ifdef ET59041C_REGULATOR_MATCH_EN
/**
 * regulator_match
 *
 *
 *
 */

__attribute__((unused)) static struct of_regulator_match
	et59041c_regulator_matches[ET59041C_MAX_REGULATORS] = {
		[ET59041C_ID_REGULATOR1] = { .name = "et59041c_ldo1" },
		[ET59041C_ID_REGULATOR2] = { .name = "et59041c_ldo2" },
		[ET59041C_ID_REGULATOR3] = { .name = "et59041c_ldo3" },
		[ET59041C_ID_REGULATOR4] = { .name = "et59041c_ldo4" },


	};

/**
 * parse dt tree data to et59041c regulator
 *
 *
 *
 */

static struct et59041c_pdata *et59041c_regulator_parse_dt(struct device *dev)
{
	struct et59041c_pdata *pdata;
	struct device_node *node;
	int i, ret, n;
	struct device_node *np = dev->of_node;
	// struct et59041c_regulator *info = dev_get_drvdata(dev);

	if (np == NULL) {
		pr_err("Error: et-changer np = NULL\n");
		return ERR_PTR(-ENODEV);
	}

	// info->reset_gpio = of_get_named_gpio(np, "et59041c,reset", 0);
	// ret = gpio_request(info->reset_gpio, "ET59041C RESET CONTRL");
	// if (ret < 0) {
	// 	pr_err("Error: failed to request RESET GPIO %d\n", info->reset_gpio);
	// 	return ERR_PTR(-ENODEV);
	// }
	// ret = gpio_direction_output(info->reset_gpio, 1);
	// if (ret < 0) {
	// 	pr_err("Error: failed to set RESET GPIO as output pin\n");
	// 	return ERR_PTR(-ENODEV);
	// }

	node = of_get_child_by_name(dev->of_node, "regulators");
	// struct of_regulator_match *match;

	if (!node) {
		dev_err(dev, "regulator node not found\n");
		return ERR_PTR(-ENODEV);
	}

	ret = of_regulator_match(dev, node, et59041c_regulator_matches,
				 ARRAY_SIZE(et59041c_regulator_matches));
	of_node_put(node);
	if (ret < 0) {
		dev_err(dev, "Error parsing regulator init data: %d\n", ret);
		return ERR_PTR(-EINVAL);
	}
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	n = 0;

	for (i = 0; i < ARRAY_SIZE(et59041c_regulator_matches); i++) {
		if (!et59041c_regulator_matches[i].init_data)
			continue;

		pdata->init_data[n] = et59041c_regulator_matches[i].init_data;
		pdata->reg_node[n] = et59041c_regulator_matches[i].of_node;
		n++;
	}

	return pdata;
}
#endif

/*
 * I2C driver interface functions
 */
static int et59041c_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	int error, ret;
	unsigned int data;
	struct et59041c_regulator *chip;
	struct pinctrl *pinctrl;
	struct pinctrl_state *et59041c_gpio_ext_buck_en;
	struct pinctrl_state *et59041c_gpio_en;
	pinctrl = devm_pinctrl_get(&i2c->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		dev_err(&i2c->dev, "Cannot find pinctrl!\n");
		//return ret;
	} else {
		et59041c_gpio_en = pinctrl_lookup_state(pinctrl, "et59041c_gpio_en0");
		if (IS_ERR(et59041c_gpio_en)) {
			ret = PTR_ERR(et59041c_gpio_en);
			dev_err(&i2c->dev, "Cannot find pinctrl et59041c_gpio_en!\n");
		} else {
			pinctrl_select_state(pinctrl, et59041c_gpio_en);
		}
		et59041c_gpio_ext_buck_en = pinctrl_lookup_state(pinctrl, "et59041c_gpio_ext_buck_en1");
		if (IS_ERR(et59041c_gpio_ext_buck_en)) {
			ret = PTR_ERR(et59041c_gpio_ext_buck_en);
			dev_err(&i2c->dev, "Cannot find pinctrl et59041c_gpio_ext_buck_en!\n");
		} else {
			pinctrl_select_state(pinctrl, et59041c_gpio_ext_buck_en);
		}
	}

	chip = devm_kzalloc(&i2c->dev, sizeof(struct et59041c_regulator),
			    GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &i2c->dev;
	chip->regmap = devm_regmap_init_i2c(i2c, &et59041c_regmap_config);
	if (IS_ERR(chip->regmap)) {
		error = PTR_ERR(chip->regmap);
		dev_err(chip->dev, "Failed to allocate register map: %d\n",
			error);
		return error;
	}

	i2c_set_clientdata(i2c, chip);

	chip->pdata = i2c->dev.platform_data;

	ret = regmap_read(chip->regmap, ET59041C_REG_CHIPID, &data);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read DEVICE_ID reg: %d\n", ret);
		return ret;
	}

	switch (data) {
	case ET59041C_DEVICE_ID:
		chip->chip_id = ET59041C;
		break;
	default:
		dev_err(chip->dev, "Unsupported device id = 0x%x.\n", data);
		return -ENODEV;
	}

	if (!chip->pdata)
		chip->pdata = et59041c_regulator_parse_dt(chip->dev);

	if (IS_ERR(chip->pdata)) {
		dev_err(chip->dev, "No regulators defined for the platform\n");
		return PTR_ERR(chip->pdata);
	}

	// if (IS_ERR(chip->pdata)) {
	// 	dev_err(chip->dev, "No regulators defined for the platform\n");
	// 	return PTR_ERR(chip->pdata);
	// }

	et59041c_init_sysfs(chip);

#ifdef REGULATOR_DESC

	ret = et59041c_regulator_init(chip);
#endif
	if (ret < 0) {
		dev_err(chip->dev, "Failed to initialize regulator: %d\n", ret);
		goto error;
	}

	dev_info(chip->dev, "initialize regulator success");

error:
	return ret;
}

static const struct i2c_device_id et59041c_i2c_id[] = {
	{ "et59041c", ET59041C },
	{},
};
MODULE_DEVICE_TABLE(i2c, et59041c_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id et59041c_dt_ids[] = {
	{ .compatible = "etek,et59041c", .data = &et59041c_i2c_id[0] },
	{},
};
MODULE_DEVICE_TABLE(of, et59041c_dt_ids);
#endif

static struct i2c_driver et59041c_regulator_driver = {
	.driver = {
		.name = "et59041c",
		.of_match_table = of_match_ptr(et59041c_dt_ids),
	},
	.probe = et59041c_i2c_probe,
	.id_table = et59041c_i2c_id,
};

module_i2c_driver(et59041c_regulator_driver);

MODULE_AUTHOR("Sommer Jiang <jiangs@etek.com.cn>");
MODULE_DESCRIPTION("Regulator device driver for ETEK ET59041C");
MODULE_LICENSE("GPL");
