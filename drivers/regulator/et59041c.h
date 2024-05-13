/*
 * et59041c-regulator.h - Regulator definitions for ET59041C
 * Copyright (C) 2021  ETEK Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ET59041C_H__
#define __ET59041C_H__

/* System Control and Event Registers */


#include <linux/regulator/machine.h>


#define ET59041C_REG_CONTROL		0x10
#define ET5904_REG_NUM (ET59041C_REG_CONTROL+1)


/* ET59041C REGULATOR IDs */
// #define ET59041C_ID_REGULATOR1	0
// #define ET59041C_ID_REGULATOR2	1
// #define ET59041C_ID_REGULATOR3	2
// #define ET59041C_ID_REGULATOR4	3


// #define ET59041C_MAX_REGULATORS	7

/* Regulators */
enum {
	ET59041C_ID_REGULATOR1,
	ET59041C_ID_REGULATOR2,
	ET59041C_ID_REGULATOR3,
	ET59041C_ID_REGULATOR4,
	ET59041C_MAX_REGULATORS,
};
enum et59041c_chip_id {
	ET59041C,
};

struct et59041c_pdata {
	/*
	 * Number of regulator
	 */
	struct device_node *reg_node[ET59041C_MAX_REGULATORS];
	struct regulator_init_data *init_data[ET59041C_MAX_REGULATORS];
};

/**
 * et59041c_regulator_data - regulator data
 * @id: regulator id
 * @name: regulator name
 * @init_data: regulator init data
 * @of_node: device tree node (optional)
 */
struct et59041c_regulator_data {
	int id;
	const char *name;
	struct regulator_init_data *init_data;
	struct device_node *of_node;
};

/**
 * et59041c_platform_data - platform data for et59041c
 * @num_regulators: number of regulators used
 * @regulators: pointer to regulators used
 */
struct et59041c_platform_data {
	int num_regulators;
	struct et59041c_regulator_data *regulators;
};


struct et59041c_regulator {
	struct device *dev;
	struct regmap *regmap;
	struct et59041c_pdata *pdata;
	struct regulator_dev *rdev[ET59041C_MAX_REGULATORS];
	int num_regulator;
	int chip_irq;
	int chip_id;
	// int reset_gpio;
};

#define	ET59041C_REG_CHIPID			0x00
#define	ET59041C_REG_ILIMIT			0x01
#define	ET59041C_REG_RDIS			0x02
#define	ET59041C_REG_DVO1			0x03
#define	ET59041C_REG_DVO2			0x04
#define	ET59041C_REG_AVO1			0x05
#define	ET59041C_REG_AVO2			0x06
#define	ET59041C_REG_SEQ1			0x0A
#define	ET59041C_REG_SEQ2			0x0B
#define	ET59041C_REG_LDO_EN			0x0E
#define	ET59041C_REG_SEQ_C			0x0F
#define	ET59041C_REG_ILIMT_COAR			0x10
#define ET59041C_LDO_REG_INDEX(n)		(ET59041C_REG_DVO1 + (n))

/* DEVICE IDs */
#define ET59041C_DEVICE_ID	0x00

#define ET59041C_BIT_0		(1 << 0)
#define ET59041C_BIT_1		(1 << 1)
#define ET59041C_BIT_2		(1 << 2)
#define ET59041C_BIT_3		(1 << 3)
#define ET59041C_BIT_4		(1 << 4)
#define ET59041C_BIT_5		(1 << 5)
#define ET59041C_BIT_6		(1 << 6)
#define ET59041C_BIT_7		(1 << 7)

#endif	/* __ET59041C_REGISTERS_H__ */
