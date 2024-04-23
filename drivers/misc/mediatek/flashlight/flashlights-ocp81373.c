/*
* Copyright (C) 2019 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/list.h>
#include "flashlight.h"
#include "flashlight-dt.h"
#include "flashlight-core.h"
#include <linux/pinctrl/consumer.h>



/* device tree should be defined in flashlight-dt.h */
#ifndef OCP81373_DTNAME
#define OCP81373_DTNAME "mediatek,flashlights_ocp81373"
#endif
#ifndef OCP81373_DTNAME_I2C
#define OCP81373_DTNAME_I2C "mediatek,flashlights_ocp81373"
#endif
#define OCP81373_NAME "flashlights-ocp81373"

#define OCP81373_DRIVER_VERSION "V1.0.2"

#define OCP81373_REG_BOOST_CONFIG     (0x07)
#define OCP81373_BIT_SOFT_RST_MASK    (~(1<<7))
#define OCP81373_BIT_SOFT_RST_ENABLE  (1<<7)
#define OCP81373_BIT_SOFT_RST_DISABLE (0<<7)

/* define registers */
#define OCP81373_REG_ENABLE           (0x01)
#define OCP81373_MASK_ENABLE_LED1     (0x01)
#define OCP81373_MASK_ENABLE_LED2     (0x02)
#define OCP81373_DISABLE              (0x00)
#define OCP81373_ENABLE_LED1          (0x01)
#define OCP81373_ENABLE_LED1_TORCH    (0x09)
#define OCP81373_ENABLE_LED1_FLASH    (0x0D)
#define OCP81373_ENABLE_LED2          (0x02)
#define OCP81373_ENABLE_LED2_TORCH    (0x0A)
#define OCP81373_ENABLE_LED2_FLASH    (0x0E)
#define OCP81373_DEVICES_ID    	      (0x0C)

#define OCP81373_REG_TORCH_LEVEL_LED1 (0x05)
#define OCP81373_REG_FLASH_LEVEL_LED1 (0x03)
#define OCP81373_REG_TORCH_LEVEL_LED2 (0x06)
#define OCP81373_REG_FLASH_LEVEL_LED2 (0x04)

#define OCP81373_REG_TIMING_CONF      (0x08)
#define OCP81373_TORCH_RAMP_TIME      (0x10)
#define OCP81373_FLASH_TIMEOUT        (0x0F)
#define OCP81373_CHIP_STANDBY         (0x80)

#define OCP81373_REG_FLAG1            (0x0A)
#define OCP81373_REG_FLAG2            (0x0B)

/* define channel, level */
#define OCP81373_CHANNEL_NUM          2
#define OCP81373_CHANNEL_CH1          0
#define OCP81373_CHANNEL_CH2          1
#define OCP81373_LEVEL_NUM            26
#define OCP81373_LEVEL_TORCH          7

#define OCP81373_HW_TIMEOUT 600 /* ms */


#define AW_I2C_RETRIES			5
#define AW_I2C_RETRY_DELAY		2

/* define mutex and work queue */
static DEFINE_MUTEX(ocp81373_mutex);

struct i2c_client *ocp81373_flashlight_client;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *ocp81373_i2c_client;


/* define pinctrl */
#define OCP81373_PINCTRL_PIN_HWEN 0
#define OCP81373_PINCTRL_PINSTATE_LOW 0
#define OCP81373_PINCTRL_PINSTATE_HIGH 1
#define OCP81373_PINCTRL_STATE_HWEN_HIGH "hwen-high"
#define OCP81373_PINCTRL_STATE_HWEN_LOW  "hwen-low"

/* charger status */
#define FLASHLIGHT_CHARGER_NOT_READY 0
#define FLASHLIGHT_CHARGER_READY     1


struct pinctrl *ocp81373_hwen_pinctrl;
struct pinctrl_state *ocp81373_hwen_high;
struct pinctrl_state *ocp81373_hwen_low;


static int ocp81373_get_flag(int num)
{
	if (num == 1)
		return i2c_smbus_read_byte_data(ocp81373_i2c_client, OCP81373_REG_FLAG1);
	else if (num == 2)
		return i2c_smbus_read_byte_data(ocp81373_i2c_client, OCP81373_REG_FLAG2);

	pr_info("Error num\n");
	return 0;
}


static int ocp81373_pinctrl_init(struct i2c_client *client)
{
	int ret = 0;

	/* get pinctrl */
	ocp81373_hwen_pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(ocp81373_hwen_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(ocp81373_hwen_pinctrl);
		return ret;
	}

	/* Flashlight HWEN pin initialization */
	ocp81373_hwen_high = pinctrl_lookup_state(
			ocp81373_hwen_pinctrl,
			OCP81373_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(ocp81373_hwen_high)) {
		pr_err("Failed to init (%s)\n",
			OCP81373_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(ocp81373_hwen_high);
	}
	ocp81373_hwen_low = pinctrl_lookup_state(
			ocp81373_hwen_pinctrl,
			OCP81373_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(ocp81373_hwen_low)) {
		pr_err("Failed to init (%s)\n", OCP81373_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(ocp81373_hwen_low);
	}

	return ret;
}


static int ocp81373_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(ocp81373_hwen_pinctrl)) {
		pr_info("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case OCP81373_PINCTRL_PIN_HWEN:
		if (state == OCP81373_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(ocp81373_hwen_low))
			pinctrl_select_state(ocp81373_hwen_pinctrl,
					ocp81373_hwen_low);
		else if (state == OCP81373_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(ocp81373_hwen_high))
			pinctrl_select_state(ocp81373_hwen_pinctrl,
					ocp81373_hwen_high);
		else
			pr_info("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_info("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}

	return ret;
}


/* platform data
* torch_pin_enable: TX1/TORCH pin isa hardware TORCH enable
* pam_sync_pin_enable: TX2 Mode The ENVM/TX2 is a PAM Sync. on input
* thermal_comp_mode_enable: LEDI/NTC pin in Thermal Comparator Mode
* strobe_pin_disable: STROBE Input disabled
* vout_mode_enable: Voltage Out Mode enable
*/
struct ocp81373_platform_data {
	u8 torch_pin_enable;
	u8 pam_sync_pin_enable;
	u8 thermal_comp_mode_enable;
	u8 strobe_pin_disable;
	u8 vout_mode_enable;
};

/* ocp81373 chip data */
struct ocp81373_chip_data {
	struct i2c_client *client;
	struct ocp81373_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};

/******************************************************************************
 * ocp81373 operations
 *****************************************************************************/
static const int ocp81373_current[OCP81373_LEVEL_NUM] = {
	 22,  46,  70,  93,  116, 140, 163, 198, 245, 304,
	351, 398, 445, 503,  550, 597, 656, 703, 750, 796,
	855, 902, 949, 996, 1054, 1101
};

static const unsigned char ocp81373_torch_level[OCP81373_LEVEL_NUM] = {
	0x1C, 0x1F, 0x23, 0x26, 0x29, 0x2D, 0x30, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned char ocp81373_flash_level[OCP81373_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x50, 0x54, 0x59, 0x5D
};


static volatile unsigned char ocp81373_reg_enable;
static volatile int ocp81373_level_ch1 = -1;
static volatile int ocp81373_level_ch2 = -1;

static int ocp81373_is_torch(int level)
{

	if (level >= OCP81373_LEVEL_TORCH)
		return -1;

	return 0;
}

static int ocp81373_verify_level(int level)
{

	if (level < 0)
		level = 0;
	else if (level >= OCP81373_LEVEL_NUM)
		level = OCP81373_LEVEL_NUM - 1;

	return level;
}

/* i2c wrapper function */
static int ocp81373_i2c_write(struct i2c_client *client, unsigned char reg, unsigned char val)
{
	int ret;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret < 0) {
			pr_info("%s: i2c_write addr=0x%02X, data=0x%02X, cnt=%d, error=%d\n",
				   __func__, reg, val, cnt, ret);
		} else {
			pr_info("%s: i2c_write addr=0x%02X, data=0x%02X, cnt=%d, ret=%d\n",
				   __func__, reg, val, cnt, ret);
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int ocp81373_i2c_read(struct i2c_client *client, unsigned char reg, unsigned char *val)
{
	int ret;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0) {
			pr_info("%s: i2c_read addr=0x%02X, cnt=%d, error=%d\n",
				   __func__, reg, cnt, ret);
		} else {
			*val = ret;
			pr_info("%s: i2c_read addr=0x%02X, cnt=%d, ret=%d\n",
				   __func__, reg, cnt, ret);
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static void ocp81373_soft_reset(void)
{
	unsigned char reg_val;

	ocp81373_i2c_read(ocp81373_i2c_client, OCP81373_REG_BOOST_CONFIG, &reg_val);
	reg_val &= OCP81373_BIT_SOFT_RST_MASK;
	reg_val |= OCP81373_BIT_SOFT_RST_ENABLE;
	ocp81373_i2c_write(ocp81373_i2c_client, OCP81373_REG_BOOST_CONFIG, reg_val);
	msleep(5);
}

/* flashlight enable function */
static int ocp81373_enable_ch1(void)
{
	unsigned char reg, val;

	reg = OCP81373_REG_ENABLE;
	if (!ocp81373_is_torch(ocp81373_level_ch1)) {
		/* torch mode */
		ocp81373_reg_enable |= OCP81373_ENABLE_LED1_TORCH;
	} else {
		/* flash mode */
		ocp81373_reg_enable |= OCP81373_ENABLE_LED1_FLASH;
	}

	val = ocp81373_reg_enable;

	return ocp81373_i2c_write(ocp81373_i2c_client, reg, val);
}

static int ocp81373_enable_ch2(void)
{
	unsigned char reg, val;

	reg = OCP81373_REG_ENABLE;
	if (!ocp81373_is_torch(ocp81373_level_ch2)) {
		/* torch mode */
		ocp81373_reg_enable |= OCP81373_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		ocp81373_reg_enable |= OCP81373_ENABLE_LED2_FLASH;
	}
	val = ocp81373_reg_enable;

	return ocp81373_i2c_write(ocp81373_i2c_client, reg, val);
}

static int ocp81373_enable(int channel)
{

	if (channel == OCP81373_CHANNEL_CH1)
		ocp81373_enable_ch1();
	else if (channel == OCP81373_CHANNEL_CH2)
		ocp81373_enable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight disable function */
static int ocp81373_disable_ch1(void)
{
	unsigned char reg, val;

	reg = OCP81373_REG_ENABLE;
	if (ocp81373_reg_enable & OCP81373_MASK_ENABLE_LED2) {
		/* if LED 2 is enable, disable LED 1 */
		ocp81373_reg_enable &= (~OCP81373_ENABLE_LED1);
	} else {
		/* if LED 2 is enable, disable LED 1 and clear mode */
		ocp81373_reg_enable &= (~OCP81373_ENABLE_LED1_FLASH);
	}
	val = ocp81373_reg_enable;

	return ocp81373_i2c_write(ocp81373_i2c_client, reg, val);
}

static int ocp81373_disable_ch2(void)
{
	unsigned char reg, val;

	reg = OCP81373_REG_ENABLE;
	if (ocp81373_reg_enable & OCP81373_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		ocp81373_reg_enable &= (~OCP81373_ENABLE_LED2);
	} else {
		/* if LED 1 is enable, disable LED 2 and clear mode */
		ocp81373_reg_enable &= (~OCP81373_ENABLE_LED2_FLASH);
	}
	val = ocp81373_reg_enable;

	return ocp81373_i2c_write(ocp81373_i2c_client, reg, val);
}

static int ocp81373_disable(int channel)
{

	if (channel == OCP81373_CHANNEL_CH1)
		ocp81373_disable_ch1();
	else if (channel == OCP81373_CHANNEL_CH2)
		ocp81373_disable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* set flashlight level */
static int ocp81373_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val = 0;
	unsigned char device_id;
	int dev_id;

	dev_id = ocp81373_i2c_read(ocp81373_i2c_client, OCP81373_DEVICES_ID, &device_id);
	pr_info("%s lllyOCP81373_DEVICES_ID %d\n", __func__, dev_id);

	level = ocp81373_verify_level(level);

	/* set torch brightness level */
	reg = OCP81373_REG_TORCH_LEVEL_LED1;
	/*select flash ic by devices id*/
	if(dev_id == 10){
	val = ocp81373_torch_level[level];
	pr_info(" lllythis is OCP81373_torch \n");
	}
	else if(dev_id == 58){
	val = ocp81373_torch_level[level];
	pr_info(" lllythis is OCP18373_torch \n");
	}

	ret = ocp81373_i2c_write(ocp81373_i2c_client, reg, val);

	ocp81373_level_ch1 = level;

	/* set flash brightness level */
	reg = OCP81373_REG_FLASH_LEVEL_LED1;
	if(dev_id == 10){
	val = ocp81373_flash_level[level];
	pr_info(" lllythis is OCP81373_flash \n");
	}
	else if(dev_id == 58){
	val = ocp81373_flash_level[level];
	pr_info(" lllythis is OCP18373_flash \n");
	ret = ocp81373_i2c_write(ocp81373_i2c_client, reg, val);
	}
	return ret;
}

int ocp81373_set_level_ch2(int level)
{
	int ret;
	unsigned char reg, val = 0;
	unsigned char device_id;
	int dev_id;

	dev_id = ocp81373_i2c_read(ocp81373_i2c_client, OCP81373_DEVICES_ID, &device_id);
	pr_info("%s lllyOCP81373_DEVICES_ID %d\n", __func__, dev_id);

	level = ocp81373_verify_level(level);

	/* set torch brightness level */
	reg = OCP81373_REG_TORCH_LEVEL_LED2;
	/*select flash ic by devices id*/
	if(dev_id == 10){
	val = ocp81373_torch_level[level];
	printk(" lllythis is OCP81373_torch\n");
	}
	else if(dev_id == 58){
	val = ocp81373_torch_level[level];
	printk("lllythis is OCP18373_torch\n");
	}
	ret = ocp81373_i2c_write(ocp81373_i2c_client, reg, val);

	ocp81373_level_ch2 = level;

	/* set flash brightness level */
	reg = OCP81373_REG_FLASH_LEVEL_LED2;
	if(dev_id == 10){
	val = ocp81373_flash_level[level];
	printk(" lllythis is OCP81373_flash \n");
	}
	else if(dev_id == 58){
	val = ocp81373_flash_level[level];
	printk(" lllythis is OCP18373_flash \n");
	ret = ocp81373_i2c_write(ocp81373_i2c_client, reg, val);
	}
	return ret;
}

static int ocp81373_set_level(int channel, int level)
{
	if (channel == OCP81373_CHANNEL_CH1)
		ocp81373_set_level_ch1(level);
	else if (channel == OCP81373_CHANNEL_CH2)
		ocp81373_set_level_ch2(level);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight init */
int ocp81373_init(void)
{
	int ret;
	unsigned char reg, val;

	usleep_range(2000, 2500);

	/* clear enable register */
	reg = OCP81373_REG_ENABLE;
	val = OCP81373_DISABLE;
	ret = ocp81373_i2c_write(ocp81373_i2c_client, reg, val);

	ocp81373_reg_enable = val;

	/* set torch current ramp time and flash timeout */
	reg = OCP81373_REG_TIMING_CONF;
	val = OCP81373_TORCH_RAMP_TIME | OCP81373_FLASH_TIMEOUT;
	ret = ocp81373_i2c_write(ocp81373_i2c_client, reg, val);

	return ret;
}

/* flashlight uninit */
int ocp81373_uninit(void)
{
	ocp81373_disable(OCP81373_CHANNEL_CH1);
	ocp81373_disable(OCP81373_CHANNEL_CH2);

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int ocp81373_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= OCP81373_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_info("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		ocp81373_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			ocp81373_enable(channel);
		} else {
			ocp81373_disable(channel);
		}
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_info("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = OCP81373_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_info("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = OCP81373_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = ocp81373_verify_level(fl_arg->arg);
		pr_info("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = ocp81373_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_info("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = OCP81373_HW_TIMEOUT;
		break;

	case FLASH_IOC_IS_CHARGER_READY:
  		pr_info("FLASH_IOC_IS_CHARGER_READY(%d)\n", channel);
 		fl_arg->arg = FLASHLIGHT_CHARGER_READY;
  		pr_info("FLASH_IOC_IS_CHARGER_READY(%d)\n", fl_arg->arg);
  		break;

	case FLASH_IOC_GET_HW_FAULT:
		pr_info("FLASH_IOC_GET_HW_FAULT(%d)\n", channel);
		fl_arg->arg = ocp81373_get_flag(1);
		break;

	case FLASH_IOC_GET_HW_FAULT2:
		pr_info("FLASH_IOC_GET_HW_FAULT2(%d)\n", channel);
		fl_arg->arg = ocp81373_get_flag(2);
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int ocp81373_open(void)
{
	/* Actual behavior move to set driver function */
	/* since power saving issue */
	return 0;
}

static int ocp81373_release(void)
{
	/* uninit chip and clear usage count */
	mutex_lock(&ocp81373_mutex);
	use_count--;
	if (!use_count)
		ocp81373_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&ocp81373_mutex);

	pr_info("Release: %d\n", use_count);

	return 0;
}

static int ocp81373_set_driver(int set)
{
	/* init chip and set usage count */
	mutex_lock(&ocp81373_mutex);
	if (!use_count)
		ocp81373_init();
	use_count++;
	mutex_unlock(&ocp81373_mutex);

	pr_info("Set driver: %d\n", use_count);

	return 0;
}

static ssize_t ocp81373_strobe_store(struct flashlight_arg arg)
{
	ocp81373_set_driver(1);
	ocp81373_set_level(arg.ct, arg.level);
	ocp81373_enable(arg.ct);
	msleep(arg.dur);
	ocp81373_disable(arg.ct);
	ocp81373_set_driver(0);

	return 0;
}

static struct flashlight_operations ocp81373_ops = {
	ocp81373_open,
	ocp81373_release,
	ocp81373_ioctl,
	ocp81373_strobe_store,
	ocp81373_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int ocp81373_chip_init(struct ocp81373_chip_data *chip)
{
	/* NOTE: Chip initialication move to
	*"set driver" operation for power saving issue.
	* ocp81373_init();
	*/

	return 0;
}

/***************************************************************************/
/*OCP81373 Debug file */
/***************************************************************************/
static ssize_t
ocp81373_get_reg(struct device *cd, struct device_attribute *attr, char *buf)
{
	unsigned char reg_val;
	unsigned char i;
	ssize_t len = 0;

	for (i = 0; i < 0x0E; i++) {
		ocp81373_i2c_read(ocp81373_i2c_client, i ,&reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len,
			"reg0x%2X = 0x%2X\n", i, reg_val);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\r\n");
	return len;
}

static ssize_t ocp81373_set_reg(struct device *cd,
		struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[2];

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		ocp81373_i2c_write(ocp81373_i2c_client, databuf[0], databuf[1]);
	return len;
}

static DEVICE_ATTR(reg, 0660, ocp81373_get_reg, ocp81373_set_reg);

static int ocp81373_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_reg);

	return err;
}

static int
ocp81373_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ocp81373_chip_data *chip;
	struct ocp81373_platform_data *pdata = client->dev.platform_data;
	int err, rval;

	pr_info("%s Probe start.\n", __func__);
	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct ocp81373_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pr_err("Platform data does not exist\n");
		pdata =
		kzalloc(sizeof(struct ocp81373_platform_data), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_init_pdata;
		}
		chip->no_pdata = 1;
	}
	rval = ocp81373_pinctrl_init(client);
	if (rval < 0){
		pr_err("Failed to ocp81373_pinctrl_init\n");
		return rval;
	}

	ocp81373_pinctrl_set(OCP81373_PINCTRL_PIN_HWEN, OCP81373_PINCTRL_PINSTATE_HIGH);

	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	ocp81373_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* soft rst */
	ocp81373_soft_reset();

	/* init chip hw */
	ocp81373_chip_init(chip);

	/* register flashlight operations */
	if (flashlight_dev_register(OCP81373_NAME, &ocp81373_ops)) {
		pr_err("Failed to register flashlight device.\n");
		err = -EFAULT;
		goto err_free;
	}

	/* clear usage count */
	use_count = 0;

	ocp81373_create_sysfs(client);

	pr_info("%s Probe done.\n", __func__);

	return 0;

err_free:
	kfree(chip->pdata);
err_init_pdata:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static void ocp81373_i2c_remove(struct i2c_client *client)
{
	struct ocp81373_chip_data *chip = i2c_get_clientdata(client);

	pr_info("Remove start.\n");

	/* unregister flashlight operations */
	flashlight_dev_unregister(OCP81373_NAME);

	/* free resource */
	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);

	pr_info("Remove done.\n");

	//return 0;
}

static void ocp81373_i2c_shutdown(struct i2c_client *client)
{
	pr_info("ocp81373 shutdown start.\n");

	ocp81373_i2c_write(ocp81373_i2c_client, OCP81373_REG_ENABLE,
						OCP81373_CHIP_STANDBY);

	pr_info("ocp81373 shutdown done.\n");
}


static const struct i2c_device_id ocp81373_i2c_id[] = {
	{OCP81373_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id ocp81373_i2c_of_match[] = {
	{.compatible = OCP81373_DTNAME_I2C},
	{},
};
#endif

static struct i2c_driver ocp81373_i2c_driver = {
	.driver = {
		   .name = OCP81373_NAME,
#ifdef CONFIG_OF
		   .of_match_table = ocp81373_i2c_of_match,
#endif
		   },
	.probe = ocp81373_i2c_probe,
	.remove = ocp81373_i2c_remove,
	.shutdown = ocp81373_i2c_shutdown,
	.id_table = ocp81373_i2c_id,
};

module_i2c_driver(ocp81373_i2c_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joseph <zhangzetao@awinic.com.cn>");
MODULE_DESCRIPTION("AW Flashlight OCP81373 Driver");

