// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>


#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#include "include/dsi-panel-mot-csot-vtdr6130-636-fhdp-dphy-cmd-120hz-lhbm-alpha.h"
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mediatek_v2/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif

#define FRAME_WIDTH				(1200)
#define FRAME_HEIGHT			(2670)
#define PLL_CLOCK				(540)
#define REAL_MODE_NUM           (6)

#define FHD_FRAME_WIDTH    (1200)
#define FHD_HFP            (15)
#define FHD_HSA            (15)
#define FHD_HBP            (15)
#define FHD_HTOTAL         (FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP)
#define FHD_FRAME_HEIGHT   (2670)
#define FHD_VFP            (45)
#define FHD_VSA            (2)
#define FHD_VBP            (35)
#define FHD_VTOTAL         (FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP)
#define FHD_FRAME_TOTAL    (FHD_HTOTAL * FHD_VTOTAL)
#define FHD_PLL_CLOCK      (390)
#define FHD_VREFRESH_DEF   (120)
#define FHD_VREFRESH_60    (60)
#define FHD_VREFRESH_90    (90)
#define FHD_VREFRESH_30    (30)
#define FHD_VREFRESH_24    (24)
#define FHD_VREFRESH_10    (10)
#define FHD_VREFRESH_1    (1)
#define FHD_CLK_DEF_X10    ((FHD_FRAME_TOTAL * FHD_VREFRESH_DEF) / 100)
#define FHD_CLK_60_X10     ((FHD_FRAME_TOTAL * FHD_VREFRESH_60) / 100)
#define FHD_CLK_90_X10     ((FHD_FRAME_TOTAL * FHD_VREFRESH_90) / 100)
#define FHD_CLK_30_X10     ((FHD_FRAME_TOTAL * FHD_VREFRESH_30) / 100)
#define FHD_CLK_24_X10     ((FHD_FRAME_TOTAL * FHD_VREFRESH_24) / 100)
#define FHD_CLK_10_X10     ((FHD_FRAME_TOTAL * FHD_VREFRESH_10) / 100)
#define FHD_CLK_1_X10      ((FHD_FRAME_TOTAL * FHD_VREFRESH_1) / 100)
#define FHD_CLK_DEF		(((FHD_CLK_DEF_X10 % 10) != 0) ?             \
			(FHD_CLK_DEF_X10 / 10 + 1) : (FHD_CLK_DEF_X10 / 10))
#define FHD_CLK_90		(((FHD_CLK_90_X10 % 10) != 0) ?              \
			(FHD_CLK_90_X10 / 10 + 1) : (FHD_CLK_90_X10 / 10))
#define FHD_CLK_60		(((FHD_CLK_60_X10 % 10) != 0) ?              \
			(FHD_CLK_60_X10 / 10 + 1) : (FHD_CLK_60_X10 / 10))
#define FHD_CLK_30		(((FHD_CLK_30_X10 % 10) != 0) ?              \
			(FHD_CLK_30_X10 / 10 + 1) : (FHD_CLK_30_X10 / 10))
#define FHD_CLK_24		(((FHD_CLK_24_X10 % 10) != 0) ?              \
			(FHD_CLK_24_X10 / 10 + 1) : (FHD_CLK_24_X10 / 10))
#define FHD_CLK_10		(((FHD_CLK_10_X10 % 10) != 0) ?              \
			(FHD_CLK_10_X10 / 10 + 1) : (FHD_CLK_10_X10 / 10))
#define FHD_CLK_1		(((FHD_CLK_1_X10 % 10) != 0) ?              \
			(FHD_CLK_1_X10 / 10 + 1) : (FHD_CLK_1_X10 / 10))

#define MODE_SWITCH_CMDQ_ENABLE 1

#define FHD_HFP_90            (15)
#define FHD_HSA_90            (15)
#define FHD_HBP_90            (15)
#define FHD_VFP_90            (989)
#define FHD_VSA_90            (2)
#define FHD_VBP_90            (35)

struct mtk_mode_switch_cmd cmd_table_120fps[] = {
	{2, {0x6C, 0x01}},
	{2, {0x71, 0x00}}
};

struct mtk_mode_switch_cmd cmd_table_90fps[] = {
	{2, {0x6C, 0x02}},
	{2, {0x71,0x00}}
};

struct mtk_mode_switch_cmd cmd_table_60fps[] = {
	{2, {0x6C, 0x01}},
	{4, {0x71,0x01,0x01,0x00}}
};

struct mtk_mode_switch_cmd cmd_table_30fps[] = {
	{2, {0x6C, 0x01}},
	{4, {0x71,0x01,0x03,0x00}}
};

struct mtk_mode_switch_cmd cmd_table_24fps[] = {
	{2, {0x6C, 0x01}},
	{4, {0x71,0x01,0x04,0x00}}
};

struct mtk_mode_switch_cmd cmd_table_10fps[] = {
	{2, {0x6C, 0x01}},
	{4, {0x71,0x01,0x0B,0x00}}
};

struct mtk_mode_switch_cmd cmd_table_1fps[] = {
	{2, {0x6C, 0x01}},
	{4, {0x71,0x01,0x77,0x00}}
};

static enum RES_SWITCH_TYPE res_switch_type = RES_SWITCH_NO_USE;

unsigned int nt37801_wqhs_dsi_cmd_120hz_dphy_buf_thresh[14] = {
	896, 1792, 2688, 3584, 4480, 5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064};
unsigned int nt37801_wqhs_dsi_cmd_120hz_dphy_range_min_qp[15] = {
	0, 4, 5, 5, 7, 7, 7, 7, 7, 7, 9, 9, 9, 13, 16};
unsigned int nt37801_wqhs_dsi_cmd_120hz_dphy_range_max_qp[15] = {
	8, 8, 9, 10, 11, 11, 11, 12, 13, 14, 14, 15, 15, 16, 17};
int nt37801_wqhs_dsi_cmd_120hz_dphy_range_bpg_ofs[15] = {
	2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -12, -12, -12, -12};

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	//struct gpio_desc *bias_pos, *bias_neg;
	struct gpio_desc *vddi_gpio;
	struct gpio_desc *dvdd_gpio;
	struct regulator *oled_vci;

	bool prepared;
	bool enabled;
	bool lhbm_en;
	unsigned int gate_ic;

	int error;
	unsigned int hbm_mode;
	unsigned int dc_mode;
	unsigned int current_backlight;
	unsigned int current_fps;
};

static struct lcm *g_ctx = NULL;
#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;

	if (ret == 0) {
		ret = lcm_dcs_read(ctx,  0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}

static int lcm_panel_get_ab_data(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	u8 buffer[3] = {0};
	int ret;

	if (!ctx->enabled)
		return 0;

	ret = lcm_dcs_read(ctx,  0xAB, buffer, 1);
	dev_info(ctx->dev, "return %d data(0x%08x) to 0xAB\n",
		 ret, buffer[0] | (buffer[1] << 8));
	ret = lcm_dcs_read(ctx,  0x0A, buffer, 1);
	dev_info(ctx->dev, "return %d data(0x%08x) to 0x0A\n",
		 ret, buffer[0] | (buffer[1] << 8));

	lcm_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00);
	ret = lcm_dcs_read(ctx,  0xC3, buffer, 1);
	dev_info(ctx->dev, "return %d data(0x%08x) to 0xC3\n",
		 ret, buffer[0] | (buffer[1] << 8));
	ret = lcm_dcs_read(ctx,  0xEA, buffer, 1);
	dev_info(ctx->dev, "return %d data(0x%08x) to 0xEA\n",
		 ret, buffer[0] | (buffer[1] << 8));

	return ret;
}

#endif

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
static struct regulator *disp_bias_pos;
static struct regulator *disp_bias_neg;

static int lcm_panel_bias_regulator_init(void)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(disp_bias_pos);
		pr_err("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		pr_err("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}

static int lcm_panel_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_err("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_err("enable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}

static int lcm_panel_bias_disable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_bias_regulator_init();

	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_err("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_err("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}
#endif

static void lcm_panel_init(struct lcm *ctx)
{
	char bl_tb[] = {0x51, 0x0f, 0xff};
	unsigned int level = 0;
	printk("%s enter  \n",__func__);
	udelay(2000);
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(3 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(1 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(2 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(20);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	lcm_dcs_write_seq_static(ctx, 0x03,0x01);
	lcm_dcs_write_seq_static(ctx, 0x35,0x00); //TE ON
	lcm_dcs_write_seq_static(ctx, 0x53,0x20);//Brightness Control
	lcm_dcs_write_seq_static(ctx, 0x51,0x0D,0xBA);//Normal mode
	lcm_dcs_write_seq_static(ctx, 0x59,0x09);  //Demura oN
	lcm_dcs_write_seq_static(ctx, 0x5e,0x00);
	lcm_dcs_write_seq_static(ctx, 0x6c,0x01);//120HZ
	//lcm_dcs_write_seq_static(ctx, 0x71,0x01,0x01,0x00);//60hz
	lcm_dcs_write_seq_static(ctx, 0x6d,0x00);
	lcm_dcs_write_seq_static(ctx, 0x6f,0x02);
	lcm_dcs_write_seq_static(ctx, 0x70,0x11,0x00,0x00,0xab,0x30,0x80,0x0a,0x6e,0x04,0xb0,0x00,0x1e,0x02,0x58,0x02,0x58,0x02,0x00,0x01,0x19,0x00,0x20,0x05,0xd0,0x00,0x08,0x00,0x01,0x00,0x47,0x03,0x0d,0x18,0x00,0x10,0xf0,0x07,0x10,0x20,0x00,0x06,0x0f,0x0f,0x33,0x0e,0x1c,0x2a,0x38,0x46,0x54,0x62,0x69,0x70,0x77,0x79,0x7b,0x7d,0x7e,0x02,0x02,0x22,0x00,0x2a,0x40,0x2a,0xbe,0x3a,0xfc,0x3a,0xfa,0x3a,0xf8,0x3b,0x38,0x3b,0x78,0x3b,0xb6,0x4b,0xb6,0x4b,0xf4,0x4b,0xf4,0x6c,0x34,0x84,0x74,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0x75,0x00);
	lcm_dcs_write_seq_static(ctx, 0x72,0x00);
	lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x18);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x00);
	lcm_dcs_write_seq_static(ctx, 0xB2,0x00);
	lcm_dcs_write_seq_static(ctx, 0xFF,0x5A,0x80);
	lcm_dcs_write_seq_static(ctx, 0x65,0x0A);
	lcm_dcs_write_seq_static(ctx, 0xF9,0x9E,0x8F);
	lcm_dcs_write_seq_static(ctx, 0x65,0x0F);
	lcm_dcs_write_seq_static(ctx, 0xF9,0x14);
	lcm_dcs_write_seq_static(ctx, 0xFF,0x5A,0x81);
	lcm_dcs_write_seq_static(ctx, 0x65,0x02);
	lcm_dcs_write_seq_static(ctx, 0xFB,0xB3,0xB3,0xB3);

	pr_info("%s current_fps:%d\n", __func__, ctx->current_fps);
	switch (ctx->current_fps) {
	case 120:
		lcm_dcs_write_seq_static(ctx, 0x6C, 0x01);
		lcm_dcs_write_seq_static(ctx, 0x71, 0x00);
		break;
	case 90:
		lcm_dcs_write_seq_static(ctx, 0x6C, 0x02);
		//lcm_dcs_write_seq_static(ctx, 0x71,0x01,0x01,0x00);
		break;
	case 60:
		lcm_dcs_write_seq_static(ctx, 0x6C, 0x01);
		lcm_dcs_write_seq_static(ctx, 0x71,0x01,0x01,0x00);
		break;
	case 30:
		lcm_dcs_write_seq_static(ctx, 0x6C, 0x01);
		lcm_dcs_write_seq_static(ctx, 0x71,0x01,0x03,0x00);
		break;
	case 24:
		lcm_dcs_write_seq_static(ctx, 0x6C, 0x01);
		lcm_dcs_write_seq_static(ctx, 0x71,0x01,0x04,0x00);
		break;
	case 10:
		lcm_dcs_write_seq_static(ctx, 0x6C, 0x01);
		lcm_dcs_write_seq_static(ctx, 0x71,0x01,0x0B,0x00);
		break;
	default:
		pr_info("%s current_fps mismatch:%d\n", __func__, ctx->current_fps);
		break;
	}


	//backlight
	level = ctx->current_backlight;
	bl_tb[1] = (level >> 8) & 0xf;
	bl_tb[2] = level & 0xFF;
	lcm_dcs_write(ctx, bl_tb, ARRAY_SIZE(bl_tb));

	lcm_dcs_write_seq_static(ctx, 0x11);
	msleep(85);
	lcm_dcs_write_seq_static(ctx, 0x29);
	ctx->hbm_mode = 0;
	ctx->dc_mode = 0;
	printk("%s exit  \n",__func__);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	printk("%s enter  \n",__func__);
	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;
	printk("%s exit  \n",__func__);
	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	int ret = 0;
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->prepared)
		return 0;
	printk("%s enter  \n",__func__);
	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(50);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(150);

	ctx->error = 0;
	ctx->prepared = false;

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	udelay(2000);
	ctx->dvdd_gpio =
		devm_gpiod_get(ctx->dev, "dvdd", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->dvdd_gpio)) {
		dev_err(ctx->dev, "%s: cannot get dvdd_gpio %ld\n",
			__func__, PTR_ERR(ctx->dvdd_gpio));
		return PTR_ERR(ctx->dvdd_gpio);
	}
	gpiod_set_value(ctx->dvdd_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->dvdd_gpio);


	udelay(2000);
	/* set voltage with min & max*/
	ret = regulator_set_voltage(ctx->oled_vci, 0, 0);
	if (ret < 0)
		pr_err("set voltage ctx->oled_vci fail, ret = %d\n", ret);


	/* disable regulator */
	ret = regulator_disable(ctx->oled_vci);
	if (ret < 0)
		pr_err("enable regulator ctx->oled_vci fail, ret = %d\n", ret);

	udelay(2000);
	ctx->vddi_gpio =
		devm_gpiod_get(ctx->dev, "vddi", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddi_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vddi_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddi_gpio));
		return PTR_ERR(ctx->vddi_gpio);
	}
	gpiod_set_value(ctx->vddi_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vddi_gpio);


	printk("%s exit  \n",__func__);
	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;
	printk("%s enter  \n",__func__);
	if (ctx->prepared)
		return 0;

	//_gate_ic_Power_on();
	ctx->hbm_mode = 0;
	ctx->dc_mode = 0;
#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_enable();
#else
	ctx->vddi_gpio =
		devm_gpiod_get(ctx->dev, "vddi", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddi_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vddi_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddi_gpio));
		return PTR_ERR(ctx->vddi_gpio);
	}
	gpiod_set_value(ctx->vddi_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddi_gpio);
	udelay(2000);


	/* set voltage with min & max*/
	ret = regulator_set_voltage(ctx->oled_vci, 3000000, 3000000);
	if (ret < 0)
		pr_err("set voltage ctx->oled_vci fail, ret = %d\n", ret);


	/* enable regulator */
	ret = regulator_enable(ctx->oled_vci);
	if (ret < 0)
		pr_err("enable regulator ctx->oled_vci fail, ret = %d\n", ret);

	udelay(2000);
	ctx->dvdd_gpio =
		devm_gpiod_get(ctx->dev, "dvdd", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->dvdd_gpio)) {
		dev_err(ctx->dev, "%s: cannot get dvdd_gpio %ld\n",
			__func__, PTR_ERR(ctx->dvdd_gpio));
		return PTR_ERR(ctx->dvdd_gpio);
	}
	gpiod_set_value(ctx->dvdd_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->dvdd_gpio);
	msleep(15);
#endif

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_rst(panel);
#endif
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif
	ctx->hbm_mode = 0;
	ctx->dc_mode = 0;

	printk("%s exit  \n",__func__);
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	printk("%s enter  \n",__func__);
	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;
	printk("%s exit  \n",__func__);
	return 0;
}

static const struct drm_display_mode default_mode = {
	.clock = 411148,
	.hdisplay = FRAME_WIDTH,//1200
	.hsync_start = FRAME_WIDTH + FHD_HFP,//1215
	.hsync_end = FRAME_WIDTH + FHD_HFP + FHD_HSA,//1230
	.htotal = FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,//1245
	.vdisplay = FRAME_HEIGHT,//2670
	.vsync_start = FRAME_HEIGHT + FHD_VFP,//2715
	.vsync_end = FRAME_HEIGHT + FHD_VFP + FHD_VSA,//2717
	.vtotal = FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP,//2752
};

static const struct drm_display_mode mode_90 = {
	.clock = 414136,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + FHD_HFP_90,
	.hsync_end = FRAME_WIDTH + FHD_HFP_90 + FHD_HSA_90,
	.htotal = FRAME_WIDTH + FHD_HFP_90 + FHD_HSA_90 + FHD_HBP_90,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + FHD_VFP_90,//3659
	.vsync_end = FRAME_HEIGHT + FHD_VFP_90 + FHD_VSA_90,
	.vtotal = FRAME_HEIGHT + FHD_VFP_90 + FHD_VSA_90 + FHD_VBP_90,//3696
};

static const struct drm_display_mode mode_60 = {
	.clock = 205574,
	.hdisplay = FRAME_WIDTH,//1200
	.hsync_start = FRAME_WIDTH + FHD_HFP,//1215
	.hsync_end = FRAME_WIDTH + FHD_HFP + FHD_HSA,//1230
	.htotal = FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,//1245
	.vdisplay = FRAME_HEIGHT,//2670
	.vsync_start = FRAME_HEIGHT + FHD_VFP,//2715
	.vsync_end = FRAME_HEIGHT + FHD_VFP + FHD_VSA,//2717
	.vtotal = FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP,//2752
};

static const struct drm_display_mode mode_30 = {
	.clock = 102787,
	.hdisplay = FRAME_WIDTH,//1200
	.hsync_start = FRAME_WIDTH + FHD_HFP,//1215
	.hsync_end = FRAME_WIDTH + FHD_HFP + FHD_HSA,//1230
	.htotal = FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,//1245
	.vdisplay = FRAME_HEIGHT,//2670
	.vsync_start = FRAME_HEIGHT + FHD_VFP,//2715
	.vsync_end = FRAME_HEIGHT + FHD_VFP + FHD_VSA,//2717
	.vtotal = FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP,//2752
};

static const struct drm_display_mode mode_24 = {
	.clock = 82229,
	.hdisplay = FRAME_WIDTH,//1200
	.hsync_start = FRAME_WIDTH + FHD_HFP,//1215
	.hsync_end = FRAME_WIDTH + FHD_HFP + FHD_HSA,//1230
	.htotal = FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,//1245
	.vdisplay = FRAME_HEIGHT,//2670
	.vsync_start = FRAME_HEIGHT + FHD_VFP,//2715
	.vsync_end = FRAME_HEIGHT + FHD_VFP + FHD_VSA,//2717
	.vtotal = FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP,//2752
};

static const struct drm_display_mode mode_10 = {
	.clock = 34262,
	.hdisplay = FRAME_WIDTH,//1200
	.hsync_start = FRAME_WIDTH + FHD_HFP,//1215
	.hsync_end = FRAME_WIDTH + FHD_HFP + FHD_HSA,//1230
	.htotal = FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,//1245
	.vdisplay = FRAME_HEIGHT,//2670
	.vsync_start = FRAME_HEIGHT + FHD_VFP,//2715
	.vsync_end = FRAME_HEIGHT + FHD_VFP + FHD_VSA,//2717
	.vtotal = FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP,//2752
};

static const struct drm_display_mode mode_1 = {
	.clock = 3426,
	.hdisplay = FRAME_WIDTH,//1200
	.hsync_start = FRAME_WIDTH + FHD_HFP,//1215
	.hsync_end = FRAME_WIDTH + FHD_HFP + FHD_HSA,//1230
	.htotal = FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,//1245
	.vdisplay = FRAME_HEIGHT,//2670
	.vsync_start = FRAME_HEIGHT + FHD_VFP,//2715
	.vsync_end = FRAME_HEIGHT + FHD_VFP + FHD_VSA,//2717
	.vtotal = FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP,//2752
};

static const struct drm_display_mode fhd_default_mode = {
	.clock = FHD_CLK_DEF,
	.hdisplay = FHD_FRAME_WIDTH,
	.hsync_start = FHD_FRAME_WIDTH + FHD_HFP,
	.hsync_end = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA,
	.htotal = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,
	.vdisplay = FHD_FRAME_HEIGHT,
	.vsync_start = FHD_FRAME_HEIGHT + FHD_VFP,
	.vsync_end = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA,
	.vtotal = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP,
};

static const struct drm_display_mode fhd_mode_90 = {
	.clock = FHD_CLK_90,
	.hdisplay = FHD_FRAME_WIDTH,
	.hsync_start = FHD_FRAME_WIDTH + FHD_HFP,
	.hsync_end = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA,
	.htotal = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,
	.vdisplay = FHD_FRAME_HEIGHT,
	.vsync_start = FHD_FRAME_HEIGHT + FHD_VFP,
	.vsync_end = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA,
	.vtotal = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP,
};

static const struct drm_display_mode fhd_mode_60 = {
	.clock = FHD_CLK_60,
	.hdisplay = FHD_FRAME_WIDTH,
	.hsync_start = FHD_FRAME_WIDTH + FHD_HFP,
	.hsync_end = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA,
	.htotal = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,
	.vdisplay = FHD_FRAME_HEIGHT,
	.vsync_start = FHD_FRAME_HEIGHT + FHD_VFP,
	.vsync_end = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA,
	.vtotal = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP,
};

static const struct drm_display_mode fhd_mode_30 = {
	.clock = FHD_CLK_30,
	.hdisplay = FHD_FRAME_WIDTH,
	.hsync_start = FHD_FRAME_WIDTH + FHD_HFP,
	.hsync_end = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA,
	.htotal = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,
	.vdisplay = FHD_FRAME_HEIGHT,
	.vsync_start = FHD_FRAME_HEIGHT + FHD_VFP,
	.vsync_end = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA,
	.vtotal = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP,
};

static const struct drm_display_mode fhd_mode_24 = {
	.clock = FHD_CLK_24,
	.hdisplay = FHD_FRAME_WIDTH,
	.hsync_start = FHD_FRAME_WIDTH + FHD_HFP,
	.hsync_end = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA,
	.htotal = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,
	.vdisplay = FHD_FRAME_HEIGHT,
	.vsync_start = FHD_FRAME_HEIGHT + FHD_VFP,
	.vsync_end = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA,
	.vtotal = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP,
};

static const struct drm_display_mode fhd_mode_10 = {
	.clock = FHD_CLK_10,
	.hdisplay = FHD_FRAME_WIDTH,
	.hsync_start = FHD_FRAME_WIDTH + FHD_HFP,
	.hsync_end = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA,
	.htotal = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,
	.vdisplay = FHD_FRAME_HEIGHT,
	.vsync_start = FHD_FRAME_HEIGHT + FHD_VFP,
	.vsync_end = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA,
	.vtotal = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP,
};

static const struct drm_display_mode fhd_mode_1 = {
	.clock = FHD_CLK_1,
	.hdisplay = FHD_FRAME_WIDTH,
	.hsync_start = FHD_FRAME_WIDTH + FHD_HFP,
	.hsync_end = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA,
	.htotal = FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,
	.vdisplay = FHD_FRAME_HEIGHT,
	.vsync_start = FHD_FRAME_HEIGHT + FHD_VFP,
	.vsync_end = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA,
	.vtotal = FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP,
};

enum SWITCH_MODE_DELAY switch_mode_delay_table[DISPLAY_MODE_NUM][DISPLAY_MODE_NUM] = {
	/*DISPLAY_MODE_0 ... DISPLAY_MODE_11*/
	//mode switch, TE really switch at (N + x)th TE, the x means delay_x
	//from [row] fps to [column] fps
	{DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_1, DELAY_1,
		DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_1, DELAY_1,},// mode_0
	{DELAY_2, DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_1,
		DELAY_2, DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_1,},// mode_1
	{DELAY_0, DELAY_2, DELAY_0, DELAY_1, DELAY_1, DELAY_1,
		DELAY_0, DELAY_2, DELAY_0, DELAY_1, DELAY_1, DELAY_1,},// mode_2
	{DELAY_0, DELAY_2, DELAY_1, DELAY_0, DELAY_1, DELAY_1,
		DELAY_0, DELAY_2, DELAY_1, DELAY_0, DELAY_1, DELAY_1,},// mode_3
	{DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_0, DELAY_1,
		DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_0, DELAY_1,},// mode_4
	{DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_1, DELAY_0,
		DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_1, DELAY_0,},// mode_5
	//{DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_1, DELAY_0,
	//	DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_1, DELAY_0,},// mode_6

	{DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_1, DELAY_1,
		DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_1, DELAY_1,},// mode_6
	{DELAY_2, DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_1,
		DELAY_2, DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_1,},// mode_7
	{DELAY_0, DELAY_2, DELAY_0, DELAY_1, DELAY_1, DELAY_1,
		DELAY_0, DELAY_2, DELAY_0, DELAY_1, DELAY_1, DELAY_1,},// mode_8
	{DELAY_0, DELAY_2, DELAY_1, DELAY_0, DELAY_1, DELAY_1,
		DELAY_0, DELAY_2, DELAY_1, DELAY_0, DELAY_1, DELAY_1,},// mode_9
	{DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_0, DELAY_1,
		DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_0, DELAY_1,},// mode_10
	{DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_1, DELAY_0,
		DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_1, DELAY_0,},// mode_11
	//{DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_1, DELAY_0,
	//	DELAY_0, DELAY_2, DELAY_1, DELAY_1, DELAY_1, DELAY_0,},// mode_13
};

#if defined(CONFIG_MTK_PANEL_EXT)
static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);
	printk("%s enter  \n",__func__);
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	printk("%s exit  \n",__func__);
	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	unsigned char data[3] = {0x00, 0x00, 0x00};
	unsigned char id[3] = {0x00, 0x00, 0x00};
	ssize_t ret;
	printk("%s enter  \n",__func__);
	ret = mipi_dsi_dcs_read(dsi, 0x4, data, 3);
	if (ret < 0) {
		pr_err("%s error\n", __func__);
		return 0;
	}

	pr_info("ATA read data %x %x %x\n", data[0], data[1], data[2]);

	if (data[0] == id[0] &&
			data[1] == id[1] &&
			data[2] == id[2])
		return 1;

	pr_info("ATA expect read data is %x %x %x\n",
			id[0], id[1], id[2]);
	printk("%s exit  \n",__func__);
	return 0;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	char bl_tb[] = {0x51, 0x0F, 0xff};
	struct lcm *ctx = g_ctx;

	if ((ctx->hbm_mode) && level) {
		pr_info("hbm_mode = %d, skip backlight(%d)\n", ctx->hbm_mode, level);
		return 0;
	}

	if (!(ctx->current_backlight && level))
		pr_info("backlight changed from %u to %u\n", ctx->current_backlight, level);
	else
		pr_debug("backlight changed from %u to %u\n", ctx->current_backlight, level);

	printk("%s enter  \n",__func__);
	printk("%s backlight level = %d  \n",__func__,level);
	bl_tb[1] = (level >> 8) & 0xF;
	bl_tb[2] = level & 0xFF;
	if (!cb)
		return -1;
	cb(dsi, handle, bl_tb, ARRAY_SIZE(bl_tb));
	ctx->current_backlight = level;
	if (!level)
		ctx->hbm_mode = 0;
	return 0;
}




static struct mtk_panel_params ext_params = {
	.pll_clk = PLL_CLOCK,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
	},
	.lp_perline_en = 1,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.lcm_index = 0,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = FRAME_HEIGHT,
		.pic_width = FRAME_WIDTH,
		.slice_height = 30,
		.slice_width = (FRAME_WIDTH/2),
		.chunk_size = 600,
		.xmit_delay = 512,
		.dec_delay = 281,
		.scale_value = 32,
		.increment_interval = 1488,
		.decrement_interval = 8,
		.line_bpg_offset = 1,
		.nfl_bpg_offset = 71,
		.slice_bpg_offset = 781,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,

		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = nt37801_wqhs_dsi_cmd_120hz_dphy_buf_thresh,
			.range_min_qp = nt37801_wqhs_dsi_cmd_120hz_dphy_range_min_qp,
			.range_max_qp = nt37801_wqhs_dsi_cmd_120hz_dphy_range_max_qp,
			.range_bpg_ofs = nt37801_wqhs_dsi_cmd_120hz_dphy_range_bpg_ofs,
			},
		},
	.data_rate = PLL_CLOCK * 2,
	/* following MIPI hopping parameter might cause screen mess */
	.dyn = {
		.switch_en = 1,
		.pll_clk = PLL_CLOCK + 1,
	},
	.dyn_fps = {
		.vact_timing_fps = 120,
	},
	.panel_cellid_reg = 0x5A,
	.panel_cellid_offset_reg = 0x65,
	.panel_cellid_len = 23,
	.panel_ver = 1,
	.panel_name = "csot_vtdr6130_636_1200_2670",
	.panel_supplier = "csot-vtdr6130",
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = PLL_CLOCK,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
	},
	.lp_perline_en = 1,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.lcm_index = 0,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = FRAME_HEIGHT,
		.pic_width = FRAME_WIDTH,
		.slice_height = 30,
		.slice_width = (FRAME_WIDTH/2),
		.chunk_size = 600,
		.xmit_delay = 512,
		.dec_delay = 281,
		.scale_value = 32,
		.increment_interval = 1488,
		.decrement_interval = 8,
		.line_bpg_offset = 1,
		.nfl_bpg_offset = 71,
		.slice_bpg_offset = 781,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,

		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = nt37801_wqhs_dsi_cmd_120hz_dphy_buf_thresh,
			.range_min_qp = nt37801_wqhs_dsi_cmd_120hz_dphy_range_min_qp,
			.range_max_qp = nt37801_wqhs_dsi_cmd_120hz_dphy_range_max_qp,
			.range_bpg_ofs = nt37801_wqhs_dsi_cmd_120hz_dphy_range_bpg_ofs,
			},
		},
	.data_rate = PLL_CLOCK * 2,
	/* following MIPI hopping parameter might cause screen mess */
	.dyn = {
		.switch_en = 1,
		.pll_clk = PLL_CLOCK + 1,
	},
	.dyn_fps = {
		.vact_timing_fps = 120,
	},
	.panel_cellid_reg = 0x5A,
	.panel_cellid_offset_reg = 0x65,
	.panel_cellid_len = 23,
	.panel_ver = 1,
	.panel_name = "csot_vtdr6130_636_1200_2670",
	.panel_supplier = "csot-vtdr6130",
};

static struct mtk_panel_params ext_params_60hz = {
	.pll_clk = PLL_CLOCK,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
	},
	.lp_perline_en = 1,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.lcm_index = 0,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = FRAME_HEIGHT,
		.pic_width = FRAME_WIDTH,
		.slice_height = 30,
		.slice_width = (FRAME_WIDTH/2),
		.chunk_size = 600,
		.xmit_delay = 512,
		.dec_delay = 281,
		.scale_value = 32,
		.increment_interval = 1488,
		.decrement_interval = 8,
		.line_bpg_offset = 1,
		.nfl_bpg_offset = 71,
		.slice_bpg_offset = 781,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,

		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = nt37801_wqhs_dsi_cmd_120hz_dphy_buf_thresh,
			.range_min_qp = nt37801_wqhs_dsi_cmd_120hz_dphy_range_min_qp,
			.range_max_qp = nt37801_wqhs_dsi_cmd_120hz_dphy_range_max_qp,
			.range_bpg_ofs = nt37801_wqhs_dsi_cmd_120hz_dphy_range_bpg_ofs,
			},
		},
	.data_rate = PLL_CLOCK * 2,
	/* following MIPI hopping parameter might cause screen mess */
	.dyn = {
		.switch_en = 1,
		.pll_clk = PLL_CLOCK + 1,
	},
	.dyn_fps = {
		.vact_timing_fps = 120,
	},
	.panel_cellid_reg = 0x5A,
	.panel_cellid_offset_reg = 0x65,
	.panel_cellid_len = 23,
	.panel_ver = 1,
	.panel_name = "csot_vtdr6130_636_1200_2670",
	.panel_supplier = "csot-vtdr6130",
};

struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}

void get_switch_mode_delay (enum SWITCH_MODE_DELAY **switch_mode_delay, unsigned int mode_num)
{
	unsigned int i = 0;
	printk("%s enter  \n",__func__);
	for (i = 0;  i < mode_num; i++) {
		memcpy(switch_mode_delay[i], switch_mode_delay_table[i],
			sizeof(enum SWITCH_MODE_DELAY) * mode_num);
	}
}


static int mtk_panel_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);
	struct lcm *ctx = panel_to_lcm(panel);
	printk("%s enter  \n",__func__);
	if (drm_mode_vrefresh(m) == 120) {
		ext_params.skip_vblank = 0;
		ext_params.vblank_off = false;
		ext->params = &ext_params;
	} else if (drm_mode_vrefresh(m) == 90) {
		ext_params_90hz.vblank_off = false;
		ext->params = &ext_params_90hz;
	} else if (drm_mode_vrefresh(m) == 60) {
		ext_params_60hz.skip_vblank = 2;
		ext_params_60hz.vblank_off = false;
		ext->params = &ext_params_60hz;
	} else if (drm_mode_vrefresh(m) == 30) {
		ext_params.skip_vblank = 4;
		ext_params.vblank_off = false;
		ext->params = &ext_params;
	} else if (drm_mode_vrefresh(m) == 24) {
		ext_params.skip_vblank = 5;
		ext_params.vblank_off = false;
		ext->params = &ext_params;
	} else if (drm_mode_vrefresh(m) == 10) {
		ext_params.skip_vblank = 12;
		ext_params.vblank_off = true;
		ext->params = &ext_params;
	} else if (drm_mode_vrefresh(m) == 1) {
		ext_params.skip_vblank = 120;
		ext_params.vblank_off = true;
		ext->params = &ext_params;
	} else
		ret = 1;

	if (!ret)
		ctx->current_fps = drm_mode_vrefresh(m);
	printk("%s exit current_fps = %d \n",__func__,ctx->current_fps);
	return ret;
}

static int mtk_panel_ext_param_get(struct drm_panel *panel,
	struct drm_connector *connector,
	struct mtk_panel_params **ext_param,
	unsigned int mode)
{
	int ret = 0;
	struct lcm *ctx = panel_to_lcm(panel);
	struct drm_display_mode *m = get_mode_by_id(connector, mode);
	printk("%s enter  \n",__func__);
	if (drm_mode_vrefresh(m) == 120)
		*ext_param = &ext_params;
	else if (drm_mode_vrefresh(m) == 90)
		*ext_param = &ext_params_90hz;
	else if (drm_mode_vrefresh(m) == 60)
		*ext_param = &ext_params_60hz;
	else if (drm_mode_vrefresh(m) == 30)
		*ext_param = &ext_params;
	else if (drm_mode_vrefresh(m) == 24)
		*ext_param = &ext_params;
	else if (drm_mode_vrefresh(m) == 10)
		*ext_param = &ext_params;
	else if (drm_mode_vrefresh(m) == 1)
		*ext_param = &ext_params;
	else
		ret = 1;

	if (!ret)
		ctx->current_fps = drm_mode_vrefresh(m);
	printk("%s exit current_fps = %d \n",__func__,ctx->current_fps);
	return ret;
}

enum RES_SWITCH_TYPE mtk_get_res_switch_type(void)
{
	pr_info("res_switch_type: %d\n", res_switch_type);
	return res_switch_type;
}

int mtk_scaling_mode_mapping(int mode_idx)
{
	return (mode_idx % REAL_MODE_NUM);
}

static void mode_switch_working(struct drm_panel *panel, int fps,
	struct mtk_mode_switch_cmd *mode_switch_cmd, size_t len)
{
	printk("%s enter  \n",__func__);
#if MODE_SWITCH_CMDQ_ENABLE
	if (fps == 120 || fps == 30 || fps == 24 || fps == 10) {
		memset(&ext_params.mode_switch_cmd, 0,
			sizeof(struct mode_switch_params));
		ext_params.mode_switch_cmd.num_cmd = len;
		memcpy(&ext_params.mode_switch_cmd.ms_table, mode_switch_cmd,
			sizeof(struct mtk_mode_switch_cmd) * len);
	} else if (fps == 90) {
		memset(&ext_params_90hz.mode_switch_cmd, 0,
			sizeof(struct mode_switch_params));
		ext_params_90hz.mode_switch_cmd.num_cmd = len;
		memcpy(&ext_params_90hz.mode_switch_cmd.ms_table, mode_switch_cmd,
			sizeof(struct mtk_mode_switch_cmd) * len);
	} else if (fps == 60) {
		memset(&ext_params_60hz.mode_switch_cmd, 0,
			sizeof(struct mode_switch_params));
		ext_params_60hz.mode_switch_cmd.num_cmd = len;
		memcpy(&ext_params_60hz.mode_switch_cmd.ms_table, mode_switch_cmd,
			sizeof(struct mtk_mode_switch_cmd) * len);
	}
#else
	int i;
	struct lcm *ctx = panel_to_lcm(panel);
	for (i = 0; i < len; i++) {
		lcm_dcs_write(ctx, mode_switch_cmd[i].para_list,
			mode_switch_cmd[i].cmd_num);
	}
#endif
	printk("%s exit  \n",__func__);
}

static int mode_switch(struct drm_panel *panel,
		struct drm_connector *connector, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, dst_mode);
	printk("%s enter  \n",__func__);
	if (stage == BEFORE_DSI_POWERDOWN) {
		ret = 1;
		return ret;
	}

	pr_info("%s cur_mode = %d dst_mode %d\n", __func__, cur_mode, dst_mode);

	if (drm_mode_vrefresh(m) == 120)
		mode_switch_working(panel, 120, cmd_table_120fps, ARRAY_SIZE(cmd_table_120fps));
	else if (drm_mode_vrefresh(m) == 90)
		mode_switch_working(panel, 90, cmd_table_90fps, ARRAY_SIZE(cmd_table_90fps));
	else if (drm_mode_vrefresh(m) == 60)
		mode_switch_working(panel, 60, cmd_table_60fps, ARRAY_SIZE(cmd_table_60fps));
	else if (drm_mode_vrefresh(m) == 30)
		mode_switch_working(panel, 30, cmd_table_30fps, ARRAY_SIZE(cmd_table_30fps));
	else if (drm_mode_vrefresh(m) == 24)
		mode_switch_working(panel, 24, cmd_table_24fps, ARRAY_SIZE(cmd_table_24fps));
	else if (drm_mode_vrefresh(m) == 10)
		mode_switch_working(panel, 10, cmd_table_10fps, ARRAY_SIZE(cmd_table_10fps));
	else if (drm_mode_vrefresh(m) == 1)
		mode_switch_working(panel, 1, cmd_table_1fps, ARRAY_SIZE(cmd_table_1fps));
	else
		ret = 1;
	printk("%s exit  \n",__func__);
	return ret;
}



static int set_lhbm_alpha(unsigned int bl_level)
{
	unsigned int alpha = 0;
	unsigned int lhbm_alpha_index = bl_level-1;

	if (bl_level == 0)
		lhbm_alpha_index = 0;
	else if (bl_level > sizeof(lhbm_alpha))
		lhbm_alpha_index = sizeof(lhbm_alpha)-1;

	alpha = lhbm_alpha[lhbm_alpha_index];

	pr_info("%s: backlight %d alpha %d\n", __func__, bl_level, alpha);
	return alpha;
}
static int panel_lhbm_set_cmdq(void *dsi, dcs_grp_write_gce cb, void *handle, uint32_t on, uint32_t bl_level, uint32_t fps)
{
	int alpha_level = 0;
	struct lcm *ctx = g_ctx;
	char lhbm_51[] = {0x51, 0x0f, 0xff};
	char lhbm_63[] = {0x63, 0x0f, 0xff, 0x0f, 0xff};

	lhbm_51[1] = (bl_level >> 8) & 0xF;
	lhbm_51[2] = bl_level & 0xFF;

	if (on) {
		if (bl_level <= ARRAY_SIZE(lhbm_alpha)) {
			alpha_level = set_lhbm_alpha(bl_level);
			lhbm_63[1] = (alpha_level >> 8) & 0xF;
			lhbm_63[2] = alpha_level & 0xFF;
			lhbm_63[3] = 0x05;
			lhbm_63[4] = 0x2A;
			lcm_dcs_write(ctx, lhbm_51, 3);
			lcm_dcs_write(ctx, lhbm_63, 5);
			lcm_dcs_write_seq_static(ctx, 0x62, 0x03);
		} else {
			lhbm_63[1] = 0xF;
			lhbm_63[2] = 0xFF;
			lhbm_63[3] = (bl_level >> 8) & 0xF;
			lhbm_63[4] = bl_level & 0xFF;
			lcm_dcs_write(ctx, lhbm_51, 3);
			lcm_dcs_write(ctx, lhbm_63, 5);
			lcm_dcs_write_seq_static(ctx, 0x62, 0x03);
		}
	} else {
		if (bl_level <= ARRAY_SIZE(lhbm_alpha)) {
			lcm_dcs_write(ctx, lhbm_51, 3);
			lcm_dcs_write_seq_static(ctx, 0x62, 0x00);
		} else {
			lcm_dcs_write_seq_static(ctx, 0x62, 0x00);
		}


	}
	return 0;

}
static int panel_hbm_set_cmdq(struct lcm *ctx, void *dsi, dcs_grp_write_gce cb, void *handle, uint32_t hbm_state)
{
	if (hbm_state > 2) return -1;

	switch (hbm_state)
	{
		case 0:
			if (ctx->lhbm_en)
				panel_lhbm_set_cmdq(dsi, cb, handle, 0, ctx->current_backlight, ctx->current_fps);
			break;
		case 1:
			if (ctx->lhbm_en) {
				panel_lhbm_set_cmdq(dsi, cb, handle, 0, ctx->current_backlight, ctx->current_fps);

			} else {
				lcm_dcs_write_seq_static(ctx, 0x51, 0x0f, 0xff);
			}
			break;
		case 2:
			if (ctx->lhbm_en)
				panel_lhbm_set_cmdq(dsi, cb, handle, 1, ctx->current_backlight,  ctx->current_fps);
			else
				lcm_dcs_write_seq_static(ctx, 0x51, 0x0f, 0xff);
			break;
		default:
			break;
	}

	ctx->hbm_mode = hbm_state;
	return 0;
}

static int pane_dc_set_cmdq(struct lcm *ctx, void *dsi, dcs_grp_write_gce cb, void *handle, uint32_t dc_state)
{
	if (dc_state) {
		lcm_dcs_write_seq_static(ctx, 0x5e, 0x01);
	} else {
		lcm_dcs_write_seq_static(ctx, 0x5e, 0x00);
	}
	ctx->dc_mode = dc_state;
	return 0;
}


static int panel_feature_get(struct drm_panel *panel, struct panel_param_info *param_info){

	struct lcm *ctx = panel_to_lcm(panel);
	int ret = 0;

	switch (param_info->param_idx) {
		case PARAM_CABC:
			break;
		case PARAM_ACL:
			ret = -1;
			break;
		case PARAM_HBM:
			param_info->value = ctx->hbm_mode;
			break;
		case PARAM_DC:
			param_info->value = ctx->dc_mode;
			break;
		default:
			ret = -1;
			break;
	}
	return ret;

}

static int panel_feature_set(struct drm_panel *panel, void *dsi,
			      dcs_grp_write_gce cb, void *handle, struct panel_param_info param_info)
{

	struct lcm *ctx = panel_to_lcm(panel);
	int ret = 0;
	if ((!cb) || (!ctx->enabled))
		return -1;
	pr_info("%s: set feature %d to %d\n", __func__, param_info.param_idx, param_info.value);

	switch (param_info.param_idx) {
		case PARAM_CABC:
			break;
		case PARAM_ACL:
			ret = -1;
			break;
		case PARAM_HBM:
			ctx->hbm_mode = param_info.value;
			panel_hbm_set_cmdq(ctx, dsi, cb, handle, param_info.value);
			break;
		case PARAM_DC:
			pane_dc_set_cmdq(ctx, dsi, cb, handle, param_info.value);
			ctx->dc_mode = param_info.value;
			break;
		default:
			ret = -1;
			break;
	}
	pr_info("%s: set feature %d to %d success\n", __func__, param_info.param_idx, param_info.value);
	return ret;
}


static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.get_res_switch_type = mtk_get_res_switch_type,
	.scaling_mode_mapping = mtk_scaling_mode_mapping,
	.mode_switch = mode_switch,
	//.set_bl_elvss_cmdq = lcm_set_bl_elvss_cmdq,
	/* Not real backlight cmd in AOD, just for QC purpose */
	.set_aod_light_mode = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
	//.lcm_update_roi = lcm_update_roi,
	//.lcm_update_roi_cmdq = lcm_update_roi_cmdq,
	//.get_lcm_power_state = lcm_panel_get_ab_data,
	.get_switch_mode_delay = get_switch_mode_delay,
	.panel_feature_set = panel_feature_set,
	.panel_feature_get = panel_feature_get,
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static int lcm_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *mode1;
	struct drm_display_mode *mode2;
	struct drm_display_mode *mode3;
	struct drm_display_mode *mode4;
	struct drm_display_mode *mode5;
	struct drm_display_mode *mode6;

	struct drm_display_mode *fhd_mode;
	struct drm_display_mode *fhd_mode1;
	struct drm_display_mode *fhd_mode2;
	struct drm_display_mode *fhd_mode3;
	struct drm_display_mode *fhd_mode4;
	struct drm_display_mode *fhd_mode5;
	struct drm_display_mode *fhd_mode6;

	printk("%s enter  \n",__func__);
	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	mode1 = drm_mode_duplicate(connector->dev, &mode_90);
	if (!mode1)
		return -ENOMEM;

	drm_mode_set_name(mode1);
	mode1->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode1);

	mode2 = drm_mode_duplicate(connector->dev, &mode_60);
	if (!mode2) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			mode_60.hdisplay, mode_60.vdisplay,
			drm_mode_vrefresh(&mode_60));
		return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode2);

	mode3 = drm_mode_duplicate(connector->dev, &mode_30);
	if (!mode3) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			mode_30.hdisplay, mode_30.vdisplay,
			drm_mode_vrefresh(&mode_30));
		return -ENOMEM;
	}

	drm_mode_set_name(mode3);
	mode3->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode3);

	mode4 = drm_mode_duplicate(connector->dev, &mode_24);
	if (!mode4) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			mode_24.hdisplay, mode_24.vdisplay,
			drm_mode_vrefresh(&mode_24));
		return -ENOMEM;
	}

	drm_mode_set_name(mode4);
	mode4->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode4);

	mode5 = drm_mode_duplicate(connector->dev, &mode_10);
	if (!mode5) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			mode_10.hdisplay, mode_10.vdisplay,
			drm_mode_vrefresh(&mode_10));
		return -ENOMEM;
	}

	drm_mode_set_name(mode5);
	mode5->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode5);

	mode6 = drm_mode_duplicate(connector->dev, &mode_1);
	if (!mode6) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			mode_1.hdisplay, mode_1.vdisplay,
			drm_mode_vrefresh(&mode_1));
		return -ENOMEM;
	}

	drm_mode_set_name(mode6);
	mode6->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode6);

	fhd_mode = drm_mode_duplicate(connector->dev, &fhd_default_mode);
	if (!fhd_mode) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			fhd_default_mode.hdisplay, fhd_default_mode.vdisplay,
			drm_mode_vrefresh(&fhd_default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(fhd_mode);
	fhd_mode->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, fhd_mode);

	fhd_mode1 = drm_mode_duplicate(connector->dev, &fhd_mode_90);
	if (!fhd_mode1) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			fhd_mode_90.hdisplay, fhd_mode_90.vdisplay,
			drm_mode_vrefresh(&fhd_mode_90));
		return -ENOMEM;
	}

	drm_mode_set_name(fhd_mode1);
	fhd_mode1->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, fhd_mode1);

	fhd_mode2 = drm_mode_duplicate(connector->dev, &fhd_mode_60);
	if (!fhd_mode2) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			fhd_mode_60.hdisplay, fhd_mode_60.vdisplay,
			drm_mode_vrefresh(&fhd_mode_60));
		return -ENOMEM;
	}

	drm_mode_set_name(fhd_mode2);
	fhd_mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, fhd_mode2);

	fhd_mode3 = drm_mode_duplicate(connector->dev, &fhd_mode_30);
	if (!fhd_mode3) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			fhd_mode_30.hdisplay, fhd_mode_30.vdisplay,
			drm_mode_vrefresh(&fhd_mode_30));
		return -ENOMEM;
	}

	drm_mode_set_name(fhd_mode3);
	fhd_mode3->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, fhd_mode3);

	fhd_mode4 = drm_mode_duplicate(connector->dev, &fhd_mode_24);
	if (!fhd_mode4) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			fhd_mode_24.hdisplay, fhd_mode_24.vdisplay,
			drm_mode_vrefresh(&fhd_mode_24));
		return -ENOMEM;
	}

	drm_mode_set_name(fhd_mode4);
	fhd_mode4->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, fhd_mode4);

	fhd_mode5 = drm_mode_duplicate(connector->dev, &fhd_mode_10);
	if (!fhd_mode5) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			fhd_mode_10.hdisplay, fhd_mode_10.vdisplay,
			drm_mode_vrefresh(&fhd_mode_10));
		return -ENOMEM;
	}

	drm_mode_set_name(fhd_mode5);
	fhd_mode5->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, fhd_mode5);

	fhd_mode6 = drm_mode_duplicate(connector->dev, &fhd_mode_1);
	if (!fhd_mode6) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			fhd_mode_1.hdisplay, fhd_mode_1.vdisplay,
			drm_mode_vrefresh(&fhd_mode_1));
		return -ENOMEM;
	}

	drm_mode_set_name(fhd_mode6);
	fhd_mode6->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, fhd_mode6);

	connector->display_info.width_mm = 64;
	connector->display_info.height_mm = 129;

	printk("%s exit  \n",__func__);
	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct lcm *ctx;
	struct device_node *backlight;
	unsigned int res_switch;
	unsigned int value;
	int ret;

	pr_info("%s+\n", __func__);
	printk("%s enter \n",__func__);
	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);
	g_ctx = ctx;
	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ret = of_property_read_u32(dev->of_node, "res-switch", &res_switch);
	if (ret < 0)
		res_switch = 0;
	else
		res_switch_type = (enum RES_SWITCH_TYPE)res_switch;



	value = 0;
	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	ctx->vddi_gpio = devm_gpiod_get(dev, "vddi", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddi_gpio)) {
		dev_err(dev, "%s: cannot get vddi_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddi_gpio));
		return PTR_ERR(ctx->vddi_gpio);
	}
	devm_gpiod_put(dev, ctx->vddi_gpio);

	ctx->dvdd_gpio = devm_gpiod_get(dev, "dvdd", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->dvdd_gpio)) {
		dev_err(dev, "%s: cannot get vddi-gpios %ld\n",
			__func__, PTR_ERR(ctx->dvdd_gpio));
		return PTR_ERR(ctx->dvdd_gpio);
	}
	devm_gpiod_put(dev, ctx->dvdd_gpio);

	ctx->oled_vci = devm_regulator_get_optional(dev, "oled-vci");
	if (IS_ERR(ctx->oled_vci)) { /* handle return value */
		ret = PTR_ERR(ctx->oled_vci);
		pr_err("get oled-vci fail, error: %d\n", ret);
		return ret;
	}
	ret = regulator_enable(ctx->oled_vci);
	if (ret < 0)
		pr_err("enable regulator ctx->oled_vci fail, ret = %d\n", ret);


	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	ctx->hbm_mode = 0;
	ctx->dc_mode = 0;
	ctx->current_fps = 120;

	ctx->lhbm_en = 1;
	printk("%s exit  \n",__func__);
	return ret;
}

static void lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif
	printk("%s enter  \n",__func__);
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	if (ext_ctx != NULL) {
		mtk_panel_detach(ext_ctx);
		mtk_panel_remove(ext_ctx);
	}
#endif

}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "csot,vtdr6130,cmd,636", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "csot_vtdr6130_636_1200_2670",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Randy Lin <randy.lin@mediatek.com>");
MODULE_DESCRIPTION("VDTR6130 CMD LCD Panel Driver");
MODULE_LICENSE("GPL v2");
