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
#include "../mediatek/mediatek_v2/mtk_corner_pattern/nt37801_cmd_120hz_rc.h"
#include "dsi-panel-mot-csot-vtdr6126-667-1220x2712-vdo-lhbm-alpha.h"
#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mediatek_v2/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif
#define FRAME_WIDTH				(1220)
#define FRAME_HEIGHT			(2712)
#define PLL_CLOCK				(505)
#define REAL_MODE_NUM           (6)
#define FHD_FRAME_WIDTH    (1220)
#define FHD_HFP            (44)
#define FHD_HSA            (4)
#define FHD_HBP            (24)
#define FHD_HTOTAL         (FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP)
#define FHD_FRAME_HEIGHT   (2712)
#define FHD_VFP            (48)
#define FHD_VSA            (4)
#define FHD_VBP            (24)
#define FHD_VTOTAL         (FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP)
#define MODE_SWITCH_CMDQ_ENABLE 1
#define FHD_HFP_90            (44)
#define FHD_HSA_90            (4)
#define FHD_HBP_90            (24)
#define FHD_VFP_90            (964)
#define FHD_VSA_90            (4)
#define FHD_VBP_90            (24)
#define FHD_HFP_60            (44)
#define FHD_HSA_60            (4)
#define FHD_HBP_60            (24)
#define FHD_VFP_60            (2816)
#define FHD_VSA_60            (4)
#define FHD_VBP_60            (24)
//extern int _lcm_i2c_write_bytes(unsigned char addr, unsigned char value);
struct mtk_mode_switch_cmd cmd_table_120fps[] = {
	{2, {0x6C,0x00}}
};
struct mtk_mode_switch_cmd cmd_table_90fps[] = {
	{2, {0x6C,0x01}}
};
struct mtk_mode_switch_cmd cmd_table_60fps[] = {
	{2, {0x6C,0x02}}
};

enum panel_version{
	PANEL_V1 = 1,
	PANEL_V2,
	PANEL_V3,
};

static enum RES_SWITCH_TYPE res_switch_type = RES_SWITCH_NO_USE;
static int current_fps = 60;
static struct lcm *g_ctx = NULL;


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
	struct regulator *oled_dvdd;
	//struct regulator *oled_vci;
	struct gpio_desc *vci_gpio;
	bool prepared;
	bool enabled;
	int error;
	bool lhbm_en;
	atomic_t hbm_mode;
	atomic_t dc_mode;
	atomic_t apl_mode;
	atomic_t current_bl;
	atomic_t current_fps;
	atomic_t pcd_mode;
	enum panel_version version;
};
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

#define APL_THRESHOLD 16000

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

static void lcm_panel_init(struct lcm *ctx)
{
	printk("%s enter  \n",__func__);
	udelay(2000);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(2 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(20);
	lcm_dcs_write_seq_static(ctx, 0x03,0x01);
	lcm_dcs_write_seq_static(ctx, 0x35,0x00);
	lcm_dcs_write_seq_static(ctx, 0x51,0x36,0xE8); //800nit
	lcm_dcs_write_seq_static(ctx, 0x53,0x20);
	lcm_dcs_write_seq_static(ctx, 0x59,0x09);
	lcm_dcs_write_seq_static(ctx, 0x5e,0x00);
	lcm_dcs_write_seq_static(ctx, 0x6c,0x00);
	lcm_dcs_write_seq_static(ctx, 0x6d,0x00);
	lcm_dcs_write_seq_static(ctx, 0x6f,0x01);
	lcm_dcs_write_seq_static(ctx, 0x72,0x00);
    lcm_dcs_write_seq_static(ctx, 0x70,0x11,0x00,0x00,0xAB,0x30,0x80,0x0A,0x98,0x04,0xC4,0x00,0x0C,0x02,0x62,0x02,0x62,0x02,0x00,0x01,0x1A,0x00,0x20,0x02,0x5B,0x00,0x08,0x00,0x01,0x00,0xBB,0x07,0x7B,0x18,0x00,0x10,0xF0,0x07,0x10,0x20,0x00,0x06,0x0F,0x0F,0x33,0x0E,0x1C,0x2A,0x38,0x46,0x54,0x62,0x69,0x70,0x77,0x79,0x7B,0x7D,0x7E,0x02,0x02,0x22,0x00,0x2A,0x40,0x2A,0xBE,0x3A,0xFC,0x3A,0xFA,0x3A,0xF8,0x3B,0x38,0x3B,0x78,0x3B,0xB6,0x4B,0xB6,0x4B,0xF4,0x4B,0xF4,0x6C,0x34,0x84,0x74,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0xf0,0xaa,0x1c);
	lcm_dcs_write_seq_static(ctx, 0xc1,0x10,0xb8,0x20,0xf0,0x37,0x6f,0x38,0x7c,0x39,0x68,0x3a,0x50,0x3b,0x30,0x3b,0xe4,0x3c,0xb4,0x3e,0x80);
	lcm_dcs_write_seq_static(ctx, 0xc2,0x1a,0x27,0x27,0x11,0x11,0x17,0x82,0x84,0x80,0x89,0x8f,0x8b,0x93,0x98,0x97,0x9c,0xa1,0xa2,0xa6,0xaa,0xad,0xad,0xb2,0xb5,0xb4,0xb8,0xbd,0xc2,0xc6,0xce);
	lcm_dcs_write_seq_static(ctx, 0xf0,0xaa,0x16);
	lcm_dcs_write_seq_static(ctx, 0xd1,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0xf0,0xaa,0x10);
	lcm_dcs_write_seq_static(ctx, 0xBE,0x3E,0x79);
	lcm_dcs_write_seq_static(ctx, 0xff,0x5a,0x81);
	lcm_dcs_write_seq_static(ctx, 0x65,0x03);
	lcm_dcs_write_seq_static(ctx, 0xf3,0x61);
	lcm_dcs_write_seq_static(ctx, 0x65,0x0b);
	lcm_dcs_write_seq_static(ctx, 0xf3,0x78);
	lcm_dcs_write_seq_static(ctx, 0xff,0x5a,0x80);
	lcm_dcs_write_seq_static(ctx, 0x65,0x25);
	lcm_dcs_write_seq_static(ctx, 0xfd,0x01);
	lcm_dcs_write_seq_static(ctx, 0xff,0x5a,0x00);
	lcm_dcs_write_seq_static(ctx, 0xf0,0xaa,0x00);

	pr_info("%s current_fps:%d\n", __func__, current_fps);
	current_fps = 120;
	switch (current_fps) {
	case 120:
		lcm_dcs_write_seq_static(ctx, 0x6C,0x00);
		break;
	case 90:
		lcm_dcs_write_seq_static(ctx, 0x6C,0x01);
		break;
	case 60:
		lcm_dcs_write_seq_static(ctx, 0x6C,0x02);
		break;
	default:
		pr_info("%s current_fps mismatch:%d\n", __func__, current_fps);
		break;
	}

	lcm_dcs_write_seq_static(ctx, 0x11);
	msleep(100);
	lcm_dcs_write_seq_static(ctx, 0x29);
	msleep(20);
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
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

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
	devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	msleep(30);
	/* disable regulator */
	ret = regulator_disable(ctx->oled_dvdd);
	if (ret < 0)
		pr_err("enable regulator ctx->oled_dvdd fail, ret = %d\n", ret);
	devm_regulator_put(ctx->oled_dvdd);
	/*devm_regulator_put(ctx->oled_dvdd);*/

	udelay(6000);
	ctx->vci_gpio =
		devm_gpiod_get(ctx->dev, "vci", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vci_gpio %ld\n",
			__func__, PTR_ERR(ctx->vci_gpio));
		return PTR_ERR(ctx->vci_gpio);
	}
	gpiod_set_value(ctx->vci_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vci_gpio);
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

	//_gate_ic_Power_off();
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

	ctx->vci_gpio =
		devm_gpiod_get(ctx->dev, "vci", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vci_gpio %ld\n",
			__func__, PTR_ERR(ctx->vci_gpio));
		return PTR_ERR(ctx->vci_gpio);
	}
	gpiod_set_value(ctx->vci_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vci_gpio);
	udelay(2000);

	ctx->oled_dvdd = devm_regulator_get_optional(ctx->dev, "oled-dvdd");
	if (IS_ERR(ctx->oled_dvdd)) { /* handle return value */
		ret = PTR_ERR(ctx->oled_dvdd);
		pr_err("get dvdd fail, error: %d\n", ret);
		return ret;
	}
	/* set voltage with min & max*/
	ret = regulator_set_voltage(ctx->oled_dvdd, 1254000, 1254000);
	if (ret < 0)
		pr_err("set voltage ctx->oled_dvdd fail, ret = %d\n", ret);
		/* enable regulator */
	ret = regulator_enable(ctx->oled_dvdd);
	if (ret < 0)
		pr_err("enable regulator ctx->oled_dvdd fail, ret = %d\n", ret);

	udelay(2000);

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
static const struct drm_display_mode mode_120hz = {
	.clock = 441653,
	.hdisplay = FRAME_WIDTH,//1200
	.hsync_start = FRAME_WIDTH + FHD_HFP,//1215
	.hsync_end = FRAME_WIDTH + FHD_HFP + FHD_HSA,//1230
	.htotal = FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,//1245
	.vdisplay = FRAME_HEIGHT,//2670
	.vsync_start = FRAME_HEIGHT + FHD_VFP,//2715
	.vsync_end = FRAME_HEIGHT + FHD_VFP + FHD_VSA,//2717
	.vtotal = FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP,//2752
};
static const struct drm_display_mode mode_90hz = {
	.clock = 441653,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + FHD_HFP_90,
	.hsync_end = FRAME_WIDTH + FHD_HFP_90 + FHD_HSA_90,
	.htotal = FRAME_WIDTH + FHD_HFP_90 + FHD_HSA_90 + FHD_HBP_90,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + FHD_VFP_90,
	.vsync_end = FRAME_HEIGHT + FHD_VFP_90 + FHD_VSA_90,
	.vtotal = FRAME_HEIGHT + FHD_VFP_90 + FHD_VSA_90 + FHD_VBP_90,
};
static const struct drm_display_mode mode_60hz = {
	.clock = 441653,
	.hdisplay = FRAME_WIDTH,//1200
	.hsync_start = FRAME_WIDTH + FHD_HFP_60,//1215
	.hsync_end = FRAME_WIDTH + FHD_HFP_60 + FHD_HSA_60,//1230
	.htotal = FRAME_WIDTH + FHD_HFP_60 + FHD_HSA_60+ FHD_HBP_60,//1245
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + FHD_VFP_60,
	.vsync_end = FRAME_HEIGHT + FHD_VFP_60 + FHD_VSA_60,
	.vtotal = FRAME_HEIGHT + FHD_VFP_60 + FHD_VSA_60 + FHD_VBP_60,
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
	struct lcm *ctx = g_ctx;
	char bl_tb[] = {0x51, 0x3F, 0xff};

	printk("%s backlight level = %d  \n",__func__,level);
	bl_tb[1] = (level >> 8) & 0x3F;
	bl_tb[2] = level & 0xFF;
	if (!cb)
		return -1;
	cb(dsi, handle, bl_tb, ARRAY_SIZE(bl_tb));
	atomic_set(&ctx->current_bl, level);
	if (!level)
		atomic_set(&ctx->hbm_mode, 0);
	return 0;
}
static struct mtk_panel_params ext_params = {
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
	},
	//.is_support_od = true,
	//.is_support_dmr = true,
	//.is_support_dbi = true,
	//.lp_perline_en = 1,
	//.te_delay = 1,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	//.dsc_param_load_mode = 2, //0: default flow; 1: key param only; 2: full control
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
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
		.slice_height = 12,
		.slice_width = (FRAME_WIDTH/2),
		.chunk_size = 610,
		.xmit_delay = 512,
		.dec_delay = 282,
		.scale_value = 32,
		.increment_interval = 603,
		.decrement_interval = 8,
		.line_bpg_offset = 1,
		.nfl_bpg_offset = 187,
		.slice_bpg_offset = 1915,
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
/* 	.dyn = {
		.switch_en = 1,
		.pll_clk = PLL_CLOCK + 1,
	}, */
	.lfr_enable = 0,
	.lfr_minimum_fps = 60,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 0,
		.vact_timing_fps = 120,
		.dfps_cmd_grp_table[0] = {2, {0x6C, 0x00} },
		.dfps_cmd_grp_size = 1,
	},
	//.mode_switch_cmdq = MODE_SWITCH_CMDQ_ENABLE,
	//.real_te_duration = 8333,
	//.merge_trig_offset = 13260,
	.panel_cellid_reg = 0xAC,
	.panel_cellid_offset_reg = 0x6F,
	.panel_cellid_offset = 0x0D,
	.panel_cellid_len = 23,
	.panel_ver = 1,
	.panel_name = "csot_vtdr6126_667_vdo_1220_2712",
	.panel_supplier = "csot-vtdr6126",
};
static struct mtk_panel_params ext_params_90hz = {
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
	},
	//.is_support_od = true,
	//.is_support_dmr = true,
	//.is_support_dbi = true,
	//.lp_perline_en = 1,
	//.te_delay = 1,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	//.dsc_param_load_mode = 2, //0: default flow; 1: key param only; 2: full control
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
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
		.slice_height = 12,
		.slice_width = (FRAME_WIDTH/2),
		.chunk_size = 610,
		.xmit_delay = 512,
		.dec_delay = 282,
		.scale_value = 32,
		.increment_interval = 603,
		.decrement_interval = 8,
		.line_bpg_offset = 1,
		.nfl_bpg_offset = 187,
		.slice_bpg_offset = 1915,
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
	.data_rate = 1080,
	/* following MIPI hopping parameter might cause screen mess */
/* 	.dyn = {
		.switch_en = 1,
		.pll_clk = PLL_CLOCK + 1,
	}, */
	.lfr_enable = 0,
	.lfr_minimum_fps = 60,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 0,
		.vact_timing_fps = 120,
		.dfps_cmd_grp_table[0] = {2, {0x6C, 0x01} },
		.dfps_cmd_grp_size = 1,
	},
	//.mode_switch_cmdq = MODE_SWITCH_CMDQ_ENABLE,
	//.real_te_duration = 8333,
	//.merge_trig_offset = 13260,
	.panel_cellid_reg = 0xAC,
	.panel_cellid_offset_reg = 0x6F,
	.panel_cellid_offset = 0x0D,
	.panel_cellid_len = 23,
	.panel_ver = 1,
	.panel_name = "csot_vtdr6126_667_vdo_1220_2712",
	.panel_supplier = "csot-vtdr6126",
};
static struct mtk_panel_params ext_params_60hz = {
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
	},
	//.is_support_od = true,
	//.is_support_dmr = true,
	//.is_support_dbi = true,
	//.lp_perline_en = 1,
	//.te_delay = 1,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	//.dsc_param_load_mode = 2, //0: default flow; 1: key param only; 2: full control
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
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
		.slice_height = 12,
		.slice_width = (FRAME_WIDTH/2),
		.chunk_size = 610,
		.xmit_delay = 512,
		.dec_delay = 282,
		.scale_value = 32,
		.increment_interval = 603,
		.decrement_interval = 8,
		.line_bpg_offset = 1,
		.nfl_bpg_offset = 187,
		.slice_bpg_offset = 1915,
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
	.data_rate = 1080,
	/* following MIPI hopping parameter might cause screen mess */
/* 	.dyn = {
		.switch_en = 1,
		.pll_clk = PLL_CLOCK + 1,
	}, */
	.lfr_enable = 0,
	.lfr_minimum_fps = 60,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 0,
		.vact_timing_fps = 120,
		.dfps_cmd_grp_table[0] = {2, {0x6C, 0x02} },
		.dfps_cmd_grp_size = 1,
	},
	//.mode_switch_cmdq = MODE_SWITCH_CMDQ_ENABLE,
	//.real_te_duration = 8333,
	//.merge_trig_offset = 13260,
	.panel_cellid_reg = 0xAC,
	.panel_cellid_offset_reg = 0x6F,
	.panel_cellid_offset = 0x0D,
	.panel_cellid_len = 23,
	.panel_ver = 1,
	.panel_name = "csot_vtdr6126_667_vdo_1220_2712",
	.panel_supplier = "csot-vtdr6126",
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
/*void get_switch_mode_delay (enum SWITCH_MODE_DELAY **switch_mode_delay, unsigned int mode_num)
{
	unsigned int i = 0;
	printk("%s enter  \n",__func__);
	for (i = 0;  i < mode_num; i++) {
		memcpy(switch_mode_delay[i], switch_mode_delay_table[i],
			sizeof(enum SWITCH_MODE_DELAY) * mode_num);
	}
}
*/
static int mtk_panel_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);
	struct lcm *ctx = panel_to_lcm(panel);

	/*if (dst_fps == 48) {
		ext->params = &ext_params_48hz;
	} else*/
	if (drm_mode_vrefresh(m) == 120) {
		ext->params = &ext_params;
	} else if (drm_mode_vrefresh(m) == 90) {
		ext->params = &ext_params_90hz;
	} else if (drm_mode_vrefresh(m) == 60) {
		ext->params = &ext_params_60hz;
	} else {
		pr_err("%s, errer\n", __func__);
		ret = -EINVAL;
	}

	printk("%s exit current_fps = %d \n",__func__,atomic_read(&ctx->current_fps));
	return ret;
}

static int mtk_panel_ext_param_get(struct drm_panel *panel,
	struct drm_connector *connector,
	struct mtk_panel_params **ext_param,
	unsigned int mode)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);
	struct lcm *ctx = panel_to_lcm(panel);

	if (drm_mode_vrefresh(m) == 120)
		*ext_param = &ext_params;
	else if (drm_mode_vrefresh(m) == 90)
		*ext_param = &ext_params_90hz;
	else if (drm_mode_vrefresh(m) == 60)
		*ext_param = &ext_params_60hz;
	/*else if (drm_mode_vrefresh(m) == 48)
		*ext_param = &ext_params_48hz;*/
	else
		ret = 1;

	if (!ret)
		atomic_set(&ctx->current_fps, drm_mode_vrefresh(m));

	return ret;
}

int mtk_scaling_mode_mapping(int mode_idx)
{
	return (mode_idx % REAL_MODE_NUM);
}

static void mode_switch_to_120(struct drm_panel *panel)
{
//	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("%s\n", __func__);
	//cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

}

static void mode_switch_to_90(struct drm_panel *panel)
{
//	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("%s\n", __func__);

}

static void mode_switch_to_60(struct drm_panel *panel)
{
//	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("%s\n", __func__);
}

static int mode_switch(struct drm_panel *panel,
		struct drm_connector *connector, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	int dst_fps = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, dst_mode);

	pr_info("%s cur_mode = %d dst_mode %d\n", __func__, cur_mode, dst_mode);

	dst_fps = m ? drm_mode_vrefresh(m) : -EINVAL;

	/*if (dst_fps == 48) {
		mode_switch_to_48(panel, stage);
	} else */
	if (dst_fps == 60) { /* 60 switch to 120 */
		mode_switch_to_60(panel);
	} else if (dst_fps == 90) { /* 1200 switch to 60 */
		mode_switch_to_90(panel);
	} else if (dst_fps == 120) { /* 1200 switch to 60 */
		mode_switch_to_120(panel);
	} else {
		pr_err("%s, dst_fps %d\n", __func__, dst_fps);
		ret = -EINVAL;
	}
	return ret;
}

static struct mtk_panel_para_table panel_lhbm_on[] = {
	{5, {0x63, 0x0f, 0xff, 0x0f, 0xa0}},
	{2, {0x62, 0x03}},

};

static struct mtk_panel_para_table panel_lhbm_off[] = {
	{2, {0x62, 0x00}},
	{3, {0x51, 0x03, 0xff}},

};

static int panel_lhbm_set_cmdq(void *dsi, dcs_grp_write_gce cb, void *handle, uint32_t on, uint32_t bl_level, uint32_t fps)
{
	unsigned int para_count = 0;
	struct mtk_panel_para_table *pTable;
	struct mtk_panel_para_table *pTable_alpha;
	unsigned int alpha = 0;

	if (on) {
		if (bl_level <= 0x10AF) {
			pTable_alpha = &panel_lhbm_on[0];

			alpha = lhbm_alpha[bl_level];

			pTable_alpha->para_list[1] = (alpha >> 8) & 0xFF;
			pTable_alpha->para_list[2] = alpha & 0xFF;
			pTable_alpha->para_list[3] = 0x10;
			pTable_alpha->para_list[4] = 0xB0;
			pr_info("%s: backlight %d alpha_hbm %d(0x%x, 0x%x)\n", __func__, bl_level, alpha, pTable_alpha->para_list[1], pTable_alpha->para_list[2]);

			para_count = sizeof(panel_lhbm_on) / sizeof(struct mtk_panel_para_table);
			pTable = panel_lhbm_on;
		} else {
			pTable_alpha = &panel_lhbm_on[0];

			pTable_alpha->para_list[1] = 0x10;
			pTable_alpha->para_list[2] = 0x00;
			pTable_alpha->para_list[3] = (bl_level >> 8) & 0xFF;
			pTable_alpha->para_list[4] = bl_level & 0xFF;

			para_count = sizeof(panel_lhbm_on) / sizeof(struct mtk_panel_para_table);
			pTable = panel_lhbm_on;
		}

	  cb(dsi, handle, pTable, para_count);

	} else {

			pTable_alpha = &panel_lhbm_off[1];
			pTable_alpha->para_list[1] = (bl_level >> 8) & 0xFF;
			pTable_alpha->para_list[2] = bl_level & 0xFF;

			para_count = sizeof(panel_lhbm_off) / sizeof(struct mtk_panel_para_table);
			pTable = panel_lhbm_off;
			cb(dsi, handle, pTable, para_count);
	}
	return 0;

}

static int panel_hbm_set_cmdq(struct lcm *ctx, void *dsi, dcs_grp_write_gce cb, void *handle, uint32_t hbm_state)
{
	//struct mtk_panel_para_table hbm_on_table = {3, {0x51, 0x0F, 0xFF}};
	unsigned int level = atomic_read(&ctx->current_bl);
	unsigned int fps = atomic_read(&ctx->current_fps);

	if (hbm_state > 2) return -1;

	switch (hbm_state)
	{
		case 0:
			if (ctx->lhbm_en){
				panel_lhbm_set_cmdq(dsi, cb, handle, 0, level,  fps);
			}
			break;
		case 1:
			if (ctx->lhbm_en) {
				panel_lhbm_set_cmdq(dsi, cb, handle, 0, level,  fps);

			}
			break;
		case 2:
			if (ctx->lhbm_en){
				panel_lhbm_set_cmdq(dsi, cb, handle, 1, level,  fps);
			}
			break;
		default:
			break;
	}

	atomic_set(&ctx->hbm_mode, hbm_state);
	return 0;
}
static struct mtk_panel_para_table panel_dc_off[] = {
	{2, {0x5E,0x00}},
};

static struct mtk_panel_para_table panel_dc_on[] = {
	{2, {0x5E,0x01}},
};

static int pane_dc_set_cmdq(struct lcm *ctx, void *dsi, dcs_grp_write_gce cb, void *handle, uint32_t dc_state)
{
	unsigned int para_count = 0;
	struct mtk_panel_para_table *pTable;

	if (dc_state) {
		para_count = sizeof(panel_dc_on) / sizeof(struct mtk_panel_para_table);
		pTable = panel_dc_on;
	} else {
		para_count = sizeof(panel_dc_off) / sizeof(struct mtk_panel_para_table);
		pTable = panel_dc_off;
	}
	cb(dsi, handle, pTable, para_count);
	atomic_set(&ctx->dc_mode, dc_state);
	pr_info("%s: current_fps %d\n", __func__, atomic_read(&ctx->current_fps));
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
			param_info->value = atomic_read(&ctx->hbm_mode);
			break;
		case PARAM_DC:
			param_info->value = atomic_read(&ctx->dc_mode);
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
			atomic_set(&ctx->hbm_mode, param_info.value);
			panel_hbm_set_cmdq(ctx, dsi, cb, handle, param_info.value);
			break;
		case PARAM_DC:
			pane_dc_set_cmdq(ctx, dsi, cb, handle, param_info.value);
			atomic_set(&ctx->dc_mode, param_info.value);
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
	.mode_switch = mode_switch,
	//.set_bl_elvss_cmdq = lcm_set_bl_elvss_cmdq,
	/* Not real backlight cmd in AOD, just for QC purpose */
	.ata_check = panel_ata_check,
	.panel_feature_set = panel_feature_set,
	.panel_feature_get = panel_feature_get,
	.scaling_mode_mapping = mtk_scaling_mode_mapping,
	//.lcm_update_roi = lcm_update_roi,
	//.lcm_update_roi_cmdq = lcm_update_roi_cmdq,
	//.get_lcm_power_state = lcm_panel_get_ab_data,
//	.get_switch_mode_delay = get_switch_mode_delay,
};

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
	struct drm_display_mode *mode1;
	struct drm_display_mode *mode2;
	struct drm_display_mode *mode3;
	//struct drm_display_mode *mode4;

	mode1 = drm_mode_duplicate(connector->dev, &mode_120hz);
	if (!mode1) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 mode_120hz.hdisplay, mode_120hz.vdisplay,
			 drm_mode_vrefresh(&mode_120hz));
		return -ENOMEM;
	}

	drm_mode_set_name(mode1);
	mode1->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode1);

	mode2 = drm_mode_duplicate(connector->dev, &mode_90hz);
	if (!mode2) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 mode_90hz.hdisplay, mode_90hz.vdisplay,
			 drm_mode_vrefresh(&mode_90hz));
		return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode2);

	mode3 = drm_mode_duplicate(connector->dev, &mode_60hz);
	if (!mode3) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 mode_60hz.hdisplay, mode_60hz.vdisplay,
			 drm_mode_vrefresh(&mode_60hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode3);
	mode3->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode3);

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
	//dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET
			// | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;
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

	ctx->oled_dvdd = devm_regulator_get_optional(dev, "oled-dvdd");
	if (IS_ERR(ctx->oled_dvdd)) { /* handle return value */
		ret = PTR_ERR(ctx->oled_dvdd);
		pr_err("get dvdd fail, error: %d\n", ret);
		return ret;
	}
	/* set voltage with min & max*/
	ret = regulator_set_voltage(ctx->oled_dvdd, 1254000, 1254000);
	if (ret < 0)
		pr_err("set voltage ctx->oled_dvdd fail, ret = %d\n", ret);
		/* enable regulator */
	ret = regulator_enable(ctx->oled_dvdd);
	if (ret < 0)
		pr_err(" probeenable regulator ctx->oled_dvdd fail, ret = %d\n", ret);

	ctx->vci_gpio = devm_gpiod_get(dev, "vci", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci_gpio)) {
		dev_err(dev, "%s: cannot get vci-gpios %ld\n",
			__func__, PTR_ERR(ctx->vci_gpio));
		return PTR_ERR(ctx->vci_gpio);
	}
	devm_gpiod_put(dev, ctx->vci_gpio);
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
atomic_set(&ctx->hbm_mode, 0);
	atomic_set(&ctx->dc_mode, 0);
	atomic_set(&ctx->apl_mode, 0);
	atomic_set(&ctx->current_fps, 120);

	ctx->lhbm_en = 1;
	pr_info("%s-\n", __func__);
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
	{ .compatible = "csot,vtdr6126,gt9916,vdo,667" },
	{ }
};
MODULE_DEVICE_TABLE(of, lcm_of_match);
static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "csot_vtdr6126_667_vdo_1220_2712",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};
module_mipi_dsi_driver(lcm_driver);
MODULE_AUTHOR("Randy Lin <randy.lin@mediatek.com>");
MODULE_DESCRIPTION("vtdr6126 VDO LCD Panel Driver");
MODULE_LICENSE("GPL v2");
