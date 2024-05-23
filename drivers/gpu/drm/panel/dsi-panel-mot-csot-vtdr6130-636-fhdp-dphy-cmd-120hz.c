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
#define PLL_CLOCK				(505)
#define REAL_MODE_NUM           (6)

#define FHD_FRAME_WIDTH    (1200)
#define FHD_HFP            (15)
#define FHD_HSA            (15)
#define FHD_HBP            (15)
#define FHD_HTOTAL         (FHD_FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP)
#define FHD_FRAME_HEIGHT   (2670)
#define FHD_VFP            (53)
#define FHD_VSA            (2)
#define FHD_VBP            (35)
#define FHD_VTOTAL         (FHD_FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP)
#define FHD_FRAME_TOTAL    (FHD_HTOTAL * FHD_VTOTAL)

#define MODE_SWITCH_CMDQ_ENABLE 1

#define FHD_HFP_90            (15)
#define FHD_HSA_90            (15)
#define FHD_HBP_90            (15)
#define FHD_VFP_90            (53)
#define FHD_VSA_90            (2)
#define FHD_VBP_90            (35)


unsigned int nt37801_wqhs_dsi_cmd_120hz_dphy_buf_thresh[14] = {
	896, 1792, 2688, 3584, 4480, 5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064};
unsigned int nt37801_wqhs_dsi_cmd_120hz_dphy_range_min_qp[15] = {
	0, 4, 5, 5, 7, 7, 7, 7, 7, 7, 9, 9, 9, 13, 16};
unsigned int nt37801_wqhs_dsi_cmd_120hz_dphy_range_max_qp[15] = {
	8, 8, 9, 10, 11, 11, 11, 12, 13, 14, 14, 15, 15, 16, 17};
int nt37801_wqhs_dsi_cmd_120hz_dphy_range_bpg_ofs[15] = {
	2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -12, -12, -12, -12};

enum panel_version{
	PANEL_V1 = 1,
	PANEL_V2,
	PANEL_V3,
};

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
	atomic_t hbm_mode;
	atomic_t dc_mode;
	atomic_t current_backlight;
	atomic_t current_fps;
	enum panel_version version;
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



static void lcm_panel_init(struct lcm *ctx)
{
	char bl_tb[] = {0x51, 0x0f, 0xff};
	unsigned int level = 0;
	printk("%s enter  \n",__func__);
	udelay(2000);
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_LOW);
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

	lcm_dcs_write_seq_static(ctx, 0x59, 0x09);
	lcm_dcs_write_seq_static(ctx, 0x03, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x35, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x53, 0x28);
	lcm_dcs_write_seq_static(ctx, 0x51, 0x0D,0xBA);
	lcm_dcs_write_seq_static(ctx, 0x5e, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x6c, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x6d, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x6f, 0x02);
	lcm_dcs_write_seq_static(ctx, 0x70, 0x11,0x00,0x00,0xab,0x30,0x80,0x0a,0x6e,0x04,0xb0,0x00,0x1e,0x02,0x58,0x02,0x58,0x02,0x00,0x01,0x19,0x00,0x20,0x05,0xd0,0x00,0x08,0x00,0x01,0x00,0x47,0x03,0x0d,0x18,0x00,0x10,0xf0,0x07,0x10,0x20,0x00,0x06,0x0f,0x0f,0x33,0x0e,0x1c,0x2a,0x38,0x46,0x54,0x62,0x69,0x70,0x77,0x79,0x7b,0x7d,0x7e,0x02,0x02,0x22,0x00,0x2a,0x40,0x2a,0xbe,0x3a,0xfc,0x3a,0xfa,0x3a,0xf8,0x3b,0x38,0x3b,0x78,0x3b,0xb6,0x4b,0xb6,0x4b,0xf4,0x4b,0xf4,0x6c,0x34,0x84,0x74,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0x75, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x72, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xff, 0x5a,0x81);
	lcm_dcs_write_seq_static(ctx, 0x65, 0x02);
	lcm_dcs_write_seq_static(ctx, 0xfb, 0xb3,0xb3,0xb3);
	lcm_dcs_write_seq_static(ctx, 0xff, 0x5a,0x80);
	lcm_dcs_write_seq_static(ctx, 0x65, 0x0a);
	lcm_dcs_write_seq_static(ctx, 0xf9, 0x9e,0x8f);
	lcm_dcs_write_seq_static(ctx, 0x65, 0x0f);
	lcm_dcs_write_seq_static(ctx, 0xf9, 0x14);
	lcm_dcs_write_seq_static(ctx, 0xff, 0x5a,0x80);
	lcm_dcs_write_seq_static(ctx, 0x65, 0x11);
	lcm_dcs_write_seq_static(ctx, 0xfa, 0x06);
	lcm_dcs_write_seq_static(ctx, 0xff, 0x5a,0x81);
	lcm_dcs_write_seq_static(ctx, 0x65, 0x03);
	lcm_dcs_write_seq_static(ctx, 0xf3, 0x24);
	lcm_dcs_write_seq_static(ctx, 0x65, 0x05);
	lcm_dcs_write_seq_static(ctx, 0xf3, 0xa0);

	lcm_dcs_write_seq_static(ctx, 0xf0,0xaa,0x10);
	lcm_dcs_write_seq_static(ctx, 0x65,0x02);
	lcm_dcs_write_seq_static(ctx, 0xc7,0xff,0xb9);

if(ctx->version == 1){
	lcm_dcs_write_seq_static(ctx, 0xf0,0xaa,0x10);
	lcm_dcs_write_seq_static(ctx, 0xc1,0x80);
}
	pr_info("%s current_fps:%d\n", __func__, atomic_read(&ctx->current_fps));
	switch (atomic_read(&ctx->current_fps)) {
	case 120:
		lcm_dcs_write_seq_static(ctx, 0x6c,0x01);
		lcm_dcs_write_seq_static(ctx, 0x71,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x1B);
		lcm_dcs_write_seq_static(ctx, 0xD0,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x10);
		lcm_dcs_write_seq_static(ctx, 0xCF,0x09);
		break;
	case 90:
		lcm_dcs_write_seq_static(ctx, 0x6c,0x02);
		lcm_dcs_write_seq_static(ctx, 0x71,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x1B);
		lcm_dcs_write_seq_static(ctx, 0xD0,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x10);
		lcm_dcs_write_seq_static(ctx, 0xCF,0x09);
		break;
	case 60:
		lcm_dcs_write_seq_static(ctx, 0x6C, 0x01);
		lcm_dcs_write_seq_static(ctx, 0x71,0x01,0x01,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x1B);
		lcm_dcs_write_seq_static(ctx, 0xD0,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x10);
		lcm_dcs_write_seq_static(ctx, 0xCF,0x16);
		break;
	case 30:
		lcm_dcs_write_seq_static(ctx, 0x6C, 0x01);
		lcm_dcs_write_seq_static(ctx, 0x71,0x01,0x03,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x1B);
		lcm_dcs_write_seq_static(ctx, 0xD0,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x10);
		lcm_dcs_write_seq_static(ctx, 0xCF,0x16);
		break;
	case 24:
		lcm_dcs_write_seq_static(ctx, 0x6C,0x01);
		lcm_dcs_write_seq_static(ctx, 0x71,0x00);
		lcm_dcs_write_seq_static(ctx, 0x75,0x01);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x1B);
		lcm_dcs_write_seq_static(ctx, 0x65,0x0C);
		lcm_dcs_write_seq_static(ctx, 0xD6,0x04,0x04,0x04,0x04);
		lcm_dcs_write_seq_static(ctx, 0xD0,0x11,0x02);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x10);
		lcm_dcs_write_seq_static(ctx, 0xCF,0x16);
		break;
	case 10:
		lcm_dcs_write_seq_static(ctx, 0x6C,0x01);
		lcm_dcs_write_seq_static(ctx, 0x71,0x00);
		lcm_dcs_write_seq_static(ctx, 0x75,0x02);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x1B);
		lcm_dcs_write_seq_static(ctx, 0xD0,0x11,0x02);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x10);
		lcm_dcs_write_seq_static(ctx, 0xCF,0x09);
		break;
	default:
		pr_info("%s current_fps mismatch:%d\n", __func__,atomic_read(&ctx->current_fps));
		break;
	}


	//backlight
	level = atomic_read(&ctx->current_backlight);
	bl_tb[1] = (level >> 8) & 0xf;
	bl_tb[2] = level & 0xFF;
	lcm_dcs_write(ctx, bl_tb, ARRAY_SIZE(bl_tb));

	lcm_dcs_write_seq_static(ctx, 0x11);
	msleep(85);
	lcm_dcs_write_seq_static(ctx, 0x29);
	atomic_set(&ctx->hbm_mode, 0);
	atomic_set(&ctx->dc_mode, 0);
	atomic_set(&ctx->current_backlight, 0);
	atomic_set(&ctx->current_fps, 120);
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

	if (!ctx->prepared)
		return 0;
	printk("%s enter  \n",__func__);
	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(50);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(150);

	ctx->error = 0;
	ctx->prepared = false;




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
	atomic_set(&ctx->hbm_mode, 0);
	atomic_set(&ctx->dc_mode, 0);
	atomic_set(&ctx->current_backlight, 0);
	atomic_set(&ctx->current_fps, 120);

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
	.clock = 412344,
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
	.clock = 309258,
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
	.clock = 206172,
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
	.clock = 103086,
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
	.clock = 82469,
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
	.clock = 412344,
	.hdisplay = FRAME_WIDTH,//1200
	.hsync_start = FRAME_WIDTH + FHD_HFP,//1215
	.hsync_end = FRAME_WIDTH + FHD_HFP + FHD_HSA,//1230
	.htotal = FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,//1245
	.vdisplay = FRAME_HEIGHT,//2670
	.vsync_start = FRAME_HEIGHT + FHD_VFP,//2715
	.vsync_end = FRAME_HEIGHT + FHD_VFP + FHD_VSA,//2717
	.vtotal = FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP+1,//2752
};

static const struct drm_display_mode mode_1 = {
	.clock = 412344,
	.hdisplay = FRAME_WIDTH,//1200
	.hsync_start = FRAME_WIDTH + FHD_HFP,//1215
	.hsync_end = FRAME_WIDTH + FHD_HFP + FHD_HSA,//1230
	.htotal = FRAME_WIDTH + FHD_HFP + FHD_HSA + FHD_HBP,//1245
	.vdisplay = FRAME_HEIGHT,//2670
	.vsync_start = FRAME_HEIGHT + FHD_VFP,//2715
	.vsync_end = FRAME_HEIGHT + FHD_VFP + FHD_VSA,//2717
	.vtotal = FRAME_HEIGHT + FHD_VFP + FHD_VSA + FHD_VBP+2,//2752
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

	if (atomic_read(&ctx->hbm_mode) && level) {
		pr_info("hbm_mode = %d, skip backlight(%d)\n", atomic_read(&ctx->hbm_mode), level);
		atomic_set(&ctx->current_backlight, level);
		return 0;
	}

	if (!(atomic_read(&ctx->current_backlight) && level))
		pr_info("backlight changed from %u to %u\n", atomic_read(&ctx->current_backlight),level);
	else
		pr_debug("backlight changed from %u to %u\n", atomic_read(&ctx->current_backlight), level);

	printk("%s enter  \n",__func__);
	printk("%s backlight level = %d  \n",__func__,level);
	bl_tb[1] = (level >> 8) & 0xF;
	bl_tb[2] = level & 0xFF;
	if (!cb)
		return -1;
	cb(dsi, handle, bl_tb, ARRAY_SIZE(bl_tb));
	atomic_set(&ctx->current_backlight, level);
	if (!level)
		atomic_set(&ctx->hbm_mode, 0);
	return 0;
}




static struct mtk_panel_params ext_params = {
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x66,
		.count = 3,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
		.para_list[2] = 0x00,
	},
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = 66240,
	.physical_height_um = 147384,
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
	.data_rate = 1010,
	/* following MIPI hopping parameter might cause screen mess */
/*	.dyn = {
		.switch_en = 1,
		.pll_clk = PLL_CLOCK + 1,
	},*/
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.data_rate = 1010,
	},
	.panel_cellid_reg = 0x5A,
	.panel_cellid_offset_reg = 0x65,
	.panel_cellid_len = 23,
	.panel_ver = 1,
	.panel_name = "csot_vtdr6130_636_1200_2670",
	.panel_supplier = "csot-vtdr6130",
};

static struct mtk_panel_params ext_params_90hz = {
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x66,
		.count = 3,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
		.para_list[2] = 0x00,
	},
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = 66240,
	.physical_height_um = 147384,
	.lp_perline_en = 1,
	.te_delay = 1,
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
	.data_rate = 760,
	/* following MIPI hopping parameter might cause screen mess */
/*	.dyn = {
		.switch_en = 1,
		.pll_clk = PLL_CLOCK + 1,
	},*/
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
		.data_rate = 760,
	},
	.panel_cellid_reg = 0x5A,
	.panel_cellid_offset_reg = 0x65,
	.panel_cellid_len = 23,
	.panel_ver = 1,
	.panel_name = "csot_vtdr6130_636_1200_2670",
	.panel_supplier = "csot-vtdr6130",
};

static struct mtk_panel_params ext_params_60hz = {
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x66,
		.count = 3,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
		.para_list[2] = 0x00,
	},
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = 66240,
	.physical_height_um = 147384,
	.lp_perline_en = 1,
	.te_delay = 1,
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
	.data_rate = 500,
	/* following MIPI hopping parameter might cause screen mess */
/*	.dyn = {
		.switch_en = 1,
		.pll_clk = PLL_CLOCK + 1,
	},*/
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 60,
		.data_rate = 500,
	},
	.panel_cellid_reg = 0x5A,
	.panel_cellid_offset_reg = 0x65,
	.panel_cellid_len = 23,
	.panel_ver = 1,
	.panel_name = "csot_vtdr6130_636_1200_2670",
	.panel_supplier = "csot-vtdr6130",
};

static struct mtk_panel_params ext_params_30hz = {
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x66,
		.count = 3,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
		.para_list[2] = 0x00,
	},
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = 66240,
	.physical_height_um = 147384,
	.lp_perline_en = 1,
	.te_delay = 1,
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
	.data_rate = 500,
	/* following MIPI hopping parameter might cause screen mess */
/*	.dyn = {
		.switch_en = 1,
		.pll_clk = PLL_CLOCK + 1,
	},*/
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 30,
		.data_rate = 500,
	},
	.panel_cellid_reg = 0x5A,
	.panel_cellid_offset_reg = 0x65,
	.panel_cellid_len = 23,
	.panel_ver = 1,
	.panel_name = "csot_vtdr6130_636_1200_2670",
	.panel_supplier = "csot-vtdr6130",
};

static struct mtk_panel_params ext_params_24hz = {
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x66,
		.count = 3,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
		.para_list[2] = 0x00,
	},
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = 66240,
	.physical_height_um = 147384,
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
	.data_rate = 1010,
	/* following MIPI hopping parameter might cause screen mess */
/*	.dyn = {
		.switch_en = 1,
		.pll_clk = PLL_CLOCK + 1,
	},*/
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.data_rate = 1010,
	},
	.panel_cellid_reg = 0x5A,
	.panel_cellid_offset_reg = 0x65,
	.panel_cellid_len = 23,
	.panel_ver = 1,
	.panel_name = "csot_vtdr6130_636_1200_2670",
	.panel_supplier = "csot-vtdr6130",
};

static struct mtk_panel_params ext_params_10hz = {
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x66,
		.count = 3,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
		.para_list[2] = 0x00,
	},
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = 66240,
	.physical_height_um = 147384,
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
	.data_rate = 1010,
	/* following MIPI hopping parameter might cause screen mess */
/*	.dyn = {
		.switch_en = 1,
		.pll_clk = PLL_CLOCK + 1,
	},*/
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.data_rate = 1010,
	},
	.panel_cellid_reg = 0x5A,
	.panel_cellid_offset_reg = 0x65,
	.panel_cellid_len = 23,
	.panel_ver = 1,
	.panel_name = "csot_vtdr6130_636_1200_2670",
	.panel_supplier = "csot-vtdr6130",
};

static struct mtk_panel_params ext_params_1hz = {
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x66,
		.count = 3,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
		.para_list[2] = 0x00,
	},
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = 66240,
	.physical_height_um = 147384,
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
	.data_rate = 1010,
	/* following MIPI hopping parameter might cause screen mess */
/*	.dyn = {
		.switch_en = 1,
		.pll_clk = PLL_CLOCK + 1,
	},*/
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.data_rate = 1010,
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

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);
	struct lcm *ctx = panel_to_lcm(panel);
	printk("%s enter  \n",__func__);
	if (drm_mode_vrefresh(m) == 120) {
		ext->params = &ext_params;
	} else if (drm_mode_vrefresh(m) == 90) {
		ext->params = &ext_params_90hz;
	} else if (drm_mode_vrefresh(m) == 60) {
		ext->params = &ext_params_60hz;
	} else if (drm_mode_vrefresh(m) == 30) {
		ext->params = &ext_params_30hz;
	} else if (drm_mode_vrefresh(m) == 24) {
		ext->params = &ext_params_24hz;
	} else if (drm_mode_vrefresh(m) == 10) {
		ext->params = &ext_params_10hz;
	} else if (drm_mode_vrefresh(m) == 1) {
		ext->params = &ext_params_1hz;
	} else
		ret = 1;


	printk("%s exit current_fps = %d \n",__func__,atomic_read(&ctx->current_fps));
	return ret;
}

static void mode_switch_to_90(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	if (stage == BEFORE_DSI_POWERDOWN) {
		struct lcm *ctx = panel_to_lcm(panel);

		lcm_dcs_write_seq_static(ctx, 0x6c,0x02);
		lcm_dcs_write_seq_static(ctx, 0x71,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x1B);
		lcm_dcs_write_seq_static(ctx, 0xD0,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x10);
		lcm_dcs_write_seq_static(ctx, 0xCF,0x09);
		atomic_set(&ctx->current_fps, 90);
	}
}

static void mode_switch_to_120(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	if (stage == BEFORE_DSI_POWERDOWN) {
		struct lcm *ctx = panel_to_lcm(panel);
		lcm_dcs_write_seq_static(ctx, 0x6c,0x01);
		lcm_dcs_write_seq_static(ctx, 0x71,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x1B);
		lcm_dcs_write_seq_static(ctx, 0xD0,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x10);
		lcm_dcs_write_seq_static(ctx, 0xCF,0x09);
		atomic_set(&ctx->current_fps, 120);
	}
}

static void mode_switch_to_60(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	if (stage == BEFORE_DSI_POWERDOWN) {
		struct lcm *ctx = panel_to_lcm(panel);

		lcm_dcs_write_seq_static(ctx, 0x6C, 0x01);
		lcm_dcs_write_seq_static(ctx, 0x71,0x01,0x01,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x1B);
		lcm_dcs_write_seq_static(ctx, 0xD0,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x10);
		lcm_dcs_write_seq_static(ctx, 0xCF,0x16);

		atomic_set(&ctx->current_fps, 60);
	}
}

static void mode_switch_to_30(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	if (stage == BEFORE_DSI_POWERDOWN) {
		struct lcm *ctx = panel_to_lcm(panel);

		lcm_dcs_write_seq_static(ctx, 0x6C, 0x01);
		lcm_dcs_write_seq_static(ctx, 0x71,0x01,0x03,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x1B);
		lcm_dcs_write_seq_static(ctx, 0xD0,0x00);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x10);
		lcm_dcs_write_seq_static(ctx, 0xCF,0x16);

		atomic_set(&ctx->current_fps, 30);
	}
}

static void mode_switch_to_24(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	if (stage == BEFORE_DSI_POWERDOWN) {
		struct lcm *ctx = panel_to_lcm(panel);

		lcm_dcs_write_seq_static(ctx, 0x6C,0x01);
		lcm_dcs_write_seq_static(ctx, 0x71,0x00);
		msleep(10);
		lcm_dcs_write_seq_static(ctx, 0x75,0x01);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x1B);
		lcm_dcs_write_seq_static(ctx, 0x65,0x0C);
		lcm_dcs_write_seq_static(ctx, 0xD6,0x04,0x04,0x04,0x04);
		lcm_dcs_write_seq_static(ctx, 0xD0,0x11,0x02);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x10);
		lcm_dcs_write_seq_static(ctx, 0xCF,0x16);

		atomic_set(&ctx->current_fps, 24);
	}
}

static void mode_switch_to_10(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	if (stage == BEFORE_DSI_POWERDOWN) {
		struct lcm *ctx = panel_to_lcm(panel);

		lcm_dcs_write_seq_static(ctx, 0x6C,0x01);
		lcm_dcs_write_seq_static(ctx, 0x71,0x00);
		msleep(10);
		lcm_dcs_write_seq_static(ctx, 0x75,0x02);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x1B);
		lcm_dcs_write_seq_static(ctx, 0xD0,0x11,0x02);
		lcm_dcs_write_seq_static(ctx, 0xF0,0xAA,0x10);
		lcm_dcs_write_seq_static(ctx, 0xCF,0x09);


		atomic_set(&ctx->current_fps, 10);
	}
}

static void mode_switch_to_1(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	if (stage == BEFORE_DSI_POWERDOWN) {
		struct lcm *ctx = panel_to_lcm(panel);

		lcm_dcs_write_seq_static(ctx, 0x6c, 0x01);
		lcm_dcs_write_seq_static(ctx, 0x71, 0x00);
		msleep(10);
		lcm_dcs_write_seq_static(ctx, 0x75, 0x03);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xAA,0x1B);
		lcm_dcs_write_seq_static(ctx, 0xD0, 0x11,0x02);

		lcm_dcs_write_seq_static(ctx, 0xF0, 0xAA,0x10);
		lcm_dcs_write_seq_static(ctx, 0xCF, 0x09);


		atomic_set(&ctx->current_fps, 1);
	}
}

static int mode_switch(struct drm_panel *panel,
		struct drm_connector *connector, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, dst_mode);

	if (cur_mode == dst_mode)
		return ret;

	pr_info("%s cur_mode = %d dst_mode %d\n", __func__, cur_mode, dst_mode);

	if (drm_mode_vrefresh(m) == 120){
		if (dst_mode == 1) {
			mode_switch_to_1(panel, stage);/*switch to 1 */
		} else if (dst_mode == 2) {
			mode_switch_to_10(panel, stage);/*switch to 10 */
		} else
			mode_switch_to_120(panel, stage);/*switch to 120 */
	} else if (drm_mode_vrefresh(m) == 90) { /*switch to 90 */
		mode_switch_to_90(panel, stage);
	} else if (drm_mode_vrefresh(m) == 60) { /*switch to 60 */
		mode_switch_to_60(panel, stage);
	} else if (drm_mode_vrefresh(m) == 30) { /*switch to 30 */
		mode_switch_to_30(panel, stage);
	} else if (drm_mode_vrefresh(m) == 24) { /*switch to 24 */
		mode_switch_to_24(panel, stage);
	} else
		ret = 1;
	return ret;
}

static struct mtk_panel_para_table panel_lhbm_on_EVT[] = {
	{3, {0xf0,0xaa,0x13}},
	{25, {0xc5,0x02,0xC4,0x02,0x8A,0x03,0x47,0x02,0xC4,0x02,0x8A,0x03,0x47,0x02,0xC4,0x02,0x8A,0x03,0x46,0x02,0xC3,0x02,0x89,0x03,0x45}},
};

static struct mtk_panel_para_table panel_lhbm_bright_on[] = {
    {5, {0x63, 0x0f, 0xff,0x0f,0xff}},
    {2, {0x62, 0x03}},
};

static struct mtk_panel_para_table panel_lhbm_dark_on[] = {
	{5, {0x63, 0x0f, 0xff, 0x0d, 0xba}},
	{2, {0x62, 0x03}},

};

static struct mtk_panel_para_table panel_lhbm_dark_off[] = {
	{2, {0x62, 0x00}},
	{3, {0x51, 0x03, 0xff}},

};
static void set_lhbm_alpha(unsigned int bl_level)
{
	struct mtk_panel_para_table *pTable = &panel_lhbm_dark_on[0];

	unsigned int alpha = 0;
	unsigned int lhbm_alpha_index = bl_level-1;

	if (bl_level == 0)
		lhbm_alpha_index = 0;
	else if (bl_level > sizeof(lhbm_alpha))
		lhbm_alpha_index = sizeof(lhbm_alpha)-1;

	alpha = lhbm_alpha[lhbm_alpha_index];

	pTable->para_list[1] = (alpha >> 8) & 0xFF;
	pTable->para_list[2] = alpha & 0xFF;
	pTable->para_list[3] = 0x05;
	pTable->para_list[4] = 0x2A;
	pr_info("%s: backlight %d alpha %d(0x%x, 0x%x)\n", __func__, bl_level, alpha, pTable->para_list[1], pTable->para_list[2]);
}

static int panel_lhbm_set_cmdq(void *dsi, dcs_grp_write_gce cb, void *handle, uint32_t on, uint32_t bl_level, uint32_t fps)
{
	unsigned int para_count = 0;
	unsigned int para_count_EVT = 0;
	struct lcm *ctx = g_ctx;
	struct mtk_panel_para_table *pTable;
	struct mtk_panel_para_table *pTable_EVT = &panel_lhbm_on_EVT[0];
	para_count_EVT = sizeof(panel_lhbm_on_EVT) / sizeof(struct mtk_panel_para_table);
	if (on) {
		if (bl_level <= ARRAY_SIZE(lhbm_alpha)) {
			set_lhbm_alpha(bl_level);
			para_count = sizeof(panel_lhbm_dark_on) / sizeof(struct mtk_panel_para_table);
			pTable = panel_lhbm_dark_on;
		} else {
			para_count = sizeof(panel_lhbm_bright_on) / sizeof(struct mtk_panel_para_table);
			panel_lhbm_bright_on[0].para_list[3] = (bl_level >> 8) & 0xFF;
			panel_lhbm_bright_on[0].para_list[4] = bl_level & 0xFF;
			pTable = panel_lhbm_bright_on;
		}
		if(ctx->version == 1){
		  cb(dsi, handle, pTable_EVT, para_count_EVT);
	      cb(dsi, handle, pTable, para_count);
		}
		else{
		  cb(dsi, handle, pTable, para_count);
		}
	} else {
			pTable = &panel_lhbm_dark_off[1];
			pTable->para_list[1] = (bl_level >> 8) & 0xFF;
			pTable->para_list[2] = bl_level & 0xFF;
			para_count = sizeof(panel_lhbm_dark_off) / sizeof(struct mtk_panel_para_table);
			pTable = panel_lhbm_dark_off;
			cb(dsi, handle, pTable, para_count);
	}
	return 0;

}
static int panel_hbm_set_cmdq(struct lcm *ctx, void *dsi, dcs_grp_write_gce cb, void *handle, uint32_t hbm_state)
{
	struct mtk_panel_para_table hbm_on_table = {3, {0x51, 0x0F, 0xFF}};
	unsigned int level = 0;
	unsigned int fps = 120;
	fps = atomic_read(&ctx->current_fps);
	level = atomic_read(&ctx->current_backlight);
	if (hbm_state > 2) return -1;

	switch (hbm_state)
	{
		case 0:
			if (ctx->lhbm_en)
				panel_lhbm_set_cmdq(dsi, cb, handle, 0, level, fps);
			break;
		case 1:
			if (ctx->lhbm_en) {
				panel_lhbm_set_cmdq(dsi, cb, handle, 0, level, fps);

			} else {
				cb(dsi, handle, &hbm_on_table, 1);
			}
			break;
		case 2:
			if (ctx->lhbm_en)
				panel_lhbm_set_cmdq(dsi, cb, handle, 1, level,  fps);
			else
				cb(dsi, handle, &hbm_on_table, 1);
			break;
		default:
			break;
	}
	atomic_set(&ctx->hbm_mode, hbm_state);
	return 0;
}

static struct mtk_panel_para_table panel_dc_off[] = {
	{2, {0x5e, 0x00}},
};

static struct mtk_panel_para_table panel_dc_on[] = {
	{2, {0x5e, 0x01}},
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

static int panel_ext_init_power(struct drm_panel *panel)
{
	int ret;
	struct lcm *ctx = panel_to_lcm(panel);
	ctx->vddi_gpio = devm_gpiod_get(ctx->dev, "vddi", GPIOD_OUT_HIGH);
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
	ctx->dvdd_gpio = devm_gpiod_get(ctx->dev, "dvdd", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->dvdd_gpio)) {
		dev_err(ctx->dev, "%s: cannot get dvdd_gpio %ld\n",
			__func__, PTR_ERR(ctx->dvdd_gpio));
		return PTR_ERR(ctx->dvdd_gpio);
	}
	gpiod_set_value(ctx->dvdd_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->dvdd_gpio);
	msleep(15);
	return ret;
}

static int panel_ext_powerdown(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret = 0;
	pr_info("%s+\n", __func__);
	if (ctx->prepared)
	    return 0;

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_LOW);
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
	pr_info("%s-\n", __func__);

	return 0;
}
static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.init_power = panel_ext_init_power,
	.power_down = panel_ext_powerdown,
	.ext_param_set = mtk_panel_ext_param_set,
	.mode_switch = mode_switch,
	.ata_check = panel_ata_check,
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

	printk("%s enter  \n",__func__);

	mode5 = drm_mode_duplicate(connector->dev, &mode_1);
	if (!mode5)
		return -ENOMEM;

	drm_mode_set_name(mode5);
	snprintf(mode5->name,  DRM_DISPLAY_MODE_LEN,  "%s@%d",mode5->name, RRGSFlag_Special_Idle_1Hz);
	mode5->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode5);

	mode4 = drm_mode_duplicate(connector->dev, &mode_10);
	if (!mode4) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			mode_10.hdisplay, mode_10.vdisplay,
			drm_mode_vrefresh(&mode_10));
		return -ENOMEM;
	}

	drm_mode_set_name(mode4);
	snprintf(mode4->name,  DRM_DISPLAY_MODE_LEN,  "%s@%d",mode4->name, RRGSFlag_Special_Idle_10Hz);
	mode4->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode4);

	mode3 = drm_mode_duplicate(connector->dev, &mode_24);
	if (!mode3) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			mode_24.hdisplay, mode_24.vdisplay,
			drm_mode_vrefresh(&mode_24));
		return -ENOMEM;
	}

	drm_mode_set_name(mode3);
	snprintf(mode3->name,  DRM_DISPLAY_MODE_LEN,  "%s@%d",mode3->name, RRGSFlag_All_No_Duplicated | RRGSFlag_120HzBased);
	mode3->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode3);

	mode2 = drm_mode_duplicate(connector->dev, &mode_30);
	if (!mode2) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			mode_30.hdisplay, mode_30.vdisplay,
			drm_mode_vrefresh(&mode_30));
		return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	snprintf(mode2->name,  DRM_DISPLAY_MODE_LEN,  "%s@%d",mode2->name, RRGSFlag_All_No_Duplicated | RRGSFlag_120HzBased | RRGSFlag_90HzBased);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode2);

	mode1 = drm_mode_duplicate(connector->dev, &mode_60);
	if (!mode1) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			mode_60.hdisplay, mode_60.vdisplay,
			drm_mode_vrefresh(&mode_60));
		return -ENOMEM;
	}

	drm_mode_set_name(mode1);
	mode1->type = DRM_MODE_TYPE_DRIVER;
	snprintf(mode1->name,  DRM_DISPLAY_MODE_LEN,  "%s@%d",mode1->name, RRGSFlag_All_No_Duplicated | RRGSFlag_120HzBased);
	drm_mode_probed_add(connector, mode1);

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	snprintf(mode->name,  DRM_DISPLAY_MODE_LEN,  "%s@%d",mode->name, RRGSFlag_All_No_Duplicated | RRGSFlag_120HzBased);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	mode6 = drm_mode_duplicate(connector->dev, &mode_90);
	if (!mode6) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			mode_90.hdisplay, mode_90.vdisplay,
			drm_mode_vrefresh(&mode_90));
		return -ENOMEM;
	}

	drm_mode_set_name(mode6);
	snprintf(mode6->name,  DRM_DISPLAY_MODE_LEN,  "%s@%d",mode6->name, RRGSFlag_All_No_Duplicated | RRGSFlag_90HzBased);
	mode6->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode6);


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
	int ret;
	const u32 *val;

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

	val = of_get_property(dev->of_node, "panel-version", NULL);
	ctx->version = val ? be32_to_cpup(val) : 1;

	printk("%s: panel version 0x%x\n", __func__, ctx->version);

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
	atomic_set(&ctx->current_fps, 120);
	atomic_set(&ctx->current_backlight, 1644);

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
