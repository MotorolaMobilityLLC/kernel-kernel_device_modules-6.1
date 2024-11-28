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

#define DIC_COMMAND_MODE_AOD

#if 0
#define AOD_AREA_IZE 1416
#define AOD_Y_START_MIN 0
#define AOD_Y_START_MAX 378
#define AOD_Y_START_STEP_MIN 4
#endif

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#include "include/dsi-panel-mot-boe-vtdr6126a-667-1220x2712-dphy-vdo-120hz-lhbm-alpha.h"
//#include "../../../misc/mediatek/gate_ic/gate_i2c.h"

/* enable this to check panel self -bist pattern */
/* #define PANEL_BIST_PATTERN */
/****************TPS65132***********/
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
//#include "lcm_i2c.h"

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
	struct gpio_desc *bias_pos;
	struct gpio_desc *bias_neg;
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
	atomic_t doze_enable;
	//atomic_t current_aod_y_start;
	enum panel_version version;
};

static struct lcm *g_ctx = NULL;

static unsigned int vdo_buf_thresh[14] = {896, 1792, 2688, 3584, 4480, 5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064};
static unsigned int vdo_range_min_qp[15] = {0, 4, 5, 5, 7, 7, 7, 7, 7, 7, 9, 9, 9, 13, 16};
static unsigned int vdo_range_max_qp[15] = {8, 8, 9, 10, 11, 11, 11, 12, 13, 14, 14, 15, 15, 16, 17};
static int vdo_range_bpg_ofs[15] = {2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -12, -12, -12, -12};

#define lcm_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = { seq };                                        \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define lcm_dcs_write_seq_static(ctx, seq...)  \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define APL_THRESHOLD 16000

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
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
		dev_info(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	pr_info("%s+\n", __func__);

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("%s  0x%08x\n", __func__, buffer[0] | (buffer[1] << 8));
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

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

static void lcm_panel_init(struct lcm *ctx)
{
//Vesa on
	lcm_dcs_write_seq_static(ctx, 0x03, 0x01);
//TE
	lcm_dcs_write_seq_static(ctx, 0x35, 0x00);
//DBV =0x00 0x00
	lcm_dcs_write_seq_static(ctx, 0x51, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x53, 0x20);
	lcm_dcs_write_seq_static(ctx, 0x55, 0x10);
//59h=09 demura on   59h=00 demura off
	lcm_dcs_write_seq_static(ctx, 0x59, 0x09);
//5Eh 0x00 BC mode 0x01 DC mode
	lcm_dcs_write_seq_static(ctx, 0x5e, 0x00);
//120hz 6c=00 ,90hz 6c=01 ,60hz 6c=02 ,48hz 6c=03
	lcm_dcs_write_seq_static(ctx, 0x6c, 0x00);
//AOD1 5nit 6D=00 ,AOD2 60nit 6D=01 ,AOD3 160nit 6D=02
	lcm_dcs_write_seq_static(ctx, 0x6d, 0x00);
//CMD mode 6F=02, Video mode 6F=01
	lcm_dcs_write_seq_static(ctx, 0x6f, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x72, 0x00);
//Vesa PPS (DSC V1.1)
	lcm_dcs_write_seq_static(ctx, 0x70, 0x11, 0x00, 0x00, 0xAB, 0x30, 0x80, 0x0A, 0x98, 0x04, 0xC4, 0x00, 0x0C, 0x02, 0x62,
					0x02, 0x62, 0x02, 0x00, 0x01, 0x1A, 0x00, 0x20, 0x02, 0x5B, 0x00, 0x08, 0x00, 0x01, 0x00, 0xBB,
					0x07, 0x7B, 0x18, 0x00, 0x10, 0xF0, 0x07, 0x10, 0x20, 0x00, 0x06, 0x0F, 0x0F, 0x33, 0x0E, 0x1C,
					0x2A, 0x38, 0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B, 0x7D, 0x7E, 0x02, 0x02, 0x22, 0x00,
					0x2A, 0x40, 0x2A, 0xBE, 0x3A, 0xFC, 0x3A, 0xFA, 0x3A, 0xF8, 0x3B, 0x38, 0x3B, 0x78, 0x3B, 0xB6,
					0x4B, 0xB6, 0x4B, 0xF4, 0x4B, 0xF4, 0x6C, 0x34, 0x84, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
//LV ON
	lcm_dcs_write_seq_static(ctx, 0xf0, 0xaa, 0x12);
	lcm_dcs_write_seq_static(ctx, 0xC7, 0x5F, 0x78, 0x43, 0x5A, 0x33, 0x33);
	lcm_dcs_write_seq_static(ctx, 0xd1, 0x06, 0x01, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xd6, 0x04);

	lcm_dcs_write_seq_static(ctx, 0xF0, 0xaa, 0x10);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x05, 0x4C, 0x01, 0x31, 0x00, 0x04, 0xC4, 0x05, 0x4C);
	lcm_dcs_write_seq_static(ctx, 0xBE, 0x3E, 0x79);

	lcm_dcs_write_seq_static(ctx, 0xf0, 0xaa, 0x16);
	lcm_dcs_write_seq_static(ctx, 0xd1, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x65, 0x03);
	lcm_dcs_write_seq_static(ctx, 0xd1, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x65, 0x06);
	lcm_dcs_write_seq_static(ctx, 0xd1, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x65, 0x09);
	lcm_dcs_write_seq_static(ctx, 0xd1, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x65, 0x0c);
	lcm_dcs_write_seq_static(ctx, 0xd1, 0x00, 0x00, 0x00);

	lcm_dcs_write_seq_static(ctx, 0xff, 0x5a, 0x81);
	lcm_dcs_write_seq_static(ctx, 0x65, 0x03);
	lcm_dcs_write_seq_static(ctx, 0xf3, 0x61);
	lcm_dcs_write_seq_static(ctx, 0x65, 0x0b);
	lcm_dcs_write_seq_static(ctx, 0xf3, 0x78);
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x5A, 0x80);
	lcm_dcs_write_seq_static(ctx, 0x65, 0x25);
	lcm_dcs_write_seq_static(ctx, 0xfd, 0x01);

	lcm_dcs_write_seq_static(ctx, 0xf0, 0xaa, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xff, 0x5a, 0x00);

	lcm_dcs_write_seq_static(ctx, 0x2A, 0x00, 0x00, 0x04, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0x2B, 0x00, 0x00, 0x05, 0x87);

	lcm_dcs_write_seq_static(ctx, 0x11);
	usleep_range(85 * 1000, 86 * 1000);
	lcm_dcs_write_seq_static(ctx, 0x29);

	atomic_set(&ctx->hbm_mode, 0);
	atomic_set(&ctx->dc_mode, 0);
	atomic_set(&ctx->apl_mode, 0);
	atomic_set(&ctx->current_bl, 0);
	atomic_set(&ctx->current_fps, 120);

	pr_info("%s-\n", __func__);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int gate_ic_Power_on(struct drm_panel *panel, int enabled)
{
	struct lcm *ctx = panel_to_lcm(panel);
	bool gpio_status;
	struct gpio_desc *pm_en_pin;
	int i;
	pr_info("%s+ enable %d\n", __func__, enabled);
	gpio_status = enabled ? 1:0;
	if (gpio_status) {
		for (i=0; i < 3; i++) {
			pm_en_pin = NULL;
			pm_en_pin = devm_gpiod_get_index(ctx->dev, "pm-enable", i, GPIOD_OUT_HIGH);
			if (IS_ERR(pm_en_pin)) {
				pr_err("cannot get bias-gpios %d %ld\n", i, PTR_ERR(pm_en_pin));
				return PTR_ERR(pm_en_pin);
			}
			gpiod_set_value(pm_en_pin, gpio_status);
			devm_gpiod_put(ctx->dev, pm_en_pin);
			usleep_range(1000, 1001);
		}
	} else {
		for (i=2; i >=0; i--) {
			pm_en_pin = NULL;
			pm_en_pin = devm_gpiod_get_index(ctx->dev, "pm-enable", i, GPIOD_OUT_LOW);
			if (IS_ERR(pm_en_pin)) {
				pr_err("cannot get bias-gpios %d %ld\n", i, PTR_ERR(pm_en_pin));
				return PTR_ERR(pm_en_pin);
			}
			gpiod_set_value(pm_en_pin, gpio_status);
			devm_gpiod_put(ctx->dev, pm_en_pin);
			usleep_range(1000, 1001);
		}
	}
	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);


	pr_info("%s+\n", __func__);
	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(10);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(90);

	ctx->error = 0;
	ctx->prepared = false;

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s+\n", __func__);
	if (ctx->prepared)
		return 0;

	// lcd reset L->H -> L -> L
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_LOW);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(11000, 11001);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(1000, 1001);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(1000, 1001);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(20000, 20001);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	// end

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0) goto error;

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	atomic_set(&ctx->hbm_mode, 0);
	atomic_set(&ctx->dc_mode, 0);
	atomic_set(&ctx->apl_mode, 0);
	atomic_set(&ctx->current_bl, 0);
	atomic_set(&ctx->current_fps, 120);
	atomic_set(&ctx->pcd_mode, 0);
	atomic_set(&ctx->doze_enable, 0);

	pr_info("%s-\n", __func__);
	return ret;
error:
	lcm_unprepare(panel);
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

#define FHDP_FRAME_WIDTH    (1220)
#define FHDP_HFP            (16)
#define FHDP_HSA            (8)
#define FHDP_HBP            (8)
#define FHDP_HTOTAL         (FHDP_FRAME_WIDTH + FHDP_HFP + FHDP_HSA + FHDP_HBP)
#define FHDP_FRAME_HEIGHT   (2712)
#define FHDP_VFP_120        (56)
#define FHDP_VFP_90         (984)
#define FHDP_VFP_60         (2840)
#define FHDP_VFP_48         (4232)
#define FHDP_VSA            (4)
#define FHDP_VBP            (12)
#define FHDP_VTOTAL_120     (FHDP_FRAME_HEIGHT + FHDP_VFP_120 + FHDP_VSA + FHDP_VBP)
#define FHDP_VTOTAL_90      (FHDP_FRAME_HEIGHT + FHDP_VFP_90 + FHDP_VSA + FHDP_VBP)
#define FHDP_VTOTAL_60      (FHDP_FRAME_HEIGHT + FHDP_VFP_60 + FHDP_VSA + FHDP_VBP)
#define FHDP_VTOTAL_48      (FHDP_FRAME_HEIGHT + FHDP_VFP_48 + FHDP_VSA + FHDP_VBP)
#define FHDP_FRAME_TOTAL_120 (FHDP_VTOTAL_120 * FHDP_HTOTAL)
#define FHDP_FRAME_TOTAL_90  (FHDP_VTOTAL_90 * FHDP_HTOTAL)
#define FHDP_FRAME_TOTAL_60  (FHDP_VTOTAL_60 * FHDP_HTOTAL)
#define FHDP_FRAME_TOTAL_48  (FHDP_VTOTAL_48 * FHDP_HTOTAL)
#define FHDP_VREFRESH_120   (120)
#define FHDP_VREFRESH_90    (90)
#define FHDP_VREFRESH_60    (60)
#define FHDP_VREFRESH_48    (48)
#define FHDP_CLK_120_X10    ((FHDP_FRAME_TOTAL_120 * FHDP_VREFRESH_120) / 100)
#define FHDP_CLK_90_X10     ((FHDP_FRAME_TOTAL_90 * FHDP_VREFRESH_90) / 100)
#define FHDP_CLK_60_X10     ((FHDP_FRAME_TOTAL_60 * FHDP_VREFRESH_60) / 100)
#define FHDP_CLK_48_X10     ((FHDP_FRAME_TOTAL_48 * FHDP_VREFRESH_48) / 100)
#define FHDP_CLK_120		   (((FHDP_CLK_120_X10 % 10) != 0) ?             \
			(FHDP_CLK_120_X10 / 10 + 1) : (FHDP_CLK_120_X10 / 10))
#define FHDP_CLK_90		   (((FHDP_CLK_90_X10 % 10) != 0) ?             \
				(FHDP_CLK_90_X10 / 10 + 1) : (FHDP_CLK_90_X10 / 10))
#define FHDP_CLK_60		   (((FHDP_CLK_60_X10 % 10) != 0) ?             \
				(FHDP_CLK_60_X10 / 10 + 1) : (FHDP_CLK_60_X10 / 10))
#define FHDP_CLK_48		   (((FHDP_CLK_48_X10 % 10) != 0) ?             \
				(FHDP_CLK_48_X10 / 10 + 1) : (FHDP_CLK_48_X10 / 10))
/*
static const struct drm_display_mode mode_48hz = {
	.clock = FHDP_CLK_48,
	.hdisplay = FHDP_FRAME_WIDTH,
	.hsync_start = FHDP_FRAME_WIDTH + FHDP_HFP,//HFP
	.hsync_end = FHDP_FRAME_WIDTH + FHDP_HFP + FHDP_HSA,//HSA
	.htotal = FHDP_FRAME_WIDTH + FHDP_HFP + FHDP_HSA + FHDP_HBP,//HBP
	.vdisplay = FHDP_FRAME_HEIGHT,
	.vsync_start = FHDP_FRAME_HEIGHT + FHDP_VFP_48,//VFP
	.vsync_end = FHDP_FRAME_HEIGHT + FHDP_VFP_48 + FHDP_VSA,//VSA
	.vtotal = FHDP_FRAME_HEIGHT + FHDP_VFP_48 + FHDP_VSA + FHDP_VBP,//VBP
};
*/

static const struct drm_display_mode mode_60hz = {
	.clock = FHDP_CLK_60,
	.hdisplay = FHDP_FRAME_WIDTH,
	.hsync_start = FHDP_FRAME_WIDTH + FHDP_HFP,//HFP
	.hsync_end = FHDP_FRAME_WIDTH + FHDP_HFP + FHDP_HSA,//HSA
	.htotal = FHDP_FRAME_WIDTH + FHDP_HFP + FHDP_HSA + FHDP_HBP,//HBP
	.vdisplay = FHDP_FRAME_HEIGHT,
	.vsync_start = FHDP_FRAME_HEIGHT + FHDP_VFP_60,//VFP
	.vsync_end = FHDP_FRAME_HEIGHT + FHDP_VFP_60 + FHDP_VSA,//VSA
	.vtotal = FHDP_FRAME_HEIGHT + FHDP_VFP_60 + FHDP_VSA + FHDP_VBP,//VBP
};

static const struct drm_display_mode mode_90hz = {
	.clock = FHDP_CLK_90,
	.hdisplay = FHDP_FRAME_WIDTH,
	.hsync_start = FHDP_FRAME_WIDTH + FHDP_HFP,//HFP
	.hsync_end = FHDP_FRAME_WIDTH + FHDP_HFP + FHDP_HSA,//HSA
	.htotal = FHDP_FRAME_WIDTH + FHDP_HFP + FHDP_HSA + FHDP_HBP,//HBP
	.vdisplay = FHDP_FRAME_HEIGHT,
	.vsync_start = FHDP_FRAME_HEIGHT + FHDP_VFP_90,//VFP
	.vsync_end = FHDP_FRAME_HEIGHT + FHDP_VFP_90 + FHDP_VSA,//VSA
	.vtotal = FHDP_FRAME_HEIGHT + FHDP_VFP_90 + FHDP_VSA + FHDP_VBP,//VBP
};

static const struct drm_display_mode mode_120hz = {
	.clock = FHDP_CLK_120,
	.hdisplay = FHDP_FRAME_WIDTH,
	.hsync_start = FHDP_FRAME_WIDTH + FHDP_HFP,//HFP
	.hsync_end = FHDP_FRAME_WIDTH + FHDP_HFP + FHDP_HSA,//HSA
	.htotal = FHDP_FRAME_WIDTH + FHDP_HFP + FHDP_HSA + FHDP_HBP,//HBP
	.vdisplay = FHDP_FRAME_HEIGHT,
	.vsync_start = FHDP_FRAME_HEIGHT + FHDP_VFP_120,//VFP
	.vsync_end = FHDP_FRAME_HEIGHT + FHDP_VFP_120 + FHDP_VSA,//VSA
	.vtotal = FHDP_FRAME_HEIGHT + FHDP_VFP_120 + FHDP_VSA + FHDP_VBP,//VBP
};

#if defined(CONFIG_MTK_PANEL_EXT)
#if 0
static struct mtk_panel_params ext_params_48hz = {
	.pll_clk = 441,
	//.vfp_low_power = 4180,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x66,
		.count = 2,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
		.para_list_aod[0] = 0x08,
		.para_list_aod[1] = 0x00,
	},
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = 69540,
	.physical_height_um = 154454,
	.lp_perline_en = 1,
	.vdo_per_frame_lp_enable = 1,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 2088,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2712,
		.pic_width = 1220,
		.slice_height = 12,
		.slice_width = 610,
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
			.rc_buf_thresh = vdo_buf_thresh,
			.range_min_qp = vdo_range_min_qp,
			.range_max_qp = vdo_range_max_qp,
			.range_bpg_ofs = vdo_range_bpg_ofs,
		},
	},

	.data_rate = 882,
	.lfr_enable = 0,
	.lfr_minimum_fps = 60,

	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 0,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2, {0x6C, 0x03} },
	},

	.panel_cellid_reg = 0x5a,
	.panel_cellid_offset_reg = 0x65,
	.panel_cellid_len = 23,

	.panel_ver = 1,
	.panel_name = "boe_vtdr6126a_667_1220_2712",
	.panel_supplier = "boe-vtdr6126a",

	.check_panel_feature = 1,
};
#endif

static struct mtk_panel_params ext_params_60hz = {
	.pll_clk = 441,
	//.vfp_low_power = 4180,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x66,
		.count = 2,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
		.para_list_aod[0] = 0x08,
	},
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = 69540,
	.physical_height_um = 154454,
	.lp_perline_en = 1,
	.vdo_per_frame_lp_enable = 1,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 2088,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2712,
		.pic_width = 1220,
		.slice_height = 12,
		.slice_width = 610,
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
			.rc_buf_thresh = vdo_buf_thresh,
			.range_min_qp = vdo_range_min_qp,
			.range_max_qp = vdo_range_max_qp,
			.range_bpg_ofs = vdo_range_bpg_ofs,
		},
	},

	.data_rate = 882,
	.lfr_enable = 0,
	.lfr_minimum_fps = 60,

	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 0,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2, {0x6C, 0x02} },
	},

	.panel_cellid_reg = 0x5a,
	.panel_cellid_offset_reg = 0x65,
	.panel_cellid_len = 23,

	.panel_ver = 1,
	.panel_name = "boe_vtdr6126a_667_1220_2712",
	.panel_supplier = "boe-vtdr6126a",

	.check_panel_feature = 1,
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 441,
	//.vfp_low_power = 2578,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x66,
		.count = 2,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
		.para_list_aod[0] = 0x08,
	},
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = 69540,
	.physical_height_um = 154454,
	.lp_perline_en = 1,
	.vdo_per_frame_lp_enable = 1,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_param_load_mode = 0, //0: default flow; 1: key param only; 2: full control
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 2088,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2712,
		.pic_width = 1220,
		.slice_height = 12,
		.slice_width = 610,
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
			.rc_buf_thresh = vdo_buf_thresh,
			.range_min_qp = vdo_range_min_qp,
			.range_max_qp = vdo_range_max_qp,
			.range_bpg_ofs = vdo_range_bpg_ofs,
		},
	},
	.data_rate = 882,
	.lfr_enable = 0,
	.lfr_minimum_fps = 60,

	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 0,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2, {0x6C, 0x01} },
	},

	.panel_cellid_reg = 0x5a,
	.panel_cellid_offset_reg = 0x65,
	.panel_cellid_offset = 0x0D,
	.panel_cellid_len = 23,

	.panel_ver = 1,
	.panel_name = "boe_vtdr6126a_667_1220_2712",
	.panel_supplier = "boe-vtdr6126a",

	.check_panel_feature = 1,
};

static struct mtk_panel_params ext_params_120hz = {
	.pll_clk = 441,
	//.vfp_low_power = 2578,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x66,
		.count = 2,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
		.para_list_aod[0] = 0x08,
	},
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = 69540,
	.physical_height_um = 154454,
	.lp_perline_en = 1,
	.vdo_per_frame_lp_enable = 1,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_param_load_mode = 0, //0: default flow; 1: key param only; 2: full control
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 2088,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2712,
		.pic_width = 1220,
		.slice_height = 12,
		.slice_width = 610,
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
			.rc_buf_thresh = vdo_buf_thresh,
			.range_min_qp = vdo_range_min_qp,
			.range_max_qp = vdo_range_max_qp,
			.range_bpg_ofs = vdo_range_bpg_ofs,
		},
	},
	.data_rate = 882,
	.lfr_enable = 0,
	.lfr_minimum_fps = 60,

	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 0,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2, {0x6C, 0x00} },
	},

	.panel_cellid_reg = 0x5a,
	.panel_cellid_offset_reg = 0x65,
	.panel_cellid_offset = 0x0D,
	.panel_cellid_len = 23,

	.panel_ver = 1,
	.panel_name = "boe_vtdr6126a_667_1220_2712",
	.panel_supplier = "boe-vtdr6126a",

	.check_panel_feature = 1,
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	struct lcm *ctx = g_ctx;
	char bl_tb[] = {0x51, 0x3E, 0x80};
	char bl_tb_aod[] = {0x6D, 0x02};
	unsigned int aod_light_mode = 0;
	static unsigned int current_aod_light_mode = 0;

	if (atomic_read(&ctx->hbm_mode) && level) {
		pr_info("hbm_mode = %d, skip backlight(%d)\n", atomic_read(&ctx->hbm_mode), level);
		atomic_set(&ctx->current_bl, level);
		return 0;
	}

	if (!cb)
		return -1;

	bl_tb[1] = (level >> 8) & 0x3F;
	bl_tb[2] = level & 0xFF;
	cb(dsi, handle, bl_tb, ARRAY_SIZE(bl_tb));

	if (level > 7300) aod_light_mode = 2;
	else if (level > 4400) aod_light_mode =1;
	else aod_light_mode = 0;

	bl_tb_aod[1] = aod_light_mode;
	if (current_aod_light_mode != aod_light_mode) {
		cb(dsi, handle, bl_tb_aod, ARRAY_SIZE(bl_tb_aod));
		current_aod_light_mode = aod_light_mode;
		if (atomic_read(&ctx->doze_enable))
			pr_info("%s: backlight_level %d aod_light_mode %d\n", __func__, level, aod_light_mode);
	}

	if (!(atomic_read(&ctx->current_bl) && level))
		pr_info("backlight changed from %u to %u\n", atomic_read(&ctx->current_bl),level);
	else
		pr_debug("backlight changed from %u to %u\n", atomic_read(&ctx->current_bl), level);

	atomic_set(&ctx->current_bl, level);
	if (!level)
		atomic_set(&ctx->hbm_mode, 0);
	return 0;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

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
	int dst_fps = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);
	struct lcm *ctx = panel_to_lcm(panel);

	dst_fps = m ? drm_mode_vrefresh(m) : -EINVAL;

	/*if (dst_fps == 48) {
		ext->params = &ext_params_48hz;
	} else*/
	if (dst_fps == 60) {
		ext->params = &ext_params_60hz;
	} else if (dst_fps == 90) {
		ext->params = &ext_params_90hz;
	} else if (dst_fps == 120) {
		ext->params = &ext_params_120hz;
	} else {
		pr_err("%s, dst_fps %d\n", __func__, dst_fps);
		ret = -EINVAL;
	}

	if (!ret)
		atomic_set(&ctx->current_fps, drm_mode_vrefresh(m));
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
		*ext_param = &ext_params_120hz;
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

#define REAL_MODE_NUM           (3)
int mtk_scaling_mode_mapping(int mode_idx)
{
	return (mode_idx % REAL_MODE_NUM);
}

static struct mtk_panel_para_table panel_lhbm_on[] = {
	{6, {0x63, 0x10, 0x00, 0x3E, 0x80, 0x02}},
	{2, {0x62, 0x03}},
};

static struct mtk_panel_para_table panel_lhbm_off[] = {
	{3, {0x51, 0x3E, 0x80}},
	{2, {0x62, 0x00}},
};


static void set_lhbm_alpha(unsigned int bl_level)
{
	struct mtk_panel_para_table *pTable = &panel_lhbm_on[0];

	unsigned int alpha = 0;
	unsigned int lhbm_alpha_index = bl_level-1;

	if (bl_level == 0)
		lhbm_alpha_index = 0;
	else if (bl_level > 16000)
		lhbm_alpha_index = 15999;

	if (bl_level > 0x3E7F) {
		pTable->para_list[1] = 0x10;
		pTable->para_list[2] = 0x00;
		pTable->para_list[3] = (bl_level >> 8) & 0xFF;
		pTable->para_list[4] = bl_level & 0xFF;
		pTable->para_list[5] = 2;
	} else if (bl_level > 0x271B) {
		pTable->para_list[1] = 0x10;
		pTable->para_list[2] = 0x00;
		pTable->para_list[3] = (bl_level >> 8) & 0xFF;
		pTable->para_list[4] = bl_level & 0xFF;
		pTable->para_list[5] = 1;
	pTable->para_list[14] = alpha & 0xFF;
	} else if (bl_level > 0x10AF) {
		pTable->para_list[1] = 0x10;
		pTable->para_list[2] = 0x00;
		pTable->para_list[3] = (bl_level >> 8) & 0xFF;
		pTable->para_list[4] = bl_level & 0xFF;
		pTable->para_list[5] = 0;
	} else {
		alpha = lhbm_alpha[lhbm_alpha_index];
		pTable->para_list[1] = 0x10;
		pTable->para_list[2] = 0x00;
		pTable->para_list[3] = (alpha >> 8) & 0xFF;
		pTable->para_list[4] = alpha & 0xFF;
		pTable->para_list[5] = 0;
	}

	pr_info("%s: backlight 0x%x, 0x%x,0x%x,0x%x,0x%x,0x%x\n", __func__, bl_level, pTable->para_list[1],pTable->para_list[2], pTable->para_list[3],pTable->para_list[4],pTable->para_list[5]);
}

static int panel_lhbm_set_cmdq(void *dsi, dcs_grp_write_gce cb, void *handle, uint32_t on, uint32_t bl_level, uint32_t fps)
{
	struct mtk_panel_para_table *pTable = NULL;
	unsigned int para_count = 0;

	if (on)
	{
		para_count = sizeof(panel_lhbm_on) / sizeof(struct mtk_panel_para_table);
		pTable = panel_lhbm_on;
		set_lhbm_alpha(bl_level);
	}
	else
	{
		para_count = sizeof(panel_lhbm_off) / sizeof(struct mtk_panel_para_table);
		pTable = panel_lhbm_off;
		if (bl_level > APL_THRESHOLD)
		{
			pTable->para_list[1] = (APL_THRESHOLD >> 8) & 0xFF;
			pTable->para_list[2] = APL_THRESHOLD & 0xFF;
		}
		else
		{
			pTable->para_list[1] = (bl_level >> 8) & 0xFF;
			pTable->para_list[2] = bl_level & 0xFF;
		}
		pr_info("%s restore bl to %u ", __func__, bl_level);
	}
	cb(dsi, handle, pTable, para_count);
	return 0;
}


static int panel_hbm_set_cmdq(struct lcm *ctx, void *dsi, dcs_grp_write_gce cb, void *handle, uint32_t hbm_state)
{
	struct mtk_panel_para_table hbm_on_table = {3, {0x51, 0x0F, 0xFF}};
	unsigned int level = 0;
	unsigned int fps = 120;
	fps = atomic_read(&ctx->current_fps);
	level = atomic_read(&ctx->current_bl);

	if (hbm_state > 2) return -1;

	pr_info("%s current_fps = %d hbm_state %d", __func__, atomic_read(&ctx->current_fps), hbm_state);

	switch (hbm_state)
	{
		case 0:
			if (ctx->lhbm_en){
				panel_lhbm_set_cmdq(dsi, cb, handle, 0, level, fps);
			}
			break;
		case 1:
			if (ctx->lhbm_en) {
				panel_lhbm_set_cmdq(dsi, cb, handle, 0, level, fps);
			} else {
				cb(dsi, handle, &hbm_on_table, 1);
			}
			break;
		case 2:
			if (ctx->lhbm_en){
				panel_lhbm_set_cmdq(dsi, cb, handle, 1, level, fps);
			} else {
				cb(dsi, handle, &hbm_on_table, 1);
			}
			break;
		default:
			break;
	}

	atomic_set(&ctx->hbm_mode, hbm_state);
	return 0;
}

static struct mtk_panel_para_table panel_dc_off[] = {
	{2, {0x5E, 0x00}},
};

static struct mtk_panel_para_table panel_dc_on[] = {
	{2, {0x5E, 0x01}},
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
	return 0;
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

static int panel_ext_init_power(struct drm_panel *panel)
{
	int ret;
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_LOW);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	ret = gate_ic_Power_on(panel, 1);
	return ret;
}

static int panel_ext_powerdown(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("%s+\n", __func__);
	if (ctx->prepared)
	    return 0;

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_LOW);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(1000, 1001);
	gate_ic_Power_on(panel, 0);

	return 0;
}

#if 0
static char aod_area_cmd[] ={0x2B, 0x00, 0x00, 0x05, 0x87};

static int panel_doze_area(struct drm_panel *panel,
	void *dsi, dcs_write_gce cb, void *handle)
{
	struct lcm *ctx = panel_to_lcm(panel);
	unsigned int current_y_start = AOD_Y_START_MIN;
	unsigned int mini_step = AOD_Y_START_STEP_MIN;
	unsigned int max_y_start = AOD_Y_START_MAX;

	if (atomic_read(&ctx->doze_enable)) {
		current_y_start = atomic_read(&ctx->current_aod_y_start);
		if (current_y_start < max_y_start) current_y_start += mini_step;
		else current_y_start = AOD_Y_START_MIN;

		aod_area_cmd[1] = (current_y_start >> 8) & 0xFF;
		aod_area_cmd[2] = current_y_start& 0xFF;
		aod_area_cmd[3] = ((current_y_start + AOD_AREA_IZE - 1) >> 8) & 0xFF;
		aod_area_cmd[4] = (current_y_start + AOD_AREA_IZE - 1) & 0xFF;

		pr_info("%s: doze_enable %d current_y_start %d\n", __func__, atomic_read(&ctx->doze_enable), current_y_start);
		pr_info("%s-2B: %d -> %d\n", __func__, (aod_area_cmd[1]*256 + aod_area_cmd[2]),
				(aod_area_cmd[3]*256 + aod_area_cmd[4]));

		cb(dsi, handle, aod_area_cmd, ARRAY_SIZE(aod_area_cmd));

		atomic_set(&ctx->current_aod_y_start, current_y_start);
	} else {
		pr_info("%s: skip update doze area because of  doze_enable = %d\n", __func__, atomic_read(&ctx->doze_enable));
	}

	atomic_set(&ctx->doze_enable, 1);
	return 0;
}
#endif

static struct mtk_panel_para_table aod_en_start_cmd[] = {
		{1, {0x28}},
		{2, {0x6F, 0x02}},
		{1, {0x39}},
	};

static int panel_doze_enable_start(struct drm_panel *panel, void *dsi, dcs_write_gce cb,
	void *handle)
{
	unsigned int para_count = 0;
	struct mtk_panel_para_table *pTable;
	unsigned int i = 0;

	if (!cb)
		return -1;

	para_count = sizeof(aod_en_start_cmd) / sizeof(struct mtk_panel_para_table);
	pr_info("%s: para_count %d\n", __func__, para_count);

	for(i = 0; i < para_count; i++) {
		pTable = &aod_en_start_cmd[i];
		pr_info("%s: para: 0x%x , count %d\n", __func__, pTable->para_list[0], pTable->count);
		cb(dsi, handle, pTable->para_list, pTable->count);
	}

	return 0;
}

static struct mtk_panel_para_table aod_en_cmd[] = {
		{1, {0x29}},
	};

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb,
	void *handle)
{
	struct lcm *ctx = panel_to_lcm(panel);
	unsigned int para_count = 0;
	struct mtk_panel_para_table *pTable;
	unsigned int i = 0;

	pr_info("%s: %d -> %d\n", __func__, atomic_read(&ctx->doze_enable), 1);

	if (!cb)
		return -1;

	para_count = sizeof(aod_en_cmd) / sizeof(struct mtk_panel_para_table);

	pr_info("%s: para_count %d\n", __func__, para_count);

	for(i = 0; i < para_count; i++) {
		pTable = &aod_en_cmd[i];
		pr_info("%s: para: 0x%x , count %d\n", __func__, pTable->para_list[0], pTable->count);
		cb(dsi, handle, pTable->para_list, pTable->count);
	}

	atomic_set(&ctx->doze_enable, 1);

	return 0;
}

static struct mtk_panel_para_table aod_disable_cmd[] = {
		{1, {0x38}},
		{2, {0x6f, 0x01}},
		//{5, {0x2B, 0x00, 0x00, 0x05, 0x87}},
	};

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb,
	void *handle)
{
	struct lcm *ctx = panel_to_lcm(panel);
	unsigned int para_count = 0;
	struct mtk_panel_para_table *pTable;
	unsigned int i = 0;

	pr_info("%s: %d -> %d\n", __func__, atomic_read(&ctx->doze_enable), 0);

	if (!cb)
		return -1;

	para_count = sizeof(aod_disable_cmd) / sizeof(struct mtk_panel_para_table);

	pr_info("%s: para_count %d\n", __func__, para_count);

	for(i = 0; i < para_count; i++) {
		pTable = &aod_disable_cmd[i];
		pr_info("%s: para: 0x%x , count %d\n", __func__, pTable->para_list[0], pTable->count);
		cb(dsi, handle, pTable->para_list, pTable->count);
		if (i==0) usleep_range(50 * 1000, 51 * 1000);
	}

	atomic_set(&ctx->doze_enable, 0);
	//atomic_set(&ctx->current_aod_y_start, AOD_Y_START_MIN);

	usleep_range(40 * 1000, 41 * 1000);

	return 0;
}

#if defined(DIC_COMMAND_MODE_AOD)
static unsigned long panel_doze_get_mode_flags(struct drm_panel *panel,
	int doze_en)
{
	unsigned long mode_flags;

	if (doze_en) {
		mode_flags = MIPI_DSI_MODE_LPM
		       | MIPI_DSI_MODE_NO_EOT_PACKET
		       | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	} else {
		mode_flags = MIPI_DSI_MODE_VIDEO
		       | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
		       | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET
		       | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	}
	pr_info("%s: mode_flags %ld\n", __func__, mode_flags);
	return mode_flags;
}
#endif

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.init_power = panel_ext_init_power,
	.power_down = panel_ext_powerdown,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	//.mode_switch = mode_switch,
	.panel_feature_set = panel_feature_set,
	.panel_feature_get = panel_feature_get,
	.scaling_mode_mapping = mtk_scaling_mode_mapping,
#if defined(DIC_COMMAND_MODE_AOD)
	.doze_get_mode_flags = panel_doze_get_mode_flags,
#endif
	.doze_enable_start = panel_doze_enable_start,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	//.doze_area = panel_doze_area,
};
#endif

static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
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

	/*
	mode4 = drm_mode_duplicate(connector->dev, &mode_48hz);
	if (!mode4) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 mode_48hz.hdisplay, mode_48hz.vdisplay,
			 drm_mode_vrefresh(&mode_48hz));
		return -ENOMEM;
	}

	drm_mode_set_name(mode4);
	mode4->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode4);
	*/

	connector->display_info.width_mm = 70;
	connector->display_info.height_mm = 154;

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
//	unsigned int value;
	int ret;

	pr_info("%s+ lcm,boe,vtdr6126a,vdo,667\n", __func__);

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
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET |
			MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params_120hz, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	atomic_set(&ctx->hbm_mode, 0);
	atomic_set(&ctx->dc_mode, 0);
	atomic_set(&ctx->apl_mode, 0);
	atomic_set(&ctx->current_fps, 120);

	ctx->lhbm_en = 1;
	pr_info("%s- lcm,boe,vtdr6126a,vdo,667\n", __func__);

	return ret;
}

static void lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

}

static const struct of_device_id lcm_of_match[] = {
	{
	    .compatible = "boe,vtdr6126a,vdo,667",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "boe_vtdr6126a_667_1220_2712",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("shaohua deng <shaohua.deng@mediatek.com>");
MODULE_DESCRIPTION("BOE VTDR6126A VDO 120HZ OLED Panel Driver");
MODULE_LICENSE("GPL v2");
