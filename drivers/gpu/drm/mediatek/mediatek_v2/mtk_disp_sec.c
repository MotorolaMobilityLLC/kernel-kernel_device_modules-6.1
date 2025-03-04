// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/module.h>
#include <linux/soc/mediatek/mtk-cmdq-ext.h>
#include <linux/of_platform.h>

#include <drm/drm_crtc.h>
#include "mtk_drm_crtc.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_drv.h"
#include "mtk_log.h"
#include "mtk_dump.h"
#include "mtk_heap.h"

#include "cmdq-sec.h"
#include "cmdq-sec-iwc-common.h"
#include "cmdq-sec-mailbox.h"

#define RETRY_SEC_CMDQ_FLUSH 3

struct mtk_disp_sec_config {
	struct cmdq_client *disp_sec_client;
	struct cmdq_pkt *sec_cmdq_handle;
	u32 tzmp_disp_sec_wait;
	u32 tzmp_disp_sec_set;
};

struct mtk_disp_sec_data {
	u32 legacy_dts_check;
	u32 with_larb_ctrl;
};


struct mtk_disp_sec_config sec_config;
static bool sec_client_already_stop;

static void mtk_disp_secure_cb(struct cmdq_cb_data data)
{
	if (!sec_config.sec_cmdq_handle) {
		DDPPR_ERR("%s: secure sec_cmdq_handle already stopped\n", __func__);
		return;
	}
	DDPINFO("%s err:%d\n", __func__, data.err);

	cmdq_pkt_destroy(sec_config.sec_cmdq_handle);
	sec_config.sec_cmdq_handle = NULL;
}

int mtk_disp_cmdq_secure_init(void)
{
	if (!sec_config.disp_sec_client) {
		DDPINFO("%s:%d, sec_config.disp_sec_client is NULL\n",
			__func__, __LINE__);
		return false;
	}

	if (sec_config.sec_cmdq_handle) {
		DDPMSG("%s cmdq_handle is already exist\n", __func__);
		return false;
	}
	if (!sec_client_already_stop)
		sec_client_already_stop = true;

	sec_config.sec_cmdq_handle =
		cmdq_pkt_create(sec_config.disp_sec_client);

	cmdq_sec_pkt_set_data(sec_config.sec_cmdq_handle, 0, 0,
		CMDQ_SEC_DEBUG, CMDQ_METAEX_TZMP);
	cmdq_sec_pkt_set_mtee(sec_config.sec_cmdq_handle, true);
	cmdq_pkt_finalize_loop(sec_config.sec_cmdq_handle);
	if (cmdq_pkt_flush_threaded(sec_config.sec_cmdq_handle,
		mtk_disp_secure_cb, NULL) < 0) {
		DDPPR_ERR("Failed to flush user_cmd\n");
		return false;
	}
	return true;
}

int mtk_disp_cmdq_secure_end(void)
{
	if (!sec_config.sec_cmdq_handle) {
		DDPPR_ERR("%s cmdq_handle is NULL\n", __func__);
		return false;
	}

	if (!sec_config.disp_sec_client) {
		DDPINFO("%s:%d, sec_config.disp_sec_client is NULL\n",
			__func__, __LINE__);
		return false;
	}

	if (sec_client_already_stop)
		sec_client_already_stop = false;
	else
		return true;

	cmdq_sec_mbox_stop(sec_config.disp_sec_client);
	return true;
}

int mtk_disp_secure_domain_enable(struct cmdq_pkt *handle,
									resource_size_t dummy_larb)
{
	int retry_count = 0;

	DDPINFO("%s+\n", __func__);
	while (!sec_config.sec_cmdq_handle) {
		retry_count++;
		DDPMSG("%s: no secure sec_cmdq_handle, retry %d\n"
				, __func__, retry_count);
		mtk_disp_cmdq_secure_init();
		if (retry_count > RETRY_SEC_CMDQ_FLUSH) {
			DDPMSG("%s: too many times init fail\n", __func__);
			return false;
		}
	}

	cmdq_pkt_write(handle, NULL, dummy_larb, BIT(1), BIT(1));
	cmdq_pkt_set_event(handle, sec_config.tzmp_disp_sec_wait);
	cmdq_pkt_wfe(handle, sec_config.tzmp_disp_sec_set);

	return true;
}

int mtk_disp_secure_domain_disable(struct cmdq_pkt *handle,
									resource_size_t dummy_larb)
{
	int retry_count = 0;

	DDPINFO("%s+\n", __func__);
	while (!sec_config.sec_cmdq_handle) {
		retry_count++;
		DDPMSG("%s: no secure sec_cmdq_handle, retry %d\n"
				, __func__, retry_count);
		mtk_disp_cmdq_secure_init();
		if (retry_count > RETRY_SEC_CMDQ_FLUSH) {
			DDPMSG("%s: too many times init fail\n", __func__);
			return false;
		}
	}

	cmdq_pkt_write(handle, NULL, dummy_larb, 0, BIT(1));
	cmdq_pkt_set_event(handle, sec_config.tzmp_disp_sec_wait);
	cmdq_pkt_wfe(handle, sec_config.tzmp_disp_sec_set);

	return true;
}

int mtk_disp_secure_get_dma_sec_status(struct dma_buf *buf_hnd)
{
	int sec_check = 0;

	sec_check = is_mtk_sec_heap_dmabuf(buf_hnd);
	DDPINFO("%s is %d\n", __func__, sec_check);
	return sec_check;
}
static int mtk_drm_disp_sec_cb_event(int value, struct cmdq_pkt *handle,
									resource_size_t dummy_larb,
									struct dma_buf *buf_hnd)
{
	DDPINFO("%s+\n", __func__);
	switch (value) {
	case DISP_SEC_START:
		return mtk_disp_cmdq_secure_init();
	case DISP_SEC_STOP:
		return mtk_disp_cmdq_secure_end();
	case DISP_SEC_ENABLE:
		return mtk_disp_secure_domain_enable(handle, dummy_larb);
	case DISP_SEC_DISABLE:
		return mtk_disp_secure_domain_disable(handle, dummy_larb);
	case DISP_SEC_CHECK:
		return mtk_disp_secure_get_dma_sec_status(buf_hnd);
	default:
		return false;
	}
}


static int disp_sec_probe(struct platform_device *pdev)
{
	void **ret;
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	const struct mtk_disp_sec_data *data;

	DDPINFO("%s+\n", __func__);

	data = of_device_get_match_data(dev);

	if (data->with_larb_ctrl) {
		sec_config.disp_sec_client = cmdq_mbox_create(dev, 0);
		if (sec_config.disp_sec_client) {
			if (data->legacy_dts_check) {
				of_property_read_u32(node, "sw_sync_token_tzmp_disp_wait",
							&sec_config.tzmp_disp_sec_wait);
				of_property_read_u32(node, "sw_sync_token_tzmp_disp_set",
							&sec_config.tzmp_disp_sec_set);
			} else {
				of_property_read_u32(node, "sw-sync-token-tzmp-disp-wait",
							&sec_config.tzmp_disp_sec_wait);
				of_property_read_u32(node, "sw-sync-token-tzmp-disp-set",
							&sec_config.tzmp_disp_sec_set);
			}
			DDPMSG("tzmp_disp_sec_wait %d tzmp_disp_sec_set %d\n",
						sec_config.tzmp_disp_sec_wait,
						sec_config.tzmp_disp_sec_set);
		}
	}
	ret = mtk_drm_disp_sec_cb_init();
	*ret = (void *) mtk_drm_disp_sec_cb_event;
	DDPINFO("%s-\n", __func__);
	return 0;
}

static int disp_sec_remove(struct platform_device *pdev)
{
	DDPINFO("%s+\n", __func__);
	return 0;
}

static const struct mtk_disp_sec_data legacy_sec_driver_data = {
	.legacy_dts_check = 1,
	.with_larb_ctrl = 1,
};

static const struct mtk_disp_sec_data current_sec_driver_data = {
	.legacy_dts_check = 0,
	.with_larb_ctrl = 1,
};

static const struct mtk_disp_sec_data current_sec_driver_data_aidctl = {
	.legacy_dts_check = 0,
	.with_larb_ctrl = 0,
};


static const struct of_device_id of_disp_sec_match_tbl[] = {
	{
		.compatible = "mediatek,disp_sec",
		.data = &legacy_sec_driver_data,
	},
	{
		.compatible = "mediatek,disp-sec",
		.data = &current_sec_driver_data,
	},
	{
		.compatible = "mediatek,disp-sec-aidctl",
		.data = &current_sec_driver_data_aidctl,
	},
	{}
};

static struct platform_driver disp_sec_drv = {
	.probe = disp_sec_probe,
	.remove = disp_sec_remove,
	.driver = {
		.name = "mtk_disp_sec",
		.of_match_table = of_disp_sec_match_tbl,
	},
};

static int __init mtk_disp_sec_init(void)
{
	s32 status;

	status = platform_driver_register(&disp_sec_drv);
	if (status) {
		DDPMSG("Failed to register disp sec driver(%d)\n", status);
		return -ENODEV;
	}
	return 0;
}

static void __exit mtk_disp_sec_exit(void)
{
	platform_driver_unregister(&disp_sec_drv);
}

module_init(mtk_disp_sec_init);
module_exit(mtk_disp_sec_exit);

MODULE_AUTHOR("Aaron Chung <Aaron.Chung@mediatek.com>");
MODULE_DESCRIPTION("MTK DRM secure Display");
MODULE_LICENSE("GPL v2");

