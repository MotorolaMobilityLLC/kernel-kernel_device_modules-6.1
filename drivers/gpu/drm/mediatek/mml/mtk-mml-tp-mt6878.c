// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Dennis-YC Hsieh <dennis-yc.hsieh@mediatek.com>
 */

#include <dt-bindings/mml/mml-mt6878.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>

#include "mtk-mml-drm-adaptor.h"
#include "mtk-mml-color.h"
#include "mtk-mml-core.h"

#define TOPOLOGY_PLATFORM	"mt6878"
#define AAL_MIN_WIDTH		50	/* TODO: define in tile? */
#define MML_OUT_MIN_W		784	/* wqhd 1440/2+64=784 */
#define MML_MIN_SIZE		64

int mml_force_rsz;
module_param(mml_force_rsz, int, 0644);

int mml_path_mode;
module_param(mml_path_mode, int, 0644);

struct path_node {
	u8 eng;
	u8 next0;
	u8 next1;
};

/* !!Following code generate by topology parser (tpparser.py)!!
 * include: topology_scenario, path_map, engine_reset_bit
 */
enum topology_scenario {
	PATH_MML_NOPQ = 0,
	PATH_MML_PQ,
	PATH_MML_MAX
};

static const struct path_node path_map[PATH_MML_MAX][MML_MAX_PATH_NODES] = {
	[PATH_MML_NOPQ] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_RDMA0, MML_WROT0,},
		{MML_WROT0,},
	},
	[PATH_MML_PQ] = {
		{MML_MMLSYS,},
		{MML_MUTEX,},
		{MML_RDMA0, MML_DLI0_SEL,},
		{MML_DLI0_SEL, MML_HDR0,},
		{MML_HDR0, MML_AAL0,},
		{MML_AAL0, MML_RSZ0,},
		{MML_RSZ0, MML_TDSHP0,},
		{MML_TDSHP0, MML_WROT0_SEL,},
		{MML_WROT0_SEL, MML_WROT0,},
		{MML_WROT0,},
	},
};

/* reset bit to each engine,
 * reverse of MMSYS_SW0_RST_B and MMSYS_SW1_RST_B
 */
static u8 engine_reset_bit[MML_ENGINE_TOTAL] = {
	[MML_MUTEX] = 0,
	[MML_RDMA0] = 3,
	[MML_HDR0] = 5,
	[MML_AAL0] = 6,
	[MML_RSZ0] = 7,
	[MML_TDSHP0] = 8,
	[MML_WROT0] = 10,
};
/* !!Above code generate by topology parser (tpparser.py)!! */

static inline bool engine_input(u32 id)
{
	return id == MML_RDMA0;
}

/* check if engine is output dma engine */
static inline bool engine_wrot(u32 id)
{
	return id == MML_WROT0;
}

/* check if engine is dma engine */
static inline bool engine_dma(u32 id)
{
	return id == MML_RDMA0 || id == MML_WROT0;
}

static inline bool engine_tdshp(u32 id)
{
	return id == MML_TDSHP0;
}

enum cmdq_clt_usage {
	MML_CLT_PIPE0,
	MML_CLT_PIPE1,
	MML_CLT_MAX
};

static const u8 clt_dispatch[PATH_MML_MAX] = {
	[PATH_MML_NOPQ] = MML_CLT_PIPE0,
	[PATH_MML_PQ] = MML_CLT_PIPE0,
};

/* mux sof group of mmlsys mout/sel */
enum mux_sof_group {
	MUX_SOF_GRP0 = 0,
	MUX_SOF_GRP1,
	MUX_SOF_GRP2,
	MUX_SOF_GRP3,
	MUX_SOF_GRP4,
	MUX_SOF_GRP5,
	MUX_SOF_GRP6,
	MUX_SOF_GRP7,
};

static const u8 grp_dispatch[PATH_MML_MAX] = {
	[PATH_MML_NOPQ] = MUX_SOF_GRP1,
	[PATH_MML_PQ] = MUX_SOF_GRP1,
};

static void tp_dump_path(const struct mml_topology_path *path)
{
	u8 i;

	for (i = 0; i < path->node_cnt; i++) {
		mml_log(
			"[topology]%u engine %u (%p) prev %p %p next %p %p comp %p tile idx %u out %u",
			i, path->nodes[i].id, &path->nodes[i],
			path->nodes[i].prev[0], path->nodes[i].prev[1],
			path->nodes[i].next[0], path->nodes[i].next[1],
			path->nodes[i].comp,
			path->nodes[i].tile_eng_idx,
			path->nodes[i].out_idx);
	}
}

static void tp_dump_path_short(struct mml_topology_path *path)
{
	char path_desc[64];
	u32 len = 0;
	u8 i;

	for (i = 0; i < path->node_cnt; i++)
		len += snprintf(path_desc + len, sizeof(path_desc) - len, " %u",
			path->nodes[i].id);
	mml_log("[topology]engines:%s", path_desc);
}

static void tp_parse_connect_prev(const struct path_node *route, struct mml_path_node *nodes,
	u8 cur_idx)
{
	u32 i;
	u32 in_idx = 0;	/* current engine input index */
	u32 eng_id = nodes[cur_idx].id;

	for (i = 0; i < cur_idx && in_idx < 2; i++) {
		u32 prev_out_idx;	/* previous engine output index */

		if (route[i].next0 == eng_id)
			prev_out_idx = 0;
		else if (route[i].next1 == eng_id)
			prev_out_idx = 1;
		else
			continue;

		nodes[i].next[prev_out_idx] = &nodes[cur_idx];
		nodes[cur_idx].prev[in_idx++] = &nodes[i];

		if (nodes[i].out_idx || prev_out_idx)
			nodes[cur_idx].out_idx = 1;
	}

	if (!in_idx && !engine_input(eng_id))
		mml_err("[topology]connect fail idx:%u engine:%u", i, eng_id);
}

static void tp_parse_path(struct mml_dev *mml, struct mml_topology_path *path,
	const struct path_node *route)
{
	u8 i, tile_idx, out_eng_idx;

	for (i = 0; i < MML_MAX_PATH_NODES; i++) {
		const u8 eng = route[i].eng;

		if (!route[i].eng) {
			path->node_cnt = i;
			break;
		}

		/* assign current engine */
		path->nodes[i].id = eng;
		path->nodes[i].comp = mml_dev_get_comp_by_id(mml, eng);
		if (!path->nodes[i].comp)
			mml_err("[topology]no comp idx:%u engine:%u", i, eng);

		/* assign reset bits for this path */
		path->reset_bits |= 1LL << engine_reset_bit[eng];

		if (eng == MML_MMLSYS) {
			path->mmlsys = path->nodes[i].comp;
			path->mmlsys_idx = i;
			continue;
		} else if (eng == MML_MUTEX) {
			path->mutex = path->nodes[i].comp;
			path->mutex_idx = i;
			continue;
		}

		/* find and connect previous engine to current node */
		tp_parse_connect_prev(route, path->nodes, i);

		/* for svp aid binding */
		if (engine_dma(eng) && path->aid_eng_cnt < MML_MAX_AID_COMPS)
			path->aid_engine_ids[path->aid_eng_cnt++] = eng;
	}
	path->node_cnt = i;

	/* 0: reset
	 * 1: not reset
	 * so we need to reverse the bits
	 */
	path->reset_bits = ~path->reset_bits;
	mml_msg("[topology]reset bits %#llx", path->reset_bits);

	/* collect tile engines */
	tile_idx = 0;
	for (i = 0; i < path->node_cnt; i++) {
		if ((!path->nodes[i].prev[0] && !path->nodes[i].next[0])) {
			path->nodes[i].tile_eng_idx = ~0;
			continue;
		}
		path->nodes[i].tile_eng_idx = tile_idx;
		path->tile_engines[tile_idx++] = i;
	}
	path->tile_engine_cnt = tile_idx;

	/* scan out engines */
	for (i = 0; i < path->node_cnt; i++) {
		if (!engine_wrot(path->nodes[i].id))
			continue;
		out_eng_idx = path->nodes[i].out_idx;
		if (path->out_engine_ids[out_eng_idx])
			mml_err("[topology]multiple out engines: was %u now %u on out idx:%u",
				path->out_engine_ids[out_eng_idx],
				path->nodes[i].id, out_eng_idx);
		path->out_engine_ids[out_eng_idx] = path->nodes[i].id;
	}

	if (path->tile_engine_cnt == 2)
		path->alpharot = true;
}

static s32 tp_init_cache(struct mml_dev *mml, struct mml_topology_cache *cache,
	struct cmdq_client **clts, u32 clt_cnt)
{
	u32 i;

	if (clt_cnt < MML_CLT_MAX) {
		mml_err("[topology]%s not enough cmdq clients to all paths",
			__func__);
		return -ECHILD;
	}
	if (ARRAY_SIZE(cache->paths) < PATH_MML_MAX) {
		mml_err("[topology]%s not enough path cache for all paths",
			__func__);
		return -ECHILD;
	}

	for (i = 0; i < PATH_MML_MAX; i++) {
		struct mml_topology_path *path = &cache->paths[i];

		tp_parse_path(mml, path, path_map[i]);
		if (mtk_mml_msg) {
			mml_log("[topology]dump path %u count %u clt id %u",
				i, path->node_cnt, clt_dispatch[i]);
			tp_dump_path(path);
		}

		/* now dispatch cmdq client (channel) to path */
		path->clt = clts[clt_dispatch[i]];
		path->clt_id = clt_dispatch[i];
		path->mux_group = grp_dispatch[i];
	}

	return 0;
}

static inline bool tp_need_resize(struct mml_frame_info *info)
{
	u32 w = info->dest[0].data.width;
	u32 h = info->dest[0].data.height;
	u32 cw = info->dest[0].crop.r.width;
	u32 ch = info->dest[0].crop.r.height;

	if (info->dest[0].rotate == MML_ROT_90 ||
		info->dest[0].rotate == MML_ROT_270)
		swap(w, h);

	mml_msg("[topology]%s target %ux%u crop %ux%u",
		__func__, w, h, cw, ch);

	return info->dest_cnt != 1 ||
		cw != w || ch != h ||
		info->dest[0].crop.x_sub_px ||
		info->dest[0].crop.y_sub_px ||
		info->dest[0].crop.w_sub_px ||
		info->dest[0].crop.h_sub_px ||
		info->dest[0].compose.width != info->dest[0].data.width ||
		info->dest[0].compose.height != info->dest[0].data.height;
}

static void tp_select_path(struct mml_topology_cache *cache,
	struct mml_frame_config *cfg,
	struct mml_topology_path **path)
{
	enum topology_scenario scene = 0;
	bool en_rsz, en_pq;

	en_rsz = tp_need_resize(&cfg->info);
	if (mml_force_rsz)
		en_rsz = true;
	en_pq = en_rsz || cfg->info.dest[0].pq_config.en;

	if (!en_pq) {
		/* rdma to wrot */
		scene = PATH_MML_NOPQ;
	} else if (mml_force_rsz == 2) {
		scene = PATH_MML_PQ;
	} else {
		scene = PATH_MML_PQ;
	}

	*path = &cache->paths[scene];
}

static s32 tp_select(struct mml_topology_cache *cache,
	struct mml_frame_config *cfg)
{
	struct mml_topology_path *path = NULL;

	cfg->shadow = false;

	tp_select_path(cache, cfg, &path);

	if (!path)
		return -EPERM;

	cfg->path[0] = path;

	if (path->alpharot) {
		u8 fmt_in = MML_FMT_HW_FORMAT(cfg->info.src.format);
		u8 i;

		cfg->alpharot = MML_FMT_IS_ARGB(fmt_in);
		for (i = 0; i < cfg->info.dest_cnt && cfg->alpharot; i++)
			if (!MML_FMT_IS_ARGB(cfg->info.dest[i].data.format))
				cfg->alpharot = false;
	}

	tp_dump_path_short(path);

	return 0;
}

static enum mml_mode tp_query_mode(struct mml_dev *mml, struct mml_frame_info *info,
	u32 *reason)
{
	if (unlikely(mml_path_mode))
		return mml_path_mode;

	/* skip all racing mode check if use prefer dc */
	if (info->mode == MML_MODE_MML_DECOUPLE ||
		info->mode == MML_MODE_MDP_DECOUPLE) {
		*reason = mml_query_userdc;
		goto decouple_user;
	}

	if (info->mode == MML_MODE_APUDC) {
		*reason = mml_query_apudc;
		goto decouple_user;
	}

	if (!MML_FMT_COMPRESS(info->src.format)) {
		*reason = mml_query_format;
		return MML_MODE_MML_DECOUPLE;
	}

	return MML_MODE_MML_DECOUPLE;

decouple_user:
	return info->mode;
}


static const struct mml_topology_ops tp_ops_mt6878 = {
	.query_mode = tp_query_mode,
	.init_cache = tp_init_cache,
	.select = tp_select,
};

static __init int mml_topology_ip_init(void)
{
	return mml_topology_register_ip(TOPOLOGY_PLATFORM, &tp_ops_mt6878);
}
module_init(mml_topology_ip_init);

static __exit void mml_topology_ip_exit(void)
{
	mml_topology_unregister_ip(TOPOLOGY_PLATFORM);
}
module_exit(mml_topology_ip_exit);

MODULE_AUTHOR("Dennis-YC Hsieh <dennis-yc.hsieh@mediatek.com>");
MODULE_DESCRIPTION("MediaTek SoC display MML for MT6878");
MODULE_LICENSE("GPL");
