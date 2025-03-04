/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __FBT_CPU_PLATFORM_H__
#define __FBT_CPU_PLATFORM_H__

#include "fbt_cpu.h"

enum FPSGO_CPU_PREFER {
	FPSGO_PREFER_NONE = 0,
	FPSGO_PREFER_BIG = 1,
	FPSGO_PREFER_LITTLE = 2,
	FPSGO_PREFER_L_M = 3,
	FPSGO_PREFER_B_M = 4,
	FPSGO_PREFER_M = 5,
	FPSGO_PREFER_TOTAL,
};

/* DO NOT change the value */
enum FPSGO_CPU_LIMIT {
	FPSGO_LIMIT_NO_LIMIT = 0,
	FPSGO_LIMIT_FREQ = 1,
	FPSGO_LIMIT_CPU = 2,
};

extern void cm_mgr_perf_set_status(int enable);
extern void set_task_ls_prefer_cpus(int pid, unsigned int cpumask_val);
extern void unset_task_ls_prefer_cpus(int pid);
extern void set_task_ls(int pid);
extern void unset_task_ls(int pid);
extern bool is_task_latency_sensitive(struct task_struct *p);
extern void set_task_basic_vip(int pid);
extern void unset_task_basic_vip(int pid);
extern void set_task_vvip(int pid);
extern void unset_task_vvip(int pid);
extern void turn_on_vvip_balance_overutilized(void);
extern void turn_off_vvip_balance_overutilized(void);



void fbt_set_boost_value(unsigned int base_blc);
void fbt_clear_boost_value(void);
void fbt_set_per_task_cap(int pid, unsigned int min_blc,
	unsigned int max_blc, unsigned int max_util);
int fbt_get_L_min_ceiling(void);
void fbt_notify_CM_limit(int reach_limit);
void fbt_reg_dram_request(int reg);
void fbt_boost_dram(int boost);
int fbt_get_default_boost_ta(void);
int fbt_get_default_adj_loading(void);
int fbt_get_default_adj_count(void);
int fbt_get_default_adj_tdiff(void);
int fbt_set_affinity(pid_t pid, unsigned int prefer_type);
int fbt_check_ls(int pid);
int fbt_check_vip(int pid);
int fbt_check_vvip(int pid);
int fbt_set_soft_affinity(int pid, int set, unsigned int prefer_type);
struct cpumask fbt_generate_user_cpu_mask(int mask_int);
int fbt_get_cluster_limit(int *cluster, int *freq, int *r_freq, int *cpu);
int fbt_get_default_qr_enable(void);
int fbt_get_default_gcc_enable(void);
int fbt_get_default_sbe_rescue_enable(void);
int fbt_get_ux_scroll_policy_type(void);
int fbt_get_l_min_bhropp(void);
void init_fbt_platform(void);
void exit_fbt_platform(void);

#endif
