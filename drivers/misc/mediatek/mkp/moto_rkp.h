/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 2023 Motorola Mobility, Inc.
 */

#ifndef __MOTO_RKP_H
#define __MOTO_RKP_H

#include "debug.h"

#define MAX_CACHED_NUM	4	// shall be the exponential of 2
#define CACHED_NUM_MASK	(MAX_CACHED_NUM - 1)
#define MOTO_RKP_SHIFT_PAGE_BIT  12
#define MOTO_RKP_LOCKED_FLAG_SIZE 3

struct avc_sbuf_content {
	unsigned long avc_node;
	u32 ssid __aligned(8);
	u32 tsid __aligned(8);
	u16 tclass __aligned(8);
	u32 ae_allowed __aligned(8);
} __aligned(8);

struct _cred_sbuf_content {
	kuid_t uid;
	kgid_t gid;
	kuid_t euid;
	kgid_t egid;
	kuid_t fsuid;
	kgid_t fsgid;
	void *security;
};

struct cred_sbuf_content {
	union {
		struct _cred_sbuf_content csc;
		unsigned long args[4];
	};
};

struct avc_sbuf_cache {
	unsigned long cached[MAX_CACHED_NUM];
	int cached_index[MAX_CACHED_NUM];
	int pos;
};

extern struct rb_root mkp_rbtree;
extern rwlock_t mkp_rbtree_rwlock;
int __init moto_rkp_init(void);
#endif
