/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Manual GC support
 *
 * Copyright 2020 Google LLC
 *
 * Authors: Jaegeuk Kim <jaegeuk@google.com>
 */

#ifndef _UFS_HID_JEDEC_H_
#define _UFS_HID_JEDEC_H_

#include <asm/unaligned.h>
#include <ufs/ufshcd.h>

/* bDefragOperation attribute values */
enum ufs_hid_defrag_operation {
	HID_ANALYSIS_AND_DEFRAG_DISABLE	= 0,
	HID_ANALYSIS_ENABLE		= 1,
	HID_ANALYSIS_AND_DEFRAG_ENABLE	= 2,
};

/* bHIDState attribute values */
enum ufs_hid_state {
	HID_IDLE		= 0,
	ANALYSIS_IN_PROGRESS	= 1,
	DEFRAG_REQUIRED		= 2,
	DEFRAG_IN_PROGRESS	= 3,
	DEFRAG_COMPLETED	= 4,
	DEFRAG_NOT_REQUIRED	= 5,
	NUM_UFS_HID_STATES	= 6,
};

static const char * const ufs_hid_states[] = {
	[HID_IDLE]		= "idle",
	[ANALYSIS_IN_PROGRESS]	= "analysis_in_progress",
	[DEFRAG_REQUIRED]	= "defrag_required",
	[DEFRAG_IN_PROGRESS]	= "defrag_in_progress",
	[DEFRAG_COMPLETED]	= "defrag_completed",
	[DEFRAG_NOT_REQUIRED]	= "defrag_not_required",
};

#define QUERY_ATTR_IDN_HID_DEFRAG_OPERATION	 0x35
#define QUERY_ATTR_IDN_HID_AVAILABLE_SIZE	 0x36
#define QUERY_ATTR_IDN_HID_SIZE			 0x37
#define QUERY_ATTR_IDN_HID_PROGRESS_RATIO	 0x38
#define QUERY_ATTR_IDN_HID_STATE		 0x39

#define UFS_DEV_HID_SUPPORT		BIT(13)

struct ufs_hid_jedec_feature {
	struct ufs_hba *hba;
	bool hid_sup;
};

extern void ufs_sysfs_update_hid(struct ufs_hba *hba);
extern int hid_jedec_init(struct ufs_hba *hba);
#endif
