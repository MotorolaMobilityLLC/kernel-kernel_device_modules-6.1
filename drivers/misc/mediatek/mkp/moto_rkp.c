// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 Motorola Mobility, Inc.
 */

#undef pr_fmt
#define pr_fmt(fmt) "MotoRKP: " fmt

#include <trace/hooks/vendor_hooks.h>
#include <trace/hooks/avc.h>
#include <trace/hooks/creds.h>
#include <trace/hooks/module.h>
#include <trace/hooks/selinux.h>
#include <trace/hooks/syscall_check.h>
#include <linux/types.h> // for list_head
#include <linux/module.h> // module_layout
#include <linux/init.h> // rodata_enable support
#include <linux/mutex.h>
#include <linux/kernel.h> // round_up
#include <linux/reboot.h>
#include <linux/workqueue.h>
#include <linux/tracepoint.h>
#include <linux/of.h>
#include <linux/libfdt.h> // fdt32_ld
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>

#include "selinux/mkp_security.h"
#include "selinux/mkp_policycap.h"

#include "moto_rkp.h"

#include "mkp.h"
#include "trace_mkp.h"
#define CREATE_TRACE_POINTS
#include "trace_mtk_mkp.h"

DEBUG_SET_LEVEL(DEBUG_LEVEL_ERR);

struct work_struct *avc_work;

static uint32_t g_ro_avc_handle __ro_after_init;
static struct page *avc_pages __ro_after_init;
int avc_array_sz __ro_after_init;
int rem;
static bool g_initialized;
static struct selinux_avc *g_avc;
static struct selinux_policy __rcu *g_policy;
const struct selinux_state *g_selinux_state;

static DEFINE_PER_CPU(struct avc_sbuf_cache, cpu_avc_sbuf);


struct rb_root mkp_rbtree = RB_ROOT;
DEFINE_RWLOCK(mkp_rbtree_rwlock);

#if !IS_ENABLED(CONFIG_KASAN_GENERIC) && !IS_ENABLED(CONFIG_KASAN_SW_TAGS)
#if !IS_ENABLED(CONFIG_GCOV_KERNEL)
static void *p_stext;
static void *p_etext;
static void *p__init_begin;
#endif
#endif

int mkp_hook_trace_on;
module_param(mkp_hook_trace_on, int, 0600);

bool mkp_hook_trace_enabled(void)
{
    return !!mkp_hook_trace_on;
}

static void set_memory_rw(unsigned long addr, int nr_pages)
{
	int ret;
	bool valid_addr = false;

	if ((unsigned long)THIS_MODULE->init_layout.base == addr)
		return;
	valid_addr = !!(is_vmalloc_or_module_addr((void *)addr));
	if (valid_addr) {
		ret = mkp_set_mapping_xxx_helper(addr, nr_pages, MKP_POLICY_DRV,
			HELPER_MAPPING_RW);
	} else
		MKP_WARN("addr is not a module or vmalloc address\n");
}

static void set_memory_nx(unsigned long addr, int nr_pages)
{
	int ret;
	bool valid_addr = false;
	int i = 0;
	unsigned long pfn;
	struct mkp_rb_node *found = NULL;
	phys_addr_t phys_addr;
	uint32_t policy;
	unsigned long flags;

	if ((unsigned long)THIS_MODULE->init_layout.base == addr)
		return;

	valid_addr = !!(is_vmalloc_or_module_addr((void *)addr));
	if (valid_addr) {
		ret = mkp_set_mapping_xxx_helper(addr, nr_pages, MKP_POLICY_DRV,
			HELPER_MAPPING_NX);
		policy = MKP_POLICY_DRV;
	} else {
		MKP_WARN("addr is not a module or vmalloc address\n");
		return;
	}

	for (i = 0; i < nr_pages; i++) {
		pfn = vmalloc_to_pfn((void *)(addr+i*PAGE_SIZE));
		phys_addr = pfn << PAGE_SHIFT;
		write_lock_irqsave(&mkp_rbtree_rwlock, flags);
		found = mkp_rbtree_search(&mkp_rbtree, phys_addr);
		if (found != NULL && found->addr != 0 && found->size != 0) {
			ret = mkp_destroy_handle(policy, found->handle);
			ret = mkp_rbtree_erase(&mkp_rbtree, phys_addr);
		}
		write_unlock_irqrestore(&mkp_rbtree_rwlock, flags);
	}
}

static void probe_android_rvh_set_module_core_rw_nx(void *ignore,
		const struct module *mod)
{
	set_memory_rw((unsigned long)mod->core_layout.base, (mod->core_layout.size) >> PAGE_SHIFT);
	set_memory_nx((unsigned long)mod->core_layout.base, (mod->core_layout.size) >> PAGE_SHIFT);
}

static void probe_android_rvh_set_module_init_rw_nx(void *ignore,
		const struct module *mod)
{
	set_memory_rw((unsigned long)mod->init_layout.base, (mod->init_layout.size) >> PAGE_SHIFT);
	set_memory_nx((unsigned long)mod->init_layout.base, (mod->init_layout.size) >> PAGE_SHIFT);
}

/* Remap Kernel .text/.rodata with 2M granuality in EL1S2 */
static bool full_kernel_code_2m = false;

/* Lock MKP once everything has been set done */
static bool mkp_ro_region_is_already_locked = false;

#if !IS_ENABLED(CONFIG_KASAN_GENERIC) && !IS_ENABLED(CONFIG_KASAN_SW_TAGS)
#if !IS_ENABLED(CONFIG_GCOV_KERNEL)
static void mkp_protect_kernel_work_fn(struct work_struct *work);

static DECLARE_DELAYED_WORK(mkp_pk_work, mkp_protect_kernel_work_fn);
static int retry_num = 100;
static void mkp_protect_kernel_work_fn(struct work_struct *work)
{
	int ret = 0;
	uint32_t policy = 0;
	uint32_t handle = 0;
	unsigned long addr_start;
	unsigned long addr_end;
	phys_addr_t phys_addr;
	int nr_pages;

	/* Map all kernel code in the EL1S2 with the granularity of 2M */
	bool kernel_code_perf = false;
	unsigned long addr_start_2m = 0, addr_end_2m = 0;

	if (policy_ctrl[MKP_POLICY_KERNEL_CODE] &&
		policy_ctrl[MKP_POLICY_KERNEL_RODATA]) {
		mkp_get_krn_info(&p_stext, &p_etext, &p__init_begin);
		if (p_stext == NULL || p_etext == NULL || p__init_begin == NULL) {
			pr_info("%s: retry in 0.1 second", __func__);
			if (--retry_num >= 0)
				schedule_delayed_work(&mkp_pk_work, HZ / 10);
			else
				MKP_ERR("protect krn failed\n");
			return;
		}
	}

	if (policy_ctrl[MKP_POLICY_KERNEL_CODE] &&
		policy_ctrl[MKP_POLICY_KERNEL_RODATA]) {
		/* It may ONLY take effects when BOTH KERNEL_CODE & KERNEL_RODATA are enabled */
		if (full_kernel_code_2m)
			kernel_code_perf = true;
	}

	if (policy_ctrl[MKP_POLICY_KERNEL_CODE] != 0) {
		// round down addr before minus operation
		addr_start = (unsigned long)p_stext;
		addr_end = (unsigned long)p_etext;
		addr_start = round_up(addr_start, PAGE_SIZE);
		addr_end = round_down(addr_end, PAGE_SIZE);

		/* Try to round_down/up the boundary in 2M */
		if (kernel_code_perf) {
			addr_start_2m = round_down(addr_start, SZ_2M);
			/* The range size of _text and _stext should SEGMENT_ALIGN */
			if ((addr_start - addr_start_2m) == SEGMENT_ALIGN) {
				addr_start = addr_start_2m;
				addr_end_2m = round_up(addr_end, SZ_2M);
				addr_end = addr_end_2m;
			}
		}
		if (addr_start == 0) {
			MKP_ERR("Cannot find the kernel text\n");
			goto protect_krn_fail;
		}

		nr_pages = (addr_end-addr_start)>>PAGE_SHIFT;
		phys_addr = __pa_symbol((void *)addr_start);
		policy = MKP_POLICY_KERNEL_CODE;
		handle = mkp_create_handle(policy, (unsigned long)phys_addr, nr_pages<<12);
		if (handle == 0) {
			MKP_ERR("%s:%d: Create handle fail\n", __func__, __LINE__);
		} else {
			ret = mkp_set_mapping_x(policy, handle);
			ret = mkp_set_mapping_ro(policy, handle);
			pr_info("mkp: protect krn code done\n");
		}
	}

	if (policy_ctrl[MKP_POLICY_KERNEL_RODATA] != 0) {
		// round down addr before minus operation
		addr_start = (unsigned long)p_etext;
		addr_end = (unsigned long)p__init_begin;
		addr_start = round_up(addr_start, PAGE_SIZE);
		addr_end = round_down(addr_end, PAGE_SIZE);

		/* Try to round_down/up the boundary in 2M */
		if (kernel_code_perf && (addr_end_2m != 0) && (addr_end_2m <= addr_end))
			addr_start = addr_end_2m;
		if (addr_start == 0) {
			MKP_ERR("Cannot find the kernel rodata\n");
			goto protect_krn_fail;
		}

		nr_pages = (addr_end-addr_start)>>PAGE_SHIFT;
		phys_addr = __pa_symbol((void *)addr_start);
		policy = MKP_POLICY_KERNEL_RODATA;
		handle = mkp_create_handle(policy, (unsigned long)phys_addr, nr_pages<<12);
		if (handle == 0)
			MKP_ERR("%s:%d: Create handle fail\n", __func__, __LINE__);
		else {
			ret = mkp_set_mapping_ro(policy, handle);
			pr_info("mkp: protect krn rodata done\n");
		}
	}

	/* Mark kernel ro region is locked */
	mkp_ro_region_is_already_locked = true;

protect_krn_fail:
	p_stext = NULL;
	p_etext = NULL;
	p__init_begin = NULL;
}
#endif
#endif

static void probe_android_rvh_set_module_permit_before_init(void *ignore,
	const struct module *mod)
{
	if (mod == THIS_MODULE && policy_ctrl[MKP_POLICY_MKP] != 0) {
		module_enable_ro(mod, false, MKP_POLICY_MKP);
		module_enable_nx(mod, MKP_POLICY_MKP);
		module_enable_x(mod, MKP_POLICY_MKP);
		return;
	}
	if (mod != THIS_MODULE && policy_ctrl[MKP_POLICY_DRV] != 0) {
		if (drv_skip((char *)mod->name))
			return;
		module_enable_ro(mod, false, MKP_POLICY_DRV);
		module_enable_nx(mod, MKP_POLICY_DRV);
		module_enable_x(mod, MKP_POLICY_DRV);
	}
}

static void probe_android_rvh_set_module_permit_after_init(void *ignore,
	const struct module *mod)
{
	if (mod == THIS_MODULE && policy_ctrl[MKP_POLICY_MKP] != 0) {
		module_enable_ro(mod, true, MKP_POLICY_MKP);
		return;
	}
	if (mod != THIS_MODULE && policy_ctrl[MKP_POLICY_DRV] != 0) {
		if (drv_skip((char *)mod->name))
			return;
		module_enable_ro(mod, true, MKP_POLICY_DRV);
	}
}

static void __update_cpu_avc_sbuf(unsigned long key, int index)
{
	struct avc_sbuf_cache *sb;

	sb = this_cpu_ptr(&cpu_avc_sbuf);
	sb->cached[sb->pos] = key;
	sb->cached_index[sb->pos] = index;
	sb->pos = (sb->pos + 1) % MAX_CACHED_NUM;
}

static void update_cpu_avc_sbuf(unsigned long key, int index)
{
	unsigned long flags;

	local_irq_save(flags);

	__update_cpu_avc_sbuf(key, index);

	local_irq_restore(flags);
}

static int fast_avc_lookup(unsigned long key)
{
	unsigned long flags;
	int pos;
	struct avc_sbuf_cache *sb;
	int index = -1;

	local_irq_save(flags);

	sb = this_cpu_ptr(&cpu_avc_sbuf);

	pos = sb->pos;
	/* Try the 1st hit */
	pos = (pos + CACHED_NUM_MASK) & CACHED_NUM_MASK;
	if (sb->cached[pos] == key) {
		index = sb->cached_index[pos];
		goto exit;
	}

	/* Try more */
	for (pos = 0; pos < MAX_CACHED_NUM; pos++) {
		if (sb->cached[pos] == key) {
			index = sb->cached_index[pos];
			goto exit;
		}
	}

exit:
	local_irq_restore(flags);

	return index;
}

static void probe_android_rvh_selinux_avc_insert(void *ignore, const struct avc_node *node)
{
	struct mkp_avc_node *temp_node = NULL;
	int ret = -1;

	if (g_ro_avc_handle == 0)
		return;

	MKP_HOOK_BEGIN(__func__);

	temp_node = (struct mkp_avc_node *)node;
	ret = mkp_update_sharebuf_4_argu(MKP_POLICY_SELINUX_AVC, g_ro_avc_handle,
		(unsigned long)temp_node, temp_node->ae.ssid,
		temp_node->ae.tsid, temp_node->ae.tclass, temp_node->ae.avd.allowed);

	__update_cpu_avc_sbuf((unsigned long)temp_node, ret);

	MKP_HOOK_END(__func__);
}

static void probe_android_rvh_selinux_avc_node_delete(void *ignore,
	const struct avc_node *node)
{
	int ret = -1;

	if (g_ro_avc_handle == 0)
		return;

	MKP_HOOK_BEGIN(__func__);

	ret = mkp_update_sharebuf_4_argu(MKP_POLICY_SELINUX_AVC, g_ro_avc_handle,
		(unsigned long)node, 0, 0, 0, 0);

	MKP_HOOK_END(__func__);
}

static void probe_android_rvh_selinux_avc_node_replace(void *ignore,
	const struct avc_node *old, const struct avc_node *new)
{
	struct mkp_avc_node *new_node = (struct mkp_avc_node *)new;
	int ret = -1;

	if (g_ro_avc_handle == 0)
		return;

	MKP_HOOK_BEGIN(__func__);

	ret = mkp_update_sharebuf_4_argu(MKP_POLICY_SELINUX_AVC, g_ro_avc_handle,
		(unsigned long)old, 0, 0, 0, 0);

	ret = mkp_update_sharebuf_4_argu(MKP_POLICY_SELINUX_AVC, g_ro_avc_handle,
		(unsigned long)new_node, new_node->ae.ssid,
		new_node->ae.tsid, new_node->ae.tclass, new_node->ae.avd.allowed);
	__update_cpu_avc_sbuf((unsigned long)new_node, ret);

	MKP_HOOK_END(__func__);
}

static void probe_android_rvh_selinux_avc_lookup(void *ignore,
	const struct avc_node *node, u32 ssid, u32 tsid, u16 tclass)
{
	void *va;
	struct avc_sbuf_content *ro_avc_sharebuf_ptr;
	int index;
	int i = -1;
	struct mkp_avc_node *temp_node = NULL;
	bool ready = false;
	static DEFINE_RATELIMIT_STATE(rs_avc, 1*HZ, 10);
#if IS_ENABLED(CONFIG_KASAN)
	bool cached = false;
#endif

	if (!node || g_ro_avc_handle == 0)
		return;

	ratelimit_set_flags(&rs_avc, RATELIMIT_MSG_ON_RELEASE);
	if (__ratelimit(&rs_avc)) {

		MKP_HOOK_BEGIN(__func__);

		temp_node = (struct mkp_avc_node *)node;
		va = page_address(avc_pages);
		ro_avc_sharebuf_ptr = (struct avc_sbuf_content *)va;

		index = fast_avc_lookup((unsigned long)temp_node);
		if (index != -1) {
			ro_avc_sharebuf_ptr += index;
			if ((unsigned long)ro_avc_sharebuf_ptr->avc_node ==
				(unsigned long)temp_node)
				ready = true;
#if IS_ENABLED(CONFIG_KASAN)
			cached = true;
#endif
		}

		if (!ready) {
			ro_avc_sharebuf_ptr = (struct avc_sbuf_content *)va;
			for (i = 0; i < avc_array_sz; ro_avc_sharebuf_ptr++, i++) {
				if ((unsigned long)ro_avc_sharebuf_ptr->avc_node ==
					(unsigned long)temp_node) {
					ready = true;
					update_cpu_avc_sbuf((unsigned long)temp_node, i);
					break;
				}
			}
		}
		if (ready) {
			if (ro_avc_sharebuf_ptr->ssid != ssid ||
				ro_avc_sharebuf_ptr->tsid != tsid ||
				ro_avc_sharebuf_ptr->tclass != tclass ||
				ro_avc_sharebuf_ptr->ae_allowed !=
					temp_node->ae.avd.allowed) {
				MKP_ERR("avc lookup is not matched\n");
#if IS_ENABLED(CONFIG_MTK_VM_DEBUG)
				MKP_ERR("CURRENT-%16lx:%16lx:%16lx:%16lx\n",
				       (unsigned long)ssid,
				       (unsigned long)tsid,
				       (unsigned long)tclass,
				       (unsigned long)temp_node->ae.avd.allowed);
				MKP_ERR("@EXPECT-%16lx:%16lx:%16lx:%16lx\n",
				       (unsigned long)ro_avc_sharebuf_ptr->ssid,
				       (unsigned long)ro_avc_sharebuf_ptr->tsid,
				       (unsigned long)ro_avc_sharebuf_ptr->tclass,
				       (unsigned long)ro_avc_sharebuf_ptr->ae_allowed);
#endif

#if IS_ENABLED(CONFIG_KASAN)
				if (!cached)
					goto report;

				MKP_ERR("Index from fast_avc_lookup: %d\n", index);

				/* Try full iteration to find out all possible aliases */
				ro_avc_sharebuf_ptr = (struct avc_sbuf_content *)va;
				for (i = 0; i < avc_array_sz; ro_avc_sharebuf_ptr++, i++) {
					if ((unsigned long)ro_avc_sharebuf_ptr->avc_node ==
							(unsigned long)temp_node) {
						MKP_ERR("Alias found: %d\n", i);
					}
				}
report:
#endif
				handle_mkp_err_action(MKP_POLICY_SELINUX_AVC);
			}
		}
		MKP_HOOK_END(__func__);
		return; // pass
	}
}

static void avc_work_handler(struct work_struct *work)
{
	int ret = 0, ret_erri_line;

	// register avc vendor hook after selinux is initialized
	if (policy_ctrl[MKP_POLICY_SELINUX_AVC] != 0 ||
		g_ro_avc_handle != 0) {
		// register avc vendor hook
		ret = register_trace_android_rvh_selinux_avc_insert(
				probe_android_rvh_selinux_avc_insert, NULL);
		if (ret) {
			ret_erri_line = __LINE__;
			goto avc_failed;
		}
		ret = register_trace_android_rvh_selinux_avc_node_delete(
				probe_android_rvh_selinux_avc_node_delete, NULL);
		if (ret) {
			ret_erri_line = __LINE__;
			goto avc_failed;
		}
		ret = register_trace_android_rvh_selinux_avc_node_replace(
				probe_android_rvh_selinux_avc_node_replace, NULL);
		if (ret) {
			ret_erri_line = __LINE__;
			goto avc_failed;
		}
		ret = register_trace_android_rvh_selinux_avc_lookup(
				probe_android_rvh_selinux_avc_lookup, NULL);
		if (ret) {
			ret_erri_line = __LINE__;
			goto avc_failed;
		}
	}
avc_failed:
	if (ret)
		MKP_ERR("register avc hooks failed, ret %d line %d\n", ret, ret_erri_line);
}
static void probe_android_rvh_selinux_is_initialized(void *ignore,
	const struct selinux_state *state)
{
	g_initialized = state->initialized;
	g_avc = state->avc;
	g_policy = state->policy;
	g_selinux_state = state;

	if (policy_ctrl[MKP_POLICY_SELINUX_AVC]) {
		if (!avc_work) {
			MKP_ERR("avc work create fail\n");
			return;
		}
		INIT_WORK(avc_work, avc_work_handler);
		schedule_work(avc_work);
	}
}

static int __init protect_mkp_self(void)
{
	module_enable_ro(THIS_MODULE, false, MKP_POLICY_MKP);
	module_enable_nx(THIS_MODULE, MKP_POLICY_MKP);
	module_enable_x(THIS_MODULE, MKP_POLICY_MKP);

	mkp_start_granting_hvc_call();
	return 0;
}

int mkp_reboot_notifier_event(struct notifier_block *nb, unsigned long event, void *v)
{
	MKP_DEBUG("mkp reboot notifier\n");
	return NOTIFY_DONE;
}
static struct notifier_block mkp_reboot_notifier = {
	.notifier_call = mkp_reboot_notifier_event,
};

/* Map full kernel text in the granularity of 2MB */
static const struct of_device_id mkp_of_match[] = {
	{ .compatible = "mediatek,mkp-drv", },
	{ }
};
MODULE_DEVICE_TABLE(of, mkp_of_match);

static int get_reserved_memory(struct device *dev)
{
	struct device_node *np;
	struct reserved_mem *rmem;

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		dev_info(dev, "no memory-region\n");
		return -EINVAL;
	}

	rmem = of_reserved_mem_lookup(np);
	of_node_put(np);

	if (!rmem) {
		dev_info(dev, "no memory-region\n");
		return -EINVAL;
	}

	/* Enable the support of full kernel code with 2M mapping */
	full_kernel_code_2m = true;
	MKP_INFO("Support FULL_KERNEL_CODE_2M\n");

	return 0;
}

static int mkp_probe(struct platform_device *pdev)
{
	get_reserved_memory(&pdev->dev);

	return 0;
}

struct platform_driver mkp_driver = {
	.probe = mkp_probe,
	.remove = NULL,
	.driver = {
		.name = "mkp-drv",
		.owner = THIS_MODULE,
		.of_match_table = mkp_of_match,
	},
};

struct proc_dir_entry *d_moto_rkp;
int __read_mostly moto_rkp_locked;
#define MOTO_RKP_PROC_DIR		"moto_rkp"
static ssize_t proc_moto_rkp_lock_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[3] = {0};
	int err, val;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	err = kstrtoint(strstrip(buffer), 10, &val);
	if (err)
		return err;

	moto_rkp_locked = val;

	if (moto_rkp_locked && (mkp_lockdown_hvc_call(MKP_BOOT_COMPLETED) == 0)) {
		pr_info("%s: MotoRkp set done and locked\n", __func__);
	}

	return count;
}

static ssize_t proc_moto_rkp_lock_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[3] = {0};
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "%d\n", moto_rkp_locked);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_moto_rkp_enabled_fops = {
	.proc_write		= proc_moto_rkp_lock_write,
	.proc_read		= proc_moto_rkp_lock_read,
};

int moto_rkp_proc_init(void)
{
	struct proc_dir_entry *proc_node = NULL;

	d_moto_rkp = proc_mkdir(MOTO_RKP_PROC_DIR, NULL);
	if (!d_moto_rkp) {
		MKP_ERR("failed to create proc dir moto_rkp\n");
		goto err_creat_d_moto_rkp;
	}

	proc_node = proc_create("locked", 0664, d_moto_rkp, &proc_moto_rkp_enabled_fops);
	if (!proc_node) {
		MKP_ERR("failed to create proc node moto_rkp_locked\n");
		goto err_creat_d_moto_rkp;
	}

	return 0;

err_creat_d_moto_rkp:
	remove_proc_entry("locked", d_moto_rkp);
	return -ENOENT;
}


int __init moto_rkp_init(void)
{
	int ret = 0, ret_erri_line;
	unsigned long size = 0x100000;
	u32 mkp_policy = 0x0001ffff;

	ret = platform_driver_register(&mkp_driver);
	if (ret)
		MKP_WARN("Failed to support FULL_KERNEL_CODE_2M\n");

	/* Set policy control */
	mkp_set_policy(mkp_policy);

	/* Protect kernel code & rodata */
	if (policy_ctrl[MKP_POLICY_KERNEL_CODE] != 0 ||
		policy_ctrl[MKP_POLICY_KERNEL_RODATA] != 0) {

#if !IS_ENABLED(CONFIG_KASAN_GENERIC) && !IS_ENABLED(CONFIG_KASAN_SW_TAGS)
#if !IS_ENABLED(CONFIG_GCOV_KERNEL)
		schedule_delayed_work(&mkp_pk_work, 0);
#endif
#endif
	}

	/* Protect MKP itself */
	if (policy_ctrl[MKP_POLICY_MKP] != 0)
		ret = protect_mkp_self();

	/* Create selinux avc sharebuf and mark it with RO*/
	if (policy_ctrl[MKP_POLICY_SELINUX_AVC] != 0) {
		g_ro_avc_handle = mkp_create_ro_sharebuf(MKP_POLICY_SELINUX_AVC, size, &avc_pages);
		if (g_ro_avc_handle != 0) {
			ret = mkp_configure_sharebuf(MKP_POLICY_SELINUX_AVC, g_ro_avc_handle,
				0, 8192 /* avc_sbuf_content */, sizeof(struct avc_sbuf_content)-8);
			rem = do_div(size, sizeof(struct avc_sbuf_content));
			avc_array_sz = size;
		} else {
			MKP_ERR("Create avc ro sharebuf fail\n");
		}
	}

	if (policy_ctrl[MKP_POLICY_SELINUX_AVC])
		avc_work = kmalloc(sizeof(struct work_struct), GFP_KERNEL);

	/* Protect the kernel pages but with module core_layout/init_layout must be confirmed in vitual addr region */
	if (policy_ctrl[MKP_POLICY_DRV] != 0 ||
		policy_ctrl[MKP_POLICY_KERNEL_PAGES] != 0 ||
		policy_ctrl[MKP_POLICY_MKP] != 0) {
		// register rw, nx
		ret = register_trace_android_rvh_set_module_core_rw_nx(
				probe_android_rvh_set_module_core_rw_nx, NULL);
		if (ret) {
			ret_erri_line = __LINE__;
			goto failed;
		}
		ret = register_trace_android_rvh_set_module_init_rw_nx(
				probe_android_rvh_set_module_init_rw_nx, NULL);
		if (ret) {
			ret_erri_line = __LINE__;
			goto failed;
		}
	}

	/* Protect the kernel drivers, mark them with readonly after init */
	if (policy_ctrl[MKP_POLICY_DRV] != 0 ||
		policy_ctrl[MKP_POLICY_MKP] != 0) {
		ret = register_trace_android_rvh_set_module_permit_before_init(
				probe_android_rvh_set_module_permit_before_init, NULL);
		if (ret) {
			ret_erri_line = __LINE__;
			goto failed;
		}

		ret = register_trace_android_rvh_set_module_permit_after_init(
				probe_android_rvh_set_module_permit_after_init, NULL);
		if (ret) {
			ret_erri_line = __LINE__;
			goto failed;
		}
	}

	/* Protect selinux initialized state */
	if (policy_ctrl[MKP_POLICY_SELINUX_STATE] != 0) {
		ret = register_trace_android_rvh_selinux_is_initialized(
				probe_android_rvh_selinux_is_initialized, NULL);
		if (ret) {
			ret_erri_line = __LINE__;
			goto failed;
		}
	}

	/* Lock down the sensitive HVC calls from EL2 MKP service */
	while (!mkp_ro_region_is_already_locked) {
		pr_info("%s: MotoRkp waiting for kernel RO region to be ready..\n", __func__);
		mdelay(100); // Align with the workqueue to be scheduled in mkp_pk_work.
	}

	if (mkp_ro_region_is_already_locked && (mkp_lockdown_hvc_call(MKP_INIT) == 0)) {
		pr_info("%s: MotoRkp set done and locked\n", __func__);
	}

	/* Create the /proc device node for userspace lockdown everyting once completed */
	if (moto_rkp_proc_init()) {
		MKP_ERR("MotoRkp: Failed to register the lockdown node in /proc\n");
	}

	/* Register a notifier for reboot cycle */
	register_reboot_notifier(&mkp_reboot_notifier);

failed:
	if (ret)
		MKP_ERR("register hooks failed, ret %d line %d\n", ret, ret_erri_line);

	/* Don't fail the kernel even if we lose the protection */
	return 0;
}
