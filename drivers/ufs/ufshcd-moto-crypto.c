// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Motorola Mobility, All Rights Reserved.
 */

#include "ufs/ufshcd.h"
#include "ufshcd-crypto.h"
#include "tlc_km.h"


/* Blk-crypto modes supported by UFS crypto */
static const struct ufs_crypto_alg_entry {
	enum ufs_crypto_alg ufs_alg;
	enum ufs_crypto_key_size ufs_key_size;
} ufs_crypto_algs[BLK_ENCRYPTION_MODE_MAX] = {
	[BLK_ENCRYPTION_MODE_AES_256_XTS] = {
		.ufs_alg = UFS_CRYPTO_ALG_AES_XTS,
		.ufs_key_size = UFS_CRYPTO_KEY_SIZE_256,
	},
};

static u8 ufshcd_crypto_get_crypto_mode(u8 cap_idx)
{
	if (cap_idx == 0)
		return HWKM_BC_AES_128_XTS;
	else if (cap_idx == 1)
		return HWKM_BC_AES_256_XTS;

	return 0;
}

static u32 ufshcd_get_crypto_para(const union ufs_crypto_cfg_entry *cfg, int slot)
{
	u32 crypto_para = 0;
	u8 mode = 0;

	mode = ufshcd_crypto_get_crypto_mode(cfg->crypto_cap_idx);

	crypto_para = ((slot & 0xFF) << 24) | /* high 8 bits for slot number */
			((mode & 0xFF) << 16) |  /* bit 16 ~ 24 for encryption mode */
			((0x40 & 0xFF) << 8);  /* bit 8 ~ 16 for total key bytes */

	/* Disable encryption if not configured yet */
    if (cfg->config_enable == 0)
		crypto_para |= 0x01;

#ifdef CONFIG_MTK_UFS_DEBUG_BUILD
	pr_notice("Moto HWKM: ufshcd_get_crypto_para with cfg=0x%x\n", crypto_para);
#endif

	return crypto_para;
}

static int ufshcd_program_wrapped_key(struct ufs_hba *hba,
			      const union ufs_crypto_cfg_entry *cfg, int slot)
{
	u32 slot_offset = hba->crypto_cfg_register + slot * sizeof(*cfg);
	int err = -EINVAL;

	u32 crypto_para = ufshcd_get_crypto_para(cfg, slot);

	ufshcd_hold(hba, false);

	/* Clear the dword 16 and ensure that CFGE is cleared before programming the key */
	ufshcd_writel(hba, 0, slot_offset + 16 * sizeof(cfg->reg_val[0]));
	wmb();

#ifdef CONFIG_MTK_UFS_DEBUG_BUILD
	/* Write dword 0-15 w/ the wrapped key through secure driver */
	pr_notice("Moto HWKM: Start programming slot: %d\n", slot);
#endif
	if (hwkm_program_key(
			cfg->crypto_key, /* 64 bytes wrapped key data */
			UFS_CRYPTO_KEY_MAX_SIZE/2 + WRAPPED_STORAGE_KEY_HEADER_SIZE, /* 40 bytes in effect */
			crypto_para,
			STORAGE_KEY_UFS) != 0) {
		pr_notice("Moto HWKM: Error: Unwrap or install key failed to ICE, slot=%d, cfg=0x%x\n", slot, crypto_para);
		goto out;
	}
#ifdef CONFIG_MTK_UFS_DEBUG_BUILD
	pr_notice("Moto HWKM: End programing slot: %d\n", slot);
#endif
	wmb();

	/* Write dword 17 */
	ufshcd_writel(hba, le32_to_cpu(cfg->reg_val[17]),
		      slot_offset + 17 * sizeof(cfg->reg_val[0]));
	wmb();

	/* Write dword 16, it must be done at last */
	ufshcd_writel(hba, le32_to_cpu(cfg->reg_val[16]),
		      slot_offset + 16 * sizeof(cfg->reg_val[0]));
	wmb();

	err = 0;

out:
	ufshcd_release(hba);
	return err;
}

static int ufshcd_moto_crypto_keyslot_program(struct blk_crypto_profile *profile,
					 const struct blk_crypto_key *key,
					 unsigned int slot)
{
	struct ufs_hba *hba =
		container_of(profile, struct ufs_hba, crypto_profile);

	const union ufs_crypto_cap_entry *ccap_array = hba->crypto_cap_array;
	const struct ufs_crypto_alg_entry *alg =
			&ufs_crypto_algs[key->crypto_cfg.crypto_mode];
	u8 data_unit_mask = key->crypto_cfg.data_unit_size / 512;
	int i = 0;
	int cap_idx = -1;
	union ufs_crypto_cfg_entry cfg = {};
	int err = 0;

	BUILD_BUG_ON(UFS_CRYPTO_KEY_SIZE_INVALID != 0);
	for (i = 0; i < hba->crypto_capabilities.num_crypto_cap; i++) {
		if (ccap_array[i].algorithm_id == alg->ufs_alg &&
		    ccap_array[i].key_size == alg->ufs_key_size &&
		    (ccap_array[i].sdus_mask & data_unit_mask)) {
			cap_idx = i;
			break;
		}
	}

	if (WARN_ON(cap_idx < 0)) {
		pr_notice("Moto HWKM: Error: No valid capabitlity index can be found!");
		return -EOPNOTSUPP;
	}

	cfg.data_unit_size = data_unit_mask;
	cfg.crypto_cap_idx = cap_idx;
	cfg.config_enable = UFS_CRYPTO_CONFIGURATION_ENABLE;

	if (ccap_array[cap_idx].algorithm_id == UFS_CRYPTO_ALG_AES_XTS) {
		/* In XTS mode, the blk_crypto_key's size is already doubled */
		memcpy(cfg.crypto_key, key->raw, key->size/2);
		memcpy(cfg.crypto_key + UFS_CRYPTO_KEY_MAX_SIZE/2,
		       key->raw + key->size/2, key->size/2);
	} else {
		memcpy(cfg.crypto_key, key->raw, key->size);
	}

	/* Invoke the secure side to unwrap the key and provision them onto HIE */
	err = ufshcd_program_wrapped_key(hba, &cfg, slot);

	memzero_explicit(&cfg, sizeof(cfg));
	return err;
}

static int ufshcd_moto_crypto_keyslot_evict(struct blk_crypto_profile *profile,
				       const struct blk_crypto_key *key,
				       unsigned int slot)
{
	struct ufs_hba *hba =
		container_of(profile, struct ufs_hba, crypto_profile);

	/*
	 * Clear the crypto cfg on the device. Clearing CFGE
	 * might not be sufficient, so just clear the entire cfg.
	 */
	union ufs_crypto_cfg_entry cfg = {{0}};

	int i = 0;
	u32 slot_offset = hba->crypto_cfg_register + slot * sizeof(cfg);
	int err = 0;

	ufshcd_hold(hba, false);

	/* Ensure that CFGE is cleared before programming the key */
	ufshcd_writel(hba, 0, slot_offset + 16 * sizeof(cfg.reg_val[0]));
	wmb();

	/* Cleanup dword 0-15 keyslot */
	for (i = 0; i < 16; i++) {
		ufshcd_writel(hba, le32_to_cpu(cfg.reg_val[i]),
			      slot_offset + i * sizeof(cfg.reg_val[0]));
	}
	wmb();

	/* Cleanup dword 17 */
	ufshcd_writel(hba, le32_to_cpu(cfg.reg_val[17]),
		      slot_offset + 17 * sizeof(cfg.reg_val[0]));
	wmb();

	/* Clenup dword 16 and it must be written at last */
	ufshcd_writel(hba, le32_to_cpu(cfg.reg_val[16]),
		      slot_offset + 16 * sizeof(cfg.reg_val[0]));
	wmb();

	ufshcd_release(hba);
	return err;
}

static int ufshcd_moto_crypto_derive_raw_secret(struct blk_crypto_profile *profile,
			const u8 *eph_key,
			size_t eph_key_size,
			u8 sw_secret[BLK_CRYPTO_SW_SECRET_SIZE])
{
	return hwkm_derive_raw_secret(eph_key, eph_key_size,
							sw_secret, BLK_CRYPTO_SW_SECRET_SIZE);
}

static const struct blk_crypto_ll_ops ufshcd_crypto_moto_ops = {
	.keyslot_program	= ufshcd_moto_crypto_keyslot_program,
	.keyslot_evict		= ufshcd_moto_crypto_keyslot_evict,
	.derive_sw_secret	= ufshcd_moto_crypto_derive_raw_secret,
};

static enum blk_crypto_mode_num
ufshcd_find_blk_crypto_mode(union ufs_crypto_cap_entry cap)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ufs_crypto_algs); i++) {
		BUILD_BUG_ON(UFS_CRYPTO_KEY_SIZE_INVALID != 0);
		if (ufs_crypto_algs[i].ufs_alg == cap.algorithm_id &&
		    ufs_crypto_algs[i].ufs_key_size == cap.key_size) {
			return i;
		}
	}
	return BLK_ENCRYPTION_MODE_INVALID;
}

/**
 * ufshcd_moto_hba_init_crypto_capabilities - Read crypto capabilities, init crypto
 *					 fields in hba
 * @hba: Per adapter instance
 *
 * Return: 0 if crypto was initialized or is not supported, else a -errno value.
 */
int ufshcd_moto_hba_init_crypto_capabilities(struct ufs_hba *hba)
{
	int cap_idx = 0;
	int err = 0;
	enum blk_crypto_mode_num blk_mode_num;

	/*
	 * Don't use crypto if either the hardware doesn't advertise the
	 * standard crypto capability bit *or* if the vendor specific driver
	 * hasn't advertised that crypto is supported.
	 */
	hba->capabilities = ufshcd_readl(hba, REG_CONTROLLER_CAPABILITIES);
	if (!(hba->capabilities & MASK_CRYPTO_SUPPORT) ||
	    !(hba->caps & UFSHCD_CAP_CRYPTO)) {
		pr_notice("Moto HWKM: Error: No UFS crypto capability support!");
		goto out;
	}

	hba->crypto_capabilities.reg_val =
			cpu_to_le32(ufshcd_readl(hba, REG_UFS_CCAP));
	hba->crypto_cfg_register =
		(u32)hba->crypto_capabilities.config_array_ptr * 0x100;
	hba->crypto_cap_array =
		devm_kcalloc(hba->dev, hba->crypto_capabilities.num_crypto_cap,
			     sizeof(hba->crypto_cap_array[0]), GFP_KERNEL);
	if (!hba->crypto_cap_array) {
		pr_notice("Moto HWKM: Error: Invalid crypto capabilities, out of memory?\n");
		err = -ENOMEM;
		goto out;
	}

	/* The actual number of configurations supported is (CFGC+1) */
	err = devm_blk_crypto_profile_init(
			hba->dev, &hba->crypto_profile,
			hba->crypto_capabilities.config_count + 1);
	if (err) {
		pr_notice("Moto HWKM: Error: Couldn't attach KSM to current hba device: %d\n", err);
		goto out;
	}

	hba->crypto_profile.ll_ops = ufshcd_crypto_moto_ops;
	/* UFS only supports 8 bytes for any DUN */
	hba->crypto_profile.max_dun_bytes_supported = 8;
	hba->crypto_profile.key_types_supported = BLK_CRYPTO_KEY_TYPE_HW_WRAPPED;
	hba->crypto_profile.dev = hba->dev;

	/*
	 * Cache all the UFS crypto capabilities and advertise the supported
	 * crypto modes and data unit sizes to the block layer.
	 */
	for (cap_idx = 0; cap_idx < hba->crypto_capabilities.num_crypto_cap;
	     cap_idx++) {
		hba->crypto_cap_array[cap_idx].reg_val =
			cpu_to_le32(ufshcd_readl(hba,
						 REG_UFS_CRYPTOCAP +
						 cap_idx * sizeof(__le32)));
		blk_mode_num = ufshcd_find_blk_crypto_mode(
						hba->crypto_cap_array[cap_idx]);
		if (blk_mode_num != BLK_ENCRYPTION_MODE_INVALID)
			hba->crypto_profile.modes_supported[blk_mode_num] |=
				hba->crypto_cap_array[cap_idx].sdus_mask * 512;
	}

	pr_notice("Moto HWKM: Successfully registered the crypto profile with wrapped key mode!");
	return 0;

out:
	/* Indicate that init failed by clearing UFSHCD_CAP_CRYPTO */
	hba->caps &= ~UFSHCD_CAP_CRYPTO;
	return err;
}
