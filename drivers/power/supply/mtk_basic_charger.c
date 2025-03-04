// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

/*
 *
 * Filename:
 * ---------
 *    mtk_basic_charger.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines functions of Battery charging
 *
 * Author:
 * -------
 * Wy Chuang
 *
 */
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/reboot.h>

#include "mtk_charger.h"
#include <linux/gpio.h>

static int _uA_to_mA(int uA)
{
	if (uA == -1)
		return -1;
	else
		return uA / 1000;
}

static void select_cv(struct mtk_charger *info)
{
	u32 constant_voltage;

	if (info->enable_sw_jeita)
		if (info->sw_jeita.cv != 0) {
			info->setting.cv = info->sw_jeita.cv;
			return;
		}

	constant_voltage = info->data.battery_cv;
	constant_voltage = info->mmi.target_fv;
	info->setting.cv = constant_voltage;
}

static bool is_typec_adapter(struct mtk_charger *info)
{
	int rp;

	if (info == NULL || info->pd_adapter == NULL)
		return false;

	rp = adapter_dev_get_property(info->pd_adapter, TYPEC_RP_LEVEL);
	if (rp > 500)
		return true;

	return false;
}

static bool support_fast_charging(struct mtk_charger *info)
{
	struct chg_alg_device *alg;
	int i = 0, state = 0;
	bool ret = false;
	bool qc_is_detect = false;
	int qc_chg_type = 0;

	for (i = 0; i < MAX_ALG_NO; i++) {
		alg = info->alg[i];
		if (alg == NULL)
			continue;

		if (info->enable_fast_charging_indicator &&
		    ((alg->alg_id & info->fast_charging_indicator) == 0))
			continue;

		charger_dev_qc_is_detect(info->chg1_dev, &qc_is_detect);
		charger_dev_get_protocol(info->chg1_dev, &qc_chg_type);
		if(qc_is_detect == true && alg->alg_id  != PE5_ID) {
			chr_err("qc is detecting and skip detect others type\n");
			return ret;
		} else if (qc_chg_type == USB_TYPE_QC30) {
			if(charger_dev_config_qc_charger(info->chg1_dev) != 0) {
				chr_err("config_qc_charger set dpdm failed\n");
			} else {
				chr_err("it is HVDCP set ICL 3A  qc_chg_type = %d\n",qc_chg_type);
			}
			return ret;
		} /*else if (qc_chg_type == USB_TYPE_QC3P_18 || qc_chg_type == USB_TYPE_QC3P_27) {
			chr_err("qc type and skip others type \n");
			return ret;
		}*/

		chg_alg_set_current_limit(alg, &info->setting);
		state = chg_alg_is_algo_ready(alg);
		chr_debug("%s %s ret:%s\n", __func__, dev_name(&alg->dev),
			  chg_alg_state_to_str(state));

		if (state == ALG_READY || state == ALG_RUNNING) {
			info->mmi.active_fast_alg |= alg->alg_id;
			if ((PE5_ID == alg->alg_id) &&
			    (PDC_ID & info->mmi.active_fast_alg)) {
				alg = get_chg_alg_by_name("pd");
				chg_alg_stop_algo(alg);
				info->mmi.active_fast_alg &= ~PDC_ID;
			}
			ret = true;
			break;
		} else
			info->mmi.active_fast_alg &= ~alg->alg_id;
	}
	return ret;
}

static bool select_charging_current_limit(struct mtk_charger *info,
	struct chg_limit_setting *setting)
{
	struct charger_data *pdata, *pdata2, *pdata_dvchg, *pdata_dvchg2;
	bool is_basic = false;
	u32 ichg1_min = 0, aicr1_min = 0;
	int ret;
	bool qc_is_detect = false;
	int qc_chg_type = 0;
	int icl_max = -1;

	select_cv(info);

	pdata = &info->chg_data[CHG1_SETTING];
	pdata2 = &info->chg_data[CHG2_SETTING];
	pdata_dvchg = &info->chg_data[DVCHG1_SETTING];
	pdata_dvchg2 = &info->chg_data[DVCHG2_SETTING];

	if (info->usb_unlimited) {
		pdata->input_current_limit =
					info->data.ac_charger_input_current;
		pdata->charging_current_limit =
					info->data.ac_charger_current;
		is_basic = true;
		goto done;
	}

	if (info->water_detected) {
		pdata->input_current_limit = info->data.usb_charger_current;
		pdata->charging_current_limit = info->data.usb_charger_current;
		is_basic = true;
		goto done;
	}

	if (((info->bootmode == 1) ||
	    (info->bootmode == 5)) && info->enable_meta_current_limit != 0) {
		pdata->input_current_limit = 200000; // 200mA
		is_basic = true;
		goto done;
	}
#ifdef MTK_BASE
	if (info->atm_enabled == true
		&& (info->chr_type == POWER_SUPPLY_TYPE_USB ||
		info->chr_type == POWER_SUPPLY_TYPE_USB_CDP)
		) {
		pdata->input_current_limit = 100000; /* 100mA */
		is_basic = true;
		goto done;
	}
#endif



	if (info->chr_type == POWER_SUPPLY_TYPE_USB &&
	    info->usb_type == POWER_SUPPLY_USB_TYPE_SDP) {
		pdata->input_current_limit =
				info->data.usb_charger_current;
		/* it can be larger */
		pdata->charging_current_limit =
				info->data.usb_charger_current;
		is_basic = true;
	} else if (info->chr_type == POWER_SUPPLY_TYPE_USB_CDP) {
		pdata->input_current_limit =
			info->data.charging_host_charger_current;
		pdata->charging_current_limit =
			info->data.charging_host_charger_current;
		is_basic = true;

	} else if (info->chr_type == POWER_SUPPLY_TYPE_USB_DCP) {
		charger_dev_get_max_input_current(info->chg1_dev, &icl_max);
		if (icl_max > 0) {
			pdata->input_current_limit = icl_max;
		} else {
			pdata->input_current_limit =
				info->data.ac_charger_input_current;
		}
		pdata->charging_current_limit =
			info->data.ac_charger_current;
		if (info->config == DUAL_CHARGERS_IN_SERIES) {
			pdata2->input_current_limit =
				pdata->input_current_limit;
			pdata2->charging_current_limit = 2000000;
		}
	} else if (info->chr_type == POWER_SUPPLY_TYPE_USB &&
	    info->usb_type == POWER_SUPPLY_USB_TYPE_DCP) {
		/* NONSTANDARD_CHARGER */
		pdata->input_current_limit =
			info->data.usb_charger_current;
		pdata->charging_current_limit =
			info->data.usb_charger_current;
		is_basic = true;
	} else {
		/*chr_type && usb_type cannot match above, set 500mA*/
		pdata->input_current_limit =
				info->data.usb_charger_current;
		pdata->charging_current_limit =
				info->data.usb_charger_current;
		is_basic = true;
	}

	info->setting.mmi_fcc_limit  =  ((info->mmi.target_fcc < 0) ? 0 : info->mmi.target_fcc);

	if (!IS_ERR_OR_NULL(info->cp_cdev) &&
			(info->cp_thermal_fcc < 0 ||
			info->cp_thermal_fcc > info->mmi.min_therm_current_limit))
		info->setting.mmi_current_limit_dvchg1 = info->cp_thermal_fcc;
	else if (IS_ERR_OR_NULL(info->cp_cdev) &&
			(pdata->thermal_charging_current_limit < 0 ||
			pdata->thermal_charging_current_limit > info->mmi.min_therm_current_limit))
		info->setting.mmi_current_limit_dvchg1 = pdata->thermal_charging_current_limit;
	else
		info->setting.mmi_current_limit_dvchg1 = info->mmi.min_therm_current_limit;

	if ((info->mmi.force_cp_fcc_ma > 0) &&
		(info->setting.mmi_current_limit_dvchg1 < 0 || info->setting.mmi_current_limit_dvchg1 > (info->mmi.force_cp_fcc_ma * 1000)))
		info->setting.mmi_current_limit_dvchg1 = info->mmi.force_cp_fcc_ma * 1000;

	if (support_fast_charging(info))
		is_basic = false;
	else {
		is_basic = true;
		/* AICL */
		if (!info->disable_aicl)
			charger_dev_run_aicl(info->chg1_dev,
				&pdata->input_current_limit_by_aicl);
		if (info->enable_dynamic_mivr) {
			if (pdata->input_current_limit_by_aicl >
				info->data.max_dmivr_charger_current)
				pdata->input_current_limit_by_aicl =
					info->data.max_dmivr_charger_current;
		}
		if (is_typec_adapter(info)) {
			if (adapter_dev_get_property(info->pd_adapter, TYPEC_RP_LEVEL)
				== 3000) {
				if (info->mmi.typec_rp_max_current)
					pdata->input_current_limit = info->mmi.typec_rp_max_current;
				else
					pdata->input_current_limit = 3000000;
				pdata->charging_current_limit = 3000000;
			} else if (adapter_dev_get_property(info->pd_adapter,
				TYPEC_RP_LEVEL) == 1500) {
				pdata->input_current_limit = 1500000;
				pdata->charging_current_limit = 2000000;
			} else {
				chr_err("type-C: inquire rp error\n");
				pdata->input_current_limit = 500000;
				pdata->charging_current_limit = 500000;
			}

			chr_err("type-C:%d current:%d\n",
				info->pd_type,
				adapter_dev_get_property(info->pd_adapter,
					TYPEC_RP_LEVEL));
		}
	}

	if (info->enable_sw_jeita) {
		if (IS_ENABLED(CONFIG_USBIF_COMPLIANCE)
			&& info->chr_type == POWER_SUPPLY_TYPE_USB)
			chr_debug("USBIF & STAND_HOST skip current check\n");
		else {
			if (info->sw_jeita.sm == TEMP_T0_TO_T1) {
				pdata->input_current_limit = 500000;
				pdata->charging_current_limit = 350000;
			}
		}
	}

	pdata->charging_current_limit = ((info->mmi.target_fcc < 0) ? 0 : info->mmi.target_fcc);

	/*When charging FULL mt6375 trigger pwr rdy interrupt frequent.
	It can't be reproduced by reduce ICL when FULL.*/
	if (info->mmi.sm_param[BASE_BATT].pres_chrg_step == STEP_FULL)
		pdata->input_current_limit = 500000;

	charger_dev_qc_is_detect(info->chg1_dev, &qc_is_detect);
	charger_dev_get_protocol(info->chg1_dev, &qc_chg_type);
	/*when qc is detect should make sure ICL is 500mA*/
	if(qc_is_detect == true){
		if(pdata->input_current_limit > 500000){
			pdata->input_current_limit = 500000;
			chr_err("qc is detect, set input_current_limit 500mA!\n");
		}
	} else {
		if(qc_chg_type == USB_TYPE_QC3P_27 || qc_chg_type == USB_TYPE_QC3P_18
			|| qc_chg_type == USB_TYPE_QC30) {
				pdata->input_current_limit = 3000000;
		}
	}

	if ((info->mmi.force_pmic_icl_ma > 0)
		&& (pdata->input_current_limit > info->mmi.force_pmic_icl_ma * 1000)) {
		pdata->input_current_limit = info->mmi.force_pmic_icl_ma * 1000;
		chr_err("user space force pmic icl %d\n", pdata->input_current_limit);
	}

	info->mmi.target_usb = pdata->input_current_limit;

	if (pdata->cp_ichg_limit!= -1) {
		if (pdata->cp_ichg_limit <
		    pdata->charging_current_limit)
			pdata->charging_current_limit =
					pdata->cp_ichg_limit;
	}

	sc_select_charging_current(info, pdata);

	if (pdata->thermal_charging_current_limit != -1) {
		if (pdata->thermal_charging_current_limit <=
			pdata->charging_current_limit) {
			pdata->charging_current_limit =
					pdata->thermal_charging_current_limit;
			info->setting.charging_current_limit1 =
					pdata->thermal_charging_current_limit;
		} else
			info->setting.charging_current_limit1 = -1;
		pdata->thermal_throttle_record = true;
	} else
		info->setting.charging_current_limit1 = info->sc.sc_ibat;

	if (pdata->thermal_input_current_limit != -1) {
		if (pdata->thermal_input_current_limit <=
			pdata->input_current_limit) {
			pdata->input_current_limit =
					pdata->thermal_input_current_limit;
			info->setting.input_current_limit1 =
					pdata->input_current_limit;
		} else
			info->setting.input_current_limit1 = -1;
		pdata->thermal_throttle_record = true;
	} else
		info->setting.input_current_limit1 = -1;

	if (pdata2->thermal_charging_current_limit != -1) {
		if (pdata2->thermal_charging_current_limit <=
			pdata2->charging_current_limit) {
			pdata2->charging_current_limit =
					pdata2->thermal_charging_current_limit;
			info->setting.charging_current_limit2 =
					pdata2->charging_current_limit;
		} else
			info->setting.charging_current_limit2 = -1;
	} else
		info->setting.charging_current_limit2 = info->sc.sc_ibat;

	if (pdata2->thermal_input_current_limit != -1) {
		if (pdata2->thermal_input_current_limit <=
			pdata2->input_current_limit) {
			pdata2->input_current_limit =
					pdata2->thermal_input_current_limit;
			info->setting.input_current_limit2 =
					pdata2->input_current_limit;
		} else
			info->setting.input_current_limit2 = -1;
	} else
		info->setting.input_current_limit2 = -1;

	if (is_basic == true && pdata->input_current_limit_by_aicl != -1
		&& !info->charger_unlimited
		&& !info->disable_aicl) {
		if (pdata->input_current_limit_by_aicl <
		    pdata->input_current_limit)
			pdata->input_current_limit =
					pdata->input_current_limit_by_aicl;
	}
	info->setting.input_current_limit_dvchg1 =
		pdata_dvchg->thermal_input_current_limit;

done:

	if ((info->atm_enabled == true) && info->wireless_online) {
		pdata->charging_current_limit = info->data.wireless_factory_max_current;
		pdata->input_current_limit = info->data.wireless_factory_max_input_current;
	}
	if (pdata->moto_chg_tcmd_ibat != -1)
		pdata->charging_current_limit = pdata->moto_chg_tcmd_ibat;

	if (pdata->moto_chg_tcmd_ichg != -1)
		pdata->input_current_limit = pdata->moto_chg_tcmd_ichg;

	ret = charger_dev_get_min_charging_current(info->chg1_dev, &ichg1_min);
	if (ret != -EOPNOTSUPP && pdata->charging_current_limit < ichg1_min) {
		pdata->charging_current_limit = 0;
		/* For TC_018, pleasae don't modify the format */
		chr_err("min_charging_current is too low %d %d\n",
			pdata->charging_current_limit, ichg1_min);
		is_basic = true;
	}

	ret = charger_dev_get_min_input_current(info->chg1_dev, &aicr1_min);
	if (ret != -EOPNOTSUPP && pdata->input_current_limit < aicr1_min) {
		pdata->input_current_limit = 0;
		/* For TC_018, pleasae don't modify the format */
		chr_err("min_input_current is too low %d %d\n",
			pdata->input_current_limit, aicr1_min);
		is_basic = true;
	}

	/* For TC_018, pleasae don't modify the format */
	chr_err("m:%d chg1:%d,%d,%d,%d chg2:%d,%d,%d,%d dvchg1:%d sc:%d %d %d type:%d:%d usb_unlimited:%d usbif:%d usbsm:%d aicl:%d atm:%d bm:%d b:%d\n",
		info->config,
		_uA_to_mA(pdata->thermal_input_current_limit),
		_uA_to_mA(pdata->thermal_charging_current_limit),
		_uA_to_mA(pdata->input_current_limit),
		_uA_to_mA(pdata->charging_current_limit),
		_uA_to_mA(pdata2->thermal_input_current_limit),
		_uA_to_mA(pdata2->thermal_charging_current_limit),
		_uA_to_mA(pdata2->input_current_limit),
		_uA_to_mA(pdata2->charging_current_limit),
		_uA_to_mA(pdata_dvchg->thermal_input_current_limit),
		info->sc.pre_ibat,
		info->sc.sc_ibat,
		info->sc.solution,
		info->chr_type, info->pd_type,
		info->usb_unlimited,
		IS_ENABLED(CONFIG_USBIF_COMPLIANCE), info->usb_state,
		pdata->input_current_limit_by_aicl, info->atm_enabled,
		info->bootmode, is_basic);

	return is_basic;
}

static int do_algorithm(struct mtk_charger *info)
{
	struct chg_alg_device *alg;
	struct charger_data *pdata;
	struct chg_alg_notify notify;
	bool is_basic = true;
	bool chg_done = false;
	int i;
	int ret, ret2, ret3, ret4;
	int val = 0;
	int lst_rnd_alg_idx = info->lst_rnd_alg_idx;

	pdata = &info->chg_data[CHG1_SETTING];
#ifdef MTK_BASE
	charger_dev_is_charging_done(info->chg1_dev, &chg_done);
#else
	if (info->mmi.sm_param[BASE_BATT].pres_chrg_step == STEP_FULL)
		chg_done = true;
#endif
	is_basic = select_charging_current_limit(info, &info->setting);

	if (info->is_chg_done != chg_done) {
		if (chg_done) {
			charger_dev_do_event(info->chg1_dev, EVENT_FULL, 0);
			info->polling_interval = CHARGING_FULL_INTERVAL;
			chr_err("%s battery full\n", __func__);
		} else {
			charger_dev_do_event(info->chg1_dev, EVENT_RECHARGE, 0);
			info->polling_interval = CHARGING_INTERVAL;
			chr_err("%s battery recharge\n", __func__);
		}
	}

	chr_err("%s is_basic:%d, active fast:%d\n", __func__, is_basic,  info->mmi.active_fast_alg);
	if (is_basic != true) {
		is_basic = true;
		for (i = 0; i < MAX_ALG_NO; i++) {
			alg = info->alg[i];
			if (alg == NULL)
				continue;

			if (info->enable_fast_charging_indicator &&
			    ((alg->alg_id & info->fast_charging_indicator) == 0))
				continue;

			if ((alg->alg_id & info->mmi.active_fast_alg) == 0)
				continue;

			if (!info->enable_hv_charging ||
			    pdata->charging_current_limit == 0 ||
			    pdata->input_current_limit == 0) {
				chg_alg_get_prop(alg, ALG_MAX_VBUS, &val);
				if (val > 5000)
					chg_alg_stop_algo(alg);
				chr_err("%s: alg:%s alg_vbus:%d\n", __func__,
					dev_name(&alg->dev), val);
				continue;
			} else if (IS_ERR_OR_NULL(info->cp_cdev)
				&& (pdata->thermal_charging_current_limit > 0)
				&& (pdata->thermal_charging_current_limit < info->mmi.min_therm_current_limit)
				&& ((alg->alg_id & PE5_ID) || (alg->alg_id & PEHV_ID))) {
				charger_dev_enable(info->chg1_dev, true);
				chg_alg_stop_algo(alg);
				chr_err("%s: alg:%s due to thermal limit current%d < %d\n", __func__,
					dev_name(&alg->dev), pdata->thermal_charging_current_limit,
					info->mmi.min_therm_current_limit);
				continue;
			} else if (!IS_ERR_OR_NULL(info->cp_cdev)
				&& (info->cp_thermal_fcc > 0)
				&& (info->cp_thermal_fcc < info->mmi.min_therm_current_limit)
				&& ((alg->alg_id & PE5_ID) || (alg->alg_id & PEHV_ID))) {
				charger_dev_enable(info->chg1_dev, true);
				chg_alg_stop_algo(alg);
				chr_err("%s: alg:%s due to thermal limit current%d < %d\n", __func__,
					dev_name(&alg->dev), info->cp_thermal_fcc,
					info->mmi.min_therm_current_limit);
				continue;
			}

			if (info->alg_new_arbitration && info->alg_unchangeable &&
				(lst_rnd_alg_idx > -1)) {
				if (lst_rnd_alg_idx != i)
					continue;
			}

			if (chg_done != info->is_chg_done) {
				if (chg_done) {
					notify.evt = EVT_FULL;
					notify.value = 0;
				} else {
					notify.evt = EVT_RECHARGE;
					notify.value = 0;
				}
				chg_alg_notifier_call(alg, &notify);
				chr_err("%s notify:%d\n", __func__, notify.evt);
			}

			chg_alg_set_current_limit(alg, &info->setting);
			ret = chg_alg_is_algo_ready(alg);

			chr_err("%s %s ret:%s\n", __func__,
				dev_name(&alg->dev),
				chg_alg_state_to_str(ret));

			if (ret == ALG_INIT_FAIL || ret == ALG_TA_NOT_SUPPORT) {
				/* try next algorithm */
				continue;
			} else if (ret == ALG_WAIVER) {
				if (info->alg_new_arbitration)
					continue; /* try next algorithm */
				else {
					is_basic = true;
					break;
				}
			} else if (ret == ALG_TA_CHECKING || ret == ALG_DONE ||
						ret == ALG_NOT_READY) {
				/* wait checking , use basic first */
				is_basic = true;
				if (info->alg_new_arbitration && !info->alg_unchangeable &&
					(lst_rnd_alg_idx > -1)) {
					if (lst_rnd_alg_idx != i && lst_rnd_alg_idx < MAX_ALG_NO)
						chg_alg_stop_algo(info->alg[lst_rnd_alg_idx]);
				}
				break;
			} else if (ret == ALG_READY || ret == ALG_RUNNING) {
				is_basic = false;
				if (info->alg_new_arbitration && !info->alg_unchangeable &&
					(lst_rnd_alg_idx > -1)) {
					if (lst_rnd_alg_idx != i && lst_rnd_alg_idx < MAX_ALG_NO)
						chg_alg_stop_algo(info->alg[lst_rnd_alg_idx]);
				}
				chg_alg_start_algo(alg);
				info->lst_rnd_alg_idx = i;
				break;
			} else {
				chr_err("algorithm ret is error");
				is_basic = true;
			}
		}
	} else {
		if (info->enable_hv_charging != true ||
		    pdata->charging_current_limit == 0 ||
		    pdata->input_current_limit == 0) {
			for (i = 0; i < MAX_ALG_NO; i++) {
				alg = info->alg[i];
				if (alg == NULL)
					continue;

				chg_alg_get_prop(alg, ALG_MAX_VBUS, &val);
				if (val > 5000 && chg_alg_is_algo_running(alg))
					chg_alg_stop_algo(alg);

				chr_err("%s: Stop hv charging. en_hv:%d alg:%s alg_vbus:%d\n",
					__func__, info->enable_hv_charging,
					dev_name(&alg->dev), val);
			}
		}
	}
	info->is_chg_done = chg_done;

	if (is_basic == true) {
		charger_dev_set_input_current(info->chg1_dev,
			pdata->input_current_limit);
		charger_dev_set_charging_current(info->chg1_dev,
			pdata->charging_current_limit);
		info->lst_rnd_alg_idx = -1;

		chr_debug("%s:old_cv=%d,cv=%d, vbat_mon_en=%d\n",
			__func__,
			info->old_cv,
			info->setting.cv,
			info->setting.vbat_mon_en);
		if (info->old_cv == 0 || (info->old_cv != info->setting.cv)
		    || info->setting.vbat_mon_en == 0) {
			charger_dev_enable_6pin_battery_charging(
				info->chg1_dev, false);
			charger_dev_set_constant_voltage(info->chg1_dev,
				info->setting.cv);
			if (info->setting.vbat_mon_en && info->stop_6pin_re_en != 1)
				charger_dev_enable_6pin_battery_charging(
					info->chg1_dev, true);
			info->old_cv = info->setting.cv;
		} else {
			if (info->setting.vbat_mon_en && info->stop_6pin_re_en != 1) {
				info->stop_6pin_re_en = 1;
				charger_dev_enable_6pin_battery_charging(
					info->chg1_dev, true);
			}
		}
	}

	if (pdata->input_current_limit == 0 ||
	    pdata->charging_current_limit == 0)
		charger_dev_enable(info->chg1_dev, false);
	else {
		alg = get_chg_alg_by_name("pe5p");
		ret = chg_alg_is_algo_ready(alg);
		alg = get_chg_alg_by_name("pe5");
		ret2 = chg_alg_is_algo_ready(alg);
		alg = get_chg_alg_by_name("hvbp");
		ret3 = chg_alg_is_algo_ready(alg);
		alg = get_chg_alg_by_name("pehv");
		ret4 = chg_alg_is_algo_ready(alg);
		if (!(ret == ALG_READY || ret == ALG_RUNNING) &&
			!(ret2 == ALG_READY || ret2 == ALG_RUNNING) &&
			!(ret3 == ALG_READY || ret3 == ALG_RUNNING) &&
			!(ret4 == ALG_READY || ret4 == ALG_RUNNING))
			charger_dev_enable(info->chg1_dev, true);
	}

	if (info->chg1_dev != NULL) {
		charger_dev_dump_registers(info->chg1_dev);
		charger_dev_kick_wdt(info->chg1_dev);
	}

	if (info->chg2_dev != NULL) {
		charger_dev_dump_registers(info->chg2_dev);
		charger_dev_kick_wdt(info->chg2_dev);
	}

	if (info->bkbstchg_dev != NULL)
		charger_dev_dump_registers(info->bkbstchg_dev);

	return 0;
}

static int enable_charging(struct mtk_charger *info,
						bool en)
{
	int i;
	struct chg_alg_device *alg;


	chr_err("%s %d\n", __func__, en);

	if (en == false) {
		for (i = 0; i < MAX_ALG_NO; i++) {
			alg = info->alg[i];
			if (alg == NULL)
				continue;
			chg_alg_stop_algo(alg);
		}
		charger_dev_enable(info->chg1_dev, false);
		charger_dev_do_event(info->chg1_dev, EVENT_DISCHARGE, 0);
	} else {
		charger_dev_enable(info->chg1_dev, true);
		charger_dev_do_event(info->chg1_dev, EVENT_RECHARGE, 0);
	}

	return 0;
}

static int charger_dev_event(struct notifier_block *nb, unsigned long event,
				void *v)
{
	struct chg_alg_device *alg;
	struct chg_alg_notify notify;
	struct mtk_charger *info =
			container_of(nb, struct mtk_charger, chg1_nb);
	struct chgdev_notify *data = v;
	int i;

	chr_err("%s %lu\n", __func__, event);

	switch (event) {
	case CHARGER_DEV_NOTIFY_EOC:
		info->stop_6pin_re_en = 1;
		notify.evt = EVT_FULL;
		notify.value = 0;
		for (i = 0; i < 10; i++) {
			alg = info->alg[i];
			chg_alg_notifier_call(alg, &notify);
		}

		break;
	case CHARGER_DEV_NOTIFY_RECHG:
		pr_info("%s: recharge\n", __func__);
		break;
	case CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT:
		info->safety_timeout = true;
		pr_info("%s: safety timer timeout\n", __func__);
		break;
	case CHARGER_DEV_NOTIFY_VBUS_OVP:
		info->vbusov_stat = data->vbusov_stat;
		pr_info("%s: vbus ovp = %d\n", __func__, info->vbusov_stat);
		break;
	case CHARGER_DEV_NOTIFY_BATPRO_DONE:
		info->batpro_done = true;
		info->setting.vbat_mon_en = 0;
		notify.evt = EVT_BATPRO_DONE;
		notify.value = 0;
		for (i = 0; i < 10; i++) {
			alg = info->alg[i];
			chg_alg_notifier_call(alg, &notify);
		}
		pr_info("%s: batpro_done = %d\n", __func__, info->batpro_done);
		break;
	case CHARGER_DEV_NOTIFY_DPDM_OVP:
		info->dpdmov_stat = data->dpdmov_stat;
		pr_info("%s: DPDM ovp = %d\n", __func__, info->dpdmov_stat);
		break;
	case CHARGER_DEV_NOTIFY_INFO_SYNC:
		pr_info("%s: INFO SYNC\n", __func__);
		break;
	case CHARGER_DEV_NOTIFY_CTD_DONE:
		pr_info("%s: CTD done\n", __func__);
		schedule_work(&info->mmi.notify_power_event_work);
		break;
	default:
		return NOTIFY_DONE;
	}

	if (info->chg1_dev->is_polling_mode == false)
		_wake_up_charger(info);

	return NOTIFY_DONE;
}

static int to_alg_notify_evt(unsigned long evt)
{
	switch (evt) {
	case CHARGER_DEV_NOTIFY_VBUS_OVP:
		return EVT_VBUSOVP;
	case CHARGER_DEV_NOTIFY_IBUSOCP:
		return EVT_IBUSOCP;
	case CHARGER_DEV_NOTIFY_IBUSUCP_FALL:
		return EVT_IBUSUCP_FALL;
	case CHARGER_DEV_NOTIFY_BAT_OVP:
		return EVT_VBATOVP;
	case CHARGER_DEV_NOTIFY_IBATOCP:
		return EVT_IBATOCP;
	case CHARGER_DEV_NOTIFY_VBATOVP_ALARM:
		return EVT_VBATOVP_ALARM;
	case CHARGER_DEV_NOTIFY_VBUSOVP_ALARM:
		return EVT_VBUSOVP_ALARM;
	case CHARGER_DEV_NOTIFY_VOUTOVP:
		return EVT_VOUTOVP;
	case CHARGER_DEV_NOTIFY_VDROVP:
		return EVT_VDROVP;
	default:
		return -EINVAL;
	}
}

static int dvchg1_dev_event(struct notifier_block *nb, unsigned long event,
			    void *data)
{
	struct mtk_charger *info =
		container_of(nb, struct mtk_charger, dvchg1_nb);
	int alg_evt = to_alg_notify_evt(event);

	chr_info("%s %ld", __func__, event);
	if (alg_evt < 0)
		return NOTIFY_DONE;
	mtk_chg_alg_notify_call(info, alg_evt, 0);
	return NOTIFY_OK;
}

static int dvchg2_dev_event(struct notifier_block *nb, unsigned long event,
			    void *data)
{
	struct mtk_charger *info =
		container_of(nb, struct mtk_charger, dvchg2_nb);
	int alg_evt = to_alg_notify_evt(event);

	chr_info("%s %ld", __func__, event);
	if (alg_evt < 0)
		return NOTIFY_DONE;
	mtk_chg_alg_notify_call(info, alg_evt, 0);
	return NOTIFY_OK;
}

static int hvdvchg1_dev_event(struct notifier_block *nb, unsigned long event,
			      void *data)
{
	struct mtk_charger *info =
		container_of(nb, struct mtk_charger, hvdvchg1_nb);
	int alg_evt = to_alg_notify_evt(event);

	chr_info("%s %ld", __func__, event);
	if (alg_evt < 0)
		return NOTIFY_DONE;
	mtk_chg_alg_notify_call(info, alg_evt, 0);
	return NOTIFY_OK;
}

static int hvdvchg2_dev_event(struct notifier_block *nb, unsigned long event,
			      void *data)
{
	struct mtk_charger *info =
		container_of(nb, struct mtk_charger, hvdvchg2_nb);
	int alg_evt = to_alg_notify_evt(event);

	chr_info("%s %ld", __func__, event);
	if (alg_evt < 0)
		return NOTIFY_DONE;
	mtk_chg_alg_notify_call(info, alg_evt, 0);
	return NOTIFY_OK;
}

#define MMI_MUX(_mos1,  _mos2, _boost, _switch, _chipstate) \
{ \
	.typec_mos = _mos1, \
	.wls_mos = _mos2, \
	.wls_boost_en = _boost, \
	.wls_loadswtich_en = _switch, \
	.wls_chip_en = _chipstate, \
}

static const struct mmi_mux_configure config_mmi_mux[MMI_MUX_CHANNEL_MAX] = {
	[MMI_MUX_CHANNEL_NONE] = MMI_MUX(MMI_DVCHG_MUX_CLOSE, MMI_DVCHG_MUX_CLOSE, false, false, true),
	[MMI_MUX_CHANNEL_TYPEC_CHG] = MMI_MUX(MMI_DVCHG_MUX_CHG_OPEN, MMI_DVCHG_MUX_CLOSE, false, false, false),
	[MMI_MUX_CHANNEL_TYPEC_OTG] = MMI_MUX(MMI_DVCHG_MUX_OTG_OPEN, MMI_DVCHG_MUX_CLOSE, false, true, false),
	[MMI_MUX_CHANNEL_WLC_CHG] = MMI_MUX(MMI_DVCHG_MUX_CLOSE, MMI_DVCHG_MUX_CHG_OPEN, false, false, true),
	[MMI_MUX_CHANNEL_WLC_OTG] = MMI_MUX(MMI_DVCHG_MUX_DISABLE, MMI_DVCHG_MUX_DISABLE, true, false, true),
	[MMI_MUX_CHANNEL_TYPEC_CHG_WLC_OTG] = MMI_MUX(MMI_DVCHG_MUX_CHG_OPEN, MMI_DVCHG_MUX_CLOSE, true, true, true),
	[MMI_MUX_CHANNEL_TYPEC_CHG_WLC_CHG] = MMI_MUX(MMI_DVCHG_MUX_CHG_OPEN, MMI_DVCHG_MUX_CLOSE, false, false, false),
	[MMI_MUX_CHANNEL_TYPEC_OTG_WLC_CHG] = MMI_MUX(MMI_DVCHG_MUX_OTG_OPEN, MMI_DVCHG_MUX_CLOSE, false, true, false),
	[MMI_MUX_CHANNEL_TYPEC_OTG_WLC_OTG] = MMI_MUX(MMI_DVCHG_MUX_OTG_OPEN, MMI_DVCHG_MUX_CLOSE,  false, false, false),
#ifdef CONFIG_MOTO_CHANNEL_SWITCH
	[MMI_MUX_CHANNEL_WLC_FW_UPDATE] = MMI_MUX(MMI_DVCHG_MUX_DISABLE, MMI_DVCHG_MUX_CHG_OPEN, false, false, false),
#else
	[MMI_MUX_CHANNEL_WLC_FW_UPDATE] = MMI_MUX(MMI_DVCHG_MUX_DISABLE, MMI_DVCHG_MUX_DISABLE, false, false, false),
#endif
	[MMI_MUX_CHANNEL_WLC_FACTORY_TEST] = MMI_MUX(MMI_DVCHG_MUX_CLOSE, MMI_DVCHG_MUX_MANUAL_OPEN, false, false, true),
};

static int mmi_mux_config(struct mtk_charger *info, enum mmi_mux_channel channel)
{
	if (info->dvchg1_dev == NULL) {
		info->dvchg1_dev = get_charger_by_name("primary_dvchg");
		if (info->dvchg1_dev)
			pr_info("mmi_mux_config Found primary divider charger\n");
		else {
			chr_err("*** Error : can't find primary divider charger ***\n");
		}
	}

	if (!info->mmi.factory_mode) {
		struct chg_alg_device *alg;

		alg = get_chg_alg_by_name("wlc");
		if ((NULL != alg) && (alg->alg_id & info->fast_charging_indicator))
			chg_alg_set_prop(alg, ALG_WLC_STATE, config_mmi_mux[channel].wls_chip_en);
	}
	charger_dev_config_mux(info->dvchg1_dev,
		config_mmi_mux[channel].typec_mos, config_mmi_mux[channel].wls_mos);
	if(gpio_is_valid(info->mmi.wls_boost_en))
		gpio_set_value(info->mmi.wls_boost_en, config_mmi_mux[channel].wls_boost_en);
	if(gpio_is_valid(info->mmi.wls_switch_en))
		gpio_set_value(info->mmi.wls_switch_en, config_mmi_mux[channel].wls_loadswtich_en);

	return 0;
}

static int mmi_wlc_set_prop(struct mtk_charger *info, enum chg_alg_props s, int en)
{
	struct chg_alg_device *alg = NULL;

	alg = get_chg_alg_by_name("wlc");
	if (!IS_ERR_OR_NULL(alg) && (alg->alg_id & info->fast_charging_indicator)) {
		pr_info("[%s] %d %d\n", __func__, s, en);
		chg_alg_set_prop(alg, s, en);
	} else {
		pr_info("[%s] not found wlc\n", __func__);
	}

	return 0;
}

static int mmi_switch_enn_config(struct mtk_charger *info, enum mmi_mux_channel channel, bool en)
{
	struct chg_alg_device *alg = NULL;

	pr_info("%s en=%d factory_mode=%d\n", __func__, en, info->mmi.factory_mode);
	if (info->mmi.factory_mode) {
		alg = get_chg_alg_by_name("wlc");
		if ((NULL != alg) && (alg->alg_id & info->fast_charging_indicator)) {
			if (gpio_is_valid(info->mmi.switch_enn_en)) {
				gpio_set_value(info->mmi.switch_enn_en, en);
				pr_info("[%s] switch_enn_en %d\n", __func__, en);
				chg_alg_set_prop(alg, ALG_WLC_STATE, en);
			}
#ifdef CONFIG_MOTO_CHANNEL_SWITCH
			if(en)
				info->wls_tcmd_test = true;
			else
				info->wls_tcmd_test = false;
			pr_info("Factory tcmd test en = %d\n", en);
			chg_alg_set_prop(alg, ALG_WLC_STATE, en);
#endif
		} else {
			pr_info("[%s] not found wlc\n", __func__);
		}
	}

	return 0;
}

#ifdef CONFIG_MOTO_CHANNEL_SWITCH
extern void wls_tcmd_set_role_device(void);
#endif
static int mmi_mux_switch(struct mtk_charger *info, enum mmi_mux_channel channel, bool on)
{
	int pre_chan, pre_on;

	if(!info->mmi.enable_mux)
		return 0;

	mutex_lock(&info->mmi_mux_lock);
	pre_chan =  info->mmi.mux_channel.chan;
	pre_on = info->mmi.mux_channel.on;
	if (pre_chan == channel && pre_on == on) {
		mutex_unlock(&info->mmi_mux_lock);
		return 0;
	}
	switch (channel) {
		case MMI_MUX_CHANNEL_NONE:
			break;
		case MMI_MUX_CHANNEL_TYPEC_CHG:
			if (on) {
				if (pre_chan == MMI_MUX_CHANNEL_WLC_CHG) {
					mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_CHG_WLC_CHG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_CHG_WLC_CHG;
					info->mmi.mux_channel.on = true;
				} else if (pre_chan == MMI_MUX_CHANNEL_WLC_OTG) {
					if (info->wls_boost_using_otg) {
						mmi_wlc_set_prop(info, ALG_WLC_TX_MODE, 0);
						mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_CHG);
						info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_CHG;
					} else {
						mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_CHG_WLC_OTG);
						info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_CHG_WLC_OTG;
					}
					info->mmi.mux_channel.on = true;
				} else {
					mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_CHG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_CHG;
					info->mmi.mux_channel.on = true;
				}
			} else {
				if (pre_chan == MMI_MUX_CHANNEL_TYPEC_CHG_WLC_CHG) {
					mmi_mux_config(info, MMI_MUX_CHANNEL_WLC_CHG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_WLC_CHG;
					info->mmi.mux_channel.on = true;
				} else if (pre_chan == MMI_MUX_CHANNEL_TYPEC_CHG_WLC_OTG) {
					mmi_mux_config(info, MMI_MUX_CHANNEL_WLC_OTG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_WLC_OTG;
					info->mmi.mux_channel.on = true;
#ifdef CONFIG_MOTO_CHANNEL_SWITCH
				} else if (pre_chan == MMI_MUX_CHANNEL_WLC_FACTORY_TEST) {
					mmi_mux_config(info, MMI_MUX_CHANNEL_WLC_FACTORY_TEST);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_WLC_FACTORY_TEST;
					info->mmi.mux_channel.on = true;
#endif
				} else {
					mmi_mux_config(info, MMI_MUX_CHANNEL_NONE);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_NONE;
					info->mmi.mux_channel.on = false;
				}
			}
			break;
		case MMI_MUX_CHANNEL_TYPEC_OTG:
			if (on) {
				if (pre_chan == MMI_MUX_CHANNEL_WLC_CHG) {
					mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_OTG_WLC_CHG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_OTG_WLC_CHG;
					info->mmi.mux_channel.on = true;
				} else if (pre_chan == MMI_MUX_CHANNEL_WLC_OTG) {
					if (info->wls_boost_using_otg) {
						mmi_wlc_set_prop(info, ALG_WLC_TX_MODE, 0);
						mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_OTG);
						info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_OTG;
					} else {
						mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_OTG_WLC_OTG);
						info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_OTG_WLC_OTG;
					}
 					info->mmi.mux_channel.on = true;
				} else {
					mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_OTG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_OTG;
					info->mmi.mux_channel.on = true;
				}
			} else {
				if (pre_chan == MMI_MUX_CHANNEL_TYPEC_OTG_WLC_CHG) {
					mmi_mux_config(info, MMI_MUX_CHANNEL_WLC_CHG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_WLC_CHG;
					info->mmi.mux_channel.on = true;
				} else if (pre_chan == MMI_MUX_CHANNEL_TYPEC_OTG_WLC_OTG) {
					mmi_mux_config(info, MMI_MUX_CHANNEL_WLC_OTG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_WLC_OTG;
					info->mmi.mux_channel.on = true;
				} else {
					mmi_mux_config(info, MMI_MUX_CHANNEL_NONE);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_NONE;
					info->mmi.mux_channel.on = false;
				}
			}
			break;
		case MMI_MUX_CHANNEL_WLC_CHG:
			if (on) {
				if (pre_chan == MMI_MUX_CHANNEL_TYPEC_CHG) {
					mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_CHG_WLC_CHG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_CHG_WLC_CHG;
					info->mmi.mux_channel.on = true;
				} else if (pre_chan == MMI_MUX_CHANNEL_TYPEC_OTG) {
					mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_OTG_WLC_CHG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_OTG_WLC_CHG;
					info->mmi.mux_channel.on = true;
#ifdef CONFIG_MOTO_CHANNEL_SWITCH
				} else if (pre_chan == MMI_MUX_CHANNEL_WLC_FACTORY_TEST) {
					pr_info("Last chan is wls_factory_test");
					wls_tcmd_set_role_device();
					mmi_mux_config(info, MMI_MUX_CHANNEL_WLC_FACTORY_TEST);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_WLC_FACTORY_TEST;
					info->mmi.mux_channel.on = true;
					charger_dev_set_input_current(info->chg1_dev, info->data.wireless_factory_max_input_current);
#endif
				} else {
					mmi_mux_config(info, MMI_MUX_CHANNEL_WLC_CHG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_WLC_CHG;
					info->mmi.mux_channel.on = true;
				}
			} else {
				if (pre_chan == MMI_MUX_CHANNEL_TYPEC_CHG_WLC_CHG) {
					mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_CHG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_CHG;
					info->mmi.mux_channel.on = true;
				} else if (pre_chan == MMI_MUX_CHANNEL_TYPEC_OTG_WLC_CHG) {
					mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_OTG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_OTG;
					info->mmi.mux_channel.on = true;
				} else {
					mmi_mux_config(info, MMI_MUX_CHANNEL_NONE);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_NONE;
					info->mmi.mux_channel.on = false;
				}
			}
			break;
		case MMI_MUX_CHANNEL_WLC_OTG:
			if (on) {
				if (pre_chan == MMI_MUX_CHANNEL_TYPEC_CHG) {
					mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_CHG_WLC_OTG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_CHG_WLC_OTG;
					info->mmi.mux_channel.on = true;
				} else if (pre_chan == MMI_MUX_CHANNEL_TYPEC_OTG) {
					mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_OTG_WLC_OTG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_OTG_WLC_OTG;
					info->mmi.mux_channel.on = true;
				} else {
					mmi_mux_config(info, MMI_MUX_CHANNEL_WLC_OTG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_WLC_OTG;
					info->mmi.mux_channel.on = true;
				}
			} else {
				if (pre_chan == MMI_MUX_CHANNEL_TYPEC_CHG_WLC_OTG) {
					mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_CHG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_CHG;
					info->mmi.mux_channel.on = true;
				} else if (pre_chan == MMI_MUX_CHANNEL_TYPEC_OTG_WLC_OTG) {
					mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_OTG);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_OTG;
					info->mmi.mux_channel.on = true;
				} else {
					mmi_mux_config(info, MMI_MUX_CHANNEL_NONE);
					info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_NONE;
					info->mmi.mux_channel.on = false;
				}
			}
			break;
		case MMI_MUX_CHANNEL_WLC_FW_UPDATE:
			if (on) {
				mmi_mux_config(info, MMI_MUX_CHANNEL_WLC_FW_UPDATE);
				info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_WLC_FW_UPDATE;
			 } else {
				mmi_mux_config(info, MMI_MUX_CHANNEL_NONE);
				info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_NONE;
			 }
			info->mmi.mux_channel.on = on;
			break;
		case MMI_MUX_CHANNEL_WLC_FACTORY_TEST:
			pr_info("%s MMI_MUX_CHANNEL_WLC_FACTORY_TEST %d\n", __func__, on);
			mmi_switch_enn_config(info, MMI_MUX_CHANNEL_WLC_FACTORY_TEST, on);
			msleep(50);
			if (on) {
				mmi_mux_config(info, MMI_MUX_CHANNEL_WLC_FACTORY_TEST);
				info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_WLC_FACTORY_TEST;
			 } else {
				mmi_mux_config(info, MMI_MUX_CHANNEL_TYPEC_CHG);
				info->mmi.mux_channel.chan = MMI_MUX_CHANNEL_TYPEC_CHG;
			 }
			info->mmi.mux_channel.on = true;
			break;
		default:
			chr_err("[%s] Unknown channel: %d\n",
			__func__, channel);
	}

	chr_err("[%s] pre= %d,%d config = %d,%d result =%d,%d\n",
		__func__, pre_chan, pre_on, channel, on,
		info->mmi.mux_channel.chan,  info->mmi.mux_channel.on);
	mutex_unlock(&info->mmi_mux_lock);

	return 0;
}
int mtk_basic_charger_init(struct mtk_charger *info)
{
	info->algo.do_mux = mmi_mux_switch;
	info->algo.do_algorithm = do_algorithm;
	info->algo.enable_charging = enable_charging;
	info->algo.do_event = charger_dev_event;
	info->algo.do_dvchg1_event = dvchg1_dev_event;
	info->algo.do_dvchg2_event = dvchg2_dev_event;
	info->algo.do_hvdvchg1_event = hvdvchg1_dev_event;
	info->algo.do_hvdvchg2_event = hvdvchg2_dev_event;
	info->lst_rnd_alg_idx = -1;
	//info->change_current_setting = mtk_basic_charging_current;
	return 0;
}
