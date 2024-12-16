// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung UFS Feature Mimic for android14-6.1
 * commit 638804ea1ce06120f440cb308fcb24ad485e7d60
 * Date:   Thu Aug 31 00:23:28 2023 +0000
 *
 * from
 *
 * linux/drivers/ufs/core/ufshcd.c
 */

#include <ufs/ufshcd.h>
#include "../core/ufshcd-priv.h"
#include "ufsfeature.h"
#include <trace/hooks/ufshcd.h>
#include <linux/delay.h>

#if defined(CONFIG_UFSHID_V3)
/* Task management command timeout */
#define TM_CMD_TIMEOUT	100 /* msecs */

/**
 * ufsf_wait_for_register - wait for register value to change
 * @hba: per-adapter interface
 * @reg: mmio register offset
 * @mask: mask to apply to the read register value
 * @val: value to wait for
 * @interval_us: polling interval in microseconds
 * @timeout_ms: timeout in milliseconds
 *
 * Return:
 * -ETIMEDOUT on error, zero on success.
 */
int ufsf_wait_for_register(struct ufs_hba *hba, u32 reg, u32 mask,
			   u32 val, unsigned long interval_us,
			   unsigned long timeout_ms)
{
	int err = 0;
	unsigned long timeout = jiffies + msecs_to_jiffies(timeout_ms);

	/* ignore bits that we don't intend to wait on */
	val = val & mask;

	while ((ufshcd_readl(hba, reg) & mask) != val) {
		usleep_range(interval_us, interval_us + 50);
		if (time_after(jiffies, timeout)) {
			if ((ufshcd_readl(hba, reg) & mask) != val)
				err = -ETIMEDOUT;
			break;
		}
	}

	return err;
}

/**
 * ufsf_utmrl_clear - Clear a bit in UTRMLCLR register
 * @hba: per adapter instance
 * @pos: position of the bit to be cleared
 */
static inline void ufsf_utmrl_clear(struct ufs_hba *hba, u32 pos)
{
	if (hba->quirks & UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR)
		ufshcd_writel(hba, (1 << pos), REG_UTP_TASK_REQ_LIST_CLEAR);
	else
		ufshcd_writel(hba, ~(1 << pos), REG_UTP_TASK_REQ_LIST_CLEAR);
}

static int ufsf_clear_tm_cmd(struct ufs_hba *hba, int tag)
{
	int err = 0;
	u32 mask = 1 << tag;
	unsigned long flags;

	if (!test_bit(tag, &hba->outstanding_tasks))
		goto out;

	spin_lock_irqsave(hba->host->host_lock, flags);
	ufsf_utmrl_clear(hba, tag);
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	/* poll for max. 1 sec to clear door bell register by h/w */
	err = ufsf_wait_for_register(hba,
			REG_UTP_TASK_REQ_DOOR_BELL,
			mask, 0, 1000, 1000);

	dev_err(hba->dev, "Clearing task management function with tag %d %s\n",
		tag, err ? "succeeded" : "failed");

out:
	return err;
}

static void ufsf_add_tm_upiu_trace(struct ufs_hba *hba, unsigned int tag,
				   enum ufs_trace_str_t str_t)
{
	trace_android_vh_ufs_send_tm_command(hba, tag, (int)str_t);

	if (str_t == UFS_TM_SEND)
		dev_info(hba->dev, "%s: UFS_TSF_TM_INPUT\n", __func__);
	else
		dev_info(hba->dev, "%s: UFS_TSF_TM_OUTPUT\n", __func__);
}

static int __ufsf_issue_tm_cmd(struct ufs_hba *hba,
			       struct utp_task_req_desc *treq, u8 tm_function)
{
	struct request_queue *q = hba->tmf_queue;
	struct Scsi_Host *host = hba->host;
	DECLARE_COMPLETION_ONSTACK(wait);
	struct request *req;
	unsigned long flags;
	int task_tag, err;

	/*
	 * blk_mq_alloc_request() is used here only to get a free tag.
	 */
	req = blk_mq_alloc_request(q, REQ_OP_DRV_OUT, 0);
	if (IS_ERR(req))
		return PTR_ERR(req);

	req->end_io_data = &wait;
	ufshcd_hold(hba, false);

	spin_lock_irqsave(host->host_lock, flags);

	task_tag = req->tag;
	WARN_ONCE(task_tag < 0 || task_tag >= hba->nutmrs, "Invalid tag %d\n",
		  task_tag);
	hba->tmf_rqs[req->tag] = req;
	treq->upiu_req.req_header.dword_0 |= cpu_to_be32(task_tag);

	memcpy(hba->utmrdl_base_addr + task_tag, treq, sizeof(*treq));
	ufshcd_vops_setup_task_mgmt(hba, task_tag, tm_function);

	/* send command to the controller */
	__set_bit(task_tag, &hba->outstanding_tasks);

	ufshcd_writel(hba, 1 << task_tag, REG_UTP_TASK_REQ_DOOR_BELL);
	/* Make sure that doorbell is committed immediately */
	wmb();

	spin_unlock_irqrestore(host->host_lock, flags);

	ufsf_add_tm_upiu_trace(hba, task_tag, UFS_TM_SEND);

	/* wait until the task management command is completed */
	err = wait_for_completion_io_timeout(&wait,
			msecs_to_jiffies(TM_CMD_TIMEOUT));
	if (!err) {
		ufsf_add_tm_upiu_trace(hba, task_tag, UFS_TM_ERR);
		dev_err(hba->dev, "%s: task management cmd 0x%.2x timed-out\n",
				__func__, tm_function);
		if (ufsf_clear_tm_cmd(hba, task_tag))
			dev_WARN(hba->dev, "%s: unable to clear tm cmd (slot %d) after timeout\n",
					__func__, task_tag);
		err = -ETIMEDOUT;
	} else {
		err = 0;
		memcpy(treq, hba->utmrdl_base_addr + task_tag, sizeof(*treq));

		ufsf_add_tm_upiu_trace(hba, task_tag, UFS_TM_COMP);
	}

	spin_lock_irqsave(hba->host->host_lock, flags);
	hba->tmf_rqs[req->tag] = NULL;
	__clear_bit(task_tag, &hba->outstanding_tasks);
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	ufshcd_release(hba);
	blk_mq_free_request(req);

	return err;
}

/**
 * ufsf_issue_tm_cmd - issues task management commands to controller
 * @hba: per adapter instance
 * @lun_id: LUN ID to which TM command is sent
 * @task_id: task ID to which the TM command is applicable
 * @tm_function: task management function opcode
 * @tm_response: task management service response return value
 *
 * Returns non-zero value on error, zero on success.
 */
int ufsf_issue_tm_cmd(struct ufs_hba *hba, int lun_id, int task_id,
		      u8 tm_function, u8 *tm_response)
{
	struct utp_task_req_desc treq = { { 0 }, };
	enum utp_ocs ocs_value;
	int err;

	/* Configure task request descriptor */
	treq.header.dword_0 = cpu_to_le32(UTP_REQ_DESC_INT_CMD);
	treq.header.dword_2 = cpu_to_le32(OCS_INVALID_COMMAND_STATUS);

	/* Configure task request UPIU */
	treq.upiu_req.req_header.dword_0 = cpu_to_be32(lun_id << 8) |
				  cpu_to_be32(UPIU_TRANSACTION_TASK_REQ << 24);
	treq.upiu_req.req_header.dword_1 = cpu_to_be32(tm_function << 16);

	/*
	 * The host shall provide the same value for LUN field in the basic
	 * header and for Input Parameter.
	 */
	treq.upiu_req.input_param1 = cpu_to_be32(lun_id);
	treq.upiu_req.input_param2 = cpu_to_be32(task_id);

	err = __ufsf_issue_tm_cmd(hba, &treq, tm_function);
	if (err == -ETIMEDOUT)
		return err;

	ocs_value = le32_to_cpu(treq.header.dword_2) & MASK_OCS;
	if (ocs_value != OCS_SUCCESS)
		dev_err(hba->dev, "%s: failed, ocs = 0x%x\n",
				__func__, ocs_value);
	else if (tm_response)
		*tm_response = be32_to_cpu(treq.upiu_rsp.output_param1) &
				MASK_TM_SERVICE_RESP;
	return err;
}
#endif

void ufsf_scsi_unblock_requests(struct ufs_hba *hba)
{
	if (atomic_dec_and_test(&hba->scsi_block_reqs_cnt))
		scsi_unblock_requests(hba->host);
}

void ufsf_scsi_block_requests(struct ufs_hba *hba)
{
	if (atomic_inc_return(&hba->scsi_block_reqs_cnt) == 1)
		scsi_block_requests(hba->host);
}

static u32 ufsf_pending_cmds(struct ufs_hba *hba)
{
	const struct scsi_device *sdev;
	u32 pending = 0;

	lockdep_assert_held(hba->host->host_lock);
	__shost_for_each_device(sdev, hba->host)
		pending += sbitmap_weight(&sdev->budget_map);

	return pending;
}

int ufsf_wait_for_doorbell_clr(struct ufs_hba *hba, u64 wait_timeout_us)
{
	unsigned long flags;
	int ret = 0;
	u32 tm_doorbell;
	u32 tr_pending;
	bool timeout = false, do_last_check = false;
	ktime_t start;

	ufshcd_hold(hba, false);
	spin_lock_irqsave(hba->host->host_lock, flags);
	/*
	 * Wait for all the outstanding tasks/transfer requests.
	 * Verify by checking the doorbell registers are clear.
	 */
	start = ktime_get();
	do {
		if (hba->ufshcd_state != UFSHCD_STATE_OPERATIONAL) {
			ret = -EBUSY;
			goto out;
		}

		tm_doorbell = ufshcd_readl(hba, REG_UTP_TASK_REQ_DOOR_BELL);
		tr_pending = ufsf_pending_cmds(hba);
		if (!tm_doorbell && !tr_pending) {
			timeout = false;
			break;
		} else if (do_last_check) {
			break;
		}

		spin_unlock_irqrestore(hba->host->host_lock, flags);
		io_schedule_timeout(msecs_to_jiffies(20));
		if (ktime_to_us(ktime_sub(ktime_get(), start)) >
		    wait_timeout_us) {
			timeout = true;
			/*
			 * We might have scheduled out for long time so make
			 * sure to check if doorbells are cleared by this time
			 * or not.
			 */
			do_last_check = true;
		}
		spin_lock_irqsave(hba->host->host_lock, flags);
	} while (tm_doorbell || tr_pending);

	if (timeout) {
		dev_err(hba->dev,
			"%s: timedout waiting for doorbell to clear (tm=0x%x, tr=0x%x)\n",
			__func__, tm_doorbell, tr_pending);
		ret = -EBUSY;
	}
out:
	spin_unlock_irqrestore(hba->host->host_lock, flags);
	ufshcd_release(hba);
	return ret;
}
MODULE_LICENSE("GPL v2");
