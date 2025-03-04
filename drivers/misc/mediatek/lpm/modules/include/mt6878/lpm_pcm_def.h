/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 MediaTek Inc.
 */

#ifndef __LPM_PCM_DEF_H__
#define __LPM_PCM_DEF_H__

/* --- R12 Define --- */
#define R12_PCM_TIMER_B                       (1U << 0)
#define R12_TWAM_PMSR_DVFSRC_IRQ              (1U << 1)
#define R12_KP_IRQ_B                          (1U << 2)
#define R12_APWDT_EVENT_B                     (1U << 3)
#define R12_APXGPT1_EVENT_B                   (1U << 4)
#define R12_CONN2AP_SPM_WAKEUP_B              (1U << 5)
#define R12_EINT_EVENT_B                      (1U << 6)
#define R12_CONN_WDT_IRQ_B                    (1U << 7)
#define R12_CCIF0_EVENT_B                     (1U << 8)
#define R12_CCIF1_EVENT_B                     (1U << 9)
#define R12_SSPM2SPM_WAKEUP_B                 (1U << 10)
#define R12_SCP2SPM_WAKEUP_B                  (1U << 11)
#define R12_PCM_WDT_WAKEUP_B                  (1U << 13)
#define R12_USB0_CDSC_B                       (1U << 14)
#define R12_USB0_POWERDWN_B                   (1U << 15)
#define R12_UART2SPM_IRQ_B                    (1U << 17)
#define R12_SYSTIMER_EVENT_B                  (1U << 18)
#define R12_EINT_EVENT_SECURED                (1U << 19)
#define R12_AFE_IRQ_MCU_B                     (1U << 20)
#define R12_THERM_CTRL_EVENT_B                (1U << 21)
#define R12_SYS_CIRQ_IRQ_B                    (1U << 22)
#define R12_MD2AP_PEER_EVENT_B                (1U << 23)
#define R12_CSYSPWREQ_B                       (1U << 24)
#define R12_MD1_WDT_B                         (1U << 25)
#define R12_AP2AP_PEER_WAKEUPEVENT_B          (1U << 26)
#define R12_SEJ_EVENT_B                       (1U << 27)
#define R12_SPM_CPU_WAKEUPEVENT_B             (1U << 28)
#define R12_APUSYS_WAKE_HOST                  (1U << 29)
#define R12_MSDC_WAKE_B                       (1U << 31)

/* define WAKE_SRC_CONN2AP for conn use */
#define WAKE_SRC_CONN2AP WAKE_SRC_STA1_CONN2AP_SPM_WAKEUP_B

#endif /* __LPM_PCM_DEF_H__ */
