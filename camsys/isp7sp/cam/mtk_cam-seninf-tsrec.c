// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.


#ifndef FS_UT
#include <linux/interrupt.h>    /* for interrupt/irq */
#include <linux/delay.h>        /* for get ktime */
#include <linux/of.h>           /* for get property */
#include <linux/workqueue.h>    /* for workqueue */
#include <linux/kfifo.h>        /* for kfifo */
#include <linux/kthread.h>      /* for kthread */
#endif //!FS_UT

#include "mtk_cam-seninf-tsrec.h"
#include "mtk_cam-seninf-tsrec-regs.h"
#include "mtk_cam-seninf-tsrec-def.h"


#define PFX "TSREC"


/******************************************************************************
 * TSREC member structure/variables
 *****************************************************************************/
#ifndef FS_UT
struct device *seninf_dev;

DEFINE_SPINLOCK(tsrec_log_concurrency_lock);
#endif


/*---------------------------------------------------------------------------*/
// TSREC console mgr structure
/*---------------------------------------------------------------------------*/
struct tsrec_console_cfg_st {
	unsigned int en;
	unsigned int val;
};

enum tsrec_console_cmd_id {
	TSREC_CON_CMD_OVW_TSREC_HW_CNT = 1,
	TSREC_CON_CMD_OVW_EXP_TRIG_SRC,
	TSREC_CON_CMD_SET_INTR_EXP_EN_MASK,
	TSREC_CON_CMD_SET_INTR_EXP_TRIG_SRC_BOTH,
};

/*---------------------------------------------------------------------------*/

struct tsrec_console_mgr_st {
	unsigned int is_init;

	unsigned int tsrec_max_cnt;

	/* assign exp trig src setting => Vsync or 1st-Hsync */
	unsigned int exp_trig_src;

	unsigned int intr_exp_en_mask;
	unsigned int intr_exp_trig_src_both;
};
static struct tsrec_console_mgr_st tsrec_con_mgr;


/*---------------------------------------------------------------------------*/
// TSREC IRQ structure
/*---------------------------------------------------------------------------*/
enum tsrec_intr_status_rg_id {
	TSREC_RG_INTR_STATUS = 1,
	TSREC_RG_INTR_STATUS_2,
};


struct tsrec_irq_info_st {
	unsigned int intr_status;
	unsigned int intr_status_2;

	unsigned long long tsrec_ts_us;
	unsigned long long sys_ts_ns;
	unsigned long long duration_ns;
};

struct tsrec_irq_event_st {
#ifndef FS_UT
	/* for linux kfifo */
	void *msg_buffer;
	unsigned int fifo_size;
	tsrec_atomic_t is_fifo_overflow;
	struct kfifo msg_fifo;
#endif // !FS_UT
};
static struct tsrec_irq_event_st tsrec_irq_event;


/*---------------------------------------------------------------------------*/
// TSREC worker & work structure
/*---------------------------------------------------------------------------*/
enum tsrec_work_event_flag {
	TSREC_WORK_1ST_VSYNC = (1UL << 0),
	TSREC_WORK_QUERY_TS = (1UL << 1),
	TSREC_WORK_SEND_TS_TO_SENSOR = (1UL << 2),
	TSREC_WORK_SENSOR_HW_PRE_LATCH = (1UL << 3),
	TSREC_WORK_DBG_DUMP_IRQ_INFO = (1UL << 31),
};


struct tsrec_work_request {
#ifndef FS_UT
#if defined(TSREC_WORK_USING_KTHREAD)
	struct kthread_work work;
#else
	struct work_struct work;
#endif // TSREC_WORK_USING_KTHREAD
#endif // !FS_UT

	void *irq_dev_ctx;
	unsigned int tsrec_no;
	unsigned int work_event_info;
	struct tsrec_irq_info_st irq_info;
};

#ifndef FS_UT
struct tsrec_kthread_st {
	struct kthread_worker kthread;
	struct task_struct *kthread_task;
};

struct tsrec_worker_st {
	/* array size is by tsrec_hw_cnt got from dts */
	/* , each tsrec HW has its own workqueue */
	// TODO: add a atomic variable for record which tsrec alloc worker.
#if defined(TSREC_WORK_USING_KTHREAD)
	struct tsrec_kthread_st **kthreads;
#else
	struct workqueue_struct **workqs;
#endif // TSREC_WORK_USING_KTHREAD
};
static struct tsrec_worker_st tsrec_worker;
#endif // !FS_UT


/*---------------------------------------------------------------------------*/
// TSREC src structure
/*---------------------------------------------------------------------------*/
/* for each exp. e.g., exp0, exp1, exp2 */
struct tsrec_seninf_exp_vc_dt_st {
	unsigned int vsync_vc;
	unsigned int hsync_vc;
	unsigned int hsync_dt;

	/* 0:vsync / 1:hsync */
	unsigned int trig_src;
};

struct tsrec_seninf_cfg_st {
	/* currently, equal to array idx */
	unsigned int seninf_idx;
	/* before ISP8, equal to seninf mux idx */
	unsigned int tsrec_no;

	/* exp. & ext. src vc/dt settings */
	unsigned long long vc_dt_src_code;
	unsigned long long cust_vc_dt_src_code;
	struct tsrec_seninf_exp_vc_dt_st exp_vc_dt[TSREC_EXP_MAX_CNT];
	struct tsrec_seninf_exp_vc_dt_st cust_vc_dt[TSREC_EXP_MAX_CNT];

	/* which exp. should trigger sensor hw pre-latch event */
	unsigned int sensor_hw_pre_latch_exp_code;
};

/*---------------------------------------------------------------------------*/

struct tsrec_src_st {
	unsigned int is_init;

	/* for each tsrec settings */
	/* array size is by seninf_hw_cnt got from dts */
	struct tsrec_seninf_cfg_st **src_cfg;
};
static struct tsrec_src_st tsrec_src;


/*---------------------------------------------------------------------------*/
// TSREC status structure
/*---------------------------------------------------------------------------*/

struct tsrec_n_timestamp_exp_st {
	unsigned int curr_idx;
	unsigned long long ticks[TSREC_TS_REC_MAX_CNT];
};

struct tsrec_n_timestamp_st {
	unsigned int seninf_idx;

	unsigned long long curr_sys_time_ns;
	unsigned long long curr_tick;
	struct tsrec_n_timestamp_exp_st exp_recs[TSREC_EXP_MAX_CNT];
};

/*---------------------------------------------------------------------------*/

struct tsrec_n_regs_st {
	unsigned int seninf_idx;

	unsigned int en;
	unsigned int exp_vc_dt[TSREC_EXP_MAX_CNT];
	unsigned int trig_src;

	/* timestamp records */
	struct tsrec_n_timestamp_st ts_recs;

	/* record irq info for debugging */
	struct tsrec_irq_info_st irq_info;

	/* sensor hw pre-latch exp num */
	unsigned int sensor_hw_pre_latch_exp_num;
};

/*---------------------------------------------------------------------------*/

struct tsrec_status_st {
	unsigned int is_init;

	/* from dts, tsrec, seninf hw support count */
	unsigned int tsrec_hw_cnt;
	unsigned int seninf_hw_cnt;

	/* tsrec IRQ number & IRQ enable/disable status */
	int tsrec_irq;
	unsigned int tsrec_irq_en_status;

	/* tsrec timer cfg */
	tsrec_atomic_t timer_en_cnt;
	tsrec_atomic_t timer_cfg;
	unsigned int timer_en_status;

	/* tsrec top cfg */
	tsrec_atomic_t top_cfg;

	/* tsrec interrupt info */
	tsrec_atomic_t intr_en;
	tsrec_atomic_t intr_status;
	tsrec_atomic_t intr_en_2;
	tsrec_atomic_t intr_status_2;

	/* tsrec regs settings & timestamp records */
	/* array size is by tsrec_hw_cnt got from dts */
	struct tsrec_n_regs_st **tsrec_n_regs;
};
static struct tsrec_status_st tsrec_status;


/******************************************************************************
 * TSREC console functions
 *****************************************************************************/
static void tsrec_con_mgr_st_reset(void)
{
	/* clear struct data */
	memset(&tsrec_con_mgr, 0, sizeof(tsrec_con_mgr));

	/* !!! setup init/default value below !!! */
	tsrec_con_mgr.tsrec_max_cnt = 0xFFFFFFFF;
	tsrec_con_mgr.exp_trig_src = TSREC_EXP_TRIG_SRC_DEF;
	tsrec_con_mgr.intr_exp_en_mask = TSREC_INTR_EXP_EN_MASK;
	tsrec_con_mgr.intr_exp_trig_src_both = 0;
}


static void tsrec_con_mgr_st_init(void)
{
	if (unlikely(tsrec_con_mgr.is_init)) {
		TSREC_LOG_INF(
			"NOTICE: already init:%u, skip reset structure data\n",
			tsrec_con_mgr.is_init);
		return;
	}

	tsrec_con_mgr_st_reset();

	/* mark as inited */
	tsrec_con_mgr.is_init = 1;
}


static unsigned int tsrec_con_mgr_decode_cmd_value(const unsigned int cmd,
	const unsigned int base, const unsigned int mod)
{
	unsigned int ret = 0;

	ret = (base != 0) ? (cmd / base) : 0;
	ret %= mod;

	return ret;
}


static enum tsrec_console_cmd_id tsrec_con_mgr_g_cmd_id(
	const unsigned int cmd)
{
	return (enum tsrec_console_cmd_id)tsrec_con_mgr_decode_cmd_value(cmd,
		TSREC_CON_CMD_ID_BASE, TSREC_CON_CMD_ID_MOD);
}


static void tsrec_con_mgr_s_cmd_value(const unsigned int cmd,
	unsigned int *p_val)
{
	*p_val = tsrec_con_mgr_decode_cmd_value(cmd,
		TSREC_CON_CMD_VAL_BASE, TSREC_CON_CMD_VAL_MOD);
}


static void tsrec_con_mgr_setup_tsrec_max_cnt(const unsigned int cmd)
{
	const unsigned int tsrec_hw_cnt = tsrec_status.tsrec_hw_cnt;
	const unsigned int val = tsrec_con_mgr_decode_cmd_value(cmd,
		TSREC_CON_CMD_VAL_BASE, TSREC_CON_CMD_VAL_MOD);

	tsrec_con_mgr.tsrec_max_cnt = (val < tsrec_hw_cnt)
		? val : tsrec_hw_cnt;
}


static void tsrec_con_mgr_process_cmd(const unsigned int cmd)
{
	enum tsrec_console_cmd_id cmd_id = 0;

	cmd_id = tsrec_con_mgr_g_cmd_id(cmd);
	switch (cmd_id) {
	case TSREC_CON_CMD_OVW_TSREC_HW_CNT:
		tsrec_con_mgr_setup_tsrec_max_cnt(cmd);
		break;

	case TSREC_CON_CMD_OVW_EXP_TRIG_SRC:
		tsrec_con_mgr_s_cmd_value(cmd,
			&tsrec_con_mgr.exp_trig_src);
		break;

	case TSREC_CON_CMD_SET_INTR_EXP_EN_MASK:
		tsrec_con_mgr_s_cmd_value(cmd,
			&tsrec_con_mgr.intr_exp_en_mask);
		break;

	case TSREC_CON_CMD_SET_INTR_EXP_TRIG_SRC_BOTH:
		tsrec_con_mgr_s_cmd_value(cmd,
			&tsrec_con_mgr.intr_exp_trig_src_both);
		break;

	default:
		break;
	}
}


/******************************************************************************
 * TSREC basic/utilities functions
 *****************************************************************************/
/*
 * return:
 *      @negative: invalid
 *      @0: ptr is valid for using.
 */
static int g_tsrec_seninf_cfg_st(const unsigned int idx,
	struct tsrec_seninf_cfg_st **ptr, const char *caller)
{
	const unsigned int seninf_hw_cnt = tsrec_status.seninf_hw_cnt;

	/* check case / error handling */
	if (unlikely(ptr == NULL)) {
		TSREC_LOG_INF(
			"[%s] ERROR: get non-valid input, ptr is nullptr, return   [idx:%u]\n",
			caller, idx);
		return -1;
	}

	if (unlikely(idx >= seninf_hw_cnt)) {
		TSREC_LOG_INF(
			"[%s] ERROR: non-valid seninf_idx:%u (DTS seninf_hw_cnt:%u), return\n",
			caller, idx, seninf_hw_cnt);
		*ptr = NULL;
		return -2;
	}

	if (unlikely(tsrec_src.src_cfg == NULL)) {
		TSREC_LOG_INF(
			"[%s] ERROR: src_cfg[] array is null, get seninf_idx:%u, DTS seninf_hw_cnt:%u, return\n",
			caller, idx, seninf_hw_cnt);
		*ptr = NULL;
		return -3;
	}

	/* check done, valid for get struct pointer */
	*ptr = tsrec_src.src_cfg[idx];
	return 0;
}


/*
 * return:
 *      @negative: invalid
 *      @0: ptr is valid for using.
 */
static int g_tsrec_n_regs_st(const unsigned int tsrec_no,
	struct tsrec_n_regs_st **ptr, const char *caller)
{
	const unsigned int tsrec_hw_cnt = tsrec_status.tsrec_hw_cnt;

	/* check case / error handling */
	if (unlikely(ptr == NULL)) {
		TSREC_LOG_INF(
			"[%s] ERROR: get non-valid input, ptr is nullptr, return   [tsrec_no:%u]\n",
			caller, tsrec_no);
		return -1;
	}

	if (unlikely(tsrec_no >= tsrec_hw_cnt)) {
		TSREC_LOG_INF(
			"[%s] ERROR: non-valid tsrec_no:%u (DTS tsrec_hw_cnt:%u), return\n",
			caller, tsrec_no, tsrec_hw_cnt);
		*ptr = NULL;
		return -2;
	}

	if (unlikely(tsrec_status.tsrec_n_regs == NULL)) {
		TSREC_LOG_INF(
			"[%s] ERROR: tsrec_n_regs[] array is null, get tsrec_no:%u, DTS tsrec_hw_cnt:%u, return\n",
			caller, tsrec_no, tsrec_hw_cnt);
		*ptr = NULL;
		return -3;
	}

	/* check done, valid for get struct pointer */
	*ptr = tsrec_status.tsrec_n_regs[tsrec_no];
	return 0;
}


unsigned int get_tsrec_timer_en_status(void)
{
	// return TSREC_ATOMIC_READ(&tsrec_status.timer_en_cnt);
	return tsrec_status.timer_en_status;
}


static unsigned int g_tsrec_exp_trig_src(void)
{
	return (unlikely(tsrec_con_mgr.exp_trig_src != TSREC_EXP_TRIG_SRC_DEF))
		? (tsrec_con_mgr.exp_trig_src)
		: TSREC_EXP_TRIG_SRC_DEF;
}


/*
 * for preventing any unexpected flow when TSREC HW does NOT exist,
 * plz call this function before every APIs that export to user module
 * (non static functions for SENINF drv / route / etc.).
 */
unsigned int chk_exist_tsrec_hw(const char *caller,
	const unsigned int do_print_log)
{
	/* !!! NO TSREC / DISABLE TSREC cases !!! */
	/* dts set TSREC HW cnt to ZERO */
	if (unlikely(tsrec_status.tsrec_hw_cnt == 0)) {
		if (do_print_log) {
			TSREC_LOG_INF(
				"[%s] NOTICE: TSREC HW does NOT exist (get property from DTS of tsrec_hw_cnt:%u), return 0\n",
				caller, tsrec_status.tsrec_hw_cnt);
		}
		return 0;
	}

#if defined(FORCE_DISABLE_TSREC)
	/* compile option for disable TSREC */
	if (do_print_log) {
		TSREC_LOG_INF(
			"[%s] WARNING: TSREC HW is turned OFF by compile option (FORCE_DISABLE_TSREC), tsrec_hw_cnt:%u, return 0\n",
			caller, tsrec_status.tsrec_hw_cnt);
	}
	return 0;
#endif

	return 1;
}


unsigned int chk_tsrec_no_valid(const unsigned int tsrec_no,
	const char *caller)
{
	const unsigned int tsrec_hw_cnt = tsrec_status.tsrec_hw_cnt;

	/* !!! NO TSREC / DISABLE TSREC cases !!! */
	/* dts set TSREC HW cnt to ZERO */
	if (unlikely(tsrec_status.tsrec_hw_cnt == 0)) {
		TSREC_LOG_INF(
			"[%s] NOTICE: TSREC HW does NOT exist (get property from DTS of tsrec_hw_cnt:%u), get tsrec_no:%u, return 0\n",
			caller, tsrec_hw_cnt, tsrec_no);
		return 0;
	}

	/* USER assign TSREC HW cnt to ZERO for using <=> sane as disable TSREC */
	if (unlikely(tsrec_con_mgr.tsrec_max_cnt == 0)) {
		TSREC_LOG_INF(
			"[%s] WARNING: USER manually set tsrec_max_cnt:%u(tsrec_hw_cnt:%u), get tsrec_no:%u, return 0\n",
			caller,
			tsrec_con_mgr.tsrec_max_cnt,
			tsrec_hw_cnt,
			tsrec_no);
		return 0;
	}

	/* USER assign TSREC HW cnt less than tsrec valid cnt. */
	if (unlikely(tsrec_con_mgr.tsrec_max_cnt < tsrec_hw_cnt)) {
		if (tsrec_no >= tsrec_con_mgr.tsrec_max_cnt) {
			TSREC_LOG_INF(
				"[%s] WARNING: USER manually set tsrec_max_cnt:%u(tsrec_hw_cnt:%u), get tsrec_no:%u, return 0\n",
				caller,
				tsrec_con_mgr.tsrec_max_cnt,
				tsrec_hw_cnt,
				tsrec_no);
			return 0;
		}
		return 1;
	}

	return (tsrec_no < tsrec_hw_cnt) ? 1 : 0;
}


unsigned int mtk_cam_seninf_g_tsrec_current_time(
	unsigned long long *p_tick, unsigned long long *p_time_us)
{
	unsigned long long tick = 0;

	if (unlikely(!chk_exist_tsrec_hw(__func__, 1)))
		return 0;

	/* case handling for tsrec timer not enabled */
	/* , these data maybe dirty value */
	if (p_tick != NULL)
		*p_tick = 0;
	if (p_time_us != NULL)
		*p_time_us = 0;

	if (tsrec_status.timer_en_status == 0) {
		TSREC_LOG_INF(
			"NOTICE: tsrec timer is not enable, plz enable tsrec timer first, return\n");
		return 0;
	}

	tick = mtk_cam_seninf_tsrec_latch_time();

	/* update data for caller */
	if (p_tick != NULL)
		*p_tick = tick;
	if (p_time_us != NULL)
		*p_time_us = TSREC_TICK_TO_US(tick);

	return 1;
}


/******************************************************************************
 * TSREC-REGS notify/get status functions
 *****************************************************************************/
void notify_tsrec_update_top_cfg(const unsigned int val)
{
	TSREC_ATOMIC_SET((int)val, &tsrec_status.top_cfg);
}


void notify_tsrec_update_timer_en_status(const unsigned int val)
{
	tsrec_status.timer_en_status = (val) ? 1 : 0;
}

void notify_tsrec_update_timer_cfg(const unsigned int val)
{
	TSREC_ATOMIC_SET((int)val, &tsrec_status.timer_cfg);
}


unsigned int notify_tsrec_get_timer_en_status(void)
{
	return tsrec_status.timer_en_status;
}


void notify_tsrec_update_intr_en(const unsigned int val)
{
	TSREC_ATOMIC_SET((int)val, &tsrec_status.intr_en);
}


void notify_tsrec_update_intr_status(const unsigned int val)
{
	TSREC_ATOMIC_SET((int)val, &tsrec_status.intr_status);
}


void notify_tsrec_update_intr_en_2(const unsigned int val)
{
	TSREC_ATOMIC_SET((int)val, &tsrec_status.intr_en_2);
}


void notify_tsrec_update_intr_status_2(const unsigned int val)
{
	TSREC_ATOMIC_SET((int)val, &tsrec_status.intr_status_2);
}


void notify_tsrec_update_tsrec_n_clk_en_status(const unsigned int tsrec_no,
	const unsigned int val)
{
	struct tsrec_n_regs_st *ptr = NULL;
	int ret;

	ret = g_tsrec_n_regs_st(tsrec_no, &ptr, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(ptr == NULL)) {
		TSREC_LOG_INF(
			"ERROR: tsrec_n_regs[%u] is nullptr (%p), return   [tsrec_no:%u, val:%u]\n",
			tsrec_no, ptr,
			tsrec_no, val);
		return;
	}

	ptr->en = (val) ? 1 : 0;
}


void notify_tsrec_update_tsrec_n_trig_src(const unsigned int tsrec_no,
	const unsigned int reg_val)
{
	struct tsrec_n_regs_st *ptr = NULL;
	int ret;

	ret = g_tsrec_n_regs_st(tsrec_no, &ptr, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(ptr == NULL)) {
		TSREC_LOG_INF(
			"ERROR: tsrec_n_regs[%u] is nullptr (%p), return   [tsrec_no:%u, reg_val:%#x]\n",
			tsrec_no, ptr,
			tsrec_no, reg_val);
		return;
	}

	ptr->trig_src = reg_val;
}


void notify_tsrec_update_tsrec_n_exp_vc_dt(const unsigned int tsrec_no,
	const unsigned int exp_n, const unsigned int reg_val)
{
	struct tsrec_n_regs_st *ptr = NULL;
	int ret;

	ret = g_tsrec_n_regs_st(tsrec_no, &ptr, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(ptr == NULL)) {
		TSREC_LOG_INF(
			"ERROR: tsrec_n_regs[%u] is nullptr (%p), return   [tsrec_no:%u, exp_n:%u, reg_val:%#x]\n",
			tsrec_no, ptr,
			tsrec_no, exp_n, reg_val);
		return;
	}

	ptr->exp_vc_dt[exp_n] = reg_val;
}


/******************************************************************************
 * TSREC static members/flow/logic CTRL functions
 *****************************************************************************/

/*---------------------------------------------------------------------------*/
// tsrec data structure reset, init, uninit
/*---------------------------------------------------------------------------*/
static unsigned int tsrec_chk_irq_info_msgfifo_not_empty(void)
{
#ifndef FS_UT
	return (kfifo_len(&tsrec_irq_event.msg_fifo)
		>= sizeof(struct tsrec_irq_info_st));
#else
	return ut_fs_tsrec_chk_fifo_not_empty();
#endif // !FS_UT
}


static void tsrec_pop_irq_info_msgfifo(struct tsrec_irq_info_st *irq_info)
{
#ifndef FS_UT
	unsigned int len;

	len = kfifo_out(
		&tsrec_irq_event.msg_fifo, irq_info, sizeof(*irq_info));

	WARN_ON(len != sizeof(*irq_info));
#else
	ut_fs_tsrec_fifo_out(
		&irq_info->intr_status, &irq_info->intr_status_2,
		&irq_info->tsrec_ts_us);
#endif // !FS_UT
}


static int tsrec_push_irq_info_msgfifo(struct tsrec_irq_info_st *irq_info)
{
#ifndef FS_UT
	int len;

	if (unlikely(kfifo_avail(
			&tsrec_irq_event.msg_fifo) < sizeof(*irq_info))) {
		TSREC_ATOMIC_SET(1, &tsrec_irq_event.is_fifo_overflow);
		return -1;
	}

	len = kfifo_in(&tsrec_irq_event.msg_fifo, irq_info, sizeof(*irq_info));

	WARN_ON(len != sizeof(*irq_info));
#else
	ut_fs_tsrec_fifo_in(
		irq_info->intr_status, irq_info->intr_status_2,
		irq_info->tsrec_ts_us);
#endif // !FS_UT

	return 0;
}

/*---------------------------------------------------------------------------*/

static void tsrec_irq_event_st_reset(void)
{
#ifndef FS_UT
	int ret = 0;

	TSREC_ATOMIC_INIT(0, &tsrec_irq_event.is_fifo_overflow);

	if (unlikely(tsrec_irq_event.msg_buffer == NULL)) {
		tsrec_irq_event.msg_buffer = TSREC_KZALLOC(tsrec_irq_event.fifo_size);

		if (unlikely(tsrec_irq_event.msg_buffer == NULL)) {
			TSREC_LOG_INF(
				"ERROR: irq msg_buffer:%p allocate memory failed, fifo_size:%u(%u/%u/%lu), return\n",
				tsrec_irq_event.msg_buffer,
				tsrec_irq_event.fifo_size,
				tsrec_status.tsrec_hw_cnt,
				TSREC_EXP_MAX_CNT,
				sizeof(struct tsrec_irq_info_st));
		}
		// WARN_ON(tsrec_irq_event.msg_buffer == NULL);
		return;
	}

	ret = kfifo_init(
		&tsrec_irq_event.msg_fifo, tsrec_irq_event.msg_buffer,
		tsrec_irq_event.fifo_size);

	if (unlikely(ret != 0)) {
		TSREC_LOG_INF(
			"ERROR: kfifo init failed, ret:%d, msg_buffer:%p, fifo_size:%u(%u/%u/%lu)\n",
			ret, tsrec_irq_event.msg_buffer,
			tsrec_irq_event.fifo_size,
			tsrec_status.tsrec_hw_cnt,
			TSREC_EXP_MAX_CNT,
			sizeof(struct tsrec_irq_info_st));
		return;
	}

	TSREC_LOG_DBG(
		"NOTICE: kfifo init done, ret:%d, msg_buffer:%p, fifo_size:%u(%u/%u/%lu)\n",
		ret, tsrec_irq_event.msg_buffer,
		tsrec_irq_event.fifo_size,
		tsrec_status.tsrec_hw_cnt,
		TSREC_EXP_MAX_CNT,
		sizeof(struct tsrec_irq_info_st));
#endif // !FS_UT
}


static void tsrec_irq_event_st_uninit(void)
{
#ifndef FS_UT
	kfifo_free(&tsrec_irq_event.msg_fifo);

	if (likely(tsrec_irq_event.msg_buffer != NULL)) {
		TSREC_KFREE(tsrec_irq_event.msg_buffer);
		tsrec_irq_event.msg_buffer = NULL;

		TSREC_LOG_INF(
			"irq msg_buffer:%p is freed\n",
			tsrec_irq_event.msg_buffer);
	}
#endif

	/* clear struct data */
	memset(&tsrec_irq_event, 0, sizeof(tsrec_irq_event));
}


static void tsrec_irq_event_st_init(void)
{
	const unsigned int tsrec_hw_cnt = tsrec_status.tsrec_hw_cnt;

#ifndef FS_UT
	/* init/setup fifo size for below dynamic mem alloc using */
	tsrec_irq_event.fifo_size = roundup_pow_of_two(
		(tsrec_hw_cnt * TSREC_EXP_MAX_CNT)
		 * sizeof(struct tsrec_irq_info_st));

	if (likely(tsrec_irq_event.msg_buffer == NULL)) {
		tsrec_irq_event.msg_buffer = TSREC_KZALLOC(tsrec_irq_event.fifo_size);

		if (unlikely(tsrec_irq_event.msg_buffer == NULL)) {
			TSREC_LOG_INF(
				"ERROR: irq msg_buffer:%p allocate memory failed, fifo_size:%u(%u/%u/%lu)\n",
				tsrec_irq_event.msg_buffer,
				tsrec_irq_event.fifo_size,
				tsrec_status.tsrec_hw_cnt,
				TSREC_EXP_MAX_CNT,
				sizeof(struct tsrec_irq_info_st));
		}
	}
#endif // !FS_UT

	tsrec_irq_event_st_reset();

	TSREC_LOG_DBG(
		"NOTICE: dynamic malloc irq event info[], array item size is tsrec_hw_cnt:%u\n",
		tsrec_hw_cnt);
}


/*---------------------------------------------------------------------------*/
// tsrec v4l2_subdev_core_ops --- command call to sensor adaptor functions
/*---------------------------------------------------------------------------*/
#ifndef FS_UT
static void tsrec_find_seninf_ctx_by_tsrec_no(void *irq_dev_ctx,
	const unsigned int tsrec_no,
	struct seninf_ctx **p_seninf_ctx, unsigned int *p_seninf_idx,
	const char *caller)
{
	struct seninf_core *core = (struct seninf_core *)(irq_dev_ctx);
	struct seninf_ctx *ctx;
	struct tsrec_n_regs_st *ptr = NULL;
	int ret;

	ret = g_tsrec_n_regs_st(tsrec_no, &ptr, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(ptr == NULL)) {
		TSREC_LOG_INF(
			"[%s] ERROR: tsrec_n_regs_st[%u] is nullptr (%p), return\n",
			caller, tsrec_no, ptr);
		return;
	}
	*p_seninf_idx = ptr->seninf_idx;

	list_for_each_entry(ctx, &core->list, list) {
		if (ctx->seninfIdx == *p_seninf_idx) {
			/* find out */
			*p_seninf_ctx = ctx;
			return;
		}
	}

	/* error case (unexpected) */
	TSREC_LOG_INF(
		"[%s] ERROR: can't find seninf_ctx:%p by (tsrec_no:%u => seninf_idx:%u)\n",
		caller, *p_seninf_ctx, tsrec_no, *p_seninf_idx);
}
#endif // !FS_UT


static void tsrec_work_notify_sensor_adaptor_hw_pre_latch(
	const unsigned int tsrec_no, const struct tsrec_work_request *req)
{
#ifndef FS_UT

	struct mtk_cam_seninf_tsrec_vsync_info vsync_info = {0};
	struct seninf_ctx *seninf_ctx = NULL;
	unsigned int seninf_idx = SENINF_IDX_NONE;

	tsrec_find_seninf_ctx_by_tsrec_no(
		req->irq_dev_ctx, tsrec_no, &seninf_ctx, &seninf_idx, __func__);
	if (unlikely(seninf_ctx == NULL))
		return;

	if (likely(seninf_ctx->sensor_sd
			&& seninf_ctx->sensor_sd->ops
			&& seninf_ctx->sensor_sd->ops->core
			&& seninf_ctx->sensor_sd->ops->core->command)) {
		/* prepare info for sending to sensor adaptor */
		/* => manually sync/copy information to user's structure */
		/*    that define in mtk_camera-v4l2-controls.h */
		vsync_info.tsrec_no = tsrec_no;
		vsync_info.seninf_idx = seninf_idx;
		vsync_info.irq_sys_time_ns = req->irq_info.sys_ts_ns;
		vsync_info.irq_tsrec_ts_us = req->irq_info.tsrec_ts_us;

		/* call v4l2_subdev_core_ops command to sensor adaptor */
		seninf_ctx->sensor_sd->ops->core->command(
			seninf_ctx->sensor_sd,
			V4L2_CMD_TSREC_NOTIFY_SENSOR_HW_PRE_LATCH,
			(void *)&vsync_info);
	} else {
		TSREC_LOG_INF(
			"ERROR: v4l2_subdev_core_ops command function not found\n");
	}

#endif // !FS_UT
}


static void tsrec_work_notify_sensor_adaptor_vsync(
	const unsigned int tsrec_no, const struct tsrec_work_request *req)
{
#ifndef FS_UT

	struct mtk_cam_seninf_tsrec_vsync_info vsync_info = {0};
	struct seninf_ctx *seninf_ctx = NULL;
	unsigned int seninf_idx = SENINF_IDX_NONE;

	tsrec_find_seninf_ctx_by_tsrec_no(
		req->irq_dev_ctx, tsrec_no, &seninf_ctx, &seninf_idx, __func__);
	if (unlikely(seninf_ctx == NULL))
		return;

	if (likely(seninf_ctx->sensor_sd
			&& seninf_ctx->sensor_sd->ops
			&& seninf_ctx->sensor_sd->ops->core
			&& seninf_ctx->sensor_sd->ops->core->command)) {
		/* prepare info for sending to sensor adaptor */
		/* => manually sync/copy information to user's structure */
		/*    that define in mtk_camera-v4l2-controls.h */
		vsync_info.tsrec_no = tsrec_no;
		vsync_info.seninf_idx = seninf_idx;
		vsync_info.irq_sys_time_ns = req->irq_info.sys_ts_ns;
		vsync_info.irq_tsrec_ts_us = req->irq_info.tsrec_ts_us;

		/* call v4l2_subdev_core_ops command to sensor adaptor */
		seninf_ctx->sensor_sd->ops->core->command(
			seninf_ctx->sensor_sd,
			V4L2_CMD_TSREC_NOTIFY_VSYNC,
			(void *)&vsync_info);
	} else {
		TSREC_LOG_INF(
			"ERROR: v4l2_subdev_core_ops command function not found\n");
	}

#endif // !FS_UT
}


static void tsrec_work_send_sensor_adaptor_timestamp_data(
	const unsigned int tsrec_no, const struct tsrec_work_request *req)
{
#ifndef FS_UT

	struct mtk_cam_seninf_tsrec_timestamp_info ts_info = {0};
	struct seninf_ctx *seninf_ctx = NULL;
	unsigned int seninf_idx = SENINF_IDX_NONE;

	tsrec_find_seninf_ctx_by_tsrec_no(
		req->irq_dev_ctx, tsrec_no, &seninf_ctx, &seninf_idx, __func__);
	if (unlikely(seninf_ctx == NULL))
		return;

	if (likely(seninf_ctx->sensor_sd
			&& seninf_ctx->sensor_sd->ops
			&& seninf_ctx->sensor_sd->ops->core
			&& seninf_ctx->sensor_sd->ops->core->command)) {
		/* prepare info for sending to sensor adaptor */
		mtk_cam_seninf_tsrec_get_timestamp_info(tsrec_no, &ts_info);
		/* preventing racing issue, overwrite irq info before sending */
		ts_info.irq_sys_time_ns = req->irq_info.sys_ts_ns;
		ts_info.irq_tsrec_ts_us = req->irq_info.tsrec_ts_us;

		/* call v4l2_subdev_core_ops command to sensor adaptor */
		seninf_ctx->sensor_sd->ops->core->command(
			seninf_ctx->sensor_sd,
			V4L2_CMD_TSREC_SEND_TIMESTAMP_INFO,
			(void *)&ts_info);
	} else {
		TSREC_LOG_INF(
			"ERROR: v4l2_subdev_core_ops command function not found\n");
	}

#endif // !FS_UT
}

/*---------------------------------------------------------------------------*/

static void mtk_cam_seninf_tsrec_n_update_irq_info_st(
	const unsigned int tsrec_no,
	const struct tsrec_irq_info_st *irq_info);

/*---------------------------------------------------------------------------*/

#ifndef FS_UT
#if defined(TSREC_WORK_USING_KTHREAD)
static void tsrec_work_handler(struct kthread_work *work)
#else /* => using workqueue */
static void tsrec_work_handler(struct work_struct *work)
#endif // TSREC_WORK_USING_KTHREAD
#else /* => FS_UT */
static void tsrec_work_handler(struct tsrec_work_request *req)
#endif // !FS_UT
{
#ifndef FS_UT
	struct tsrec_work_request *req = NULL;

	/* error handle (unexpected case) */
	if (unlikely(work == NULL))
		return;

	/* convert/cast work_struct */
	req = container_of_safe(work, struct tsrec_work_request, work);
	if (unlikely(req == NULL)) {
		TSREC_LOG_INF(
			"ERROR: container_of_safe() casting failed, return\n");
		return;
	}
#endif // !FS_UT

	if (unlikely(req->irq_dev_ctx == NULL)) {
		TSREC_LOG_INF(
			"ERROR: irq_dev_ctx:%p is nullptr, req:(tsrec_no:%u, irq_info(sys_ts:%llu(ns)(~ +%llu ns), tsrec_ts:%llu(us), intr(|%#x/%#x)), work_event_info:%#x), return\n",
			req->irq_dev_ctx,
			req->tsrec_no,
			req->irq_info.sys_ts_ns,
			req->irq_info.duration_ns,
			req->irq_info.tsrec_ts_us,
			req->irq_info.intr_status,
			req->irq_info.intr_status_2,
			req->work_event_info);
		TSREC_KFREE(req);
		return;
	}


	TSREC_SPIN_LOCK(&tsrec_log_concurrency_lock);
	TSREC_LOG_DBG(
		"Force DUMP: req(tsrec_no:%u, irq_info(sys_ts:%llu(ns)(~ +%llu ns), tsrec_ts:%llu(us), intr(%#x/%#x|%#x/%#x)), work_event_info:%#x)\n",
		req->tsrec_no,
		req->irq_info.sys_ts_ns,
		req->irq_info.duration_ns,
		req->irq_info.tsrec_ts_us,
		TSREC_ATOMIC_READ(&tsrec_status.intr_en),
		TSREC_ATOMIC_READ(&tsrec_status.intr_en_2),
		req->irq_info.intr_status,
		req->irq_info.intr_status_2,
		req->work_event_info);
	TSREC_SPIN_UNLOCK(&tsrec_log_concurrency_lock);


	/* update tsrec irq info for debugging */
	mtk_cam_seninf_tsrec_n_update_irq_info_st(
		req->tsrec_no, &req->irq_info);


	/* check work event */
	/* !!! has priority / first-after order !!! */
	if (req->work_event_info & TSREC_WORK_SENSOR_HW_PRE_LATCH) {
		tsrec_work_notify_sensor_adaptor_hw_pre_latch(
			req->tsrec_no, req);
	}

	if (req->work_event_info & TSREC_WORK_1ST_VSYNC)
		tsrec_work_notify_sensor_adaptor_vsync(req->tsrec_no, req);

	if (req->work_event_info & TSREC_WORK_QUERY_TS) {
		mtk_cam_seninf_tsrec_query_ts_records(req->tsrec_no);

		TSREC_SPIN_LOCK(&tsrec_log_concurrency_lock);
		mtk_cam_seninf_tsrec_dbg_dump_ts_records(req->tsrec_no);
		TSREC_SPIN_UNLOCK(&tsrec_log_concurrency_lock);
	}

	if (req->work_event_info & TSREC_WORK_SEND_TS_TO_SENSOR) {
		// mtk_cam_seninf_tsrec_dbg_dump_ts_records(req->tsrec_no);
		tsrec_work_send_sensor_adaptor_timestamp_data(
			req->tsrec_no, req);
	}

	if (req->work_event_info & TSREC_WORK_DBG_DUMP_IRQ_INFO) {
		TSREC_SPIN_LOCK(&tsrec_log_concurrency_lock);
		TSREC_LOG_INF(
			"req(tsrec_no:%u, irq_info(sys_ts:%llu(ns)(~ +%llu ns), tsrec_ts:%llu(us), intr(%#x/%#x|%#x/%#x)), work_event_info:%#x)\n",
			req->tsrec_no,
			req->irq_info.sys_ts_ns,
			req->irq_info.duration_ns,
			req->irq_info.tsrec_ts_us,
			TSREC_ATOMIC_READ(&tsrec_status.intr_en),
			TSREC_ATOMIC_READ(&tsrec_status.intr_en_2),
			req->irq_info.intr_status,
			req->irq_info.intr_status_2,
			req->work_event_info);
		TSREC_SPIN_UNLOCK(&tsrec_log_concurrency_lock);
	}

	TSREC_KFREE(req);
}


static void tsrec_work_init_and_queue(const unsigned int tsrec_no,
	struct tsrec_work_request *req)
{
#ifndef FS_UT

#if defined(TSREC_WORK_USING_KTHREAD)
	kthread_init_work(&req->work, tsrec_work_handler);
	kthread_queue_work(
		&tsrec_worker.kthreads[tsrec_no]->kthread, &req->work);
#else /* => using workqueue */
	INIT_WORK(&req->work, tsrec_work_handler);
	queue_work(tsrec_worker.workqs[tsrec_no], &req->work);
#endif // TSREC_WORK_USING_KTHREAD

#else
	tsrec_work_handler(req);
#endif // !FS_UT
}


static void tsrec_work_setup(int irq, void *data,
	const struct tsrec_irq_info_st *irq_info,
	const unsigned int tsrec_no, const unsigned int work_event_info)
{
	struct tsrec_work_request *req;

	req = TSREC_KZALLOC(sizeof(struct tsrec_work_request));
	if (unlikely(req == NULL)) {
		TSREC_LOG_INF(
			"ERROR: work request:%p dynamic alloc mem failed, return\n",
			req);
		return;
	}

	/* copy input data/info */
	req->irq_dev_ctx = data;
	req->tsrec_no = tsrec_no;
	req->work_event_info = work_event_info;
	memcpy(&req->irq_info, irq_info, sizeof(*irq_info));

	/* init & queue work */
	tsrec_work_init_and_queue(tsrec_no, req);
}


static void tsrec_worker_flush(const unsigned int tsrec_no)
{
#ifndef FS_UT
	struct tsrec_kthread_st *ptr = NULL;
	const unsigned int tsrec_hw_cnt = tsrec_status.tsrec_hw_cnt;

#if defined(TSREC_WORK_USING_KTHREAD)

	if (unlikely((tsrec_worker.kthreads == NULL)
			|| (tsrec_no >= tsrec_hw_cnt)))
		return;

	ptr = tsrec_worker.kthreads[tsrec_no];
	if (ptr == NULL)
		return;

	TSREC_LOG_INF(
		"NOTICE: tsrec_no:%u, kthread flush\n",
		tsrec_no);

	kthread_flush_worker(&ptr->kthread);

#else /* => workqueue */

	if (unlikely((tsrec_worker.workqs == NULL)
			|| (tsrec_no >= tsrec_hw_cnt)))
		return;

	ptr = tsrec_worker.workqs[tsrec_no];
	if (ptr == NULL)
		return;

	TSREC_LOG_INF(
		"NOTICE: tsrec_no:%u, workqueue drain\n",
		tsrec_no);

	drain_workqueue(ptr);

#endif // TSREC_WORK_USING_KTHREAD
#endif // !FS_UT

	TSREC_LOG_DBG(
		"NOTICE: tsrec_no:%u, worker flush/drain done\n",
		tsrec_no);
}


#ifndef FS_UT
#if defined(TSREC_WORK_USING_KTHREAD)
static void tsrec_kthread_uninit(const unsigned int tsrec_no)
{
	struct tsrec_kthread_st *ptr = NULL;
	const unsigned int tsrec_hw_cnt = tsrec_status.tsrec_hw_cnt;

	if (unlikely((tsrec_worker.kthreads == NULL)
			|| (tsrec_no >= tsrec_hw_cnt)))
		return;

	ptr = tsrec_worker.kthreads[tsrec_no];
	if (ptr == NULL)
		return;

	kthread_flush_worker(&ptr->kthread);
	if (likely(ptr->kthread_task != NULL)) {
		kthread_stop(ptr->kthread_task);
		tsrec_worker.kthreads[tsrec_no]->kthread_task = NULL;
	}
	tsrec_worker.kthreads[tsrec_no] = NULL;

	TSREC_LOG_INF(
		"kthreads[%u]:%p is freed (%p)\n",
		tsrec_no, tsrec_worker.kthreads[tsrec_no], ptr);
}


static void tsrec_kthread_init(const unsigned int tsrec_no)
{
	struct tsrec_kthread_st *ptr = NULL;
	const unsigned int tsrec_hw_cnt = tsrec_status.tsrec_hw_cnt;

	if (unlikely((tsrec_worker.kthreads == NULL)
			|| (tsrec_no >= tsrec_hw_cnt)))
		return;

	ptr = tsrec_worker.kthreads[tsrec_no];
	if (ptr != NULL)
		return;
	ptr = TSREC_KZALLOC(sizeof(struct tsrec_kthread_st));
	if (ptr == NULL) {
		TSREC_LOG_INF(
			"ERROR: (tsrec_kthread_st) memory allocate failed (%p), kthreads[%u]:%p, return\n",
			ptr, tsrec_no, tsrec_worker.kthreads[tsrec_no]);
		return;
	}

	kthread_init_worker(&ptr->kthread);
	ptr->kthread_task = kthread_run(kthread_worker_fn,
		&ptr->kthread,
		"seninf_tsrec-%u",
		tsrec_no);
	if (IS_ERR(ptr->kthread_task)) {
		TSREC_LOG_INF(
			"ERROR: failed to run kthread_task, return   [tsrec_no:%u]\n",
			tsrec_no);
		return;
	}
	sched_set_fifo(ptr->kthread_task);
	tsrec_worker.kthreads[tsrec_no] = ptr;

	TSREC_LOG_DBG(
		"NOTICE: kthread run (%p, (%p/%p)), kthreads[%u]:%p (%p/%p)\n",
		ptr, &ptr->kthread, ptr->kthread_task,
		tsrec_no,
		tsrec_worker.kthreads[tsrec_no],
		&tsrec_worker.kthreads[tsrec_no]->kthread,
		tsrec_worker.kthreads[tsrec_no]->kthread_task);
}


#else /* => workqueue */
static void tsrec_workqueue_uninit(const unsigned int tsrec_no)
{
	struct workqueue_struct *ptr = NULL;
	const unsigned int tsrec_hw_cnt = tsrec_status.tsrec_hw_cnt;

	if (unlikely((tsrec_worker.workqs == NULL)
			|| (tsrec_no >= tsrec_hw_cnt)))
		return;

	ptr = tsrec_worker.workqs[tsrec_no];
	if (ptr == NULL)
		return;

	drain_workqueue(ptr);
	destroy_workqueue(ptr);
	tsrec_worker.workqs[tsrec_no] = NULL;

	TSREC_LOG_INF(
		"workqs[%u]:%p is freed (%p)\n",
		tsrec_no, tsrec_worker.workqs[tsrec_no], ptr);
}


static void tsrec_workqueue_init(const unsigned int tsrec_no)
{
	struct workqueue_struct *ptr = NULL;
	const unsigned int tsrec_hw_cnt = tsrec_status.tsrec_hw_cnt;

	if (unlikely((tsrec_worker.workqs == NULL)
			|| (tsrec_no >= tsrec_hw_cnt)))
		return;

	ptr = tsrec_worker.workqs[tsrec_no];
	if (ptr != NULL)
		return;

	ptr = alloc_workqueue(
		"tsrec-wq", WQ_HIGHPRI | WQ_CPU_INTENSIVE, 0);
	//	"tsrec-wq", WQ_HIGHPRI | WQ_UNBOUND, 0);
	// ptr = alloc_ordered_workqueue(
	//	"tsrec-wq", WQ_HIGHPRI | WQ_FREEZABLE);
	if (unlikely(ptr == NULL)) {
		TSREC_LOG_INF(
			"ERROR: failed to alloc workqueue, return   [tsrec_no:%u]\n",
			tsrec_no);
		return;
	}
	tsrec_worker.workqs[tsrec_no] = ptr;

	TSREC_LOG_INF(
		"NOTICE: workqueue be allocated (%p), workqs[%u]:%p\n",
		ptr, tsrec_no, tsrec_worker.workqs[tsrec_no]);
}
#endif // TSREC_WORK_USING_KTHREAD


static void tsrec_worker_st_uninit(void)
{
	const unsigned int tsrec_hw_cnt = tsrec_status.tsrec_hw_cnt;
	unsigned int i;

	for (i = 0; i < tsrec_hw_cnt; ++i) {
#if defined(TSREC_WORK_USING_KTHREAD)
		tsrec_kthread_uninit(i);
#else
		tsrec_workqueue_uninit(i);
#endif // TSREC_WORK_USING_KTHREAD
	}

	/* free all dynamic malloc data after each structure been freed */
#if defined(TSREC_WORK_USING_KTHREAD)

	if (likely(tsrec_worker.kthreads != NULL)) {
		TSREC_KFREE(tsrec_worker.kthreads);
		tsrec_worker.kthreads = NULL;

		TSREC_LOG_INF(
			"kthreads[]:%p array of pointer is freed\n",
			tsrec_worker.kthreads);
	}

#else /* => using workqueue */

	if (likely(tsrec_worker.workqs != NULL)) {
		TSREC_KFREE(tsrec_worker.workqs);
		tsrec_worker.workqs = NULL;

		TSREC_LOG_INF(
			"workqs[]:%p array of pointer is freed\n",
			tsrec_worker.workqs);
	}

#endif // TSREC_WORK_USING_KTHREAD

	/* clear struct data */
	memset(&tsrec_worker, 0, sizeof(tsrec_worker));
}


static void tsrec_worker_st_init(void)
{
	const unsigned int tsrec_hw_cnt = tsrec_status.tsrec_hw_cnt;

#if defined(TSREC_WORK_USING_KTHREAD)

	/* tsrec kthread array dynamic alloc */
	if (likely(tsrec_worker.kthreads == NULL)) {
		/* case handle */
		if (unlikely(tsrec_hw_cnt == 0)) {
			TSREC_LOG_INF(
				"WARNING: DTS tsrec_hw_cnt:%u => not allocate workqueue array, return\n",
				tsrec_hw_cnt);
			return;
		}

		tsrec_worker.kthreads = TSREC_KCALLOC(
			tsrec_hw_cnt, sizeof(struct tsrec_kthread_st *));

		if (unlikely(tsrec_worker.kthreads == NULL)) {
			TSREC_LOG_INF(
				"ERROR: kthreads[] (kthread_worker & task_struct *) array of pointer allocate memory failed   [tsrec_hw_cnt:%u]\n",
				tsrec_hw_cnt);
		}

		TSREC_LOG_DBG(
			"NOTICE: dynamic malloc kthreads[] (kthread_worker & task_struct *) array of pointer, item size is tsrec_hw_cnt:%u\n",
			tsrec_hw_cnt);
	}

#else /* => using workqueue */

	/* workqueue array dynamic alloc */
	if (likely(tsrec_worker.workqs == NULL)) {
		/* case handle */
		if (unlikely(tsrec_hw_cnt == 0)) {
			TSREC_LOG_INF(
				"WARNING: DTS tsrec_hw_cnt:%u => not allocate workqueue array, return\n",
				tsrec_hw_cnt);
			return;
		}

		tsrec_worker.workqs = TSREC_KCALLOC(
			tsrec_hw_cnt, sizeof(struct workqueue_struct *));

		if (unlikely(tsrec_worker.workqs == NULL)) {
			TSREC_LOG_INF(
				"ERROR: workqs[] array of pointer allocate memory failed   [tsrec_hw_cnt:%u]\n",
				tsrec_hw_cnt);
		}

		TSREC_LOG_DBG(
			"NOTICE: dynamic malloc workqs[] array of pointer, item size is tsrec_hw_cnt:%u\n",
			tsrec_hw_cnt);
	}

#endif // TSREC_WORK_USING_KTHREAD
}
#endif // !FS_UT

/*---------------------------------------------------------------------------*/

static void tsrec_seninf_cfg_st_uninit(const unsigned int idx)
{
	struct tsrec_seninf_cfg_st *ptr = NULL;
	int ret;

	ret = g_tsrec_seninf_cfg_st(idx, &ptr, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(ptr == NULL)) {
		TSREC_LOG_DBG(
			"NOTICE: seninf src_cfg[%u] is nullptr (%p), skip\n",
			idx, ptr);
		return;
	}

	/* free the memory on this index of array and reset to NULL */
	TSREC_KFREE(ptr);
	tsrec_src.src_cfg[idx] = NULL;

	TSREC_LOG_INF(
		"seninf src_cfg[%u]:%p is freed (%p)\n",
		idx, tsrec_src.src_cfg[idx], ptr);
}


static void tsrec_seninf_cfg_st_init(const unsigned int idx)
{
	struct tsrec_seninf_cfg_st *ptr = NULL;
	int ret;

	ret = g_tsrec_seninf_cfg_st(idx, &ptr, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(ptr != NULL)) {
		TSREC_LOG_DBG(
			"NOTICE: seninf src_cfg[%u] already inited (%p), skip\n",
			idx, ptr);
		return;
	}

	/* allocate a memory on this index of array and keep this address */
	ptr = TSREC_KZALLOC(sizeof(struct tsrec_seninf_cfg_st));
	if (ptr == NULL) {
		TSREC_LOG_INF(
			"ERROR: memory allocate failed (%p), src_cfg[%u]:%p, return\n",
			ptr, idx, tsrec_src.src_cfg[idx]);
		return;
	}

	tsrec_src.src_cfg[idx] = ptr;

	TSREC_LOG_DBG(
		"memory be allocated (%p), src_cfg[%u]:%p\n",
		ptr, idx, tsrec_src.src_cfg[idx]);
}


static void tsrec_seninf_cfg_st_reset(const unsigned int idx)
{
	struct tsrec_seninf_cfg_st *ptr = NULL;
	const unsigned int exp_trig_src = g_tsrec_exp_trig_src();
	unsigned int i;
	int ret;

	ret = g_tsrec_seninf_cfg_st(idx, &ptr, __func__);
	if (unlikely(ret != 0))
		return;
	if (ptr == NULL) {
		TSREC_LOG_DBG(
			"NOTICE: seninf src_cfg[%u] is nullptr (%p), init it\n",
			idx, ptr);

		/* call init for this structure (memory maybe alloc failed) */
		tsrec_seninf_cfg_st_init(idx);

		/* get pointer and check it again */
		ret = g_tsrec_seninf_cfg_st(idx, &ptr, __func__);
		if (unlikely(ret != 0))
			return;
		if (unlikely(ptr == NULL)) {
			TSREC_LOG_INF(
				"ERROR: seninf src_cfg[%u] init failed (%p), return\n",
				idx, ptr);
			return;
		}
	}

	/* clear all old data */
	memset(ptr, 0, sizeof(struct tsrec_seninf_cfg_st));

	/* reset/init all needed tsrec_src members/variables */
	ptr->seninf_idx = SENINF_IDX_NONE;
	ptr->tsrec_no = TSREC_NO_NONE;

	if (unlikely(exp_trig_src != TSREC_EXP_TRIG_SRC_DEF)) {
		TSREC_LOG_INF(
			"NOTICE: USER manually assign exp_trig_src:%u => use exp_trig_src:%u\n",
			tsrec_con_mgr.exp_trig_src,
			exp_trig_src);
	}

	/* reset/init to some special default value */
	for (i = 0; i < TSREC_EXP_MAX_CNT; ++i) {
		ptr->exp_vc_dt[i].vsync_vc = TSREC_VC_NONE;
		ptr->exp_vc_dt[i].trig_src = exp_trig_src;

		ptr->cust_vc_dt[i].vsync_vc = TSREC_VC_NONE;
		ptr->cust_vc_dt[i].trig_src = exp_trig_src;
	}
}


static void tsrec_src_st_reset(void)
{
	/* reset/init all needed tsrec_src members/variables */
	/* this value will be overwrite by parsing dts info */
	;
}


static void tsrec_src_st_uninit(void)
{
	const unsigned int seninf_hw_cnt = tsrec_status.seninf_hw_cnt;
	unsigned int i = 0;

	/* uninit all struct data */
	for (i = 0; i < seninf_hw_cnt; ++i)
		tsrec_seninf_cfg_st_uninit(i);

	/* free all dynamic malloc data after each structure been freed */
	if (tsrec_src.src_cfg != NULL) {
		TSREC_KFREE(tsrec_src.src_cfg);
		tsrec_src.src_cfg = NULL;

		TSREC_LOG_INF(
			"src_cfg[]:%p array is freed\n",
			tsrec_src.src_cfg);
	}

	/* clear struct data */
	memset(&tsrec_src, 0, sizeof(tsrec_src));
}


static void tsrec_src_st_init(void)
{
	const unsigned int seninf_hw_cnt = tsrec_status.seninf_hw_cnt;

	if (unlikely(tsrec_src.is_init)) {
		TSREC_LOG_INF(
			"NOTICE: already init:%u, skip reset structure data\n",
			tsrec_src.is_init);
		return;
	}

	/* !!! first, dynamic malloc all needed data !!! */
	/* for src_cfg[] array */
	if (likely(tsrec_src.src_cfg == NULL)) {
		/* case handle */
		if (seninf_hw_cnt == 0) {
			TSREC_LOG_INF(
				"WARNING: DTS seninf_hw_cnt:%u => not allocate any memory for src_cfg[] array, return\n",
				seninf_hw_cnt);
			return;
		}

		tsrec_src.src_cfg = TSREC_KCALLOC(
			seninf_hw_cnt, sizeof(struct tsrec_seninf_cfg_st *));

		if (unlikely(tsrec_src.src_cfg == NULL)) {
			TSREC_LOG_INF(
				"ERROR: src_cfg[] array allocate memory failed   [seninf_hw_cnt:%u]\n",
				seninf_hw_cnt);
		}

		TSREC_LOG_DBG(
			"NOTICE: dynamic malloc src_cfg[], array item size is seninf_hw_cnt:%u\n",
			seninf_hw_cnt);
	}

	tsrec_src_st_reset();

	/* mark as inited */
	tsrec_src.is_init = 1;
}

/*---------------------------------------------------------------------------*/

static void tsrec_n_regs_st_uninit(const unsigned int tsrec_no)
{
	struct tsrec_n_regs_st *ptr = NULL;
	int ret;

	ret = g_tsrec_n_regs_st(tsrec_no, &ptr, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(ptr == NULL)) {
		TSREC_LOG_DBG(
			"NOTICE: tsrec_n_regs[%u] is nullptr (%p), skip\n",
			tsrec_no, ptr);
		return;
	}

	/* free the memory on this index of array and reset to NULL */
	TSREC_KFREE(ptr);
	tsrec_status.tsrec_n_regs[tsrec_no] = NULL;

	TSREC_LOG_INF(
		"tsrec_n_regs[%u]:%p is freed (%p)\n",
		tsrec_no, tsrec_status.tsrec_n_regs[tsrec_no], ptr);
}


static void tsrec_n_regs_st_init(const unsigned int tsrec_no)
{
	struct tsrec_n_regs_st *ptr = NULL;
	int ret;

	ret = g_tsrec_n_regs_st(tsrec_no, &ptr, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(ptr != NULL)) {
		TSREC_LOG_DBG(
			"NOTICE: tsrec_n_regs[%u] already inited (%p), skip\n",
			tsrec_no, ptr);
		return;
	}

	/* allocate a memory on this index of array and keep this address */
	ptr = TSREC_KZALLOC(sizeof(struct tsrec_n_regs_st));
	if (ptr == NULL) {
		TSREC_LOG_INF(
			"ERROR: memory allocate failed (%p), tsrec_n_regs[%u]:%p, return\n",
			ptr, tsrec_no, tsrec_status.tsrec_n_regs[tsrec_no]);
		return;
	}

	tsrec_status.tsrec_n_regs[tsrec_no] = ptr;

	TSREC_LOG_DBG(
		"memory be allocated (%p) tsrec_n_regs[%u]:%p\n",
		ptr, tsrec_no, tsrec_status.tsrec_n_regs[tsrec_no]);
}


static void tsrec_n_regs_st_reset(const unsigned int tsrec_no)
{
	struct tsrec_n_regs_st *ptr = NULL;
	int ret;

	ret = g_tsrec_n_regs_st(tsrec_no, &ptr, __func__);
	if (unlikely(ret != 0))
		return;
	if (ptr == NULL) {
		TSREC_LOG_DBG(
			"NOTICE: tsrec_n_regs[%u] is nullptr (%p), init it\n",
			tsrec_no, ptr);

		/* call init for this structure (memory maybe alloc failed) */
		tsrec_n_regs_st_init(tsrec_no);

		/* get pointer and check it again */
		ret = g_tsrec_n_regs_st(tsrec_no, &ptr, __func__);
		if (unlikely(ret != 0))
			return;
		if (unlikely(ptr == NULL)) {
			TSREC_LOG_INF(
				"ERROR: tsrec_n_regs[%u] init failed (%p), return\n",
				tsrec_no, ptr);
			return;
		}

#ifndef FS_UT
#if defined(TSREC_WORK_USING_KTHREAD)
		/* init kthread */
		tsrec_kthread_init(tsrec_no);
#else
		/* init workqueue */
		tsrec_workqueue_init(tsrec_no);
#endif // TSREC_WORK_USING_KTHREAD
#endif // !FS_UT
	}

	/* clear all old data */
	memset(ptr, 0, sizeof(struct tsrec_n_regs_st));

	/* reset/init all needed tsrec_src members/variables */
	ptr->seninf_idx = SENINF_IDX_NONE;
}


static void tsrec_status_st_reset(void)
{
	/* reset/init all needed tsrec_src members/variables */
	TSREC_ATOMIC_INIT(0, &tsrec_status.timer_en_cnt);
	TSREC_ATOMIC_INIT(0, &tsrec_status.timer_cfg);
	TSREC_ATOMIC_INIT(0, &tsrec_status.top_cfg);
	TSREC_ATOMIC_INIT(0, &tsrec_status.intr_en);
	TSREC_ATOMIC_INIT(0, &tsrec_status.intr_status);
	TSREC_ATOMIC_INIT(0, &tsrec_status.intr_en_2);
	TSREC_ATOMIC_INIT(0, &tsrec_status.intr_status_2);
}


static void tsrec_status_st_uninit(void)
{
	const unsigned int tsrec_hw_cnt = tsrec_status.tsrec_hw_cnt;
	unsigned int i = 0;

	/* uninit all struct data */
	for (i = 0; i < tsrec_hw_cnt; ++i)
		tsrec_n_regs_st_uninit(i);

	/* free all dynamic malloc data after each structure been freed */
	if (likely(tsrec_status.tsrec_n_regs != NULL)) {
		TSREC_KFREE(tsrec_status.tsrec_n_regs);
		tsrec_status.tsrec_n_regs = NULL;

		TSREC_LOG_INF(
			"tsrec_n_regs[]:%p array is freed\n",
			tsrec_status.tsrec_n_regs);
	}

	/* clear struct data */
	memset(&tsrec_status, 0, sizeof(tsrec_status));
}


static void tsrec_status_st_init(void)
{
	const unsigned int tsrec_hw_cnt = tsrec_status.tsrec_hw_cnt;

	if (unlikely(tsrec_status.is_init)) {
		TSREC_LOG_INF(
			"NOTICE: already init:%u, skip reset structure data\n",
			tsrec_status.is_init);
		return;
	}

	/* !!! first, dynamic malloc all needed data !!! */
	/* for tsrec_n_regs[] array */
	if (likely(tsrec_status.tsrec_n_regs == NULL)) {
		/* case handle */
		if (unlikely(tsrec_hw_cnt == 0)) {
			TSREC_LOG_INF(
				"WARNING: DTS tsrec_hw_cnt:%u => not allocate any memory for tsrec_n_regs[] array, return\n",
				tsrec_hw_cnt);
			return;
		}

		tsrec_status.tsrec_n_regs = TSREC_KCALLOC(
			tsrec_hw_cnt, sizeof(struct tsrec_n_regs_st *));

		if (unlikely(tsrec_status.tsrec_n_regs == NULL)) {
			TSREC_LOG_INF(
				"ERROR: tsrec_n_regs[] array allocate memory failed   [tsrec_hw_cnt:%u]\n",
				tsrec_hw_cnt);
		}

		TSREC_LOG_DBG(
			"NOTICE: dynamic malloc tsrec_n_regs[], array item size is tsrec_hw_cnt:%u\n",
			tsrec_hw_cnt);
	}

	tsrec_status_st_reset();

	/* mark as inited */
	tsrec_status.is_init = 1;
}

/*---------------------------------------------------------------------------*/

static void tsrec_data_uninit(void)
{
	tsrec_src_st_uninit();
	tsrec_status_st_uninit();
	tsrec_irq_event_st_uninit();

#ifndef FS_UT
	tsrec_worker_st_uninit();
#endif // !FS_UT
}


static void tsrec_data_init(void)
{
	tsrec_src_st_init();
	tsrec_status_st_init();
	tsrec_con_mgr_st_init();
	tsrec_irq_event_st_init();

#ifndef FS_UT
	tsrec_worker_st_init();
#endif // !FS_UT
}

/*---------------------------------------------------------------------------*/


/*
 * return:
 *      @negative: has some error happened.
 *      @0 : this tsrec has NOT been used before.
 *      @1 : this tsrec has been used, settings will be overwrite!
 */
static int tsrec_n_regs_st_chk_used_status(
	const unsigned int tsrec_no)
{
	struct tsrec_n_regs_st *ptr = NULL;
	unsigned int target_seninf_idx;
	int ret = 0;

	ret = g_tsrec_n_regs_st(tsrec_no, &ptr, __func__);
	if (unlikely(ret != 0))
		return -1;
	if (ptr == NULL) {
		TSREC_LOG_DBG(
			"NOTICE: tsrec_n_regs[%u] is nullptr (%p), return 0\n",
			tsrec_no, ptr);
		return 0;
	}

	target_seninf_idx = ptr->seninf_idx;

	/* check using situation */
	// if ((target_seninf_idx == 0) || (target_seninf_idx == SENINF_IDX_NONE))
	if (target_seninf_idx == SENINF_IDX_NONE)
		ret = 0;
	else
		ret = 1;

	return ret;
}


static void tsrec_irq_enable(const unsigned int en)
{
	if (!tsrec_status.tsrec_irq) {
		TSREC_LOG_INF(
			"ERROR: TSREC-IRQ non-valid(tsrec_irq:%d), irq_en_status:%u, return   [en:%u]\n",
			tsrec_status.tsrec_irq,
			tsrec_status.tsrec_irq_en_status,
			en);
		return;
	}

	tsrec_status.tsrec_irq_en_status = (en) ? 1 : 0;

#ifndef FS_UT
	if (en) {
		tsrec_irq_event_st_reset();
		enable_irq(tsrec_status.tsrec_irq);
	} else
		disable_irq(tsrec_status.tsrec_irq);
#endif // !FS_UT

	TSREC_LOG_DBG(
		"TSREC-IRQ(irq:%d, irq_en_status:%u)   [en:%u]\n",
		tsrec_status.tsrec_irq,
		tsrec_status.tsrec_irq_en_status,
		en);
}


static void tsrec_intr_reset(const unsigned int tsrec_no)
{
	const unsigned int exp_trig_src = g_tsrec_exp_trig_src();

	/* reset/disable tsrec intr ctrl */
	mtk_cam_seninf_s_tsrec_intr_wclr_en(0);

	if (unlikely(exp_trig_src != TSREC_EXP_TRIG_SRC_DEF)) {
		TSREC_LOG_INF(
			"NOTICE: USER manually assign exp_trig_src:%u => use exp_trig_src:%u\n",
			tsrec_con_mgr.exp_trig_src,
			exp_trig_src);
	}

	/* tsrec a~d (0~3), e~f (4~5) are ctrl by different RG */
	if (tsrec_no < 4) {
		/* disable all exp Vsync & 1st-Hsync interrupt */
		mtk_cam_seninf_s_tsrec_intr_en(tsrec_no,
			1, 1, 1, 0, 0);
		mtk_cam_seninf_s_tsrec_intr_en(tsrec_no,
			1, 1, 1, 1, 0);
	} else {
		/* disable all exp Vsync & 1st-Hsync interrupt */
		mtk_cam_seninf_s_tsrec_intr_en_2(tsrec_no,
			1, 1, 1, 0, 0);
		mtk_cam_seninf_s_tsrec_intr_en_2(tsrec_no,
			1, 1, 1, 1, 0);
	}

	TSREC_LOG_INF(
		"tsrec_no:%u, intr reset/disable => intr_en(%#x/%#x), w_clr:%u\n",
		tsrec_no,
		TSREC_ATOMIC_READ(&tsrec_status.intr_en),
		TSREC_ATOMIC_READ(&tsrec_status.intr_en_2),
		0);
}


static void tsrec_intr_ctrl_exp_vsync_en(
	const unsigned int tsrec_no, const unsigned int reg_group,
	const unsigned int exp0, const unsigned int exp1, const unsigned int exp2,
	const unsigned int target_exp_id, const unsigned int en)
{
	switch (target_exp_id) {
	case TSREC_EXP_ID_1:
	{
		switch (reg_group) {
		case 0:
			mtk_cam_seninf_s_tsrec_intr_en(tsrec_no,
				exp0, 0, 0, 0, en);
			break;
		case 1:
			mtk_cam_seninf_s_tsrec_intr_en_2(tsrec_no,
				exp0, 0, 0, 0, en);
			break;
		}
	}
		break;
	case TSREC_EXP_ID_2:
	{
		switch (reg_group) {
		case 0:
			mtk_cam_seninf_s_tsrec_intr_en(tsrec_no,
				0, exp1, 0, 0, en);
			break;
		case 1:
			mtk_cam_seninf_s_tsrec_intr_en_2(tsrec_no,
				0, exp1, 0, 0, en);
			break;
		}
	}
		break;
	case TSREC_EXP_ID_3:
	{
		switch (reg_group) {
		case 0:
			mtk_cam_seninf_s_tsrec_intr_en(tsrec_no,
				0, 0, exp2, 0, en);
			break;
		case 1:
			mtk_cam_seninf_s_tsrec_intr_en_2(tsrec_no,
				0, 0, exp2, 0, en);
			break;
		}
	}
		break;
	}
}


static void tsrec_intr_ctrl_exp_vsync_setup(
	const unsigned int tsrec_no, const unsigned int reg_group,
	const unsigned int exp0, const unsigned int exp1, const unsigned int exp2,
	const unsigned int sensor_hw_pre_latch_exp_num, const unsigned int en)
{
	/* exp num = exp id - 1 */
	const unsigned int tsrec_1st_exp_id = (TSREC_1ST_EXP_NUM + 1);
	const unsigned int sensor_hw_pre_latch_exp_id =
		(sensor_hw_pre_latch_exp_num + 1);

	/* setup 1st vsync interrupt */
	tsrec_intr_ctrl_exp_vsync_en(tsrec_no, reg_group,
		exp0, exp1, exp2, tsrec_1st_exp_id, en);

	/* if need to add extra vsync interrupt, notice that */
	if (tsrec_1st_exp_id != sensor_hw_pre_latch_exp_id) {
		TSREC_LOG_INF(
			"NOTICE: get custom assign sensor hw pre-latch exp (id:%u/num:%u), vsync intr en %u\n",
			sensor_hw_pre_latch_exp_id, sensor_hw_pre_latch_exp_num, en);
	}

	/* setup sensor hw pre-latch exp vsync interrupt */
	switch (sensor_hw_pre_latch_exp_id) {
	case TSREC_EXP_ID_1:
		tsrec_intr_ctrl_exp_vsync_en(tsrec_no, reg_group,
			1, 0, 0, sensor_hw_pre_latch_exp_id, en);
		break;
	case TSREC_EXP_ID_2:
		tsrec_intr_ctrl_exp_vsync_en(tsrec_no, reg_group,
			0, 1, 0, sensor_hw_pre_latch_exp_id, en);
		break;
	case TSREC_EXP_ID_3:
		tsrec_intr_ctrl_exp_vsync_en(tsrec_no, reg_group,
			0, 0, 1, sensor_hw_pre_latch_exp_id, en);
		break;
	}
}


static void tsrec_intr_ctrl_sensor_hw_pre_latch_cheker(
	const unsigned int tsrec_no, const unsigned int seninf_idx,
	const unsigned int sensor_hw_pre_latch_exp_code,
	unsigned int *p_sensor_hw_pre_latch_exp_num)
{
	unsigned int i;

	if (likely(sensor_hw_pre_latch_exp_code == 0)) {
		*p_sensor_hw_pre_latch_exp_num = 0;
		return;
	}

	/* --- error handling --- */
	if (unlikely(TSREC_POPCNT_32(sensor_hw_pre_latch_exp_code) > 1)) {
		*p_sensor_hw_pre_latch_exp_num = TSREC_1ST_EXP_NUM;

		TSREC_LOG_INF(
			"ERROR: seninf src_cfg[%u] <=> tsrec_no:%u, sensor_hw_pre_latch_exp_code:%#x (set more than 1 bit), plz check sensor frame description settings, auto assign to %u, return\n",
			seninf_idx, tsrec_no,
			sensor_hw_pre_latch_exp_code,
			*p_sensor_hw_pre_latch_exp_num);
		return;
	}

	for (i = 0; i < TSREC_EXP_MAX_CNT; ++i) {
		const unsigned int exp_num_bit_shift = i + TSREC_EXP_ID_1;

		if ((sensor_hw_pre_latch_exp_code >> exp_num_bit_shift) & 1U) {
			*p_sensor_hw_pre_latch_exp_num = i;

			TSREC_LOG_INF(
				"NOTICE: seninf src_cfg[%u] <=> tsrec_no:%u, sensor_hw_pre_latch_exp_code:%#x => sensor_hw_pre_latch_exp_num:%u\n",
				seninf_idx, tsrec_no,
				sensor_hw_pre_latch_exp_code,
				*p_sensor_hw_pre_latch_exp_num);
		}
	}
}


static void tsrec_intr_ctrl_checker(const struct tsrec_seninf_cfg_st *p_src_cfg,
	unsigned int *p_exp0, unsigned int *p_exp1, unsigned int *p_exp2)
{
	unsigned long long vc_dt_src_code, cust_vc_dt_src_code;
	unsigned int exp0 = 0, exp1 = 0, exp2 = 0;
	const unsigned int user_intr_exp_en_mask =
		tsrec_con_mgr.intr_exp_en_mask;

	/* detect situation for config intr enable */
	vc_dt_src_code = p_src_cfg->vc_dt_src_code;
	exp0 |= (unsigned int)(vc_dt_src_code >> PAD_SRC_RAW0) & (1ULL);
	exp1 |= (unsigned int)(vc_dt_src_code >> PAD_SRC_RAW1) & (1ULL);
	exp2 |= (unsigned int)(vc_dt_src_code >> PAD_SRC_RAW2) & (1ULL);

	cust_vc_dt_src_code = p_src_cfg->cust_vc_dt_src_code;
	exp0 |= (unsigned int)(cust_vc_dt_src_code >> TSREC_EXP_ID_1) & (1ULL);
	exp1 |= (unsigned int)(cust_vc_dt_src_code >> TSREC_EXP_ID_2) & (1ULL);
	exp2 |= (unsigned int)(cust_vc_dt_src_code >> TSREC_EXP_ID_3) & (1ULL);


	/* setup/overwrite which exp INTR will be enabled */
	*p_exp0 = exp0 & ((user_intr_exp_en_mask >> 0) & 1UL);
	*p_exp1 = exp1 & ((user_intr_exp_en_mask >> 1) & 1UL);
	*p_exp2 = exp2 & ((user_intr_exp_en_mask >> 2) & 1UL);

	if (unlikely(user_intr_exp_en_mask != TSREC_INTR_EXP_EN_MASK)) {
		TSREC_LOG_INF(
			"NOTICE: USER manually assign intr_exp_en_mask:%#x => exp(%u/%u/%u => %u/%u/%u)\n",
			user_intr_exp_en_mask,
			exp0, exp1, exp2,
			*p_exp0, *p_exp1, *p_exp2);
		return;
	}

	TSREC_LOG_INF(
		"NOTICE: default intr_exp_en_mask:%#x => exp(%u/%u/%u => %u/%u/%u)\n",
		user_intr_exp_en_mask,
		exp0, exp1, exp2,
		*p_exp0, *p_exp1, *p_exp2);
}


static void tsrec_intr_ctrl(const unsigned int tsrec_no,
	const unsigned int seninf_idx)
{
	struct tsrec_seninf_cfg_st *p_src_cfg = NULL;
	struct tsrec_n_regs_st *p_tsrec_n_regs = NULL;
	const unsigned int exp_trig_src = g_tsrec_exp_trig_src();
	const unsigned int user_intr_exp_trig_src_both =
		tsrec_con_mgr.intr_exp_trig_src_both;
	unsigned int exp0, exp1, exp2;
	int ret;

	/* source needed info for starting settings configure procedure */
	ret = g_tsrec_seninf_cfg_st(seninf_idx, &p_src_cfg, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(p_src_cfg == NULL)) {
		TSREC_LOG_INF(
			"ERROR: seninf src_cfg[%u] is nullptr (%p), chk if reset & update vc/dt info or not, return   [tsrec_no:%u, seninf_idx:%u]\n",
			seninf_idx, p_src_cfg,
			tsrec_no,
			seninf_idx);
		return;
	}
	ret = g_tsrec_n_regs_st(tsrec_no, &p_tsrec_n_regs, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(p_tsrec_n_regs == NULL)) {
		TSREC_LOG_INF(
			"ERROR: tsrec_n_regs_st[%u] is nullptr (%p), return   [tsrec_no:%u, seninf_idx:%u]\n",
			tsrec_no, p_tsrec_n_regs,
			tsrec_no,
			seninf_idx);
		return;
	}

	/* setup tsrec intr ctrl */
	mtk_cam_seninf_s_tsrec_intr_wclr_en(TSREC_INTR_W_CLR_EN);

	/* check/setup which exp INTR will be enabled */
	tsrec_intr_ctrl_checker(p_src_cfg, &exp0, &exp1, &exp2);

	/* check/setup sensor hw pre-latch exp setting situation */
	tsrec_intr_ctrl_sensor_hw_pre_latch_cheker(tsrec_no, seninf_idx,
		p_src_cfg->sensor_hw_pre_latch_exp_code,
		&p_tsrec_n_regs->sensor_hw_pre_latch_exp_num);


	if (unlikely(exp_trig_src != TSREC_EXP_TRIG_SRC_DEF)) {
		TSREC_LOG_INF(
			"NOTICE: USER manually assign exp_trig_src:%u => use exp_trig_src:%u\n",
			tsrec_con_mgr.exp_trig_src,
			exp_trig_src);
	}

	/* !!! due to tsrec a~d (0~3), e~f (4~5) are ctrl by different RG !!! */
	if (tsrec_no < 4) {
		if (unlikely(user_intr_exp_trig_src_both)) {
			/* Vsync intr enable */
			mtk_cam_seninf_s_tsrec_intr_en(tsrec_no,
				exp0, exp1, exp2, 0, 1);

			/* 1st-Hsync intr enable */
			mtk_cam_seninf_s_tsrec_intr_en(tsrec_no,
				exp0, exp1, exp2, 1, 1);

			TSREC_LOG_INF(
				"NOTICE: USER set intr_exp_trig_src_both:%#x => EN Vsync & 1st-Hsync interrupt on INTR_EN RG\n",
				user_intr_exp_trig_src_both);
		} else {
			mtk_cam_seninf_s_tsrec_intr_en(tsrec_no,
				exp0, exp1, exp2, exp_trig_src, 1);

			/* enable 1st exp Vsync interrupt */
			tsrec_intr_ctrl_exp_vsync_setup(tsrec_no, 0,
				exp0, exp1, exp2,
				p_tsrec_n_regs->sensor_hw_pre_latch_exp_num, 1);
		}
	} else {
		if (unlikely(user_intr_exp_trig_src_both)) {
			/* Vsync intr enable */
			mtk_cam_seninf_s_tsrec_intr_en_2(tsrec_no,
				exp0, exp1, exp2, 0, 1);

			/* 1st-Hsync intr enable */
			mtk_cam_seninf_s_tsrec_intr_en_2(tsrec_no,
				exp0, exp1, exp2, 1, 1);

			TSREC_LOG_INF(
				"NOTICE: USER set intr_exp_trig_src_both:%#x => EN Vsync & 1st-Hsync interrupt on INTR_EN_2 RG\n",
				user_intr_exp_trig_src_both);
		} else {
			mtk_cam_seninf_s_tsrec_intr_en_2(tsrec_no,
				exp0, exp1, exp2, exp_trig_src, 1);

			/* enable 1st exp Vsync interrupt */
			tsrec_intr_ctrl_exp_vsync_setup(tsrec_no, 1,
				exp0, exp1, exp2,
				p_tsrec_n_regs->sensor_hw_pre_latch_exp_num, 1);
		}
	}

	TSREC_LOG_INF(
		"tsrec_no:%u, en:1(%u/%u/%u, code:%#llx/%#llx, sensor pre-latch:%#x), 1st_exp_num:%u => intr_en(%#x/%#x), w_clr:%u\n",
		tsrec_no,
		exp0, exp1, exp2,
		p_src_cfg->vc_dt_src_code,
		p_src_cfg->cust_vc_dt_src_code,
		p_src_cfg->sensor_hw_pre_latch_exp_code,
		TSREC_1ST_EXP_NUM,
		TSREC_ATOMIC_READ(&tsrec_status.intr_en),
		TSREC_ATOMIC_READ(&tsrec_status.intr_en_2),
		TSREC_INTR_W_CLR_EN);
}


static void tsrec_seninf_exp_vc_dt_reset(const unsigned int tsrec_no)
{
	unsigned int i = 0;

	/* setup each exp. settings */
	/* TODO: add exp. cnt check for not every tsrec_no has 3 vc dt info */
	for (i = 0; i < TSREC_EXP_MAX_CNT; ++i) {
		/* reset to default RG status */
		mtk_cam_seninf_s_tsrec_trig_src(tsrec_no, (int)i, 0);

		/* reset to default exp vc dt RG status */
		mtk_cam_seninf_s_tsrec_exp_vc_dt(tsrec_no, i, 0, 0, 0);
	}
}


static void tsrec_seninf_exp_vc_dt_setup(const unsigned int tsrec_no,
	const unsigned int seninf_idx)
{
	struct tsrec_seninf_cfg_st *p_src_cfg = NULL;
	struct tsrec_n_regs_st *p_tsrec_n_regs = NULL;
	// unsigned long long vc_dt_src_code = 0;
	unsigned long long cust_vc_dt_src_code = 0;
	unsigned int i = 0;
	int ret;

	/* source needed info for starting settings configure procedure */
	ret = g_tsrec_seninf_cfg_st(seninf_idx, &p_src_cfg, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(p_src_cfg == NULL)) {
		TSREC_LOG_INF(
			"ERROR: seninf src_cfg[%u] is nullptr (%p), chk if reset & update vc/dt info or not, return   [tsrec_no:%u, seninf_idx:%u]\n",
			seninf_idx, p_src_cfg,
			tsrec_no,
			seninf_idx);
		return;
	}

	ret = g_tsrec_n_regs_st(tsrec_no, &p_tsrec_n_regs, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(p_tsrec_n_regs == NULL)) {
		TSREC_LOG_INF(
			"ERROR: tsrec_n_regs_st[%u] is nullptr (%p), return   [tsrec_no:%u, seninf_idx:%u]\n",
			tsrec_no, p_tsrec_n_regs,
			tsrec_no,
			seninf_idx);
		return;
	}

	/* setup basic info */
	p_tsrec_n_regs->seninf_idx = seninf_idx;


	/* setup info that needed */
	// vc_dt_src_code = p_src_cfg->vc_dt_src_code;
	cust_vc_dt_src_code = p_src_cfg->cust_vc_dt_src_code;


	/* setup each exp. settings */
	/* TODO: add exp. cnt check for maybe not every tsrec_no has 3 vc dt info */
	for (i = 0; i < TSREC_EXP_MAX_CNT; ++i) {
		const unsigned int exp_num_bit_shift = i + TSREC_EXP_ID_1;

		/* choose trigger src type (vsync/1st hsync) */
		mtk_cam_seninf_s_tsrec_trig_src(tsrec_no, (int)i,
			p_src_cfg->exp_vc_dt[i].trig_src);

		/* setup exp vc dt info */
		if ((cust_vc_dt_src_code >> exp_num_bit_shift) & 1U) {
			/* using custom assign */
			mtk_cam_seninf_s_tsrec_exp_vc_dt(tsrec_no, i,
				p_src_cfg->cust_vc_dt[i].vsync_vc,
				p_src_cfg->cust_vc_dt[i].hsync_vc,
				p_src_cfg->cust_vc_dt[i].hsync_dt);
		} else {
			/* using default assign */
			mtk_cam_seninf_s_tsrec_exp_vc_dt(tsrec_no, i,
				p_src_cfg->exp_vc_dt[i].vsync_vc,
				p_src_cfg->exp_vc_dt[i].hsync_vc,
				p_src_cfg->exp_vc_dt[i].hsync_dt);
		}
	}
}


static void tsrec_n_settings_clear(const unsigned int tsrec_no)
{
	/* check if tsrec_no is valid */
	if (unlikely(!chk_tsrec_no_valid(tsrec_no, __func__))) {
		TSREC_LOG_INF(
			"NOTICE: get non-valid tsrec_no:%u (tsrec_hw_cnt:%u), skip config\n",
			tsrec_no,
			tsrec_status.tsrec_hw_cnt);
		return;
	}

	/* first disable/stop this tsrec hw */
	mtk_cam_seninf_s_tsrec_top_cfg_clk_en_bit(tsrec_no, 0);

	/* INTR disable */
	tsrec_intr_reset(tsrec_no);

	/* reset exp vc dt info */
	tsrec_seninf_exp_vc_dt_reset(tsrec_no);

	/* reset/setup info for debugging */
	tsrec_n_regs_st_reset(tsrec_no);
}


static void tsrec_n_settings_start(const unsigned int tsrec_no,
	const unsigned int seninf_idx)
{
	const unsigned int seninf_hw_cnt = tsrec_status.seninf_hw_cnt;

	/* check if tsrec_no is valid */
	if (unlikely(!chk_tsrec_no_valid(tsrec_no, __func__))) {
		TSREC_LOG_INF(
			"NOTICE: get non-valid tsrec_no:%u (tsrec_hw_cnt:%u), seninf_idx:%u, skip config\n",
			tsrec_no,
			tsrec_status.tsrec_hw_cnt,
			seninf_idx);
		return;
	}

	/* check case / error handling */
	if (unlikely(seninf_idx >= seninf_hw_cnt)) {
		TSREC_LOG_INF(
			"ERROR: non-valid seninf_idx:%u (DTS seninf_hw_cnt:%u), tsrec_no:%u, return\n",
			seninf_idx,
			seninf_hw_cnt,
			tsrec_no);
		return;
	}

	/* !!! set tsrec SW RST !!! */
	mtk_cam_seninf_s_tsrec_sw_rst(tsrec_no, 1);

	/* reset tsrec n regs info (chk if overwrite or not at before flow) */
	tsrec_n_regs_st_reset(tsrec_no);

	/* first disable/stop this tsrec hw */
	mtk_cam_seninf_s_tsrec_top_cfg_clk_en_bit(tsrec_no, 0);

	/* INTR disable */
	tsrec_intr_reset(tsrec_no);

	/* setup exp vc dt info */
	tsrec_seninf_exp_vc_dt_setup(tsrec_no, seninf_idx);

	/* INTR enable */
	tsrec_intr_ctrl(tsrec_no, seninf_idx);

	/* enable/start this tsrec hw */
	mtk_cam_seninf_s_tsrec_top_cfg_clk_en_bit(tsrec_no, 1);

	/* !!! unset tsrec SW RST !!! */
	mtk_cam_seninf_s_tsrec_sw_rst(tsrec_no, 0);
}


static void tsrec_chk_auto_self_disable(void)
{
	struct tsrec_n_regs_st *ptr = NULL;
	const unsigned int tsrec_hw_cnt = tsrec_status.tsrec_hw_cnt;
	unsigned int i = 0;
	int ret;

	for (i = 0; i < tsrec_hw_cnt; ++i) {
		ret = g_tsrec_n_regs_st(i, &ptr, __func__);
		if (unlikely(ret != 0))
			return;
		if ((ptr == NULL) || (ptr->en == 0))
			continue;

		TSREC_LOG_DBG(
			"suspend procedure, auto clear/stop still enable hw, tsrec_no:%u, top_cfg:%#x\n",
			i, TSREC_ATOMIC_READ(&tsrec_status.top_cfg));

		tsrec_n_settings_clear(i);
		tsrec_worker_flush(i);
		mtk_cam_seninf_tsrec_dbg_dump_tsrec_n_regs_info(__func__);
	}
}


static void tsrec_n_timestamp_ticks_re_order(const unsigned int tsrec_no,
	unsigned long long tick_ordered[][TSREC_TS_REC_MAX_CNT])
{
	struct tsrec_n_regs_st *p_tsrec_n_regs = NULL;
	struct tsrec_n_timestamp_st *p_ts_st = NULL;
	unsigned int i = 0, j = 0;
	int ret;

	/* error handling (unexpected case) */
	if (unlikely(!chk_tsrec_no_valid(tsrec_no, __func__))) {
		TSREC_LOG_INF(
			"ERROR: get non-valid tsrec_no:%u (tsrec_hw_cnt:%u), return\n",
			tsrec_no,
			tsrec_status.tsrec_hw_cnt);
		return;
	}

	if (unlikely(tick_ordered == NULL)) {
		TSREC_LOG_INF("ERROR: tick_ordered is NULL, return\n");
		return;
	}

	ret = g_tsrec_n_regs_st(tsrec_no, &p_tsrec_n_regs, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(p_tsrec_n_regs == NULL)) {
		TSREC_LOG_INF(
			"ERROR: tsrec_n_regs_st[%u] is nullptr (%p), return   [tsrec_no:%u]\n",
			tsrec_no, p_tsrec_n_regs,
			tsrec_no);
		return;
	}
	p_ts_st = &p_tsrec_n_regs->ts_recs;

	/* reordering */
	for (i = 0; i < TSREC_EXP_MAX_CNT; ++i) {
		for (j = 0; j < TSREC_TS_REC_MAX_CNT; ++j) {
			const unsigned int idx = TSREC_RING_BACK(
				p_ts_st->exp_recs[i].curr_idx, j+1);

			tick_ordered[i][j] = p_ts_st->exp_recs[i].ticks[idx];

#ifndef REDUCE_TSREC_LOG
			TSREC_LOG_INF(
				"curr_idx:%u, ring_back:%u => idx:%u, tick_ordered[%u][%u]:%llu\n",
				p_ts_st->exp_recs[i].curr_idx,
				j+1,
				idx,
				i, j,
				tick_ordered[i][j]);
#endif // !REDUCE_TSREC_LOG
		}
	}
}


/******************************************************************************
 * TSREC flow/logic CTRL functions
 *****************************************************************************/
void mtk_cam_seninf_tsrec_timer_enable(const unsigned int en)
{
	unsigned int has_reg_op = 0; // 0:NO / 1:set / 2:clr
	const int timer_en_cnt_before =
		TSREC_ATOMIC_READ(&tsrec_status.timer_en_cnt);

	if (unlikely(!chk_exist_tsrec_hw(__func__, 1)))
		return;

	if (en) {
		/* check if NOT enable before */
		if (TSREC_ATOMIC_READ(&tsrec_status.timer_en_cnt) == 0) {
			/* !!! update pwr status for TSREC RG read/write operation !!! */
			/* (timer enable is synchronized to seninf runtime resume/suspend) */
			notify_tsrec_update_timer_en_status(1);

			mtk_cam_seninf_s_tsrec_timer_cfg(1);
			tsrec_irq_enable(1);

			// mark as set RG
			has_reg_op = 1;
		}

		/* increase timer en cnt */
		TSREC_ATOMIC_INC(&tsrec_status.timer_en_cnt);
	} else {
		/* decrease timer en cnt */
		TSREC_ATOMIC_DEC(&tsrec_status.timer_en_cnt);

		/* check if en cnt go back to zero */
		if (TSREC_ATOMIC_READ(&tsrec_status.timer_en_cnt) == 0) {
			tsrec_irq_enable(0);
			tsrec_chk_auto_self_disable();
			mtk_cam_seninf_s_tsrec_timer_cfg(0);

			/* !!! update pwr status for TSREC RG read/write operation !!! */
			/* (timer enable is synchronized to seninf runtime resume/suspend) */
			notify_tsrec_update_timer_en_status(0);

			// mark as clr RG
			has_reg_op = 2;
		}

		/* error handling, timer en cnt is negative case */
		if (TSREC_ATOMIC_READ(&tsrec_status.timer_en_cnt) < 0) {
			TSREC_LOG_INF(
				"WARNING: timer_en_cnt:%d decrease to negative value, auto set to zero, check if calling enable/disable were in pair\n",
				TSREC_ATOMIC_READ(&tsrec_status.timer_en_cnt));

			TSREC_ATOMIC_SET(0, &tsrec_status.timer_en_cnt);
		}
	}

	TSREC_LOG_INF(
		"timer_en_cnt:(%d => %d), timer_cfg:%#x, timer_en_status:%u, has_reg_op:%u(0:NO/1:set/2:clr), IRQ:%d(en:%u)   [en:%u]\n",
		timer_en_cnt_before,
		TSREC_ATOMIC_READ(&tsrec_status.timer_en_cnt),
		TSREC_ATOMIC_READ(&tsrec_status.timer_cfg),
		tsrec_status.timer_en_status,
		has_reg_op,
		tsrec_status.tsrec_irq,
		tsrec_status.tsrec_irq_en_status,
		en);
}


void mtk_cam_seninf_tsrec_reset_vc_dt_info(const unsigned int seninf_idx)
{
	const unsigned int seninf_hw_cnt = tsrec_status.seninf_hw_cnt;

	if (unlikely(!chk_exist_tsrec_hw(__func__, 0)))
		return;

	/* check case / error handling */
	if (unlikely(seninf_idx >= seninf_hw_cnt)) {
		TSREC_LOG_INF(
			"ERROR: non-valid seninf_idx:%u (DTS seninf_hw_cnt:%u), return\n",
			seninf_idx,
			seninf_hw_cnt);
		return;
	}

	/* reset (maybe will with init) */
	tsrec_seninf_cfg_st_reset(seninf_idx);
}


void mtk_cam_seninf_tsrec_update_vc_dt_info(const unsigned int seninf_idx,
	const struct mtk_cam_seninf_tsrec_vc_dt_info *vc_dt_info)
{
	struct tsrec_seninf_cfg_st *p_src_cfg = NULL;
	const unsigned int exp_trig_src = g_tsrec_exp_trig_src();
	int ret;

	if (unlikely(!chk_exist_tsrec_hw(__func__, 0)))
		return;

	ret = g_tsrec_seninf_cfg_st(seninf_idx, &p_src_cfg, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(p_src_cfg == NULL)) {
		TSREC_LOG_INF(
			"WARNING: seninf src_cfg[%u] is nullptr (%p), due to not calling reset vc/dt info before calling update, auto handle this case\n",
			seninf_idx, p_src_cfg);

		/* should be called before updating vc/dt info */
		/* (by calling reset vc/dt info) */
		/* (memory maybe alloc failed) */
		tsrec_seninf_cfg_st_reset(seninf_idx);

		/* get pointer and check it again */
		ret = g_tsrec_seninf_cfg_st(seninf_idx, &p_src_cfg, __func__);
		if (unlikely(ret != 0))
			return;
		if (unlikely(p_src_cfg == NULL)) {
			TSREC_LOG_INF(
				"ERROR: seninf src_cfg[%u] reset failed (%p), src_cfg[%u]:%p, return   [seninf_idx:%u, vc:%u, dt:%u, out_pad:%u, cust_assign_to_tsrec_exp_id:%u, is_sensor_hw_pre_latch_exp:%u]\n",
				seninf_idx, p_src_cfg,
				seninf_idx, tsrec_src.src_cfg[seninf_idx],
				seninf_idx,
				vc_dt_info->vc,
				vc_dt_info->dt,
				vc_dt_info->out_pad,
				vc_dt_info->cust_assign_to_tsrec_exp_id,
				vc_dt_info->is_sensor_hw_pre_latch_exp);
			return;
		}
	}
	if (unlikely(exp_trig_src != TSREC_EXP_TRIG_SRC_DEF)) {
		TSREC_LOG_INF(
			"NOTICE: USER manually assign exp_trig_src:%u => use exp_trig_src:%u\n",
			tsrec_con_mgr.exp_trig_src,
			exp_trig_src);
	}


	/* !!! setup basic info !!! */
	p_src_cfg->seninf_idx = seninf_idx;
	p_src_cfg->tsrec_no = TSREC_NO_NONE;

	/* by pad type, setup vc/dt info */
	switch (vc_dt_info->out_pad) {
	case PAD_SRC_RAW0:
		p_src_cfg->vc_dt_src_code |= (1ULL << PAD_SRC_RAW0);

		p_src_cfg->exp_vc_dt[0].vsync_vc = vc_dt_info->vc;
		p_src_cfg->exp_vc_dt[0].hsync_vc = vc_dt_info->vc;
		p_src_cfg->exp_vc_dt[0].hsync_dt = vc_dt_info->dt;

		p_src_cfg->exp_vc_dt[0].trig_src = exp_trig_src;

		if (vc_dt_info->is_sensor_hw_pre_latch_exp
			&& vc_dt_info->cust_assign_to_tsrec_exp_id ==
				TSREC_EXP_ID_AUTO) {
			p_src_cfg->sensor_hw_pre_latch_exp_code |=
				(1ULL << TSREC_EXP_ID_1);
		}
		break;

	case PAD_SRC_RAW1:
		p_src_cfg->vc_dt_src_code |= (1ULL << PAD_SRC_RAW1);

		p_src_cfg->exp_vc_dt[1].vsync_vc = vc_dt_info->vc;
		p_src_cfg->exp_vc_dt[1].hsync_vc = vc_dt_info->vc;
		p_src_cfg->exp_vc_dt[1].hsync_dt = vc_dt_info->dt;

		p_src_cfg->exp_vc_dt[1].trig_src = exp_trig_src;

		if (vc_dt_info->is_sensor_hw_pre_latch_exp
			&& vc_dt_info->cust_assign_to_tsrec_exp_id ==
				TSREC_EXP_ID_AUTO) {
			p_src_cfg->sensor_hw_pre_latch_exp_code |=
				(1ULL << TSREC_EXP_ID_2);
		}
		break;

	case PAD_SRC_RAW2:
		p_src_cfg->vc_dt_src_code |= (1ULL << PAD_SRC_RAW2);

		p_src_cfg->exp_vc_dt[2].vsync_vc = vc_dt_info->vc;
		p_src_cfg->exp_vc_dt[2].hsync_vc = vc_dt_info->vc;
		p_src_cfg->exp_vc_dt[2].hsync_dt = vc_dt_info->dt;

		p_src_cfg->exp_vc_dt[2].trig_src = exp_trig_src;

		if (vc_dt_info->is_sensor_hw_pre_latch_exp
			&& vc_dt_info->cust_assign_to_tsrec_exp_id ==
				TSREC_EXP_ID_AUTO) {
			p_src_cfg->sensor_hw_pre_latch_exp_code |=
				(1ULL << TSREC_EXP_ID_3);
		}
		break;

	default:
		break;
	}

	/* by user assign value, setup custom exp vc/dt info */
	switch (vc_dt_info->cust_assign_to_tsrec_exp_id) {
	case TSREC_EXP_ID_AUTO:
		/* no custom assign => using results that by pad */
		break;

	case TSREC_EXP_ID_1:
		p_src_cfg->cust_vc_dt_src_code |= (1ULL << TSREC_EXP_ID_1);

		p_src_cfg->cust_vc_dt[0].vsync_vc = vc_dt_info->vc;
		p_src_cfg->cust_vc_dt[0].hsync_vc = vc_dt_info->vc;
		p_src_cfg->cust_vc_dt[0].hsync_dt = vc_dt_info->dt;

		p_src_cfg->cust_vc_dt[0].trig_src = exp_trig_src;

		if (vc_dt_info->is_sensor_hw_pre_latch_exp) {
			p_src_cfg->sensor_hw_pre_latch_exp_code |=
				(1ULL << TSREC_EXP_ID_1);
		}
		break;

	case TSREC_EXP_ID_2:
		p_src_cfg->cust_vc_dt_src_code |= (1ULL << TSREC_EXP_ID_2);

		p_src_cfg->cust_vc_dt[1].vsync_vc = vc_dt_info->vc;
		p_src_cfg->cust_vc_dt[1].hsync_vc = vc_dt_info->vc;
		p_src_cfg->cust_vc_dt[1].hsync_dt = vc_dt_info->dt;

		p_src_cfg->cust_vc_dt[1].trig_src = exp_trig_src;

		if (vc_dt_info->is_sensor_hw_pre_latch_exp) {
			p_src_cfg->sensor_hw_pre_latch_exp_code |=
				(1ULL << TSREC_EXP_ID_2);
		}
		break;

	case TSREC_EXP_ID_3:
		p_src_cfg->cust_vc_dt_src_code |= (1ULL << TSREC_EXP_ID_3);

		p_src_cfg->cust_vc_dt[2].vsync_vc = vc_dt_info->vc;
		p_src_cfg->cust_vc_dt[2].hsync_vc = vc_dt_info->vc;
		p_src_cfg->cust_vc_dt[2].hsync_dt = vc_dt_info->dt;

		p_src_cfg->cust_vc_dt[2].trig_src = exp_trig_src;

		if (vc_dt_info->is_sensor_hw_pre_latch_exp) {
			p_src_cfg->sensor_hw_pre_latch_exp_code |=
				(1ULL << TSREC_EXP_ID_3);
		}
		break;

	default:
		TSREC_LOG_INF(
			"WARNING: src_cfg[%u]:(seninf_idx:%u, tsrec_no:%u, get non-valid cust_assign_to_tsrec_exp_id:%u, do nothing, plz chk sensor frame description\n",
			seninf_idx,
			p_src_cfg->seninf_idx,
			p_src_cfg->tsrec_no,
			vc_dt_info->cust_assign_to_tsrec_exp_id);
		break;
	}

	// mtk_cam_seninf_tsrec_dbg_dump_vc_dt_info(seninf_idx, __func__);
}


void mtk_cam_seninf_tsrec_n_reset(const unsigned int seninf_idx)
{
	struct tsrec_seninf_cfg_st *p_src_cfg = NULL;
	struct tsrec_n_regs_st *p_tsrec_n_regs = NULL;
	const unsigned int tsrec_hw_cnt = tsrec_status.tsrec_hw_cnt;
	unsigned int i = 0;
	int ret;

	if (unlikely(!chk_exist_tsrec_hw(__func__, 0)))
		return;

	/* below API will reset all info that updated before calling reset */
	/* and will lead to lost data if this flow is right. */
	// tsrec_seninf_cfg_st_reset(seninf_idx);
	//
	// so only reset tsrec_no instead.
	ret = g_tsrec_seninf_cfg_st(seninf_idx, &p_src_cfg, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(p_src_cfg == NULL)) {
		TSREC_LOG_INF(
			"WARNING: seninf src_cfg[%u] is nullptr (%p), skip clear for tsrec_no (to TSREC_NO_NONE)\n",
			seninf_idx, p_src_cfg);
	} else
		p_src_cfg->tsrec_no = TSREC_NO_NONE;


	/* !!! [below api procedure] clear & stop this tsrec !!! */
	for (i = 0; i < tsrec_hw_cnt; ++i) {
		ret = g_tsrec_n_regs_st(i, &p_tsrec_n_regs, __func__);
		if (unlikely(ret != 0))
			return;
		if ((p_tsrec_n_regs == NULL)
				|| (p_tsrec_n_regs->seninf_idx != seninf_idx))
			continue;

		/* reset/clear this tsrec settings */
		tsrec_n_settings_clear(i);
		tsrec_worker_flush(i);
		mtk_cam_seninf_tsrec_dbg_dump_tsrec_n_regs_info(__func__);
	}
}


void mtk_cam_seninf_tsrec_n_start(const unsigned int seninf_idx,
	const unsigned int tsrec_no)
{
	struct tsrec_seninf_cfg_st *p_src_cfg = NULL;
	int ret;

	if (unlikely(!chk_exist_tsrec_hw(__func__, 0)))
		return;

	/* check if tsrec_no is valid */
	if (unlikely(!chk_tsrec_no_valid(tsrec_no, __func__))) {
		TSREC_LOG_INF(
			"NOTICE: non-valid tsrec_no:%u (tsrec_hw_cnt:%u), seninf_idx:%u, skip config\n",
			tsrec_no,
			tsrec_status.tsrec_hw_cnt,
			seninf_idx);
		return;
	}

	ret = g_tsrec_seninf_cfg_st(seninf_idx, &p_src_cfg, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(p_src_cfg == NULL)) {
		TSREC_LOG_INF(
			"ERROR: seninf src_cfg[%u] is nullptr (%p) => do NOT have vc/dt info for setup TSREC, return\n",
			seninf_idx, p_src_cfg);
		return;
	}

	/* assign/set tsrec no. <=> seninf relationship */
	p_src_cfg->tsrec_no = tsrec_no;

	/* for debugging, check vc dt info that will be setup below */
	mtk_cam_seninf_tsrec_dbg_dump_vc_dt_info(seninf_idx, __func__);

	/* check if tsrec_no has been used before */
	ret = tsrec_n_regs_st_chk_used_status(tsrec_no);
	if (ret != 0) {
		TSREC_LOG_INF(
			"WARNING: chk_used_status:%d, tsrec_no:%u, overwrite the last tsrec settings\n",
			ret,
			tsrec_no);

		mtk_cam_seninf_tsrec_dbg_dump_tsrec_n_regs_info(__func__);

		/* TODO: add some case handling procedure below? */
	}


	/* !!! [below api procedure] start config/setup this tsrec !!! */
	tsrec_n_settings_start(tsrec_no, seninf_idx);
	mtk_cam_seninf_tsrec_dbg_dump_tsrec_n_regs_info(__func__);
}


static void mtk_cam_seninf_tsrec_n_update_irq_info_st(
	const unsigned int tsrec_no,
	const struct tsrec_irq_info_st *irq_info)
{
	struct tsrec_n_regs_st *ptr = NULL;
	int ret;

	/* error handling (unexpected case) */
	if (unlikely(irq_info == NULL))
		return;
	if (unlikely(!chk_tsrec_no_valid(tsrec_no, __func__))) {
		TSREC_LOG_INF(
			"ERROR: get non-valid tsrec_no:%u (tsrec_hw_cnt:%u), irq_info(sys_ts:%llu(ns)(~ +%llu ns), tsrec_ts:%llu(us), intr(|%#x/%#x)), return\n",
			tsrec_no,
			tsrec_status.tsrec_hw_cnt,
			irq_info->sys_ts_ns,
			irq_info->duration_ns,
			irq_info->tsrec_ts_us,
			irq_info->intr_status,
			irq_info->intr_status_2);
		return;
	}

	ret = g_tsrec_n_regs_st(tsrec_no, &ptr, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(ptr == NULL)) {
		TSREC_LOG_INF(
			"ERROR: tsrec_n_regs_st[%u] is nullptr (%p), return\n",
			tsrec_no, ptr);
		return;
	}
	ptr->irq_info = *irq_info;
}


void mtk_cam_seninf_tsrec_query_ts_records(const unsigned int tsrec_no)
{
	struct tsrec_n_regs_st *ptr = NULL;
	struct tsrec_n_timestamp_st ts_recs = {0};
	unsigned long long curr_tick = 0;
	unsigned long long curr_sys_time_ns = 0;
	unsigned int reg_ts_cnt = 0;
	unsigned int i = 0, j = 0;
	int ret;

	if (unlikely(!chk_exist_tsrec_hw(__func__, 1)))
		return;

	/* prepare basic info */
	curr_tick = mtk_cam_seninf_tsrec_latch_time();
	curr_sys_time_ns = ktime_get_boottime_ns();

	/* error handling (unexpected case) */
	if (unlikely(!chk_tsrec_no_valid(tsrec_no, __func__))) {
		TSREC_LOG_INF(
			"ERROR: get non-valid tsrec_no:%u (tsrec_hw_cnt:%u), curr_ts:%llu(%llu), sys_ts_ns:%llu, dump dbg info then return\n",
			tsrec_no,
			tsrec_status.tsrec_hw_cnt,
			curr_tick/TSREC_TICK_FACTOR,
			curr_tick,
			curr_sys_time_ns);

		/* for debugging */
		mtk_cam_seninf_tsrec_dbg_dump_tsrec_n_regs_info(__func__);

		return;
	}

	/* set basic info */
	ts_recs.curr_tick = curr_tick;
	ts_recs.curr_sys_time_ns = curr_sys_time_ns;

	/* get all exp timestamp cnt */
	reg_ts_cnt = mtk_cam_seninf_g_tsrec_ts_cnt(tsrec_no);

	/* query and setup timestamp info */
	for (i = 0; i < TSREC_EXP_MAX_CNT; ++i) {
		ts_recs.exp_recs[i].curr_idx =
			mtk_cam_seninf_g_tsrec_ts_cnt_by_exp_n(reg_ts_cnt, i);

		for (j = 0; j < TSREC_TS_REC_MAX_CNT; ++j) {
			/* direct query, without any re-order */
			ts_recs.exp_recs[i].ticks[j] =
				mtk_cam_seninf_g_tsrec_exp_cnt(tsrec_no, i, j);
		}
	}


	/* !!! sync new ts recs data for keeping it !!! */
	ret = g_tsrec_n_regs_st(tsrec_no, &ptr, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(ptr == NULL)) {
		TSREC_LOG_INF(
			"ERROR: tsrec_n_regs_st[%u] is nullptr (%p), can NOT get correspond seninf_idx and keeping current new timestamp data\n",
			tsrec_no, ptr);
		return;
	}
	ts_recs.seninf_idx = ptr->seninf_idx;
	ptr->ts_recs = ts_recs;
}


void mtk_cam_seninf_tsrec_get_timestamp_info(const unsigned int tsrec_no,
	struct mtk_cam_seninf_tsrec_timestamp_info *ts_info)
{
	struct tsrec_n_regs_st *ptr = NULL;
	unsigned long long tick_ordered[TSREC_EXP_MAX_CNT][TSREC_TS_REC_MAX_CNT] = {0};
	unsigned int i, j;
	int ret;

	if (unlikely(!chk_exist_tsrec_hw(__func__, 1)))
		return;

	/* error handling (unexpected case) */
	if (unlikely(ts_info == NULL)) {
		TSREC_LOG_INF("ERROR: get ts_info:%p, return\n", ts_info);
		return;
	}
	if (unlikely(!chk_tsrec_no_valid(tsrec_no, __func__))) {
		TSREC_LOG_INF(
			"ERROR: get non-valid tsrec_no:%u (tsrec_hw_cnt:%u), ts_info:%p, return\n",
			tsrec_no,
			tsrec_status.tsrec_hw_cnt,
			ts_info);
		return;
	}

	ret = g_tsrec_n_regs_st(tsrec_no, &ptr, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(ptr == NULL)) {
		TSREC_LOG_INF(
			"ERROR: tsrec_n_regs_st[%u] is nullptr (%p), return\n",
			tsrec_no, ptr);
		return;
	}

	/* reordering ticks data */
	tsrec_n_timestamp_ticks_re_order(tsrec_no, tick_ordered);

	/* manually sync/copy information to user's structure */
	/* that define in mtk_camera-v4l2-controls.h */
	ts_info->tsrec_no = tsrec_no;
	ts_info->seninf_idx = ptr->seninf_idx;
	ts_info->tick_factor = TSREC_TICK_FACTOR;
	ts_info->irq_sys_time_ns = ptr->irq_info.sys_ts_ns;
	ts_info->irq_tsrec_ts_us = ptr->irq_info.tsrec_ts_us;
	ts_info->tsrec_curr_tick = ptr->ts_recs.curr_tick;
	for (i = 0; i < TSREC_EXP_MAX_CNT; ++i) {
		for (j = 0; j < TSREC_TS_REC_MAX_CNT; ++j) {
			ts_info->exp_recs[i].ts_us[j] =
				TSREC_TICK_TO_US(tick_ordered[i][j]);
		}
	}
}


/******************************************************************************
 * TSREC ISR functions
 *****************************************************************************/

/*---------------------------------------------------------------------------*/
// interrupt handler --- bottom-half (sub functions)
/*---------------------------------------------------------------------------*/
static void tsrec_isr_event_handler(int irq, void *data,
	const struct tsrec_irq_info_st *irq_info,
	const unsigned int tsrec_no, const unsigned int status)
{
	struct tsrec_n_regs_st *p_tsrec_n_regs = NULL;
	const unsigned int fl_target_exp_num = TSREC_1ST_EXP_NUM;
	const unsigned int exp_trig_src = g_tsrec_exp_trig_src();
	/* See enum tsrec_work_event_flag for details */
	unsigned int work_event_info = 0;
	unsigned int exp_num, pre_latch_exp_num;
	int ret;

	ret = g_tsrec_n_regs_st(tsrec_no, &p_tsrec_n_regs, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(p_tsrec_n_regs == NULL)) {
		TSREC_LOG_INF(
			"ERROR: tsrec_n_regs_st[%u] is nullptr (%p), return   [tsrec_no:%u, status:%#x, irq_info(sys_ts:%llu(ns)(~ +%llu ns), tsrec_ts:%llu(us), intr(|%#x/%#x))]\n",
			tsrec_no, p_tsrec_n_regs,
			tsrec_no, status,
			irq_info->sys_ts_ns,
			irq_info->duration_ns,
			irq_info->tsrec_ts_us,
			irq_info->intr_status,
			irq_info->intr_status_2);
		return;
	}

	pre_latch_exp_num = p_tsrec_n_regs->sensor_hw_pre_latch_exp_num;

	/* check status and convert to work event info */
	for (exp_num = 0; exp_num < TSREC_EXP_MAX_CNT; ++exp_num) {
		const unsigned int vsync_mask = (1U << exp_num);
		const unsigned int hsync_mask = (1U << exp_num) << 16;
		const unsigned int exp_num_mask = vsync_mask | hsync_mask;

		/* check exp 0~2 status */
		/* --- e.g., exp 0~2 => 0x10001 / 0x20002 / 0x40004 --- */
		if (!(status & exp_num_mask))
			continue;

		/* check exp_trig_src: (0 => Vsync) / (1 => 1st-Hsync) */

		/* !!! Vsync (intr status) job event !!! */
		/* --- exp 0~2 => 0x00001 / 0x00002 / 0x00004 --- */
		if (status & vsync_mask) {
			if (exp_trig_src == 0) {
				if (fl_target_exp_num == exp_num) {
					work_event_info |=
						TSREC_WORK_1ST_VSYNC;
					work_event_info |=
						TSREC_WORK_SEND_TS_TO_SENSOR;
				}
				work_event_info |= TSREC_WORK_QUERY_TS;
			}
			if (exp_trig_src == 1) {
				if (fl_target_exp_num == exp_num) {
					work_event_info |=
						TSREC_WORK_1ST_VSYNC;
				} else if (exp_num != pre_latch_exp_num) {
					work_event_info |=
						TSREC_WORK_DBG_DUMP_IRQ_INFO;
				}
			}
			if (pre_latch_exp_num == exp_num) {
				work_event_info |=
					TSREC_WORK_SENSOR_HW_PRE_LATCH;
			}
		}
		/* !!! [END] Vsync (intr status) job event [END] !!! */


		/* !!! 1st-Hsync (intr status) job event !!! */
		/* --- exp 0~2 => 0x10000 / 0x20000 / 0x40000 --- */
		if (status & hsync_mask) {
			if (exp_trig_src == 1) {
				if (fl_target_exp_num == exp_num) {
					work_event_info |=
						TSREC_WORK_SEND_TS_TO_SENSOR;
				}
				work_event_info |= TSREC_WORK_QUERY_TS;
			}
			if (exp_trig_src == 0) {
				work_event_info |=
					TSREC_WORK_DBG_DUMP_IRQ_INFO;
			}
		}
		/* !!! [END] 1st-Hsync (intr status) job event [END] !!! */
	}

	tsrec_work_setup(irq, data, irq_info, tsrec_no, work_event_info);
}


static unsigned int tsrec_irq_intr_status_checker(int irq, void *data,
	const struct tsrec_irq_info_st *irq_info,
	const unsigned int intr_status_rg_id)
{
	const unsigned int intr_chk_mask = TSREC_INTR_CHK_MASK;
	unsigned int intr_status, start, end;
	unsigned int status = 0, done = 0, shift = 0;
	unsigned int i = 0;

	/* check intr status RG id, get intr status and determin check range */
	switch (intr_status_rg_id) {
	case TSREC_RG_INTR_STATUS:
		/* tsrec_no: a~d (0~3) */
		start = 0;
		end = TSREC_INTR_EN_MAX_NUM;
		intr_status = irq_info->intr_status;

		break;
	case TSREC_RG_INTR_STATUS_2:
		/* tsrec_no: e~f (4~5) */
		start = TSREC_INTR_EN_MAX_NUM;
		end = TSREC_INTR_EN_2_MAX_NUM;
		intr_status = irq_info->intr_status_2;

		break;
	default:
		TSREC_LOG_INF(
			"ERROR: get invalid intr_status_rg_id:%u (%u ~ %u), return 0\n",
			intr_status_rg_id,
			TSREC_RG_INTR_STATUS, TSREC_RG_INTR_STATUS_2);
		return 0;
	}

	/* go through, check each tsrec interrupt status */
	for (i = start; i < end; ++i) {
		shift = (i - start) * TSREC_EXP_MAX_CNT;
		status = ((intr_status >> shift) & intr_chk_mask);
		/* get & check individual TSREC intr status */
		if (status) {
			tsrec_isr_event_handler(irq, data, irq_info, i, status);
			done |= (status << shift);
		}
	}

	/* case check for handling all tsrec */
	if (unlikely(intr_status != done)) {
		TSREC_LOG_INF(
			"ERROR: get intr_status:%#x (tsrec_no: %u ~ %u), but only handle/done:%#x\n",
			intr_status, start, end - 1, done);
	}

	return done;
}


static void tsrec_irq_handler(int irq, void *data,
	const struct tsrec_irq_info_st *irq_info)
{
	unsigned int done;

	/* check each interrupt status RGs value */
	if (irq_info->intr_status) {
		done = tsrec_irq_intr_status_checker(
			irq, data, irq_info, TSREC_RG_INTR_STATUS);

		/* please call this API whether write clear is enabled or not */
		mtk_cam_seninf_clr_tsrec_intr_status(done);
	}

	if (irq_info->intr_status_2) {
		done = tsrec_irq_intr_status_checker(
			irq, data, irq_info, TSREC_RG_INTR_STATUS_2);

		/* please call this API whether write clear is enabled or not */
		mtk_cam_seninf_clr_tsrec_intr_status_2(done);
	}
}
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
// interrupt handler --- top-half (main function)
/*---------------------------------------------------------------------------*/
static irqreturn_t mtk_irq_seninf_tsrec(int irq, void *data)
{
	struct tsrec_irq_info_st irq_info = {0};
	unsigned long long start, end, tsrec_tick;
	unsigned int intr_status, intr_status_2;
	unsigned int wake_thread = 0;

	tsrec_tick = mtk_cam_seninf_tsrec_latch_time();
	start = ktime_get_boottime_ns();

	/* get TSREC interrupt status info */
	intr_status	 = mtk_cam_seninf_g_tsrec_intr_status();
	intr_status_2	 = mtk_cam_seninf_g_tsrec_intr_status_2();

	end = ktime_get_boottime_ns();

	/* sync irq information */
	irq_info.intr_status = intr_status;
	irq_info.intr_status_2 = intr_status_2;
	irq_info.tsrec_ts_us = TSREC_TICK_TO_US(tsrec_tick);
	irq_info.sys_ts_ns = start;
	irq_info.duration_ns = end - start;

	if (tsrec_push_irq_info_msgfifo(&irq_info) == 0)
		wake_thread = 1;

	return (wake_thread) ? IRQ_WAKE_THREAD : IRQ_HANDLED;
}


/*---------------------------------------------------------------------------*/
// interrupt handler --- bottom-half (main function)
/*---------------------------------------------------------------------------*/
static irqreturn_t mtk_thread_irq_seninf_tsrec(int irq, void *data)
{
	struct tsrec_irq_info_st irq_info;

#ifndef FS_UT
	/* error handling (check case and print information) */
	if (unlikely(atomic_cmpxchg(&tsrec_irq_event.is_fifo_overflow, 1, 0)))
		TSREC_LOG_INF("WARNING: irq msg fifo overflow\n");
#endif // !FS_UT

	while (tsrec_chk_irq_info_msgfifo_not_empty()) {
		tsrec_pop_irq_info_msgfifo(&irq_info);

#if !defined(REDUCE_TSREC_LOG_IN_ISR_FUNC)
		TSREC_LOG_DBG(
			"IRQ: sys_ts:%llu(ns)(~ +%llu ns), tsrec_ts:%llu(us), intr(%#x/%#x|%#x/%#x)\n",
			irq_info.sys_ts_ns,
			irq_info.duration_ns,
			irq_info.tsrec_ts_us,
			TSREC_ATOMIC_READ(&tsrec_status.intr_en),
			TSREC_ATOMIC_READ(&tsrec_status.intr_en_2),
			irq_info.intr_status,
			irq_info.intr_status_2);
#endif // !REDUCE_TSREC_LOG_IN_ISR_FUNC

		/* inform interrupt information */
		tsrec_irq_handler(irq, data, &irq_info);
	}

	return IRQ_HANDLED;
}


/******************************************************************************
 * TSREC debug/unit-test functions
 *****************************************************************************/
void mtk_cam_seninf_tsrec_dbg_dump_vc_dt_info(const unsigned int seninf_idx,
	const char *caller)
{
	struct tsrec_seninf_cfg_st *p_src_cfg = NULL;
	int ret;

	if (unlikely(!chk_exist_tsrec_hw(__func__, 1)))
		return;

	ret = g_tsrec_seninf_cfg_st(seninf_idx, &p_src_cfg, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(p_src_cfg == NULL)) {
		TSREC_LOG_INF(
			"NOTICE: seninf src_cfg[%u] is nullptr (%p), do NOT have vc/dt info, return\n",
			seninf_idx, p_src_cfg);
		return;
	}

	TSREC_LOG_INF(
		"[%s] src_cfg[%u]:(seninf_idx:%u, tsrec_no:%u, vc_dt(exp(0:(%#x/%#x/%#x,%u)/1:(%#x/%#x/%#x,%u)/2:(%#x/%#x/%#x,%u)), cust(0:(%#x/%#x/%#x,%u)/1:(%#x/%#x/%#x,%u)/2:(%#x/%#x/%#x,%u)), src_code:%#llx(RAW0:%u/RAW1:%u/RAW2:%u)), cust_code:%#llx, sensor_hw_pre_latch_exp_code:%#x)\n",
		caller,
		seninf_idx,
		p_src_cfg->seninf_idx,
		p_src_cfg->tsrec_no,
		p_src_cfg->exp_vc_dt[0].vsync_vc,
		p_src_cfg->exp_vc_dt[0].hsync_vc,
		p_src_cfg->exp_vc_dt[0].hsync_dt,
		p_src_cfg->exp_vc_dt[0].trig_src,
		p_src_cfg->exp_vc_dt[1].vsync_vc,
		p_src_cfg->exp_vc_dt[1].hsync_vc,
		p_src_cfg->exp_vc_dt[1].hsync_dt,
		p_src_cfg->exp_vc_dt[1].trig_src,
		p_src_cfg->exp_vc_dt[2].vsync_vc,
		p_src_cfg->exp_vc_dt[2].hsync_vc,
		p_src_cfg->exp_vc_dt[2].hsync_dt,
		p_src_cfg->exp_vc_dt[2].trig_src,
		p_src_cfg->cust_vc_dt[0].vsync_vc,
		p_src_cfg->cust_vc_dt[0].hsync_vc,
		p_src_cfg->cust_vc_dt[0].hsync_dt,
		p_src_cfg->cust_vc_dt[0].trig_src,
		p_src_cfg->cust_vc_dt[1].vsync_vc,
		p_src_cfg->cust_vc_dt[1].hsync_vc,
		p_src_cfg->cust_vc_dt[1].hsync_dt,
		p_src_cfg->cust_vc_dt[1].trig_src,
		p_src_cfg->cust_vc_dt[2].vsync_vc,
		p_src_cfg->cust_vc_dt[2].hsync_vc,
		p_src_cfg->cust_vc_dt[2].hsync_dt,
		p_src_cfg->cust_vc_dt[2].trig_src,
		p_src_cfg->vc_dt_src_code,
		PAD_SRC_RAW0,
		PAD_SRC_RAW1,
		PAD_SRC_RAW2,
		p_src_cfg->cust_vc_dt_src_code,
		p_src_cfg->sensor_hw_pre_latch_exp_code);
}


void mtk_cam_seninf_tsrec_dbg_dump_tsrec_n_regs_info(const char *caller)
{
	struct tsrec_n_regs_st *p_no_regs_val = NULL;
	const unsigned int tsrec_hw_cnt = tsrec_status.tsrec_hw_cnt;
	unsigned int i = 0;
	int ret = 0;
	char *log_buf = NULL;

	if (unlikely(!chk_exist_tsrec_hw(__func__, 1)))
		return;

	log_buf = TSREC_KCALLOC(TSREC_LOG_BUF_STR_LEN, sizeof(char));
	if (unlikely(log_buf == NULL)) {
		TSREC_LOG_INF("ERROR: log_buf allocate memory failed\n");
		return;
	}

	/* show log info format msg */
	ret = snprintf(log_buf + strlen(log_buf),
		TSREC_LOG_BUF_STR_LEN - strlen(log_buf),
		"tsrec_no(en):(seninf_idx,exp_vc_dt[%u],trig_src)",
		TSREC_EXP_MAX_CNT);

	if (unlikely(ret < 0))
		TSREC_LOG_INF("ERROR: LOG snprintf error, ret:%d\n", ret);


	/* print each tsrec regs values */
	for (i = 0; i < tsrec_hw_cnt; ++i) {
		ret = g_tsrec_n_regs_st(i, &p_no_regs_val, __func__);
		if (unlikely(ret != 0))
			return;
		if (p_no_regs_val == NULL) {
			ret = snprintf(log_buf + strlen(log_buf),
				TSREC_LOG_BUF_STR_LEN - strlen(log_buf),
				", %u(nullptr)",
				i);

			if (unlikely(ret < 0)) {
				TSREC_LOG_INF(
					"ERROR: LOG snprintf error, ret:%d\n", ret);
			}
		} else {
			ret = snprintf(log_buf + strlen(log_buf),
				TSREC_LOG_BUF_STR_LEN - strlen(log_buf),
				", %u(%u):(%u,(%#x/%#x/%#x),%#x,%u)",
				i,
				p_no_regs_val->en,
				p_no_regs_val->seninf_idx,
				p_no_regs_val->exp_vc_dt[0],
				p_no_regs_val->exp_vc_dt[1],
				p_no_regs_val->exp_vc_dt[2],
				p_no_regs_val->trig_src,
				p_no_regs_val->sensor_hw_pre_latch_exp_num);

			if (unlikely(ret < 0)) {
				TSREC_LOG_INF(
					"ERROR: LOG snprintf error, ret:%d\n", ret);
			}
		}
	}


	ret = snprintf(log_buf + strlen(log_buf),
		TSREC_LOG_BUF_STR_LEN - strlen(log_buf),
		" [top_cfg:%#x, timer_cfg:%#x, intr(%#x/%#x, %#x/%#x)]",
		TSREC_ATOMIC_READ(&tsrec_status.top_cfg),
		TSREC_ATOMIC_READ(&tsrec_status.timer_cfg),
		TSREC_ATOMIC_READ(&tsrec_status.intr_en),
		TSREC_ATOMIC_READ(&tsrec_status.intr_en_2),
		TSREC_ATOMIC_READ(&tsrec_status.intr_status),
		TSREC_ATOMIC_READ(&tsrec_status.intr_status_2));

	if (unlikely(ret < 0))
		TSREC_LOG_INF("ERROR: LOG snprintf error, ret:%d\n", ret);


	TSREC_LOG_INF("[%s] %s\n", caller, log_buf);

	TSREC_KFREE(log_buf);
}


void mtk_cam_seninf_tsrec_dbg_dump_ts_records(const unsigned int tsrec_no)
{
#ifndef REDUCE_TSREC_LOG_DUMP_RAW_TICK
	unsigned int i = 0;
#endif
	struct tsrec_n_regs_st *p_tsrec_n_regs = NULL;
	struct tsrec_n_timestamp_st *p_ts_st = NULL;
	unsigned long long tick_ordered[TSREC_EXP_MAX_CNT][TSREC_TS_REC_MAX_CNT] = {0};
	unsigned long long curr_tick_caller = 0;
	int ret = 0;
	char *log_buf = NULL;

	if (unlikely(!chk_exist_tsrec_hw(__func__, 1)))
		return;

	/* error handling (unexpected case) */
	if (unlikely(!chk_tsrec_no_valid(tsrec_no, __func__))) {
		TSREC_LOG_INF(
			"ERROR: get non-valid tsrec_no:%u (tsrec_hw_cnt:%u), return\n",
			tsrec_no,
			tsrec_status.tsrec_hw_cnt);
		return;
	}

	/* prepare basic data pointer/variable */
	ret = g_tsrec_n_regs_st(tsrec_no, &p_tsrec_n_regs, __func__);
	if (unlikely(ret != 0))
		return;
	if (unlikely(p_tsrec_n_regs == NULL)) {
		TSREC_LOG_INF(
			"ERROR: tsrec_n_regs_st[%u] is nullptr (%p), return   [tsrec_no:%u]\n",
			tsrec_no, p_tsrec_n_regs,
			tsrec_no);
		return;
	}
	p_ts_st = &p_tsrec_n_regs->ts_recs;

	/* !!! prepare log buffer/memory, MUST carefully use on below flow !!! */
	/* !!! before you call return, MUST free the log buffer/memory !!! */
	log_buf = TSREC_KCALLOC(TSREC_LOG_BUF_STR_LEN, sizeof(char));
	if (unlikely(log_buf == NULL)) {
		TSREC_LOG_INF("ERROR: log_buf allocate memory failed\n");
		return;
	}

	curr_tick_caller = mtk_cam_seninf_tsrec_latch_time();

	/* reordering ticks data */
	tsrec_n_timestamp_ticks_re_order(tsrec_no, tick_ordered);

	ret = snprintf(log_buf + strlen(log_buf),
		TSREC_LOG_BUF_STR_LEN - strlen(log_buf),
		"tsrec_no:%u, seninf_idx:%u, sys_ts:%llu(ns), tsrec_ts:%llu(%llu), intr(%#x/%#x|%#x/%#x), irq_info(%llu(ns)(~ +%llu ns), %llu(us), intr(|%#x/%#x)), ",
		tsrec_no,
		p_ts_st->seninf_idx,
		p_ts_st->curr_sys_time_ns,
		TSREC_TICK_TO_US(p_ts_st->curr_tick),
		p_ts_st->curr_tick,
		TSREC_ATOMIC_READ(&tsrec_status.intr_en),
		TSREC_ATOMIC_READ(&tsrec_status.intr_en_2),
		TSREC_ATOMIC_READ(&tsrec_status.intr_status),
		TSREC_ATOMIC_READ(&tsrec_status.intr_status_2),
		p_tsrec_n_regs->irq_info.sys_ts_ns,
		p_tsrec_n_regs->irq_info.duration_ns,
		p_tsrec_n_regs->irq_info.tsrec_ts_us,
		p_tsrec_n_regs->irq_info.intr_status,
		p_tsrec_n_regs->irq_info.intr_status_2);

	if (unlikely(ret < 0))
		TSREC_LOG_INF("ERROR: LOG snprintf error, ret:%d\n", ret);


#ifndef REDUCE_TSREC_LOG_DUMP_RAW_TICK
	for (i = 0; i < TSREC_EXP_MAX_CNT; ++i) {
		ret = snprintf(log_buf + strlen(log_buf),
			TSREC_LOG_BUF_STR_LEN - strlen(log_buf),
			"%u:(%u,%llu/%llu/%llu/%llu)/",
			i,
			p_ts_st->exp_recs[i].curr_idx,
			p_ts_st->exp_recs[i].ticks[0],
			p_ts_st->exp_recs[i].ticks[1],
			p_ts_st->exp_recs[i].ticks[2],
			p_ts_st->exp_recs[i].ticks[3]);

		if (unlikely(ret < 0)) {
			TSREC_LOG_INF(
				"ERROR: LOG snprintf error, ret:%d\n",
				ret);
		}
	}
#endif // !REDUCE_TSREC_LOG_DUMP_RAW_TICK


	ret = snprintf(log_buf + strlen(log_buf),
		TSREC_LOG_BUF_STR_LEN - strlen(log_buf),
		" => ts(0:(%llu/%llu/%llu/%llu), 1:(%llu/%llu/%llu/%llu), 2:(%llu/%llu/%llu/%llu))",
		TSREC_TICK_TO_US(tick_ordered[0][0]),
		TSREC_TICK_TO_US(tick_ordered[0][1]),
		TSREC_TICK_TO_US(tick_ordered[0][2]),
		TSREC_TICK_TO_US(tick_ordered[0][3]),
		TSREC_TICK_TO_US(tick_ordered[1][0]),
		TSREC_TICK_TO_US(tick_ordered[1][1]),
		TSREC_TICK_TO_US(tick_ordered[1][2]),
		TSREC_TICK_TO_US(tick_ordered[1][3]),
		TSREC_TICK_TO_US(tick_ordered[2][0]),
		TSREC_TICK_TO_US(tick_ordered[2][1]),
		TSREC_TICK_TO_US(tick_ordered[2][2]),
		TSREC_TICK_TO_US(tick_ordered[2][3]));

	if (unlikely(ret < 0))
		TSREC_LOG_INF("ERROR: LOG snprintf error, ret:%d\n", ret);


	ret = snprintf(log_buf + strlen(log_buf),
		TSREC_LOG_BUF_STR_LEN - strlen(log_buf),
#if (TSREC_WITH_64_BITS_TIMER_RG)
		", act_fl(%llu/%llu/%llu), %llu(+%llu)(+%llu)",
#else
		", act_fl(%u/%u/%u), %u(+%u)(+%u)",
#endif
		TSREC_TICK_TO_US(
			(tsrec_tick_t)tick_ordered[TSREC_1ST_EXP_NUM][0] -
			(tsrec_tick_t)tick_ordered[TSREC_1ST_EXP_NUM][1]),
		TSREC_TICK_TO_US(
			(tsrec_tick_t)tick_ordered[TSREC_1ST_EXP_NUM][1] -
			(tsrec_tick_t)tick_ordered[TSREC_1ST_EXP_NUM][2]),
		TSREC_TICK_TO_US(
			(tsrec_tick_t)tick_ordered[TSREC_1ST_EXP_NUM][2] -
			(tsrec_tick_t)tick_ordered[TSREC_1ST_EXP_NUM][3]),
		TSREC_TICK_TO_US(
			(tsrec_tick_t)tick_ordered[TSREC_1ST_EXP_NUM][0]),
		TSREC_TICK_TO_US(
			(tsrec_tick_t)p_ts_st->curr_tick -
			(tsrec_tick_t)tick_ordered[TSREC_1ST_EXP_NUM][0]),
		TSREC_TICK_TO_US(
			(tsrec_tick_t)curr_tick_caller -
			(tsrec_tick_t)p_ts_st->curr_tick));

	if (unlikely(ret < 0))
		TSREC_LOG_INF("ERROR: LOG snprintf error, ret:%d\n", ret);


	TSREC_LOG_DBG("%s\n", log_buf);

	TSREC_KFREE(log_buf);
}


/******************************************************************************
 * TSREC init/sysfs/irq-init functions
 *****************************************************************************/
#ifndef FS_UT
/*---------------------------------------------------------------------------*/
// tsrec console framework functions
/*---------------------------------------------------------------------------*/
static ssize_t tsrec_console_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int len = 0;

	return len;
}


static ssize_t tsrec_console_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int cmd = 0;
	int ret = 0;

	/* convert string to unsigned int */
	ret = kstrtouint(buf, 0, &cmd);
	if (ret != 0) {
		// SHOW(str_buf, len,
		//	"\n\t[fsync_console]: kstrtoint failed, input:%s, cmd:%u, ret:%d\n",
		//	buf,
		//	cmd,
		//	ret);
	}

	tsrec_con_mgr_process_cmd(cmd);

	return count;
}


static DEVICE_ATTR_RW(tsrec_console);
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
// tsrec sysfs framwwork functions
/*---------------------------------------------------------------------------*/
static void mtk_cam_seninf_tsrec_create_sysfs_file(struct device *dev)
{
	int ret = 0;

	/* case handling (unexpected case) */
	if (unlikely(dev == NULL)) {
		TSREC_LOG_INF(
			"ERROR: failed to create sysfs file, dev is NULL, return\n");
		return;
	}

	/* !!! create each sysfs file you want !!! */
	ret = device_create_file(dev, &dev_attr_tsrec_console);
	if (ret) {
		TSREC_LOG_INF(
			"ERROR: call device_create_file() failed, ret:%d\n",
			ret);
	}
}


static void mtk_cam_seninf_tsrec_remove_sysfs_file(struct device *dev)
{
	/* case handling (unexpected case) */
	if (unlikely(dev == NULL)) {
		TSREC_LOG_INF(
			"ERROR: *dev is NULL, abort process device_remove_file, return\n");
		return;
	}

	/* !!! remove each sysfs file you created !!! */
	device_remove_file(dev, &dev_attr_tsrec_console);
}
/*---------------------------------------------------------------------------*/


static void mtk_cam_seninf_tsrec_get_property(struct device *dev)
{
	unsigned int val = 0;
	int ret = 0;

	/* read platform properties from device tree */
	/* return: 0 on succes */
	/*         , -EINVAL    if the property does not exist */
	/*         , -ENODATA   if the property does not have a value */
	/*         , -EOVERFLOW if the property data isn't large enough. */
	/* get tsrec hw cnt from dts */
	ret = of_property_read_u32(dev->of_node,
		"tsrec-num",
		&val);
	if (likely(ret == 0)) {
		tsrec_status.tsrec_hw_cnt = val;

		if (unlikely(tsrec_status.tsrec_hw_cnt == 0)) {
			TSREC_LOG_INF(
				"WARNING: TSREC HW cnt is %u, this will cause TSREC flow NOT running\n",
				tsrec_status.tsrec_hw_cnt);
		}
	} else {
		tsrec_status.tsrec_hw_cnt = 0;

		TSREC_LOG_INF(
			"WARNING: get property of TSREC HW cnt from dts failed, ret:%d (-EINVAL:%d/-ENODATA:%d/-EOVERFLOW:%d) => tsrec_hw_cnt:%u\n",
			ret, -EINVAL, -ENODATA, -EOVERFLOW,
			tsrec_status.tsrec_hw_cnt);
	}

	/* get seninf hw cnt from dts */
	ret = of_property_read_u32(dev->of_node,
		"seninf-num",
		&val);
	if (likely(ret == 0)) {
		tsrec_status.seninf_hw_cnt = val;

		if (unlikely(tsrec_status.seninf_hw_cnt == 0)) {
			TSREC_LOG_INF(
				"WARNING: SENINF HW cnt is %u, this will cause TSREC flow NOT running\n",
				tsrec_status.seninf_hw_cnt);
		}
	} else {
		tsrec_status.seninf_hw_cnt = 0;

		TSREC_LOG_INF(
			"ERROR: get property of SENINF HW cnt from dts failed, ret:%d (-EINVAL:%d/-ENODATA:%d/-EOVERFLOW:%d) => seninf_hw_cnt:%u\n",
			ret, -EINVAL, -ENODATA, -EOVERFLOW,
			tsrec_status.seninf_hw_cnt);
	}


	TSREC_LOG_INF(
		"NOTICE: set HW cnt info from dts, tsrec_hw_cnt:%u/seninf_hw_cnt:%u\n",
		tsrec_status.tsrec_hw_cnt,
		tsrec_status.seninf_hw_cnt);
}


void mtk_cam_seninf_tsrec_irq_init(struct seninf_core *core, const int irq)
{
	int ret = 0;

	if (unlikely(!chk_exist_tsrec_hw(__func__, 1)))
		return;

	/* error handling (unexpected case) */
	if (unlikely(!core || !irq)) {
		TSREC_LOG_INF(
			"ERROR: get non-valid input, (addr of seninf_core:%p || irq:%d), return\n",
			core, irq);
		return;
	}

	tsrec_status.tsrec_irq = irq;

	ret = devm_request_threaded_irq(seninf_dev, tsrec_status.tsrec_irq,
					mtk_irq_seninf_tsrec,
					mtk_thread_irq_seninf_tsrec,
					0, "seninf-tsrec", core);
	if (ret) {
		TSREC_LOG_INF(
			"ERROR: failed to request TSREC-IRQ:%d\n",
			tsrec_status.tsrec_irq);
		return;
	}

	tsrec_irq_enable(0);

	TSREC_LOG_INF(
		"NOTICE: registered TSREC-IRQ:%d success, disable IRQ\n",
		tsrec_status.tsrec_irq);
}
#endif // !FS_UT


#ifndef FS_UT
void mtk_cam_seninf_tsrec_init(struct device *dev, void __iomem *p_seninf_base)
#else
void mtk_cam_seninf_tsrec_init(void)
#endif
{
#ifndef FS_UT
	seninf_dev = dev;

	mtk_cam_seninf_tsrec_regs_iomem_init(p_seninf_base);
	mtk_cam_seninf_tsrec_create_sysfs_file(seninf_dev);
	mtk_cam_seninf_tsrec_get_property(seninf_dev);

	if (unlikely(!chk_exist_tsrec_hw(__func__, 1)))
		return;
#endif

	/* init tsrec data */
	tsrec_data_init();
}


void mtk_cam_seninf_tsrec_uninit(void)
{
#ifndef FS_UT
	tsrec_irq_enable(0);

	mtk_cam_seninf_tsrec_remove_sysfs_file(seninf_dev);

	seninf_dev = NULL;
#endif

	/* uninit tsrec data */
	tsrec_data_uninit();
}


/******************************************************************************
 * TSREC ONLY for unit-test functions
 *****************************************************************************/
#ifdef FS_UT
void mtk_cam_seninf_tsrec_ut_dbg_sysfs_ctrl(const unsigned int cmd_val)
{
	tsrec_con_mgr_process_cmd(cmd_val);
}


void mtk_cam_seninf_tsrec_ut_dbg_irq_seninf_tsrec(void)
{
	irqreturn_t irq_ret = IRQ_NONE;
	void *data = NULL;

	irq_ret = mtk_irq_seninf_tsrec(tsrec_status.tsrec_irq, data);
	switch (irq_ret) {
	case IRQ_NONE:
		TSREC_LOG_INF(
			"get irq_ret:IRQ_NONE\n");
		break;

	case IRQ_HANDLED:
		TSREC_LOG_INF(
			"get irq_ret:IRQ_HANDLED\n");
		break;

	case IRQ_WAKE_THREAD:
		TSREC_LOG_INF(
			"get irq_ret:IRQ_WAKE_THREAD\n");
		mtk_thread_irq_seninf_tsrec(tsrec_status.tsrec_irq, data);
		break;

	default:
		TSREC_LOG_INF(
			"ERROR: get irq_ret NO defined, irq_ret:%d\n",
			irq_ret);
		break;
	}
}


void mtk_cam_seninf_tsrec_init_for_ut_test(const unsigned int flag,
	const unsigned int tsrec_hw_cnt, const unsigned int seninf_hw_cnt)
{
	if (flag > 0) {
		tsrec_status.tsrec_hw_cnt = tsrec_hw_cnt;
		tsrec_status.seninf_hw_cnt = seninf_hw_cnt;
		tsrec_status.tsrec_irq = 255;

		tsrec_data_init();
	} else {
		tsrec_data_uninit();

		tsrec_status.seninf_hw_cnt = 0;
		tsrec_status.tsrec_hw_cnt = 0;
		tsrec_status.tsrec_irq = 0;
	}
}
#endif // FS_UT
