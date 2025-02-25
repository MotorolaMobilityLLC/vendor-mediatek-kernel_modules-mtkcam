// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 MediaTek Inc.
 *
 * Author: Yuan-Jung Kuo <yuan-jung.kuo@mediatek.com>
 *          Nick.wen <nick.wen@mediatek.com>
 *
 */
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/iopoll.h>
#include <linux/of_device.h>
#include <linux/soc/mediatek/mtk-cmdq-ext.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include "vcp_status.h"
#include "mtk_imgsys-engine-isp8s.h"
#include "mtk_imgsys-cmdq.h"
#include "mtk_imgsys-cmdq-plat.h"
#include "mtk_imgsys-cmdq-qof-data.h"
#include "mtk-smi-dbg.h"
#include "mtk_imgsys-trace.h"

/* HW event: restore event for qof */
#define CMDQ_EVENT_IMG_QOF_RESTORE_EVENT_0_DIP			(55)
#define CMDQ_EVENT_IMG_QOF_RESTORE_EVENT_1_TRAW			(56)
#define CMDQ_EVENT_IMG_QOF_RESTORE_EVENT_2_WPE_EIS		(57)
#define CMDQ_EVENT_IMG_QOF_RESTORE_EVENT_3_WPE_TNR		(58)
#define CMDQ_EVENT_IMG_QOF_RESTORE_EVENT_4_LITE			(59)
/* SW event: sw event for qof */
#define CMDQ_SW_EVENT_QOF_DIP							(768)
#define CMDQ_SW_EVENT_QOF_TRAW							(769)
#define CMDQ_SW_EVENT_QOF_WPE_EIS						(770)
#define CMDQ_SW_EVENT_QOF_WPE_TNR						(771)
#define CMDQ_SW_EVENT_QOF_LITE							(772)
#define CMDQ_SW_EVENT_QOF_DIP_CINE						(773)

/* delay cnt */
#define IMG_HWV_DELAY_CNT								(0xFFFF)
#define IMG_CG_UNGATING_DELAY_CNT						(0xFFFF)
#define IMG_MTCMOS_STABLE_CNT							(0xFFFF)
#define INT_STATUS_POLL_TIMES							(20)
#define INT_STATUS_POLL_US								(50000)
#define POLL_DELAY_US									(1)
#define TIMEOUT_500US									(500)
#define TIMEOUT_1000US									(1000)	// = 1ms
#define TIMEOUT_2000US									(2000)	// = 2ms
#define TIMEOUT_100000US								(100000)
/* others */
#define MOD_BIT_OFST									(3)
#define QOF_ERROR_CODE									(0xdead)
/* func */
#define IS_MOD_SUPPORT_QOF(m)		(((m < QOF_TOTAL_MODULE)) && ((g_qof_ver & BIT(m)) == BIT(m)))
#define IS_CON_PWR_ON(val)			((val & (BIT(30)|BIT(31))) == (BIT(30)|BIT(31)) ? TRUE:FALSE)
#define QOF_GET_REMAP_ADDR(addr)	(g_maped_rg[MAPED_RG_QOF_REG_BASE] + (addr - QOF_REG_BASE))
#define QOF_LOGI(fmt, args...)		pr_info("[QOF_LOGI]%s:%d " fmt "\n", __func__, __LINE__, ##args)
#define QOF_LOGE(fmt, args...)		pr_err("[QOF_ERROR]%s:%d " fmt "\n", __func__, __LINE__, ##args)

#define write_mask(addr, val, mask)\
	do {\
		unsigned int tmp = readl(addr) & ~(mask);\
		tmp = tmp | ((val) & mask);\
		writel(tmp, addr);\
	} while(0)

#define QOF_WRITE_VAL(reg_table, val) \
		REG_FLD_SET(reg_table.field, QOF_GET_REMAP_ADDR(reg_table.addr), val)

/* common params */
static u32 imgsys_voter_cnt_locked;
static u32 g_qof_ver;
static u32 g_ftrace_time;

struct mtk_imgsys_dev *g_imgsys_dev;
static void __iomem *g_maped_rg[MAPED_RG_LIST_NUM] = {0};
static struct qof_events qof_events_isp8s[ISP8S_PWR_NUM];
static struct cmdq_client *imgsys_pwr_clt[QOF_TOTAL_THREAD] = {NULL};

static unsigned int g_qof_smi_cnt;

spinlock_t qof_lock;
static bool is_smi_use_qof_locked[ISP8S_PWR_NUM] = {false};
static bool is_poll_event_mode;

/* dbg params*/
static bool imgsys_ftrace_qof_thread_en;
static int g_qof_debug_level;

module_param(imgsys_ftrace_qof_thread_en, bool, 0644);
MODULE_PARM_DESC(imgsys_ftrace_qof_thread_en, "imgsys ftrace thread enable, 0 (default)");

enum QOF_DEBUG_MODE {
	QOF_DEBUG_MODE_PERFRAME_DUMP = 1,
	QOF_DEBUG_MODE_IMMEDIATE_DUMP = 2,
	QOF_DEBUG_MODE_IMMEDIATE_CG_DUMP = 3,
};

enum QOF_GCE_THREAD_LIST {
	QOF_GCE_THREAD_DIP = IMGSYS_NOR_THD,
	QOF_GCE_THREAD_TRAW,
	QOF_GCE_THREAD_WPE1,
	QOF_GCE_THREAD_WPE2,
	QOF_GCE_THREAD_WPE3,
};

u32 sw_event_lock_list[ISP8S_PWR_NUM] = {
	CMDQ_SW_EVENT_QOF_DIP,
	CMDQ_SW_EVENT_QOF_TRAW,
	CMDQ_SW_EVENT_QOF_WPE_EIS,
	CMDQ_SW_EVENT_QOF_WPE_TNR,
	CMDQ_SW_EVENT_QOF_LITE,
};

u32 hw_event_restore_list[ISP8S_PWR_NUM] = {
	CMDQ_EVENT_IMG_QOF_RESTORE_EVENT_0_DIP,
	CMDQ_EVENT_IMG_QOF_RESTORE_EVENT_1_TRAW,
	CMDQ_EVENT_IMG_QOF_RESTORE_EVENT_2_WPE_EIS,
	CMDQ_EVENT_IMG_QOF_RESTORE_EVENT_3_WPE_TNR,
	CMDQ_EVENT_IMG_QOF_RESTORE_EVENT_4_LITE,
};

struct cmdq_pwr_buf {
	struct cmdq_pkt *restore_pkt;
};

static struct cmdq_pwr_buf pwr_buf_handle[QOF_TOTAL_THREAD] = {
	{
		.restore_pkt = NULL,
	},
	{
		.restore_pkt = NULL,
	},
	{
		.restore_pkt = NULL,
	},
	{
		.restore_pkt = NULL,
	},
	{
		.restore_pkt = NULL,
	},
};

/* mtcmos data */
static void qof_locked_stream_off_sync(void);
static void qof_locked_set_engine_off(const u32 mod);
static void imgsys_cmdq_qof_set_restore_done(struct cmdq_pkt *pkt, u32 pwr_id);
static void qof_module_vote_add(struct cmdq_pkt *pkt, u32 pwr, u32 user);
static void qof_module_vote_sub(struct cmdq_pkt *pkt, u32 pwr, u32 user);
static void qof_stop_all_gce_loop(void);
static void imgsys_qof_set_dbg_thread(bool enable);

static void mtk_qof_print_mtcmos_status(void);

static struct imgsys_mtcmos_data isp8s_module_data[] = {
	[ISP8S_PWR_DIP] = {
		.pwr_id = ISP8S_PWR_DIP,
		.qof_restore_done = imgsys_cmdq_qof_set_restore_done,
	},
	[ISP8S_PWR_TRAW] = {
		.pwr_id = ISP8S_PWR_TRAW,
		.qof_restore_done = imgsys_cmdq_qof_set_restore_done,
	},
	[ISP8S_PWR_WPE_1_EIS] = {
		.pwr_id = ISP8S_PWR_WPE_1_EIS,
		.qof_restore_done = imgsys_cmdq_qof_set_restore_done,
	},
	[ISP8S_PWR_WPE_2_TNR] = {
		.pwr_id = ISP8S_PWR_WPE_2_TNR,
		.qof_restore_done = imgsys_cmdq_qof_set_restore_done,
	},
	[ISP8S_PWR_WPE_3_LITE] = {
		.pwr_id = ISP8S_PWR_WPE_3_LITE,
		.qof_restore_done = imgsys_cmdq_qof_set_restore_done,
	},
};

static bool rtff_init(u32 mod)
{
	//Main
	write_mask(g_maped_rg[MAPED_RG_RTFF_BASE] + 0x118, BIT(1)|BIT(2), 0xffffffff);
	switch(mod) {
	case QOF_SUPPORT_DIP:
		//DIP
		write_mask(g_maped_rg[MAPED_RG_RTFF_BASE] + 0x110, BIT(1)|BIT(2), 0xffffffff);
		//DIP_CINE
		write_mask(g_maped_rg[MAPED_RG_RTFF_BASE] + 0x114, BIT(1)|BIT(2), 0xffffffff);
		break;
	case QOF_SUPPORT_TRAW:
		//TRAW
		write_mask(g_maped_rg[MAPED_RG_RTFF_BASE] + 0x11C, BIT(1)|BIT(2), 0xffffffff);
		break;
	case QOF_SUPPORT_WPE_EIS:
		//WPE0
		write_mask(g_maped_rg[MAPED_RG_RTFF_BASE] + 0x120, BIT(1)|BIT(2), 0xffffffff);
		break;
	case QOF_SUPPORT_WPE_TNR:
		//WPE1
		write_mask(g_maped_rg[MAPED_RG_RTFF_BASE] + 0x124, BIT(1)|BIT(2), 0xffffffff);
		break;
	case QOF_SUPPORT_WPE_LITE:
		//WPE2
		write_mask(g_maped_rg[MAPED_RG_RTFF_BASE] + 0x128, BIT(1)|BIT(2), 0xffffffff);
		break;
	default:
		QOF_LOGE("module not match\n");
		return false;
	}
	return true;
}

static bool rtff_uninit(u32 mod)
{
	//Main
	write_mask(g_maped_rg[MAPED_RG_RTFF_BASE] + 0x118, BIT(1), 0xffffffff);
	switch(mod) {
	case QOF_SUPPORT_DIP:
		//DIP
		write_mask(g_maped_rg[MAPED_RG_RTFF_BASE] + 0x110, BIT(1), 0xffffffff);
		//DIP_CINE
		write_mask(g_maped_rg[MAPED_RG_RTFF_BASE] + 0x114, BIT(1), 0xffffffff);
		break;
	case QOF_SUPPORT_TRAW:
		//TRAW
		write_mask(g_maped_rg[MAPED_RG_RTFF_BASE] + 0x11C, BIT(1), 0xffffffff);
		break;
	case QOF_SUPPORT_WPE_EIS:
		//WPE0
		write_mask(g_maped_rg[MAPED_RG_RTFF_BASE] + 0x120, BIT(1), 0xffffffff);
		break;
	case QOF_SUPPORT_WPE_TNR:
		//WPE1
		write_mask(g_maped_rg[MAPED_RG_RTFF_BASE] + 0x124, BIT(1), 0xffffffff);
		break;
	case QOF_SUPPORT_WPE_LITE:
		//WPE2
		write_mask(g_maped_rg[MAPED_RG_RTFF_BASE] + 0x128, BIT(1), 0xffffffff);
		break;
	default:
		QOF_LOGE("module not match\n");
		return false;
	}
	return true;
}

static void hwccf_unlink(int mod)
{
	// unlink == set this reg !
	// it's wired but this is true
	u32 bit = 0;

	switch(mod) {
	case QOF_SUPPORT_DIP:
		bit = BIT(9) | BIT(10);
		break;
	case QOF_SUPPORT_TRAW:
		bit = BIT(11);
		break;
	case QOF_SUPPORT_WPE_EIS:
		bit = BIT(6);
		break;
	case QOF_SUPPORT_WPE_TNR:
		bit = BIT(7);
		break;
	case QOF_SUPPORT_WPE_LITE:
		bit = BIT(8);
		break;
	default:
		QOF_LOGE("module not match\n");
	}
	write_mask(g_maped_rg[MAPED_RG_HWCCF_REG_SET],
				bit,
				bit);
}

static void hwccf_link(int mod)
{
	// link == clr this reg !
	// it's wired but this is true
	u32 bit = 0;

	switch(mod) {
	case QOF_SUPPORT_DIP:
		bit = BIT(9) | BIT(10);
		break;
	case QOF_SUPPORT_TRAW:
		bit = BIT(11);
		break;
	case QOF_SUPPORT_WPE_EIS:
		bit = BIT(6);
		break;
	case QOF_SUPPORT_WPE_TNR:
		bit = BIT(7);
		break;
	case QOF_SUPPORT_WPE_LITE:
		bit = BIT(8);
		break;
	default:
		QOF_LOGE("module not match\n");
	}
	write_mask(g_maped_rg[MAPED_RG_HWCCF_REG_CLR],
				bit,
				bit);
}

static unsigned int get_thread_id_by_pwr_id(u32 pwr)
{
	u32 th_id = IMG_GCE_THREAD_PWR_START;

	switch(pwr) {
	case ISP8S_PWR_DIP:
		th_id = IMG_GCE_THREAD_DIP;
		break;
	case ISP8S_PWR_TRAW:
		th_id = IMG_GCE_THREAD_TRAW;
		break;
	case ISP8S_PWR_WPE_1_EIS:
		th_id = IMG_GCE_THREAD_WPE_1_EIS;
		break;
	case ISP8S_PWR_WPE_2_TNR:
		th_id = IMG_GCE_THREAD_WPE_2_TNR;
		break;
	case ISP8S_PWR_WPE_3_LITE:
		th_id = IMG_GCE_THREAD_WPE_3_LITE;
		break;
	default:
		QOF_LOGI("qof pwr[%u] is not mapping\n", pwr);
		break;
	}
	return th_id;
}

bool is_qof_engine_enabled(enum ISP8S_IMG_PWR mod)
{
	int bit_mod = mod + MOD_BIT_OFST;
	int addr = qof_reg_table[mod][QOF_REG_IMG_QOF_ENG_EN].addr;

	return (readl(QOF_GET_REMAP_ADDR(addr)) & BIT(bit_mod)) == BIT(bit_mod);
}

bool check_cg_status(u32 cg_reg_idx, u32 val)
{
	if((readl(g_maped_rg[cg_reg_idx]) & val) != 0) {
		QOF_LOGE("g_maped_rg[%d] CG not ungate(0x%x)!!!\n", cg_reg_idx, readl(g_maped_rg[cg_reg_idx]));
		mtk_imgsys_cmdq_qof_dump(0, false);
		return false;
	}
	return true;
}

bool qof_check_module_cg_status(u32 pwr)
{
	unsigned int val;
	void __iomem *addr;

	if ((readl(g_maped_rg[MAPED_RG_ISP_MAIN_PWR_CON]) & (BIT(30)|BIT(31)))
		!= (BIT(30)|BIT(31))) {
		QOF_LOGI("mod[%u] isp_main is off. sta=0x%x\n",
			pwr, readl(g_maped_rg[MAPED_RG_ISP_MAIN_PWR_CON]));
		return false;
	}
	addr = QOF_GET_REMAP_ADDR(qof_reg_table[pwr][QOF_REG_IMG_QOF_STATE_DBG].addr);
	if ((readl(addr) & BIT(1)) != BIT(1)) {
		QOF_LOGI("qof[%u] mtcmos is off. sta=0x%x\n", pwr, readl(addr));
		return false;
	}

	switch(pwr) {
	case ISP8S_PWR_DIP:
		val = BIT(0)|BIT(1);
		if (!check_cg_status(MAPED_RG_IMG_CG_DIP_NR1_DIP1, val))
			return false;
		val = BIT(0)|BIT(1)|BIT(2);
		if (!check_cg_status(MAPED_RG_IMG_CG_DIP_NR2_DIP1, val))
			return false;
		val = BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8);
		if (!check_cg_status(MAPED_RG_IMG_CG_DIP_TOP_DIP1, val))
			return false;
		break;
	case ISP8S_PWR_TRAW:
		val = BIT(0);
		if (!check_cg_status(MAPED_RG_IMG_CG_TRAW_CAP_DIP1, val))
			return false;
		val = BIT(0)|BIT(1)|BIT(2)|BIT(3);
		if (!check_cg_status(MAPED_RG_IMG_CG_TRAW_DIP1, val))
			return false;
		break;
	case ISP8S_PWR_WPE_1_EIS:
		val = BIT(0)|BIT(1)|BIT(2);
		if (!check_cg_status(MAPED_RG_IMG_CG_WPE1_DIP1, val))
			return false;
		break;
	case ISP8S_PWR_WPE_2_TNR:
		val = BIT(0)|BIT(1)|BIT(2);
		if (!check_cg_status(MAPED_RG_IMG_CG_WPE2_DIP1, val))
			return false;
		break;
	case ISP8S_PWR_WPE_3_LITE:
		val = BIT(0)|BIT(1)|BIT(2);
		if (!check_cg_status(MAPED_RG_IMG_CG_WPE3_DIP1, val))
			return false;
		break;
	default:
		break;
	}
	return true;
}

static int qof_smi_isp_module_get_if_in_use(void *data, enum ISP8S_IMG_PWR module)
{
	int get_result = -1;
	unsigned long flag;
	bool smi_use_qof = false;
	void __iomem *io_addr;
	u32 tmp;

	if (data == NULL) {
		QOF_LOGE("qof get if in use data is null\n");
		return -1;
	}
	if (g_qof_ver == 0 )
		return 1;
	spin_lock_irqsave(&qof_lock, flag);

	if (imgsys_voter_cnt_locked == 0) {
		// pm get power fail
		smi_use_qof = false;
		get_result = -1;
		goto RETURN_FLOW;
	} else {
		if (is_qof_engine_enabled(module)) {
			if (g_qof_smi_cnt == 0) {
				for (unsigned int i = ISP8S_PWR_START; i < ISP8S_PWR_NUM; i ++)
					QOF_WRITE_VAL(qof_reg_table[i][QOF_REG_IMG_SCP_SET], 0x1);
			}
			g_qof_smi_cnt++;
			io_addr = QOF_GET_REMAP_ADDR(qof_reg_table[module][QOF_REG_IMG_QOF_STATE_DBG].addr);
			if (readl_poll_timeout_atomic
				(io_addr, tmp, (tmp & BIT(1)) == BIT(1), POLL_DELAY_US, TIMEOUT_1000US) < 0 ||
				qof_check_module_cg_status(module) == false) {
				QOF_LOGE("Error: mod[%d] waiting qof pwr on timeout\n",module);
				smi_use_qof = false;
				get_result = -1;
				goto RETURN_FLOW;
			} else {
				// qof get power success
				smi_use_qof = true;
				get_result = 1;
				imgsys_voter_cnt_locked++;
				goto RETURN_FLOW;
			}
		} else {
			smi_use_qof = false;
			get_result = 1;
		    imgsys_voter_cnt_locked++;
			goto RETURN_FLOW;
		}
	}

RETURN_FLOW:
    //prevent to print log during interrupt disable
	*(bool *)data = smi_use_qof;
	spin_unlock_irqrestore(&qof_lock, flag);
	QOF_LOGI("[%d]get_result=%d, smi_use_qof=%u, mem_cnt=%u, MAIN[0x%x], VCORE[0x%x]\n",
		module,
		get_result,
		*(bool *)data,
		g_qof_smi_cnt,
		(readl(g_maped_rg[MAPED_RG_ISP_MAIN_PWR_CON])),
		(readl(g_maped_rg[MAPED_RG_ISP_VCORE_PWR_CON])));

	return get_result;
}

#ifndef CONFIG_FPGA_EARLY_PORTING
static int qof_smi_isp_module_get(void *data, enum ISP8S_IMG_PWR module)
{
	unsigned long flag;
	if (g_qof_ver == 0)
		return 1;
	/* get is for smi force_all_on dbg mode use */
	if (imgsys_voter_cnt_locked == 0) {
		QOF_LOGI("Staus occur before stream on, turn off qof\n");
		*(bool *)data = false;
		g_qof_ver = 0;
	} else {
		spin_lock_irqsave(&qof_lock, flag);
		imgsys_voter_cnt_locked++;
		spin_unlock_irqrestore(&qof_lock, flag);

		if (qof_smi_isp_module_get_if_in_use(data, module) == -1)
			QOF_LOGE("qof get fail\n");

		spin_lock_irqsave(&qof_lock, flag);
		imgsys_voter_cnt_locked--;
		spin_unlock_irqrestore(&qof_lock, flag);
	}

	return 0;
}
#endif

static int qof_smi_isp_module_put(void *data, int module)
{
	unsigned long flag;
	bool smi_use_qof;

	if (g_qof_ver == 0)
		return 1;
	spin_lock_irqsave(&qof_lock, flag);
	if (data == NULL) {
		QOF_LOGE("data is null,set to default\n");
		smi_use_qof = false;
	} else
		smi_use_qof = *((bool *)data);

	if (imgsys_voter_cnt_locked == 0)
		QOF_LOGE("imgsys_voter_cnt_locked is 0\n");
	else {
		imgsys_voter_cnt_locked--;
		if (smi_use_qof) {
			g_qof_smi_cnt--;
			if (g_qof_smi_cnt == 0) {
				for(int i =0; i < ISP8S_PWR_NUM; i++)
					QOF_WRITE_VAL(qof_reg_table[i][QOF_REG_IMG_SCP_CLR], 0x1);
			}
		}
	}
	spin_unlock_irqrestore(&qof_lock, flag);
	QOF_LOGI("[%d]smi_use_qof=%u, mem_cnt=%u, MAIN[0x%x], VCORE[0x%x]\n",
		module,
		smi_use_qof, g_qof_smi_cnt,
		(readl(g_maped_rg[MAPED_RG_ISP_MAIN_PWR_CON])),
		(readl(g_maped_rg[MAPED_RG_ISP_VCORE_PWR_CON])));

	return 0;
}

/* SMI DIP CB */
#ifndef CONFIG_FPGA_EARLY_PORTING
static int smi_isp_dip_get(void *data)
{
	return qof_smi_isp_module_get(data, ISP8S_PWR_DIP);
}
#endif

int smi_isp_dip_get_if_in_use(void *data)
{
	return qof_smi_isp_module_get_if_in_use(data, ISP8S_PWR_DIP);
}
EXPORT_SYMBOL(smi_isp_dip_get_if_in_use);

int smi_isp_dip_put(void *data)
{
	return qof_smi_isp_module_put(data, ISP8S_PWR_DIP);
}
EXPORT_SYMBOL(smi_isp_dip_put);

/* SMI TRAW CB */
#ifndef CONFIG_FPGA_EARLY_PORTING
static int smi_isp_traw_get(void *data)
{
	return qof_smi_isp_module_get(data, ISP8S_PWR_TRAW);
}
#endif

int smi_isp_traw_get_if_in_use(void *data)
{
	return qof_smi_isp_module_get_if_in_use(data, ISP8S_PWR_TRAW);
}
EXPORT_SYMBOL(smi_isp_traw_get_if_in_use);

int smi_isp_traw_put(void *data)
{
	return qof_smi_isp_module_put(data, ISP8S_PWR_TRAW);
}
EXPORT_SYMBOL(smi_isp_traw_put);

/* SMI WPE1 CB */
#ifndef CONFIG_FPGA_EARLY_PORTING
static int smi_isp_wpe1_eis_get(void *data)
{
	return qof_smi_isp_module_get(data, ISP8S_PWR_WPE_1_EIS);
}
#endif

int smi_isp_wpe1_eis_get_if_in_use(void *data)
{
	return qof_smi_isp_module_get_if_in_use(data, ISP8S_PWR_WPE_1_EIS);
}
EXPORT_SYMBOL(smi_isp_wpe1_eis_get_if_in_use);

int smi_isp_wpe1_eis_put(void *data)
{
	return qof_smi_isp_module_put(data, ISP8S_PWR_WPE_1_EIS);
}
EXPORT_SYMBOL(smi_isp_wpe1_eis_put);

/* SMI WPE2 CB */
#ifndef CONFIG_FPGA_EARLY_PORTING
static int smi_isp_wpe2_tnr_get(void *data)
{
	return qof_smi_isp_module_get(data, ISP8S_PWR_WPE_2_TNR);
}
#endif

int smi_isp_wpe2_tnr_get_if_in_use(void *data)
{
	return qof_smi_isp_module_get_if_in_use(data, ISP8S_PWR_WPE_2_TNR);
}
EXPORT_SYMBOL(smi_isp_wpe2_tnr_get_if_in_use);

int smi_isp_wpe2_tnr_put(void *data)
{
	return qof_smi_isp_module_put(data, ISP8S_PWR_WPE_2_TNR);
}
EXPORT_SYMBOL(smi_isp_wpe2_tnr_put);

/* SMI WPE3 CB */
#ifndef CONFIG_FPGA_EARLY_PORTING
static int smi_isp_wpe3_lite_get(void *data)
{
	return qof_smi_isp_module_get(data, ISP8S_PWR_WPE_3_LITE);
}
#endif

int smi_isp_wpe3_lite_get_if_in_use(void *data)
{
	return qof_smi_isp_module_get_if_in_use(data, ISP8S_PWR_WPE_3_LITE);
}
EXPORT_SYMBOL(smi_isp_wpe3_lite_get_if_in_use);

int smi_isp_wpe3_lite_put(void *data)
{
	return qof_smi_isp_module_put(data, ISP8S_PWR_WPE_3_LITE);
}
EXPORT_SYMBOL(smi_isp_wpe3_lite_put);

#ifndef CONFIG_FPGA_EARLY_PORTING
static struct smi_user_pwr_ctrl smi_isp_dip_pwr_cb = {
	 .name = "qof_isp_dip",
	 .data = &is_smi_use_qof_locked[ISP8S_PWR_DIP],
	 .smi_user_id =  MTK_SMI_IMG_DIP,
	 .smi_user_get = smi_isp_dip_get,
	 .smi_user_get_if_in_use = smi_isp_dip_get_if_in_use,
	 .smi_user_put = smi_isp_dip_put,
};

static struct smi_user_pwr_ctrl smi_isp_traw_pwr_cb = {
	 .name = "qof_isp_traw",
	 .data = &is_smi_use_qof_locked[ISP8S_PWR_TRAW],
	 .smi_user_id =  MTK_SMI_IMG_TRAW,
	 .smi_user_get = smi_isp_traw_get,
	 .smi_user_get_if_in_use = smi_isp_traw_get_if_in_use,
	 .smi_user_put = smi_isp_traw_put,
};

static struct smi_user_pwr_ctrl smi_isp_wpe1_eis_pwr_cb = {
	 .name = "qof_isp_wpe1_eis",
	 .data = &is_smi_use_qof_locked[ISP8S_PWR_WPE_1_EIS],
	 .smi_user_id =  MTK_SMI_IMG_WPE_EIS,
	 .smi_user_get = smi_isp_wpe1_eis_get,
	 .smi_user_get_if_in_use = smi_isp_wpe1_eis_get_if_in_use,
	 .smi_user_put = smi_isp_wpe1_eis_put,
};

static struct smi_user_pwr_ctrl smi_isp_wpe2_tnr_pwr_cb = {
	 .name = "qof_isp_wpe2_tnr",
	 .data = &is_smi_use_qof_locked[ISP8S_PWR_WPE_2_TNR],
	 .smi_user_id =  MTK_SMI_IMG_WPE_TNR,
	 .smi_user_get = smi_isp_wpe2_tnr_get,
	 .smi_user_get_if_in_use = smi_isp_wpe2_tnr_get_if_in_use,
	 .smi_user_put = smi_isp_wpe2_tnr_put,
};

static struct smi_user_pwr_ctrl smi_isp_wpe3_lite_pwr_cb = {
	 .name = "qof_isp_wpe3_lite",
	 .data = &is_smi_use_qof_locked[ISP8S_PWR_WPE_3_LITE],
	 .smi_user_id =  MTK_SMI_IMG_WPE_LITE,
	 .smi_user_get = smi_isp_wpe3_lite_get,
	 .smi_user_get_if_in_use = smi_isp_wpe3_lite_get_if_in_use,
	 .smi_user_put = smi_isp_wpe3_lite_put,
};
#endif

static void imgsys_cmdq_qof_set_restore_done(struct cmdq_pkt *pkt, u32 pwr_id)
{
	unsigned int addr;
	unsigned int val;
	unsigned int mask;
	struct qof_events *qof_event;

	if (pkt == NULL || pwr_id >= ISP8S_PWR_NUM) {
		QOF_LOGI("qof param wrong!\n");
		return;
	}

	addr = qof_reg_table[pwr_id][QOF_REG_IMG_GCE_RESTORE_DONE].addr;
	val = qof_reg_table[pwr_id][QOF_REG_IMG_GCE_RESTORE_DONE].val;
	mask = qof_reg_table[pwr_id][QOF_REG_IMG_GCE_RESTORE_DONE].mask;
	qof_event = &qof_events_isp8s[pwr_id];

	// set restore done
	cmdq_pkt_write(pkt, NULL, addr /* address*/ , val /* val */ , mask /* mask */ );

	// put down hw event
	cmdq_pkt_clear_event(pkt, qof_event->hw_event_restore);

}

void mtk_imgsys_qof_print_hw_info(u32 mod)
{
	struct qof_events *event;

	if (IS_MOD_SUPPORT_QOF(mod) == false)
		return;

	event = &qof_events_isp8s[mod];

	QOF_LOGI("MOD[%d]A_event_cnt(0x%x):0x%x;B_event_cnt(0x%x):0x%x;C_event_cnt(0x%x):0x%x;",
	mod,
	qof_reg_table[mod][QOF_REG_IMG_VM_A].addr,
	readl
	(g_maped_rg[MAPED_RG_QOF_CNT_A_REG_BASE] + (qof_reg_table[mod][QOF_REG_IMG_VM_A].addr - QOF_IMG_EVENT_A)),
	qof_reg_table[mod][QOF_REG_IMG_VM_B].addr,
	readl
	(g_maped_rg[MAPED_RG_QOF_CNT_B_REG_BASE] + (qof_reg_table[mod][QOF_REG_IMG_VM_B].addr - QOF_IMG_EVENT_B)),
	qof_reg_table[mod][QOF_REG_IMG_VM_C].addr,
	readl
	(g_maped_rg[MAPED_RG_QOF_CNT_C_REG_BASE] + (qof_reg_table[mod][QOF_REG_IMG_VM_C].addr - QOF_IMG_EVENT_C)));

	QOF_LOGI("VOTE_DEBUG(0x%x):0x%x;TRIG_CNT(0x%x):0x%x;ITC_SRC_SEL(0x%x):0x%x;ACK_2ND_WAIT_TH(0x%x):0x%x;",
	qof_reg_table[mod][QOF_REG_IMG_QOF_VOTER_DBG].addr,
	readl(QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_QOF_VOTER_DBG].addr)),
	qof_reg_table[mod][QOF_REG_IMG_TRG_ON_CNT].addr,
	readl(QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_TRG_ON_CNT].addr)),
	qof_reg_table[mod][QOF_REG_IMG_ITC_SRC_SEL].addr,
	readl(QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_ITC_SRC_SEL].addr)),
	qof_reg_table[mod][QOF_REG_IMG_PWR_ACK_2ND_WAIT_TH].addr,
	readl(QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_PWR_ACK_2ND_WAIT_TH].addr)));

	QOF_LOGI("PWR_ST(0x%x):0x%x;SAVE_DONE(0x%x):0x%x;SP2_TOP(0x%x):0x%08x;QOF(0x%x):0x%x;GCE(0x%x):0x%x;\n",
	qof_reg_table[mod][QOF_REG_IMG_POWER_STATE].addr,
	readl(QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_POWER_STATE].addr)),
	qof_reg_table[mod][QOF_REG_IMG_GCE_SAVE_DONE].addr,
	readl(QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_GCE_SAVE_DONE].addr)),
	QOF_REG_BASE+0xa74,
	readl(QOF_GET_REMAP_ADDR(QOF_REG_BASE+0xa74)),
	qof_reg_table[mod][QOF_REG_IMG_HW_SET_EN].addr,
	readl(QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_HW_SET_EN].addr)),
	qof_reg_table[mod][QOF_REG_IMG_GCE_RESTORE_EN].addr,
	readl(QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_GCE_RESTORE_EN].addr)));


	QOF_LOGI("qof_hw_info:rg(0x%x):0x%x;rg(0x%x):0x%x;rg(0x%x):0x%x;rg(0x%x):0x%x;rg(0x%x):0x%x\n",
	qof_reg_table[mod][QOF_REG_IMG_QOF_VOTER_DBG].addr,
	readl(QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_QOF_VOTER_DBG].addr)),
	qof_reg_table[mod][QOF_REG_IMG_QOF_DONE_STATUS].addr,
	readl(QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_QOF_DONE_STATUS].addr)),
	qof_reg_table[mod][QOF_REG_IMG_ITC_STATUS].addr,
	readl(QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_ITC_STATUS].addr)),
	qof_reg_table[mod][QOF_REG_IMG_QOF_MTC_ST_LSB].addr,
	readl(QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_QOF_MTC_ST_LSB].addr)),
	qof_reg_table[mod][QOF_REG_IMG_QOF_MTC_ST_MSB2].addr,
	readl(QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_QOF_MTC_ST_MSB2].addr)));
}

static void mtk_qof_print_mtcmos_status(void)
{
	QOF_LOGI("MTCMOS:cnt[%u]MAIN[0x%x]VCORE[0x%x]DIP[0x%x]TRAW[0x%x]W1[0x%x]W2[0x%x]W3[0x%x]Link[0x%x]",
		imgsys_voter_cnt_locked,
		(readl(g_maped_rg[MAPED_RG_ISP_MAIN_PWR_CON])),
		(readl(g_maped_rg[MAPED_RG_ISP_VCORE_PWR_CON])),
		(readl(g_maped_rg[MAPED_RG_ISP_DIP_PWR_CON])),
		(readl(g_maped_rg[MAPED_RG_ISP_TRAW_PWR_CON])),
		(readl(g_maped_rg[MAPED_RG_ISP_WPE_EIS_PWR_CON])),
		(readl(g_maped_rg[MAPED_RG_ISP_WPE_TNR_PWR_CON])),
		(readl(g_maped_rg[MAPED_RG_ISP_WPE_LITE_PWR_CON])),
		(readl(g_maped_rg[MAPED_RG_HWCCF_REG_STA])));
}

static void mtk_qof_print_cg_status(void)
{
	void __iomem *addr;
	unsigned int addr_val;

	addr = QOF_GET_REMAP_ADDR(qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_QOF_STATE_DBG].addr);
	addr_val = readl(addr);
	if (((addr_val & BIT(1)) == BIT(1)) ||
		((addr_val & BIT(6)) == BIT(6))) {
		QOF_LOGI(" dip CG Status: IMG_MAIN[0x%x], NR1[0x%x], NR2[0x%x], DIP_TOP[0x%x]",
		(readl(g_maped_rg[MAPED_RG_IMG_CG_IMGSYS_MAIN])),
		(readl(g_maped_rg[MAPED_RG_IMG_CG_DIP_NR1_DIP1])),
		(readl(g_maped_rg[MAPED_RG_IMG_CG_DIP_NR2_DIP1])),
		(readl(g_maped_rg[MAPED_RG_IMG_CG_DIP_TOP_DIP1])));
	} else
		QOF_LOGI("qof mtcmos dip is off. sta=0x%x", readl(addr));
	addr = QOF_GET_REMAP_ADDR(qof_reg_table[ISP8S_PWR_TRAW][QOF_REG_IMG_QOF_STATE_DBG].addr);
	addr_val = readl(addr);
	if (((addr_val & BIT(1)) == BIT(1)) ||
		((addr_val & BIT(6)) == BIT(6))) {
		QOF_LOGI(" traw CG Status: TRAW_CAP[0x%x], TRAW_DIP1[0x%x]",
		(readl(g_maped_rg[MAPED_RG_IMG_CG_TRAW_CAP_DIP1])),
		(readl(g_maped_rg[MAPED_RG_IMG_CG_TRAW_DIP1])));
	} else
		QOF_LOGI("qof mtcmos traw is off. sta=0x%x",  readl(addr));
	addr = QOF_GET_REMAP_ADDR(qof_reg_table[ISP8S_PWR_WPE_1_EIS][QOF_REG_IMG_QOF_STATE_DBG].addr);
	addr_val = readl(addr);
	if (((addr_val & BIT(1)) == BIT(1)) ||
		((addr_val & BIT(6)) == BIT(6))) {
		QOF_LOGI(" wpe1 CG Status: WPE1[0x%x]",
		(readl(g_maped_rg[MAPED_RG_IMG_CG_WPE1_DIP1])));
	} else
		QOF_LOGI("qof mtcmos wpe1 is off. sta=0x%x", readl(addr));
	addr = QOF_GET_REMAP_ADDR(qof_reg_table[ISP8S_PWR_WPE_2_TNR][QOF_REG_IMG_QOF_STATE_DBG].addr);
	addr_val = readl(addr);
	if (((addr_val & BIT(1)) == BIT(1)) ||
		((addr_val & BIT(6)) == BIT(6))) {
		QOF_LOGI(" wpe2 CG Status: WPE2[0x%x]",
		(readl(g_maped_rg[MAPED_RG_IMG_CG_WPE2_DIP1])));
	} else
		QOF_LOGI("qof mtcmos wpe2 is off. sta=0x%x", readl(addr));
	addr = QOF_GET_REMAP_ADDR(qof_reg_table[ISP8S_PWR_WPE_3_LITE][QOF_REG_IMG_QOF_STATE_DBG].addr);
	addr_val = readl(addr);
	if (((addr_val & BIT(1)) == BIT(1)) ||
		((addr_val & BIT(6)) == BIT(6))) {
		QOF_LOGI(" wpe3 CG Status: WPE3[0x%x]",
		(readl(g_maped_rg[MAPED_RG_IMG_CG_WPE3_DIP1])));
	} else
		QOF_LOGI("qof mtcmos wpe3 is off. sta=0x%x", readl(addr));
}

static void imgsys_cmdq_init_qof_events(void)
{
	struct qof_events *event;
	u32 usr_id = 0;

	for(usr_id = ISP8S_PWR_START; usr_id < ISP8S_PWR_NUM; usr_id++) {
		event = &qof_events_isp8s[usr_id];
		event->sw_event_lock = sw_event_lock_list[usr_id];
		event->hw_event_restore = hw_event_restore_list[usr_id];
	}
}

static void imgsys_cmdq_qof_init_pwr_thread(struct mtk_imgsys_dev *imgsys_dev)
{
	struct device *dev = imgsys_dev->dev;
	u32 thd_idx = 0;
	u32 idx = 0;

	for (thd_idx = IMGSYS_NOR_THD; thd_idx < IMGSYS_NOR_THD + IMGSYS_PWR_THD; thd_idx++) {
		idx = thd_idx - IMGSYS_NOR_THD;
		if (idx < QOF_TOTAL_THREAD) {
			imgsys_pwr_clt[thd_idx - IMGSYS_NOR_THD] = cmdq_mbox_create(dev, thd_idx);
			QOF_LOGI(
				"%s: cmdq_mbox_create pwr_thd(%d, 0x%lx)",
				__func__, thd_idx, (unsigned long)imgsys_pwr_clt[thd_idx - IMGSYS_NOR_THD]);
		} else {
			imgsys_pwr_clt[thd_idx - IMGSYS_NOR_THD] = NULL;
			QOF_LOGI("qof thread indexs (%u) are not match !", thd_idx);
		}
	}
}

static void qof_cmdq_set_module_rst(struct mtk_imgsys_dev *imgsys_dev,
		struct cmdq_pkt *pkt,
		const struct imgsys_mtcmos_data *pwr)
{

	if (imgsys_dev == NULL || pkt == NULL || pwr == NULL) {
		QOF_LOGE("param is null (%d/%d/%d)",
			(imgsys_dev == NULL), (pkt == NULL), (pwr == NULL));
		return;
	}

	switch (pwr->pwr_id) {
	case ISP8S_PWR_DIP:
		if (imgsys_dev->modules[IMGSYS_MOD_DIP].cmdq_set)
			imgsys_dev->modules[IMGSYS_MOD_DIP].cmdq_set(imgsys_dev, pkt, REG_MAP_E_DIP);
		else
			QOF_LOGE("cmdq_set function pointer is null");
		break;
	case ISP8S_PWR_TRAW:
		if (imgsys_dev->modules[IMGSYS_MOD_TRAW].cmdq_set)
			imgsys_dev->modules[IMGSYS_MOD_TRAW].cmdq_set(imgsys_dev, pkt, REG_MAP_E_TRAW);
		else
			QOF_LOGE("cmdq_set function pointer is null");
		break;
	case ISP8S_PWR_WPE_1_EIS:
		if (imgsys_dev->modules[IMGSYS_MOD_WPE].cmdq_set)
			imgsys_dev->modules[IMGSYS_MOD_WPE].cmdq_set(imgsys_dev, pkt, REG_MAP_E_WPE_EIS);
		else
			QOF_LOGE("cmdq_set function pointer is null");
		if (imgsys_dev->modules[IMGSYS_MOD_PQDIP].cmdq_set)
			imgsys_dev->modules[IMGSYS_MOD_PQDIP].cmdq_set(imgsys_dev, pkt, REG_MAP_E_PQDIP_A);
		else
			QOF_LOGE("cmdq_set function pointer is null");
		break;
	case ISP8S_PWR_WPE_2_TNR:
		if (imgsys_dev->modules[IMGSYS_MOD_OMC].cmdq_set)
			imgsys_dev->modules[IMGSYS_MOD_OMC].cmdq_set(imgsys_dev, pkt, REG_MAP_E_OMC_TNR);
		else
			QOF_LOGE("cmdq_set function pointer is null");
		if (imgsys_dev->modules[IMGSYS_MOD_PQDIP].cmdq_set)
			imgsys_dev->modules[IMGSYS_MOD_PQDIP].cmdq_set(imgsys_dev, pkt, REG_MAP_E_PQDIP_B);
		else
			QOF_LOGE("cmdq_set function pointer is null");

		if (imgsys_dev->modules[IMGSYS_MOD_ME].cmdq_set)
			imgsys_dev->modules[IMGSYS_MOD_ME].cmdq_set(imgsys_dev, pkt, REG_MAP_E_ME);
		else
			QOF_LOGE("cmdq_set function pointer is null");
		break;
	case ISP8S_PWR_WPE_3_LITE:
		if (imgsys_dev->modules[IMGSYS_MOD_WPE].cmdq_set)
			imgsys_dev->modules[IMGSYS_MOD_WPE].cmdq_set(imgsys_dev, pkt, REG_MAP_E_WPE_LITE);
		else
			QOF_LOGE("cmdq_set function pointer is null");
		if (imgsys_dev->modules[IMGSYS_MOD_OMC].cmdq_set)
			imgsys_dev->modules[IMGSYS_MOD_OMC].cmdq_set(imgsys_dev, pkt, REG_MAP_E_OMC_LITE);
		else
			QOF_LOGE("cmdq_set function pointer is null");
		if (imgsys_dev->modules[IMGSYS_MOD_DFP].cmdq_set)
			imgsys_dev->modules[IMGSYS_MOD_DFP].cmdq_set(imgsys_dev, pkt, REG_MAP_E_DFP_TOP);
		else
			QOF_LOGE("cmdq_set function pointer is null");
		break;
	default:
		QOF_LOGE("case invalid[%u]\n", pwr->pwr_id);
		break;
	}
}

static void imgsys_cmdq_restore_locked(struct mtk_imgsys_dev *imgsys_dev,
		struct cmdq_pkt *pkt,
		const struct imgsys_mtcmos_data *pwr)
{
	if (imgsys_dev == NULL || pkt == NULL || pwr == NULL) {
		QOF_LOGE("param is null (%d/%d/%d)\n",
			(imgsys_dev == NULL), (pkt == NULL), (pwr == NULL));
		return;
	}

	/* set module rst */

	qof_cmdq_set_module_rst(imgsys_dev, pkt, pwr);

	/* register qof */
	if (pwr->qof_restore_done) {
		pwr->qof_restore_done(pkt, pwr->pwr_id);
	}

}

static void qof_start_pwr_restore_task(struct mtk_imgsys_dev *imgsys_dev,
		struct cmdq_client *client, u32 pwr_id, u32 th_id,
		struct qof_events *event)
{
	struct cmdq_pkt *restore_pkt;

	if (pwr_id >= ISP8S_PWR_NUM || th_id >= IMG_GCE_THREAD_PWR_END) {
		QOF_LOGI("%s:qof id is invalid %u/%u\n", __func__, pwr_id, th_id);
		return;
	}

	restore_pkt = pwr_buf_handle[th_id].restore_pkt;

	/* Power on kernel thread */
	cmdq_mbox_enable(client->chan);

	if (!restore_pkt) {
		restore_pkt = pwr_buf_handle[th_id].restore_pkt = cmdq_pkt_create(client);
		if (!restore_pkt) {
			QOF_LOGI("%s:create cmdq package fail\n", __func__);
			return;
		}
	}

	/* Program start */
	/* Wait for restore hw event */
	cmdq_pkt_wfe(restore_pkt, event->hw_event_restore);
	// wa for gce thd lacked
	imgsys_cmdq_restore_locked(imgsys_dev, restore_pkt,
		&isp8s_module_data[pwr_id]);

	/* Program end */
	restore_pkt->priority = IMGSYS_PRI_HIGH;
	restore_pkt->no_irq = true;
	cmdq_pkt_finalize_loop(restore_pkt);
	cmdq_pkt_flush_async(restore_pkt, NULL, (void *)restore_pkt);
}

//TODO: refine it with qof_start_pwr_loop
static void qof_mapping_pwr_loop(u32 pwr, struct mtk_imgsys_dev *imgsys_dev)
{
	u32 th_id = IMG_GCE_THREAD_PWR_START;

	if (!imgsys_dev) {
		cmdq_err("qof imgsys_dev is NULL");
		return;
	}

	th_id = get_thread_id_by_pwr_id(pwr);
	if (th_id >= QOF_TOTAL_THREAD) {
		cmdq_err("qof th_id mapping wrong[%u]\n", th_id);
		return;
	}

	if (imgsys_pwr_clt[th_id])
		qof_start_pwr_restore_task(imgsys_dev, imgsys_pwr_clt[th_id], pwr, th_id, &qof_events_isp8s[pwr]);
	else
		QOF_LOGI("qof imgsys_pwr_clt[%u] = null thid=%d\n", pwr, th_id);
}

static void qof_start_pwr_loop(struct mtk_imgsys_dev *imgsys_dev)
{
	u32 pwr = ISP8S_PWR_START;

	if (!imgsys_dev) {
		cmdq_err("qof imgsys_dev is NULL");
		return;
	}

	for (; (pwr < ISP8S_PWR_NUM); pwr++) {
		/* Initial power restore thread */
		qof_mapping_pwr_loop(pwr, imgsys_dev);
	}
}

void qof_start_all_gce_loop(struct mtk_imgsys_dev *imgsys_dev)
{
	qof_start_pwr_loop(imgsys_dev);
}

static void qof_stop_all_gce_loop(void)
{
	int idx = 0;
	u32 thd_idx = 0;

	/* pwr thread */
	for (thd_idx = IMGSYS_NOR_THD; thd_idx < IMGSYS_NOR_THD + IMGSYS_PWR_THD; thd_idx++) {
		idx = thd_idx - IMGSYS_NOR_THD;
		if (idx >= QOF_TOTAL_THREAD || idx < 0) {
			QOF_LOGE("idx is wrong %d\n", idx);
			continue;
		}

		if (imgsys_pwr_clt[idx] != NULL) {
			cmdq_mbox_stop(imgsys_pwr_clt[idx]);
			cmdq_mbox_disable(imgsys_pwr_clt[idx]->chan);
		}
		if (pwr_buf_handle[idx].restore_pkt != NULL) {
			cmdq_pkt_destroy(pwr_buf_handle[idx].restore_pkt);
			pwr_buf_handle[idx].restore_pkt = NULL;
		}
	}
}

void mtk_imgsys_cmdq_qof_init(struct mtk_imgsys_dev *imgsys_dev, struct cmdq_client *imgsys_clt)
{
	int ver = 0;
	int rg_idx = 0;
	unsigned long flag;
	u32 pwr = 0;

	QOF_LOGI("+\n");
	if (of_property_read_u32_index(imgsys_dev->dev->of_node,
		"mediatek,imgsys-qof-ver", 0, &ver) == 0)
		QOF_LOGI("[%s] qof version = %u\n", __func__, ver);

	/* init global val */
	imgsys_dev->qof_ver = ver;
	g_qof_ver = ver;
	g_qof_debug_level = 0;
	g_imgsys_dev = imgsys_dev;
	g_qof_smi_cnt = 0;
	is_poll_event_mode = false;
	spin_lock_init(&qof_lock);
	spin_lock_irqsave(&qof_lock, flag);
	imgsys_voter_cnt_locked = 0;
	for(pwr = ISP8S_PWR_START; pwr < ISP8S_PWR_NUM; pwr++)
		is_smi_use_qof_locked[pwr] = false;
	spin_unlock_irqrestore(&qof_lock, flag);

	if (imgsys_dev->qof_ver == MTK_IMGSYS_QOF_FUNCTION_OFF)
		return;

	QOF_LOGI("qof version = %u, poll_event_mode = %d", ver, is_poll_event_mode);

	imgsys_cmdq_qof_init_pwr_thread(imgsys_dev);

	imgsys_cmdq_init_qof_events();

	/* ioremap rg */
	g_maped_rg[MAPED_RG_ISP_TRAW_PWR_CON]		= ioremap(ISP_TRAW_PWR_CON, 4);
	g_maped_rg[MAPED_RG_ISP_DIP_PWR_CON]		= ioremap(ISP_DIP_PWR_CON, 4);
	g_maped_rg[MAPED_RG_ISP_MAIN_PWR_CON]		= ioremap(ISP_MAIN_PWR_CON, 4);
	g_maped_rg[MAPED_RG_ISP_VCORE_PWR_CON]		= ioremap(ISP_VCORE_PWR_CON, 4);
	g_maped_rg[MAPED_RG_ISP_WPE_EIS_PWR_CON]	= ioremap(ISP_WPE_EIS_PWR_CON, 4);
	g_maped_rg[MAPED_RG_ISP_WPE_TNR_PWR_CON]	= ioremap(ISP_WPE_TNR_PWR_CON, 4);
	g_maped_rg[MAPED_RG_ISP_WPE_LITE_PWR_CON]	= ioremap(ISP_WPE_LITE_PWR_CON, 4);
	g_maped_rg[MAPED_RG_IMG_CG_IMGSYS_MAIN]		= ioremap(IMG_CG_IMGSYS_MAIN, 4);
	g_maped_rg[MAPED_RG_IMG_CG_DIP_NR1_DIP1]	= ioremap(IMG_CG_DIP_NR1_DIP1, 4);
	g_maped_rg[MAPED_RG_IMG_CG_DIP_NR2_DIP1]	= ioremap(IMG_CG_DIP_NR2_DIP1, 4);
	g_maped_rg[MAPED_RG_IMG_CG_DIP_TOP_DIP1]	= ioremap(IMG_CG_DIP_TOP_DIP1, 4);
	g_maped_rg[MAPED_RG_IMG_CG_TRAW_CAP_DIP1]	= ioremap(IMG_CG_TRAW_CAP_DIP1, 4);
	g_maped_rg[MAPED_RG_IMG_CG_TRAW_DIP1]		= ioremap(IMG_CG_TRAW_DIP1, 4);
	g_maped_rg[MAPED_RG_IMG_CG_WPE1_DIP1]		= ioremap(IMG_CG_WPE1_DIP1, 4);
	g_maped_rg[MAPED_RG_IMG_CG_WPE2_DIP1]		= ioremap(IMG_CG_WPE2_DIP1, 4);
	g_maped_rg[MAPED_RG_IMG_CG_WPE3_DIP1]		= ioremap(IMG_CG_WPE3_DIP1, 4);
	g_maped_rg[MAPED_RG_QOF_REG_BASE]			= ioremap(QOF_REG_BASE, 4);
	g_maped_rg[MAPED_RG_QOF_CNT_A_REG_BASE]		= ioremap(QOF_IMG_EVENT_A, 4);
	g_maped_rg[MAPED_RG_QOF_CNT_B_REG_BASE]		= ioremap(QOF_IMG_EVENT_B, 4);
	g_maped_rg[MAPED_RG_QOF_CNT_C_REG_BASE]		= ioremap(QOF_IMG_EVENT_C, 4);
	g_maped_rg[MAPED_RG_MMPC_REG_BASE]			= ioremap(MMPC_REG_BASE, 4);
	g_maped_rg[MAPED_RG_HWCCF_REG_CLR]			= ioremap(HWCCF_LINK_CLR_ADDR, 4);
	g_maped_rg[MAPED_RG_HWCCF_REG_SET]			= ioremap(HWCCF_LINK_SET_ADDR, 4);
	g_maped_rg[MAPED_RG_HWCCF_REG_STA]			= ioremap(HWCCF_LINK_STA_ADDR, 4);
	g_maped_rg[MAPED_RG_RTFF_BASE]				= ioremap(RTFF_REG_BASE, 4);

	for (rg_idx = MAPED_RG_LIST_START; rg_idx < MAPED_RG_LIST_NUM; rg_idx++) {
		if (!g_maped_rg[rg_idx]) {
			QOF_LOGI("qof %s Unable to ioremap %d registers !\n",
				__func__, rg_idx);
		}
	}
#ifndef CONFIG_FPGA_EARLY_PORTING
	/* smi cb register */
	if (IS_MOD_SUPPORT_QOF((int)ISP8S_PWR_DIP))
		mtk_smi_dbg_register_pwr_ctrl_cb(&smi_isp_dip_pwr_cb);
	if (IS_MOD_SUPPORT_QOF((int)ISP8S_PWR_TRAW))
		mtk_smi_dbg_register_pwr_ctrl_cb(&smi_isp_traw_pwr_cb);
	if (IS_MOD_SUPPORT_QOF((int)ISP8S_PWR_WPE_1_EIS))
		mtk_smi_dbg_register_pwr_ctrl_cb(&smi_isp_wpe1_eis_pwr_cb);
	if (IS_MOD_SUPPORT_QOF((int)ISP8S_PWR_WPE_2_TNR))
		mtk_smi_dbg_register_pwr_ctrl_cb(&smi_isp_wpe2_tnr_pwr_cb);
	if (IS_MOD_SUPPORT_QOF((int)ISP8S_PWR_WPE_3_LITE))
		mtk_smi_dbg_register_pwr_ctrl_cb(&smi_isp_wpe3_lite_pwr_cb);
#endif
	QOF_LOGI("-\n");
}

void mtk_imgsys_cmdq_qof_release(struct mtk_imgsys_dev *imgsys_dev, struct cmdq_client *imgsys_clt)
{
	/* release resource */
	int idx = 0;
	int rg_idx = 0;
	u32 thd_idx = 0;

	if (!imgsys_clt) {
		cmdq_err("cl is NULL");
		dump_stack();
		return;
	}
	QOF_LOGI("release resource +\n");

	/* pwr thread */
	for (thd_idx = IMGSYS_NOR_THD; thd_idx < IMGSYS_NOR_THD + IMGSYS_PWR_THD; thd_idx++) {
		idx = thd_idx - IMGSYS_NOR_THD;
		if (idx >= QOF_TOTAL_THREAD || idx < 0) {
			QOF_LOGE("idx is wrong %d\n", idx);
			continue;
		}
	}

	/* iounmap */
	for (rg_idx = MAPED_RG_LIST_START; rg_idx < MAPED_RG_LIST_NUM; rg_idx++) {
		if (g_maped_rg[rg_idx])
			iounmap(g_maped_rg[rg_idx]);
	}

	QOF_LOGI("release resource -\n");
}



static void qof_engine_on_setting(const u32 mod)
{
	void __iomem *io_addr;
	u32 tmp;
	u32 chk_val;
	u32 chk_mask;

	/* DIP_CINE setting*/
	if (mod == ISP8S_PWR_DIP) {
		QOF_WRITE_VAL(qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_OPT_MTC_ACT], 0x1);

		io_addr = QOF_GET_REMAP_ADDR(qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_OPT_MTC_ACT].addr);
		if (readl_poll_timeout_atomic
			(io_addr, tmp, (tmp & BIT(23)) == BIT(23), POLL_DELAY_US, TIMEOUT_1000US) < 0)
			QOF_LOGE("QOF_REG_IMG_OPT_MTC_ACT vote fail 0x%08x\n", readl(io_addr));
	}

	io_addr = QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_HW_CLR_EN].addr);
	write_mask(io_addr,
		qof_reg_table[mod][QOF_REG_IMG_HW_CLR_EN].val,
		qof_reg_table[mod][QOF_REG_IMG_HW_CLR_EN].mask);

	io_addr = QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_HW_SET_EN].addr);
	write_mask(io_addr,
		qof_reg_table[mod][QOF_REG_IMG_HW_SET_EN].val,
		qof_reg_table[mod][QOF_REG_IMG_HW_SET_EN].mask);

	io_addr = QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_GCE_RESTORE_EN].addr);
	write_mask(io_addr,
		qof_reg_table[mod][QOF_REG_IMG_GCE_RESTORE_EN].val,
		qof_reg_table[mod][QOF_REG_IMG_GCE_RESTORE_EN].mask);

	QOF_WRITE_VAL(qof_reg_table[mod][QOF_REG_IMG_HW_HWCCF_EN], 0x0);

	QOF_WRITE_VAL(qof_reg_table[mod][QOF_REG_IMG_HWCCF_SW_VOTE_ON], 0x1);

	/* DIP_CINE pulse*/
	if (mod == ISP8S_PWR_DIP) {
		QOF_WRITE_VAL(qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_QOF_SPARE1_TOP], 0x8);
		QOF_WRITE_VAL(qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_QOF_SPARE1_TOP], 0x0);
	}

	io_addr = QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_HWCCF_MTCMOS].addr);
	chk_val = qof_reg_table[mod][QOF_REG_IMG_HWCCF_MTCMOS].val;
	chk_mask = qof_reg_table[mod][QOF_REG_IMG_HWCCF_MTCMOS].mask;
	if (readl_poll_timeout_atomic
		(io_addr, tmp, (tmp & chk_mask) == chk_val, POLL_DELAY_US, TIMEOUT_1000US) < 0)
		QOF_LOGE("[%d] HWCCF SW vote fail 0x%08x\n", mod, readl(io_addr));

	/* UNLINK HWCCF */
	hwccf_unlink(mod);

	/* rtff initial */
	rtff_init(mod);

	/* Add voter */
	io_addr = QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_APMCU_SET].addr);
	write_mask(io_addr,
		qof_reg_table[mod][QOF_REG_IMG_APMCU_SET].val,
		qof_reg_table[mod][QOF_REG_IMG_APMCU_SET].mask);

	/* qof engine enable */
	io_addr = QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_QOF_ENG_EN].addr);
	write_mask(io_addr,
		qof_reg_table[mod][QOF_REG_IMG_QOF_ENG_EN].val,
		qof_reg_table[mod][QOF_REG_IMG_QOF_ENG_EN].mask);
}

static void qof_set_engine_on(const u32 mod)
{
	void __iomem *io_addr;
	int tmp;

	if(IS_MOD_SUPPORT_QOF(mod) == false)
		return;

	qof_engine_on_setting(mod);

	io_addr = QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_QOF_STATE_DBG].addr);
	if (readl_poll_timeout_atomic
		(io_addr, tmp, (tmp & BIT(1)) == BIT(1), POLL_DELAY_US, TIMEOUT_1000US) < 0) {
		QOF_LOGE("mod[%d] waiting for qof state pwr on timeout, disable support\n",mod);
		mtk_imgsys_cmdq_qof_dump(0, false);
		g_qof_ver &= (~BIT(mod));
		return;
	}
	/* qof APMCU Vote sub */
	io_addr = QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_APMCU_CLR].addr);
	write_mask(io_addr,
		qof_reg_table[mod][QOF_REG_IMG_APMCU_CLR].val,
		qof_reg_table[mod][QOF_REG_IMG_APMCU_CLR].mask);
	io_addr = QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_QOF_STATE_DBG].addr);
	if (readl_poll_timeout_atomic
		(io_addr, tmp, (tmp & BIT(4)) == BIT(4), POLL_DELAY_US, TIMEOUT_1000US) < 0) {
		QOF_LOGI("Warning: mod[%d] waiting qof state pwr off timeout, qof may be voted by smi\n",mod);
		mtk_imgsys_cmdq_qof_dump(0, false);
		// engine off
	}
}

static void qof_engine_off_setting(const u32 mod)
{
	void __iomem *io_addr;

	// disable hw_seq
	io_addr =  QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_HW_HWCCF_EN].addr);
	write_mask(io_addr,
		0,
		qof_reg_table[mod][QOF_REG_IMG_HW_HWCCF_EN].mask);
	// Set engine enable = 0
	io_addr =  QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_QOF_ENG_EN].addr);
	write_mask(io_addr,
		0,
		qof_reg_table[mod][QOF_REG_IMG_QOF_ENG_EN].mask);
	io_addr =  QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_APMCU_CLR].addr);
	write_mask(io_addr,
		qof_reg_table[mod][QOF_REG_IMG_APMCU_CLR].val,
		qof_reg_table[mod][QOF_REG_IMG_APMCU_CLR].mask);

	hwccf_link(mod);

	/* QOF_REG_IMG_HWCCF_SW_VOTE_OFF, handover HWCCF*/
	QOF_WRITE_VAL(qof_reg_table[mod][QOF_REG_IMG_HWCCF_SW_VOTE_OFF], 0x1);

	/* DIP_CINE pulse*/
	if (mod == ISP8S_PWR_DIP) {
		QOF_WRITE_VAL(qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_QOF_SPARE1_TOP], 0x4);
		QOF_WRITE_VAL(qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_QOF_SPARE1_TOP], 0x0);
	}
	// disable rtff
	rtff_uninit(mod);
}

static void qof_locked_set_engine_off(const u32 mod)
{
	void __iomem *io_addr;
	int tmp;

	if (IS_MOD_SUPPORT_QOF(mod) == false)
		return;

	io_addr =  QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_APMCU_SET].addr);
	write_mask(io_addr,
		qof_reg_table[mod][QOF_REG_IMG_APMCU_SET].val,
		qof_reg_table[mod][QOF_REG_IMG_APMCU_SET].mask);

	io_addr =  QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_QOF_STATE_DBG].addr);
	if (readl_poll_timeout_atomic
		(io_addr, tmp, (tmp & BIT(1)) == BIT(1), POLL_DELAY_US, TIMEOUT_100000US) < 0) {
		QOF_LOGE("fatal error: mod[%d] waiting for qof state pwr on timeout, disable support\n", mod);
		mtk_imgsys_cmdq_qof_dump(0, false);
		g_qof_ver &= (~BIT(mod));
	}
	qof_engine_off_setting(mod);

}

static void qof_locked_stream_off_sync(void)
{
	u32 qof_module = QOF_SUPPORT_START;

	QOF_LOGI("qof stream off+\n");
	for (; qof_module < QOF_TOTAL_MODULE; qof_module++) {
		qof_locked_set_engine_off(qof_module);
	}
	QOF_LOGI("qof stream off-\n");
}

void mtk_imgsys_cmdq_qof_stream_on(struct mtk_imgsys_dev *imgsys_dev)
{
	u32 mod = 0;
	unsigned long flag;

	QOF_LOGI("qof stream on+\n");

	qof_start_all_gce_loop(imgsys_dev);

	spin_lock_irqsave(&qof_lock, flag);

	for (mod = QOF_SUPPORT_START; mod < QOF_TOTAL_MODULE; mod++) {
		if (IS_MOD_SUPPORT_QOF(mod))
			qof_set_engine_on(mod);
		else
			QOF_LOGI("module[%u] not support QOF, ver[%u]\n", mod, g_qof_ver);
	}

	imgsys_voter_cnt_locked++;

	spin_unlock_irqrestore(&qof_lock, flag);

	mtk_imgsys_cmdq_qof_dump(0, false);

	imgsys_qof_set_dbg_thread(true);

	QOF_LOGI("stream on-. success. imgsys_voter_cnt_locked=%u\n", imgsys_voter_cnt_locked);
}

void mtk_imgsys_cmdq_get_non_qof_module(u32 *non_qof_modules)
{
	if (non_qof_modules == NULL) {
		QOF_LOGE("ver[%u], param is null!\n", g_qof_ver);
		return;
	}

	*non_qof_modules = 0;
	if ((g_qof_ver & BIT(QOF_SUPPORT_DIP)) == BIT(QOF_SUPPORT_DIP))
		*non_qof_modules |= BIT(IMGSYS_MOD_DIP);
	if ((g_qof_ver & BIT(QOF_SUPPORT_TRAW)) == BIT(QOF_SUPPORT_TRAW))
		*non_qof_modules |= BIT(IMGSYS_MOD_TRAW);
	if (((g_qof_ver & BIT(QOF_SUPPORT_WPE_EIS)) == BIT(QOF_SUPPORT_WPE_EIS)) &&
		((g_qof_ver & BIT(QOF_SUPPORT_WPE_TNR)) == BIT(QOF_SUPPORT_WPE_TNR)) &&
		((g_qof_ver & BIT(QOF_SUPPORT_WPE_LITE)) == BIT(QOF_SUPPORT_WPE_LITE))) {
		*non_qof_modules |=
			(
				BIT(IMGSYS_MOD_PQDIP) |
				BIT(IMGSYS_MOD_WPE) |
				BIT(IMGSYS_MOD_OMC) |
				BIT(IMGSYS_MOD_ME) |
				BIT(IMGSYS_MOD_DPE)
			);
	}
	*non_qof_modules = ~*non_qof_modules;
}

void mtk_imgsys_cmdq_qof_stream_off(struct mtk_imgsys_dev *imgsys_dev)
{
	unsigned long flag;

	imgsys_qof_set_dbg_thread(false);

	spin_lock_irqsave(&qof_lock, flag);

	imgsys_voter_cnt_locked--;

	qof_locked_stream_off_sync();

	spin_unlock_irqrestore(&qof_lock, flag);

	qof_stop_all_gce_loop();

	QOF_LOGI("stream off success. imgsys_voter_cnt_locked=%u\n", imgsys_voter_cnt_locked);
}

void imgsys_cmdq_imgsys_dip_cine_link(struct cmdq_pkt *pkt)
{
	//vote sub outer
    // make sure DIP_CINE stable
	cmdq_pkt_poll_sleep(pkt, 0/*poll val*/,
		qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_HWCCF_MTCMOS].addr /*addr*/,
		BIT(20)|BIT(21)|BIT(22) /*mask*/);

	cmdq_pkt_write(pkt, NULL, qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_OPT_MTC_ACT].addr,
					qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_OPT_MTC_ACT].val,
					qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_OPT_MTC_ACT].mask);

	cmdq_pkt_poll_sleep(pkt, qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_OPT_MTC_ACT].val/*poll val*/,
		qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_OPT_MTC_ACT].addr /*addr*/,
		qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_OPT_MTC_ACT].mask /*mask*/);

	cmdq_pkt_write(pkt, NULL, qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_HWCCF_SW_VOTER_ON_SUB_OUTER].addr,
					qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_HWCCF_SW_VOTER_ON_SUB_OUTER].val,
					qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_HWCCF_SW_VOTER_ON_SUB_OUTER].mask);

	cmdq_pkt_write(pkt, NULL, qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_HWCCF_SW_VOTER_ON_SUB_OUTER].addr,
					0x0,
					qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_HWCCF_SW_VOTER_ON_SUB_OUTER].mask);

	cmdq_pkt_poll_sleep(pkt, qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_HWCCF_DIP_CINE].val/*poll val*/,
		(qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_HWCCF_DIP_CINE].addr) /*addr*/,
		(qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_HWCCF_DIP_CINE].mask) /*mask*/);
}

void imgsys_cmdq_imgsys_dip_cine_unlink(struct cmdq_pkt *pkt)
{
	//SW_VOTE_OFF_SUB_OUTER
	cmdq_pkt_write(pkt, NULL, qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_HWCCF_SW_VOTER_OFF_SUB_OUTER].addr,
					qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_HWCCF_SW_VOTER_OFF_SUB_OUTER].val,
					qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_HWCCF_SW_VOTER_OFF_SUB_OUTER].mask);

	cmdq_pkt_write(pkt, NULL, qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_OPT_MTC_ACT].addr,
					0x0,
					qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_OPT_MTC_ACT].mask);

	cmdq_pkt_poll_sleep(pkt, 0/*poll val*/,
		(qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_HWCCF_DIP_CINE].addr)/*addr*/,
		(qof_reg_table[ISP8S_PWR_DIP][QOF_REG_IMG_HWCCF_DIP_CINE].mask) /*mask*/);

}

static void qof_module_vote_add(struct cmdq_pkt *pkt, u32 pwr, u32 user)
{
	struct qof_events *qof_event;

	qof_event = &qof_events_isp8s[pwr];
	if (pkt == NULL) {
		QOF_LOGE("parameter wrong !\n");
		return;
	}

	/* Enter critical section */
	cmdq_pkt_acquire_event(pkt, qof_event->sw_event_lock);


	cmdq_pkt_write(pkt, NULL, (qof_reg_table[pwr][QOF_REG_IMG_EVENT_CNT_ADD].addr) /*address*/,
		qof_reg_table[pwr][QOF_REG_IMG_EVENT_CNT_ADD].val,
		qof_reg_table[pwr][QOF_REG_IMG_EVENT_CNT_ADD].mask);

	cmdq_pkt_poll_sleep(pkt, BIT(1)/*poll val*/,
		(qof_reg_table[pwr][QOF_REG_IMG_QOF_STATE_DBG].addr)/*addr*/, BIT(1) /*mask*/);

	/* End of critical section */
	cmdq_pkt_clear_event(pkt, qof_event->sw_event_lock);
}

static void qof_module_vote_sub(struct cmdq_pkt *pkt, u32 pwr, u32 user)
{
	struct qof_events *qof_event;

	if(IS_MOD_SUPPORT_QOF(pwr) == false)
		return;

	if (pkt == NULL) {
		QOF_LOGE("parameter wrong !\n");
		return;
	}
	qof_event = &qof_events_isp8s[pwr];

	/* Enter critical section */
	cmdq_pkt_acquire_event(pkt, qof_event->sw_event_lock);


	cmdq_pkt_write(pkt, NULL, (qof_reg_table[pwr][QOF_REG_IMG_EVENT_CNT_SUB].addr),
		qof_reg_table[pwr][QOF_REG_IMG_EVENT_CNT_SUB].val,
		qof_reg_table[pwr][QOF_REG_IMG_EVENT_CNT_SUB].mask);

	cmdq_pkt_poll_sleep(pkt, 0,
		(qof_reg_table[pwr][QOF_REG_IMG_QOF_STATE_DBG].addr), BIT(3));

	/* End of critical section */
	cmdq_pkt_clear_event(pkt, qof_event->sw_event_lock);
}

void mtk_imgsys_cmdq_qof_add(struct cmdq_pkt *pkt, bool *qof_need_sub, u32 hw_comb)
{
	u32 pwr = 0;

	if(g_qof_debug_level == QOF_DEBUG_MODE_PERFRAME_DUMP)
		mtk_imgsys_cmdq_qof_dump(0, false);

	for (pwr = QOF_SUPPORT_START; pwr < QOF_TOTAL_MODULE; pwr++) {

		if ((IS_MOD_SUPPORT_QOF(pwr)) &&
			(qof_need_sub[pwr] == false) &&
			(hw_comb & pwr_group[pwr])) {
			qof_module_vote_add(pkt, pwr, QOF_USER_GCE);
			qof_need_sub[pwr] = true;
		}
	}

}

void mtk_imgsys_cmdq_qof_sub(struct cmdq_pkt *pkt, bool *qof_need_sub)
{
	u32 pwr = 0;

	if(g_qof_debug_level == QOF_DEBUG_MODE_PERFRAME_DUMP)
		mtk_imgsys_cmdq_qof_dump(0, false);

	for (pwr = ISP8S_PWR_START; pwr < ISP8S_PWR_NUM; pwr++) {
		if (qof_need_sub[pwr] == true) {
			qof_module_vote_sub(pkt, pwr, QOF_USER_GCE);
			qof_need_sub[pwr] = false;
		}
	}
}

void mtk_imgsys_cmdq_qof_dump(uint32_t hwcomb, bool need_dump_cg)
{
	int mod = 0;

	if (g_qof_ver == MTK_IMGSYS_QOF_FUNCTION_OFF)
		return;

	QOF_LOGI("Common info ver[%u], hwcomb[0x%x], need_dump_cg[%u]", g_qof_ver, hwcomb, need_dump_cg);
	if ((readl(g_maped_rg[MAPED_RG_ISP_MAIN_PWR_CON]) & (BIT(30)|BIT(31)))
		!= (BIT(30)|BIT(31))) {
		QOF_LOGI("isp_main is off. sta=0x%x",
			readl(g_maped_rg[MAPED_RG_ISP_MAIN_PWR_CON]));
		return;
	}
	for (mod = ISP8S_PWR_START; mod < ISP8S_PWR_NUM; mod++)
		mtk_imgsys_qof_print_hw_info(mod);

	mtk_qof_print_mtcmos_status();

	if (need_dump_cg)
		mtk_qof_print_cg_status();
}

static void imgsys_qof_dbg_print_trace(int mod)
{
	u32 value;
	char buf[128];
	int ret;

	ret = snprintf(buf, sizeof(buf),
		"qof_mod_%d",
		mod);
	if (ret < 0 || mod < 0 || mod >= ISP8S_PWR_NUM) {
		pr_err("snprintf failed\n");
		return;
	}
	value = readl(QOF_GET_REMAP_ADDR(qof_reg_table[mod][QOF_REG_IMG_POWER_STATE].addr));
	if ((value & BIT(1)) == BIT(1))
		value = 1;
	else
		value = 0;
	switch(mod) {
	case ISP8S_PWR_DIP:
		ftrace_imgsys_qof_DIP("%s=%u",
			buf,
			(u32) (value & (0xff)));
		break;
	case ISP8S_PWR_TRAW:
		ftrace_imgsys_qof_TRAW("%s=%u",
			buf,
			(u32) (value & (0xff)));
		break;
	case ISP8S_PWR_WPE_1_EIS:
		ftrace_imgsys_qof_EIS("%s=%u",
			buf,
			(u32) (value & (0xff)));
		break;
	case ISP8S_PWR_WPE_2_TNR:
		ftrace_imgsys_qof_TNR("%s=%u",
			buf,
			(u32) (value & (0xff)));
		break;
	case ISP8S_PWR_WPE_3_LITE:
		ftrace_imgsys_qof_LITE("%s=%u",
			buf,
			(u32) (value & (0xff)));
		break;
	}
}

static int imgsys_qof_dbg_thread(void *data)
{
	// loop to trace qof

	while (!kthread_should_stop()) {
		imgsys_qof_dbg_print_trace(ISP8S_PWR_DIP);
		imgsys_qof_dbg_print_trace(ISP8S_PWR_TRAW);
		imgsys_qof_dbg_print_trace(ISP8S_PWR_WPE_1_EIS);
		imgsys_qof_dbg_print_trace(ISP8S_PWR_WPE_2_TNR);
		imgsys_qof_dbg_print_trace(ISP8S_PWR_WPE_3_LITE);
		usleep_range(g_ftrace_time, g_ftrace_time + 5);
	}
	return 0;
}

static void imgsys_qof_set_dbg_thread(bool enable)
{
	static struct task_struct *kthr;
	static bool dbg_en;

	if (imgsys_ftrace_qof_thread_en && enable && !dbg_en) {
		QOF_LOGI("thread START\n");
		kthr = kthread_run(imgsys_qof_dbg_thread,
				NULL, "imgsys-qof-dbg");
		if (IS_ERR(kthr))
			pr_info("[%s] create kthread imgsys-qof-dbg failed\n", __func__);
		else
			dbg_en = true;
	} else if (dbg_en) {
		kthread_stop(kthr);
		dbg_en = false;
	}
}

int mtk_imgsys_qof_ctrl(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = sscanf(val, "%u %u %u", &g_qof_debug_level, &g_qof_ver, &g_ftrace_time);
	QOF_LOGI("g_qof_debug_level[%u], force ver:g_qof_ver[%u], g_ftrace_time[%u]\n",
		g_qof_debug_level,
		g_qof_ver,
		g_ftrace_time);
	if (ret <= 0) {
		QOF_LOGE("sscanf ret is wrong %d\n", ret);
		return 0;
	}
	if (g_qof_debug_level == QOF_DEBUG_MODE_IMMEDIATE_DUMP)
		mtk_imgsys_cmdq_qof_dump(0, false);
	else if (g_qof_debug_level == QOF_DEBUG_MODE_IMMEDIATE_CG_DUMP)
		mtk_imgsys_cmdq_qof_dump(0, true);

	return 0;
}

static const struct kernel_param_ops qof_ctrl_ops = {
	.set = mtk_imgsys_qof_ctrl,
};

module_param_cb(imgsys_qof_ctrl, &qof_ctrl_ops, NULL, 0644);
MODULE_PARM_DESC(imgsys_qof_ctrl, "imgsys_qof_ctrl");
