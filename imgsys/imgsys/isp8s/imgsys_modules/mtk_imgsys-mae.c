// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 MediaTek Inc.
 *
 * Author: Ming-Hsuan Chiang <ming-hsuan.chiang@mediatek.com>
 *
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/of_address.h>
#include "mtk_imgsys-mae.h"
#include "mtk_imgsys-v4l2-debug.h"
#include "mtk-hcp.h"

/*******************************************************************************
 * Global Define
 *******************************************************************************/
static void __iomem *g_maeRegBA;

#define MAE_BASE  0x34320000
// 0x1000 MAE_CTRL_CENTER
#define MAE_CTRL_CENTER_BASE              (0x1000)
#define MAE_CTRL_CENTER_LEN               (0x308)

// 0x4000 MAE_COSSIM
#define MAE_COSSIM_BASE                   (0x4000)
#define MAE_COSSIM_LEN                    (0x000C)

// 0x4200 MAE_MAISR
#define MAE_MAISR_BASE                    (0x4200)
#define MAE_MAISR_LEN                     (0x0600)

// 0x4800 MAE_RSZ0
#define MAE_RSZ0_BASE                     (0x4800)
#define MAE_RSZ0_LEN                      (0x01F4)

// 0x4A00 MAE_CMP_2
#define MAE_CMP_2_BASE                    (0x4A00)
#define MAE_CMP_2_LEN                     (0x01F4)

// 0x4C00 MAE_UDMA_W
#define MAE_UDMA_W_BASE                   (0x4C00)
#define MAE_UDMA_W_LEN                    (0x01FC)

// 0x4E00 MAE_WDMA
#define MAE_WDMA_BASE                     (0x4E00)
#define MAE_WDMA_LEN                      (0x01F8)

// 0x5000 MAE_CMP
#define MAE_CMP_BASE                      (0x5000)
#define MAE_CMP_LEN                       (0x0190)

// 0x5200 MAE_UDMA_R
#define MAE_UDMA_R_BASE                   (0x5200)
#define MAE_UDMA_R_LEN                    (0x600)

// 0x5800 MAE_RDMA_5
#define MAE_RDMA_5_BASE                   (0x5800)
#define MAE_RDMA_5_LEN                    (0x1FC)

// 0x5A00 MAE_RDMA_F
#define MAE_RDMA_F_BASE                   (0x5C00)
#define MAE_RDMA_F_LEN                    (0x6C)

// 0x5C00 MAE_CMP_R
#define MAE_CMP_R_BASE                    (0x5C00)
#define MAE_CMP_R_LEN                     (0x1FC)

// 0x5E00 MAE_CTRL_CENTER
#define MAE_5E00_BASE                     (0x5E00)
#define MAE_5E00_LEN                      (0xFC)

// 0X6000 RSZ1
#define RSZ1_BASE                         (0x6000)
#define RSZ1_LEN                          (0x1FC)

// 0X6200 RSZ2
#define RSZ2_BASE                         (0x6200)
#define RSZ2_LEN                          (0x1FC)

// 0X6400 RSZ3
#define RSZ3_BASE                         (0x6400)
#define RSZ3_LEN                          (0x1FC)

// 0X6600 MAE_nve_top_sys
#define MAE_nve_top_sys_BASE              (0x6600)
#define MAE_nve_top_sys_LEN               (0x10C)

// 0X6800 MAE_cossim_apb
#define MAE_cossim_apb_BASE               (0x6800)
#define MAE_cossim_apb_LEN                (0x4)

// 0X6A00 MAISR_APB_BASE
#define MAE_MAISR_APB_BASE                (0x6A00)
#define MAE_MAISR_APB_LEN                 (0x1FC)

// 0X6C00 MAE_POST0
#define MAE_POST0_APB_BASE                (0x6C00)
#define MAE_POST0_APB_LEN                 (0x11C)

// 0X6E00 MAE_POST1
#define MAE_POST1_APB_BASE                (0x6E00)
#define MAE_POST1_APB_LEN                 (0x1FC)

// 0x7000 MAE_MMFD_POST
#define MAE_MMFD_POST_BASE                (0x7000)
#define MAE_MMFD_POST_LEN                 (0x0160)

// 0x7200 MAE_DRV_2
#define MAE_DRV_2_BASE                    (0x7200)
#define MAE_DRV_2_LEN                     (0x01D0)

// 0x7400 MAE_DRV
#define MAE_DRV_W_BASE                    (0x7400)
#define MAE_DRV_W_LEN                     (0x01CC)

// 0x7600 MAE_DRV_R
#define MAE_DRV_R_BASE                    (0x7600)
#define MAE_DRV_R_LEN                     (0x01FC)

// 0x7800 MAE_DRV_R_2
#define MAE_DRV_R_2_BASE                  (0x7800)
#define MAE_DRV_R_2_LEN                   (0x01FC)

#define MAE_maisr_apb_reg_0170_maisr_apb  (0x6b70)
#define MAE_DRV_R_reg_0178_MAE_DRV_R      (0x7778)
#define MAE_DRV_R_reg_017C_MAE_DRV_R      (0x777C)
#define MAE_UDMA_W_Base_reg_01C0_UDMA_W   (0x4DC0)
#define MAE_UDMA_R_Base_reg_01C0_UDMA_R   (0x53C0)
#define MAE_CTRL_CENTER_reg_000C_CTRL_CENTER   (0x5E0C)
#define MAE_nve_top_sys_reg_00E8_nve_top_sys   (0x66FC)
#define MAE_WDMA_reg_0190_MAE_WDMA        (0x4F90)
#define MAE_DRV_reg_01C0_MAE_DRV          (0x75C0)
#define MAE_DRV_reg_01C4_MAE_DRV          (0x75C4)
#define MAE_UDMA_W_Base_reg_0064_UDMA_W_Base   (0x4C64)
#define MAE_UDMA_W_Base_reg_0068_UDMA_W_Base   (0x4C68)
#define MAE_dma_sys_MAE_STATUS            (0x1300)

// module param
int g_imgsys_debug_opt;
module_param(g_imgsys_debug_opt, int, 0644);

/*******************************************************************************
* static Functions
*******************************************************************************/
bool get_imgsys_debug_opt(void)
{
	return g_imgsys_debug_opt;
}

void mae_reg_dump(void)
{
	void __iomem *maeRegBA = 0L;
	int i = 0;
	unsigned int val;

	/* iomap registers */
	maeRegBA = g_maeRegBA;
	if (!maeRegBA) {
		pr_info("%s Unable to ioremap MAE registers\n",
			__func__);
	}

	// reset driver domain for read back compiler domain reg
	val = (unsigned int)ioread32((void *)(maeRegBA + MAE_maisr_apb_reg_0170_maisr_apb));
	iowrite32(val | (0x1 << 3), (void *)(maeRegBA + MAE_maisr_apb_reg_0170_maisr_apb));
	iowrite32(val & ~(0x1 << 3), (void *)(maeRegBA + MAE_maisr_apb_reg_0170_maisr_apb));

	val = (unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_R_reg_0178_MAE_DRV_R));
	iowrite32(val & (0xffffffe0),
		(void *)(maeRegBA + MAE_DRV_R_reg_0178_MAE_DRV_R));
	pr_info("%s: [0x%08X] %08X, sel = 0, [0x%08X] %08X", __func__,
		(unsigned int)(MAE_BASE + MAE_DRV_R_reg_0178_MAE_DRV_R),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_R_reg_0178_MAE_DRV_R)),
		(unsigned int)(MAE_BASE + MAE_DRV_R_reg_017C_MAE_DRV_R),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_R_reg_017C_MAE_DRV_R)));

	val = (unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_R_reg_0178_MAE_DRV_R));
	iowrite32((val & (0xffffffe0)) + 1,
		(void *)(maeRegBA + MAE_DRV_R_reg_0178_MAE_DRV_R));
	pr_info("%s: [0x%08X] %08X, sel = 1, [0x%08X] %08X", __func__,
		(unsigned int)(MAE_BASE + MAE_DRV_R_reg_0178_MAE_DRV_R),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_R_reg_0178_MAE_DRV_R)),
		(unsigned int)(MAE_BASE + MAE_DRV_R_reg_017C_MAE_DRV_R),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_R_reg_017C_MAE_DRV_R)));

	val = (unsigned int)ioread32((void *)(maeRegBA + MAE_UDMA_W_Base_reg_01C0_UDMA_W));
	iowrite32((val & (0xfffffffc)) + 0x1,
		(void *)(maeRegBA + MAE_UDMA_W_Base_reg_01C0_UDMA_W));
	pr_info("%s: [0x%08X] %08X", __func__,
		(unsigned int)(MAE_BASE + MAE_UDMA_W_Base_reg_01C0_UDMA_W),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_UDMA_W_Base_reg_01C0_UDMA_W)));

	val = (unsigned int)ioread32((void *)(maeRegBA + MAE_UDMA_R_Base_reg_01C0_UDMA_R));
	iowrite32((val & (0xfffffffc)) + 0x1,
		(void *)(maeRegBA + MAE_UDMA_R_Base_reg_01C0_UDMA_R));
	pr_info("%s: [0x%08X] %08X", __func__,
		(unsigned int)(MAE_BASE + MAE_UDMA_R_Base_reg_01C0_UDMA_R),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_UDMA_R_Base_reg_01C0_UDMA_R)));


	val = (unsigned int)ioread32((void *)(maeRegBA + MAE_dma_sys_MAE_STATUS));
	iowrite32(0x8000 + (val & 0x00ff),
		(void *)(maeRegBA + MAE_CTRL_CENTER_reg_000C_CTRL_CENTER));
	pr_info("%s: [0x%08X] %08X", __func__,
		(unsigned int)(MAE_BASE + MAE_CTRL_CENTER_reg_000C_CTRL_CENTER),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_CTRL_CENTER_reg_000C_CTRL_CENTER)));

	val = (unsigned int)ioread32((void *)(maeRegBA + MAE_nve_top_sys_reg_00E8_nve_top_sys));
	iowrite32((val & (0xfffffffe)) + 0x1,
		(void *)(maeRegBA + MAE_nve_top_sys_reg_00E8_nve_top_sys));
	pr_info("%s: [0x%08X] %08X", __func__,
		(unsigned int)(MAE_BASE + MAE_nve_top_sys_reg_00E8_nve_top_sys),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_nve_top_sys_reg_00E8_nve_top_sys)));

	val = (unsigned int)ioread32((void *)(maeRegBA + MAE_WDMA_reg_0190_MAE_WDMA));
	iowrite32((val & (0xfffffffe)) + 0x1,
		(void *)(maeRegBA + MAE_WDMA_reg_0190_MAE_WDMA));
	pr_info("%s: [0x%08X] %08X", __func__,
		(unsigned int)(MAE_BASE + MAE_WDMA_reg_0190_MAE_WDMA),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_WDMA_reg_0190_MAE_WDMA)));

	val = (unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_reg_01C0_MAE_DRV));
	iowrite32((val & (0xffffffc0)) + 0x0,
		(void *)(maeRegBA + MAE_DRV_reg_01C0_MAE_DRV));
	pr_info("%s: [0x%08X] %08X, sel = 0, [0x%08X] %08X", __func__,
		(unsigned int)(MAE_BASE + MAE_DRV_reg_01C0_MAE_DRV),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_reg_01C0_MAE_DRV)),
		(unsigned int)(MAE_BASE + MAE_DRV_reg_01C4_MAE_DRV),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_reg_01C4_MAE_DRV)));

	val = (unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_reg_01C0_MAE_DRV));
	iowrite32((val & (0xffffffc0)) + 0x1,
		(void *)(maeRegBA + MAE_DRV_reg_01C0_MAE_DRV));
	pr_info("%s: [0x%08X] %08X, sel = 0, [0x%08X] %08X", __func__,
		(unsigned int)(MAE_BASE + MAE_DRV_reg_01C0_MAE_DRV),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_reg_01C0_MAE_DRV)),
		(unsigned int)(MAE_BASE + MAE_DRV_reg_01C4_MAE_DRV),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_reg_01C4_MAE_DRV)));

	val = (unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_reg_01C0_MAE_DRV));
	iowrite32((val & (0xffffffc0)) + 0x10,
		(void *)(maeRegBA + MAE_DRV_reg_01C0_MAE_DRV));
	pr_info("%s: [0x%08X] %08X, sel = 0, [0x%08X] %08X", __func__,
		(unsigned int)(MAE_BASE + MAE_DRV_reg_01C0_MAE_DRV),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_reg_01C0_MAE_DRV)),
		(unsigned int)(MAE_BASE + MAE_DRV_reg_01C4_MAE_DRV),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_reg_01C4_MAE_DRV)));

	val = (unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_reg_01C0_MAE_DRV));
	iowrite32((val & (0xffffffc0)) + 0x20,
		(void *)(maeRegBA + MAE_DRV_reg_01C0_MAE_DRV));
	pr_info("%s: [0x%08X] %08X, sel = 0, [0x%08X] %08X", __func__,
		(unsigned int)(MAE_BASE + MAE_DRV_reg_01C0_MAE_DRV),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_reg_01C0_MAE_DRV)),
		(unsigned int)(MAE_BASE + MAE_DRV_reg_01C4_MAE_DRV),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_DRV_reg_01C4_MAE_DRV)));

	// checksum
	pr_info("%s: checksum [0x%08X] %08X, [0x%08X] %08X", __func__,
		(unsigned int)(MAE_BASE + MAE_UDMA_W_Base_reg_0064_UDMA_W_Base),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_UDMA_W_Base_reg_0064_UDMA_W_Base)),
		(unsigned int)(MAE_BASE + MAE_UDMA_W_Base_reg_0068_UDMA_W_Base),
		(unsigned int)ioread32((void *)(maeRegBA + MAE_UDMA_W_Base_reg_0068_UDMA_W_Base)));

	// 0x1000 MAE_CTRL_CENTER
	for (i = MAE_CTRL_CENTER_BASE; i <= MAE_CTRL_CENTER_BASE + MAE_CTRL_CENTER_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x4000 MAE_COSSIM
	for (i = MAE_COSSIM_BASE; i <= MAE_COSSIM_BASE + MAE_COSSIM_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x4200 MAE_RSZ0
	for (i = MAE_MAISR_BASE; i <= MAE_MAISR_BASE + MAE_MAISR_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x4800 MAE_RSZ0
	for (i = MAE_RSZ0_BASE; i <= MAE_RSZ0_BASE + MAE_RSZ0_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x4A00 MAE_CMP_2
	for (i = MAE_CMP_2_BASE; i <= MAE_CMP_2_BASE + MAE_CMP_2_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x4C00 MAE_UDMA_W
	for (i = MAE_UDMA_W_BASE; i <= MAE_UDMA_W_BASE + MAE_UDMA_W_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x4E00 MAE_WDMA_BASE
	for (i = MAE_WDMA_BASE; i <= MAE_WDMA_BASE + MAE_WDMA_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x5000 MAE_CMP
	for (i = MAE_CMP_BASE; i <= MAE_CMP_BASE + MAE_CMP_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x5200 MAE_UDMA_R
	for (i = MAE_UDMA_R_BASE; i <= MAE_UDMA_R_BASE + MAE_UDMA_R_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x5800 MAE_RDMA_5
	for (i = MAE_RDMA_5_BASE; i <= MAE_RDMA_5_BASE + MAE_RDMA_5_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x5A00 MAE_RDMA_F
	for (i = MAE_RDMA_F_BASE; i <= MAE_RDMA_F_BASE + MAE_RDMA_F_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x5C00 MAE_CMP_R
	for (i = MAE_CMP_R_BASE; i <= MAE_CMP_R_BASE + MAE_CMP_R_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x5E00 MAE_CTRL_CENTER
	for (i = MAE_5E00_BASE; i <= MAE_5E00_BASE + MAE_5E00_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0X6000 RSZ1
	for (i = RSZ1_BASE; i <= RSZ1_BASE + RSZ1_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0X6200 RSZ2
	for (i = RSZ2_BASE; i <= RSZ2_BASE + RSZ2_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0X6400 RSZ3
	for (i = RSZ3_BASE; i <= RSZ3_BASE + RSZ3_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0X6600 MAE_nve_top_sys
	for (i = MAE_nve_top_sys_BASE; i <= MAE_nve_top_sys_BASE + MAE_nve_top_sys_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0X6800 MAE_cossim_apb
	for (i = MAE_cossim_apb_BASE; i <= MAE_cossim_apb_BASE + MAE_cossim_apb_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0X6A00 MAISR_APB_BASE
	for (i = MAE_MAISR_APB_BASE; i <= MAE_MAISR_APB_BASE + MAE_MAISR_APB_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0X6C00 MAE_POST0_APB_BASE
	for (i = MAE_POST0_APB_BASE; i <= MAE_POST0_APB_BASE + MAE_POST0_APB_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0X6E00 MAE_POST1_APB_BASE
	for (i = MAE_POST1_APB_BASE; i <= MAE_POST1_APB_BASE + MAE_POST1_APB_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x7000 MAE_MMFD_POST
	for (i = MAE_MMFD_POST_BASE; i <= MAE_MMFD_POST_BASE + MAE_MMFD_POST_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x7200 MAE_DRV_2
	for (i = MAE_DRV_2_BASE; i <= MAE_DRV_2_BASE + MAE_DRV_2_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x7400 MAE_DRV
	for (i = MAE_DRV_W_BASE; i <= MAE_DRV_W_BASE + MAE_DRV_W_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x7600 MAE_DRV_R
	for (i = MAE_DRV_R_BASE; i <= MAE_DRV_R_BASE + MAE_DRV_R_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}

	// 0x7800 MAE_DRV_R_2
	for (i = MAE_DRV_R_2_BASE; i <= MAE_DRV_R_2_BASE + MAE_DRV_R_2_LEN; i += 0x10) {
		pr_info("%s: [0x%08X] %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(MAE_BASE + i),
			(unsigned int)ioread32((void *)(maeRegBA + i)),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x4))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0x8))),
			(unsigned int)ioread32((void *)(maeRegBA + (i+0xC))));
	}
}

/*******************************************************************************
 * Functions
 *******************************************************************************/
int MAE_TranslationFault_callback(int port, dma_addr_t mva, void *data)
{
	pr_info("%s: dump mae regs\n", __func__);
	mae_reg_dump();

	return 1;
}

void imgsys_mae_init(struct mtk_imgsys_dev *imgsys_dev)
{
	pr_info("%s: +\n", __func__);

	g_maeRegBA = of_iomap(imgsys_dev->dev->of_node, REG_MAP_E_MAE);
}

void imgsys_mae_set(struct mtk_imgsys_dev *imgsys_dev)
{
#ifdef MAE_SET_INIT_VALUE
	void __iomem *maeRegBA = 0L;

	/* iomap registers */
	maeRegBA = g_maeRegBA;
	if (!maeRegBA) {
		pr_info("%s Unable to ioremap MAE registers\n",
			__func__);
	}
#endif

	pr_info("%s: +\n", __func__);

#ifdef MAE_SET_INIT_VALUE
	// set default setting if need
	iowrite32(0x00002000, (void *)(maeRegBA + 0x00001010));
#endif
}

void imgsys_mae_debug_dump(struct mtk_imgsys_dev *imgsys_dev,
			unsigned int engine)
{
	dev_info(imgsys_dev->dev, "%s: dump mae regs\n", __func__);
	mae_reg_dump();
}

bool imgsys_mae_done_chk(struct mtk_imgsys_dev *imgsys_dev, uint32_t engine)
{
	bool ret = true; //true: done

	uint32_t value = 0;

	value = (uint32_t)ioread32((void *)(g_maeRegBA + 0x1018));
	if (!(value & 0x1)) {
		ret = false;
		pr_info(
		"%s: hw_comb:0x%x, polling MAE done fail!!! [0x%08x] 0x%x",
		__func__, engine,
		(uint32_t)(MAE_BASE + 0x1018), value);
	}

	return ret;
}

void imgsys_mae_uninit(struct mtk_imgsys_dev *imgsys_dev)
{
	pr_info("%s: +\n", __func__);

	if (g_maeRegBA) {
		iounmap(g_maeRegBA);
		g_maeRegBA = 0L;
	}
}
