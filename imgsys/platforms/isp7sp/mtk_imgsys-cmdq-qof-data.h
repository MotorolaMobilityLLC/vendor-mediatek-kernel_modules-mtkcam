/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 MediaTek Inc.
 *
 * Author: Nick.wen <nick.wen@mediatek.com>
 *
 */

#include "mtk_imgsys-cmdq-qof.h"

/**
 * @brief List all Qof related data.
 * 		  such as larb golden.
 */


/* Module group */
#define ISP_DIP_MODULES ( \
		BIT(IMGSYS_MOD_IMGMAIN) | \
		BIT(IMGSYS_MOD_WPE)  | \
		BIT(IMGSYS_MOD_DIP)  | \
		BIT(IMGSYS_MOD_PQDIP)| \
		BIT(IMGSYS_MOD_LTRAW))

#define ISP_TRAW_MODULES ( \
		BIT(IMGSYS_MOD_TRAW))

#define IMG_TRAW_PWR_CON		(0x1EC00000+0x3EE44)
#define IMG_ISP_DIP_PWR_CON		(0x1EC00000+0x3EE40)
#define IMG_CCF_MTCMOS_SET_STA	(0x1ec3b46c)
#define IMG_CCF_MTCMOS_CLR_STA	(0x1ec3b470)

/* CG macro */
#define IMG_CG_IMGSYS_MAIN 	(0x15000000)
#define IMG_CG_DIP_TOP_DIP1 (0x15110000)
#define IMG_CG_DIP_NR1_DIP1 (0x15130000)
#define IMG_CG_DIP_NR2_DIP1 (0x15170000)
#define IMG_CG_TRAW_DIP1 	(0x15710000)
#define IMG_CG_TRAW_CAP_DIP (0x15740000)
#define IMG_CG_WPE1_DIP1 	(0x15220000)
#define IMG_CG_WPE2_DIP1 	(0x15520000)
#define IMG_CG_WPE3_DIP1 	(0x15620000)

/* Larb reg base */
#define IMG_LARB10_BASE (0x15120000)
#define IMG_LARB11_BASE (0x15230000)
#define IMG_LARB15_BASE (0x15140000)
#define IMG_LARB22_BASE (0x15530000)
#define IMG_LARB23_BASE (0x15630000)
#define IMG_LARB38_BASE (0x15190000)
#define IMG_LARB39_BASE (0x15180000)
#define IMG_LARB28_BASE (0x15720000)
#define IMG_LARB40_BASE (0x15730000)


struct imgsys_cg_data dip_cg_data = {
	.clr_ofs = 0x8,
	.sta_ofs = 0
};

struct imgsys_cg_data traw_cg_data = {
	.clr_ofs = 0x8,
	.sta_ofs = 0
};

/* Larb golden */
struct qof_reg_data qof_larb10_golden[] = {
	{0x24,	0x370256},
	{0x524,	0x370256},
	{0x40,	0x1},
	{0x70,	0xffffffff},
	{0x200,	0x2b},
	{0x204,	0x9},
	{0x208,	0x5},
	{0x20c,	0x4},
	{0x210,	0x39},
	{0x214,	0x2b},
	{0x218,	0x1d},
	{0x21c,	0x2b},
	{0x220,	0x3},
	{0x224,	0x1}
};

struct qof_reg_data qof_larb11_golden[] = {
	{0x24,	0x370256},
	{0x524,	0x370256},
	{0x40,	0x1},
	{0x70,	0xffffffff},
	{0x200,	0x8},
	{0x204,	0x16},
	{0x208,	0x16},
	{0x20c,	0x1e},
	{0x210,	0xe},
	{0x214,	0x1},
	{0x218,	0x1},
	{0x21c,	0x1},
	{0x220,	0x10},
	{0x224,	0x16},
	{0x228,	0x2}
};

struct qof_reg_data qof_larb15_golden[] = {
	{0x24,	0x370256},
	{0x524,	0x370256},
	{0x40,	0x1},
	{0x70,	0xffffffff},
	{0x200,	0x24},
	{0x204,	0x19},
	{0x208,	0x1},
	{0x20c,	0x2b},
	{0x210,	0xa},
	{0x214,	0x25},
	{0x218,	0x1}
};

struct qof_reg_data qof_larb22_golden[] = {
	{0x24,	0x370256},
	{0x524,	0x370256},
	{0x40,	0x1},
	{0x70,	0xffffffff},
	{0x200,	0x8},
	{0x204,	0x16},
	{0x208,	0x16},
	{0x20c,	0x1e},
	{0x210,	0xe},
	{0x214,	0x1},
	{0x218,	0x1},
	{0x21c,	0x1},
	{0x220,	0x10},
	{0x224,	0x16},
	{0x228,	0x2}
};

struct qof_reg_data qof_larb23_golden[] = {
	{0x24,	0x370256},
	{0x524,	0x370256},
	{0x40,	0x1},
	{0x70,	0xffffffff},
	{0x200,	0x8},
	{0x204,	0x16},
	{0x208,	0x16},
	{0x20c,	0x1e},
	{0x210,	0xe},
	{0x214,	0x1},
	{0x218,	0x1},
	{0x21c,	0x1},
	{0x220,	0x10},
	{0x224,	0x16},
	{0x228,	0x2}
};

struct qof_reg_data qof_larb38_golden[] = {
	{0x24,	0x370256},
	{0x524,	0x370256},
	{0x40,	0x1},
	{0x70,	0xffffffff},
	{0x200,	0x1},
	{0x204,	0x1b},
	{0x208,	0xa},
	{0x20c,	0x7},
	{0x210,	0x4},
	{0x214,	0x4},
	{0x218,	0x1},
	{0x21c,	0x1},
	{0x220,	0x1},
	{0x224,	0x18},
	{0x228,	0x7},
	{0x22c,	0x4},
	{0x230,	0x2},
	{0x234,	0x2}
};

struct qof_reg_data qof_larb39_golden[] = {
	{0x24,	0x370256},
	{0x524,	0x370256},
	{0x40,	0x1},
	{0x70,	0xffffffff},
	{0x200,	0x1},
	{0x204,	0x1},
	{0x208,	0xa},
	{0x20c,	0xc},
	{0x210,	0x6},
	{0x214,	0xe},
	{0x218,	0xe},
	{0x21c,	0x6},
	{0x220,	0x6},
	{0x224,	0x40},
	{0x228,	0x40},
	{0x22c,	0x2},
	{0x230,	0x2},
	{0x234,	0x1},
	{0x238,	0x1},
	{0x23c,	0x4},
	{0x240,	0xe},
	{0x244,	0xe}
};

struct qof_reg_data qof_larb28_golden[] = {
	{0x24,	0x370256},
	{0x524,	0x370256},
	{0x40,	0x1},
	{0x70,	0xffffffff},
	{0x200,	0x1d},
	{0x204,	0x16},
	{0x208,	0x16},
	{0x20c,	0x5},
	{0x210,	0x2b},
	{0x214,	0x1b},
	{0x218,	0x1e},
	{0x21c,	0x2f},
	{0x220,	0x1}
};

struct qof_reg_data qof_larb40_golden[] = {
	{0x24,	0x370256},
	{0x524,	0x370256},
	{0x40,	0x1},
	{0x70,	0xffffffff},
	{0x200,	0x9},
	{0x204,	0x7},
	{0x208,	0x4},
	{0x20c,	0x5},
	{0x210,	0x7},
	{0x214,	0x25},
	{0x218,	0x13},
	{0x21c,	0x1},
	{0x220,	0x1},
	{0x224,	0x1},
	{0x228,	0x1},
	{0x22c,	0x7},
	{0x230,	0x9},
	{0x234,	0x7},
	{0x238,	0xb}
};

/* Larb data */
struct qof_larb_info qof_larb10_info = {
	.reg_ba = IMG_LARB10_BASE,
	.reg_list_size = sizeof(qof_larb10_golden)/sizeof(struct qof_reg_data),
	.larb_reg_list = qof_larb10_golden,
};

struct qof_larb_info qof_larb11_info = {
	.reg_ba = IMG_LARB11_BASE,
	.reg_list_size = sizeof(qof_larb11_golden)/sizeof(struct qof_reg_data),
	.larb_reg_list = qof_larb11_golden,
};

struct qof_larb_info qof_larb15_info = {
	.reg_ba = IMG_LARB15_BASE,
	.reg_list_size = sizeof(qof_larb15_golden)/sizeof(struct qof_reg_data),
	.larb_reg_list = qof_larb15_golden,
};

struct qof_larb_info qof_larb22_info = {
	.reg_ba = IMG_LARB22_BASE,
	.reg_list_size = sizeof(qof_larb22_golden)/sizeof(struct qof_reg_data),
	.larb_reg_list = qof_larb22_golden,
};

struct qof_larb_info qof_larb23_info = {
	.reg_ba = IMG_LARB23_BASE,
	.reg_list_size = sizeof(qof_larb23_golden)/sizeof(struct qof_reg_data),
	.larb_reg_list = qof_larb23_golden,
};

struct qof_larb_info qof_larb38_info = {
	.reg_ba = IMG_LARB38_BASE,
	.reg_list_size = sizeof(qof_larb38_golden)/sizeof(struct qof_reg_data),
	.larb_reg_list = qof_larb38_golden,
};

struct qof_larb_info qof_larb39_info = {
	.reg_ba = IMG_LARB39_BASE,
	.reg_list_size = sizeof(qof_larb39_golden)/sizeof(struct qof_reg_data),
	.larb_reg_list = qof_larb39_golden,
};

struct qof_larb_info qof_larb28_info = {
	.reg_ba = IMG_LARB28_BASE,
	.reg_list_size = sizeof(qof_larb28_golden)/sizeof(struct qof_reg_data),
	.larb_reg_list = qof_larb28_golden,
};

struct qof_larb_info qof_larb40_info = {
	.reg_ba = IMG_LARB40_BASE,
	.reg_list_size = sizeof(qof_larb40_golden)/sizeof(struct qof_reg_data),
	.larb_reg_list = qof_larb40_golden,
};