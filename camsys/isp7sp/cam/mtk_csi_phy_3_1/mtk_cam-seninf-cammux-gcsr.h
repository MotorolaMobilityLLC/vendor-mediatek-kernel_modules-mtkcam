/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __SENINF_CAM_MUX_GCSR_C_HEADER_H__
#define __SENINF_CAM_MUX_GCSR_C_HEADER_H__

#define SENINF_CAM_MUX_GCSR_CTRL 0x0000
#define RG_SENINF_CAM_MUX_SW_RST_SHIFT 0
#define RG_SENINF_CAM_MUX_SW_RST_MASK (0x1 << 0)
#define RG_SENINF_CAM_MUX_IRQ_SW_RST_SHIFT 1
#define RG_SENINF_CAM_MUX_IRQ_SW_RST_MASK (0x1 << 1)
#define RG_SENINF_CAM_MUX_SLICE_FULL_OPT_SHIFT 7
#define RG_SENINF_CAM_MUX_SLICE_FULL_OPT_MASK (0x1 << 7)
#define RG_SENINF_CAM_MUX_IRQ_CLR_MODE_SHIFT 8
#define RG_SENINF_CAM_MUX_IRQ_CLR_MODE_MASK (0x1 << 8)
#define RG_SENINF_CAM_MUX_IRQ_VERIF_EN_SHIFT 9
#define RG_SENINF_CAM_MUX_IRQ_VERIF_EN_MASK (0x1 << 9)

#define SENINF_CAM_MUX_GCSR_DYN_CTRL 0x0004
#define RG_SENINF_CAM_MUX_DYN_SWITCH_BSY0_SHIFT 0
#define RG_SENINF_CAM_MUX_DYN_SWITCH_BSY0_MASK (0x1 << 0)
#define RG_SENINF_CAM_MUX_DYN_SWITCH_BSY1_SHIFT 1
#define RG_SENINF_CAM_MUX_DYN_SWITCH_BSY1_MASK (0x1 << 1)
#define RG_SENINF_CAM_MUX_DYN_PAGE_SEL_SHIFT 7
#define RG_SENINF_CAM_MUX_DYN_PAGE_SEL_MASK (0x1 << 7)
#define RG_SENINF_CAM_MUX_OCC_ALL_EN_SHIFT 8
#define RG_SENINF_CAM_MUX_OCC_ALL_EN_MASK (0x1 << 8)
#define RG_SENINF_CAM_MUX_DYN_SKIP_CURR_EN_SHIFT 9
#define RG_SENINF_CAM_MUX_DYN_SKIP_CURR_EN_MASK (0x1 << 9)
#define RO_SENINF_CAM_MUX_DYN_SAT_SWITCH_BSY0_SHIFT 12
#define RO_SENINF_CAM_MUX_DYN_SAT_SWITCH_BSY0_MASK (0x1 << 12)
#define RO_SENINF_CAM_MUX_DYN_SAT_SWITCH_BSY1_SHIFT 13
#define RO_SENINF_CAM_MUX_DYN_SAT_SWITCH_BSY1_MASK (0x1 << 13)
#define RG_SENINF_CAM_MUX_DYN_SAT_SWITCH_EN_SHIFT 14
#define RG_SENINF_CAM_MUX_DYN_SAT_SWITCH_EN_MASK (0x3 << 14)
#define RG_SENINF_CAM_MUX_TIMER_SEL0_SHIFT 16
#define RG_SENINF_CAM_MUX_TIMER_SEL0_MASK (0x1 << 16)
#define RG_SENINF_CAM_MUX_TIMER_SEL1_SHIFT 17
#define RG_SENINF_CAM_MUX_TIMER_SEL1_MASK (0x1 << 17)
#define RG_SENINF_CAM_MUX_TIMER_INIT_SHIFT 18
#define RG_SENINF_CAM_MUX_TIMER_INIT_MASK (0x3 << 18)

#define SENINF_CAM_MUX_GCSR_IRQ_EN 0x0008
#define RG_SENINF_CAM_MUX_ALL_VSYNC_IRQ_EN_SHIFT 0
#define RG_SENINF_CAM_MUX_ALL_VSYNC_IRQ_EN_MASK (0x3 << 0)
#define RG_SENINF_CAM_MUX_ALL_VSYNC_NONE_IRQ_EN_SHIFT 4
#define RG_SENINF_CAM_MUX_ALL_VSYNC_NONE_IRQ_EN_MASK (0x3 << 4)
#define RG_SENINF_CAM_MUX_SKIP_NEXT_FRAME_IRQ_EN_SHIFT 8
#define RG_SENINF_CAM_MUX_SKIP_NEXT_FRAME_IRQ_EN_MASK (0x3 << 8)

#define SENINF_CAM_MUX_GCSR_IRQ_STS 0x000c
#define RO_SENINF_CAM_MUX_ALL_VSYNC_IRQ_SHIFT 0
#define RO_SENINF_CAM_MUX_ALL_VSYNC_IRQ_MASK (0x3 << 0)
#define RO_SENINF_CAM_MUX_ALL_VSYNC_NONE_IRQ_SHIFT 4
#define RO_SENINF_CAM_MUX_ALL_VSYNC_NONE_IRQ_MASK (0x3 << 4)
#define RO_SENINF_CAM_MUX_SKIP_NEXT_FRAME_IRQ_SHIFT 8
#define RO_SENINF_CAM_MUX_SKIP_NEXT_FRAME_IRQ_MASK (0x3 << 8)

#define SENINF_CAM_MUX_GCSR_IRQ_TRIG 0x0010
#define RG_SENINF_CAM_MUX_GCSR_IRQ_TRIG_SHIFT 0
#define RG_SENINF_CAM_MUX_GCSR_IRQ_TRIG_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_VSYNC_T0_WINDOW_L 0x0018
#define RG_SENINF_CAM_MUX_VSYNC_TIMER0_WINDOW_L_SHIFT 0
#define RG_SENINF_CAM_MUX_VSYNC_TIMER0_WINDOW_L_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_VSYNC_T0_WINDOW_H 0x001c
#define RG_SENINF_CAM_MUX_VSYNC_TIMER0_WINDOW_H_SHIFT 0
#define RG_SENINF_CAM_MUX_VSYNC_TIMER0_WINDOW_H_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_VSYNC_T1_WINDOW_L 0x0020
#define RG_SENINF_CAM_MUX_VSYNC_TIMER1_WINDOW_L_SHIFT 0
#define RG_SENINF_CAM_MUX_VSYNC_TIMER1_WINDOW_L_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_VSYNC_T1_WINDOW_H 0x0024
#define RG_SENINF_CAM_MUX_VSYNC_TIMER1_WINDOW_H_SHIFT 0
#define RG_SENINF_CAM_MUX_VSYNC_TIMER1_WINDOW_H_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GSCR_AOV_SEC_LOCK0 0x0030
#define RG_CAM_MUX_AOV_SECURE_LOCK31_0_SHIFT 0
#define RG_CAM_MUX_AOV_SECURE_LOCK31_0_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GSCR_AOV_SEC_LOCK1 0x0034
#define RG_CAM_MUX_AOV_SECURE_LOCK51_32_SHIFT 0
#define RG_CAM_MUX_AOV_SECURE_LOCK51_32_MASK (0xfffff << 0)

#define SENINF_CAM_MUX_GSCR_AOV_SEC_EXC0_0 0x0038
#define RG_CAM_MUX_AOV_SECURE_EXCLUDE0_31_0_SHIFT 0
#define RG_CAM_MUX_AOV_SECURE_EXCLUDE0_31_0_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GSCR_AOV_SEC_EXC0_1 0x003c
#define RG_CAM_MUX_AOV_SECURE_EXCLUDE0_63_32_SHIFT 0
#define RG_CAM_MUX_AOV_SECURE_EXCLUDE0_63_32_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GSCR_AOV_SEC_EXC0_2 0x0040
#define RG_CAM_MUX_AOV_SECURE_EXCLUDE0_73_64_SHIFT 0
#define RG_CAM_MUX_AOV_SECURE_EXCLUDE0_73_64_MASK (0x3ff << 0)

#define SENINF_CAM_MUX_GSCR_AOV_SEC_EXC1_0 0x0044
#define RG_CAM_MUX_AOV_SECURE_EXCLUDE1_31_0_SHIFT 0
#define RG_CAM_MUX_AOV_SECURE_EXCLUDE1_31_0_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GSCR_AOV_SEC_EXC1_1 0x0048
#define RG_CAM_MUX_AOV_SECURE_EXCLUDE1_63_32_SHIFT 0
#define RG_CAM_MUX_AOV_SECURE_EXCLUDE1_63_32_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GSCR_AOV_SEC_EXC1_2 0x004c
#define RG_CAM_MUX_AOV_SECURE_EXCLUDE1_73_64_SHIFT 0
#define RG_CAM_MUX_AOV_SECURE_EXCLUDE1_73_64_MASK (0x3ff << 0)

#define SENINF_CAM_MUX_GSCR_AOV_MASK0 0x0050
#define RG_CAM_MUX_AOV_SECURE_VIO_MASK31_0_SHIFT 0
#define RG_CAM_MUX_AOV_SECURE_VIO_MASK31_0_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GSCR_AOV_MASK1 0x0054
#define RG_CAM_MUX_AOV_SECURE_VIO_MASK51_32_SHIFT 0
#define RG_CAM_MUX_AOV_SECURE_VIO_MASK51_32_MASK (0xfffff << 0)

#define SENINF_CAM_MUX_GCSR_MUX_EN 0x0080
#define RG_SENINF_CAM_MUX_EN_SHIFT 0
#define RG_SENINF_CAM_MUX_EN_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_MUX_EN_H 0x0084
#define RG_SENINF_CAM_MUX_EN_H_SHIFT 0
#define RG_SENINF_CAM_MUX_EN_H_MASK (0x7ff << 0)

#define SENINF_CAM_MUX_GCSR_CHK_EN 0x0088
#define RG_SENINF_CAM_MUX_CHK_EN_SHIFT 0
#define RG_SENINF_CAM_MUX_CHK_EN_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_CHK_EN_H 0x008c
#define RG_SENINF_CAM_MUX_CHK_EN_H_SHIFT 0
#define RG_SENINF_CAM_MUX_CHK_EN_H_MASK (0x7ff << 0)

#define SENINF_CAM_MUX_GCSR_DYN_EN0 0x0090
#define RG_SENINF_CAM_MUX_DYN_SWITCH_EN0_SHIFT 0
#define RG_SENINF_CAM_MUX_DYN_SWITCH_EN0_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_DYN_EN0_H 0x0094
#define RG_SENINF_CAM_MUX_DYN_SWITCH_EN0_H_SHIFT 0
#define RG_SENINF_CAM_MUX_DYN_SWITCH_EN0_H_MASK (0x7ff << 0)

#define SENINF_CAM_MUX_GCSR_DYN_EN1 0x0098
#define RG_SENINF_CAM_MUX_DYN_SWITCH_EN1_SHIFT 0
#define RG_SENINF_CAM_MUX_DYN_SWITCH_EN1_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_DYN_EN1_H 0x009c
#define RG_SENINF_CAM_MUX_DYN_SWITCH_EN1_H_SHIFT 0
#define RG_SENINF_CAM_MUX_DYN_SWITCH_EN1_H_MASK (0x7ff << 0)

#define SENINF_CAM_MUX_GCSR_VC_SEL_EN 0x00a0
#define RG_SENINF_CAM_MUX_VC_SEL_EN_SHIFT 0
#define RG_SENINF_CAM_MUX_VC_SEL_EN_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_VC_SEL_EN_H 0x00a4
#define RG_SENINF_CAM_MUX_VC_SEL_EN_H_SHIFT 0
#define RG_SENINF_CAM_MUX_VC_SEL_EN_H_MASK (0x7ff << 0)

#define SENINF_CAM_MUX_GCSR_DT_SEL_EN 0x00a8
#define RG_SENINF_CAM_MUX_DT_SEL_EN_SHIFT 0
#define RG_SENINF_CAM_MUX_DT_SEL_EN_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_DT_SEL_EN_H 0x00ac
#define RG_SENINF_CAM_MUX_DT_SEL_EN_H_SHIFT 0
#define RG_SENINF_CAM_MUX_DT_SEL_EN_H_MASK (0x7ff << 0)

#define SENINF_CAM_MUX_GCSR_HSIZE_ERR_IRQ_EN 0x00b0
#define RG_SENINF_CAM_MUX_HSIZE_ERR_IRQ_EN_SHIFT 0
#define RG_SENINF_CAM_MUX_HSIZE_ERR_IRQ_EN_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_HSIZE_ERR_IRQ_EN_H 0x00b4
#define RG_SENINF_CAM_MUX_HSIZE_ERR_IRQ_EN_H_SHIFT 0
#define RG_SENINF_CAM_MUX_HSIZE_ERR_IRQ_EN_H_MASK (0x7ff << 0)

#define SENINF_CAM_MUX_GCSR_VSIZE_ERR_IRQ_EN 0x00b8
#define RG_SENINF_CAM_MUX_VSIZE_ERR_IRQ_EN_SHIFT 0
#define RG_SENINF_CAM_MUX_VSIZE_ERR_IRQ_EN_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_VSIZE_ERR_IRQ_EN_H 0x00bc
#define RG_SENINF_CAM_MUX_VSIZE_ERR_IRQ_EN_H_SHIFT 0
#define RG_SENINF_CAM_MUX_VSIZE_ERR_IRQ_EN_H_MASK (0x7ff << 0)

#define SENINF_CAM_MUX_GCSR_VSYNC_IRQ_EN 0x00c0
#define RG_SENINF_CAM_MUX_VSYNC_IRQ_EN_SHIFT 0
#define RG_SENINF_CAM_MUX_VSYNC_IRQ_EN_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_VSYNC_IRQ_EN_H 0x00c4
#define RG_SENINF_CAM_MUX_VSYNC_IRQ_EN_H_SHIFT 0
#define RG_SENINF_CAM_MUX_VSYNC_IRQ_EN_H_MASK (0x7ff << 0)

#define SENINF_CAM_MUX_GCSR_HSIZE_ERR_IRQ_STS 0x00c8
#define RO_SENINF_CAM_MUX_HSIZE_ERR_IRQ_SHIFT 0
#define RO_SENINF_CAM_MUX_HSIZE_ERR_IRQ_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_HSIZE_ERR_IRQ_STS_H 0x00cc
#define RO_SENINF_CAM_MUX_HSIZE_ERR_IRQ_H_SHIFT 0
#define RO_SENINF_CAM_MUX_HSIZE_ERR_IRQ_H_MASK (0x7ff << 0)

#define SENINF_CAM_MUX_GCSR_VSIZE_ERR_IRQ_STS 0x00d0
#define RO_SENINF_CAM_MUX_VSIZE_ERR_IRQ_SHIFT 0
#define RO_SENINF_CAM_MUX_VSIZE_ERR_IRQ_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_VSIZE_ERR_IRQ_STS_H 0x00d4
#define RO_SENINF_CAM_MUX_VSIZE_ERR_IRQ_H_SHIFT 0
#define RO_SENINF_CAM_MUX_VSIZE_ERR_IRQ_H_MASK (0x7ff << 0)

#define SENINF_CAM_MUX_GCSR_VSYNC_IRQ_STS 0x00d8
#define RO_SENINF_CAM_MUX_VSYNC_IRQ_SHIFT 0
#define RO_SENINF_CAM_MUX_VSYNC_IRQ_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_VSYNC_IRQ_STS_H 0x00dc
#define RO_SENINF_CAM_MUX_VSYNC_IRQ_H_SHIFT 0
#define RO_SENINF_CAM_MUX_VSYNC_IRQ_H_MASK (0x7ff << 0)

#define SENINF_CAM_MUX_GCSR_DBG_CTRL 0x00f0
#define RG_SENINF_CAM_MUX_DBG_EN_SHIFT 0
#define RG_SENINF_CAM_MUX_DBG_EN_MASK (0x1 << 0)
#define RG_SENINF_CAM_MUX_DBG_SEL_SHIFT 8
#define RG_SENINF_CAM_MUX_DBG_SEL_MASK (0xff << 8)

#define SENINF_CAM_MUX_GCSR_DBG_OUT 0x00f4
#define RO_SENINF_CAM_MUX_DBG_OUT_SHIFT 0
#define RO_SENINF_CAM_MUX_DBG_OUT_MASK (0xffffffff << 0)

#define SENINF_CAM_MUX_GCSR_SPARE 0x00f8
#define RG_SENINF_TOP_SPARE_0_SHIFT 0
#define RG_SENINF_TOP_SPARE_0_MASK (0xff << 0)
#define RG_SENINF_TOP_SPARE_1_SHIFT 16
#define RG_SENINF_TOP_SPARE_1_MASK (0xff << 16)

#endif
