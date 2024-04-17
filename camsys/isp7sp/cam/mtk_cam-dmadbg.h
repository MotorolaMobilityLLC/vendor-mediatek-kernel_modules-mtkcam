/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __MTK_CAM_RAW_DMADBG_H
#define __MTK_CAM_RAW_DMADBG_H

#include "mtk_cam-raw_debug.h"

static __maybe_unused struct dma_debug_item dbg_UFD_R2[] = {
	{0x00002101, "ufd_r2_status"},
	{0x00002201, "ufd_r2_ufdg_crop_cnt"},
	{0x00002301, "ufd_r2_ufdb_crop_cnt"},
	{0x00002401, "ufd_r2_kernel_pixel_number_eq"},
	{0x00002501, "ufd_r2_error"},
	{0x00002601, "ufd_r2_ufdg_kernel_line_cnt"},
	{0x00002701, "ufd_r2_ufdb_kernel_line_cnt"},
	{0x00002801, "ufd_r2_state_checksum"},
	{0x00002901, "ufd_r2_line_pix_cnt_tmp"},
	{0x00002A01, "ufd_r2_line_pix_cnt"},
	{0x00002B01, "ufd_r2_ufod_gr_crc"},
};

static __maybe_unused struct dma_debug_item dbg_UFD_R5[] = {
	{0x00002109, "ufd_r5_status"},
	{0x00002209, "ufd_r5_ufdg_crop_cnt"},
	{0x00002309, "ufd_r5_ufdb_crop_cnt"},
	{0x00002409, "ufd_r5_kernel_pixel_number_eq"},
	{0x00002509, "ufd_r5_error"},
	{0x00002609, "ufd_r5_ufdg_kernel_line_cnt"},
	{0x00002709, "ufd_r5_ufdb_kernel_line_cnt"},
	{0x00002809, "ufd_r5_state_checksum"},
	{0x00002909, "ufd_r5_line_pix_cnt_tmp"},
	{0x00002A09, "ufd_r5_line_pix_cnt"},
	{0x00002B09, "ufd_r5_ufod_gr_crc"},
};

static __maybe_unused struct dma_debug_item dbg_RAWI_R2[] = {
	{0x00000006, "rawi_r2 32(hex) 0000"},
	{0x00000106, "rawi_r2 state_checksum"},
	{0x00000206, "rawi_r2 line_pix_cnt_tmp"},
	{0x00000306, "rawi_r2 line_pix_cnt"},
	{0x00000506, "rawi_r2 smi_debug_data (case 0)"},
	{0x00010606, "rawi_r2 aff(fifo)_debug_data (case 1)"},
	{0x00030606, "rawi_r2 aff(fifo)_debug_data (case 3)"},
	{0x01000046, "rawi_r2_smi_port / plane-0 / data-crc"},
	{0x01000047, "rawi_r2_smi_port / plane-0 / addr-crc"},
	{0x00000081, "rawi_r2_smi_port / smi_latency_mon output"},
	{0x000000A4, "rawi_r2_smi_port / plane-0 / { len-cnt, dle-cnt }"},
	{0x000004C0, "rawi_r2_smi_port / plane-0 / maddr_max record"},
	{0x000004C1, "rawi_r2_smi_port / plane-0 / maddr_min record"},
};

static __maybe_unused struct dma_debug_item dbg_RAWI_R2_UFD[] = {
	{0x00000007, "rawi_r2 ufd 32(hex) 0000"},
	{0x00000107, "rawi_r2 ufd state_checksum"},
	{0x00000207, "rawi_r2 ufd line_pix_cnt_tmp"},
	{0x00000307, "rawi_r2 ufd line_pix_cnt"},
	{0x01000048, "rawi_r2_smi_port / plane-1 / data-crc"},
	{0x000000A5, "rawi_r2_smi_port / plane-1 / { len-cnt, dle-cnt }"},
	{0x000005C0, "rawi_r2_smi_port / plane-1 / maddr_max record"},
	{0x000005C1, "rawi_r2_smi_port / plane-1 / maddr_min record"},
};

static __maybe_unused struct dma_debug_item dbg_RAWI_R3[] = {
	{0x00000009, "rawi_r3 32(hex) 0000"},
	{0x00000109, "rawi_r3 state_checksum"},
	{0x00000209, "rawi_r3 line_pix_cnt_tmp"},
	{0x00000309, "rawi_r3 line_pix_cnt"},
	{0x00000509, "rawi_r3 smi_debug_data (case 0)"},
	{0x00010609, "rawi_r3 aff(fifo)_debug_data (case 1)"},
	{0x00030609, "rawi_r3 aff(fifo)_debug_data (case 3)"},
	{0x01000049, "rawi_r3_smi_port / plane-0 / data-crc"},
	{0x0100004A, "rawi_r3_smi_port / plane-0 / addr-crc"},
	{0x00000082, "rawi_r3_smi_port / smi_latency_mon output"},
	{0x000000A6, "rawi_r3_smi_port / plane-0 / { len-cnt, dle-cnt }"},
	{0x000006C0, "rawi_r3_smi_port / plane-0 / maddr_max record"},
	{0x000006C1, "rawi_r3_smi_port / plane-0 / maddr_min record"},
};

static __maybe_unused struct dma_debug_item dbg_RAWI_R3_UFD[] = {
	{0x0000000A, "rawi_r3 ufd 32(hex) 0000"},
	{0x0000010A, "rawi_r3 ufd state_checksum"},
	{0x0000020A, "rawi_r3 ufd line_pix_cnt_tmp"},
	{0x0000030A, "rawi_r3 ufd line_pix_cnt"},
	{0x0100004B, "rawi_r3_smi_port / plane-1 / data-crc"},
	{0x000000A7, "rawi_r3_smi_port / plane-1 / { len-cnt, dle-cnt }"},
	{0x000007C0, "rawi_r3_smi_port / plane-1 / maddr_max record"},
	{0x000007C1, "rawi_r3_smi_port / plane-1 / maddr_min record"},
};

static __maybe_unused struct dma_debug_item dbg_RAWI_R5[] = {
	{0x0000000C, "rawi_r5 32(hex) 0000"},
	{0x0000010C, "rawi_r5 state_checksum"},
	{0x0000020C, "rawi_r5 line_pix_cnt_tmp"},
	{0x0000030C, "rawi_r5 line_pix_cnt"},
	{0x0000050C, "rawi_r5 smi_debug_data (case 0)"},
	{0x0001060C, "rawi_r5 aff(fifo)_debug_data (case 1)"},
	{0x0003060C, "rawi_r5 aff(fifo)_debug_data (case 3)"},
	{0x0100004C, "rawi_r5_smi_port / plane-0 / data-crc"},
	{0x0100004D, "rawi_r5_smi_port / plane-0 / addr-crc"},
	{0x00000083, "rawi_r5_smi_port / smi_latency_mon output"},
	{0x000000A8, "rawi_r5_smi_port / plane-0 / { len-cnt, dle-cnt }"},
	{0x000008C0, "rawi_r5_smi_port / plane-0 / maddr_max record"},
	{0x000008C1, "rawi_r5_smi_port / plane-0 / maddr_min record"},
};

static __maybe_unused struct dma_debug_item dbg_RAWI_R5_UFD[] = {
	{0x0000000D, "rawi_r5 ufd 32(hex) 0000"},
	{0x0000010D, "rawi_r5 ufd state_checksum"},
	{0x0000020D, "rawi_r5 ufd line_pix_cnt_tmp"},
	{0x0000030D, "rawi_r5 ufd line_pix_cnt"},
	{0x0100004E, "rawi_r5_smi_port / plane-1 / data-crc"},
	{0x000000A9, "rawi_r5_smi_port / plane-1 / { len-cnt, dle-cnt }"},
	{0x000009C0, "rawi_r5_smi_port / plane-1 / maddr_max record"},
	{0x000009C1, "rawi_r5_smi_port / plane-1 / maddr_min record"},
};

static __maybe_unused struct dma_debug_item dbg_IMGO_R1[] = {
	{0x00000017, "imgo_r1 32(hex) 0000"},
	{0x00000117, "imgo_r1 state_checksum"},
	{0x00000217, "imgo_r1 line_pix_cnt_tmp"},
	{0x00000317, "imgo_r1 line_pix_cnt"},
	{0x00000817, "imgo_r1 smi_debug_data (case 0)"},
	{0x00010717, "imgo_r1 aff(fifo)_debug_data (case 1)"},
	{0x00030717, "imgo_r1 aff(fifo)_debug_data (case 3)"},

	{0x01000057, "imgo_r1_smi_port / plane-0 (i.e. imgo_r1) / data-crc"},

	{0x00000087, "imgo_r1_smi_port / smi_latency_mon output"},

	{0x000000AA, "imgo_r1_smi_port / plane-0 / {len-cnt, dle-cnt}"},
	{0x000000AB, "imgo_r1_smi_port / plane-0 / {load_com-cnt, bvalid-cnt}"},

	{0x000012C0, "imgo_r1_smi_port / plane-0 / maddr_max record"},
	{0x000012C1, "imgo_r1_smi_port / plane-0 / maddr_min record"},
};

static __maybe_unused struct dma_debug_item dbg_AAHO_R1[] = {
	{0x0000001D, "32(hex) 0000"},
	{0x0000011D, "state_checksum"},
	{0x0000021D, "line_pix_cnt_tmp"},
	{0x0000031D, "line_pix_cnt"},
	{0x0000041D, "important_status"},
	{0x0000051D, "cmd_data_cnt"},
	{0x0000061D, "cmd_cnt_for_bvalid_phase"},
	{0x0000071D, "input_h_cnt"},
	{0x0000081D, "input_v_cnt"},
	{0x0000091D, "xfer_y_cnt"},

	{0x0100005D, "aao_r1_smi_port_data_crc_p1"},
};

static __maybe_unused struct dma_debug_item dbg_PDO_R1[] = {
	{0x0000001B, "32(hex) 0000"},
	{0x0000011B, "state_checksum"},
	{0x0000021B, "line_pix_cnt_tmp"},
	{0x0000031B, "line_pix_cnt"},
	{0x0000041B, "important_status"},
	{0x0000051B, "cmd_data_cnt"},
	{0x0000061B, "cmd_cnt_for_bvalid_phase"},
	{0x0000071B, "input_h_cnt"},
	{0x0000081B, "input_v_cnt"},
	{0x0000091B, "xfer_y_cnt"},

	{0x0100005B, "ufeo_r1_smi_port_data_crc_p2"},
};

static __maybe_unused struct dma_debug_item dbg_PDI_R1[] = {
	{0x00000015, "pdi_r1 32(hex) 0000"},
	{0x00000115, "pdi_r1 state_checksum"},
	{0x00000215, "pdi_r1 line_pix_cnt_tmp"},
	{0x00000315, "pdi_r1 line_pix_cnt"},
	{0x00000415, "pdi_r1 important_status"},
	{0x00000515, "pdi_r1 cmd_data_cnt"},
	{0x00000615, "pdi_r1 tilex_byte_cnt"},
	{0x00000715, "pdi_r1 tiley_cnt"},
	{0x00000815, "pdi_r1 burst_line_cnt"},
};

static __maybe_unused struct dma_debug_item dbg_YUVO_R1[] = {
	{0x00000000, "yuvo_r1 32(hex) 0000"},
	{0x00000100, "yuvo_r1 state_checksum"},
	{0x00000200, "yuvo_r1 line_pix_cnt_tmp"},
	{0x00000300, "yuvo_r1 line_pix_cnt"},
	{0x00000800, "yuvo_r1 smi_debug_data (case 0)"},
	{0x00010700, "yuvo_r1 aff(fifo)_debug_data (case 1)"},
	{0x00030700, "yuvo_r1 aff(fifo)_debug_data (case 3)"},

	{0x00000001, "yuvbo_r1 32(hex) 0000"},
	{0x00000101, "yuvbo_r1 state_checksum"},
	{0x00000201, "yuvbo_r1 line_pix_cnt_tmp"},
	{0x00000301, "yuvbo_r1 line_pix_cnt"},
	{0x00000801, "yuvbo_r1 smi_debug_data (case 0)"},
	{0x00010701, "yuvbo_r1 aff(fifo)_debug_data (case 1)"},
	{0x00030701, "yuvbo_r1 aff(fifo)_debug_data (case 3)"},

	{0x01000040, "yuvo_r1 / data-crc"},
	{0x01000041, "yuvbo_r1 / data-crc"},

	{0x000000AD, "yuvo_r1 / { len-cnt, dle-cnt }"},
	{0x000000AE, "yuvo_r1 / { load_com-cnt, bvalid-cnt }"},
	{0x000000AF, "yuvbo_r1 / { len-cnt, dle-cnt }"},
	{0x000000B0, "yuvbo_r1 / { load_com-cnt, bvalid-cnt }"},

	{0x000000C0, "yuvo_r1 / maddr_max record"},
	{0x000000C1, "yuvo_r1 / maddr_min record"},
	{0x000001C0, "yuvbo_r1 / maddr_max record"},
	{0x000001C1, "yuvbo_r1 / maddr_min record"},

	{0x00000080, "yuvo_r1_smi_port / smi_latency_mon output"},
};

static __maybe_unused struct dma_debug_item dbg_YUVCO_R1[] = {
	{0x00000002, "yuvco_r1 32(hex) 0000"},
	{0x00000102, "yuvco_r1 state_checksum"},
	{0x00000202, "yuvco_r1 line_pix_cnt_tmp"},
	{0x00000302, "yuvco_r1 line_pix_cnt"},
	{0x00000802, "yuvco_r1 smi_debug_data (case 0)"},
	{0x00010702, "yuvco_r1 aff(fifo)_debug_data (case 1)"},
	{0x00030702, "yuvco_r1 aff(fifo)_debug_data (case 3)"},

	{0x00000003, "yuvdo_r1 32(hex) 0000"},
	{0x00000103, "yuvdo_r1 state_checksum"},
	{0x00000203, "yuvdo_r1 line_pix_cnt_tmp"},
	{0x00000303, "yuvdo_r1 line_pix_cnt"},
	{0x00000803, "yuvdo_r1 smi_debug_data (case 0)"},
	{0x00010703, "yuvdo_r1 aff(fifo)_debug_data (case 1)"},
	{0x00030703, "yuvdo_r1 aff(fifo)_debug_data (case 3)"},

	{0x01000042, "yuvco_r1 / data-crc"},
	{0x01000043, "yuvdo_r1 / data-crc"},

	{0x000000B1, "yuvco_r1 / { len-cnt, dle-cnt }"},
	{0x000000B2, "yuvco_r1 / { load_com-cnt, bvalid-cnt }"},
	{0x000000B3, "yuvdo_r1 / { len-cnt, dle-cnt }"},
	{0x000000B4, "yuvdo_r1 / { load_com-cnt, bvalid-cnt }"},

	{0x000002C0, "yuvco_r1 / maddr_max record"},
	{0x000002C1, "yuvco_r1 / maddr_min record"},
	{0x000003C0, "yuvdo_r1 / maddr_max record"},
	{0x000003C1, "yuvdo_r1 / maddr_min record"},

	{0x00000080, "yuvo_r1_smi_port / smi_latency_mon output"},
};

static __maybe_unused struct dma_debug_item cqi_stat[] = {
	{0x00009000, "CQ 9000 "},
	{0x00009100, "CQ 9100 "},
	{0x00009200, "CQ 9200 "},
	{0x00009300, "CQ 9300 "},
	{0x00009400, "CQ 9400 "},
	{0x00009500, "CQ 9500 "},
	{0x00009600, "CQ 9600 "},
	{0x00009700, "CQ 9700 "},
	{0x00009800, "CQ 9800 "},
	{0x00009900, "CQ 9900 "},
	{0x00009a00, "CQ 9a00 "},
	{0x00009a00, "CQ 9a00 "},
};

static __maybe_unused struct dma_debug_item cqi_sum[] = {
	{0x00009000, "CQ 9000 "},
	{0x00009100, "CQ 9100 "},
	{0x00009200, "CQ 9200 "},
	{0x00009400, "CQ 9400 "},
	{0x00009500, "CQ 9500 "},
	{0x00009600, "CQ 9600 "},
	{0x00009800, "CQ 9800 "},
	{0x00009900, "CQ 9900 "},
	{0x00009A00, "CQ 9a00 "},
	{0x00009C00, "CQ 9c00 "},
	{0x00009D00, "CQ 9d00 "},
	{0x00009300, "CQ 9e00 "},
};

static __maybe_unused struct dma_debug_item dbg_YUVO_R3[] = {
	{0x00000004, "yuvo_r3 32(hex) 0000"},
	{0x00000104, "yuvo_r3 state_checksum"},
	{0x00000204, "yuvo_r3 line_pix_cnt_tmp"},
	{0x00000304, "yuvo_r3 line_pix_cnt"},
	{0x00000804, "smi_debug_data (case 0)"},
	{0x00010704, "aff(fifo)_debug_data (case 1)"},
	{0x00030704, "aff(fifo)_debug_data (case 3)"},

	{0x00000005, "yuvbo_r3 32(hex) 0000"},
	{0x00000105, "yuvbo_r3 state_checksum"},
	{0x00000205, "yuvbo_r3 line_pix_cnt_tmp"},
	{0x00000305, "yuvbo_r3 line_pix_cnt"},
	{0x00000805, "yuvbo_r3 smi_debug_data (case 0)"},
	{0x00010705, "yuvbo_r3 aff(fifo)_debug_data (case 1)"},
	{0x00030705, "yuvbo_r3 aff(fifo)_debug_data (case 3)"},

	{0x01000044, "yuvo_r3 / data-crc"},
	{0x01000045, "yuvbo_r3 / data-crc"},

	{0x000000B5, "yuvo_r3 / { len-cnt, dle-cnt }"},
	{0x000000B6, "yuvo_r3 / { load_com-cnt, bvalid-cnt }"},
	{0x000000B7, "yuvbo_r3 / { len-cnt, dle-cnt }"},
	{0x000000B8, "yuvbo_r3 / { load_com-cnt, bvalid-cnt }"},

	{0x000004C0, "yuvo_r3 / maddr_max record"},
	{0x000004C1, "yuvo_r3 / maddr_min record"},
	{0x000005C0, "yuvbo_r3 / maddr_max record"},
	{0x000005C1, "yuvbo_r3 / maddr_min record"},

	{0x00000081, "yuvo_r3_smi_port / smi_latency_mon output"},
};

static __maybe_unused struct dma_debug_item dbg_YUVCO_R3[] = {
	{0x00000006, "yuvco_r3 32(hex) 0000"},
	{0x00000106, "yuvco_r3 state_checksum"},
	{0x00000206, "yuvco_r3 line_pix_cnt_tmp"},
	{0x00000306, "yuvco_r3 line_pix_cnt"},
	{0x00000806, "yuvco_r3 smi_debug_data (case 0)"},
	{0x00010706, "yuvco_r3 aff(fifo)_debug_data (case 1)"},
	{0x00030706, "yuvco_r3 aff(fifo)_debug_data (case 3)"},

	{0x00000007, "yuvdo_r3 32(hex) 0000"},
	{0x00000107, "yuvdo_r3 state_checksum"},
	{0x00000207, "yuvdo_r3 line_pix_cnt_tmp"},
	{0x00000307, "yuvdo_r3 line_pix_cnt"},
	{0x00000807, "yuvdo_r3 smi_debug_data (case 0)"},
	{0x00010707, "yuvdo_r3 aff(fifo)_debug_data (case 1)"},
	{0x00030707, "yuvdo_r3 aff(fifo)_debug_data (case 3)"},

	{0x01000046, "yuvco_r1 / data-crc"},
	{0x01000047, "yuvdo_r1 / data-crc"},

	{0x000000B9, "yuvco_r3 / { len-cnt, dle-cnt }"},
	{0x000000BA, "yuvco_r3 / { load_com-cnt, bvalid-cnt }"},
	{0x000000BB, "yuvdo_r3 / { len-cnt, dle-cnt }"},
	{0x000000BC, "yuvdo_r3 / { load_com-cnt, bvalid-cnt }"},

	{0x000006C0, "yuvco_r3 / maddr_max record"},
	{0x000006C1, "yuvco_r3 / maddr_min record"},
	{0x000007C0, "yuvdo_r3 / maddr_max record"},
	{0x000007C1, "yuvdo_r3 / maddr_min record"},

	{0x00000081, "yuvo_r3_smi_port / smi_latency_mon output"},
};

static __maybe_unused struct dma_debug_item dbg_YUVO_R2[] = {
	{0x00000008, "yuvo_r2 32(hex) 0000"},
	{0x00000108, "yuvo_r2 state_checksum"},
	{0x00000208, "yuvo_r2 line_pix_cnt_tmp"},
	{0x00000308, "yuvo_r2 line_pix_cnt"},
	{0x00000408, "yuvo_r2 important_status"},
	{0x00000508, "yuvo_r2 cmd_data_cnt"},
	{0x00000608, "yuvo_r2 cmd_cnt_for_bvalid_phase"},
	{0x00000708, "yuvo_r2 input_h_cnt"},
	{0x00000808, "yuvo_r2 input_v_cnt"},
	{0x00000908, "yuvo_r2 xfer_y_cnt"},

	{0x00000009, "yuvbo_r2 32(hex) 0000"},
	{0x00000109, "yuvbo_r2 state_checksum"},
	{0x00000209, "yuvbo_r2 line_pix_cnt_tmp"},
	{0x00000309, "yuvbo_r2 line_pix_cnt"},
	{0x00000409, "yuvbo_r2 important_status"},
	{0x00000509, "yuvbo_r2 cmd_data_cnt"},
	{0x00000609, "yuvbo_r2 cmd_cnt_for_bvalid_phase"},
	{0x00000709, "yuvbo_r2 input_h_cnt"},
	{0x00000809, "yuvbo_r2 input_v_cnt"},
	{0x00000909, "yuvbo_r2 xfer_y_cnt"},

	{0x01000048, "yuvo_r2 / data-crc"},
	{0x01000049, "yuvbo_r2 / data-crc"},

	{0x000008C0, "yuvo_r2 / maddr_max record"},
	{0x000008C1, "yuvo_r2 / maddr_min record"},
	{0x000009C0, "yuvbo_r2 / maddr_max record"},
	{0x000009C1, "yuvbo_r2 / maddr_min record"},

	{0x00000082, "yuvo_r2_smi_port / smi_latency_mon output"},
};

static __maybe_unused struct dma_debug_item dbg_YUVO_R4[] = {
	{0x0000000A, "yuvo_r4 32(hex) 0000"},
	{0x0000010A, "yuvo_r4 state_checksum"},
	{0x0000020A, "yuvo_r4 line_pix_cnt_tmp"},
	{0x0000030A, "yuvo_r4 line_pix_cnt"},
	{0x0000040A, "yuvo_r4 important_status"},
	{0x0000050A, "yuvo_r4 cmd_data_cnt"},
	{0x0000060A, "yuvo_r4 cmd_cnt_for_bvalid_phase"},
	{0x0000070A, "yuvo_r4 input_h_cnt"},
	{0x0000080A, "yuvo_r4 input_v_cnt"},
	{0x0000090A, "yuvo_r4 xfer_y_cnt"},

	{0x0000000B, "yuvbo_r4 32(hex) 0000"},
	{0x0000010B, "yuvbo_r4 state_checksum"},
	{0x0000020B, "yuvbo_r4 line_pix_cnt_tmp"},
	{0x0000030B, "yuvbo_r4 line_pix_cnt"},
	{0x0000040B, "yuvbo_r4 important_status"},
	{0x0000050B, "yuvbo_r4 cmd_data_cnt"},
	{0x0000060B, "yuvbo_r4 cmd_cnt_for_bvalid_phase"},
	{0x0000070B, "yuvbo_r4 input_h_cnt"},
	{0x0000080B, "yuvbo_r4 input_v_cnt"},
	{0x0000090B, "yuvbo_r4 xfer_y_cnt"},

	{0x0100004A, "yuvo_r4 / data-crc"},
	{0x0100004B, "yuvbo_r4 / data-crc"},

	{0x00000AC0, "yuvo_r4 / maddr_max record"},
	{0x00000AC1, "yuvo_r4/ maddr_min record"},
	{0x00000BC0, "yuvbo_r4 / maddr_max record"},
	{0x00000BC1, "yuvbo_r4 / maddr_min record"},

	{0x00000082, "yuvo_r2_smi_port / smi_latency_mon output"},
};

static __maybe_unused struct dma_debug_item dbg_RZH1N2TO_R1[] = {
	{0x00000016, "rzh1n2to_r1 32(hex) 0000"},
	{0x00000116, "rzh1n2to_r1 state_checksum"},
	{0x00000216, "rzh1n2to_r1 line_pix_cnt_tmp"},
	{0x00000316, "rzh1n2to_r1 line_pix_cnt"},
	{0x00000416, "rzh1n2to_r1 important_status"},
	{0x00000516, "rzh1n2to_r1 cmd_data_cnt"},
	{0x00000616, "rzh1n2to_r1 cmd_cnt_for_bvalid_phase"},
	{0x00000716, "rzh1n2to_r1 input_h_cnt"},
	{0x00000816, "rzh1n2to_r1 input_v_cnt"},
	{0x00000916, "rzh1n2to_r1 xfer_y_cnt"},

	{0x0000000E, "rzh1n2tbo_r1 32(hex) 0000"},
	{0x0000010E, "rzh1n2tbo_r1 state_checksum"},
	{0x0000020E, "rzh1n2tbo_r1 line_pix_cnt_tmp"},
	{0x0000030E, "rzh1n2tbo_r1 line_pix_cnt"},
	{0x0000040E, "rzh1n2tbo_r1 important_status"},
	{0x0000050E, "rzh1n2tbo_r1 cmd_data_cnt"},
	{0x0000060E, "rzh1n2tbo_r1 cmd_cnt_for_bvalid_phase"},
	{0x0000070E, "rzh1n2tbo_r1 input_h_cnt"},
	{0x0000080E, "rzh1n2tbo_r1 input_v_cnt"},
	{0x0000090E, "rzh1n2tbo_r1 xfer_y_cnt"},

	{0x01000055, "rzh1n2to_r1 / data-crc"},
	{0x0100004E, "rzh1n2tbo_r1 / data-crc"},

	{0x000016C0, "rzh1n2to_r1 / maddr_max record"},
	{0x000016C1, "rzh1n2to_r1 / maddr_min record"},
	{0x00000EC0, "rzh1n2tbo_r1 / maddr_max record"},
	{0x00000EC1, "rzh1n2tbo_r1 / maddr_min record"},

	{0x00000083, "yuvo_r5_smi_port / smi_latency_mon output"},
	{0x00000085, "drz4no_r3_smi_port / smi_latency_mon output"},
};

static __maybe_unused struct dma_debug_item dbg_RZH1N2TO_R2[] = {
	{0x00000011, "rzh1n2to_r2 32(hex) 0000"},
	{0x00000111, "rzh1n2to_r2 state_checksum"},
	{0x00000211, "rzh1n2to_r2 line_pix_cnt_tmp"},
	{0x00000311, "rzh1n2to_r2 line_pix_cnt"},
	{0x00000411, "rzh1n2to_r2 important_status"},
	{0x00000511, "rzh1n2to_r2 cmd_data_cnt"},
	{0x00000611, "rzh1n2to_r2 cmd_cnt_for_bvalid_phase"},
	{0x00000711, "rzh1n2to_r2 input_h_cnt"},
	{0x00000811, "rzh1n2to_r2 input_v_cnt"},
	{0x00000911, "rzh1n2to_r2 xfer_y_cnt"},

	{0x01000051, "rzh1n2to_r2 / data-crc"},

	{0x000011C0, "rzh1n2to_r2 / maddr_max record"},
	{0x000011C1, "rzh1n2to_r2 / maddr_min record"},

	{0x00000084, "tcyso_r1_smi_port / smi_latency_mon output"},
};

static __maybe_unused struct dma_debug_item dbg_RZH1N2TO_R3[] = {
	{0x00000015, "rzh1n2to_r3 32(hex) 0000"},
	{0x00000115, "rzh1n2to_r3 state_checksum"},
	{0x00000215, "rzh1n2to_r3 line_pix_cnt_tmp"},
	{0x00000315, "rzh1n2to_r3 line_pix_cnt"},
	{0x00000415, "rzh1n2to_r3 important_status"},
	{0x00000515, "rzh1n2to_r3 cmd_data_cnt"},
	{0x00000615, "rzh1n2to_r3 cmd_cnt_for_bvalid_phase"},
	{0x00000715, "rzh1n2to_r3 input_h_cnt"},
	{0x00000815, "rzh1n2to_r3 input_v_cnt"},
	{0x00000915, "rzh1n2to_r3 xfer_y_cnt"},

	{0x0000000F, "rzh1n2tbo_r3 32(hex) 0000"},
	{0x0000010F, "rzh1n2tbo_r3 state_checksum"},
	{0x0000020F, "rzh1n2tbo_r3 line_pix_cnt_tmp"},
	{0x0000030F, "rzh1n2tbo_r3 line_pix_cnt"},
	{0x0000040F, "rzh1n2tbo_r3 important_status"},
	{0x0000050F, "rzh1n2tbo_r3 cmd_data_cnt"},
	{0x0000060F, "rzh1n2tbo_r3 cmd_cnt_for_bvalid_phase"},
	{0x0000070F, "rzh1n2tbo_r3 input_h_cnt"},
	{0x0000080F, "rzh1n2tbo_r3 input_v_cnt"},
	{0x0000090F, "rzh1n2tbo_r3 xfer_y_cnt"},

	{0x01000054, "rzh1n2to_r3 / data-crc"},
	{0x0100004f, "rzh1n2tbo_r3 / data-crc"},

	{0x000015C0, "rzh1n2to_r3 / maddr_max record"},
	{0x000015C1, "rzh1n2to_r3 / maddr_min record"},
	{0x00000FC0, "rzh1n2tbo_r3 / maddr_max record"},
	{0x00000FC1, "rzh1n2tbo_r3 / maddr_min record"},

	{0x00000083, "yuvo_r5_smi_port / smi_latency_mon output"},
	{0x00000085, "drz4no_r3_smi_port / smi_latency_mon output"},
};

static __maybe_unused struct dma_debug_item dbg_DRZS4NO_R1[] = {
	{0x00000012, "drzs4no_r1 32(hex) 0000"},
	{0x00000112, "drzs4no_r1 state_checksum"},
	{0x00000212, "drzs4no_r1 line_pix_cnt_tmp"},
	{0x00000312, "drzs4no_r1 line_pix_cnt"},
	{0x00000412, "drzs4no_r1 important_status"},
	{0x00000512, "drzs4no_r1 cmd_data_cnt"},
	{0x00000612, "drzs4no_r1 cmd_cnt_for_bvalid_phase"},
	{0x00000712, "drzs4no_r1 input_h_cnt"},
	{0x00000812, "drzs4no_r1 input_v_cnt"},
	{0x00000912, "drzs4no_r1 xfer_y_cnt"},

	{0x01000052, "drzs4no_r1 / data-crc"},

	{0x000012C0, "drzs4no_r1 / maddr_max record"},
	{0x000012C1, "drzs4no_r1 / maddr_min record"},

	{0x00000084, "tcyso_r1_smi_port / smi_latency_mon output"},
};

static __maybe_unused struct dma_debug_item dbg_DRZS4NO_R3[] = {
	{0x00000014, "drzs4no_r3 32(hex) 0000"},
	{0x00000114, "drzs4no_r3 state_checksum"},
	{0x00000214, "drzs4no_r3 line_pix_cnt_tmp"},
	{0x00000314, "drzs4no_r3 line_pix_cnt"},
	{0x00000414, "drzs4no_r3 important_status"},
	{0x00000514, "drzs4no_r3 cmd_data_cnt"},
	{0x00000614, "drzs4no_r3 cmd_cnt_for_bvalid_phase"},
	{0x00000714, "drzs4no_r3 input_h_cnt"},
	{0x00000814, "drzs4no_r3 input_v_cnt"},
	{0x00000914, "drzs4no_r3 xfer_y_cnt"},

	{0x000014C0, "drzs4no_r3 / maddr_max record"},
	{0x000014C1, "drzs4no_r3 / maddr_min record"},

	{0x00000085, "drzs4no_r3_smi_port / smi_latency_mon output"},
};

static __maybe_unused struct dma_debug_item dbg_DRZH2NO_R8[] = {
	{0x00000013, "drzh2no_r8 32(hex) 0000"},
	{0x00000113, "drzh2no_r8 state_checksum"},
	{0x00000213, "drzh2no_r8 line_pix_cnt_tmp"},
	{0x00000313, "drzh2no_r8 line_pix_cnt"},
	{0x00000413, "drzh2no_r8 important_status"},
	{0x00000513, "drzh2no_r8 cmd_data_cnt"},
	{0x00000613, "drzh2no_r8 cmd_cnt_for_bvalid_phase"},
	{0x00000713, "drzh2no_r8 input_h_cnt"},
	{0x00000813, "drzh2no_r8 input_v_cnt"},
	{0x00000913, "drzh2no_r8 xfer_y_cnt"},

	{0x01000053, "drzh2no_r8 / data-crc"},

	{0x000013C0, "drzh2no_r8 maddr_max record"},
	{0x000013C1, "drzh2no_r8 maddr_min record"},

	{0x00000084, "tcyso_r1_smi_port / smi_latency_mon output"},
};

static __maybe_unused struct dma_debug_item dbg_TCYSO_R1[] = {
	{0x00000010, "tcyso_r1 32(hex) 0000"},
	{0x00000110, "tcyso_r1 state_checksum"},
	{0x00000210, "tcyso_r1 line_pix_cnt_tmp"},
	{0x00000310, "tcyso_r1 line_pix_cnt"},
	{0x00000410, "tcyso_r1 important_status"},
	{0x00000510, "tcyso_r1 cmd_data_cnt"},
	{0x00000610, "tcyso_r1 cmd_cnt_for_bvalid_phase"},
	{0x00000710, "tcyso_r1 input_h_cnt"},
	{0x00000810, "tcyso_r1 input_v_cnt"},
	{0x00000910, "tcyso_r1 xfer_y_cnt"},

	{0x01000050, "tcyso_r1 / data-crc"},

	{0x000010C0, "tcyso_r1 / maddr_max record"},
	{0x000010C1, "tcyso_r1 / maddr_min record"},

	{0x00000084, "tcyso_r1_smi_port / smi_latency_mon output"},
};


static __maybe_unused struct dma_debug_item dbg_ulc_cmd_cnt[] = {
	{0x00000511, "bpci_r1 cmd_data_cnt"},
	{0x00000512, "bpci_r2 cmd_data_cnt"},
	{0x00000513, "bpci_r3 cmd_data_cnt"},
	{0x00000516, "aai_r1 cmd_data_cnt"},
	{0x00000518, "rawi_r6 cmd_data_cnt"},
	{0x0000051F, "tsfso_r1 cmd_data_cnt"},
	{0x0000061F, "tsfso_r1 cmd_cnt_for_bvalid_phase"},
	{0x00000520, "ltmso_r1 cmd_data_cnt"},
	{0x00000620, "ltmso_r1 cmd_cnt_for_bvalid_phase"},
	{0x00000521, "tsfso_r2 cmd_data_cnt"},
	{0x00000621, "tsfso_r2 cmd_cnt_for_bvalid_phase"},
	{0x00000522, "flko_r1 cmd_data_cnt"},
	{0x00000622, "flko_r1 cmd_cnt_for_bvalid_phase"},
	{0x00000523, "ufeo_r1 cmd_data_cnt"},
	{0x00000623, "ufeo_r1 cmd_cnt_for_bvalid_phase"},
	{0x00000528, "bino_r1 cmd_data_cnt"},
	{0x00000628, "bino_r1 cmd_cnt_for_bvalid_phase"},
};

static __maybe_unused struct dma_debug_item dbg_ori_cmd_cnt[] = {
	{0x000000A0, "rawi_r2_smi_port / plane-0 (i.e. rawi_r2) / { len-cnt, dle-cnt }"},
	{0x000000A1, "ufdi_r2_smi_port / plane-0 (i.e. ufdi_r2) / { len-cnt, dle-cnt }"},
	{0x000000A2, "rawi_r3_smi_port / plane-0 (i.e. rawi_r3) / { len-cnt, dle-cnt }"},
	{0x000000A3, "ufdi_r3_smi_port / plane-0 (i.e. ufdi_r3) / { len-cnt, dle-cnt }"},
	{0x000000A4, "rawi_r4_smi_port / plane-0 (i.e. rawi_r4) / { len-cnt, dle-cnt }"},
	{0x000000A5, "rawi_r5_smi_port / plane-0 (i.e. rawi_r5) / { len-cnt, dle-cnt }"},
	{0x000000A6, "cqi_r1_smi_port / plane-0 (i.e. cqi_r1) / { len-cnt, dle-cnt }"},
	{0x000000A7, "cqi_r1_smi_port / plane-1 (i.e. cqi_r3) / { len-cnt, dle-cnt }"},
	{0x000000A8, "cqi_r2_smi_port / plane-0 (i.e. cqi_r2) / { len-cnt, dle-cnt }"},
	{0x000000A9, "cqi_r2_smi_port / plane-1 (i.e. cqi_r4) / { len-cnt, dle-cnt }"},
	{0x000000AA, "lsci_r1_smi_port / plane-0 (i.e. lsci_r1) / { len-cnt, dle-cnt }"},
	{0x000000AB, "imgo_r1_smi_port / plane-0 (i.e. imgo_r1) / { len-cnt, dle-cnt }"},
	{0x000000AC, "imgo_r1_smi_port / plane-0 (i.e. imgo_r1) / { load_com-cnt, bvalid-cnt }"},
	{0x000000AD, "fho_r1_smi_port / plane-0 (i.e. fho_r1) / { len-cnt, dle-cnt }"},
	{0x000000AE, "fho_r1_smi_port / plane-0 (i.e. fho_r1) / { load_com-cnt, bvalid-cnt }"},
	{0x000000AF, "fho_r1_smi_port / plane-1 (i.e. aaho_r1) / { len-cnt, dle-cnt }"},
	{0x000000B0, "fho_r1_smi_port / plane-1 (i.e. aaho_r1) / { load_com-cnt, bvalid-cnt }"},
	{0x000000B1, "fho_r1_smi_port / plane-2 (i.e. pdo_r1) / { len-cnt, dle-cnt }"},
	{0x000000B2, "fho_r1_smi_port / plane-2 (i.e. pdo_r1) / { load_com-cnt, bvalid-cnt }"},
	{0x000000B3, "aao_r1_smi_port / plane-0 (i.e. aao_r1) / { len-cnt, dle-cnt }"},
	{0x000000B4, "aao_r1_smi_port / plane-0 (i.e. aao_r1) / { load_com-cnt, bvalid-cnt }"},
	{0x000000B5, "aao_r1_smi_port / plane-1 (i.e. afo_r1) / { len-cnt, dle-cnt }"},
	{0x000000B6, "aao_r1_smi_port / plane-1 (i.e. afo_r1) / { load_com-cnt, bvalid-cnt }"},
	{0x000000B7, "ufdi_r5_smi_port / plane-0 (i.e. ufdi_r5) / { len-cnt, dle-cnt }"},
};
#endif /*__MTK_CAM_RAW_DMADBG_H*/
