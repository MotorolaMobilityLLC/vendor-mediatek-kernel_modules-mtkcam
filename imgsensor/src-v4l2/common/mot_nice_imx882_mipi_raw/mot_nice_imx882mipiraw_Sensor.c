// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 imx882mipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include "mot_nice_imx882mipiraw_Sensor.h"
#include "mot_nice_imx882_cali.h"

#define IMX882_EMBEDDED_DATA_EN 0
#define ENABLE_IMX882_LONG_EXPOSURE 1


static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int imx882_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx882_get_min_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static int imx882_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);

#if  ENABLE_IMX882_LONG_EXPOSURE
static int imx882_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
#endif

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, imx882_set_test_pattern},
	{SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, imx882_get_min_shutter},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, imx882_seamless_switch},
#if  ENABLE_IMX882_LONG_EXPOSURE
	{SENSOR_FEATURE_SET_ESHUTTER, imx882_set_shutter},
#endif
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
			//.is_active_line = TRUE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.is_active_line = TRUE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
			//.is_active_line = TRUE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.is_active_line = TRUE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
			//.is_active_line = TRUE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.is_active_line = TRUE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0800,
			.vsize = 0x0480,
			.user_data_desc = VC_STAGGER_NE,
			//.is_active_line = TRUE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
			//.is_active_line = TRUE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.is_active_line = TRUE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0800,
			.vsize = 0x0480,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0800,
			.vsize = 0x0600,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.is_active_line = TRUE,
		},
	},
};


static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0200,
			.vsize = 0x0480,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
			.is_active_line = TRUE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x2000,
			.vsize = 0x1800,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0800,
			.vsize = 0x0600,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0800,
			.vsize = 0x0180,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.is_active_line = TRUE,
		},
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
	},
	.iMirrorFlip = IMAGE_HV_MIRROR,
	.i4ModeIndex = 3,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_cus2_info = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4PosL = { {0, 0} },
	.i4PosR = { {0, 0} },
	.i4Crop = {
		// <pre> <cap> <normal_video> <hs_video> <<slim_video>>
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// <<cust1>> <<cust2>> <<cust3>> <cust4> <cust5>
		{0, 0}, {2048, 1536}, {0, 0}, {0, 0}, {0, 0},
	},
	.iMirrorFlip = IMAGE_HV_MIRROR,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
	.i4FullRawW = 8192,
	.i4FullRawH = 6144,
	.i4ModeIndex = 3,
	.i4VCPackNum = 1,
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_cus3_info = {
	.i4OffsetX = 16,
	.i4OffsetY = 0,
	.i4PitchX = 8,
	.i4PitchY = 16,
	.i4PairNum = 4,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4PosL = {{16, 3}, {20, 5}, {19, 10}, {23, 12}},
	.i4PosR = {{18, 1}, {22, 7}, {17, 8}, {21, 14}},
	.i4Crop = {
		// <pre> <cap> <normal_video> <hs_video> <<slim_video>>
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// <<cust1>> <<cust2>> <<cust3>> <cust4> <cust5>
		{0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0},
	},
	.i4BlockNumX = 508,
	.i4BlockNumY = 144,
	.i4VolumeX = 1,
	.i4VolumeY = 1,
	.iMirrorFlip = IMAGE_NORMAL,
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV,
	.i4ModeIndex = 0,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		.i4PDPattern = 3, //pair PD
		.i4PDRepetition = 8,
		.i4PDOrder = {1,0,0,1,1,0,0,1}, // R = 1, L = 0
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = imx882_preview_setting,
		.mode_setting_len = ARRAY_SIZE(imx882_preview_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = imx882_seamless_preview,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx882_seamless_preview),
		//.hdr_mode = HDR_NONE,
		//.raw_cnt = 1,
		//.exp_cnt = 1,//
		.pclk = 878400000,
		.linelength = 7500,
		.framelength = 3900,
		.max_framerate = 300,
		.mipi_pixel_rate = 800000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.min_exposure_line = 6,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = 1.4287*BASEGAIN,
		.ana_gain_max = 64*BASEGAIN,
		.dig_gain_min = 1*BASEGAIN,
		.dig_gain_max = 1*BASEGAIN,
		.dig_gain_step = 4,
		.csi_param = {
			.cphy_settle = 73,
		},
		.dpc_enabled = true, /* reg 0x0b06 */
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = imx882_capture_setting,
		.mode_setting_len = ARRAY_SIZE(imx882_capture_setting),
		//.seamless_switch_group = PARAM_UNDEFINED,
		//.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		//.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		//.hdr_mode = HDR_NONE,
		//.raw_cnt = 1,
		//.exp_cnt = 1,
		.pclk = 878400000,
		.linelength = 7500,
		.framelength = 3900,
		.max_framerate = 300,
		.mipi_pixel_rate = 800000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.min_exposure_line = 6,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,

		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = 1.4287*BASEGAIN,
		.ana_gain_max = 64*BASEGAIN,
		.dig_gain_min = 1*BASEGAIN,
		.dig_gain_max = 1*BASEGAIN,
		.dig_gain_step = 4,
		.csi_param = {
			.cphy_settle = 73,
		},
		.dpc_enabled = true, /* reg 0x0b06 */
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = imx882_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(imx882_normal_video_setting),
		//.seamless_switch_group = PARAM_UNDEFINED,
		//.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		//.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		//.hdr_mode = HDR_NONE,
		//.raw_cnt = 1,
		//.exp_cnt = 1,
		.pclk = 878400000,
		.linelength = 7500,
		.framelength = 3900,
		.max_framerate = 300,
		.mipi_pixel_rate = 800000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.min_exposure_line = 6,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,

		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = 1.4287*BASEGAIN,
		.ana_gain_max = 64*BASEGAIN,
		.dig_gain_min = 1*BASEGAIN,
		.dig_gain_max = 1*BASEGAIN,
		.dig_gain_step = 4,
		.csi_param = {
			.cphy_settle = 73,
		},
		.dpc_enabled = true, /* reg 0x0b06 */
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = imx882_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(imx882_hs_video_setting),
		//.seamless_switch_group = PARAM_UNDEFINED,
		//.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		//.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		//.hdr_mode = HDR_NONE,
		//.raw_cnt = 1,
		//.exp_cnt = 1,
		.pclk = 806400000,
		.linelength = 2468,
		.framelength = 2702,
		.max_framerate = 1200,
		.mipi_pixel_rate = 800000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.min_exposure_line = 8,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,

		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		//.pdaf_cap = FALSE,
		//.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = 1.4287*BASEGAIN,
		.ana_gain_max = 64*BASEGAIN,
		.dig_gain_min = 1*BASEGAIN,
		.dig_gain_max = 1*BASEGAIN,
		.dig_gain_step = 4,
		.csi_param = {
			.cphy_settle = 73,
		},
		.dpc_enabled = true, /* reg 0x0b06 */
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = imx882_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(imx882_slim_video_setting),
		//.seamless_switch_group = PARAM_UNDEFINED,
		//.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		//.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		//.hdr_mode = HDR_NONE,
		//.raw_cnt = 1,
		//.exp_cnt = 1,
		.pclk = 878400000,
		.linelength = 7500,
		.framelength = 3900,
		.max_framerate = 300,
		.mipi_pixel_rate = 800000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.min_exposure_line = 6,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,

		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = 1.4287*BASEGAIN,
		.ana_gain_max = 64*BASEGAIN,
		.dig_gain_min = 1*BASEGAIN,
		.dig_gain_max = 1*BASEGAIN,
		.dig_gain_step = 4,
		.csi_param = {
			.cphy_settle = 73,
		},
		.dpc_enabled = true, /* reg 0x0b06 */
	},
	{//custom1  (2048x1152)@240fps
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = addr_data_pair_custom1,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom1),
		.pclk = 806400000,
		.linelength = 2468,
		.framelength = 1350,
		.max_framerate = 2400,
		.mipi_pixel_rate = 800000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.min_exposure_line = 8,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = 1.4287*BASEGAIN,
		.ana_gain_max = 64*BASEGAIN,
		.dig_gain_min = 1*BASEGAIN,
		.dig_gain_max = 1*BASEGAIN,
		.dig_gain_step = 4,
		.csi_param = {
			.cphy_settle = 73,
		},
		.dpc_enabled = true, /* reg 0x0b06 */
	},
	{//custom2  full crop
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = addr_data_pair_custom2,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom2),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = imx882_seamless_custom2,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx882_seamless_custom2),
		//.hdr_mode = HDR_NONE,
		//.raw_cnt = 1,
		//.exp_cnt = 1,
		.pclk = 878400000,
		.linelength = 8960,
		.framelength = 3252,
		.max_framerate = 300,
		.mipi_pixel_rate = 800000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.min_exposure_line = 10,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 10,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 2048,
			.y0_offset = 1536,
			.w0_size = 4096,
			.h0_size = 3072,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_cus2_info,
		.ae_binning_ratio = 1000,
		.ana_gain_min = 1*BASEGAIN,
		.ana_gain_max = 16*BASEGAIN,
		.dig_gain_min = 1*BASEGAIN,
		.dig_gain_max = 1*BASEGAIN,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 73,
		},
		.dpc_enabled = true, /* reg 0x0b06 */
	},
	{//custom3  (4096x2304)FHD@60fps
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = addr_data_pair_custom3,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom3),
		.pclk = 804000000,
		.linelength = 4984,
		.framelength = 2684,
		.max_framerate = 600,
		.mipi_pixel_rate = 800000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 5,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 5,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_cus3_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = 1.4287*BASEGAIN,
		.ana_gain_max = 64*BASEGAIN,
		.dig_gain_min = 1*BASEGAIN,
		.dig_gain_max = 1*BASEGAIN,
		.dig_gain_step = 4,
		.csi_param = {
			.cphy_settle = 73,
		},
		.dpc_enabled = true, /* reg 0x0b06 */
	},
	{//custom4  (8192x6144)50M@14fps
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = addr_data_pair_custom4,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom4),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 878400000,
		.linelength = 9968,
		.framelength = 6240,
		.max_framerate = 140,
		.mipi_pixel_rate = 800000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.min_exposure_line = 10,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 10,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 8192,
			.scale_h = 6144,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 8192,
			.h1_size = 6144,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 8192,
			.h2_tg_size = 6144,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = NULL,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = 1*BASEGAIN,
		.ana_gain_max = 16*BASEGAIN,
		.dig_gain_min = 1*BASEGAIN,
		.dig_gain_max = 1*BASEGAIN,
		.dig_gain_step = 4,
		.csi_param = {
			.cphy_settle = 73,
		},
		.dpc_enabled = true, /* reg 0x0b06 */
	},
	{//custom5  2048x1536@30fps
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = addr_data_pair_custom5,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom5),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 878400000,
		.linelength = 4024,
		.framelength = 7272,
		.max_framerate = 300,
		.mipi_pixel_rate = 800000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 8,
		.coarse_integ_step = 8,
		.min_exposure_line = 12,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 12,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 2048,
			.scale_h = 1536,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1536,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1536,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = 1.4287*BASEGAIN,
		.ana_gain_max = 64*BASEGAIN,//36dB
		.dig_gain_min = 1*BASEGAIN,
		.dig_gain_max = 1*BASEGAIN,
		.dig_gain_step = 4,
		.csi_param = {
			.cphy_settle = 73,
		},
		.dpc_enabled = true, /* reg 0x0b06 */
	}
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = MOT_NICE_IMX882_SENSOR_ID,
	.reg_addr_sensor_id = {0x0016, 0x0017},
	.i2c_addr_table = {0x20, 0xFF},
	//.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	//.eeprom_info = eeprom_info,
	//.eeprom_num = ARRAY_SIZE(eeprom_info),
	.eeprom_info = 0,
	.eeprom_num = 0,
	.resolution = {8192, 6144},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1.4287,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 0,
	.ana_gain_step = 1,
	.ana_gain_table = imx882_ana_gain_table,
	.ana_gain_table_size = sizeof(imx882_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 6,
	.exposure_max = 128 * (0xFFFF - 56),
	.exposure_step = 4, //4
	.exposure_margin = 56,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = BASE_DGAIN * 1,
	.dig_gain_step = 4,

	.frame_length_max = 0xFFC7,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 2315000,
	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,

	//.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
	//.hdr_type = HDR_SUPPORT_STAGGER_FDOL,
	//.seamless_switch_support = TRUE,
	//.seamless_switch_type = SEAMLESS_SWITCH_CUT_VB_INIT_SHUT,
	//.seamless_switch_hw_re_init_time_ns = 2750000,
	//.seamless_switch_prsh_hw_fixed_value = 32,
	//.seamless_switch_prsh_length_lc = 0,
	//.reg_addr_prsh_length_lines = {0x3058, 0x3059, 0x305a, 0x305b},
	//.reg_addr_prsh_mode = 0x3056,

	//.temperature_support = TRUE,

	//.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,
	//.s_cali = set_sensor_cali,

	.s_cali = mot_imx882_apply_qsc_spc_data,
	.seamless_switch_support = TRUE,
	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {
			{0x0202, 0x0203},
	},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x3160,
	.reg_addr_ana_gain = {
			{0x0204, 0x0205},
	},
	.reg_addr_dig_gain = {
			{0x020E, 0x020F},
	},
	.reg_addr_frame_length = {0x0340, 0x0341},
	//.reg_addr_temp_en = 0x0138,
	//.reg_addr_temp_read = 0x013A,
	.reg_addr_auto_extend = 0x0,
	.reg_addr_frame_count = 0x0005,
	.reg_addr_fast_mode = 0x3010,

	.init_setting_table = imx882_init_setting,
	.init_setting_len = ARRAY_SIZE(imx882_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	//.chk_s_off_sta = 1,
	//.chk_s_off_end = 0,

	.checksum_value = 0xAF3E324F,

	/*.ebd_info = {
		.frm_cnt_loc = {
			.loc_line = 1,
			.loc_pix = {7},
		},
		.coarse_integ_loc = {
			{  // NE
				.loc_line = 1,
				.loc_pix = {47, 49},
			},
			{  // ME
				.loc_line = 2,
				.loc_pix = {105, 107},
			},
			{  // SE
				.loc_line = 1,
				.loc_pix = {73, 75},
			},
		},
		.ana_gain_loc = {
			{  // NE
				.loc_line = 1,
				.loc_pix = {51, 53},
			},
			{  // ME
				.loc_line = 2,
				.loc_pix = {109, 111},
			},
			{  // SE
				.loc_line = 1,
				.loc_pix = {63, 65},
			},
		},
		.dig_gain_loc = {
			{  // NE
				.loc_line = 1,
				.loc_pix = {57, 59},
			},
			{  // ME
				.loc_line = 2,
				.loc_pix = {113, 115},
			},
			{  // SE
				.loc_line = 1,
				.loc_pix = {67, 69},
			},
		},
		.coarse_integ_shift_loc = {
			.loc_line = 2,
			.loc_pix = {97},
		},
		.dol_loc = {
			.loc_line = 2,
			.loc_pix = {145, 147}, // dol_en and dol_mode
		},
		.framelength_loc = {
			.loc_line = 1,
			.loc_pix = {111, 113},
		},
		.temperature_loc = {
			.loc_line = 1,
			.loc_pix = {37},
		},
	},*/
};

static struct subdrv_ops ops = {
	.get_id = common_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = common_open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.vsync_notify = vsync_notify,
	.update_sof_cnt = common_update_sof_cnt,
	.parse_ebd_line = common_parse_ebd_line,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_AVDD, 2800000, 6},
	{HW_ID_DVDD, 1100000, 6},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 1},
	{HW_ID_RST, 1, 5}
};

const struct subdrv_entry mot_nice_imx882_mipi_raw_entry = {
	.name = "mot_nice_imx882_mipi_raw",
	.id = MOT_NICE_IMX882_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	if (en)
		set_i2c_buffer(ctx, 0x0104, 0x01);
	else
		set_i2c_buffer(ctx, 0x0104, 0x00);
}

static u16 get_gain2reg(u32 gain)
{//Should be multiple of 4
	u32 regGain = (16384 - (16384 * BASEGAIN) / gain);
	regGain = (regGain+2)/4*4;
	return regGain;
}

static int imx882_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);

	if (mode != ctx->test_pattern) {
		if (mode == 0) {
			subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /* No pattern */
		} else {
			subdrv_i2c_wr_u8(ctx, 0x0601, 0x01);/* Solid black */
		}
	}

	ctx->test_pattern = mode;
	return ERROR_NONE;
}

static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_prsh_mode, 0x00);
		set_i2c_buffer(ctx, 0x3010, 0x00);
		commit_i2c_buffer(ctx);
	}
	return 0;
}

static int imx882_get_min_shutter_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id,
		u64 *min_shutter, u64 *exposure_step)
{
	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOG(ctx, "invalid cur_sid:%u, mode_num:%u set default\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = 0;
	}

	if (ctx->s_ctx.mode[scenario_id].min_exposure_line) {
		*min_shutter = ctx->s_ctx.mode[scenario_id].min_exposure_line;
	} else {
		*min_shutter = ctx->s_ctx.exposure_min;
	}

	if (ctx->s_ctx.mode[scenario_id].coarse_integ_step) {
		*exposure_step = ctx->s_ctx.mode[scenario_id].coarse_integ_step;
	} else {
		*exposure_step = ctx->s_ctx.exposure_step;
	}
	DRV_LOG(ctx, "scenario_id:%d, min shutter:%llu, exp_step:%llu",
				scenario_id, *min_shutter, *exposure_step);
	return ERROR_NONE;
}

static int imx882_get_min_shutter(struct subdrv_ctx *ctx, u8 *feature_para, u32 *feature_para_len)
{
	u64 *feature_data = (u64 *) feature_para;
	return imx882_get_min_shutter_by_scenario(ctx,
			(enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
			feature_data + 1, feature_data + 2);
}

#if  ENABLE_IMX882_LONG_EXPOSURE
static void imx882_set_long_exposure(struct subdrv_ctx *ctx)
{
	u32 shutter = ctx->exposure[IMGSENSOR_STAGGER_EXPOSURE_LE];
	u32 l_shutter = 0;
	u16 l_shift = 0;

	if (shutter > (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin)) {
		if (ctx->s_ctx.long_exposure_support == FALSE) {
			DRV_LOGE(ctx, "sensor no support of exposure lshift!\n");
			return;
		}
		if (ctx->s_ctx.reg_addr_exposure_lshift == PARAM_UNDEFINED) {
			DRV_LOGE(ctx, "please implement lshift register address\n");
			return;
		}
		for (l_shift = 1; l_shift < 7; l_shift++) {
			l_shutter = ((shutter - 1) >> l_shift) + 1;
			if (l_shutter < (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin))
				break;
		}
		if (l_shift > 7) {
			DRV_LOGE(ctx, "unable to set exposure:%u, set to max\n", shutter);
			l_shift = 7;
		}
		shutter = ((shutter - 1) >> l_shift) + 1;
		ctx->frame_length = shutter + ctx->s_ctx.exposure_margin;
		DRV_LOG(ctx, "long exposure mode: lshift %u times, shutter:%u\n", l_shift, shutter);
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_exposure_lshift, l_shift);
		set_i2c_buffer(ctx, 0x301C, 0);
		ctx->l_shift = l_shift;
		/* Frame exposure mode customization for LE*/
		ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		ctx->current_ae_effective_frame = 2;
	} else {
		if (ctx->s_ctx.reg_addr_exposure_lshift != PARAM_UNDEFINED) {
			set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_exposure_lshift, l_shift);
			set_i2c_buffer(ctx, 0x301C, 1);
			ctx->l_shift = l_shift;
		}
		ctx->current_ae_effective_frame = 2;
	}
	ctx->exposure[IMGSENSOR_STAGGER_EXPOSURE_LE] = shutter;
}

static void imx882_set_shutter_frame_length(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length)
{
	int fine_integ_line = 0;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	DRV_LOG(ctx, "shutter =%lld \n", shutter);
	ctx->frame_length = frame_length ? frame_length : ctx->min_frame_length;
	check_current_scenario_id_bound(ctx);
	/* check boundary of shutter */
	fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	shutter = FINE_INTEG_CONVERT(shutter, fine_integ_line);
	shutter = max_t(u64, shutter,
		(u64)ctx->s_ctx.mode[ctx->current_scenario_id].min_exposure_line);
	shutter = min_t(u64, shutter,
		(u64)ctx->s_ctx.mode[ctx->current_scenario_id].multi_exposure_shutter_range[0].max);
	/* check boundary of framelength */
	ctx->frame_length = max((u32)shutter + ctx->s_ctx.exposure_margin, ctx->min_frame_length);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	/* restore shutter */
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	ctx->exposure[0] = (u32) shutter;
	/* group hold start */
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);
	/* write shutter */
	imx882_set_long_exposure(ctx);
	/* enable auto extend */
	if (ctx->s_ctx.reg_addr_auto_extend)
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_auto_extend, 0x01);
	/* write framelength */
	if (set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend){
		write_frame_length(ctx, ctx->frame_length);
	}
	if (ctx->s_ctx.reg_addr_exposure[0].addr[2]) {
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[0],
			(ctx->exposure[0] >> 16) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[1],
			(ctx->exposure[0] >> 8) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[2],
			ctx->exposure[0] & 0xFF);
	} else {
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[0],
			(ctx->exposure[0] >> 8) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[1],
			ctx->exposure[0] & 0xFF);
	}
	DRV_LOG(ctx, "exp[0x%x], fll(input/output):%u/%u, flick_en:%d\n",
		ctx->exposure[0], frame_length, ctx->frame_length, ctx->autoflicker_en);
	if (!ctx->ae_ctrl_gph_en) {
		if (gph)
			ctx->s_ctx.s_gph((void *)ctx, 0);
		commit_i2c_buffer(ctx);
	}
	/* group hold end */
}

static int imx882_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 shutter = *((u64 *)para);
	imx882_set_shutter_frame_length(ctx, shutter,0);
	return 0;
}
#endif

static int imx882_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	u64 *feature_data = (u64 *)para;
	u32 exp_cnt = 0;

	if (feature_data == NULL) {
		DRV_LOG(ctx, "input scenario is null!");
		return ERROR_NONE;
	}
	scenario_id = *feature_data;

	if ((feature_data + 1) != NULL)
		ae_ctrl = (struct mtk_hdr_ae *)((uintptr_t)(*(feature_data + 1)));
	else
		DRV_LOG(ctx, "no ae_ctrl input");

	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, "E: set seamless switch %u %u\n", ctx->current_scenario_id, scenario_id);
	if (!ctx->extend_frame_length_en)
		DRV_LOG(ctx, "please extend_frame_length before seamless_switch!\n");
	ctx->extend_frame_length_en = FALSE;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOG(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		return ERROR_NONE;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_group == 0 ||
		ctx->s_ctx.mode[scenario_id].seamless_switch_group !=
			ctx->s_ctx.mode[ctx->current_scenario_id].seamless_switch_group) {
		DRV_LOG(ctx, "seamless_switch not supported\n");
		return ERROR_NONE;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table == NULL) {
		DRV_LOG(ctx, "Please implement seamless_switch setting\n");
		return ERROR_NONE;
	}

	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	ctx->is_seamless = TRUE;


	update_mode_info(ctx, scenario_id);
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x02);
	i2c_table_write(ctx,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);

	if (ae_ctrl) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_DCG_RAW:
			set_shutter(ctx, ae_ctrl->exposure.le_exposure);
			if (ctx->s_ctx.mode[scenario_id].dcg_info.dcg_gain_mode
				== IMGSENSOR_DCG_DIRECT_MODE)
				set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			else
				set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		default:
			set_shutter(ctx, ae_ctrl->exposure.le_exposure);
			set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		}
	}
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	DRV_LOG_MUST(ctx, "X: set seamless switch done\n");
	return ERROR_NONE;
}
