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


static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int imx882_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx882_get_min_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, imx882_set_test_pattern},
	{SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, imx882_get_min_shutter},
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
	.i4ModeIndex = 3,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = imx882_preview_setting,
		.mode_setting_len = ARRAY_SIZE(imx882_preview_setting),
		//.seamless_switch_group = 1,
		//.seamless_switch_mode_setting_table = imx882_seamless_preview,
		//.seamless_switch_mode_setting_len = ARRAY_SIZE(imx882_seamless_preview),
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
		.delay_frame = 3,
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
		.delay_frame = 3,
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
		.delay_frame = 3,
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
		.delay_frame = 3,
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
		.delay_frame = 3,
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
		.delay_frame = 1,
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
		//.seamless_switch_group = 1,
		//.seamless_switch_mode_setting_table = imx882_seamless_custom2,
		//.seamless_switch_mode_setting_len = ARRAY_SIZE(imx882_seamless_custom2),
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
	.frame_time_delay_frame = 3,
	.start_exposure_offset = 500000,
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
	.reg_addr_auto_extend = 0x0350,
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
	{HW_ID_AVDD, 2800000, 1},
	{HW_ID_DVDD, 1100000, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 1},
	{HW_ID_RST, 1, 3}
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
	/* 1:Solid Color 2:Color Bar 5:Black */
	switch (mode) {
	case 5:
		subdrv_i2c_wr_u8(ctx, 0x020E, 0x00); /* dig_gain = 0 */
		break;
	default:
		subdrv_i2c_wr_u8(ctx, 0x0601, mode);
		break;
	}

	if ((ctx->test_pattern) && (mode != ctx->test_pattern)) {
		if (ctx->test_pattern == 5)
			subdrv_i2c_wr_u8(ctx, 0x020E, 0x01);
		else if (mode == 0)
			subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /* No pattern */
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
