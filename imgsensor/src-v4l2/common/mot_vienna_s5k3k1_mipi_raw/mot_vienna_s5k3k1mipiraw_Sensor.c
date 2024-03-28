// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 mot_vienna_s5k3k1mipiraw_Sensor.c
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
#include "mot_vienna_s5k3k1mipiraw_Sensor.h"

static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int mot_vienna_s5k3k1_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int mot_vienna_s5k3k1_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static void mot_vienna_s5k3k1_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);

#define ENABLE_S5K3K1_PD TRUE
#define S5K3K1_PD_DT 0x2b
#define S5K3K1_DATA_DESC VC_PDAF_STATS
#define S5K3K1_PD_X_SIZE 456
#define S5K3K1_PD_Y_SIZE 680

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, mot_vienna_s5k3k1_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, mot_vienna_s5k3k1_set_test_pattern_data},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0xe40,
			.vsize = 0xab0,
		},
	},
#ifdef ENABLE_S5K3K1_PD
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = S5K3K1_PD_DT,
			.hsize = S5K3K1_PD_X_SIZE,
			.vsize = S5K3K1_PD_Y_SIZE,
			.user_data_desc = S5K3K1_DATA_DESC,
		},
	}
#endif
};
/*static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0xe40,
			.vsize = 0xab0,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0xe40,
			.vsize = 0xab0,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0xe40,
			.vsize = 0xab0,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0xe40,
			.vsize = 0xab0,
		},
	},
};*/

#ifdef ENABLE_S5K3K1_PD
static struct SET_PD_BLOCK_INFO_T s5k3k1_pd_info = {
	.i4OffsetX = 0,
	.i4OffsetY = 8,
	.i4PitchX = 8,
	.i4PitchY = 32,
	.i4PairNum = 4,
	.i4SubBlkW = 8,
	.i4SubBlkH = 8,
	.i4PosL = {
		{2, 9}, {6, 21}, {2, 29}, {6, 33}
	},
	.i4PosR = {
		{2, 13}, {6, 17}, {2, 25}, {6, 37}
	},
	.i4BlockNumX = 456,
	.i4BlockNumY = 85,
	.i4Crop = {
		// <pre> <cap> <normal_video> <hs_video> <<slim_video>>
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// <<cust1>> <<cust2>> <<cust3>> <cust4> <cust5>
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// <cust6> <cust7> <cust8> cust9 cust10
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// cust11 cust12 cust13 <cust14> <cust15>
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// <cust16> <cust17> cust18 <cust19> cust20
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// <cust21> <cust22> <cust23> <cust24> <cust25>
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// cust26 <cust27> cust28
		{0, 0}, {0, 0}, {0, 0},
	},
	.i4FullRawW = 3648,
	.i4FullRawH = 2736,
	.iMirrorFlip = 0,
	.PDAF_Support = PDAF_SUPPORT_CAMSV,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		.i4PDPattern = 3, // sparse PD non-interleaved
		.i4PDRepetition = 8,
		.i4PDOrder = { 0, 1, 1, 0, 1, 0, 0, 1 }, // L = 0, R = 1
	},
};
#endif

static struct subdrv_mode_struct mode_struct[] = {
	{//preview
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = preview_setting_array,
		.mode_setting_len = ARRAY_SIZE(preview_setting_array),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 796000000,
		.linelength = 5760,
		.framelength = 4602,
		.max_framerate = 300,
		.mipi_pixel_rate = 576000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3648,
			.full_h = 2736,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3648,
			.h0_size = 2736,
			.scale_w = 3648,
			.scale_h = 2736,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3648,
			.h1_size = 2736,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3648,
			.h2_tg_size = 2736,
		},
#ifdef ENABLE_S5K3K1_PD
		.pdaf_cap = ENABLE_S5K3K1_PD,
		.imgsensor_pd_info = &s5k3k1_pd_info,
#else
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
#endif
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//capture
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = preview_setting_array,
		.mode_setting_len = ARRAY_SIZE(preview_setting_array),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 796000000,
		.linelength = 5760,
		.framelength = 4602,
		.max_framerate = 300,
		.mipi_pixel_rate = 576000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3648,
			.full_h = 2736,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3648,
			.h0_size = 2736,
			.scale_w = 3648,
			.scale_h = 2736,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3648,
			.h1_size = 2736,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3648,
			.h2_tg_size = 2736,
		},
#ifdef ENABLE_S5K3K1_PD
		.pdaf_cap = ENABLE_S5K3K1_PD,
		.imgsensor_pd_info = &s5k3k1_pd_info,
#else
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
#endif
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//normal_video
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = preview_setting_array,
		.mode_setting_len = ARRAY_SIZE(preview_setting_array),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 796000000,
		.linelength = 5760,
		.framelength = 4602,
		.max_framerate = 300,
		.mipi_pixel_rate = 576000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3648,
			.full_h = 2736,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3648,
			.h0_size = 2736,
			.scale_w = 3648,
			.scale_h = 2736,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3648,
			.h1_size = 2736,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3648,
			.h2_tg_size = 2736,
		},
#ifdef ENABLE_S5K3K1_PD
		.pdaf_cap = ENABLE_S5K3K1_PD,
		.imgsensor_pd_info = &s5k3k1_pd_info,
#else
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
#endif
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//hs_video
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = preview_setting_array,
		.mode_setting_len = ARRAY_SIZE(preview_setting_array),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 796000000,
		.linelength = 5760,
		.framelength = 4602,
		.max_framerate = 300,
		.mipi_pixel_rate = 576000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3648,
			.full_h = 2736,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3648,
			.h0_size = 2736,
			.scale_w = 3648,
			.scale_h = 2736,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3648,
			.h1_size = 2736,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3648,
			.h2_tg_size = 2736,
		},
#ifdef ENABLE_S5K3K1_PD
		.pdaf_cap = ENABLE_S5K3K1_PD,
		.imgsensor_pd_info = &s5k3k1_pd_info,
#else
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
#endif
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//slim_video
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = preview_setting_array,
		.mode_setting_len = ARRAY_SIZE(preview_setting_array),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 796000000,
		.linelength = 5760,
		.framelength = 4602,
		.max_framerate = 300,
		.mipi_pixel_rate = 576000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3648,
			.full_h = 2736,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3648,
			.h0_size = 2736,
			.scale_w = 3648,
			.scale_h = 2736,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3648,
			.h1_size = 2736,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3648,
			.h2_tg_size = 2736,
		},
#ifdef ENABLE_S5K3K1_PD
		.pdaf_cap = ENABLE_S5K3K1_PD,
		.imgsensor_pd_info = &s5k3k1_pd_info,
#else
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
#endif
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = MOT_VIENNA_S5K3K1_SENSOR_ID,
	.reg_addr_sensor_id = {0x0000, 0x0001},
	.i2c_addr_table = {0x20, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info = PARAM_UNDEFINED,
	.eeprom_num = PARAM_UNDEFINED,
	.resolution = {3648, 2736},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_type = 2,
	.ana_gain_step = 2,
	.ana_gain_table = mot_vienna_s5k3k1_ana_gain_table,
	.ana_gain_table_size = sizeof(mot_vienna_s5k3k1_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 9,
	.exposure_max = (0xFFFF*128) - 20,
	.exposure_step = 1,
	.exposure_margin = 20,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 500000, // 05.11.23 (m.d.y) update

#ifdef ENABLE_S5K3K1_PD
	.pdaf_type = PDAF_SUPPORT_CAMSV,
#else
	.pdaf_type = PDAF_SUPPORT_NA,
#endif
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,
	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {{0x0202, 0x0203},},
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x0204, 0x0205},},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = 0x0005,

	.init_setting_table = PARAM_UNDEFINED,
	.init_setting_len = PARAM_UNDEFINED,
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,

	.checksum_value = 0xE4087030,
};

static struct subdrv_ops ops = {
	.get_id = common_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_RST, 0, 1},
	{HW_ID_MCLK, 24, 0},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_AVDD, 2800000, 1}, // pmic_ldo for avdd
	{HW_ID_DVDD, 1050000, 1}, // pmic_ldo for dvdd
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 0},
	{HW_ID_RST, 1, 5}
};

const struct subdrv_entry mot_vienna_s5k3k1_mipi_raw_entry = {
	.name = "mot_vienna_s5k3k1_mipi_raw",
	.id = MOT_VIENNA_S5K3K1_SENSOR_ID,
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
{
	return gain * 32 / BASEGAIN;
}

static int mot_vienna_s5k3k1_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	if (mode)
		subdrv_i2c_wr_u16(ctx, 0x0600, mode); /*100% Color bar*/
	else if (ctx->test_pattern)
		subdrv_i2c_wr_u16(ctx, 0x0600, 0x0000); /*No pattern*/

	ctx->test_pattern = mode;
	return ERROR_NONE;
}

static int mot_vienna_s5k3k1_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct mtk_test_pattern_data *data = (struct mtk_test_pattern_data *)para;
	u16 R = (data->Channel_R >> 22) & 0x3ff;
	u16 Gr = (data->Channel_Gr >> 22) & 0x3ff;
	u16 Gb = (data->Channel_Gb >> 22) & 0x3ff;
	u16 B = (data->Channel_B >> 22) & 0x3ff;

	subdrv_i2c_wr_u16(ctx, 0x0602, R);
	subdrv_i2c_wr_u16(ctx, 0x0604, Gr);
	subdrv_i2c_wr_u16(ctx, 0x0606, B);
	subdrv_i2c_wr_u16(ctx, 0x0608, Gb);

	DRV_LOG(ctx, "mode(%u) R/Gr/Gb/B = 0x%04x/0x%04x/0x%04x/0x%04x\n",
		ctx->test_pattern, R, Gr, Gb, B);
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

static void mot_vienna_s5k3k1_sensor_init(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "E\n");
	subdrv_i2c_wr_u16(ctx, 0x6028, 0x2000);
	subdrv_i2c_wr_u16(ctx, 0x0000, 0x0002);
	subdrv_i2c_wr_u16(ctx, 0x0000, 0x30B1);
	subdrv_i2c_wr_u16(ctx, 0x6010, 0x0001);
	mdelay(5);
	subdrv_i2c_wr_regs_u16(ctx, sensor_init_setting_array1,
		ARRAY_SIZE(sensor_init_setting_array1));

	/* set pdaf DT to 0x2B */
	//subdrv_i2c_wr_u8(ctx, 0x0116, 0x2B);
	DRV_LOG(ctx, "X\n");
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (common_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	mot_vienna_s5k3k1_sensor_init(ctx);

	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	memset(ctx->ana_gain, 0, sizeof(ctx->gain));
	ctx->exposure[0] = ctx->s_ctx.exposure_def;
	ctx->ana_gain[0] = ctx->s_ctx.ana_gain_def;
	ctx->current_scenario_id = scenario_id;
	ctx->pclk = ctx->s_ctx.mode[scenario_id].pclk;
	ctx->line_length = ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length = ctx->s_ctx.mode[scenario_id].framelength;
	ctx->current_fps = 10 * ctx->pclk / ctx->line_length / ctx->frame_length;
	ctx->readout_length = ctx->s_ctx.mode[scenario_id].readout_length;
	ctx->read_margin = ctx->s_ctx.mode[scenario_id].read_margin;
	ctx->min_frame_length = ctx->frame_length;
	ctx->autoflicker_en = FALSE;
	ctx->test_pattern = 0;
	ctx->ihdr_mode = 0;
	ctx->pdaf_mode = 0;
	ctx->hdr_mode = 0;
	ctx->extend_frame_length_en = 0;
	ctx->is_seamless = 0;
	ctx->fast_mode_on = 0;
	ctx->sof_cnt = 0;
	ctx->ref_sof_cnt = 0;
	ctx->is_streaming = 0;

	return ERROR_NONE;
} /* open */
