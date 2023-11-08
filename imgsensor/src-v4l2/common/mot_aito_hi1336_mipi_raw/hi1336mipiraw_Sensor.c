// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 hi1336mipiraw_Sensor.c
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
#include "hi1336mipiraw_Sensor.h"

//static void set_sensor_cali(void *arg);
//static int get_sensor_temperature(void *arg);
//static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int hi1336_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
//static int hi1336_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, hi1336_set_test_pattern},
};

#if 0
static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x010B00FF,
		.addr_header_id = 0x0000000B,
		.i2c_write_id = 0xA0,
	},
};



static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 56,
	.i4OffsetY = 24,
	.i4PitchX = 32,
	.i4PitchY = 32,
	.i4PairNum = 8,
	.i4SubBlkW = 16,
	.i4SubBlkH = 8,
	.i4PosL = {{58,27},{74,27},{66,39},{82,39},{58,43},{74,43},{66,55},{82,55}},
	.i4PosR = {{58,31},{74,31},{66,35},{82,35},{58,47},{74,47},{66,51},{82,51}},
	.i4BlockNumX = 128,
	.i4BlockNumY = 96,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 376}, {0, 0}, {0, 0},
		{0, 0}
	},
	.iMirrorFlip = 0,
};

#endif

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1070,
			.vsize = 0x0c30,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1070,
			.vsize = 0x0c30,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1070,
			.vsize = 0x0c30,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1070,
			.vsize = 0x0c30,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1070,
			.vsize = 0x0c30,
		},
	},
};


static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = hi1336_preview_setting,
		.mode_setting_len = ARRAY_SIZE(hi1336_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 600000000,
		.linelength = 6004,
		.framelength = 3330,
		.max_framerate = 300,
		.mipi_pixel_rate = 600000000,
		.readout_length = 0,
		.read_margin = 10, //need check
		.imgsensor_winsize_info = {
			.full_w = 4208,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4208,
			.h0_size = 3120,
			.scale_w = 4208,
			.scale_h = 3120,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4208,
			.h1_size = 3120,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4208,
			.h2_tg_size = 3120,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = hi1336_capture_setting,
		.mode_setting_len = ARRAY_SIZE(hi1336_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 600000000,
		.linelength = 6004,
		.framelength = 3330,
		.max_framerate = 300,
		.mipi_pixel_rate = 600000000,
		.readout_length = 0,
		.read_margin = 10,
		.imgsensor_winsize_info = {
			.full_w = 4208,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4208,
			.h0_size = 3120,
			.scale_w = 4208,
			.scale_h = 3120,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4208,
			.h1_size = 3120,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4208,
			.h2_tg_size = 3120,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = hi1336_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(hi1336_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 600000000,
		.linelength = 6004,
		.framelength = 3330,
		.max_framerate = 300,
		.mipi_pixel_rate = 600000000,
		.readout_length = 0,
		.read_margin = 10,
		.imgsensor_winsize_info = {
			.full_w = 4208,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 376,
			.w0_size = 4208,
			.h0_size = 2368,
			.scale_w = 4208,
			.scale_h = 2368,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4208,
			.h1_size = 2368,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4208,
			.h2_tg_size = 2368,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = hi1336_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(hi1336_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 600000000,
		.linelength = 6004,
		.framelength = 3316,
		.max_framerate = 300,
		.mipi_pixel_rate = 566400000,
		.readout_length = 0,
		.read_margin = 10,
		.imgsensor_winsize_info = {
			.full_w = 4208,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4208,
			.h0_size = 3120,
			.scale_w = 4208,
			.scale_h = 3120,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4208,
			.h1_size = 3120,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4208,
			.h2_tg_size = 3120,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = hi1336_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(hi1336_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 600000000,
		.linelength = 6004,
		.framelength = 3316,
		.max_framerate = 300,
		.mipi_pixel_rate = 566400000,
		.readout_length = 0,
		.read_margin = 10,
		.imgsensor_winsize_info = {
			.full_w = 4208,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4208,
			.h0_size = 3120,
			.scale_w = 4208,
			.scale_h = 3120,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4208,
			.h1_size = 3120,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4208,
			.h2_tg_size = 3120,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},

};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = MOT_AITO_HI1336_SENSOR_ID,
	.reg_addr_sensor_id = {0x0716,0x0717},
	.i2c_addr_table = {0x40, 0xff},
	.i2c_burst_write_support = FALSE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info = 0,
	.eeprom_num = 0,
	.resolution = {4208, 3120},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_type = 3,
	.ana_gain_step = 4,// no used
	.ana_gain_table = 0,
	.ana_gain_table_size = 0,
	.min_gain_iso = 100, // no change
	.exposure_def = 0x3D0, // no change
	.exposure_min = 4,
	.exposure_max = 24983,
	.exposure_step = 4,              // Get Maximum Step ,  need check
	.exposure_margin = 10,

	.frame_length_max = 0xFFFF,
	.frame_time_delay_frame = 2,

	.pdaf_type = PDAF_SUPPORT_NA,  //need check
	.hdr_type = HDR_SUPPORT_NA,    //check this sensor is stagger or not.
	.seamless_switch_support = 0,
	.temperature_support = 0,

	.g_temp = 0,
	.g_gain2reg = get_gain2reg,
	.s_gph = 0,
	.s_cali = 0,

	.reg_addr_stream = 0x0b00,
	.reg_addr_mirror_flip = 0,
	.reg_addr_exposure = {
			{0x020a, 0x020B},
	},

	.long_exposure_support = 0,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {
			{0x0212, 0x0213},
	},

	.reg_addr_frame_length = {0x020e, 0x020F},

	.reg_addr_temp_en = 0,
	.reg_addr_temp_read = 0,
	.reg_addr_auto_extend = 0,
	.reg_addr_frame_count = 0,
	.reg_addr_fast_mode = 0,

	.init_setting_table = hi1336_init_setting,
	.init_setting_len = ARRAY_SIZE(hi1336_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.checksum_value = 0x30a07776,
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
	.get_csi_param = common_get_csi_param,
	.vsync_notify = vsync_notify,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 1},
	{HW_ID_AVDD, 2800000, 1},
	{HW_ID_DVDD, 1100000, 1},
	{HW_ID_DOVDD, 1800000, 2},
	{HW_ID_RST, 1, 2},
};

const struct subdrv_entry mot_aito_hi1336_mipi_raw_entry = {
	.name = "mot_aito_hi1336_mipi_raw",
	.id = MOT_AITO_HI1336_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */
#if 0
static void set_sensor_cali(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	u16 idx = 0;
	u8 support = FALSE;
	u8 *pbuf = NULL;
	u16 size = 0;
	u16 addr = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	if (!probe_eeprom(ctx))
		return;

	idx = ctx->eeprom_index;

	/* QSC data */
	support = info[idx].qsc_support;
	pbuf = info[idx].preload_qsc_table;
	size = info[idx].qsc_size;
	addr = info[idx].sensor_reg_addr_qsc;
	if (support) {
		if (pbuf != NULL && addr > 0 && size > 0) {
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
			subdrv_i2c_wr_u8(ctx, 0x3206, 0x01);
			DRV_LOG(ctx, "set QSC calibration data done.");
		} else {
			subdrv_i2c_wr_u8(ctx, 0x3206, 0x00);
		}
	}
}

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u8 temperature = 0;
	int temperature_convert = 0;

	temperature = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);

	if (temperature < 0x55)
		temperature_convert = temperature;
	else if (temperature < 0x80)
		temperature_convert = 85;
	else if (temperature < 0xED)
		temperature_convert = -20;
	else
		temperature_convert = (char)temperature;

	DRV_LOG(ctx, "temperature: %d degrees\n", temperature_convert);
	return temperature_convert;
}

static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	if (en)
		set_i2c_buffer(ctx, 0x0104, 0x01);
	else
		set_i2c_buffer(ctx, 0x0104, 0x00);
}
#endif

static u16 get_gain2reg(u32 gain)
{
	return (u16)gain/64 - 16;
}



static int hi1336_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
#if 0
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	switch (mode){
	case 5:
		subdrv_i2c_wr_u8(ctx, 0x0b04, 0x037d);
		subdrv_i2c_wr_u8(ctx, 0x0C0A, 0x0100);
		break;
	default:
		subdrv_i2c_wr_u8(ctx, 0x0b04, 0x037c);
		subdrv_i2c_wr_u8(ctx, 0x0C0A, 0x0000);
	break;
	}
	if ((ctx->test_pattern) && (mode != ctx->test_pattern)) {
	   if (ctx->test_pattern == 5){
	       subdrv_i2c_wr_u8(ctx, 0x0b04, 0x037d);
		subdrv_i2c_wr_u8(ctx, 0x0C0A, 0x0100);
		}
	 else if (mode == 0){
		subdrv_i2c_wr_u8(ctx, 0x0b04, 0x037c);
		subdrv_i2c_wr_u8(ctx, 0x0C0A, 0x0000);
		}
	}
	ctx->test_pattern = mode;
#endif
	return ERROR_NONE;
}


#if 0
static int hi1336_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct mtk_test_pattern_data *data = (struct mtk_test_pattern_data *)para;
	u16 R = (data->Channel_R >> 22) & 0x3ff;
	u16 Gr = (data->Channel_Gr >> 22) & 0x3ff;
	u16 Gb = (data->Channel_Gb >> 22) & 0x3ff;
	u16 B = (data->Channel_B >> 22) & 0x3ff;

	subdrv_i2c_wr_u8(ctx, 0x0602, (R >> 8));
	subdrv_i2c_wr_u8(ctx, 0x0603, (R & 0xff));
	subdrv_i2c_wr_u8(ctx, 0x0604, (Gr >> 8));
	subdrv_i2c_wr_u8(ctx, 0x0605, (Gr & 0xff));
	subdrv_i2c_wr_u8(ctx, 0x0606, (B >> 8));
	subdrv_i2c_wr_u8(ctx, 0x0606, (B & 0xff));
	subdrv_i2c_wr_u8(ctx, 0x0608, (Gb >> 8));
	subdrv_i2c_wr_u8(ctx, 0x0608, (Gb & 0xff));

	DRV_LOG(ctx, "mode(%u) R/Gr/Gb/B = 0x%04x/0x%04x/0x%04x/0x%04x\n",
		ctx->test_pattern, R, Gr, Gb, B);
	return ERROR_NONE;
}
#endif

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
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
		commit_i2c_buffer(ctx);
	}
	return 0;
}
