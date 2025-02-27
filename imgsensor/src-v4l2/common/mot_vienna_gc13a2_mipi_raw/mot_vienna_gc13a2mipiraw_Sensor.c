// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 gc13a2mipiraw_Sensor.c
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
#include "mot_vienna_gc13a2mipiraw_Sensor.h"

static int gc13a2_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc13a2_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc13a2_set_shutter_frame_length(struct subdrv_ctx *ctx,u8 *para, u32 *len);
static int gc13a2_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
#if ENABLE_GC13A2_FAST_STANDBY
static int gc13a2_streaming_control(struct subdrv_ctx *ctx, kal_bool enable);
static int gc13a2_set_fast_standby_stream_off(struct subdrv_ctx *ctx,u8 *para, u32 *len);
static int gc13a2_set_fast_standby_stream_on(struct subdrv_ctx *ctx,u8 *para, u32 *len);
static int gc13a2_ops_close(struct subdrv_ctx *ctx);
#endif

//#define MOT_GC13A2_PDAF_DEBUG
#ifdef MOT_GC13A2_PDAF_DEBUG
static unsigned int gc13a2_pd_dt = 0x32;
static unsigned int gc13a2_pd_ddesc = VC_PDAF_STATS_NE_PIX_1;
static unsigned int gc13a2_pd_en = 1;
module_param(gc13a2_pd_dt, uint, 0644);
module_param(gc13a2_pd_ddesc, uint, 0644);
module_param(gc13a2_pd_en, uint, 0644);
#endif

#define ENABLE_GC13A2_PD TRUE
#define GC13A2_PD_DT 0x32
//#define GC13A2_DATA_DESC VC_PDAF_STATS_NE_PIX_1
#define GC13A2_DATA_DESC VC_PDAF_STATS

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, gc13a2_set_test_pattern},
	{SENSOR_FEATURE_SET_GAIN, gc13a2_set_gain},
	{SENSOR_FEATURE_SET_ESHUTTER, gc13a2_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, gc13a2_set_shutter_frame_length},
#if ENABLE_GC13A2_FAST_STANDBY
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, gc13a2_set_fast_standby_stream_off},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, gc13a2_set_fast_standby_stream_on},
#endif
};

// 1000 base for dcg gain ratio
//static u32 gc13a2_dag_ratio_table_12bit[] = {4000};

static struct mtk_sensor_saturation_info imgsensor_saturation_info_10bit = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};

/*
static struct mtk_sensor_saturation_info imgsensor_saturation_info_12bit = {
	.gain_ratio = 4000,
	.OB_pedestal = 256,
	.saturation_level = 4095,
};
*/

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1040,  // 4160
			.vsize = 0x0C30,  // 3120
		},
	},
#ifdef ENABLE_GC13A2_PD
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = GC13A2_PD_DT,
			.hsize = 0x00a0,
			.vsize = 0x0300,
			//.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = GC13A2_DATA_DESC,
		},
	}
#endif
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1040,  // 4160
			.vsize = 0x0C30,  // 3120
		},
	},
#ifdef ENABLE_GC13A2_PD
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = GC13A2_PD_DT,
			.hsize = 0x00a0,
			.vsize = 0x0300,
			//.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = GC13A2_DATA_DESC,
		},
	}
#endif
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1040,  // 4160
			.vsize = 0x0C30,  // 3120
		},
	},
#ifdef ENABLE_GC13A2_PD
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = GC13A2_PD_DT,
			.hsize = 0x00a0,
			.vsize = 0x0300,
			//.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = GC13A2_DATA_DESC,
		},
	}
#endif
};
/*
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2c,
			.hsize = 0x1040,  // 4160
			.vsize = 0x0C30,  // 3120
		},
	},
};
*/

#ifdef ENABLE_GC13A2_PD
static struct SET_PD_BLOCK_INFO_T gc13a2_pd_info = {
	.i4OffsetX = 32,
	.i4OffsetY = 24,
	.i4PitchX = 64,
	.i4PitchY = 64,
	.i4PairNum = 16,
	.i4SubBlkW = 16,
	.i4SubBlkH = 16,
	.i4PosR = {
		{39, 28},		{55, 32},		{75, 32},		{91, 28},
		{43, 48},		{59, 52},		{71, 52},		{87, 48},
		{43, 64},		{59, 60},		{71, 60},		{87, 64},
		{39, 84},		{55, 80},		{75, 80},		{91, 84},
	},
	.i4PosL = {
		{39, 32},		{55, 36},		{75, 36},		{91, 32},
		{43, 44},		{59, 48},		{71, 48},		{87, 44},
		{43, 68},		{59, 64},		{71, 64},		{87, 68},
		{39, 80},		{55, 76},		{75, 76},		{91, 80},
	},
	.i4BlockNumX = 64,
	.i4BlockNumY = 48,
	.i4Crop = {
		// <pre> <cap> <normal_video> <hs_video> <<slim_video>>
		{0, 0}, {0, 0}, {0, 390}, {0, 0}, {0, 0},
		// <<cust1>> <<cust2>> <<cust3>> <cust4> <cust5>
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
	},
	.iMirrorFlip = 3,
	.i4VCPackNum = 2,
	.i4FullRawW = 4160,
	.i4FullRawH = 3120,
	.PDAF_Support = PDAF_SUPPORT_CAMSV,
	/* VC's PD pattern description */
};
#endif

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = gc13a2_4160x3120_data_raw10,
		.mode_setting_len = ARRAY_SIZE(gc13a2_4160x3120_data_raw10),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 412000000,
		.linelength = 4224,
		.framelength = 3248,
		.max_framerate = 300,
		.mipi_pixel_rate = 473600000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4160,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4160,
			.h0_size = 3120,
			.scale_w = 4160,
			.scale_h = 3120,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4160,
			.h1_size = 3120,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4160,
			.h2_tg_size = 3120,
		},
#ifdef ENABLE_GC13A2_PD
		.pdaf_cap = ENABLE_GC13A2_PD,
		.imgsensor_pd_info = &gc13a2_pd_info,
#endif
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 1,
		.csi_param = {
			.dphy_trail = 92,
		},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = gc13a2_4160x3120_data_raw10,
		.mode_setting_len = ARRAY_SIZE(gc13a2_4160x3120_data_raw10),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 412000000,
		.linelength = 4224,
		.framelength = 3248,
		.max_framerate = 300,
		.mipi_pixel_rate = 473600000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4160,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4160,
			.h0_size = 3120,
			.scale_w = 4160,
			.scale_h = 3120,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4160,
			.h1_size = 3120,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4160,
			.h2_tg_size = 3120,
		},
#ifdef ENABLE_GC13A2_PD
		.pdaf_cap = ENABLE_GC13A2_PD,
		.imgsensor_pd_info = &gc13a2_pd_info,
#endif
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 1,
		.csi_param = {
			.dphy_trail = 92,
		},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = gc13a2_4160x3120_data_raw10,
		.mode_setting_len = ARRAY_SIZE(gc13a2_4160x3120_data_raw10),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 412000000,
		.linelength = 4224,
		.framelength = 3248,
		.max_framerate = 300,
		.mipi_pixel_rate = 473600000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4160,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4160,
			.h0_size = 3120,
			.scale_w = 4160,
			.scale_h = 3120,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4160,
			.h1_size = 3120,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4160,
			.h2_tg_size = 3120,
		},
#ifdef ENABLE_GC13A2_PD
		.pdaf_cap = ENABLE_GC13A2_PD,
		.imgsensor_pd_info = &gc13a2_pd_info,
#endif
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 1,
		.csi_param = {
			.dphy_trail  = 92,
		},
	},
	/*
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = gc13a2_4160x3120_data_raw12,
		.mode_setting_len = ARRAY_SIZE(gc13a2_4160x3120_data_raw12),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_RAW_DCG_COMPOSE,
		.raw_cnt = 1,
		.exp_cnt = 2,
		.pclk = 412000000,
		.linelength = 4224,
		.framelength = 3248,
		.max_framerate = 300,
		.mipi_pixel_rate = 474666667,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4160,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4160,
			.h0_size = 3120,
			.scale_w = 4160,
			.scale_h = 3120,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4160,
			.h1_size = 3120,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4160,
			.h2_tg_size = 3120,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail  = 98,
		},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW12_Gr,
		.saturation_info = &imgsensor_saturation_info_12bit,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_COMPOSE,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_ratio_min = 4000,
			.dcg_gain_ratio_max = 4000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = gc13a2_dag_ratio_table_12bit,
			.dcg_gain_table_size = sizeof(gc13a2_dag_ratio_table_12bit),
		},
	},
	*/
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = MOT_VIENNA_GC13A2_SENSOR_ID,
	.reg_addr_sensor_id = {0x03F0, 0x03F1},
	.i2c_addr_table = {0x72, 0x20, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = 0,
	.eeprom_num = 0,
	.resolution = {4160, 3120},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 8,
	.ana_gain_type = 4,
	.ana_gain_step = 1,
	.ana_gain_table = PARAM_UNDEFINED,
	.ana_gain_table_size = PARAM_UNDEFINED,
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = 0xFFFF - 16,
	.exposure_step = 1,
	.exposure_margin = 16,
	.saturation_info = &imgsensor_saturation_info_10bit,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 1,
	.start_exposure_offset = 0,

#if ENABLE_GC13A2_PD
	.pdaf_type = PDAF_SUPPORT_CAMSV,
#else
	.pdaf_type = PDAF_SUPPORT_NA,
#endif

	.hdr_type = HDR_SUPPORT_DCG,
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,
	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = PARAM_UNDEFINED,
	.s_gph = PARAM_UNDEFINED,
	.s_cali = PARAM_UNDEFINED,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = PARAM_UNDEFINED,
	.reg_addr_exposure = {{0x0202, 0x0203},},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x0204, 0x0205, 0x0206},},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = PARAM_UNDEFINED,
	.init_setting_table = gc13a2_init_setting,
	.init_setting_len = ARRAY_SIZE(gc13a2_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.checksum_value = 0x3a98f032,
#if DEBUG_STREAM_OFF_DELAY
	/* custom stream control delay timing for hw limitation (ms) */
	.custom_stream_ctrl_delay = 1000,
#endif
};

#ifdef MOT_GC13A2_PDAF_DEBUG
int gc13a2_overlay_pd_settings(void)
{
	int i, len;

	len = ARRAY_SIZE(mode_struct);
	for (i=0; i<len-1; i++) {
		mode_struct[i].pdaf_cap = gc13a2_pd_en;
		mode_struct[i].frame_desc[1].bus.csi2.data_type = gc13a2_pd_dt;
		mode_struct[i].frame_desc[1].bus.csi2.user_data_desc = gc13a2_pd_ddesc;
		if (!gc13a2_pd_en) {
			mode_struct[i].num_entries = 1;
		}
	}

	if (gc13a2_pd_en) {
		static_ctx.pdaf_type = PDAF_SUPPORT_CAMSV;
	} else {
		static_ctx.pdaf_type = PDAF_SUPPORT_NA;
	}
	return 0;
}
#endif

static struct subdrv_ops ops = {
	.get_id = common_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = common_open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
#if ENABLE_GC13A2_FAST_STANDBY
	.close = gc13a2_ops_close,
#else
	.close = common_close,
#endif
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
};

#ifdef MOT_GC13A2_PDAF_DEBUG
	gc13a2_overlay_pd_settings();
#endif

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 0},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_DVDD, 1100000, 1},
	{HW_ID_AVDD, 2800000, 1},
	{HW_ID_RST, 1, 5},
};

const struct subdrv_entry mot_vienna_gc13a2_mipi_raw_entry = {
	.name = "mot_vienna_gc13a2_mipi_raw",
	.id = MOT_VIENNA_GC13A2_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static int gc13a2_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 gain = *((u32 *)para);
	u32 rg_gain;

	DRV_LOG(ctx, "platform_gain = 0x%x \n", gain);
	/* check boundary of gain */
	gain = max(gain, ctx->s_ctx.ana_gain_min);
	gain = min(gain, ctx->s_ctx.ana_gain_max);
	rg_gain = gain * 8192 / BASEGAIN;
	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	ctx->ana_gain[0] = gain;
	/* write gain */
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_ana_gain[0].addr[0],
		(rg_gain >> 16) & 0xFF);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_ana_gain[0].addr[1],
		(rg_gain >> 8) & 0xFF);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_ana_gain[0].addr[2],
		rg_gain & 0xFF);

	DRV_LOG(ctx, "13a2_rg_gain = 0x%x \n", rg_gain);
	return ERROR_NONE;
}

static int gc13a2_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	return gc13a2_set_shutter_frame_length(ctx, para, len);
}


#if ENABLE_GC13A2_FAST_STANDBY
static int gc13a2_streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
	DRV_LOG_MUST(ctx, "13a2 fast_standby streming. enable=%d(0=stream off, 1=stream on) \n", enable);
	if(enable)
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x01);
	else
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x80);

	return ERROR_NONE;
}

static int gc13a2_ops_close(struct subdrv_ctx *ctx)
{
	gc13a2_streaming_control(ctx, KAL_FALSE);
	DRV_LOG_MUST(ctx, "subdrv close \n");
	return ERROR_NONE;
}

static int gc13a2_set_fast_standby_stream_off(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	return gc13a2_streaming_control(ctx, KAL_FALSE);
}

static int gc13a2_set_fast_standby_stream_on(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	unsigned long long *feature_data = (unsigned long long *)para;

	if(*feature_data != 0)
		gc13a2_set_shutter(ctx, para, len);
	return gc13a2_streaming_control(ctx, KAL_TRUE);
}
#endif

static int gc13a2_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u32 shutter = *feature_data;
	u32 frame_length = *(feature_data + 1);
	u32 fine_integ_line = 0;
	static int longexposue = 0;
	u32 fll = 0;
	u32 fll_step = 0;
	u32 dol_cnt = 1;

	DRV_LOG(ctx, "13a2_shutter = 0x%x \n", shutter);
	DRV_LOG(ctx, "13a2_frame_length = 0x%x \n", frame_length);
	ctx->frame_length = frame_length ? frame_length : ctx->frame_length;
	check_current_scenario_id_bound(ctx);
	/* check boundary of framelength */
	ctx->frame_length =	max(shutter + ctx->s_ctx.exposure_margin, ctx->min_frame_length);
	ctx->frame_length =	min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	/* check boundary of shutter */
	fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	shutter = FINE_INTEG_CONVERT(shutter, fine_integ_line);
	shutter = max(shutter, ctx->s_ctx.exposure_min);
	/* restore shutter */
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	ctx->exposure[0] = shutter;
	/* set_long_exposure */
	if (ctx->s_ctx.long_exposure_support == TRUE) {
		if (shutter > 0xffee) {
			DRV_LOGE(ctx, "gc13a2 enter long exposure!");
			longexposue = 1;
			shutter = (shutter - 0xd10)- 1;
			subdrv_i2c_wr_u8(ctx, 0x0202, 0x0d);
			subdrv_i2c_wr_u8(ctx, 0x0203, 0x10);
			subdrv_i2c_wr_u8(ctx, 0x0340, 0x0d);
			subdrv_i2c_wr_u8(ctx, 0x0341, 0x20);
			subdrv_i2c_wr_u8(ctx, 0x022c, 0x8c);
			subdrv_i2c_wr_u8(ctx, 0x022d, (shutter >> 16) & 0xFF);
			subdrv_i2c_wr_u8(ctx, 0x022e, (shutter >> 8) & 0xFF);
			subdrv_i2c_wr_u8(ctx, 0x022f, (shutter) & 0xFF);
			subdrv_i2c_wr_u8(ctx, 0x0230, 0x0c);
		} else {
			if (longexposue == 1) {
				DRV_LOGE(ctx, "gc13a2 exit long exposure!");
				subdrv_i2c_wr_u8(ctx, 0x022f, 0x00);
				subdrv_i2c_wr_u8(ctx, 0x022e, 0x00);
				subdrv_i2c_wr_u8(ctx, 0x022d, 0x00);
				subdrv_i2c_wr_u8(ctx, 0x0232, 0x00);
				subdrv_i2c_wr_u8(ctx, 0x0230, 0x08);
				longexposue = 0;
			}
		}
	}
	shutter = min(shutter, ctx->s_ctx.exposure_max);
	/* write framelength */
	if (set_auto_flicker(ctx, 0) || frame_length ||
		!ctx->s_ctx.reg_addr_auto_extend) {
		fll = ctx->frame_length;
		fll_step = ctx->s_ctx.mode[ctx->current_scenario_id].framelength_step;
		if (fll_step)
			fll = round_up(fll, fll_step);
		ctx->frame_length = fll;
		if (ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_STAGGER)
			dol_cnt = ctx->s_ctx.mode[ctx->current_scenario_id].exp_cnt;
		fll = fll / dol_cnt;
		if (ctx->extend_frame_length_en == FALSE) {
			subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0],
				(fll >> 8) & 0xFF);
			subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[1],
				fll & 0xFF);
		}
	}
	/* write shutter */
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[0],
		(ctx->exposure[0] >> 8) & 0xFF);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[1],
		ctx->exposure[0] & 0xFF);

	DRV_LOG(ctx, "exp[0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		ctx->exposure[0], frame_length, ctx->frame_length, ctx->autoflicker_en);
	return ERROR_NONE;
}

static int gc13a2_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	bool enable = mode;

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	if (enable) {
		subdrv_i2c_wr_u8(ctx, 0x0089, 0x00); /* 100% Color bar */
		subdrv_i2c_wr_u8(ctx, 0x00ce, 0x19);
		subdrv_i2c_wr_u8(ctx, 0x00cf, 0x00);
	} else {
		subdrv_i2c_wr_u8(ctx, 0x0089, 0x02);
		subdrv_i2c_wr_u8(ctx, 0x00ce, 0x09); /* No pattern */
		subdrv_i2c_wr_u8(ctx, 0x00cf, 0x10);
	}
	ctx->test_pattern = enable;

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
