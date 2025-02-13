// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 mot_nice_gc32e1mipiraw_Sensor.c
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
#include "mot_nice_gc32e1mipiraw_Sensor.h"

static int gc32e1_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int gc32e1_open(struct subdrv_ctx *ctx);
static int gc32e1_close(struct subdrv_ctx *ctx);
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int gc32e1_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc32e1_set_shutter_frame_length(struct subdrv_ctx *ctx,u8 *para, u32 *len);
static int gc32e1_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int gc32e1_streaming_control(struct subdrv_ctx *ctx, kal_bool enable);
static int gc32e1_set_fast_standby_stream_off(struct subdrv_ctx *ctx,u8 *para, u32 *len);
static int gc32e1_set_fast_standby_stream_on(struct subdrv_ctx *ctx,u8 *para, u32 *len);


/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, gc32e1_set_test_pattern},
	{SENSOR_FEATURE_SET_ESHUTTER, gc32e1_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, gc32e1_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, gc32e1_set_fast_standby_stream_off},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, gc32e1_set_fast_standby_stream_on},
};


// static struct mtk_sensor_saturation_info imgsensor_saturation_info_10bit = {
// 	.gain_ratio = 1000,
// 	.OB_pedestal = 64,
// 	.saturation_level = 1023,
// };

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,  // 3264
			.vsize = 0x0990,  // 2448
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,  // 3264
			.vsize = 0x0990,  // 2448
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,  // 3264
			.vsize = 0x0990,  // 2448
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,  // 3264
			.vsize = 0x0990,  // 2448
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,  // 3264
			.vsize = 0x0990,  // 2448
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = gc32e1_preview_setting,
		.mode_setting_len = ARRAY_SIZE(gc32e1_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 345600000,
		.linelength = 4276,
		.framelength = 2694,
		.max_framerate = 300,
		.mipi_pixel_rate = 311040000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6528,
			.h0_size = 4896,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = gc32e1_capture_setting,
		.mode_setting_len = ARRAY_SIZE(gc32e1_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 345600000,
		.linelength = 4276,
		.framelength = 2694,
		.max_framerate = 300,
		.mipi_pixel_rate = 311040000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6528,
			.h0_size = 4896,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = gc32e1_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(gc32e1_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 345600000,
		.linelength = 4276,
		.framelength = 2694,
		.max_framerate = 300,
		.mipi_pixel_rate = 311040000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6528,
			.h0_size = 4896,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = gc32e1_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(gc32e1_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 345600000,
		.linelength = 4276,
		.framelength = 2694,
		.max_framerate = 300,
		.mipi_pixel_rate = 311040000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6528,
			.h0_size = 4896,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = gc32e1_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(gc32e1_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 345600000,
		.linelength = 4276,
		.framelength = 2694,
		.max_framerate = 300,
		.mipi_pixel_rate = 311040000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6528,
			.h0_size = 4896,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = MOT_NICE_GC32E1_SENSOR_ID,
	.reg_addr_sensor_id = {0x03F0, 0x03F1},
	.i2c_addr_table = {0x94, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = 0,
	.eeprom_num = 0,
	.resolution = {6528, 4896},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 32,
	.ana_gain_type = 4,
	.ana_gain_step = 1,
	.ana_gain_table = PARAM_UNDEFINED,
	.ana_gain_table_size = PARAM_UNDEFINED,
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = (0xFFFF*128) - 16,
	.exposure_step = 1,
	.exposure_margin = 32,
	// .saturation_info = &imgsensor_saturation_info_10bit,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
#ifdef IMGSENSOR_FUSION_TEST_WORKAROUND
	.start_exposure_offset_custom = 1909000,
#endif
	.start_exposure_offset = 0,

	.pdaf_type = PDAF_SUPPORT_NA,
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,
	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {{0x0202, 0x0203},},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x022e,
	.reg_addr_ana_gain = {{0x0204, 0x0205},},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = 0,
	.reg_addr_frame_count = PARAM_UNDEFINED,
	.init_setting_table = gc32e1_init_setting,
	.init_setting_len = ARRAY_SIZE(gc32e1_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.checksum_value = 0x3a98f032,
};

static struct subdrv_ops ops = {
	.get_id = gc32e1_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = gc32e1_open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = gc32e1_close,
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 0},
	{HW_ID_DOVDD, 1800000, 1}, // pmic_ldo/gpio(1.8V ldo) for dovdd
	{HW_ID_DVDD, 1200000, 7}, // pmic_ldo for dvdd
	{HW_ID_AVDD, 2800000, 7}, // pmic_ldo for avdd
	{HW_ID_RST, 1, 5},
};

const struct subdrv_entry mot_nice_gc32e1_mipi_raw_entry = {
	.name = SENSOR_DRVNAME_MOT_NICE_GC32E1_MIPI_RAW,
	.id = MOT_NICE_GC32E1_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

int gc32e1_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = 2;
	u32 addr_h = ctx->s_ctx.reg_addr_sensor_id.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_sensor_id.addr[1];
	DRV_LOG(ctx, "Enter");

	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) |
				subdrv_i2c_rd_u8(ctx, addr_l);

			//sensor id + 1 for gc32e1 front on nice
			if(*sensor_id == 0x32e1)
				*sensor_id += 1;

			DRV_LOG_MUST(ctx, "i2c_write_id:0x%x sensor_id(cur/exp):0x%x/0x%x\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == ctx->s_ctx.sensor_id)
				return ERROR_NONE;
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != ctx->s_ctx.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	DRV_LOG(ctx, "Exit");
	return ERROR_NONE;
}

static int gc32e1_open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (gc32e1_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	if (ctx->s_ctx.aov_sensor_support && !ctx->s_ctx.init_in_open)
		DRV_LOG_MUST(ctx, "sensor init not in open stage!\n");
	else
		sensor_init(ctx);

	if (ctx->s_ctx.s_cali != NULL)
		ctx->s_ctx.s_cali((void *) ctx);
	else
		write_sensor_Cali(ctx);

	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	memset(ctx->ana_gain, 0, sizeof(ctx->gain));
	ctx->exposure[0] = ctx->s_ctx.exposure_def;
	ctx->ana_gain[0] = ctx->s_ctx.ana_gain_def;
	ctx->current_scenario_id = scenario_id;
	ctx->pclk = ctx->s_ctx.mode[scenario_id].pclk;
	ctx->line_length = ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length = ctx->s_ctx.mode[scenario_id].framelength;
	ctx->frame_length_rg = ctx->frame_length;
	ctx->current_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
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
	if (ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_LBMF) {
		memset(ctx->frame_length_in_lut, 0,
			sizeof(ctx->frame_length_in_lut));

		switch (ctx->s_ctx.mode[ctx->current_scenario_id].exp_cnt) {
		case 2:
			ctx->frame_length_in_lut[0] = ctx->readout_length + ctx->read_margin;
			ctx->frame_length_in_lut[1] = ctx->frame_length -
				ctx->frame_length_in_lut[0];
			break;
		case 3:
			ctx->frame_length_in_lut[0] = ctx->readout_length + ctx->read_margin;
			ctx->frame_length_in_lut[1] = ctx->readout_length + ctx->read_margin;
			ctx->frame_length_in_lut[2] = ctx->frame_length -
				ctx->frame_length_in_lut[1] - ctx->frame_length_in_lut[0];
			break;
		default:
			break;
		}

		memcpy(ctx->frame_length_in_lut_rg, ctx->frame_length_in_lut,
			sizeof(ctx->frame_length_in_lut_rg));
	}

	return ERROR_NONE;
} /* gc32e1_open */

static int gc32e1_close(struct subdrv_ctx *ctx)
{
	gc32e1_streaming_control(ctx, KAL_FALSE);
	DRV_LOG_MUST(ctx, "subdrv gc32e1 close \n");
	return ERROR_NONE;
} /* gc32e1_close */

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
	return gain * 1024 / BASEGAIN;
}

static int gc32e1_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	return gc32e1_set_shutter_frame_length(ctx, para, len);
}

static int gc32e1_streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
	DRV_LOG_MUST(ctx, "fast_standby streming. enable=%d(0=stream off, 1=stream on) \n", enable);
	if(enable)
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x01);
	else
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x80);

	return ERROR_NONE;
}

static int gc32e1_set_fast_standby_stream_off(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	return gc32e1_streaming_control(ctx, KAL_FALSE);
}

static int gc32e1_set_fast_standby_stream_on(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	unsigned long long *feature_data = (unsigned long long *)para;

	if(*feature_data != 0)
		gc32e1_set_shutter(ctx, para, len);
	return gc32e1_streaming_control(ctx, KAL_TRUE);
}

static void gc32e1_set_long_exposure(struct subdrv_ctx *ctx)
{
	u32 shutter = ctx->exposure[0];
	u32 l_shutter = 0;
	u16 l_shift = 0;
	static int longexposue = 0;
	u32 cal_shutter = 0;

	if (shutter > 0xffff) {
		DRV_LOGE(ctx, "gc32e1 enter long exposure!");
		longexposue = 1;
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
			if (l_shutter
				< (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin))
				break;
		}
		if (l_shift > 7) {
			DRV_LOGE(ctx, "unable to set exposure:%u, set to max\n", shutter);
			l_shift = 7;
		}

		cal_shutter = (shutter - 0xc90) / 4- 1;
		subdrv_i2c_wr_u8(ctx, 0x0202, 0x0c);
		subdrv_i2c_wr_u8(ctx, 0x0203, 0x90);
		subdrv_i2c_wr_u8(ctx, 0x0340, 0x0c);
		subdrv_i2c_wr_u8(ctx, 0x0341, 0xa0);
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure_lshift, (cal_shutter >> 16) & 0xFF);
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure_lshift+1, (cal_shutter >> 8) & 0xFF);
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure_lshift+2, (cal_shutter) & 0xFF);
		subdrv_i2c_wr_u8(ctx, 0x022d, 0x03);

		shutter = ((shutter - 1) >> l_shift) + 1;
		shutter = min(shutter, ctx->s_ctx.exposure_max);
		ctx->frame_length = shutter + ctx->s_ctx.exposure_margin;
		ctx->frame_length_rg = ctx->frame_length;
		ctx->l_shift = l_shift;
	    DRV_LOGE(ctx, "long exposure mode: lshift %u times, normal shutter=0x%x, long shutter=0x%x\n",
				l_shift,
				cal_shutter - shutter - 1,
				cal_shutter);

		/* Frame exposure mode customization for LE*/
		ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		ctx->current_ae_effective_frame = 2;
	} else {
		if (longexposue == 1) {
			DRV_LOGE(ctx, "gc32e1 exit long exposure!");
			subdrv_i2c_wr_u8(ctx, 0x0202, 0x0c);
			subdrv_i2c_wr_u8(ctx, 0x0203, 0x90);
			subdrv_i2c_wr_u8(ctx, 0x0340, 0x0c);
			subdrv_i2c_wr_u8(ctx, 0x0341, 0xa0);
			subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure_lshift+2, 0x00);
			subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure_lshift+1, 0x00);
			subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure_lshift, 0x20);
			subdrv_i2c_wr_u8(ctx, 0x022d, 0x02);
			longexposue = 0;
		}

		if (ctx->s_ctx.reg_addr_exposure_lshift != PARAM_UNDEFINED) {
			ctx->l_shift = l_shift;
		}
		shutter = min(shutter, ctx->s_ctx.exposure_max);
		/* write framelength&shutter */
		if (set_auto_flicker(ctx, 0) || ctx->frame_length) {
			subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0],
				(ctx->frame_length >> 8) & 0xFF);
			subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[1],
				ctx->frame_length & 0xFF);
		}
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[0],
			(ctx->exposure[0] >> 8) & 0xFF);
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[1],
			ctx->exposure[0] & 0xFF);
		DRV_LOG(ctx, "normal exposure mode: lshift %u times, normal shutter=0x%x, frame_length=%d\n",
				l_shift,
				shutter,
				ctx->frame_length);

		ctx->current_ae_effective_frame = 2;
	}
	ctx->exposure[0] = shutter;
}

static int gc32e1_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u32 shutter = *feature_data;
	u32 frame_length = 0;
	u32 fine_integ_line = 0;

	DRV_LOG(ctx, "shutter = 0x%x \n", shutter);
	DRV_LOG(ctx, "frame_length = 0x%x \n", frame_length);
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
		gc32e1_set_long_exposure(ctx);
	} else {
		shutter = min(shutter, ctx->s_ctx.exposure_max);
		/* write framelength&shutter */
		if (set_auto_flicker(ctx, 0) || ctx->frame_length) {
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0],
			(ctx->frame_length >> 8) & 0xFF);
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[1],
			ctx->frame_length & 0xFF);
		}
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[0],
			(ctx->exposure[0] >> 8) & 0xFF);
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[1],
			ctx->exposure[0] & 0xFF);
	}

	DRV_LOG(ctx, "exp[0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		ctx->exposure[0], frame_length, ctx->frame_length, ctx->autoflicker_en);
	return ERROR_NONE;
}

static int gc32e1_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	bool enable = mode;

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	if (enable) {
		subdrv_i2c_wr_u8(ctx, 0x008c, 0x01); /* 100% Color bar */
		subdrv_i2c_wr_u8(ctx, 0x008d, 0x00);
	} else {
		subdrv_i2c_wr_u8(ctx, 0x008c, 0x00); /* No pattern */
		subdrv_i2c_wr_u8(ctx, 0x008d, 0x10);
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
