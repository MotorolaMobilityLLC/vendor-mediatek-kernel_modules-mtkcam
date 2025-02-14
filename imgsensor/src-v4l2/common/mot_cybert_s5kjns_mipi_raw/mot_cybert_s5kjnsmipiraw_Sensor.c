// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5kjnsmipiraw_Sensor.c
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
#include "mot_cybert_s5kjnsmipiraw_Sensor.h"
#include "mot_s5kjns_cali.h"
#define USING_DPHY_N_LANE 4

static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int s5kjns_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static void s5kjns_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
static int s5kjns_set_ctrl_locker(struct subdrv_ctx *ctx, u32 cid, bool *is_lock);
static int s5kjns_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int s5kjns_ops_close(struct subdrv_ctx *ctx);
static int s5kjns_streaming_off(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjns_streaming_on(struct subdrv_ctx *ctx, u8 *para, u32 *len);

static bool streaming_status = false;

#define ENABLE_S5KJNS_LONG_EXPOSURE TRUE
#if  ENABLE_S5KJNS_LONG_EXPOSURE
static int s5kjns_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void s5kjns_set_shutter_frame_length(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length);
#endif
/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, s5kjns_set_test_pattern},
#if  ENABLE_S5KJNS_LONG_EXPOSURE
	{SENSOR_FEATURE_SET_ESHUTTER, s5kjns_set_shutter},
#endif
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, s5kjns_streaming_off},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, s5kjns_streaming_on},
};

#if  ENABLE_S5KJNS_LONG_EXPOSURE
static void s5kjns_set_long_exposure(struct subdrv_ctx *ctx)
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
			if (l_shutter
				< (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin))
				break;
		}
		if (l_shift > 7) {
			DRV_LOGE(ctx, "unable to set exposure:%u, set to max\n", shutter);
			l_shift = 7;
		}
		shutter = ((shutter - 1) >> l_shift) + 1;
		ctx->frame_length = shutter + ctx->s_ctx.exposure_margin;
		DRV_LOG(ctx, "long exposure mode: lshift %u times\n", l_shift);
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_exposure_lshift, l_shift);
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_exposure_lshift+2, l_shift);
		ctx->l_shift = l_shift;
		/* Frame exposure mode customization for LE*/
		ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		ctx->current_ae_effective_frame = 2;
	} else {
		if (ctx->s_ctx.reg_addr_exposure_lshift != PARAM_UNDEFINED) {
			set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_exposure_lshift, l_shift);
			set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_exposure_lshift+2, l_shift);
			ctx->l_shift = l_shift;
		}
		ctx->current_ae_effective_frame = 2;
	}
	ctx->exposure[IMGSENSOR_STAGGER_EXPOSURE_LE] = shutter;
}


static void s5kjns_set_shutter_frame_length(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length)
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
		(u64)ctx->s_ctx.mode[ctx->current_scenario_id].multi_exposure_shutter_range[0].min);
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
	s5kjns_set_long_exposure(ctx);
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

static int s5kjns_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 shutter = *((u64 *)para);
	s5kjns_set_shutter_frame_length(ctx, shutter,0);
	return 0;
}
#endif

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0X0FF0,
			.vsize = 0X0BF4,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0X0FF0,
			.vsize = 0X0BF4,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0X0FF0,
			.vsize = 0X0BF4,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0X07F8,
			.vsize = 0x047C,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0X0FF0,
			.vsize = 0X0BF4,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cust1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0X07F8,
			.vsize = 0x05F8,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cust2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0F00,
			.vsize = 0x0870,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cust3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0X07F8,
			.vsize = 0x047C,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = addr_data_pair_preview_mot_cybert_s5kjns,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_preview_mot_cybert_s5kjns),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 560000000,
		.linelength = 4224,
		.framelength = 4416,
		.max_framerate = 300,
		.mipi_pixel_rate = 792000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6120,
			.scale_w = 4080,
			.scale_h = 3060,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3060,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3060,
		},
		//.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = BASEGAIN * 1,
		.ana_gain_max = BASEGAIN * 64,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = addr_data_pair_preview_mot_cybert_s5kjns,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_preview_mot_cybert_s5kjns),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 560000000,
		.linelength = 4224,
		.framelength = 4416,
		.max_framerate = 300,
		.mipi_pixel_rate = 792000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6120,
			.scale_w = 4080,
			.scale_h = 3060,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3060,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3060,
		},
		//.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = BASEGAIN * 1,
		.ana_gain_max = BASEGAIN * 64,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = addr_data_pair_preview_mot_cybert_s5kjns,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_preview_mot_cybert_s5kjns),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 560000000,
		.linelength = 4224,
		.framelength = 4416,
		.max_framerate = 300,
		.mipi_pixel_rate = 792000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6120,
			.scale_w = 4080,
			.scale_h = 3060,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3060,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3060,
		},
		//.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = BASEGAIN * 1,
		.ana_gain_max = BASEGAIN * 64,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = addr_data_pair_120fps_s5kjns,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_120fps_s5kjns),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 600000000,
		.linelength = 2208,
		.framelength = 2256,
		.max_framerate = 1200,
		.mipi_pixel_rate = 801600000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6120,
			.x0_offset = 0,
			.y0_offset = 764,
			.w0_size = 8160,
			.h0_size = 4592,
			.scale_w = 2040,
			.scale_h = 1148,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2040,
			.h1_size = 1148,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2040,
			.h2_tg_size = 1148,
		},
		//.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = BASEGAIN * 1,
		.ana_gain_max = BASEGAIN * 16,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = addr_data_pair_preview_mot_cybert_s5kjns,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_preview_mot_cybert_s5kjns),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 560000000,
		.linelength = 4224,
		.framelength = 4416,
		.max_framerate = 300,
		.mipi_pixel_rate = 792000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6120,
			.scale_w = 4080,
			.scale_h = 3060,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3060,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3060,
		},
		//.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = BASEGAIN * 1,
		.ana_gain_max = BASEGAIN * 64,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cust1,
		.num_entries = ARRAY_SIZE(frame_desc_cust1),
		.mode_setting_table = addr_data_pair_custom1_s5kjns,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom1_s5kjns),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 600000000,
		.linelength = 4096,
		.framelength = 4880,
		.max_framerate = 300,
		.mipi_pixel_rate = 792000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6120,
			.x0_offset = 0,
			.y0_offset = 4,
			.w0_size = 8160,
			.h0_size = 6112,
			.scale_w = 2040,
			.scale_h = 1528,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2040,
			.h1_size = 1528,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2040,
			.h2_tg_size = 1528,
		},
		//.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = BASEGAIN * 1,
		.ana_gain_max = BASEGAIN * 16,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cust2,
		.num_entries = ARRAY_SIZE(frame_desc_cust2),
		.mode_setting_table = addr_data_pair_custom2_s5kjns,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom2_s5kjns),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 560000000,
		.linelength = 4096,
		.framelength = 2276,
		.max_framerate = 600,
		.mipi_pixel_rate = 792000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6120,
			.x0_offset = 240,
			.y0_offset = 900,
			.w0_size = 7680,
			.h0_size = 4320,
			.scale_w = 3840,
			.scale_h = 2160,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3840,
			.h1_size = 2160,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3840,
			.h2_tg_size = 2160,
		},
		//.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = BASEGAIN * 1,
		.ana_gain_max = BASEGAIN * 64,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cust3,
		.num_entries = ARRAY_SIZE(frame_desc_cust3),
		.mode_setting_table = addr_data_pair_custom3_s5kjns,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom3_s5kjns),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 600000000,
		.linelength = 6208,
		.framelength = 1608,
		.max_framerate = 600,
		.mipi_pixel_rate = 792000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6120,
			.x0_offset = 0,
			.y0_offset = 764,
			.w0_size = 8160,
			.h0_size = 4592,
			.scale_w = 2040,
			.scale_h = 1148,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2040,
			.h1_size = 1148,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2040,
			.h2_tg_size = 1148,
		},
		//.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.ana_gain_min = BASEGAIN * 1,
		.ana_gain_max = BASEGAIN * 16,
		.csi_param = {0},
	},
};
static struct subdrv_static_ctx static_ctx = {
	.sensor_id = MOT_CYBERT_S5KJNS_SENSOR_ID,
	.reg_addr_sensor_id = {0x0000,0x0001},
	.i2c_addr_table = {0x20, 0xFF},
	.i2c_burst_write_support = FALSE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	//.eeprom_info = eeprom_info,
	//.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {8160, 6120},
	.mirror = IMAGE_HV_MIRROR,
	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
#if USING_DPHY_N_LANE == 1
	.mipi_lane_num = SENSOR_MIPI_1_LANE,
#elif USING_DPHY_N_LANE == 2
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
#else
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
#endif
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 2,
	.ana_gain_step = 2,
	.ana_gain_table = PARAM_UNDEFINED,
	.ana_gain_table_size = PARAM_UNDEFINED,
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = 0xFFFF*128 - 10,
	.exposure_step = 2,
	.exposure_margin = 10,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 2336000,
#ifdef IMGSENSOR_FUSION_TEST_WORKAROUND
	.start_exposure_offset_custom = 1642000,
#endif
	.pdaf_type = PDAF_SUPPORT_NA,
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,
	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,
	.s_cali = mot_s5kjns_apply_xtc_data,
	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {{0x0202, 0x0203},},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x0702,
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
	.chk_s_off_end = 1,

	.checksum_value = 0x31E3FBE2,

	/* custom stream control delay timing for hw limitation (ms) */
	//.custom_stream_ctrl_delay = 3,
};

static struct subdrv_ops ops = {
	.get_id = s5kjns_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = s5kjns_ops_close,
	.get_frame_desc = common_get_frame_desc,
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
	.set_ctrl_locker = s5kjns_set_ctrl_locker,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_RST, 0, 1},
	{HW_ID_MCLK, 24, 1},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_DVDD, 1050000, 8},
	{HW_ID_AVDD, 2800000,4},
	{HW_ID_RST, 1, 17},
};

const struct subdrv_entry mot_cybert_s5kjns_mipi_raw_entry = {
	.name = "mot_cybert_s5kjns_mipi_raw",
	.id = MOT_CYBERT_S5KJNS_SENSOR_ID,
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


static u16 get_gain2reg(const u32 gain)
{
	u32 reg_gain = 0x0;

	reg_gain = gain *32/ BASEGAIN;
	return (u32) reg_gain;
}

static int s5kjns_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
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

			//sensor id +1 for s5kjns only
			if(*sensor_id == 0x38EE)
			*sensor_id += 4;

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

static int s5kjns_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	if (mode)
		subdrv_i2c_wr_u16(ctx, 0x0600, 0x0001); /*Black*/
	else if (ctx->test_pattern)
		subdrv_i2c_wr_u16(ctx, 0x0600, 0x0000); /*No pattern*/

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

static int s5kjns_streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
	int timeout = ctx->current_fps ? (10000 / ctx->current_fps) +1 : 101;
	int i = 0;
	int framecnt = 0;
	int check_cnt = 100;

	DRV_LOG(ctx, "streaming_control. enable=%d(0=stream off, 1=stream on)\n", enable);
	if(enable)
	{
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x01);
		mDELAY(10);
		streaming_status = true;
	}
	else
	{
		if(streaming_status == true){
			for (i = 0; i < check_cnt; i++)
			{
				mDELAY(1);
				framecnt = subdrv_i2c_rd_u8(ctx, 0x0005);
				if(framecnt != 0xFF)
				{
					DRV_LOG_MUST(ctx,"last stream on OK at i=%d.\n", i);
					break;
				}
			}
		}

		streaming_status = false;
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x00);
		for (i = 0; i < timeout; i++)
		{
			mDELAY(5);
			framecnt = subdrv_i2c_rd_u8(ctx, 0x0005);
			if(framecnt == 0xFF)
			{
				DRV_LOG_MUST(ctx,"stream off OK at i=%d.\n", i);
				return ERROR_NONE;
			}
		}
		DRV_LOG_MUST(ctx, "stream off Fail! framecnt = %d.\n", framecnt);
	}
	return ERROR_NONE;
}

static int s5kjns_streaming_on(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG_MUST(ctx, "subdrv open \n");
	return s5kjns_streaming_control(ctx, KAL_TRUE);
}

static int s5kjns_streaming_off(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG_MUST(ctx, "subdrv close \n");
	return s5kjns_streaming_control(ctx, KAL_FALSE);
}

static int s5kjns_ops_close(struct subdrv_ctx *ctx)
{
	s5kjns_streaming_control(ctx, KAL_FALSE);
	DRV_LOG_MUST(ctx, "subdrv close \n");
	return ERROR_NONE;
}

static void s5kjns_sensor_init(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "E\n");
	DRV_LOG(ctx, "MOT CYBERT S5KJNS init start\n");
	subdrv_i2c_wr_u16(ctx, 0x6028, 0x4000);
	subdrv_i2c_wr_u16(ctx, 0x0000, 0x0001);
	subdrv_i2c_wr_u16(ctx, 0x0000, 0x38EE);
	subdrv_i2c_wr_u16(ctx, 0x001E, 0x000B);
	subdrv_i2c_wr_u16(ctx, 0x6028, 0x4000);
	subdrv_i2c_wr_u16(ctx, 0x6010, 0x0001);
	mdelay(13);
	subdrv_i2c_wr_u16(ctx, 0x6226, 0x0001);

	i2c_table_write(ctx, addr_data_pair_init_mot_cybert_s5kjns,
			ARRAY_SIZE(addr_data_pair_init_mot_cybert_s5kjns));
	DRV_LOG(ctx, "MOT CYBERT S5KJNS init end\n");
	DRV_LOG(ctx, "X\n");
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;
	/* get sensor id */
	if (s5kjns_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;
	/* initail setting */
	s5kjns_sensor_init(ctx);
	if (ctx->s_ctx.s_cali != NULL){
		DRV_LOG_MUST(ctx, "HW_GGC: will apply hw_ggc data");
		ctx->s_ctx.s_cali((void *) ctx);
	}
	else{
		write_sensor_Cali(ctx);
	}
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

static int s5kjns_set_ctrl_locker(struct subdrv_ctx *ctx,
		u32 cid, bool *is_lock)
{
	bool lock_set_ctrl = false;

	if (unlikely(is_lock == NULL)) {
		pr_info("[%s][ERROR] is_lock %p is NULL\n", __func__, is_lock);
		return -EINVAL;
	}

	switch (cid) {
	case V4L2_CID_MTK_STAGGER_AE_CTRL:
	case V4L2_CID_MTK_MAX_FPS:
		if ((ctx->sof_no == 0) && (ctx->is_streaming)) {
			lock_set_ctrl = true;
			DRV_LOG(ctx,
				"[%s] Target lock cid(%u) lock_set_ctrl(%d), sof_no(%d) is_streaming(%d)\n",
				__func__,
				cid,
				lock_set_ctrl,
				ctx->sof_no,
				ctx->is_streaming);
		}
		break;
	default:
		break;
	}

	*is_lock = lock_set_ctrl;
	return ERROR_NONE;
} /* s5kjns_set_ctrl_locker */
