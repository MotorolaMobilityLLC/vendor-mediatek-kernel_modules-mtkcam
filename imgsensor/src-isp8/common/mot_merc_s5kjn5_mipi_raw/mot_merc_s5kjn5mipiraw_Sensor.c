// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 mot_merc_s5kjn5mipiraw_Sensor.c
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
#include "mot_merc_s5kjn5mipiraw_Sensor.h"
#define USING_DPHY_N_LANE 4

static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int s5kjn5_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static void s5kjn5_sensor_init(struct subdrv_ctx *ctx);
static int s5kjn5_open(struct subdrv_ctx *ctx);
static int s5kjn5_set_ctrl_locker(struct subdrv_ctx *ctx, u32 cid, bool *is_lock);
static int s5kjn5_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5_lens_position(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void get_sensor_cali(struct subdrv_ctx *ctx);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);

#define ENABLE_S5KJN5_PD TRUE

#define ENABLE_S5KJN5_LONG_EXPOSURE TRUE
#if  ENABLE_S5KJN5_LONG_EXPOSURE
static int s5kjn5_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void s5kjn5_set_shutter_frame_length(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length);
#endif
/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, s5kjn5_set_test_pattern},
	{SENSOR_FEATURE_SET_AWB_GAIN, s5kjn5_awb_gain},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, s5kjn5_seamless_switch},
	{SENSOR_FEATURE_SET_LENS_POSITION, s5kjn5_lens_position},
#if  ENABLE_S5KJN5_LONG_EXPOSURE
	{SENSOR_FEATURE_SET_ESHUTTER, s5kjn5_set_shutter},
#endif
};

#if  ENABLE_S5KJN5_LONG_EXPOSURE
static void s5kjn5_set_long_exposure(struct subdrv_ctx *ctx)
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


static void s5kjn5_set_shutter_frame_length(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length)
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
	s5kjn5_set_long_exposure(ctx);
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

static int s5kjn5_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 shutter = *((u64 *)para);
	s5kjn5_set_shutter_frame_length(ctx, shutter,0);
	return 0;
}
#endif

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0C00,
			.is_active_line = TRUE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
#if ENABLE_S5KJN5_PD
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10, // for dt 0x30
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.is_active_line = TRUE,
		},
	}
#endif
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0C00,
			.is_active_line = TRUE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
#if ENABLE_S5KJN5_PD
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
	}
#endif
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0C00,
			.is_active_line = TRUE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
#if ENABLE_S5KJN5_PD
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
	}
#endif
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0C00,
			.is_active_line = TRUE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
#if ENABLE_S5KJN5_PD
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
	}
#endif
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0C00,
			.is_active_line = TRUE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
#if ENABLE_S5KJN5_PD
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
	}
#endif
};

static struct mtk_mbus_frame_desc_entry frame_desc_cust1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x2000,
			.vsize = 0x1800,
			.is_active_line = TRUE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cust2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.is_active_line = TRUE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
#if ENABLE_S5KJN5_PD
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0240,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.is_active_line = TRUE,
		},
	}
#endif
};

static struct mtk_mbus_frame_desc_entry frame_desc_cust3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0C00,
			.is_active_line = TRUE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
#if ENABLE_S5KJN5_PD
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
	}
#endif
};

#if ENABLE_S5KJN5_PD
static struct SET_PD_BLOCK_INFO_T s5kjn5_pd_info = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0}},
	.i4PosR = {{0, 0}},
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4Crop = {
		// <pre> <cap> <normal_video> <hs_video> <<slim_video>>
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// <<cust1>> <<cust2>> <<cust3>> <cust4> <cust5>
		{0, 0}, {0, 384}, {0, 0}, {0, 0}, {0, 0},
	},
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.iMirrorFlip = IMAGE_V_MIRROR,
	.i4ModeIndex = 3,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		.i4PDPattern = 1, // all PD
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {0, 1}, // L = 0, R = 1
	},
};

static struct SET_PD_BLOCK_INFO_T s5kjn5_pd_cus3_info = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0}},
	.i4PosR = {{0, 0}},
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4Crop = {
		// <pre> <cap> <normal_video> <hs_video> <<slim_video>>
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// <<cust1>> <<cust2>> <<cust3>> <cust4> <cust5>
		{0, 0}, {0, 0}, {2048, 1536}, {0, 0}, {0, 0},
	},
	.i4FullRawW = 8192,
	.i4FullRawH = 6144,
	.iMirrorFlip = IMAGE_V_MIRROR,
	.i4ModeIndex = 3,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		.i4PDPattern = 1, // all PD
		.i4BinFacX = 4,
		.i4BinFacY = 8,
		.i4PDRepetition = 0,
		.i4PDOrder = {0, 1}, // L = 0, R = 1
	},
};
#endif

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = addr_data_pair_preview_mot_merc_s5kjn5,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_preview_mot_merc_s5kjn5),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = addr_data_pair_seamless_preview_mot_merc_s5kjn5,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(addr_data_pair_seamless_preview_mot_merc_s5kjn5),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
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
#if ENABLE_S5KJN5_PD
		.pdaf_cap = ENABLE_S5KJN5_PD,
		.imgsensor_pd_info = &s5kjn5_pd_info,
#else
		.pdaf_cap = PARAM_UNDEFINED,
		.imgsensor_pd_info = PARAM_UNDEFINED,
#endif
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 160,
		.min_exposure_line = 4,
		.read_margin = 12,
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = addr_data_pair_preview_mot_merc_s5kjn5,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_preview_mot_merc_s5kjn5),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
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
#if ENABLE_S5KJN5_PD
		.pdaf_cap = ENABLE_S5KJN5_PD,
		.imgsensor_pd_info = &s5kjn5_pd_info,
#else
		.pdaf_cap = PARAM_UNDEFINED,
		.imgsensor_pd_info = PARAM_UNDEFINED,
#endif
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 160,
		.min_exposure_line = 4,
		.read_margin = 12,
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = addr_data_pair_preview_mot_merc_s5kjn5,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_preview_mot_merc_s5kjn5),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
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
#if ENABLE_S5KJN5_PD
		.pdaf_cap = ENABLE_S5KJN5_PD,
		.imgsensor_pd_info = &s5kjn5_pd_info,
#else
		.pdaf_cap = PARAM_UNDEFINED,
		.imgsensor_pd_info = PARAM_UNDEFINED,
#endif
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 160,
		.min_exposure_line = 4,
		.read_margin = 12,
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = addr_data_pair_preview_mot_merc_s5kjn5,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_preview_mot_merc_s5kjn5),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
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
#if ENABLE_S5KJN5_PD
		.pdaf_cap = ENABLE_S5KJN5_PD,
		.imgsensor_pd_info = &s5kjn5_pd_info,
#else
		.pdaf_cap = PARAM_UNDEFINED,
		.imgsensor_pd_info = PARAM_UNDEFINED,
#endif
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 160,
		.min_exposure_line = 4,
		.read_margin = 12,
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = addr_data_pair_preview_mot_merc_s5kjn5,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_preview_mot_merc_s5kjn5),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
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
#if ENABLE_S5KJN5_PD
		.pdaf_cap = ENABLE_S5KJN5_PD,
		.imgsensor_pd_info = &s5kjn5_pd_info,
#else
		.pdaf_cap = PARAM_UNDEFINED,
		.imgsensor_pd_info = PARAM_UNDEFINED,
#endif
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 160,
		.min_exposure_line = 4,
		.read_margin = 12,
	},
	{
		.frame_desc = frame_desc_cust1,
		.num_entries = ARRAY_SIZE(frame_desc_cust1),
		.mode_setting_table = addr_data_pair_custom1_mot_merc_s5kjn5,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom1_mot_merc_s5kjn5),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 920000000,
		.linelength = 9600,
		.framelength = 6346,
		.max_framerate = 150,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
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
		.pdaf_cap = PARAM_UNDEFINED,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 16,
		.min_exposure_line = 8,
		.read_margin = 40,
		.awb_enabled = 1,
	},
	{
		.frame_desc = frame_desc_cust2,
		.num_entries = ARRAY_SIZE(frame_desc_cust2),
		.mode_setting_table = addr_data_pair_custom2_mot_merc_s5kjn5,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom2_mot_merc_s5kjn5),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 3204,
		.max_framerate = 600,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
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
#if ENABLE_S5KJN5_PD
		.pdaf_cap = ENABLE_S5KJN5_PD,
		.imgsensor_pd_info = &s5kjn5_pd_info,
#else
		.pdaf_cap = PARAM_UNDEFINED,
		.imgsensor_pd_info = PARAM_UNDEFINED,
#endif
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 160,
		.min_exposure_line = 4,
		.read_margin = 12,
	},
	{
		.frame_desc = frame_desc_cust3,
		.num_entries = ARRAY_SIZE(frame_desc_cust3),
		.mode_setting_table = addr_data_pair_custom3_mot_merc_s5kjn5,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom3_mot_merc_s5kjn5),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = addr_data_pair_seamless_custom3_mot_merc_s5kjn5,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(addr_data_pair_seamless_custom3_mot_merc_s5kjn5),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 920000000,
		.linelength = 9200,
		.framelength = 3332,
		.max_framerate = 300,
		.mipi_pixel_rate = 1222400000,
		.readout_length = 0,
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
#if ENABLE_S5KJN5_PD
		.pdaf_cap = ENABLE_S5KJN5_PD,
		.imgsensor_pd_info = &s5kjn5_pd_cus3_info,
#else
		.pdaf_cap = PARAM_UNDEFINED,
		.imgsensor_pd_info = PARAM_UNDEFINED,
#endif
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 16,
		.min_exposure_line = 8,
		.read_margin = 40,
		.awb_enabled = 1,
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = MOT_MERC_S5KJN5_SENSOR_ID,
	.reg_addr_sensor_id = {0x0000,0x0001},
	.i2c_addr_table = {0x5A, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	//.eeprom_info = eeprom_info,
	//.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {8192, 6144},
	.mirror = IMAGE_V_MIRROR,
	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 160,
	.ana_gain_type = 2,
	.ana_gain_step = 2,
	.ana_gain_table = PARAM_UNDEFINED,
	.ana_gain_table_size = PARAM_UNDEFINED,
	.tuning_iso_base = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = 0xFFFF*128 - 12,
	.exposure_step = 1,
	.exposure_margin = 12,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 500000,
#ifdef IMGSENSOR_FUSION_TEST_WORKAROUND
	.start_exposure_offset_custom = 1000000,
#endif
#if ENABLE_S5KJN5_PD
	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
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
	.get_id = common_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = s5kjn5_open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
	.set_ctrl_locker = s5kjn5_set_ctrl_locker,
	.vsync_notify = vsync_notify,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_RST, {0}, 1000},
	{HW_ID_MCLK, {24}, 1000},
	{HW_ID_MCLK_DRIVING_CURRENT, {6}, 0},
	{HW_ID_DOVDD, {1804000,1804000}, 4000},
	{HW_ID_AVDD, {2200000,2200000}, 6000},
	{HW_ID_DVDD, {1000000,1000000}, 6000},
	{HW_ID_AFVDD, {3300000, 3300000}, 1000},
	{HW_ID_RST, {1}, 10000},
};

const struct subdrv_entry mot_merc_s5kjn5_mipi_raw_entry = {
	.name = "mot_merc_s5kjn5_mipi_raw",
	.id = MOT_MERC_S5KJN5_SENSOR_ID,
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


static int s5kjn5_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static void s5kjn5_sensor_init(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "E\n");
	DRV_LOG(ctx, "MOT MERC S5KJN5 init start\n");
	subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
	subdrv_i2c_wr_u16(ctx, 0x0000, 0x000E);
	subdrv_i2c_wr_u16(ctx, 0x0000, 0x38E5);
	subdrv_i2c_wr_u16(ctx, 0x6018, 0x0001);
	subdrv_i2c_wr_u16(ctx, 0x7002, 0x0408);
	subdrv_i2c_wr_u16(ctx, 0x6014, 0x0001);
	subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x2002);
	subdrv_i2c_wr_u16(ctx, 0x1E92, 0x8000);
	subdrv_i2c_wr_u16(ctx, 0x1E84, 0x282B);
	subdrv_i2c_wr_u16(ctx, 0x1E86, 0x0320);
	subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
	subdrv_i2c_wr_u16(ctx, 0x7002, 0x0008);
	mdelay(5);

	i2c_table_write(ctx, addr_data_pair_init_1_mot_merc_s5kjn5,
			ARRAY_SIZE(addr_data_pair_init_1_mot_merc_s5kjn5));
	i2c_table_write(ctx, addr_data_pair_init_2_mot_merc_s5kjn5,
			ARRAY_SIZE(addr_data_pair_init_2_mot_merc_s5kjn5));
	i2c_table_write(ctx, addr_data_pair_init_3_mot_merc_s5kjn5,
			ARRAY_SIZE(addr_data_pair_init_3_mot_merc_s5kjn5));
	DRV_LOG(ctx, "MOT MERC S5KJN5 init end\n");
	DRV_LOG(ctx, "X\n");
}

static int s5kjn5_open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;
	/* get sensor id */
	if (common_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;
	/* initail setting */
	s5kjn5_sensor_init(ctx);

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
	get_sensor_cali(ctx);
	return ERROR_NONE;
} /* s5kjn5_open */

static int s5kjn5_set_ctrl_locker(struct subdrv_ctx *ctx,
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
} /* s5kjn5_set_ctrl_locker */

static int crc_reverse_byte(int data)
{
	return ((data * 0x0802LU & 0x22110LU) |
		(data * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
}

static int32_t check_crc16(uint8_t  *data, uint32_t size, uint32_t ref_crc)
{
	int32_t crc_match = 0;
	uint16_t crc = 0x0000;
	uint16_t crc_reverse = 0x0000;
	uint32_t i, j;

	uint32_t tmp;
	uint32_t tmp_reverse;

	/* Calculate both methods of CRC since integrators differ on
	  * how CRC should be calculated. */
	for (i = 0; i < size; i++) {
		tmp_reverse = crc_reverse_byte(data[i]);
		tmp = data[i] & 0xff;
		for (j = 0; j < 8; j++) {
			if (((crc & 0x8000) >> 8) ^ (tmp & 0x80))
				crc = (crc << 1) ^ 0x8005;
			else
				crc = crc << 1;
			tmp <<= 1;

			if (((crc_reverse & 0x8000) >> 8) ^ (tmp_reverse & 0x80))
				crc_reverse = (crc_reverse << 1) ^ 0x8005;
			else
				crc_reverse = crc_reverse << 1;

			tmp_reverse <<= 1;
		}
	}

	crc_reverse = (crc_reverse_byte(crc_reverse) << 8) |
		crc_reverse_byte(crc_reverse >> 8);

	if (crc == ref_crc || crc_reverse == ref_crc)
		crc_match = 1;

	return crc_match;
}

/* FUNCTION */
#define  S5KJN5_AF_DATA_START 0x0027
#define  S5KJN5_AF_DATA_LEN 24
#define  S5KJN5_EEPROM_ADDR 0xA0
static u16 af_macro_val =0;
static u16 af_inf_val =0;
static bool s5kjn5_af_data_ready = FALSE;
static void get_sensor_cali(struct subdrv_ctx *ctx)
{

	int ret = 0;
	u16 ref_crc = 0;
	u8 s5kjn5_af_data[26];
	if (s5kjn5_af_data_ready) {
		DRV_LOG_MUST(ctx, "af data is ready.");
		return;
	}
	ret = adaptor_i2c_rd_p8(ctx->i2c_client, (S5KJN5_EEPROM_ADDR >> 1), S5KJN5_AF_DATA_START, s5kjn5_af_data, S5KJN5_AF_DATA_LEN+2) ;
	if (ret < 0) {
		DRV_LOGE(ctx, "Read af data failed. ret:%d", ret);
		s5kjn5_af_data_ready = FALSE;
		return;
	}
	ref_crc = ((s5kjn5_af_data[S5KJN5_AF_DATA_LEN] << 8) |s5kjn5_af_data[S5KJN5_AF_DATA_LEN+1]);
	if (check_crc16(s5kjn5_af_data, S5KJN5_AF_DATA_LEN, ref_crc)) {
		s5kjn5_af_data_ready = TRUE;
		DRV_LOG(ctx, "af data ready now.");
	} else {
		DRV_LOGE(ctx, "AF data CRC error!");
	}

	if( s5kjn5_af_data_ready)
	{
		af_macro_val = (s5kjn5_af_data[2]<<8 | s5kjn5_af_data[3])/16;
		af_inf_val = (s5kjn5_af_data[6]<<8 | s5kjn5_af_data[7])/16;
	}
	DRV_LOG(ctx, "af_macro_val =%d  af_inf_val =%d \n", af_macro_val,af_inf_val);
}

static int s5kjn5_lens_position(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 lens_position_reg_val;
	u32 lens_position = *((u32 *)para);
	if(s5kjn5_af_data_ready == FALSE)
	{
		DRV_LOG(ctx, "s5kjn5_af_data_ready =%d \n", s5kjn5_af_data_ready);
		return -1;
	}
	if(lens_position > af_macro_val)
	{
		lens_position =af_macro_val;
	}
	if(lens_position < af_inf_val)
	{
		lens_position =af_inf_val;
	}
	if ((ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM1) ||
		(ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3))
	{
		lens_position_reg_val = (lens_position-af_inf_val)*1023 /(af_macro_val -af_inf_val);
		DRV_LOG(ctx, "lens_position =%d  lens_position_reg_val =%d",lens_position,lens_position_reg_val);
		DRV_LOG(ctx, "af_macro_val =%d  af_inf_val =%d",af_macro_val,af_inf_val);
		subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x2001);
		subdrv_i2c_wr_u16(ctx, 0x2566, ((lens_position_reg_val & 0xFF) << 8)|((lens_position_reg_val & 0xFF00) >> 8));
		subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
	}
	return 0;
}

#define REG_GAIN 0x0400
#define WB_GAIN_FACTOR 512
static int s5kjn5_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct SET_SENSOR_AWB_GAIN *pSetSensorAWB = (( struct SET_SENSOR_AWB_GAIN  *)para);

	UINT32 rgain_32, grgain_32, gbgain_32, bgain_32, ggain_32;
	if ((ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM1) ||
		(ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3))
	{
		grgain_32 = (pSetSensorAWB->ABS_GAIN_GR * REG_GAIN ) / WB_GAIN_FACTOR;
		rgain_32 = (pSetSensorAWB->ABS_GAIN_R * REG_GAIN ) / WB_GAIN_FACTOR;
		bgain_32 = (pSetSensorAWB->ABS_GAIN_B * REG_GAIN ) / WB_GAIN_FACTOR;
		gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB * REG_GAIN ) / WB_GAIN_FACTOR;
		ggain_32 = (grgain_32+gbgain_32)/2;

		DRV_LOG(ctx, "[%s] ABS_GAIN_GR:%d, grgain_32:%d, ABS_GAIN_R:%d, rgain_32:%d , ABS_GAIN_B:%d, bgain_32:%d,ABS_GAIN_GB:%d, gbgain_32:%d\n",
			__func__,
			pSetSensorAWB->ABS_GAIN_GR, grgain_32,
			pSetSensorAWB->ABS_GAIN_R, rgain_32,
			pSetSensorAWB->ABS_GAIN_B, bgain_32,
			pSetSensorAWB->ABS_GAIN_GB, gbgain_32);
		subdrv_i2c_wr_u16(ctx, 0x0d82, rgain_32);
		subdrv_i2c_wr_u16(ctx, 0x0d84, ggain_32);
		subdrv_i2c_wr_u16(ctx, 0x0d86, bgain_32);
	}
	return ERROR_NONE;
}

static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
		commit_i2c_buffer(ctx);
	}
	return 0;
}

static int s5kjn5_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	u64 *feature_data = (u64 *)para;
	u32 exp_cnt = 0;
	u32 gain =0;
	if (feature_data == NULL) {
		DRV_LOGE(ctx, "input scenario is null!");
		return ERROR_NONE;
	}
	scenario_id = *feature_data;
	if ((feature_data + 1) != NULL)
		ae_ctrl = (struct mtk_hdr_ae *)((uintptr_t)(*(feature_data + 1)));
	else
		DRV_LOGE(ctx, "no ae_ctrl input");

	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, "E: set seamless switch %u %u\n", ctx->current_scenario_id, scenario_id);
	if (!ctx->extend_frame_length_en)
		DRV_LOGE(ctx, "please extend_frame_length before seamless_switch!\n");
	ctx->extend_frame_length_en = FALSE;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOGE(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		return ERROR_NONE;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_group == 0 ||
		ctx->s_ctx.mode[scenario_id].seamless_switch_group !=
			ctx->s_ctx.mode[ctx->current_scenario_id].seamless_switch_group) {
		DRV_LOGE(ctx, "seamless_switch not supported\n");
		return ERROR_NONE;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table == NULL) {
		DRV_LOGE(ctx, "Please implement seamless_switch setting\n");
		return ERROR_NONE;
	}

	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	gain = ae_ctrl->gain.le_gain/32;
	ctx->is_seamless = TRUE;


	update_mode_info(ctx, scenario_id);
	i2c_table_write(ctx,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);

	if (ae_ctrl) {
		subdrv_i2c_wr_u16(ctx, 0x0202, ae_ctrl->exposure.le_exposure);
		subdrv_i2c_wr_u16(ctx, 0x0204, gain);
	}
	subdrv_i2c_wr_u16(ctx, 0x0104, 0x0001);
	ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	DRV_LOG(ctx, "X: set seamless switch done\n");
	return ERROR_NONE;
}