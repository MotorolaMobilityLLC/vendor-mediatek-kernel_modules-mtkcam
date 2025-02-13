// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5k3p9spmipiraw_Sensor.c
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
#include "s5kgn8mipiraw_Sensor.h"
static void get_sensor_cali(struct subdrv_ctx *ctx);
static u16 get_gain2reg(u32 gain);
static int s5kgn8set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static void s5kgn8sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
static int s5kgn8set_ctrl_locker(struct subdrv_ctx *ctx, u32 cid, bool *is_lock);
static int s5kgn8_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kgn8_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kgn8_lens_position(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);

#define ENABLE_S5KGN8_LONG_EXPOSURE TRUE
#define VENDOR_OV TRUE
#define VENDOR_QT TRUE
#if  ENABLE_S5KGN8_LONG_EXPOSURE
static int s5kgn8_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kgn8_get_margin(struct subdrv_ctx *ctx, u8 *feature_para, u32 *feature_para_len);
static int s5kgn8_get_min_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void s5kgn8_set_shutter_frame_length(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length);
#endif

static int mot_s5kgn8_Manufacturer_ID = 0;
module_param(mot_s5kgn8_Manufacturer_ID,int, 0644);
/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, s5kgn8set_test_pattern},
	{SENSOR_FEATURE_SET_AWB_GAIN, s5kgn8_awb_gain},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, s5kgn8_seamless_switch},
	{SENSOR_FEATURE_SET_LENS_POSITION, s5kgn8_lens_position},
	{SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO,s5kgn8_get_margin},
	{SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, s5kgn8_get_min_shutter},
#if  ENABLE_S5KGN8_LONG_EXPOSURE
	{SENSOR_FEATURE_SET_ESHUTTER, s5kgn8_set_shutter},
#endif
};

static int s5kgn8_get_min_shutter_by_scenario(struct subdrv_ctx *ctx,
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

static int s5kgn8_get_min_shutter(struct subdrv_ctx *ctx, u8 *feature_para, u32 *feature_para_len)
{
	u64 *feature_data = (u64 *) feature_para;
	return s5kgn8_get_min_shutter_by_scenario(ctx,
			(enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
			feature_data + 1, feature_data + 2);
}

static int s5kgn8_get_margin(struct subdrv_ctx *ctx, u8 *feature_para, u32 *feature_para_len)
{

	enum SENSOR_SCENARIO_ID_ENUM scenario_id =ctx->current_scenario_id;
	u64 * margin =  (((u64 *)feature_para)+2);
	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOG(ctx, "invalid cur_sid:%u, mode_num:%u set default\n",
			ctx->current_scenario_id, ctx->s_ctx.sensor_mode_num);
		return -1;
	}
	*(feature_para + 1) = 1;
	if(margin)
	{
		*margin= ctx->s_ctx.mode[scenario_id].read_margin;
		DRV_LOG(ctx, " scenario_id:%d, read_margin:%llu\n",scenario_id, *margin);
	} else {
		DRV_LOGE(ctx, " scenario_id:%d get margin fail,margin is NULL\n",scenario_id);
	}
	return ERROR_NONE;
}
#if  ENABLE_S5KGN8_LONG_EXPOSURE
static void s5kgn8_set_long_exposure(struct subdrv_ctx *ctx)
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

static void s5kgn8_set_shutter_frame_length(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length)
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
	s5kgn8_set_long_exposure(ctx);
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

static int s5kgn8_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 shutter = *((u64 *)para);
	s5kgn8_set_shutter_frame_length(ctx, shutter,0);
	return 0;
}
#endif
static int s5kgn8_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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


static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
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
			.channel = 1,
			.data_type = 0x30,
			.hsize = 0x0400,
			.vsize = 0x0c00,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
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
		},
	},

	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x30,
			.hsize = 0x0400,
			.vsize = 0x0c00,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},

};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2c,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},

	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x30,
			.hsize = 0x0400,
			.vsize = 0x0900,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.valid_bit = 10,
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
		},
	},

	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x30,
			.hsize = 0x0400,
			.vsize = 0x0c00,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},

};


static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
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

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
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
			.channel = 1,
			.data_type = 0x30,
			.hsize = 0x0400,
			.vsize = 0x0900,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},

};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
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


static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2c,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},

	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x30,
			.hsize = 0x0400,
			.vsize = 0x0900,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.valid_bit = 10,
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
			.channel = 1,
			.data_type = 0x30,
			.hsize = 0x0200,
			.vsize = 0x0600,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.valid_bit = 10,
		},
	},

};





static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
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
			.channel = 1,
			.data_type = 0x30,
			.hsize = 0x0200,
			.vsize = 0x0600,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},

};
//1000 base for dcg gain ratio
static u32 s5kgn8_dcg_ratio_table_12bit[] = {1000};
static struct mtk_sensor_saturation_info imgsensor_saturation_info_12bit = {
	.gain_ratio = 1000,
	.OB_pedestal = 256,
	.saturation_level = 4095,
};
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
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
	.i4LeFirst = 0,
	.i4Crop = {
		// <pre> <cap> <normal_video> <hs_video> <<slim_video>>
		{0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0},
		// <<cust1>> <<cust2>> <<cust3>> <cust4> <cust5>
		{0, 0}, {0, 384}, {0, 0}, {0, }, {0, 0},
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
	.iMirrorFlip = 0,
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.i4ModeIndex = 3,
	.i4VCPackNum = 4,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		.i4PDPattern = 1, //all PD
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {0,1}, // R = 1, L = 0
	},
};


static struct SET_PD_BLOCK_INFO_T imgsensor_pd_cus5_info = {
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
	.i4LeFirst = 0,
	.i4Crop = {
		// <pre> <cap> <normal_video> <hs_video> <<slim_video>>
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// <<cust1>> <<cust2>> <<cust3>> <cust4> <cust5>
		{0, 0}, {0, 0}, {0, 0}, {0, }, {0, 0},
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
	.iMirrorFlip = 0,
	.i4FullRawW = 2048,
	.i4FullRawH = 1536,
	.i4ModeIndex = 3,
	.i4VCPackNum = 4,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		.i4PDPattern = 1, //all PD
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {0,1}, // R = 1, L = 0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_cus6_info = {
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
	.i4LeFirst = 0,
	.i4Crop = {
		// <pre> <cap> <normal_video> <hs_video> <<slim_video>>
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// <<cust1>> <<cust2>> <<cust3>> <cust4> <cust5>
		{0, 0}, {0, 0}, {0, 0}, {0, }, {0, 0},
		// <cust6> <cust7> <cust8> cust9 cust10
		{2048, 1536}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// cust11 cust12 cust13 <cust14> <cust15>
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// <cust16> <cust17> cust18 <cust19> cust20
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// <cust21> <cust22> <cust23> <cust24> <cust25>
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// cust26 <cust27> cust28
		{0, 0}, {0, 0}, {0, 0},
	},
	.iMirrorFlip = 0,
	.i4FullRawW = 8192,
	.i4FullRawH = 6144,
	.i4ModeIndex = 3,
	.i4VCPackNum = 4,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		.i4PDPattern = 1, //all PD
		.i4BinFacX = 4,
		.i4BinFacY = 8,
		.i4PDRepetition = 0,
		.i4PDOrder = {0,1}, // R = 1, L = 0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_vid_info = {
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
	.i4LeFirst = 0,
	.i4Crop = {
		// <pre> <cap> <normal_video> <hs_video> <<slim_video>>
		{0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0},
		// <<cust1>> <<cust2>> <<cust3>> <cust4> <cust5>
		{0, 0}, {0, 384}, {0, 0}, {0, }, {0, 0},
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
	.iMirrorFlip = 0,
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.i4ModeIndex = 3,
	.i4VCPackNum = 4,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		.i4PDPattern = 1, //all PD
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {0,1}, // R = 1, L = 0
	},
};

#if VENDOR_OV
static struct subdrv_mode_struct mode_ov_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = OV_addr_data_pair_preview,
		.mode_setting_len = ARRAY_SIZE(OV_addr_data_pair_preview),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = OV_s5kgn8_seamless_preview,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(OV_s5kgn8_seamless_preview),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 8136,
		.framelength = 6556,
		.max_framerate = 300,
		.mipi_pixel_rate = 2079360000,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
	        .min_exposure_line = 16,
		.read_margin =48,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 64,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = OV_addr_data_pair_capture,
		.mode_setting_len = ARRAY_SIZE(OV_addr_data_pair_capture),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 8136,
		.framelength = 6556,
		.max_framerate = 300,
		.mipi_pixel_rate = 2079360000,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
	        .min_exposure_line = 16,
		.read_margin =48,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 64,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = OV_addr_data_pair_normal_video,
		.mode_setting_len = ARRAY_SIZE(OV_addr_data_pair_normal_video),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = OV_s5kgn8_seamless_normal_video,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(OV_s5kgn8_seamless_normal_video),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 8136,
		.framelength = 6554,
		.max_framerate = 300,
		.mipi_pixel_rate = 1732800000,
		.readout_length = 0,
		.saturation_info = &imgsensor_saturation_info_12bit,
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
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW12_Gr,
		.imgsensor_pd_info = &imgsensor_pd_vid_info,
	        .min_exposure_line = 16,
		.read_margin =48,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 64,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = OV_addr_data_pair_hs_video,
		.mode_setting_len = ARRAY_SIZE(OV_addr_data_pair_hs_video),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 5000,
		.framelength = 2660,
		.max_framerate = 1200,
		.mipi_pixel_rate = 2079360000,
		.readout_length = 0,
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
	        .min_exposure_line = 16,
		.read_margin =48,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 64,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = OV_addr_data_pair_slim_video,
		.mode_setting_len = ARRAY_SIZE(OV_addr_data_pair_slim_video),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 8136,
		.framelength = 6556,
		.max_framerate = 300,
		.mipi_pixel_rate = 2079360000,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
	        .min_exposure_line = 16,
		.read_margin =48,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 64,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},

	{//custom1
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = OV_addr_data_pair_custom1,
		.mode_setting_len = ARRAY_SIZE(OV_addr_data_pair_custom1),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 9544,
		.framelength = 6984,
		.max_framerate = 240,
		.mipi_pixel_rate = 2079360000,
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
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
	        .min_exposure_line = 32,
		.read_margin =96,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 16,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom2
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = OV_addr_data_pair_custom2,
		.mode_setting_len = ARRAY_SIZE(OV_addr_data_pair_custom2),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 8136,
		.framelength = 3276,
		.max_framerate = 600,
		.mipi_pixel_rate = 2079360000,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
	        .min_exposure_line = 16,
		.read_margin =48,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 64,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom3
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = OV_addr_data_pair_custom3,
		.mode_setting_len = ARRAY_SIZE(OV_addr_data_pair_custom3),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 5000,
		.framelength = 1332,
		.max_framerate = 2400,
		.mipi_pixel_rate = 2079360000,
		.readout_length = 0,
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
	        .min_exposure_line = 16,
		.read_margin =48,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 64,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom4
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = OV_addr_data_pair_custom4,
		.mode_setting_len = ARRAY_SIZE(OV_addr_data_pair_custom4),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = OV_s5kgn8_seamless_custom4,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(OV_s5kgn8_seamless_custom4),
		.hdr_mode = HDR_RAW_DCG_COMPOSE,
		.raw_cnt = 1,
		.exp_cnt = 2,
		.pclk = 1600000000,
		.linelength = 19080,
		.framelength = 2788,
		.max_framerate = 300,
		.mipi_pixel_rate = 1732800000,
		.readout_length = 0,
		.read_margin = 0,
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
		.imgsensor_pd_info = &imgsensor_pd_vid_info,
	        .min_exposure_line = 8,
		.read_margin =24,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 16,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW12_Gr,
		.saturation_info = &imgsensor_saturation_info_12bit,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_COMPOSE,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_LCG_BASE,
			.dcg_gain_ratio_min = 1000,
			.dcg_gain_ratio_max = 1000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = s5kgn8_dcg_ratio_table_12bit,
			.dcg_gain_table_size = sizeof(s5kgn8_dcg_ratio_table_12bit),
		},
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 16,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 16,
	},
	{//custom5
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = OV_addr_data_pair_custom5,
		.mode_setting_len = ARRAY_SIZE(OV_addr_data_pair_custom5),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 5000,
		.framelength = 10668,
		.max_framerate = 300,
		.mipi_pixel_rate = 2079360000,
		.readout_length = 0,
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
		.imgsensor_pd_info =  &imgsensor_pd_cus5_info,
	        .min_exposure_line = 16,
		.read_margin =48,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 64,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},

	{//custom6  full crop
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = OV_addr_data_pair_custom6,
		.mode_setting_len = ARRAY_SIZE(OV_addr_data_pair_custom6),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = OV_s5kgn8_seamless_custom6,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(OV_s5kgn8_seamless_custom6),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 9544,
		.framelength = 5588,
		.max_framerate = 300,
		.mipi_pixel_rate = 2079360000,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_cus6_info,
		.min_exposure_line = 32,
		.read_margin = 96,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 16,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
};

static struct subdrv_static_ctx static_ov_ctx = {
	.sensor_id = MOT_AITO_S5KGN8_SENSOR_ID,
	.reg_addr_sensor_id = {0x0000, 0x0001},
	.i2c_addr_table = {0x20, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info = 0,
	.eeprom_num = 0,
	.resolution = {8192, 6144},
	.mirror = IMAGE_NORMAL,
	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 2,
	.ana_gain_step = 2,
	.ana_gain_table = PARAM_UNDEFINED,
	.ana_gain_table_size = PARAM_UNDEFINED,
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 16,                     //def mode
	.exposure_max = (0xFFFF*128) - 48,      //def mode
	.exposure_step = 1,
	.exposure_margin = 48,                  //def mode

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 500000,    // CTS sensor fusion test
	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
	.hdr_type = HDR_SUPPORT_DCG,
	.seamless_switch_support = TRUE,
	.temperature_support = FALSE,
	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = get_gain2reg,
	.s_gph = PARAM_UNDEFINED,
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
	.reg_addr_frame_count = 0x0005,             //samsung need

	.init_setting_table = PARAM_UNDEFINED,
	.init_setting_len = PARAM_UNDEFINED,
	.mode = mode_ov_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_ov_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,

	.checksum_value = 0x31E3FBE2,

	/* custom stream control delay timing for hw limitation (ms) */
	.custom_stream_ctrl_delay = 0,
};
#endif


#if VENDOR_QT
static struct subdrv_mode_struct mode_qt_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = QT_addr_data_pair_preview,
		.mode_setting_len = ARRAY_SIZE(QT_addr_data_pair_preview),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = QT_s5kgn8_seamless_preview,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(QT_s5kgn8_seamless_preview),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 8136,
		.framelength = 6556,
		.max_framerate = 300,
		.mipi_pixel_rate = 2079360000,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
	        .min_exposure_line = 16,
		.read_margin =48,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 64,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = QT_addr_data_pair_capture,
		.mode_setting_len = ARRAY_SIZE(QT_addr_data_pair_capture),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 8136,
		.framelength = 6556,
		.max_framerate = 300,
		.mipi_pixel_rate = 2079360000,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
	        .min_exposure_line = 16,
		.read_margin =48,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 64,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = QT_addr_data_pair_normal_video,
		.mode_setting_len = ARRAY_SIZE(QT_addr_data_pair_normal_video),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = QT_s5kgn8_seamless_normal_video,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(QT_s5kgn8_seamless_normal_video),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 8136,
		.framelength = 6554,
		.max_framerate = 300,
		.mipi_pixel_rate = 1732800000,
		.readout_length = 0,
		.saturation_info = &imgsensor_saturation_info_12bit,
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
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW12_Gr,
		.imgsensor_pd_info = &imgsensor_pd_vid_info,
	        .min_exposure_line = 16,
		.read_margin =48,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 64,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = QT_addr_data_pair_hs_video,
		.mode_setting_len = ARRAY_SIZE(QT_addr_data_pair_hs_video),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 5000,
		.framelength = 2660,
		.max_framerate = 1200,
		.mipi_pixel_rate = 2079360000,
		.readout_length = 0,
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
	        .min_exposure_line = 16,
		.read_margin =48,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 64,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = QT_addr_data_pair_slim_video,
		.mode_setting_len = ARRAY_SIZE(QT_addr_data_pair_slim_video),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 8136,
		.framelength = 6556,
		.max_framerate = 300,
		.mipi_pixel_rate = 2079360000,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
	        .min_exposure_line = 16,
		.read_margin =48,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 64,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},

	{//custom1
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = QT_addr_data_pair_custom1,
		.mode_setting_len = ARRAY_SIZE(QT_addr_data_pair_custom1),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 9544,
		.framelength = 6984,
		.max_framerate = 240,
		.mipi_pixel_rate = 2079360000,
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
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
	        .min_exposure_line = 32,
		.read_margin =96,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 16,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom2
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = QT_addr_data_pair_custom2,
		.mode_setting_len = ARRAY_SIZE(QT_addr_data_pair_custom2),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 8136,
		.framelength = 3276,
		.max_framerate = 600,
		.mipi_pixel_rate = 2079360000,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
	        .min_exposure_line = 16,
		.read_margin =48,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 64,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom3
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = QT_addr_data_pair_custom3,
		.mode_setting_len = ARRAY_SIZE(QT_addr_data_pair_custom3),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 5000,
		.framelength = 1332,
		.max_framerate = 2400,
		.mipi_pixel_rate = 2079360000,
		.readout_length = 0,
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
	        .min_exposure_line = 16,
		.read_margin =48,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 64,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom4
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = QT_addr_data_pair_custom4,
		.mode_setting_len = ARRAY_SIZE(QT_addr_data_pair_custom4),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = QT_s5kgn8_seamless_custom4,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(QT_s5kgn8_seamless_custom4),
		.hdr_mode = HDR_RAW_DCG_COMPOSE,
		.raw_cnt = 1,
		.exp_cnt = 2,
		.pclk = 1600000000,
		.linelength = 19080,
		.framelength = 2788,
		.max_framerate = 300,
		.mipi_pixel_rate = 1732800000,
		.readout_length = 0,
		.read_margin = 0,
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
		.imgsensor_pd_info = &imgsensor_pd_vid_info,
	        .min_exposure_line = 8,
		.read_margin =24,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 16,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW12_Gr,
		.saturation_info = &imgsensor_saturation_info_12bit,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_COMPOSE,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_LCG_BASE,
			.dcg_gain_ratio_min = 1000,
			.dcg_gain_ratio_max = 1000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = s5kgn8_dcg_ratio_table_12bit,
			.dcg_gain_table_size = sizeof(s5kgn8_dcg_ratio_table_12bit),
		},
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 16,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 16,
	},
	{//custom5
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = QT_addr_data_pair_custom5,
		.mode_setting_len = ARRAY_SIZE(QT_addr_data_pair_custom5),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 5000,
		.framelength = 10668,
		.max_framerate = 300,
		.mipi_pixel_rate = 2079360000,
		.readout_length = 0,
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
		.imgsensor_pd_info =  &imgsensor_pd_cus5_info,
	        .min_exposure_line = 16,
		.read_margin =48,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 64,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},

	{//custom6  full crop
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = QT_addr_data_pair_custom6,
		.mode_setting_len = ARRAY_SIZE(QT_addr_data_pair_custom6),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = QT_s5kgn8_seamless_custom6,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(QT_s5kgn8_seamless_custom6),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1600000000,
		.linelength = 9544,
		.framelength = 5588,
		.max_framerate = 300,
		.mipi_pixel_rate = 2079360000,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_cus6_info,
		.min_exposure_line = 32,
		.read_margin = 96,
                .ana_gain_min = BASEGAIN * 1,
                .ana_gain_max = BASEGAIN * 16,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
};

static struct subdrv_static_ctx static_qt_ctx = {
	.sensor_id = MOT_AITO_S5KGN8_SENSOR_ID,
	.reg_addr_sensor_id = {0x0000, 0x0001},
	.i2c_addr_table = {0x20, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info = 0,
	.eeprom_num = 0,
	.resolution = {8192, 6144},
	.mirror = IMAGE_NORMAL,
	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 2,
	.ana_gain_step = 2,
	.ana_gain_table = PARAM_UNDEFINED,
	.ana_gain_table_size = PARAM_UNDEFINED,
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 16,                     //def mode
	.exposure_max = (0xFFFF*128) - 48,      //def mode
	.exposure_step = 1,
	.exposure_margin = 48,                  //def mode

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 500000,    // CTS sensor fusion test
	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
	.hdr_type = HDR_SUPPORT_DCG,
	.seamless_switch_support = TRUE,
	.temperature_support = FALSE,
	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = get_gain2reg,
	.s_gph = PARAM_UNDEFINED,
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
	.reg_addr_frame_count = 0x0005,             //samsung need

	.init_setting_table = PARAM_UNDEFINED,
	.init_setting_len = PARAM_UNDEFINED,
	.mode = mode_qt_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_qt_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,

	.checksum_value = 0x31E3FBE2,

	/* custom stream control delay timing for hw limitation (ms) */
	.custom_stream_ctrl_delay = 0,
};
#endif
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
	.set_ctrl_locker = s5kgn8set_ctrl_locker,
	.vsync_notify = vsync_notify,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 0},
	{HW_ID_DOVDD, 1800000, 7},
	{HW_ID_DVDD, 1000000, 1},
	{HW_ID_AVDD, 2200000, 4},
	{HW_ID_MCLK_DRIVING_CURRENT, 6, 0},
	{HW_ID_RST, 1, 14}
};

const struct subdrv_entry mot_aito_s5kgn8_mipi_raw_entry = {
	.name = "mot_aito_s5kgn8_mipi_raw",
	.id = MOT_AITO_S5KGN8_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};


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
#define  S5KGN8_AF_DATA_START 0x0027
#define  S5KGN8_AF_DATA_LEN 24
#define  S5KGN8_EEPROM_ADDR 0xA0
static u16 af_macro_val =0;
static u16 af_inf_val =0;
static bool s5kgn8_af_data_ready = FALSE;
static void get_sensor_cali(struct subdrv_ctx *ctx)
{

	int ret = 0;
	u16 ref_crc = 0;
	u8 s5kgn8_af_data[26];
	if (s5kgn8_af_data_ready) {
		DRV_LOG_MUST(ctx, "af data is ready.");
		return;
	}
	ret = adaptor_i2c_rd_p8(ctx->i2c_client, (S5KGN8_EEPROM_ADDR >> 1), S5KGN8_AF_DATA_START, s5kgn8_af_data, S5KGN8_AF_DATA_LEN+2) ;
	if (ret < 0) {
		DRV_LOGE(ctx, "Read af data failed. ret:%d", ret);
		s5kgn8_af_data_ready = FALSE;
		return;
	}
	ref_crc = ((s5kgn8_af_data[S5KGN8_AF_DATA_LEN] << 8) |s5kgn8_af_data[S5KGN8_AF_DATA_LEN+1]);
	if (check_crc16(s5kgn8_af_data, S5KGN8_AF_DATA_LEN, ref_crc)) {
		s5kgn8_af_data_ready = TRUE;
		DRV_LOG(ctx, "af data ready now.");
	} else {
		DRV_LOGE(ctx, "AF data CRC error!");
	}

	if( s5kgn8_af_data_ready)
	{
		af_macro_val = (s5kgn8_af_data[2]<<8 | s5kgn8_af_data[3])/64;
		af_inf_val = (s5kgn8_af_data[6]<<8 | s5kgn8_af_data[7])/64;
	}
	DRV_LOG(ctx, "af_macro_val =%d  af_inf_val =%d \n", af_macro_val,af_inf_val);
}

static int s5kgn8_lens_position(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 lens_position_reg_val;
	u32 lens_position = *((u32 *)para);
	if(s5kgn8_af_data_ready == FALSE)
	{
		DRV_LOG(ctx, "s5kgn8_af_data_ready =%d \n", s5kgn8_af_data_ready);
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
		(ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM6))
	{
		lens_position_reg_val = (lens_position-af_inf_val)*1023 /(af_macro_val -af_inf_val);
		DRV_LOG(ctx, "lens_position =%d  lens_position_reg_val =%d",lens_position,lens_position_reg_val);
		DRV_LOG(ctx, "af_macro_val =%d  af_inf_val =%d",af_macro_val,af_inf_val);
		subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x2000);
		subdrv_i2c_wr_u16(ctx, 0x3592, lens_position_reg_val);
		subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
	}
	return 0;
}
static void s5kgn8_check_manufacturer_id(struct subdrv_ctx *ctx)
{

	int ret = 0;
	u8 s5kgn8_Manufacturer[2];
	u8 retry = 3;
	if(mot_s5kgn8_Manufacturer_ID != 0)
	{
       		return;
	}

	do {
		ret = adaptor_i2c_rd_p8(ctx->i2c_client, (S5KGN8_EEPROM_ADDR >> 1),
						0x000d, s5kgn8_Manufacturer, 2) ;
		if (ret < 0) {
			DRV_LOGE(ctx, "Read eeprom Manufacturer data failed. ret:%d retry=%d \n", ret,retry);
			mot_s5kgn8_Manufacturer_ID = 0;
		}
       		if((s5kgn8_Manufacturer[0]  == 0x4f)&& (s5kgn8_Manufacturer[1]  == 0x46))
       		{
       			mot_s5kgn8_Manufacturer_ID =1;
			DRV_LOG(ctx, "s5kgn8 is OV module\n");
			return;
		}
       		if((s5kgn8_Manufacturer[0]  == 0x51)&& (s5kgn8_Manufacturer[1]  == 0x54))
       		{
       			mot_s5kgn8_Manufacturer_ID =2;
			DRV_LOG(ctx, "s5kgn8 is QT module\n");
			return;
		}
		retry--;
		mdelay(2);
	} while (retry > 0);
	return;
}

static u16 get_gain2reg(u32 gain)
{
	return gain * 32 / BASEGAIN;
}

#define REG_GAIN 0x0400
#define WB_GAIN_FACTOR 512
static int s5kgn8_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct SET_SENSOR_AWB_GAIN *pSetSensorAWB = (( struct SET_SENSOR_AWB_GAIN  *)para);

	UINT32 rgain_32, grgain_32, gbgain_32, bgain_32, ggain_32;
	if ((ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM1) ||
		(ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM6))
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

static int s5kgn8set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	if (mode) {
		subdrv_i2c_wr_u16(ctx, 0x0600, 0x0001); /*Black*/
		subdrv_i2c_wr_u16(ctx, 0x0620, 0x0001); /*Black*/
	} else if (ctx->test_pattern) {
		subdrv_i2c_wr_u16(ctx, 0x0600, 0x0000); /*No pattern*/
		subdrv_i2c_wr_u16(ctx, 0x0620, 0x0000); /*No pattern*/
	}
	ctx->test_pattern = mode;
	return ERROR_NONE;
}

static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	s5kgn8_check_manufacturer_id(ctx);
	if(mot_s5kgn8_Manufacturer_ID == 1){
		memcpy(&(ctx->s_ctx), &static_ov_ctx, sizeof(struct subdrv_static_ctx));
	} else if(mot_s5kgn8_Manufacturer_ID == 2){
		memcpy(&(ctx->s_ctx), &static_qt_ctx, sizeof(struct subdrv_static_ctx));
	} else {
		memcpy(&(ctx->s_ctx), &static_ov_ctx, sizeof(struct subdrv_static_ctx));
	}
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;

	return 0;
}

static void s5kgn8sensor_init(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "E\n");
	mdelay(10);
	i2c_table_write(ctx, uTnpArrayInit_0,ARRAY_SIZE(uTnpArrayInit_0));
	mdelay(10);
	// 1 4 5 7
	i2c_table_write(ctx, uTnpArrayInit_1,ARRAY_SIZE(uTnpArrayInit_1));
	mot_subdrv_i2c_wr_burst_p16(ctx, 0x6F12,uTnpArrayInit_2,
		ARRAY_SIZE(uTnpArrayInit_2));
	i2c_table_write(ctx, uTnpArrayInit_3,ARRAY_SIZE(uTnpArrayInit_3));
	if(mot_s5kgn8_Manufacturer_ID == 1)
	{
		i2c_table_write(ctx, uTnpArray_ov_gos,ARRAY_SIZE(uTnpArray_ov_gos));
	} else if (mot_s5kgn8_Manufacturer_ID == 2) {
		i2c_table_write(ctx, uTnpArray_qt_gos,ARRAY_SIZE(uTnpArray_qt_gos));
	} else {
		i2c_table_write(ctx, uTnpArray_ov_gos,ARRAY_SIZE(uTnpArray_ov_gos));
	}
	DRV_LOG(ctx, "X\n");
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (common_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;
	if(mot_s5kgn8_Manufacturer_ID == 0)
	{
		DRV_LOGE(ctx, "Read eeprom Manufacturer data failed. used default sensor ov\n");
	}
	/* initail setting */
	s5kgn8sensor_init(ctx);

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
} /* open */

static int s5kgn8set_ctrl_locker(struct subdrv_ctx *ctx,
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
} /* s5kgn8set_ctrl_locker */


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

