// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 imx598mipiraw_Sensor.c
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
#include "mot_imx258mipiraw_Sensor.h"

//static int get_sensor_temperature(void *arg);
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int imx258_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx258_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static void mot_imx258_apply_spc_data(void *sensor_ctx);

/* STRUCT */
static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, imx258_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, imx258_set_test_pattern_data},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1040,
			.vsize = 0x0C30,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1040,
			.vsize = 0x0C30,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1040,
			.vsize = 0x0924,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1040,
			.vsize = 0x0C30,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1040,
			.vsize = 0x0C30,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0820,
			.vsize = 0x0492,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = imx258_4160x3120_30fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx258_4160x3120_30fps_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 518400000,
		.linelength = 5352,
		.framelength = 3224,
		.max_framerate = 300,
		.mipi_pixel_rate = 518400000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 6,
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
		.imgsensor_pd_info = NULL,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_data_settle = 59,
			.dphy_clk_settle = 59,
		},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = imx258_4160x3120_30fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx258_4160x3120_30fps_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 518400000,
		.linelength = 5352,
		.framelength = 3224,
		.max_framerate = 307,
		.mipi_pixel_rate = 518400000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 6,
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
		.imgsensor_pd_info = NULL,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_data_settle = 59,
			.dphy_clk_settle = 59,
		},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = imx258_4160x2340_30fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx258_4160x2340_30fps_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 518400000,
		.linelength = 5352,
		.framelength = 3224,
		.max_framerate = 300,
		.mipi_pixel_rate = 518400000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 6,
		.imgsensor_winsize_info = {
			.full_w = 4160,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 390,
			.w0_size = 4160,
			.h0_size = 2340,
			.scale_w = 4160,
			.scale_h = 2340,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4160,
			.h1_size = 2340,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4160,
			.h2_tg_size = 2340,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = NULL,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_data_settle = 59,
			.dphy_clk_settle = 59,
		},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = imx258_4160x3120_30fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx258_4160x3120_30fps_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 518400000,
		.linelength = 5352,
		.framelength = 3224,
		.max_framerate = 300,
		.mipi_pixel_rate = 518400000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 6,
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
		.imgsensor_pd_info = NULL,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_data_settle = 59,
			.dphy_clk_settle = 59,
		},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = imx258_4160x3120_30fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx258_4160x3120_30fps_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 518400000,
		.linelength = 5352,
		.framelength = 3224,
		.max_framerate = 300,
		.mipi_pixel_rate = 518400000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 6,
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
		.imgsensor_pd_info = NULL,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_data_settle = 59,
			.dphy_clk_settle = 59,
		},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	},
	{
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = imx258_2080x1170_60fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx258_2080x1170_60fps_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 518400000,
		.linelength = 5352,
		.framelength = 1612,
		.max_framerate = 600,
		.mipi_pixel_rate = 518400000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 5,
		.imgsensor_winsize_info = {
			.full_w = 4160,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 390,
			.w0_size = 4160,
			.h0_size = 2340,
			.scale_w = 2080,
			.scale_h = 1170,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2080,
			.h1_size = 1170,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2080,
			.h2_tg_size = 1170,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = NULL,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 357,
		.delay_frame = 2,
		.csi_param = {
			.dphy_data_settle = 59,
			.dphy_clk_settle = 59,
		},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = MOT_AITO_IMX258_SENSOR_ID,
	.reg_addr_sensor_id = {0x0A26, 0x0A27},
	.i2c_addr_table = {0x20, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = NULL,
	.eeprom_num = 0,
	.resolution = {4160, 3120},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = 1150,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 0,//0:sony, 1:ov or samusng....etc no used
	.ana_gain_step = 1,// no used
	.ana_gain_table = imx258_ana_gain_table,
	.ana_gain_table_size = sizeof(imx258_ana_gain_table),
	.min_gain_iso = 100, // no change
	.exposure_def = 0x3D0, //no change
	.exposure_min = 16,
	.exposure_max = 0xFFFF - 48,
	.exposure_step = 1, // Get Maximum Step
	.exposure_margin = 10,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = BASE_DGAIN * 1,
	.dig_gain_step = 4,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,///SHUTTER AND GAIN N+1 (long expose used)
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 5500000,

	.pdaf_type = PDAF_SUPPORT_NA,
	.hdr_type = HDR_SUPPORT_NA,  //check this sensor is stagger or not.
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,

	.g_temp = NULL,//get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,
	.s_cali = mot_imx258_apply_spc_data,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {
			{0x0202, 0x0203},
	},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x3100,
	.reg_addr_ana_gain = {
			{0x0204, 0x0205},
	},
	.reg_addr_dig_gain = {
			{0x020E, 0x020F},
	},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = 0x0138,
	.reg_addr_temp_read = 0x013A,
	.reg_addr_auto_extend = 0x0350,
	.reg_addr_frame_count = 0x0005,

	.init_setting_table = imx258_init_setting,
	.init_setting_len = ARRAY_SIZE(imx258_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 0,
	.chk_s_off_end = 0,

	.checksum_value = 0xecaae2a0,
};

int imx258_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = 10;
	u32 addr_h = ctx->s_ctx.reg_addr_sensor_id.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_sensor_id.addr[1];

	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
		        subdrv_i2c_wr_u8(ctx, 0x0A02, 0x0F);
			subdrv_i2c_wr_u8(ctx, 0x0A00, 0x01);

 			if (subdrv_i2c_rd_u8(ctx, 0x0A01) == 0x01)
			{
				*sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) |
					subdrv_i2c_rd_u8(ctx, addr_l);
				*sensor_id = (*sensor_id >> 4) & 0x0fff;
				DRV_LOG(ctx, "i2c_write_id:0x%x sensor_id(cur/exp):0x%x/0x%x\n",
					ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);

				if (*sensor_id == ctx->s_ctx.sensor_id)
					return ERROR_NONE;
			}
			retry--;
		} while (retry > 0);
		i++;
		retry = 10;
	}

	if (*sensor_id != ctx->s_ctx.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


int imx258_open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (imx258_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	if (ctx->s_ctx.aov_sensor_support && !ctx->s_ctx.init_in_open)
		DRV_LOG_MUST(ctx, "sensor init not in open stage!\n");
	else
		sensor_init(ctx);

	if (ctx->s_ctx.s_cali != NULL) {
		DRV_LOG(ctx, "SPC calibration!\n");
		ctx->s_ctx.s_cali((void *) ctx);
	} else
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
}

static struct subdrv_ops ops = {
	.get_id = imx258_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = imx258_open,
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
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 1},
	{HW_ID_AFVDD, 2800000, 1},
	{HW_ID_AVDD, 2800000, 1},
	{HW_ID_DVDD, 1200000, 1},
	{HW_ID_DOVDD, 1800000, 2},
	{HW_ID_RST, 1, 13},
};

const struct subdrv_entry mot_aito_imx258_mipi_raw_entry = {
	.name = "mot_aito_imx258_mipi_raw",
	.id = MOT_AITO_IMX258_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */
#if 0
static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u8 temperature = 0;
	int temperature_convert = 0;

	temperature = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);

	if (temperature < 0x50)
		temperature_convert = temperature;
	else if (temperature < 0x80)
		temperature_convert = 80;
	else if (temperature < 0xED)
		temperature_convert = -20;
	else
		temperature_convert = (char)temperature;

	DRV_LOG(ctx, "temperature: %d degrees\n", temperature_convert);
	return temperature_convert;
}
#endif

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
	/*the below formula is unconditional carry */
	return (1024 - (1024 * BASEGAIN + (gain >> 1)) / gain);
}

static int imx258_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG_MUST(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	if (mode)
		subdrv_i2c_wr_u8(ctx, 0x0601, mode); /*100% Color bar*/
	else if (ctx->test_pattern)
		subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /*No pattern*/

	ctx->test_pattern = mode;

	return ERROR_NONE;
}

static int imx258_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
	subdrv_i2c_wr_u8(ctx, 0x0607, (B & 0xff));
	subdrv_i2c_wr_u8(ctx, 0x0608, (Gb >> 8));
	subdrv_i2c_wr_u8(ctx, 0x0609, (Gb & 0xff));

	DRV_LOG_MUST(ctx, "mode(%u) R/Gr/Gb/B = 0x%04x/0x%04x/0x%04x/0x%04x\n",
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

static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt)
{
#if 0
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
		set_i2c_buffer(ctx, 0x3010, 0x00);
		commit_i2c_buffer(ctx);
	}
#endif
	return 0;
}

#define IMX258_SPC_DEBUG 0
#define IMX258_SPC_DUMP 0
#define IMX258_SPC_EEPROM_ADDR 0xA0
#define IMX258_SPC_DATA_START 0x192A
#define IMX258_SPC_DATA_LEN 126
#define IMX258_SPC_REG1_START 0xD04C
#define IMX258_SPC_REG2_START 0xD08C
static u8 imx258_spc_data[IMX258_SPC_DATA_LEN+2];
static u8 imx258_spc_data_ready = 0;
#if IMX258_SPC_DEBUG
#ifdef DRV_LOG
#undef DRV_LOG
#define DRV_LOG DRV_LOG_MUST
#endif
static u8 imx258_spc_readback[IMX258_SPC_DATA_LEN+2];
#endif


static int imx258_crc_reverse_byte(int data)
{
	return ((data * 0x0802LU & 0x22110LU) |
		(data * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
}

static int32_t imx258_check_crc16(struct subdrv_ctx *ctx, uint8_t  *data, uint32_t size, uint32_t ref_crc)
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
		tmp_reverse = imx258_crc_reverse_byte(data[i]);
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

	crc_reverse = (imx258_crc_reverse_byte(crc_reverse) << 8) |
		imx258_crc_reverse_byte(crc_reverse >> 8);

	if (crc == ref_crc || crc_reverse == ref_crc)
		crc_match = 1;

	DRV_LOG(ctx, "ref_crc 0x%x, crc 0x%x, crc_reverse 0x%x, matches? %d\n",
		ref_crc, crc, crc_reverse, crc_match);

	return crc_match;
}

static int mot_imx258_read_spc_data(struct subdrv_ctx *ctx)
{
	int ret = 0;
	u16 ref_crc = 0;
	if (imx258_spc_data_ready) {
		DRV_LOG_MUST(ctx, "spc data is ready.");
		return 0;
	}

	ret = adaptor_i2c_rd_p8(ctx->i2c_client, (IMX258_SPC_EEPROM_ADDR >> 1), IMX258_SPC_DATA_START, imx258_spc_data, IMX258_SPC_DATA_LEN+2) ;
	if (ret < 0) {
		DRV_LOGE(ctx, "Read SPC data failed. ret:%d", ret);
		return -1;
	}

	ref_crc = ((imx258_spc_data[IMX258_SPC_DATA_LEN] << 8) |imx258_spc_data[IMX258_SPC_DATA_LEN+1]);
	if (imx258_check_crc16(ctx, imx258_spc_data, IMX258_SPC_DATA_LEN, ref_crc)) {
		imx258_spc_data_ready = 1;
		DRV_LOG(ctx, "SPC data ready now.");
	} else {
		/*When CRC error, each time camera open will try to read SPC data from EEPROM, maybe retry for several time is better. Currently
		  we don't avoid retry each time camera open to try best to eliminate potential randomly read failure.*/
		DRV_LOGE(ctx, "SPC data CRC error!");
	}

	return 0;
}

static void mot_imx258_apply_spc_data(void *sensor_ctx)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)sensor_ctx;

	if (!imx258_spc_data_ready) {
		mot_imx258_read_spc_data(ctx);
	}

	if (imx258_spc_data_ready) {
		int ret =	adaptor_i2c_wr_p8(ctx->i2c_client, (ctx->i2c_write_id>>1), IMX258_SPC_REG1_START, imx258_spc_data, IMX258_SPC_DATA_LEN/2);
		if (ret < 0) {
			DRV_LOGE(ctx, "Write Left SPC data failed. ret:%d", ret);
			return;
		}

		ret = adaptor_i2c_wr_p8(ctx->i2c_client, (ctx->i2c_write_id>>1), IMX258_SPC_REG2_START,
		                        &imx258_spc_data[IMX258_SPC_DATA_LEN/2], IMX258_SPC_DATA_LEN/2);
		if (ret < 0) {
			DRV_LOGE(ctx, "Read Right SPC data failed. ret:%d", ret);
			return;
		}
#if IMX258_SPC_DEBUG
		{
			int i;
			u8 spc_en_reg[2];

			ret = adaptor_i2c_rd_p8(ctx->i2c_client, (ctx->i2c_write_id>>1), IMX258_SPC_REG1_START, imx258_spc_readback, IMX258_SPC_DATA_LEN/2) ;
			if (ret < 0) {
				DRV_LOGE(ctx, "L SPC data readback failed. ret:%d", ret);
				return;
			} else {
				for (i=0; i<IMX258_SPC_DATA_LEN/2; i++) {
					if (imx258_spc_data[i] != imx258_spc_readback[i]) {
						DRV_LOGE(ctx, "SPC[%d] E(%02x) != R(%02x)", i, imx258_spc_data[i], imx258_spc_readback[i]);
						return;
					}
				}
				DRV_LOG_MUST(ctx, "L spc readback check pass.");
			}

			ret = adaptor_i2c_rd_p8(ctx->i2c_client, (ctx->i2c_write_id>>1), IMX258_SPC_REG2_START,
			                        &imx258_spc_readback[IMX258_SPC_DATA_LEN/2], IMX258_SPC_DATA_LEN/2) ;
			if (ret < 0) {
				DRV_LOGE(ctx, "R SPC data readback failed. ret:%d", ret);
				return;
			} else {
				for (i=IMX258_SPC_DATA_LEN/2; i<IMX258_SPC_DATA_LEN; i++) {
					if (imx258_spc_data[i] != imx258_spc_readback[i]) {
						DRV_LOGE(ctx, "SPC[%d] E(%02x) != R(%02x)", i, imx258_spc_data[i], imx258_spc_readback[i]);
						return;
					}
				}
				DRV_LOG_MUST(ctx, "R spc readback check pass.");
			}

			ret = adaptor_i2c_rd_p8(ctx->i2c_client, (ctx->i2c_write_id>>1), 0x7BC8, spc_en_reg, 2) ;
			if (ret < 0) {
				DRV_LOGE(ctx, "SPC en reg read failed. ret:%d", ret);
			}
			DRV_LOG(ctx, "SPC en reg:%02x, %02x", spc_en_reg[0], spc_en_reg[1]);

#if IMX258_SPC_DUMP
			for (i=0; i<IMX258_SPC_DATA_LEN; i++) {
				DRV_LOG(ctx, "SPC[%d]\t%02x\t%02x", i, imx258_spc_data[i], imx258_spc_readback[i]);
			}
#endif
		}
#endif
		DRV_LOG_MUST(ctx, "SPC apply done.");
		return;
	}
	DRV_LOGE(ctx, "IMX258 SPC data is NOT ready");
	return;
}
