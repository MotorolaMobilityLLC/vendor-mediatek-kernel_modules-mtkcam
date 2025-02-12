
// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 motniceov08dmipiraw_Sensor.c
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
#include "mot_nice_ov08dmipiraw_Sensor.h"
#include "mot_nice_ov08dSensor_setting.h"
#define PFX "NOV08D"
#define LOG_INF(format, args...) pr_info(PFX "[%s] " format, __func__, ##args)
int vblank_convert = 0;
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int motniceov08d_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int ov08d_streaming_on(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int ov08d_streaming_off(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int ov08d_set_gain_by(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int ov08d_set_frame_length_by(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int ov08d_set_shutter_frame_length_by(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int ov08d_set_shutter_by(struct subdrv_ctx *ctx,  u8 *para, u32 *len);
static int ov08d_set_max_framerate_by_scenario_by(struct subdrv_ctx *ctx,u8 *para, u32 *len);
static int motniceov08d_get_imgsensor_id(struct subdrv_ctx *ctx, UINT32 *sensor_id);
static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND,ov08d_streaming_off},
	{SENSOR_FEATURE_SET_STREAMING_RESUME,ov08d_streaming_on},
	{SENSOR_FEATURE_SET_ESHUTTER,ov08d_set_shutter_by},
	{SENSOR_FEATURE_SET_GAIN,ov08d_set_gain_by},
	{SENSOR_FEATURE_SET_FRAMELENGTH,ov08d_set_frame_length_by},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME,ov08d_set_shutter_frame_length_by},
	{SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO,ov08d_set_max_framerate_by_scenario_by},
	{SENSOR_FEATURE_SET_TEST_PATTERN,motniceov08d_set_test_pattern},
};
/* STRUCT */
static struct mtk_mbus_frame_desc_entry frame_desc_cap [] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,
			.vsize = 0x0990,
			//.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,
			.vsize = 0x0990,
			//.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,
			.vsize = 0x0990,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,
			.vsize = 0x0990,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,
			.vsize = 0x0990,
		},
	},
};
static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = addr_data_pair_preview_mot_nice_ov08d,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_preview_mot_nice_ov08d),
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 36000000,    /*record different mode's pclk*/
		.linelength  =  460,    /*record different mode's linelength*/
		.framelength = 2608,    /*record different mode's framelength*/
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
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
		.aov_mode = 0,
		.s_dummy_support = 1,
		.ae_ctrl_support = 1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 85,
		},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = addr_data_pair_capture_mot_nice_ov08d,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_capture_mot_nice_ov08d),
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 36000000,    /*record different mode's pclk*/
		.linelength  =  460,    /*record different mode's linelength*/
		.framelength = 2608,    /*record different mode's framelength*/
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
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
		.aov_mode = 0,
		.s_dummy_support = 1,
		.ae_ctrl_support = 1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 85,
		},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = addr_data_pair_video_mot_nice_ov08d,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_video_mot_nice_ov08d),
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 36000000,    /*record different mode's pclk*/
		.linelength  =  460,    /*record different mode's linelength*/
		.framelength = 2608,    /*record different mode's framelength*/
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
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
		.aov_mode = 0,
		.s_dummy_support = 1,
		.ae_ctrl_support = 1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 85,
		},
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = addr_data_pair_hs_video_mot_nice_ov08d,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_slim_video_mot_nice_ov08d),
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 36000000,    /*record different mode's pclk*/
		.linelength  =  460,    /*record different mode's linelength*/
		.framelength = 2608,    /*record different mode's framelength*/
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
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
		.aov_mode = 0,
		.s_dummy_support = 1,
		.ae_ctrl_support = 1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 85,
		},
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = addr_data_pair_slim_video_mot_nice_ov08d,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_slim_video_mot_nice_ov08d),
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 36000000,    /*record different mode's pclk*/
		.linelength  =  460,    /*record different mode's linelength*/
		.framelength = 2608,    /*record different mode's framelength*/
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
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
		.aov_mode = 0,
		.s_dummy_support = 1,
		.ae_ctrl_support = 1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 85,
		},
	},
};
static struct subdrv_static_ctx static_ctx = {
	.sensor_id = MOT_NICE_OV08D_SENSOR_ID,
	.reg_addr_sensor_id = {0x0001, 0x0002},
	.i2c_addr_table = {0x20, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.resolution = {3264, 2448},
	.mirror = IMAGE_NORMAL,
	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_2MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.ob_pedestal = 0x40,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 15.5,
	.ana_gain_type = 1,
	.ana_gain_step = 1,
	.dig_gain_min = 1,
	.dig_gain_max = 1,
	.dig_gain_step = 1,
	.ana_gain_table = ov08d_ana_gain_table,
	.ana_gain_table_size = sizeof(ov08d_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,  //24
	.exposure_max = 78260,
	.exposure_step = 1, //4
	.exposure_margin = 20,
	.frame_length_max = 0x89c7,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 0,
	.pdaf_type = PDAF_SUPPORT_NA,
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = false,
	.temperature_support = false,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,
	//.s_cali = set_sensor_cali,
	//.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = PARAM_UNDEFINED,
	.reg_addr_exposure = {{0x05,0x06},},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x02,
	//.reg_addr_ana_gain = {},
	.reg_addr_frame_length = {0x34, 0x35, 0x36},
	//.reg_addr_temp_en = 0x4D12,
	//.reg_addr_temp_read = 0x4D13,
	.reg_addr_auto_extend = 0,
	//.reg_addr_frame_count = 0x387F,
	.init_setting_table = addr_data_pair_init_mot_nice_ov08d,
	.init_setting_len = ARRAY_SIZE(addr_data_pair_init_mot_nice_ov08d),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,
	.checksum_value = 0x43daf615,
	.aov_sensor_support = FALSE,
	.init_in_open = TRUE,
	.streaming_ctrl_imp = FALSE,
};

int ov08d_i2c_rd_a8_d8(struct i2c_client *i2c_client, u8 *a_pSendData, u16 a_sizeSendData,
		u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId)
{
	int  i4RetValue = 0;
	struct i2c_msg msg[2];

	i2c_client->addr = (i2cId );

	msg[0].addr = i2cId;
	msg[0].flags = i2c_client->flags & I2C_M_TEN;
	msg[0].len = a_sizeSendData;
	msg[0].buf = a_pSendData;

	msg[1].addr = i2cId;
	msg[1].flags = i2c_client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = a_sizeRecvData;
	msg[1].buf = a_pRecvData;

	i4RetValue = i2c_transfer(i2c_client->adapter, msg, 2);

	if (i4RetValue != 2) {
		dev_info(&i2c_client->dev, "i2c transfer failed (%d)\n", i4RetValue);
		return -1;
	}
	return 0;
}


int ov08d_i2c_wr_a8_d8(struct i2c_client *i2c_client,
		u16 addr, u8 reg, u8 val)
{
	int ret;
	u8 buf[2];
	struct i2c_msg msg;

	if (i2c_client == NULL)
		return -ENODEV;

	buf[0] = reg & 0xff;
	buf[1] = val & 0xff;

	msg.addr = addr;
	msg.flags = i2c_client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(i2c_client->adapter, &msg, 1);
	if (ret < 0)
		dev_info(&i2c_client->dev, "i2c transfer failed (%d)\n", ret);

	return ret;
}


static kal_uint16 ov08d_table_write_cmos_sensor(struct subdrv_ctx *ctx,
		kal_uint16 *para, kal_uint32 len)
{
	kal_uint32 IDX;

	//pr_debug("%s len %d\n", __func__, len);
	IDX = 0;
	while (len > IDX) {
	    write_cmos_sensor_8(ctx, para[IDX]&0xFF, para[IDX+1]&0xFF);
		IDX += 2;
	}
	return 0;
}

static void ov08d_set_dummy(struct subdrv_ctx *ctx)
{
	if (ctx->frame_length%2 != 0) {
		ctx->frame_length = ctx->frame_length - ctx->frame_length % 2;
	}

	ctx->frame_length = ((ctx->frame_length + 3) >> 2) << 2;// need to set to  multi 4
	if (ctx->frame_length > static_ctx.frame_length_max) {
		ctx->frame_length = static_ctx.frame_length_max;
	}
	write_cmos_sensor_8(ctx, 0xfd, 0x01);
	write_cmos_sensor_8(ctx, 0x05, (((ctx->frame_length - vblank_convert)*2) & 0xFF00) >> 8);
	write_cmos_sensor_8(ctx, 0x06, ((ctx->frame_length - vblank_convert)*2) & 0xFF);
	write_cmos_sensor_8(ctx, 0x01, 0x01);
}

static int longexposue = 0;
static void ov08d_set_long_exposure(struct subdrv_ctx *ctx)
{
	u32 shutter = ctx->exposure[0];
	u32 l_shutter = 0;
	u16 l_shift = 1;
	u32 cal_shutter = 0;

	if (shutter > ctx->s_ctx.exposure_max) {
		DRV_LOG(ctx, "ov08d enter long exposure!");
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
		write_cmos_sensor_8(ctx, 0xfd, 0x01);
		write_cmos_sensor_8(ctx, 0x24, 0x10);
		write_cmos_sensor_8(ctx, 0x02, 0x02);
		write_cmos_sensor_8(ctx, 0x03, 0x63);
		write_cmos_sensor_8(ctx, 0x04, 0x69);

		//write_cmos_sensor_8(ctx, ctx->s_ctx.reg_addr_exposure_lshift, (shutter*2 >> 16) & 0xFF);
		//write_cmos_sensor_8(ctx, ctx->s_ctx.reg_addr_exposure_lshift+1, (shutter*2 >> 8) & 0xFF);
		//write_cmos_sensor_8(ctx, ctx->s_ctx.reg_addr_exposure_lshift+2, (shutter*2) & 0xFF);

		write_cmos_sensor_8(ctx, 0x01, 0x01);

		shutter = ((shutter - 1) >> l_shift) + 1;
		shutter = min(shutter, ctx->s_ctx.exposure_max);
		ctx->frame_length = shutter + ctx->s_ctx.exposure_margin;
		ctx->frame_length_rg = ctx->frame_length;
		ctx->l_shift = l_shift;
		DRV_LOG(ctx, "long exposure mode: lshift %u times, normal shutter=0x%x, long shutter=0x%x\n",
				l_shift,
				cal_shutter - shutter - 1,
				cal_shutter);

		/* Frame exposure mode customization for LE*/
		ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		ctx->current_ae_effective_frame = 2;
	} else {
		if (longexposue == 1) {
			DRV_LOG(ctx, "ov08d exit long exposure!");
			write_cmos_sensor_8(ctx, 0xfd, 0x00);
			write_cmos_sensor_8(ctx, 0x24, 0x10);
			write_cmos_sensor_8(ctx, 0x02, 0x00);
			write_cmos_sensor_8(ctx, 0x03, 0x06);
			write_cmos_sensor_8(ctx, 0x04, 0x1D);
			write_cmos_sensor_8(ctx, 0x01, 0x01);
			longexposue = 0;
		}

		if (ctx->s_ctx.reg_addr_exposure_lshift != PARAM_UNDEFINED) {
			ctx->l_shift = l_shift;
		}
		/* write framelength&shutter */
		/*if (set_auto_flicker(ctx, 0) || ctx->frame_length) {
			write_cmos_sensor_8(ctx, 0xfd,0x01);
			write_cmos_sensor_8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0],
				(ctx->frame_length >> 8) & 0xFF);
			write_cmos_sensor_8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[1],
				ctx->frame_length & 0xFF);
			write_cmos_sensor_8(ctx, 0x01,0x01);
		}*/
		if(longexposue == 0)
		{
			ctx->frame_length = ((ctx->frame_length + 3) >> 2) << 2;// need to set to  multi 4
			if (ctx->frame_length > static_ctx.frame_length_max) {
				ctx->frame_length = static_ctx.frame_length_max;
			}

			write_cmos_sensor_8(ctx, 0xfd, 0x01);
			write_cmos_sensor_8(ctx, 0x05, (((ctx->frame_length - vblank_convert)*2) & 0xFF00) >> 8);
			write_cmos_sensor_8(ctx, 0x06, ((ctx->frame_length - vblank_convert)*2) & 0xFF);
			write_cmos_sensor_8(ctx, 0x01, 0x01);
		}
		shutter = min(shutter, ctx->s_ctx.exposure_max);
		shutter = max(shutter, ctx->s_ctx.exposure_min);

		shutter = (shutter >> 1) << 1;
		write_cmos_sensor_8(ctx, 0xfd, 0x01);
		write_cmos_sensor_8(ctx, 0x02, (shutter*2 >> 16) & 0xFF);
		write_cmos_sensor_8(ctx, 0x03, (shutter*2 >> 8) & 0xFF);
		write_cmos_sensor_8(ctx, 0x04,  shutter*2  & 0xFF);
		write_cmos_sensor_8(ctx, 0x01, 0x01);
		/*write_cmos_sensor_8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[0],
			(ctx->exposure[0] *2 >> 8) & 0xFF);
		write_cmos_sensor_8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[1],
			ctx->exposure[0] *2 & 0xFF);
		write_cmos_sensor_8(ctx, 0x01,0x01);*/
		DRV_LOG(ctx, "normal exposure mode: lshift %u times, normal shutter=0x%x, frame_length=%d\n",
				l_shift,
				shutter,
				ctx->frame_length);

		ctx->current_ae_effective_frame = 2;
	}
	ctx->exposure[0] = shutter;
}

static int ov08d_set_shutter_frame_length_by(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u32 shutter = *feature_data;
	u32 frame_length = 0;
	u32 fine_integ_line = 0;

	DRV_LOG(ctx, "ov08d_shutter = 0x%x \n", shutter);
	DRV_LOG(ctx, "ov08d_frame_length = 0x%x \n", frame_length);
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
		ov08d_set_long_exposure(ctx);
	} else {
		shutter = min(shutter, ctx->s_ctx.exposure_max);
		/* write framelength&shutter */
		if (set_auto_flicker(ctx, 0) || ctx->frame_length) {
		write_cmos_sensor_8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0],
			(ctx->frame_length >> 16) & 0xFF);
		write_cmos_sensor_8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[1],
			(ctx->frame_length >> 8) & 0xFF);
		write_cmos_sensor_8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[2],
			ctx->frame_length & 0xFF);
		}
		write_cmos_sensor_8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[0],
			(ctx->exposure[0] >> 8) & 0xFF);
		write_cmos_sensor_8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[1],
			ctx->exposure[0] & 0xFF);
	}

	DRV_LOG(ctx, "exp[0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		ctx->exposure[0], frame_length, ctx->frame_length, ctx->autoflicker_en);
	return ERROR_NONE;
}

static kal_uint16 ov08d_gain2reg(struct subdrv_ctx *ctx, const kal_uint32 gain)
{
	kal_uint16 iReg = 0x0000;
	//platform 1xgain = 64, sensor driver 1*gain = 0x100
	iReg = gain*16/BASEGAIN;
	return iReg;		/* sensorGlobalGain */
}

static void ov08d_sensor_init(struct subdrv_ctx *ctx)
{
	LOG_INF("%s E\n", __func__);
	vblank_convert = 0;
	write_cmos_sensor_8(ctx, 0xfd, 0x01);
	LOG_INF("%s x\n", __func__);
}
static int ov08dopen(struct subdrv_ctx *ctx)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 1;
	kal_uint32 sensor_id = 0;
	u32 scenario_id = 0;
	LOG_INF("%s E\n", __func__);
	while (static_ctx.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = static_ctx.i2c_addr_table[i];
		do {
			write_cmos_sensor_8(ctx, 0xfd, 0x00);
			sensor_id = ((read_cmos_sensor_8(ctx, 0x01) << 8) | read_cmos_sensor_8(ctx, 0x02));
			LOG_INF(" sensor_id 0x%x,ctx->i2c_write_id 0x%x\n", sensor_id, ctx->i2c_write_id);
	        	if (sensor_id == static_ctx.sensor_id) {
				break;
	        	}
			retry--;
   	    	} while (retry > 0);

		if (sensor_id == static_ctx.sensor_id) {
			break;
		}
		i++;
		retry = 1;
	}
	if (static_ctx.sensor_id != sensor_id) {
		LOG_INF("Open sensor id: 0x%x fail\n", sensor_id);
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	ov08d_sensor_init(ctx);
	//write_sensor_PDC(ctx);
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	memset(ctx->ana_gain, 0, sizeof(ctx->gain));
	ctx->exposure[0] = ctx->s_ctx.exposure_def;
	ctx->ana_gain[0] = ctx->s_ctx.ana_gain_def;
	ctx->current_scenario_id = scenario_id;
	ctx->pclk = ctx->s_ctx.mode[scenario_id].pclk;
	ctx->line_length = ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length = ctx->s_ctx.mode[scenario_id].framelength;
	ctx->frame_length_rg = ctx->frame_length;
	ctx->current_fps = ctx->s_ctx.mode[scenario_id].max_framerate;
	ctx->readout_length = ctx->s_ctx.mode[scenario_id].readout_length;
	ctx->read_margin = ctx->s_ctx.mode[scenario_id].read_margin;
	ctx->min_frame_length = ctx->frame_length;
	ctx->autoflicker_en = FALSE;
	ctx->test_pattern = 0;
	ctx->ihdr_mode = 0;
	ctx->pdaf_mode = 0;
	ctx->hdr_mode = 0;
	ctx->extend_frame_length_en = 0;
	//ctx->is_seamless = 0;
	ctx->fast_mode_on = 0;
	ctx->sof_cnt = 0;
	ctx->ref_sof_cnt = 0;
	ctx->is_streaming = 0;

	return ERROR_NONE;
}
static int ov08dclose(struct subdrv_ctx *ctx)
{
	LOG_INF("%s E\n", __func__);
	//ov08d_streaming_control(ctx,KAL_FALSE);
	return ERROR_NONE;
}   /*  close  */
static kal_uint32 preview(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	//LOG_INF("%s E\n", __func__);
	LOG_INF("%s E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_PREVIEW;
	ctx->pclk = mode_struct[SENSOR_SCENARIO_ID_NORMAL_PREVIEW].pclk;
	ctx->line_length = mode_struct[SENSOR_SCENARIO_ID_NORMAL_PREVIEW].linelength;
	ctx->frame_length = mode_struct[SENSOR_SCENARIO_ID_NORMAL_PREVIEW].framelength;
	ctx->min_frame_length = mode_struct[SENSOR_SCENARIO_ID_NORMAL_PREVIEW].framelength;
	write_cmos_sensor_8(ctx, 0xfd, 0x00);
	write_cmos_sensor_8(ctx, 0x20, 0x0e);
	mdelay(3);
	ov08d_table_write_cmos_sensor(ctx, addr_data_pair_preview_mot_nice_ov08d,
			sizeof(addr_data_pair_preview_mot_nice_ov08d)/sizeof(kal_uint16));
	//LOG_INF("%s X\n", __func__);
	vblank_convert = 2504;
	return ERROR_NONE;
}
static kal_uint32 capture(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_CAPTURE;
	ctx->pclk = mode_struct[SENSOR_SCENARIO_ID_NORMAL_CAPTURE].pclk;
	ctx->line_length = mode_struct[SENSOR_SCENARIO_ID_NORMAL_CAPTURE].linelength;
	ctx->frame_length = mode_struct[SENSOR_SCENARIO_ID_NORMAL_CAPTURE].framelength;
	ctx->min_frame_length = mode_struct[SENSOR_SCENARIO_ID_NORMAL_CAPTURE].framelength;
	write_cmos_sensor_8(ctx, 0xfd, 0x00);
	write_cmos_sensor_8(ctx, 0x20, 0x0e);
	mdelay(3);
	ov08d_table_write_cmos_sensor(ctx, addr_data_pair_capture_mot_nice_ov08d,
			sizeof(addr_data_pair_capture_mot_nice_ov08d)/sizeof(kal_uint16));
	LOG_INF("%s X\n", __func__);
	vblank_convert = 2504;
	return ERROR_NONE;
} /* capture(ctx) */
static kal_uint32 normal_video(struct subdrv_ctx *ctx,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	//LOG_INF("%s E\n", __func__);
	LOG_INF("%s E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_VIDEO;
	ctx->pclk = mode_struct[SENSOR_SCENARIO_ID_NORMAL_VIDEO].pclk;
	ctx->line_length = mode_struct[SENSOR_SCENARIO_ID_NORMAL_VIDEO].linelength;
	ctx->frame_length = mode_struct[SENSOR_SCENARIO_ID_NORMAL_VIDEO].framelength;
	ctx->min_frame_length = mode_struct[SENSOR_SCENARIO_ID_NORMAL_VIDEO].framelength;
	write_cmos_sensor_8(ctx, 0xfd, 0x00);
	write_cmos_sensor_8(ctx, 0x20, 0x0e);
	mdelay(3);
	ov08d_table_write_cmos_sensor(ctx, addr_data_pair_video_mot_nice_ov08d,
			sizeof(addr_data_pair_video_mot_nice_ov08d)/sizeof(kal_uint16));
	//LOG_INF("%s X\n", __func__);
	vblank_convert = 2504;
	return ERROR_NONE;
}
static kal_uint32 hs_video(struct subdrv_ctx *ctx,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	//LOG_INF("%s E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	ctx->pclk = mode_struct[SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO].pclk;
	ctx->line_length = mode_struct[SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO].linelength;
	ctx->frame_length = mode_struct[SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO].framelength;
	ctx->min_frame_length = mode_struct[SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO].framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;
	write_cmos_sensor_8(ctx, 0xfd, 0x00);
	write_cmos_sensor_8(ctx, 0x20, 0x0e);
	mdelay(3);
	ov08d_table_write_cmos_sensor(ctx, addr_data_pair_hs_video_mot_nice_ov08d,
			sizeof(addr_data_pair_hs_video_mot_nice_ov08d)/sizeof(kal_uint16));
	//LOG_INF("%s X\n", __func__);
	vblank_convert = 2504;
	return ERROR_NONE;
}
static kal_uint32 slim_video(struct subdrv_ctx *ctx,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	//LOG_INF("%s E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	ctx->pclk = mode_struct[SENSOR_SCENARIO_ID_SLIM_VIDEO].pclk;
	ctx->line_length =mode_struct[SENSOR_SCENARIO_ID_SLIM_VIDEO].linelength;
	ctx->frame_length =mode_struct[SENSOR_SCENARIO_ID_SLIM_VIDEO].framelength;
	ctx->min_frame_length = mode_struct[SENSOR_SCENARIO_ID_SLIM_VIDEO].framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;
	write_cmos_sensor_8(ctx, 0xfd, 0x00);
	write_cmos_sensor_8(ctx, 0x20, 0x0e);
	mdelay(3);
	ov08d_table_write_cmos_sensor(ctx, addr_data_pair_slim_video_mot_nice_ov08d,
			sizeof(addr_data_pair_slim_video_mot_nice_ov08d)/sizeof(kal_uint16));
	//LOG_INF("%s X\n", __func__);
	vblank_convert = 2504;
	return ERROR_NONE;
}
#if 0
static int ov08dget_resolution(struct subdrv_ctx *ctx,
		MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	int i = 0;
	for (i = SENSOR_SCENARIO_ID_MIN; i < SENSOR_SCENARIO_ID_MAX; i++) {
		if (i < static_ctx.sensor_mode_num) {
			sensor_resolution->SensorWidth[i] = mode_struct[i].imgsensor_winsize_info.w2_tg_size;
			sensor_resolution->SensorHeight[i] = mode_struct[i].imgsensor_winsize_info.h2_tg_size;
		} else {
			sensor_resolution->SensorWidth[i] = 0;
			sensor_resolution->SensorHeight[i] = 0;
		}
	}
	return ERROR_NONE;
}   /*  get_resolution  */
static int ov08dget_info(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
		      MSDK_SENSOR_INFO_STRUCT *sensor_info,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = KAL_FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */
	sensor_info->SensroInterfaceType = static_ctx.sensor_interface_type;
	sensor_info->MIPIsensorType = static_ctx.mipi_sensor_type;
	//sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		static_ctx.sensor_output_dataformat;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_PREVIEW] =3;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_CAPTURE] =2;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_VIDEO] =2;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO] =2;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_SLIM_VIDEO] = 2;
	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = static_ctx.isp_driving_current;
/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame = 0;
	/* The frame of setting sensor gain */
	sensor_info->AESensorGainDelayFrame = 0;
	sensor_info->AEISPGainDelayFrame =2;
	sensor_info->IHDR_Support = 0;
	sensor_info->IHDR_LE_FirstLine = 0;
	sensor_info->SensorModeNum = static_ctx.sensor_mode_num;
	sensor_info->PDAF_Support = 0;
	//sensor_info->HDR_Support = 0; /*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR*/
	sensor_info->SensorMIPILaneNumber = static_ctx.mipi_lane_num;
	sensor_info->SensorClockFreq = static_ctx.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;   // 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;
	return ERROR_NONE;
}   /*  get_info  */
#endif
static int ov08dcontrol(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->current_scenario_id = scenario_id;
	LOG_INF("%s E,scenario_id = %d\n", __func__,scenario_id);
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		preview(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		capture(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		normal_video(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		hs_video(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		slim_video(ctx, image_window, sensor_config_data);
	break;
/* ITD: Modify Dualcam By Jesse 190924 End */
	default:
		//LOG_INF("Error ScenarioId setting");
		preview(ctx, image_window, sensor_config_data);
	return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}   /* control(ctx) */

static int motniceov08d_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		LOG_INF("mode(%u->%u)\n", ctx->test_pattern, mode);
	if (mode){
		write_cmos_sensor_8(ctx, 0xfd, 0x01);
		write_cmos_sensor_8(ctx, 0x21, 0x00);
		write_cmos_sensor_8(ctx, 0x22, 0x00);
		write_cmos_sensor_8(ctx, 0x01, 0x01);
	}
	else if (ctx->test_pattern){
		write_cmos_sensor_8(ctx, 0xfd, 0x01);
		write_cmos_sensor_8(ctx, 0x21, 0x02);
		write_cmos_sensor_8(ctx, 0x22, 0x00);
		write_cmos_sensor_8(ctx, 0x01, 0x01);
	}
	ctx->test_pattern = mode;

	return ERROR_NONE;
}

bool my_is_streaming = KAL_FALSE;
static int ov08d_streaming_on(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	unsigned int *feature_data = (unsigned int *) para;
	if (my_is_streaming== KAL_TRUE)
	{
		LOG_INF("%s E already on\n", __func__);
		//return ERROR_NONE;
	}
	mdelay(5);
	LOG_INF("%s E feature_data = %d\n", __func__,*feature_data);
	write_cmos_sensor_8(ctx, 0xfd, 0x02);
	LOG_INF("vsize = 0x%x\n", ((read_cmos_sensor_8(ctx, 0xa2) << 8) | read_cmos_sensor_8(ctx, 0xa3)));
	LOG_INF("hszie = 0x%x\n", ((read_cmos_sensor_8(ctx, 0xa6) << 8) | read_cmos_sensor_8(ctx, 0xa7)));
	write_cmos_sensor_8(ctx, 0xfd, 0x00);
	write_cmos_sensor_8(ctx, 0x21, 0x0e);
	write_cmos_sensor_8(ctx, 0x21, 0x00);

	write_cmos_sensor_8(ctx, 0xfd, 0x01);
	write_cmos_sensor_8(ctx, 0x01, 0x03);

	write_cmos_sensor_8(ctx, 0xfd, 0x00);
	write_cmos_sensor_8(ctx, 0x20, 0x0f);
	write_cmos_sensor_8(ctx, 0xe7, 0x03);
	write_cmos_sensor_8(ctx, 0xe7, 0x00);
	write_cmos_sensor_8(ctx, 0xc2, 0x30);

	write_cmos_sensor_8(ctx, 0xfd, 0x01);
	write_cmos_sensor_8(ctx, 0x15, 0x01);

	write_cmos_sensor_8(ctx, 0xfd, 0x00);
	write_cmos_sensor_8(ctx, 0xa0, 0x01);
	write_cmos_sensor_8(ctx, 0xfd, 0x01);
	//write_cmos_sensor_8(ctx, 0x12, 0x01);
	//LOG_INF("0xfd = 0x%x\n", read_cmos_sensor_8(ctx, 0xfd));
	ctx->is_streaming = KAL_TRUE;
	my_is_streaming = KAL_TRUE;
	mdelay(10);
	return ERROR_NONE;
}
static int ov08d_set_shutter_by(struct subdrv_ctx *ctx,  u8 *para, u32 *len)
{
	ov08d_set_shutter_frame_length_by(ctx, para, len);
	return  ERROR_NONE;
}
static int ov08d_set_gain_by(struct subdrv_ctx *ctx,  u8 *para, u32 *len)
{
	kal_uint16 reg_gain;
	kal_uint32 gain = *((kal_uint32 *) para);
	LOG_INF("gain_from_external = %d\n", gain);

	if (gain < static_ctx.ana_gain_min || gain > static_ctx.ana_gain_max) {
		pr_debug("Error gain setting");

		if (gain < static_ctx.ana_gain_min)
			gain = static_ctx.ana_gain_min;
		else if (gain > static_ctx.ana_gain_max)
			gain = static_ctx.ana_gain_max;
	}

	reg_gain = ov08d_gain2reg(ctx, gain);
	ctx->gain = reg_gain;
	write_cmos_sensor_8(ctx, 0xfd, 0x01);
	write_cmos_sensor_8(ctx, 0x24, (reg_gain & 0xFF));
	write_cmos_sensor_8(ctx, 0x01, 0x01);
	//DEBUG_LOG(ctx, "gain = %d , reg_gain = 0x%x\n", gain, reg_gain);
	return gain;
}
static int ov08d_set_frame_length_by(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	kal_uint16 frame_length =  *((kal_uint16 *) para);
	if (frame_length > 1)
		ctx->frame_length = frame_length;
	if (ctx->frame_length > 0x7FEA)
		ctx->frame_length = 0x7FEA;
	if (ctx->min_frame_length > ctx->frame_length)
		ctx->frame_length = ctx->min_frame_length;
	/* Extend frame length */
	write_cmos_sensor_8(ctx, 0xfd, 0x01);
	write_cmos_sensor_8(ctx, 0x05, (((ctx->frame_length - vblank_convert)*2) & 0xFF00) >> 8);
	write_cmos_sensor_8(ctx, 0x06, ((ctx->frame_length - vblank_convert)*2) & 0xFF);
	write_cmos_sensor_8(ctx, 0x01, 0x01);
	LOG_INF("Framelength: set=%d/input=%d/min=%d\n",
		ctx->frame_length, frame_length, ctx->min_frame_length);
	return  ERROR_NONE;
}

static int ov08d_set_max_framerate_by_scenario_by(struct subdrv_ctx *ctx,u8 *para, u32 *len)
{
	unsigned long long *feature_data = (unsigned long long *) para;
	enum MSDK_SCENARIO_ID_ENUM  scenario_id = (enum MSDK_SCENARIO_ID_ENUM)*(feature_data);
	MUINT32 framerate = (MUINT32) *(feature_data+1);
	kal_uint32 frameHeight = 0;
	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);
	if (framerate == 0)
		return ERROR_NONE;
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
	    frameHeight = mode_struct[SENSOR_SCENARIO_ID_NORMAL_PREVIEW].pclk / framerate * 10 /
			mode_struct[SENSOR_SCENARIO_ID_NORMAL_PREVIEW].linelength;
		ctx->dummy_line =
			(frameHeight > mode_struct[SENSOR_SCENARIO_ID_NORMAL_PREVIEW].framelength) ?
			(frameHeight - mode_struct[SENSOR_SCENARIO_ID_NORMAL_PREVIEW].framelength):0;
	    ctx->frame_length = mode_struct[SENSOR_SCENARIO_ID_NORMAL_PREVIEW].framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
		//LOG_INF("scenario_id = %d, vblank_convert = %d\n", scenario_id, ctx->vblank_convert);
			ov08d_set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
	    frameHeight = mode_struct[SENSOR_SCENARIO_ID_NORMAL_VIDEO].pclk / framerate * 10 /
				mode_struct[SENSOR_SCENARIO_ID_NORMAL_VIDEO].linelength;
		ctx->dummy_line = (frameHeight >
			mode_struct[SENSOR_SCENARIO_ID_NORMAL_VIDEO].framelength) ?
		(frameHeight - mode_struct[SENSOR_SCENARIO_ID_NORMAL_VIDEO].framelength):0;
	    ctx->frame_length = mode_struct[SENSOR_SCENARIO_ID_NORMAL_VIDEO].framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
		//LOG_INF("scenario_id = %d, vblank_convert = %d\n", scenario_id, ctx->vblank_convert);
			ov08d_set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
	    frameHeight = mode_struct[SENSOR_SCENARIO_ID_NORMAL_CAPTURE].pclk / framerate * 10 /
			mode_struct[SENSOR_SCENARIO_ID_NORMAL_CAPTURE].linelength;
		ctx->dummy_line =
			(frameHeight > mode_struct[SENSOR_SCENARIO_ID_NORMAL_CAPTURE].framelength) ?
			(frameHeight - mode_struct[SENSOR_SCENARIO_ID_NORMAL_CAPTURE].framelength):0;
	    ctx->frame_length = mode_struct[SENSOR_SCENARIO_ID_NORMAL_CAPTURE].framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
		//LOG_INF("scenario_id = %d, vblank_convert = %d\n", scenario_id, ctx->vblank_convert);
			ov08d_set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
	    frameHeight = mode_struct[SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO].pclk / framerate * 10 /
			mode_struct[SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO].linelength;
		ctx->dummy_line =
			(frameHeight > mode_struct[SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO].framelength) ?
			(frameHeight - mode_struct[SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO].framelength):0;
		ctx->frame_length = mode_struct[SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO].framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
		//LOG_INF("scenario_id = %d, vblank_convert = %d\n", scenario_id, ctx->vblank_convert);
			ov08d_set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
	    frameHeight = mode_struct[SENSOR_SCENARIO_ID_SLIM_VIDEO].pclk / framerate * 10 /
			mode_struct[SENSOR_SCENARIO_ID_SLIM_VIDEO].linelength;
		ctx->dummy_line = (frameHeight >
			mode_struct[SENSOR_SCENARIO_ID_SLIM_VIDEO].framelength) ?
			(frameHeight - mode_struct[SENSOR_SCENARIO_ID_SLIM_VIDEO].framelength):0;
	    ctx->frame_length = mode_struct[SENSOR_SCENARIO_ID_SLIM_VIDEO].framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
		//LOG_INF("scenario_id = %d, vblank_convert = %d\n", scenario_id, ctx->vblank_convert);
			ov08d_set_dummy(ctx);
	break;
	default:  //coding with  preview scenario by default
	    frameHeight = mode_struct[SENSOR_SCENARIO_ID_NORMAL_PREVIEW].pclk / framerate * 10 /
			mode_struct[SENSOR_SCENARIO_ID_NORMAL_PREVIEW].linelength;
		ctx->dummy_line = (frameHeight >
			mode_struct[SENSOR_SCENARIO_ID_NORMAL_PREVIEW].framelength) ?
			(frameHeight - mode_struct[SENSOR_SCENARIO_ID_NORMAL_PREVIEW].framelength):0;
	    ctx->frame_length = mode_struct[SENSOR_SCENARIO_ID_NORMAL_PREVIEW].framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			ov08d_set_dummy(ctx);
	break;
	}
	//DEBUG_LOG(ctx, "scenario_id = %d, framerate = %d done\n", scenario_id, framerate);
	return ERROR_NONE;
}

static int ov08d_streaming_off(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	if (my_is_streaming == KAL_FALSE)
	{
		LOG_INF("%s E already off\n", __func__);
		return ERROR_NONE;
	}
	LOG_INF("%s E\n", __func__);
	write_cmos_sensor_8(ctx, 0xfd, 0x00);
	write_cmos_sensor_8(ctx, 0xa0, 0x00);
	write_cmos_sensor_8(ctx, 0x20, 0x0b);
	write_cmos_sensor_8(ctx, 0xe7, 0x03);
	write_cmos_sensor_8(ctx, 0xe7, 0x00);
	write_cmos_sensor_8(ctx, 0xc2, 0x32);
	write_cmos_sensor_8(ctx, 0x21, 0x0f);

	write_cmos_sensor_8(ctx, 0xfd, 0x01);
	LOG_INF("0xfd = 0x%x\n", read_cmos_sensor_8(ctx, 0xfd));
	ctx->is_streaming = KAL_FALSE;
	my_is_streaming = KAL_FALSE;
	mdelay(10);
	return ERROR_NONE;
}

static struct subdrv_ops ops = {
	.get_id = motniceov08d_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = ov08dopen,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = ov08dcontrol,
	.feature_control = common_feature_control,
	.close = ov08dclose,
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_RST, 0, 1},
	//{HW_ID_DOVDD, 1800000, 0}, // pmic_ldo/gpio(1.8V ldo) for dovdd
	{HW_ID_AVDD, 2800000, 6}, // pmic_ldo for avdd
	{HW_ID_DVDD, 1200000, 11}, // pmic_ldo for dvdd
	{HW_ID_AFVDD, 2800000, 2}, // pmic_ldo for dvdd
	{HW_ID_MCLK, 24, 1},
	{HW_ID_RST, 1, 11},
	{HW_ID_MCLK_DRIVING_CURRENT, 2, 9},
};
const struct subdrv_entry mot_nice_ov08d_mipi_raw_entry = {
	.name = SENSOR_DRVNAME_MOT_NICE_OV08D_MIPI_RAW,
	.id = MOT_NICE_OV08D_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};
/* STRUCT */
static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	if (en) {
		set_i2c_buffer(ctx, 0x3208, 0x00);
	} else {
		set_i2c_buffer(ctx, 0x3208, 0x10);
		set_i2c_buffer(ctx, 0x3208, 0xA0);
	}
}
static u16 get_gain2reg(u32 gain)
{
	return gain*16/BASEGAIN;
}
static int motniceov08d_get_imgsensor_id(struct subdrv_ctx *ctx, UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	while (static_ctx.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = static_ctx.i2c_addr_table[i];
	    do {
		    write_cmos_sensor_8(ctx, 0xfd, 0x00);
	        *sensor_id = ((read_cmos_sensor_8(ctx, 0x01) << 8) | read_cmos_sensor_8(ctx, 0x02));
			LOG_INF(" sensor_id 0x%x,ctx->i2c_write_id 0x%x\n", *sensor_id,ctx->i2c_write_id);
	        if (*sensor_id == static_ctx.sensor_id) {
		        return ERROR_NONE;
	        }
		    retry--;
		} while (retry > 0);
		i++;
		retry = 1;
	}
	if (*sensor_id != static_ctx.sensor_id) {
		LOG_INF("%s: 0x%x fail\n", __func__, *sensor_id);
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
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
