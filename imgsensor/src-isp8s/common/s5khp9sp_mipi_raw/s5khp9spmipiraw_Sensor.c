// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5khp9spmipiraw_Sensor.c
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
#include "s5khp9spmipiraw_Sensor.h"
#include "adaptor-subdrv-ctrl.h"
#include "adaptor-subdrv.h"
#include "adaptor-ctrls.h"
#include "s5khp9sp_ana_gain_table.h"
#include <linux/of.h>
#include "adaptor-i2c.h"
#include <linux/regulator/consumer.h>

static int get_sensor_temperature(void *arg);
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int s5khp9sp_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5khp9sp_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int s5khp9sp_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
static int s5khp9sp_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt, u64 sof_ts);
static int s5khp9sp_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5khp9sp_control(struct subdrv_ctx *ctx,
			enum SENSOR_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data);

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, s5khp9sp_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, s5khp9sp_set_test_pattern_data},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, s5khp9sp_seamless_switch},
	{SENSOR_FEATURE_SET_AWB_GAIN, s5khp9sp_set_awb_gain},
};


static struct mtk_mbus_frame_desc_entry frame_desc_FCM_2[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 8160,
			.vsize = 6144,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 4080,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 1016,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_3[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 8160,
			.vsize = 6144,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 4080,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 1016,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_4[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 8160,
			.vsize = 6144,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 2040,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 504,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_5[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 4080,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 1016,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_6[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 1016,
			.vsize = 192,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 248,
			.vsize = 192,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_7[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 1016,
			.vsize = 192,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 248,
			.vsize = 192,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_8[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 1016,
			.vsize = 192,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 248,
			.vsize = 192,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_9[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 2040,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 504,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_10[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 2040,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 504,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_11[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 2040,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 504,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_12[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 4080,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 1016,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_13[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 2040,
			.vsize = 1536,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 2040,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 504,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_14[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 2560,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 4080,
			.vsize = 640,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 1016,
			.vsize = 640,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_15[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 2560,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 2560,
			.user_data_desc = VC_STAGGER_ME,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x2b, /* 0x30, */
			.hsize = 4080,
			.vsize = 640,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x3,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 1016,
			.vsize = 640,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_16[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 2560,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 2040,
			.vsize = 320,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 504,
			.vsize = 320,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_17[] = {
	{
		.bus.csi2 = {
		.channel = 0x0,
		.data_type = 0x2b,
		.hsize = 4080,
		.vsize = 2560,
		.user_data_desc = VC_STAGGER_NE,
		.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
	.bus.csi2 = {
		.channel = 0x1,
		.data_type = 0x30, /* 0x2b, */
		.hsize = 4080,
		.vsize = 640,
		.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
	.bus.csi2 = {
		.channel = 0x2,
		.data_type = 0x30, /* 0x2b, */
		.hsize = 1016,
		.vsize = 640,
		.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
		.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_18[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 2560,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 1016,
			.vsize = 160,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 248,
			.vsize = 160,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_19[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 2560,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 1016,
			.vsize = 160,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 248,
			.vsize = 160,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_FCM_23[] = {
	{
		.bus.csi2 = {
			.channel = 0x0,
			.data_type = 0x2b,
			.hsize = 4080,
			.vsize = 2560,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x1,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 4080,
			.vsize = 640,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0x2,
			.data_type = 0x30, /* 0x2b, */
			.hsize = 1016,
			.vsize = 640,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_2,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
/* prev -> FCM_12 */
/* 17_Volcano1_HP3_8Fdsum_2H1V_12.5Mp_30FPS_4080x3072_direct.sset */
	{
		.frame_desc = frame_desc_FCM_12,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_12),
		.mode_setting_table = FCM_12_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_12_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = FCM_12_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_12_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2002000000,
		.linelength = 7920,
		.framelength = 8374,
		.max_framerate = 300,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-4080*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4080*4,
			.h0_size = 3072*4,
			.scale_w = 4080*2,
			.scale_h = 3072*2,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 360,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
	},
/* cap -> FCM_12 */
/* 17_Volcano1_HP3_8Fdsum_2H1V_12.5Mp_30FPS_4080x3072_direct.sset */
	{
		.frame_desc = frame_desc_FCM_12,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_12),
		.mode_setting_table = FCM_12_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_12_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = FCM_12_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_12_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2002000000,
		.linelength = 7920,
		.framelength = 8374,
		.max_framerate = 300,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-4080*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4080*4,
			.h0_size = 3072*4,
			.scale_w = 4080*2,
			.scale_h = 3072*2,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 360,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
	},
/* video -> FCM_14 */
/* 17_Volcano4_HP3_8Fdsum_2H1V_12.5Mp_60FPS_4080x2616_direct.sset */
	{
		.frame_desc = frame_desc_FCM_14,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_14),
		.mode_setting_table = FCM_14_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_14_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = FCM_14_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_14_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2002000000,
		.linelength = 7920,
		.framelength = 8374,
		.max_framerate = 300,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-4080*4)/2,
			.y0_offset = (12288-2560*4)/2,
			.w0_size = 4080*4,
			.h0_size = 2560*4,
			.scale_w = 4080*2,
			.scale_h = 2560*2,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 2560,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 2560,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 360,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
	},
/* hs_video -> FCM_23 */
/* 17_Volcano4_HP3_8Fdsum_2H1V_12.5Mp_60FPS_4080x2616_direct.sset */
	{
		.frame_desc = frame_desc_FCM_23,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_23),
		.mode_setting_table = FCM_23_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_23_setting),
		.seamless_switch_group = 3,
		.seamless_switch_mode_setting_table = FCM_23_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_23_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2002000000,
		.linelength = 7920,
		.framelength = 4198,
		.max_framerate = 600,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-4080*4)/2,
			.y0_offset = (12288-2560*4)/2,
			.w0_size = 4080*4,
			.h0_size = 2560*4,
			.scale_w = 4080*2,
			.scale_h = 2560*2,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 2560,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 2560,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 360,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
	},
/* no FPM filled slim_video -> FCM_2 */
/* 06_Volcano_HP3_Fdsum_50Mp_24FPS_8160x6144_direct.sset */
	{
		.frame_desc = frame_desc_FCM_2,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_2),
		.mode_setting_table = FCM_2_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_2_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = FCM_2_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_2_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1759333333,
		.linelength = 11344,
		.framelength = 6452,
		.max_framerate = 240,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { /* HW RMSC */
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-8160*2)/2,
			.y0_offset = (12288-6144*2)/2,
			.w0_size = 8160*2,
			.h0_size = 6144*2,
			.scale_w = 8160*2,
			.scale_h = 6144*2,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 8160,
			.h1_size = 6144,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 8160,
			.h2_tg_size = 6144,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 316,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
	},
/* cus1 -> FCM_3 */
/* 07_Volcano_HP3_Fdsum_50Mp_24FPS_8160x6144_direct.sset */
	{
		.frame_desc = frame_desc_FCM_3,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_3),
		.mode_setting_table = FCM_3_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_3_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = FCM_3_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_3_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1759333333,
		.linelength = 11344,
		.framelength = 6452,
		.max_framerate = 240,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { /* 4cell */
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-8160*2)/2,
			.y0_offset = (12288-6144*2)/2,
			.w0_size = 8160*2,
			.h0_size = 6144*2,
			.scale_w = 8160*2,
			.scale_h = 6144*2,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 8160,
			.h1_size = 6144,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 8160,
			.h2_tg_size = 6144,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 316,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_2X2, /* tetra */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gr,
		.awb_enabled = 1,
	},

/* cus2 -> FCM_4 */
/* 02_Volcano_HP3_Full_50Mp_15.5FPS_8160x6144_direct.sset */
	{
		.frame_desc = frame_desc_FCM_4,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_4),
		.mode_setting_table = FCM_4_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_4_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = FCM_4_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_4_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2002000000,
		.linelength = 20320,
		.framelength = 6351,
		.max_framerate = 155,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { /* 16cell */
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-8160)/2,
			.y0_offset = (12288-6144)/2,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 8160,
			.scale_h = 6144,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 8160,
			.h1_size = 6144,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 8160,
			.h2_tg_size = 6144,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 360,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_4X4, /* hexdeca */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gr,
		.awb_enabled = 1,
	},

/* cus3 -> FCM_5 */
/* 17_Volcano1_HP3_8Fdsum_2H1V_12.5Mp_30FPS_4080x3072_direct.sset */
	{
		.frame_desc = frame_desc_FCM_5,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_5),
		.mode_setting_table = FCM_5_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_5_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = FCM_5_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_5_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2002000000,
		.linelength = 7920,
		.framelength = 4202,
		.max_framerate = 600,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-4080*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4080*4,
			.h0_size = 3072*4,
			.scale_w = 4080*2,
			.scale_h = 3072*2,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 360,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
	},


/* cus4 -> FCM_6 */
/* 02_Volcano2_HP3_Full_12.5Mp_30FPS_4080x3072_HCGonly.sset */
	{
		.frame_desc = frame_desc_FCM_6,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_6),
		.mode_setting_table = FCM_6_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_6_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = FCM_6_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_6_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2002000000,
		.linelength = 20320,
		.framelength = 3279,
		.max_framerate = 300,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-4080*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4080*4,
			.h0_size = 3072*4,
			.scale_w = 4080*2,
			.scale_h = 3072*2,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 360,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
	},

/* cus5 -> FCM_7 */
/* 02_Volcano1_HP3_Full_12.5Mp_30FPS_4080x3072_direct.sset */
	{
		.frame_desc = frame_desc_FCM_7,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_7),
		.mode_setting_table = FCM_7_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_7_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = FCM_7_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_7_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2002000000,
		.linelength = 20320,
		.framelength = 3279,
		.max_framerate = 300,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { /* HW RM */
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-4080)/2,
			.y0_offset = (12288-3072)/2,
			.w0_size = 4080,
			.h0_size = 3072,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 360,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
	},

/* cus6 -> FCM_8 */
/* 02_Volcano2_HP3_Full_12.5Mp_30FPS_4080x3072_HCGonly.sset */
	{
		.frame_desc = frame_desc_FCM_8,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_8),
		.mode_setting_table = FCM_8_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_8_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = FCM_8_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_8_setting),
		.hdr_mode = HDR_NONE, /* HDR_RAW_LBMF */
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 2002000000,
		.linelength = 20320,
		.framelength = 3272,
		.max_framerate = 300,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { /* 16cell */
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-4080)/2,
			.y0_offset = (12288-3072)/2,
			.w0_size = 4080,
			.h0_size = 3072,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 360,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_4X4, /* hexdeca */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gr,
		.awb_enabled = 1,
		.exposure_order_in_lbmf = IMGSENSOR_LBMF_EXPOSURE_SE_FIRST,
		.mode_type_in_lbmf = IMGSENSOR_LBMF_MODE_MANUAL,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].max = 0xFFFF,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].max = 0xFFFF,
	},

/* cus7 -> FCM_9 */
/* 11_Volcano_HP3_Fdsum_12.5Mp_30FPS_4080x3072_direct.sset */
	{
		.frame_desc = frame_desc_FCM_9,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_9),
		.mode_setting_table = FCM_9_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_9_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = FCM_9_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_9_setting),
		.hdr_mode = HDR_NONE, /* HDR_RAW_LBMF */
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 1759333333,
		.linelength = 11562,
		.framelength = 5072, /* origin: 2796, modify to 30 FPS */
		.max_framerate = 300, /* origin: 544, modify to 30 FPS */
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { /* 4cell */
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-4080*2)/2,
			.y0_offset = (12288-3072*2)/2,
			.w0_size = 4080*2,
			.h0_size = 3072*2,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 316,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_2X2, /* tetra */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gr,
		.awb_enabled = 1,
		.exposure_order_in_lbmf = IMGSENSOR_LBMF_EXPOSURE_SE_FIRST,
		.mode_type_in_lbmf = IMGSENSOR_LBMF_MODE_MANUAL,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].max = 0xFFFF,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].max = 0xFFFF,
	},

/* cus8 -> FCM_10 */
/* 11_Volcano_HP3_Fdsum_12.5Mp_30FPS_4080x3072_direct.sset */
	{
		.frame_desc = frame_desc_FCM_10,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_10),
		.mode_setting_table = FCM_10_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_10_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = FCM_10_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_10_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1759333333,
		.linelength = 11680,
		.framelength = 5006,
		.max_framerate = 300,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { /* HW RM */
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-4080*2)/2,
			.y0_offset = (12288-3072*2)/2,
			.w0_size = 4080*2,
			.h0_size = 3072*2,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 316,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
	},

/* cus9 -> FCM_11 */
/* 11_Volcano_HP3_Fdsum_12.5Mp_30FPS_4080x3072_direct.sset */
	{
		.frame_desc = frame_desc_FCM_11,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_11),
		.mode_setting_table = FCM_11_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_11_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = FCM_11_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_11_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1759333333,
		.linelength = 11680,
		.framelength = 5006,
		.max_framerate = 300,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { /* 4cell */
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-4080*2)/2,
			.y0_offset = (12288-3072*2)/2,
			.w0_size = 4080*2,
			.h0_size = 3072*2,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 316,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_2X2, /* tetra */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gr,
		.awb_enabled = 1,
	},

/* cus10 -> FCM_13 */
/* 63_Volcano2_HP3_8Fdsum_4H2V_FHD_120FPS_1920x1080_direct.sset */
	{
		.frame_desc = frame_desc_FCM_13,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_13),
		.mode_setting_table = FCM_13_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_13_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = FCM_13_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_13_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2002000000,
		.linelength = 12320,
		.framelength = 5402,
		.max_framerate = 300,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-2040*4)/2,
			.y0_offset = (12288-1536*4)/2,
			.w0_size = 2040*4,
			.h0_size = 1536*4,
			.scale_w = 2040*2,
			.scale_h = 1536*2,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2040,
			.h1_size = 1536,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2040,
			.h2_tg_size = 1536,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 360,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
	},


/* cus11 -> FCM_15 (stagger 2-exp) */
/* 31_Volcano1_HP3_8Fdsum_2H1V_stHDR_2exp_12.5Mp_30FPS_3192x2390_direct.sset */
	{
		.frame_desc = frame_desc_FCM_15,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_15),
		.mode_setting_table = FCM_15_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_15_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = FCM_15_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_15_setting),
		.hdr_mode = HDR_RAW_STAGGER,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 2002000000,
		.linelength = 6880,
		.framelength = 9648,
		.max_framerate = 300,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 3072,
		.read_margin = 64,
		.min_vblanking_line = 0, /* TBD */
		.imgsensor_winsize_info = {
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-4080*4)/2,
			.y0_offset = (12288-2560*4)/2,
			.w0_size = 4080*4,
			.h0_size = 2560*4,
			.scale_w = 4080*2,
			.scale_h = 2560*2,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 2560,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 2560,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 360,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
	},

/* cus12 -> FCM_16 */
/* 11_Volcano5_HP3_Fdsum_12.5Mp_54.4FPS_4080x2616_direct.sset */
	{
		.frame_desc = frame_desc_FCM_16,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_16),
		.mode_setting_table = FCM_16_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_16_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = FCM_16_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_16_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1759333333,
		.linelength = 11562,
		.framelength = 2796,
		.max_framerate = 544,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { /* 4cell */
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-4080*2)/2,
			.y0_offset = (12288-2560*2)/2,
			.w0_size = 4080*2,
			.h0_size = 2560*2,
			.scale_w = 4080*2,
			.scale_h = 2560*2,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 2560,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 2560,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 316,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_2X2, /* tetra */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gr,
		.awb_enabled = 1,
	},


/* cus13 -> FCM_17 */
/* 17_Volcano4_HP3_8Fdsum_2H1V_12.5Mp_60FPS_4080x2616_direct.sset */
	{
		.frame_desc = frame_desc_FCM_17,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_17),
		.mode_setting_table = FCM_17_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_17_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = FCM_17_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_17_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2002000000,
		.linelength = 15840,
		.framelength = 5402,
		.max_framerate = 234,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-4080*4)/2,
			.y0_offset = (12288-2560*4)/2,
			.w0_size = 4080*4,
			.h0_size = 2560*4,
			.scale_w = 4080*2,
			.scale_h = 2560*2,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 2560,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 2560,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 360,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
	},

/* cus14 -> FCM_18 */
/* 02_Volcano1_HP3_Full_12.5Mp_30FPS_4080x3072_direct.sset */
	{
		.frame_desc = frame_desc_FCM_18,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_18),
		.mode_setting_table = FCM_18_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_18_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = FCM_18_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_18_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2002000000,
		.linelength = 20320,
		.framelength = 3279,
		.max_framerate = 300,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { /* 16cell */
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-4080)/2,
			.y0_offset = (12288-2560)/2,
			.w0_size = 4080,
			.h0_size = 2560,
			.scale_w = 4080,
			.scale_h = 2560,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 2560,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 2560,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 360,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_4X4, /* hexdeca */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gr,
		.awb_enabled = 1,
	},

/* cus15 -> FCM_19 */
/* 02_Volcano2_HP3_Full_12.5Mp_30FPS_4080x3072_HCGonly.sset */
	{
		.frame_desc = frame_desc_FCM_19,
		.num_entries = ARRAY_SIZE(frame_desc_FCM_19),
		.mode_setting_table = FCM_19_setting,
		.mode_setting_len = ARRAY_SIZE(FCM_19_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = FCM_19_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(FCM_19_setting),
		.hdr_mode = HDR_NONE, /* HDR_RAW_LBMF */
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 2002000000,
		.linelength = 20320,
		.framelength = 3279,
		.max_framerate = 300,
		.mipi_pixel_rate = 1842285714,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { /* 16cell */
			.full_w = 16320,
			.full_h = 12288,
			.x0_offset = (16320-4080)/2,
			.y0_offset = (12288-2560)/2,
			.w0_size = 4080,
			.h0_size = 2560,
			.scale_w = 4080,
			.scale_h = 2560,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 2560,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 2560,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 360,
		.delay_frame = 2,
		.min_exposure_line = 16,
		.ana_gain_max = BASEGAIN * 128,
		.coarse_integ_step = 8,
		.framelength_step = 8,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_4X4, /* hexdeca */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gr,
		.awb_enabled = 1,
		.exposure_order_in_lbmf = IMGSENSOR_LBMF_EXPOSURE_SE_FIRST,
		.mode_type_in_lbmf = IMGSENSOR_LBMF_MODE_MANUAL,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].max = 0xFFFF,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].max = 0xFFFF,
	},

};




static struct subdrv_static_ctx static_ctx = {
	.sensor_id = S5KHP3SP_SENSOR_ID,
	.reg_addr_sensor_id = { 0x0000, 0x0001 },
	.i2c_addr_table = {0x20, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info = PARAM_UNDEFINED,
	.eeprom_num = PARAM_UNDEFINED,
	.resolution = { 16320, 12288 },
	.mirror = IMAGE_NORMAL,

	.mclk = 26,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 128,
	.ana_gain_type = 2,
	.ana_gain_step = 1,
	.ana_gain_table = s5khp9sp_ana_gain_table,
	.ana_gain_table_size = sizeof(s5khp9sp_ana_gain_table),
	.tuning_iso_base = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 6,
	.exposure_max = (0xFFFF - 74) << 7, /* cit_lshift_max = 7 */
	.exposure_step = 1,
	.exposure_margin = 74,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = BASE_DGAIN * 16,
	.dig_gain_step = 4,
	.frame_length_max = 0xFFFF << 7, /* fll_lshift_max = 7 */
	/* .frame_length_max_without_lshift = 0xFFFF, */
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 3000000,

	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
	.hdr_type = HDR_SUPPORT_STAGGER_DOL,
	.seamless_switch_support = TRUE,
	.temperature_support = TRUE,
	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {
		{ 0x0202, 0x0203 },
		{ 0x022C, 0x022D },
		{ 0x0226, 0x0227 },
	},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x0704,
	.reg_addr_frame_length_lshift = 0x0702,
	/* .fll_lshift_max = 7, */
	/* .cit_lshift_max = 7, */

	.reg_addr_ana_gain = {
		{ 0x0204, 0x0205 },
		{ 0x0208, 0x0209 },
		{ 0x0206, 0x0207 },
	},

	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = 0x0020,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = 0x0005,

	.reg_addr_frame_length_in_lut = {
			{0x0E16, 0x0E17},  /* LUT_A_FRM_LENGTH_LINES */
			{0x0E24, 0x0E25},  /* LUT_B_FRM_LENGTH_LINES */
	},

	.reg_addr_exposure_in_lut = {
			{0x0E10, 0x0E11}, /* LUT_A_COARSE_INTEG_TIME */
			{0x0E1E, 0x0E1F}, /* LUT_B_COARSE_INTEG_TIME */
	},

	.reg_addr_ana_gain_in_lut = {
			{0x0E12, 0x0E13}, /* LUT_A_ANA_GAIN_GLOBAL */
			{0x0E20, 0x0E21}, /* LUT_B_ANA_GAIN_GLOBAL */
	},


	.init_setting_table = s5khp9sp_init_setting,
	.init_setting_len = ARRAY_SIZE(s5khp9sp_init_setting),
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
	.control = s5khp9sp_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
	.vsync_notify = vsync_notify,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_AVDD, {2200000, 2200000}, 1000},
	{HW_ID_DVDD, {900000, 900000}, 1000},
	{HW_ID_DOVDD, {1800000, 1800000}, 1000},
	{HW_ID_AFVDD, {2800000, 2800000}, 1000},
	{HW_ID_OISVDD, {3250000, 3250000}, 5000},
	{HW_ID_RST, {1}, 5000},
	{HW_ID_MCLK_DRIVING_CURRENT, {6}, 1000},
	{HW_ID_MCLK, {26}, 10000},
};


const struct subdrv_entry s5khp9sp_mipi_raw_entry = {
	.name = "s5khp9sp_mipi_raw",
	.id = S5KHP9SP_SENSOR_ID, /* 0x1b73 */
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */
static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u16 temperature = 0;
	int temperature_convert = 0;

	temperature = subdrv_i2c_rd_u16(ctx, ctx->s_ctx.reg_addr_temp_read);

	temperature_convert = (temperature>>8)&0xFF;

	temperature_convert = (temperature_convert > 100) ? 100: temperature_convert;

	DRV_LOG(ctx, "temperature: %d degrees\n", temperature_convert);
	return temperature_convert;
}

void seamless_switch_update_hw_re_init_time(struct subdrv_ctx *ctx,
	enum SENSOR_SCENARIO_ID_ENUM pre_seamless_scenario_id,
	enum SENSOR_SCENARIO_ID_ENUM scenario_id)
{
	if (ctx->s_ctx.mode[pre_seamless_scenario_id].pclk != ctx->s_ctx.mode[scenario_id].pclk)
		ctx->s_ctx.seamless_switch_hw_re_init_time_ns = 16000000;
	else
		ctx->s_ctx.seamless_switch_hw_re_init_time_ns = 4000000;

	DRV_LOG_MUST(ctx,
			"[%s] scen:%u(pclk:%llu) => scen:%u(pclk:%llu), hw_re_init_time_ns:%u\n",
			__func__,
			pre_seamless_scenario_id,
			ctx->s_ctx.mode[pre_seamless_scenario_id].pclk,
			scenario_id,
			ctx->s_ctx.mode[scenario_id].pclk,
			ctx->s_ctx.seamless_switch_hw_re_init_time_ns);
}


static int s5khp9sp_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	struct v4l2_subdev *_sd = NULL;
	struct adaptor_ctx *_adaptor_ctx = NULL;
	u64 *feature_data = (u64 *)para;
	u32 frame_length_in_lut[IMGSENSOR_STAGGER_EXPOSURE_CNT] = {0};
	u32 exp_cnt = 0;
	enum SENSOR_SCENARIO_ID_ENUM pre_seamless_scenario_id = ctx->current_scenario_id;

	if (ctx->i2c_client)
		_sd = i2c_get_clientdata(ctx->i2c_client);
	if (_sd)
		_adaptor_ctx = to_ctx(_sd);

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
	ctx->is_seamless = TRUE;

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);

	if (ctx->s_ctx.reg_addr_fast_mode_in_lbmf &&
		(ctx->s_ctx.mode[scenario_id].hdr_mode == HDR_RAW_LBMF ||
		ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_LBMF))
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode_in_lbmf, 0x4);

	update_mode_info(ctx, scenario_id);
	i2c_table_write(ctx,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);

	if (ae_ctrl) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_STAGGER:
			set_multi_shutter_frame_length(ctx, (u64 *)&ae_ctrl->exposure, exp_cnt, 0);
			set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		case HDR_RAW_LBMF:
			set_multi_shutter_frame_length_in_lut(ctx,
				(u64 *)&ae_ctrl->exposure, exp_cnt, 0, frame_length_in_lut);
			set_multi_gain_in_lut(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		case HDR_RAW_DCG_RAW:
			set_multi_shutter_frame_length(ctx, (u64 *)&ae_ctrl->exposure, 1, 0);
			if (ctx->s_ctx.mode[scenario_id].dcg_info.dcg_gain_mode
				== IMGSENSOR_DCG_DIRECT_MODE)
				set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			else
				set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		default:
			set_multi_shutter_frame_length(ctx, (u64 *)&ae_ctrl->exposure, 1, 0);
			set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		}
	}
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	seamless_switch_update_hw_re_init_time(ctx, pre_seamless_scenario_id, scenario_id);
	DRV_LOG(ctx, "X: set seamless switch done\n");
	return ERROR_NONE;
}

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

static int s5khp9sp_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int s5khp9sp_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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


static int s5khp9sp_sensor_init(struct subdrv_ctx *ctx)
{
	int rc = 0;

	DRV_LOG(ctx, "E\n");
	subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
	subdrv_i2c_wr_u16(ctx, 0x0000, 0x0001); /* version */
	subdrv_i2c_wr_u16(ctx, 0x0000, 0x1B73); /* model ID */
	subdrv_i2c_wr_u16(ctx, 0x6012, 0x0001); /* reset */
	subdrv_i2c_wr_u16(ctx, 0x7002, 0x0008); /* boot with PLL*/
	subdrv_i2c_wr_u16(ctx, 0x6014, 0x0001); /* SW load complete*/
	mdelay(20);
	i2c_table_write(ctx, s5khp9sp_init_setting, sizeof(s5khp9sp_init_setting)/sizeof(u16));
	DRV_LOG(ctx, "X\n");

	return rc;
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (common_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	s5khp9sp_sensor_init(ctx);

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

static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt, u64 sof_ts)
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

static int s5khp9sp_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct SET_SENSOR_AWB_GAIN *awb_gain = (struct SET_SENSOR_AWB_GAIN *)para;

	adaptor_i2c_wr_u16(ctx->i2c_client, ctx->i2c_write_id >> 1,
		0x0D82, awb_gain->ABS_GAIN_R * 2); /* red 1024(1x) */
	adaptor_i2c_wr_u16(ctx->i2c_client, ctx->i2c_write_id >> 1,
		0x0D86, awb_gain->ABS_GAIN_B * 2); /* blue */

	DRV_LOG(ctx, "[test] ABS_GAIN_GR(%d) ABS_GAIN_R(%d) ABS_GAIN_B(%d) ABS_GAIN_GB(%d)",
			awb_gain->ABS_GAIN_GR,
			awb_gain->ABS_GAIN_R,
			awb_gain->ABS_GAIN_B,
			awb_gain->ABS_GAIN_GB);
	DRV_LOG(ctx, "[test] 0x0D82(red) = (0x%x)", subdrv_i2c_rd_u16(ctx, 0x0D82));
	DRV_LOG(ctx, "[test] 0x0D84(green) = (0x%x)", subdrv_i2c_rd_u16(ctx, 0x0D84));
	DRV_LOG(ctx, "[test] 0x0D86(blue) = (0x%x)", subdrv_i2c_rd_u16(ctx, 0x0D86));
	return 0;
}

static int s5khp9sp_control(struct subdrv_ctx *ctx,
			enum SENSOR_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	int ret = ERROR_NONE;
	u16 idx = 0;
	u8 support = FALSE;
	u8 *pbuf = NULL;
	u16 size = 0;
	u16 addr = 0;
	u64 time_boot_begin = 0;
	u64 ixc_time = 0;
	u32 fast_mode_in_lbmf = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;
	struct adaptor_ctx *_adaptor_ctx = NULL;
	struct v4l2_subdev *sd = NULL;

	u16 *FCM_basic_setting = NULL;
	u32 FCM_basic_setting_len = 0;

	if (ctx->i2c_client)
		sd = i2c_get_clientdata(ctx->i2c_client);
	if (ctx->ixc_client.protocol)
		sd = adaptor_ixc_get_clientdata(&ctx->ixc_client);
	if (sd)
		_adaptor_ctx = to_ctx(sd);
	if (!_adaptor_ctx) {
		DRV_LOGE(ctx, "null _adaptor_ctx\n");
		return -ENODEV;
	}

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOGE(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
		ret = ERROR_INVALID_SCENARIO_ID;
	}
	if (ctx->s_ctx.chk_s_off_sta)
		check_stream_off(ctx);
	update_mode_info(ctx, scenario_id);

	if (ctx->s_ctx.mode[scenario_id].mode_setting_table != NULL) {
		/* FCM_basic_setting */
		switch (ctx->s_ctx.mode[scenario_id].seamless_switch_group) {
		case 1:
			FCM_basic_setting = S1_FCM_basic_setting;
			FCM_basic_setting_len = ARRAY_SIZE(S1_FCM_basic_setting);
			break;
		case 2:
			FCM_basic_setting = S2_FCM_basic_setting;
			FCM_basic_setting_len = ARRAY_SIZE(S2_FCM_basic_setting);
			break;
		case 3:
			FCM_basic_setting = S3_FCM_basic_setting;
			FCM_basic_setting_len = ARRAY_SIZE(S3_FCM_basic_setting);
			break;
		default:
			break;
		}
		DRV_LOG(ctx, "E: sid:%u size:%u\n", scenario_id,
			ctx->s_ctx.mode[scenario_id].mode_setting_len);
		if ((ctx->power_on_profile_en != NULL) &&
			(*ctx->power_on_profile_en))
			time_boot_begin = ktime_get_boottime_ns();

		/* initail setting */
		if (ctx->s_ctx.aov_sensor_support) {
			if (ctx->s_ctx.mode[scenario_id].aov_mode &&
				ctx->s_ctx.s_pwr_seq_reset_view_to_sensing != NULL)
				ctx->s_ctx.s_pwr_seq_reset_view_to_sensing((void *) ctx);

			if (!ctx->s_ctx.init_in_open)
				sensor_init(ctx);
		}
		switch (ctx->sensor_mode_ops) {
		case AOV_MODE_CTRL_OPS_SENSING_CTRL:
		default:
			if (FCM_basic_setting)
				ixc_table_write(ctx, FCM_basic_setting, FCM_basic_setting_len);

			ixc_time = ixc_table_write(ctx, ctx->s_ctx.mode[scenario_id].mode_setting_table,
				ctx->s_ctx.mode[scenario_id].mode_setting_len);
			break;
		}

		if ((ctx->power_on_profile_en != NULL) &&
			(*ctx->power_on_profile_en)) {
			ctx->sensor_pw_on_profile.i2c_cfg_period =
					ktime_get_boottime_ns() - time_boot_begin;

			ctx->sensor_pw_on_profile.i2c_cfg_table_len =
					ctx->s_ctx.mode[scenario_id].mode_setting_len;
		}
		DRV_LOG_MUST(ctx, "X: sid:%u size:%u, ixc_time(us): %lld\n", scenario_id,
			ctx->s_ctx.mode[scenario_id].mode_setting_len,
			ixc_time);
	} else {
		DRV_LOGE(ctx, "please implement mode setting(sid:%u)!\n", scenario_id);
	}

	if (check_is_no_crop(ctx, scenario_id) && probe_eeprom(ctx)) {
		idx = ctx->eeprom_index;
		support = info[idx].xtalk_support;
		pbuf = info[idx].preload_xtalk_table;
		size = info[idx].xtalk_size;
		addr = info[idx].sensor_reg_addr_xtalk;
		if (support) {
			if (pbuf != NULL && addr > 0 && size > 0) {
				subdrv_ixc_wr_seq_p8(ctx, addr, pbuf, size);
				DRV_LOG(ctx, "set XTALK calibration data done.");
			}
		}
	}

	if (ctx->s_ctx.aov_sensor_support &&
		ctx->s_ctx.s_data_rate_global_timing_phy_ctrl != NULL)
		ctx->s_ctx.s_data_rate_global_timing_phy_ctrl((void *) ctx);

	set_mirror_flip(ctx, ctx->s_ctx.mirror);

	if (ctx->s_ctx.reg_addr_fast_mode_in_lbmf &&
		ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_LBMF) {
		/* enable bit[2] on lbmf mode */
		fast_mode_in_lbmf =
			subdrv_ixc_rd_u8(ctx, ctx->s_ctx.reg_addr_fast_mode_in_lbmf) | 0x4;
		subdrv_ixc_wr_u8(ctx,
			ctx->s_ctx.reg_addr_fast_mode_in_lbmf,
			fast_mode_in_lbmf);
	}

	return ret;
}
