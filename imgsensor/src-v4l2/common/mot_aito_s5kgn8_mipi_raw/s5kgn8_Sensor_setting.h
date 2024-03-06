/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5kgn8_ensor_setting.h
 *
 * Project:
 * --------
 * Description:
 * ------------
 *	 CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _S5KGN8_SENSOR_SETTING_H
#define _S5KGN8_SENSOR_SETTING_H

#include "kd_camera_typedef.h"



static u16 uTnpArrayInit_1[] = {
#include "setting/s5kgn8_initsettting_1.h"
};

static u16 uTnpArrayInit_2[] = {
#include "setting/s5kgn8_initsettting_2.h"
};

static u16 uTnpArrayInit_3[] = {
#include "setting/s5kgn8_initsettting_3.h"
#include "setting/calibration_xtc.h"
#include "setting/init_end.h"
};

static u16 addr_data_pair_preview[] = {
#include "setting/s5kgn8_4096_3072_30fps.h"
};


static u16 s5kgn8_seamless_preview[] = {
0xFCFC, 0x4000,
#include "setting/scm_index.h"
0xFCFC, 0x4000,
0x0104, 0x0101,
#include "setting/s5kgn8_4096_3072_30fps_sub.h"
0xFCFC, 0x4000,
0x0B32, 0x0100,
0x0B30, 0x0101,
0x0340, 0x199A,
0x0104, 0x0001,
};

static u16 addr_data_pair_capture[] = {
#include "setting/s5kgn8_4096_3072_30fps.h"
};
static u16 addr_data_pair_normal_video[] = {
#include "setting/s5kgn8_4096_2304_30fps.h"
};


static u16 s5kgn8_seamless_normal_video[] = {
0xFCFC, 0x4000,
#include "setting/scm_index.h"
0xFCFC, 0x4000,
0x0104, 0x0101,
#include "setting/s5kgn8_4096_2304_30fps_sub.h"
0xFCFC, 0x4000,
0x0B32, 0x0001,
0x0B30, 0x0100,
0x0340, 0x199A,
0x0104, 0x0001,
};

static u16 addr_data_pair_hs_video[] = {
#include "setting/s5kgn8_2048_1152_120fps.h"
};

static u16 addr_data_pair_slim_video[] = {
#include "setting/s5kgn8_4096_3072_30fps.h"
};
static u16 addr_data_pair_custom1[] = {
#include "setting/s5kgn8_8192_6144_24fps.h"
};

static u16 addr_data_pair_custom2[] = {
#include "setting/s5kgn8_4096_2304_60fps.h"
};

static u16 addr_data_pair_custom3[] = {
#include "setting/s5kgn8_2048_1152_240fps.h"
};

static u16 addr_data_pair_custom4[] = {
#include "setting/s5kgn8_4096_2304_30fps_IDCG.h"
};


static u16 s5kgn8_seamless_custom4[] = {
0xFCFC, 0x4000,
#include "setting/scm_index.h"
0xFCFC, 0x4000,
0x0104, 0x0101,
#include "setting/s5kgn8_4096_2304_30fps_IDCG_sub.h"
0xFCFC, 0x4000,
0x0B32, 0x0100,
0x0B30, 0x0101,
0x0340, 0x0AE4,
0x0104, 0x0001,
};

//3rd video call
static u16 addr_data_pair_custom5[] = {
#include "setting/s5kgn8_2048_1536_30fps.h"
};


//in sensor zoom
static u16 addr_data_pair_custom6[] = {
#include "setting/s5kgn8_4096_3072_30fps_crop.h"
};

static u16 s5kgn8_seamless_custom6[] = {
0xFCFC, 0x4000,
#include "setting/scm_index.h"
0xFCFC, 0x4000,
0x0104, 0x0101,
#include "setting/s5kgn8_4096_3072_30fps_crop_sub.h"
0xFCFC, 0x4000,
0x0B32, 0x0100,
0x0B30, 0x0100,
0x0340, 0x15CC,
0x0104, 0x0001,
};


#endif
