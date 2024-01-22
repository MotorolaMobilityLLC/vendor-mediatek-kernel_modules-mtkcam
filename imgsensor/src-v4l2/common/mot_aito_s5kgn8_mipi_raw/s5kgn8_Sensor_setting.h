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



static u16 uTnpArrayInit[] = {
#include "setting/s5kgn8_initsettting.h"
};

static u16 addr_data_pair_preview[] = {
0xFCFC, 0x4000,
#include "setting/fcm.h"
#include "setting/calibration_xtc.h"
#include "setting/s5kgn8_4096_3072_30fps.h"
#include "setting/fcm_end.h"
};


static u16 s5kgn8_seamless_preview[] = {
0xFCFC, 0x4000,
0x6000, 0x0005,
0x0104, 0x0101,
#include "setting/s5kgn8_4096_3072_30fps_sub.h"
0xFCFC, 0x4000,
0x0B32, 0x0000,
0x0B30, 0x0100,
0x0340, 0x199C,
0x0104, 0x0001,
0x6000, 0x0085,
};

static u16 addr_data_pair_capture[] = {
#include "setting/s5kgn8_4096_3072_30fps.h"
};
static u16 addr_data_pair_normal_video[] = {
#include "setting/s5kgn8_4096_2304_30fps.h"
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

//3rd video call
static u16 addr_data_pair_custom5[] = {
#include "setting/s5kgn8_2048_1536_30fps.h"
};


//in sensor zoom
static u16 addr_data_pair_custom6[] = {
0xFCFC, 0x4000,
#include "setting/fcm.h"
#include "setting/calibration_xtc.h"
#include "setting/s5kgn8_4096_3072_30fps_crop.h"
#include "setting/fcm_end.h"
};

static u16 s5kgn8_seamless_custom6[] = {
0xFCFC, 0x4000,
0x6000, 0x0005,
0x0104, 0x0101,
#include "setting/s5kgn8_4096_3072_30fps_crop_sub.h"
0xFCFC, 0x4000,
0x0B32, 0x0000,
0x0B30, 0x0101,
0x0340, 0x15D4,
0x0104, 0x0001,
0x6000, 0x0085,
};


#endif
