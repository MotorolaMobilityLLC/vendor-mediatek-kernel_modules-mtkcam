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



//common
static u16 uTnpArrayInit_0[] = {
0xFCFC, 0x4000,
0x6010, 0x0001,
};

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


//OV
static u16 uTnpArray_ov_gos[] = {
#include "setting/OV/gos_ov.h"
};


static u16 OV_addr_data_pair_preview[] = {
#include "setting/OV/s5kgn8_4096_3072_30fps.h"
};


static u16 OV_s5kgn8_seamless_preview[] = {
0xFCFC, 0x4000,
#include "setting/scm_index.h"
0xFCFC, 0x4000,
0x0104, 0x0101,
#include "setting/OV/s5kgn8_4096_3072_30fps_sub.h"
0xFCFC, 0x4000,
0x0B32, 0x0000,
0x0B30, 0x0101,
0x0340, 0x199A,
//0x0104, 0x0001,
};

static u16 OV_addr_data_pair_capture[] = {
#include "setting/OV/s5kgn8_4096_3072_30fps.h"
};
static u16 OV_addr_data_pair_normal_video[] = {
#include "setting/OV/s5kgn8_4096_2304_30fps.h"
};


static u16 OV_s5kgn8_seamless_normal_video[] = {
0xFCFC, 0x4000,
#include "setting/scm_index.h"
0xFCFC, 0x4000,
0x0104, 0x0101,
#include "setting/OV/s5kgn8_4096_2304_30fps_sub.h"
0xFCFC, 0x4000,
0x0B32, 0x0000,
0x0B30, 0x0100,
0x0340, 0x199A,
//0x0104, 0x0001,
};

static u16 OV_addr_data_pair_hs_video[] = {
#include "setting/OV/s5kgn8_2048_1152_120fps.h"
};

static u16 OV_addr_data_pair_slim_video[] = {
#include "setting/OV/s5kgn8_4096_3072_30fps.h"
};
static u16 OV_addr_data_pair_custom1[] = {
#include "setting/OV/s5kgn8_8192_6144_24fps.h"
};

static u16 OV_addr_data_pair_custom2[] = {
#include "setting/OV/s5kgn8_4096_2304_60fps.h"
};

static u16 OV_addr_data_pair_custom3[] = {
#include "setting/OV/s5kgn8_2048_1152_240fps.h"
};

static u16 OV_addr_data_pair_custom4[] = {
#include "setting/OV/s5kgn8_4096_2304_30fps_IDCG.h"
};


static u16 OV_s5kgn8_seamless_custom4[] = {
0xFCFC, 0x4000,
#include "setting/scm_index.h"
0xFCFC, 0x4000,
0x0104, 0x0101,
#include "setting/OV/s5kgn8_4096_2304_30fps_IDCG_sub.h"
0xFCFC, 0x4000,
0x0B32, 0x0000,
0x0B30, 0x0101,
0x0340, 0x0AE4,
//0x0104, 0x0001,
};

//3rd video call
static u16 OV_addr_data_pair_custom5[] = {
#include "setting/OV/s5kgn8_2048_1536_30fps.h"
};


//in sensor zoom
static u16 OV_addr_data_pair_custom6[] = {
#include "setting/OV/s5kgn8_4096_3072_30fps_crop.h"
};

static u16 OV_s5kgn8_seamless_custom6[] = {
0xFCFC, 0x4000,
#include "setting/scm_index.h"
0xFCFC, 0x4000,
0x0104, 0x0101,
#include "setting/OV/s5kgn8_4096_3072_30fps_crop_sub.h"
0xFCFC, 0x4000,
0x0B32, 0x0000,
0x0B30, 0x0100,
0x0340, 0x15CC,
//0x0104, 0x0001,
};







//QT

static u16 uTnpArray_qt_gos[] = {
#include "setting/QT/gos_qt.h"
};

static u16 QT_addr_data_pair_preview[] = {
#include "setting/QT/s5kgn8_4096_3072_30fps.h"
};


static u16 QT_s5kgn8_seamless_preview[] = {
0xFCFC, 0x4000,
#include "setting/scm_index.h"
0xFCFC, 0x4000,
0x0104, 0x0101,
#include "setting/QT/s5kgn8_4096_3072_30fps_sub.h"
0xFCFC, 0x4000,
0x0B32, 0x0000,
0x0B30, 0x0101,
0x0340, 0x199A,
//0x0104, 0x0001,
};

static u16 QT_addr_data_pair_capture[] = {
#include "setting/QT/s5kgn8_4096_3072_30fps.h"
};
static u16 QT_addr_data_pair_normal_video[] = {
#include "setting/QT/s5kgn8_4096_2304_30fps.h"
};


static u16 QT_s5kgn8_seamless_normal_video[] = {
0xFCFC, 0x4000,
#include "setting/scm_index.h"
0xFCFC, 0x4000,
0x0104, 0x0101,
#include "setting/QT/s5kgn8_4096_2304_30fps_sub.h"
0xFCFC, 0x4000,
0x0B32, 0x0000,
0x0B30, 0x0100,
0x0340, 0x199A,
//0x0104, 0x0001,
};

static u16 QT_addr_data_pair_hs_video[] = {
#include "setting/QT/s5kgn8_2048_1152_120fps.h"
};

static u16 QT_addr_data_pair_slim_video[] = {
#include "setting/QT/s5kgn8_4096_3072_30fps.h"
};
static u16 QT_addr_data_pair_custom1[] = {
#include "setting/QT/s5kgn8_8192_6144_24fps.h"
};

static u16 QT_addr_data_pair_custom2[] = {
#include "setting/QT/s5kgn8_4096_2304_60fps.h"
};

static u16 QT_addr_data_pair_custom3[] = {
#include "setting/QT/s5kgn8_2048_1152_240fps.h"
};

static u16 QT_addr_data_pair_custom4[] = {
#include "setting/QT/s5kgn8_4096_2304_30fps_IDCG.h"
};


static u16 QT_s5kgn8_seamless_custom4[] = {
0xFCFC, 0x4000,
#include "setting/scm_index.h"
0xFCFC, 0x4000,
0x0104, 0x0101,
#include "setting/QT/s5kgn8_4096_2304_30fps_IDCG_sub.h"
0xFCFC, 0x4000,
0x0B32, 0x0000,
0x0B30, 0x0101,
0x0340, 0x0AE4,
//0x0104, 0x0001,
};

//3rd video call
static u16 QT_addr_data_pair_custom5[] = {
#include "setting/QT/s5kgn8_2048_1536_30fps.h"
};


//in sensor zoom
static u16 QT_addr_data_pair_custom6[] = {
#include "setting/QT/s5kgn8_4096_3072_30fps_crop.h"
};

static u16 QT_s5kgn8_seamless_custom6[] = {
0xFCFC, 0x4000,
#include "setting/scm_index.h"
0xFCFC, 0x4000,
0x0104, 0x0101,
#include "setting/QT/s5kgn8_4096_3072_30fps_crop_sub.h"
0xFCFC, 0x4000,
0x0B32, 0x0000,
0x0B30, 0x0100,
0x0340, 0x15CC,
//0x0104, 0x0001,
};


#endif
