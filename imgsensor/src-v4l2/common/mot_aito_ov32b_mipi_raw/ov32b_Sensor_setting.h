/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 ov32b2q_Sensor_setting.h
 *
 * Project:
 * --------
 * Description:
 * ------------
 *	 CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _OV32B_SENSOR_SETTING_H
#define _OV32B_SENSOR_SETTING_H

u16 addr_data_pair_init_ov32b2q[] = {
#include "setting/mot_ov32b_init.h"
};
u16 addr_data_pair_preview_ov32b2q[] = {
#include "setting/OV32B40_3264X2448_MIPI852_30FPS_NORMAL_noPD.h"
};
u16 addr_data_pair_capture_ov32b2q[] = {
#include "setting/OV32B40_3264X2448_MIPI852_30FPS_NORMAL_noPD.h"
};
u16 addr_data_pair_video_ov32b2q[] = {
#include "setting/OV32B40_3264X1836_MIPI852_30FPS_NORMAL_noPD.h"
};
u16 addr_data_pair_hs_video_ov32b2q[] = {
#include "setting/OV32B40_1632X918_MIPI1704_120FPS_NORMAL_noPD.h"
};
u16 addr_data_pair_slim_video_ov32b2q[] = {
#include "setting/OV32B40_3264X2448_MIPI852_30FPS_NORMAL_noPD.h"
};
u16 addr_data_pair_custom1[] = {
#include "setting/OV32B40_3264X1836_MIPI1704_60FPS_NORMAL_noPD.h"
};
#endif