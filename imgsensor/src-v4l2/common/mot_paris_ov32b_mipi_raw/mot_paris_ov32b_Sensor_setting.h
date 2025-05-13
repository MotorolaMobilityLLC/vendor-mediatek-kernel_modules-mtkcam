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
#include "setting/mot_paris_ov32b_init.h"
};
u16 addr_data_pair_preview_ov32b2q[] = {
#include "setting/mot_paris_ov32b_Res_3264x2448_4C1_30fps_751Mbps.h"
};
u16 addr_data_pair_capture_ov32b2q[] = {
#include "setting/mot_paris_ov32b_Res_3264x2448_4C1_30fps_751Mbps.h"
};
u16 addr_data_pair_video_ov32b2q[] = {
#include "setting/mot_paris_ov32b_Res_3264x2448_4C1_30fps_751Mbps.h"
};
u16 addr_data_pair_hs_video_ov32b2q[] = {
#include "setting/mot_paris_ov32b_Res_1920x1080_4C1_120fps_1704Mbps.h"
};
u16 addr_data_pair_slim_video_ov32b2q[] = {
#include "setting/mot_paris_ov32b_Res_3264x2448_4C1_30fps_751Mbps.h"
};
u16 addr_data_pair_custom1[] = {
#include "setting/mot_paris_ov32b_Res_3264x1836_4C1_60fps_1704Mbps.h"
};
#endif