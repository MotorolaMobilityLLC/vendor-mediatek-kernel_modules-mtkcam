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
#include "setting/s5kgn8_4096_3072_30fps.h"
};

static u16 addr_data_pair_capture[] = {
#include "setting/s5kgn8_4096_3072_30fps.h"
};
static u16 addr_data_pair_normal_video[] = {
#include "setting/s5kgn8_4096_3072_30fps.h"
};

static u16 addr_data_pair_hs_video[] = {
#include "setting/s5kgn8_4096_3072_30fps.h"
};

static u16 addr_data_pair_slim_video[] = {
#include "setting/s5kgn8_4096_3072_30fps.h"
};

static u16 addr_data_pair_cust1[] = {
#include "setting/s5kgn8_4096_3072_30fps.h"
};


#endif
