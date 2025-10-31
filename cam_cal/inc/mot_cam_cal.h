/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2024 Motorola Inc.
 */

#ifndef __MOT_CAM_CAL_H__
#define __MOT_CAM_CAL_H__

#define EEPROM_DEBUG
#define MOTO_OB_VALUE 64
#define MOTO_WB_VALUE_BASE 64
#define MOT_AWB_RB_MIN_VALUE 200
#define MOT_AWB_G_MIN_VALUE 760
#define MOT_AWB_RBG_MAX_VALUE 880
#define MOT_AWB_GRGB_RATIO_MIN_1000TIMES 970
#define MOT_AWB_GRGB_RATIO_MAX_1000TIMES 1030
#define MOT_AF_ADDR 0x27
#define MOT_AF_DATA_SIZE 24
#define MOT_AWB_ADDR 0x41
#define MOT_AWB_DATA_SIZE 43
#define MAX_temp(a,b,c) (a)>(b)?((a)>(c)?(a):(c)):((b)>(c)?(b):(c))
#define ABS(a,b) a>=b?(a-b):(b-a)

#endif
