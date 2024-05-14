/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5k3m5sx_Sensor_setting.h
 *
 * Project:
 * --------
 * Description:
 * ------------
 *	 CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _MOT_VIENNA_S5K3K1SENSOR_SETTING_H
#define _MOT_VIENNA_S5K3K1SENSOR_SETTING_H

#include "kd_camera_typedef.h"

static u16 sensor_init_setting_array1[] = {
    0x6028, 0x2000,
    0x0000, 0x0002,
    0x0000, 0x30B1,
    0x6010, 0x0001,
};

static u16 sensor_init_setting_array2[] = {
    0x6214,0xFF7D,
    0x6218,0x0000,
    0x0A02,0x00C0,
    0x6028,0x2000,
    0x602A,0x8E70,
};

static u16 sensor_init_setting_array3_burst[] = {
    0x00F0,
    0x82B8,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0x2DE9,
    0xF041,
    0x0028,
    0x4FD1,
    0x0021,
    0x0120,
    0x00F0,
    0xB6F8,
    0x4D49,
    0xB1F8,
    0x8008,
    0x020A,
    0x4C48,
    0x0068,
    0x80F8,
    0xC020,
    0x91F8,
    0x8128,
    0x80F8,
    0xC220,
    0x91F8,
    0x7828,
    0x00F8,
    0xB82F,
    0x91F8,
    0x7628,
    0x8270,
    0x91F8,
    0x7B18,
    0x0171,
    0x0021,
    0x0846,
    0x00F0,
    0x9CF8,
    0x4248,
    0x434C,
    0x434E,
    0x0068,
    0x434A,
    0x4FF0,
    0x200C,
    0xC08A,
    0xA51E,
    0xB71C,
    0x931C,
    0x4149,
    0xBCEB,
    0x102F,
    0x0FD1,
    0x4868,
    0x2080,
    0x000C,
    0x2880,
    0x087A,
    0x3080,
    0x4889,
    0x3880,
    0x087B,
    0x18B1,
    0x0020,
    0x1080,
    0xC889,
    0x13E0,
    0x0120,
    0xFAE7,
    0x4FF0,
    0x220C,
    0xBCEB,
    0x102F,
    0x0DD1,
    0x0869,
    0x2080,
    0x000C,
    0x2880,
    0x087D,
    0x3080,
    0xC88A,
    0x3880,
    0x087E,
    0x28B1,
    0x0020,
    0x1080,
    0x488B,
    0x1880,
    0xBDE8,
    0xF081,
    0x0120,
    0xF8E7,
    0x0028,
    0x06D1,
    0x2648,
    0x2449,
    0x0068,
    0xB0F8,
    0xE001,
    0xC1F8,
    0x7802,
    0x7047,
    0x00F0,
    0x5FB8,
    0x10B5,
    0x0128,
    0x13D1,
    0x1E4C,
    0x94F8,
    0x3607,
    0x0128,
    0x0ED1,
    0x94F8,
    0x3407,
    0x0028,
    0x0AD0,
    0x00F0,
    0x56F8,
    0x0028,
    0x06D0,
    0x94F8,
    0x3507,
    0xC007,
    0x02D1,
    0x0120,
    0x84F8,
    0x3907,
    0x10BD,
    0x1348,
    0x10B5,
    0x1949,
    0x1A4A,
    0xC0F8,
    0x0017,
    0x511A,
    0xA0F8,
    0x0417,
    0x1848,
    0x0321,
    0x90F8,
    0xF820,
    0x47F6,
    0x0240,
    0x00F0,
    0x40F8,
    0xAFF2,
    0x1B10,
    0x144C,
    0xE062,
    0xAFF2,
    0x7300,
    0x6065,
    0x0D48,
    0x0078,
    0x38B1,
    0x0022,
    0xAFF2,
    0x6F01,
    0xA36E,
    0x0F48,
    0x9847,
    0x0F49,
    0x0860,
    0xAFF2,
    0x7700,
    0xE064,
    0x10BD,
    0x0000,
    0x2000,
    0x4740,
    0x2000,
    0x5120,
    0x2000,
    0x0C30,
    0x4000,
    0xF470,
    0x4000,
    0x9E00,
    0x4000,
    0xFF00,
    0x2000,
    0x9030,
    0x2000,
    0x9054,
    0x2001,
    0xEA00,
    0x2000,
    0x13C0,
    0x2000,
    0x4420,
    0x0000,
    0x0BE7,
    0x2000,
    0x9050,
    0x42F2,
    0x5D6C,
    0xC0F2,
    0x010C,
    0x6047,
    0x40F6,
    0x693C,
    0xC0F2,
    0x000C,
    0x6047,
    0x40F6,
    0xDF1C,
    0xC0F2,
    0x000C,
    0x6047,
    0x48F6,
    0x274C,
    0xC0F2,
    0x000C,
    0x6047,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0x0100,
    0x0000,
    0x0000,
    0x4280,
    0x0000,
    0x00A3,
    0x0400,
    0x0050,
    0x0018,
    0x4280,
    0x0000,
    0x016B,
    0x0400,
    0x03C0,
    0x0000,
    0x0000,
};

static u16 sensor_init_setting_array4[] = {
    0x6028,0x2000,
    0x602A,0x1492,
    0x6F12,0x0100,
    0x602A,0x14BA,
    0x6F12,0x0100,
    0x602A,0x2AE6,
    0x6F12,0x0100,
    0x6F12,0x0FA0,
    0x602A,0x32A4,
    0x6F12,0x2B0A,
    0x602A,0x32A8,
    0x6F12,0xDFCD,
    0x602A,0x32AC,
    0x6F12,0x038F,
    0x602A,0x337A,
    0x6F12,0x006F,
    0x602A,0x3716,
    0x6F12,0x00C0,
    0x602A,0x3A50,
    0x6F12,0x193C,
    0x602A,0x3A5E,
    0x6F12,0x0000,
    0x602A,0x3B5A,
    0x6F12,0x0201,
    0x602A,0x3D60,
    0x6F12,0x0200,
    0x602A,0x3D66,
    0x6F12,0x0001,
    0x602A,0x4218,
    0x6F12,0x0101,
    0x602A,0x4260,
    0x6F12,0x0008,
    0x6F12,0x0801,
    0x602A,0x42D6,
    0x6F12,0x0000,
    0x602A,0x4500,
    0x6F12,0xF5B0,
    0x6F12,0x0000,
    0x6F12,0xF5B2,
    0x6F12,0x0000,
    0x6F12,0xF5B8,
    0x6F12,0x0000,
    0x602A,0x4690,
    0x6F12,0x0000,
    0x602A,0x9034,
    0x6F12,0x0000,
    0x6028,0x4000,
    0x001E,0x000D,
    0x6B5E,0xFF00,
    0x6B60,0x0FFF,
    0x6B72,0x3E00,
    0x6B76,0x0000,
    0xF466,0x0000,
    0xF468,0x4000,
    0xF472,0x001F,
    0xFA08,0x000E,
    0xFA0A,0x0011,
    0x6B76,0x8000,
    0x0101,0x0000,
};
static u16 preview_setting_array[] = {
	0x6028,0x2000,
	0x0136,0x1800,
	0x013E,0x0000,
	0x0304,0x0003,
	0x0306,0x00C7,
	0x030C,0x0001,
	0x0302,0x0001,
	0x0300,0x0004,
	0x030E,0x0003,
	0x0310,0x00B4,
	0x0312,0x0001,
	0x030A,0x0001,
	0x0308,0x0004,
	0x602A,0x1350,
	0x6F12,0x0002,
	0x6F12,0x49F0,
	0x0344,0x0060,
	0x0348,0x0EAF,
	0x0346,0x0050,
	0x034A,0x0B0F,
	0x0350,0x0008,
	0x0352,0x0008,
	0x034C,0x0E40,
	0x034E,0x0AB0,
	0x602A,0x3AC4,
	0x6F12,0x0E40,
	0x602A,0x3AC8,
	0x6F12,0x0AB0,
	0x0900,0x0111,
	0x0404,0x1000,
	0x0380,0x0001,
	0x0382,0x0001,
	0x0384,0x0001,
	0x0386,0x0001,
	0x0342,0x1680,
	0x0340,0x11FA,
	0x0702,0x0000,
	0x0112,0x0A0A,
	0x0114,0x0301,
	0x0116,0x2B00,
	0x0118,0x0002,
	0x011C,0x0100,
	0x081C,0x0000,
	0x081E,0x0000,
	0x0D00,0x0101,
	0x0D02,0x0101,
	0x0D04,0x0101,
	0x0B06,0x0101,
	0x0B08,0x0000,
	0x602A,0x13DE,
	0x6F12,0xAFC8,
	0x602A,0x1480,
	0x6F12,0x0400,
	0x602A,0x1488,
	0x6F12,0x0380,
	0x602A,0x2A64,
	0x6F12,0x0000,
	0x602A,0x2A82,
	0x6F12,0xFFFF,
	0x602A,0x42E0,
	0x6F12,0x0000,
	0x6F12,0x00FA,
	0x602A,0x9038,
	0x6F12,0x0000,
	0x6F12,0x03BA,
	0x6F12,0x0400,
	0x6F12,0x00C0,
	0x602A,0x9044,
	0x6F12,0x0000,
	0x6F12,0x00FA,
	0x6F12,0x0400,
	0x6F12,0x0380,
	0x602A,0x2AF0,
	0x6F12,0x0008,
	0x6F12,0x0013,
	0x602A,0x2B00,
	0x6F12,0x0100,
	0x602A,0x2BAC,
	0x6F12,0x0002,
	0x6F12,0x002D,
	0x6F12,0x003D,
	0x602A,0x2C2A,
	0x6F12,0x002A,
	0x6F12,0x0042,
	0x602A,0x327E,
	0x6F12,0x021C,
	0x602A,0x3288,
	0x6F12,0x0100,
	0x602A,0x32A0,
	0x6F12,0x0048,
	0x6F12,0x4280,
	0x602A,0x3384,
	0x6F12,0x7E24,
	0x602A,0x33A8,
	0x6F12,0x0000,
	0x602A,0x3706,
	0x6F12,0x8010,
	0x6F12,0x0025,
	0x602A,0x3710,
	0x6F12,0x0007,
	0x602A,0x3BAE,
	0x6F12,0x000F,
	0x602A,0x3BC4,
	0x6F12,0x0005,
	0x602A,0x3C06,
	0x6F12,0x000F,
	0x602A,0x3C1C,
	0x6F12,0x0005,
	0x602A,0x3C5E,
	0x6F12,0x000F,
	0x602A,0x3C74,
	0x6F12,0x0005,
	0x602A,0x3CB6,
	0x6F12,0x000F,
	0x602A,0x3CCC,
	0x6F12,0x0005,
	0x602A,0x3D0E,
	0x6F12,0x000F,
	0x602A,0x3D24,
	0x6F12,0x0005,
	0x602A,0x4172,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x602A,0x4264,
	0x6F12,0x0400,
	0x602A,0x4268,
	0x6F12,0x0000,
	0x602A,0x4698,
	0x6F12,0x0003,
	0x602A,0x9036,
	0x6F12,0x4280,
	0x602A,0x9040,
	0x6F12,0x0048,
	0x6F12,0x4280,
	0x0BCC,0x0000,
};

static u16 capture_setting_array[] = {
	0x6028,0x2000,
	0x0136,0x1800,
	0x013E,0x0000,
	0x0304,0x0003,
	0x0306,0x00C7,
	0x030C,0x0001,
	0x0302,0x0001,
	0x0300,0x0004,
	0x030E,0x0003,
	0x0310,0x00B4,
	0x0312,0x0001,
	0x030A,0x0001,
	0x0308,0x0004,
	0x602A,0x1350,
	0x6F12,0x0002,
	0x6F12,0x49F0,
	0x0344,0x0060,
	0x0348,0x0EAF,
	0x0346,0x0050,
	0x034A,0x0B0F,
	0x0350,0x0008,
	0x0352,0x0008,
	0x034C,0x0E40,
	0x034E,0x0AB0,
	0x602A,0x3AC4,
	0x6F12,0x0E40,
	0x602A,0x3AC8,
	0x6F12,0x0AB0,
	0x0900,0x0111,
	0x0404,0x1000,
	0x0380,0x0001,
	0x0382,0x0001,
	0x0384,0x0001,
	0x0386,0x0001,
	0x0342,0x1680,
	0x0340,0x11FA,
	0x0702,0x0000,
	0x0112,0x0A0A,
	0x0114,0x0301,
	0x0116,0x2B00,
	0x0118,0x0002,
	0x011C,0x0100,
	0x081C,0x0000,
	0x081E,0x0000,
	0x0D00,0x0101,
	0x0D02,0x0101,
	0x0D04,0x0101,
	0x0B06,0x0101,
	0x0B08,0x0000,
	0x602A,0x13DE,
	0x6F12,0xAFC8,
	0x602A,0x1480,
	0x6F12,0x0400,
	0x602A,0x1488,
	0x6F12,0x0380,
	0x602A,0x2A64,
	0x6F12,0x0000,
	0x602A,0x2A82,
	0x6F12,0xFFFF,
	0x602A,0x42E0,
	0x6F12,0x0000,
	0x6F12,0x00FA,
	0x602A,0x9038,
	0x6F12,0x0000,
	0x6F12,0x03BA,
	0x6F12,0x0400,
	0x6F12,0x00C0,
	0x602A,0x9044,
	0x6F12,0x0000,
	0x6F12,0x00FA,
	0x6F12,0x0400,
	0x6F12,0x0380,
	0x602A,0x2AF0,
	0x6F12,0x0008,
	0x6F12,0x0013,
	0x602A,0x2B00,
	0x6F12,0x0100,
	0x602A,0x2BAC,
	0x6F12,0x0002,
	0x6F12,0x002D,
	0x6F12,0x003D,
	0x602A,0x2C2A,
	0x6F12,0x002A,
	0x6F12,0x0042,
	0x602A,0x327E,
	0x6F12,0x021C,
	0x602A,0x3288,
	0x6F12,0x0100,
	0x602A,0x32A0,
	0x6F12,0x0048,
	0x6F12,0x4280,
	0x602A,0x3384,
	0x6F12,0x7E24,
	0x602A,0x33A8,
	0x6F12,0x0000,
	0x602A,0x3706,
	0x6F12,0x8010,
	0x6F12,0x0025,
	0x602A,0x3710,
	0x6F12,0x0007,
	0x602A,0x3BAE,
	0x6F12,0x000F,
	0x602A,0x3BC4,
	0x6F12,0x0005,
	0x602A,0x3C06,
	0x6F12,0x000F,
	0x602A,0x3C1C,
	0x6F12,0x0005,
	0x602A,0x3C5E,
	0x6F12,0x000F,
	0x602A,0x3C74,
	0x6F12,0x0005,
	0x602A,0x3CB6,
	0x6F12,0x000F,
	0x602A,0x3CCC,
	0x6F12,0x0005,
	0x602A,0x3D0E,
	0x6F12,0x000F,
	0x602A,0x3D24,
	0x6F12,0x0005,
	0x602A,0x4172,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x602A,0x4264,
	0x6F12,0x0400,
	0x602A,0x4268,
	0x6F12,0x0000,
	0x602A,0x4698,
	0x6F12,0x0003,
	0x602A,0x9036,
	0x6F12,0x4280,
	0x602A,0x9040,
	0x6F12,0x0048,
	0x6F12,0x4280,
	0x0BCC,0x0000,
};

static u16 normal_video_setting_array[] = {
	0x6028,0x2000,
	0x0136,0x1800,
	0x013E,0x0000,
	0x0304,0x0003,
	0x0306,0x00C7,
	0x030C,0x0001,
	0x0302,0x0001,
	0x0300,0x0004,
	0x030E,0x0003,
	0x0310,0x00B4,
	0x0312,0x0001,
	0x030A,0x0001,
	0x0308,0x0004,
	0x602A,0x1350,
	0x6F12,0x0002,
	0x6F12,0x49F0,
	0x0344,0x0060,
	0x0348,0x0EAF,
	0x0346,0x0050,
	0x034A,0x0B0F,
	0x0350,0x0008,
	0x0352,0x0008,
	0x034C,0x0E40,
	0x034E,0x0AB0,
	0x602A,0x3AC4,
	0x6F12,0x0E40,
	0x602A,0x3AC8,
	0x6F12,0x0AB0,
	0x0900,0x0111,
	0x0404,0x1000,
	0x0380,0x0001,
	0x0382,0x0001,
	0x0384,0x0001,
	0x0386,0x0001,
	0x0342,0x1680,
	0x0340,0x11FA,
	0x0702,0x0000,
	0x0112,0x0A0A,
	0x0114,0x0301,
	0x0116,0x2B00,
	0x0118,0x0002,
	0x011C,0x0100,
	0x081C,0x0000,
	0x081E,0x0000,
	0x0D00,0x0101,
	0x0D02,0x0101,
	0x0D04,0x0101,
	0x0B06,0x0101,
	0x0B08,0x0000,
	0x602A,0x13DE,
	0x6F12,0xAFC8,
	0x602A,0x1480,
	0x6F12,0x0400,
	0x602A,0x1488,
	0x6F12,0x0380,
	0x602A,0x2A64,
	0x6F12,0x0000,
	0x602A,0x2A82,
	0x6F12,0xFFFF,
	0x602A,0x42E0,
	0x6F12,0x0000,
	0x6F12,0x00FA,
	0x602A,0x9038,
	0x6F12,0x0000,
	0x6F12,0x03BA,
	0x6F12,0x0400,
	0x6F12,0x00C0,
	0x602A,0x9044,
	0x6F12,0x0000,
	0x6F12,0x00FA,
	0x6F12,0x0400,
	0x6F12,0x0380,
	0x602A,0x2AF0,
	0x6F12,0x0008,
	0x6F12,0x0013,
	0x602A,0x2B00,
	0x6F12,0x0100,
	0x602A,0x2BAC,
	0x6F12,0x0002,
	0x6F12,0x002D,
	0x6F12,0x003D,
	0x602A,0x2C2A,
	0x6F12,0x002A,
	0x6F12,0x0042,
	0x602A,0x327E,
	0x6F12,0x021C,
	0x602A,0x3288,
	0x6F12,0x0100,
	0x602A,0x32A0,
	0x6F12,0x0048,
	0x6F12,0x4280,
	0x602A,0x3384,
	0x6F12,0x7E24,
	0x602A,0x33A8,
	0x6F12,0x0000,
	0x602A,0x3706,
	0x6F12,0x8010,
	0x6F12,0x0025,
	0x602A,0x3710,
	0x6F12,0x0007,
	0x602A,0x3BAE,
	0x6F12,0x000F,
	0x602A,0x3BC4,
	0x6F12,0x0005,
	0x602A,0x3C06,
	0x6F12,0x000F,
	0x602A,0x3C1C,
	0x6F12,0x0005,
	0x602A,0x3C5E,
	0x6F12,0x000F,
	0x602A,0x3C74,
	0x6F12,0x0005,
	0x602A,0x3CB6,
	0x6F12,0x000F,
	0x602A,0x3CCC,
	0x6F12,0x0005,
	0x602A,0x3D0E,
	0x6F12,0x000F,
	0x602A,0x3D24,
	0x6F12,0x0005,
	0x602A,0x4172,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x602A,0x4264,
	0x6F12,0x0400,
	0x602A,0x4268,
	0x6F12,0x0000,
	0x602A,0x4698,
	0x6F12,0x0003,
	0x602A,0x9036,
	0x6F12,0x4280,
	0x602A,0x9040,
	0x6F12,0x0048,
	0x6F12,0x4280,
	0x0BCC,0x0000,
};

static u16 hs_video_setting_array[] = {
	0x6028,0x2000,
	0x0136,0x1800,
	0x013E,0x0000,
	0x0304,0x0003,
	0x0306,0x00C7,
	0x030C,0x0001,
	0x0302,0x0001,
	0x0300,0x0004,
	0x030E,0x0003,
	0x0310,0x00B4,
	0x0312,0x0001,
	0x030A,0x0001,
	0x0308,0x0004,
	0x602A,0x1350,
	0x6F12,0x0002,
	0x6F12,0x49F0,
	0x0344,0x0060,
	0x0348,0x0EAF,
	0x0346,0x0050,
	0x034A,0x0B0F,
	0x0350,0x0008,
	0x0352,0x0008,
	0x034C,0x0E40,
	0x034E,0x0AB0,
	0x602A,0x3AC4,
	0x6F12,0x0E40,
	0x602A,0x3AC8,
	0x6F12,0x0AB0,
	0x0900,0x0111,
	0x0404,0x1000,
	0x0380,0x0001,
	0x0382,0x0001,
	0x0384,0x0001,
	0x0386,0x0001,
	0x0342,0x1680,
	0x0340,0x11FA,
	0x0702,0x0000,
	0x0112,0x0A0A,
	0x0114,0x0301,
	0x0116,0x2B00,
	0x0118,0x0002,
	0x011C,0x0100,
	0x081C,0x0000,
	0x081E,0x0000,
	0x0D00,0x0101,
	0x0D02,0x0101,
	0x0D04,0x0101,
	0x0B06,0x0101,
	0x0B08,0x0000,
	0x602A,0x13DE,
	0x6F12,0xAFC8,
	0x602A,0x1480,
	0x6F12,0x0400,
	0x602A,0x1488,
	0x6F12,0x0380,
	0x602A,0x2A64,
	0x6F12,0x0000,
	0x602A,0x2A82,
	0x6F12,0xFFFF,
	0x602A,0x42E0,
	0x6F12,0x0000,
	0x6F12,0x00FA,
	0x602A,0x9038,
	0x6F12,0x0000,
	0x6F12,0x03BA,
	0x6F12,0x0400,
	0x6F12,0x00C0,
	0x602A,0x9044,
	0x6F12,0x0000,
	0x6F12,0x00FA,
	0x6F12,0x0400,
	0x6F12,0x0380,
	0x602A,0x2AF0,
	0x6F12,0x0008,
	0x6F12,0x0013,
	0x602A,0x2B00,
	0x6F12,0x0100,
	0x602A,0x2BAC,
	0x6F12,0x0002,
	0x6F12,0x002D,
	0x6F12,0x003D,
	0x602A,0x2C2A,
	0x6F12,0x002A,
	0x6F12,0x0042,
	0x602A,0x327E,
	0x6F12,0x021C,
	0x602A,0x3288,
	0x6F12,0x0100,
	0x602A,0x32A0,
	0x6F12,0x0048,
	0x6F12,0x4280,
	0x602A,0x3384,
	0x6F12,0x7E24,
	0x602A,0x33A8,
	0x6F12,0x0000,
	0x602A,0x3706,
	0x6F12,0x8010,
	0x6F12,0x0025,
	0x602A,0x3710,
	0x6F12,0x0007,
	0x602A,0x3BAE,
	0x6F12,0x000F,
	0x602A,0x3BC4,
	0x6F12,0x0005,
	0x602A,0x3C06,
	0x6F12,0x000F,
	0x602A,0x3C1C,
	0x6F12,0x0005,
	0x602A,0x3C5E,
	0x6F12,0x000F,
	0x602A,0x3C74,
	0x6F12,0x0005,
	0x602A,0x3CB6,
	0x6F12,0x000F,
	0x602A,0x3CCC,
	0x6F12,0x0005,
	0x602A,0x3D0E,
	0x6F12,0x000F,
	0x602A,0x3D24,
	0x6F12,0x0005,
	0x602A,0x4172,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x602A,0x4264,
	0x6F12,0x0400,
	0x602A,0x4268,
	0x6F12,0x0000,
	0x602A,0x4698,
	0x6F12,0x0003,
	0x602A,0x9036,
	0x6F12,0x4280,
	0x602A,0x9040,
	0x6F12,0x0048,
	0x6F12,0x4280,
	0x0BCC,0x0000,
};

static u16 slim_video_setting_array[] = {
	0x6028,0x2000,
	0x0136,0x1800,
	0x013E,0x0000,
	0x0304,0x0003,
	0x0306,0x00C7,
	0x030C,0x0001,
	0x0302,0x0001,
	0x0300,0x0004,
	0x030E,0x0003,
	0x0310,0x00B4,
	0x0312,0x0001,
	0x030A,0x0001,
	0x0308,0x0004,
	0x602A,0x1350,
	0x6F12,0x0002,
	0x6F12,0x49F0,
	0x0344,0x0060,
	0x0348,0x0EAF,
	0x0346,0x0050,
	0x034A,0x0B0F,
	0x0350,0x0008,
	0x0352,0x0008,
	0x034C,0x0E40,
	0x034E,0x0AB0,
	0x602A,0x3AC4,
	0x6F12,0x0E40,
	0x602A,0x3AC8,
	0x6F12,0x0AB0,
	0x0900,0x0111,
	0x0404,0x1000,
	0x0380,0x0001,
	0x0382,0x0001,
	0x0384,0x0001,
	0x0386,0x0001,
	0x0342,0x1680,
	0x0340,0x11FA,
	0x0702,0x0000,
	0x0112,0x0A0A,
	0x0114,0x0301,
	0x0116,0x2B00,
	0x0118,0x0002,
	0x011C,0x0100,
	0x081C,0x0000,
	0x081E,0x0000,
	0x0D00,0x0101,
	0x0D02,0x0101,
	0x0D04,0x0101,
	0x0B06,0x0101,
	0x0B08,0x0000,
	0x602A,0x13DE,
	0x6F12,0xAFC8,
	0x602A,0x1480,
	0x6F12,0x0400,
	0x602A,0x1488,
	0x6F12,0x0380,
	0x602A,0x2A64,
	0x6F12,0x0000,
	0x602A,0x2A82,
	0x6F12,0xFFFF,
	0x602A,0x42E0,
	0x6F12,0x0000,
	0x6F12,0x00FA,
	0x602A,0x9038,
	0x6F12,0x0000,
	0x6F12,0x03BA,
	0x6F12,0x0400,
	0x6F12,0x00C0,
	0x602A,0x9044,
	0x6F12,0x0000,
	0x6F12,0x00FA,
	0x6F12,0x0400,
	0x6F12,0x0380,
	0x602A,0x2AF0,
	0x6F12,0x0008,
	0x6F12,0x0013,
	0x602A,0x2B00,
	0x6F12,0x0100,
	0x602A,0x2BAC,
	0x6F12,0x0002,
	0x6F12,0x002D,
	0x6F12,0x003D,
	0x602A,0x2C2A,
	0x6F12,0x002A,
	0x6F12,0x0042,
	0x602A,0x327E,
	0x6F12,0x021C,
	0x602A,0x3288,
	0x6F12,0x0100,
	0x602A,0x32A0,
	0x6F12,0x0048,
	0x6F12,0x4280,
	0x602A,0x3384,
	0x6F12,0x7E24,
	0x602A,0x33A8,
	0x6F12,0x0000,
	0x602A,0x3706,
	0x6F12,0x8010,
	0x6F12,0x0025,
	0x602A,0x3710,
	0x6F12,0x0007,
	0x602A,0x3BAE,
	0x6F12,0x000F,
	0x602A,0x3BC4,
	0x6F12,0x0005,
	0x602A,0x3C06,
	0x6F12,0x000F,
	0x602A,0x3C1C,
	0x6F12,0x0005,
	0x602A,0x3C5E,
	0x6F12,0x000F,
	0x602A,0x3C74,
	0x6F12,0x0005,
	0x602A,0x3CB6,
	0x6F12,0x000F,
	0x602A,0x3CCC,
	0x6F12,0x0005,
	0x602A,0x3D0E,
	0x6F12,0x000F,
	0x602A,0x3D24,
	0x6F12,0x0005,
	0x602A,0x4172,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x602A,0x4264,
	0x6F12,0x0400,
	0x602A,0x4268,
	0x6F12,0x0000,
	0x602A,0x4698,
	0x6F12,0x0003,
	0x602A,0x9036,
	0x6F12,0x4280,
	0x602A,0x9040,
	0x6F12,0x0048,
	0x6F12,0x4280,
	0x0BCC,0x0000,
};

static u16 custom1_setting_array[] = {
	0x6028,0x2000,
	0x0136,0x1800,
	0x013E,0x0000,
	0x0304,0x0003,
	0x0306,0x00C7,
	0x030C,0x0001,
	0x0302,0x0001,
	0x0300,0x0004,
	0x030E,0x0003,
	0x0310,0x00B4,
	0x0312,0x0001,
	0x030A,0x0001,
	0x0308,0x0004,
	0x602A,0x1350,
	0x6F12,0x0002,
	0x6F12,0x49F0,
	0x0344,0x0060,
	0x0348,0x0EAF,
	0x0346,0x0050,
	0x034A,0x0B0F,
	0x0350,0x0008,
	0x0352,0x0008,
	0x034C,0x0E40,
	0x034E,0x0AB0,
	0x602A,0x3AC4,
	0x6F12,0x0E40,
	0x602A,0x3AC8,
	0x6F12,0x0AB0,
	0x0900,0x0111,
	0x0404,0x1000,
	0x0380,0x0001,
	0x0382,0x0001,
	0x0384,0x0001,
	0x0386,0x0001,
	0x0342,0x1680,
	0x0340,0x11FA,
	0x0702,0x0000,
	0x0112,0x0A0A,
	0x0114,0x0301,
	0x0116,0x2B00,
	0x0118,0x0002,
	0x011C,0x0100,
	0x081C,0x0000,
	0x081E,0x0000,
	0x0D00,0x0101,
	0x0D02,0x0101,
	0x0D04,0x0101,
	0x0B06,0x0101,
	0x0B08,0x0000,
	0x602A,0x13DE,
	0x6F12,0xAFC8,
	0x602A,0x1480,
	0x6F12,0x0400,
	0x602A,0x1488,
	0x6F12,0x0380,
	0x602A,0x2A64,
	0x6F12,0x0000,
	0x602A,0x2A82,
	0x6F12,0xFFFF,
	0x602A,0x42E0,
	0x6F12,0x0000,
	0x6F12,0x00FA,
	0x602A,0x9038,
	0x6F12,0x0000,
	0x6F12,0x03BA,
	0x6F12,0x0400,
	0x6F12,0x00C0,
	0x602A,0x9044,
	0x6F12,0x0000,
	0x6F12,0x00FA,
	0x6F12,0x0400,
	0x6F12,0x0380,
	0x602A,0x2AF0,
	0x6F12,0x0008,
	0x6F12,0x0013,
	0x602A,0x2B00,
	0x6F12,0x0100,
	0x602A,0x2BAC,
	0x6F12,0x0002,
	0x6F12,0x002D,
	0x6F12,0x003D,
	0x602A,0x2C2A,
	0x6F12,0x002A,
	0x6F12,0x0042,
	0x602A,0x327E,
	0x6F12,0x021C,
	0x602A,0x3288,
	0x6F12,0x0100,
	0x602A,0x32A0,
	0x6F12,0x0048,
	0x6F12,0x4280,
	0x602A,0x3384,
	0x6F12,0x7E24,
	0x602A,0x33A8,
	0x6F12,0x0000,
	0x602A,0x3706,
	0x6F12,0x8010,
	0x6F12,0x0025,
	0x602A,0x3710,
	0x6F12,0x0007,
	0x602A,0x3BAE,
	0x6F12,0x000F,
	0x602A,0x3BC4,
	0x6F12,0x0005,
	0x602A,0x3C06,
	0x6F12,0x000F,
	0x602A,0x3C1C,
	0x6F12,0x0005,
	0x602A,0x3C5E,
	0x6F12,0x000F,
	0x602A,0x3C74,
	0x6F12,0x0005,
	0x602A,0x3CB6,
	0x6F12,0x000F,
	0x602A,0x3CCC,
	0x6F12,0x0005,
	0x602A,0x3D0E,
	0x6F12,0x000F,
	0x602A,0x3D24,
	0x6F12,0x0005,
	0x602A,0x4172,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x602A,0x4264,
	0x6F12,0x0400,
	0x602A,0x4268,
	0x6F12,0x0000,
	0x602A,0x4698,
	0x6F12,0x0003,
	0x602A,0x9036,
	0x6F12,0x4280,
	0x602A,0x9040,
	0x6F12,0x0048,
	0x6F12,0x4280,
	0x0BCC,0x0000,
};

#endif
