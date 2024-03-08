/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 gc13a2_Sensor_setting.h
 *
 * Project:
 * --------
 * Description:
 * ------------
 *	 CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _GC13A2_SENSOR_SETTING_H
#define _GC13A2_SENSOR_SETTING_H

/* SENSOR MIRROR FLIP INFO */
#define GC13A2_IMAGE_NORMAL           1
#define GC13A2_IMAGE_H_MIRROR         0
#define GC13A2_IMAGE_V_MIRROR         0
#define GC13A2_IMAGE_HV_MIRROR        0

#if GC13A2_IMAGE_NORMAL
#define MIRROR	     0x00
#define PARA	     0x01
#elif GC13A2_IMAGE_H_MIRROR
#define MIRROR	     0x01
#define PARA	     0x02
#elif GC13A2_IMAGE_V_MIRROR
#define MIRROR	     0x02
#define PARA	     0x01
#elif GC13A2_IMAGE_HV_MIRROR
#define MIRROR	     0x03
#define PARA	     0x02
#else
#define MIRROR	    0x00
#define PARA	     0x01
#endif

#define GC13A2_INIT__V0_01_SETTING 1
#define DEBUG_STREAM_OFF_DELAY 0

#define GC13A2_RAW10_V0_02_SETTING 1

#define GC13A2_RAW12_V0_02_SETTING 0
#define GC13A2_RAW12_V0_03_SETTING 0

// V0.01
#if GC13A2_INIT__V0_01_SETTING
/* init setting */
u16 gc13a2_init_setting[] = {
	/*system*/
	0x031c, 0x20,
	0x0fbd, 0x04,
	0x0900, 0x10,
	0x0136, 0x03,
	0x013c, 0x00,
	0x013d, 0x94,
	0x013e, 0x10,
	0x0901, 0x00,
	0x0134, 0x10,
	0x0135, 0x02,
	0x0137, 0x77,
	0x0f61, 0x03,
	0x0f63, 0x00,
	0x0f64, 0xce,
	0x0f66, 0x77,
	0x0f65, 0x50,
	0x0fbe, 0xf7,
	0x0314, 0x01,
	0x0315, 0xd8,
	0x031a, 0x06,
	0x013f, 0x00,
	0x0901, 0x1f,
	0x0134, 0x54,
	0x0f62, 0x1b,
	0x031c, 0xe0,
	/*Pre_set*/
	0x0f88, 0x33,
	0x0f87, 0x8c,
	0x0f8c, 0x01,
	0x0f8e, 0x04,
	0x0fbd, 0x04,
	0x0218, 0x00,
	0x0580, 0x00,
	0x008b, 0x08,
	0x008c, 0x00,
	0x008e, 0x30,
	0x008f, 0x00,
	0x0083, 0x02,
	0x0084, 0x21,
	0x0087, 0x50,
	0x0af4, 0x40,
	0x0510, 0x01,
	0x00bd, 0x02,
	0x0359, 0x00,
	0x0101, 0x03,
	/*analog*/
	0x0f0a, 0x00,
	0x0f0b, 0x04,
	0x0f0c, 0x04,
	0x0f0d, 0x24,
	0x0faa, 0x00,
	0x0fab, 0x04,
	0x0fac, 0x04,
	0x0fad, 0x24,
	0x0346, 0x00,
	0x0347, 0x30,
	0x0348, 0x10,
	0x0349, 0x80,
	0x034a, 0x0c,
	0x034b, 0x40,
	0x0f25, 0x14,
	0x0f45, 0x14,
	0x0202, 0x0b,
	0x0203, 0x64,
	0x0342, 0x08,
	0x0343, 0x40,
	0x0340, 0x0c,
	0x0341, 0xb0,
	0x0226, 0x0c,
	0x0227, 0x60,
	0x0225, 0x04,
	0x0254, 0x40,
	0x0219, 0x44,
	0x021a, 0x17,
	0x021d, 0x02,
	0x0220, 0x04,
	0x0221, 0x02,
	0x0e5e, 0x01,
	0x0e5f, 0x00,
	0x0e5c, 0x1e,
	0x0e0c, 0x03,
	0x0e0f, 0x00,
	0x0ed9, 0x03,
	0x0eda, 0x00,
	0x0f27, 0x00,
	0x0f28, 0x40,
	0x0f47, 0x00,
	0x0f48, 0x40,
	0x0e5b, 0x36,
	0x0e5d, 0x36,
	0x0edb, 0x36,
	0x0edc, 0x36,
	0x0f10, 0x10,
	0x0f8c, 0x2f,
	0x0f81, 0x4c,
	0x0f82, 0x11,
	0x0f12, 0x45,
	0x0f15, 0x0e,
	0x0f1a, 0x17,
	0x0f17, 0x66,
	0x0f18, 0x45,
	0x0f19, 0x0d,
	0x0f1c, 0x1c,
	0x0f80, 0xb5,
	0x0f83, 0x4a,
	0x0f86, 0x3f,
	0x0e90, 0x21,
	0x0e9e, 0x80,
	0x0ea6, 0x00,
	0x0ea1, 0x30,
	0x0e99, 0x44,
	0x0e94, 0x04,
	0x0e95, 0x30,
	0x0e96, 0x0f,
	0x0e97, 0x40,
	0x0e9a, 0x1c,
	0x0e9b, 0x4f,
	0x0e9c, 0x03,
	0x0ea2, 0x24,
	0x0ea3, 0x24,
	0x0ea7, 0x03,
	0x0ed6, 0x0c,
	0x0edd, 0x01,
	0x0ede, 0x22,
	0x0247, 0x04,
	0x0e1f, 0x01,
	0x0e22, 0x01,
	0x0e69, 0x01,
	0x0e6c, 0x01,
	0x0e58, 0x01,
	0x0e73, 0x01,
	0x0e64, 0x00,
	0x0e65, 0x00,
	0x0e75, 0x00,
	0x0ed1, 0x04,
	0x0e6f, 0x66,
	0x0e13, 0x01,
	0x0e14, 0x01,
	0x0e16, 0x05,
	0x0e1b, 0x05,
	0x0e29, 0x05,
	0x0e4d, 0x60,
	0x0e50, 0x48,
	0x0ec0, 0x1f,
	0x0ec1, 0x01,
	0x0ec3, 0x1e,
	0x0ec5, 0x14,
	0x0ec6, 0x14,
	0x0ecb, 0x14,
	0x0ecc, 0x14,
	0x0ecd, 0x14,
	0x0ece, 0x14,
	0x0edf, 0x21,
	0x0ee0, 0x35,
	/*autoload*/
	0x0a8c, 0x01,
	0x0a8c, 0x00,
	0x0a70, 0x80,
	0x0a67, 0x80,
	0x0a66, 0x00,
	0x0a56, 0x0e,
	0x0a74, 0x05,
	0x0a68, 0x04,
	0x0a6f, 0x04,
	0x0a79, 0x0c,
	0x0a50, 0x00,
	0x00ce, 0x09,
	0x00a1, 0x06,
	0x00a4, 0x00,
	0x00a5, 0x01,
	0x0a90, 0x13,
	0x0a91, 0x12,
	0x0a92, 0x02,
	0x0a93, 0x7f,
	0x0a95, 0xf4,
	0x00aa, 0x00,
	0x00ab, 0x00,
	0x00ac, 0x00,
	0x00ad, 0x00,
	0x00a6, 0x10,
	0x00a7, 0x80,
	0x00a8, 0x0c,
	0x00a9, 0x40,
	0x0aa6, 0x03,
	0x0aa7, 0x1a,
	0x0aa8, 0xb8,
	0x0aab, 0x1b,
	0x0aac, 0x60,
	0x0aad, 0x40,
	0x0aae, 0x00,
	0x0aaf, 0xe0,
	0x0a94, 0x80,
	/*testline*/
	0x0511, 0x01,
	0x0082, 0x3f,
	0x0250, 0xd8,
	0x0251, 0x00,
	0x0350, 0x01,
	0x0ea4, 0x23,
	0x0f8d, 0x02,
	0x0f8f, 0x23,
	0x0f90, 0x01,
	/*standby*/
	0x0af4, 0x68,
	0x0af6, 0x20,
	0x0b00, 0x04,
	0x0b01, 0xbd,
	0x0b02, 0x1f,
	0x0b03, 0x00,
	0x0b04, 0x01,
	0x0b05, 0x14,
	0x0b06, 0x13,
	0x0b07, 0x00,
	0x0b08, 0x1b,
	0x0b09, 0x62,
	0x0b0a, 0x1f,
	0x0b0b, 0x00,
	0x0b0c, 0x1f,
	0x0b0d, 0x01,
	0x0b0e, 0x19,
	0x0b0f, 0x00,
	0x0b10, 0x54,
	0x0b11, 0x34,
	0x0b12, 0x11,
	0x0b13, 0x00,
	0x0b14, 0x80,
	0x0b15, 0x1c,
	0x0b16, 0x13,
	0x0b17, 0x00,
	0x0b18, 0x30,
	0x0b19, 0xfe,
	0x0b1a, 0x13,
	0x0b1b, 0x00,
	0x0b1c, 0x00,
	0x0b1d, 0xfe,
	0x0b1e, 0x13,
	0x0b1f, 0x00,
	0x0b20, 0x9f,
	0x0b21, 0x1c,
	0x0b22, 0x13,
	0x0b23, 0x00,
	0x0b24, 0x00,
	0x0b25, 0xfe,
	0x0b26, 0x13,
	0x0b27, 0x00,
	0x0b28, 0x00,
	0x0b29, 0xfe,
	0x0b2a, 0x13,
	0x0b2b, 0x00,
	0x0b2c, 0x80,
	0x0b2d, 0x1c,
	0x0b2e, 0x13,
	0x0b2f, 0x00,
	0x0b30, 0x30,
	0x0b31, 0xfe,
	0x0b32, 0x13,
	0x0b33, 0x00,
	0x0b34, 0x00,
	0x0b35, 0xfe,
	0x0b36, 0x13,
	0x0b37, 0x00,
	0x0b38, 0x9f,
	0x0b39, 0x1c,
	0x0b3a, 0x13,
	0x0b3b, 0x00,
	0x0b3c, 0x01,
	0x0b3d, 0x14,
	0x0b3e, 0x19,
	0x0b3f, 0x00,
	0x0b40, 0x01,
	0x0b41, 0x34,
	0x0b42, 0x19,
	0x0b43, 0x00,
	0x0b44, 0x01,
	0x0b45, 0x54,
	0x0b46, 0x19,
	0x0b47, 0x00,
	0x0b48, 0x01,
	0x0b49, 0x74,
	0x0b4a, 0x19,
	0x0b4b, 0x00,
	0x0b4c, 0x01,
	0x0b4d, 0x94,
	0x0b4e, 0x19,
	0x0b4f, 0x00,
	0x0b50, 0x0a,
	0x0b51, 0x02,
	0x0b52, 0x19,
	0x0b53, 0x00,
	0x0b54, 0x10,
	0x0b55, 0x84,
	0x0b56, 0x11,
	0x0b57, 0x00,
	0x0b58, 0x1f,
	0x0b59, 0x81,
	0x0b5a, 0x11,
	0x0b5b, 0x00,
	0x0b5c, 0xd9,
	0x0b5d, 0x02,
	0x0b5e, 0x11,
	0x0b5f, 0x00,
	0x0b60, 0x00,
	0x0b61, 0xfe,
	0x0b62, 0x73,
	0x0b63, 0x00,
	0x0b64, 0x00,
	0x0b65, 0xfe,
	0x0b66, 0x73,
	0x0b67, 0x00,
	0x0b68, 0x00,
	0x0b69, 0xfe,
	0x0b6a, 0x73,
	0x0b6b, 0x00,
	0x0b6c, 0xd9,
	0x0b6d, 0x02,
	0x0b6e, 0x11,
	0x0b6f, 0x00,
	0x0b70, 0x00,
	0x0b71, 0xfe,
	0x0b72, 0x13,
	0x0b73, 0x00,
	0x0b74, 0x07,
	0x0b75, 0x14,
	0x0b76, 0x13,
	0x0b77, 0x00,
	0x0b78, 0x00,
	0x0b79, 0x84,
	0x0b7a, 0x11,
	0x0b7b, 0x00,
	0x0b7c, 0x00,
	0x0b7d, 0x81,
	0x0b7e, 0x71,
	0x0b7f, 0x00,
	0x0b80, 0xe0,
	0x0b81, 0x1c,
	0x0b82, 0x13,
	0x0b83, 0x00,
	0x0b84, 0x00,
	0x0b85, 0x14,
	0x0b86, 0x19,
	0x0b87, 0x00,
	0x0b88, 0x00,
	0x0b89, 0x34,
	0x0b8a, 0x19,
	0x0b8b, 0x00,
	0x0b8c, 0x00,
	0x0b8d, 0x54,
	0x0b8e, 0x19,
	0x0b8f, 0x00,
	0x0b90, 0x00,
	0x0b91, 0x74,
	0x0b92, 0x19,
	0x0b93, 0x00,
	0x0b94, 0x00,
	0x0b95, 0x94,
	0x0b96, 0x19,
	0x0b97, 0x00,
	0x0b98, 0x00,
	0x0b99, 0x01,
	0x0b9a, 0x19,
	0x0b9b, 0x00,
	0x0b9c, 0x08,
	0x0b9d, 0x02,
	0x0b9e, 0x19,
	0x0b9f, 0x00,
	0x0ba0, 0x1a,
	0x0ba1, 0x62,
	0x0ba2, 0x1f,
	0x0ba3, 0x00,
	0x0ba4, 0x14,
	0x0ba5, 0x34,
	0x0ba6, 0x11,
	0x0ba7, 0x00,
	0x0ba8, 0x00,
	0x0ba9, 0xbd,
	0x0baa, 0x1f,
	0x0bab, 0x00,
	0x0aea, 0x2b,
	0x0ae9, 0x18,
	0x0266, 0x03,
	0x0ae8, 0x14,
	/*Gain_p*/
	0x05a0, 0xC1,
	0x05ac, 0x00,
	0x05ad, 0x01,
	0x05ae, 0x00,
	0x0800, 0x05,
	0x0801, 0x06,
	0x0802, 0x09,
	0x0803, 0x0c,
	0x0804, 0x11,
	0x0805, 0x17,
	0x0806, 0x0f,
	0x0807, 0x8f,
	0x0808, 0x00,
	0x0809, 0x9a,
	0x080A, 0x00,
	0x080B, 0x9b,
	0x080C, 0x0e,
	0x080D, 0xd9,
	0x080E, 0x0e,
	0x080F, 0xda,
	0x0810, 0x0e,
	0x0811, 0x0c,
	0x0812, 0x0e,
	0x0813, 0x0f,
	0x0814, 0x0e,
	0x0815, 0xa1,
	0x0816, 0x0e,
	0x0817, 0x58,
	0x0818, 0x0e,
	0x0819, 0x73,
	0x081a, 0x27,
	0x081b, 0x00,
	0x081C, 0xb4,
	0x081D, 0x02,
	0x081E, 0x10,
	0x081F, 0x02,
	0x0820, 0x10,
	0x0821, 0x30,
	0x0822, 0x10,
	0x0823, 0x10,
	0x0824, 0x37,
	0x0825, 0x00,
	0x0826, 0xb4,
	0x0827, 0x01,
	0x0828, 0x28,
	0x0829, 0x01,
	0x082A, 0x28,
	0x082B, 0x30,
	0x082C, 0x10,
	0x082D, 0x10,
	0x082E, 0x47,
	0x082F, 0x00,
	0x0830, 0xb4,
	0x0831, 0x01,
	0x0832, 0x2e,
	0x0833, 0x01,
	0x0834, 0x2e,
	0x0835, 0x10,
	0x0836, 0x10,
	0x0837, 0x10,
	0x0838, 0x56,
	0x0839, 0x00,
	0x083A, 0xb4,
	0x083B, 0x01,
	0x083C, 0x26,
	0x083D, 0x01,
	0x083E, 0x26,
	0x083F, 0x10,
	0x0840, 0x10,
	0x0841, 0x10,
	0x0842, 0x66,
	0x0843, 0x00,
	0x0844, 0xb4,
	0x0845, 0x01,
	0x0846, 0x1e,
	0x0847, 0x01,
	0x0848, 0x1e,
	0x0849, 0x10,
	0x084A, 0x0e,
	0x084B, 0x0e,
	0x084C, 0x65,
	0x084D, 0x00,
	0x084E, 0xb4,
	0x084F, 0x01,
	0x0850, 0x14,
	0x0851, 0x01,
	0x0852, 0x14,
	0x0853, 0x10,
	0x0854, 0x0e,
	0x0855, 0x0e,
	0x0856, 0x65,
	0x0857, 0x00,
	0x0858, 0xb4,
	0x0859, 0x01,
	0x085A, 0x0a,
	0x085B, 0x01,
	0x085C, 0x0a,
	0x085D, 0x10,
	0x085E, 0x12,
	0x085F, 0x12,
	0x0860, 0x00,
	0x0861, 0x00,
	0x0862, 0x00,
	0x0863, 0x00,
	0x0864, 0x00,
	0x0865, 0x00,
	0x0866, 0x00,
	0x0867, 0x01,
	0x0868, 0x00,
	0x0869, 0x01,
	0x086a, 0x03,
	0x086b, 0x01,
	0x086c, 0x54,
	0x086d, 0x00,
	0x086e, 0x04,
	0x086f, 0x01,
	0x0870, 0xdd,
	0x0871, 0x00,
	0x0872, 0x05,
	0x0873, 0x02,
	0x0874, 0xa9,
	0x0875, 0x00,
	0x0876, 0x06,
	0x0877, 0x03,
	0x0878, 0xbc,
	0x0879, 0x00,
	0x087a, 0x07,
	0x087b, 0x05,
	0x087c, 0x46,
	0x087d, 0x04,
	0x087e, 0x97,
	0x087f, 0x07,
	0x0880, 0x8a,
	0x0881, 0x08,
	0x0882, 0x07,
	0x05ae, 0x00,
	0x05ac, 0x01,
	0x05a5, 0x00,
	0x05a6, 0x00,
	0x05a3, 0x07,
	0x05a4, 0x0a,
	0x0590, 0x00,
	0x0591, 0x60,
	0x05ab, 0x07,
	0x020a, 0x04,
	0x05a0, 0xc1,
	/*RTC*/
	0x0d80, 0x30,
	0x0d81, 0x2e,
	0x0d82, 0x28,
	0x0d83, 0x20,
	0x0d84, 0x1c,
	0x0d85, 0x18,
	0x0d86, 0x60,
	0x0d87, 0xe0,
	0x0d88, 0x08,
	0x0d89, 0x06,
	0x0d8a, 0x20,
	0x0d8b, 0x30,
	0x0d8c, 0x0a,
	0x0d8d, 0x10,
	0x0d8e, 0x16,
	0x0d8f, 0x60,
	0x0d90, 0x33,
	0x0d91, 0x33,
	0x0d92, 0x33,
	0x0d93, 0x43,
	0x0d94, 0x33,
	0x0d95, 0x33,
	0x0d96, 0x22,
	0x0d97, 0x22,
	0x0d98, 0x00,
	0x00cc, 0x03,
	0x00ce, 0x09,
	0x00c0, 0x18,
	0x00c1, 0x00,
	0x00c2, 0x35,
	0x00c3, 0x10,
	0x00c9, 0x0c,
	0x00ca, 0x00,
	0x00cb, 0x08,
	0x00c4, 0x10,
	0x00f4, 0x08,
	/*BLK*/
	0x0040, 0x22,
	0x0030, 0x0f,
	0x0031, 0xf0,
	0x0032, 0x00,
	0x0033, 0x0f,
	0x0041, 0x20,
	0x0043, 0x00,
	0x0044, 0x46,
	0x0096, 0x20,
	0x0098, 0x40,
	0x0097, 0x03,
	/*dither*/
	0x0070, 0x01,
	0x0071, 0x81,
	0x0072, 0x87,
	/*ISP*/
	0x0089, 0x02,
	0x0080, 0xd0,
	0x0073, 0x01,
	0x0074, 0x00,
	0x0075, 0x10,
	/*PDAF*/
	0x00d3, 0x23,
	0x00ec, 0x00,
	0x00ed, 0x20,
	0x00ee, 0x00,
	0x00ef, 0x20,
	0x00f0, 0x0c,
	0x00f1, 0x20,
	0x00f2, 0x08,
	0x00f3, 0x20,
	0x00dc, 0x00,
	0x00dd, 0x20,
	0x00de, 0x00,
	0x00df, 0x20,
	0x00e0, 0x0c,
	0x00e1, 0x20,
	0x00e2, 0x08,
	0x00e3, 0x20,
	0x0145, 0x80,
	0x0147, 0xa0,
	0x0142, 0x00,
	0x0140, 0x83,
	0x0107, 0x80,
	0x0109, 0x0c,
	0x0123, 0x23,
	0x0124, 0x01,
	0x0125, 0x02,
	0x00a0, 0x07,
	0x00d2, 0x01,
	/*gain*/
	0x009a, 0x00,
	0x009b, 0xb4,
	0x0204, 0x00,
	0x0205, 0x20,
	0x0206, 0x00,
	/*nolinear*/
	0x05cf, 0x03,
	0x05d0, 0x83,
	0x05d3, 0x3f,
	0x05d4, 0x3D,
	0x05cc, 0x1a,
	0x05d1, 0x1c,
	0x0f33, 0x00,
	0x0f30, 0x00,
	0x0f31, 0x08,
	0x0f32, 0xa4,
	0x0f34, 0x3f,
	0x0f4b, 0x00,
	0x0f4c, 0x3f,
	0x0f37, 0x23,
	0x0f50, 0x23,
	0x0255, 0xe4,
	/*out*/
	0x0351, 0x00,
	0x0352, 0x08,
	0x0353, 0x00,
	0x0354, 0x08,
	0x034c, 0x10,
	0x034d, 0x70,
	0x034e, 0x0c,
	0x034f, 0x30,
	/*MIPI*/
	0x0113, 0x02,
	0x0114, 0x02,
	0x0119, 0x04,
	0x0128, 0x2b,
	0x0182, 0x20,
	0x0181, 0x00,
	0x0184, 0x10,
	0x0902, 0x0a,
	0x0903, 0x05,
	0x0914, 0x01,
	0x0934, 0x01,
	0x0954, 0x01,
	0x0974, 0x01,
	0x0994, 0x01,
	0x096b, 0x9f,
	0x0190, 0x00,
	0x01a5, 0x0a,
	0x01a4, 0x02,
	0x01a2, 0x0b,
	0x01a3, 0x30,
	0x01a6, 0x0d,
	0x01a1, 0x0e,
	0x01a9, 0x0b,
	0x01aa, 0x12,
	0x01ab, 0x0c,
	0x01a7, 0x17,
	/*auto off*/
	0x00a4, 0x80,
	0x0a90, 0x00,
	0x0a70, 0x00,
	0x0a67, 0x00,
};
#endif

#if GC13A2_RAW10_V0_02_SETTING
u16 gc13a2_4160x3120_data_raw10[] = {
	/*system*/
	0x031c, 0x20,
	0x0fbd, 0x04,
	0x0900, 0x10,
	0x0136, 0x03,
	0x013c, 0x00,
	0x013d, 0x94,
	0x013e, 0x10,
	0x0135, 0x02,
	0x0137, 0x77,
	0x0f61, 0x03,
	0x0f63, 0x00,
	0x0f64, 0xce,
	0x0f66, 0x77,
	0x0f65, 0x50,
	0x0fbe, 0xf7,
	0x0314, 0x01,
	0x0315, 0xd8,
	0x031a, 0x06,
	0x013f, 0x00,
	0x0901, 0x1f,
	0x0134, 0x54,
	0x0f62, 0x1b,
	0x031c, 0xe0,
	/*Pre_set*/
	0x0f88, 0x33,
	0x0f87, 0x8c,
	0x0f8c, 0x2f,
	0x0f8e, 0x04,
	0x0fbd, 0x04,
	0x0218, 0x00,
	0x0580, 0x00,
	0x008b, 0x08,
	0x008c, 0x00,
	0x008e, 0x30,
	0x008f, 0x00,
	0x0087, 0x50,
	0x0af4, 0x68,
	0x0510, 0x01,
	/*analog*/
	0x0f0a, 0x00,
	0x0f0b, 0x04,
	0x0f0c, 0x04,
	0x0f0d, 0x24,
	0x0faa, 0x00,
	0x0fab, 0x04,
	0x0fac, 0x04,
	0x0fad, 0x24,
	0x0346, 0x00,
	0x0347, 0x30,
	0x0348, 0x10,
	0x0349, 0x80,
	0x034a, 0x0c,
	0x034b, 0x40,
	0x0f25, 0x14,
	0x0f45, 0x14,
	0x0202, 0x0b,
	0x0203, 0x64,
	0x0342, 0x08,
	0x0343, 0x40,
	0x0340, 0x0c,
	0x0341, 0xb0,
	0x0226, 0x0c,
	0x0227, 0x60,
	0x0225, 0x04,
	0x0254, 0x40,
	0x0219, 0x44,
	0x021a, 0x17,
	0x021d, 0x02,
	0x0220, 0x04,
	0x0221, 0x02,
	/*DD*/
	0x00aa, 0x00,
	0x00ab, 0x00,
	/*PDAF*/
	0x00d3, 0x23,
	0x00ec, 0x00,
	0x00ed, 0x20,
	0x00ee, 0x00,
	0x00ef, 0x20,
	0x00f0, 0x0c,
	0x00f1, 0x20,
	0x00f2, 0x08,
	0x00f3, 0x20,
	0x00dc, 0x00,
	0x00dd, 0x20,
	0x00de, 0x00,
	0x00df, 0x20,
	0x00e0, 0x0c,
	0x00e1, 0x20,
	0x00e2, 0x08,
	0x00e3, 0x20,
	0x0145, 0x80,
	0x0147, 0xa0,
	0x0142, 0x00,
	0x0140, 0x83,
	0x0107, 0x80,
	0x0109, 0x0c,
	0x0123, 0x23,
	0x0124, 0x01,
	0x0125, 0x02,
	0x00a0, 0x07,
	0x00d2, 0x01,
	/*gain*/
	0x009a, 0x00,
	0x009b, 0xb4,
	0x0204, 0x00,
	0x0205, 0x20,
	0x0206, 0x00,
	/*out*/
	0x0351, 0x00,
	0x0352, 0x08,
	0x0353, 0x00,
	0x0354, 0x20,
	0x034c, 0x10,
	0x034d, 0x40,
	0x034e, 0x0c,
	0x034f, 0x30,
	/*MIPI*/
	0x0113, 0x02,
	0x0114, 0x02,
	0x0119, 0x04,
	0x0128, 0x2b,
	0x0182, 0x20,
	0x0181, 0x00,
	0x0184, 0x10,
	0x0902, 0x0a,
	0x0903, 0x05,
	0x0914, 0x01,
	0x0934, 0x01,
	0x0954, 0x01,
	0x0974, 0x01,
	0x0994, 0x01,
	0x096b, 0x9f,
	0x0190, 0x00,
	0x01a5, 0x0a,
	0x01a4, 0x02,
	0x01a2, 0x0b,
	0x01a3, 0x30,
	0x01a6, 0x0d,
	0x01a1, 0x0e,
	0x01a9, 0x0b,
	0x01aa, 0x12,
	0x01ab, 0x0c,
	0x01a7, 0x17,
	/*autoload*/
	0x0a70, 0x80,
	0x0a67, 0x80,
	0x0a90, 0x41,
	0x0a94, 0x80,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	/*standby*/
	0x0af6, 0x20,
	0x0b00, 0x04,
	/*auto off*/
	0x00a4, 0x80,
	0x0a90, 0x00,
	0x0a70, 0x00,
	0x0a67, 0x00,
	/*CIS rst*/
	0x031c, 0x80,
	0x03fe, 0x30,
	0x03fe, 0x00,
	0x031c, 0x9f,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0x80,
	0x03fe, 0x30,
	0x03fe, 0x00,
	0x031c, 0x9f,
	0x0083, 0x00,
};
#endif

#endif