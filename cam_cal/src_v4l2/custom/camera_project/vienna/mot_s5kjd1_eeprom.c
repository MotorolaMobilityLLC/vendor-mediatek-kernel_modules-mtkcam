// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "cam_cal_config.h"

unsigned int s5kjd1_mot_do_factory_verify(struct EEPROM_DRV_FD_DATA *pdata, unsigned int *pGetSensorCalData);

#define S5KJD1_MOT_EEPROM_ADDR 0x00
#define S5KJD1_MOT_EEPROM_DATA_SIZE 0x0F43
#define S5KJD1_MOT_SERIAL_NUMBER_ADDR 0x15
#define S5KJD1_MOT_MNF_ADDR 0x00
#define S5KJD1_MOT_MNF_DATA_SIZE 37
#define S5KJD1_MOT_MNF_CHECKSUM_ADDR 0x25
#define S5KJD1_MOT_AWB_ADDR 0x41
#define S5KJD1_MOT_AWB_DATA_SIZE 43
#define S5KJD1_MOT_AWB_CHECKSUM_ADDR 0x6C
#define S5KJD1_MOT_LSC_ADDR 0x07E1
#define S5KJD1_MOT_LSC_DATA_SIZE 1868
#define S5KJD1_MOT_LSC_CHECKSUM_ADDR 0X0F2D

static struct STRUCT_CALIBRATION_LAYOUT_STRUCT cal_layout_table = {
	0x00000003, 0x32384439, CAM_CAL_SINGLE_EEPROM_DATA,
	{
		{0x00000000, 0x00000000, 0x00000000, do_module_version},
		{0x00000000, 0x00000000, 0x00000025, do_part_number},
		{0x00000001, 0x000007E1, 0x0000074C, mot_do_single_lsc},
		{0x00000001, 0x00000041, 0x0000002B, mot_do_awb_gain},
		{0x00000000, 0x00000000, 0x00000000, mot_do_pdaf},
		{0x00000000, 0x00000000, 0x00000000, do_stereo_data},
		{0x00000000, 0x00000000, 0x00000000, do_dump_all},
		{0x00000000, 0x00000000, 0x00000000, do_lens_id},
		{0x00000000, 0x00000000, 0x00000000, NULL},
		{0x00000001, 0x00000000, 0x00000025, mot_do_manufacture_info}
	}
};

struct STRUCT_CAM_CAL_CONFIG_STRUCT mot_s5kjd1_eeprom = {
	.name = "mot_s5kjd1_eeprom",
	.sensor_type = FRONT_CAMERA,
	.check_layout_function = mot_layout_no_ck,
	.read_function = Common_read_region,
	.mot_do_factory_verify_function = s5kjd1_mot_do_factory_verify,
	.layout = &cal_layout_table,
	.sensor_id = MOT_VIENNA_S5KJD1_SENSOR_ID,
	.i2c_write_id = 0xA2,
	.max_size = 0x8000,
	.serial_number_bit = 16,
	.enable_preload = 1,
	.preload_size = S5KJD1_MOT_EEPROM_DATA_SIZE,
};

unsigned int s5kjd1_mot_do_factory_verify(struct EEPROM_DRV_FD_DATA *pdata, unsigned int *pGetSensorCalData)
{
	struct STRUCT_MOT_EEPROM_DATA *pCamCalData =
				(struct STRUCT_MOT_EEPROM_DATA *)pGetSensorCalData;
	int read_data_size, checkSum;

	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			S5KJD1_MOT_EEPROM_ADDR, S5KJD1_MOT_EEPROM_DATA_SIZE, (unsigned char *)pCamCalData->DumpAllEepromData);
	if (read_data_size <= 0) {
		return CAM_CAL_ERR_NO_DEVICE;
	}
	//mnf check
	checkSum = (pCamCalData->DumpAllEepromData[S5KJD1_MOT_MNF_CHECKSUM_ADDR])<< 8
		|(pCamCalData->DumpAllEepromData[S5KJD1_MOT_MNF_CHECKSUM_ADDR+1]);
	debug_log("checkSum  = 0x%x", checkSum);

	if(check_crc16(pCamCalData->DumpAllEepromData, S5KJD1_MOT_MNF_DATA_SIZE, checkSum)) {
		debug_log("check mnf crc16 ok");
	} else {
		debug_log("check mnf crc16 err");
		return CAM_CAL_ERR_NO_PARTNO;
	}
	//for dump serial_number
	memcpy(pCamCalData->serial_number, &pCamCalData->DumpAllEepromData[S5KJD1_MOT_SERIAL_NUMBER_ADDR],
		pCamCalData->serial_number_bit);

	//awb check
	if((mot_check_awb_data(pCamCalData->DumpAllEepromData+S5KJD1_MOT_AWB_ADDR, S5KJD1_MOT_AWB_DATA_SIZE + 2)) == NO_ERRORS) {
		debug_log("check awb data ok");
	} else {
		debug_log("check awb data err");
		return CAM_CAL_ERR_NO_3A_GAIN;
	}

	//lsc check
	checkSum = (pCamCalData->DumpAllEepromData[S5KJD1_MOT_LSC_CHECKSUM_ADDR])<< 8
		|(pCamCalData->DumpAllEepromData[S5KJD1_MOT_LSC_CHECKSUM_ADDR+1]);
	debug_log("checkSum  = 0x%x", checkSum);

	if(check_crc16(pCamCalData->DumpAllEepromData+S5KJD1_MOT_LSC_ADDR, S5KJD1_MOT_LSC_DATA_SIZE, checkSum)) {
		debug_log("check lsc crc16 ok");
	} else {
		debug_log("check lsc crc16 err");
		return CAM_CAL_ERR_NO_SHADING;
	}

	return CAM_CAL_ERR_NO_ERR;
}
