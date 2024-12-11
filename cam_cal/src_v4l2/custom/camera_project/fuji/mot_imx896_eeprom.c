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
#include "mot_cam_cal.h"

static unsigned int imx896_mot_do_factory_verify(struct EEPROM_DRV_FD_DATA *pdata, unsigned int *pGetSensorCalData);
static unsigned int mot_imx896_do_manufacture_info(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int mot_imx896_do_2a_gain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int mot_imx896_do_pdaf(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
#define IMX896_MOT_EEPROM_ADDR 0x00
#define IMX896_MOT_EEPROM_DATA_SIZE 0x325B
#define IMX896_MOT_SERIAL_NUMBER_ADDR 0x16
#define IMX896_MOT_MNF_ADDR 0x00
#define IMX896_MOT_MNF_DATA_SIZE 38
#define IMX896_MOT_MNF_CHECKSUM_ADDR 0x26
#define IMX896_MOT_AF_ADDR 0x28
#define IMX896_MOT_AF_DATA_SIZE 24
#define IMX896_MOT_AF_CHECKSUM_ADDR 0x40
#define IMX896_MOT_AWB_ADDR 0x42
#define IMX896_MOT_AWB_DATA_SIZE 43
#define IMX896_MOT_AWB_CHECKSUM_ADDR 0x6D
#define IMX896_MOT_LSC_ADDR 0x0C91
#define IMX896_MOT_LSC_DATA_SIZE 1868
#define IMX896_MOT_LSC_CHECKSUM_ADDR 0x13DD
#define IMX896_MOT_PDAF1_ADDR 0x13DF
#define IMX896_MOT_PDAF2_ADDR 0x15CF
#define IMX896_MOT_PDAF_DATA1_SIZE 496
#define IMX896_MOT_PDAF_DATA2_SIZE 1004
#define IMX896_MOT_PDAF_CHECKSUM_ADDR 0x19BB
#define IMX896_MOT_AF_SYNC_DATA_ADDR 0x19BF
#define IMX896_MOT_AF_SYNC_DATA_SIZE 24
#define IMX896_MOT_AF_SYNC_DATA_CHECKSUM_ADDR 0x19D7

#define IMX896_MOT_MTK_NECESSARY_DATA_ADDR 0x19D9
#define IMX896_MOT_MTK_NECESSARY_DATA_SIZE 19
#define IMX896_MOT_MTK_NECESSARY_DATA_CHECKSUM_ADDR 0x19EC

#define IMX896_QSC_DATA_ADDR 0x19EE
#define IMX896_QSC_DATA_SIZE 3072
#define IMX896_QSC_CRC_ADDR 0x25EE

#define IMX896_SPC_DATA_ADDR 0x25F0
#define IMX896_SPC_DATA_SIZE 384
#define IMX896_SPC_CRC_ADDR 0x2770

#define IMX896_FULL_PD_PROC1_ADDR 0x13DF
#define IMX896_FULL_PD_PROC1_SIZE 496
#define IMX896_FULL_PD_PROC2_ADDR 0x15CF
#define IMX896_FULL_PD_PROC2_SIZE 1004
#define IMX896_FULL_PD_SIZE (IMX896_FULL_PD_PROC1_SIZE+IMX896_FULL_PD_PROC2_SIZE)

#define IMX896_PARTIAL_PD_PROC1_ADDR 0x2C39
#define IMX896_PARTIAL_PD_PROC1_SIZE 496
#define IMX896_PARTIAL_PD_PROC2_ADDR 0x2E29
#define IMX896_PARTIAL_PD_PROC2_SIZE 1004
#define IMX896_PARTIAL_PD_SIZE (IMX896_PARTIAL_PD_PROC1_SIZE+IMX896_PARTIAL_PD_PROC2_SIZE)

static struct STRUCT_CALIBRATION_LAYOUT_STRUCT cal_layout_table = {
	0x00000003, 0x32443832, CAM_CAL_SINGLE_EEPROM_DATA,
	{
		{0x00000000, 0x00000000, 0x00000000, do_module_version},//CAMERA_CAM_CAL_DATA_MODULE_VERSION
		{0x00000000, 0x00000000, 0x00000025, do_part_number},//CAMERA_CAM_CAL_DATA_PART_NUMBER
		{0x00000001, IMX896_MOT_LSC_ADDR, IMX896_MOT_LSC_DATA_SIZE, mot_do_single_lsc},//CAMERA_CAM_CAL_DATA_SHADING_TABLE
		{0x00000001, 0x00000027, 0x00000045, mot_imx896_do_2a_gain},//CAMERA_CAM_CAL_DATA_3A_GAIN
		{0x00000001, 0x00000000, 0x00000000, mot_imx896_do_pdaf},//CAMERA_CAM_CAL_DATA_PDAF
		{0x00000000, 0x00000FAE, 0x00000550, do_stereo_data},//CAMERA_CAM_CAL_DATA_STEREO_DATA
		{0x00000000, 0x00000000, 0x00000E25, do_dump_all},//CAMERA_CAM_CAL_DATA_DUMP
		{0x00000000, 0x00000F80, 0x0000000A, do_lens_id},//CAMERA_CAM_CAL_DATA_LENS_ID
		{0x00000000, 0x00000000, 0x00000000, NULL},//CAMERA_CAM_CAL_DATA_SHADING_TABLE_16_9
		{0x00000001, IMX896_MOT_MNF_ADDR, IMX896_MOT_MNF_DATA_SIZE, mot_imx896_do_manufacture_info}//CAMERA_CAM_CAL_DATA_MANUFACTURE
	}
};

struct STRUCT_CAM_CAL_CONFIG_STRUCT mot_imx896_eeprom = {
	.name = "mot_imx896_eeprom",
	.sensor_type = MAIN_CAMERA,
	.check_layout_function = mot_layout_no_ck,
	.read_function = Common_read_region,
	.mot_do_factory_verify_function = imx896_mot_do_factory_verify,
	.layout = &cal_layout_table,
	.sensor_id = MOT_FUJI_IMX896_SENSOR_ID,
	.i2c_write_id = 0xA0,
	.max_size = 0x8000,
	.serial_number_bit = 16,
	.enable_preload = 1,
	.preload_size = IMX896_MOT_EEPROM_DATA_SIZE,
};

static unsigned int mot_imx896_do_pdaf(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;

	int read_data_size, checkSum1, checkSum2;
	int err =  CamCalReturnErr[pCamCalData->Command];
	uint8_t  tempBuf[1504] = {0};

	//Partial PD calibration data
	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID, IMX896_PARTIAL_PD_PROC1_ADDR,
	                           IMX896_PARTIAL_PD_SIZE + 4, (unsigned char *)tempBuf);
	if (read_data_size <= 0) {
		err = CAM_CAL_ERR_NO_PDAF;
		return err;
	}
	checkSum1 = tempBuf[1500] << 8 | tempBuf[1501];
	checkSum2 = tempBuf[1502] << 8 | tempBuf[1503];
	debug_log("checkSum1  = 0x%x, checkSum2 = 0x%x", checkSum1, checkSum2);

	if(check_crc16(tempBuf, 496, checkSum1) && check_crc16(tempBuf +496, 1004, checkSum2)) {
		debug_log("partial PD calibration check_crc16 ok");
		err = CAM_CAL_ERR_NO_ERR;
	} else {
		debug_log("partial PD calibration check_crc16 err");
		err = CAM_CAL_ERR_NO_PDAF;
		return err;
	}

	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID, IMX896_PARTIAL_PD_PROC1_ADDR,
	                           IMX896_PARTIAL_PD_SIZE, (unsigned char *)&pCamCalData->PDAF.Data[0]);
	if (read_data_size <= 0) {
		err = CAM_CAL_ERR_NO_PDAF;
		return err;
	}

	//Full PD calibration data
	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID, IMX896_FULL_PD_PROC1_ADDR,
	                           IMX896_FULL_PD_SIZE + 4, (unsigned char *)tempBuf);
	if (read_data_size <= 0) {
		err = CAM_CAL_ERR_NO_PDAF;
		return err;
	}
	checkSum1 = tempBuf[1500] << 8 | tempBuf[1501];
	checkSum2 = tempBuf[1502] << 8 | tempBuf[1503];
	debug_log("checkSum1  = 0x%x, checkSum2 = 0x%x", checkSum1, checkSum2);

	if(check_crc16(tempBuf, 496, checkSum1) && check_crc16(tempBuf +496, 1004, checkSum2)) {
		debug_log("full PD calibration check_crc16 ok");
		err = CAM_CAL_ERR_NO_ERR;
	} else {
		debug_log("full PD calibration check_crc16 err");
		err = CAM_CAL_ERR_NO_PDAF;
		return err;
	}

	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID, IMX896_FULL_PD_PROC1_ADDR,
	                           IMX896_FULL_PD_SIZE,
	                           (unsigned char *)&pCamCalData->PDAF.Data[IMX896_PARTIAL_PD_SIZE]);
	if (read_data_size <= 0) {
		err = CAM_CAL_ERR_NO_PDAF;
		return err;
	}

	pCamCalData->PDAF.Size_of_PDAF = IMX896_PARTIAL_PD_SIZE  + IMX896_FULL_PD_SIZE;
	debug_log("PDAF total size=%d\n", block_size);

	debug_log("======================PDAF Data==================\n");
	debug_log("First five of partial pd: %x, %x, %x, %x, %x\n",
		pCamCalData->PDAF.Data[0],
		pCamCalData->PDAF.Data[1],
		pCamCalData->PDAF.Data[2],
		pCamCalData->PDAF.Data[3],
		pCamCalData->PDAF.Data[4]);
	debug_log("First five of full pd: %x, %x, %x, %x, %x\n",
		pCamCalData->PDAF.Data[IMX896_PARTIAL_PD_SIZE],
		pCamCalData->PDAF.Data[IMX896_PARTIAL_PD_SIZE+1],
		pCamCalData->PDAF.Data[IMX896_PARTIAL_PD_SIZE+2],
		pCamCalData->PDAF.Data[IMX896_PARTIAL_PD_SIZE+3],
		pCamCalData->PDAF.Data[IMX896_PARTIAL_PD_SIZE+4]);
	debug_log("RETURN = 0x%x\n", err);
	debug_log("======================PDAF Data==================\n");

	return err;

}

unsigned int mot_imx896_do_2a_gain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
	int read_data_size, checkSum;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];

	unsigned char AWBAFConfig = 0x3; //set af awb enable
	unsigned short AFInf, AFMacro, AFInfDistance, AFMacroDistance;
	int RGBGratioDeviation, tempMax = 0;
	int CalR = 1, CalGr = 1, CalGb = 1, CalG = 1, CalB = 1;
	int FacR = 1, FacGr = 1, FacGb = 1, FacG = 1, FacB = 1;
	int RawCalR = 1, RawCalGr = 1, RawCalGb = 1, RawCalB = 1;
	int RawFacR = 1, RawFacGr = 1, RawFacGb = 1, RawFacB = 1;
	int rg_ratio_gold = 1, bg_ratio_gold = 1, grgb_ratio_gold = 1;
	int rg_ratio_unit = 1, bg_ratio_unit = 1, grgb_ratio_unit = 1;
	uint8_t  af_data[26] = {0};
	uint8_t  awb_data[45] = {0};
	int af_addr = IMX896_MOT_AF_ADDR;
	int af_size = IMX896_MOT_AF_DATA_SIZE;
	int awb_addr = IMX896_MOT_AWB_ADDR;
	int awb_size = IMX896_MOT_AWB_DATA_SIZE;

	debug_log("block_size=%d sensor_id=%x\n", block_size, pCamCalData->sensorID);

	memset((void *)&pCamCalData->Single2A, 0, sizeof(struct STRUCT_CAM_CAL_SINGLE_2A_STRUCT));

	if (pCamCalData->DataVer >= CAM_CAL_TYPE_NUM) {
		err = CAM_CAL_ERR_NO_DEVICE;
		error_log("Read Failed\n");
		show_cmd_error_log(pCamCalData->Command);
		return err;
	}
	if (block_size == 0) {
		error_log("block_size(%d) is not correct\n", block_size);
		show_cmd_error_log(pCamCalData->Command);
		return err;
	}

	pCamCalData->Single2A.S2aVer = 0x01;
	pCamCalData->Single2A.S2aBitEn = (0x03 & AWBAFConfig);
	pCamCalData->Single2A.S2aAfBitflagEn = (0x0c & AWBAFConfig);

	/* AWB Calibration Data*/
	if (0x1 & AWBAFConfig) {
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_addr, awb_size + 2, (unsigned char *)awb_data);
		if (read_data_size > 0)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}

		checkSum = awb_data[43]<<8 | awb_data[44];
		RGBGratioDeviation = awb_data[6];
		debug_log("RGBGratioDeviation = %d", RGBGratioDeviation);
		if(check_crc16(awb_data, 43, checkSum)) {
			debug_log("WBC check_crc16 ok");
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			debug_log("WBC check_crc16 err");
			err = CAM_CAL_ERR_NO_3A_GAIN;
			return err;
		}
		//check ratio limt
		rg_ratio_unit = (awb_data[32]<<8 | awb_data[33])*1000/16384;
		bg_ratio_unit = (awb_data[34]<<8 | awb_data[35])*1000/16384;
		grgb_ratio_unit = (awb_data[36]<<8 | awb_data[37])*1000/16384;
		rg_ratio_gold = (awb_data[18]<<8 | awb_data[19])*1000/16384;
		bg_ratio_gold = (awb_data[20]<<8 | awb_data[21])*1000/16384;
		grgb_ratio_gold = (awb_data[22]<<8 | awb_data[23])*1000/16384;
		debug_log("ratio*1000, Unit R/G = %d, B/G = %d, Gr/Gb = %d, Gold R/G = %d, B/G = %d, Gr/Gb = %d",
			rg_ratio_unit, bg_ratio_unit, grgb_ratio_unit, rg_ratio_gold, bg_ratio_gold, grgb_ratio_gold);
		if(grgb_ratio_unit<MOT_AWB_GRGB_RATIO_MIN_1000TIMES || grgb_ratio_unit>MOT_AWB_GRGB_RATIO_MAX_1000TIMES
			|| grgb_ratio_gold<MOT_AWB_GRGB_RATIO_MIN_1000TIMES || grgb_ratio_gold>MOT_AWB_GRGB_RATIO_MAX_1000TIMES
			|| (ABS(rg_ratio_unit, rg_ratio_gold))>RGBGratioDeviation
			|| (ABS(bg_ratio_unit, bg_ratio_gold))>RGBGratioDeviation) {
			debug_log("ratio check err");
			err = CAM_CAL_ERR_NO_3A_GAIN;
			return err;
		}

		RawCalR  = (awb_data[24]<<8 | awb_data[25]);
		RawCalGr = (awb_data[26]<<8 | awb_data[27]);
		RawCalGb = (awb_data[28]<<8 | awb_data[29]);
		RawCalB  = (awb_data[30]<<8 | awb_data[31]);

		CalR  = RawCalR / MOTO_WB_VALUE_BASE;
		CalGr = RawCalGr / MOTO_WB_VALUE_BASE;
		CalGb = RawCalGb / MOTO_WB_VALUE_BASE;
		CalB  = RawCalB / MOTO_WB_VALUE_BASE;

		if(CalR<MOT_AWB_RB_MIN_VALUE || CalR>MOT_AWB_RBG_MAX_VALUE
			|| CalGr<MOT_AWB_G_MIN_VALUE ||CalGr>MOT_AWB_RBG_MAX_VALUE
			|| CalGb<MOT_AWB_G_MIN_VALUE ||CalGb>MOT_AWB_RBG_MAX_VALUE
			|| CalB<MOT_AWB_RB_MIN_VALUE || CalB>MOT_AWB_RBG_MAX_VALUE) {
			debug_log("check unit R Gr Gb B limit error");
			err = CAM_CAL_ERR_NO_3A_GAIN;
			return err;
		}

		//Let's use EEPROM programmed values instead actual values to improve accuracy here
#ifdef MOTO_OB_VALUE
		CalR  = RawCalR - MOTO_OB_VALUE * MOTO_WB_VALUE_BASE;
		CalGr = RawCalGr - MOTO_OB_VALUE * MOTO_WB_VALUE_BASE;
		CalGb = RawCalGb - MOTO_OB_VALUE * MOTO_WB_VALUE_BASE;
		CalB  = RawCalB - MOTO_OB_VALUE * MOTO_WB_VALUE_BASE;
#else
		CalR  = RawCalR;
		CalGr = RawCalGr;
		CalGb = RawCalGb;
		CalB  = RawCalB;
#endif

		CalG = ((CalGr + CalGb) + 1) >> 1;

		debug_log("Unit R = %d, Gr= %d, Gb = %d, B = %d, G = %d", CalR/MOTO_WB_VALUE_BASE,
		          CalGr/MOTO_WB_VALUE_BASE, CalGb/MOTO_WB_VALUE_BASE, CalB/MOTO_WB_VALUE_BASE, CalG/MOTO_WB_VALUE_BASE);

		tempMax = MAX_temp(CalR,CalG,CalB);

		pCamCalData->Single2A.S2aAwb.rUnitGainu4R = (u32)((tempMax*512 + (CalR >> 1))/CalR);
		pCamCalData->Single2A.S2aAwb.rUnitGainu4G = (u32)((tempMax*512 + (CalG >> 1))/CalG);
		pCamCalData->Single2A.S2aAwb.rUnitGainu4B  = (u32)((tempMax*512 + (CalB >> 1))/CalB);

		RawFacR  = (awb_data[10]<<8 | awb_data[11]);
		RawFacGr = (awb_data[12]<<8 | awb_data[13]);
		RawFacGb = (awb_data[14]<<8 | awb_data[15]);
		RawFacB  = (awb_data[16]<<8 | awb_data[17]);

		FacR  = RawFacR / MOTO_WB_VALUE_BASE;
		FacGr = RawFacGr / MOTO_WB_VALUE_BASE;
		FacGb = RawFacGb / MOTO_WB_VALUE_BASE;
		FacB  = RawFacB / MOTO_WB_VALUE_BASE;

		if(FacR<MOT_AWB_RB_MIN_VALUE || FacR>MOT_AWB_RBG_MAX_VALUE
			|| FacGr<MOT_AWB_G_MIN_VALUE ||FacGr>MOT_AWB_RBG_MAX_VALUE
			|| FacGb<MOT_AWB_G_MIN_VALUE ||FacGb>MOT_AWB_RBG_MAX_VALUE
			|| FacB<MOT_AWB_RB_MIN_VALUE || FacB>MOT_AWB_RBG_MAX_VALUE) {
			debug_log("check gold R Gr Gb B limit error");
			err = CAM_CAL_ERR_NO_3A_GAIN;
			return err;
		}

		//Let's use EEPROM programmed values instead actual values to improve accuracy here
#ifdef MOTO_OB_VALUE
		FacR  = RawFacR - MOTO_OB_VALUE * MOTO_WB_VALUE_BASE;
		FacGr = RawFacGr - MOTO_OB_VALUE * MOTO_WB_VALUE_BASE;
		FacGb = RawFacGb - MOTO_OB_VALUE * MOTO_WB_VALUE_BASE;
		FacB  = RawFacB - MOTO_OB_VALUE * MOTO_WB_VALUE_BASE;
#else
		FacR  = RawFacR;
		FacGr = RawFacGr;
		FacGb = RawFacGb;
		FacB  = RawFacB;
#endif

		FacG = ((FacGr + FacGb) + 1) >> 1;

		debug_log("Gold R = %d, Gr= %d, Gb = %d, B = %d, G = %d", FacR/MOTO_WB_VALUE_BASE,
		          FacGr/MOTO_WB_VALUE_BASE, FacGb/MOTO_WB_VALUE_BASE, FacB/MOTO_WB_VALUE_BASE, FacG/MOTO_WB_VALUE_BASE);

		tempMax = MAX_temp(FacR,FacG,FacB);

		pCamCalData->Single2A.S2aAwb.rGoldGainu4R = (u32)((tempMax * 512 + (FacR >> 1)) /FacR);
		pCamCalData->Single2A.S2aAwb.rGoldGainu4G = (u32)((tempMax * 512 + (FacG >> 1)) /FacG);
		pCamCalData->Single2A.S2aAwb.rGoldGainu4B  = (u32)((tempMax * 512 + (FacB >> 1)) /FacB);

		pCamCalData->Single2A.S2aAwb.rValueR   = CalR/MOTO_WB_VALUE_BASE;
		pCamCalData->Single2A.S2aAwb.rValueGr  = CalGr/MOTO_WB_VALUE_BASE;
		pCamCalData->Single2A.S2aAwb.rValueGb  = CalGb/MOTO_WB_VALUE_BASE;
		pCamCalData->Single2A.S2aAwb.rValueB   = CalB/MOTO_WB_VALUE_BASE;
		pCamCalData->Single2A.S2aAwb.rGoldenR  = FacR/MOTO_WB_VALUE_BASE;
		pCamCalData->Single2A.S2aAwb.rGoldenGr = FacGr/MOTO_WB_VALUE_BASE;
		pCamCalData->Single2A.S2aAwb.rGoldenGb = FacGb/MOTO_WB_VALUE_BASE;
		pCamCalData->Single2A.S2aAwb.rGoldenB  = FacB/MOTO_WB_VALUE_BASE;
		debug_log("======================AWB CAM_CAL==================\n");
		debug_log("[rCalGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rValueR);
		debug_log("[rCalGain.u4Gr] = %d\n", pCamCalData->Single2A.S2aAwb.rValueGr);
		debug_log("[rCalGain.u4Gb] = %d\n", pCamCalData->Single2A.S2aAwb.rValueGb);
		debug_log("[rCalGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rValueB);
		debug_log("[rFacGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldenR);
		debug_log("[rFacGain.u4Gr] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldenGr);
		debug_log("[rFacGain.u4Gb] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldenGb);
		debug_log("[rFacGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldenB);
		debug_log("[rCalGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4R);
		debug_log("[rCalGain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4G);
		debug_log("[rCalGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4B);
		debug_log("[rFacGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4R);
		debug_log("[rFacGain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4G);
		debug_log("[rFacGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4B);
		debug_log("======================AWB CAM_CAL==================\n");

	}
	/* AF Calibration Data*/
	if (0x2 & AWBAFConfig) {
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				af_addr, af_size + 2, (unsigned char *)af_data);
		if (read_data_size > 0)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}

		checkSum = af_data[24]<<8 | af_data[25];
		if(check_crc16(af_data, 24, checkSum)) {
			debug_log("AFC check_crc16 ok");
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			debug_log("AFC check_crc16 err");
			err = CAM_CAL_ERR_NO_3A_GAIN;
			return err;
		}
		AFMacro = (af_data[2]<<8 | af_data[3])/64;
		AFInf = (af_data[6]<<8 | af_data[7])/64;
		AFInfDistance = af_data[4]<<8 | af_data[5];
		AFMacroDistance = af_data[0]<<8 | af_data[1];
		pCamCalData->Single2A.S2aAf[0] = AFInf;
		pCamCalData->Single2A.S2aAf[1] = AFMacro;
		pCamCalData->Single2A.S2aAF_t.AF_infinite_pattern_distance = AFInfDistance;
		pCamCalData->Single2A.S2aAF_t.AF_Macro_pattern_distance = AFMacroDistance;

		debug_log("======================AF CAM_CAL==================\n");
		debug_log("[AFInfDistance] = %dmm\n", AFInfDistance);
		debug_log("[AFMacroDistance] = %dmm\n", AFMacroDistance);
		debug_log("[AFInf] = %d\n", AFInf);
		debug_log("[AFMacro] = %d\n", AFMacro);
		debug_log("======================AF CAM_CAL==================\n");
	}

	if(pCamCalData->sensorID == MOT_VIENNA_IMX896_SENSOR_ID) {
		#define AF_POSTURE_LEN 21
		#define AF_POSTURE_START 0x19D9
		//af posture calibration data
		unsigned char AF_POSTURE[AF_POSTURE_LEN];
		unsigned int af_posture_data_offset = AF_POSTURE_START;
		unsigned int af_inf_posture, af_macro_posture, AF_infinite_calibration_temperature;
		memset(AF_POSTURE, 0, AF_POSTURE_LEN);
		debug_log("af_posture_data_offset = 0x%x\n", af_posture_data_offset);

		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				af_posture_data_offset, AF_POSTURE_LEN, (unsigned char *) AF_POSTURE);
		if (read_data_size > 0)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}

		checkSum = AF_POSTURE[AF_POSTURE_LEN-2]<<8 | AF_POSTURE[AF_POSTURE_LEN-1];
		if(check_crc16(AF_POSTURE, AF_POSTURE_LEN-2, checkSum)) {
			debug_log("AF pos comp check_crc16 ok");
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			debug_log("AF pos comp check_crc16 err");
			err = CAM_CAL_NONE_BITEN;
			return err;
		}

		//Vienna's IMX896 AF is close loop driver IC, AF drift is 0, the programmed values is 0xFFFF.
		af_inf_posture = 0;
		af_macro_posture = 0;
		AF_infinite_calibration_temperature = AF_POSTURE[17];
		pCamCalData->Single2A.S2aAF_t.Posture_AF_infinite_calibration = af_inf_posture;
		pCamCalData->Single2A.S2aAF_t.Posture_AF_macro_calibration = af_macro_posture;
		pCamCalData->Single2A.S2aAF_t.AF_infinite_calibration_temperature = AF_infinite_calibration_temperature;

		debug_log("======================AF POSTURE CAM_CAL==================\n");
		debug_log("[AFInfPosture] = 0x%x\n", af_inf_posture);
		debug_log("[AFMacroPosture] = 0x%x\n", af_macro_posture);
		debug_log("[AFInfiniteCalibrationTemperature] = 0x%x\n", AF_infinite_calibration_temperature);
		debug_log("======================AF POSTURE CAM_CAL==================\n");
	}
	return err;
}

static unsigned int imx896_mot_do_factory_verify(struct EEPROM_DRV_FD_DATA *pdata, unsigned int *pGetSensorCalData)
{
	struct STRUCT_MOT_EEPROM_DATA *pCamCalData =
				(struct STRUCT_MOT_EEPROM_DATA *)pGetSensorCalData;
	int read_data_size, checkSum, checkSum1;
        debug_log("imx896_mot_do_factory_verify");
	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			IMX896_MOT_EEPROM_ADDR, IMX896_MOT_EEPROM_DATA_SIZE, (unsigned char *)pCamCalData->DumpAllEepromData);
	if (read_data_size <= 0) {
		return CAM_CAL_ERR_NO_DEVICE;
	}
	//mnf check
	checkSum = (pCamCalData->DumpAllEepromData[IMX896_MOT_MNF_CHECKSUM_ADDR])<< 8
		|(pCamCalData->DumpAllEepromData[IMX896_MOT_MNF_CHECKSUM_ADDR+1]);
	debug_log("checkSum  = 0x%x", checkSum);

	if(check_crc16(pCamCalData->DumpAllEepromData, IMX896_MOT_MNF_DATA_SIZE, checkSum)) {
		debug_log("check mnf crc16 ok");
	} else {
		debug_log("check mnf crc16 err");
		return CAM_CAL_ERR_NO_PARTNO;
	}
	//for dump serial_number
	memcpy(pCamCalData->serial_number, &pCamCalData->DumpAllEepromData[IMX896_MOT_SERIAL_NUMBER_ADDR],
		pCamCalData->serial_number_bit);

	//af check
	checkSum = (pCamCalData->DumpAllEepromData[IMX896_MOT_AF_CHECKSUM_ADDR])<< 8
		|(pCamCalData->DumpAllEepromData[IMX896_MOT_AF_CHECKSUM_ADDR+1]);
	debug_log("checkSum  = 0x%x", checkSum);

	if(check_crc16(pCamCalData->DumpAllEepromData+IMX896_MOT_AF_ADDR, IMX896_MOT_AF_DATA_SIZE, checkSum)) {
		debug_log("check af crc16 ok");
	} else {
		debug_log("check af crc16 err");
		return CAM_CAL_ERR_NO_3A_GAIN;
	}

	//awb check
	if((mot_check_awb_data(pCamCalData->DumpAllEepromData+IMX896_MOT_AWB_ADDR, IMX896_MOT_AWB_DATA_SIZE + 2)) == NO_ERRORS) {
		debug_log("check awb data ok");
	} else {
		debug_log("check awb data err");
		return CAM_CAL_ERR_NO_3A_GAIN;
	}

	//lsc check
	checkSum = (pCamCalData->DumpAllEepromData[IMX896_MOT_LSC_CHECKSUM_ADDR])<< 8
		|(pCamCalData->DumpAllEepromData[IMX896_MOT_LSC_CHECKSUM_ADDR+1]);
	debug_log("checkSum  = 0x%x", checkSum);

	if(check_crc16(pCamCalData->DumpAllEepromData+IMX896_MOT_LSC_ADDR, IMX896_MOT_LSC_DATA_SIZE, checkSum)) {
		debug_log("check lsc crc16 ok");
	} else {
		debug_log("check lsc crc16 err");
		return CAM_CAL_ERR_NO_SHADING;
	}

	//pdaf check
	checkSum = (pCamCalData->DumpAllEepromData[IMX896_MOT_PDAF_CHECKSUM_ADDR])<< 8
		|(pCamCalData->DumpAllEepromData[IMX896_MOT_PDAF_CHECKSUM_ADDR+1]);
	checkSum1 = (pCamCalData->DumpAllEepromData[IMX896_MOT_PDAF_CHECKSUM_ADDR+2])<< 8
		|(pCamCalData->DumpAllEepromData[IMX896_MOT_PDAF_CHECKSUM_ADDR+3]);
	debug_log("checkSum  = 0x%x, checkSum1 = 0x%x", checkSum, checkSum1);

	if(check_crc16(pCamCalData->DumpAllEepromData+IMX896_MOT_PDAF1_ADDR, IMX896_MOT_PDAF_DATA1_SIZE, checkSum)
		&& check_crc16(pCamCalData->DumpAllEepromData+IMX896_MOT_PDAF2_ADDR, IMX896_MOT_PDAF_DATA2_SIZE, checkSum1)) {
		debug_log("check pdaf crc16 ok");
	} else {
		debug_log("check pdaf crc16 err");
		return CAM_CAL_ERR_NO_PDAF;
	}

	//af sync data check
	checkSum = (pCamCalData->DumpAllEepromData[IMX896_MOT_AF_SYNC_DATA_CHECKSUM_ADDR])<< 8
		|(pCamCalData->DumpAllEepromData[IMX896_MOT_AF_SYNC_DATA_CHECKSUM_ADDR+1]);
	debug_log("checkSum  = 0x%x", checkSum);
	if(check_crc16(pCamCalData->DumpAllEepromData+IMX896_MOT_AF_SYNC_DATA_ADDR, IMX896_MOT_AF_SYNC_DATA_SIZE, checkSum)) {
		debug_log("check af sync data crc16 ok");
	} else {
		debug_log("check af sync data crc16 err");
		return CAM_CAL_ERR_NO_3A_GAIN;
	}

	//mtk necessary data check
	checkSum = (pCamCalData->DumpAllEepromData[IMX896_MOT_MTK_NECESSARY_DATA_CHECKSUM_ADDR])<< 8
		|(pCamCalData->DumpAllEepromData[IMX896_MOT_MTK_NECESSARY_DATA_CHECKSUM_ADDR+1]);
	debug_log("checkSum  = 0x%x", checkSum);
	if(check_crc16(pCamCalData->DumpAllEepromData+IMX896_MOT_MTK_NECESSARY_DATA_ADDR, IMX896_MOT_MTK_NECESSARY_DATA_SIZE, checkSum)) {
		debug_log("check necessary data crc16 ok");
	} else {
		debug_log("check necessary data crc16 err");
		return CAM_CAL_ERR_NO_PARTNO;
	}

	//imx896 qsc data check
	checkSum = (pCamCalData->DumpAllEepromData[IMX896_QSC_CRC_ADDR])<< 8
		|(pCamCalData->DumpAllEepromData[IMX896_QSC_CRC_ADDR+1]);
	debug_log("checkSum  = 0x%x", checkSum);
	if(check_crc16(pCamCalData->DumpAllEepromData+IMX896_QSC_DATA_ADDR, IMX896_QSC_DATA_SIZE, checkSum)) {
		debug_log("check QSC data crc16 ok");
	} else {
		debug_log("check QSC data crc16 err");
		return CAM_CAL_ERR_NO_PARTNO;
	}

	//imx896 spc data check
	checkSum = (pCamCalData->DumpAllEepromData[IMX896_SPC_CRC_ADDR])<< 8
		|(pCamCalData->DumpAllEepromData[IMX896_SPC_CRC_ADDR+1]);
	debug_log("checkSum  = 0x%x", checkSum);
	if(check_crc16(pCamCalData->DumpAllEepromData+IMX896_SPC_DATA_ADDR, IMX896_SPC_DATA_SIZE, checkSum)) {
		debug_log("check SPC data crc16 ok");
	} else {
		debug_log("check SPC data crc16 err");
		return CAM_CAL_ERR_NO_PARTNO;
	}

	return CAM_CAL_ERR_NO_ERR;
}



static unsigned imx896_lens_id_to_name(uint8_t id,unsigned int block_size,unsigned char * lens_id,unsigned int err)
{
    int i=0,flag=0,ret=0;

    const struct STRUCT_MOT_LENS__ID lens_table[] =
    {
	{0x21,"38134A-400"},
	{0x22,"39411A-400"},
	{0x80,"505265A02"},
	{0x24,"39374A-400"},
	{0x65,"Sunny39733A-400"},
    };

    for (i =0;i <  sizeof(lens_table)/sizeof(struct STRUCT_MOT_LENS__ID);i++)
   {
      if(id == lens_table[i].lens_id)
      {
          ret = snprintf(lens_id, MAX_CALIBRATION_STRING, lens_table[i].lens_name);
          flag = 1;
          break;
      }
   }

   if (flag == 0)
   {
       ret = snprintf(lens_id, MAX_CALIBRATION_STRING, "Unknow");
   }

   if (ret < 0 || ret >= block_size)
   {
	debug_log("snprintf of mnf->lens_id failed");
	memset(lens_id, 0,MAX_CALIBRATION_STRING);
	err = CAM_CAL_ERR_NO_PARTNO;
   }
   return err;
}

static unsigned int mot_imx896_do_manufacture_info(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];
	int read_data_size, checkSum, ret;
	uint8_t  tempBuf[40] = {0};

	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			start_addr, block_size + 2, (unsigned char *)tempBuf);
	if (read_data_size <= 0) {
		debug_log("read data failed");
		err = CAM_CAL_ERR_NO_PARTNO;
		return err;
	}

	checkSum = (tempBuf[38]<<8) | (tempBuf[39]);

	if(check_crc16(tempBuf, 38, checkSum)) {
		debug_log("check_crc16 ok");
		err = CAM_CAL_ERR_NO_ERR;
	} else {
		debug_log("check_crc16 err");
		err = CAM_CAL_ERR_NO_PARTNO;
		return err;
	}
	 //eeprom_table_version
	ret = snprintf(pCamCalData->ManufactureData.eeprom_table_version, MAX_CALIBRATION_STRING, "0x%x", tempBuf[0]);

	if (ret < 0 || ret >= block_size) {
		debug_log("snprintf of mnf->eeprom_table_version failed");
		memset(pCamCalData->ManufactureData.eeprom_table_version, 0,
			sizeof(pCamCalData->ManufactureData.eeprom_table_version));
		err = CAM_CAL_ERR_NO_PARTNO;
	}

	//part_number
	ret = snprintf(pCamCalData->ManufactureData.part_number, MAX_CALIBRATION_STRING, "%c%c%c%c%c%c%c%c",
		tempBuf[3], tempBuf[4], tempBuf[5], tempBuf[6],
		tempBuf[7], tempBuf[8], tempBuf[9], tempBuf[10]);

	if (ret < 0 || ret >= block_size) {
		debug_log("snprintf of mnf->mot_part_number failed");
		memset(pCamCalData->ManufactureData.part_number, 0,
			sizeof(pCamCalData->ManufactureData.part_number));
		err = CAM_CAL_ERR_NO_PARTNO;
	}

	//actuator_id
	ret = snprintf(pCamCalData->ManufactureData.actuator_id, MAX_CALIBRATION_STRING, "0x%x", tempBuf[12]);

	if (ret < 0 || ret >= block_size) {
		debug_log("snprintf of mnf->actuator_id failed");
		memset(pCamCalData->ManufactureData.actuator_id, 0,
			sizeof(pCamCalData->ManufactureData.actuator_id));
		err = CAM_CAL_ERR_NO_PARTNO;
	}
	//lens_id
        err = imx896_lens_id_to_name(tempBuf[13],block_size,pCamCalData->ManufactureData.lens_id,err);
	//manufacture id
	if(tempBuf[14] == 'T' && tempBuf[15] == 'S') {
		ret = snprintf(pCamCalData->ManufactureData.manufacturer_id, MAX_CALIBRATION_STRING, "Tianshi");
	} else if(tempBuf[14] == 'S' && tempBuf[15] == 'U') {
		ret = snprintf(pCamCalData->ManufactureData.manufacturer_id, MAX_CALIBRATION_STRING, "Sunny");
	} else if(tempBuf[14] == 'Q' && tempBuf[15] == 'T') {
		ret = snprintf(pCamCalData->ManufactureData.manufacturer_id, MAX_CALIBRATION_STRING, "Qtech");
	} else if(tempBuf[14] == 'S' && tempBuf[15] == 'W') {
		ret = snprintf(pCamCalData->ManufactureData.manufacturer_id, MAX_CALIBRATION_STRING, "Sunwin");
	} else if(tempBuf[14] == 'O' && tempBuf[15] == 'F') {
		ret = snprintf(pCamCalData->ManufactureData.manufacturer_id, MAX_CALIBRATION_STRING, "Ofilm");
	} else {
		ret = snprintf(pCamCalData->ManufactureData.manufacturer_id, MAX_CALIBRATION_STRING, "Unknow");
	}

	if (ret < 0 || ret >= block_size) {
		debug_log("snprintf of mnf->manufacturer_id failed");
		memset(pCamCalData->ManufactureData.manufacturer_id, 0,
			sizeof(pCamCalData->ManufactureData.manufacturer_id));
		err = CAM_CAL_ERR_NO_PARTNO;
	}

	//factory_id
	ret = snprintf(pCamCalData->ManufactureData.factory_id, MAX_CALIBRATION_STRING, "%c%c", tempBuf[16], tempBuf[17]);

	if (ret < 0 || ret >= block_size) {
		debug_log("snprintf of mnf->factory_id failed");
		memset(pCamCalData->ManufactureData.factory_id, 0,
			sizeof(pCamCalData->ManufactureData.factory_id));
		err = CAM_CAL_ERR_NO_PARTNO;
	}

	//manufacture_line
	ret = snprintf(pCamCalData->ManufactureData.manufacture_line, MAX_CALIBRATION_STRING, "0x%x", tempBuf[18]);

	if (ret < 0 || ret >= block_size) {
		debug_log("snprintf of mnf->manufacture_line failed");
		memset(pCamCalData->ManufactureData.manufacture_line, 0,
			sizeof(pCamCalData->ManufactureData.manufacture_line));
		err = CAM_CAL_ERR_NO_PARTNO;
	}

	//manufacture_date
	ret = snprintf(pCamCalData->ManufactureData.manufacture_date, MAX_CALIBRATION_STRING, "20%02u%02u%02u",
		tempBuf[19], tempBuf[20], tempBuf[21]);

	if (ret < 0 || ret >= block_size) {
		debug_log("snprintf of mnf->manufacture_date failed");
		memset(pCamCalData->ManufactureData.manufacture_date, 0,
			sizeof(pCamCalData->ManufactureData.manufacture_date));
		err = CAM_CAL_ERR_NO_PARTNO;
	}

	//serial_number
	ret = snprintf(pCamCalData->ManufactureData.serial_number, MAX_CALIBRATION_STRING,
		"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
		tempBuf[22], tempBuf[23], tempBuf[24], tempBuf[25], tempBuf[26], tempBuf[27], tempBuf[28], tempBuf[29],
		tempBuf[30], tempBuf[31], tempBuf[32], tempBuf[33], tempBuf[34], tempBuf[35], tempBuf[36], tempBuf[37]);

	if (ret < 0 || ret >= block_size) {
		debug_log("snprintf of mnf->serial_number failed");
		memset(pCamCalData->ManufactureData.serial_number, 0,
			sizeof(pCamCalData->ManufactureData.serial_number));
		err = CAM_CAL_ERR_NO_PARTNO;
	}
#ifdef EEPROM_DEBUG
	debug_log("eeprom_table_version: %s\n", pCamCalData->ManufactureData.eeprom_table_version);
	debug_log("part_number: %s\n", pCamCalData->ManufactureData.part_number);
	debug_log("actuator_id: %s\n", pCamCalData->ManufactureData.actuator_id);
	debug_log("lens_id: %s\n", pCamCalData->ManufactureData.lens_id);
	debug_log("manufacturer_id: %s\n", pCamCalData->ManufactureData.manufacturer_id);
	debug_log("factory_id: %s\n", pCamCalData->ManufactureData.factory_id);
	debug_log("manufacture_line: %s\n", pCamCalData->ManufactureData.manufacture_line);
	debug_log("manufacture_date: %s\n", pCamCalData->ManufactureData.manufacture_date);
	debug_log("serial_number: %s\n", pCamCalData->ManufactureData.serial_number);
#endif

	return err;
}
