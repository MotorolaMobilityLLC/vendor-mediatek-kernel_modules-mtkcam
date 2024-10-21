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

unsigned int gc13a2_mot_do_manufacture_info(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int mot_gc13a2_do_factory_verify(struct EEPROM_DRV_FD_DATA *pdata, unsigned int *pGetSensorCalData);
static unsigned int mot_gc13a2_do_2a_gain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);

#define MOT_GC13A2_EEPROM_ADDR 0x00
#define MOT_GC13A2_EEPROM_DATA_SIZE 0x192A
#define MOT_GC13A2_SERIAL_NUMBER_ADDR 0x15
#define MOT_GC13A2_MNF_ADDR 0x00
#define MOT_GC13A2_MNF_DATA_SIZE 37
#define MOT_GC13A2_MNF_CHECKSUM_ADDR 0x25
#define MOT_GC13A2_AF_ADDR 0x27
#define MOT_GC13A2_AF_DATA_SIZE 24
#define MOT_GC13A2_AF_CHECKSUM_ADDR 0x3F
#define MOT_GC13A2_AWB_ADDR 0x41
#define MOT_GC13A2_AWB_DATA_SIZE 43
#define MOT_GC13A2_AWB_CHECKSUM_ADDR 0x6C
#define MOT_GC13A2_LSC_ADDR 0x0BCE
#define MOT_GC13A2_LSC_DATA_SIZE 1868
#define MOT_GC13A2_LSC_CHECKSUM_ADDR 0X131A
#define MOT_GC13A2_PDAF1_ADDR 0x131C
#define MOT_GC13A2_PDAF2_ADDR 0x150C
#define MOT_GC13A2_PDAF_DATA1_SIZE 496
#define MOT_GC13A2_PDAF_DATA2_SIZE 1004
#define MOT_GC13A2_PDAF_DATA_SIZE (MOT_GC13A2_PDAF_DATA1_SIZE+MOT_GC13A2_PDAF_DATA2_SIZE)
#define MOT_GC13A2_PDAF_CHECKSUM_ADDR 0x18F8
#define MOT_GC13A2_AF_SYNC_DATA_ADDR 0x18FC
#define MOT_GC13A2_AF_SYNC_DATA_SIZE 24
#define MOT_GC13A2_AF_SYNC_DATA_CHECKSUM_ADDR 0x1914

#define MOT_GC13A2_MTK_NECESSARY_DATA_ADDR 0x1916
#define MOT_GC13A2_MTK_NECESSARY_DATA_SIZE 18
#define MOT_GC13A2_MTK_NECESSARY_DATA_CHECKSUM_ADDR 0x1928

#define MOTO_OB_VALUE 64
#define MOTO_WB_VALUE_BASE 64
#define MOT_AWB_RB_MIN_VALUE 200
#define MOT_AWB_G_MIN_VALUE 760
#define MOT_AWB_RBG_MAX_VALUE 880
#define MOT_AWB_GRGB_RATIO_MIN_1000TIMES 970
#define MOT_AWB_GRGB_RATIO_MAX_1000TIMES 1030
#define ABS(a,b) a>=b?(a-b):(b-a)
#define MAX_temp(a,b,c) (a)>(b)?((a)>(c)?(a):(c)):((b)>(c)?(b):(c))

static struct STRUCT_CALIBRATION_LAYOUT_STRUCT cal_layout_table = {
	0x00000003, 0x32443832, CAM_CAL_SINGLE_EEPROM_DATA,
	{
		{0x00000000, 0x00000000, 0x00000000, do_module_version},
		{0x00000000, 0x00000000, 0x00000000, do_part_number},
		{0x00000001, MOT_GC13A2_LSC_ADDR, MOT_GC13A2_LSC_DATA_SIZE, mot_do_single_lsc},
		{0x00000001, 0x00000027, 0x00000045, mot_gc13a2_do_2a_gain},
		{0x00000001, MOT_GC13A2_PDAF1_ADDR, MOT_GC13A2_PDAF_DATA_SIZE, mot_do_pdaf},
		{0x00000000, 0x00000000, 0x00000000, do_stereo_data},
		{0x00000000, 0x00000000, 0x00000000, do_dump_all},
		{0x00000000, 0x00000000, 0x00000000, do_lens_id},
		{0x00000000, 0x00000000, 0x00000000, NULL},
		{0x00000001, MOT_GC13A2_MNF_ADDR, MOT_GC13A2_MNF_DATA_SIZE, gc13a2_mot_do_manufacture_info}
	}
};

struct STRUCT_CAM_CAL_CONFIG_STRUCT mot_gc13a2_eeprom = {
	.name = "mot_gc13a2_eeprom",
	.sensor_type = UW_CAMERA,
	.check_layout_function = mot_layout_no_ck,
	.read_function = Common_read_region,
	.mot_do_factory_verify_function = mot_gc13a2_do_factory_verify,
	.layout = &cal_layout_table,
	.sensor_id = MOT_CYBERT_GC13A2_SENSOR_ID,
	.i2c_write_id = 0xA0,
	.max_size = 0x8000,
	.serial_number_bit = 16,
	.enable_preload = 1,
	.preload_size = MOT_GC13A2_EEPROM_DATA_SIZE,
};

static unsigned int mot_gc13a2_do_2a_gain(struct EEPROM_DRV_FD_DATA *pdata,
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
	int af_addr = MOT_GC13A2_AF_ADDR;
	int af_size = MOT_GC13A2_AF_DATA_SIZE;
	int awb_addr = MOT_GC13A2_AWB_ADDR;
	int awb_size = MOT_GC13A2_AWB_DATA_SIZE;

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
			debug_log("WB check_crc16 ok");
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			debug_log("WB check_crc16 err");
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
			debug_log("AF check_crc16 ok");
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			debug_log("AF check_crc16 err");
			err = CAM_CAL_ERR_NO_3A_GAIN;
			return err;
		}
		AFMacroDistance = af_data[0]<<8 | af_data[1];
		AFMacro = (af_data[2]<<8 | af_data[3])/64;
		AFInfDistance = af_data[4]<<8 | af_data[5];
		AFInf = (af_data[6]<<8 | af_data[7])/64;
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

	if(pCamCalData->sensorID == MOT_CYBERT_GC13A2_SENSOR_ID) {
		//af posture calibration data
		unsigned char AF_POSTURE[MOT_GC13A2_MTK_NECESSARY_DATA_SIZE+2];
		unsigned int af_posture_data_offset = MOT_GC13A2_MTK_NECESSARY_DATA_ADDR;
		unsigned int af_inf_posture, af_macro_posture, AF_infinite_calibration_temperature;
		memset(AF_POSTURE, 0, 20);
		debug_log("af_posture_data_offset = 0x%x\n", af_posture_data_offset);

		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				af_posture_data_offset, MOT_GC13A2_MTK_NECESSARY_DATA_SIZE+2, (unsigned char *) AF_POSTURE);
		if (read_data_size > 0)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}

		checkSum = AF_POSTURE[MOT_GC13A2_MTK_NECESSARY_DATA_SIZE]<<8 | AF_POSTURE[MOT_GC13A2_MTK_NECESSARY_DATA_SIZE+1];
		if(check_crc16(AF_POSTURE, MOT_GC13A2_MTK_NECESSARY_DATA_SIZE, checkSum)) {
			debug_log("AF pos comp check_crc16 ok");
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			debug_log("AF pos comp check_crc16 err");
			err = CAM_CAL_NONE_BITEN;
			return err;
		}

		af_inf_posture = AF_POSTURE[10]<<8|AF_POSTURE[9];
		af_macro_posture = AF_POSTURE[12]<<8|AF_POSTURE[11];
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


static unsigned gc13a2_lens_id_to_name(uint8_t id,unsigned int block_size,unsigned char * lens_id,unsigned int err)
{
    int i=0,flag=0,ret=0;

    const struct STRUCT_MOT_LENS__ID lens_table[] =
    {
	{0x43,"39374D-400"},
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

unsigned int gc13a2_mot_do_manufacture_info(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];
	int read_data_size, checkSum, ret;
	uint8_t  tempBuf[39] = {0};

	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			start_addr, block_size + 2, (unsigned char *)tempBuf);
	if (read_data_size <= 0) {
		err = CAM_CAL_ERR_NO_PARTNO;
		return err;
	}

	checkSum = (tempBuf[37]<<8) | (tempBuf[38]);

	if(check_crc16(tempBuf, 37, checkSum)) {
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
	ret = snprintf(pCamCalData->ManufactureData.actuator_id, MAX_CALIBRATION_STRING, "0x%x", tempBuf[11]);

	if (ret < 0 || ret >= block_size) {
		debug_log("snprintf of mnf->actuator_id failed");
		memset(pCamCalData->ManufactureData.actuator_id, 0,
			sizeof(pCamCalData->ManufactureData.actuator_id));
		err = CAM_CAL_ERR_NO_PARTNO;
	}
	//lens_id
        err = gc13a2_lens_id_to_name(tempBuf[12],block_size,pCamCalData->ManufactureData.lens_id,err);
	//manufacture id
	if(tempBuf[13] == 'T' && tempBuf[14] == 'S') {
		ret = snprintf(pCamCalData->ManufactureData.manufacturer_id, MAX_CALIBRATION_STRING, "Tianshi");
	} else if(tempBuf[13] == 'S' && tempBuf[14] == 'U') {
		ret = snprintf(pCamCalData->ManufactureData.manufacturer_id, MAX_CALIBRATION_STRING, "Sunny");
	} else if(tempBuf[13] == 'Q' && tempBuf[14] == 'T') {
		ret = snprintf(pCamCalData->ManufactureData.manufacturer_id, MAX_CALIBRATION_STRING, "Qtech");
	} else if(tempBuf[13] == 'S' && tempBuf[14] == 'W') {
		ret = snprintf(pCamCalData->ManufactureData.manufacturer_id, MAX_CALIBRATION_STRING, "Sunwin");
	} else if(tempBuf[13] == 'O' && tempBuf[14] == 'F') {
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
	ret = snprintf(pCamCalData->ManufactureData.factory_id, MAX_CALIBRATION_STRING, "%c%c", tempBuf[15], tempBuf[16]);

	if (ret < 0 || ret >= block_size) {
		debug_log("snprintf of mnf->factory_id failed");
		memset(pCamCalData->ManufactureData.factory_id, 0,
			sizeof(pCamCalData->ManufactureData.factory_id));
		err = CAM_CAL_ERR_NO_PARTNO;
	}

	//manufacture_line
	ret = snprintf(pCamCalData->ManufactureData.manufacture_line, MAX_CALIBRATION_STRING, "0x%x", tempBuf[17]);

	if (ret < 0 || ret >= block_size) {
		debug_log("snprintf of mnf->manufacture_line failed");
		memset(pCamCalData->ManufactureData.manufacture_line, 0,
			sizeof(pCamCalData->ManufactureData.manufacture_line));
		err = CAM_CAL_ERR_NO_PARTNO;
	}

	//manufacture_date
	ret = snprintf(pCamCalData->ManufactureData.manufacture_date, MAX_CALIBRATION_STRING, "20%02u%02u%02u",
		tempBuf[18], tempBuf[19], tempBuf[20]);

	if (ret < 0 || ret >= block_size) {
		debug_log("snprintf of mnf->manufacture_date failed");
		memset(pCamCalData->ManufactureData.manufacture_date, 0,
			sizeof(pCamCalData->ManufactureData.manufacture_date));
		err = CAM_CAL_ERR_NO_PARTNO;
	}

	//serial_number
	ret = snprintf(pCamCalData->ManufactureData.serial_number, MAX_CALIBRATION_STRING,
		"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
		tempBuf[21], tempBuf[22], tempBuf[23], tempBuf[24], tempBuf[25], tempBuf[26], tempBuf[27], tempBuf[28],
		tempBuf[29], tempBuf[30], tempBuf[31], tempBuf[32], tempBuf[33], tempBuf[34], tempBuf[35], tempBuf[36]);

	if (ret < 0 || ret >= block_size) {
		debug_log("snprintf of mnf->serial_number failed");
		memset(pCamCalData->ManufactureData.serial_number, 0,
			sizeof(pCamCalData->ManufactureData.serial_number));
		err = CAM_CAL_ERR_NO_PARTNO;
	}

	debug_log("eeprom_table_version: %s\n", pCamCalData->ManufactureData.eeprom_table_version);
	debug_log("part_number: %s\n", pCamCalData->ManufactureData.part_number);
	debug_log("actuator_id: %s\n", pCamCalData->ManufactureData.actuator_id);
	debug_log("lens_id: %s\n", pCamCalData->ManufactureData.lens_id);
	debug_log("manufacturer_id: %s\n", pCamCalData->ManufactureData.manufacturer_id);
	debug_log("factory_id: %s\n", pCamCalData->ManufactureData.factory_id);
	debug_log("manufacture_line: %s\n", pCamCalData->ManufactureData.manufacture_line);
	debug_log("manufacture_date: %s\n", pCamCalData->ManufactureData.manufacture_date);
	debug_log("serial_number: %s\n", pCamCalData->ManufactureData.serial_number);

	return err;
}

static unsigned int mot_gc13a2_do_factory_verify(struct EEPROM_DRV_FD_DATA *pdata, unsigned int *pGetSensorCalData)
{
	struct STRUCT_MOT_EEPROM_DATA *pCamCalData =
				(struct STRUCT_MOT_EEPROM_DATA *)pGetSensorCalData;
	int read_data_size, checkSum, checkSum1;

	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			MOT_GC13A2_EEPROM_ADDR, MOT_GC13A2_EEPROM_DATA_SIZE, (unsigned char *)pCamCalData->DumpAllEepromData);
	if (read_data_size <= 0) {
		return CAM_CAL_ERR_NO_DEVICE;
	}
	//mnf check
	checkSum = (pCamCalData->DumpAllEepromData[MOT_GC13A2_MNF_CHECKSUM_ADDR])<< 8
		|(pCamCalData->DumpAllEepromData[MOT_GC13A2_MNF_CHECKSUM_ADDR+1]);
	debug_log("checkSum  = 0x%x", checkSum);

	if(check_crc16(pCamCalData->DumpAllEepromData, MOT_GC13A2_MNF_DATA_SIZE, checkSum)) {
		debug_log("check mnf crc16 ok");
	} else {
		debug_log("check mnf crc16 err");
		return CAM_CAL_ERR_NO_PARTNO;
	}
	//for dump serial_number
	memcpy(pCamCalData->serial_number, &pCamCalData->DumpAllEepromData[MOT_GC13A2_SERIAL_NUMBER_ADDR],
		pCamCalData->serial_number_bit);

	//af check
	checkSum = (pCamCalData->DumpAllEepromData[MOT_GC13A2_AF_CHECKSUM_ADDR])<< 8
		|(pCamCalData->DumpAllEepromData[MOT_GC13A2_AF_CHECKSUM_ADDR+1]);
	debug_log("checkSum  = 0x%x", checkSum);

	if(check_crc16(pCamCalData->DumpAllEepromData+MOT_GC13A2_AF_ADDR, MOT_GC13A2_AF_DATA_SIZE, checkSum)) {
		debug_log("check af crc16 ok");
	} else {
		debug_log("check af crc16 err");
		return CAM_CAL_ERR_NO_3A_GAIN;
	}

	//awb check
	if((mot_check_awb_data(pCamCalData->DumpAllEepromData+MOT_GC13A2_AWB_ADDR, MOT_GC13A2_AWB_DATA_SIZE + 2)) == NO_ERRORS) {
		debug_log("check awb data ok");
	} else {
		debug_log("check awb data err");
		return CAM_CAL_ERR_NO_3A_GAIN;
	}

	//lsc check
	checkSum = (pCamCalData->DumpAllEepromData[MOT_GC13A2_LSC_CHECKSUM_ADDR])<< 8
		|(pCamCalData->DumpAllEepromData[MOT_GC13A2_LSC_CHECKSUM_ADDR+1]);
	debug_log("checkSum  = 0x%x", checkSum);

	if(check_crc16(pCamCalData->DumpAllEepromData+MOT_GC13A2_LSC_ADDR, MOT_GC13A2_LSC_DATA_SIZE, checkSum)) {
		debug_log("check lsc crc16 ok");
	} else {
		debug_log("check lsc crc16 err");
		return CAM_CAL_ERR_NO_SHADING;
	}

	//pdaf check
	checkSum = (pCamCalData->DumpAllEepromData[MOT_GC13A2_PDAF_CHECKSUM_ADDR])<< 8
		|(pCamCalData->DumpAllEepromData[MOT_GC13A2_PDAF_CHECKSUM_ADDR+1]);
	checkSum1 = (pCamCalData->DumpAllEepromData[MOT_GC13A2_PDAF_CHECKSUM_ADDR+2])<< 8
		|(pCamCalData->DumpAllEepromData[MOT_GC13A2_PDAF_CHECKSUM_ADDR+3]);
	debug_log("checkSum  = 0x%x, checkSum1 = 0x%x", checkSum, checkSum1);

	if(check_crc16(pCamCalData->DumpAllEepromData+MOT_GC13A2_PDAF1_ADDR, MOT_GC13A2_PDAF_DATA1_SIZE, checkSum)
		&& check_crc16(pCamCalData->DumpAllEepromData+MOT_GC13A2_PDAF2_ADDR, MOT_GC13A2_PDAF_DATA2_SIZE, checkSum1)) {
		debug_log("check pdaf crc16 ok");
	} else {
		debug_log("check pdaf crc16 err");
		return CAM_CAL_ERR_NO_PDAF;
	}

	//af sync data check
	checkSum = (pCamCalData->DumpAllEepromData[MOT_GC13A2_AF_SYNC_DATA_CHECKSUM_ADDR])<< 8
		|(pCamCalData->DumpAllEepromData[MOT_GC13A2_AF_SYNC_DATA_CHECKSUM_ADDR+1]);
	debug_log("checkSum  = 0x%x", checkSum);
	if(check_crc16(pCamCalData->DumpAllEepromData+MOT_GC13A2_AF_SYNC_DATA_ADDR, MOT_GC13A2_AF_SYNC_DATA_SIZE, checkSum)) {
		debug_log("check af sync data crc16 ok");
	} else {
		debug_log("check af sync data crc16 err");
		return CAM_CAL_ERR_NO_3A_GAIN;
	}

	//mtk necessary data check
	checkSum = (pCamCalData->DumpAllEepromData[MOT_GC13A2_MTK_NECESSARY_DATA_CHECKSUM_ADDR])<< 8
		|(pCamCalData->DumpAllEepromData[MOT_GC13A2_MTK_NECESSARY_DATA_CHECKSUM_ADDR+1]);
	debug_log("checkSum  = 0x%x", checkSum);
	if(check_crc16(pCamCalData->DumpAllEepromData+MOT_GC13A2_MTK_NECESSARY_DATA_ADDR, MOT_GC13A2_MTK_NECESSARY_DATA_SIZE, checkSum)) {
		debug_log("check necessary data crc16 ok");
	} else {
		debug_log("check necessary data crc16 err");
		return CAM_CAL_ERR_NO_PARTNO;
	}

	return CAM_CAL_ERR_NO_ERR;
}
