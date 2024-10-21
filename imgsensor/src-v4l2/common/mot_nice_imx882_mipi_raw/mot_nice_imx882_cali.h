#ifndef __MOT_NICE_IMX882_CALI_H__
#define __MOT_NICE_IMX882_CALI_H__
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define_v4l2.h"
#include "kd_imgsensor_errcode.h"

#include "adaptor-subdrv-ctrl.h"
#include "adaptor-i2c.h"
#include "adaptor.h"

/*SPC definitions*/
#define IMX882_SPC_DEBUG 0
#define IMX882_SPC_DUMP 0
#define IMX882_SPC_EEPROM_ADDR 0xA0
#define IMX882_SPC_DATA_START 0x2673
#define IMX882_SPC_DATA_LEN 384
#define IMX882_SPC_REG1_START 0xD200
#define IMX882_SPC_REG2_START 0xD300

/*QSC definitions*/
#define IMX882_QSC_DEBUG 0
#define IMX882_QSC_DUMP 0
#define IMX882_QSC_EEPROM_ADDR 0xA0
#define IMX882_QSC_DATA_START 0x1A71
#define IMX882_QSC_DATA_LEN 3072
#define IMX882_QSC_REG_START 0xC000

void mot_imx882_apply_qsc_spc_data(void *sensor_ctx);

#endif