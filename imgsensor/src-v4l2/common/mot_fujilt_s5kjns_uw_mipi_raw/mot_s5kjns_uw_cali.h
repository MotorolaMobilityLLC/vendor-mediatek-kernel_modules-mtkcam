#ifndef __MOT_S5KJNS_CALI_H__
#define __MOT_S5KJNS_CALI_H__
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


/*HW_GGC definitions*/
#define S5KJNS_HW_GGC_DEBUG 0
#define S5KJNS_HW_GGC_EEPROM_ADDR 0xA0
#define S5KJNS_HW_GGC_DATA_START 0x19F2
#define S5KJNS_HW_GGC_DATA_LEN 346
#define S5KJNS_HW_GGC_REG_START 0x6F12

void mot_s5kjns_uw_apply_xtc_data(void *sensor_ctx);

#endif
