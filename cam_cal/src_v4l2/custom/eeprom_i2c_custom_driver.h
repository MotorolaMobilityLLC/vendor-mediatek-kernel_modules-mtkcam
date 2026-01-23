/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __EEPROM_I2C_CUSTOM_DRIVER_H
#define __EEPROM_I2C_CUSTOM_DRIVER_H
#include <linux/i2c.h>

/*custom_eeprom_read only for Terrain tele s5kjn5*/
void custom_eeprom_read(bool *is_valid, void *d_data);

/************************************************************
 * I2C read function (Custom)
 * Customer's driver can put on here
 * Below is an example
 ************************************************************/
unsigned int Custom_read_region(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);

#endif				/* __CAM_CAL_LIST_H */
