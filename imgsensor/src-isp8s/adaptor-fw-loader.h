/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2024 MediaTek Inc. */

#ifndef __ADAPTOR_FW_LOADER_H__
#define __ADAPTOR_FW_LOADER_H__

#include "adaptor.h"

#define firmware_support(__x__) \
	((__x__) && (__x__)->is_fw_support)

#define for_each_firmware(__f__, __x__) \
	list_for_each_entry((__f__), &(__x__)->fw_loader->fw_list, list)

/* Macro to copy a member from one struct to another based on the sam member name */
#define COPY_COMMON_MEMBER(dest_struct, src_struct, member) \
{ \
	(dest_struct)->member = (src_struct)->member; \
}

/* Macro to copy a member from one struct to another based on specific member name */
#define COPY_SPECIFIC_MEMBER(dest_struct, src_struct, dest_member, src_member) \
{ \
	(dest_struct)->dest_member = (src_struct)->src_member; \
}

/**
 * Look up firmware list
 *
 * @param loader The firmware loader
 * @param fw_file_name The string array to be check if exist
 * @param count The array element count of {@code fw_file_name}
 *
 * @return zero if successful or negative number if error occurred
 */
int lookup_firmwares(struct sensor_firmware_loader * const loader,
		     const char *fw_file_name[], int count);

/**
 * Loading the correspond firmware
 *
 * @param ctx The adaptor context
 * @param fw_name The firmware file path
 *
 * @return zero if successful
 */
int loading_firmware(struct adaptor_ctx *ctx, const char * const fw_name);

#endif
