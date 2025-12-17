/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __MTK_CAM_OISINFO_H
#define __MTK_CAM_OISINFO_H

#include <linux/kthread.h>
#include <linux/mutex.h>

#include "mtk_cam.h"
#include "hf_manager.h"
#include "mtk_cam-tuning.h"

#define OIS_DATA_BUFFER_SIZE 60
#define EXP_THRESHOLD (3 * 1000 * 1000)

struct ois_global_data {
	struct hf_manager_event buffer[OIS_DATA_BUFFER_SIZE];
	int head;
	int tail;
	int count;
	struct mutex lock;
};

struct mtk_cam_ois_info {
	int64_t timestamp;
	int32_t pos_x;
	int32_t pos_y;
};

struct ois_client_info {
	int8_t camera_id;
	struct task_struct *task;
	struct hf_client *client;
	struct mtk_cam_ois_info ois_info;
	struct sensor_info info;
	struct mutex ois_lock;
};

/* different cam for different ois type */
enum ois_sensor_id_enum {
	WIDE = 2,
	TELE,
	ULTRA_TELE,
	ULTRA_WIDE,
	CAM_MAX,
};

extern struct ois_client_info OisClientInfo[CAM_MAX];

void mtk_cam_ois_info_create(void);
void mtk_cam_ois_info_init(int camera_id);
void mtk_cam_ois_info_uinit(int camera_id);
struct mtk_cam_ois_info *mtk_cam_ois_info_update(struct mtk_cam_tuning *param);
int ois_calculate_average(struct ois_client_info *data,
	struct mtk_cam_tuning *param);


#endif /*__MTK_CAM_OISINFO_H*/





