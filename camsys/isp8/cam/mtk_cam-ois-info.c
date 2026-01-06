/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include "mtk_cam-ois-info.h"

struct ois_client_info OisClientInfo[CAM_MAX];
struct ois_global_data OisTotalData;
#define IDX_HALL_X 8
#define IDX_HALL_Y 9


static int Poll_Ois_data(void *arg)
{
	int data_size = 0;
	struct hf_manager_event  data[1];
	struct ois_client_info  *resource = NULL;
	int camera_id = 0;

	if ( arg == NULL) {
		pr_err("[%s]: invalid thread data!\n", __func__);
		return -1;
	}

	resource = (struct ois_client_info *)arg;
	camera_id = resource->camera_id;
	pr_info("[%s]CamID:%d task %p start\n", __func__, camera_id, resource->task);

	while (!kthread_should_stop()) {
		data_size = hf_client_poll_sensor_timeout(resource->client,
					data, ARRAY_SIZE(data), 5000000);

		if (data_size <= 0) {
			pr_err("[%s]CamID:%d fetch ois data fail %d\n", __func__, camera_id, data_size);
			break;
			return 0;
		}
    /* update ois data to OisTotalData*/
		mutex_lock(&OisTotalData.lock);
		int write_index = OisTotalData.head;

		memcpy(&OisTotalData.buffer[write_index], &data[0], sizeof(struct hf_manager_event));
		OisTotalData.head = (write_index + 1) % OIS_DATA_BUFFER_SIZE;

		if (OisTotalData.count < OIS_DATA_BUFFER_SIZE)
			OisTotalData.count++;
		else
			OisTotalData.tail = (OisTotalData.tail + 1) % OIS_DATA_BUFFER_SIZE;

		mutex_unlock(&OisTotalData.lock);
	}

	pr_info("[%s]CamID:%d Poll_Ois_data task %p exit! ", __func__,
		camera_id, resource->task);
	return 0;
}


int ois_calculate_average(struct ois_client_info *data,
struct mtk_cam_tuning *param)
{
	int count = 0;
	u64 ois_start = 0;
	u64 ois_end = 0;
	int32_t pos_x_sum = 0;
	int32_t pos_y_sum = 0;
	int sensorGain = 1;

	/*Calculate the valid OIS data timestamp interval for the current frame.*/
	if ( param->exp_time_ns >= EXP_THRESHOLD ){
		ois_start = (param->sof_boottime_ns - param->exp_time_ns + (param->readout_ns / 2));
		ois_end = ois_start + param->exp_time_ns;
	} else {
		if(param->readout_ns / 2 >= EXP_THRESHOLD){
			ois_start = (param->sof_boottime_ns - param->exp_time_ns) +
						((param->readout_ns / 2) - (EXP_THRESHOLD - param->exp_time_ns));
			ois_end = ois_start + EXP_THRESHOLD;
		}else{
			ois_start = (param->sof_boottime_ns - param->exp_time_ns);
			ois_end = ois_start + (param->readout_ns / 2);
		}
	}

	/*get the average ois data during the valid timestamp*/
	mutex_lock(&OisTotalData.lock);
	int index = OisTotalData.tail;

	for (int i = 0; i < OisTotalData.count; i++) {
		struct hf_manager_event *event = &OisTotalData.buffer[index];

		if (event->timestamp >= ois_start && event->timestamp <= ois_end) {
			pos_x_sum += event->word[IDX_HALL_X];
			pos_y_sum += event->word[IDX_HALL_Y];
			count++;
		}
		index = (index + 1) % OIS_DATA_BUFFER_SIZE;
	}

	/*get sensor gain*/
	sensorGain = data->info.gain;
	/*use OisTotalData latest timestamp for ois_info ts*/
	int latestIdx = OisTotalData.head % OIS_DATA_BUFFER_SIZE ?
			(OisTotalData.head % OIS_DATA_BUFFER_SIZE -1) : (OIS_DATA_BUFFER_SIZE-1);
	struct hf_manager_event *latestEvent = &OisTotalData.buffer[latestIdx];
	/*update ois info*/
	data->ois_info.timestamp = latestEvent->timestamp;
	data->ois_info.pos_x = pos_x_sum /(sensorGain*count);
	data->ois_info.pos_y = pos_y_sum /(sensorGain*count);
	mutex_unlock(&OisTotalData.lock);

	return 0;
}


static int mtk_cam_ois_find_sensor(int camera_id)
{
	int sensor_type = SENSOR_TYPE_INVALID;

	switch (camera_id) {
	case WIDE:
		sensor_type = SENSOR_TYPE_OIS;
		break;
	case TELE:
		sensor_type = SENSOR_TYPE_OIS;
		break;
	case ULTRA_WIDE:
	case ULTRA_TELE:
		sensor_type = SENSOR_TYPE_OIS2;
		break;
	default:
		pr_info("[%s]: error type\n", __func__);
		break;
	}
	return sensor_type;
}

void mtk_cam_ois_info_create(void)
{
	int camera_id = 0;

	pr_info("[%s]: E\n", __func__);
	camera_id = TELE;
	memset(OisClientInfo, 0, (CAM_MAX * sizeof(struct ois_client_info)));
	/*demo code: create a ois client for each camera*/

	for (camera_id = WIDE; camera_id < CAM_MAX; camera_id++) {
		OisClientInfo[camera_id].client = hf_client_create();
		if ( OisClientInfo[camera_id].client == NULL) {
			pr_err("[%s]CamID:%d hf_client_create fail!\n", __func__, camera_id);
			continue;
		}
		OisClientInfo[camera_id].camera_id = camera_id;
		mutex_init(&OisClientInfo[camera_id].ois_lock);
	}

	pr_info("[%s]: X\n", __func__);
}

void mtk_cam_ois_info_init(int camera_id)
{
	int ret = 0;
	int ois_sensor_type = SENSOR_TYPE_INVALID;
	struct hf_client  *client = NULL;
	struct hf_manager_cmd  cmd;
	struct hf_manager_batch  *cfg_batch = NULL;
	struct ois_client_info *ois_resource = NULL;

	if (camera_id >= CAM_MAX || camera_id < WIDE) {
		pr_err("[%s] Invalid CamID:%d!\n", __func__, camera_id);
		return;
	}

	ois_resource = &OisClientInfo[camera_id];
	ois_sensor_type = mtk_cam_ois_find_sensor(camera_id);

	if (ois_sensor_type == SENSOR_TYPE_INVALID) {
		pr_err("[%s]invalid sensor type!\n", __func__);
		return;
	}
	pr_info("[%s]CamID:%d, sensor_type:%d\n", __func__, camera_id, ois_sensor_type);

	/*check ois_resource clinet status */
	if ( ois_resource->client != NULL) {
		client = ois_resource->client;
	} else {
		client = hf_client_create();
		if (!client) {
			pr_err("[%s]CamID:%d hf_client_create fail!\n", __func__, camera_id);
			return;
		}
		ois_resource->client = client;
		ois_resource->camera_id = camera_id;
	}

	/* find ois sensor*/
	ret = hf_client_find_sensor(client, ois_sensor_type);
	if (ret < 0) {
		pr_err("[%s] hf_client_find_sensor %u fail\n", __func__, ois_sensor_type);
		goto client_destroy;
	}

	memset(&cmd, 0, sizeof(cmd));
	cmd.sensor_type = ois_sensor_type;
	cmd.action = HF_MANAGER_SENSOR_ENABLE;
	/*cmd.down_sample = 0;*/
	cfg_batch = (struct hf_manager_batch *)cmd.data;
	cfg_batch->delay = 1000000;
	cfg_batch->latency = 0;
	ret = hf_client_control_sensor(client, &cmd);
	if (ret < 0) {
		pr_err("[%s]CamID:%d hf_client_control_sensor %u %u fail\n", __func__,
				camera_id, ois_sensor_type, cmd.action);
		goto client_destroy;
	}

	/*get sensor gain*/
	ret = hf_client_get_sensor_info(client, ois_sensor_type, &(ois_resource->info));
	if (ret < 0) {
		pr_warn("%s: (%d)(%d) hf_client_get_sensor_info fail ret(%d)\n",
				__func__, ois_resource->camera_id, ois_sensor_type, ret);
		goto client_destroy;
	}

	/* create ois query data thread */
	if (!ois_resource->task) {
		ois_resource->task = kthread_run(Poll_Ois_data, ois_resource, "ois_read_kthread");
		if (IS_ERR(ois_resource->task)) {
			pr_err("[%s]CamID:%d create ois read thread fail\n", __func__, camera_id);
			goto client_destroy;
		}
		pr_info("[%s]CamID:%d create task %p done\n", __func__, camera_id,
			ois_resource->task);
	}

	/*init ois global buffer*/
	OisTotalData.head = 0;
	OisTotalData.tail = 0;
	OisTotalData.count = 0;
	mutex_init(&OisTotalData.lock);

	pr_info("[%s]CamID:%d success X\n", __func__, camera_id);
	return;

client_destroy:
	if (client) {
		hf_client_destroy(client);
		ois_resource->client = NULL;
		pr_err("[%s]CamID:%d detroy ois hf client\n", __func__, camera_id);
		if (ois_resource->task) {
			pr_info("[%s]CamID:%d stop ois_resource task %p\n", __func__,
					camera_id, ois_resource->task);
			kthread_stop(ois_resource->task);
			ois_resource->task = NULL;
		}
	}
}

void mtk_cam_ois_info_uinit(int camera_id)
{
	int sensor_type = SENSOR_TYPE_INVALID;
	struct hf_client *client = NULL;
	struct task_struct *task = NULL;
	struct hf_manager_cmd cmd;
	struct ois_client_info *resource = NULL;

	if (camera_id >= CAM_MAX || camera_id < WIDE) {
		pr_err("[%s]CamID:%d invalid camera!\n", __func__, camera_id);
		return;
	}

	resource = &OisClientInfo[camera_id];
	sensor_type = mtk_cam_ois_find_sensor(camera_id);
	if (SENSOR_TYPE_INVALID == sensor_type) {
		pr_err("[%s]CamID:%d invalid sensor!\n", __func__, camera_id);
		return;
	}

	if ( resource->task == NULL || resource->client == NULL) {
		pr_err("[%s]CamID:%d invalid resource, client %p task %p\n", __func__,
						camera_id, resource->client, resource->task);
		return;
	}
	client = resource->client;
	task = resource->task;

	if (task) {
		kthread_stop(task);
		resource->task = NULL;
		task = NULL;
	}

	if (client) {
		memset(&cmd, 0, sizeof(cmd));
		cmd.sensor_type = sensor_type;
		cmd.action = HF_MANAGER_SENSOR_DISABLE;
		hf_client_control_sensor(client, &cmd);
		pr_info("[%s]CamID:%d destroy client %p\n", __func__, camera_id, client);
		hf_client_destroy(client);
		resource->client = NULL;
		OisTotalData.head = 0;
		OisTotalData.tail = 0;
		OisTotalData.count = 0;
		mutex_destroy(&OisTotalData.lock);
		client = NULL;
	}
	pr_info("[%s]CamID:%d success X\n", __func__, camera_id);

}

struct mtk_cam_ois_info *mtk_cam_ois_info_update(struct
mtk_cam_tuning *param)
{
	static struct mtk_cam_ois_info dummy_ois_data = {0};
	pr_info("[%s]: E\n", __func__);
	struct ois_client_info *resource = NULL;
	struct mtk_cam_ois_info *ois_data = NULL;
	int camid = param->sensor_idx;

	resource = &OisClientInfo[camid];
	if (resource->task ) {
		ois_calculate_average(resource, param);
		ois_data = &resource->ois_info;

	} else{
		pr_info("[%s]: OIS not initialized for camera %d\n", __func__, camid);
		ois_data = &dummy_ois_data;
	}
	return ois_data;
}



