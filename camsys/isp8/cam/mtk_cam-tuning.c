/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include "mtk_cam.h"
#include "mtk_cam-ois-info.h"


void mtk_cam_tuning_probe(void)
{
  /*create HF client*/
  pr_info("%s: E !! \n", __func__);
  mtk_cam_ois_info_create();
}

void mtk_cam_tuning_init(struct mtk_cam_tuning *param)
{
  pr_info("%s: E !!\n", __func__);
  mtk_cam_ois_info_init(WIDE);
  param->sensor_idx = WIDE;
}

void mtk_cam_tuning_uninit(void)
{
  pr_info("%s: E !!\n", __func__);
  mtk_cam_ois_info_uinit(WIDE);
}

void mtk_cam_tuning_update(struct mtk_cam_tuning *param)
{

  pr_info("%s: E !!\n", __func__);
  struct mtk_cam_ois_info *ois_data = NULL;
  /* perframe get ois data for algo*/
  ois_data = mtk_cam_ois_info_update(param);

#ifdef SAMPLE_CODE
  /* after algo finished */
  u64 current_ts_ns = ktime_get_boottime_ns();
  
  if (current_ts_ns - param->begin_ts_ns < CAM_TUNING_ALGO_DEADLINE_NS) {
    /* update shading table */
    memcpy(param->shading_tbl, /*algo_buf*/, MTK_CAM_LSCI_TABLE_SIZE);
  } else {
    /* print warning log */
    pr_info("bypass shading table update");
  }
#endif

}
