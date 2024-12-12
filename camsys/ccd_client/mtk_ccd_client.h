/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2024 MediaTek Inc.
 */

#ifndef __MTK_CCD_CLIENT_H
#define __MTK_CCD_CLIENT_H

#include <linux/platform_data/mtk_ccd.h>

int mtk_ccd_client_start(struct mtk_ccd *ccd);
int mtk_ccd_client_stop(struct mtk_ccd *ccd);
int mtk_ccd_client_get_channel(struct mtk_ccd *ccd, struct mtk_ccd_client_cb *cb);
int mtk_ccd_client_put_channel(struct mtk_ccd *ccd, int id_mask);
int mtk_ccd_client_msg_send(struct mtk_ccd *ccd, int id_mask, void *data, int len);

#endif /* __MTK_CCD_CLIENT_H */
