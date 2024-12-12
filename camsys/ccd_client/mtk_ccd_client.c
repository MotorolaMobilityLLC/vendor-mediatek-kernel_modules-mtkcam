// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 MediaTek Inc.
 */

#include <linux/remoteproc.h>
#include <linux/rpmsg/mtk_ccd_rpmsg.h>

#include "mtk_ccd_client.h"

#define BITS_CCD_CENTER_ID 16
#define _MASK_CCD_CENTER_ID (BIT(BITS_CCD_CENTER_ID) - 1)

/*
 * ccd channel ipi mask = [31:16] center_id + [15:0] ipi_id
 */
static inline unsigned int center_id_from_mask(unsigned int mask)
{
	return mask >> BITS_CCD_CENTER_ID;
}

static inline unsigned int channel_id_from_mask(unsigned int mask)
{
	return mask & _MASK_CCD_CENTER_ID;
}

static inline unsigned int to_ccd_id_mask(unsigned int center_id,
					  unsigned int channel_id)
{
	return center_id << BITS_CCD_CENTER_ID |
	       channel_id;
}

int mtk_ccd_client_start(struct mtk_ccd *ccd)
{
	/* TODO: for multi open */
	(void) ccd;
	return 0;
}

int mtk_ccd_client_stop(struct mtk_ccd *ccd)
{
	/* TODO: for multi open */
	(void) ccd;
	return 0;
}

int mtk_ccd_client_get_channel(struct mtk_ccd *ccd,
			       struct mtk_ccd_client_cb *cb)
{
	int center_id, channel_id, id_mask;

	/* get channel center */
	center_id = 1;  /* TODO: for multi open */

	/* get channel - get rpmsg dev + create ept */
	channel_id = mtk_ccd_get_channel(ccd, center_id, cb);
	if (channel_id == -1) {
		pr_info("%s center-%d get channel failed\n", __func__, center_id);
		return -EINVAL;
	}

	/* channel init - start worker service */
	if (mtk_ccd_channel_init(ccd, center_id, channel_id)) {
		pr_info("%s channel-%d-%d init failed\n", __func__,
			center_id, channel_id);
		mtk_ccd_put_channel(ccd, center_id, channel_id);
		return -EINVAL;
	}

	id_mask = to_ccd_id_mask(center_id, channel_id);
	pr_info("%s, id mask:%#010x", __func__, id_mask);

	return id_mask;
}
EXPORT_SYMBOL(mtk_ccd_client_get_channel);

int mtk_ccd_client_put_channel(struct mtk_ccd *ccd, int id_mask)
{
	int center_id, channel_id;

	center_id = center_id_from_mask(id_mask);
	channel_id = channel_id_from_mask(id_mask);

	/* channel uninit - stop worker service */
	mtk_ccd_channel_uninit(ccd, center_id, channel_id);

	/* put channel - free ept + return rpmsg dev */
	mtk_ccd_put_channel(ccd, center_id, channel_id);

	pr_info("%s, id mask:%#010x", __func__, id_mask);

	return 0;
}
EXPORT_SYMBOL(mtk_ccd_client_put_channel);

int mtk_ccd_client_msg_send(struct mtk_ccd *ccd, int id_mask,
			    void *data, int len)
{
	int center_id, channel_id;

	center_id = center_id_from_mask(id_mask);
	channel_id = channel_id_from_mask(id_mask);

	// debug only, pr_debug
	pr_info("%s, id mask:%#010x", __func__, id_mask);

	if (mtk_ccd_channel_send(ccd, center_id, channel_id, data, len)) {
		pr_info("%s failed, id mask:%#010x", __func__, id_mask);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(mtk_ccd_client_msg_send);
