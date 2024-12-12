// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 MediaTek Inc.
 */

#include <linux/remoteproc.h>
#include <linux/rpmsg/mtk_ccd_rpmsg.h>
#include <linux/platform_data/mtk_ccd.h>

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
EXPORT_SYMBOL(mtk_ccd_client_start);

int mtk_ccd_client_stop(struct mtk_ccd *ccd)
{
	/* TODO: for multi open */
	(void) ccd;
	return 0;
}
EXPORT_SYMBOL(mtk_ccd_client_stop);

int mtk_ccd_client_get_channel(struct mtk_ccd *ccd,
			       struct mtk_ccd_client_cb *cb)
{
	struct rpmsg_channel_info msg;
	struct mtk_rpmsg_device *rpmsg_dev;
	struct mtk_rpmsg_rproc_subdev *mtk_subdev;
	int ipi_id, id_mask;

	ipi_id = cb->ipi_id;  /* FIXME: No need */

	/* TODO: no need get by msg */
	(void)snprintf(msg.name, RPMSG_NAME_SIZE, "mtk-camsys\%d",
		       ipi_id - CCD_IPI_ISP_MAIN);
	msg.src = ipi_id;

	rpmsg_dev = mtk_get_client_msgdevice(ccd->rpmsg_subdev, &msg,
					     cb->send_msg_ack,
					     cb->priv);
	if (!rpmsg_dev) {
		pr_info("%s failed get_client_msgdevice\n", __func__);
		return -EINVAL;
	}
	rpmsg_dev->channel_cb = cb;

	/* to check get the same channel for access by arr index */
	mtk_subdev = to_mtk_subdev(ccd->rpmsg_subdev);
	if (mtk_subdev->channels[ipi_id] != rpmsg_dev) {
		pr_info("%s channels mismatch\n", __func__);
		goto err_put_rpmsg_dev;
	}

	/* TODO: for multi access */
	id_mask = to_ccd_id_mask(0, ipi_id);

	pr_info("%s, id mask:%#010x", __func__, id_mask);

	return id_mask;

err_put_rpmsg_dev:
	rpmsg_dev->channel_cb = NULL;
	mtk_destroy_client_msgdevice(ccd->rpmsg_subdev, &msg);
	return -EINVAL;
}
EXPORT_SYMBOL(mtk_ccd_client_get_channel);

int mtk_ccd_client_put_channel(struct mtk_ccd *ccd, int id_mask)
{
	struct rpmsg_channel_info msg;
	struct mtk_rpmsg_device *rpmsg_dev;
	struct mtk_rpmsg_rproc_subdev *mtk_subdev;
	int ipi_id;

	pr_info("%s, id mask:%#010x", __func__, id_mask);

	ipi_id = channel_id_from_mask(id_mask);
	/* TODO: search by idx directly */
	(void)snprintf(msg.name, RPMSG_NAME_SIZE, "mtk-camsys\%d",
		       ipi_id - CCD_IPI_ISP_MAIN);
	msg.src = ipi_id;

	mtk_subdev = to_mtk_subdev(ccd->rpmsg_subdev);
	rpmsg_dev = mtk_subdev->channels[ipi_id];
	rpmsg_dev->channel_cb = NULL;
	mtk_destroy_client_msgdevice(ccd->rpmsg_subdev, &msg);

	return 0;
}
EXPORT_SYMBOL(mtk_ccd_client_put_channel);

int mtk_ccd_client_msg_send(struct mtk_ccd *ccd, int id_mask,
			    void *data, int len)
{
	struct mtk_rpmsg_device *rpmsg_dev;
	struct mtk_rpmsg_rproc_subdev *mtk_subdev;
	int ipi_id;

	// debug only
	pr_info("%s, id mask:%#010x", __func__, id_mask);

	ipi_id = channel_id_from_mask(id_mask);
	mtk_subdev = to_mtk_subdev(ccd->rpmsg_subdev);
	rpmsg_dev = mtk_subdev->channels[ipi_id];

	if (!rpmsg_dev->rpdev.ept) {
		pr_info("%s failed, id mask:%#010x", __func__, id_mask);
		return -1;
	}

	rpmsg_send(rpmsg_dev->rpdev.ept, data, len);

	return 0;
}
EXPORT_SYMBOL(mtk_ccd_client_msg_send);
