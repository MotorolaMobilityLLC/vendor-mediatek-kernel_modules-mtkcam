// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2018 MediaTek Inc.

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/slab.h>

#include <uapi/linux/mtk_ccd_controls.h>
#include <linux/rpmsg/mtk_ccd_rpmsg.h>
#include <linux/platform_data/mtk_ccd.h>

#include "rpmsg_internal.h"
#include "mtk_ccd_rpmsg_internal.h"

static const struct rpmsg_endpoint_ops mtk_rpmsg_endpoint_ops;

void __ept_release(struct kref *kref)
{
	struct rpmsg_endpoint *ept = container_of(kref, struct rpmsg_endpoint,
						  refcount);
	struct mtk_ccd_rpmsg_endpoint *mept = to_mtk_rpmsg_endpoint(ept);
	struct mtk_rpmsg_rproc_subdev *mtk_subdev = mept->mtk_subdev;
	struct rpmsg_device *rpdev = ept->rpdev;

	dev_info(&mtk_subdev->pdev->dev, "free mtk rpmsg endpoint: %p\n",
		 mept);
	kfree(to_mtk_rpmsg_endpoint(ept));

	rpdev->ept = NULL;
	put_device(&rpdev->dev);
}

static struct rpmsg_endpoint *
__rpmsg_create_ept(struct mtk_rpmsg_rproc_subdev *mtk_subdev,
		   struct rpmsg_device *rpdev, rpmsg_rx_cb_t cb, void *priv,
		   u32 id)
{
	struct mtk_ccd_rpmsg_endpoint *mept;
	struct rpmsg_endpoint *ept;
	struct platform_device *pdev = mtk_subdev->pdev;

	mept = kzalloc(sizeof(*mept), GFP_KERNEL);
	if (!mept)
		return NULL;
	mept->mtk_subdev = mtk_subdev;

	ept = &mept->ept;
	kref_init(&ept->refcount);
	mutex_init(&ept->cb_lock);

	get_device(&rpdev->dev);
	ept->rpdev = rpdev;
	ept->cb = cb;
	ept->priv = priv;
	ept->ops = &mtk_rpmsg_endpoint_ops;
	ept->addr = id;

	mept->mchinfo.chinfo.src = id;
	mept->mchinfo.chinfo.dst = RPMSG_ADDR_ANY;
	mept->mchinfo.id = id;

	INIT_LIST_HEAD(&mept->pending_sendq.queue);
	spin_lock_init(&mept->pending_sendq.queue_lock);
	init_waitqueue_head(&mept->worker_readwq);
	atomic_set(&mept->ccd_cmd_sent, 0);
	atomic_set(&mept->ccd_mep_state, CCD_MENDPOINT_CREATED);

	dev_dbg(&pdev->dev, "%s: %d\n", __func__, ept->addr);
	return ept;
}

static struct rpmsg_endpoint *
mtk_rpmsg_create_ept(struct rpmsg_device *rpdev, rpmsg_rx_cb_t cb, void *priv,
		     struct rpmsg_channel_info chinfo)
{
	struct mtk_ccd_rpmsg_endpoint *mept;
	struct mtk_rpmsg_rproc_subdev *mtk_subdev =
		to_mtk_rpmsg_device(rpdev)->mtk_subdev;
	struct rpmsg_endpoint *ept =
		__rpmsg_create_ept(mtk_subdev, rpdev, cb, priv, chinfo.src);

	if (!ept)
		return NULL;

	mept = to_mtk_rpmsg_endpoint(ept);
	memcpy(mept->mchinfo.chinfo.name, chinfo.name, RPMSG_NAME_SIZE);
	return ept;
}

static void mtk_rpmsg_destroy_ept(struct rpmsg_endpoint *ept)
{
	struct mtk_ccd_params *ccd_params;
	struct mtk_ccd_rpmsg_endpoint *mept = to_mtk_rpmsg_endpoint(ept);
	struct mtk_rpmsg_rproc_subdev *mtk_subdev = mept->mtk_subdev;

	dev_info(&mtk_subdev->pdev->dev, "%s: src[%d]\n", __func__, ept->addr);

	atomic_set(&mept->ccd_mep_state, CCD_MENDPOINT_DESTROY);
	wake_up(&mept->worker_readwq);

	while (atomic_read(&mept->ccd_cmd_sent) > 0) {
		dev_info(&mtk_subdev->pdev->dev, "%s: cmd_sent: %d\n",
			 __func__, atomic_read(&mept->ccd_cmd_sent));

		spin_lock(&mept->pending_sendq.queue_lock);
		ccd_params = list_first_entry(&mept->pending_sendq.queue,
					      struct mtk_ccd_params,
					      list_entry);
		list_del(&ccd_params->list_entry);
		spin_unlock(&mept->pending_sendq.queue_lock);

		atomic_dec(&mept->ccd_cmd_sent);

		/* Directly call callback to return */
		mutex_lock(&ept->cb_lock);
		if (ept->cb)
			ept->cb(ept->rpdev,
				ccd_params->worker_obj.sbuf,
				ccd_params->worker_obj.len,
				ept->priv, ept->addr);

		mutex_unlock(&ept->cb_lock);

		/* free ccd_param */
		kfree(ccd_params);
	}

	/* make sure new inbound messages can't find this ept anymore */
	mutex_lock(&mtk_subdev->endpoints_lock);
	kref_put(&ept->refcount, __ept_release);
	mutex_unlock(&mtk_subdev->endpoints_lock);
}

static int mtk_rpmsg_send(struct rpmsg_endpoint *ept, void *data, int len)
{
	struct mtk_ccd_rpmsg_endpoint *mept = to_mtk_rpmsg_endpoint(ept);
	struct mtk_rpmsg_rproc_subdev *mtk_subdev = mept->mtk_subdev;

	return mtk_subdev->ops->ccd_send(mtk_subdev, mept, data, len, 0);
}

static int mtk_rpmsg_trysend(struct rpmsg_endpoint *ept, void *data, int len)
{
	struct mtk_rpmsg_rproc_subdev *mtk_subdev =
		to_mtk_rpmsg_endpoint(ept)->mtk_subdev;
	struct mtk_ccd_rpmsg_endpoint *mept = to_mtk_rpmsg_endpoint(ept);

	/*
	 * TODO: This currently is same as mtk_rpmsg_send, and wait until SCP
	 * received the last command.
	 */
	return mtk_subdev->ops->ccd_send(mtk_subdev, mept, data, len, 0);
}

static const struct rpmsg_endpoint_ops mtk_rpmsg_endpoint_ops = {
	.destroy_ept = mtk_rpmsg_destroy_ept,
	.send = mtk_rpmsg_send,
	.trysend = mtk_rpmsg_trysend,
};

static void mtk_rpmsg_release_device(struct device *dev)
{
	struct rpmsg_device *rpdev = to_rpmsg_device(dev);
	struct mtk_rpmsg_device *mdev = to_mtk_rpmsg_device(rpdev);

	dev_info(dev, "%s: rpdev %p\n", __func__, rpdev);

	kfree(mdev);
}

static const struct rpmsg_device_ops mtk_rpmsg_device_ops = {
	.create_ept = mtk_rpmsg_create_ept,
};

void
mtk_rpmsg_destroy_rpmsgdev(struct rproc_subdev *subdev)
{
	struct mtk_rpmsg_rproc_subdev *mtk_subdev = to_mtk_subdev(subdev);
	struct rpmsg_device *rpdev;
	struct device *dev;
	struct rpmsg_channel_info msg;
	u32 ipi_id = 0, len = 0;

	/* destroy rpmsg device */
	for (ipi_id = 0; ipi_id < CCD_IPI_MRAW_CMD; ipi_id++) {
		msg.src = ipi_id + 1;
		len = snprintf(msg.name,
			RPMSG_NAME_SIZE, "mtk-camsys\%d", ipi_id);

		if (len >= RPMSG_NAME_SIZE)
			pr_info("%s: snprintf fail\n", __func__);

		dev = rpmsg_find_device(&mtk_subdev->pdev->dev, &msg);
		if (dev) {
			if (rpmsg_unregister_device(&mtk_subdev->pdev->dev, &msg))
				dev_info(dev, "%s:rpmsg_unregister_device failed, info->src(%x)\n",
					 __func__, msg.src);

			rpdev = to_rpmsg_device(dev);
			rpmsg_destroy_ept(rpdev->ept);
			put_device(dev);
		}
		mtk_subdev->channels[msg.src] = NULL;
	}
}
EXPORT_SYMBOL_GPL(mtk_rpmsg_destroy_rpmsgdev);

int
mtk_destroy_client_msgdevice(struct rproc_subdev *subdev,
			     struct rpmsg_channel_info *info)
{
	int ret = 0;
	u32 listen_obj_rdy;
	struct rpmsg_device *rpdev;
	struct mtk_rpmsg_rproc_subdev *mtk_subdev = to_mtk_subdev(subdev);
	struct device *dev = rpmsg_find_device(&mtk_subdev->pdev->dev, info);

	if (!dev)
		return -EINVAL;

	rpdev = to_rpmsg_device(dev);
	mutex_lock(&mtk_subdev->master_listen_lock);

	listen_obj_rdy = atomic_read(&mtk_subdev->listen_obj_rdy);
	if (listen_obj_rdy == CCD_LISTEN_OBJECT_READY) {
		mutex_unlock(&mtk_subdev->master_listen_lock);
		wait_event_interruptible_timeout
			(mtk_subdev->ccd_listen_wq,
			(atomic_read(&mtk_subdev->listen_obj_rdy) ==
			CCD_LISTEN_OBJECT_PREPARING),
			msecs_to_jiffies(400));
		mutex_lock(&mtk_subdev->master_listen_lock);
	}

	memcpy(mtk_subdev->listen_obj.name,
	       info->name, RPMSG_NAME_SIZE);
	mtk_subdev->listen_obj.src = info->src;
	mtk_subdev->listen_obj.cmd = CCD_MASTER_CMD_DESTROY;

	atomic_set(&mtk_subdev->listen_obj_rdy, CCD_LISTEN_OBJECT_READY);
	wake_up(&mtk_subdev->master_listen_wq);
	mutex_unlock(&mtk_subdev->master_listen_lock);

	rpmsg_destroy_ept(rpdev->ept);

	dev_info(&mtk_subdev->pdev->dev, "%s %p\n", __func__, rpdev);

	put_device(dev);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_destroy_client_msgdevice);

static struct mtk_rpmsg_device *
mtk_rpmsg_create_rpmsgdev(struct mtk_rpmsg_rproc_subdev *mtk_subdev,
			  struct rpmsg_channel_info *info)
{
	struct rpmsg_device *rpdev;
	struct mtk_rpmsg_device *mdev;
	struct platform_device *pdev = mtk_subdev->pdev;
	int ret = 0;

	mdev = kzalloc(sizeof(*mdev), GFP_KERNEL);
	if (!mdev)
		return NULL;

	mdev->mtk_subdev = mtk_subdev;

	rpdev = &mdev->rpdev;
	rpdev->src = info->src;
	rpdev->dst = info->dst;
	rpdev->ops = &mtk_rpmsg_device_ops;

	/* for rpmsg_find_device */
	strncpy(rpdev->id.name, info->name, RPMSG_NAME_SIZE);

	/* If ccd hw ready, register rpmsg_device/rpmsg_driver here */

	rpdev->dev.parent = &pdev->dev;
	rpdev->dev.release = mtk_rpmsg_release_device;
	/* TODO: check relese flow, it is skipped currently */

	ret = rpmsg_register_device(rpdev);  /* add to parent */
	if (ret) {
		kfree(mdev);
		return NULL;
	}

	mutex_lock(&mtk_subdev->endpoints_lock);
	/* probe state, maybe no need lock */
	/* info-src == ipi_id */
	mtk_subdev->channels[info->src] = mdev;
	mutex_unlock(&mtk_subdev->endpoints_lock);

	dev_info(&pdev->dev, "%s: %d\n", __func__, rpdev->src);

	return mdev;
}

void
mtk_create_client_msgdevice(struct rproc_subdev *subdev)
{
	struct mtk_rpmsg_rproc_subdev *mtk_subdev = to_mtk_subdev(subdev);
	struct rpmsg_channel_info msg;
	u32 ipi_id = 0, len = 0;

	memset(&msg, 0, sizeof(msg));

	/* create client rpmsg device */
	for (ipi_id = 0; ipi_id < CCD_IPI_MRAW_CMD; ipi_id++) {
		msg.src = ipi_id + 1;
		len = snprintf(msg.name,
			RPMSG_NAME_SIZE, "mtk-camsys\%d", ipi_id);

		if (len >= RPMSG_NAME_SIZE)
			pr_info("%s: snprintf fail\n", __func__);

		if (mtk_rpmsg_create_rpmsgdev(mtk_subdev, &msg))
			pr_info("%s: mtk-camsys\%d\n", __func__, ipi_id);
	}
}
EXPORT_SYMBOL_GPL(mtk_create_client_msgdevice);

struct mtk_rpmsg_device *
mtk_get_client_msgdevice(struct rproc_subdev *subdev,
			    struct rpmsg_channel_info *info,
			    rpmsg_rx_cb_t cb, void *priv)
{
	struct mtk_rpmsg_rproc_subdev *mtk_subdev = to_mtk_subdev(subdev);
	struct device *dev;
	struct rpmsg_device *rpdev;
	struct mtk_rpmsg_device *mdev;
	int ret;
	u32 listen_obj_rdy;

	/* make sure put_device */
	dev = rpmsg_find_device(&mtk_subdev->pdev->dev, info);
	if (!dev)
		goto find_failed;

	rpdev = to_rpmsg_device(dev);
	if (!rpdev)
		goto get_failed;

	mdev = to_mtk_rpmsg_device(rpdev);
	if (!mdev)
		goto get_failed;

	dev_info(&mtk_subdev->pdev->dev, "%s: src:%d, %p\n",
		__func__, info->src, rpdev);

	mdev->rpdev.ept = rpmsg_create_ept(&mdev->rpdev, cb, priv, *info);

	if (IS_ERR(mdev->rpdev.ept)) {
		dev_info(&mtk_subdev->pdev->dev, "%s: creat ept faile(src:%lu)\n",
		__func__, (unsigned long)mdev->rpdev.ept);
		goto get_failed;
	}

	mutex_lock(&mtk_subdev->master_listen_lock);

	listen_obj_rdy = atomic_read(&mtk_subdev->listen_obj_rdy);
	if (listen_obj_rdy == CCD_LISTEN_OBJECT_READY) {
		mutex_unlock(&mtk_subdev->master_listen_lock);
		ret = wait_event_interruptible_timeout
			(mtk_subdev->ccd_listen_wq,
			 (atomic_read(&mtk_subdev->listen_obj_rdy) ==
			 CCD_LISTEN_OBJECT_PREPARING),
			 msecs_to_jiffies(2000));
		if (ret == 0)
			dev_info(&mtk_subdev->pdev->dev, "%s wait timeout\n", __func__);
		else if (ret < 0)
			dev_info(&mtk_subdev->pdev->dev,
				"ccd listen wait error: %d\n", ret);

		mutex_lock(&mtk_subdev->master_listen_lock);
	}

	memcpy(mtk_subdev->listen_obj.name,
	       mdev->rpdev.id.name, RPMSG_NAME_SIZE);
	mtk_subdev->listen_obj.src = mdev->rpdev.src;
	mtk_subdev->listen_obj.cmd = CCD_MASTER_CMD_CREATE;

	atomic_set(&mtk_subdev->listen_obj_rdy, CCD_LISTEN_OBJECT_READY);
	wake_up(&mtk_subdev->master_listen_wq);
	mutex_unlock(&mtk_subdev->master_listen_lock);

	put_device(dev);
	return mdev;

get_failed:
	put_device(dev);
find_failed:
	dev_info(&mtk_subdev->pdev->dev, "%s: get msgdev fail(src:%d)\n",
		__func__, info->src);

	return NULL;
}
EXPORT_SYMBOL_GPL(mtk_get_client_msgdevice);

struct rproc_subdev *
mtk_rpmsg_create_rproc_subdev(struct platform_device *pdev,
			      struct mtk_ccd_rpmsg_ops *ops)
{
	struct mtk_rpmsg_rproc_subdev *mtk_subdev;
	int i;

	mtk_subdev = kzalloc(sizeof(*mtk_subdev), GFP_KERNEL);
	if (!mtk_subdev)
		return NULL;

	mtk_subdev->pdev = pdev;
	mtk_subdev->ops = ops;

	mutex_init(&mtk_subdev->endpoints_lock);

	mutex_init(&mtk_subdev->master_listen_lock);
	atomic_set(&mtk_subdev->listen_obj_rdy, CCD_LISTEN_OBJECT_PREPARING);
	init_waitqueue_head(&mtk_subdev->master_listen_wq);
	init_waitqueue_head(&mtk_subdev->ccd_listen_wq);

	/* channels initialization */
	for (i = 0; i < CCD_IPI_MAX; i++)
		mtk_subdev->channels[i] = NULL;

	return &mtk_subdev->subdev;
}
EXPORT_SYMBOL_GPL(mtk_rpmsg_create_rproc_subdev);

void mtk_rpmsg_destroy_rproc_subdev(struct rproc_subdev *subdev)
{
	struct mtk_rpmsg_rproc_subdev *mtk_subdev = to_mtk_subdev(subdev);
	int i;

	/* channels check */
	for (i = 0; i < CCD_IPI_MAX; i++)
		if (mtk_subdev->channels[i]) {
			pr_info("%s: channel-%d is not released\n", __func__, i);
			WARN_ON(1);
		}

	kfree(mtk_subdev);
}
EXPORT_SYMBOL_GPL(mtk_rpmsg_destroy_rproc_subdev);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek ccd rpmsg driver");
