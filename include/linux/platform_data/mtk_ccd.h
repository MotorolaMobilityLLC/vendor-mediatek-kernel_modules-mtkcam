/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018 MediaTek Inc.
 */

#ifndef _MTK_CCD_H
#define _MTK_CCD_H

#include <linux/platform_device.h>
#include <linux/cdev.h>

struct dma_buf;
struct mtk_ccd_memory;
struct mtk_rpmsg_rproc_subdev;
struct mtk_ccd_rpmsg_endpoint;
struct ccd_master_status_item;
struct ccd_master_listen_item;
struct ccd_worker_item;
struct mtk_ccd_memory;

/**
 * struct mem_obj - memory buffer allocated in kernel
 *
 * @iova:	iova of buffer
 * @len:	buffer length
 * @va: kernel virtual address
 */
struct mem_obj {
	dma_addr_t iova;
	unsigned int len;
	void *va;
};

struct ccd_master_status {
	unsigned int state;
};

struct mtk_ccd {
	struct device *dev;
	struct device *smmu_dev;
	struct rproc *rproc;

	dev_t ccd_devno;
	struct cdev ccd_cdev;
	struct class *ccd_class;

	struct rproc_subdev *rpmsg_subdev;
	struct ccd_master_status master_status;
	struct mtk_ccd_memory *ccd_memory;
};

/**
 * rpmsg_ccd_ipi_send - send data from AP to ccd.
 *
 * @pdev:	CCD platform device
 * @id:		IPI ID
 * @buf:	the data buffer
 * @len:	the data buffer length
 * @wait:	1: need ack
 *
 * This function is thread-safe. When this function returns,
 * CCD has received the data and starts the processing.
 * When the processing completes, IPI handler registered
 * by ccd_ipi_register will be called in interrupt context.
 *
 * Return: Return 0 if sending data successfully, otherwise it is failed.
 **/
int rpmsg_ccd_ipi_send(struct mtk_rpmsg_rproc_subdev *mtk_subdev,
		       struct mtk_ccd_rpmsg_endpoint *mept,
		       void *buf, unsigned int len, unsigned int wait);

void ccd_master_listen(struct mtk_ccd *ccd,
			      struct ccd_master_listen_item *listen_obj);

void ccd_master_destroy(struct mtk_ccd *ccd,
			struct ccd_master_status_item *master_obj);

int ccd_worker_read(struct mtk_ccd *ccd,
		     struct ccd_worker_item *read_obj);

void ccd_worker_write(struct mtk_ccd *ccd,
		      struct ccd_worker_item *write_obj);

void mtk_ccd_get_service(struct mtk_ccd *ccd,
			 struct task_struct **task,
			 struct files_struct **f);

void *mtk_ccd_get_buffer(struct mtk_ccd *ccd,
			 struct mem_obj *mem_buff_data);
int mtk_ccd_put_buffer(struct mtk_ccd *ccd,
			struct mem_obj *mem_buff_data);

int mtk_ccd_get_buffer_fd(struct mtk_ccd *ccd, void *mem_priv);

struct dma_buf *mtk_ccd_get_buffer_dmabuf(struct mtk_ccd *ccd,
			void *mem_priv);
#endif /* _MTK_CCD_H */
