/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018 MediaTek Inc.
 */

#ifndef _MTK_CCD_H
#define _MTK_CCD_H

#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/rpmsg.h>

struct dma_buf;
struct mtk_ccd_memory;
struct mtk_rpmsg_rproc_subdev;
struct mtk_ccd_rpmsg_endpoint;
struct ccd_master_status_item;
struct ccd_master_listen_item;
struct ccd_worker_item;
struct mtk_ccd_memory;

struct mtk_ccd_client_cb {
	/**
	 * FIXME: phase out id for channel optimization
	 * name id = ipi_id - CCD_IPI_ISP_MAIN;
	 */
	int ipi_id;
	rpmsg_rx_cb_t send_msg_ack;
	void *priv; /* point the client top struct.*/
	// int (*master_destroy)();
	// int (*worker_destroy)();
};

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

	struct rproc_subdev *rpmsg_subdev;  /* TODO: re-name */
	struct ccd_master_status master_status;
	struct mtk_ccd_memory *ccd_memory;
};

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

/* For ccd client */
int mtk_ccd_get_channel(struct mtk_ccd *ccd, unsigned int center_id,
			struct mtk_ccd_client_cb *client_cb);
int mtk_ccd_put_channel(struct mtk_ccd *ccd,
			unsigned int center_id, unsigned int channel_id);

int mtk_ccd_channel_init(struct mtk_ccd *ccd,
			 unsigned int center_id, unsigned int channel_id);
int mtk_ccd_channel_uninit(struct mtk_ccd *ccd,
			   unsigned int center_id, unsigned int channel_id);

int mtk_ccd_channel_send(struct mtk_ccd *ccd,
			 unsigned int center_id, unsigned int channel_id,
			 void *data, int len);

/* ccd memory */
void *mtk_ccd_get_buffer(struct mtk_ccd *ccd,
			 struct mem_obj *mem_buff_data);
int mtk_ccd_put_buffer(struct mtk_ccd *ccd,
			struct mem_obj *mem_buff_data);

int mtk_ccd_get_buffer_fd(struct mtk_ccd *ccd, void *mem_priv);

struct dma_buf *mtk_ccd_get_buffer_dmabuf(struct mtk_ccd *ccd,
			void *mem_priv);
#endif /* _MTK_CCD_H */
