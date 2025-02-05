/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __MTK_CAM_PDA_H
#define __MTK_CAM_PDA_H

#include <linux/kfifo.h>
#include <linux/suspend.h>

#include "mtk_cam-plat.h"
#include "mtk_cam-engine.h"

#define PDA_IRQ_NUM 1

struct mtk_pda_device {
	struct device *dev;
	struct mtk_cam_device *cam;
	unsigned int id;
	int irq[PDA_IRQ_NUM];
	void __iomem *base;
	void __iomem *base_inner;
	unsigned int num_clks;
	struct clk **clks;

	int fifo_size;
	void *msg_buffer;
	struct kfifo msg_fifo;
	atomic_t is_fifo_overflow;

#ifdef CONFIG_PM_SLEEP
	struct notifier_block notifier_blk;
#endif
	/* mmqos */
	struct mtk_camsys_qos qos;
};
int mtk_cam_pda_dev_config(struct mtk_pda_device *pda_dev);
void pda_reset(struct mtk_pda_device *pda_dev);
int mtk_pda_runtime_suspend(struct device *dev);
int mtk_pda_runtime_resume(struct device *dev);
extern struct platform_driver mtk_cam_pda_driver;
#endif
