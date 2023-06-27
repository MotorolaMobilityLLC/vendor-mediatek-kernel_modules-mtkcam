/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018 MediaTek Inc.
 *
 * Author: Frederic Chen <frederic.chen@mediatek.com>
 *         Holmes Chiou <holmes.chiou@mediatek.com>
 *
 */

#ifndef _MTK_IMGSYS_DATA_H_
#define _MTK_IMGSYS_DATA_H_

#include "mtk_imgsys-of.h"
/* For ISP 7 */
#include "mtk_imgsys-modops.h"
#include "mtk_imgsys-plat.h"
#include "mtk_imgsys_v4l2_vnode.h"
#include "mtk_imgsys-debug.h"
/* For ISP 7.1 */

/* imgsys_data is a keyword, please don't rename */
const struct cust_data imgsys_data_mt6897[] = {
	/* ISP 7 */
	[0] = {
	.clks = imgsys_isp7_clks_mt6897,
	.clk_num = MTK_IMGSYS_CLK_NUM_MT6897,
	.module_pipes = module_pipe_isp7,
	.mod_num = ARRAY_SIZE(module_pipe_isp7),
	.pipe_settings = pipe_settings_isp7,
	.pipe_num = ARRAY_SIZE(pipe_settings_isp7),
	.imgsys_modules = imgsys_isp7_modules,
	.imgsys_modules_num = MTK_IMGSYS_MODULE_NUM,
	.dump = imgsys_debug_dump_routine,
	},
	/* ISP 7.1 */

};

const struct cust_data imgsys_data_mt6989[] = {
	[0] = {
	.clks = imgsys_isp7_clks_mt6989,
	.clk_num = MTK_IMGSYS_CLK_NUM_MT6989,
	.module_pipes = module_pipe_isp7,
	.mod_num = ARRAY_SIZE(module_pipe_isp7),
	.pipe_settings = pipe_settings_isp7,
	.pipe_num = ARRAY_SIZE(pipe_settings_isp7),
	.imgsys_modules = imgsys_isp7_modules,
	.imgsys_modules_num = MTK_IMGSYS_MODULE_NUM,
	.dump = imgsys_debug_dump_routine,
	},
};
#endif /* _MTK_IMGSYS_DATA_H_ */

