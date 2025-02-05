// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2019 MediaTek Inc.

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/vmalloc.h>

#include <soc/mediatek/smi.h>

#include "mtk_cam.h"
#include "mtk_cam-pda.h"
#include "mtk_cam-trace.h"
#include "mtk_cam-plat.h"

#include "iommu_debug.h"

static const struct of_device_id mtk_pda_of_ids[] = {
	{.compatible = "mediatek,pda",},
	{}
};
MODULE_DEVICE_TABLE(of, mtk_pda_of_ids);

int mtk_pda_translation_fault_callback(int port, dma_addr_t mva, void *data)
{
	//struct mtk_pda_device *pda_dev = (struct mtk_pda_device *)data;

	/*po-hao todo*/
	return 0;
}
static int reset_msgfifo(struct mtk_pda_device *pda_dev)
{
	atomic_set(&pda_dev->is_fifo_overflow, 0);
	return kfifo_init(&pda_dev->msg_fifo, pda_dev->msg_buffer, pda_dev->fifo_size);
}

int mtk_cam_pda_dev_config(struct mtk_pda_device *pda_dev)
{
	/*po-hao todo*/
	return 0;
}
static irqreturn_t mtk_irq_pda(int irq, void *data)
{
	/* po-hao todo*/
	return IRQ_WAKE_THREAD;
}

static irqreturn_t mtk_thread_irq_pda(int irq, void *data)
{
	/* po-hao todo */
	return IRQ_WAKE_THREAD;
}

static int mtk_pda_pm_suspend(struct device *dev)
{
	struct mtk_pda_device *pda_dev = dev_get_drvdata(dev);
	int ret = 0;

	dev_info_ratelimited(dev, "- %s\n", __func__);

	if (pm_runtime_suspended(dev))
		return 0;

	/* Force ISP HW to idle */
	ret = pm_runtime_put_sync(dev);
	return ret;
}

static int mtk_pda_pm_resume(struct device *dev)
{
	//struct mtk_pda_device *pda_dev = dev_get_drvdata(dev);
	int ret = 0;

	dev_info_ratelimited(dev, "- %s\n", __func__);

	if (pm_runtime_suspended(dev))
		return 0;

	/* Force ISP HW to resume */
	ret = pm_runtime_get_sync(dev);
	if (ret)
		return ret;


	return 0;
}

static int mtk_pda_suspend_pm_event(struct notifier_block *notifier,
			unsigned long pm_event, void *unused)
{
	struct mtk_pda_device *pda_dev =
		container_of(notifier, struct mtk_pda_device, notifier_blk);
	struct device *dev = pda_dev->dev;

	switch (pm_event) {
	case PM_HIBERNATION_PREPARE:
		return NOTIFY_DONE;
	case PM_RESTORE_PREPARE:
		return NOTIFY_DONE;
	case PM_POST_HIBERNATION:
		return NOTIFY_DONE;
	case PM_SUSPEND_PREPARE: /* before enter suspend */
		mtk_pda_pm_suspend(dev);
		return NOTIFY_DONE;
	case PM_POST_SUSPEND: /* after resume */
		mtk_pda_pm_resume(dev);
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}
static int mtk_pda_of_probe(struct platform_device *pdev,
			    struct mtk_pda_device *pda_dev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *larb_pdev;
	struct device_node *larb_node;
	struct of_phandle_args args;
	struct device_link *link;
	struct resource *res;
	unsigned int i;
	int ret, num_clks, num_larbs, num_ports, smmus;

	ret = of_property_read_u32(dev->of_node, "mediatek,pda-id",
						       &pda_dev->id);
	if (ret) {
		dev_dbg(dev, "missing pda id property\n");
		return ret;
	}

	/* base outer register */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "base");
	if (!res) {
		dev_info(dev, "failed to get mem\n");
		return -ENODEV;
	}

	pda_dev->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pda_dev->base)) {
		dev_dbg(dev, "failed to map register base\n");
		return PTR_ERR(pda_dev->base);
	}
	dev_dbg(dev, "pda, map_addr=0x%pK\n", pda_dev->base);

	/* base inner register */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "inner_base");
	if (!res) {
		dev_dbg(dev, "failed to get mem\n");
		return -ENODEV;
	}

	pda_dev->base_inner = devm_ioremap_resource(dev, res);
	if (IS_ERR(pda_dev->base_inner)) {
		dev_dbg(dev, "failed to map register inner base\n");
		return PTR_ERR(pda_dev->base_inner);
	}

	dev_dbg(dev, "pda, map_addr=0x%pK\n", pda_dev->base_inner);

	for (i = 0; i < PDA_IRQ_NUM; i++) {
		pda_dev->irq[i] = platform_get_irq(pdev, i);
		if (!pda_dev->irq[i]) {
			dev_dbg(dev, "failed to get irq\n");
			return -ENODEV;
		}
	}

	ret = devm_request_threaded_irq(dev, pda_dev->irq[i],
				mtk_irq_pda,
				mtk_thread_irq_pda,
				0, dev_name(dev), pda_dev);

	if (ret) {
		dev_dbg(dev, "failed to request irq=%d\n", pda_dev->irq[i]);
		return ret;
	}

	dev_info(dev, "registered irq=%d\n", pda_dev->irq[i]);

	disable_irq(pda_dev->irq[i]);

	dev_info(dev, "%s:disable irq %d\n", __func__, pda_dev->irq[i]);


	num_clks = of_count_phandle_with_args(pdev->dev.of_node, "clocks",
			"#clock-cells");

	pda_dev->num_clks = (num_clks < 0) ? 0 : num_clks;
	dev_info(dev, "clk_num:%d\n", pda_dev->num_clks);

	if (pda_dev->num_clks) {
		pda_dev->clks = devm_kcalloc(dev, pda_dev->num_clks, sizeof(*pda_dev->clks),
					 GFP_KERNEL);
		if (!pda_dev->clks)
			return -ENOMEM;
	}

	for (i = 0; i < pda_dev->num_clks; i++) {
		pda_dev->clks[i] = of_clk_get(pdev->dev.of_node, i);
		if (IS_ERR(pda_dev->clks[i])) {
			dev_info(dev, "failed to get clk %d\n", i);
			return -ENODEV;
		}
	}

	num_larbs = of_count_phandle_with_args(
					pdev->dev.of_node, "mediatek,larbs", NULL);
	num_larbs = (num_larbs < 0) ? 0 : num_larbs;
	dev_info(dev, "larb_num:%d\n", num_larbs);

	for (i = 0; i < num_larbs; i++) {
		larb_node = of_parse_phandle(
					pdev->dev.of_node, "mediatek,larbs", i);
		if (!larb_node) {
			dev_info(dev, "failed to get larb node\n");
			continue;
		}

		larb_pdev = of_find_device_by_node(larb_node);
		if (WARN_ON(!larb_pdev)) {
			of_node_put(larb_node);
			dev_info(dev, "failed to get larb pdev\n");
			continue;
		}
		of_node_put(larb_node);

		link = device_link_add(&pdev->dev, &larb_pdev->dev,
						DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
		if (!link)
			dev_info(dev, "unable to link smi larb%d\n", i);
	}

	num_ports = of_count_phandle_with_args(
					pdev->dev.of_node, "iommus", "#iommu-cells");
	num_ports = (num_ports < 0) ? 0 : num_ports;
	dev_info(dev, "port_num:%d\n", num_ports);

	for (i = 0; i < num_ports; i++) {
		if (!of_parse_phandle_with_args(pdev->dev.of_node,
			"iommus", "#iommu-cells", i, &args)) {
			mtk_iommu_register_fault_callback(
				args.args[0],
				mtk_pda_translation_fault_callback,
				(void *)pda_dev, false);
		}
	}

	smmus = of_property_count_u32_elems(
		pdev->dev.of_node, "mediatek,smmu-dma-axid");
	smmus = (smmus > 0) ? smmus : 0;
	dev_info(dev, "smmu_num:%d\n", smmus);
	for (i = 0; i < smmus; i++) {
		u32 axid;

		if (!of_property_read_u32_index(
			pdev->dev.of_node, "mediatek,smmu-dma-axid", i, &axid)) {
			mtk_iommu_register_fault_callback(
				axid, mtk_pda_translation_fault_callback,
				(void *)pda_dev, false);
		}
	}
#ifdef CONFIG_PM_SLEEP
	pda_dev->notifier_blk.notifier_call = mtk_pda_suspend_pm_event;
	pda_dev->notifier_blk.priority = 0;
	ret = register_pm_notifier(&pda_dev->notifier_blk);
	if (ret) {
		dev_info(dev, "Failed to register PM notifier");
		return -ENODEV;
	}
#endif
	return 0;
}

static int mtk_pda_component_bind(
	struct device *dev,
	struct device *master,
	void *data)
{
	struct mtk_pda_device *pda_dev = dev_get_drvdata(dev);
	struct mtk_cam_device *cam_dev = data;

	pda_dev->cam = cam_dev;
	return mtk_cam_set_dev_pda(cam_dev->dev, pda_dev->id, dev);
}

static void mtk_pda_component_unbind(
	struct device *dev,
	struct device *master,
	void *data)
{
}

static const struct component_ops mtk_pda_component_ops = {
	.bind = mtk_pda_component_bind,
	.unbind = mtk_pda_component_unbind,
};

static int mtk_pda_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_pda_device *pda_dev;
	int ret;

	pda_dev = devm_kzalloc(dev, sizeof(*pda_dev), GFP_KERNEL);
	if (!pda_dev)
		return -ENOMEM;

	pda_dev->dev = dev;
	dev_set_drvdata(dev, pda_dev);

	ret = mtk_pda_of_probe(pdev, pda_dev);
	if (ret)
		return ret;


	ret = mtk_cam_qos_probe(dev, &pda_dev->qos, SMI_PORT_PDA_NUM);
	if (ret)
		goto UNREGISTER_PM_NOTIFIER;


	pda_dev->fifo_size =
		roundup_pow_of_two(8 * sizeof(struct mtk_camsys_irq_info));
	pda_dev->msg_buffer = devm_kzalloc(dev, pda_dev->fifo_size,
					     GFP_KERNEL);
	if (!pda_dev->msg_buffer) {
		ret = -ENOMEM;
		goto UNREGISTER_PM_NOTIFIER;
	}

	pm_runtime_enable(dev);

	ret = component_add(dev, &mtk_pda_component_ops);

	if (ret)
		goto UNREGISTER_PM_NOTIFIER;

	return ret;

UNREGISTER_PM_NOTIFIER:
	unregister_pm_notifier(&pda_dev->notifier_blk);
	return ret;
}

static void mtk_pda_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_pda_device *pda_dev = dev_get_drvdata(dev);

#ifdef CONFIG_PM_SLEEP
	unregister_pm_notifier(&pda_dev->notifier_blk);
#endif
	pm_runtime_disable(dev);

	component_del(dev, &mtk_pda_component_ops);

}

int mtk_pda_runtime_suspend(struct device *dev)
{
	struct mtk_pda_device *pda_dev = dev_get_drvdata(dev);
	int i;

	dev_dbg(dev, "%s:disable clock\n", __func__);

	for (i = pda_dev->num_clks - 1; i >= 0; i--)
		clk_disable_unprepare(pda_dev->clks[i]);

	return 0;
}
void pda_reset(struct mtk_pda_device *pda_dev)
{
	/* po-hao todo*/
}
int mtk_pda_runtime_resume(struct device *dev)
{
	struct mtk_pda_device *pda_dev = dev_get_drvdata(dev);
	int i, ret;

	/* reset_msgfifo before enable_irq */
	ret = reset_msgfifo(pda_dev);
	if (ret)
		return ret;

	dev_dbg(dev, "%s:enable clock\n", __func__);
	for (i = 0; i < pda_dev->num_clks; i++) {
		ret = clk_prepare_enable(pda_dev->clks[i]);
		if (ret) {
			dev_info(dev, "enable failed at clk #%d, ret = %d\n",
				 i, ret);
			i--;
			while (i >= 0)
				clk_disable_unprepare(pda_dev->clks[i--]);

			return ret;
		}
	}

	pda_reset(pda_dev);

	for (i = 0; i < PDA_IRQ_NUM; i++) {
		enable_irq(pda_dev->irq[i]);
		dev_dbg(dev, "%s:enable irq %d\n", __func__, pda_dev->irq[i]);
	}

	dev_info(dev, "%s:enable irq\n", __func__);

	return 0;
}

static const struct dev_pm_ops mtk_pda_pm_ops = {
	SET_RUNTIME_PM_OPS(mtk_pda_runtime_suspend, mtk_pda_runtime_resume,
			   NULL)
};

struct platform_driver mtk_cam_pda_driver = {
	.probe   = mtk_pda_probe,
	.remove  = mtk_pda_remove,
	.driver  = {
		.name  = "mtk-cam pda",
		.of_match_table = of_match_ptr(mtk_pda_of_ids),
		.pm     = &mtk_pda_pm_ops,
	}
};
