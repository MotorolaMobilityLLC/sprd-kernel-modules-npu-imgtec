/*
 * Chip depended functions for qogirn6pro.
 *
 * Copyright (C) 2018 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/pm_domain.h>
#include <linux/pm_wakeup.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>

#include "vha_chipdep.h"
#include "vha_common.h"
#include <dt-bindings/soc/sprd,qogirn6pro-regs.h>
#include <dt-bindings/soc/sprd,qogirn6pro-mask.h>
#include "sprd,qogirn6pro-npu-mask.h"
#include "sprd,qogirn6pro-npu-regs.h"

struct npu_pm_domain {
	struct generic_pm_domain pd;
	struct device *dev;
	struct regmap *pmu_apb_regs;
	union {
		u32 args[2];
		struct {
			u32 cfg_regoff, cfg_mask;
		} s;
	} u;
};

struct npu_regmap{
	struct regmap *ai_apb_regs;
	struct regmap *ai_clock_regs;
	struct regmap *ai_dvfs_regs;
	struct regmap *ai_mtx_regs;
	struct regmap *ai_busmon_regs;
	struct regmap *ai_pd_regs;
};
static struct npu_regmap ai_regs;

struct ai_qos {
	unsigned int arqos_threshold;
	unsigned int awqos_threshold;
	unsigned int arqos_powervr;
	unsigned int awqos_powervr;
};
static struct ai_qos vha_ai_qos;
#define QOS_BITS_MASK   (0xf)

static inline unsigned int vha_get_mask_shift(unsigned int mask)
{
	unsigned int count = 0;

	while (!(mask & 1)) {
		mask = mask >> 1;
		count++;
	}

	return count;
}

static int vha_set_qos(struct device *dev)
{
	unsigned int val, mask;
	unsigned int arqos_threshold_shift =
		vha_get_mask_shift(MASK_AI_APB_ARQOS_THRESHOLD_AI_MAIN_MTX);
	unsigned int awqos_threshold_shift =
		vha_get_mask_shift(MASK_AI_APB_AWQOS_THRESHOLD_AI_MAIN_MTX);
	unsigned int arqos_powervr_shift =
		vha_get_mask_shift(MASK_AI_APB_ARQOS_POWERVR_SW_AI_MAIN_MTX);
	unsigned int awqos_powervr_shift =
		vha_get_mask_shift(MASK_AI_APB_AWQOS_POWERVR_SW_AI_MAIN_MTX);

	val = (vha_ai_qos.arqos_threshold << arqos_threshold_shift)
		| (vha_ai_qos.awqos_threshold << awqos_threshold_shift)
		| (vha_ai_qos.arqos_powervr << arqos_powervr_shift)
		| (vha_ai_qos.awqos_powervr << awqos_powervr_shift)
		| MASK_AI_APB_QOS_POWERVR_SEL_SW_AI_MAIN_MTX;

	mask = (MASK_AI_APB_ARQOS_THRESHOLD_AI_MAIN_MTX
                | MASK_AI_APB_AWQOS_THRESHOLD_AI_MAIN_MTX
                | MASK_AI_APB_ARQOS_POWERVR_SW_AI_MAIN_MTX
                | MASK_AI_APB_AWQOS_POWERVR_SW_AI_MAIN_MTX
		| MASK_AI_APB_QOS_POWERVR_SEL_SW_AI_MAIN_MTX);

	regmap_update_bits(ai_regs.ai_apb_regs, REG_AI_APB_AI_QOS_CTRL, mask, val);

	return 0;
}

static int vha_qos_init(struct device *dev)
{
	struct device_node *qos_node = NULL;
	struct device_node *vha_node = dev->of_node;
	unsigned char val;

	qos_node = of_parse_phandle(vha_node, "sprd,qos", 0);
	if (qos_node) {
		if (of_property_read_u8(qos_node, "arqos-threshold-ai", &val)) {
			pr_warn("arqos-threshold-ai reading fail.\n");
			val = 0xf;
		}

		vha_ai_qos.arqos_threshold = ((unsigned int)val) & QOS_BITS_MASK;

		if (of_property_read_u8(qos_node, "awqos-threshold-ai", &val)) {
			pr_warn("awqos-threshold-ai reading fail.\n");
			val = 0xf;
		}

		vha_ai_qos.awqos_threshold = ((unsigned int)val) & QOS_BITS_MASK;

		if (of_property_read_u8(qos_node, "arqos-powervr", &val)) {
			pr_warn("arqos-powervr reading fail.\n");
			val = 0x6;
		}

		vha_ai_qos.arqos_powervr = ((unsigned int)val) & QOS_BITS_MASK;

		if (of_property_read_u8(qos_node, "awqos-powervr", &val)) {
			pr_warn("awqos-powervr reading fail.\n");
			val = 0x6;
		}

		vha_ai_qos.awqos_powervr = ((unsigned int)val) & QOS_BITS_MASK;

	} else {
		pr_warn("%s fail to get ai_qos node\n", __func__);
		vha_ai_qos.arqos_threshold = 0xf;
		vha_ai_qos.awqos_threshold = 0xf;
		vha_ai_qos.arqos_powervr = 0x6;
		vha_ai_qos.awqos_powervr = 0x6;
	}


	return 0;
}

static int vha_init_regs(struct device *dev)
{
	struct device_node *np = dev->of_node;

	ai_regs.ai_apb_regs = syscon_regmap_lookup_by_name(np, "apb_reg");
	if (IS_ERR(ai_regs.ai_apb_regs)) {
		dev_err_once(dev, "failed to get apb_reg\n");
		return -EINVAL;
	}

	ai_regs.ai_clock_regs = syscon_regmap_lookup_by_name(np, "clock_reg");
	if (IS_ERR(ai_regs.ai_clock_regs)) {
		dev_err_once(dev, "failed to get clock_reg\n");
		return -EINVAL;
	}

	ai_regs.ai_dvfs_regs = syscon_regmap_lookup_by_name(np, "dvfs_reg");
	if (IS_ERR(ai_regs.ai_dvfs_regs)) {
		dev_err_once(dev, "failed to get dvfs_reg\n");
		return -EINVAL;
	}

	ai_regs.ai_mtx_regs = syscon_regmap_lookup_by_name(np, "mtx_reg");
	if (IS_ERR(ai_regs.ai_mtx_regs)) {
		dev_err_once(dev, "failed to get mtx_reg\n");
		return -EINVAL;
	}

	ai_regs.ai_busmon_regs = syscon_regmap_lookup_by_name(np, "busmon");
	if (IS_ERR(ai_regs.ai_busmon_regs)) {
		dev_err_once(dev, "failed to get busmon_reg\n");
		return -EINVAL;
	}

	ai_regs.ai_pd_regs = syscon_regmap_lookup_by_name(np, "pd_ai_sys");
	if (IS_ERR(ai_regs.ai_pd_regs)) {
		dev_err_once(dev, "failed to get pd_ai_reg\n");
		return -EINVAL;
	}
	return 0;
}

static int vha_power_on(struct npu_pm_domain *pd)
{
	uint32_t reg, mask;
	reg = pd->u.s.cfg_regoff;
	mask = pd->u.s.cfg_mask;

	regmap_update_bits(pd->pmu_apb_regs, reg, mask, 0);

	return 0;
}

static int vha_power_off(struct npu_pm_domain *pd)
{
	uint32_t reg, mask;
	reg = pd->u.s.cfg_regoff;
	mask = pd->u.s.cfg_mask;

	regmap_update_bits(pd->pmu_apb_regs, reg, mask, mask);

	return 0;
}

static struct npu_pm_domain pd_ai_sys;
static int vha_powerdomain_init(struct device *dev)
{
	int err;
	struct device_node *np = dev->of_node;
	struct npu_pm_domain *pd = &pd_ai_sys;
	dev_info(dev, "domain_np fullname %s\n", np->full_name);
	pd->pmu_apb_regs = ai_regs.ai_pd_regs; 

	err = syscon_get_args_by_name(np, "pd_ai_sys", 2, pd->u.args);
	if (err != 2) {
		dev_err_once(dev, "failed to parse domain cfg reg\n");
		return -EINVAL;
	}
	pd->dev = dev;

	return 0;
}

#define  LOOP_TIME  34  // (2000us - 300 us)/50us =34 times
static int vha_wait_power_ready(struct npu_pm_domain *pd)
{
	uint32_t ret, mask, reg, val, loop=0;

	reg = REG_PMU_APB_PWR_STATUS_DBG_10;
	mask = MASK_PMU_APB_PD_AI_STATE;

	/* delay for setup power domain */
	udelay(300);
	/* check setup if ok*/
	if (pd->pmu_apb_regs) {
		while(loop < LOOP_TIME){
			ret = regmap_read(pd->pmu_apb_regs,reg, &val);
			if (ret){
				dev_err_once(pd->dev, "read pmu_apb_regs error, regs 0x%x\n", reg);
				return ret;
			}

			if ((val & mask) == 0){
				return 0;
			}

			loop++;
			udelay(50);
		}
	}
	else {
		dev_err_once(pd->dev, "pmu_apb_regs uninit\n");
	}

	dev_err_once(pd->dev, "check pmu_apb_regs regs 0x%x  timeout!!!\n", reg);
	return 1;
}

static int vha_powerdomain_setup(void)
{
	vha_power_on(&pd_ai_sys);
	vha_wait_power_ready(&pd_ai_sys);

	return 0;
}

static int vha_powerdomain_unsetup(void)
{
	vha_power_off(&pd_ai_sys);

	return 0;
}

static int vha_auto_ckg_setup(struct device *dev)
{
	unsigned int mask;

	mask =	MASK_AI_APB_POWERVR_NNA_AUTO_GATE_EN |
		MASK_AI_APB_POWERVR_BUSMON_AUTO_GATE_EN |
		MASK_AI_APB_MAIN_MTX_FR_AUTO_GATE_EN |
		MASK_AI_APB_CFG_MTX_FR_AUTO_GATE_EN |
		MASK_AI_APB_OCM_FR_AUTO_GATE_EN |
		MASK_AI_APB_DVFS_FR_AUTO_GATE_EN;

	regmap_update_bits(ai_regs.ai_apb_regs, REG_AI_APB_USER_GATE_AUTO_GATE_EN, mask, mask);

	return 0;
}

static struct clk_bulk_data clks[] = {
	[0].id = "ai_apll",
	[1].id = "ai_eb",
	[2].id = "powervr_eb",
	[3].id = "mtx_eb",
	[4].id = "dvfs_eb",
	[5].id = "ocm_eb",
	/* pmon_eb: performance monitor enable, default is disabled*/
	/* aon_to_ocm_eb: aon to ocm path enable, default is disabled*/
};

static int vha_clockdomain_init(struct device *dev)
{
	int num_clks = ARRAY_SIZE(clks);
	int ret;

	ret = clk_bulk_get(dev, num_clks, &clks[0]);
	if (ret) {
		dev_err(dev, "clk_bulk_get failed, ret %d\n", ret);
		return ret;
	}

	return 0;
}

static int vha_clockdomain_setup(struct device *dev)
{
	int num_clks = ARRAY_SIZE(clks);
	clk_bulk_prepare_enable(num_clks, clks);

	vha_auto_ckg_setup(dev);

	return 0;
}

static int vha_clockdomain_unsetup(struct device *dev)
{
	int num_clks = ARRAY_SIZE(clks);

	clk_bulk_disable_unprepare(num_clks, clks);

	return 0;
}

int vha_chip_init(struct device *dev)
{
	device_set_wakeup_capable(dev, true);
	device_wakeup_enable(dev);
	vha_init_regs(dev);
	vha_powerdomain_init(dev);
	vha_powerdomain_setup();
	vha_clockdomain_init(dev);
	vha_qos_init(dev);

	return 0;
}

int vha_chip_deinit(struct device *dev)
{
	device_wakeup_disable(dev);

	return 0;
}

int vha_chip_runtime_resume(struct device *dev)
{
	vha_powerdomain_setup();
	vha_clockdomain_setup(dev);
	vha_set_qos(dev);
#ifdef VHA_DEVFREQ
	vha_devfreq_resume(dev);
#endif

	return 0;
}

int vha_chip_runtime_suspend(struct device *dev)
{
#ifdef VHA_DEVFREQ
	vha_devfreq_suspend(dev);
#endif
	vha_clockdomain_unsetup(dev);
	vha_powerdomain_unsetup();

	return 0;
}

#define VHA_N6PRO_DMAMASK DMA_BIT_MASK(35)
u64 vha_get_chip_dmamask(void)
{
	return VHA_N6PRO_DMAMASK;
}

