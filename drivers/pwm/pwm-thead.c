// SPDX-License-Identifier: GPL-2.0
/*
 * T-HEAD PWM driver
 *
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pwm.h>
#include <linux/slab.h>

#define MAX_PWM_NUM	6

#define LIGHT_PWM_CHN_BASE(n)		((n) * 0x20)
#define LIGHT_PWM_CTRL(n)		(LIGHT_PWM_CHN_BASE(n) + 0x00)
#define LIGHT_PWM_RPT(n)		(LIGHT_PWM_CHN_BASE(n) + 0x04)
#define LIGHT_PWM_PER(n)		(LIGHT_PWM_CHN_BASE(n) + 0x08)
#define LIGHT_PWM_FP(n)			(LIGHT_PWM_CHN_BASE(n) + 0x0c)
#define LIGHT_PWM_STATUS(n)		(LIGHT_PWM_CHN_BASE(n) + 0x10)

/* bit definition PWM_CTRL */
#define PWM_START				BIT(0)
#define PWM_SOFT_RST				BIT(1)
#define PWM_CFG_UPDATE				BIT(2)
#define PWM_INT_EN				BIT(3)
#define PWM_ONE_SHOT_MODE			BIT(4)
#define PWM_CONTINUOUS_MODE			BIT(5)
#define PWM_EVT_RISING_TRIG_UNDER_ONE_SHOT	BIT(6)
#define PWM_EVT_FALLING_TRIG_UNDER_ONE_SHOT	BIT(7)
#define PWM_FPOUT				BIT(8)
#define PWM_INFACTOUT				BIT(9)

struct thead_pwm_chip {
	struct clk *clk;
	void __iomem *mmio_base;
	struct pwm_chip chip;
};

#define to_thead_pwm_chip(chip)		container_of(chip, struct thead_pwm_chip, chip)

static int thead_pwm_clk_prepare_enable(struct thead_pwm_chip *pc)
{
	return clk_prepare_enable(pc->clk);
}

static void thead_pwm_clk_disable_unprepare(struct thead_pwm_chip *pc)
{
	clk_disable_unprepare(pc->clk);
}

static int thead_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct thead_pwm_chip *pc = to_thead_pwm_chip(chip);
	u32 value;
	int ret;

	ret = pm_runtime_get_sync(chip->dev);
	if (ret < 0) {
		dev_err(chip->dev, "failed to clock on the pwm device(%d)\n", ret);
		return ret;
	}

	value = readl(pc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));
	value |= PWM_START;
	writel(value, pc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));

	return 0;
}

static void thead_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct thead_pwm_chip *pc = to_thead_pwm_chip(chip);
	u32 value;

	value = readl(pc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));
	value &= ~PWM_START;
	writel(value, pc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));

	pm_runtime_put_sync(chip->dev);
}

static int thead_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			    int duty_ns, int period_ns)
{
	struct thead_pwm_chip *pc = to_thead_pwm_chip(chip);
	unsigned long rate = clk_get_rate(pc->clk);
	unsigned long duty_cycle, period_cycle;
	u32 pwm_cfg = PWM_INFACTOUT | PWM_FPOUT | PWM_CONTINUOUS_MODE | PWM_INT_EN;
	int ret;

	if (duty_ns > period_ns) {
		dev_err(chip->dev, "invalid pwm configure\n");
		return -EINVAL;
	}

	ret = pm_runtime_get_sync(chip->dev);
	if (ret < 0) {
		dev_err(chip->dev, "failed to clock on the pwm device(%d)\n", ret);
		return ret;
	}

	writel(pwm_cfg, pc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));

	period_cycle = period_ns * rate;
	do_div(period_cycle, NSEC_PER_SEC);
	writel(period_cycle, pc->mmio_base + LIGHT_PWM_PER(pwm->hwpwm));

	duty_cycle = duty_ns * rate;
	do_div(duty_cycle, NSEC_PER_SEC);
	writel(duty_cycle, pc->mmio_base + LIGHT_PWM_FP(pwm->hwpwm));

	pwm_cfg = readl(pc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));
	pwm_cfg |= PWM_CFG_UPDATE;
	writel(pwm_cfg, pc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));

	pm_runtime_put_sync(chip->dev);

	return 0;
}

static int thead_pwm_set_polarity(struct pwm_chip *chip,
				  struct pwm_device *pwm,
				  enum pwm_polarity polarity)
{
	struct thead_pwm_chip *pc = to_thead_pwm_chip(chip);
	u32 value = readl(pc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));
	int ret;

	ret = pm_runtime_get_sync(chip->dev);
	if (ret < 0) {
		dev_err(chip->dev, "failed to clock on the pwm device(%d)\n", ret);
		return ret;
	}

	if (polarity == PWM_POLARITY_NORMAL)
		value |= PWM_FPOUT;
	else
		value &= ~PWM_FPOUT;

	writel(value, pc->mmio_base + LIGHT_PWM_CTRL(pwm->hwpwm));

	pm_runtime_put_sync(chip->dev);

	return 0;
}

static int thead_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			   const struct pwm_state *state)
{
	int err;
	bool enabled = pwm->state.enabled;

	if (state->polarity != pwm->state.polarity)
		thead_pwm_set_polarity(chip, pwm, state->polarity);

	if (!state->enabled) {
		if (enabled)
			thead_pwm_disable(chip, pwm);
		return 0;
	}

	err = thead_pwm_config(chip, pwm, state->duty_cycle, state->period);
	if (err)
		return err;

	if (!enabled)
		return thead_pwm_enable(chip, pwm);

	return 0;
}

static const struct pwm_ops thead_pwm_ops = {
	.apply = thead_pwm_apply,
	.owner = THIS_MODULE,
};

static int __maybe_unused thead_pwm_runtime_suspend(struct device *dev)
{
	struct thead_pwm_chip *pc = dev_get_drvdata(dev);

	thead_pwm_clk_disable_unprepare(pc);

	return 0;
}

static int __maybe_unused thead_pwm_runtime_resume(struct device *dev)
{
	struct thead_pwm_chip *pc = dev_get_drvdata(dev);
	int ret;

	ret = thead_pwm_clk_prepare_enable(pc);
	if (ret) {
		dev_err(dev, "failed to enable pwm clock(%d)\n", ret);
		return ret;
	}

	return 0;
}

static int thead_pwm_probe(struct platform_device *pdev)
{
	struct thead_pwm_chip *pc;
	int ret;

	pc = devm_kzalloc(&pdev->dev, sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	platform_set_drvdata(pdev, pc);

	pc->mmio_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pc->mmio_base))
		return PTR_ERR(pc->mmio_base);

	pc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pc->clk))
		return PTR_ERR(pc->clk);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	ret = thead_pwm_clk_prepare_enable(pc);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable pwm clock(%d)\n", ret);
		goto err_pm_disable;
	}

	pc->chip.ops = &thead_pwm_ops;
	pc->chip.dev = &pdev->dev;
	pc->chip.npwm = MAX_PWM_NUM;

	ret = pwmchip_add(&pc->chip);
	if (ret)
		goto err_clk_disable;

	pm_runtime_put(&pdev->dev);

	return 0;

err_clk_disable:
	thead_pwm_clk_disable_unprepare(pc);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
	return ret;
}

static void thead_pwm_remove(struct platform_device *pdev)
{
	struct thead_pwm_chip *pc = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	thead_pwm_clk_disable_unprepare(pc);
	pwmchip_remove(&pc->chip);
}

static const struct of_device_id thead_pwm_dt_ids[] = {
	{.compatible = "thead,th1520-pwm",},
	{/* sentinel */}
};
MODULE_DEVICE_TABLE(of, thead_pwm_dt_ids);

static const struct dev_pm_ops thead_pwm_pm_ops = {
	SET_RUNTIME_PM_OPS(thead_pwm_runtime_suspend, thead_pwm_runtime_resume, NULL)
};

static struct platform_driver thead_pwm_driver = {
	.driver = {
		.name = "thead-pwm",
		.of_match_table = thead_pwm_dt_ids,
		.pm = &thead_pwm_pm_ops,
	},
	.probe = thead_pwm_probe,
	.remove_new = thead_pwm_remove,
};
module_platform_driver(thead_pwm_driver);

MODULE_AUTHOR("wei.liu <lw312886@linux.alibaba.com>");
MODULE_AUTHOR("Jisheng Zhang <jszhang@kernel.org>");
MODULE_DESCRIPTION("T-HEAD pwm driver");
MODULE_LICENSE("GPL v2");
