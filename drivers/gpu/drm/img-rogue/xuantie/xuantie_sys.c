/*************************************************************************/ /*!
@File
@Title          System Configuration
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    System Configuration functions
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/thermal.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/sysfs.h>
#include <linux/utsname.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <drm/drm_device.h>

#include "pvrsrv.h"
#include "pvr_drv.h"
#include "proc_stats.h"
#include "pvrversion.h"
#include "rgxhwperf.h"
#include "rgxinit.h"
#include "process_stats.h"
#include "xuantie_sys.h"

int xuantie_sysfs_init(struct device *dev)
{
	return 0;
}

void xuantie_sysfs_uninit(struct device *dev)
{
}

int xuantie_mfg_enable(struct gpu_plat_if *mfg)
{
    int ret;
    int val;
	ret = pm_runtime_get_sync(mfg->dev);
	/* don't check ret > 0 here for pm status maybe ACTIVE */
	if (ret < 0)
		return ret;

	xuantie_debug("xuantie_mfg_enable aclk\n");
	if (mfg->gpu_aclk) {
		ret = clk_prepare_enable(mfg->gpu_aclk);
		if (ret) {
	        xuantie_debug("xuantie_mfg_enable aclk\n");
            goto err_pm_runtime_put;
        }
	}
	if (mfg->gpu_cclk) {
		ret = clk_prepare_enable(mfg->gpu_cclk);
		if (ret) {
	        xuantie_debug("xuantie_mfg_enable cclk\n");
			clk_disable_unprepare(mfg->gpu_aclk);
            goto err_pm_runtime_put;
		}
	}

	regmap_read(mfg->vosys_regmap, 0x0, &val);
	if (val)
	{
		regmap_update_bits(mfg->vosys_regmap, 0x0, 3, 0);
		regmap_read(mfg->vosys_regmap, 0x0, &val);
		if (val) {
			pr_info("[GPU_RST]" "val is %x\r\n", val);
			clk_disable_unprepare(mfg->gpu_cclk);
			clk_disable_unprepare(mfg->gpu_aclk);
			goto err_pm_runtime_put;
		}
		udelay(1);
	}
    /* rst gpu clkgen */
    regmap_update_bits(mfg->vosys_regmap, 0x0, 2, 2);
    regmap_read(mfg->vosys_regmap, 0x0, &val);
    if (!(val & 0x2)) {
        pr_info("[GPU_CLK_RST]" "val is %x\r\n", val);
        clk_disable_unprepare(mfg->gpu_cclk);
        clk_disable_unprepare(mfg->gpu_aclk);
        goto err_pm_runtime_put;
    }
    udelay(1);
    /* rst gpu */
    regmap_update_bits(mfg->vosys_regmap, 0x0, 1, 1);
    regmap_read(mfg->vosys_regmap, 0x0, &val);
    if (!(val & 0x1)) {
        pr_info("[GPU_RST]" "val is %x\r\n", val);
        clk_disable_unprepare(mfg->gpu_cclk);
        clk_disable_unprepare(mfg->gpu_aclk);
        goto err_pm_runtime_put;
    }
	return 0;
err_pm_runtime_put:
	pm_runtime_put_sync(mfg->dev);
	return ret;
}

void xuantie_mfg_disable(struct gpu_plat_if *mfg)
{
    int val;
    regmap_update_bits(mfg->vosys_regmap, 0x0, 3, 0);
    regmap_read(mfg->vosys_regmap, 0x0, &val);
    if (val) {
        pr_info("[GPU_RST]" "val is %x\r\n", val);
        return;
    }
	if (mfg->gpu_aclk) {
		clk_disable_unprepare(mfg->gpu_aclk);
	    xuantie_debug("xuantie_mfg_disable aclk\n");
    }
	if (mfg->gpu_cclk) {
		clk_disable_unprepare(mfg->gpu_cclk);
	    xuantie_debug("xuantie_mfg_disable cclk\n");
    }

	xuantie_debug("xuantie_mfg_disable cclk\n");
	pm_runtime_put_sync(mfg->dev);
}

struct gpu_plat_if *dt_hw_init(struct device *dev)
{
	struct gpu_plat_if *mfg;

	xuantie_debug("gpu_plat_if_create Begin\n");

	mfg = devm_kzalloc(dev, sizeof(*mfg), GFP_KERNEL);
	if (!mfg)
		return ERR_PTR(-ENOMEM);
	mfg->dev = dev;

    mfg->gpu_cclk = devm_clk_get(dev, "cclk");
	if (IS_ERR(mfg->gpu_cclk)) {
		dev_err(dev, "devm_clk_get cclk failed !!!\n");
	    pm_runtime_disable(dev);
		return ERR_PTR(PTR_ERR(mfg->gpu_aclk));
	}

    mfg->gpu_aclk = devm_clk_get(dev, "aclk");
	if (IS_ERR(mfg->gpu_aclk)) {
		dev_err(dev, "devm_clk_get aclk failed !!!\n");
	    pm_runtime_disable(dev);
		return ERR_PTR(PTR_ERR(mfg->gpu_aclk));
	}

    mfg->vosys_regmap = syscon_regmap_lookup_by_phandle(dev->of_node, "vosys-regmap");
	if (IS_ERR(mfg->vosys_regmap)) {
		dev_err(dev, "syscon_regmap_lookup_by_phandle vosys-regmap failed !!!\n");
	    pm_runtime_disable(dev);
		return ERR_PTR(PTR_ERR(mfg->vosys_regmap));
	}

	mutex_init(&mfg->set_power_state);

	pm_runtime_enable(dev);

	xuantie_debug("gpu_plat_if_create End\n");

	return mfg;
}

void dt_hw_uninit(struct gpu_plat_if *mfg)
{
	pm_runtime_disable(mfg->dev);
}
