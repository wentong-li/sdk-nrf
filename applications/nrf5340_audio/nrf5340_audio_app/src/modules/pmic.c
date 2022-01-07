/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "pmic.h"

#include <zephyr.h>
#include <stdio.h>
#include <device.h>
#include <logging/log_ctrl.h>

#include "max14690.h"
#include "macros_common.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(pmic, CONFIG_LOG_PMIC_LEVEL);

int pmic_defaults_set(void)
{
	int ret;

	ret = max14690_boot_cfg_stay_on_cfg(PMIC_PWR_CFG_STAY_ON_ENABLED);
	RET_IF_ERR(ret);
	ret = max14690_mtn_chg_tmr_cfg(PMIC_MTN_CHG_TMR_60_MIN);
	RET_IF_ERR(ret);
	ret = max14690_pmic_thrm_cfg(PMIC_THRM_CFG_THERM_ENABLED_JEITA_ENABLED);
	RET_IF_ERR(ret);

	return 0;
}

int pmic_pwr_off(void)
{
	LOG_WRN("PMIC powering off");
	log_panic();
	return max14690_pwr_off();
}

int pmic_init(void)
{
	int ret;

	const struct device *i2c_dev = device_get_binding("I2C_1");

	ret = max14690_init(i2c_dev);
	RET_IF_ERR(ret);

	return 0;
}
