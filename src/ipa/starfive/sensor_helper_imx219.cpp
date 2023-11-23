/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Starfive Technology Co., Ltd.
 *
 * sensor_helper_imx219.cpp - Starfive sensor helper for imx219 sensor
 */

#include "sensor_helper.h"

namespace libcamera {

namespace ipa::starfive {

class StarfiveSensorHelperImx219 : public StarfiveSensorHelper
{
public:
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
};

uint32_t StarfiveSensorHelperImx219::gainCode(double gain) const
{
	return (uint32_t)(256 - 256 / gain);
}

double StarfiveSensorHelperImx219::gain(uint32_t gainCode) const
{
	return 256.0 / (256 - gainCode);
}

static StarfiveSensorHelper *imx219HelperCreator()
{
	return new StarfiveSensorHelperImx219;
}

static struct StarfiveSHImx219Register
{
	StarfiveSHImx219Register()
	{
		registerStarfiveSensorHelperFunc("imx219", imx219HelperCreator);
	}
} starfiveSHImx219Register;

} // namespace ipa::starfive

} // namespace libcamera
