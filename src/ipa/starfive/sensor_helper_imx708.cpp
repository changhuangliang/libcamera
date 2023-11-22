/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Starfive Technology Co., Ltd.
 *
 * sensor_helper_imx708.cpp - Starfive sensor helper for imx708 sensor
 */

#include "sensor_helper.h"

namespace libcamera {

namespace ipa::starfive {

class StarfiveSensorHelperImx708 : public StarfiveSensorHelper
{
public:
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
};

uint32_t StarfiveSensorHelperImx708::gainCode(double gain) const
{
	return static_cast<uint32_t>(1024 - 1024 / gain);
}

double StarfiveSensorHelperImx708::gain(uint32_t gainCode) const
{
	return 1024.0 / (1024 - gainCode);
}

static StarfiveSensorHelper *imx708HelperCreator()
{
	return new StarfiveSensorHelperImx708;
}

static struct StarfiveSHImx708Register
{
	StarfiveSHImx708Register()
	{
		registerStarfiveSensorHelperFunc("imx708", imx708HelperCreator);
	}
} starfiveSHImx708Register;

} // namespace ipa::starfive

} // namespace libcamera
