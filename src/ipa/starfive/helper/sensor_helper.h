/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Starfive Technology Co., Ltd.
 *
 * sensor_helper.h - Starfive helper class providing sensor information
 */

#pragma once

#include <memory>
#include <string>

namespace libcamera {

namespace ipa::starfive {

class StarfiveSensorHelper
{
public:
	virtual ~StarfiveSensorHelper() {};

	virtual uint32_t gainCode(double gain) const = 0;
	virtual double gain(uint32_t gainCode) const = 0;
};

typedef StarfiveSensorHelper *(*StarfiveSensorHelperCreateFunc)();
void registerStarfiveSensorHelperFunc(std::string model, StarfiveSensorHelperCreateFunc func);

std::unique_ptr<StarfiveSensorHelper> StarfiveCreateSensorHelper(std::string model);

} // namespace ipa::starfive

} // namespace libcamera
