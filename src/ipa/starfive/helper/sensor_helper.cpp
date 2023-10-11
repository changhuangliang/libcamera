/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Starfive Technology Co., Ltd.
 *
 * sensor_helper.cpp - Starfive helper class providing sensor information
 */

#include <unordered_map>
#include "sensor_helper.h"

namespace libcamera {

namespace ipa::starfive {

static std::unordered_map<std::string, StarfiveSensorHelperCreateFunc> createFuncs;
void registerStarfiveSensorHelperFunc(std::string model, StarfiveSensorHelperCreateFunc func)
{
	createFuncs[model] = func;
}

std::unique_ptr<StarfiveSensorHelper> StarfiveCreateSensorHelper(std::string model)
{
	auto funcIt = createFuncs.find(model);
	if (funcIt == createFuncs.end())
		return nullptr;

	StarfiveSensorHelperCreateFunc func = funcIt->second;
	return std::unique_ptr<StarfiveSensorHelper>(func());
}

} // namespace ipa::starfive

} // namespace libcamera
