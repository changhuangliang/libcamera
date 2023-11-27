/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Starfive Technology Co., Ltd.
 *
 * ipa_interface.cpp - starfive IPA interface
 */

#include <unordered_map>
#include <iostream>
#include <string>
#include "../ipa_interface.h"

namespace libcamera {

namespace ipa::starfive {

void getISPModules([[maybe_unused]] std::unordered_map<std::string, std::unique_ptr<Algorithm>> &modules)
{
    modules.clear();
}

double getLineDuration([[maybe_unused]] Algorithm *agcAlgo)
{
    return 1.f;
}

void setEV([[maybe_unused]] Algorithm *agcAlgo, [[maybe_unused]] float ev)
{
}

void resetAgc([[maybe_unused]] Algorithm *agcAlgo, [[maybe_unused]] uint32_t preExposure, [[maybe_unused]] double preGain)
{
}

void getCurrentEV([[maybe_unused]] Algorithm *agcAlgo, [[maybe_unused]] uint32_t &exposure, [[maybe_unused]] double &gain, [[maybe_unused]] enum AE_SetEVPhase phase)
{
}

void getISO([[maybe_unused]] Algorithm *agcAlgo, [[maybe_unused]] uint32_t & level, [[maybe_unused]] double & factor)
{
}

void getColorTemperatureInfo([[maybe_unused]] Algorithm *awbAlgo, [[maybe_unused]] uint32_t & tempIndex, [[maybe_unused]] double & tempFactor)
{
}

} // namespace ipa::starfive

} // namespace libcamera

