/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Starfive Technology Co., Ltd.
 *
 * ipa_interface.h - starfive IPA interface
 */

#pragma once

#include "algo_interface.h"

namespace libcamera {

namespace ipa::starfive {

void getISPModules(std::unordered_map<std::string, std::unique_ptr<Algorithm>> &modules);

enum AE_SetEVPhase
{
    ae_sevph_1st = 0,
	ae_sevph_2nd,
};

double getLineDuration(Algorithm *agcAlgo);
void setEV(Algorithm *agcAlgo, float ev);
void resetAgc(Algorithm *agcAlgo, uint32_t preExposure, double preGain);
void getCurrentEV(Algorithm *agcAlgo, uint32_t &exposure, double &gain, enum AE_SetEVPhase phase);
void getISO(Algorithm *agcAlgo, uint32_t & level, double & factor);

void getColorTemperatureInfo(Algorithm *awbAlgo, uint32_t & tempIndex, double & tempFactor);

} // namespace ipa::starfive

} // namespace libcamera
