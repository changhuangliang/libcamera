/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Starfive Technology Co., Ltd.
 *
 * algo_interface.h - algorithm interface
 */
#pragma once

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

namespace ipa::starfive {

struct StarfiveFrameContext {
	uint32_t exposure;
	double gain;

	uint32_t isoLevel;
	double isoFactor;

	uint32_t tempIndex;
	double tempFactor;
};

class Algorithm
{
public:
	Algorithm(){}

	virtual ~Algorithm() {}

	virtual void parseConfiguration([[maybe_unused]] const libcamera::YamlObject &yamlObj) = 0;

	virtual void init(const void *params, ...) = 0;

	virtual void process([[maybe_unused]] const uint8_t *scBuffer, [[maybe_unused]] const StarfiveFrameContext &frameContext) = 0;

	enum ModuleType {
		modty_ctl = 0,
		modty_mod
	};
	virtual bool isEnabled(enum ModuleType mt) = 0;

	virtual void enable(enum ModuleType mt, bool en) = 0;

	virtual void getControl(enum ModuleType mt, ControlList &ctrlList) = 0;

};

//namespace Agc {
//
//} // Agc

} // namespace ipa::starfive

} // libcamera