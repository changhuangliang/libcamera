/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Starfive Technology Co., Ltd.
 *
 * starfive.cpp - Starfive Image Processing Algorithms
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <stdint.h>
#include <utility>
#include <vector>
#include <ratio>
#include <linux/v4l2-controls.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/framebuffer.h>
#include <libcamera/request.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/starfive_ipa_interface.h>

#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/yaml_parser.h"

#include "helper/sensor_helper.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASTARFIVE)

using namespace std::literals::chrono_literals;

namespace ipa::starfive {

class IPASTARFIVE : public IPAStarfiveInterface
{
public:
	IPASTARFIVE()
		: sensorISPEnable_(false), hardwareInited_(true)
	{
	};
	~IPASTARFIVE(){};

	int init(const IPASettings &settings, const IPACameraSensorInfo &sensorInfo, 
		const ControlInfoMap &sensorControls) override;
	int start(const ControlList &controls) override;
	void stop() override {}

	int configure(const ControlInfoMap &ispControls, const ControlInfoMap &sensorControls) override;

	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;

	void queueRequest(const ControlList &controls) override;
	void statBufferReady(const uint32_t bufferId, const ControlList &sensorControls) override;

	bool isSensorISPEnabled() { return sensorISPEnable_; }

private:
	bool sensorISPEnable_;
	bool hardwareInited_;

	std::map<uint32_t, MappedFrameBuffer> scBufferMaps_;
};

int IPASTARFIVE::init([[maybe_unused]] const IPASettings &settings, 
	[[maybe_unused]] const IPACameraSensorInfo &sensorInfo, 
	[[maybe_unused]] const ControlInfoMap &sensorControls)
{
	return 0;
}

int IPASTARFIVE::start([[maybe_unused]] const ControlList &controls)
{
	return 0;
}

int IPASTARFIVE::configure([[maybe_unused]] const ControlInfoMap &ispControls, 
	[[maybe_unused]] const ControlInfoMap &sensorControls)
{
	return 0;
}

void IPASTARFIVE::mapBuffers(const std::vector<IPABuffer> & buffers)
{
	for (const IPABuffer &buffer : buffers) {
		const FrameBuffer fb(buffer.planes);
		scBufferMaps_.emplace(buffer.id,
				 MappedFrameBuffer(&fb, MappedFrameBuffer::MapFlag::Read));
	}
}

void IPASTARFIVE::unmapBuffers(const std::vector<unsigned int> & ids)
{
	for (unsigned int id : ids) {
		auto it = scBufferMaps_.find(id);
		if (it == scBufferMaps_.end())
			continue;

		scBufferMaps_.erase(id);
	}
}

void IPASTARFIVE::queueRequest([[maybe_unused]] const ControlList & controls)
{
}

void IPASTARFIVE::statBufferReady([[maybe_unused]] const uint32_t bufferId, 
	[[maybe_unused]] const ControlList &sensorControls)
{
}

} // namespace ipa::starfive

/**
 * \brief External IPA module interface
 *
 * The IPAModuleInfo is required to match an IPA module construction against the
 * intented pipeline handler with the module. The API and pipeline handler
 * versions must match the corresponding IPA interface and pipeline handler.
 *
 * \sa struct IPAModuleInfo
 */
extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"PipelineHandlerStarfive",
	"starfive",
};

/**
 * \brief Create an instance of the IPA interface
 *
 * This function is the entry point of the IPA module. It is called by the IPA
 * manager to create an instance of the IPA interface for each camera. When
 * matched against with a pipeline handler, the IPAManager will construct an IPA
 * instance for each associated Camera.
 */
IPAInterface *ipaCreate()
{
	return new ipa::starfive::IPASTARFIVE();
}
}

} // namespace libcamera
