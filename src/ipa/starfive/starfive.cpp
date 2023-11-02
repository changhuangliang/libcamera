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
#include "linux/jh7110-isp.h"

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

	int configure(const ControlInfoMap &ispControls, const ControlInfoMap &sensorControls, 
		const IPACameraSensorInfo &sensorInfo, const std::vector<ssParams> &outSSParams) override;

	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;

	void queueRequest(const ControlList &controls) override;
	void statBufferReady(const uint32_t bufferId, const ControlList &sensorControls) override;

	bool isSensorISPEnabled() { return sensorISPEnable_; }

private:
	bool sensorISPEnable_;
	bool hardwareInited_;
	Size sensorSize_;

	ControlInfoMap ispCtrlInfoMap_;

	std::map<uint32_t, MappedFrameBuffer> scBufferMaps_;

	struct ssParams ssParams_[2];

private:
	void setSSParams(const std::vector<ssParams> &params, const Size &sensorOutput);
	void setSSParamsControl(ControlList &ctrlList);
};

int IPASTARFIVE::init([[maybe_unused]] const IPASettings &settings, 
	[[maybe_unused]] const IPACameraSensorInfo &sensorInfo, 
	[[maybe_unused]] const ControlInfoMap &sensorControls)
{

	sensorSize_ = sensorInfo.outputSize;

	ssParams_[0].outWidth = sensorInfo.outputSize.width;
	ssParams_[0].outHeight = sensorInfo.outputSize.height;
	ssParams_[1] = ssParams_[0];

	return 0;
}

void IPASTARFIVE::setSSParamsControl(ControlList &ctrlList)
{
	uint8_t buffer[sizeof(struct jh7110_isp_outss_setting) * 2];

	for(uint32_t i = 0; i < 2; i++) {
		struct jh7110_isp_outss_setting *setting = (struct jh7110_isp_outss_setting *)buffer + i;

		setting->which = (uint8_t)i;
		setting->stride = (ssParams_[i].outWidth + 7) & 0xfffffff8; //8-byte(64bit) granularity
		if(ssParams_[i].outWidth < sensorSize_.width) {
			setting->hsm = 0;	// scale down
			setting->hsf = ((ssParams_[i].outWidth << 12) + (sensorSize_.width - 1)) / sensorSize_.width;
		}else {
			setting->hsm = 2;	// no scale
			setting->hsf = 0;
		}
		if(ssParams_[i].outHeight < sensorSize_.height) {
			setting->vsm = 0;	// scale down
			setting->vsf = ((ssParams_[i].outHeight << 12) + (sensorSize_.height - 1)) / sensorSize_.height;
		}else {
			setting->vsm = 2;	// no scale
			setting->vsf = 0;
		}

		ControlValue ctrlV(Span<const uint8_t>(reinterpret_cast<uint8_t *>(setting),
				       sizeof(struct jh7110_isp_outss_setting)));
		ctrlList.set(V4L2_CID_USER_JH7110_ISP_OUTSS0_SETTING + i, ctrlV);
	}
}

int IPASTARFIVE::start([[maybe_unused]] const ControlList &controls)
{
	ControlList moduleControls(ispCtrlInfoMap_);

	setSSParamsControl(moduleControls);

	setIspControls.emit(moduleControls);

	return 0;
}

void IPASTARFIVE::setSSParams(const std::vector<ssParams> &params, const Size &sensorOutputSize)
{
	int index = 0;

	for(const ssParams &p : params) {
		if(p.outWidth <= sensorOutputSize.width && p.outHeight <= sensorOutputSize.height)
			ssParams_[index++] = p;
	}
}

int IPASTARFIVE::configure([[maybe_unused]] const ControlInfoMap &ispControls, 
	[[maybe_unused]] const ControlInfoMap &sensorControls, const IPACameraSensorInfo &sensorInfo, 
	const std::vector<ssParams> &outSSParams)
{
	ispCtrlInfoMap_ = ispControls;
	sensorSize_ = sensorInfo.outputSize;
	setSSParams(outSSParams, sensorInfo.outputSize);

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
