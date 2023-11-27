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

#include "sc_dump_def.h"

#include "ipa_interface.h"
#include "sensor_helper.h"
#include "linux/jh7110-isp.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASTARFIVE)

using namespace std::literals::chrono_literals;

namespace ipa::starfive {

class IPASTARFIVE : public IPAStarfiveInterface
{
public:
	IPASTARFIVE()
		: sensorISPEnable_(false), hardwareInited_(true), hasModuleAlgo_(false)
	{
		memset(&frameContext_, 0, sizeof(frameContext_));
	};
	~IPASTARFIVE()
	{
	};

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
	void calcSCCropInfo(Size imageSize, struct SCCropInfo &info);
	void initParamters(const IPACameraSensorInfo &sensorInfo, const ControlInfoMap &sensorControls);
	void processRequest(const ControlList &controls);
	void getControls(Algorithm::ModuleType mt, ControlList &ctrlList);
	void setSSParams(const std::vector<ssParams> &params, const Size &sensorOutput);
	void setSSParamsControl(ControlList &ctrlList);

	Algorithm *getISPModule(std::string key) {
		if(!hasModuleAlgo_ || !ispModules_.size())
			return nullptr;
		auto algoMap = ispModules_.find(key);
		if (algoMap != ispModules_.end())
			return (algoMap->second).get();
		return nullptr;
	}

private:
	bool sensorISPEnable_;
	bool hardwareInited_;
	Size sensorSize_;

	ControlInfoMap ispCtrlInfoMap_;
	ControlInfoMap sensorCtrls_;

	std::map<uint32_t, MappedFrameBuffer> scBufferMaps_;

	std::unique_ptr<StarfiveSensorHelper> sensorHelper_;

	StarfiveFrameContext frameContext_;

	struct ssParams ssParams_[2];

	bool hasModuleAlgo_;
	std::unordered_map<std::string, std::unique_ptr<Algorithm>> ispModules_;
};

#define impleAlgorithm_void(key, func)	{	\
	Algorithm * algo = getISPModule(key);	\
	if(algo) algo->func;}


void IPASTARFIVE::calcSCCropInfo(Size imageSize, struct SCCropInfo &info)
{
	uint32_t xDecFactor = (imageSize.width - 1) / (16 * 32);
	uint32_t yDecFactor = (imageSize.height - 1) / (16 * 32);
	uint32_t subWidth = (imageSize.width / (xDecFactor + 1)) & 0xfffe;
	uint32_t subHeight = (imageSize.height / (yDecFactor + 1)) & 0xfffe;

	subWidth = (subWidth / 16) & 0xfffe;
	subHeight = (subHeight / 16) & 0xfffe;

	info.hStart = ((imageSize.width - subWidth * 16 * (xDecFactor + 1)) >> 1) & 0xfffe;
	info.vStart = ((imageSize.height - subHeight * 16 * (yDecFactor + 1)) >> 1) & 0xfffe;
	info.swWidth = subWidth - 1;
	info.swHeight = subHeight - 1;
	info.hPeriod = (xDecFactor + 1) * 2 - 1;
	info.hKeep = 1;
	info.vPeriod = (yDecFactor + 1) * 2 - 1;
	info.vKeep = 1;
}

void IPASTARFIVE::initParamters(const IPACameraSensorInfo & sensorInfo, const ControlInfoMap & sensorControls)
{
	const ControlInfo &v4l2HBlank = sensorControls.find(V4L2_CID_HBLANK)->second;
	uint32_t hblank = v4l2HBlank.def().get<int32_t>();
	uint32_t lineLength = sensorInfo.outputSize.width + hblank;

	const ControlInfo &v4l2Exposure = sensorControls.find(V4L2_CID_EXPOSURE)->second;
	uint32_t expoRange[2] = { (uint32_t)v4l2Exposure.min().get<int32_t>(), (uint32_t)v4l2Exposure.max().get<int32_t>() };

	const ControlInfo &v4l2AnalGain = sensorControls.find(V4L2_CID_ANALOGUE_GAIN)->second;
	double analGainRange[2] = { sensorHelper_->gain(v4l2AnalGain.min().get<int32_t>()), 
		sensorHelper_->gain(v4l2AnalGain.max().get<int32_t>()) };

	struct SCCropInfo scCropInfo;

	calcSCCropInfo(sensorInfo.outputSize, scCropInfo);

	impleAlgorithm_void("star5.ae", init(&scCropInfo, lineLength * 1000000.0 / sensorInfo.pixelRate, expoRange, analGainRange));
	impleAlgorithm_void("star5.sc", init(&scCropInfo));
}

int IPASTARFIVE::init(const IPASettings &settings, const IPACameraSensorInfo &sensorInfo, 
	const ControlInfoMap &sensorControls)
{
	/* Load the tuning data file. */
	File file(settings.configurationFile);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		int ret = file.error();
		LOG(IPASTARFIVE, Error)
			<< "Failed to open configuration file "
			<< settings.configurationFile << ": " << strerror(-ret);
		return ret;
	}

	std::unique_ptr<libcamera::YamlObject> root = YamlParser::parse(file);
	if (!root)
		return -EINVAL;

	if ("jh7110" != (*root)["target"].get<std::string>("")) {
		LOG(IPASTARFIVE, Error)
			<< "Tuning file isn't for jh7110";
		return -EINVAL;
	}

	if (root->contains("sensor_isp_enable"))
		sensorISPEnable_ = (*root)["sensor_isp_enable"].get<int>(0) ? true : false;

	if (root->contains("algorithms")) {
		sensorHelper_ = StarfiveCreateSensorHelper(settings.sensorModel);
		if (!sensorHelper_) {
			LOG(IPASTARFIVE, Error)
				<< "Can't find the sensor helper for " << settings.sensorModel;
			return -EINVAL;
		}

		getISPModules(ispModules_);
		if(ispModules_.size())
			hasModuleAlgo_ = true;
		else {
			LOG(IPASTARFIVE, Error)
					<< "Can't get the isp control modules.";
		}

		if(hasModuleAlgo_) {
			for (auto const &[key, value] : (*root)["algorithms"].asDict()) {
				impleAlgorithm_void(key, parseConfiguration(value));
			}
		}

		initParamters(sensorInfo, sensorControls);

		hardwareInited_ = false;
	} else {
		if (!sensorISPEnable_) {
			LOG(IPASTARFIVE, Error)
				<< "Tuning file doesn't contain any algorithm";
			return -EINVAL;
		}
	}

	sensorSize_ = sensorInfo.outputSize;

	ssParams_[0].outWidth = sensorInfo.outputSize.width;
	ssParams_[0].outHeight = sensorInfo.outputSize.height;
	ssParams_[1] = ssParams_[0];
	
	return 0;
}

void IPASTARFIVE::processRequest(const ControlList &controls)
{
	if(!hasModuleAlgo_)
		return;

	Algorithm *algoAgc = getISPModule("star5.ae");
	Algorithm *algoAwb = getISPModule("star5.wb");
	if(!algoAgc || !algoAwb)
		return;

	ControlList sensorCtrls(sensorCtrls_);

	if (controls.contains(controls::AE_ENABLE))
		algoAgc->enable(Algorithm::modty_ctl, controls.get(controls::AE_ENABLE).get<bool>());
	if (controls.contains(controls::AWB_ENABLE))
		algoAwb->enable(Algorithm::modty_ctl, controls.get(controls::AWB_ENABLE).get<bool>());

	for (auto const &ctrl : controls) {
		switch (ctrl.first) {
		case controls::AE_ENABLE:
		case controls::AWB_ENABLE:
			break;
		case controls::EXPOSURE_TIME:
			if (!algoAgc->isEnabled(Algorithm::modty_ctl)) {
				uint32_t exposureLines = (uint32_t)((double)(ctrl.second.get<int32_t>()) / getLineDuration(algoAgc));
				if (exposureLines > 0)
					sensorCtrls.set(V4L2_CID_EXPOSURE, (int32_t)exposureLines);
			}
			break;
		case controls::ANALOGUE_GAIN:
			if (!algoAgc->isEnabled(Algorithm::modty_ctl))
				sensorCtrls.set(V4L2_CID_ANALOGUE_GAIN, (int32_t)sensorHelper_->gainCode((double)(ctrl.second.get<float>())));
			break;
		case controls::EXPOSURE_VALUE:
			if (!algoAgc->isEnabled(Algorithm::modty_ctl))
				setEV(algoAgc, pow(2.0, ctrl.second.get<float>()));
			break;
		default:
			break;
		}
	}

	if (sensorCtrls.size())
		setDelayedControls.emit(sensorCtrls);
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

int IPASTARFIVE::start(const ControlList &controls)
{
	processRequest(controls);

	ControlList moduleControls(ispCtrlInfoMap_);

	setSSParamsControl(moduleControls);
	if (!hardwareInited_) {
		getControls(Algorithm::ModuleType::modty_mod, moduleControls);
		hardwareInited_ = true;
	} else {
		impleAlgorithm_void("star5.sc", getControl(Algorithm::ModuleType::modty_mod, moduleControls));
	}

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

int IPASTARFIVE::configure(const ControlInfoMap &ispControls, 
	const ControlInfoMap &sensorControls, const IPACameraSensorInfo &sensorInfo, 
	const std::vector<ssParams> &outSSParams)
{
	ispCtrlInfoMap_ = ispControls;
	sensorCtrls_ = sensorControls;

	sensorSize_ = sensorInfo.outputSize;
	setSSParams(outSSParams, sensorInfo.outputSize);

	const ControlInfo &v4l2Exposure = sensorControls.find(V4L2_CID_EXPOSURE)->second;
	const ControlInfo &v4l2AnalGain = sensorControls.find(V4L2_CID_ANALOGUE_GAIN)->second;
	resetAgc(getISPModule("star5.ae"), v4l2Exposure.def().get<int32_t>(), sensorHelper_->gain(v4l2AnalGain.def().get<int32_t>()));

	struct SCCropInfo scCropInfo;
	calcSCCropInfo(sensorInfo.outputSize, scCropInfo);
	impleAlgorithm_void("star5.sc", init(&scCropInfo));

	return 0;
}

void IPASTARFIVE::mapBuffers(const std::vector<IPABuffer> & buffers)
{
	if(!hasModuleAlgo_)
		return;
	for (const IPABuffer &buffer : buffers) {
		const FrameBuffer fb(buffer.planes);
		scBufferMaps_.emplace(buffer.id,
				 MappedFrameBuffer(&fb, MappedFrameBuffer::MapFlag::Read));
	}
}

void IPASTARFIVE::unmapBuffers(const std::vector<unsigned int> & ids)
{
	if(!hasModuleAlgo_)
		return;
	for (unsigned int id : ids) {
		auto it = scBufferMaps_.find(id);
		if (it == scBufferMaps_.end())
			continue;

		scBufferMaps_.erase(id);
	}
}

void IPASTARFIVE::queueRequest(const ControlList & controls)
{
	processRequest(controls);
}

void IPASTARFIVE::statBufferReady(const uint32_t bufferId, const ControlList &sensorControls)
{
	if(!hasModuleAlgo_)
		return;

	Algorithm *algoAgc = getISPModule("star5.ae");
	Algorithm *algoAwb = getISPModule("star5.wb");
	if(!algoAgc || !algoAwb)
		return;

	auto it = scBufferMaps_.find(bufferId);
	if (it == scBufferMaps_.end()) {
		LOG(IPASTARFIVE, Error) << "Could not find stats buffer!";
		return;
	}

	frameContext_.exposure = sensorControls.get(V4L2_CID_EXPOSURE).get<int32_t>();
	frameContext_.gain = sensorHelper_->gain(sensorControls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>());
	Span<uint8_t> mem = it->second.planes()[0];

	const uint8_t *scData = mem.data();
	const uint8_t *aeawbData = scData + ISP_YHIST_BUFFER_SIZE;
	const uint16_t *typeFlag = (uint16_t *)(aeawbData + ISP_SCD_BUFFER_SIZE);

	uint32_t exposureLines = 0;
	double sensorGain = 0.0;
	ControlList sensorCtrls(sensorCtrls_);

	if (typeFlag[0]) {
		// AGC
		algoAgc->process(aeawbData, frameContext_);
		getCurrentEV(algoAgc, exposureLines, sensorGain, ae_sevph_1st);
		getISO(algoAgc, frameContext_.isoLevel, frameContext_.isoFactor);
	} else {
		getCurrentEV(algoAgc, exposureLines, sensorGain, ae_sevph_2nd);

		// AWB
		algoAwb->process(aeawbData, frameContext_);
		getColorTemperatureInfo(algoAwb, frameContext_.tempIndex, frameContext_.tempFactor);
	}

	for (auto &algo : ispModules_) {
		if(algo.first != "star5.ae" && algo.first != "star5.wb")
			algo.second.get()->process(scData, frameContext_);
	}

	ControlList ctlControls(ispCtrlInfoMap_);
	getControls(Algorithm::ModuleType::modty_ctl, ctlControls);

	setIspControls.emit(ctlControls);

	if (exposureLines)
		sensorCtrls.set(V4L2_CID_EXPOSURE, (int32_t)exposureLines);
	if (sensorGain > 0.0)
		sensorCtrls.set(V4L2_CID_ANALOGUE_GAIN, (int32_t)sensorHelper_->gainCode(sensorGain));
	if (exposureLines || sensorGain > 0.0)
		setDelayedControls.emit(sensorCtrls);
}

void IPASTARFIVE::getControls(Algorithm::ModuleType mt, ControlList &ctrlList)
{
	for (auto &algo : ispModules_) {
		algo.second.get()->getControl(mt, ctrlList);
	}
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
