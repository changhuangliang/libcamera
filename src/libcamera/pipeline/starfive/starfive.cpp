/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Starfive Technology Co., Ltd.
 *
 * starfive.cpp - Pipeline handler for StarFive ISP
 */
#include <algorithm>
#include <assert.h>
#include <cmath>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <queue>
#include <unordered_set>
#include <utility>

#include <libcamera/base/file.h>
#include <libcamera/base/shared_fd.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/ipa/starfive_ipa_interface.h>
#include <libcamera/ipa/starfive_ipa_proxy.h>
#include <libcamera/logging.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>

#include <linux/media-bus-format.h>
#include <linux/videodev2.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_lens.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/delayed_controls.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/v4l2_videodevice.h"
#include "libcamera/internal/yaml_parser.h"
#include "libcamera/internal/mapped_framebuffer.h"

#include "starfive.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(STARFIVE)

class PipelineHandlerStarfive;

const std::map<uint32_t, uint32_t> mbusCodesTransform = {
	{ MEDIA_BUS_FMT_SBGGR10_1X10, MEDIA_BUS_FMT_SBGGR12_1X12 },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, MEDIA_BUS_FMT_SGBRG12_1X12 },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, MEDIA_BUS_FMT_SGRBG12_1X12 },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, MEDIA_BUS_FMT_SRGGB12_1X12 },
	{ MEDIA_BUS_FMT_SBGGR12_1X12, MEDIA_BUS_FMT_SBGGR12_1X12 },
	{ MEDIA_BUS_FMT_SGBRG12_1X12, MEDIA_BUS_FMT_SGBRG12_1X12 },
	{ MEDIA_BUS_FMT_SGRBG12_1X12, MEDIA_BUS_FMT_SGRBG12_1X12 },
	{ MEDIA_BUS_FMT_SRGGB12_1X12, MEDIA_BUS_FMT_SRGGB12_1X12 },
};

class StarfiveCameraData : public StarfiveCameraDataBase
{
public:
	StarfiveCameraData(PipelineHandler *pipe, MediaDevice *media)
		: StarfiveCameraDataBase(pipe, media)
	{
	}

	int init() override;

	std::unique_ptr<V4L2VideoDevice> videoSS0_;
	std::unique_ptr<V4L2VideoDevice> videoSS1_;
	std::unique_ptr<V4L2Subdevice> dvpDev_;
	std::unique_ptr<V4L2Subdevice> csiPhySubDev_;
	std::unique_ptr<V4L2Subdevice> vinIspSubDev_;
	std::unique_ptr<V4L2Subdevice> wrSubDev_;

	enum class SensorDataInterface
	{
		Mipi = 0,
		Dvp,
		Unknown
	} sdIf_ = SensorDataInterface::Unknown;

protected:
	void collectCompMbusCode() override;

};

int StarfiveCameraData::init()
{
	int ret = 0;

	MediaEntity *csiPhyEntity = media_->getEntityByName("stf_csiphy0");
	const std::vector<MediaPad *> &csiPads = csiPhyEntity->pads();
	MediaEntity *dvpEntity = media_->getEntityByName("stf_dvp0");
	const std::vector<MediaPad *> &dvpPads = dvpEntity->pads();
	MediaEntity *sensorEntity = nullptr;

	// Try to get the sensor entity from the CSI PHY first.
	if (!csiPads.empty()) {
		const std::vector<MediaLink *> &links = csiPads[0]->links();
		if (!links.empty()) {
			sensorEntity = links[0]->source()->entity();
			sdIf_ = SensorDataInterface::Mipi;
		}
	}

	// If the sensor dose not connect to the CSI PHY, then try the DVP.
	if (SensorDataInterface::Unknown == sdIf_ && !dvpPads.empty()) {
		const std::vector<MediaLink *> &links = dvpPads[0]->links();
		if (!links.empty()) {
			sensorEntity = links[0]->source()->entity();
			sdIf_ = SensorDataInterface::Dvp;
		}
	}

	// Create the CameraSensor object by the sensor entity.
	if (!sensorEntity) {
		LOG(STARFIVE, Error) << "Can not find the sensor entity.";
		return -ENODEV;
	}
	sensor_ = std::make_unique<CameraSensor>(sensorEntity);
	ret = sensor_->init();
	if (ret)
		return ret;

	// Load and initialize the IPA module.
	ret = loadIPA();
	if (ret)
		return ret;
	disableISP = ipa_->isSensorISPEnabled();
	
	dvpDev_ = std::make_unique<V4L2Subdevice>(dvpEntity);
	ret = dvpDev_->open();
	if (ret)
		return ret;
	
	csiPhySubDev_ = std::make_unique<V4L2Subdevice>(csiPhyEntity);
	ret = csiPhySubDev_->open();
	if (ret)
		return ret;

	if (!disableISP) {
		// Use the StarFive ISP pipeline.
		//MediaEntity *ispEntity = media_->getEntityByName("stf_isp0");
		ispSubDev_ = std::make_unique<V4L2Subdevice>(media_->getEntityByName("stf_isp0"));
		ret = ispSubDev_->open();
		if (ret)
			return ret;

		//ispEntity = media_->getEntityByName("stf_vin0_isp0");
		vinIspSubDev_ = std::make_unique<V4L2Subdevice>(media_->getEntityByName("stf_vin0_isp0"));
		ret = vinIspSubDev_->open();
		if (ret)
			return ret;

		video_ = V4L2VideoDevice::fromEntityName(media_, "stf_vin0_isp0_video1");
		ret = video_->open();
		if (ret)
			return ret;
		
		videoSS0_ = V4L2VideoDevice::fromEntityName(media_, "stf_vin0_isp0_ss0_video2");
		ret = videoSS0_->open();
		if (ret)
			return ret;

		videoSS1_ = V4L2VideoDevice::fromEntityName(media_, "stf_vin0_isp0_ss1_video3");
		ret = videoSS1_->open();
		if (ret)
			return ret;

		scDev_ = V4L2VideoDevice::fromEntityName(media_, "stf_vin0_isp0_scd_y_video7");
		ret = scDev_->open();
		if (ret)
			return ret;
		connectSCSignal();
	} else {
		// Use the sensor own isp pipeline.
		//MediaEntity *wrEntity = media_->getEntityByName("stf_vin0_wr");
		wrSubDev_ = std::make_unique<V4L2Subdevice>(media_->getEntityByName("stf_vin0_wr"));
		ret = wrSubDev_->open();
		if (ret)
			return ret;

		video_ = V4L2VideoDevice::fromEntityName(media_, "stf_vin0_wr_video0");
		ret = video_->open();
		if (ret)
			return ret;

		//collectSensorCapFormat();
	}
	
	raw_ = V4L2VideoDevice::fromEntityName(media_, "stf_vin0_isp0_raw_video6");
	ret = raw_->open();
	if (!ret)
		haveRaw_ = true;

	connectVideoSignal();

	//MediaEntity *csiEntity = media_->getEntityByName("stf_csi0");
	csiSubDev_ = std::make_unique<V4L2Subdevice>(media_->getEntityByName("stf_csi0"));
	ret = csiSubDev_->open();
	if (ret)
		return ret;

	collectCompMbusCode();

	properties_ = sensor_->properties();

	return 0;
}

void StarfiveCameraData::collectCompMbusCode()
{
	const std::vector<unsigned int> &mbusCodes = sensor_->mbusCodes();
	std::vector<unsigned int> sinkCodes;

	if (SensorDataInterface::Unknown == sdIf_)
		return;

	// Collect the sensor mbus codes.
	for (const uint32_t &code : mbusCodes) {
		std::vector<Size> allSize = sensor_->sizes(code);
		bool hasSuitableSize = false;
		if (!allSize.size())
			continue;
		for(const Size &sz : allSize) {
			if(sz.width <= STARFIVE_ISP_MAX_WIDTH && sz.height <= STARFIVE_ISP_MAX_HEIGHT) {
				hasSuitableSize = true;
				break;
			}
		}
		if(!hasSuitableSize)
			continue;
		
		V4L2SubdeviceFormat sensorFormat = sensor_->getFormat(mbusCodes, allSize[0]);
		if (!disableISP) {
			if (ColorSpace::Primaries::Raw == sensorFormat.colorSpace->primaries)
				sinkCodes.push_back(code);
		} else {
			if (ColorSpace::Primaries::Raw != sensorFormat.colorSpace->primaries)
				sinkCodes.push_back(code);
		}
	}

	auto takeMatchCode = [](std::vector<unsigned int> &codes, const V4L2Subdevice::Formats &fmts)
		-> const std::vector<unsigned int> & {
		std::vector<unsigned int> resCodes;

		for (const uint32_t code : codes) {
			auto it = fmts.find(code);
			if (it != fmts.end())
				resCodes.push_back(code);
		}
		codes = std::move(resCodes);
		return codes;
	};
	
	// Match the sensor to the CSI/DVP.
	V4L2Subdevice::Formats fmts; 
	if (SensorDataInterface::Mipi == sdIf_) {
		fmts = csiPhySubDev_->formats(0);
		mbufCodeLink_.push_back(takeMatchCode(sinkCodes, fmts));

		fmts = csiSubDev_->formats(0);
	} else
		fmts = dvpDev_->formats(0);

	mbufCodeLink_.push_back(takeMatchCode(sinkCodes, fmts));

	if (!disableISP) {
		// Match the CSI/DVP to ISP.
		fmts = ispSubDev_->formats(0);
		mbufCodeLink_.push_back(takeMatchCode(sinkCodes, fmts));

		// Match the ISP to VIN_ISP.
		fmts = ispSubDev_->formats(1);
		sinkCodes = utils::map_keys(fmts);
		
		fmts = vinIspSubDev_->formats(0);
		mbufCodeLink_.push_back(takeMatchCode(sinkCodes, fmts));
	} else {
		fmts = wrSubDev_->formats(0);
		mbufCodeLink_.push_back(takeMatchCode(sinkCodes, fmts));
	}

	collectISPSizeRange();
}

class StarfiveCameraConfiguration : public CameraConfiguration
{
public:
	StarfiveCameraConfiguration(StarfiveCameraDataBase *data) : CameraConfiguration()
    {
        data_ = data;
    }

	Status validate() override;

	/* Cache the combinedTransform_ that will be applied to the sensor */
	Transform combinedTransform_;

private:
	StarfiveCameraDataBase *data_;
};

CameraConfiguration::Status StarfiveCameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty() || data_->mbufCodeLink_.empty()) {
		LOG(STARFIVE, Error) << "No configuration is found.";
		return Invalid;
	}

	Transform requestedTransform = transform;
	combinedTransform_ = data_->sensor_->validateTransform(&transform);
	if (transform != requestedTransform)
		status = Adjusted;

	std::vector<unsigned int> mbusCodes = data_->mbufCodeLink_.front();

	/* Cap the number of entries to the available streams. */
	uint32_t kMaxStreams = !data_->disableISP ? 2 : 1;
	if (config_.size() > kMaxStreams) {
		config_.resize(kMaxStreams);
		status = Adjusted;
	}

	for (unsigned int i = 0; i < config_.size(); ++i) {
		const PixelFormatInfo &info = PixelFormatInfo::info(config_[i].pixelFormat);
		StreamConfiguration &cfg = config_[i];

		const Size size = cfg.size;

		cfg.size.width = std::max(8u, std::min(1920u, cfg.size.width));
		cfg.size.height = std::max(8u, std::min(1080u, cfg.size.height));

		if (cfg.size != size) {
			status = Adjusted;
		}

		if (cfg.bufferCount < STREAM_BUFFER_COUNT)
			cfg.bufferCount = STREAM_BUFFER_COUNT;

		if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW) {
			if (data_->haveRaw_ && !data_->disableISP) {
				V4L2DeviceFormat format;
				format.fourcc = data_->raw_->toV4L2PixelFormat(cfg.pixelFormat);
				format.size = cfg.size;

				int ret = data_->raw_->tryFormat(&format);
				if (ret)
					return Invalid;

				V4L2SubdeviceFormat sensorFormat = data_->sensor_->getFormat(mbusCodes, cfg.size);
				ret = data_->sensor_->tryFormat(&sensorFormat);
				if (ret)
					return Invalid;

				cfg.stride = ((cfg.size.width * 12 / 8 + 8 * 16 - 1) / (8 * 16)) * 128;
				cfg.frameSize = info.frameSize(cfg.size, 64);

				cfg.setStream(&data_->rawStream_);
			}
		} else {
			if (!data_->disableISP) {
				V4L2DeviceFormat format;
				format.fourcc = data_->video_->toV4L2PixelFormat(cfg.pixelFormat);
				format.size = cfg.size;

				int ret = data_->video_->tryFormat(&format);
				if (ret)
					return Invalid;

				V4L2SubdeviceFormat sensorFormat = data_->sensor_->getFormat(mbusCodes, cfg.size);
				ret = data_->sensor_->tryFormat(&sensorFormat);
				if (ret)
					return Invalid;

				cfg.pixelFormat = formats::NV12;
				cfg.stride = format.planes[0].bpl;
				cfg.frameSize = format.planes[0].size;

				cfg.setStream(&data_->outStream_);
			} else {
				if (!data_->mbufCodeLink_.size())
					return Invalid;

				V4L2PixelFormat fitFormat;

				if (data_->findMatchVideoFormat(cfg.pixelFormat, cfg.size, fitFormat)) {

					cfg.setStream(&data_->outStream_);
				}
			}
		}
	}

	return status;
}

class StarfivePipelineAdapter : public StarfivePipelineAdapterBase
{
public:
	StarfivePipelineAdapter(StarfiveCameraDataBase *data)
		: StarfivePipelineAdapterBase(data)
	{
	}

	uint32_t mbusCodeTrans(uint32_t code) override
	{
		auto mbusCTMap = mbusCodesTransform.find(code);
		if (mbusCTMap == mbusCodesTransform.end())
			return 0;
		else
			return mbusCTMap->second;
	}

	int setRawSubDevFormat(V4L2SubdeviceFormat &sensorFormat) override
	{
		StarfiveCameraData *data = dynamic_cast<StarfiveCameraData *>(data_);
		return data->ispSubDev_->setFormat(6, &sensorFormat);
	}

	int setSSOutFormat(V4L2DeviceFormat &format) override
	{
		StarfiveCameraData *data = dynamic_cast<StarfiveCameraData *>(data_);
		V4L2DeviceFormat orgFormat = format;

		int ret = data->videoSS0_->setFormat(&orgFormat);
		if (ret)
			return ret;

		ret = data->videoSS1_->setFormat(&format);
		if (ret)
			return ret;

		return 0;
	}

	PixelFormat getSCPixFormat(Size size) override
	{
		StarfiveCameraData *data = dynamic_cast<StarfiveCameraData *>(data_);
		std::vector<unsigned int> mbusCodes = data->mbufCodeLink_.front();
		V4L2SubdeviceFormat sensorFormat = data->sensor_->getFormat(mbusCodes, size);
		uint32_t mbusCode = mbusCodeTrans(sensorFormat.mbus_code);
		V4L2VideoDevice::Formats formats = data->scDev_->formats(mbusCode);

		if(formats.empty())
			return PixelFormat();

		return formats.begin()->first.toPixelFormat();
	}

	int linkPipeline() override;
	int setupFormats(const V4L2DeviceFormat videoFormat) override;
};

int StarfivePipelineAdapter::linkPipeline()
{
	StarfiveCameraData *data = dynamic_cast<StarfiveCameraData *>(data_);
	MediaDevice *sfMediaDev = data->media_;
	MediaLink * csiLink = sfMediaDev->link("stf_csi0", 1, !data->disableISP ? "stf_isp0" : "stf_vin0_wr", 0);
	MediaLink * dvpLink = sfMediaDev->link("stf_dvp0", 1, !data->disableISP ? "stf_isp0" : "stf_vin0_wr", 0);
	int ret = 0;

	switch(data->sdIf_) {
	case StarfiveCameraData::SensorDataInterface::Mipi:
		if (!csiLink) {
			LOG(STARFIVE, Error) << "Can not find the link between CSI and ISP.";
			return -ENODEV;
		}
		
		if(dvpLink && (dvpLink->flags() & MEDIA_LNK_FL_ENABLED)) {
			ret = dvpLink->setEnabled(false);
			if(ret)
				return ret;
		}

		if(!(csiLink->flags() & MEDIA_LNK_FL_ENABLED)) {
			ret = csiLink->setEnabled(true);
			if(ret)
				return ret;
		}
		break;
	case StarfiveCameraData::SensorDataInterface::Dvp:
		if (!dvpLink) {
			LOG(STARFIVE, Error) << "Can not find the link between DVP and ISP.";
			return -ENODEV;
		}

		if(csiLink && (csiLink->flags() & MEDIA_LNK_FL_ENABLED)) {
			ret = csiLink->setEnabled(false);
			if(ret)
				return ret;
		}

		if(!(dvpLink->flags() & MEDIA_LNK_FL_ENABLED)) {
			ret = dvpLink->setEnabled(true);
			if(ret)
				return ret;
		}
		break;
	default:
		LOG(STARFIVE, Error) << "Unknown sensor data interface.";
		return -ENODEV;
	}

	if (!data->disableISP) {
		MediaLink *ispLink = sfMediaDev->link("stf_isp0", 1, "stf_vin0_isp0", 0);
		if (ispLink) {
			ret = ispLink->setEnabled(true);
			if (ret)
				return ret;
		}
		ispLink = sfMediaDev->link("stf_isp0", 6, "stf_vin0_isp0_raw", 0);
		if (ispLink) {
			ret = ispLink->setEnabled(true);
			if (ret)
				return ret;
		}
		ispLink = sfMediaDev->link("stf_isp0", 7, "stf_vin0_isp0_scd_y", 0);
		if (ispLink) {
			ret = ispLink->setEnabled(true);
			if (ret)
				return ret;
		}
	}

	MediaLink *ssLink = sfMediaDev->link("stf_isp0", 2, "stf_vin0_isp0_ss0", 0);
	if (ssLink) {
		ret = ssLink->setEnabled(true);
		if (ret)
			return ret;
	}
	ssLink = sfMediaDev->link("stf_isp0", 3, "stf_vin0_isp0_ss1", 0);
	if (ssLink) {
		ret = ssLink->setEnabled(true);
		if (ret)
			return ret;
	}
	
	return 0;
}

int StarfivePipelineAdapter::setupFormats(const V4L2DeviceFormat videoFormat)
{
	StarfiveCameraData *data = dynamic_cast<StarfiveCameraData *>(data_);
	if (!data->mbufCodeLink_.size())
		return 0;

	//const MediaEntity *curEntity = nullptr;
	V4L2SubdeviceFormat format{};
	format.mbus_code = 0;
	//format.size = data->sensor_->resolution();
	format.size = data->getSuitableSensorSize();

	const std::vector<uint32_t> &mbusCode = data->mbufCodeLink_.back();
	V4L2VideoDevice::Formats fmts;

	for (auto &code : mbusCode) {
		fmts = data->video_->formats(code);
		for (const auto &cur : fmts) {
			if (videoFormat.fourcc == cur.first) {
				for (auto &sr : cur.second) {
					if (sr.contains(format.size)) {
						format.mbus_code = code;
						goto setupFormats_next;
					}
				}
			}
		}
	}

	if (!format.mbus_code) {
		LOG(STARFIVE, Error) << "Can not find the suitable mbus_code in the video device.";
		return -EINVAL;
	}

setupFormats_next:

	V4L2SubdeviceFormat curFormat = format;
	if (!data->disableISP) {
		data->vinIspSubDev_->setFormat(0, &curFormat);
		data->vinIspSubDev_->setFormat(1, &curFormat);

		data->ispSubDev_->setFormat(1, &curFormat);

		curFormat = data->sensor_->getFormat(data->mbufCodeLink_.front(), format.size);
		data->ispSubDev_->setFormat(0, &curFormat);
	} else {
		data->wrSubDev_->setFormat(0, &curFormat);
		data->wrSubDev_->setFormat(1, &curFormat);
	}

	if (StarfiveCameraData::SensorDataInterface::Mipi == data->sdIf_) {
		data->csiSubDev_->setFormat(1, &curFormat);
		if (MEDIA_BUS_FMT_AYUV8_1X32 == curFormat.mbus_code) {
			format.mbus_code = MEDIA_BUS_FMT_UYVY8_2X8;
			curFormat = format;
		}
		data->csiSubDev_->setFormat(0, &curFormat);
		data->csiPhySubDev_->setFormat(0, &curFormat);
		data->csiPhySubDev_->setFormat(1, &curFormat);
	} else {
		data->dvpDev_->setFormat(0, &curFormat);
		data->dvpDev_->setFormat(1, &curFormat);
	}

	return 0;
}

class PipelineHandlerStarfive : public PipelineHandler
{
public:
	PipelineHandlerStarfive(CameraManager *manager);

	std::unique_ptr <CameraConfiguration> generateConfiguration(Camera *camera,
						   const StreamRoles &roles) override;
	int configure(Camera *camera, CameraConfiguration *c) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	MediaDevice *findPipeline(DeviceEnumerator *enumerator);

	StarfiveCameraData *cameraData(Camera *camera)
	{
		return static_cast<StarfiveCameraData *>(camera->_d());
	}

	int linkPipeline()
	{
		return adapter_->linkPipeline();
	}
	int registerCameras(MediaDevice *sfMediaDev);

	int setupFormats(const V4L2DeviceFormat videoFormat)
	{
		return adapter_->setupFormats(videoFormat);
	}

	int freeBuffers(Camera *camera);

private:
	std::unique_ptr<StarfivePipelineAdapterBase> adapter_;
	Starfive::PipelineType pplType_;
};

PipelineHandlerStarfive::PipelineHandlerStarfive(CameraManager *manager)
	: PipelineHandler(manager), pplType_(Starfive::PipelineType::pplt_unknown)
{
}

std::unique_ptr<CameraConfiguration> 
PipelineHandlerStarfive::generateConfiguration(Camera *camera, const StreamRoles &roles)
{
	StarfiveCameraData *data = cameraData(camera);
	std::unique_ptr<StarfiveCameraConfiguration> config =
		std::make_unique<StarfiveCameraConfiguration>(data);
	std::optional<ColorSpace> colorSpace;

	if (roles.empty())
		return config;

	if (!data->disableISP) {
		for (const StreamRole role : roles) {
			std::map<PixelFormat, std::vector<SizeRange>> streamFormats;
			unsigned int bufferCount;
			PixelFormat pixelFormat;
			Size sensorResolution;

			//sensorResolution = data->sensor_->resolution();
			sensorResolution = data->getSuitableSensorSize();
			switch (role) {
			case StreamRole::StillCapture:
			case StreamRole::Viewfinder:
			case StreamRole::VideoRecording:
				pixelFormat = formats::NV12;
				bufferCount = STREAM_BUFFER_COUNT;
				streamFormats[pixelFormat] = data->sensorSizes();
				colorSpace = ColorSpace::Sycc;
				break;

			case StreamRole::Raw: {
				if(!data->haveRaw_)
					continue;

				std::vector<unsigned int> mbusCodes = data->mbufCodeLink_.front();

				V4L2SubdeviceFormat sensorFormat = data->sensor_->getFormat(mbusCodes, sensorResolution);
				if (!sensorFormat.mbus_code || !adapter_->mbusCodeTrans(sensorFormat.mbus_code)) {
					break;
				}

				V4L2VideoDevice::Formats videoFormat = data->raw_->formats(adapter_->mbusCodeTrans(sensorFormat.mbus_code));
				if(videoFormat.empty())
					break;

				pixelFormat = videoFormat.begin()->first.toPixelFormat();
				sensorResolution = sensorFormat.size;
				bufferCount = STREAM_BUFFER_COUNT;
				colorSpace = ColorSpace::Raw;

				streamFormats[pixelFormat] = data->sensorSizes();

				break;
			}
			default:
				break;
			}

			StreamFormats formats(streamFormats);
			StreamConfiguration cfg(formats);
			cfg.size = sensorResolution;
			cfg.pixelFormat = pixelFormat;
			cfg.bufferCount = bufferCount;
			cfg.colorSpace = colorSpace;
			config->addConfiguration(cfg);
		}
	} else {
		if (!data->mbufCodeLink_.size())
			return {};

		Size sensorResolution = data->sensor_->resolution();
		PixelFormat pixelFormat;
		std::map<PixelFormat, std::vector<SizeRange>> streamFormats;
		unsigned int bufferCount = STREAM_BUFFER_COUNT;
		V4L2PixelFormat fitFormat;

		if (data->findMatchVideoFormat(PixelFormat(), sensorResolution, fitFormat)) {
			pixelFormat = fitFormat.toPixelFormat();
			streamFormats[pixelFormat] = data->sensorSizes();

			StreamFormats formats(streamFormats);
			StreamConfiguration cfg(formats);
			cfg.size = sensorResolution;
			cfg.pixelFormat = pixelFormat;
			cfg.bufferCount = bufferCount;
			config->addConfiguration(cfg);
		}
	}

	if (config->validate() == CameraConfiguration::Invalid)
		return {};

	return config;
}

int PipelineHandlerStarfive::configure(Camera *camera, CameraConfiguration *c)
{
	StarfiveCameraConfiguration *config =
		static_cast<StarfiveCameraConfiguration *>(c);
	StarfiveCameraData *data = cameraData(camera);
	bool sensorFmtSet = false;
	int ret = 0;

	// Set the sensor's format
	std::vector<unsigned int> mbusCodes = data->mbufCodeLink_.front();
	//Size sensorResolution = data->sensor_->resolution();
	Size sensorResolution = data->getSuitableSensorSize();
	V4L2SubdeviceFormat sensorFormat = data->sensor_->getFormat(mbusCodes, sensorResolution);
	ret = data->sensor_->setFormat(&sensorFormat, config->combinedTransform_);
	if (ret)
		return ret;

	// Set the sensor transform.
	sensorFormat.mbus_code = adapter_->mbusCodeTrans(sensorFormat.mbus_code);
	if(!sensorFormat.mbus_code) {
		LOG(STARFIVE, Error) << "Can not find the mbus_code for the pad 6 of the ISP sub-device.";
		return -EINVAL;
	}

	// Configure the isp pad 6.
	ret = adapter_->setRawSubDevFormat(sensorFormat);
	if (ret)
		return ret;

	// Configure the video.
	data->rawStreamEnable_ = false;
	for (unsigned int i = 0; i < config->size(); ++i) {
		StreamConfiguration &cfg = (*config)[i];
		Stream *stream = cfg.stream();

		if(!data->disableISP) {
			if(!data->isValidSize(cfg.size)) {
				LOG(STARFIVE, Error) << "The ISP can not support " << cfg.size.width << "x" << cfg.size.height;
				LOG(STARFIVE, Info) << "You can try " << sensorResolution.width << "x" << 
					sensorResolution.height << ".";
				return -EINVAL;
			}
		}

		if (stream == &data->rawStream_) {
			sensorFormat = data->sensor_->getFormat(mbusCodes, cfg.size);
			ret = data->sensor_->setFormat(&sensorFormat, config->combinedTransform_);
			if (ret)
				return ret;
			sensorFmtSet = true;

			V4L2DeviceFormat format = {};
			format.fourcc = data->raw_->toV4L2PixelFormat(cfg.pixelFormat);
			format.size = cfg.size;

			ret = data->raw_->setFormat(&format);
			if (ret)
				return ret;

			if (format.size != cfg.size ||
			    format.fourcc != data->raw_->toV4L2PixelFormat(cfg.pixelFormat)) {
				LOG(STARFIVE, Error) << "The raw format is not supported.";
				return -EINVAL;
			}

			data->rawStreamEnable_ = true;
		} else if (stream == &data->outStream_) {
			if(!sensorFmtSet ) {
				sensorFormat = data->sensor_->getFormat(mbusCodes, cfg.size);
				ret = data->sensor_->setFormat(&sensorFormat, config->combinedTransform_);
				if (ret)
					return ret;
				sensorFmtSet = true;
			}

			V4L2DeviceFormat format = {};
			format.fourcc = data->video_->toV4L2PixelFormat(cfg.pixelFormat);
			format.size = cfg.size;

			ret = data->video_->setFormat(&format);
			if (ret)
				return ret;

			if (format.size != cfg.size ||
			    format.fourcc != data->video_->toV4L2PixelFormat(cfg.pixelFormat)) {
				LOG(STARFIVE, Error) << "The video format is not supported.";
			 	return -EINVAL;
			}

			format.fourcc = data->video_->toV4L2PixelFormat(cfg.pixelFormat);
			format.size = cfg.size;
			ret = adapter_->setSSOutFormat(format);
			if (ret)
				return ret;

			ret = setupFormats(format);
			if (ret)
				return ret;

			if (!data->disableISP && !data->setSCFormat_) {
				PixelFormat pixelFormat = adapter_->getSCPixFormat(cfg.size);
				V4L2DeviceFormat scFormat = {};

				scFormat.fourcc = data->raw_->toV4L2PixelFormat(pixelFormat);
				scFormat.size = cfg.size;
				ret = data->scDev_->setFormat(&scFormat);
				if (ret)
					return -EINVAL;
				data->setSCFormat_ = true;
			}
		}
	}

	IPACameraSensorInfo sensorInfo{};
	ret = data->sensor_->sensorInfo(&sensorInfo);
	if (ret)
		return ret;

	std::vector<ipa::starfive::ssParams> outSSParams = {
		{(uint16_t)sensorInfo.outputSize.width, (uint16_t)sensorInfo.outputSize.height},
		{(uint16_t)sensorInfo.outputSize.width, (uint16_t)sensorInfo.outputSize.height}
	};
	//ret = data->ipa_->configure(data->ispSubDev_->controls(), data->sensor_->controls(),
	//	sensorInfo, outSSParams);
	ret = data->ipa_->configure(adapter_->getISPControls(), data->sensor_->controls(),
		sensorInfo, outSSParams);
	if (ret)
		return ret;

	return 0;
}

int PipelineHandlerStarfive::exportFrameBuffers(Camera *camera, Stream *stream,
					   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	StarfiveCameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	if (stream == &data->outStream_)
		return data->video_->exportBuffers(count, buffers);
	else if (stream == &data->rawStream_)
		return data->raw_->exportBuffers(count, buffers);
	else
		return -EINVAL;
}

int PipelineHandlerStarfive::start(Camera *camera, const ControlList *controls)
{
	StarfiveCameraData *data = cameraData(camera);
	unsigned int count = data->outStream_.configuration().bufferCount;
	int ret = -EINVAL;

	data->delayedCtrls_->reset();

	ret = data->video_->importBuffers(count);
	if (ret < 0)
		return ret;

	ret = data->video_->streamOn();
	if (ret < 0)
		goto error_start_0;

	if (data->rawStreamEnable_) {
		count = data->rawStream_.configuration().bufferCount;

		if (count) {
			ret = data->raw_->importBuffers(count);
			if (ret < 0)
				goto error_start_1;

			ret = data->raw_->streamOn();
			if (ret < 0)
				goto error_start_2;
		}
	}

	if (!data->disableISP) {
		ret = data->scDev_->allocateBuffers(STREAM_BUFFER_COUNT, &data->scBuffers_);
		if (ret < 0)
			goto error_start_3;

		uint32_t bufID = 0;
		std::vector<IPABuffer> ipaBuffers;
		for (auto &buf : data->scBuffers_) {
			buf->setCookie(bufID++);
			ipaBuffers.emplace_back(buf->cookie(), buf->planes());

			ret = data->scDev_->queueBuffer(buf.get());
			if (ret < 0)
				goto error_start_3;
		}
		data->ipa_->mapBuffers(ipaBuffers);

		data->scDev_->setFrameStartEnabled(true);

		ret = data->scDev_->streamOn();
		if (ret < 0)
			goto error_start_4;
	}

	ret = adapter_->start();
	if (ret < 0)
		goto error_start_4;

	ret = data->ipa_->start(controls ? *controls : ControlList{ controls::controls });
	if (ret < 0)
		goto error_start_4;

	data->running_ = true;

	return 0;

error_start_4:
	freeBuffers(camera);
	if (!data->disableISP) {
		data->scDev_->setFrameStartEnabled(false);
		data->scDev_->streamOff();
		data->scDev_->releaseBuffers();
		data->scBuffers_.clear();
		data->stopOhterDevice();
	}
error_start_3:
	data->raw_->streamOff();
error_start_2:
	data->raw_->releaseBuffers();
error_start_1:
	data->video_->streamOff();
error_start_0:
	data->ipa_->stop();
	data->video_->releaseBuffers();
	return ret;
}

void PipelineHandlerStarfive::stopDevice(Camera *camera)
{
	StarfiveCameraData *data = cameraData(camera);

	data->running_ = false;

	data->video_->streamOff();
	data->video_->releaseBuffers();

	if (!data->disableISP) {
		freeBuffers(camera);
		data->scDev_->setFrameStartEnabled(false);
		data->scDev_->streamOff();
		data->scDev_->releaseBuffers();
		data->scBuffers_.clear();

		data->stopOhterDevice();
	}

	if (data->haveRaw_) {
		data->raw_->streamOff();
		data->raw_->releaseBuffers();
	}
}

int PipelineHandlerStarfive::queueRequestDevice(Camera *camera, Request *request)
{
	StarfiveCameraData *data = cameraData(camera);
	int ret = 0;

	if(request->cookie()) {
		if(!request->controls().empty()) {
			data->getInfoRequest_[(uint32_t)request->cookie()] = request;
			data->ipa_->queueRequest(request->controls());
		} else {
			//data->ipa_->queueRequest(request->controls());
			completeBuffer(request, request->buffers().begin()->second);
			completeRequest(request);
		}
		return 0;
	}

	data->ipa_->queueRequest(request->controls());

	for (auto it : request->buffers()) {
		const Stream *stream = it.first;
		FrameBuffer *buffer = it.second;

		if (stream == &data->outStream_) {
			ret = data->video_->queueBuffer(buffer);
		} else if (stream == &data->rawStream_) {
			ret = data->raw_->queueBuffer(buffer);
		}
		else
			continue;
	}

	return ret;
}

int PipelineHandlerStarfive::registerCameras(MediaDevice *sfMediaDev)
{
	int ret = 0;

	std::unique_ptr<StarfiveCameraDataBase> data = Starfive::PipelineType::pplt_normal == pplType_ ?
		std::unique_ptr<StarfiveCameraDataBase>(new StarfiveCameraData(this, sfMediaDev)) :
		std::unique_ptr<StarfiveCameraDataBase>(new StarfiveSimpleCameraData(this, sfMediaDev));

	ret = data->init();
	if (ret)
		return ret;

	adapter_ = Starfive::PipelineType::pplt_normal == pplType_ ? 
		std::unique_ptr<StarfivePipelineAdapterBase>(new StarfivePipelineAdapter(data.get())) :
		std::unique_ptr<StarfivePipelineAdapterBase>(new StarfiveSimplePipelineAdapter(data.get()));

	ret = linkPipeline();
	if (ret)
		return ret;

	std::unordered_map<uint32_t, DelayedControls::ControlParams> params = {
		{ V4L2_CID_ANALOGUE_GAIN, { 1, false } },
		{ V4L2_CID_EXPOSURE, { 1, false } },
	};

	data->delayedCtrls_ = std::make_unique<DelayedControls>(data->sensor_->device(), params);

	std::set<Stream *> streams;
	streams.insert(&data->outStream_);
	if(data->haveRaw_)
		streams.insert(&data->rawStream_);

	const std::string &cameraId = data->sensor_->id();
	std::shared_ptr<Camera> camera =
		Camera::create(std::move(data), cameraId, streams);
	PipelineHandler::registerCamera(std::move(camera));

	return 0;
	
}

int PipelineHandlerStarfive::freeBuffers(Camera * camera)
{
	StarfiveCameraData *data = cameraData(camera);

	std::vector<unsigned int> ids;
	for (int i = 0; i < STREAM_BUFFER_COUNT; i++)
		ids.push_back(i);
	data->ipa_->unmapBuffers(ids);

	return 0;
}

MediaDevice *PipelineHandlerStarfive::findPipeline(DeviceEnumerator *enumerator)
{
	MediaDevice *sfMediaDev = nullptr;
	{
		DeviceMatch sf_dm("jh7110-vin");
		sf_dm.add("stf_vin0_wr_video0");
		sf_dm.add("stf_vin0_isp0_video1");
		sf_dm.add("stf_vin0_isp0_raw_video6");

		sfMediaDev = acquireMediaDevice(enumerator, sf_dm);
		if(sfMediaDev) {
			pplType_ = Starfive::PipelineType::pplt_normal;
			return sfMediaDev;
		}
	}

	{
		DeviceMatch sf_dm("starfive-camss");
    	sf_dm.add("capture_yuv");
		sfMediaDev = acquireMediaDevice(enumerator, sf_dm);
		if(sfMediaDev) {
			pplType_ = Starfive::PipelineType::pplt_simple;
			return sfMediaDev;
		}
	}

	return nullptr;
}

bool PipelineHandlerStarfive::match(DeviceEnumerator *enumerator)
{
	MediaDevice *sfMediaDev = findPipeline(enumerator);
	if (!sfMediaDev)
		return false;

	if (registerCameras(sfMediaDev))
		return false;

	return true;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerStarfive)

} /* namespace libcamera */

