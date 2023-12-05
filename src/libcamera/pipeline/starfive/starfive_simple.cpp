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

//class PipelineHandlerStarfiveSimple;

int StarfiveSimpleCameraData::init() 
{
    int ret = 0;

    MediaEntity *csiEntity = media_->getEntityByName("cdns_csi2rx.19800000.csi-bridge");
    const std::vector<MediaPad *> &csiPads = csiEntity->pads();
    MediaEntity *sensorEntity = nullptr;

    // Try to get the sensor entity first.
    if (!csiPads.empty()) {
        const std::vector<MediaLink *> &links = csiPads[0]->links();
        if (!links.empty()) {
            sensorEntity = links[0]->source()->entity();
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
    
    csiSubDev_ = std::make_unique<V4L2Subdevice>(csiEntity);
    ret = csiSubDev_->open();
    if (ret)
        return ret;

    if (!disableISP) {
        // Use the StarFive ISP pipeline.
        ispSubDev_ = std::make_unique<V4L2Subdevice>(media_->getEntityByName("stf_isp"));
        ret = ispSubDev_->open();
        if (ret)
            return ret;

        video_ = V4L2VideoDevice::fromEntityName(media_, "capture_yuv");
        ret = video_->open();
        if (ret)
            return ret;

        scDev_ = V4L2VideoDevice::fromEntityName(media_, "capture_scd");
        ret = scDev_->open();
        if (ret)
            return ret;

        connectSCSignal();
    } else {
        // Use the sensor own isp pipeline.
        video_ = V4L2VideoDevice::fromEntityName(media_, "capture_raw");
        ret = video_->open();
        if (ret)
            return ret;
    }

    //raw_ = V4L2VideoDevice::fromEntityName(media_, "stf_vin0_isp0_raw_video6");
    //ret = raw_->open();
    //if (!ret)
    //    haveRaw_ = true;

    connectVideoSignal();

    collectCompMbusCode();

    properties_ = sensor_->properties();

    return 0;
}

void StarfiveSimpleCameraData::collectCompMbusCode()
{
    const std::vector<unsigned int> &mbusCodes = sensor_->mbusCodes();
    std::vector<unsigned int> sinkCodes;

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

    //auto takeMatchCode = [](std::vector<unsigned int> &codes, const V4L2Subdevice::Formats &fmts)
    //    -> const std::vector<unsigned int> & {
    //    std::vector<unsigned int> resCodes;
//
    //    for (const uint32_t code : codes) {
    //        auto it = fmts.find(code);
    //        if (it != fmts.end())
    //            resCodes.push_back(code);
    //    }
    //    codes = std::move(resCodes);
    //    return codes;
    //};

    // Match the sensor to the CSI/DVP.
    //V4L2Subdevice::Formats fmts = csiSubDev_->formats(0);
    //mbufCodeLink_.push_back(takeMatchCode(sinkCodes, fmts));

    //if (!disableISP) {
    //    // Match the sensor to the CSI/DVP.
    //    V4L2Subdevice::Formats fmts = ispSubDev_->formats(0);
    //    mbufCodeLink_.push_back(takeMatchCode(sinkCodes, fmts));
    //    fmts = ispSubDev_->formats(1);
    //    mbufCodeLink_.push_back(utils::map_keys(fmts));
    //} else
    //    mbufCodeLink_.push_back(sinkCodes);

    mbufCodeLink_.push_back(sinkCodes);
    std::vector<uint32_t> isp1Mbcode = {0x2004};
    mbufCodeLink_.push_back(isp1Mbcode);

    //collectISPSizeRange();
    setSizeRange(Size(1920, 1080));
}

int StarfivSimplePipelineAdapter::linkPipeline()
{
    StarfiveSimpleCameraData *data = dynamic_cast<StarfiveSimpleCameraData *>(data_);
	MediaDevice *sfMediaDev = data->media_;
	int ret = 0;

	if (!data->disableISP) {
		MediaLink *ispLink = sfMediaDev->link("stf_isp", 1, "capture_raw", 0);
		if (ispLink) {
			ret = ispLink->setEnabled(false);
			if (ret)
				return ret;
		}
        ispLink = sfMediaDev->link("stf_isp", 1, "capture_yuv", 0);
		if (ispLink) {
			ret = ispLink->setEnabled(true);
			if (ret)
				return ret;
		}
		ispLink = sfMediaDev->link("capture_yuv", 2, "capture_scd", 0);
		if (ispLink) {
			ret = ispLink->setEnabled(true);
			if (ret)
				return ret;
		}
	} else {
        MediaLink *ispLink = sfMediaDev->link("stf_isp", 1, "capture_yuv", 0);
		if (ispLink) {
			ret = ispLink->setEnabled(false);
			if (ret)
				return ret;
		}
        ispLink = sfMediaDev->link("stf_isp", 1, "capture_raw", 0);
		if (ispLink) {
			ret = ispLink->setEnabled(true);
			if (ret)
				return ret;
		}
    }
	
	return 0;
}

int StarfivSimplePipelineAdapter::setupFormats(const V4L2DeviceFormat videoFormat)
{
    StarfiveSimpleCameraData *data = dynamic_cast<StarfiveSimpleCameraData *>(data_);
	if (!data->mbufCodeLink_.size())
		return 0;

	V4L2SubdeviceFormat format{};
	format.mbus_code = 0;
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

    int ret = 0;

	V4L2SubdeviceFormat curFormat = format;
    ret = data->ispSubDev_->setFormat(1, &curFormat);
    if (ret){
        return -EINVAL;
    }
	if (!data->disableISP) {
        V4L2Subdevice::Formats ispSDFormat = data->ispSubDev_->formats(2);
        curFormat.mbus_code = ispSDFormat.begin()->first;
        curFormat.size = format.size;
        ret = data->ispSubDev_->setFormat(2, &curFormat);
        if (ret)
            return -EINVAL;

        const std::vector<uint32_t> &csiMbusCode = data->mbufCodeLink_.at(data->mbufCodeLink_.size() - 2);
        ispSDFormat = data->ispSubDev_->formats(0);
        //for(const uint32_t &code : csiMbusCode) {
            //std::vector<SizeRange> &srs = ispSDFormat.at(code);
            //for(auto &sr : srs) {
            //    if(sr.contains(format.size)) {
                    curFormat.mbus_code = csiMbusCode[0];
                    curFormat.size = format.size;
                    ret = data->ispSubDev_->setFormat(0, &curFormat);
                    if (ret)
                        return -EINVAL;
                    ret = data->csiSubDev_->setFormat(0, &curFormat);
                    if (ret)
                        return -EINVAL;
                    ret = data->csiSubDev_->setFormat(1, &curFormat);
                    if (ret)
                        return -EINVAL;
                    return 0;
            //    }
            //}
        //}
	} else {
        V4L2Subdevice::Formats ispSDFormat = data->ispSubDev_->formats(1);
        for(const uint32_t &code : mbusCode) {
            std::vector<SizeRange> &srs = ispSDFormat.at(code);
            for(auto &sr : srs) {
                if(sr.contains(format.size)) {
                    curFormat.mbus_code = code;
                    curFormat.size = format.size;
                    ret = data->csiSubDev_->setFormat(0, &curFormat);
                    if (ret)
                        return -EINVAL;
                    ret = data->csiSubDev_->setFormat(1, &curFormat);
                    if (ret)
                        return -EINVAL;
                    break;
                }
            }
        }
    }

	return 0;
}


} // namespace libcamera