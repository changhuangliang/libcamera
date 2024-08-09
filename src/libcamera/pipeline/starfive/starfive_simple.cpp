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
#include "linux/jh7110-isp.h"

namespace libcamera {

#define ISP_OUTPUT_PARAMS_MBUSCODE                  0x7001
#define ISP_YUV_OUTPUT_MBUSCODE                     0x2004
#define ISP_SC_OUTPUT_MBUSCODE                      0x7001

#define V4L2_META_FMT_STF_ISP_PARAMS	            v4l2_fourcc('S', 'T', 'F', 'P')
#define V4L2_META_FMT_STF_ISP_STAT_3A               v4l2_fourcc('S', 'T', 'F', 'S')

#define REGISTER_BUFFER_NUMBER                      4

namespace starfive::control {

const Control<Span<const uint8_t, sizeof(jh7110_isp_wb_setting)>> SFWBGainCtrl(V4L2_CID_USER_JH7110_ISP_WB_SETTING, "StarfiveColourGains");
const Control<Span<const uint8_t, sizeof(jh7110_isp_car_setting)>> SFCarCtrl(V4L2_CID_USER_JH7110_ISP_CAR_SETTING, "StarfiveCar");
const Control<Span<const uint8_t, sizeof(jh7110_isp_ccm_setting)>> SFCCMCtrl(V4L2_CID_USER_JH7110_ISP_CCM_SETTING, "StarfiveCCM");
const Control<Span<const uint8_t, sizeof(jh7110_isp_cfa_setting)>> SFCFACtrl(V4L2_CID_USER_JH7110_ISP_CFA_SETTING, "StarfiveCFA");
const Control<Span<const uint8_t, sizeof(jh7110_isp_ctc_setting)>> SFCTCCtrl(V4L2_CID_USER_JH7110_ISP_CTC_SETTING, "StarfiveCTC");
const Control<Span<const uint8_t, sizeof(jh7110_isp_dbc_setting)>> SFDBCCtrl(V4L2_CID_USER_JH7110_ISP_DBC_SETTING, "StarfiveDBC");
const Control<Span<const uint8_t, sizeof(jh7110_isp_dnyuv_setting)>> SFDNYUVCtrl(V4L2_CID_USER_JH7110_ISP_DNYUV_SETTING, "StarfiveDNYUV");
const Control<Span<const uint8_t, sizeof(jh7110_isp_gmargb_setting)>> SFGMARGBCtrl(V4L2_CID_USER_JH7110_ISP_GMARGB_SETTING, "StarfiveGMARGB");
const Control<Span<const uint8_t, sizeof(jh7110_isp_lccf_setting)>> SFLCCFCtrl(V4L2_CID_USER_JH7110_ISP_LCCF_SETTING, "StarfiveLCCF");
const Control<Span<const uint8_t, sizeof(jh7110_isp_blacklevel_setting)>> SFOBCCtrl(V4L2_CID_USER_JH7110_ISP_OBC_SETTING, "StarfiveOBC");
const Control<Span<const uint8_t, sizeof(jh7110_isp_oecf_setting)>> SFOECFCtrl(V4L2_CID_USER_JH7110_ISP_OECF_SETTING, "StarfiveOECF");
const Control<Span<const uint8_t, sizeof(jh7110_isp_r2y_setting)>> SFR2YCtrl(V4L2_CID_USER_JH7110_ISP_R2Y_SETTING, "StarfiveR2Y");
const Control<Span<const uint8_t, sizeof(jh7110_isp_sat_setting)>> SFSATCtrl(V4L2_CID_USER_JH7110_ISP_SAT_SETTING, "StarfiveSAT");
const Control<Span<const uint8_t, sizeof(jh7110_isp_sharp_setting)>> SFSHRPCtrl(V4L2_CID_USER_JH7110_ISP_SHRP_SETTING, "StarfiveSHRP");
const Control<Span<const uint8_t, sizeof(jh7110_isp_ycrv_setting)>> SFYCRVCtrl(V4L2_CID_USER_JH7110_ISP_YCRV_SETTING, "StarfiveYCRV");
const Control<Span<const uint8_t, sizeof(jh7110_isp_sc_setting)>> SFSCCtrl(V4L2_CID_USER_JH7110_ISP_STAT_SETTING, "StarfiveSC");
const Control<Span<const uint8_t, sizeof(jh7110_isp_outss_setting)>> SFOUTSS0Ctrl(V4L2_CID_USER_JH7110_ISP_OUTSS0_SETTING, "StarfiveSS0");
const Control<Span<const uint8_t, sizeof(jh7110_isp_outss_setting)>> SFOUTSS1Ctrl(V4L2_CID_USER_JH7110_ISP_OUTSS1_SETTING, "StarfiveSS1");

const ControlIdMap ispControls {
    { V4L2_CID_USER_JH7110_ISP_WB_SETTING, &SFWBGainCtrl },
    { V4L2_CID_USER_JH7110_ISP_CAR_SETTING, &SFCarCtrl },
    { V4L2_CID_USER_JH7110_ISP_CCM_SETTING, &SFCCMCtrl },
    { V4L2_CID_USER_JH7110_ISP_CFA_SETTING, &SFCFACtrl },
    { V4L2_CID_USER_JH7110_ISP_CTC_SETTING, &SFCTCCtrl },
    { V4L2_CID_USER_JH7110_ISP_DBC_SETTING, &SFDBCCtrl },
    { V4L2_CID_USER_JH7110_ISP_DNYUV_SETTING, &SFDNYUVCtrl },
    { V4L2_CID_USER_JH7110_ISP_GMARGB_SETTING, &SFGMARGBCtrl },
    { V4L2_CID_USER_JH7110_ISP_LCCF_SETTING, &SFLCCFCtrl },
    { V4L2_CID_USER_JH7110_ISP_OBC_SETTING, &SFOBCCtrl },
    { V4L2_CID_USER_JH7110_ISP_OECF_SETTING, &SFOECFCtrl },
    { V4L2_CID_USER_JH7110_ISP_R2Y_SETTING, &SFR2YCtrl },
    { V4L2_CID_USER_JH7110_ISP_SAT_SETTING, &SFSATCtrl },
    { V4L2_CID_USER_JH7110_ISP_SHRP_SETTING, &SFSHRPCtrl },
    { V4L2_CID_USER_JH7110_ISP_YCRV_SETTING, &SFYCRVCtrl },
    { V4L2_CID_USER_JH7110_ISP_STAT_SETTING, &SFSCCtrl },
    //{ V4L2_CID_USER_JH7110_ISP_OUTSS0_SETTING, &SFOUTSS0Ctrl },
    //{ V4L2_CID_USER_JH7110_ISP_OUTSS1_SETTING, &SFOUTSS1Ctrl }
};

static const ControlInfoMap::Map ipaControls{
    { &SFWBGainCtrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SFCarCtrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SFCCMCtrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SFCFACtrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SFCTCCtrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SFDBCCtrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SFDNYUVCtrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SFGMARGBCtrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SFLCCFCtrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SFOBCCtrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SFOECFCtrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SFR2YCtrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SFSATCtrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SFSHRPCtrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SFYCRVCtrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SFSCCtrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    //{ &SFOUTSS0Ctrl, ControlInfo((uint8_t)0, (uint8_t)0) },
    //{ &SFOUTSS1Ctrl, ControlInfo((uint8_t)0, (uint8_t)0) }
};

} // namespace starfive::control

int StarfiveSimpleCameraData::init()
{
    int ret = 0;

    MediaEntity *csiEntity = media_->getEntityByName("cdns_csi2rx.19800000.csi");
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
        videoOutputParams_ = V4L2VideoDevice::fromEntityName(media_, "output_params");

        ret = videoOutputParams_->open();
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

void StarfiveSimpleCameraData::connectVideoSignal()
{
    StarfiveCameraDataBase::connectVideoSignal();
    videoOutputParams_->bufferReady.connect(this, &StarfiveSimpleCameraData::regBufferReady);
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
    std::vector<uint32_t> isp1Mbcode = {ISP_YUV_OUTPUT_MBUSCODE};
    mbufCodeLink_.push_back(isp1Mbcode);

    //collectISPSizeRange();
    setSizeRange(Size(1920, 1080));
}

void StarfiveSimpleCameraData::regBufferReady(FrameBuffer *buffer)
{
    freeOPBufferCookie_.push_back(buffer->cookie());
}

void StarfiveSimpleCameraData::processAGCControls(const ControlList &controls)
{
    delayedCtrls_->push(controls);
    delayedCtrls_->applyControls(scSequence_);
}

struct CtrlInBufInfo
{
    uint32_t positionFlag;
    uint32_t offset;
};

static const CtrlInBufInfo CtrlInBufInfos[] =
{
    {JH7110_ISP_MODULE_WB_SETTING, offsetof(jh7110_isp_params_buffer, wb_setting)},
    {JH7110_ISP_MODULE_CAR_SETTING, offsetof(jh7110_isp_params_buffer, car_setting)},
    {JH7110_ISP_MODULE_CCM_SETTING, offsetof(jh7110_isp_params_buffer, ccm_setting)},
    {JH7110_ISP_MODULE_CFA_SETTING, offsetof(jh7110_isp_params_buffer, cfa_setting)},
    {JH7110_ISP_MODULE_CTC_SETTING, offsetof(jh7110_isp_params_buffer, ctc_setting)},
    {JH7110_ISP_MODULE_DBC_SETTING, offsetof(jh7110_isp_params_buffer, dbc_setting)},
    {JH7110_ISP_MODULE_DNYUV_SETTING, offsetof(jh7110_isp_params_buffer, dnyuv_setting)},
    {JH7110_ISP_MODULE_GMARGB_SETTING, offsetof(jh7110_isp_params_buffer, gmargb_setting)},
    {JH7110_ISP_MODULE_LCCF_SETTING, offsetof(jh7110_isp_params_buffer, lccf_setting)},
    {JH7110_ISP_MODULE_OBC_SETTING, offsetof(jh7110_isp_params_buffer, obc_setting)},
    {JH7110_ISP_MODULE_OECF_SETTING, offsetof(jh7110_isp_params_buffer, oecf_setting)},
    {JH7110_ISP_MODULE_R2Y_SETTING, offsetof(jh7110_isp_params_buffer, r2y_setting)},
    {JH7110_ISP_MODULE_SAT_SETTING, offsetof(jh7110_isp_params_buffer, sat_setting)},
    {JH7110_ISP_MODULE_SHARP_SETTING, offsetof(jh7110_isp_params_buffer, sharp_setting)},
    {JH7110_ISP_MODULE_YCRV_SETTING, offsetof(jh7110_isp_params_buffer, ycrv_setting)},
    {JH7110_ISP_MODULE_SC_SETTING, offsetof(jh7110_isp_params_buffer, sc_setting)},
};

void StarfiveSimpleCameraData::processIPAControls(ControlList &ctrlList)
{
    if(freeOPBufferCookie_.empty())
        LOG(STARFIVE, Error) << "No free output parameter buffer.";

    uint32_t cookie = freeOPBufferCookie_.front();
    auto bmIt = opBufferMaps_.find(cookie);
	if (bmIt == opBufferMaps_.end()) {
		LOG(STARFIVE, Error) << "Could not find stats buffer!";
		return;
	}
    Span<uint8_t> mem = bmIt->second.planes()[0];
    struct jh7110_isp_params_buffer *parBuf = (struct jh7110_isp_params_buffer *)mem.data();

    parBuf->enable_setting = 0;
    for(const auto &ctrl: ctrlList) {
        auto control = starfive::control::ispControls.find(ctrl.first);
        if(control != starfive::control::ispControls.end()) {
            const ControlValue &value = ctrl.second;
            const Span<const uint8_t> &spanData = value.get<Span<const uint8_t>>();
            size_t parSize = spanData.size_bytes();
            const uint8_t *data = spanData.data();
            int index = ctrl.first - V4L2_CID_USER_JH7110_ISP_WB_SETTING;
            const CtrlInBufInfo *cibInfo = &CtrlInBufInfos[index];

            /* fill struct jh7110_isp_params_buffer */
            parBuf->enable_setting |= cibInfo->positionFlag;
            memcpy(mem.data() + cibInfo->offset, data, parSize);
        }
    }

    if(parBuf->enable_setting) {
        freeOPBufferCookie_.pop_front();
        opBuffers_[cookie]->_d()->metadata().planes()[0].bytesused = sizeof(struct jh7110_isp_params_buffer);
        videoOutputParams_->queueBuffer(opBuffers_[cookie].get());
    }
}

StarfiveSimplePipelineAdapter::StarfiveSimplePipelineAdapter(StarfiveCameraDataBase *data)
		: StarfivePipelineAdapterBase(data)
{
    ControlInfoMap::Map ctrlMap = starfive::control::ipaControls;
    const ControlInfoMap &driverControls = data_->ispSubDev_->controls();
    ControlIdMap idMap = driverControls.idmap();

    ctrlMap.insert(driverControls.begin(), driverControls.end());
    idMap.insert(starfive::control::ispControls.begin(), starfive::control::ispControls.end());
    localCtrlMap_ = ControlInfoMap(std::move(ctrlMap), idMap);
}

const ControlInfoMap &StarfiveSimplePipelineAdapter::getISPControls()
{
    return localCtrlMap_;
}

PixelFormat StarfiveSimplePipelineAdapter::getSCPixFormat([[maybe_unused]] Size size)
{
    return PixelFormat(V4L2_META_FMT_STF_ISP_PARAMS, (uint64_t)0);
}

int StarfiveSimplePipelineAdapter::linkPipeline()
{
    StarfiveSimpleCameraData *data = dynamic_cast<StarfiveSimpleCameraData *>(data_);
	MediaDevice *sfMediaDev = data->media_;
	int ret = 0;

	if (!data->disableISP) {
		MediaLink *ispLink = sfMediaDev->link("cdns_csi2rx.19800000.csi", 1, "capture_raw", 0);
		if (ispLink) {
			ret = ispLink->setEnabled(false);
			if (ret)
				return ret;
		}
        ispLink = sfMediaDev->link("stf_isp", 1, "output_params", 0);
		if (ispLink) {
			ret = ispLink->setEnabled(true);
			if (ret)
				return ret;
		}
        ispLink = sfMediaDev->link("stf_isp", 2, "capture_yuv", 0);
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
        ispLink = sfMediaDev->link("stf_isp", 1, "output_params", 0);
		if (ispLink) {
			ret = ispLink->setEnabled(false);
			if (ret)
				return ret;
		}
        ispLink = sfMediaDev->link("cdns_csi2rx.19800000.csi", 1, "capture_raw", 0);
		if (ispLink) {
			ret = ispLink->setEnabled(true);
			if (ret)
				return ret;
		}
    }

	return 0;
}

int StarfiveSimplePipelineAdapter::setupFormats(const V4L2DeviceFormat videoFormat)
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
    ret = data->ispSubDev_->setFormat(2, &curFormat);
    if (ret){
        return -EINVAL;
    }
	if (!data->disableISP) {
        V4L2DeviceFormat oVideoFormat = {};
        oVideoFormat.fourcc = libcamera::V4L2PixelFormat(V4L2_META_FMT_STF_ISP_PARAMS);
        oVideoFormat.size = format.size;
        ret = data->videoOutputParams_->setFormat(&oVideoFormat);
        if (ret)
            return ret;

        curFormat.mbus_code = ISP_SC_OUTPUT_MBUSCODE;
        curFormat.size = format.size;
        ret = data->ispSubDev_->setFormat(1, &curFormat);
        if (ret)
            return -EINVAL;

        curFormat.mbus_code = ISP_OUTPUT_PARAMS_MBUSCODE;
        curFormat.size = format.size;
        ret = data->ispSubDev_->setFormat(3, &curFormat);
        if (ret)
            return -EINVAL;

        oVideoFormat.fourcc = libcamera::V4L2PixelFormat(V4L2_META_FMT_STF_ISP_STAT_3A);
        oVideoFormat.size = format.size;
        ret = data->scDev_->setFormat(&oVideoFormat);
        if (ret)
            return ret;
        data->setSCFormat_ = true;

        const std::vector<uint32_t> &csiMbusCode = data->mbufCodeLink_.at(data->mbufCodeLink_.size() - 2);
        //V4L2Subdevice::Formats ispSDFormat = data->ispSubDev_->formats(0);
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
        V4L2Subdevice::Formats ispVideoFormat = data->ispSubDev_->formats(1);
        for(const uint32_t &code : mbusCode) {
            std::vector<SizeRange> &srs = ispVideoFormat.at(code);
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

int StarfiveSimplePipelineAdapter::start()
{
    StarfiveSimpleCameraData *data = dynamic_cast<StarfiveSimpleCameraData *>(data_);
    int ret = 0;

    if (!data->disableISP) {
        ret = data->videoOutputParams_->allocateBuffers(STREAM_BUFFER_COUNT, &data->opBuffers_);
		if (ret < 0)
			return ret;

        data->freeOPBufferCookie_.clear();
		uint32_t bufID = 0;
		for (auto &buf : data->opBuffers_) {
			buf->setCookie(bufID);
			data->opBufferMaps_.emplace(bufID,
				 MappedFrameBuffer(buf.get(), MappedFrameBuffer::MapFlag::ReadWrite));
            data->freeOPBufferCookie_.push_back(bufID++);
		}

		ret = data->videoOutputParams_->streamOn();
		if (ret < 0)
			return ret;
    }

    return 0;
}

} // namespace libcamera