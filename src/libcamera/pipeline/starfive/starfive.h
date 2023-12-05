/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Starfive Technology Co., Ltd.
 *
 * starfive.h - starfive basic ISP pipeline definition
 */
#pragma once

namespace libcamera {

const LogCategory &logCategorySTARFIVE();

#define STREAM_BUFFER_COUNT						4
#define STARFIVE_ISP_MAX_WIDTH					1920
#define STARFIVE_ISP_MAX_HEIGHT					1080

class StarfiveCameraDataBase : public Camera::Private
{
public:
	StarfiveCameraDataBase(PipelineHandler *pipe, MediaDevice *media)
		: Camera::Private(pipe), disableISP(false), media_(media)
	{
	}

	virtual int init() = 0;

	int loadIPA()
    {
        ipa_ = IPAManager::createIPA<ipa::starfive::IPAProxyStarfive>(pipe(), 1, 1);
        int ret = 0;

        if (!ipa_) {
            LOG(STARFIVE, Error) << "Can not create the IPA object.";
            return -ENOENT;
        }

        IPACameraSensorInfo sensorInfo{};
        ret = sensor_->sensorInfo(&sensorInfo);
        if (ret)
            return ret;

        std::string fileName = sensor_->model() + ".json";
        std::string configurationFile = ipa_->configurationFile(fileName);
        IPASettings settings(configurationFile, sensor_->model());

        connectIPASignal();

        return ipa_->init(settings, sensorInfo, sensor_->controls());
    }

	std::vector<SizeRange> sensorSizes() const
    {
        std::vector<SizeRange> sizes;

        for (const Size &size : sensor_->sizes(sensor_->mbusCodes().at(0)))
            sizes.emplace_back(size, size);

        return sizes;
    }

	void bufferReady(FrameBuffer *buffer)
    {
        PipelineHandler *pipe = Camera::Private::pipe();
        Request *request = buffer->request();

        request->metadata().set(controls::SensorTimestamp,
                    buffer->metadata().timestamp);

        if (!pipe->completeBuffer(request, buffer))
            return;

        pipe->completeRequest(request);
    }

	void scBufferReady(FrameBuffer *buffer)
    {
        if (running_) {
            ipa_->statBufferReady(buffer->cookie(), delayedCtrls_->get(buffer->metadata().sequence));

            scDev_->queueBuffer(buffer);
        }
    }

	void frameStarted(uint32_t sequence)
    {
        delayedCtrls_->applyControls(sequence);
    }

	void setDelayedControls(const ControlList &controls)
    {
        delayedCtrls_->push(controls);
    }

	void setIspControls(const ControlList &controls)
    {
        ControlList ctrlList = controls;

	    ispSubDev_->setControls(&ctrlList);
    }

	bool findMatchVideoFormat(const PixelFormat &pixelFormat, const Size &size, V4L2PixelFormat &format)
    {
        if (!mbufCodeLink_.size())
            return false;

        const std::vector<uint32_t> &mbusCodes = mbufCodeLink_.back();
        V4L2VideoDevice::Formats fmts;

        for (auto &code : mbusCodes) {
            fmts = video_->formats(code);
            for (const auto &cur : fmts) {
                if (!pixelFormat.isValid() || cur.first.toPixelFormat() == pixelFormat) {
                    for (auto &sr : cur.second) {
                        if (sr.contains(size)) {
                            format = cur.first;
                            return true;
                        }
                    }
                }
            }
        }

        return false;
    }

	Size getSuitableSensorSize()
    {
        Size sensorResolution = sensor_->resolution();
        if(sensorResolution.width <= STARFIVE_ISP_MAX_WIDTH && sensorResolution.height <= STARFIVE_ISP_MAX_HEIGHT)
            return sensorResolution;

        return maxISPSize_;
    }

	bool isValidSize(const Size &size)
    {
        if (!ispSizeRange_.size())
            return false;
        for(const Size &sz : ispSizeRange_) {
            if(sz.width == size.width && sz.height == size.height)
                return true;
        }
        return false;
    }

public:
    bool disableISP;

	std::unique_ptr<ipa::starfive::IPAProxyStarfive> ipa_;

	MediaDevice *media_;
	std::unique_ptr<V4L2VideoDevice> video_;
	std::unique_ptr<V4L2VideoDevice> raw_;
    std::unique_ptr<V4L2VideoDevice> scDev_;
	std::unique_ptr<V4L2Subdevice> csiSubDev_;
	std::unique_ptr<V4L2Subdevice> ispSubDev_;
	std::unique_ptr <CameraSensor> sensor_;
	Stream outStream_;
	Stream rawStream_;

	bool haveRaw_ = false;
	bool rawStreamEnable_ = false;
	bool setSCFormat_ = false;

	std::vector<std::vector<uint32_t>> mbufCodeLink_;
	std::vector<Size> ispSizeRange_;

	std::vector<std::unique_ptr<FrameBuffer>> scBuffers_;

	std::unique_ptr<DelayedControls> delayedCtrls_;

	bool running_ = false;

protected:
    virtual void connectSCSignal()
    {
        scDev_->bufferReady.connect(this, &StarfiveCameraDataBase::scBufferReady);
        scDev_->frameStart.connect(this, &StarfiveCameraDataBase::frameStarted);
    }

    virtual void connectVideoSignal()
    {
        video_->bufferReady.connect(this, &StarfiveCameraDataBase::bufferReady);
        if(haveRaw_)
            raw_->bufferReady.connect(this, &StarfiveCameraDataBase::bufferReady);
    }

    virtual void connectIPASignal()
    {
        ipa_->setDelayedControls.connect(this, &StarfiveCameraDataBase::setDelayedControls);
        ipa_->setIspControls.connect(this, &StarfiveCameraDataBase::setIspControls);
    }

	virtual void collectCompMbusCode() = 0;

	void collectISPSizeRange()
    {
        uint64_t maxArea = 0;

        if (!mbufCodeLink_.size())
            return;

        const std::vector<uint32_t> &mbusCodes = mbufCodeLink_.front();
        for (const uint32_t &code : mbusCodes) {
            std::vector<Size> allSize = sensor_->sizes(code);

            if (!allSize.size())
                continue;

            for(const Size &sz : allSize) {
                if(sz.width <= STARFIVE_ISP_MAX_WIDTH && sz.height <= STARFIVE_ISP_MAX_HEIGHT) {
                    uint64_t area = sz.width * sz.height;
                    ispSizeRange_.push_back(sz);
                    if(area > maxArea) {
                        maxArea = area;
                        maxISPSize_ = sz;
                    }
                }
            }
        }
    }

protected:
	Size maxISPSize_;
};

class StarfivPipelineAdapterBase
{
public:
	StarfivPipelineAdapterBase(StarfiveCameraDataBase *data) : data_(data)
	{
	}

	virtual ~StarfivPipelineAdapterBase(){}

	virtual uint32_t mbusCodeTrans(uint32_t code) {return code;};
	virtual int setRawSubDevFormat([[maybe_unused]] V4L2SubdeviceFormat &sensorFormat) {return 0;};
	virtual int setSSOutFormat([[maybe_unused]] V4L2DeviceFormat &format)
	{
		return 0;
	}
	virtual PixelFormat getSCPixFormat([[maybe_unused]] Size size)
	{
		V4L2VideoDevice::Formats formats = data_->scDev_->formats();
		if(!formats.empty())
			return formats.begin()->first.toPixelFormat();
		else
			return PixelFormat();
	}

	virtual int linkPipeline() = 0;
	virtual int setupFormats(const V4L2DeviceFormat videoFormat) = 0;

protected:
	StarfiveCameraDataBase *const data_;

private:
	StarfivPipelineAdapterBase() = default;
};

class StarfiveSimpleCameraData : public StarfiveCameraDataBase
{
public:
	StarfiveSimpleCameraData(PipelineHandler *pipe, MediaDevice *media)
		: StarfiveCameraDataBase(pipe, media)
	{
	}

    ~StarfiveSimpleCameraData() {};

    int init() override;

protected:

    void collectCompMbusCode() override;
	void setSizeRange(Size sz) {
		ispSizeRange_.push_back(sz);
		maxISPSize_ = sz;
	}
};

class StarfivSimplePipelineAdapter : public StarfivPipelineAdapterBase
{
public:
	StarfivSimplePipelineAdapter(StarfiveCameraDataBase *data)
		: StarfivPipelineAdapterBase(data)
	{
	}

	int linkPipeline() override;
	int setupFormats(const V4L2DeviceFormat videoFormat) override;
};


namespace Starfive
{
enum PipelineType
{
	pplt_normal = 0,
	pplt_simple,
	pplt_unknown
};
} // namespace Starfive

} // namespace libcamera
