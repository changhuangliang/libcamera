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
		: Camera::Private(pipe), disableISP(false), media_(media), scSequence_(0)
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
            scSequence_ = buffer->metadata().sequence;
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
        //delayedCtrls_->push(controls);
        processAGCControls(controls);
    }

	void setIspControls(const ControlList &controls)
    {
        ControlList ctrlList = controls;

	    //ispSubDev_->setControls(&ctrlList);
        processIPAControls(ctrlList);
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

    virtual void stopOhterDevice() {};

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

    virtual void processAGCControls(const ControlList &controls)
    {
        delayedCtrls_->push(controls);
    }

    virtual void processIPAControls(ControlList &ctrlList)
    {
        ispSubDev_->setControls(&ctrlList);
    }

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

    uint32_t scSequence_;
};

class StarfivePipelineAdapterBase
{
public:
	StarfivePipelineAdapterBase(StarfiveCameraDataBase *data) : data_(data)
	{
	}

	virtual ~StarfivePipelineAdapterBase(){}

    virtual const ControlInfoMap &getISPControls()
    {
        return data_->ispSubDev_->controls();
    }

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

    virtual int start() { return 0; };

protected:
	StarfiveCameraDataBase *const data_;

private:
	StarfivePipelineAdapterBase() = default;
};

class StarfiveSimpleCameraData : public StarfiveCameraDataBase
{
public:
	StarfiveSimpleCameraData(PipelineHandler *pipe, MediaDevice *media)
		: StarfiveCameraDataBase(pipe, media)
	{
	}

    ~StarfiveSimpleCameraData() 
    {
        opBufferMaps_.clear();
    }

    int init() override;

    void stopOhterDevice() override
    {
        videoOutputParams_->streamOff();
        videoOutputParams_->releaseBuffers();
        opBufferMaps_.clear();
        opBuffers_.clear();
        freeOPBufferCookie_.clear();
    }

public:
    std::unique_ptr<V4L2VideoDevice> videoOutputParams_;
    std::vector<std::unique_ptr<FrameBuffer>> opBuffers_;
    std::map<uint32_t, MappedFrameBuffer> opBufferMaps_;
    std::list<uint32_t> freeOPBufferCookie_;

protected:

    void connectVideoSignal() override;

    void collectCompMbusCode() override;
    void processIPAControls(ControlList &ctrlList) override;
    void processAGCControls(const ControlList &controls) override;

	void setSizeRange(Size sz) {
		ispSizeRange_.push_back(sz);
		maxISPSize_ = sz;
	}

    void regBufferReady(FrameBuffer *buffer);

private:
    std::vector<std::unique_ptr<FrameBuffer>> registerBuffers_;
    std::unordered_map<uint32_t, MappedFrameBuffer> registerBufferMaps_;
    std::list<uint32_t> freeRegBufID_;
};

class StarfiveSimplePipelineAdapter : public StarfivePipelineAdapterBase
{
public:
    StarfiveSimplePipelineAdapter(StarfiveCameraDataBase *data);

    const ControlInfoMap &getISPControls() override;
    PixelFormat getSCPixFormat([[maybe_unused]] Size size) override;

	int linkPipeline() override;
	int setupFormats(const V4L2DeviceFormat videoFormat) override;

    int start() override;

private:
    ControlInfoMap localCtrlMap_;
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
