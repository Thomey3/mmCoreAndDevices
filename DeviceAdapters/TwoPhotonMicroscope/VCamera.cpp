#include "VCamera.h"

const char* g_DeviceNameVCamera = "Camera";


VCamera::VCamera():
	channelData(4),
	initialized_(false),
	bitDepth_(8),
	roiX_(0),
	roiY_(0),
	isSequenceable_(false),
	sequenceMaxLength_(100),
	sequenceRunning_(false),
	sequenceIndex_(0)
{
	thd_ = new LiveThread(this);
	CreateHubIDProperty();
}

VCamera::~VCamera()
{
	Stop();
	delete thd_;
}

int VCamera::Initialize() {
	initialized_ = true;
	// initialize image buffer
	GenerateEmptyImage(img_);
	return DEVICE_OK;
}

int VCamera::Shutdown() {

}

void VCamera::GenerateEmptyImage(ImgBuffer& img)
{
	MMThreadGuard g(imgPixelsLock_);
	if (img.Height() == 0 || img.Width() == 0 || img.Depth() == 0)
		return;
	unsigned char* pBuf = const_cast<unsigned char*>(img.GetPixels());
	memset(pBuf, 0, img.Height() * img.Width() * img.Depth());
}

int VCamera::SnapImage()
{
	static int callCounter = 0;
	++callCounter;

	sequence = false;
	// 准备scanner的电压
	int err = GetHub()->GalvoAOSequence();
	if (!err)
	{
		return err;
	}
    // daq准备采集，等待触发
	err = GetHub()->DAQStart();
	if (!err)
	{
		return err;
	}
	// 输出触发
	err = GetHub()->TriggerDOSequence();
	if (!err)
	{
		return err;
	}

	return DEVICE_OK;
}

const unsigned char* VCamera::GetImageBuffer()
{
	MMThreadGuard g(imgPixelsLock_);

	// 等待图像采集完成
	while (!GetHub()->waitDAQ())
	{
		// 你可以选择在这里稍微休眠一下，以减少CPU的占用
		CDeviceUtils::SleepMs(1);
	}
	imageDataProcess();
	unsigned char* data = getChannelData(0);
	return data;
}

const unsigned char* VCamera::GetImageBuffer(unsigned channelNr)
{
	MMThreadGuard g(imgPixelsLock_);

	// 等待图像采集完成
	while (!GetHub()->waitDAQ())
	{
		// 你可以选择在这里稍微休眠一下，以减少CPU的占用
		CDeviceUtils::SleepMs(1);
	}
	imageDataProcess();
	unsigned char* data = getChannelData(channelNr);
	return data;
}

unsigned VCamera::GetImageWidth() const {
	return img_.Width();
}
unsigned VCamera::GetImageHeight() const {
	return img_.Height();
}
unsigned VCamera::GetImageBytesPerPixel() const {
	return img_.Depth();
}
unsigned VCamera::GetBitDepth() const {
	return bitDepth_;
}

long VCamera::GetImageBufferSize() const
{
	return img_.Width() * img_.Height() * GetImageBytesPerPixel();
}

unsigned VCamera::GetImageBytesPerPixel() const
{
	return img_.Depth();
}

double VCamera::GetExposure() const
{
	char buf[MM::MaxStrLength];
	int ret = GetProperty(MM::g_Keyword_Exposure, buf);
	if (ret != DEVICE_OK)
		return 0.0;
	return atof(buf);
}

void VCamera::SetExposure(double exp)
{
	SetProperty(MM::g_Keyword_Exposure, CDeviceUtils::ConvertToString(exp));
	GetCoreCallback()->OnExposureChanged(this, exp);;
}

int VCamera::SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize)
{
	multiROIXs_.clear();
	multiROIYs_.clear();
	multiROIWidths_.clear();
	multiROIHeights_.clear();
	if (xSize == 0 && ySize == 0)
	{
		// effectively clear ROI
		roiX_ = 0;
		roiY_ = 0;
		img_.Resize(512,512,2);
	}
	else
	{
		// apply ROI
		img_.Resize(xSize, ySize);
		roiX_ = x;
		roiY_ = y;
	}
	return DEVICE_OK;
}
int VCamera::GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize)
{
	x = roiX_;
	y = roiY_;

	xSize = img_.Width();
	ySize = img_.Height();

	return DEVICE_OK;
}

int VCamera::ClearROI() {
	img_.Resize(512, 512, 2);
	roiX_ = 0;
	roiY_ = 0;
	multiROIXs_.clear();
	multiROIYs_.clear();
	multiROIWidths_.clear();
	multiROIHeights_.clear();
	return DEVICE_OK;
}

bool VCamera::SupportsMultiROI()
{
	return supportsMultiROI_;
}

bool VCamera::IsMultiROISet()
{
	return multiROIXs_.size() > 0;
}

int VCamera::GetMultiROICount(unsigned int& count)
{
	count = (unsigned int)multiROIXs_.size();
	return DEVICE_OK;
}

int VCamera::SetMultiROI(const unsigned int* xs, const unsigned int* ys,
	const unsigned* widths, const unsigned int* heights,
	unsigned numROIs)
{
	multiROIXs_.clear();
	multiROIYs_.clear();
	multiROIWidths_.clear();
	multiROIHeights_.clear();
	unsigned int minX = UINT_MAX;
	unsigned int minY = UINT_MAX;
	unsigned int maxX = 0;
	unsigned int maxY = 0;
	for (unsigned int i = 0; i < numROIs; ++i)
	{
		multiROIXs_.push_back(xs[i]);
		multiROIYs_.push_back(ys[i]);
		multiROIWidths_.push_back(widths[i]);
		multiROIHeights_.push_back(heights[i]);
		if (minX > xs[i])
		{
			minX = xs[i];
		}
		if (minY > ys[i])
		{
			minY = ys[i];
		}
		if (xs[i] + widths[i] > maxX)
		{
			maxX = xs[i] + widths[i];
		}
		if (ys[i] + heights[i] > maxY)
		{
			maxY = ys[i] + heights[i];
		}
	}
	img_.Resize(maxX - minX, maxY - minY);
	roiX_ = minX;
	roiY_ = minY;
	return DEVICE_OK;
}

int VCamera::GetMultiROI(unsigned* xs, unsigned* ys, unsigned* widths,
	unsigned* heights, unsigned* length)
{
	unsigned int roiCount = (unsigned int)multiROIXs_.size();
	if (roiCount > *length)
	{
		// This should never happen.
		return DEVICE_INTERNAL_INCONSISTENCY;
	}
	for (unsigned int i = 0; i < roiCount; ++i)
	{
		xs[i] = multiROIXs_[i];
		ys[i] = multiROIYs_[i];
		widths[i] = multiROIWidths_[i];
		heights[i] = multiROIHeights_[i];
	}
	*length = roiCount;
	return DEVICE_OK;
}

int VCamera::PrepareSequenceAcqusition() {
	int iRet = GetHub()->StopDOSequence();
	iRet = GetHub()->StopGalvo();
	iRet = GetHub()->DAQstop();
	iRet = GetHub()->VoltageConvert();
	iRet = GetHub()->GalvoAOSequence();
	iRet = GetHub()->DAQStart();
}

int VCamera::StartSequenceAcquisition(double interval) {
	if (IsCapturing())
		return DEVICE_CAMERA_BUSY_ACQUIRING;

	// this will open the shutter
	GetCoreCallback()->PrepareForAcq(this);

	thd_->SetNumImages(-1);

	startTime_ = GetCoreCallback()->GetCurrentMMTime();
	thd_->activate();

	return DEVICE_OK;
}

int VCamera::StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow) {
	if (IsCapturing())
		return DEVICE_CAMERA_BUSY_ACQUIRING;

	// this will open the shutter
	GetCoreCallback()->PrepareForAcq(this);

	startTime_ = GetCurrentMMTime();

	thd_->SetNumImages(numImages);
	startTime_ = GetCoreCallback()->GetCurrentMMTime();
	thd_->activate();

	return DEVICE_OK;
}

int VCamera::StopSequenceAcquisition() {
	Stop();
	thd_->Abort();
	GetCoreCallback()->AcqFinished(this, 0);
	return DEVICE_OK;
}

bool VCamera::IsCapturing()
{
	return thd_->IsRunning();
}

int VCamera::GetBinning() const
{
	char buf[MM::MaxStrLength];
	int ret = GetProperty(MM::g_Keyword_Binning, buf);
	if (ret != DEVICE_OK)
		return 1;
	return atoi(buf);
}

int VCamera::SetBinning(int binF)
{
	return SetProperty(MM::g_Keyword_Binning, CDeviceUtils::ConvertToString(binF));
}

int VCamera::Stop() {
	DeviceHub* hub_ = GetHub();
	int iRet = hub_->StopDOSequence();
	iRet = hub_->DAQstop();
	iRet = hub_->StopGalvo();
	return iRet;
}

int VCamera::IsExposureSequenceable(bool& isSequenceable) const
{
	isSequenceable = isSequenceable_;
	return DEVICE_OK;
}

int VCamera::GetExposureSequenceMaxLength(long& nrEvents) const
{
	if (!isSequenceable_) {
		return DEVICE_UNSUPPORTED_COMMAND;
	}

	nrEvents = sequenceMaxLength_;
	return DEVICE_OK;
}

int VCamera::StartExposureSequence()
{
	if (!isSequenceable_) {
		return DEVICE_UNSUPPORTED_COMMAND;
	}

	// may need thread lock
	sequenceRunning_ = true;
	return DEVICE_OK;
}

int VCamera::StopExposureSequence()
{
	if (!isSequenceable_) {
		return DEVICE_UNSUPPORTED_COMMAND;
	}

	// may need thread lock
	sequenceRunning_ = false;
	sequenceIndex_ = 0;
	return DEVICE_OK;
}

int VCamera::ClearExposureSequence()
{
	if (!isSequenceable_) {
		return DEVICE_UNSUPPORTED_COMMAND;
	}

	exposureSequence_.clear();
	return DEVICE_OK;
}

int VCamera::AddToExposureSequence(double exposureTime_ms)
{
	if (!isSequenceable_) {
		return DEVICE_UNSUPPORTED_COMMAND;
	}

	exposureSequence_.push_back(exposureTime_ms);
	return DEVICE_OK;
}

int VCamera::SendExposureSequence() const {
	if (!isSequenceable_) {
		return DEVICE_UNSUPPORTED_COMMAND;
	}

	return DEVICE_OK;
}

int LiveThread::svc()
{
	stopRunning_ = false;
	running_ = true;
	imageCounter_ = 0;

	// put the hardware into a continuous acqusition state
	while (true) {
		if (stopRunning_)
			break;

		int ret = cam_->SnapImage();

		if (ret != DEVICE_OK) {
			char txt[1000];
			sprintf(txt, "BitFlow live thread: ImageSnap() error %d", ret);
			cam_->GetCoreCallback()->LogMessage(cam_, txt, false);
			break;
		}

		char label[MM::MaxStrLength];

		cam_->GetLabel(label);

		MM::MMTime timestamp = cam_->GetCurrentMMTime();
		Metadata md;

		MetadataSingleTag mstElapsed(MM::g_Keyword_Elapsed_Time_ms, label, true);
		MM::MMTime elapsed = timestamp - cam_->startTime_;
		mstElapsed.SetValue(CDeviceUtils::ConvertToString(elapsed.getMsec()));
		md.SetTag(mstElapsed);

		MetadataSingleTag mstCount(MM::g_Keyword_Metadata_ImageNumber, label, true);
		mstCount.SetValue(CDeviceUtils::ConvertToString(imageCounter_));
		md.SetTag(mstCount);


		// insert all channels
		for (unsigned i = 0; i < cam_->GetNumberOfChannels(); i++)
		{
			char buf[MM::MaxStrLength];
			MetadataSingleTag mstChannel(MM::g_Keyword_CameraChannelIndex, label, true);
			snprintf(buf, MM::MaxStrLength, "%d", i);
			mstChannel.SetValue(buf);
			md.SetTag(mstChannel);

			MetadataSingleTag mstChannelName(MM::g_Keyword_CameraChannelName, label, true);
			cam_->GetChannelName(i, buf);
			mstChannelName.SetValue(buf);
			md.SetTag(mstChannelName);


			ret = cam_->GetCoreCallback()->InsertImage(cam_, cam_->GetImageBuffer(i),
				cam_->GetImageWidth(),
				cam_->GetImageHeight(),
				cam_->GetImageBytesPerPixel(),
				md.Serialize().c_str());
			if (ret == DEVICE_BUFFER_OVERFLOW) {
				cam_->GetCoreCallback()->ClearImageBuffer(cam_);
				cam_->GetCoreCallback()->InsertImage(cam_, cam_->GetImageBuffer(i),
					cam_->GetImageWidth(),
					cam_->GetImageHeight(),
					cam_->GetImageBytesPerPixel(),
					md.Serialize().c_str());
			}
			else if (ret != DEVICE_OK) {
				cam_->GetCoreCallback()->LogMessage(cam_, "BitFlow thread: error inserting image", false);
				break;
			}
		}


		imageCounter_++;
		if (numImages_ >= 0 && imageCounter_ >= numImages_) {
			cam_->Stop();
			break;
		}
	}
	running_ = false;
	return 0;
}

void LiveThread::Abort() {
	stopRunning_ = true;
	wait();
}

int VCamera::imageDataProcess()
{
	std::vector<std::vector<uint8_t>> data = GetHub()->getDAQ_DATA();

	size_t sampleCount = data[0].size() / 2; // 每两个字节代表一个14-bit数据
	size_t samplePerChannel = sampleCount / 4;
	size_t lineCount = data.size();
	for (size_t j = 0; j < lineCount; j++) {
		// 通道数为4，存储转换后的电压值
		double channel_sum[4] = { 0,0,0,0 };
		for (size_t i = 0; i < sampleCount; i++) {
			// 从 buffer 中读取 14-bit 数据
			int16_t adcValue = (data[j][2 * i + 1] << 8) | (data[j][2 * i] & 0x3F); // 取14位数据

			// 如果最高位是1，表示负数，进行符号扩展
			if (adcValue & 0x2000) {
				adcValue |= 0xC000; // 符号扩展
			}

			// 根据通道顺序保存数据
			int channel = i % 4; // 按通道顺序分配
			channel_sum[channel] += adcValue;
		}
		for (int channel = 0; channel < 4; channel++) {
			double average = channel_sum[channel] / samplePerChannel;
			channelData[channel].push_back(average);
		}
	}
	return DEVICE_OK;
}
unsigned char* VCamera::getChannelData(int channelNr)
{
	// 将 channelData 转换为 unsigned char 格式
	std::vector<unsigned char> resultData;
	for (double value : channelData[channelNr]) {
		// 将 double 值转换为 unsigned char
		uint16_t int_value = static_cast<uint16_t>(value); // 假设我们只需转换为16-bit整数
		resultData.push_back(static_cast<unsigned char>(int_value & 0xFF)); // 低8位
		resultData.push_back(static_cast<unsigned char>((int_value >> 8) & 0xFF)); // 高8位
	}

	// 返回结果
	return resultData.data();
}

