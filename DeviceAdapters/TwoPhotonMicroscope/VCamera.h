#pragma once
#include "DeviceBase.h"
#include "ImgBuffer.h"


extern const char* g_DeviceNameVCamera;

class VCamera : public CCameraBase<VCamera>
{
public:
	VCamera();
	~VCamera();

	int Initialize();
	int Shutdown();

	void GetName(char* name) const;

	//MM Camera
	int SnapImage();
	const unsigned char* GetImageBuffer();
	const unsigned char* GetImageBuffer(unsigned channelNr);
	unsigned GetImageWidth() const;
	unsigned GetImageHeight() const;
	unsigned GetImageBytesPerPixel() const;
	unsigned GetBitDepth() const;
	long GetImageBufferSize() const;
	double GetExposure() const;
	void SetExposure(double exp);
	int SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize);
	int GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize);
	int ClearROI();
	bool SupportsMultiROI();
	bool IsMultiROISet();
	int GetMultiROICount(unsigned& count);
	int SetMultiROI(const unsigned* xs, const unsigned* ys,
		const unsigned* widths, const unsigned* heights,
		unsigned numROIs);
	int GetMultiROI(unsigned* xs, unsigned* ys, unsigned* widths,
		unsigned* heights, unsigned* length);
	int PrepareSequenceAcqusition();
	int StartSequenceAcquisition(double interval);
	int StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow);
	int StopSequenceAcquisition();
	bool IsCapturing();
	int GetBinning() const;
	int SetBinning(int bS);

	int IsExposureSequenceable(bool& isSequenceable) const;
	int GetExposureSequenceMaxLength(long& nrEvents) const;
	int StartExposureSequence();
	int StopExposureSequence();
	int ClearExposureSequence();
	int AddToExposureSequence(double exposureTime_ms);
	int SendExposureSequence() const;

public:
	std::vector<std::vector<double>> channelData;

public:
	bool initialized_;
private:
	void GenerateEmptyImage(ImgBuffer& img);
	int Stop();
	int imageDataProcess();
	unsigned char* getChannelData(int channelNr);

	DeviceHub* GetHub() const
	{
		return static_cast<DeviceHub*>(GetParentHub());
	};


	MMThreadLock imgPixelsLock_;
	ImgBuffer img_;
	std::vector<unsigned> multiROIXs_;
	std::vector<unsigned> multiROIYs_;
	std::vector<unsigned> multiROIWidths_;
	std::vector<unsigned> multiROIHeights_;
	unsigned roiX_;
	unsigned roiY_;

	MM::MMTime startTime_;

	bool supportsMultiROI_;

	bool sequence;
	int bitDepth_;
	friend class LiveThread;
	LiveThread* thd_;

	bool isSequenceable_;
	long sequenceMaxLength_;
	bool sequenceRunning_;

	unsigned long sequenceIndex_;
	std::vector<double> exposureSequence_;
};

class LiveThread : public MMDeviceThreadBase {
public:
	LiveThread(VCamera* cam) : cam_(cam), stopRunning_(false), running_(false), imageCounter_(0),
		numImages_(-1) {}
	~LiveThread() {}

	bool IsRunning() { return running_; }
	void Abort();
	void SetNumImages(long num) { numImages_ = num; }

	// thread procedure
	int svc();

private:
	VCamera* cam_;
	bool running_;
	bool stopRunning_;
	long imageCounter_;
	long numImages_;
};