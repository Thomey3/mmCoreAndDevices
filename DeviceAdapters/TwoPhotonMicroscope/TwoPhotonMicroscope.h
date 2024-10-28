#pragma once
#include "QT12136DC.h"
#include "NIDAQ.h"
#include "optotune.h"
#include "ImgBuffer.h"

//#include <omp.h>  // 用来多核运算循环
#include <cmath>  // 计算反三角函数

#include "DeviceBase.h"

#include "mkl.h"
#include <math.h>
#include <algorithm>
//#include <complex.h>
#include <stdio.h>
#include <chrono>

#include <array>
#include <ppl.h> 
//#include <array>
//#include <oneapi/tbb.h>
//#include <atomic>

#define M_PI 3.14159265358979323846
#define pointNumber 262144

class VCamera;

class DeviceHub : public HubBase<DeviceHub>
{
public:
	DeviceHub() :
		initialized_(false),
		busy_(false),
		scanmode(0),
		kl(0),
		kv(0),
		bv(0),
		x_fullsize(525),
		y_fullsize(525),
		voltage_x(pointNumber),
		voltage_y(pointNumber),
		done(false),
		iterationNumber(0),
		size(5),
		linepoint(512),
		Fps(1.07),
		resolution_x(512),
		resolution_y(512),
		PixelDuration(0.00000356),
		resolution(pointNumber),
		SampleForPixel(1000)
	{}
	~DeviceHub() {}
	int Initialize();
	int Shutdown() { return DEVICE_OK; };
	void GetName(char* pName) const;
	bool Busy() { return busy_; };
	int DetectInstalledDevices();

public:
	bool initialized_;
	bool busy_;

	NIDAQHub* nidaqhub_;
	QT12136DC* kcdaq_;
	VCamera* VCamera_;

public:
	int DeviceSetting();
	int OnTestFunction(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnSampleForPixel(MM::PropertyBase* pProp, MM::ActionType eAct);

	int OnFps(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnResolution(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnPixelDuration(MM::PropertyBase* pProp, MM::ActionType eAct);

	int OnSize(MM::PropertyBase* pProp, MM::ActionType eAct);

	// galvo controll
	int VoltageConvert();
	int ILC(); // Iterative learning control
	std::vector<double> resizeVector(const std::vector<double>& data, int dataSize, int resolution);

	int Scan();
	int GalvoAOSequence();
	int StopGalvo();
	std::vector<double> generateTriangleWave(int size, double amplitude, int numWaves);
	int TriggerDOSequence();
	int SetLineclock(int state);
	int SetFrameclock(int state);
	int StopDOSequence();

	int DAQStart();
	bool waitDAQ();
	int DAQstop();
	std::vector<std::vector<std::vector<uint8_t>>> getDAQ_DATA();


public:
	std::string triggerPortName;
	std::string LineTriggerPort;
	std::string FrameTriggerPort;
	int linepoint;
	int resolution_x;
	int resolution_y;
	double resolution;
	int realpoint;
	double Fps;
	double PixelDuration;
	//galvo control
	std::string x_portName; // x-galvo portName
	std::string y_portName; // y-galvo portName
	std::string AItriggerport;
	std::string x_feedbackPort;
	std::string y_feedbackPort;
	int scanmode; // 0是单向扫， 1是双向扫
	double kl; // angle->length
	double kv; // angle->voltage
	double bv; // voltage bias
	unsigned x_fullsize, y_fullsize; // full FOV
	std::vector<double> xvoltage;
	std::vector<double> yvoltage;
	std::vector<double> voltage_x; // x-galvo sequence
	std::vector<double> voltage_y; // y-galvo sequence
	// Iterative learning control
	std::vector<double> feedbackWaveform;
	std::vector<double> desiredWaveform;
	std::vector<double> nextOutputWaveform;
	std::vector<double> outputWaveform;
	bool done;
	int	iterationNumber;


	double size;

	size_t SampleForPixel;



	class HubThread : public MMDeviceThreadBase {
	public:
		HubThread(DeviceHub* hub, NIDAQHub* daq) :hub_(hub), daq_(daq), stopRunning_(false), running_(false), dataCounter_(0),
			numDATA_(-1) {}
		~HubThread() {}

		bool IsRunning() { return running_; }
		void Abort();
		void SetNumDATA(long num) { numDATA_ = num; }


		// thread procedure
		int svc();

	private:
		DeviceHub* hub_;
		NIDAQHub* daq_;
		bool running_;
		bool stopRunning_;
		long dataCounter_;
		long numDATA_;
	};
	HubThread* hubThread_;
};

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

	unsigned  GetNumberOfComponents() const { return nComponents_; };
public:
	std::vector<std::vector<uint8_t>> channelData;
	size_t pixel_number;
	size_t sample_per_pixel;
	size_t channelnumber;
	int imageCounter_;
	int imageNumber_;

	double readoutUs_;
	bool stopOnOverflow_;
private:
	int OnBinning(MM::PropertyBase* pProp, MM::ActionType eAct);
public:
	bool initialized_;
	int Stop();
	int AcquireSnapshot();
	int RunSequenceOnThread();
	int InsertImage();
private:
	long binSize_;
	int SetAllowedBinning();
	void GenerateEmptyImage(ImgBuffer& img);
	unsigned char* getChannelData(int channelNr);
	double GetSequenceExposure();

	DeviceHub* GetHub() const
	{
		return static_cast<DeviceHub*>(GetParentHub());
	};
	DeviceHub* hub_;

	MMThreadLock imgPixelsLock_;
	std::vector<ImgBuffer> img_;
	std::vector<unsigned> multiROIXs_;
	std::vector<unsigned> multiROIYs_;
	std::vector<unsigned> multiROIWidths_;
	std::vector<unsigned> multiROIHeights_;
	unsigned roiX_;
	unsigned roiY_;
	bool ROIchange_;
	

	MM::MMTime startTime_;
	MM::MMTime readoutStartTime_;

	bool supportsMultiROI_;

	bool sequence;
	int bitDepth_;
	friend class LiveThread;
	LiveThread* thd_;
	std::string str;
	bool isSequenceable_;
	long sequenceMaxLength_;
	bool sequenceRunning_;
	bool fastImage_;

	size_t pixelnumber;
	size_t sampleCount; // 每两个字节代表一个14-bit数据
	size_t samplePerChannel;
	size_t sampleperPixel;
	size_t lineCount;
	double pixelsum[4] = { 0,0,0,0 };
	uint8_t pixelsum8bit[4];
	double maxRange; // 假设所有采样值都接近于最大值
	double minRange; // 假设所有采样值都接近于最小值

	unsigned long sequenceIndex_;
	std::vector<double> exposureSequence_;

	int nComponents_;
	int ch_;
	std::vector<std::vector<uint8_t>> data;
};

class LiveThread : public MMDeviceThreadBase {
public:
	LiveThread(VCamera* cam) : cam_(cam), stopRunning_(false), running_(false), imageCounter_(0),
		numImages_(-1) {}
	~LiveThread() {}

	void Start(long numImages, double intervalMs);
	bool IsRunning() { return running_; }
	void Abort();
	bool IsStopped();
	void Suspend();
	bool IsSuspended();
	void Resume();

	void SetNumImages(long num) { numImages_ = num; }
	double GetIntervalMs() { return intervalMs_; }
	void SetLength(long images) { numImages_ = images; }
	long GetLength() const { return numImages_; }
	long GetImageCounter() { return imageCounter_; }
	MM::MMTime GetStartTime() { return startTime_; }
	MM::MMTime GetActualDuration() { return actualDuration_; }

	// thread procedure
	int svc() noexcept;

private:
	double intervalMs_;

	VCamera* cam_;
	bool running_;
	bool stopRunning_;
	long imageCounter_;
	long numImages_;

	MM::MMTime startTime_;
	MM::MMTime actualDuration_;
	MM::MMTime lastFrameTime_;

	bool stop_;
	bool suspend_;

	MMThreadLock stopLock_;
	MMThreadLock suspendLock_;

};