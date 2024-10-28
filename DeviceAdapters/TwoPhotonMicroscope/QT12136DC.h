#pragma once

#include "DeviceBase.h"

#include "QTXdmaApi.h"
#include "TraceLog.h"

#include "./QT12136DC/include/pingpong_example.h"
#include "pthread.h"
#include "semaphore.h"
#include "./QT12136DC/include/ThreadFileToDisk.h"
#include "./QT12136DC/include/3rd/TraceLog.h"
#include <windows.h>
#include <conio.h> 
#include <stdlib.h>
#include <math.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include <boost/lockfree/spsc_queue.hpp>

#include <chrono>
#include <ppl.h>
#include <mutex>
#include <atomic>
#include "bxjs.cuh"

#define MB *1024*1024

extern const char* g_DeviceNameDAQ;


class QT12136DC : public CSignalIOBase<QT12136DC>
{
public:
	QT12136DC();
	~QT12136DC();

	virtual int Initialize();
	virtual int Shutdown();
	virtual void GetName(char* name) const;
	virtual bool Busy() { return false; }
	virtual int SetGateOpen(bool open);
	virtual int GetGateOpen(bool& open);
	virtual int SetSignal(double /*volts*/) { return DEVICE_UNSUPPORTED_COMMAND; }
	virtual int GetSignal(double& volts);
	virtual int GetLimits(double& minVolts, double& maxVolts);
	virtual int IsDASequenceable(bool& isSequenceable) const;
	virtual int GetDASequenceMaxLength(long& maxLength) const;
	virtual int StartDASequence();
	virtual int StopDASequence();
	virtual int ClearDASequence();
	virtual int AddToDASequence(double voltage);
	virtual int SendDASequence();

	int LoadOffsetFile();
	int InitialDevice();

public:
	int OnChannels(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnOffset(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnSave(MM::PropertyBase* pProp, MM::ActionType eAct);

	bool initialized_;
	bool gateOpen_;
	double gatedVoltage_;
	bool sequenceRunning_;
	bool supportsTriggering_ = true;
	size_t maxSequenceLength_;
	double minVolts_;
	double maxVolts_;
	double channelOn;
	std::vector<int> offset;

	double pretriglength; // 预触发长度
	int frameheader; // 帧头使能
	int clockmode; // 时钟模式
	uint64_t once_trig_bytes; // 单次触发数据量（单位：字节）
	uint32_t triggercount; // 触发次数
	uint64_t DMATotolbytes;
	int triggermode;
	std::vector<std::vector<std::vector<uint8_t>>> data;
	int current_size;
	int line;
	int numPicture;
	int numPicture_;
	bool data_complete;
	int PixelNumber;

	int SampleForPixel;
	int pulse_period;
	int scanmode;
	double SegmentDuration;
	uint64_t* p;
	bool savepicture_;

	const char* FileNamePath = "./QT12136DC_offset.txt";

	MMThreadLock rawDataLock_;
	MMThreadLock pictureNumberLock_;
	//void sendData(std::vector<uint8_t> DATA);
	void getData(std::vector<std::vector<std::vector<uint8_t>>>& DATA);
	int getPictureNumber();
	void sendPictureNumber(int num);

	bxjs* cudacompute;

	friend class data_collectThread;
	class data_collectThread : public MMDeviceThreadBase {
	public:
		data_collectThread(QT12136DC* daq) : daq_(daq), stopRunning_(false), running_(false),
		scanmode(0),data_offset(0), data_select(false){}
		~data_collectThread() {}

		bool IsRunning() { return running_; }
		void START();
		void Abort();

		// thread procedure
		int svc();

		std::vector<uint8_t> data1;
		std::vector<uint8_t> data2;
		std::atomic<bool> data_select;
		std::mutex data1_mutex, data2_mutex;
		std::condition_variable data1_cond, data2_cond;
		bool data1_ready;
		bool data2_ready;
	private:
		QT12136DC* daq_;
		bool running_;
		bool stopRunning_;
		std::string str;
		int64_t data_offset;
		int data_;
		uint32_t once_trig_bytes;
		uint32_t datacapacity;
		int scanmode;  // 0代表galvo scan, 1代表resonance scan

		long ping_getdata = 0;
		long pong_getdata = 0;
		long once_readbytes = 8 MB;
	};
	data_collectThread* data_collect_;

	friend class wait_intr_c2h_0;
	class wait_intr_c2h_0 : public MMDeviceThreadBase {
	public:
		wait_intr_c2h_0(QT12136DC* daq) : daq_(daq), stopRunning_(false), running_(false),
			intr_cnt(1)
		{
		}
		~wait_intr_c2h_0() {}

		bool IsRunning() { return running_; }
		void START();
		void Abort();

		// thread procedure
		int svc();
	private:
		QT12136DC* daq_;
		bool running_;
		bool stopRunning_;
		std::string str;
		int intr_cnt;
	};
	wait_intr_c2h_0* wait_intr_c2h_0_;

	friend class DataProcessThread;
	class DataProcessThread : public MMDeviceThreadBase {
	public:
		DataProcessThread(QT12136DC* daq) : daq_(daq), stopRunning_(false), running_(false),
			input(0), nChannels(4), nPixels(0), nSamplesPerPixel(0),
			processedChannels(2), totalPixels(512 * 512), line(0), width(512), pictureNumber(0),
			target(1), pictureSelect_(false),complete_(false),save_(false)
		{
			DCth_ = daq_->data_collect_;
		}
		~DataProcessThread() {}

		bool IsRunning() { return running_; }
		void START();
		void Abort();
		bool getdata();
		void CalculatePixel();
		void ProcessLines(int linesToProcess);
		void SaveCurrentImage();
		void ResetForNextImage();
		void CopyDataForReading();
		void SAVE();
		

		// thread procedure
		int svc();


		int pictureNumber;
		int target;
		std::atomic<bool> pictureSelect_;
		std::vector<std::vector<uint8_t>> imgData1_;
		std::vector<std::vector<uint8_t>> imgData2_;

		std::mutex imgDataMutex_;
		std::condition_variable dataReadyCondition_;
		bool dataProcessed = false;

		std::mutex syncMutex_;
		std::condition_variable code2CompleteCondition_;
		std::atomic<bool> code2Complete_;
		
		std::atomic<bool> complete_;
		bool save_;
	private:
		QT12136DC* daq_;
		data_collectThread* DCth_;
		bool running_;
		bool stopRunning_;
		std::string str;
		std::vector<uint8_t>* input;
		std::vector<uint16_t> data;
		std::vector<std::vector<uint8_t>> means;
		int nSamplesPerPixel;
		int totalPixels;
		int nChannels;
		int processedChannels;
		int nPixels;
		int line;
		int width;
		int height;
		int input_line;

		size_t lineDataSize;
	};
	DataProcessThread* DataProcessThread_;
};




