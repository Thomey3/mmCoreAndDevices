#ifndef CUDA_COMMON_H
#define CUDA_COMMON_H

#include "cuda_runtime.h"
#include <vector>
#include <chrono>


class bxjs {
public:
	bxjs(){}
	~bxjs(){}

	int compute(int width, int line, int processedChannels, int lineToProcess, int nSamplesPerPixel, int nChannels, std::vector<uint16_t> data, std::vector<std::vector<uint8_t>>&out);

	int CreatePageLockedMemory(size_t datasize, size_t imagesize);

	void FreePageLockedMemory();

private:
	// 使用 extern 声明全局变量
	uint16_t* dataPtr;    // 传输给 GPU 的原始数据
	uint8_t* deviceOutPtr; // GPU 计算完的图像数据
	uint8_t* hostOutPtr;   // GPU 传输给 CPU 的图像数据
	std::vector<uint8_t> hostOutVector;  // 用于管理数据的 std::vector

	size_t ImageSize;
};


#endif