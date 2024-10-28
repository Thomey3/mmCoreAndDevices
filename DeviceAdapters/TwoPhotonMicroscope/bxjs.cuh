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
	// ʹ�� extern ����ȫ�ֱ���
	uint16_t* dataPtr;    // ����� GPU ��ԭʼ����
	uint8_t* deviceOutPtr; // GPU �������ͼ������
	uint8_t* hostOutPtr;   // GPU ����� CPU ��ͼ������
	std::vector<uint8_t> hostOutVector;  // ���ڹ������ݵ� std::vector

	size_t ImageSize;
};


#endif