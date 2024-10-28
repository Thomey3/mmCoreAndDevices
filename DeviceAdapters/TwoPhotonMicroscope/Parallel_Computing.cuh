#ifndef CUDA_COMMON_H
#define CUDA_COMMON_H

#include <vector>
#include <cstdint>
#include <cuda_runtime.h>
#include <iostream>
#include <chrono>

extern "C" 
{
	#define DLL_EXPORT_API_ __declspec(dllexport)

	DLL_EXPORT_API_ int Parallel_Computing(std::vector<uint16_t> data, int nSamplesPerPixel, int linesToProcess, std::vector<std::vector<uint8_t>>& out, int line);

}

#endif