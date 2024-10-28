#include "bxjs.cuh"

__global__ void computingimage(uint16_t* in, uint8_t* out, int nSamplesPerPixel, int lineToProcess) {
    uint64_t sum = 0;

    // 计算求和
    for (int i = 0; i < nSamplesPerPixel; i++) {
        sum += in[i * 4 + blockIdx.x * nSamplesPerPixel * 4 + threadIdx.x];
    }
    uint16_t average = static_cast<uint16_t>(sum / nSamplesPerPixel);

    // 缩放并限制值到 [0, 255]
    double scaled_value = (average + 8192.0) / 16383.0 * 255.0;
    if (scaled_value < 0.0) scaled_value = 0.0;
    if (scaled_value > 255.0) scaled_value = 255.0;

    out[threadIdx.x * lineToProcess * 512 + blockIdx.x] = static_cast<uint8_t>(scaled_value);
}

int bxjs::compute(int width, int line, int processedChannels, int lineToProcess, int nSamplesPerPixel, int nChannels, std::vector<uint16_t> data, std::vector<std::vector<uint8_t>>& out)
{
    if (dataPtr != nullptr) {
        std::memcpy(dataPtr, data.data(), data.size() * sizeof(uint16_t));
    }

    size_t rows = 2;
    size_t cols = 512 * lineToProcess;
    size_t size2D = rows * cols * sizeof(uint8_t);

    // 设置线程块和网格大小
    int blockSize = 4;
    int gridSize = 512 * lineToProcess;
    computingimage <<<gridSize, blockSize>>> (dataPtr, deviceOutPtr, nSamplesPerPixel,lineToProcess);
    cudaError_t err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        return -4;
    }


    std::vector<uint8_t> flattened_data2D(rows * cols);
    err = cudaMemcpy(flattened_data2D.data(), deviceOutPtr, size2D, cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        return -5;
    }


    out.resize(2);
    // 将扁平化的数据转换为二维形式
    for (size_t i = 0; i < 2; ++i) {
        out[i].resize(cols);
        std::copy(flattened_data2D.begin() + i * cols, flattened_data2D.begin() + (i + 1) * cols, out[i].begin());
    }

    return 0;
}

int bxjs::CreatePageLockedMemory(size_t datasize, size_t imagesize)
{
    // 分配锁页内存
    cudaError_t status = cudaMallocHost((void**)&dataPtr, datasize);
    if (status != cudaSuccess) {
        return -1;
    }

    // 分配设备内存
    status = cudaMalloc((void**)&deviceOutPtr, imagesize);
    if (status != cudaSuccess) {
        cudaFreeHost(dataPtr);  // 若分配失败则释放已分配的锁页内存
        return -1;
    }

    //// 分配设备内存
    //status = cudaMalloc((void**)&hostOutPtr, imagesize);
    //if (status != cudaSuccess) {
    //    cudaFreeHost(dataPtr);  // 若分配失败则释放已分配的锁页内存
    //    cudaFree(deviceOutPtr);
    //    return -1;
    //}
    ImageSize = imagesize;
    return 0;  // 返回 0 表示分配成功
}

void bxjs::FreePageLockedMemory()
{
    if (dataPtr) {
        cudaFreeHost(dataPtr);
        dataPtr = nullptr;
    }
    if (deviceOutPtr) {
        cudaFree(deviceOutPtr);
        deviceOutPtr = nullptr;
    }
    if (hostOutPtr) {
        cudaFreeHost(hostOutPtr);
        hostOutPtr = nullptr;
    }
}