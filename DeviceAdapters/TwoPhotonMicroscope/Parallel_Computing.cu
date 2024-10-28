#include "Parallel_Computing.cuh"



__global__ void computingimage(uint16_t* in, uint8_t* out, int nSamplesPerPixel, int lineToProcess) {
    uint64_t sum = 0;

    // 使用共享内存来提升内存访问速度
    extern __shared__ uint16_t shared_data[];

    // 将输入数据加载到共享内存中
    for (int i = threadIdx.x; i < nSamplesPerPixel * 4; i += blockDim.x) {
        shared_data[i] = in[blockIdx.x * nSamplesPerPixel * 4 + i];
    }
    __syncthreads();

    // 计算求和
    for (int i = 0; i < nSamplesPerPixel; i++) {
        sum += shared_data[i * 4 + threadIdx.x];
    }
    uint16_t average = static_cast<uint16_t>(sum / nSamplesPerPixel);

    // 缩放并限制值到 [0, 255]
    double scaled_value = (average + 8192.0) / 16383.0 * 255.0;
    if (scaled_value < 0.0) scaled_value = 0.0;
    if (scaled_value > 255.0) scaled_value = 255.0;

    out[threadIdx.x * lineToProcess * 512 + blockIdx.x] = static_cast<uint8_t>(scaled_value);
}

int Parallel_Computing(std::vector<uint16_t> data, int nSamplesPerPixel,int linesToProcess, std::vector<std::vector<uint8_t>>& out, int line) {
    // CUDA 相关变量
    uint8_t* d_data2D;
    size_t rows = 4;
    size_t cols = 512 * linesToProcess;
    size_t size2D = rows * cols * sizeof(uint8_t);

    // 在 GPU 上开辟二维 uint8_t 数组的空间
    cudaError_t err = cudaMalloc((void**)&d_data2D, size2D);
    if (err != cudaSuccess) {
        std::cerr << "CUDA malloc for data2D failed: " << cudaGetErrorString(err) << std::endl;
        return -1;
    }

    // CUDA 相关变量用于 data
    uint16_t* d_data;
    size_t sizeData = data.size() * sizeof(uint16_t);

    // 在 GPU 上开辟 uint16_t 数组的空间
    err = cudaMalloc((void**)&d_data, sizeData);
    if (err != cudaSuccess) {
        std::cerr << "CUDA malloc for data failed: " << cudaGetErrorString(err) << std::endl;
        return -1;
    }

    // 将数据从 host 复制到 device
    err = cudaMemcpy(d_data, data.data(), sizeData, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        std::cerr << "CUDA memcpy to d_data failed: " << cudaGetErrorString(err) << std::endl;
        return -1;
    }

    // 设置线程块和网格大小
    int blockSize = 4;
    int gridSize = 512 * linesToProcess;

    // 调用 CUDA 核函数，进行图像计算
    computingimage <<<gridSize, blockSize, nSamplesPerPixel * 4 * sizeof(uint16_t) >> > (d_data, d_data2D, nSamplesPerPixel,linesToProcess);

    // 等待 GPU 计算完成
    cudaDeviceSynchronize();

    // 定义一个用于存储结果的二维 vector
    std::vector<std::vector<uint8_t>> data2D(rows, std::vector<uint8_t>(cols));

    // 将结果从 device 复制到 host
    std::vector<uint8_t> flattened_data2D(rows * cols);
    err = cudaMemcpy(flattened_data2D.data(), d_data2D, size2D, cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        std::cerr << "CUDA memcpy from d_data2D failed: " << cudaGetErrorString(err) << std::endl;
        return -1;
    }

    // 将扁平化的数据转换为二维形式
    for (size_t i = 0; i < 2; ++i) {
        std::copy(flattened_data2D.begin() + i * cols, flattened_data2D.begin() + (i + 1) * cols, out[i].begin() + line * 512);
    }

    // 在完成后释放 GPU 空间
    err = cudaFree(d_data2D);
    if (err != cudaSuccess) {
        std::cerr << "CUDA free for data2D failed: " << cudaGetErrorString(err) << std::endl;
        return -1;
    }

    err = cudaFree(d_data);
    if (err != cudaSuccess) {
        std::cerr << "CUDA free for data failed: " << cudaGetErrorString(err) << std::endl;
        return -1;
    }

    return 0;
}