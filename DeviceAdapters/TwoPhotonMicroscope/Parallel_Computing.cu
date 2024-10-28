#include "Parallel_Computing.cuh"



__global__ void computingimage(uint16_t* in, uint8_t* out, int nSamplesPerPixel, int lineToProcess) {
    uint64_t sum = 0;

    // ʹ�ù����ڴ��������ڴ�����ٶ�
    extern __shared__ uint16_t shared_data[];

    // ���������ݼ��ص������ڴ���
    for (int i = threadIdx.x; i < nSamplesPerPixel * 4; i += blockDim.x) {
        shared_data[i] = in[blockIdx.x * nSamplesPerPixel * 4 + i];
    }
    __syncthreads();

    // �������
    for (int i = 0; i < nSamplesPerPixel; i++) {
        sum += shared_data[i * 4 + threadIdx.x];
    }
    uint16_t average = static_cast<uint16_t>(sum / nSamplesPerPixel);

    // ���Ų�����ֵ�� [0, 255]
    double scaled_value = (average + 8192.0) / 16383.0 * 255.0;
    if (scaled_value < 0.0) scaled_value = 0.0;
    if (scaled_value > 255.0) scaled_value = 255.0;

    out[threadIdx.x * lineToProcess * 512 + blockIdx.x] = static_cast<uint8_t>(scaled_value);
}

int Parallel_Computing(std::vector<uint16_t> data, int nSamplesPerPixel,int linesToProcess, std::vector<std::vector<uint8_t>>& out, int line) {
    // CUDA ��ر���
    uint8_t* d_data2D;
    size_t rows = 4;
    size_t cols = 512 * linesToProcess;
    size_t size2D = rows * cols * sizeof(uint8_t);

    // �� GPU �Ͽ��ٶ�ά uint8_t ����Ŀռ�
    cudaError_t err = cudaMalloc((void**)&d_data2D, size2D);
    if (err != cudaSuccess) {
        std::cerr << "CUDA malloc for data2D failed: " << cudaGetErrorString(err) << std::endl;
        return -1;
    }

    // CUDA ��ر������� data
    uint16_t* d_data;
    size_t sizeData = data.size() * sizeof(uint16_t);

    // �� GPU �Ͽ��� uint16_t ����Ŀռ�
    err = cudaMalloc((void**)&d_data, sizeData);
    if (err != cudaSuccess) {
        std::cerr << "CUDA malloc for data failed: " << cudaGetErrorString(err) << std::endl;
        return -1;
    }

    // �����ݴ� host ���Ƶ� device
    err = cudaMemcpy(d_data, data.data(), sizeData, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        std::cerr << "CUDA memcpy to d_data failed: " << cudaGetErrorString(err) << std::endl;
        return -1;
    }

    // �����߳̿�������С
    int blockSize = 4;
    int gridSize = 512 * linesToProcess;

    // ���� CUDA �˺���������ͼ�����
    computingimage <<<gridSize, blockSize, nSamplesPerPixel * 4 * sizeof(uint16_t) >> > (d_data, d_data2D, nSamplesPerPixel,linesToProcess);

    // �ȴ� GPU �������
    cudaDeviceSynchronize();

    // ����һ�����ڴ洢����Ķ�ά vector
    std::vector<std::vector<uint8_t>> data2D(rows, std::vector<uint8_t>(cols));

    // ������� device ���Ƶ� host
    std::vector<uint8_t> flattened_data2D(rows * cols);
    err = cudaMemcpy(flattened_data2D.data(), d_data2D, size2D, cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        std::cerr << "CUDA memcpy from d_data2D failed: " << cudaGetErrorString(err) << std::endl;
        return -1;
    }

    // ����ƽ��������ת��Ϊ��ά��ʽ
    for (size_t i = 0; i < 2; ++i) {
        std::copy(flattened_data2D.begin() + i * cols, flattened_data2D.begin() + (i + 1) * cols, out[i].begin() + line * 512);
    }

    // ����ɺ��ͷ� GPU �ռ�
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