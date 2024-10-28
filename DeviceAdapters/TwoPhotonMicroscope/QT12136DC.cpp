#include "QT12136DC.h"

const char* g_DeviceNameDAQ = "QT12136DC";
STXDMA_CARDINFO pstCardInfo;
Log_TraceLog g_Log(std::string("./logs/KunchiUpperMonitor.log"));
Log_TraceLog* pLog = &g_Log;

sem_t c2h_ping;		//ping信号量
sem_t c2h_pong;		//pong信号量
void printfLog(int nLevel, const char* fmt, ...)
{
    if (pLog == NULL)
        return;

    char buf[1024];
    va_list list;
    va_start(list, fmt);
    vsprintf(buf, fmt, list);
    va_end(list);

    pLog->Trace(nLevel, buf);
}

QT12136DC::QT12136DC() :
    initialized_(false),
    gateOpen_(true),
    gatedVoltage_(0.0),
    sequenceRunning_(false),
    maxSequenceLength_(1024),
    minVolts_(0.0),
    maxVolts_(0.2),
    clockmode(0),
    frameheader(0),
    once_trig_bytes(4096000),
    triggercount(0),
    triggermode(1),
    pretriglength(0),
    channelOn(1),
    DMATotolbytes(1000000),
    data(0),
    line(512),
    numPicture(1),
    numPicture_(1),
    SampleForPixel(1000),
    PixelNumber(262144),
    data_complete(false),
    scanmode(0),
    SegmentDuration(10),
    savepicture_(false)
{
    InitializeDefaultErrorMessages();
}
QT12136DC::~QT12136DC()
{
    Shutdown();
}
int QT12136DC::Initialize()
{
    data_collect_ = new data_collectThread(this);
    wait_intr_c2h_0_ = new wait_intr_c2h_0(this);
    DataProcessThread_ = new DataProcessThread(this);
    cudacompute = new bxjs();

    int iRet = QTXdmaOpenBoard(&pstCardInfo, 0); // 打开板卡
    QT_BoardGetCardInfo(); // 获得板卡信息
    InitialDevice();
    CPropertyAction* pAct = new CPropertyAction(this, &QT12136DC::OnChannels);
    iRet = CreateStringProperty("Channel", "1", false, pAct);  // 选取要调整的通道
    AddAllowedValue("Channel", "1");
    AddAllowedValue("Channel", "2");
    AddAllowedValue("Channel", "3");
    AddAllowedValue("Channel", "4");
    pAct = new CPropertyAction(this, &QT12136DC::OnOffset);
    iRet = CreateFloatProperty("offset", 0, false, pAct);  // 输入偏置值
    pAct = new CPropertyAction(this, &QT12136DC::OnSave);
    iRet = CreateStringProperty("Save Offset", "off", false, pAct);  // 选择是否保存
    AddAllowedValue("Save Offset", "on");
    AddAllowedValue("Save Offset", "off");
    return DEVICE_OK;
}
int QT12136DC::Shutdown()
{
    if (!initialized_)
        return DEVICE_OK;
    QT_BoardSetADCStop();
    QTXdmaCloseBoard(&pstCardInfo);
    initialized_ = false;

    return DEVICE_OK;
}
void QT12136DC::GetName(char* name) const
{
    CDeviceUtils::CopyLimitedString(name, g_DeviceNameDAQ);
}
int QT12136DC::SetGateOpen(bool open)
{
    gateOpen_ = open;
    return DEVICE_OK;
}
int QT12136DC::GetGateOpen(bool& open)
{
    open = gateOpen_;
    return DEVICE_OK;
}
int QT12136DC::GetSignal(double& volts)
{
    volts = gatedVoltage_;
    return DEVICE_OK;
}
int QT12136DC::GetLimits(double& minVolts, double& maxVolts)
{
    minVolts = minVolts_;
    maxVolts = maxVolts_;
    return DEVICE_OK;
}
int QT12136DC::IsDASequenceable(bool& isSequenceable) const
{
    isSequenceable = supportsTriggering_;
    return DEVICE_OK;
}
int QT12136DC::GetDASequenceMaxLength(long& maxLength) const
{
    maxLength = static_cast<long>(maxSequenceLength_);
    return DEVICE_OK;
}
int QT12136DC::StartDASequence()
{
    sem_init(&c2h_ping, 0, 0);
    sem_init(&c2h_pong, 0, 0);
    std::string str = "StartDASequence";
    LogMessage(str);
    //DAhd_->START();
    InitialDevice();
    data_collect_->START();
    DataProcessThread_->START();
    QT_BoardSetADCStart();
    wait_intr_c2h_0_->START();
    return DEVICE_OK;
}
int QT12136DC::StopDASequence()
{
    //DAhd_->Abort();
    data_collect_->Abort();
    DataProcessThread_->Abort();
    wait_intr_c2h_0_->Abort();
    QT_BoardSetADCStop();
    QT_BoardSetTransmitMode(0, 0);
    return DEVICE_OK;
}
int QT12136DC::ClearDASequence()
{
    QT_BoardSetADCStop();
    return DEVICE_OK;
}
int QT12136DC::AddToDASequence(double voltage)
{
    if (voltage < minVolts_ || voltage > maxVolts_)
        return DEVICE_ERR;
    return DEVICE_OK;
}
int QT12136DC::SendDASequence()
{
    if (sequenceRunning_)
        return DEVICE_ERR;
    // We don't actually "write" the sequence here, because writing
    // needs to take place once the correct task has been set up for
    // all of the AO channels.
    return DEVICE_OK;
}

int QT12136DC::OnChannels(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet)
    {
        std::string Name;
        pProp->Get(Name);
        if (Name == "1")
        {
            channelOn = 1;
        }
        else if (Name == "2")
        {
            channelOn = 2;
        }
        else if (Name == "3")
        {
            channelOn = 3;
        }
        else if (Name == "4")
        {
            channelOn = 4;
        }
    }
    return DEVICE_OK;
}
int QT12136DC::OnOffset(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet)
    {
        double OFFSET = 0;
        pProp->Get(OFFSET);
        double channel = channelOn - 1;
        offset[channel] = OFFSET;
        QT_BoardSetOffset12136DC_2(channel, offset[channel]);
    }
    return DEVICE_OK;
}
int QT12136DC::OnSave(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet)
    {
        std::string save;
        pProp->Get(save);
        if (save == "on")
        {
            std::ofstream outputFile(FileNamePath, std::ios::app);
            if (!outputFile) {
                std::cerr << "Unable to open file";
                return 1;
            }
            outputFile << std::endl;
            for (int num : offset) {
                outputFile << num << " ";
            }
            outputFile.close();
        }
    }
    return DEVICE_OK;
}

void QT12136DC::getData(std::vector<std::vector<std::vector<uint8_t>>>& DATA) {
    //MMThreadGuard g(rawDataLock_);
    DATA.push_back(data.back());
}

int QT12136DC::getPictureNumber() {
    MMThreadGuard g(pictureNumberLock_);
    return DataProcessThread_->pictureNumber;
}

int QT12136DC::LoadOffsetFile()
{
    std::ifstream inputFile(FileNamePath);
    if (!inputFile) {
        std::cerr << "Unable to open file";
        return DEVICE_ERR;
    }
    std::string lastLine;
    std::string line;
    while (std::getline(inputFile, line)) {
        lastLine = line;
    }
    inputFile.close();
    // Split the last line into four numbers
    std::istringstream iss(lastLine);
    std::vector<int> numbers;
    int number;
    while (iss >> number) {
        numbers.push_back(number);
    }
    // Check if we have exactly four numbers
    if (numbers.size() == 4) {
        offset = numbers;
        return DEVICE_OK;
    }
    else {
        std::cerr << "Error: The last line does not contain exactly four numbers." << std::endl;
        return DEVICE_ERR;
    }
}
int QT12136DC::InitialDevice()
{
    int iRet = LoadOffsetFile(); // 载入手动调节偏置到offset
    QT_BoardSetADCStop(); // 停止采集
    QT_BoardSetInterruptClear(); // 中断清除
    QT_BoardSetSoftReset(); // 软件复位
    for (size_t i = 0; i < 4; i++)
    {
        QT_BoardSetOffset12136DC_2(i, offset[i]); // 载入手动调节偏置
    }
    QT_BoardSetPerTrigger(pretriglength); // 设置预触发，默认为0
    QT_BoardSetFrameheader(frameheader); // 设置帧头，默认为无帧头
    //QT_BoardExternalTrigger(triggermode, triggercount); // 设置为外部上升沿触发
    SegmentDuration = 500000 / (512 * 512); // 微秒
    pulse_period = SegmentDuration * 1000;
    QT_BoardInternalPulseTrigger(triggercount, pulse_period, 10);
    p = QT_BoardSetOnceTrigBytes(SegmentDuration);
    once_trig_bytes = p[0];
    DMATotolbytes = p[0] * 512;
    QT_BoardSetStdMultiDMAParameter(once_trig_bytes, DMATotolbytes); // 设置DMA参数
    QT_BoardSetTransmitMode(1, 0); // 设置传输模式为无限次传输模式
    QT_BoardSetInterruptSwitch(); // 使能PCIE中断
    return DEVICE_OK;
}

void QT12136DC::data_collectThread::START() {
    str = "data_collectThread start ";
    daq_->LogMessage(str);
    ping_getdata = 0;
    pong_getdata = 0;
    once_trig_bytes = daq_->once_trig_bytes;
    if (scanmode == 0)
    {
        datacapacity = once_trig_bytes * 512 * 512;
    }
    else
    {
        datacapacity = once_trig_bytes * 512;
    }
    data1.reserve(datacapacity);
    data2.reserve(datacapacity);
    data1_ready = false;
    data2_ready = false;
    activate();
}
int QT12136DC::data_collectThread::svc() { 
    running_ = true;
    stopRunning_ = false;
    while (1) {
        if (stopRunning_) break;
        // ping数据搬移
        {
            sem_wait(&c2h_ping);
            int iBufferIndex = -1;
            int64_t remain_size = daq_->DMATotolbytes;
            uint64_t offsetaddr_ping = 0x0;
            std::vector<uint8_t> buffer(daq_->DMATotolbytes, 0);
            while (remain_size > 0)
            {
                int iLoopCount = 0;
                if (remain_size >= once_readbytes)
                {
                    QTXdmaGetDataBuffer(offsetaddr_ping, &pstCardInfo,
                        buffer.data()+ iLoopCount * once_readbytes, once_readbytes, 0);
                }
                else if (remain_size > 0)
                {
                    QTXdmaGetDataBuffer(offsetaddr_ping, &pstCardInfo,
                        buffer.data()+ iLoopCount * once_readbytes, remain_size, 0);
                }

                offsetaddr_ping += once_readbytes;
                remain_size -= once_readbytes;

                if (remain_size <= 0)
                {
                    printfLog(5, "break ");
                    break;
                }
                iLoopCount++;
            }
            data_offset += remain_size;
            // 选择缓冲区
            if (!data_select.load()) {
                std::unique_lock<std::mutex> lock(data1_mutex);
                data1.insert(data1.end(), buffer.begin(), buffer.end());
                //data1_ready = true;
                lock.unlock();
                //data1_cond.notify_one();
            }
            else {
                std::unique_lock<std::mutex> lock(data2_mutex);
                data2.insert(data2.end(), buffer.begin(), buffer.end());
                //data2_ready = true;
                lock.unlock();
                //data2_cond.notify_one();
            }

        }
        ping_getdata++;
        str = "ping: " + std::to_string(ping_getdata);
        daq_->LogMessage(str);
        {
            sem_wait(&c2h_pong);
            int iBufferIndex = -1;
            int64_t remain_size = daq_->DMATotolbytes;
            uint64_t offsetaddr_pong = 0x100000000;
            std::vector<uint8_t> buffer(daq_->DMATotolbytes, 0);
            while (remain_size > 0)
            {
                int iLoopCount = 0;
                if (remain_size >= once_readbytes)
                {
                    QTXdmaGetDataBuffer(offsetaddr_pong, &pstCardInfo,
                        buffer.data() + iLoopCount * once_readbytes, once_readbytes, 0);
                }
                else if (remain_size > 0)
                {
                    QTXdmaGetDataBuffer(offsetaddr_pong, &pstCardInfo,
                        buffer.data() + iLoopCount * once_readbytes, remain_size, 0);
                }

                offsetaddr_pong += once_readbytes;
                remain_size -= once_readbytes;

                if (remain_size <= 0)
                {
                    printfLog(5, "break ");
                    break;
                }
                iLoopCount++;
            }
            data_offset += remain_size;
            // 选择缓冲区
            if (!data_select.load()) {
                std::unique_lock<std::mutex> lock(data1_mutex);
                data1.insert(data1.end(), buffer.begin(), buffer.end());
                data1_ready = true;
                lock.unlock();
                data1_cond.notify_one();
            }
            else {
                std::unique_lock<std::mutex> lock(data2_mutex);
                data2.insert(data2.end(), buffer.begin(), buffer.end());
                data2_ready = true;
                lock.unlock();
                data2_cond.notify_one();
            }
        }
        pong_getdata++;
        str = "pong: " + std::to_string(pong_getdata);
        daq_->LogMessage(str);
    }
    running_ = false;
    return DEVICE_OK;
}
void QT12136DC::data_collectThread::Abort() {
    stopRunning_ = true;
    running_ = false;
    wait();
}

void QT12136DC::DataProcessThread::START() {
    str = "DataProcessThread start ";
    daq_->LogMessage(str);
    nChannels = 4;
    pictureNumber = 0;
    totalPixels = 512 * 512;
    nSamplesPerPixel = daq_->once_trig_bytes / 8;
    pictureSelect_ = false;
    processedChannels = 2;
    line = 0;
    width = 512;
    height = 512;
    target = daq_->numPicture_;
    save_ = daq_->savepicture_;
    input_line = 0;
    lineDataSize = width * nSamplesPerPixel * nChannels;
    dataProcessed = false;
    code2Complete_ = false;
    daq_->cudacompute->CreatePageLockedMemory(4ULL * 1024 * 1024 * 1024 / sizeof(uint16_t) ,width * height * sizeof(uint8_t));
    activate();
}
int QT12136DC::DataProcessThread::svc() {
    stopRunning_ = false;
    running_ = true;
    const bool infiniteMode = (target < 0);
    while (!stopRunning_ && (infiniteMode || pictureNumber < target))
    {
        if (stopRunning_) break;
        if (!infiniteMode && pictureNumber >= target) break;
        if (!getdata()) continue;
        daq_->LogMessage("get data success!");
        CalculatePixel();
        if (!infiniteMode && pictureNumber >= target) break;
    }
    running_ = false;
    daq_->cudacompute->FreePageLockedMemory();
    daq_->StopDASequence();
    return DEVICE_OK;
}
bool QT12136DC::DataProcessThread::getdata() {
    //if(DCth_->data_select.load()) input = &DCth_->data1;
    //else input = &DCth_->data2;
    //if (input->empty())
    //{
    //    DCth_->data_select.store(!DCth_->data_select.load());
    //    return false;
    //}
    //DCth_->data_select.store(!DCth_->data_select.load());
    //data.resize(input->size() / 2);
    //const uint16_t* inputPtr = reinterpret_cast<const uint16_t*>(input->data());
    //std::copy(inputPtr, inputPtr + data.size(), data.begin());
    //input->clear();
    //return true;
    if (!DCth_->data_select.load()) {
        // 读取 data1
        std::unique_lock<std::mutex> lock(DCth_->data1_mutex);
        // 等待数据准备好
        DCth_->data1_cond.wait(lock, [this] { return DCth_->data1_ready; });
        // 切换 data_select，仅由数据采集线程修改
        DCth_->data_select.store(!DCth_->data_select.load());
        // 处理数据
        input = &DCth_->data1;
        data.resize(input->size() / 2);
        const uint16_t* inputPtr = reinterpret_cast<const uint16_t*>(input->data());
        std::copy(inputPtr, inputPtr + data.size(), data.begin());
        input->clear();
        DCth_->data1_ready = false;
        lock.unlock();
    }
    else {
        // 读取 data2
        std::unique_lock<std::mutex> lock(DCth_->data2_mutex);
        // 等待数据准备好
        DCth_->data2_cond.wait(lock, [this] { return DCth_->data2_ready; }); 
        // 切换 data_select，仅由数据采集线程修改
        DCth_->data_select.store(!DCth_->data_select.load());
        // 处理数据
        input = &DCth_->data2;
        data.resize(input->size() / 2);
        const uint16_t* inputPtr = reinterpret_cast<const uint16_t*>(input->data());
        std::copy(inputPtr, inputPtr + data.size(), data.begin());
        input->clear();
        DCth_->data2_ready = false;
        lock.unlock();
    }
    return true;
}
void QT12136DC::DataProcessThread::CalculatePixel() {
    // 确保每个通道的means vector的大小足够
    if (means.size() != processedChannels) {
        means.resize(processedChannels);
        for (auto& channel : means) {
            channel.resize(totalPixels, 0); // 分配整个图像的数据空间
        }
    }
    // 确保每个通道的imgData1_ vector的大小足够
    if (imgData1_.size() != processedChannels) {
        imgData1_.resize(processedChannels);
        for (auto& channel : imgData1_) {
            channel.resize(totalPixels, 0); // 分配整个图像的数据空间
        }
    }
    // 确保每个通道的imgData1_ vector的大小足够
    if (imgData2_.size() != processedChannels) {
        imgData2_.resize(processedChannels);
        for (auto& channel : imgData2_) {
            channel.resize(totalPixels, 0); // 分配整个图像的数据空间
        }
    }
    input_line = data.size() / (daq_->DMATotolbytes/2);
    size_t dataOffset = 0;
    int linesToProcess = input_line;

    ProcessLines(linesToProcess);
}
void QT12136DC::DataProcessThread::ProcessLines(int linesToProcess) {
    int pixelsToProcess = linesToProcess * width;
    std::vector<std::vector<uint8_t>> means_temp;
    int errr = daq_->cudacompute->compute(width, line, processedChannels, linesToProcess, nSamplesPerPixel, nChannels, data, means_temp);
    if (errr != 0) {
        std::string er = "cuda error: " + std::to_string(errr);
        daq_->LogMessage(er);
        daq_->StopDASequence();
    }
    if (line + linesToProcess <= height) {
        for (int i = 0; i < 2; i++) {
            std::copy(means_temp[i].begin(), means_temp[i].end(), means[i].begin()+line * width);
        }
        // 行反转逻辑
        for (int row = line; row < line + linesToProcess; ++row) {
            if (row % 2 == 0) {
                for (size_t channel = 0; channel < processedChannels; ++channel) {
                    auto row_start = means[channel].begin() + row * width;
                    auto row_end = row_start + width;
                    std::reverse(row_start, row_end);
                }
            }
        }
        line += linesToProcess;
        linesToProcess = 0;
        str = "line : " + std::to_string(line);
        daq_->LogMessage(str);
        if (target != 1) {
            // 定期复制数据，例如每处理完64行
            if (line % 128 >= 0 && line % 128 <= 10 || line >= height) {
                CopyDataForReading();
            }
        }
        if (line == height) {
            CDeviceUtils::SleepMs(100);
            ResetForNextImage();
            pictureNumber++;
            str = "picture : " + std::to_string(pictureNumber);
            daq_->LogMessage(str);
        }
    }
    else
    {
        int remain = line + linesToProcess - height;
        std::vector<std::vector<uint8_t>> temp;
        temp.resize(2);
        for (int i = 0; i < 2; i++) {
            temp[i].resize(remain * width);
            std::copy(means_temp[i].end() - remain * width, means_temp[i].end(), temp[i].begin());
            std::copy(means_temp[i].begin(), means_temp[i].end() - remain * width, means[i].begin() + line * width);
        }
        
        // 行反转逻辑
        for (int row = line; row < height; ++row) {
            if (row % 2 == 0) {
                for (size_t channel = 0; channel < processedChannels; ++channel) {
                    auto row_start = means[channel].begin() + row * width;
                    auto row_end = row_start + width;
                    std::reverse(row_start, row_end);
                }
            }
        }
        CopyDataForReading();
        CDeviceUtils::SleepMs(10);
        ResetForNextImage();
        pictureNumber++;
        str = "picture : " + std::to_string(pictureNumber);
        daq_->LogMessage(str);
        means.resize(processedChannels);
        for (auto& channel : means) {
            channel.resize(totalPixels, 0); // 分配整个图像的数据空间
        }
        for (int i = 0; i < 2; i++) {
            std::copy(temp[i].begin(), temp[i].end(), means[i].begin());
        }
        line = remain;
        str = "line : " + std::to_string(line);
        daq_->LogMessage(str);
        linesToProcess = 0;
    }

}
void QT12136DC::DataProcessThread::SaveCurrentImage() {
    std::cout << "图像已经完全计算。" << std::endl;
    // 复制最终的图像数据，确保完整性
    SAVE();
    CopyDataForReading();
}
void QT12136DC::DataProcessThread::ResetForNextImage() {
    line = 0;

    means.clear();
    imgData1_.clear();
    imgData2_.clear();
}
void QT12136DC::DataProcessThread::SAVE() {
    daq_->data.push_back(means);
}
void QT12136DC::DataProcessThread::CopyDataForReading() {
    {
    std::lock_guard<std::mutex> lock(imgDataMutex_);
    // 确保每个通道的imgData1_ vector的大小足够
    if (imgData1_.size() != processedChannels) {
        imgData1_.resize(processedChannels);
        for (auto& channel : imgData1_) {
            channel.resize(totalPixels, 0); // 分配整个图像的数据空间
        }
    }
    // 确保每个通道的imgData1_ vector的大小足够
    if (imgData2_.size() != processedChannels) {
        imgData2_.resize(processedChannels);
        for (auto& channel : imgData2_) {
            channel.resize(totalPixels, 0); // 分配整个图像的数据空间
        }
    }
    if (pictureSelect_.load()) {
        for (int channel = 0; channel < processedChannels; ++channel) {
            imgData1_[channel] = means[channel];
        }
    }
    else {
        for (int channel = 0; channel < processedChannels; ++channel) {
            imgData2_[channel] = means[channel];
        }
    }
    str = "load means to imgData_";
    daq_->LogMessage(str);
    complete_ = true;
    }
    dataReadyCondition_.notify_one();
}
void QT12136DC::DataProcessThread::Abort() {
    stopRunning_ = true;
    //wait();
}

void QT12136DC::wait_intr_c2h_0::START() {
    str = "wait_intr_c2h_0 start ";
    daq_->LogMessage(str);
    intr_cnt = 1;
    activate();
}
int QT12136DC::wait_intr_c2h_0::svc() {
    stopRunning_ = false;
    running_ = true;
    while (1)
    {
        if (stopRunning_) break;
        QT_BoardInterruptGatherType();

        if (intr_cnt % 2 == 0)
        {
            sem_post(&c2h_pong);
            //printf("pong is %d free list size %d\n", intr_pong, ThreadFileToDisk::Ins().GetFreeSizePing());
        }
        else
        {
            sem_post(&c2h_ping);
            //printf("ping is %d free list size %d\n", intr_ping, ThreadFileToDisk::Ins().GetFreeSizePing());
        }
        intr_cnt++;
    }
    running_ = false;
    return DEVICE_OK;
}
void QT12136DC::wait_intr_c2h_0::Abort() {
    stopRunning_ = true;
    wait();
}