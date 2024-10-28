#include "TwoPhotonMicroscope.h"

const char* g_DeviceHubName = "DeviceHub";
const char* g_DeviceNameVCamera = "Camera";


const char* g_PixelType_16bit = "16bit";

MODULE_API void InitializeModuleData()
{
    RegisterDevice(g_DeviceNameNIDAQHub, MM::HubDevice, "NIDAQHub");
    RegisterDevice(g_DeviceNameDAQ, MM::SignalIODevice, "QT12136DC");
    RegisterDevice(g_DeviceNameoptotune, MM::GenericDevice, "optotune");
    RegisterDevice(g_DeviceNameVCamera, MM::CameraDevice, "Camera");
    RegisterDevice(g_DeviceHubName, MM::HubDevice, "DeviceHub");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
    if (deviceName == 0)
        return 0;

    // decide which device class to create based on the deviceName parameter
    if (strcmp(deviceName, g_DeviceNameNIDAQHub) == 0)
    {
        // create NIDAQ
        return new NIDAQHub();
    }
    else if (std::string(deviceName).
        substr(0, strlen(g_DeviceNameNIDAQAOPortPrefix)) ==
        g_DeviceNameNIDAQAOPortPrefix)
    {
        return new NIAnalogOutputPort(std::string(deviceName).
            substr(strlen(g_DeviceNameNIDAQAOPortPrefix)));
    }
    else if (std::string(deviceName).substr(0, strlen(g_DeviceNameNIDAQDOPortPrefix)) ==
        g_DeviceNameNIDAQDOPortPrefix)
    {
        return new DigitalOutputPort(std::string(deviceName).
            substr(strlen(g_DeviceNameNIDAQDOPortPrefix)));
    }
    else if (strcmp(deviceName, g_DeviceHubName) == 0)
    {
        return new DeviceHub();
    }
    else if (strcmp(deviceName, g_DeviceNameDAQ) == 0)
    {
        return new QT12136DC();
    }
    else if (strcmp(deviceName, g_DeviceNameoptotune) == 0)
    {
        return new optotune();
    }
    else if (strcmp(deviceName, g_DeviceNameVCamera) == 0)
    {
        return new VCamera();
    }

    // ...supplied name not recognized
    return 0;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
    delete pDevice;
}

int DeviceHub::Initialize(){
    nidaqhub_ = static_cast<NIDAQHub*>(GetDevice(g_DeviceNameNIDAQHub));
    kcdaq_ = static_cast<QT12136DC*>(GetDevice(g_DeviceNameDAQ));
    VCamera_ = static_cast<VCamera*>(GetDevice(g_DeviceNameVCamera));

    DeviceSetting();

    hubThread_ = new HubThread(this, nidaqhub_);
    // test botton
    CPropertyAction* pAct = new CPropertyAction(this, &DeviceHub::OnTestFunction);
    CreateProperty("TestFunction", "off", MM::String, false, pAct);
    AddAllowedValue("TestFunction", "off");
    AddAllowedValue("TestFunction", "on");

    // Fps botton
    pAct = new CPropertyAction(this, &DeviceHub::OnSampleForPixel);
    CreateFloatProperty("Sample(pixel)", 1000, false, pAct);

    // resolution botton
    pAct = new CPropertyAction(this, &DeviceHub::OnResolution);
    CreateProperty("resolution", "512*512", MM::String, false, pAct);
    
    // Fps botton
    pAct = new CPropertyAction(this, &DeviceHub::OnFps);
    CreateFloatProperty("Fps", 1.07,  false, pAct);

    // Pixelrate botton
    pAct = new CPropertyAction(this, &DeviceHub::OnPixelDuration);
    CreateFloatProperty("Pixel Duration", 1.07, false, pAct);

    // Pixelrate botton
    pAct = new CPropertyAction(this, &DeviceHub::OnSize);
    CreateFloatProperty("Size", 10000, false, pAct);


    return DEVICE_OK;
}
void DeviceHub::GetName(char* pName) const
{
    CDeviceUtils::CopyLimitedString(pName, g_DeviceHubName);
}
int DeviceHub::DetectInstalledDevices()
{
    ClearInstalledDevices();

    // make sure this method is called before we look for available devices
    InitializeModuleData();

    char hubName[MM::MaxStrLength];
    GetName(hubName); // this device name
    for (unsigned i = 0; i < GetNumberOfDevices(); i++)
    {
        char deviceName[MM::MaxStrLength];
        bool success = GetDeviceName(i, deviceName, MM::MaxStrLength);
        if (success && (strcmp(hubName, deviceName) != 0))
        {
            MM::Device* pDev = CreateDevice(deviceName);
            AddInstalledDevice(pDev);
        }
    }
    return DEVICE_OK;
}

int DeviceHub::DeviceSetting() {
    x_portName = "Dev1/ao0";
    y_portName = "Dev1/ao1";
    x_feedbackPort = "Dev1/ai0";
    y_feedbackPort = "Dev1/ai1";
    triggerPortName = "Dev1/port0/line0";
    AItriggerport = "/Dev1/PFI0";
    LineTriggerPort = "Dev1/port0/line1";
    return DEVICE_OK;
}

int DeviceHub::OnTestFunction(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet)
    {
        std::string setset;
        pProp->Get(setset);
        if (setset == "off")
        {
            //StopDOSequence();
            //StopGalvo();
            //hubThread_->Abort();
            DAQstop();
            //SetLineclock(0);
        }
        else if (setset == "on")
        {
            //voltage_x = generateTriangleWave(pointNumber, size,512);
            //voltage_y = generateTriangleWave(pointNumber, size,512);

            //voltage_x = generateTriangleWave(pointNumber, size, 512);
            //voltage_y = generateTriangleWave(pointNumber, size, 512);


            //SetLineclock(1);
            //ILC();
            DAQStart();

            //Sleep(10);
            //GalvoAOSequence();
            //Sleep(10);
            //TriggerDOSequence();
             
             
            //Scan();
        }
    }
    return DEVICE_OK;
}
int DeviceHub::OnFps(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(Fps);
    }
    if (eAct == MM::AfterSet)
    {
        double rateHz;
        pProp->Get(rateHz);
        Fps = rateHz;
        PixelDuration = 1000000 / (Fps * resolution);
        kcdaq_->SegmentDuration = PixelDuration;
    }
    return DEVICE_OK;
}
int DeviceHub::OnSampleForPixel(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        double p = SampleForPixel;
        pProp->Set(p);
    }
    if (eAct == MM::AfterSet)
    {
        double p;
        pProp->Get(p);
        SampleForPixel = p;
        kcdaq_->SampleForPixel = p;
        Fps = 1000000000 / (SampleForPixel * 512 * 512);
        PixelDuration = 1 / (Fps * resolution);
    }
    return DEVICE_OK;
}
int DeviceHub::OnResolution(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet)
    {
        std::string RESOLUTION;
        pProp->Get(RESOLUTION);
        std::size_t pos = RESOLUTION.find('*');

        if (pos != std::string::npos) {
            // 提取乘号两边的子字符串
            std::string firstPart = RESOLUTION.substr(0, pos);
            std::string secondPart = RESOLUTION.substr(pos + 1);

            // 使用 stringstream 将字符串转换为 int
            std::stringstream ss1(firstPart);
            std::stringstream ss2(secondPart);

            ss1 >> resolution_x;
            ss2 >> resolution_y;
            resolution = resolution_x * resolution_y;
        }
        else {
            return DEVICE_ERR;
        }

    }
    return DEVICE_OK;
}
int DeviceHub::OnPixelDuration(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(PixelDuration);
    }
    if (eAct == MM::AfterSet)
    {
        double rateHz;
        pProp->Get(rateHz);
        PixelDuration = rateHz;
        Fps = 1 / (PixelDuration * resolution);
    }
    return DEVICE_OK;
}

int DeviceHub::OnSize(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(size);
    }
    if (eAct == MM::AfterSet)
    {
        double SIZE;
        pProp->Get(SIZE);
        size = SIZE;
    }
    return DEVICE_OK;
}

int DeviceHub::VoltageConvert()
{
    unsigned x, y, xsize, ysize;
    //这里要获得ROI
    double x_angle1, x_angle2;
    double y_angle1, y_angle2;

    x_angle1 = (std::atan((x - x_fullsize / 2) / kl)) * 180.0 / M_PI;
    x_angle2 = (std::atan((x + xsize - x_fullsize / 2) / kl)) * 180.0 / M_PI;
    y_angle1 = (std::atan((y - y_fullsize / 2) / kl)) * 180.0 / M_PI;
    y_angle2 = (std::atan((y + ysize - y_fullsize / 2) / kl)) * 180.0 / M_PI;

    double x_voltage_line[512], y_voltage_line[512], x_voltage_line_m[512];
    double step1 = (x_angle2 - x_angle1) / (511);
    double step2 = (y_angle2 - y_angle1) / (511);
#pragma omp parallel for
    for (size_t i = 0; i < 512; ++i) {
        x_voltage_line[i] = (x_angle1 + i * step1) * kv + bv;
        y_voltage_line[i] = (y_angle1 + i * step2) * kv + bv;
        x_voltage_line_m[511 - i] = x_voltage_line[i];
    }

    if (scanmode == 0)
    {
#pragma omp parallel for
        for (size_t i = 0; i < 512; ++i)
        {
            for (size_t j = 0; j < 512; ++j)
            {
                xvoltage[i + 512 * j] = x_voltage_line[i];
                yvoltage[i * 512 + j] = y_voltage_line[i];
            }
        }
    }
    else if (scanmode == 1)
    {
#pragma omp parallel for
        for (size_t i = 0; i < 512; ++i)
        {
            for (size_t j = 0; j < 512; ++j)
            {
                if (j % 2 == 0)
                {
                    xvoltage[i + 512 * j] = x_voltage_line[i];
                }
                else
                {
                    xvoltage[i + 512 * j] = x_voltage_line_m[i];
                }
                yvoltage[i * 512 + j] = y_voltage_line[i];
            }
        }
    }

    return DEVICE_OK;
}
int DeviceHub::ILC()
{
    // 在函数中分配空间
    feedbackWaveform.resize(pointNumber);
    desiredWaveform.resize(pointNumber);
    nextOutputWaveform.resize(pointNumber);
    outputWaveform.resize(pointNumber);
    xvoltage.resize(pointNumber);
    yvoltage.resize(pointNumber);

    MKL_LONG waveformLength = pointNumber;
    MKL_LONG status;
    DFTI_DESCRIPTOR_HANDLE my_desc_handle = NULL;
    iterationNumber = 0;
    std::string aoportName;
    std::string aiportName;
    std::vector<double> nextwf(pointNumber);
    std::vector<double> err(waveformLength);

    if (nidaqhub_)
    {
        for (int i = 0; i < 2; i++)
        {
            if (i == 0)
            {
                aoportName = x_portName;
                desiredWaveform = xvoltage;
                nextOutputWaveform = xvoltage;
                outputWaveform = xvoltage;
                aiportName = x_feedbackPort;
                done = false;
                iterationNumber = 0;
            }
            else if (i == 1)
            {
                desiredWaveform = yvoltage;
                nextOutputWaveform = yvoltage;
                outputWaveform = yvoltage;
                aoportName = y_portName;
                aiportName = y_feedbackPort;
                done = false;
                iterationNumber = 0;
            }
            double AISampleRate = 1 / Fps;
            double DOSampleRate = Fps * resolution;
            while (!done)
            {
                nextwf = nextOutputWaveform;
                int result = nidaqhub_->StartAOSequenceForPort(aoportName, nextwf);
                if (result != DEVICE_OK) {
                    // 处理错误
                    return result;
                };
                result = nidaqhub_->StartAISequenceForPort(aiportName, AItriggerport, AISampleRate,resolution);
                if (result != DEVICE_OK) {
                    // 处理错误
                    return result;
                };
                int state = DAQmx_Val_FiniteSamps;
                result = nidaqhub_->StartDOBlankingAndOrSequenceWithoutTrigger(triggerPortName, true, false,
                    1, false, pointNumber, 10000,state);
                
                while (1)
                {
                    if (nidaqhub_->complete) break;
                }

                std::vector<float64> data = nidaqhub_->data;
                int32 dataSize = nidaqhub_->dataSize;

                feedbackWaveform = resizeVector(data, dataSize, resolution);

                result = nidaqhub_->StopAOSequenceForPort(aoportName);

                outputWaveform = nextOutputWaveform;

                iterationNumber = iterationNumber + 1;
                // Calculate the error
                
                for (MKL_LONG i = 0; i < waveformLength; i++) {
                    err[i] = feedbackWaveform[i] - desiredWaveform[i];
                }

                // Check if desiredWaveform is constant
                int is_constant = 1;
                for (MKL_LONG i = 1; i < waveformLength; i++) {
                    if (desiredWaveform[i] != desiredWaveform[0]) {
                        is_constant = 0;
                        break;
                    }
                }
                if (is_constant) {
                    float mean_err = 0.0;
                    for (MKL_LONG i = 0; i < waveformLength; i++) {
                        mean_err += err[i];
                    }
                    mean_err /= waveformLength;

                    for (MKL_LONG i = 0; i < waveformLength; i++) {
                        nextOutputWaveform[i] = desiredWaveform[i] - mean_err;
                    }

                    done = true;
                    return DEVICE_OK;
                }

                // FFT of feedbackWaveform and outputWaveform
                status = DftiCreateDescriptor(&my_desc_handle, DFTI_DOUBLE, DFTI_REAL, 1, waveformLength);
                status = DftiCommitDescriptor(my_desc_handle);

                std::vector<MKL_Complex8> fft_feedback(pointNumber);
                std::vector<MKL_Complex8> fft_output(pointNumber);
                std::vector<MKL_Complex8> fft_err(pointNumber);
                status = DftiComputeForward(my_desc_handle, feedbackWaveform.data(), fft_feedback.data());
                status = DftiComputeForward(my_desc_handle, outputWaveform.data(), fft_output.data());
                status = DftiComputeForward(my_desc_handle, err.data(), fft_err.data());
                // Calculate H

                std::vector<MKL_Complex8> H(pointNumber);
                for (MKL_LONG i = 0; i < waveformLength; i++) {
                    double a = fft_feedback[i].real;
                    double b = fft_feedback[i].imag;
                    double c = fft_output[i].real;
                    double d = fft_output[i].imag;

                    double mo = c * c + d * d;

                    float magnitude = std::sqrt(mo);
                    if (magnitude < 0.05) {
                        H[i].imag = std::numeric_limits<float>::infinity();
                        H[i].real = std::numeric_limits<float>::infinity();
                    }
                    else {
                        // 使用 MKL 函数进行复数除法
                        H[i].real = (a * c + b * d) / mo;
                        H[i].imag = (b * c - a * d) / mo;
                    }
                }

                // FFT of err



                // Calculate err_i in frequency domain
                std::vector<MKL_Complex8> err_i(pointNumber);
                for (MKL_LONG i = 0; i < waveformLength; i++) {
                    double a = fft_err[i].real;
                    double b = fft_err[i].imag;
                    double c = H[i].real;
                    double d = H[i].imag;

                    double mo = c * c + d * d;
                    float magnitude = std::sqrt(mo);
                    if (magnitude == INFINITY) {
                        err_i[i].real = 0;
                        err_i[i].imag = 0;
                    }
                    else {
                        err_i[i].real = (a * c + b * d) / mo;
                        err_i[i].imag = (b * c - a * d) / mo;
                    }
                }

                // Inverse FFT of err_i
                status = DftiComputeBackward(my_desc_handle, err_i.data());
                // Calculate mean of err_i
                float mean_err_i = 0.0;
                for (MKL_LONG i = 0; i < waveformLength; i++) {
                    mean_err_i += err_i[i].real;
                }
                mean_err_i /= waveformLength;

                // Calculate mean of err
                float mean_err = 0.0;
                for (MKL_LONG i = 0; i < waveformLength; i++) {
                    mean_err += err[i];
                }
                mean_err /= waveformLength;

                // Calculate nextOutputWaveform
                for (MKL_LONG i = 0; i < waveformLength; i++) {
                    nextOutputWaveform[i] = outputWaveform[i] - (err_i[i].real - mean_err_i) - mean_err;
                }

                // Free descriptor
                status = DftiFreeDescriptor(&my_desc_handle);

                // Check if optimization is done
                done = (iterationNumber >= 6);
            }
            if (i == 0)
            {
                voltage_x = nextOutputWaveform;
            }
            else if (i == 1)
            {
                voltage_y = nextOutputWaveform;
            }
        }
    }
    return DEVICE_OK;
}
std::vector<double> DeviceHub::resizeVector(const std::vector<double>& data, int dataSize, int resolution) {
    std::vector<double> resizedData(resolution, 0.0);

    // Calculate the size of each segment
    double segmentSize = static_cast<double>(dataSize) / resolution;

    for (int i = 0; i < resolution; ++i) {
        // Determine the range of the current segment
        int startIdx = static_cast<int>(i * segmentSize);
        int endIdx = static_cast<int>((i + 1) * segmentSize);

        // Adjust the end index to ensure it does not go out of bounds
        endIdx = (std::min)(endIdx, dataSize);

        // Calculate the sum of the segment
        double segmentSum = 0.0;
        for (int j = startIdx; j < endIdx; ++j) {
            segmentSum += data[j];
        }

        // Calculate the average and assign it to the resized vector
        int segmentLength = endIdx - startIdx;
        if (segmentLength > 0) {
            resizedData[i] = segmentSum / segmentLength;
        }
    }

    return resizedData;
}
int DeviceHub::GalvoAOSequence() 
{
    if (nidaqhub_)
    {
        int result = nidaqhub_->StartAOSequenceForPort(x_portName, voltage_x);
        if (result != DEVICE_OK) {
            // 处理错误
            std::cerr << "Error starting AO sequence on port " << x_portName << std::endl;
            return result;
        };
        result = nidaqhub_->StartAOSequenceForPort(y_portName, voltage_y);
        if (result != DEVICE_OK) {
            // 处理错误
            std::cerr << "Error starting AO sequence on port " << y_portName << std::endl;
            return result;
        };
        return DEVICE_OK;
    }
}
int DeviceHub::StopGalvo()
{
    if (nidaqhub_)
    {
        int result = nidaqhub_->StopAOSequenceForPort(x_portName);
        if (result != DEVICE_OK) {
            // 处理错误
            return result;
        };
        result = nidaqhub_->StopAOSequenceForPort(y_portName);
        if (result != DEVICE_OK) {
            // 处理错误
            return result;
        };
        return DEVICE_OK;
    }
}
std::vector<double> DeviceHub::generateTriangleWave(int size, double amplitude, int numWaves) {
    //锯齿波原型
    //std::vector<double> wave(size);
    //int period = size / numWaves;
    //for (int i = 0; i < size; ++i) {
    //    double value = (i % period < period / 2)
    //        ? (2.0 * amplitude * (i % period) / (period / 2)) - amplitude
    //        : (2.0 * amplitude * ((i % period) - (period / 2)) / (period / 2)) - amplitude;
    //    wave[i] = value;
    //}
    //return wave;

    //三角波原型
    //std::vector<double> wave(size);
    //int period = size / numWaves;
    //for (int i = 0; i < size; ++i) {
    //    int positionInPeriod = i % period;
    //    double value;
    //    if (positionInPeriod < period / 2) {
    //        value = (2.0 * amplitude * positionInPeriod / (period / 2)) - amplitude;
    //    }
    //    else {
    //        value = amplitude - (2.0 * amplitude * (positionInPeriod - period / 2) / (period / 2));
    //    }
    //    wave[i] = value;
    //}
    //return wave;

    // 改良三角波
    int effectperiod = size / numWaves;
    double effectamplitude = amplitude;
    
    int extrapoint = effectamplitude * 150 * Fps;
    double extraAm = 0.5;
    int realperiod = effectperiod + extrapoint;
    linepoint = realperiod;

    double realamplitude = effectamplitude + extraAm;
    int totalpoint = realperiod * numWaves;
    realpoint = totalpoint;
    std::vector<double> wave(totalpoint);
    for (int i = 0; i < totalpoint; ++i) {
        int positionInPeriod = i % realperiod;
        double value;
        if (positionInPeriod < realperiod / 2) {
            value = (2.0 * realamplitude * positionInPeriod / (realperiod / 2)) - realamplitude;
        }
        else {
            value = realamplitude - (2.0 * realamplitude * (positionInPeriod - realperiod / 2) / (realperiod / 2));
        }
        wave[i] = value;
    }
    return wave;

    // 改良锯齿波
    //double effectperiod = size / numWaves;
    //double effectamplitude = amplitude;
    //double effectperiod_step = 2 * effectamplitude / effectperiod;

    //int beforeEffectperiod = 300;
    //double beforeEffectperiod_amplitude = 1;
    //double beforeEffectperiod_step = effectperiod_step;

    //int realperiod = effectperiod + beforeEffectperiod;

    //int totalpoint = realperiod * numWaves;
    //double value;
    //std::vector<double> wave(totalpoint);
    //for (int i = 0; i < totalpoint; ++i) {
    //    int positionInPeriod = i % realperiod;
    //    if (positionInPeriod <= beforeEffectperiod) {
    //        value =  - effectamplitude;
    //    }
    //    else {
    //        value = effectperiod_step * (positionInPeriod - beforeEffectperiod) - effectamplitude;
    //    }
    //    wave[i] = value;
    //}
    //return wave;

    //增加采样点数
    //std::vector<double> wave(size);
    //int period = size / numWaves;
    //for (int i = 0; i < size; ++i) {
    //    double value = (i % period < period / 2)
    //        ? (2.0 * amplitude * (i % period) / (period / 2)) - amplitude
    //        : amplitude - (2.0 * amplitude * ((i % period) - (period / 2)) / (period / 2));
    //    wave[i] = value;
    //}
    //return wave;

    //插入过渡段
    //std::vector<double> wave(size);
    //int period = size / numWaves;
    //int transition = period / 10;  // 过渡段的长度，可以根据需要调整
    //for (int i = 0; i < size; ++i) {
    //    int positionInPeriod = i % period;
    //    double value;
    //    if (positionInPeriod < period / 2) {
    //        value = (2.0 * amplitude * positionInPeriod / (period / 2)) - amplitude;
    //        // 过渡段处理
    //        if (positionInPeriod < transition) {
    //            value *= positionInPeriod / static_cast<double>(transition);
    //        }
    //    }
    //    else {
    //        value = amplitude - (2.0 * amplitude * (positionInPeriod - period / 2) / (period / 2));
    //        // 过渡段处理
    //        if (positionInPeriod > period - transition) {
    //            value *= (period - positionInPeriod) / static_cast<double>(transition);
    //        }
    //    }
    //    wave[i] = value;
    //}
    //return wave;


    //int extraPoints = 10;
    //std::vector<double> wave(size);
    //int period = size / numWaves;
    //int totalPeriod = period + 2 * extraPoints; // 总周期包含前后的缓和段
    //double overshoot = 1.1 * amplitude; // 轻微的超调
    //double undershoot = -0.1 * amplitude; // 轻微的欠调

    //for (int i = 0; i < size; ++i) {
    //    int localIndex = i % totalPeriod;

    //    if (localIndex < extraPoints) {
    //        // 初始欠调段
    //        double t = double(localIndex) / extraPoints;
    //        wave[i] = undershoot + (amplitude - undershoot) * t; // 从欠调逐渐增加到有效区域起点
    //    }
    //    else if (localIndex < extraPoints + period / 2) {
    //        wave[i] = (2.0 * amplitude * (localIndex - extraPoints) / (period / 2)) - amplitude;
    //    }
    //    else if (localIndex < extraPoints + period) {
    //        wave[i] = (2.0 * amplitude * ((localIndex - extraPoints) - (period / 2)) / (period / 2)) - amplitude;
    //    }
    //    else {
    //        // 结束超调段
    //        double t = double(localIndex - (extraPoints + period)) / extraPoints;
    //        wave[i] = amplitude + (overshoot - amplitude) * t; // 从有效区域结束逐渐增加到超调值
    //    }
    //}
    //return wave;
    

}
int DeviceHub::TriggerDOSequence() {
    if (nidaqhub_) {
        int state = DAQmx_Val_FiniteSamps;
         //DAQmx_Val_ContSamps  DAQmx_Val_FiniteSamps
        double sampleRate = Fps * realpoint;
        //int result = nidaqhub_->AddDOPortToSequencing(LineTriggerPort, linetrigger);
        //result = nidaqhub_->StartDOBlankingAndOrSequence(LineTriggerPort, 32, true, true,
        //    1, false, AItriggerport);
        int result = nidaqhub_->StartDOBlankingAndOrSequenceWithoutTrigger(triggerPortName, true, true,
            1, false, linepoint, sampleRate, state);
        if (result != DEVICE_OK) {
            // 处理错误
            std::cerr << "Error starting AO sequence on port " << triggerPortName << std::endl;
            return result;
        };
        return DEVICE_OK;
    }

}
int DeviceHub::SetLineclock(int state) {
    if (nidaqhub_) {
        int result = nidaqhub_->SetLineClockPortState(state);
        if (result != DEVICE_OK) {
            // 处理错误
            std::cerr << "Error starting AO sequence on port " << triggerPortName << std::endl;
            return result;
        };
        return DEVICE_OK;
    }
}
int DeviceHub::SetFrameclock(int state) {
    if (nidaqhub_) {
        int result = nidaqhub_->SetLineClockPortState(state);
        if (result != DEVICE_OK) {
            // 处理错误
            std::cerr << "Error starting AO sequence on port " << triggerPortName << std::endl;
            return result;
        };
        return DEVICE_OK;
    }
}
int DeviceHub::Scan()
{
    hubThread_->activate();
    return DEVICE_OK;
}
int DeviceHub::HubThread::svc()
{
    Sleep(10);
    hub_->GalvoAOSequence();
    Sleep(10);
    hub_->DAQStart();
    Sleep(10);
    hub_->TriggerDOSequence();
    Sleep(10);

    return DEVICE_OK;
}
void DeviceHub::HubThread::Abort()
{
    stopRunning_ = true;
    hub_->StopDOSequence();
    hub_->StopGalvo();
    wait();
}
int DeviceHub::StopDOSequence() {
    if (nidaqhub_) {
        int result = nidaqhub_->StopDOBlankingAndSequence(32);
        if (result != DEVICE_OK) {
            std::cerr << "Error starting AO sequence on port " << triggerPortName << std::endl;
            return result;
        }
        return DEVICE_OK;
    }
    return DEVICE_ERR;
}
int DeviceHub::DAQStart() {
    int err = kcdaq_->StartDASequence();
    return err;
}
bool DeviceHub::waitDAQ() {
    return kcdaq_->data_complete;
}
int DeviceHub::DAQstop() {
    int err = kcdaq_->StopDASequence();
    return err;
}


VCamera::VCamera() :
    channelData(4),
    initialized_(false),
    bitDepth_(8),
    roiX_(0),
    roiY_(0),
    isSequenceable_(false),
    sequenceMaxLength_(100),
    sequenceRunning_(false),
    sequenceIndex_(0),
    binSize_(1),
    nComponents_(1),
    fastImage_(false),
    ROIchange_(true)
{
    thd_ = new LiveThread(this);
    CreateHubIDProperty();
    img_.resize(4);
    channelData.resize(4);
}
VCamera::~VCamera()
{
    Stop();
    delete thd_;
}
int VCamera::Initialize() {
    initialized_ = true;
    hub_ = GetHub();
    int nRet = CreateStringProperty(MM::g_Keyword_Name, g_DeviceNameVCamera, true);
    if (DEVICE_OK != nRet)
        return nRet;

    // Description
    nRet = CreateStringProperty(MM::g_Keyword_Description, "VCamera Device Adapter", true);
    if (DEVICE_OK != nRet)
        return nRet;

    // CameraName
    nRet = CreateStringProperty(MM::g_Keyword_CameraName, "VCamera", true);
    assert(nRet == DEVICE_OK);

    // CameraID
    nRet = CreateStringProperty(MM::g_Keyword_CameraID, "V1.0", true);
    assert(nRet == DEVICE_OK);

    // binning
    CPropertyAction* pAct = new CPropertyAction(this, &VCamera::OnBinning);
    nRet = CreateIntegerProperty(MM::g_Keyword_Binning, 1, false, pAct);
    assert(nRet == DEVICE_OK);

    nRet = SetAllowedBinning();
    if (nRet != DEVICE_OK)
        return nRet;

    nRet = SetProperty(MM::g_Keyword_PixelType, g_PixelType_16bit);

    // exposure
    nRet = CreateFloatProperty(MM::g_Keyword_Exposure, 10.0, false);
    if (nRet != DEVICE_OK)
        return nRet;
    SetPropertyLimits(MM::g_Keyword_Exposure, 0.0, 100);

    // camera gain
    nRet = CreateIntegerProperty(MM::g_Keyword_Gain, 0, false);
    if (nRet != DEVICE_OK)
        return nRet;
    SetPropertyLimits(MM::g_Keyword_Gain, -5, 8);

    // camera offset
    nRet = CreateIntegerProperty(MM::g_Keyword_Offset, 0, false);
    if (nRet != DEVICE_OK)
        return nRet;

    for (int i = 0; i < 4; i++) {
        img_[i].Resize(512, 512, 1);
        img_[i].ResetPixels();
    }

    return DEVICE_OK;
}
int VCamera::Shutdown() {
    Stop();
    return DEVICE_OK;
}
void VCamera::GenerateEmptyImage(ImgBuffer& img)
{
    MMThreadGuard g(imgPixelsLock_);
    if (img.Height() == 0 || img.Width() == 0 || img.Depth() == 0)
        return;
    unsigned char* pBuf = const_cast<unsigned char*>(img.GetPixels());
    memset(pBuf, 0, img.Height() * img.Width() * img.Depth());
}
void VCamera::GetName(char* name) const
{
    CDeviceUtils::CopyLimitedString(name, g_DeviceNameVCamera);
}
int VCamera::SnapImage()
{
    static int callCounter = 0;
    ++callCounter;
    auto start = std::chrono::high_resolution_clock::now();
    MM::MMTime startTime = GetCurrentMMTime();
    double exp = GetExposure();
    hub_->kcdaq_->numPicture_ = 1;
    hub_->kcdaq_->savepicture_ = true;
    if (sequenceRunning_ && IsCapturing())
    {
        exp = GetSequenceExposure();
    }
    if (!fastImage_)
    {
        AcquireSnapshot();
        // 输出触发
        hub_->TriggerDOSequence();
    }
    std::unique_lock<std::mutex> lock(hub_->kcdaq_->DataProcessThread_->imgDataMutex_);
    hub_->kcdaq_->DataProcessThread_->dataReadyCondition_.wait(lock, [this] { return hub_->kcdaq_->DataProcessThread_->complete_.load(); });
    std::vector<std::vector<std::vector<uint8_t>>>  vec;
    hub_->kcdaq_->getData(vec);
    for (size_t ch = 0; ch < 2; ++ch) {
        img_[ch].SetPixels(vec[0][ch].data());
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration1 = end - start;
    readoutStartTime_ = GetCurrentMMTime();
    return DEVICE_OK;
}
int VCamera::AcquireSnapshot() {
    std::string str = "AcquireSnapshot start";
    LogMessage(str);
    //if (ROIchange_) {
    //    hub_->voltage_x = hub_->generateTriangleWave(pointNumber, 1, 512);
    //    hub_->voltage_y = hub_->generateTriangleWave(pointNumber, 1, 512);
    //}
    //int err = hub_->StopGalvo();
    //// 准备scanner的电压
    //err = hub_->GalvoAOSequence();
    //if (err)
    //{
    //    return err;
    //}
    //hub_->StopDOSequence();
    hub_->DAQstop();
    sequence = false;
    Sleep(10);
    // daq准备采集，等待触发
    hub_->DAQStart();
    Sleep(10);
    return DEVICE_OK;
}
const unsigned char* VCamera::GetImageBuffer()
{
    MMThreadGuard g(imgPixelsLock_);
    MM::MMTime readoutTime(readoutUs_);
    return img_[0].GetPixels();
}
const unsigned char* VCamera::GetImageBuffer(unsigned channelNr)
{
    return img_[channelNr].GetPixels();
}
unsigned VCamera::GetImageWidth() const {
    return img_[0].Width();
}
unsigned VCamera::GetImageHeight() const {
    return img_[0].Height();
}
unsigned VCamera::GetImageBytesPerPixel() const {
    return img_[0].Depth();
}
unsigned VCamera::GetBitDepth() const {
    return bitDepth_;
}
long VCamera::GetImageBufferSize() const
{
    return img_[0].Width() * img_[0].Height() * img_[0].Depth();
}
double VCamera::GetExposure() const
{
    char buf[MM::MaxStrLength];
    int ret = GetProperty(MM::g_Keyword_Exposure, buf);
    if (ret != DEVICE_OK)
        return 0.0;
    return atof(buf);
}
double VCamera::GetSequenceExposure()
{
    if (exposureSequence_.size() == 0)
        return this->GetExposure();

    double exposure = exposureSequence_[sequenceIndex_];

    sequenceIndex_++;
    if (sequenceIndex_ >= exposureSequence_.size())
        sequenceIndex_ = 0;

    return exposure;
}
void VCamera::SetExposure(double exp)
{
    SetProperty(MM::g_Keyword_Exposure, CDeviceUtils::ConvertToString(exp));
    GetCoreCallback()->OnExposureChanged(this, exp);;
}
int VCamera::SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize)
{
    multiROIXs_.clear();
    multiROIYs_.clear();
    multiROIWidths_.clear();
    multiROIHeights_.clear();
    if (xSize == 0 && ySize == 0)
    {
        // effectively clear ROI
        roiX_ = 0;
        roiY_ = 0;
        for (vector<ImgBuffer>::iterator i = img_.begin(); i != img_.end(); i++)
            i->Resize(512, 512,1);
    }
    else
    {
        // apply ROI
        for (vector<ImgBuffer>::iterator i = img_.begin(); i != img_.end(); i++)
            i->Resize(xSize, ySize);
        roiX_ = x;
        roiY_ = y;
    }
    return DEVICE_OK;
}
int VCamera::GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize)
{
    x = roiX_;
    y = roiY_;

    xSize = img_[0].Width();
    ySize = img_[0].Height();

    return DEVICE_OK;
}
int VCamera::ClearROI() {
    for (vector<ImgBuffer>::iterator i = img_.begin(); i != img_.end(); i++)
        i->Resize(512, 512, 1);
    roiX_ = 0;
    roiY_ = 0;
    multiROIXs_.clear();
    multiROIYs_.clear();
    multiROIWidths_.clear();
    multiROIHeights_.clear();
    return DEVICE_OK;
}
bool VCamera::SupportsMultiROI()
{
    return supportsMultiROI_;
}
bool VCamera::IsMultiROISet()
{
    return multiROIXs_.size() > 0;
}
int VCamera::GetMultiROICount(unsigned int& count)
{
    count = (unsigned int)multiROIXs_.size();
    return DEVICE_OK;
}
int VCamera::SetMultiROI(const unsigned int* xs, const unsigned int* ys,
    const unsigned* widths, const unsigned int* heights,
    unsigned numROIs)
{
    multiROIXs_.clear();
    multiROIYs_.clear();
    multiROIWidths_.clear();
    multiROIHeights_.clear();
    unsigned int minX = UINT_MAX;
    unsigned int minY = UINT_MAX;
    unsigned int maxX = 0;
    unsigned int maxY = 0;
    for (unsigned int i = 0; i < numROIs; ++i)
    {
        multiROIXs_.push_back(xs[i]);
        multiROIYs_.push_back(ys[i]);
        multiROIWidths_.push_back(widths[i]);
        multiROIHeights_.push_back(heights[i]);
        if (minX > xs[i])
        {
            minX = xs[i];
        }
        if (minY > ys[i])
        {
            minY = ys[i];
        }
        if (xs[i] + widths[i] > maxX)
        {
            maxX = xs[i] + widths[i];
        }
        if (ys[i] + heights[i] > maxY)
        {
            maxY = ys[i] + heights[i];
        }
    }
    img_[0].Resize(maxX - minX, maxY - minY);
    roiX_ = minX;
    roiY_ = minY;
    return DEVICE_OK;
}
int VCamera::GetMultiROI(unsigned* xs, unsigned* ys, unsigned* widths,
    unsigned* heights, unsigned* length)
{
    unsigned int roiCount = (unsigned int)multiROIXs_.size();
    if (roiCount > *length)
    {
        // This should never happen.
        return DEVICE_INTERNAL_INCONSISTENCY;
    }
    for (unsigned int i = 0; i < roiCount; ++i)
    {
        xs[i] = multiROIXs_[i];
        ys[i] = multiROIYs_[i];
        widths[i] = multiROIWidths_[i];
        heights[i] = multiROIHeights_[i];
    }
    *length = roiCount;
    return DEVICE_OK;
}
int VCamera::PrepareSequenceAcqusition() {
    int iRet = hub_->StopDOSequence();
    iRet = hub_->StopGalvo();
    iRet = hub_->DAQstop();
    iRet = hub_->VoltageConvert();
    iRet = hub_->GalvoAOSequence();
    iRet = hub_->DAQStart();
    return iRet;
}
int VCamera::StartSequenceAcquisition(double interval) {
    std::string str = "StartSequenceAcquisition only interval";
    LogMessage(str);
    return StartSequenceAcquisition(-1, interval, false);
}
int VCamera::StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow) {
    if (IsCapturing())
        return DEVICE_CAMERA_BUSY_ACQUIRING;

    std::string str = "StartSequenceAcquisition all";
    LogMessage(str);

    // this will open the shutter
    int ret = GetCoreCallback()->PrepareForAcq(this);
    if (ret != DEVICE_OK)
        return ret;
    startTime_ = GetCurrentMMTime();
    imageCounter_ = 0;
    hub_->kcdaq_->numPicture_ = numImages;
    hub_->kcdaq_->savepicture_ = false;
    thd_->Start(numImages, interval_ms);
    stopOnOverflow_ = stopOnOverflow;
    return DEVICE_OK;
}
int VCamera::StopSequenceAcquisition() {
    thd_->Abort();
    Stop();
    GetCoreCallback()->AcqFinished(this, 0);
    return DEVICE_OK;
}
bool VCamera::IsCapturing()
{
    return !thd_->IsStopped();
}
int VCamera::GetBinning() const
{
    char buf[MM::MaxStrLength];
    int ret = GetProperty(MM::g_Keyword_Binning, buf);
    if (ret != DEVICE_OK)
        return 1;
    return atoi(buf);
}
int VCamera::SetBinning(int binF)
{
    return SetProperty(MM::g_Keyword_Binning, CDeviceUtils::ConvertToString(binF));
}
int VCamera::Stop() {
    int iRet = hub_->StopDOSequence();
    iRet = hub_->DAQstop();
    iRet = hub_->StopGalvo();
    return DEVICE_OK;
}
int VCamera::IsExposureSequenceable(bool& isSequenceable) const
{
    isSequenceable = isSequenceable_;
    return DEVICE_OK;
}
int VCamera::GetExposureSequenceMaxLength(long& nrEvents) const
{
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    nrEvents = sequenceMaxLength_;
    return DEVICE_OK;
}
int VCamera::StartExposureSequence()
{
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    // may need thread lock
    sequenceRunning_ = true;
    return DEVICE_OK;
}
int VCamera::StopExposureSequence()
{
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    // may need thread lock
    sequenceRunning_ = false;
    sequenceIndex_ = 0;
    return DEVICE_OK;
}
int VCamera::ClearExposureSequence()
{
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    exposureSequence_.clear();
    return DEVICE_OK;
}
int VCamera::AddToExposureSequence(double exposureTime_ms)
{
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    exposureSequence_.push_back(exposureTime_ms);
    return DEVICE_OK;
}
int VCamera::SendExposureSequence() const {
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    return DEVICE_OK;
}
int VCamera::SetAllowedBinning()
{
    std::vector<std::string> binValues;
    binValues.push_back("1");
    binValues.push_back("2");

    LogMessage("Setting Allowed Binning settings", true);
    return SetAllowedValues(MM::g_Keyword_Binning, binValues);
}
int VCamera::OnBinning(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int ret = DEVICE_ERR;
    switch (eAct)
    {
    case MM::AfterSet:
    {
        if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

        // the user just set the new value for the property, so we have to
        // apply this value to the 'hardware'.
        long binFactor;
        pProp->Get(binFactor);
        if (binFactor > 0 && binFactor < 10)
        {
            // calculate ROI using the previous bin settings
            double factor = (double)binFactor / (double)binSize_;
            roiX_ = (unsigned int)(roiX_ / factor);
            roiY_ = (unsigned int)(roiY_ / factor);
            for (unsigned int i = 0; i < multiROIXs_.size(); ++i)
            {
                multiROIXs_[i] = (unsigned int)(multiROIXs_[i] / factor);
                multiROIYs_[i] = (unsigned int)(multiROIYs_[i] / factor);
                multiROIWidths_[i] = (unsigned int)(multiROIWidths_[i] / factor);
                multiROIHeights_[i] = (unsigned int)(multiROIHeights_[i] / factor);
            }
            for (vector<ImgBuffer>::iterator i = img_.begin(); i != img_.end(); i++)
                i->Resize((unsigned int)(i->Width() / factor),
                    (unsigned int)(i->Height() / factor));

            binSize_ = binFactor;
            std::ostringstream os;
            os << binSize_;
            OnPropertyChanged("Binning", os.str().c_str());
            ret = DEVICE_OK;
        }
    }break;
    case MM::BeforeGet:
    {
        ret = DEVICE_OK;
        pProp->Set(binSize_);
    }break;
    default:
        break;
    }
    return ret;
}

int VCamera::InsertImage()
{
    MM::MMTime timeStamp = this->GetCurrentMMTime();
    char label[MM::MaxStrLength];
    this->GetLabel(label);

    // Important:  metadata about the image are generated here:
    Metadata md;
    md.put("Camera", label);
    md.put(MM::g_Keyword_Elapsed_Time_ms, CDeviceUtils::ConvertToString((timeStamp - startTime_).getMsec()));
    md.put(MM::g_Keyword_Metadata_ROI_X, CDeviceUtils::ConvertToString((long)roiX_));
    md.put(MM::g_Keyword_Metadata_ROI_Y, CDeviceUtils::ConvertToString((long)roiY_));

    char buf[MM::MaxStrLength];
    GetProperty(MM::g_Keyword_Binning, buf);
    md.put(MM::g_Keyword_Binning, buf);

    MMThreadGuard g(imgPixelsLock_);

    const unsigned char* pI;
    pI = GetImageBuffer();

    unsigned int w = GetImageWidth();
    unsigned int h = GetImageHeight();
    unsigned int b = GetImageBytesPerPixel();

    int ret = GetCoreCallback()->InsertImage(this, pI, w, h, b, nComponents_, md.Serialize().c_str());
    if (!stopOnOverflow_ && ret == DEVICE_BUFFER_OVERFLOW)
    {
        // do not stop on overflow - just reset the buffer
        GetCoreCallback()->ClearImageBuffer(this);
        // don't process this same image again...
        return GetCoreCallback()->InsertImage(this, pI, w, h, b, nComponents_, md.Serialize().c_str(), false);
    }
    else
    {
        return ret;
    }
}
int VCamera::RunSequenceOnThread()
{
    int ret = DEVICE_ERR;
    MM::MMTime startTime = GetCurrentMMTime();

    double exposure = GetSequenceExposure();

    if (!fastImage_)
    {
        //hub_->StopDOSequence();
        //hub_->TriggerDOSequence();
    }
    str = "wait imgdata";
    LogMessage(str);
    std::unique_lock<std::mutex> lock(hub_->kcdaq_->DataProcessThread_->imgDataMutex_);
    hub_->kcdaq_->DataProcessThread_->dataReadyCondition_.wait(lock, [this] { return hub_->kcdaq_->DataProcessThread_->complete_.load(); });
    hub_->kcdaq_->DataProcessThread_->pictureSelect_.store(!hub_->kcdaq_->DataProcessThread_->pictureSelect_.load());
    hub_->kcdaq_->DataProcessThread_->complete_ = false;
    if (!hub_->kcdaq_->DataProcessThread_->pictureSelect_.load()) {
        data = std::move(hub_->kcdaq_->DataProcessThread_->imgData1_);
    }
    else {
        data = std::move(hub_->kcdaq_->DataProcessThread_->imgData2_);
    }
    str = "get imgdata";
    LogMessage(str);

    for (size_t ch = 0; ch < 2; ++ch) {
        img_[ch].SetPixels(data[ch].data());
    }
    str = "insert imgdata";
    LogMessage(str);
    ret = InsertImage();

    if (ret != DEVICE_OK)
    {
        return ret;
    }
    return ret;
};
void LiveThread::Start(long numImages, double intervalMs) {
    std::string str = "LiveThread start";
    cam_->LogMessage(str);
    MMThreadGuard g1(this->stopLock_);
    MMThreadGuard g2(this->suspendLock_);
    numImages_ = numImages;
    intervalMs_ = intervalMs;
    imageCounter_ = 0;
    stop_ = false;
    suspend_ = false;
    cam_->AcquireSnapshot();
    activate();
    actualDuration_ = MM::MMTime{};
    startTime_ = cam_->GetCurrentMMTime();
    lastFrameTime_ = MM::MMTime{};
}
bool LiveThread::IsStopped() {
    MMThreadGuard g(this->stopLock_);
    return stop_;
}
void LiveThread::Suspend() {
    MMThreadGuard g(this->suspendLock_);
    suspend_ = true;
}
bool LiveThread::IsSuspended() {
    MMThreadGuard g(this->suspendLock_);
    return suspend_;
}
void LiveThread::Resume() {
    MMThreadGuard g(this->suspendLock_);
    suspend_ = false;
}
int LiveThread::svc() noexcept
{
    int ret = DEVICE_ERR;
    try
    {
        do
        {
            if (IsStopped()) break;
            ret = cam_->RunSequenceOnThread();
            CDeviceUtils::SleepMs(10);
            
        } while (DEVICE_OK == ret && !IsStopped() && imageCounter_++ < numImages_ - 1 || numImages_ == -1);
        if (IsStopped())
            cam_->LogMessage("SeqAcquisition interrupted by the user\n");
    }
    catch (...) {
        cam_->LogMessage(g_Msg_EXCEPTION_IN_THREAD, false);
    }
    stop_ = true;
    actualDuration_ = cam_->GetCurrentMMTime() - startTime_;
    cam_->OnThreadExiting();
    return ret;
}
void LiveThread::Abort() {
    MMThreadGuard g(this->stopLock_);
    stop_ = true;
}