#include "optotune.h"
const char* g_DeviceNameoptotune = "optotune";

optotune::optotune() :
    initialized_(false)
{
    // Port
    CPropertyAction* pAct = new CPropertyAction(this, &optotune::OnPort);
    CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);
}

optotune::~optotune()
{
    Shutdown();
}

int optotune::Initialize()
{
    initialized_ = true;
    std::string answer = sendStart();
    if (answer != "OK")
    {
        return DEVICE_ERR;
    }
    FPMin = getFPMin();
    FPMax = getFPMax();
    CPropertyAction* pAct = new CPropertyAction(this, &optotune::onSetFP);
    int nRet = CreateProperty("Focal Power(dpt)", "0.00", MM::Float, false, pAct);
    SetPropertyLimits("Focal Power(dpt)", FPMin, FPMax);

    return DEVICE_OK;
}
int optotune::Shutdown()
{
    initialized_ = false;
    return DEVICE_OK;
}

void optotune::GetName(char* name) const
{
    CDeviceUtils::CopyLimitedString(name, g_DeviceNameoptotune);
}


int optotune::OnPort(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(port.c_str());
    }
    else if (eAct == MM::AfterSet)
    {
        if (initialized_)
        {
            // revert
            pProp->Set(port.c_str());
            return DEVICE_ERR;
        }

        pProp->Get(port);
    }

    return DEVICE_OK;
}

int optotune::onSetFP(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        double FP = getFP();
        pProp->Set(FP);
    }
    else if (eAct == MM::AfterSet)
    {
        double FP;
        std::ostringstream command;
        pProp->Get(FP);
        std::string fp = std::to_string(FP);
        command << "SETFP=" << fp;
        int result = SendSerialCommand(port.c_str(), command.str().c_str(), "\r\n");
        std::string answer;
        result = GetSerialAnswer(port.c_str(), "\r\n", answer);
        PurgeComPort(port.c_str());
        if (answer != "OK")
        {
            return DEVICE_ERR;
        }
    }
    return DEVICE_OK;
}

std::string optotune::sendStart()
{
    int result = SendSerialCommand(port.c_str(), "start", "\r\n");

    std::string answer;
    result = GetSerialAnswer(port.c_str(), "\r\n", answer);
    PurgeComPort(port.c_str());
    if (result == DEVICE_OK) {
        if (answer == "OK") {
            // 处理接收到的"ready"信息
            std::cout << "Device is ready." << std::endl;
        }
        else {
            // 接收到的信息不是"ready"
            std::cout << "Received unexpected response: " << answer << std::endl;
        }
    }
    else {
        // 错误处理
        std::cerr << "Failed to receive answer: " << result << std::endl;
    }

    return answer;
}

float optotune::getFPMin()
{
    int result = SendSerialCommand(port.c_str(), "getfpmin", "\r\n");

    std::string answer;
    result = GetSerialAnswer(port.c_str(), "\r\n", answer);
    PurgeComPort(port.c_str());
    if (result == DEVICE_OK) {
        return std::stof(answer);
    }
    else
    {
        return DEVICE_ERR;
    }

}

float optotune::getFPMax()
{
    int result = SendSerialCommand(port.c_str(), "getfpmax", "\r\n");

    std::string answer;
    result = GetSerialAnswer(port.c_str(), "\r\n", answer);
    PurgeComPort(port.c_str());
    if (result == DEVICE_OK) {
        return std::stof(answer);
    }
    else
    {
        return DEVICE_ERR;
    }

}

float optotune::getFP()
{
    int result = SendSerialCommand(port.c_str(), "getfp", "\r\n");

    std::string answer;
    result = GetSerialAnswer(port.c_str(), "\r\n", answer);
    PurgeComPort(port.c_str());
    return std::stof(answer);
}