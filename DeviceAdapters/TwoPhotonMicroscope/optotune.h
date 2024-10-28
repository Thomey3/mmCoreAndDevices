#pragma once
#include "DeviceBase.h"
#include <iostream>

extern const char* g_DeviceNameoptotune;

class optotune : public CGenericBase < optotune >
{
public:
	optotune();
	virtual ~optotune();

	virtual int Initialize();
	virtual int Shutdown();
	void GetName(char* name) const;
	bool Busy() { return false; };

private:
	int		OnPort(MM::PropertyBase* pProp, MM::ActionType eAct);
	int		Baudrate(MM::PropertyBase* pProp, MM::ActionType eAct);
	int		onSetFP(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
	std::string sendStart();
	float	getFPMin();
	float	getFPMax();
	float	getFP();
private:
	bool initialized_;

	std::string port;

	float FPMin = -293;
	float FPMax = 293;
};
