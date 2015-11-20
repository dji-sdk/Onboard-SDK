#ifndef WINDOWSHARDDRIBER
#define WINDOWSHARDDRIBER

#include "DJI_HardDriver.h"

class WinHardDriver:public DJI::onboardSDK::HardDriver
{
public:
	WinHardDriver();
public:
	void init();
	unsigned int getTimeStamp();
	size_t send(const uint8_t *buf, size_t len);
	size_t readall(uint8_t *buf, size_t maxlen);

public:
	void lockMemory();
	void freeMemory();

	void lockMSG();
	void freeMSG();
};

#endif//WINDOWSHARDDRIBER
