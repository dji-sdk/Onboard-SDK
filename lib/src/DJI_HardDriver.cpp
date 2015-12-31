#include "DJI_HardDriver.h"

using namespace DJI::onboardSDK;

char DJI::onboardSDK::buffer[DJI::onboardSDK::bufsize];

void HardDriver::displayLog(char *buf)
{
    if (buf)
        printf("%s", buf);
    else
        printf("%s", DJI::onboardSDK::buffer);
}

void HardDriver::inputStream(char *buf, size_t size)
{
}
