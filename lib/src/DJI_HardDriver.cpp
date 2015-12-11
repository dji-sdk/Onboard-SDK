#include "DJI_HardDriver.h"

using namespace DJI::onboardSDK;

char DJI::onboardSDK::buffer[DJI::onboardSDK::bufsize];

/*! @note in this file we will implement some common corss platform drivers
 *later
 *  @todo implement
 *
 *
 *
 * */

void HardDriver::displayLog(char *buf)
{
    if (buf)
        printf("%s", buf);
    else
        printf("%s", DJI::onboardSDK::buffer);
}
