/*! @file DJI_HardDriver.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Serial device driver abstraction. See DJI_HardDriver.h for more info.
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "DJI_HardDriver.h"

using namespace DJI::onboardSDK;

char DJI::onboardSDK::buffer[DJI::onboardSDK::bufsize];

void HardDriver::displayLog(const char *buf)
{
  if (buf)
    printf("%s", buf);
  else
    printf("%s", DJI::onboardSDK::buffer);
}

