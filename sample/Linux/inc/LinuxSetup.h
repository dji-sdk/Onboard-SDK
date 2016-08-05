/*! @file LinuxSetup.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Executes some setup commands that are required by the Onboard SDK
 *  but are not pertinent to the user's work. Also sets up threads and memory.    
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */


#ifndef LINUX_SETUP_H
#define LINUX_SETUP_H

#include "../../platform/LinuxSerialDevice.h"
#include "LinuxThread.h"
#include "ReadUserConfig.h"

typedef struct ackReturnData{
  int status; 
  uint16_t ack;
} ackReturnData;

typedef struct ackReturnToMobile{
  uint16_t cmdID; 
  uint16_t ack;
} ackReturnToMobile;

int setup(LinuxSerialDevice* serialDevice, CoreAPI* api, LinuxThread* read);
int parseUserConfig();
ackReturnData activate(CoreAPI* api);
ackReturnData takeControl(CoreAPI* api);

#endif //LINUX_SETUP_H