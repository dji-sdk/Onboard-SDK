/*! @file LinuxCleanup.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Executes some cleanup commands that are required by the Onboard SDK
 *  but are not pertinent to the user's work. Also cleans up threads and memory.    
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */


#ifndef LINUX_CLEANUP_H
#define LINUX_CLEANUP_H

#include "LinuxSerialDevice.h"
#include "LinuxThread.h"
#include "ReadUserConfig.h"
#include "LinuxSetup.h"


int cleanup(LinuxSerialDevice* serialDevice, CoreAPI* api, Flight* flight, LinuxThread* read);
int cleanupNonBlocking(LinuxSerialDevice* serialDevice, CoreAPI* api, Flight* flight, LinuxThread* read,  LinuxThread* callback);
ackReturnData releaseControl(CoreAPI* api);
void releaseControlNonBlocking(CoreAPI* api);


#endif //LINUX_CLEANUP_H
