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

#include "LinuxSerialDevice.h"
#include "LinuxThread.h"
#include "ReadUserConfig.h"
#ifdef LIDAR_LOGGING
#include "udpDriver.h"
#endif

typedef struct ackReturnData{
  int status; 
  uint16_t ack;
} ackReturnData;

typedef struct ackReturnToMobile{
  uint16_t cmdID; 
  uint16_t ack;
} ackReturnToMobile;

int setup(LinuxSerialDevice* serialDevice, CoreAPI* api, LinuxThread* read, std::string userConfigPath = UserConfig::defaultUserConfigPath);
bool validateSerialDevice(LinuxSerialDevice* serialDevice, CoreAPI* api);
int parseUserConfig(std::string userConfigPath = UserConfig::defaultUserConfigPath);
ackReturnData activate(CoreAPI* api);
ackReturnData takeControl(CoreAPI* api);


//! Non-Blocking calls
int setupNonBlocking(LinuxSerialDevice* serialDevice, CoreAPI* api, LinuxThread* read, LinuxThread* callback,
		std::string userConfigPath = UserConfig::defaultUserConfigPath);
void activateNonBlocking(CoreAPI* api);
void activateCallback(CoreAPI* api, Header *protHeader, DJI::UserData data);
void takeControlNonBlocking(CoreAPI* api);

//! LiDAR LAS recording functions
#ifdef LIDAR_LOGGING
void udpAndLogging(void);
void managementProcessing(void);
void handler(const boost::system::error_code& error, int signal_number);
void signalHandler(const boost::system::error_code& error, int signal_number);
void startLiDARlogging();
void stopLiDARlogging();

#endif


#endif //LINUX_SETUP_H
