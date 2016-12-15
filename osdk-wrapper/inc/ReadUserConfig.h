/*! @file ReadUserConfig.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  User-side configuration parsing for the Linux Onboard SDK interface. 
 *  Meant to provide configuration capability outside the core library.
 *
 *  Actual user config lies in Linux/UserConfig.txt - this file and the
 *  implementation merely read and parse it and populate library variables.
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef USER_CONFIG_H
#define USER_CONFIG_H

//#define API_TRACE_DATA
//#define API_DEBUG_DATA
//#define API_ERROR_DATA
//#define API_BUFFER_DATA
//#define API_MISSION_DATA
//#define API_RTK_DEBUG

#include <DJI_Version.h>
#include <string>
#include <cstring>
#include <iostream>

using namespace DJI::onboardSDK;

namespace UserConfig
{
  const std::string defaultUserConfigPath = "UserConfig.txt";

  //! If using Manifold, the UART2 port shows up as /dev/ttyTHS1.
  //! The Expansion I/O port shows up as /dev/ttyTHS0. 
  //! If using a USB-Serial adapter, use /dev/ttyUSB0 
  //! (may differ if you have other USB-Serial devices plugged in)
  extern std::string deviceName;
  //! Your DJI Developer Account App ID
  extern unsigned int userAppID;
  //! Your DJI Developer Account App Key. 
  extern std::string userKey; 
  //! Default baudRate is 230400. If you want to change this value, also go to 
  //! DJI Assistant 2's SDK tab and change it to match this value.
  extern unsigned int baudRate;
  //! Available choices for targetVersion are versionM100_23, versionA3_31 and versionM100_31
  extern Version targetVersion; 
}

void readUserConfig(std::string UserConfigPath = UserConfig::defaultUserConfigPath);

#endif //USER_CONFIG_H
