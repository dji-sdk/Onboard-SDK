/*! @file ReadUserConfig.cpp
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

#include "ReadUserConfig.h" 
#include <fstream>

using namespace UserConfig;
using namespace std;

namespace UserConfig{
  //! If using Manifold, the UART2 port shows up as /dev/ttyTHS1. 
  //! The Expansion I/O port shows up as /dev/ttyTHS0. 
  //! If using a USB-Serial adapter, use /dev/ttyUSB0 
  //! (may differ if you have other USB-Serial devices plugged in)
  std::string deviceName;
  //! Your DJI Developer Account App ID
  unsigned int userAppID;
  //! Your DJI Developer Account App Key. 
  std::string userKey; 
  //! Default baudRate is 230400. If you want to change this value, also go to 
  //! DJI Assistant 2's SDK tab and change it to match this value.
  unsigned int baudRate ;
  //! Available choices for targetVersion are versionM100_23, versionA3_31 and versionM100_31
  DJI::onboardSDK::Version targetVersion; 
}

void readUserConfig(string UserConfigPath)
{
  char line[1024];

  static char key[70];
  char devName[20];
  char droneVer[20];
  int id;


  bool set_ID, set_KEY, set_Baud, set_Serial_Device, set_Drone_Version;

  ifstream read(UserConfigPath);

  if (read.is_open())
  {
    while (!read.eof())
    {
      read.getline(line, 1024);
      if (*line != 0) //! @note sscanf have features on empty buffer.
      {
        if (sscanf(line, "ID:%d", &UserConfig::userAppID))
        {
          set_ID = true;
        }
        if (sscanf(line, "KEY:%s", key))
        {
          UserConfig::userKey = std::string(key);
          set_KEY = true;
        }
        if (sscanf(line, "DeviceName:%s", devName))
        {
          UserConfig::deviceName = std::string(devName);
          set_Serial_Device = true;
        }
        if (sscanf(line, "Version:%s", droneVer))
        {
          if (!strcmp(droneVer,"versionM100_31"))
            UserConfig::targetVersion = versionM100_31;
          if (!strcmp(droneVer,"versionM100_23"))
            UserConfig::targetVersion = versionM100_23;
          if (!strcmp(droneVer,"versionA3_31"))
            UserConfig::targetVersion = versionA3_31;
          if (!strcmp(droneVer,"versionA3_32"))
            UserConfig::targetVersion = versionA3_32;
          set_Drone_Version = true;
        }
        if (sscanf(line, "BaudRate:%d", &UserConfig::baudRate))
        {
          set_Baud = true;
        }
      }
    }
    if (set_Baud && set_ID && set_KEY && set_Drone_Version && set_Serial_Device)
      cout << "User Configuration read successfully. \n\n";
    else
      cout << "There's an error with your UserConfig.txt file.\n";

  }
  else
    cout << "User config file could not be opened. Make sure you are running this app from the sample/Linux/ directory\n"
         << "and have sufficient permissions." << endl;
  read.close();
}

