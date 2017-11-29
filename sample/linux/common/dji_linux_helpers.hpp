/*! @file dji_linux_helpers.hpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Helper functions to handle user configuration parsing, version query and
 * activation.
 *
 *  @copyright
 *  2017 DJI. All rights reserved.
 * */

#ifndef ONBOARDSDK_HELPERS_H
#define ONBOARDSDK_HELPERS_H

#include <fstream>
#include <dji_linux_environment.hpp>
#include <dji_vehicle.hpp>

class LinuxSetup
{
public:
  LinuxSetup(int argc, char **argv, bool enableAdvancedSensing = false);
  ~LinuxSetup();

public:
  void setupEnvironment(int argc, char** argv);
  void initVehicle();
  bool validateSerialPort();

public:
  void setTestSerialDevice(DJI::OSDK::LinuxSerialDevice* serialDevice);
  DJI_Environment* getEnvironment()
  {
    return this->environment;
  }
  DJI::OSDK::Vehicle* getVehicle()
  {
    return this->vehicle;
  }
  DJI::OSDK::Vehicle::ActivateData* getActivateData()
  {
    return &activateData;
  }

private:
  DJI::OSDK::Vehicle*              vehicle;
  DJI::OSDK::LinuxSerialDevice*    testSerialDevice;
  DJI_Environment*                 environment;
  DJI::OSDK::Vehicle::ActivateData activateData;
  int                              functionTimeout; // seconds
  bool                             useAdvancedSensing;
};

#endif // ONBOARDSDK_HELPERS_H
