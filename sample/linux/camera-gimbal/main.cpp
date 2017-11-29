/*! @file camera-gimbal/main.cpp
 *  @version 3.3
 *  @date Sep 12 2017
 *
 *  @brief
 *  main for Camera and Gimbal Control API usage in a Linux environment.
 *  Shows example usage of camera commands and gimbal position/speed control
 *  APIs
 *
 *  @copyright
 *  2017 DJI. All rights reserved.
 * */

#include "camera_gimbal_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int
main(int argc, char** argv)
{

  // Setup the OSDK: Read config file, create vehicle, activate.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Exercise gimbal and camera control                         |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;

  switch (inputChar)
  {
    case 'a':
      gimbalCameraControl(vehicle);
      break;
    default:
      break;
  }

  return 0;
}
