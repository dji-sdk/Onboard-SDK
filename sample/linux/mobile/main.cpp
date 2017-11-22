/*! @file mobile/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Mobile SDK Communication API usage in a Linux environment.
 *  Shows example usage of the mobile<-->onboard SDK communication API.
 *
 *  @copyright
 *  2017 DJI. All rights reserved.
 * */

#include "mobile_sample.hpp"

using namespace DJI;
using namespace DJI::OSDK;

int
main(int argc, char** argv)
{

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == nullptr)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  setupMSDKParsing(vehicle, &linuxEnvironment);

  return 0;
}