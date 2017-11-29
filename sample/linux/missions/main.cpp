/*! @file missions/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for GPS Missions API usage in a Linux environment.
 *  Shows example usage of the Waypoint Missions and Hotpoint Missions through
 *  the
 *  Mission Manager API.
 *
 *  @copyright
 *  2017 DJI. All rights reserved.
 * */

#include "mission_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int
main(int argc, char** argv)
{
  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);

  // Setup variables for use
  uint8_t wayptPolygonSides;
  int     hotptInitRadius;
  int     responseTimeout = 1;

  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Waypoint Mission                                           |"
    << std::endl;
  std::cout
    << "| [b] Hotpoint Mission                                           |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;
  switch (inputChar)
  {
    case 'a':
      // Waypoint call
      wayptPolygonSides = 6;
      runWaypointMission(vehicle, wayptPolygonSides, responseTimeout);
      break;
    case 'b':
      hotptInitRadius = 10;
      runHotpointMission(vehicle, hotptInitRadius, responseTimeout);
      break;
    default:
      break;
  }

  return 0;
}