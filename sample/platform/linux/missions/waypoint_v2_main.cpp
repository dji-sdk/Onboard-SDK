/*! @file missions/waypoint_v2_main.cpp
 *  @version 3.8
 *  @date Mar 07 2019
 *
 *  @brief
 *  main for Waypoint Missions V2 API usage in a Linux environment.
 *  Shows example usage of the Waypoint Missions through
 *  the Mission Manager API.
 *
 *  @Copyright (c) 2019 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "waypoint_v2_sample.hpp"

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

  int     responseTimeout = 1;

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);


  // Set up telemetry subscription
  if (!setUpSubscription(vehicle, responseTimeout))
  {
    std::cout << "Failed to set up Subscription!" << std::endl;
    return -1;
  }

  // Setup variables for use
  uint8_t wayptPolygonSides;

  // wait for the subscription
  sleep(2);

  wayptPolygonSides = 6;
  createAndUploadWaypointMission(vehicle, wayptPolygonSides, responseTimeout);

  using namespace dji::waypointv2;

  // wait for the WP2 state to be ReadyToExecute
  sleep(4);

  std::vector<WaypointActionConfig> actions;
  char inputChar = ' ';

  int actionCounter = 0;
  while(inputChar != 'n')
  {
    if(actionCounter >= wayptPolygonSides)
    {
      std::cout << "There are enough actions for this waypoint mission.\n";
      std::cout << "You could add more but for demo purpose we will exit here\n";
      break;
    }

    std::cout
      << "| Would you like to add some or more actions to your mission?"
      << std::endl;
    std::cout
      << "| [y] Yes"
      << std::endl;
    std::cout
      << "| [n] No"
      << std::endl;
    std::cin >> inputChar;

    if(inputChar == 'n')
    {
      break;
    }

    WaypointActionActuatorType actuatorType;
    WaypointActionTriggerType triggerType;

    std::cout
      << "| Available Actuator Type:"
      << std::endl;
    std::cout
      << "| [a] Camera"
      << std::endl;
    std::cout
      << "| [b] Gimbal"
      << std::endl;
    std::cout
      << "| [c] Aircraft Control"
      << std::endl;

    std::cin >> inputChar;
    switch (inputChar)
    {
      case 'a':
      { actuatorType = dji::waypointv2::Camera;}
        break;
      case 'b':
      { actuatorType = dji::waypointv2::Gimbal; // namespace ambiguity }
        break;
        case 'c':
        { actuatorType = AircraftControl; }
        break;
        default:
        {
          std::cout << "User input the wrong actuator type" << std::endl;
          continue;
        }
          break;
      }
    }

    std::cout
      << "| Available Trigger Type:"
      << std::endl;
    std::cout
      << "| [a] ReachPoints (Not supported yet)"
      << std::endl;
    std::cout
      << "| [b] Associate"
      << std::endl;
    std::cout
      << "| [c] Trajectory (Not supported yet)"
      << std::endl;
    std::cout
      << "| [d] Simple Interval"
      << std::endl;
    std::cout
      << "| [e] Simple Reach Point"
      << std::endl;

    std::cin >> inputChar;
    switch (inputChar)
    {
      case 'a':
      { triggerType= ReachPoints;
        std::cout << "This trigger is not supported yet" << std::endl;
        continue;
      }
        break;
      case 'b':
      { triggerType= Associate; }
        break;
      case 'c':
      { triggerType= Trajectory;
        std::cout << "This trigger is not supported yet" << std::endl;
        continue;
      }
        break;
      case 'd':
      { triggerType= SimpleInterval; }
        break;
      case 'e':
      { triggerType= SimpleReachPoint; }
        break;
      default:
      {
        std::cout << "User input the wrong trigger type" << std::endl;
        continue;
      }
        break;
    }

    if(createActions(vehicle, actuatorType, triggerType, actions))
    {
      ++actionCounter;
    }
    sleep(1);
  }

  if(actionCounter > 0)
  {
    uploadActions(vehicle, actions);
  }

  sleep(1);

  startWaypointMission(vehicle);

  if(!teardownSubscription(vehicle, DEFAULT_PACKAGE_INDEX, responseTimeout))
  {
    std::cout << "Failed to tear down Subscription!" << std::endl;
    return -1;
  }

  // Mission will continue when we exit here
  sleep(6);

  return 0;
}
