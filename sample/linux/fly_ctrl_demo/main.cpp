/*! @file flight-control/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
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

#include "flight_control_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

bool velocity_control_body_fru_coor(Vehicle *vehicle, float vx, float vy, float vz, float yaw)
{
  uint8_t ctrl_flag = \
    ( DJI::OSDK::Control::VERTICAL_VELOCITY | \
      DJI::OSDK::Control::HORIZONTAL_VELOCITY | \
      DJI::OSDK::Control::YAW_RATE | \
      DJI::OSDK::Control::HORIZONTAL_BODY);
      DJI::OSDK::Control::CtrlData data(ctrl_flag, vx, vy, vz, yaw);
  vehicle->control->flightCtrl(data);
}

bool velocity_control_body_neu_coor(Vehicle *vehicle, float vx, float vy, float vz, float yaw)
{
  uint8_t ctrl_flag = \
    ( DJI::OSDK::Control::VERTICAL_VELOCITY | \
      DJI::OSDK::Control::HORIZONTAL_VELOCITY | \
      DJI::OSDK::Control::YAW_RATE | \
      DJI::OSDK::Control::HORIZONTAL_GROUND);
      DJI::OSDK::Control::CtrlData data(ctrl_flag, vx, vy, vz, yaw);
  vehicle->control->flightCtrl(data);
}

/*! main
 *
 */
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
  // vehicle->obtainCtrlAuthority(functionTimeout);

  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Monitored Takeoff + Landing                                |"
    << std::endl;
  std::cout
    << "| [b] Monitored Takeoff + Position Control + Landing             |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;

  char subChar;

  float d_param = 0.0f;

  while ( inputChar != 'e' )
  {
    switch (inputChar)
    {
      case 'a':
        monitoredTakeoff(vehicle);
        break;
      case 'b':
        monitoredLanding(vehicle);
        break;
      case 'v':
        vehicle->obtainCtrlAuthority(functionTimeout);
        break;
      case 'r':
        vehicle->releaseCtrlAuthority(functionTimeout);
        break;
      case 'x':
        std::cout << "Coor Mode you want" << std::endl;
        std::cin >> subChar;
        std::cout << "Input Vx speed" << std::endl;
        std::cin >> d_param;
        if ( subChar == 'g' )
        {
          velocity_control_body_neu_coor(vehicle, d_param, 0.0f, 0.0f, 0.0f);
        }
        else if ( subChar == 'b' )
        {
          velocity_control_body_fru_coor(vehicle, d_param, 0.0f, 0.0f, 0.0f);
        }
        break;
      case 'y':
        std::cout << "Coor Mode you want" << std::endl;
        std::cin >> subChar;
        std::cout << "Input Vy speed" << std::endl;
        std::cin >> d_param;
        if ( subChar == 'g' )
        {
          velocity_control_body_neu_coor(vehicle, 0.0f, d_param, 0.0f, 0.0f);
        }
        else if ( subChar == 'b' )
        {
          velocity_control_body_fru_coor(vehicle, 0.0f, d_param, 0.0f, 0.0f);
        }
        break;
      case 'z':
        std::cout << "Coor Mode you want" << std::endl;
        std::cin >> subChar;
        std::cout << "Input Vz speed" << std::endl;
        std::cin >> d_param;
        if ( subChar == 'g' )
        {
          velocity_control_body_neu_coor(vehicle, 0.0f, 0.0f, d_param, 0.0f);
        }
        else if ( subChar == 'b' )
        {
          velocity_control_body_fru_coor(vehicle, 0.0f, 0.0f, d_param, 0.0f);
        }
        break;
      case 'w':
        std::cout << "Coor Mode you want" << std::endl;
        std::cin >> subChar;
        std::cout << "Input yaw speed" << std::endl;
        std::cin >> d_param;
        if ( subChar == 'g' )
        {
          velocity_control_body_neu_coor(vehicle, 0.0f, 0.0f, 0.0f, d_param);
        }
        else if ( subChar == 'b' )
        {
          velocity_control_body_fru_coor(vehicle, 0.0f, 0.0f, 0.0f, d_param);
        }
        break;
      case 'p':
        std::cout << "Which pos your wanna go: " << std::endl;
        std::cin >> subChar;
        if ( subChar == 'x' )
        {
          moveByPositionOffset(vehicle, 10, 0, 0, 30);
        }
        else if ( subChar == 'y' )
        {
          moveByPositionOffset(vehicle, 0, 10, 0, 30);
        }
        else if ( subChar == 'z' )
        {
          moveByPositionOffset(vehicle, 0, 0, 10, 30);
        }
        else if ( subChar == 'w' )
        {
          std::cout << "Input yaw desired" << std::endl;
          std::cin >> d_param;
          moveByPositionOffset(vehicle, 0, 0, 0, d_param);
        }
        break;
      default:
        break;
    }
    std::cin >> inputChar;
  }

  return 0;
}
