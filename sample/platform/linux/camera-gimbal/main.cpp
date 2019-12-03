/*! @file camera-gimbal/main.cpp
 *  @version 3.3
 *  @date Sep 12 2017
 *
 *  @brief
 *  main for Camera and Gimbal Control API usage in a Linux environment.
 *  Shows example usage of camera commands and gimbal position/speed control
 *  APIs
 *
 *  @Copyright (c) 2017 DJI
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
    << "| [a] Exercise gimbal and camera control                         |\n"
    << "| [b] Exercise camera zoom control                               |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;

  switch (inputChar)
  {
    case 'a':
      gimbalCameraControl(vehicle);
      break;
    case 'b':
      cameraZoomControl(vehicle);
      break;
    default:
      break;
  }

  return 0;
}