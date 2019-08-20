/*! @file mfio/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Multi-Function I/O API usage in a Linux environment.
 *  Shows example usage of the APIs available for controlling the MFIO pins
 *  on the vehicle/FC.
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

#include "mfio_sample.hpp"

using namespace DJI::OSDK;

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

  std::cout
    << "To run the MFIO sample, please first do the following steps:\n"
       "1. Open DJI Assistant 2 and go to the Tools page.\n"
       "2. Click on the Function Channels tab.\n"
       "3. Select SDK4 for channel F3.\n"
       "4. Connect a logic analyzer/oscilloscope to this channel.\n"
       "   The pin diagram, from top to bottom, is Gnd, 5V, Signal.\n"
       "   You will only need to connect to the Gnd and Signal pins.\n\n";
  std::cout << "All loopback demos use SDK4 as output and SDK5 as input \n\n";

  std::cout << "ADC demo uses SDK5 as input \n\n";

  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Set PWM output (block api)                                 |"
    << std::endl;
  std::cout
    << "| [b] Set PWM output (non-block api)                             |"
    << std::endl;
  std::cout
    << "| [c] Set GPIO loopback (blocking)                               |"
    << std::endl;
  std::cout
    << "| [d] Set GPIO loopback (non-blocking)                           |"
    << std::endl;
  std::cout
    << "| [e] Set ADC (blocking)                                         |"
    << std::endl;
  std::cout
    << "| [f] Set ADC (non-blocking)                                     |"
    << std::endl;

  char inputChar;
  std::cin >> inputChar;

  switch (inputChar)
  {
    case 'a':
      pwmOutputBlockingApiDemo(vehicle);
      break;
    case 'b':
      pwmOutputNonBlockingApiDemo(vehicle);
      break;
    case 'c':
      gpioLoopbackBlockingApiDemo(vehicle);
      break;
    case 'd':
      gpioLoopbackNonBlockingApiDemo(vehicle);
      break;
    case 'e':
      adcBlockingApiDemo(vehicle);
      break;
    case 'f':
      adcNonBlockingApiDemo(vehicle);
      break;
    default:
      break;
  }

  return 0;
}
