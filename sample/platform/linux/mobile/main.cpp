/*! @file mobile/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Mobile SDK Communication API usage in a Linux environment.
 *  Shows example usage of the mobile<-->onboard SDK communication API.
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

#include <sstream>
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

  std::string input = "";

  // How to get a string/sentence with spaces
  std::cout << "Please choose the test :\n 1 means testing moc \n 2 means testing mobileDevice\n 3 means testing both of them\n"<< std::endl;

  // How to get a number.
  int myNumber = 0;

  while (true) {
    std::cout << "Please enter a valid number: ";
    getline(std::cin, input);

    // This code converts from string to number safely.
    std::stringstream myStream(input);
    if (myStream >> myNumber && myNumber >=1 && myNumber <=3)
      break;
    std::cout << "Invalid number, please try again" << std::endl;
  }
  std::cout << "You entered: " << myNumber << std::endl << std::endl;
  setTestSuite(myNumber);
  setupMSDKParsing(vehicle, &linuxEnvironment);

  return 0;
}