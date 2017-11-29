/*! @file logging/main.cpp
 *  @version 3.4
 *  @date Sep 15 2017
 *
 *  @brief
 *  Logging API usage in a Linux environment.
 *  Shows example usage of various logging APIs and controls.
 *
 *  @copyright
 *  2017 DJI. All rights reserved.
 * */

#include "logging_sample.hpp"

using namespace DJI::OSDK;

int
main(int argc, char** argv)
{
  DSTATUS("Logging is completely independent of DJI::OSDK::Vehicle.");
  DSTATUS("In this example, we don't instantiate a Vehicle at all.\n");
  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Logging Example                                            |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;
  switch (inputChar)
  {
    case 'a':
      // Waypoint call
      dynamicLoggingControlExample();
      break;
    default:
      break;
  }
  return 0;
}