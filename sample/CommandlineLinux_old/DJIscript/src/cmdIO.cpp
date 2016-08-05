/*! @file cmdIO.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  I/O core interface for DJI Onboard SDK command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "cmdIO.h"

#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;

bool addTask(Script* script, UserData data __UNUSED)
{
  cout << endl;
  cout << "|------------------DJI OnboardSDK command line-------------------|" << endl;
  cout << "|                                                                |" << endl;
  cout << "| ***NOTE: This sample will not be updated. Move to the new      |" << endl;
  cout << "|          Linux C++ sample for new funcitonality.               |" << endl;
  cout << "|                                                                |" << endl;
  cout << "| - Available Command Sets                                       |" << endl;
  cout << "| - < CA > core API                                              |" << endl;
  cout << "| - < FC > flight control                                        |" << endl;
  cout << "| - < CC > camera control                                        |" << endl;
  cout << "| - < VC > virtual remote control                                |" << endl;
  cout << "| - < FM > follow mission                                        |" << endl;
  cout << "| - < WP > waypoint mission                                      |" << endl;
  cout << "| - < HP > hotpoint mission                                      |" << endl;
  cout << "| - < SS > script settings                                       |" << endl;
  cout << "|                                                                |" << endl;
  cout << "| Command examples:                                              |" << endl;
  cout << "| --help <module>                                                |" << endl;
  cout << "|   e.g. --help CA                                               |" << endl;
  cout << "| --<module> <command> <data>                                    |" << endl;
  cout << "|   e.g. --load SS <path> to load your key and                   |" << endl;
  cout << "|        --CA ac to activate your drone                          |" << endl;
  cout << "|                                                                |" << endl;
  cout << "| {<data>} means you can repeat the data like:                   |" << endl;
  cout << "|      --<module> <command> <data1> <data2>                      |" << endl;
  cout << "|      is equal to:                                              |" << endl;
  cout << "|      --<module> <command> <data1>                              |" << endl;
  cout << "|      --<module> <command> <data2>                              |" << endl;
  cout << "|                                                                |" << endl;
  cout << "| If you're new here, try getting the drone version to verify    |" << endl;
  cout << "| that you're connected. Type --help CA to see how to query      |" << endl;
  cout << "| the drone for its version.                                     |" << endl;
  cout << "|                                                                |" << endl;
  cout << "| Visit developer.dji.com/onboard-sdk/documentation for more.    |" << endl;
  cout << "|                                                                |" << endl;
  cout << "|------------------DJI onboardSDK command line-------------------|" << endl;

  script->addTask(waitInput);

  return true;
}

bool waitInput(Script* script, UserData data __UNUSED)
{
  char* inputCmd = new char[512];
  cout << "Waiting input" << endl;
  cin.getline(inputCmd, 512);

  char command[100];
  sscanf(inputCmd, "--%s", command);
  script->addTask((UserData)command, inputCmd);

  return true;
}

bool help(Script* script, UserData data)
{
  char* inputData = (char*)data;
  char command[100];
  if (sscanf(inputData, "--%*s%s", command))
    script->addTask((UserData)command, inputData);
  else
    script->addTask(addTask);
  return true;
}

