/*! @file cmdWayPoint.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Waypoint commands for DJI Onboard SDK command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "cmdIO.h"
#include "cmdWayPoint.h"

#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;

bool WP(Script *script, UserData data)
{
  char *inputData = (char *)data;
  char command[100];
  sscanf(inputData, "--%s", command);
  if (strcmp(command, "help") == 0)
  {
    cout << "|------------------DJI onboardSDK - Waypoint-------------------|" << endl;
    cout << "| Usage:                                                       |" << endl;
    cout << "| --WP <command> <data>                                        |" << endl;
    cout << "| Following are valid commands:                                |" << endl;
    cout << "| - init <indexNumber>                                         |" << endl;
    cout << "| - ap <index> <latitude> <longitude> <altitude>               |" << endl;
    cout << "|   NOTE:  (enter lat and lon in radians)                      |" << endl;
    cout << "|                                                              |" << endl;
    cout << "| - start    <N/A>                                             |" << endl;
    cout << "| - stop     <N/A>                                             |" << endl;
    cout << "| - pause    <N/A>                                             |" << endl;
    cout << "| - restart  <N/A>                                             |" << endl;
    cout << "|                                                              |" << endl;
    cout << "| **NOTE:  First run init with indexNumber =  no. of waypoints |" << endl;
    cout << "|          then run ap 'indexNumber' times                     |" << endl; 
    cout << "|          with 'index' values going from 0 -> indexNumber     |" << endl;
    cout << "|          then run start                                      |" << endl; 
    cout << "| **Ex:    --WP init 4                                         |" << endl;
    cout << "|          --WP ap 0 0.393446 1.988958 20                      |" << endl;
    cout << "|          --WP ap 1 0.393448 1.988958 40                      |" << endl;
    cout << "|          --WP ap 2 0.393448 1.988960 30                      |" << endl;
    cout << "|          --WP ap 3 0.393446 1.988958 20                      |" << endl;
    cout << "|          --WP start                                          |" << endl;    
    cout << "|------------------DJI onboardSDK - Waypoint-------------------|" << endl;

    __DELETE(data);
    script->addTask(waitInput);
  }
  else
  {
    sscanf(inputData, "--%*s%s", command);
    strcat(command, "WP");
    script->addTask((UserData)command, data);
  }
  return true;
}

bool initWP(Script *script, UserData data)
{
  stringstream s;

  WayPointInitData fdata;
  int temp_indexNumber_data; //! Stringstream does not natively convert to uint_8.

  s << (char*)data;
  char drop[20];
  s >> drop >> drop;

  fdata.maxVelocity = 10;
  fdata.idleVelocity = 5;
  fdata.finishAction = 0;
  fdata.executiveTimes = 1;
  fdata.yawMode = 0;
  fdata.traceMode = 0;
  fdata.RCLostAction = 0;
  fdata.gimbalPitch = 0;
  fdata.latitude = 0;
  fdata.longitude = 0;
  fdata.altitude = 0;
  s >> temp_indexNumber_data;

  fdata.indexNumber = temp_indexNumber_data;

  script->getWaypoint()->init(&fdata);

  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool startWP(Script *script, UserData data)
{
  script->getWaypoint()->start();
  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool stopWP(Script *script, UserData data)
{
  script->getWaypoint()->stop();
  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool pauseWP(Script *script, UserData data)
{
  script->getWaypoint()->pause(true);
  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool restartWP(Script *script, UserData data)
{
  script->getWaypoint()->pause(false);
  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool apWP(Script *script, UserData data)
{
  stringstream s;

  WayPointData fdata;
  //! Stringstream does not natively convert to uint_8.
  int temp_index_data;
  
  //Handle with doubles instead of floats for Manifold
  double latInput, lonInput, altiInput;

  s << (char *)data;
  char drop[20];
  s >> drop >> drop;

  fdata.damping = 0;
  fdata.yaw = 0;
  fdata.gimbalPitch = 0;
  fdata.turnMode = 0;
  fdata.hasAction = 0;
  fdata.actionTimeLimit = 100;
  fdata.actionNumber = 0;
  fdata.actionRepeat = 0;
  for (int i = 0; i < 16; ++i)
  {
    fdata.commandList[i] = 0;
    fdata.commandParameter[i] = 0;
  }
  s >> temp_index_data >> latInput >> lonInput >> altiInput;
  
  fdata.longitude = lonInput;
  fdata.latitude = latInput;
  fdata.altitude = altiInput;
  fdata.index = temp_index_data;

  script->getWaypoint()->uploadIndexData(&fdata);
  __DELETE(data);
  script->addTask(waitInput);
  return true;
}
