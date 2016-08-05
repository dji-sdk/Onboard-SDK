/*! @file cmdFollow.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Follow Me interface for DJI Onboard SDK command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "cmdIO.h"
#include "cmdFollow.h"

#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;

bool FM(Script *script, UserData data)
{
  char *inputData = (char *)data;
  char command[100];
  sscanf(inputData, "--%s", command);
  if (strcmp(command, "help") == 0)
  {
    cout << "|------------------DJI onboardSDK - Follow mission ----|" << endl;
    cout << "| Usage                                                |" << endl;
    cout << "| --FM <command> <data>                                |" << endl;
    cout << "| Following are valid commands:                        |" << endl;
    cout << "| - start    <latitude> <longitude> <height>           |" << endl;
    cout << "|            <latitude>  type double, unit 1 rad       |" << endl;
    cout << "|            <longitude> type double, unit 1 rad       |" << endl;
    cout << "|            <height>    type int, unit 1 m            |" << endl;
    cout << "| - stop     <nodata> stop follow mission              |" << endl;
    cout << "| - pause    <nodata> pause follow mission             |" << endl;
    cout << "| - restart  <nodata> restart follow mission           |" << endl;
    cout << "| - update   <latitude> <longitude> <altitude>         |" << endl;
    cout << "|      update target position                          |" << endl;
    cout << "|------------------DJI onboardSDK - Script Settings----|" << endl;

    script->addTask(waitInput);
    script->addTask(waitInput);
  }
  else
  {
    if (sscanf(inputData, "--%*s%s", command))
    {
      strcat(command, "FM");
      script->addTask((UserData)command, data);
    }
    else
      script->addTask(addTask);
  }
  return true;
}

bool startFM(Script *script, UserData data)
{
  stringstream s;

  FollowData fdata;

  //Handle with doubles instead of floats for Manifold
  double latInput, lonInput, heightInput;

  s << (char *)data;
  char drop[20];
  s >> drop >> drop;

  fdata.mode = Follow::MODE_RELATIVE;
  fdata.yaw = Follow::YAW_TOTARGET;
  fdata.target.angle = 0;
  fdata.sensitivity = 1;
  s >> latInput >> lonInput >> heightInput;

  fdata.target.latitude = latInput;
  fdata.target.longitude = lonInput;
  fdata.target.height = heightInput;

  script->getFollow()->start(&fdata,0,0);

  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool stopFM(Script *script, UserData data)
{
  script->getFollow()->stop();

  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool pauseFM(Script *script, UserData data)
{
  script->getFollow()->pause(true);

  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool restartFM(Script *script, UserData data)
{
  script->getFollow()->pause(false);

  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool updateFM(Script *script, UserData data)
{
  stringstream s;

  FollowTarget fdata;

  //Handle with doubles instead of floats for Manifold
  double latInput, lonInput, heightInput;

  s << (char *)data;
  char drop[20];
  s >> drop >> drop;

  s >> latInput >> lonInput >> heightInput;
  fdata.latitude = latInput;
  fdata.longitude = lonInput;
  fdata.height = heightInput;

  script->getFollow()->updateTarget(fdata);

  __DELETE(data);
  script->addTask(waitInput);
  return true;
}
