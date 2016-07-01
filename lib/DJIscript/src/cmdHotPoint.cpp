/*! @file cmdHotPoint.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  HotPoint commands for DJI Onboard SDK command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */


#include "cmdIO.h"
#include "cmdHotPoint.h"

#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;

bool HP(Script *script, UserData data)
{
  char *inputData = (char *)data;
  char command[100];
  sscanf(inputData, "--%s", command);
  if (strcmp(command, "help") == 0)
  {
    cout << "|------------------DJI onboardSDK - Hot Point mission------------|" << endl;
    cout << "| Usage:                                                         |" << endl;
    cout << "| --HP <command> <data>                                          |" << endl;
    cout << "| Following are valid commands:                                  |" << endl;
    cout << "| - start    <latitude> <longitude> <height> <radius> <yawrate>  |" << endl;
    cout << "|            <latitude>  type double, unit 1 rad                 |" << endl;
    cout << "|            <longitude> type double, unit 1 rad                 |" << endl;
    cout << "|            <height>    type double, unit 1 meter               |" << endl;
    cout << "|            <radius>    type float,  unit 1 meter               |" << endl;
    cout << "|            <yawrate>   type float,  unit 1 degree/s            |" << endl;
    cout << "| - stop     <nodata> stop hot point                             |" << endl;
    cout << "| - pause    <nodata> pause hot point                            |" << endl;
    cout << "| - restart  <nodata> restart hot point                          |" << endl;
    cout << "|------------------DJI onboardSDK - Script Settings--------------|" << endl;

    script->addTask(waitInput);
    script->addTask(waitInput);
  }
  else
  {
    if (sscanf(inputData, "--%*s%s", command))
    {
      strcat(command, "HP");
      script->addTask((UserData)command, data);
    }
    else
      script->addTask(addTask);
  }
  return true;
}

bool startHP(Script *script, UserData data)
{
  stringstream s;

  HotPointData fdata;

  s << (char *)data;
  char drop[20];
  s >> drop >> drop;

  //Handle with local variables for manifold
  float32_t radiusInput;
  double latInput, lonInput, heightInput, yawRateInput;

  s >> latInput >> lonInput >> heightInput >> radiusInput >> yawRateInput ;

  //Implicit typecast for manifold
  fdata.latitude = latInput;
  fdata.longitude = lonInput;
  fdata.height = heightInput;
  fdata.yawRate = yawRateInput;

  if (radiusInput > 0)
  {
    fdata.radius = radiusInput;
    fdata.clockwise = 0;
  }
  else
  {
    fdata.radius = -radiusInput;
    fdata.clockwise = 1;
  }
  //! @todo add operational entrance
  fdata.yawMode = HotPoint::YAW_CUSTOM;
  fdata.startPoint = HotPoint::VIEW_NEARBY;
  script->getHotpoint()->setData(fdata);
  script->getHotpoint()->start();

  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool stopHP(Script *script, UserData data)
{
  script->getHotpoint()->stop();

  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool pauseHP(Script *script, UserData data)
{
  script->getHotpoint()->pause(true);

  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool restartHP(Script *script, UserData data)
{
  script->getHotpoint()->pause(false);

  __DELETE(data);
  script->addTask(waitInput);
  return true;
}
