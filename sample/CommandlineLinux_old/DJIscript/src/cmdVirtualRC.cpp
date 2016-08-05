/*! @file cmdVirtualRC.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Virtual RC interface for DJI Onboard SDK command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "cmdIO.h"
#include "cmdVirtualRC.h"

#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;

bool VC(Script *script, UserData data)
{
  char* inputData = (char*)data;
  char command[100];
  sscanf(inputData, "--%s", command);
  if (strcmp(command, "help") == 0)
  {
    cout << "|--------------DJI onboardSDK - Virtual Remote control---------|" << endl;
    cout << "| Usage:                                                       |" << endl;
    cout << "| --VC <command> <data>                                        |" << endl;
    cout << "| Following are valid commands:                                |" << endl;
    cout << "| - start {<channel> <value>} start follow mission             |" << endl;
    cout << "|          <channel>   0 roll                                  |" << endl;
    cout << "|                      1 pitch                                 |" << endl;
    cout << "|                      2 throttle                              |" << endl;
    cout << "|                      3 yaw                                   |" << endl;
    cout << "|                      4 gear                                  |" << endl;
    cout << "|                      5 mode                                  |" << endl;
    cout << "|          <value>     364 to 1684, middle is 1024             |" << endl;
    cout << "| - stop   <nodata>    stop follow mission                     |" << endl;
    cout << "|------------------DJI onboardSDK - Script Settings------------|" << endl;

    script->addTask(waitInput);
    script->addTask(waitInput);
  }
  else
  {
    if (sscanf(inputData, "--%*s%s", command))
    {
      strcat(command, "VC");
      script->addTask((UserData)command, data);
    }
    else
      script->addTask(addTask);
  }
  return true;
}

bool startVC(Script *script, UserData data)
{
  script->getVirtualRC()->setControl(true,VirtualRC::CutOff_ToRealRC);
  script->addTask(ctVC,data,500);
    script->addTask(waitInput);

  return true;
}

bool stopVC(Script *script, UserData data)
{
  script->getVirtualRC()->setControl(false,VirtualRC::CutOff_ToRealRC);
  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool ctVC(Script *script, UserData data __UNUSED)
{

  stringstream s;

  s << (char*)data;
  char drop[20];
  s >> drop >> drop;
  
  VirtualRCData UserVRC;
  //! @note Initialize VRC data to defaults
  UserVRC.roll = 1024;
  UserVRC.pitch = 1024;
  UserVRC.throttle = 1024;
  UserVRC.yaw = 1024;
  UserVRC.gear = 1024;
  UserVRC.reserved = 1024;
  //! Initialize VRC mode to be position mode. VRC ATTI mode is unstable and not recommended at this time.
  UserVRC.mode = 1552; 
  UserVRC.Channel_07 = 1024;
  UserVRC.Channel_08 = 1024;
  UserVRC.Channel_09 = 1024;
  UserVRC.Channel_10 = 1024;
  UserVRC.Channel_11 = 1024;
  UserVRC.Channel_12 = 1024;
  UserVRC.Channel_13 = 1024;
  UserVRC.Channel_14 = 1024;
  UserVRC.Channel_15 = 1024;

  //! Parse the user data
  int stickData, channel;

  while (!s.eof())
  {
    s >> channel >> stickData;
    switch (channel)
    {
      case 0:
        UserVRC.roll = stickData;
        break;
      case 1:
        UserVRC.pitch = stickData;
        break;
      case 2:
        UserVRC.throttle = stickData;
        break;
      case 3:
        UserVRC.yaw = stickData;
        break;
      case 4:
        UserVRC.gear = stickData;
        break;
      case 5:
        UserVRC.mode = stickData;
        break;
      default:
        std:cout << "Invalid channel entered: %d"<< channel << std::endl;
        break;  
    }
  }

  script->getVirtualRC()->sendData(UserVRC);
  //__DELETE(data);
  return true;
}
