/*! @file cmdCoreAPI.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Core API Commands for DJI Onboard SDK Command line example. 
 *  Use for getting/setting data, activation, taking control etc.
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "cmdIO.h"
#include "cmdCoreAPI.h"

#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;

bool CA(Script* script __UNUSED, UserData data)
{
  char* inputData = (char*)data;
  char command[100];
  sscanf(inputData, "--%s", command);
  if (strcmp(command, "help") == 0)
  {
    cout << "|------------------DJI onboardSDK - Core API-----------------------|" << endl;
    cout << "|  Usage:                                                          |" << endl;
    cout << "|  --CA <command> <data>                                           |" << endl;
    cout << "|  Following are valid commands:                                   |" << endl;
    cout << "|  - ac <N/A> activate, load data from files. See also SS load     |" << endl;
    cout << "|  - vs <N/A> read flight control version                          |" << endl;
    cout << "|  - bf {<channel> <frequency>} set broadcast frequency            |" << endl;
    cout << "|        <channel>   0 ~ 11                                        |" << endl;
    cout << "|          See SDK tab in Assistant 2 for order of channels        |" << endl;
    cout << "|        <frequency> 0 1 10 50 100                                 |" << endl;
    cout << "|          other input will hold the latest frequency              |" << endl;
    cout << "|  - bd  <flag> show broadcast data based on flag.                 |" << endl;
    cout << "|        <flag>  0x0001 time                                       |" << endl;
    cout << "|                0x0002 quaternion                                 |" << endl;
    cout << "|                0x0004 acceleration                               |" << endl;
    cout << "|                0x0008 velocity                                   |" << endl;
    cout << "|                0x0010 yawRate                                    |" << endl;
    cout << "|                0x0020 position                                   |" << endl;
    cout << "|                0x0040 magnetometer                               |" << endl;
    cout << "|                0x0080 remote controller                          |" << endl;
    cout << "|                0x0100 gimbal                                     |" << endl;
    cout << "|                0x0200 device status                              |" << endl;
    cout << "|                0x0400 battery                                    |" << endl;
    cout << "|                0x0800 control status                             |" << endl;
    cout << "|  - ct <bool>  true for obtain control, false for release control |" << endl;
    cout << "|       <bool>  1 Obtain Control                                   |" << endl;
    cout << "|       <bool>  0 Release Control                                  |" << endl;
    cout << "|  - sy <freq> set sync frequency in Hz                            |" << endl;
    cout << "|------------------DJI onboardSDK - Core API-----------------------|" << endl;

    __DELETE(data);
    script->addTask(waitInput);
  }
  else
  {
    sscanf(inputData, "--%*s%s", command);
    strcat(command, "CA");
    script->addTask((UserData)command, data);
  }
  return true;
}

bool bdCA(Script* script, UserData data)
{
  stringstream s;
  std::string hexCmd;

  s << (char*)data;
  char drop[20];
  s >> drop >> drop;
  uint16_t flag;
  s >> hexCmd;
  flag = strtoul(hexCmd.c_str(),NULL,16);

  BroadcastData bd = script->getApi()->getBroadcastData();
  for (int i = 0; i < 12; ++i)
  {
    if (flag & (1 << i))
    {
      cout << endl;
      switch (i)
      {
        case 0:
          if (script->getApi()->getSDKVersion() != versionM100_23)
          {
            cout << "Time: " << bd.timeStamp.time << endl;
            cout << "Nano: " << bd.timeStamp.nanoTime << endl;
            cout << "Sync: " << (int)bd.timeStamp.syncFlag << endl;
          }
          else
            cout << "Time: " << bd.timeStamp.time << endl;
          break;
        case 1:
          cout << "Q0: " << bd.q.q0 << endl;
          cout << "Q1: " << bd.q.q1 << endl;
          cout << "Q2: " << bd.q.q2 << endl;
          cout << "Q3: " << bd.q.q3 << endl;
          break;
        case 2:
          cout << "Ax: " << bd.a.x << endl;
          cout << "Ay: " << bd.a.y << endl;
          cout << "Az: " << bd.a.z << endl;
          break;
        case 3:
          cout << "Vx: " << bd.v.x << endl;
          cout << "Vy: " << bd.v.y << endl;
          cout << "Vz: " << bd.v.z << endl;
          break;
       case 4:
          cout << "Wx: " << bd.w.x << endl;
          cout << "Wy: " << bd.w.y << endl;
          cout << "Wz: " << bd.w.z << endl;
          break;
        case 5:
          cout << "Longitude: " << bd.pos.longitude << endl;
          cout << "Latitude:  " << bd.pos.latitude << endl;
          cout << "Altitude:  " << bd.pos.altitude << endl;
          cout << "Height:  " << bd.pos.height << endl;
          cout << "Health:  " << (int)bd.pos.health << endl;
          break;
        case 6:
          cout << "Mx: " << bd.mag.x << endl;
          cout << "My: " << bd.mag.y << endl;
          cout << "Mz: " << bd.mag.z << endl;
          break;
        case 7:
          cout << "Throttle: " << bd.rc.throttle << endl;
          cout << "Yaw:    " << bd.rc.yaw << endl;
          cout << "Roll:   " << bd.rc.roll << endl;
          cout << "Pitch:  " << bd.rc.pitch << endl;
          cout << "Gear:   " << bd.rc.gear << endl;
          cout << "Mode:   " << bd.rc.mode << endl;
          break;
        case 8:
          cout << "Yaw:   " << bd.gimbal.yaw << endl;
          cout << "Roll:  " << bd.gimbal.roll << endl;
          cout << "Pitch: " << bd.gimbal.pitch << endl;
          break;
        case 9:
          cout << "Status: " << (int)bd.status << endl;
          break;
        case 10:
          cout << "Battery: " << (int)bd.battery << endl;
          break;
        case 11:
          cout << "Control: " << (int)Flight::MODE_NOT_SUPPORTED << endl;
          cout << "Device:  " << (int)bd.ctrlInfo.deviceStatus << endl;
          cout << "Status:  " << (int)bd.ctrlInfo.flightStatus << endl;
          break;
      }
      cout << endl;
    }
  }

  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool bfCA(Script* script, UserData data)
{
  stringstream s;

  s << (char*)data;
  char drop[20];
  s >> drop >> drop;
  uint8_t freq[16];
  while (!s.eof())
  {
    int channel, freqData;
    s >> channel >> freqData;
    switch (freqData)
    {
      case 0:
        freq[channel] = BROADCAST_FREQ_0HZ;
        break;
      case 1:
        freq[channel] = BROADCAST_FREQ_1HZ;
        break;
      case 10:
        freq[channel] = BROADCAST_FREQ_10HZ;
        break;
      case 50:
        freq[channel] = BROADCAST_FREQ_50HZ;
        break;
      case 100:
        freq[channel] = BROADCAST_FREQ_100HZ;
        break;
      default:
        freq[channel] = BROADCAST_FREQ_HOLD;
        break;
    }
  }

  script->getApi()->setBroadcastFreq(freq);

  __DELETE(data);
  script->addTask(waitInput);

  return true;
}

bool acCA(Script* script, UserData data)
{
  script->getApi()->activate(&script->adata);

  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool vsCA(Script* script, UserData data)
{
  script->getApi()->getDroneVersion();

  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool ctCA(Script* script, UserData data)
{
  stringstream s;

  s << (char*)data;
  char drop[20];
  s >> drop >> drop;

  int boolean;
  s >> boolean;

  if (boolean)
    script->getApi()->setControl(true);
  else
    script->getApi()->setControl(false);

  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool syCA(Script* script, UserData data)
{
  stringstream s;

  s << (char*)data;
  char drop[20];
  s >> drop >> drop;

  int freq;
  s >> freq;

  script->getApi()->setSyncFreq(freq);

  __DELETE(data);
  script->addTask(waitInput);
  return true;
}
