/*! @file cmdFlight.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Flight Control Commands for DJI Onboard SDK Command line example
 *  Execute tasks like takeoff/landing/RTH. Also execute movement control
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */


#include "cmdIO.h"
#include "cmdFlight.h"

#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;

bool FC(Script *script, UserData data)
{
  char *inputData = (char *)data;
  char command[100];
  sscanf(inputData, "--%s", command);
  if (strcmp(command, "help") == 0)
  {
    cout << "|------------------DJI onboardSDK - Flight Control-----|" << endl;
    cout << "|  Usage:                                              |" << endl;
    cout << "|  --FC <command> <data>                               |" << endl;
    cout << "|  Following are valid commands                        |" << endl;
    cout << "|  - tk <task code> run internal programmed functions  |" << endl;
    cout << "|       <task code> 1 go home                          |" << endl;
    cout << "|                   4 take off                         |" << endl;
    cout << "|                   6 landing                          |" << endl;
    cout << "|  - mc <bool>  stop motor or standby                  |" << endl;
    cout << "|       <bool>  0 stop                                 |" << endl;
    cout << "|               1 standby                              |" << endl;
    cout << "|                                                      |" << endl;
    cout << "|  - fl <flag> <x> <y> <z> <yaw> flight control        |" << endl;
    cout << "|       <flag>  default  VERTICAL_VELOCITY             |" << endl;
    cout << "|               0x10     VERTICAL_POSITION             |" << endl;
    cout << "|               0x20     VERTICAL_THRUST               |" << endl;
    cout << "|                                                      |" << endl;
    cout << "|               default  HORIZONTAL_ANGLE              |" << endl;
    cout << "|               0x40     HORIZONTAL_VELOCITY           |" << endl;
    cout << "|               0X80     HORIZONTAL_POSITION           |" << endl;
    cout << "|                                                      |" << endl;
    cout << "|               default  YAW_ANGLE                     |" << endl;
    cout << "|               0x08     YAW_RATE                      |" << endl;
    cout << "|                                                      |" << endl;
    cout << "|               default  HORIZONTAL_GROUND             |" << endl;
    cout << "|               0x02     HORIZONTAL_BODY               |" << endl;
    cout << "|                                                      |" << endl;
    cout << "|               default  YAW_GROUND (2.3 feature)      |" << endl;
    cout << "|               0X01     YAW_BODY   (2.3 feature)      |" << endl;
    cout << "|                                                      |" << endl;
    cout << "|               default  SMOOTH_DISABLE (3.0+ feature) |" << endl;
    cout << "|               0x01     SMOOTH_ENABLE  (3.0+ feature) |" << endl;
    cout << "|                                                      |" << endl;
    cout << "|       <x>     float data                             |" << endl;
    cout << "|       <y>     float data                             |" << endl;
    cout << "|       <z>     float data                             |" << endl;
    cout << "|       <yaw>   float data                             |" << endl;
    cout << "|                                                      |" << endl;
    cout << "|------------------DJI onboardSDK - Flight Control-----|" << endl;

    __DELETE(data);
    script->addTask(waitInput);
  }
  else
  {
    sscanf(inputData, "--%*s%s", command);
    strcat(command, "FC");
    script->addTask((UserData)command, data);
  }
  return true;
}

bool tkFC(Script *script, UserData data)
{
  stringstream s;

  s << (char *)data;
  char drop[20];
  s >> drop >> drop;
  int flag;
  s >> flag;

  script->getFlight()->task((DJI::onboardSDK::Flight::TASK)flag);

  return true;
}

bool mcFC(Script *script, UserData data)
{
  stringstream s;

  s << (char *)data;
  char drop[20];
  s >> drop >> drop;
  int flag;
  s >> flag;

  if (flag)
    script->getFlight()->setArm(true);
  else
    script->getFlight()->setArm(false);

  return true;
}

bool flFC(Script *script, UserData data)
{
  stringstream s;
  std::string hexCmd;

  s << (char *)data;
  char drop[20];
  s >> drop >> drop;

  //Handle with double instead of float32 for manifold
  double xInput, yInput, zInput, yawInput;
  uint8_t flag;

  s >> hexCmd >> xInput >> yInput >> zInput >> yawInput;
  flag = strtoul(hexCmd.c_str(),NULL,16);

  script->getFlight()->setMovementControl(flag, xInput, yInput, zInput, yawInput);
  
  return true;
}
