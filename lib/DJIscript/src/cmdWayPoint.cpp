/*! @brief
 *  @file cmdWayPoint.cpp
 *  @version 3.0
 *  @date Jan 20, 2016
 *
 *  @abstract
 *
 *
 *  @attention
 *  Project configuration:
 *  None
 *
 *  @version features:
 *  -* @version V3.0
 *  -* DJI-onboard-SDK for Windows,QT,STM32,ROS,Cmake
 *  -* @date Jan 20, 2016
 *  -* @author william.wu
 *
 * */

#include "cmdIO.h"
#include "cmdWayPoint.h"

#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace DJI::onboardSDK;

bool WP(Script *script, UserData data)
{
    __DELETE(data);
    script->addTask(waitInput);
    return true;
}

bool initWP(Script *script, UserData data)
{
    __DELETE(data);
    script->addTask(waitInput);
    return true;
}

bool startWP(Script *script, UserData data)
{
    __DELETE(data);
    script->addTask(waitInput);
    return true;
}

bool stopWP(Script *script, UserData data)
{
    __DELETE(data);
    script->addTask(waitInput);
    return true;
}

bool pauseWP(Script *script, UserData data)
{
    __DELETE(data);
    script->addTask(waitInput);
    return true;
}

bool retartWP(Script *script, UserData data)
{
    __DELETE(data);
    script->addTask(waitInput);
    return true;
}

bool apWP(Script *script, UserData data)
{
    __DELETE(data);
    script->addTask(waitInput);
    return true;
}
