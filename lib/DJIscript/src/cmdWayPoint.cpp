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
    char *inputData = (char *)data;
    char command[100];
    sscanf(inputData, "--%s", command);
    if (strcmp(command, "help") == 0)
    {
        cout << "|------------------DJI onboardSDK - Waypoint--------------------|" << endl;
        cout << "| --WP <command> <data>                                         |" << endl;
        cout << "| - init <indexNumber>                                          |" << endl;
        cout << "| - start <N/A>                                                 |" << endl;
        cout << "| - stop  <N/A>                                                 |" << endl;
        cout << "|         input integer numbers                                    |" << endl;
        cout << "| - pause <yaw> <roll> <pitch> <time> move gimbal to an angle      |" << endl;
        cout << "| - restart <N/A>                                               |" << endl;
        cout << "| - ap <index> <latitude> <longitude> <altitude>                |" << endl;
        cout << "|------------------DJI onboardSDK - Camera control--------------|" << endl;

        __DELETE(data);
        script->addTask(waitInput);
    }
    else
    {
        sscanf(inputData, "--%*s%s", command);
        strcat(command, "CC");
        script->addTask((UserData)command, data);
    }
    return true;
}

bool initWP(Script *script, UserData data)
{
    stringstream s;

    WayPointInitData fdata;

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
    s >> fdata.indexNumber;

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
    s >> fdata.index >> fdata.latitude >> fdata.longitude >> fdata.altitude;

    script->getWaypoint()->uploadIndexData(&fdata);
    __DELETE(data);
    script->addTask(waitInput);
    return true;
}
