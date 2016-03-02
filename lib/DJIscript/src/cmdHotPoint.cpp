/*! @brief
 *  @file cmdHotPoint.cpp
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
#include "cmdHotPoint.h"

#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace DJI::onboardSDK;

bool HP(Script *script, UserData data)
{
    char *inputData = (char *)data;
    char command[100];
    sscanf(inputData, "--%s", command);
    if (strcmp(command, "help") == 0)
    {
        cout << "|------------------DJI onboardSDK - Hot Point mission-----------|" << endl;
        cout << "| --HP <command> <data>                                         |" << endl;
        cout << "| - start <latitude> <longitude> <height> <radius>              |" << endl;
        cout << "|      <latitude>  type double, unit 1 rad                      |" << endl;
        cout << "|      <longitude> type double, unit 1 rad                      |" << endl;
        cout << "|      <height>    type double, unit 1 meter                    |" << endl;
        cout << "|      <radius>    type float,  unit 1 meter                    |" << endl;
        cout << "| - stop <nodata> stop hot point                                |" << endl;
        cout << "| - pause <nodata> pause hot point                              |" << endl;
        cout << "| - restart <nodata> restart hot point                          |" << endl;
        cout << "|------------------DJI onboardSDK - Script Settings-------------|" << endl;

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

    float32_t radius;
    s >> fdata.latitude >> fdata.longitude >> fdata.height >> radius;

    if (radius > 0)
    {
        fdata.radius = radius;
        fdata.clockwise = 0;
    }
    else
    {
        fdata.radius = -radius;
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
