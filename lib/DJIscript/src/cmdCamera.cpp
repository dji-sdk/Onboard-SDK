#include "cmdIO.h"
#include "cmdCamera.h"

#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace DJI::onboardSDK;

bool CC(Script *script, UserData data)
{
    char *inputData = (char *)data;
    char command[100];
    sscanf(inputData, "--%s", command);
    if (strcmp(command, "help") == 0)
    {
        cout << "|------------------DJI onboardSDK - Camera control--------------|" << endl;
        cout << "| --CC <command> <data>                                         |" << endl;
        cout << "| - cm <function> operat the camera                             |" << endl;
        cout << "|      <function> 32 take a picture                             |" << endl;
        cout << "|                 33 start record a video                       |" << endl;
        cout << "|                 34 stop record a video                        |" << endl;
        cout << "| - ag <yaw> <roll> <pitch> <time> move gimbal to an angle      |" << endl;
        cout << "|      input integer numbers                                    |" << endl;
        cout << "| - sg <yaw> <roll> <pitch> move gimbal in given speed          |" << endl;
        cout << "|      input integer numbers                                    |" << endl;
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

bool cmCC(Script *script, UserData data)
{
    stringstream s;

    s << (char *)data;
    char drop[20];
    s >> drop >> drop;

    uint8_t f;
    s >> f;

    script->getCamera()->setCamera((Camera::CAMERA_CODE)f);

    __DELETE(data);
    script->addTask(waitInput);
    return true;
}

bool agCC(Script *script, UserData data)
{
    stringstream s;

    s << (char *)data;
    char drop[20];
    s >> drop >> drop;

    GimbalAngleData a;
    s >> a.yaw >> a.roll >> a.pitch >> a.duration;
    a.mode = 1;

    script->getCamera()->setGimbalAngle(&a);

    __DELETE(data);
    script->addTask(waitInput);
    return true;
}

bool sgCC(Script *script, UserData data)
{
    stringstream s;

    s << (char *)data;
    char drop[20];
    s >> drop >> drop;

    GimbalSpeedData a;
    s >> a.yaw >> a.roll >> a.pitch;

    script->getCamera()->setGimbalSpeed(&a);

    __DELETE(data);
    script->addTask(waitInput);
    return true;
}
