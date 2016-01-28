#include "cmdIO.h"
#include "cmdFlight.h"

#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace DJI::onboardSDK;

bool FC(Script *script, UserData data)
{
    char *inputData = (char *)data;
    char command[100];
    sscanf(inputData, "--%s", command);
    if (strcmp(command, "help") == 0)
    {
        cout << "|------------------DJI onboardSDK - Filght Control--------------|" << endl;
        cout << "| --FC <command> <data>                                         |" << endl;
        cout << "| - tk <task code> run internal programmed functions            |" << endl;
        cout << "|      <task code> 1 go home                                    |" << endl;
        cout << "|                  4 take off                                   |" << endl;
        cout << "|                  6 landing                                    |" << endl;
        cout << "| - mc <bool> stop motor or standby                             |" << endl;
        cout << "|      <bool> 0 stop                                            |" << endl;
        cout << "|             1 standby                                         |" << endl;
        cout << "|                                                               |" << endl;
        cout << "| - fl <flag> <x> <y> <z> <yaw> flight control                  |" << endl;
        cout << "|      <flag> default  VERTICAL_VELOCITY                        |" << endl;
        cout << "|             0x10     VERTICAL_POSSITION                       |" << endl;
        cout << "|             0x20     VERTICAL_THRUST                          |" << endl;
        cout << "|                                                               |" << endl;
        cout << "|             default  HORIZONTAL_ANGLE                         |" << endl;
        cout << "|             0x40     HORIZONTAL_VELOCITY                      |" << endl;
        cout << "|             0X80     HORIZONTAL_POSSITION                     |" << endl;
        cout << "|                                                               |" << endl;
        cout << "|             default  YAW_ANGLE                                |" << endl;
        cout << "|             0x08     YAW_PALSTANCE                            |" << endl;
        cout << "|                                                               |" << endl;
        cout << "|             default  HORIZONTAL_GROUND                        |" << endl;
        cout << "|             0x02     HORIZONTAL_BODY                          |" << endl;
        cout << "|                                                               |" << endl;
        cout << "|             default  YAW_GROUND (2.3 feature)                 |" << endl;
        cout << "|             0X01     YAW_BODY   (2.3 feature)                 |" << endl;
        cout << "|                                                               |" << endl;
        cout << "|             default  SMOOTH_DISABLE (3.0 feature)             |" << endl;
        cout << "|             0x01     SMOOTH_ENABLE  (3.0 feature)             |" << endl;
        cout << "|                                                               |" << endl;
        cout << "|      <x>    float data                                        |" << endl;
        cout << "|      <y>    float data                                        |" << endl;
        cout << "|      <z>    float data                                        |" << endl;
        cout << "|      <yaw>  float data                                        |" << endl;
        cout << "|                                                               |" << endl;
        cout << "|------------------DJI onboardSDK - Filght Control--------------|" << endl;

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

    s << (char *)data;
    char drop[20];
    s >> drop >> drop;
    FlightData fd;

    s >> fd.flag >> fd.x >> fd.y >> fd.z >> fd.yaw;

    script->getFlight()->setFlight(&fd);

    return true;
}
