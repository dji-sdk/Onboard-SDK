
#include "cmdIO.h"
#include "cmdVirtualRC.h"

#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace DJI::onboardSDK;

bool VC(Script *script, UserData data)
{
    char* inputData = (char*)data;
    char command[100];
    sscanf(inputData, "--%s", command);
    if (strcmp(command, "help") == 0)
    {
        cout << "|--------------DJI onboardSDK - Virtual Remote control----------|" << endl;
        cout << "| --VC <command> <data>                                         |" << endl;
        cout << "| - start {<channel> <value>} start follow mission              |" << endl;
        cout << "|          <channel> 0 roll                                     |" << endl;
        cout << "|                    1 pitch                                    |" << endl;
        cout << "|                    2 throttle                                 |" << endl;
        cout << "|                    3 yaw                                      |" << endl;
        cout << "|                    4 gear                                     |" << endl;
        cout << "|                    6 mode                                     |" << endl;
        cout << "|          <value>   364 to 1684, middle is 1024                |" << endl;
        cout << "| - stop <nodata> stop follow mission                           |" << endl;
        cout << "|------------------DJI onboardSDK - Script Settings-------------|" << endl;

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
    script->addTask(ctVC,data,-1,0);
    return true;
}

bool stopVC(Script *script, UserData data)
{
    script->getVirtualRC()->setControl(false,VirtualRC::CutOff_ToRealRC);
    __DELETE(data);
    script->addTask(waitInput);
    return true;
}

bool ctVC(Script *script, UserData data)
{
    //__DELETE(data);

    script->addTask(waitInput);
    return true;
}
