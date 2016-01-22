
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
        cout << "|--------------DJI onboardSDK - Virtual Remote contorl----------|" << endl;
        cout << "| --VC <command> <data>                                         |" << endl;
        cout << "| - start <nodata> start follow mission                         |" << endl;
        cout << "| - stop <nodata> stop follow mission                           |" << endl;
        cout << "| - ct <nodata> pause follow mission                            |" << endl;
        cout << "|------------------DJI onboardSDK - Script Settings-------------|" << endl;

        script->addTask(waitInput);
        script->addTask(waitInput);
    }
    else
    {
        if (sscanf(inputData, "--%*s%s", command))
        {
            strcat(command, "FM");
            script->addTask((UserData)command, data);
        }
        else
            script->addTask(addTask);
    }
    return true;
}

bool startVC(Script *script, UserData data)
{
    __DELETE(data);
    script->addTask(waitInput);
    return true;
}

bool stopVC(Script *script, UserData data)
{
    __DELETE(data);
    script->addTask(waitInput);
    return true;
}

bool ctVC(Script *script, UserData data)
{
    __DELETE(data);
    script->addTask(waitInput);
    return true;
}
