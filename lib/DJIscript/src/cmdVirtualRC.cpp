
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
    __DELETE(data);
    script->addTask(waitInput);
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
