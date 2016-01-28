#include "cmdIO.h"

#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace DJI::onboardSDK;

bool addTask(Script* script, UserData data __UNUSED)
{
    cout << endl;
    cout << "|------------------DJI onboardSDK command line------------------|" << endl;
    cout << "| - available modules                                           |" << endl;
    cout << "| - < CA > core API module                                      |" << endl;
    cout << "| - < FC > flight control module                                |" << endl;
    cout << "| - < CC > camera control module                                |" << endl;
    cout << "| - < VC > virtual remote control module                        |" << endl;
    cout << "| - < FM > follow mission module                                |" << endl;
    cout << "| - < WP > waypoint mission module                              |" << endl;
    cout << "| - < HP > hotpoint mission module                              |" << endl;
    cout << "| - < SS > script settings module                               |" << endl;
    cout << "|                                                               |" << endl;
    cout << "| Command examples:                                             |" << endl;
    cout << "| --help <module>                                               |" << endl;
    cout << "|   e.g. --help CA                                              |" << endl;
    cout << "| --<module> <command> <data>                                   |" << endl;
    cout << "|   e.g. --CA ac                                                |" << endl;
    cout << "|                                                               |" << endl;
    cout << "| {<data>} means you can repeat the data like:                  |" << endl;
    cout << "|          --<module> <command> <data1> <data2>                 |" << endl;
    cout << "|          is equal to:                                         |" << endl;
    cout << "|          --<module> <command> <data1>                         |" << endl;
    cout << "|          --<module> <command> <data2>                         |" << endl;
    cout << "|                                                               |" << endl;
    cout << "| Attention! this sample is not a full functional version,      |" << endl;
    cout << "| Quick start only.                                             |" << endl;
    cout << "|------------------DJI onboardSDK command line------------------|" << endl;

    script->addTask(waitInput);

    return true;
}

bool waitInput(Script* script, UserData data __UNUSED)
{
    char* inputCmd = new char[512];
    cout << "Waiting input" << endl;
    cin.getline(inputCmd, 512);

    char command[100];
    sscanf(inputCmd, "--%s", command);
    script->addTask((UserData)command, inputCmd);

    return true;
}

bool help(Script* script, UserData data)
{
    char* inputData = (char*)data;
    char command[100];
    if (sscanf(inputData, "--%*s%s", command))
        script->addTask((UserData)command, inputData);
    else
        script->addTask(addTask);
    return true;
}

