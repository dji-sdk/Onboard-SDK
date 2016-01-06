#include "conboardsdktask.h"
#include <iostream>
#include <sstream>
#include <fstream>

#include <cmdIO.h>
#include <cmdCoreAPI.h>

using namespace std;

bool SS(Script* script, UserData data);
bool loadSS(Script* script, UserData data);

bool SS(Script* script, UserData data)
{
    char* inputData = (char*)data;
    char command[100];
    sscanf(inputData, "--%s", command);
    if (strcmp(command, "help") == 0)
    {
        cout << "|------------------DJI onboardSDK - Script Settings-------------|" << endl;
        cout << "| --SS <command> <data>                                         |" << endl;
        cout << "| - id <id> set activate ID                                     |" << endl;
        cout << "| - key <key> set activate encript key                          |" << endl;
        cout << "| - save <N/A> save id and key                                  |" << endl;
        cout << "| - load <file name> load and print id and key                  |" << endl;
        cout << "| - sp <number> select serial port                              |" << endl;
        cout << "|      - ls print serial ports list                             |" << endl;
        cout << "|      - auto select automaticaly                               |" << endl;
        cout << "|------------------DJI onboardSDK - Script Settings-------------|" << endl;

        script->addTask(waitInput);
        script->addTask(waitInput);
    }
    else
    {
        if (sscanf(inputData, "--%*s%s", command))
        {
            strcat(command, "SS");
            script->addTask((UserData)command, data);
        }
        else
            script->addTask(addTask);
    }
    return true;
}

bool loadSS(Script* script, UserData data)
{
    char* inputData = (char*)data;
    char command[100];
    char line[1024];
    static char key[50];
    if (sscanf(inputData, "--%*s%*s%s", command))
        cout << "Path: " << command;
    else
        memcpy(command, "settings.ini", 13);
    ifstream read(command);

    while (!read.eof())
    {
        read.getline(line, 1024);
        if (*line != 0) //! @note sscanf have features on empty buffer.
        {
            int id;
            if (sscanf(line, "ID:%d", &id))
                script->adata.app_id = id;
            script->adata.app_api_level = 2;
            script->adata.app_ver = SDK_VERSION;
            if (sscanf(line, "KEY:%s", key))
                script->adata.app_key = key;
        }
    }
    read.close();

    __DELETE(data);
    script->addTask(waitInput);
    return true;
}

TaskSetItem cmdTaskSet[] = { TASK_ITEM(addTask),   //
                             TASK_ITEM(waitInput), //
                             TASK_ITEM(help),	  //
                             TASK_ITEM(CA),		   //
                             TASK_ITEM(acCA),	  //
                             TASK_ITEM(vsCA),	  //
                             TASK_ITEM(bfCA),	  //
                             TASK_ITEM(SS),		   //
                             TASK_ITEM(loadSS) };

ConboardSDKScript::ConboardSDKScript(CoreAPI* api)
    : Script(api, cmdTaskSet, sizeof(cmdTaskSet) / sizeof(TaskSetItem))
{
}

Task ConboardSDKScript::match(const char* name) { return Script::match((UserData)name); }

void ConboardSDKScript::addTask(const char* Name, UserData Data, time_t Timeout)
{
    Script::addTask((UserData)Name, Data, Timeout);
}

ScriptThread::ScriptThread(ConboardSDKScript* Script, QObject* parent) : QThread(parent)
{
    script = Script;
}

void ScriptThread::run()
{
    while (1)
    {
        script->addTask("addTask");
        script->run();
    }
}
