#include "conboardsdktask.h"
#include <iostream>

using namespace std;
// using namespace DJI::onboardSDK;

bool addTask(Script* script, UserData data __UNUSED);
bool waitInput(Script* script __UNUSED, UserData data __UNUSED);
bool help(Script* script __UNUSED, UserData data __UNUSED);
bool CAhelp(Script* script __UNUSED, UserData data __UNUSED);
bool SShelp(Script* script __UNUSED, UserData data __UNUSED);

bool addTask(Script* script, UserData data __UNUSED)
{
    cout << endl;
    cout << "|----------------------DJI onboardSDK command line----------------------|" << endl;
    cout << "| --help <module>                                                       |" << endl;
    cout << "| - CA core API module                                                  |" << endl;
    cout << "| - FC flight control module                                            |" << endl;
    cout << "| - CC camera control module                                            |" << endl;
    cout << "| - VC virtual remote control module                                    |" << endl;
    cout << "| - FM follow mission module                                            |" << endl;
    cout << "| - WP waypoint mission module                                          |" << endl;
    cout << "| - HP hotpoint mission module                                          |" << endl;
    cout << "| - SS script settings module                                           |" << endl;
    cout << "|                                                                       |" << endl;
    cout << "| --<module> <command> <data>                                           |" << endl;
    cout << "|----------------------DJI onboardSDK command line----------------------|" << endl;

    script->addTask(waitInput);

    return true;
}

bool waitInput(Script* script __UNUSED, UserData data __UNUSED)
{
    char inputCmd[512];
    cout << "Waiting input" << endl;
    cin >> inputCmd;

    script->addTask((UserData)inputCmd);

    return true;
}

bool help(Script* script __UNUSED, UserData data __UNUSED)
{
    char inputData[512];
    cin >> inputData;
    strcat(inputData, "help");
    script->addTask((UserData)inputData);
    return true;
}

bool SShelp(Script* script __UNUSED, UserData data __UNUSED)
{
    cout << "|----------------------DJI onboardSDK - Script Settings-----------------|" << endl;
    cout << "| --SS <command> <data>                                                 |" << endl;
    cout << "| - id <id> set activate ID                                             |" << endl;
    cout << "| - key <key> set activate encript key                                  |" << endl;
    cout << "| - save <N/A> save id and key                                          |" << endl;
    cout << "| - load <file name> load and print id and key                          |" << endl;
    cout << "|----------------------DJI onboardSDK - Script Settings-----------------|" << endl;

    script->addTask(waitInput);
    return true;
}

bool CAhelp(Script* script __UNUSED, UserData data __UNUSED)
{
    cout << "|----------------------DJI onboardSDK - Core API------------------------|" << endl;
    cout << "| --CA <command> <data>                                                 |" << endl;
    cout << "| - ac <N/A> activate                                                   |" << endl;
    cout << "|----------------------DJI onboardSDK - Core API------------------------|" << endl;

    script->addTask(waitInput);
    return true;
}

TaskSetItem cmdTaskSet[] = { TASK_ITEM(addTask),   //
                             TASK_ITEM(waitInput), //
                             CMD_ITEM(help),	   //
                             TASK_ITEM(CAhelp),	//
                             TASK_ITEM(SShelp) };

ConboardSDKScript::ConboardSDKScript(CoreAPI* api)
    : Script(api, cmdTaskSet, sizeof(cmdTaskSet) / sizeof(TaskSetItem))
{
}

Task ConboardSDKScript::match(const char* name) { return Script::match((UserData)name); }

void ConboardSDKScript::addTask(char* Name, UserData Data, time_t Timeout)
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
