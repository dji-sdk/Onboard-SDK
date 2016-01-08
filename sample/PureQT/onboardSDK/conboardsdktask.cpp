#include "conboardsdktask.h"
#include <iostream>
#include <sstream>
#include <fstream>

#include <cmdIO.h>
#include <cmdSettings.h>
#include <cmdCoreAPI.h>
#include <cmdFlight.h>

using namespace std;

TaskSetItem cmdTaskSet[] = {
    TASK_ITEM(addTask),   //
    TASK_ITEM(waitInput), //
    TASK_ITEM(help),	  //

    TASK_ITEM(SS),	 //
    TASK_ITEM(idSS),   //
    TASK_ITEM(keySS),  //
    TASK_ITEM(saveSS), //
    TASK_ITEM(loadSS), //
    TASK_ITEM(spSS),   //

    TASK_ITEM(CA),   //
    TASK_ITEM(acCA), //
    TASK_ITEM(vsCA), //
    TASK_ITEM(bfCA), //
    TASK_ITEM(bdCA), //
    TASK_ITEM(ctCA), //
    TASK_ITEM(syCA), //

    TASK_ITEM(FC),   //
    TASK_ITEM(tkFC), //
    TASK_ITEM(mcFC), //
    TASK_ITEM(flFC), //

};

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
