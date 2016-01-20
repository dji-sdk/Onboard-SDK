#include "conboardsdktask.h"
#include <iostream>
#include <sstream>
#include <fstream>

#include <cmdIO.h>
#include <cmdSettings.h>
#include <cmdCoreAPI.h>
#include <cmdFlight.h>
#include <cmdFollow.h>
#include <cmdHotPoint.h>
#include <cmdWayPoint.h>
#include <cmdVirtualRC.h>
#include <cmdCamera.h>

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

    TASK_ITEM(FM),		  //
    TASK_ITEM(startFM),   //
    TASK_ITEM(stopFM),	//
    TASK_ITEM(pauseFM),   //
    TASK_ITEM(restartFM), //
    TASK_ITEM(updateFM),  //

    TASK_ITEM(HP),		  //
    TASK_ITEM(startHP),   //
    TASK_ITEM(stopHP),	//
    TASK_ITEM(pauseHP),   //
    TASK_ITEM(restartHP), //

    TASK_ITEM(VC),		//
    TASK_ITEM(startVC), //
    TASK_ITEM(stopVC),  //
    TASK_ITEM(ctVC),	//

    TASK_ITEM(WP),		 //
    TASK_ITEM(initWP),   //
    TASK_ITEM(startWP),  //
    TASK_ITEM(stopWP),   //
    TASK_ITEM(pauseWP),  //
    TASK_ITEM(retartWP), //
    TASK_ITEM(apWP),	 //

    TASK_ITEM(CC),   //
    TASK_ITEM(cmCC), //
    TASK_ITEM(agCC), //
    TASK_ITEM(sgCC), //
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
