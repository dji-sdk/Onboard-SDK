#ifndef CONBOARDSDKTASK_H
#define CONBOARDSDKTASK_H

#include <DJI_Script.h>

#include <cmdIO.h>
#include <cmdSettings.h>
#include <cmdCoreAPI.h>
#include <cmdFlight.h>
#include <cmdFollow.h>
#include <cmdHotPoint.h>
#include <cmdVirtualRC.h>
#include <cmdWayPoint.h>
#include <cmdCamera.h>

using namespace DJI::onboardSDK;

class ConboardSDKScript : public Script
{
  public:
    ConboardSDKScript(CoreAPI* api);

    TaskSetItem match(const char* name);
    void addTask(const char* Name, UserData Data = 0, time_t Timeout = 0);
};

class ScriptThread
{
  public:
    ScriptThread(ConboardSDKScript* Script = 0);

    void run();

  private:
    ConboardSDKScript* script;
};

#endif // CONBOARDSDKTASK_H
