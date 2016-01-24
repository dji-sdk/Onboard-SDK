#ifndef CONBOARDSDKTASK_H
#define CONBOARDSDKTASK_H

#include <DJI_Script.h>
#include "QonboardSDK.h"

using namespace DJI::onboardSDK;

class ConboardSDKScript : public Script
{
  public:
    ConboardSDKScript(CoreAPI* api);

    TaskSetItem match(const char* name);
    void addTask(const char* Name, UserData Data = 0, time_t Timeout = 0);
};

class ScriptThread : public QThread
{
  public:
    ScriptThread(ConboardSDKScript* Script = 0, QObject* parent = 0);

    void run();

  private:
    ConboardSDKScript* script;
};

#endif // CONBOARDSDKTASK_H
