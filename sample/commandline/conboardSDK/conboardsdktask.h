#ifndef CONBOARDSDKTASK_H
#define CONBOARDSDKTASK_H

#include <DJI_Script.h>
#include <thread>
#include "QonboardSDK.h"

using namespace DJI::onboardSDK;

class ConboardSDKScript : public Script
{
  public:
    ConboardSDKScript(CoreAPI* api);

    Task match(const char* name);
    void addTask(char* Name, UserData Data = 0, time_t Timeout = 0);
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
