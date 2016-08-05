/*! @file conboardsdktask.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Task abstraction implementation for DJI Script for DJI Onboard SDK command line sample.
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

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

using namespace DJI;
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
