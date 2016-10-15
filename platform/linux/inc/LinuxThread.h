/*! @file LinuxThread.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Pthread-based threading for DJI Onboard SDK Linux example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef LINUXTHREAD_H
#define LINUXTHREAD_H

#include <DJI_HardDriver.h>
#include <DJI_Camera.h>
#include <DJI_Flight.h>
#include <DJI_HotPoint.h>
#include <DJI_Follow.h>
#include <DJI_WayPoint.h>
#include <DJI_VirtualRC.h>
#include <DJI_API.h>
#include <pthread.h>

#include <string>

using namespace DJI::onboardSDK;

class LinuxThread
{
  public:
    LinuxThread();
    LinuxThread(CoreAPI *api, int type);

    bool createThread();
    int stopThread();

  private:
    CoreAPI *api;
    int type;
    pthread_t threadID;
    pthread_attr_t attr;

    static void *send_call(void *param);
    static void *read_call(void *param);
    static void *callback_call(void *param);

};

#endif // LINUXTHREAD_H
