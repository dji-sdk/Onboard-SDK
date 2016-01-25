#ifndef CONBOARDSDK_H
#define CONBOARDSDK_H

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

class APIThread
{
  public:
    APIThread();
    APIThread(CoreAPI *API, int Type);

    bool createThread();

  private:
    CoreAPI *api;
    int type;

    static void *send_call(void *param);
    static void *read_call(void *param);
};

#endif // CONBOARDSDK_H
