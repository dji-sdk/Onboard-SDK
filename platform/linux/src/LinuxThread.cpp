/*! @file LinuxThread.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Pthread-based threading for DJI Onboard SDK linux example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

//! To Implement

 
#include "LinuxThread.h"

LinuxThread::LinuxThread()
{
    api = 0;
    type = 0;
}

LinuxThread::LinuxThread(CoreAPI *API, int Type)
{
  api = API;
  type = Type;
  api->stopCond = false;

}

bool LinuxThread::createThread()
{
  int ret = -1;
  std::string infoStr;

  /* Initialize and set thread detached attribute */
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

  if(1 == type)
  {
    ret = pthread_create(&threadID, NULL, send_call, (void*)api);
    infoStr = "sendPoll";
  }
  else if(2 == type)
  {
    ret = pthread_create(&threadID, NULL, read_call, (void*)api);
    infoStr = "readPoll";
  }

  else if (3==type)
  {
    ret = pthread_create(&threadID, NULL, callback_call, (void*)api);
    infoStr = "callback";
  }
  else
  {
    infoStr = "error type number";
  }

  if(0 != ret)
  {
    API_LOG(api->getDriver(), ERROR_LOG, "fail to create thread for %s!\n",
    infoStr.c_str());
    return false;
  }

  ret = pthread_setname_np(threadID, infoStr.c_str());
  if(0 != ret)
  {
    API_LOG(api->getDriver(), ERROR_LOG, "fail to set thread name for %s!\n",
            infoStr.c_str());
    return false;
  }
  return true;
}

int LinuxThread::stopThread()
{
  int ret = -1;
  void *status;
  api->stopCond = true;

  /* Free attribute and wait for the other threads */
  pthread_attr_destroy(&attr);

  ret = pthread_join(threadID, &status);

  if(ret)
  {
    // Return error code
    return ret;
  }

  API_LOG(api->getDriver(), DEBUG_LOG, "Main: completed join with thread\n");
  return 0;
}

void *LinuxThread::send_call(void *param)
{
  while(true)
  {
    ((CoreAPI*)param)->sendPoll();
  }
}

void *LinuxThread::read_call(void *param)
{
  while(!((CoreAPI*)param)->stopCond)
  {
    ((CoreAPI*)param)->readPoll();
  }
}

void *LinuxThread::callback_call(void *param)
{
  while(true)
  {
    ((CoreAPI*)param)->callbackPoll((CoreAPI*)param);
  }
}
