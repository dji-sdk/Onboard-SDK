/*! @file APIThread.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Threading for DJI Onboard SDK command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "APIThread.h"

APIThread::APIThread()
{
  api = 0;
  type = 0;
}

APIThread::APIThread(CoreAPI *API, int Type)
{
  api = API;
  type = Type;
  api->stopCond = false;
}

bool APIThread::createThread()
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
  return true;
}

int APIThread::stop()
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

void *APIThread::send_call(void *param)
{
  while(!((CoreAPI*)param)->stopCond)
  {
      ((CoreAPI*)param)->sendPoll();
  }
}

void *APIThread::read_call(void *param)
{
  while(!((CoreAPI*)param)->stopCond)
  {
    ((CoreAPI*)param)->readPoll();
  }
}
