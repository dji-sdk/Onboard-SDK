/*! @file utility_thread.cpp
 *  @version 3.4
 *  @date Oct 10 2017
 *
 *  @brief
 *  Pthread-based threading for DJI Onboard SDK on linux platforms
 *
 *  @copyright
 *  2016-17 DJI. All rights reserved.
 */

#include "utility_thread.hpp"

UtilityThread::UtilityThread()
{
  this->imageContainer = 0;
  this->type           = 0;
  this->stop_condition = false;
}

UtilityThread::UtilityThread(ImageProcessContainer* imgContainer_ptr, int type)
  : imageContainer(imgContainer_ptr)
  , type(type)
{
  this->stop_condition = false;
}

bool
UtilityThread::createThread()
{
  int         ret = -1;

  /* Initialize and set thread detached attribute */
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

  if (type == 1)
  {
    ret     = pthread_create(&threadID, NULL, img_process_call, (void *) this->imageContainer);
    this->thread_name = "ImgProcessPoll";
  }


  if (0 != ret)
  {
    DERROR("fail to create thread for %s!\n", this->thread_name.c_str());
    return false;
  }
  else
  {
    DSTATUS("a thread named %s is created.\n", this->thread_name.c_str());
  }

  ret = pthread_setname_np(threadID, this->thread_name.c_str());
  if (0 != ret)
  {
    DERROR("fail to set thread name for %s!\n", this->thread_name.c_str());
    return false;
  }
  return true;
}

int
UtilityThread::stopThread()
{
  int   ret = -1;
  void* status;
  this->stop_condition = true;

  /* Free attribute and wait for the other threads */
  if (int i = pthread_attr_destroy(&attr))
  {
    DERROR("fail to destroy thread %d\n", i);
  }
  else
  {
    DDEBUG("success to distory thread\n");
  }
  ret = pthread_join(threadID, &status);

  DDEBUG("Main: completed join with thread code: %d\n", ret);
  if (ret)
  {
    // Return error code
    return ret;
  }

  return 0;
}

bool
UtilityThread::getStopCondition()
{
  return this->stop_condition;
}

std::string
UtilityThread::getThreadName()
{
  return this->thread_name;
}

void*
UtilityThread::img_process_call(void *param)
{
  ImageProcessContainer* img_container_ptr = (ImageProcessContainer*)param;

  while (!img_container_ptr->getImgProcessThread()->getStopCondition())
  {
    img_container_ptr->newDataPoll();

    usleep(100);
  }

  DDEBUG("Quit image process function\n");
}
