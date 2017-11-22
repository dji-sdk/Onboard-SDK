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

#ifndef POSIX_THREAD_HPP
#define POSIX_THREAD_HPP

#include <pthread.h>
#include <string>
#include "image_process_container.hpp"

class ImageProcessContainer;

class UtilityThread
{
public:

  UtilityThread();
  UtilityThread(ImageProcessContainer* imgContainer_ptr, int type);
  ~UtilityThread(){}

  bool createThread();
  int  stopThread();

  std::string getThreadName();

private:
  pthread_t      threadID;
  pthread_attr_t attr;
  int type;
  bool stop_condition;
  std::string thread_name;

  ImageProcessContainer*imageContainer;

private:
  bool getStopCondition();
  static void* img_process_call(void* param);

};

#endif // POSIX_THREAD_HPP