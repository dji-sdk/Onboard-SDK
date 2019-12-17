/*! @file utility_thread.cpp
 *  @version 3.4
 *  @date Oct 10 2017
 *
 *  @brief
 *  Pthread-based threading for DJI Onboard SDK on linux platforms
 *
 *  @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
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