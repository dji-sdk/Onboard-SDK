/** @file dji_platform.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Vehicle API for DJI onboardSDK library
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

#ifndef OSDK_DJI_PLATFORM_H_
#define OSDK_DJI_PLATFORM_H_

#include "osdk_platform.h"
#include "osdk_logger.h"

namespace DJI
{
namespace OSDK
{

class Platform
{
public:
  Platform();
  ~Platform();

  bool isOsalReady();
 
  bool isHalUartReady();

  bool isHalUdpReady();

  bool isLoggerReady();

  bool registerHalUartHandler(const T_OsdkHalUartHandler *halUartHandler);

  bool registerHalUdpHandler(const T_OsdkHalUdpHandler *halUdpHandler);

  bool registerOsalHandler(const T_OsdkOsalHandler *osalHandler);

  bool registerLoggerConsole(T_OsdkLoggerConsole *console);

  bool taskCreate(T_OsdkTaskHandle *task, void *(*taskFunc)(void *), uint32_t stackSize, void *arg);

  bool taskDestroy(T_OsdkTaskHandle task);

  bool taskSleepMs(uint32_t timeMs);

  bool mutexCreate(T_OsdkMutexHandle *mutex);

  bool mutexDestroy(T_OsdkMutexHandle mutex);

  bool mutexLock(T_OsdkMutexHandle mutex);

  bool mutexUnlock(T_OsdkMutexHandle mutex);

  bool semaphoreCreate(T_OsdkSemHandle *semaphore, uint32_t initValue);

  bool semaphoreDestroy(T_OsdkSemHandle semaphore);

  bool semaphorePost(T_OsdkSemHandle semaphore);

  bool semaphoreWait(T_OsdkSemHandle semaphore);

  bool semaphoreTimedWait(T_OsdkSemHandle semaphore, uint32_t waitTime);

  bool getTimeMs(uint32_t *ms);

  #ifdef OS_DEBUG
  bool getTimeUs(uint64_t *us);
  #endif

  void* malloc(uint32_t size);

  void free(void *ptr);

private:
  bool osalRegFlag;
  bool halUartRegFlag;
  bool halUdpRegFlag;
  bool loggerConsoleRegFlag;

};
}
}

class Mutex
{
public:
  Mutex();
  ~Mutex();
  
  bool lock();

  bool unlock();

private:
  T_OsdkMutexHandle mutex;
};


#endif
