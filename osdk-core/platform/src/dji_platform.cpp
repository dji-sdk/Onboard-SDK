/** @file dji_platform.cpp
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

#include "dji_platform.hpp"
#include <new>

using namespace DJI;
using namespace DJI::OSDK;

Platform::Platform()
{
  osalRegFlag = false;
  halUartRegFlag = false;
  loggerConsoleRegFlag = false;
}

Platform::~Platform()
{}

bool
Platform::registerHalUartHandler(const T_OsdkHalUartHandler *halUartHandler)
{
  E_OsdkStat errCode;
  errCode = OsdkPlatform_RegHalUartHandler(halUartHandler);

  if (errCode == OSDK_STAT_OK) {
    halUartRegFlag = true;
    return true;
  }else {
    halUartRegFlag = false;
    return false;
  }
}

#ifdef __linux__
bool Platform::registerHalUSBBulkHandler(const T_OsdkHalUSBBulkHandler *halUSBBulkHandler)
{
  E_OsdkStat errCode;
  errCode = OsdkPlatform_RegHalUSBBulkHandler(halUSBBulkHandler);

  if (errCode == OSDK_STAT_OK) {
    return true;
  }else {
    return false;
  }
}
#endif

bool
Platform::registerOsalHandler(const T_OsdkOsalHandler *osalHandler)
{
  E_OsdkStat errCode;
  errCode = OsdkPlatform_RegOsalHandler(osalHandler);

  if (errCode == OSDK_STAT_OK) {
    osalRegFlag = true;
    return true;
  }else {
    osalRegFlag = false;
    return false;
  }
}

bool
Platform::registerLoggerConsole(T_OsdkLoggerConsole *console)
{
  E_OsdkStat errCode;
  errCode = OsdkLogger_AddConsole(console);

    if (errCode == OSDK_STAT_OK) {
    loggerConsoleRegFlag = true;
    return true;
  }else {
    loggerConsoleRegFlag = false;
    return false;
  }
}

bool
Platform::isOsalReady()
{
  return osalRegFlag;
}
 
bool
Platform::isHalUartReady()
{
  return halUartRegFlag;
}

bool
Platform::isLoggerReady()
{
  return loggerConsoleRegFlag;
}

bool
Platform::taskCreate(T_OsdkTaskHandle *task, void *(*taskFunc)(void *), uint32_t stackSize, void *arg)
{
  E_OsdkStat errCode;
  errCode = OsdkOsal_TaskCreate(task, taskFunc, stackSize, arg);

  return (errCode == OSDK_STAT_OK)? true : false;
}

bool
Platform::taskDestroy(T_OsdkTaskHandle task)
{
  E_OsdkStat errCode;
  errCode = OsdkOsal_TaskDestroy(task);

  return (errCode == OSDK_STAT_OK)? true : false;
}

bool
Platform::taskSleepMs(uint32_t timeMs)
{
  E_OsdkStat errCode;
  errCode = OsdkOsal_TaskSleepMs(timeMs);

  return (errCode == OSDK_STAT_OK)? true : false;
}

bool
Platform::mutexCreate(T_OsdkMutexHandle *mutex)
{
  E_OsdkStat errCode;
  errCode = OsdkOsal_MutexCreate(mutex);

  return (errCode == OSDK_STAT_OK)? true : false;
}

bool
Platform::mutexDestroy(T_OsdkMutexHandle mutex)
{
  E_OsdkStat errCode;
  errCode = OsdkOsal_MutexDestroy(mutex);

  return (errCode == OSDK_STAT_OK)? true : false;
}

bool
Platform::mutexLock(T_OsdkMutexHandle mutex)
{
  E_OsdkStat errCode;
  errCode = OsdkOsal_MutexLock(mutex);

  return (errCode == OSDK_STAT_OK)? true : false;
}

bool
Platform::mutexUnlock(T_OsdkMutexHandle mutex)
{
  E_OsdkStat errCode;
  errCode = OsdkOsal_MutexUnlock(mutex);

  return (errCode == OSDK_STAT_OK)? true : false;
}


Mutex::Mutex()
{
  OsdkOsal_MutexCreate(&mutex);
}

Mutex::~Mutex()
{
  OsdkOsal_MutexDestroy(mutex);
}

bool
Mutex::lock()
{
  E_OsdkStat errCode;
  errCode = OsdkOsal_MutexLock(mutex);

  return (errCode == OSDK_STAT_OK)? true : false;
}

bool
Mutex::unlock()
{
  E_OsdkStat errCode;
  errCode = OsdkOsal_MutexUnlock(mutex);

  return (errCode == OSDK_STAT_OK)? true : false;
}

bool
Platform::semaphoreCreate(T_OsdkSemHandle *semaphore, uint32_t initValue)
{
  E_OsdkStat errCode;
  errCode = OsdkOsal_SemaphoreCreate(semaphore, initValue);

  return (errCode == OSDK_STAT_OK)? true : false;
}

bool
Platform::semaphoreDestroy(T_OsdkSemHandle semaphore)
{
  E_OsdkStat errCode;
  errCode = OsdkOsal_SemaphoreDestroy(semaphore);

  return (errCode == OSDK_STAT_OK)? true : false;
}

bool
Platform::semaphorePost(T_OsdkSemHandle semaphore)
{
  E_OsdkStat errCode;
  errCode = OsdkOsal_SemaphorePost(semaphore);

  return (errCode == OSDK_STAT_OK)? true : false;
}

bool
Platform::semaphoreWait(T_OsdkSemHandle semaphore)
{
  E_OsdkStat errCode;
  errCode = OsdkOsal_SemaphoreWait(semaphore);

  return (errCode == OSDK_STAT_OK)? true : false;
}

bool
Platform::semaphoreTimedWait(T_OsdkSemHandle semaphore, uint32_t waitTime)
{
  E_OsdkStat errCode;
  errCode = OsdkOsal_SemaphoreTimedWait(semaphore, waitTime);

  return (errCode == OSDK_STAT_OK)? true : false;
}

bool
Platform::getTimeMs(uint32_t *ms)
{
  E_OsdkStat errCode;
  errCode = OsdkOsal_GetTimeMs(ms);

  return (errCode == OSDK_STAT_OK)? true : false;
}

#ifdef OS_DEBUG
bool
Platform::getTimeUs(uint64_t *us)
{
  E_OsdkStat errCode;
  errCode = OsdkOsal_GetTimeUs(us);

  return (errCode == OSDK_STAT_OK)? true : false;
}
#endif

void*
Platform::malloc(uint32_t size)
{
  return OsdkOsal_Malloc(size);
}

void
Platform::free(void *ptr)
{
  return OsdkOsal_Free(ptr);
}

