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
#include "dji_singleton.hpp"
#include "osdk_logger.h"

#define DJI_REG_UART_HANDLER(handlerPtr)                            \
  DJI::OSDK::Platform::instance()                                   \
  .registerHalUartHandler(handlerPtr)

#ifdef __linux__
#define DJI_REG_USB_BULK_HANDLER(handlerPtr)                        \
  DJI::OSDK::Platform::instance()                                   \
  .registerHalUSBBulkHandler(handlerPtr)
#endif

#define DJI_REG_OSAL_HANDLER(handlerPtr)                            \
  DJI::OSDK::Platform::instance()                                   \
  .registerOsalHandler(handlerPtr)

#define DJI_REG_LOGGER_CONSOLE(consolePtr)                          \
  DJI::OSDK::Platform::instance()                                   \
  .registerLoggerConsole(consolePtr)

#define DJI_TASK_CREATE(taskPtr, taskFunc, stackSize, arg)          \
  DJI::OSDK::Platform::instance()                                   \
  .taskCreate(taskPtr, taskFunc, stackSize, arg)

#define DJI_TASK_DESTROY(task)                                      \
  DJI::OSDK::Platform::instance()                                   \
  .taskDestroy(task)

#define DJI_TASK_SLEEP_MS(timeMs)                                   \
  DJI::OSDK::Platform::instance()                                   \
  .taskSleepMs(timeMs)

#define DJI_MUTEX_CREATE(mutexPtr)                                  \
  DJI::OSDK::Platform::instance()                                   \
  .mutexCreate(mutexPtr)

#define DJI_MUTEX_DESTROY(mutex)                                    \
  DJI::OSDK::Platform::instance()                                   \
  .mutexDestroy(mutex)

#define DJI_MUTEX_LOCK(mutex)                                       \
  DJI::OSDK::Platform::instance()                                   \
  .mutexLock(mutex)

#define DJI_MUTEX_UNLOCK(mutex)                                     \
  DJI::OSDK::Platform::instance()                                   \
  .mutexUnlock(mutex)

#define DJI_SEM_CREATE(semPtr, initValue)                           \
  DJI::OSDK::Platform::instance()                                   \
  .semaphoreCreate(mutexPtr, initValue)

#define DJI_SEM_DESTROY(sem)                                        \
  DJI::OSDK::Platform::instance()                                   \
  .semaphoreDestroy(sem)

#define DJI_SEM_POST(sem)                                           \
  DJI::OSDK::Platform::instance()                                   \
  .semaphorePost(sem)

#define DJI_SEM_WAIT(sem)                                           \
  DJI::OSDK::Platform::instance()                                   \
  .semaphoreWait(sem)

#define DJI_SEM_TIMED_WAIT(sem, waitTime)                           \
  DJI::OSDK::Platform::instance()                                   \
  .semaphoreTimedWait(sem, waitTime)

#define DJI_MEM_MALLOC(size)                                        \
  DJI::OSDK::Platform::instance()                                   \
  .malloc(size)

#define DJI_MEM_FREE(ptr)                                           \
  DJI::OSDK::Platform::instance()                                   \
  .free(ptr)

#define DJI_GET_TIME_MS(msPtr)                                      \
  DJI::OSDK::Platform::instance()                                   \
  .getTimeMs(msPtr)

#ifdef OS_DEBUG
#define DJI_GET_TIME_US(usPtr)                                      \
  DJI::OSDK::Platform::instance()                                   \
  .getTimeUs(usPtr)
#endif

namespace DJI
{
namespace OSDK
{

class Platform : public Singleton<Platform>
{
public:
  Platform();
  ~Platform();

  bool isOsalReady();
 
  bool isHalUartReady();

  bool isLoggerReady();

  bool registerHalUartHandler(const T_OsdkHalUartHandler *halUartHandler);

#ifdef __linux__
  bool registerHalUSBBulkHandler(const T_OsdkHalUSBBulkHandler *halUSBBulkHandler);
#endif

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
  bool loggerConsoleRegFlag;

};
}
}

//temporary implement for dji_log
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
