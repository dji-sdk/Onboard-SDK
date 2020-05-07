/**
 ********************************************************************
 * @file    osdkosal_linux.c
 * @version V1.0.0
 * @date    2019/09/17
 * @brief
 *
 * @copyright (c) 2018-2019 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "osdkosal_linux.h"

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/

/* Private functions definition-----------------------------------------------*/

/**
 * @brief Create task.
 * @param task:  pointer to the created task handle.
 * @param taskFunc:  pointer to the task function.
 * @param stackSize:  stack size for created task.
 * @param arg:  task function arg.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_TaskCreate(T_OsdkTaskHandle *task, void *(*taskFunc)(void *),
                                uint32_t stackSize, void *arg) {
  int result;

  *task = malloc(sizeof(pthread_t));
  result = pthread_create(*task, NULL, taskFunc, arg);
  if (result != 0) {
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Destroy task.
 * @param task:  pointer to the created task handle.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_TaskDestroy(T_OsdkTaskHandle task) {
  pthread_cancel(*(pthread_t *)task);
  pthread_join(*(pthread_t *)task, NULL);

  return OSDK_STAT_OK;
}

/**
 * @brief Task sleep function.
 * @param time_ms: task sleep time. unit: millisecond.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_TaskSleepMs(uint32_t time_ms) {
  usleep(1000 * time_ms);

  return OSDK_STAT_OK;
}

/**
 * @brief Create and initialize the mutex handle.
 * @param mutex:  pointer to the created mutex handle.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_MutexCreate(T_OsdkMutexHandle *mutex) {
  int result;

  *mutex = malloc(sizeof(pthread_mutex_t));
  result = pthread_mutex_init(*mutex, NULL);
  if (result != 0) {
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Delete the created mutex.
 * @param mutex:  pointer to the created mutex handle.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_MutexDestroy(T_OsdkMutexHandle mutex) {
  int result;

  result = pthread_mutex_destroy(mutex);
  if (result != 0) {
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Acquire and lock the mutex when peripheral access is required
 * @param mutex:  pointer to the created mutex Handle.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_MutexLock(T_OsdkMutexHandle mutex) {
  int result;

  result = pthread_mutex_lock(mutex);
  if (result != 0) {
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Unlock and release the mutex, when done with the peripheral access.
 * @param mutex:  pointer to the created mutex handle.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_MutexUnlock(T_OsdkMutexHandle mutex) {
  int result = pthread_mutex_unlock(mutex);

  if (result != 0) {
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Create and initialize the semaphore handle.
 * @param semaphore: pointer to the created semaphore handle.
 * @param initValue: initial value of semaphore.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_SemaphoreCreate(T_OsdkSemHandle *semaphore,
                                     uint32_t initValue) {
  int result;

  *semaphore = malloc(sizeof(sem_t));
  result = sem_init(*semaphore, 0, (unsigned int)initValue);
  if (result != 0) {
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Delete the created semaphore.
 * @param semaphore: pointer to the created semaphore handle.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_SemaphoreDestroy(T_OsdkSemHandle semaphore) {
  int result;

  result = sem_destroy(semaphore);
  if (result != 0) {
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Wait the semaphore until token becomes available.
 * @param semaphore: pointer to the created semaphore handle.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_SemaphoreWait(T_OsdkSemHandle semaphore) {
  int result;

  result = sem_wait(semaphore);
  if (result != 0) {
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Wait the semaphore until token becomes available.
 * @param semaphore: pointer to the created semaphore handle.
 * @param waitTime: timeout value of waiting semaphore, unit: millisecond.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_SemaphoreTimedWait(T_OsdkSemHandle semaphore,
                                        uint32_t waitTime) {
  int result;
  struct timespec semaphoreWaitTime;
  struct timeval systemTime;

  gettimeofday(&systemTime, NULL);

  systemTime.tv_usec += waitTime * 1000;
  if (systemTime.tv_usec >= 1000000) {
    systemTime.tv_sec += systemTime.tv_usec / 1000000;
    systemTime.tv_usec %= 1000000;
  }

  semaphoreWaitTime.tv_sec = systemTime.tv_sec;
  semaphoreWaitTime.tv_nsec = systemTime.tv_usec * 1000;

  result = sem_timedwait(semaphore, &semaphoreWaitTime);
  if (result != 0) {
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Release the semaphore token.
 * @param semaphore: pointer to the created semaphore handle.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_SemaphorePost(T_OsdkSemHandle semaphore) {
  int result;

  result = sem_post(semaphore);
  if (result != 0) {
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Get the system time for ms.
 * @param us: time of system, uint:ms
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_GetTimeMs(uint32_t *ms) {
  struct timeval time;

  gettimeofday(&time, NULL);
  *ms = (time.tv_sec * 1000 + time.tv_usec / 1000);

  return OSDK_STAT_OK;
}

/**
 * @brief Get the system time for us, os related.
 * @param us: time of system, uint:us
 * @return an enum that represents a status of OSDK
 */
#ifdef OS_DEBUG
E_OsdkStat OsdkLinux_GetTimeUs(uint64_t *us) {
  struct timeval time;

  gettimeofday(&time, NULL);
  *us = (time.tv_sec * 1000000 + time.tv_usec);

  return OSDK_STAT_OK;
}
#endif

void *OsdkLinux_Malloc(uint32_t size)
{
  return malloc(size);
}

void OsdkLinux_Free(void *ptr)
{
  free(ptr);
}
/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
