/**
 ********************************************************************
 * @file    osdkosal_stm32.c
 * @version V4.0
 * @date    2019/11/22
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
#include "osdkosal_stm32.h"
#include "limits.h"
#include "FreeRTOS.h"
#include "task.h"

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/

/* Private functions definition-----------------------------------------------*/
E_OsdkStat OsdkSTM32_TaskCreate(T_OsdkTaskHandle *task, void *(*taskFunc)(void *),
                               uint32_t stackSize, void *arg) {
  /* @TODO The type of taskFunc should be infirmed */
  BaseType_t result;
  static uint8_t taskCnt = 0;
  char taskName[100] = {0};
  
  snprintf(taskName, sizeof(taskName), "Task_%d", taskCnt);
  result = xTaskCreate((TaskFunction_t)taskFunc, taskName, stackSize, arg, 0, *task);
  taskCnt++;
  if (result != pdTRUE) {
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkSTM32_TaskDestroy(T_OsdkTaskHandle task) {
  vTaskDelete(task);

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkSTM32_TaskSleepMs(uint32_t time_ms) {
  vTaskDelay(time_ms * portTICK_RATE_MS);
  return OSDK_STAT_OK;
}

/**
 * @brief Declare the mutex container, initialize the mutex, and
 * create mutex ID.
 * @param mutex:  pointer to the created mutex ID.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkSTM32_MutexCreate(T_OsdkMutexHandle *mutex) {
  *mutex = xSemaphoreCreateMutex();
  if (*mutex == NULL) {
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Delete the created mutex.
 * @param mutex:  pointer to the created mutex ID.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkSTM32_MutexDestroy(T_OsdkMutexHandle mutex) {
  vQueueDelete(mutex);

  return OSDK_STAT_OK;
}

/**
 * @brief Acquire and lock the mutex when peripheral access is required
 * @param mutex:  pointer to the created mutex ID.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkSTM32_MutexLock(T_OsdkMutexHandle mutex) {
  TickType_t ticks;

  if (mutex == NULL) {
      return OSDK_STAT_ERR_PARAM;
  }

  ticks = portMAX_DELAY;

  if (xSemaphoreTake(mutex, ticks) != pdTRUE) {
      return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Unlock and release the mutex, when done with the peripheral access.
 * @param mutex:  pointer to the created mutex ID.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkSTM32_MutexUnlock(T_OsdkMutexHandle mutex) {
  if (xSemaphoreGive(mutex) != pdTRUE) {
      return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Declare the semaphore container, initialize the semaphore, and
 * create semaphore ID.
 * @param semaphore: pointer to the created semaphore ID.
 * @param initValue: initial value of semaphore.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkSTM32_SemaphoreCreate(T_OsdkSemHandle *semaphore,
                                     uint32_t initValue) {
  uint32_t maxCount = UINT_MAX;

  *semaphore = xSemaphoreCreateCounting(maxCount, initValue);

  if (*semaphore == NULL) {
      return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Delete the created semaphore.
 * @param semaphore: pointer to the created semaphore ID.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkSTM32_SemaphoreDestroy(T_OsdkSemHandle semaphore) {
  vSemaphoreDelete(semaphore);

  return OSDK_STAT_OK;
}

/**
 * @brief Wait the semaphore until token becomes available.
 * @param semaphore: pointer to the created semaphore ID.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkSTM32_SemaphoreWait(T_OsdkSemHandle semaphore) {
  OsdkOsal_SemaphoreTimedWait(semaphore, portMAX_DELAY);

  return OSDK_STAT_OK;
}

/**
 * @brief Wait the semaphore until token becomes available.
 * @param semaphore: pointer to the created semaphore ID.
 * @param waitTime: timeout value of waiting semaphore, unit: millisecond.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkSTM32_SemaphoreTimedWait(T_OsdkSemHandle semaphore,
                                        uint32_t waitTime) {
  TickType_t ticks;

  if (semaphore == NULL) {
      return OSDK_STAT_ERR_PARAM;
  }

  ticks = 0;
  if (waitTime == portMAX_DELAY) {
      ticks = portMAX_DELAY;
  } else if (waitTime != 0) {
      ticks = waitTime / portTICK_PERIOD_MS;
      if (ticks == 0) {
          ticks = 1;
      }
  }
  if (xSemaphoreTake(semaphore, ticks) != pdTRUE) {
      return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Release the semaphore token.
 * @param semaphore: pointer to the created semaphore ID.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkSTM32_SemaphorePost(T_OsdkSemHandle semaphore) {
  if (xSemaphoreGive(semaphore) != pdTRUE) {
      return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Get the system time for ms.
 * @return an uint32 that the time of system, uint:ms
 */
E_OsdkStat OsdkSTM32_GetTimeMs(uint32_t *ms) {
  *ms = xTaskGetTickCount();

  return OSDK_STAT_OK;
}

#ifdef OS_DEBUG
/**
 * @brief Get the system time for us.
 * @return an uint64 that the time of system, uint:us
 */
E_OsdkStat OsdkSTM32_GetTimeUs(uint64_t *us) {
  return OSDK_STAT_ERR;
}
#endif

void *OsdkSTM32_Malloc(uint32_t size)
{
  return pvPortMalloc(size);
}

void OsdkSTM32_Free(void *ptr)
{
  vPortFree(ptr);
}
/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
