/**
 ********************************************************************
 * @file    osdk_osal.c
 * @version V2.0.0
 * @date    2019/07/01
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
#include <string.h>
#include "osdk_platform.h"
#include "osdk_logger_internal.h"

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/

/* Private functions definition-----------------------------------------------*/
static T_OsdkOsalHandler s_osdkPlatformOsal;

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
E_OsdkStat OsdkPlatform_RegOsalHandler(const T_OsdkOsalHandler *osalHandler)
{
    if (osalHandler->TaskCreate == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (osalHandler->TaskDestroy == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (osalHandler->TaskSleepMs == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (osalHandler->MutexCreate == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (osalHandler->MutexDestroy == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (osalHandler->MutexLock == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (osalHandler->MutexUnlock == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (osalHandler->SemaphoreCreate == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (osalHandler->SemaphoreDestroy == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (osalHandler->SemaphorePost == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (osalHandler->SemaphoreWait == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (osalHandler->SemaphoreTimedWait == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (osalHandler->GetTimeMs == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (osalHandler->Malloc == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (osalHandler->Free == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    memcpy(&s_osdkPlatformOsal, osalHandler, sizeof(T_OsdkOsalHandler));

    return OSDK_STAT_OK;
}

E_OsdkStat
OsdkOsal_TaskCreate(T_OsdkTaskHandle *task, void *(*taskFunc)(void *), uint32_t stackSize, void *arg)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkPlatformOsal.TaskCreate(task, taskFunc, stackSize, arg);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "task create error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

E_OsdkStat OsdkOsal_TaskDestroy(T_OsdkTaskHandle task)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkPlatformOsal.TaskDestroy(task);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "task destroy error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

E_OsdkStat OsdkOsal_TaskSleepMs(uint32_t timeMs)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkPlatformOsal.TaskSleepMs(timeMs);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "task sleep ms error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

/**
 * @brief Declare the mutex container, initialize the mutex, and
 * create mutex ID.
 * @param mutex:  pointer to the created mutex ID.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkOsal_MutexCreate(T_OsdkMutexHandle *mutex)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkPlatformOsal.MutexCreate(mutex);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "mutex create error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

/**
 * @brief Delete the created mutex.
 * @param mutex:  pointer to the created mutex ID.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkOsal_MutexDestroy(T_OsdkMutexHandle mutex)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkPlatformOsal.MutexDestroy(mutex);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "mutex destroy error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

/**
 * @brief Acquire and lock the mutex when peripheral access is required
 * @param mutex:  pointer to the created mutex ID.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkOsal_MutexLock(T_OsdkMutexHandle mutex)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkPlatformOsal.MutexLock(mutex);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "mutex lock error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return OSDK_STAT_OK;
}

/**
 * @brief Unlock and release the mutex, when done with the peripheral access.
 * @param mutex:  pointer to the created mutex ID.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkOsal_MutexUnlock(T_OsdkMutexHandle mutex)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkPlatformOsal.MutexUnlock(mutex);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "mutex unlock error, stat:%lld", osdkStat);
        return osdkStat;
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
E_OsdkStat OsdkOsal_SemaphoreCreate(T_OsdkSemHandle *semaphore, uint32_t initValue)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkPlatformOsal.SemaphoreCreate(semaphore, initValue);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "semaphore create error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

/**
 * @brief Delete the created semaphore.
 * @param semaphore: pointer to the created semaphore ID.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkOsal_SemaphoreDestroy(T_OsdkSemHandle semaphore)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkPlatformOsal.SemaphoreDestroy(semaphore);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "semaphore destroy error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

/**
 * @brief Wait the semaphore until token becomes available.
 * @param semaphore: pointer to the created semaphore ID.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkOsal_SemaphoreWait(T_OsdkSemHandle semaphore)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkPlatformOsal.SemaphoreWait(semaphore);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "semaphore wait error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

/**
 * @brief Wait the semaphore until token becomes available.
 * @param semaphore: pointer to the created semaphore ID.
 * @param waitTime: timeout value of waiting semaphore, unit: millisecond.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkOsal_SemaphoreTimedWait(T_OsdkSemHandle semaphore, uint32_t waitTime)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkPlatformOsal.SemaphoreTimedWait(semaphore, waitTime);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "semaphore wait time error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

/**
 * @brief Release the semaphore token.
 * @param semaphore: pointer to the created semaphore ID.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkOsal_SemaphorePost(T_OsdkSemHandle semaphore)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkPlatformOsal.SemaphorePost(semaphore);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "semaphore post error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

/**
 * @brief Get the system time for ms.
 * @param us: time of system, uint:ms
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkOsal_GetTimeMs(uint32_t *ms)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkPlatformOsal.GetTimeMs(ms);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "get time ms error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

/**
 * @brief Get the system time for us, os related.
 * @param us: time of system, uint:us
 * @return an enum that represents a status of OSDK
 */
#ifdef OS_DEBUG
E_OsdkStat OsdkOsal_GetTimeUs(uint64_t *us) {
    E_OsdkStat osdkStat;

    osdkStat = s_osdkPlatformOsal.GetTimeUs(us);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "get time ms error, stat:%lld", osdkStat);
        return osdkStat;
    }

  return OSDK_STAT_OK;
}
#endif

void *OsdkOsal_Malloc(uint32_t size)
{
    return s_osdkPlatformOsal.Malloc(size);
}

void OsdkOsal_Free(void *ptr)
{
    s_osdkPlatformOsal.Free(ptr);
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
