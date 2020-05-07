/**
 ********************************************************************
 * @file    osdkosal_linux.h
 * @version V2.0.0
 * @date    2019/07/01
 * @brief   This is the header file for "osdk_osal.c", defining the structure
 *and
 * (exported) function prototypes.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OSDK_OSAL_LINUX_H
#define OSDK_OSAL_LINUX_H

/* Includes ------------------------------------------------------------------*/
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#include "osdk_typedef.h"
#include "osdk_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
// TODO:adapter for different system task interface void*
E_OsdkStat OsdkLinux_TaskCreate(T_OsdkTaskHandle *task, void *(*taskFunc)(void *),
                                uint32_t stackSize, void *arg);
E_OsdkStat OsdkLinux_TaskDestroy(T_OsdkTaskHandle task);
E_OsdkStat OsdkLinux_TaskSleepMs(uint32_t time_ms);

E_OsdkStat OsdkLinux_MutexCreate(T_OsdkMutexHandle *mutex);
E_OsdkStat OsdkLinux_MutexDestroy(T_OsdkMutexHandle mutex);
E_OsdkStat OsdkLinux_MutexLock(T_OsdkMutexHandle mutex);
E_OsdkStat OsdkLinux_MutexUnlock(T_OsdkMutexHandle mutex);

E_OsdkStat OsdkLinux_SemaphoreCreate(T_OsdkSemHandle *semaphore,
                                     uint32_t initValue);
E_OsdkStat OsdkLinux_SemaphoreDestroy(T_OsdkSemHandle semaphore);
E_OsdkStat OsdkLinux_SemaphoreWait(T_OsdkSemHandle semaphore);
E_OsdkStat OsdkLinux_SemaphoreTimedWait(T_OsdkSemHandle semaphore,
                                        uint32_t waitTime);
E_OsdkStat OsdkLinux_SemaphorePost(T_OsdkSemHandle semaphore);

E_OsdkStat OsdkLinux_GetTimeMs(uint32_t *ms);

#ifdef OS_DEBUG
E_OsdkStat OsdkLinux_GetTimeUs(uint64_t *us);
#endif
void *OsdkLinux_Malloc(uint32_t size);
void OsdkLinux_Free(void *ptr);


#ifdef __cplusplus
}
#endif

#endif  // OSDK_OSAL_LINUX_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
