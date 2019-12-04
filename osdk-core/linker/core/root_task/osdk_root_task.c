/**
 ********************************************************************
 * @file    osdk_root_task.c
 * @version V1.0.0
 * @date    2019/09/25
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
#include "osdk_root_task.h"
#include "osdk_logger_internal.h"
#include "osdk_osal.h"
#include "osdk_util.h"

/* Private constants ---------------------------------------------------------*/
#define OSDK_ROOT_TASK_FREQ_HZ (1000)

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/
static void *OsdkCore_RootTask(void *arg);

/* Private variables ---------------------------------------------------------*/
static T_OsdkTaskHandle s_osdkRootThread;
static T_OsdkWork s_workList;

/* Exported functions definition ---------------------------------------------*/
T_OsdkWork *OsdkCore_GetWorkInstance(void) { return &s_workList; }

uint16_t OsdkCore_GetRootTaskFreq(void) { return OSDK_ROOT_TASK_FREQ_HZ; }

/* Private functions definition-----------------------------------------------*/
E_OsdkStat OsdkCore_RootTaskInit(void) {
  E_OsdkStat osdkStat;
  s_workList.name = "root_task_list";
  osdkStat = OsdkWork_Init(&s_workList);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CORE, "osdk work list init error:%d", osdkStat);
    return osdkStat;
  }

  osdkStat = OsdkOsal_TaskCreate(&s_osdkRootThread, OsdkCore_RootTask,
                                 OSDK_TASK_STACK_SIZE_DEFAULT, NULL);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CORE, "core root task create error:%d",
                   osdkStat);
    return osdkStat;
  }

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkCore_RootTaskDeInit(void) {
  return OsdkOsal_TaskDestroy(s_osdkRootThread);
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

static void *OsdkCore_RootTask(void *arg) {
  OSDK_UTIL_UNUSED(arg);

  while (1) {
    OsdkOsal_TaskSleepMs(1000 / OSDK_ROOT_TASK_FREQ_HZ);
    OsdkWork_CallWorkList(&s_workList);
  }
  return 0;
}

#pragma clang diagnostic pop

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
