/**
 ********************************************************************
 * @file    osdk_work.c
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
#include "osdk_work.h"
#include "osdk_logger_internal.h"

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/
static E_OsdkStat OsdkWork_CheckDuplicate(T_OsdkWork *work,
                                          T_OsdkWorkNode *node);

/* Exported functions definition ---------------------------------------------*/
E_OsdkStat OsdkWork_Init(T_OsdkWork *work) {
  OsdkList_Init(&work->head);
  work->workCnt = 0;

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkWork_DeInit(T_OsdkWork *work) {
  memset(work, 0, sizeof(T_OsdkWork));

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkWork_AddNode(T_OsdkWork *work, T_OsdkWorkNode *node) {
  E_OsdkStat osdkStat;

  if (!node || !work) {
    OSDK_LOG_INFO(MODULE_NAME_UTIL, "osdk add node param is NULL");
    return OSDK_STAT_ERR_PARAM;
  }

  if (node->taskFunc == NULL || node->name == NULL) {
    OSDK_LOG_INFO(MODULE_NAME_UTIL, "osdk work new node info is NULL");
    return OSDK_STAT_ERR_PARAM;
  }

  osdkStat = OsdkWork_CheckDuplicate(work, node);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "osdk work add new node error:%d",
                   osdkStat);
    return osdkStat;
  }
  OsdkList_AddTail(&node->head, &work->head);

  work->workCnt++;
  OSDK_LOG_INFO(MODULE_NAME_UTIL, "osdk work add new node success:%d",
                work->workCnt);

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkWork_DeleteNode(T_OsdkWork *work, T_OsdkWorkNode *node) {
  if (!node || !work) {
    OSDK_LOG_INFO(MODULE_NAME_UTIL, "osdk del node param is NULL");
    return OSDK_STAT_ERR_PARAM;
  }

  OsdkList_Del(&node->head);

  work->workCnt--;
  OSDK_LOG_INFO(MODULE_NAME_UTIL, "osdk work del node success:%d",
                work->workCnt);

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkWork_CallWorkList(T_OsdkWork *work) {
  T_OsdkWorkNode *pNode = NULL;
  T_OsdkWorkNode *pBackup = NULL;

  if (!work) {
    OSDK_LOG_INFO(MODULE_NAME_UTIL, "osdk call worklist param is NULL");
    return OSDK_STAT_ERR_PARAM;
  }

  OSDK_LIST_FOR_EACH_ENTRY_SAFE(pNode, pBackup, &work->head, head) {
    if (pNode->taskFunc != NULL) {
      pNode->taskFunc(pNode->arg);
    } else {
      OSDK_LOG_ERROR(MODULE_NAME_UTIL, "osdk call worklist param is NULL");
      return OSDK_STAT_ERR;
    }
  }

  return OSDK_STAT_OK;
}

/* Private functions definition-----------------------------------------------*/
static E_OsdkStat OsdkWork_CheckDuplicate(T_OsdkWork *work,
                                          T_OsdkWorkNode *node) {
  T_OsdkWorkNode *pNode = NULL;
  T_OsdkWorkNode *pBackup = NULL;

  if (!work) {
    OSDK_LOG_INFO(MODULE_NAME_UTIL, "osdk check duplicate param is NULL");
    return OSDK_STAT_ERR_PARAM;
  }

  if (work->workCnt > 0) {
    OSDK_LIST_FOR_EACH_ENTRY_SAFE(pNode, pBackup, &work->head, head) {
      if (strcmp(node->name, pNode->name) == 0) {
        OSDK_LOG_WARN(MODULE_NAME_UTIL,
                      "osdk work reg new node duplicate name:%s", node->name);
        return OSDK_STAT_ERR;
      }

      if (node->taskFunc == pNode->taskFunc) {
        OSDK_LOG_WARN(MODULE_NAME_UTIL,
                      "osdk work reg new node duplicate func:%s", node->name);
        return OSDK_STAT_ERR;
      }
    }
  }

  return OSDK_STAT_OK;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
