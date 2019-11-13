/**
 ********************************************************************
 * @file    osdk_list.c
 * @version V1.0.0
 * @date    2019/09/15
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
#include "osdk_list.h"

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
static void __OsdkList_Add(T_OsdkList *node, T_OsdkList *prev,
                           T_OsdkList *next);
static void __OsdkList_Del(T_OsdkList *prev, T_OsdkList *next);

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
void OsdkList_Init(T_OsdkList *list) {
  list->pNext = list;
  list->pPrev = list;
}

void OsdkList_Add(T_OsdkList *node, T_OsdkList *head) {
  __OsdkList_Add(node, head, head->pPrev);
}

void OsdkList_AddTail(T_OsdkList *node, T_OsdkList *head) {
  __OsdkList_Add(node, head->pPrev, head);
}

void OsdkList_Del(T_OsdkList *list) {
  __OsdkList_Del(list->pPrev, list->pNext);
}

/* Private functions definition-----------------------------------------------*/
static void __OsdkList_Add(T_OsdkList *node, T_OsdkList *prev,
                           T_OsdkList *next) {
  next->pPrev = node;
  node->pNext = next;
  node->pPrev = prev;
  prev->pNext = node;
}

static void __OsdkList_Del(T_OsdkList *prev, T_OsdkList *next) {
  next->pPrev = prev;
  prev->pNext = next;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/