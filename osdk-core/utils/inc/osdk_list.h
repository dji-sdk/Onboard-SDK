/**
 ********************************************************************
 * @file    osdk_list.h
 * @version V1.0.0
 * @date    2019/09/15
 * @brief   This is the header file for "osdk_list.c", defining the structure
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
#ifndef OSDK_LIST_H
#define OSDK_LIST_H

/* Includes ------------------------------------------------------------------*/
#include "osdk_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
#define OSDK_LIST_IS_EMPTY(list) ((list) == (list)->pNext)

#define OSDK_LIST_ENTRY(ptr, type, member) \
  ((type *)((char *)(ptr) - (unsigned long)(&((type *)0)->member)))

#define OSDK_LIST_FOR_EACH_ENTRY_SAFE(pos, n, head, member)         \
  for (pos = OSDK_LIST_ENTRY((head)->pNext, typeof(*pos), member),  \
      n = OSDK_LIST_ENTRY(pos->member.pNext, typeof(*pos), member); \
       &pos->member != (head);                                      \
       pos = n, n = OSDK_LIST_ENTRY(n->member.pNext, typeof(*n), member))

#define OSDK_LIST_FOR_EACH_SAFE(pos, n, head)              \
  for (pos = (head)->pNext, n = pos->pNext; pos != (head); \
       pos = n, n = pos->pNext)

/* Exported types ------------------------------------------------------------*/
typedef struct listHead T_OsdkList;
struct listHead {
  struct listHead *pNext;
  struct listHead *pPrev;
};

/* Exported functions --------------------------------------------------------*/
void OsdkList_Init(T_OsdkList *list);
void OsdkList_Add(T_OsdkList *list, T_OsdkList *head);
void OsdkList_AddTail(T_OsdkList *list, T_OsdkList *head);
void OsdkList_Del(T_OsdkList *entry);

#ifdef __cplusplus
}
#endif

#endif  // OSDK_LIST_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
