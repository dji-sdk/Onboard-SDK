/**
 ********************************************************************
 * @file    osdk_msgq.h
 * @version V1.0.0
 * @date    2019/09/25
 * @brief   This is the header file for "osdk_msgq.c", defining the structure
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
#ifndef OSDK_MSGQ_H
#define OSDK_MSGQ_H

/* Includes ------------------------------------------------------------------*/
#include "osdk_logger_internal.h"
#include "osdk_module_name.h"
#include "osdk_osal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
#define MSG_MAX_LEN 0x400
#define MSGQ_SIZE (MSG_MAX_LEN * 8)

#define OSDK_WAIT_ONE_SECOND (uint32_t)(1000)
#define OSDK_WAIT_POLLING (uint32_t)(0)
#define OSDK_WAIT_FOREVER (uint32_t)(-1)

#define DATA_SIZE(mq) ((mq->head - mq->tail) & (mq->bufSize - 1))
#define FREE_SIZE(mq) (mq->bufSize - 1 - DATA_SIZE(mq))

/* Exported types ------------------------------------------------------------*/
typedef struct {
  char *name;
  uint32_t bufSize;
} T_msgqAttrib;

typedef struct {
  T_msgqAttrib attrib;
  T_OsdkMutexHandle sendMutex;
  T_OsdkMutexHandle recvMutex;
  T_OsdkSemHandle sendSem;
  T_OsdkSemHandle recvSem;
  uint32_t bufSize;
  uint32_t head;
  uint32_t tail;
  uint8_t *buffer;
} T_msgQueue;

/* Exported functions --------------------------------------------------------*/
E_OsdkStat OsdkMsgq_Create(T_msgqAttrib *attrib, T_msgQueue **msgq);
E_OsdkStat OsdkMsgq_Destroy(T_msgQueue *msgq);
E_OsdkStat OsdkMsgq_Recv(T_msgQueue *msgq, uint8_t *buffer, uint32_t *size,
                         uint32_t timeout);
E_OsdkStat OsdkMsgq_Send(T_msgQueue *msgq, void *message, uint32_t size,
                         uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif  // OSDK_MSGQ_H
        /************************ (C) COPYRIGHT DJI Innovations *******END OF
         * FILE******/
