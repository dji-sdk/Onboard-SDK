/**
 ********************************************************************
 * @file    osdk_msgq.c
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
#include "osdk_msgq.h"

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
E_OsdkStat OsdkMsgq_Create(T_msgqAttrib *attrib, T_msgQueue **msgq) {
  T_msgQueue *tm;
  uint32_t bufSize;

  if (attrib == NULL || msgq == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "message queue param error");
    return OSDK_STAT_ERR_PARAM;
  }
  bufSize = 1;
  while (bufSize <= attrib->bufSize) {
    bufSize <<= 1;
  }
  tm = OsdkOsal_Malloc(sizeof(T_msgQueue) + bufSize);
  if (tm == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "msgqueue malloc error");
    return OSDK_STAT_ERR_ALLOC;
  }
  memcpy(&tm->attrib, attrib, sizeof(T_msgqAttrib));
  tm->buffer = (uint8_t *)((uint8_t *)tm + sizeof(T_msgQueue));
  tm->bufSize = bufSize;
  tm->head = tm->tail = 0;

  if (OsdkOsal_MutexCreate(&tm->sendMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "send mutex create failed");
    goto err;
  }
  if (OsdkOsal_MutexCreate(&tm->recvMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "recv mutex create failed");
    goto err;
  }
  if (OsdkOsal_SemaphoreCreate(&tm->sendSem, 0) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "send semaphore create failed");
    goto err;
  }
  if (OsdkOsal_SemaphoreCreate(&tm->recvSem, 0) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "recv semaphore create failed");
    goto err;
  }
  *msgq = tm;
  return OSDK_STAT_OK;

err:
  OsdkOsal_Free(tm);
  return OSDK_STAT_ERR;
}

E_OsdkStat OsdkMsgq_Destroy(T_msgQueue *msgq) {
  if (OsdkOsal_MutexDestroy(msgq->sendMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "send mutex destroy failed");
    return OSDK_STAT_ERR;
  }
  if (OsdkOsal_MutexDestroy(msgq->recvMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "recv mutex destroy failed");
    return OSDK_STAT_ERR;
  }
  if (OsdkOsal_SemaphoreDestroy(msgq->sendSem) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "send semaphore destroy failed");
    return OSDK_STAT_ERR;
  }
  if (OsdkOsal_SemaphoreDestroy(msgq->recvSem) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "recv semaphore destroy failed");
    return OSDK_STAT_ERR;
  }
  OsdkOsal_Free(msgq);

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkMsgq_Recv(T_msgQueue *msgq, void *buffer, uint32_t *size,
                         uint32_t timeout) {
  if (msgq == NULL || buffer == NULL || size == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "msg recv param error");
    return OSDK_STAT_ERR_PARAM;
  }

  OsdkOsal_MutexLock(msgq->recvMutex);
  while (DATA_SIZE(msgq) == 0) {
    if (timeout == OSDK_WAIT_POLLING) {
      OsdkOsal_MutexUnlock(msgq->recvMutex);
      OSDK_LOG_ERROR(MODULE_NAME_UTIL, "no enough resource");
      return OSDK_STAT_ERR;
    } else if (timeout == OSDK_WAIT_FOREVER) {
      if (OsdkOsal_SemaphoreWait(msgq->recvSem) != OSDK_STAT_OK) {
        OsdkOsal_MutexUnlock(msgq->recvMutex);
        OSDK_LOG_ERROR(MODULE_NAME_UTIL, "semaphore wait error");
        return OSDK_STAT_ERR;
      }
    } else {
      if (OsdkOsal_SemaphoreTimedWait(msgq->recvSem, timeout) !=
          OSDK_STAT_OK) {
        OsdkOsal_MutexUnlock(msgq->recvMutex);
        OSDK_LOG_ERROR(MODULE_NAME_UTIL, "semaphore wait timeout");
        return OSDK_STAT_ERR_TIMEOUT;
      }
    }
  }

  uint32_t i, curSize = DATA_SIZE(msgq), tempSize = 0;

  if (curSize > *size) {
    curSize = *size;
  } else {
    *size = curSize;
  }
  tempSize = *size;
  i = msgq->tail;
  if (i + tempSize > msgq->bufSize) {
    memcpy(buffer, &msgq->buffer[i], msgq->bufSize - i);
    buffer = (uint8_t *)(buffer + msgq->bufSize - i);
    tempSize -= msgq->bufSize - i;
    i = 0;
  }
  memcpy(buffer, &msgq->buffer[i], tempSize);
  msgq->tail = i + tempSize;

  if (OsdkOsal_MutexUnlock(msgq->recvMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "unlock recv mutex failed");
    return OSDK_STAT_ERR;
  }

  if (OsdkOsal_SemaphorePost(msgq->sendSem) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "send semaphore post failed");
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkMsgq_Send(T_msgQueue *msgq, void *message, uint32_t size,
                         uint32_t timeout) {
  if (msgq == NULL || message == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "msg send param error");
    return OSDK_STAT_ERR_PARAM;
  }

  if (OsdkOsal_MutexLock(msgq->sendMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "lock send mutex failed");
    return OSDK_STAT_ERR;
  }

  while (FREE_SIZE(msgq) < size) {
    if (timeout == OSDK_WAIT_POLLING) {
      OsdkOsal_MutexUnlock(msgq->sendMutex);
      OSDK_LOG_ERROR(MODULE_NAME_UTIL, "no enough resource");
      return OSDK_STAT_ERR;
    } else if (timeout == OSDK_WAIT_FOREVER) {
      if (OsdkOsal_SemaphoreWait(msgq->sendSem) != OSDK_STAT_OK) {
        OsdkOsal_MutexUnlock(msgq->sendMutex);
        OSDK_LOG_ERROR(MODULE_NAME_UTIL, "semaphore wait error");
        return OSDK_STAT_ERR;
      }
    } else {
      if (OsdkOsal_SemaphoreTimedWait(msgq->sendSem, timeout) !=
          OSDK_STAT_OK) {
        OsdkOsal_MutexUnlock(msgq->sendMutex);
        OSDK_LOG_ERROR(MODULE_NAME_UTIL, "semaphore wait timeout");
        return OSDK_STAT_ERR_TIMEOUT;
      }
    }
  }

  uint32_t i = msgq->head;
  uint8_t *data = (uint8_t *)message;
  if (i + size > msgq->bufSize) {
    memcpy(&msgq->buffer[i], data, msgq->bufSize - i);
    data += msgq->bufSize - i;
    size -= msgq->bufSize - i;
    i = 0;
  }
  memcpy(&msgq->buffer[i], data, size);
  msgq->head = i + size;
  if (OsdkOsal_MutexUnlock(msgq->sendMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "unlock send mutex failed");
    return OSDK_STAT_ERR;
  }

  if (OsdkOsal_SemaphorePost(msgq->recvSem) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "recv semaphore post failed");
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}
/* Private functions definition-----------------------------------------------*/

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
