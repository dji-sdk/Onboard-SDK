/**
 ********************************************************************
 * @file    osdk_buffer.c
 * @version V2.0.0
 * @date    2019/8/2
 * @brief
 *
 * @copyright (c) 2017-2018 DJI. All rights reserved.
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

#include "osdk_buffer.h"
#include "osdk_logger_internal.h"
#include "osdk_util.h"

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
static E_OsdkStat OsdkBuffer_CutBufferSizeToPowerOfTwo(uint16_t inSize,
                                                       uint16_t *outSize);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief
 * @note this function will cut size to the largest power of two that is not
 * greater than size. So, it is better that
 * size is power of two in order to not waste memory space.
 * @param buffer
 * @param space
 * @param size
 * @return
 */
E_OsdkStat OsdkBuffer_Init(T_OsdkBuffer *buffer, uint8_t *space,
                           uint16_t size) {
  E_OsdkStat osdkStat;

  if (buffer == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "input pointer is null.");
    return OSDK_STAT_ERR_PARAM;
  }

  if (space == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "input pointer space is null.");
    return OSDK_STAT_ERR_PARAM;
  }

  buffer->readIndex = 0;
  buffer->writeIndex = 0;
  buffer->bufferPointer = space;
  osdkStat = OsdkBuffer_CutBufferSizeToPowerOfTwo(size, &buffer->bufferSize);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "cut size for buffer error: %d.",
                   osdkStat);
    return OSDK_STAT_SYS_ERR;
  }

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkBuffer_DeInit(T_OsdkBuffer *buffer) {
  if (buffer == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "input pointer is null.");
    return OSDK_STAT_ERR_PARAM;
  }

  buffer->bufferPointer = NULL;
  buffer->bufferSize = 0;
  buffer->readIndex = 0;
  buffer->writeIndex = 0;

  return OSDK_STAT_OK;
}

/**
 * @brief
 * @note this function can not promise putting all data to buffer. If unused
 * data size of buffer is less than size of
 * data to be put, this function put front data whose size is unused data size
 * of buffer to buffer.
 * @note if there exist multi-producer or multi-consumer operating a buffer,
 * please lock this buffer to protect data
 * before call this function.
 * @param buffer
 * @param data
 * @param len
 * @return
 */
E_OsdkStat OsdkBuffer_PutToBack(T_OsdkBuffer *buffer, const uint8_t *data,
                                uint16_t len, uint16_t *realPutLen) {
  E_OsdkStat osdkStat;
  uint16_t writeUpLen = 0;
  uint16_t unusedSize = 0;
  uint16_t lenToPut = 0;

  if (realPutLen == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "input pointer is null.");
    return OSDK_STAT_ERR_PARAM;
  }

  osdkStat = OsdkBuffer_GetUnusedSize(buffer, &unusedSize);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_FLOWCONTROLLER,
                   "get unused size of buffer error: %d.", osdkStat);
    return OSDK_STAT_SYS_ERR;
  }

  lenToPut = OSDK_UTIL_MIN(len, unusedSize);

  // fill up data
  writeUpLen = OSDK_UTIL_MIN(
      lenToPut, (uint16_t)(buffer->bufferSize -
                           (buffer->writeIndex & (buffer->bufferSize - 1))));
  memcpy(
      buffer->bufferPointer + (buffer->writeIndex & (buffer->bufferSize - 1)),
      data, writeUpLen);

  // fill down data
  memcpy(buffer->bufferPointer, data + writeUpLen, lenToPut - writeUpLen);

  buffer->writeIndex += lenToPut;
  *realPutLen = lenToPut;

  return OSDK_STAT_OK;
}

/**
 * @brief
 * @note this function can not promise putting all data to buffer. If unused
 * data size of buffer is less than size of
 * data to be put, this function put back data whose size is unused data size of
 * buffer to buffer.
 * @note if there exist multi-producer or multi-consumer operating a buffer,
 * please lock this buffer to protect data
 * before call this function.
 * @param buffer
 * @param data
 * @param len
 * @return
 */
E_OsdkStat OsdkBuffer_PutToFront(T_OsdkBuffer *buffer, const uint8_t *data,
                                 uint16_t len, uint16_t *realPutLen) {
  E_OsdkStat osdkStat;
  uint16_t writeDownLen = 0;
  uint16_t unusedSize = 0;
  uint16_t lenToPut = 0;

  if (realPutLen == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "input pointer is null.");
    return OSDK_STAT_ERR_PARAM;
  }

  osdkStat = OsdkBuffer_GetUnusedSize(buffer, &unusedSize);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_FLOWCONTROLLER,
                   "get unused size of buffer error: %d.", osdkStat);
    return OSDK_STAT_SYS_ERR;
  }

  lenToPut = OSDK_UTIL_MIN(len, unusedSize);

  // fill down data
  writeDownLen = OSDK_UTIL_MIN(
      lenToPut, (uint16_t)(buffer->readIndex & (buffer->bufferSize - 1)));
  memcpy(buffer->bufferPointer +
             (buffer->readIndex & (buffer->bufferSize - 1)) - writeDownLen,
         data + len - writeDownLen, writeDownLen);

  // fill up data
  memcpy(buffer->bufferPointer + buffer->bufferSize - (lenToPut - writeDownLen),
         data + len - lenToPut, lenToPut - writeDownLen);

  buffer->readIndex -= lenToPut;
  *realPutLen = lenToPut;

  return OSDK_STAT_OK;
}

/**
 * @brief
 * @note if there exist multi-producer or multi-consumer operating a buffer,
 * please lock this buffer to protect data
 * before call this function.
 * @param buffer
 * @param data
 * @param len
 * @return
 */
E_OsdkStat OsdkBuffer_GetFromFront(T_OsdkBuffer *buffer, uint8_t *data,
                                   uint16_t len, uint16_t *realGetLen) {
  E_OsdkStat osdkStat;
  uint16_t readUpLen;
  uint16_t unusedSize = 0;
  uint16_t usedSize = 0;
  uint16_t lenToGet = 0;

  if (realGetLen == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "input pointer is null.");
    return OSDK_STAT_ERR_PARAM;
  }

  osdkStat = OsdkBuffer_GetUnusedSize(buffer, &unusedSize);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_FLOWCONTROLLER,
                   "get unused size of buffer error: %d.", osdkStat);
    return OSDK_STAT_SYS_ERR;
  }

  usedSize = buffer->bufferSize - unusedSize;
  lenToGet = OSDK_UTIL_MIN(len, usedSize);

  // get up data
  readUpLen = OSDK_UTIL_MIN(
      lenToGet, (uint16_t)(buffer->bufferSize -
                           (buffer->readIndex & (buffer->bufferSize - 1))));
  memcpy(data,
         buffer->bufferPointer + (buffer->readIndex & (buffer->bufferSize - 1)),
         readUpLen);

  // get down data
  memcpy(data + readUpLen, buffer->bufferPointer, lenToGet - readUpLen);

  buffer->readIndex += lenToGet;
  *realGetLen = lenToGet;

  return OSDK_STAT_OK;
}

/**
 * @brief
 * @note if there exist multi-producer or multi-consumer operating a buffer,
 * please lock this buffer to protect data
 * before call this function.
 * @param buffer
 * @param unusedSize
 * @return
 */
E_OsdkStat OsdkBuffer_GetUnusedSize(T_OsdkBuffer *buffer,
                                    uint16_t *unusedSize) {
  if (unusedSize == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "input pointer is null.");
    return OSDK_STAT_ERR_PARAM;
  }

  *unusedSize =
      (uint16_t)(buffer->bufferSize - buffer->writeIndex + buffer->readIndex);

  return OSDK_STAT_OK;
}

/* Private functions definition-----------------------------------------------*/
static E_OsdkStat OsdkBuffer_CutBufferSizeToPowerOfTwo(uint16_t inSize,
                                                       uint16_t *outSize) {
  int32_t i = 0;

  if (outSize == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "input pointer is null.");
    return OSDK_STAT_ERR_PARAM;
  }

  while ((1 << (++i)) <= inSize)
    ;
  *outSize = (uint16_t)(1 << (--i));

  return OSDK_STAT_OK;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
