/**
 ********************************************************************
 * @file    osdk_buffer.h
 * @version V1.0.0
 * @date    2019/9/15
 * @brief   This is the header file for "osdk_buffer.c", defining the structure
 *and
 * (exported) function prototypes.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OSDK_BUFFER_H
#define OSDK_BUFFER_H

/* Includes ------------------------------------------------------------------*/
#include "osdk_osal.h"
#include "osdk_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct {
  uint8_t *bufferPointer;
  uint16_t bufferSize;
  uint16_t readIndex;
  uint16_t writeIndex;
} T_OsdkBuffer;

/* Exported functions --------------------------------------------------------*/
E_OsdkStat OsdkBuffer_Init(T_OsdkBuffer *buffer, uint8_t *space, uint16_t size);
E_OsdkStat OsdkBuffer_DeInit(T_OsdkBuffer *buffer);
E_OsdkStat OsdkBuffer_PutToBack(T_OsdkBuffer *buffer, const uint8_t *data,
                                uint16_t len, uint16_t *realPutLen);
E_OsdkStat OsdkBuffer_PutToFront(T_OsdkBuffer *buffer, const uint8_t *data,
                                 uint16_t len, uint16_t *realPutLen);
E_OsdkStat OsdkBuffer_GetFromFront(T_OsdkBuffer *buffer, uint8_t *data,
                                   uint16_t len, uint16_t *realGetLen);
E_OsdkStat OsdkBuffer_GetUnusedSize(T_OsdkBuffer *buffer, uint16_t *unusedSize);

#ifdef __cplusplus
}
#endif

#endif  // OSDK_BUFFER_H

/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
