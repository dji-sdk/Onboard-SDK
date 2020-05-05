/**
 ********************************************************************
 * @file    osdk_hal.h
 * @version V2.0.0
 * @date    2019/8/28
 * @brief   This is the header file for "osdk_hal.c", defining the structure and
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
#ifndef OSDK_HAL_H
#define OSDK_HAL_H

/* Includes ------------------------------------------------------------------*/
#include "osdk_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
#define HAL_ONCE_READ_LEN 1024

/* Exported types ------------------------------------------------------------*/
typedef E_OsdkStat (*Hal_Send)(
    const T_HalObj *obj,
    const uint8_t *pBuf,
    uint32_t bufLen);

typedef E_OsdkStat (*Hal_Read)(
    const T_HalObj *obj,
    uint8_t *pBuf,
    uint32_t *bufLen);

typedef E_OsdkStat (*Hal_Close)(
    T_HalObj *obj);

typedef struct {
  char *name;
  Hal_Send  Send;
  Hal_Read  Read;
  Hal_Close Close;
} T_HalOps;
/* Exported functions --------------------------------------------------------*/
E_OsdkStat OsdkHal_UartInit(const char *port, const int baudrate, T_HalObj *obj);
E_OsdkStat OsdkHal_UartSendData(const T_HalObj *obj, const uint8_t *pBuf, uint32_t bufLen);
E_OsdkStat OsdkHal_UartReadData(const T_HalObj *obj, uint8_t *pBuf, uint32_t *bufLen);
E_OsdkStat OsdkHal_UartClose(T_HalObj *obj);
#ifdef __linux__
E_OsdkStat OsdkHal_USBBulkInit(uint16_t pid, uint16_t vid, uint16_t num, uint16_t epIn, uint16_t epOut, T_HalObj *obj);
E_OsdkStat OsdkHal_USBBulkSendData(const T_HalObj *obj, const uint8_t *pBuf, uint32_t bufLen);
E_OsdkStat OsdkHal_USBBulkReadData(const T_HalObj *obj, uint8_t *pBuf, uint32_t *bufLen);
E_OsdkStat OsdkHal_USBBulkClose(T_HalObj *obj);
#endif
E_OsdkStat OsdkHal_GetHalOps(const char *interface, T_HalOps *ops);

#ifdef __cplusplus
}
#endif

#endif // OSDK_HAL_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
