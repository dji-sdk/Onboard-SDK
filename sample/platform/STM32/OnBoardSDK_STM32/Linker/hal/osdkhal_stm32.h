/**
 ********************************************************************
 * @file    osdk_hal.h
 * @version V2.0.0
 * @date    2019/07/01
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
#ifndef OSDKHAL_STM32_H
#define OSDKHAL_STM32_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include <stdio.h>
#include <string.h>
#include "osdk_typedef.h"
#include "osdk_platform.h"

#define UART_PORT "UART"
#define ACM_PORT  "ACM"
#define HAL_ONCE_READ_LEN 1024

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

E_OsdkStat OsdkSTM32_UartSendData(const T_HalObj *obj, const uint8_t *pBuf, uint32_t bufLen);
E_OsdkStat OsdkSTM32_UartReadData(const T_HalObj *obj, uint8_t *pBuf, uint32_t *bufLen);
E_OsdkStat OsdkSTM32_UartClose(T_HalObj *obj);
E_OsdkStat OsdkSTM32_UartInit(const char *port, const int baudrate, T_HalObj *obj);
E_OsdkStat OsdkSTM32_UartClose(T_HalObj *obj);

#ifdef __cplusplus
}
#endif

#endif // OSDKHAL_STM32_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
