/**
 ********************************************************************
 * @file    osdkhal_linux.h
 * @version V2.0.0
 * @date    2019/07/01
 * @brief   This is the header file for "osdkhal_linux.c", defining the structure and
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
#ifndef OSDK_HAL_LINUX_H
#define OSDK_HAL_LINUX_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "osdk_typedef.h"
#include "osdk_platform.h"

#ifdef ADVANCED_SENSING
#include <libusb-1.0/libusb.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

E_OsdkStat OsdkLinux_UartSendData(const T_HalObj *obj, const uint8_t *pBuf, uint32_t bufLen);
E_OsdkStat OsdkLinux_UartReadData(const T_HalObj *obj, uint8_t *pBuf, uint32_t *bufLen);
E_OsdkStat OsdkLinux_UartInit(const char *port, const int baudrate, T_HalObj *obj);
E_OsdkStat OsdkLinux_UartClose(T_HalObj *obj);

#ifdef ADVANCED_SENSING
E_OsdkStat OsdkLinux_USBBulkInit(uint16_t pid, uint16_t vid, uint16_t num, uint16_t epIn,
                                 uint16_t epOut, T_HalObj *obj);
E_OsdkStat OsdkLinux_USBBulkSendData(const T_HalObj *obj, const uint8_t *pBuf,
                                     uint32_t bufLen);
E_OsdkStat OsdkLinux_USBBulkReadData(const T_HalObj *obj, uint8_t *pBuf,
                                     uint32_t *bufLen);
E_OsdkStat OsdkLinux_USBBulkClose(T_HalObj *obj);
#endif

#ifdef __cplusplus
}
#endif

#endif // OSDK_HAL_LINUX_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
