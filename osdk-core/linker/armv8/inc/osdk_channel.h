/**
 ********************************************************************
 * @file    osdk_channel.h
 * @version V1.0.0
 * @date    2019/09/15
 * @brief   This is the header file for "osdk_channel.c", defining the structure
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
#ifndef OSDK_CHANNEL_H
#define OSDK_CHANNEL_H

/* Includes ------------------------------------------------------------------*/
#include "osdk_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
#define CHANNEL_TYPE_UART 0x1
#define CHANNEL_TYPE_USB 0x2
#define CHANNEL_TYPE_UDP 0x3
#define CHANNEL_TYPE_INVALID 0xFF

#define CHANNEL_MAX_SUPPORT_NUM 8

#define CHANNEL_ID(channel_type, id) \
  (uint32_t)(((channel_type << 16) & 0x00FF0000) | ((id << 8) & 0x0000FF00))


typedef enum {
  FC_UART_CHANNEL_ID = CHANNEL_ID(CHANNEL_TYPE_UART, 1),
  USB_ACM_CHANNEL_ID = CHANNEL_ID(CHANNEL_TYPE_UART, 2),
  RNDIS_UDP_CHANNEL_ID = CHANNEL_ID(CHANNEL_TYPE_UDP, 0),
  USB_BULK_LIVEVIEW_CHANNEL_ID = CHANNEL_ID(CHANNEL_TYPE_USB, 1),
  USB_BULK_ADVANCED_SENSING_CHANNEL_ID = CHANNEL_ID(CHANNEL_TYPE_USB, 2),
} E_ChannelIDType;

/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
E_OsdkStat OsdkChannel_InitInstance(void);
E_OsdkStat OsdkChannel_DeinitInstance(void);

E_OsdkStat OsdkChannel_InitUartChannel(const char *port,
                                       const uint32_t baudRate,
                                       E_ChannelIDType id);
#ifdef __linux__
E_OsdkStat OsdkChannel_InitUSBBulkChannel(uint16_t pid, uint16_t vid, uint16_t num,
                                          uint16_t epIn, uint16_t epOut,
                                          E_ChannelIDType id);
#endif

#ifdef __cplusplus
}
#endif

#endif  // OSDK_CHANNEL_H
        /************************ (C) COPYRIGHT DJI Innovations *******END OF
         * FILE******/
