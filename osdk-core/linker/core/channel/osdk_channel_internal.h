/**
 ********************************************************************
 * @file    osdk_channel_internal.h
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
#ifndef OSDK_CHANNEL_INTERNAL_H
#define OSDK_CHANNEL_INTERNAL_H

/* Includes ------------------------------------------------------------------*/
#include "osdk_hal.h"
#include "osdk_msgq.h"
#include "osdk_osal.h"
#include "osdk_protocol.h"
#include "osdk_routetable.h"
#include "osdk_typedef.h"
#include "osdk_work.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
#define CHANNEL_MAX_SUPPORT_NUM 8
#define UART_CHANNEL_USB0_INDEX 1
#define UART_CHANNEL_ACM0_INDEX 2

/* Exported types ------------------------------------------------------------*/
struct _ChannelItem;

typedef E_OsdkStat (*ChannelSend)(struct _ChannelItem *channelItem,
                                  const T_CmdInfo *cmdInfo,
                                  const uint8_t *cmdData);

/* @TODO param need to be change*/
typedef E_OsdkStat (*ChannelRecv)(struct _ChannelItem *channelItem,
                                  const T_CmdInfo *cmdInfo,
                                  const uint8_t *cmdData);

typedef struct _ChannelItem {
  T_OsdkList head;
  char *channelName;
  uint32_t channelId;
  uint16_t seqNum;
  uint8_t sendFrameBuff[OSDK_PACKAGE_MAX_LEN];
  T_ProtocolOps protocolOps;
  T_HalObj halObject;
  T_HalOps halOps;
  T_OsdkMutexHandle sendMutex;
  T_OsdkMutexHandle seqMutex;
  ChannelSend Send;
  ChannelRecv Recv;
  T_CmdParse recvParse;
  void *protocolExtData;
} T_ChannelItem;

typedef struct {
  T_OsdkList head;
  uint32_t chnCount;
} T_ChannelListItem;

typedef struct {
  uint32_t channelId;
  E_ProtocolType type;
} T_ProtocolMapKey;
/* Exported functions --------------------------------------------------------*/
E_OsdkStat OsdkChannel_CheckDuplicate(T_ChannelListItem *chnListCtx,
                                      T_ChannelItem *channelItem);
E_OsdkStat OsdkChannel_AddChannel(T_ChannelItem *channelItem);
E_OsdkStat OsdkChannel_ChannelListInit(T_ChannelListItem *chnListCtx);
E_OsdkStat OsdkChannel_GetChannelItemByChnId(T_ChannelListItem *chnListCtx,
                                             uint32_t channelId,
                                             T_ChannelItem **channelItem);

E_OsdkStat OsdkChannel_GetProtoOps(const uint32_t channelId,
                                   T_ProtocolOps *ops);
E_OsdkStat OsdkChannel_GetSeqNum(T_ChannelItem *channelItem, uint16_t *seq);

E_OsdkStat OsdkChannel_CommonSend(T_ChannelItem *channelItem,
                                  const T_CmdInfo *cmdInfo,
                                  const uint8_t *cmdData);
E_OsdkStat OsdkChannel_CommonRecv(T_ChannelItem *channelItem,
                                  const T_CmdInfo *cmdInfo,
                                  const uint8_t *cmdData);
T_msgQueue *OsdkChannel_GetMsgqInstance(void);
T_ChannelListItem *OsdkChannel_GetListInstance(void);

#ifdef __cplusplus
}
#endif

#endif  // OSDK_CHANNEL_INTERNAL_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
