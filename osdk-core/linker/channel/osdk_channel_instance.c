/**
 ********************************************************************
 * @file    osdk_channel_instance.c
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
#include "osdk_channel_instance.h"

/* Private constants ---------------------------------------------------------*/
static T_ChannelListItem s_chnListItem;
static T_msgQueue *s_msgQueuePointer;

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/
static E_OsdkStat OsdkChannel_MsgQueueInit(void);

/* Exported functions definition ---------------------------------------------*/
T_ChannelListItem *OsdkChannel_GetListInstance(void) { return &s_chnListItem; }

E_OsdkStat OsdkChannel_InitInstance(void) {
  if (OsdkChannel_MsgQueueInit() != OSDK_STAT_OK) {
    return OSDK_STAT_ERR;
  }
  return OsdkChannel_ChannelListInit(&s_chnListItem);
}

T_msgQueue *OsdkChannel_GetMsgqInstance(void) { return s_msgQueuePointer; }

/* Private functions definition-----------------------------------------------*/
static E_OsdkStat OsdkChannel_MsgQueueInit(void) {
  T_msgqAttrib attrib;
  attrib.name = "channelMsgQueue";
  attrib.bufSize = MSGQ_SIZE;
  return OsdkMsgq_Create(&attrib, &s_msgQueuePointer);
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
