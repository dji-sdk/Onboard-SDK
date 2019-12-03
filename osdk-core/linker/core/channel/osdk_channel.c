/**
 ********************************************************************
 * @file    osdk_channel.c
 * @version V1.0.0
 * @date    2019/09/17
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
#include <string.h>
#include "osdk_channel.h"
#include "osdk_channel_internal.h"
#include "osdk_root_task.h"
#include "osdk_util.h"

/* Private constants ---------------------------------------------------------*/
#define OSDK_CHANNEL_TASK_FREQ (1000)

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static T_ChannelListItem s_chnListItem;
const static T_ProtocolMapKey protoMapTable[] = {
    {FC_UART_CHANNEL_ID, PROTOCOL_SDK},
    {USB_ACM_CHANNEL_ID, PROTOCOL_V1}};

static T_msgQueue *s_msgQueuePointer;
static T_OsdkWorkNode s_osdkChannelWorkNode = {0};
static uint32_t s_osdkChannelWorkStep = 0;

/* Private functions declaration ---------------------------------------------*/
static E_OsdkStat OsdkChannel_MsgQueueInit(void);
static void OsdkChannel_Task(void *arg);
static E_OsdkStat OsdkChannel_RecvProcess(T_ChannelItem *channelItem,
                                          const uint8_t *pData, uint16_t len);

/* Exported functions definition ---------------------------------------------*/

/* Private functions definition-----------------------------------------------*/

/**
 * @brief Check if there are duplicates in channel list.
 * @param chnListCtx: pointer to channel list.
 * @param channelItem: pointer to channel item.
 * @return error code.
 */
E_OsdkStat OsdkChannel_CheckDuplicate(T_ChannelListItem *chnListCtx,
                                      T_ChannelItem *channelItem) {
  T_ChannelItem *pNode = NULL;
  T_ChannelItem *pBackup = NULL;

  if (!chnListCtx || !channelItem) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL,
                  "OsdkChannel_CheckDuplicate param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  if (chnListCtx->chnCount > 0) {
    OSDK_LIST_FOR_EACH_ENTRY_SAFE(pNode, pBackup, &chnListCtx->head, head) {
      if (channelItem->channelId == pNode->channelId) {
        OSDK_LOG_WARN(MODULE_NAME_CHANNEL, "channel id exist: %d",
                      channelItem->channelId);
        return OSDK_STAT_ERR;
      }
    }
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Add channel item to channel list
 * @param channelItem: pointer to channel item.
 * @return error code.
 */
E_OsdkStat OsdkChannel_AddChannel(T_ChannelItem *channelItem) {
  E_OsdkStat osdkStat;

  T_ChannelListItem *chnListCtx = NULL;

  if(!channelItem) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL,
                  "OsdkChannel_AddChannel param check failed");
    return OSDK_STAT_ERR_PARAM;
  }
  chnListCtx = OsdkChannel_GetListInstance();

  osdkStat = OsdkChannel_CheckDuplicate(chnListCtx, channelItem);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "osdk channel add new node error:%d",
                   osdkStat);
    return osdkStat;
  }
  if (chnListCtx->chnCount >= CHANNEL_MAX_SUPPORT_NUM) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "do not have enough resource");
    return OSDK_STAT_ERR;
  }
  OsdkList_AddTail(&channelItem->head, &chnListCtx->head);

  chnListCtx->chnCount++;
  OSDK_LOG_INFO(MODULE_NAME_CHANNEL, "channel list add new node success:%d",
                chnListCtx->chnCount);

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkChannel_ChannelListInit(T_ChannelListItem *chnListCtx) {
  E_OsdkStat osdkStat;

  if(!chnListCtx) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL,
                  "OsdkChannel_ChannelListInit param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  OsdkList_Init(&chnListCtx->head);
  chnListCtx->chnCount = 0;
  s_osdkChannelWorkNode.name = "channelTask";
  s_osdkChannelWorkNode.taskFunc = OsdkChannel_Task;
  s_osdkChannelWorkNode.arg = (void *)NULL;

  osdkStat =
      OsdkWork_AddNode(OsdkCore_GetWorkInstance(), &s_osdkChannelWorkNode);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "create command task error:%d",
                   osdkStat);
  }
  return osdkStat;
}

/**
 * @brief Get channel item by channel id.
 * @param chnListCtx: pointer to channel list.
 * @param channelId: target channel id.
 * @param channelItem: pointer to channel item.
 * @return error code.
 */
E_OsdkStat OsdkChannel_GetChannelItemByChnId(T_ChannelListItem *chnListCtx,
                                             uint32_t channelId,
                                             T_ChannelItem **channelItem) {
  T_ChannelItem *pNode = NULL;
  T_ChannelItem *pBackup = NULL;

  if (!chnListCtx) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "GetChannelItemByDevId param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  if (chnListCtx->chnCount > 0) {
    OSDK_LIST_FOR_EACH_ENTRY_SAFE(pNode, pBackup, &chnListCtx->head, head) {
      if (channelId == pNode->channelId) {
        *channelItem = pNode;
        return OSDK_STAT_OK;
      }
    }
  }
  return OSDK_STAT_ERR;
}

/**
 * @brief Init uart channel item.
 * @param port: uart port name.
 * @param baudRate: uart port baud rate.
 * @return error code.
 */
E_OsdkStat OsdkChannel_InitUartChannel(const char *port,
                                       const uint32_t baudRate,
                                       E_ChannelIDType id) {
  T_ChannelItem *channelItem = NULL;

  if(!port) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "OsdkChannel_InitUartChannel param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  channelItem = OsdkOsal_Malloc(sizeof(T_ChannelItem));
  if (channelItem == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "malloc memory for channel fail");
    return OSDK_STAT_ERR_ALLOC;
  }

  channelItem->channelName = "UART";
  channelItem->channelId = id;
  channelItem->seqNum = 0;
  channelItem->recvParse.parseIndex = 0;
  memset(channelItem->recvParse.parseBuff, 0, OSDK_PACKAGE_MAX_LEN);
  channelItem->Send = OsdkChannel_CommonSend;
  channelItem->Recv = OsdkChannel_CommonRecv;
  memset(channelItem->sendFrameBuff, 0x00, sizeof(channelItem->sendFrameBuff));

  if (OsdkHal_GetHalOps("UART", &channelItem->halOps) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "can't get hal ops");
    goto err;
  }

  if (OsdkHal_UartInit(port, baudRate, &channelItem->halObject) !=
      OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "UART init failed");
    goto err;
  }

  if (OsdkChannel_GetProtoOps(channelItem->channelId,
                              &channelItem->protocolOps) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "can't get protocol ops");
    goto err;
  }

  if (channelItem->protocolOps.Init(&channelItem->protocolExtData) !=
      OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "protocol init error");
    goto err;
  }

  if (OsdkOsal_MutexCreate(&channelItem->sendMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "send mutex create error");
    goto err;
  }

  if (OsdkOsal_MutexCreate(&channelItem->seqMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "seq mutex create error");
    goto err;
  }

  if (OsdkChannel_AddChannel(channelItem) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "add uart channel error");
    goto err;
  }

  return OSDK_STAT_OK;

err:
  OsdkOsal_Free(channelItem);
  return OSDK_STAT_ERR;
}

/**
 * @brief Init UDP channel item.
 * @param addr: udp address.
 * @param port: udp port.
 * @return error code.
 */
E_OsdkStat OsdkChannel_InitUDPChannel(const char *addr, uint16_t port,
                                      E_ChannelIDType id) {
  T_ChannelItem *channelItem = NULL;
  E_OsdkStat osdkStat = OSDK_STAT_OK;

  if(!addr) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "OsdkChannel_InitUDPChannel param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  channelItem = OsdkOsal_Malloc(sizeof(T_ChannelItem));
  if (channelItem == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "malloc memory for channel fail");
    return OSDK_STAT_ERR_ALLOC;
  }
  channelItem->channelName = "UDP";
  channelItem->channelId = id;
  channelItem->seqNum = 0;
  channelItem->Send = OsdkChannel_CommonSend;
  channelItem->Recv = OsdkChannel_CommonRecv;
  memset(channelItem->sendFrameBuff, 0x00, sizeof(channelItem->sendFrameBuff));

  if (OsdkHal_GetHalOps("UDP", &channelItem->halOps) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "get hal ops failed");
    goto err;
  }

  if (OsdkHal_UdpInit(addr, port, &channelItem->halObject) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "UART init failed");
    goto err;
  }

  if (OsdkChannel_GetProtoOps(channelItem->channelId,
                              &channelItem->protocolOps) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "can't get protocol ops");
    goto err;
  }

  if (channelItem->protocolOps.Init(&channelItem->protocolExtData) !=
      OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "protocol init error");
    goto err;
  }

  if (OsdkOsal_MutexCreate(&channelItem->sendMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "send mutex create error");
    return OSDK_STAT_ERR;
  }

  if (OsdkOsal_MutexCreate(&channelItem->seqMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "seq mutex create error");
    goto err;
  }

  if (OsdkChannel_AddChannel(channelItem) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "add udp channel error");
    goto err;
  }

  return OSDK_STAT_OK;

err:
  OsdkOsal_Free(channelItem);
  return OSDK_STAT_ERR;
}

/**
 * @brief Get protocol oprations.
 * @param interface: communication interface name.
 * @param ops: pointer to protocol operations.
 * @return error code.
 */
E_OsdkStat OsdkChannel_GetProtoOps(const uint32_t channelId,
                                   T_ProtocolOps *ops) {
  int i = 0;
  E_ProtocolType type;
  
  if(!ops) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "OsdkChannel_GetProtoOps param check failed");
    return OSDK_STAT_ERR_PARAM;
  }
  for (; i < sizeof(protoMapTable) / sizeof(protoMapTable[0]); ++i) {
    if (protoMapTable[i].channelId == channelId) {
      type = protoMapTable[i].type;
      break;
    }
  }
  if (i == sizeof(protoMapTable) / sizeof(protoMapTable[0])) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "can't find channelId");
    return OSDK_STAT_ERR;
  }
  if (OsdkProtocol_getProtocolOps(type, ops) == OSDK_STAT_ERR) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "can't find protocol");
    return OSDK_STAT_ERR;
  }
  return OSDK_STAT_OK;
}

/**
 * @brief Get channel sequence number.
 * @param channelItem: pointer to channel item.
 * @param seq: pointer to store sequence number.
 * @return error code.
 */
E_OsdkStat OsdkChannel_GetSeqNum(T_ChannelItem *channelItem, uint16_t *seq) {

  if(!channelItem || !seq) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "OsdkChannel_GetSeqNum param check failed");
    return OSDK_STAT_ERR_PARAM;
  }
  if (OsdkOsal_MutexLock(channelItem->seqMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "mutex lock error");
    return OSDK_STAT_ERR;
  }
  channelItem->seqNum++;
  *seq = channelItem->seqNum;

  if (OsdkOsal_MutexUnlock(channelItem->seqMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "mutex unlock error");
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/* @TODO To be completed */
E_OsdkStat OsdkChannel_CommonSend(T_ChannelItem *channelItem,
                                  const T_CmdInfo *cmdInfo,
                                  const uint8_t *cmdData) {
  uint16_t length = 0;

  if(!channelItem || !cmdInfo || !cmdData) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "OsdkChannel_CommonSend param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  if (OsdkOsal_MutexLock(channelItem->sendMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "mutex lock error");
    return OSDK_STAT_ERR;
  }

  if (channelItem->protocolOps.Pack(channelItem->protocolExtData,
                                    channelItem->sendFrameBuff, &length,
                                    cmdInfo, cmdData) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "protocol pack error");
    return OSDK_STAT_ERR;
  }

  if (channelItem->halOps.Send(&channelItem->halObject,
                               channelItem->sendFrameBuff,
                               length) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "hal send error");
    return OSDK_STAT_ERR;
  }

  if (OsdkOsal_MutexUnlock(channelItem->sendMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "mutex unlock error");
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/* @TODO To be completed */
E_OsdkStat OsdkChannel_CommonRecv(T_ChannelItem *channelItem,
                                  const T_CmdInfo *cmdInfo,
                                  const uint8_t *cmdData) {
  E_OsdkStat osdkStat;
  uint8_t dataRecv[HAL_ONCE_READ_LEN] = {0};
  uint16_t recvLen = 0;

  if(!channelItem) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "OsdkChannel_CommonRecv param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  osdkStat =
      channelItem->halOps.Read(&channelItem->halObject, dataRecv, &recvLen);
  if (osdkStat == OSDK_STAT_OK) {
    if (recvLen != 0) {
      OsdkChannel_RecvProcess(channelItem, dataRecv, recvLen);
    }
  } else {
    OSDK_LOG_WARN(MODULE_NAME_CHANNEL, "hal read error");
  }

  return osdkStat;
}

static E_OsdkStat OsdkChannel_RecvProcess(T_ChannelItem *channelItem,
                                          const uint8_t *pData, uint16_t len) {
  uint16_t parseLen;
  uint8_t *pFrame;
  E_OsdkStat OsdkStat = OSDK_STAT_OK;
  uint8_t buf[OSDK_PACKAGE_MAX_LEN + sizeof(T_CmdInfo)];
  T_CmdInfo *cmdInfo = (T_CmdInfo *)buf;
  uint8_t *databuf = &buf[sizeof(T_CmdInfo)];
  uint64_t timeFuncBefore;
  uint64_t timeFuncAfter;
  int i;

  if(!channelItem || !pData) {
    OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "OsdkChannel_RecvProcess param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  for (i = 0; i < len; i++) {
    OsdkStat = channelItem->protocolOps.Parse(&channelItem->recvParse, pData[i],
                                              &pFrame, &parseLen);

    if (OsdkStat == OSDK_STAT_OK) {
      if (channelItem->protocolOps.Unpack(channelItem->protocolExtData, pFrame,
                                          cmdInfo, databuf) != OSDK_STAT_OK) {
        OSDK_LOG_INFO(MODULE_NAME_CHANNEL, "protocol unpack error");
      } else {
        cmdInfo->channelId = channelItem->channelId;
#ifdef OS_DEBUG
        if (OsdkOsal_GetTimeUs(&timeFuncBefore) != OSDK_STAT_OK) {
          OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "get system time error");
        }
#endif
        if (OsdkMsgq_Send(OsdkChannel_GetMsgqInstance(), (void *)buf,
                          sizeof(T_CmdInfo) + cmdInfo->dataLen,
                          OSDK_WAIT_ONE_SECOND) != OSDK_STAT_OK) {
          OSDK_LOG_WARN(MODULE_NAME_CHANNEL, "msg send error");
        }
#ifdef OS_DEBUG
        if (OsdkOsal_GetTimeUs(&timeFuncAfter) != OSDK_STAT_OK) {
          OSDK_LOG_ERROR(MODULE_NAME_CHANNEL, "get system time error");
        }
        if((timeFuncAfter - timeFuncBefore) > 1000)
		      OSDK_LOG_DEBUG(MODULE_NAME_CHANNEL, "OsdkMsgq_Send about: %d us",
		                    (timeFuncAfter - timeFuncBefore));
#endif
      }
    } else {
    }
  }

  return OsdkStat;
}

static E_OsdkStat OsdkChannel_MsgQueueInit(void) {
  T_msgqAttrib attrib;
  attrib.name = "channelMsgQueue";
  attrib.bufSize = MSGQ_SIZE;
  return OsdkMsgq_Create(&attrib, &s_msgQueuePointer);
}

T_ChannelListItem *OsdkChannel_GetListInstance(void) { return &s_chnListItem; }

E_OsdkStat OsdkChannel_InitInstance(void) {
  if (OsdkChannel_MsgQueueInit() != OSDK_STAT_OK) {
    return OSDK_STAT_ERR;
  }
  return OsdkChannel_ChannelListInit(&s_chnListItem);
}

T_msgQueue *OsdkChannel_GetMsgqInstance(void) { return s_msgQueuePointer; }

void OsdkChannel_Task(void *arg) {
  T_ChannelItem *pNode = NULL;
  T_ChannelItem *pBackup = NULL;
  T_ChannelListItem *chnListCtx = NULL;

  if (OSDK_UTIL_WORK_RUN(s_osdkChannelWorkStep++, OSDK_CHANNEL_TASK_FREQ,
                         OsdkCore_GetRootTaskFreq())) {
    chnListCtx = OsdkChannel_GetListInstance();
    if (chnListCtx->chnCount > 0) {
      OSDK_LIST_FOR_EACH_ENTRY_SAFE(pNode, pBackup, &chnListCtx->head, head) {
        if (pNode->Recv(pNode, NULL, NULL) != OSDK_STAT_OK) {
          OSDK_LOG_WARN(MODULE_NAME_CHANNEL, "channel recv error");
        }
      }
    }
  }
}
/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
