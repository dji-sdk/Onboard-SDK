/**
 ********************************************************************
 * @file    osdk_command.c
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
 * further disseminate the information, and you must immediately remove
 *thechannel recv error
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "osdk_command.h"
#include "osdk_root_task.h"
#include "osdk_util.h"
#include "osdk_channel_internal.h"
#include "osdk_channel.h"
#include "osdk_logger_internal.h"
#include "osdk_msgq.h"
#include "osdk_osal.h"
#include "osdk_protocol.h"
#include "osdk_work.h"

/* Private constants ---------------------------------------------------------*/
#define OSDK_Send_POLL_TASK_FREQ (100)

static T_OsdkTaskHandle s_osdkRecvThread;
static T_OsdkTaskHandle s_osdkSendPollThread;
/* Private types -------------------------------------------------------------*/
typedef struct {
  T_OsdkSemHandle waitAckSemaphore;
  E_OsdkStat ackCbType;
  T_CmdInfo ackInfo;
  uint8_t ackData[OSDK_PACKAGE_MAX_LEN];
} T_CmdSyncInfo;

typedef struct {
  uint32_t addr;
  uint32_t mask;
  uint32_t channelId;
} T_RouteKey;
/* Private functions declaration ---------------------------------------------*/
static E_OsdkStat OsdkCommand_Resend(T_CmdInfo *cmdInfo, const uint8_t *cmdData);
static E_OsdkStat OsdkCommand_SendAckData(T_CmdInfo *cmdInfo, const uint8_t *ackData,
                                   uint16_t ackDataLen);
static E_OsdkStat OsdkCommand_DealCmd(T_CmdHandle *cmdHandle,
                                      T_CmdInfo *cmdInfo, uint8_t *cmdData);
static void OsdkCommand_SendPoll(T_CmdHandle *cmdHandle);
static void *OsdkCommand_SendPollTask(void *arg);
static void *OsdkCommand_RecvTask(void *arg);
static void OsdkCommand_SendSyncCallback(const T_CmdInfo *cmdInfo,
                                               const uint8_t *cmdData,
                                               void *userdata, E_OsdkStat cb_type);
static E_OsdkStat OsdkCommand_GetPackageFromMsgq(T_msgQueue *msgq,
                                                 T_CmdInfo *pInfo,
                                                 uint8_t *dataBuf);
static E_OsdkStat OsdkCommand_GetChannelItemByAddr(uint32_t userAddr,
                                                   T_ChannelItem **channelItem);

/* Private variables ---------------------------------------------------------*/
const static T_RouteKey routeTable[] = {
    {GEN_ADDR_INDEX_ONLY(ADDR_SDK_COMMAND_INDEX), ADDR_INDEX_ONLY_MASK,
     FC_UART_CHANNEL_ID},
    {GEN_ADDR_INDEX_ONLY(ADDR_V1_COMMAND_INDEX), ADDR_INDEX_ONLY_MASK,
     USB_ACM_CHANNEL_ID}};

/* Exported functions definition ---------------------------------------------*/

/* @TODO To be completed */
E_OsdkStat OsdkCommand_Init(T_CmdHandle *cmdHandle, const T_CmdInitConf *conf) {
  E_OsdkStat osdkStat;
  if(!cmdHandle || !conf) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "OsdkCommand_Init param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  memcpy(&(cmdHandle->conf), conf, sizeof(T_CmdInitConf));

  memset(cmdHandle->recvDataBuff, 0x00, sizeof(cmdHandle->recvDataBuff));

  memset(cmdHandle->recvCmdHandleList, 0x00,
         sizeof(cmdHandle->recvCmdHandleList));
  cmdHandle->recvCmdHandleListCount = 0;

  if (OsdkOsal_MutexCreate(&cmdHandle->waitAckItemMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "mutex create error");
    return OSDK_STAT_ERR;
  }

  if (OsdkOsal_MutexCreate(&cmdHandle->recvCmdHandleListMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "mutex create error");
    return OSDK_STAT_ERR;
  }

  osdkStat = OsdkOsal_TaskCreate(&s_osdkRecvThread, OsdkCommand_RecvTask,
                                 OSDK_TASK_STACK_SIZE_DEFAULT, cmdHandle);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "recv task create error:%d", osdkStat);
    return osdkStat;
  }

  osdkStat = OsdkOsal_TaskCreate(&s_osdkSendPollThread, OsdkCommand_SendPollTask,
                                 OSDK_TASK_STACK_SIZE_DEFAULT, cmdHandle);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "send poll task create error:%d", osdkStat);
    return osdkStat;
  }

  return OSDK_STAT_OK;
}

/* @TODO To be completed */
E_OsdkStat OsdkCommand_DeInit() {
  OsdkOsal_TaskDestroy(s_osdkRecvThread);
  OsdkOsal_TaskDestroy(s_osdkSendPollThread);
  return OSDK_STAT_OK;
}

/**
 * @brief send function that does not require resend and ack.
 * @param cmdInfo: pointer to command information.
 * @param cmdData: pointer to command data.
 * @return error code.
 */
E_OsdkStat OsdkCommand_Send(T_CmdInfo *cmdInfo, const uint8_t *cmdData) {
  T_ChannelItem *channelItem;
  uint64_t time;

  if(!cmdInfo || !cmdData) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "OsdkCommand_Send param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  if (OsdkCommand_GetChannelItemByAddr(cmdInfo->addr, &channelItem) !=
      OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "get channel item error");
    return OSDK_STAT_ERR;
  }
  cmdInfo->channelId = channelItem->channelId;
  if (OsdkChannel_GetSeqNum(channelItem, &cmdInfo->seqNum) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "get seq num failed!");
    return OSDK_STAT_ERR;
  }

  if (channelItem->Send(channelItem, cmdInfo, cmdData) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "channel send failed!");
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkCommand_Resend(T_CmdInfo *cmdInfo, const uint8_t *cmdData) {
  T_ChannelItem *channelItem;

  if(!cmdInfo || !cmdData) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "OsdkCommand_Resend param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  if (OsdkChannel_GetChannelItemByChnId(OsdkChannel_GetListInstance(),
                                        cmdInfo->channelId,
                                        &channelItem) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "can't get channel item");
    return OSDK_STAT_ERR;
  }

  if (channelItem->Send(channelItem, cmdInfo, cmdData) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "channel send failed!");
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Async send function.
 * @param cmdHandle: pointer to command handle structure.
 * @param cmdInfo: pointer to command information.
 * @param cmdData: pointer to command data.
 * @param func: user callback function.
 * @param userData: pointer to user data;
 * @param timeOut: send timeout variable.
 * @param retryTimes: send retry times.
 * @return void.
 */
void OsdkCommand_SendAsync(T_CmdHandle *cmdHandle, T_CmdInfo *cmdInfo,
                                 const uint8_t *cmdData,
                                 Command_SendCallback func, void *userData,
                                 uint32_t timeOut, uint16_t retryTimes) {
  E_OsdkStat result = OSDK_STAT_OK;
  int index;

  if(!cmdHandle || !cmdInfo) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "OsdkCommand_SendAsync param check failed");
    result = OSDK_STAT_ERR_PARAM;
    goto err;
  }

  if (cmdInfo->packetType != OSDK_COMMAND_PACKET_TYPE_REQUEST ||
      cmdInfo->needAck == OSDK_COMMAND_NEED_ACK_NO_NEED || timeOut == 0) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "OsdkCommand_SendAsync param error");
    result = OSDK_STAT_ERR_PARAM;
    goto err;
  }

  if (OsdkCommand_Send(cmdInfo, cmdData) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "OsdkCommand_Send error");
    result = OSDK_STAT_ERR;
    goto err;
  }

  if (OsdkOsal_MutexLock(cmdHandle->waitAckItemMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "mutex lock error");
    result = OSDK_STAT_ERR;
    goto err;
  }

  for (index = 0; index < PROT_MAX_WAIT_ACK_LIST; index++) {
    if (!cmdHandle->waitAckItem[index].isValid) {
      if (OsdkOsal_GetTimeMs(&cmdHandle->waitAckItem[index].sendTimeStamp) !=
          OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "get system time error");
        result = OSDK_STAT_SYS_ERR;
        goto err;
      }
      cmdHandle->waitAckItem[index].timeOut = timeOut;
      cmdHandle->waitAckItem[index].retryTimes = retryTimes;
      memcpy(&cmdHandle->waitAckItem[index].sendInfo, cmdInfo,
             sizeof(T_CmdInfo));
      memcpy(&cmdHandle->waitAckItem[index].sendData[0], &cmdData[0],
             cmdInfo->dataLen);

      cmdHandle->waitAckItem[index].callback = func;
      cmdHandle->waitAckItem[index].userData = userData;

      cmdHandle->waitAckItem[index].isValid = true;
      break;
    }
  }

  if (OsdkOsal_MutexUnlock(cmdHandle->waitAckItemMutex) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "mutex unlock error");
    result = OSDK_STAT_ERR;
    goto err;
  }

  if (index == PROT_MAX_WAIT_ACK_LIST) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND,
                   "not have enough resource for async list");
    result = OSDK_STAT_SYS_ERR;
    goto err;
  }
  
  return;

err:
  if(func) {
    func(cmdInfo, cmdData, userData, result);
  }
}

/**
 * @brief Sync send function.
 * @param cmdHandle: pointer to command handle structure.
 * @param cmdInfo: pointer to command information.
 * @param cmdData: pointer to command data.
 * @param ackInfo: pointer to store ack information.
 * @param ackData: pointer to store ack data;
 * @param timeOut: send timeout variable.
 * @param retryTimes: send retry times.
 * @return error code.
 */
E_OsdkStat OsdkCommand_SendSync(T_CmdHandle *cmdHandle, T_CmdInfo *cmdInfo,
                                const uint8_t *cmdData, T_CmdInfo *ackInfo,
                                uint8_t *ackData, uint32_t timeOut,
                                uint16_t retryTimes) {
  E_OsdkStat osdkStat = OSDK_STAT_ERR;
  T_CmdSyncInfo syncInfo = {0};

  if(!cmdHandle || !cmdInfo || !ackInfo || !ackData) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "OsdkCommand_SendSync param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  if (cmdInfo->packetType != OSDK_COMMAND_PACKET_TYPE_REQUEST ||
      cmdInfo->needAck == OSDK_COMMAND_NEED_ACK_NO_NEED || timeOut == 0) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "OsdkCommand_SendSync param error!");
    return OSDK_STAT_ERR_PARAM;
  }

  if (OsdkOsal_SemaphoreCreate(&syncInfo.waitAckSemaphore, 0) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "sendSync semaphore create error");
    return OSDK_STAT_ERR;
  }

  OsdkCommand_SendAsync(cmdHandle, cmdInfo, cmdData,
                        OsdkCommand_SendSyncCallback, &syncInfo, timeOut,
                        retryTimes);

  if (OsdkOsal_SemaphoreWait(syncInfo.waitAckSemaphore) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "sendSync wait semaphore failed");
    osdkStat = OSDK_STAT_ERR_TIMEOUT;
    goto out;
  }

  if (syncInfo.ackCbType == OSDK_STAT_ERR_TIMEOUT) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "sendSync callback timeout");
    osdkStat = OSDK_STAT_ERR_TIMEOUT;

  } else if (syncInfo.ackCbType == OSDK_STAT_OK) {
    OSDK_LOG_DEBUG(MODULE_NAME_COMMAND, "sendSync callback success");
    osdkStat = OSDK_STAT_OK;

    *ackInfo = syncInfo.ackInfo;
    memcpy(ackData, syncInfo.ackData, syncInfo.ackInfo.dataLen);
  }

out:
  if (OsdkOsal_SemaphoreDestroy(syncInfo.waitAckSemaphore) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "sendSync semaphore destroy error");
    return OSDK_STAT_ERR;
  }

  return osdkStat;
}

/**
 * @brief function to register command handler.
 * @param recvCmdHandle: pointer to command handler.
 * @return error code.
 */
E_OsdkStat OsdkCommand_RegRecvCmdHandler(T_CmdHandle *cmdHandle,
                                         T_RecvCmdHandle *recvCmdHandle) {
  int i, j, k;

  if(!cmdHandle || !recvCmdHandle) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "OsdkCommand_RegRecvCmdHandler param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  // check cmdHandle itself register error
  for (i = 0; i < recvCmdHandle->cmdCount; i++) {
    for (j = i + 1; j < recvCmdHandle->cmdCount; j++) {
      if (recvCmdHandle->cmdList[i].host == recvCmdHandle->cmdList[j].host &&
          recvCmdHandle->cmdList[i].device ==
              recvCmdHandle->cmdList[j].device &&
          recvCmdHandle->cmdList[i].cmdSet ==
              recvCmdHandle->cmdList[j].cmdSet &&
          recvCmdHandle->cmdList[i].cmdId == recvCmdHandle->cmdList[j].cmdId &&
          recvCmdHandle->cmdList[i].mask == recvCmdHandle->cmdList[j].mask) {
        OSDK_LOG_ERROR(
            MODULE_NAME_COMMAND,
            "Please check reg itself error: cmdSet:0x%02X cmdId:0x%02X",
            recvCmdHandle->cmdList[i].cmdSet, recvCmdHandle->cmdList[i].cmdId);
        return OSDK_STAT_ERR_PARAM;
      }
    }
  }

  for (i = 0; i < cmdHandle->recvCmdHandleListCount; i++) {
    if (cmdHandle->recvCmdHandleList[i].protoType == recvCmdHandle->protoType) {
      for (j = 0; j < cmdHandle->recvCmdHandleList[i].cmdCount; j++) {
        for (k = 0; k < recvCmdHandle->cmdCount; k++) {
          if (cmdHandle->recvCmdHandleList[i].cmdList[j].cmdSet ==
                  recvCmdHandle->cmdList[k].cmdSet &&
              cmdHandle->recvCmdHandleList[i].cmdList[j].cmdSet != 0) {
            break;
          }
        }
      }
    }
  }

  if (i == cmdHandle->recvCmdHandleListCount) {
    if (cmdHandle->recvCmdHandleListCount == PROT_MAX_SUPPORT_CMD_SET) {
      OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "Not have enough resource");
      return OSDK_STAT_ERR;
    } else {

      /*! lock recvCmdHandleList */
      if (OsdkOsal_MutexLock(cmdHandle->recvCmdHandleListMutex)
          != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "mutex lock error");
        return OSDK_STAT_ERR;
      }

      /*! add new cmd handle in recvCmdHandleList */
      cmdHandle->recvCmdHandleList[i] = *recvCmdHandle;
      cmdHandle->recvCmdHandleListCount++;

      /*! unlock recvCmdHandleList */
      if (OsdkOsal_MutexUnlock(cmdHandle->recvCmdHandleListMutex)
          != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "mutex unlock error");
        return OSDK_STAT_ERR;
      }
    }
  } else {

    /*! lock recvCmdHandleList */
    if (OsdkOsal_MutexLock(cmdHandle->recvCmdHandleListMutex) != OSDK_STAT_OK) {
      OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "mutex lock error");
      return OSDK_STAT_ERR;
    }

    /*! replace cmd handle in recvCmdHandleList */
    cmdHandle->recvCmdHandleList[i] = *recvCmdHandle;

    /*! unlock recvCmdHandleList */
    if (OsdkOsal_MutexUnlock(cmdHandle->recvCmdHandleListMutex)
        != OSDK_STAT_OK) {
      OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "mutex unlock error");
      return OSDK_STAT_ERR;
    }
  }

  return OSDK_STAT_OK;
}

/**
 * @brief Get channel item by addr.
 * @param userAddr: user input addr.
 * @param channelItem: pointer to store channel item.
 * @return error code.
 */
static E_OsdkStat OsdkCommand_GetChannelItemByAddr(
    uint32_t userAddr, T_ChannelItem **channelItem) {
  int i = 0;
  uint32_t channelId = 0;

  if(!channelItem) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "OsdkCommand_GetChannelItemByAddr param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  for (i = 0; i < sizeof(routeTable) / sizeof(routeTable[0]); i++) {
    if ((userAddr & routeTable[i].mask) == routeTable[i].addr) {
      channelId = routeTable[i].channelId;
      break;
    }
  }

  if (i == sizeof(routeTable) / sizeof(routeTable[0])) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "can't get channel id");
    return OSDK_STAT_ERR;
  }

  if (OsdkChannel_GetChannelItemByChnId(OsdkChannel_GetListInstance(),
                                        channelId,
                                        channelItem) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "can't get channel item");
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/* Private functions definition-----------------------------------------------*/
/**
 * @brief Deal command function.
 * @param cmdHandle: pointer to command handler structure.
 * @param cmdInfo: pointer to command information.
 * @param cmdData: pointer to command data.
 * @return error code.
 */
static E_OsdkStat OsdkCommand_DealCmd(T_CmdHandle *cmdHandle,
                                      T_CmdInfo *cmdInfo, uint8_t *cmdData) {
  bool findHandleFlag;
  int i;
  int j = 0;
  uint64_t timeFuncBefore;
  uint64_t timeFuncAfter;

  if(!cmdHandle || !cmdInfo || !cmdData) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "OsdkCommand_DealCmd param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  if (cmdInfo->packetType == OSDK_COMMAND_PACKET_TYPE_ACK) {
    for (int k = 0; k < PROT_MAX_WAIT_ACK_LIST; k++) {
      if (cmdHandle->waitAckItem[k].isValid == true) {
        if (cmdHandle->waitAckItem[k].sendInfo.seqNum == cmdInfo->seqNum &&
            cmdHandle->waitAckItem[k].sendInfo.cmdSet == cmdInfo->cmdSet &&
            cmdHandle->waitAckItem[k].sendInfo.cmdId == cmdInfo->cmdId) {
          if (OsdkOsal_MutexLock(cmdHandle->waitAckItemMutex) !=
              OSDK_STAT_OK) {
            OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "mutex lock error");
            return OSDK_STAT_ERR;
          }

          cmdHandle->waitAckItem[k].isValid = false;
#ifdef OS_DEBUG
          if (OsdkOsal_GetTimeUs(&timeFuncBefore) != OSDK_STAT_OK) {
            OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "get system time error");
          }
#endif
          cmdHandle->waitAckItem[k].callback(cmdInfo, cmdData,
                                             cmdHandle->waitAckItem[k].userData,
                                             OSDK_STAT_OK);
#ifdef OS_DEBUG
          if (OsdkOsal_GetTimeUs(&timeFuncAfter) != OSDK_STAT_OK) {
            OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "get system time error");
          }

          OSDK_LOG_DEBUG(MODULE_NAME_COMMAND, "Func deal about: %d us",
                        (timeFuncAfter - timeFuncBefore));
#endif
          if (OsdkOsal_MutexUnlock(cmdHandle->waitAckItemMutex) !=
              OSDK_STAT_OK) {
            OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "mutex unlock error");
            return OSDK_STAT_ERR;
          }
        }
      }
    }
  } else if (cmdInfo->packetType == OSDK_COMMAND_PACKET_TYPE_REQUEST) {
    findHandleFlag = false;

    /*! lock recvCmdHandleList */
    if (OsdkOsal_MutexLock(cmdHandle->recvCmdHandleListMutex) != OSDK_STAT_OK) {
      OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "mutex lock error");
      return OSDK_STAT_ERR;
    }

    /*! deal the recvCmdHandleList callback */
    for (i = 0; i < cmdHandle->recvCmdHandleListCount; i++) {
      if (cmdHandle->recvCmdHandleList[i].protoType == cmdInfo->protoType) {
        for (j = 0; j < cmdHandle->recvCmdHandleList[i].cmdCount; j++) {
          if ((*cmdInfo).cmdId ==
                  cmdHandle->recvCmdHandleList[i].cmdList[j].cmdId &&
              (*cmdInfo).cmdSet ==
                  cmdHandle->recvCmdHandleList[i].cmdList[j].cmdSet) {
            findHandleFlag = true;
            if(cmdHandle->recvCmdHandleList[i].cmdList[j].pFunc != NULL) {
		          if (cmdHandle->recvCmdHandleList[i].cmdList[j].pFunc(
		                  cmdHandle, cmdInfo, cmdData, 
		              cmdHandle->recvCmdHandleList[i].cmdList[j].userData)
		            != OSDK_STAT_OK) {
		            OSDK_LOG_ERROR(
		                MODULE_NAME_COMMAND,
		                "cmd handle failure, cmdset = 0x%02x, cmdid = 0x%02x",
		                (*cmdInfo).cmdSet, (*cmdInfo).cmdId);
		          }
            }
            break;
          }
        }
      }
    }

    /*! unlock recvCmdHandleList */
    if (OsdkOsal_MutexUnlock(cmdHandle->recvCmdHandleListMutex)
        != OSDK_STAT_OK) {
      OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "mutex unlock error");
      return OSDK_STAT_ERR;
    }

    if (findHandleFlag == true) {
      OSDK_LOG_DEBUG(MODULE_NAME_COMMAND,
                    "found cmd handle list - table:%d list:%d", i, j);
    }

    //@TODO answer unsupport ack
  }

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkCommand_SendAckData(T_CmdInfo *cmdInfo, const uint8_t *ackData,
                                   uint16_t ackDataLen) {
  T_CmdInfo ackCmdInfo = {0};
  uint8_t *cmdData;
  
  if(!cmdInfo || !ackData) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "OsdkCommand_SendAckData param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  if (ackDataLen > OSDK_PACKAGE_MAX_LEN) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "ack data len out of range");
    return OSDK_STAT_ERR_OUT_OF_RANGE;
  }

  ackCmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_ACK;
  ackCmdInfo.needAck = OSDK_COMMAND_NEED_ACK_NO_NEED;

  ackCmdInfo.channelId = cmdInfo->channelId;
  ackCmdInfo.sender = cmdInfo->receiver;
  ackCmdInfo.receiver = cmdInfo->sender;
  ackCmdInfo.sessionId = cmdInfo->sessionId;
  ackCmdInfo.seqNum = cmdInfo->seqNum;
  ackCmdInfo.encType = cmdInfo->encType;
  ackCmdInfo.cmdSet = cmdInfo->cmdSet;
  ackCmdInfo.cmdId = cmdInfo->cmdId;
  ackCmdInfo.dataLen = ackDataLen;

  if (OsdkCommand_Resend(&ackCmdInfo, ackData) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "ack send failed!");
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/**
 * @brief recv package process func.
 * @param cmdHandle: pointer to command handler structure.
 * @param pData: pointer to data.
 * @param len: data length.
 * @return void.
 */
static void OsdkCommand_SendPoll(T_CmdHandle *cmdHandle) {
  int index;
  uint32_t ms;
  uint64_t timeFuncBefore;
  uint64_t timeFuncAfter;

  if(!cmdHandle) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "OsdkCommand_SendASyncHandle param check failed");
    return;
  }

  for (index = 0; index < PROT_MAX_WAIT_ACK_LIST; index++) {
    if (cmdHandle->waitAckItem[index].isValid) {
      if (OsdkOsal_GetTimeMs(&ms) != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "get system time error");
      }
      if ((ms - cmdHandle->waitAckItem[index].sendTimeStamp) >
          cmdHandle->waitAckItem[index].timeOut) {
        if (OsdkOsal_MutexLock(cmdHandle->waitAckItemMutex) != OSDK_STAT_OK) {
          OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "mutex lock error");
          return;
        }

        if (OsdkOsal_GetTimeMs(&cmdHandle->waitAckItem[index].sendTimeStamp) !=
            OSDK_STAT_OK) {
          OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "get system time error");
        }

        if (cmdHandle->waitAckItem[index].retryTimes > 0) {
          OSDK_LOG_INFO(MODULE_NAME_COMMAND, "Command async send retry:%d %d",
                        index, cmdHandle->waitAckItem[index].retryTimes);

          OsdkCommand_Resend(&cmdHandle->waitAckItem[index].sendInfo,
                             cmdHandle->waitAckItem[index].sendData);
          cmdHandle->waitAckItem[index].retryTimes--;
        } else {
          cmdHandle->waitAckItem[index].isValid = 0;
          OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "Command async send error %d",
                         index);

#ifdef OS_DEBUG
          if (OsdkOsal_GetTimeUs(&timeFuncBefore) != OSDK_STAT_OK) {
            OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "get system time error");
          }
#endif
          cmdHandle->waitAckItem[index].callback(
              NULL, NULL, cmdHandle->waitAckItem[index].userData,
              OSDK_STAT_ERR_TIMEOUT);

#ifdef OS_DEBUG
          if (OsdkOsal_GetTimeUs(&timeFuncAfter) != OSDK_STAT_OK) {
            OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "get system time error");
          }

          OSDK_LOG_INFO(MODULE_NAME_COMMAND, "Timeout func deal about: %d us",
                        (timeFuncAfter - timeFuncBefore));
#endif
        }

        if (OsdkOsal_MutexUnlock(cmdHandle->waitAckItemMutex) !=
            OSDK_STAT_OK) {
          OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "mutex unlock error");
          return;
        }
      }
    }
  }
}

/**
 * @brief Sync send callback function.
 * @param cmdInfo: pointer to command information.
 * @param cmdData: pointer to command data.
 * @param userdata: pointer to user data.
 * @param cb_type: variable that indicates the callback function execution
 * status.
 * @return error code.
 */
static void OsdkCommand_SendSyncCallback(const T_CmdInfo *cmdInfo,
                                               const uint8_t *cmdData,
                                               void *userdata,
                                               E_OsdkStat cb_type) {
  T_CmdSyncInfo *syncInfo = (T_CmdSyncInfo *)userdata;

  if (cb_type == OSDK_STAT_OK) {
    OSDK_LOG_DEBUG(MODULE_NAME_COMMAND, "OsdkCommand_SendSyncCallback success");
    memcpy(&syncInfo->ackInfo, cmdInfo, sizeof(T_CmdInfo));
    memcpy(&syncInfo->ackData, cmdData, cmdInfo->dataLen);
  }

  syncInfo->ackCbType = cb_type;
  OsdkOsal_SemaphorePost(syncInfo->waitAckSemaphore);
}

static E_OsdkStat OsdkCommand_GetPackageFromMsgq(T_msgQueue *msgq,
                                                 T_CmdInfo *pInfo,
                                                 uint8_t *dataBuf) {
  uint32_t size;
  uint32_t timeOut = OSDK_WAIT_FOREVER;

  size = sizeof(T_CmdInfo);

  if(!msgq || !pInfo || !dataBuf) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "OsdkCommand_GetPackageFromMsgq param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  if (OsdkMsgq_Recv(msgq, (void *)pInfo, &size, timeOut) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "msgq recv info failed");
    return OSDK_STAT_ERR;
  }

  if (size != sizeof(T_CmdInfo)) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "message size check failed, size = %u", size);
    return OSDK_STAT_ERR;
  }

  if (pInfo->dataLen > 0) {
    size = pInfo->dataLen;
    if (size > 1024) {
      OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "msgq data size error");
      return OSDK_STAT_ERR;
    }

    if (OsdkMsgq_Recv(msgq, (void *)(dataBuf), &size, timeOut) !=
        OSDK_STAT_OK) {
      OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "msgq recv data failed");
      return OSDK_STAT_ERR;
    }
    if (size != pInfo->dataLen) {
      OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "data size check failed");
      return OSDK_STAT_ERR;
    }
  }
  return OSDK_STAT_OK;
}

E_OsdkStat OsdkCommand_SetKey(const char *key)
{
  T_ChannelItem *channelItem;
  if (OsdkCommand_GetChannelItemByAddr(GEN_ADDR_INDEX_ONLY(ADDR_SDK_COMMAND_INDEX),
                                       &channelItem) != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "get channel item error");
    return OSDK_STAT_ERR;
  }
  channelItem->protocolOps.SetKey(key);
  return OSDK_STAT_OK;
  
}

/* @TODO need to be fixed */
/**
 * @brief Recv task.
 * @param arg: pointer to task argument.
 * @return void.
 */
void *OsdkCommand_RecvTask(void *arg) {

  if(!arg) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "OsdkCommand_RecvTask param check failed");
    return 0;
  }
  T_CmdHandle *cmdHandle = (T_CmdHandle *)arg;
  T_CmdInfo cmdInfo = {0};

  while (1) {

    if (OsdkCommand_GetPackageFromMsgq(OsdkChannel_GetMsgqInstance(), &cmdInfo,
                                       cmdHandle->recvDataBuff) !=
        OSDK_STAT_OK) {
      OSDK_LOG_WARN(MODULE_NAME_COMMAND, "get package failed");
    } else {
      if (OsdkCommand_DealCmd(cmdHandle, &cmdInfo, cmdHandle->recvDataBuff) !=
          OSDK_STAT_OK) {
        OSDK_LOG_WARN(MODULE_NAME_COMMAND, "deal command failed");
      }
    }
  }
  return 0;
}

void *OsdkCommand_SendPollTask(void *arg) {

  if(!arg) {
    OSDK_LOG_ERROR(MODULE_NAME_COMMAND, "OsdkCommand_SendPollTask param check failed");
    return 0;
  }
  T_CmdHandle *cmdHandle = (T_CmdHandle *)arg;

  while (1) {
    OsdkOsal_TaskSleepMs(1000 / OSDK_Send_POLL_TASK_FREQ);
    OsdkCommand_SendPoll(cmdHandle);
  }
  return 0;
}


/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
