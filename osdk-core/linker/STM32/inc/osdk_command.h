/**
 ********************************************************************
 * @file    osdk_command.h
 * @version V1.0.0
 * @date    2019/09/15
 * @brief   This is the header file for "osdk_command.c", defining the structure
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
#ifndef OSDK_COMMAND_H
#define OSDK_COMMAND_H

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>
#include "osdk_routetable.h"
#include "osdk_typedef.h"
#include "osdk_protocol_common.h"
#include "osdk_platform.h"
#include "osdk_command.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
#define PROT_MAX_SUPPORT_CMD_SET 32
#define PROT_MAX_WAIT_ACK_LIST 32

#define PROT_CMD_ITEM(hostId, deviceId, set, id, maskId, data, func)      \
  {                                                                       \
    hostId, deviceId, set, id, maskId, data, func                         \
  }

/* Exported types ------------------------------------------------------------*/
typedef enum {
  MASK_HOST_DEVICE_SET_ID = 0xFFFFFFFF,
  MASK_HOST_XXXXXX_SET_ID = 0xFF00FFFF,
  MASK_HOST_DEVICE_SET_XX = 0xFFFFFF00,
  MASK_HOST_DEVICE_XXX_XX = 0xFFFF0000,
  MASK_HOST_XXXXXX_XXX_XX = 0xFF000000,
} E_RecvCmdHandleListMask;

struct _CommandHandle;

typedef struct _ProtCmdItem {
  uint8_t host;
  uint8_t device;
  uint8_t cmdSet;
  uint8_t cmdId;
  E_RecvCmdHandleListMask mask;
  void    *userData;
  E_OsdkStat (*pFunc)(struct _CommandHandle *cmdHandle,
                      const T_CmdInfo *cmdInfo,
                      const uint8_t *cmdData, void *userData);
} T_RecvCmdItem;

typedef struct {
  T_RecvCmdItem *cmdList;
  E_ProtocolType protoType;
  uint16_t cmdCount;
} T_RecvCmdHandle;

typedef void (*Command_SendCallback)(const T_CmdInfo *cmdInfo,
                                     const uint8_t *cmdData,
                                     void *userData, E_OsdkStat cb_type);

typedef struct {
  bool isValid;
  uint32_t sendTimeStamp;
  uint32_t timeOut;
  uint16_t retryTimes;
  T_CmdInfo sendInfo;
  uint8_t sendData[OSDK_PACKAGE_MAX_LEN];
  Command_SendCallback callback;
  void *userData;
} T_CmdWaitAckItem;

typedef struct _CmdInitConf { uint8_t encKey[32]; } T_CmdInitConf;

typedef struct _CommandHandle {
  T_CmdInitConf conf;
  uint8_t recvDataBuff[OSDK_PACKAGE_MAX_LEN];
  T_RecvCmdHandle recvCmdHandleList[PROT_MAX_SUPPORT_CMD_SET];
  uint16_t recvCmdHandleListCount;
  T_OsdkMutexHandle waitAckItemMutex;
  T_OsdkMutexHandle recvCmdHandleListMutex;
  T_CmdWaitAckItem waitAckItem[PROT_MAX_WAIT_ACK_LIST];
} T_CmdHandle;

/* Exported functions --------------------------------------------------------*/
E_OsdkStat OsdkCommand_Init(T_CmdHandle *cmdHandle, const T_CmdInitConf *conf);
E_OsdkStat OsdkCommand_DeInit(T_CmdHandle *cmdHandle);
E_OsdkStat OsdkCommand_CheckLiveViewChannel(void);
#if defined(__linux__)
E_OsdkStat OsdkCommand_CreateMopTask(void);
E_OsdkStat OsdkCommand_DestroyMopTask(void);
#endif
E_OsdkStat OsdkCommand_CreateLiveViewTask(void);
E_OsdkStat OsdkCommand_DestroyLiveViewTask(void);
E_OsdkStat OsdkCommand_CreateAdvancedSensingTask(void);
E_OsdkStat OsdkCommand_DestroyAdvancedSensingTask(void);

E_OsdkStat OsdkCommand_Send(T_CmdInfo *cmdInfo, const uint8_t *cmdData);
E_OsdkStat OsdkCommand_BigDataSend(T_CmdInfo *cmdInfo, const uint8_t *cmdData);

E_OsdkStat OsdkCommand_RegRecvCmdHandler(T_CmdHandle *cmdHandle,
                                         T_RecvCmdHandle *recvCmdHandle);
void OsdkCommand_SendAsync(T_CmdHandle *cmdHandle, T_CmdInfo *cmdInfo,
                                 const uint8_t *cmdData,
                                 Command_SendCallback func, void *userData,
                                 uint32_t timeOut, uint16_t retryTimes);
E_OsdkStat OsdkCommand_SendSync(T_CmdHandle *cmdHandle, T_CmdInfo *cmdInfo,
                                const uint8_t *cmdData, T_CmdInfo *ackInfo,
                                uint8_t *ackData, uint32_t timeOut,
                                uint16_t retryTimes);
E_OsdkStat OsdkCommand_SendAckData(const T_CmdInfo *cmdInfo, const uint8_t *ackData,
                                   uint16_t ackDataLen);
E_OsdkStat OsdkCommand_SetKey(const char *key);

#ifdef __cplusplus
}
#endif

#endif  // OSDK_COMMAND_H
        /************************ (C) COPYRIGHT DJI Innovations *******END OF
         * FILE******/
