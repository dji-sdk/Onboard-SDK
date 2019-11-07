/**
 ********************************************************************
 * @file    osdk_timesync.c
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
#include "osdk_timesync.h"
#include "dji_common_define.h"
#include "osdk_device_id.h"
#include "osdk_root_task.h"
#include "osdk_util.h"

/* Private constants ---------------------------------------------------------*/
#define OSDK_TIMESTAMP_OFFSET_MAX_ERR_MS 10
#define OSDK_TIMESTAMP_SYNC_CMD_TIMEOUT_MS 3000
#define OSDK_TIMESTAMP_SYNC_CMD_RETRY_COUNT 1

#define OSDK_TIME_SYNC_TASK_FREQ (1)
/* Private types -------------------------------------------------------------*/
typedef struct {
  int32_t offset;
  int32_t lastOffset;
  int32_t offsetErr;
} T_OsdkLoggerTimeSync;

static T_OsdkLoggerTimeSync s_timeSync;
static T_OsdkWorkNode s_osdkTimeSyncWorkNode = {0};
static uint32_t s_osdkTimeSyncWorkStep = 0;

/* Private functions declaration ---------------------------------------------*/
static void OsdkTimeSync_ASyncCallback(const T_CmdInfo *cmdInfo,
                                             const uint8_t *cmdData,
                                             void *userdata, uint8_t cb_type);
static E_OsdkStat OsdkTimeSync_ASyncRequest(void);
static void OsdkTimeSync_Task(void *arg);

/* Exported functions definition ---------------------------------------------*/
E_OsdkStat OsdkTimeSync_GetSyncTimeMs(uint32_t *syncTimeMs) {
  E_OsdkStat osdkStat;
  uint32_t localTimeMs;

  osdkStat = OsdkOsal_GetTimeMs(&localTimeMs);
  *syncTimeMs = localTimeMs + s_timeSync.offset;

  return osdkStat;
}

E_OsdkStat OsdkTimeSync_Init(void) {
  E_OsdkStat osdkStat;

  s_osdkTimeSyncWorkNode.name = "timeSyncTask";
  s_osdkTimeSyncWorkNode.taskFunc = OsdkTimeSync_Task;

  osdkStat =
      OsdkWork_AddNode(OsdkCore_GetWorkInstance(), &s_osdkTimeSyncWorkNode);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_LOGGER, "create command task error:%d",
                   osdkStat);
    return osdkStat;
  }

  return OSDK_STAT_OK;
}

/* Private functions definition-----------------------------------------------*/
static void OsdkTimeSync_ASyncCallback(const T_CmdInfo *cmdInfo,
                                             const uint8_t *cmdData,
                                             void *userdata, uint8_t cb_type) {
  uint32_t ms;
  E_OsdkStat osdkStat = OSDK_STAT_OK;

  if (cb_type == OSDK_COMMAND_SENDASYNC_SUCCESS) {
    dji_general_get_simple_time_sync_ack *ack =
        (dji_general_get_simple_time_sync_ack *)cmdData;

    OSDK_LOG_DEBUG(MODULE_NAME_LOGGER, "timesync from FC success");
    OSDK_LOG_DEBUG(MODULE_NAME_LOGGER, "%d.%03d", ack->send_time / 1000,
                   ack->send_time % 1000);
    osdkStat = OsdkOsal_GetTimeMs(&ms);
    if (osdkStat != OSDK_STAT_OK) {
      OSDK_LOG_ERROR(MODULE_NAME_LOGGER, "osdk osal get time error");
    }

    if (s_timeSync.offsetErr > OSDK_TIMESTAMP_OFFSET_MAX_ERR_MS ||
        s_timeSync.offsetErr < -OSDK_TIMESTAMP_OFFSET_MAX_ERR_MS) {
      s_timeSync.offset = s_timeSync.lastOffset;
    } else {
      s_timeSync.offset = ack->send_time - ms;
    }

    s_timeSync.offsetErr = s_timeSync.offset - s_timeSync.lastOffset;
    s_timeSync.lastOffset = s_timeSync.offset;

  } else if (cb_type == OSDK_COMMAND_SENDASYNC_TIMEOUT) {
    OSDK_LOG_WARN(MODULE_NAME_LOGGER, "timesync from FC timeout");
    osdkStat = OSDK_STAT_ERR_TIMEOUT;
  }
}

static E_OsdkStat OsdkTimeSync_ASyncRequest(void) {
  T_CmdInfo info;
  dji_general_get_simple_time_sync_req timeStamp;
  E_OsdkStat osdkStat;

  timeStamp.sub_command_id = DJI_DELAY_COMPUTER_REQUEST;
  timeStamp.timestamp = 0;

  info.sender = OSDK_COMMAND_LIB_HOST_ID;
  info.addr = GEN_ADDR_RECEIVER_ONLY(OSDK_COMMAND_FC_DEVICE_ID);
  info.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  info.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  info.cmdSet = OSDK_COMMAND_CMDSET_COMMON;
  info.cmdId = OSDK_COMMON_CMD_ID_SIMPLE_TIME_SYNC;
  info.dataLen = sizeof(dji_general_get_simple_time_sync_req);
  info.channelId = 0;

  osdkStat = OsdkCommand_SendAsync(
      OsdkCommand_GetInstance(), &info, (uint8_t *)&timeStamp,
      OsdkTimeSync_ASyncCallback, NULL, OSDK_TIMESTAMP_SYNC_CMD_TIMEOUT_MS,
      OSDK_TIMESTAMP_SYNC_CMD_RETRY_COUNT);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_LOGGER, "timesync async send error");
  }

  return osdkStat;
}

static void OsdkTimeSync_Task(void *arg) {
  E_OsdkStat osdkStat;
  OSDK_UTIL_UNUSED(arg);

  if (OSDK_UTIL_WORK_RUN(s_osdkTimeSyncWorkStep++, OSDK_TIME_SYNC_TASK_FREQ,
                         OsdkCore_GetRootTaskFreq())) {
    OSDK_LOG_DEBUG(MODULE_NAME_LOGGER, "start timesync");
    osdkStat = OsdkTimeSync_ASyncRequest();
    if (osdkStat != OSDK_STAT_OK) {
      OSDK_LOG_WARN(MODULE_NAME_LOGGER, "timesync async send failed");
    }
  }
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
