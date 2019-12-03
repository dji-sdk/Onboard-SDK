/**
 ********************************************************************
 * @file    osdk_logger.c
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
#include "osdk_logger.h"
#include "osdk_logger_internal.h"

/* Private constants ---------------------------------------------------------*/
#define LOGGER_BUF_MAX_SIZE 512

// attention:hex printf length should be less than this macro
#define LOGGER_HEX_MAX_NUM 64
#define LOGGER_HEX_LINEBREAK_NUM 16

#define LOGGER_CONSOLE_MAX_NUM 8
#define LOGGER_REMOTE_RECORD_LEVEL OSDK_LOGGER_CONSOLE_LOG_LEVEL_INFO

/* Private types -------------------------------------------------------------*/
static T_OsdkLoggerConsole s_consoleList[LOGGER_CONSOLE_MAX_NUM];

/* Private functions declaration ---------------------------------------------*/
static void OsdkLogger_RemoteRecord(const uint8_t *data, uint16_t dataLen);

/* Exported functions definition ---------------------------------------------*/
E_OsdkStat OsdkLogger_AddConsole(T_OsdkLoggerConsole *console) {
  int i;

  if (console->consoleLevel > OSDK_LOGGER_CONSOLE_LOG_LEVEL_DEBUG ||
      console->consoleLevel < OSDK_LOGGER_CONSOLE_LOG_LEVEL_ERROR ||
      console->func == NULL) {
    return OSDK_STAT_ERR_PARAM;
  }

  for (i = 0; i < LOGGER_CONSOLE_MAX_NUM; i++) {
    if (s_consoleList[i].func == NULL ||
        s_consoleList[i].consoleLevel < OSDK_LOGGER_CONSOLE_LOG_LEVEL_ERROR ||
        s_consoleList[i].consoleLevel > OSDK_LOGGER_CONSOLE_LOG_LEVEL_DEBUG) {
      break;
    }
  }

  if (i == LOGGER_CONSOLE_MAX_NUM) {
    return OSDK_STAT_ERR_OUT_OF_RANGE;
  }

  s_consoleList[i].func = console->func;
  s_consoleList[i].consoleLevel = console->consoleLevel;

  return OSDK_STAT_OK;
}

void OsdkLogger_Log(const char *moduleName, uint8_t level, const char *fmt,
                    ...) {
  va_list args;
  int length;
  char *logBuf;
  char *formatBuf;
  uint32_t timeMs;

  OsdkOsal_GetTimeMs(&timeMs);
  logBuf = OsdkOsal_Malloc(LOGGER_BUF_MAX_SIZE);
  if (logBuf == NULL) {
    return;
  }

  formatBuf = OsdkOsal_Malloc(LOGGER_BUF_MAX_SIZE);
  if (formatBuf == NULL) {
    OsdkOsal_Free(logBuf);
    return;
  }

  va_start(args, fmt);
  snprintf(formatBuf, LOGGER_BUF_MAX_SIZE, "[%d.%03d][module_%s]-%s\r\n",
           timeMs / 1000, timeMs % 1000, moduleName, fmt);
  length = vsnprintf(logBuf, LOGGER_BUF_MAX_SIZE, formatBuf, args);

  if (length > 0) {
    for (int i = 0; i < LOGGER_CONSOLE_MAX_NUM; i++) {
      if (s_consoleList[i].consoleLevel > OSDK_LOGGER_CONSOLE_LOG_LEVEL_DEBUG ||
          s_consoleList[i].consoleLevel < OSDK_LOGGER_CONSOLE_LOG_LEVEL_ERROR ||
          s_consoleList[i].func == NULL) {
        break;
      }

      if (level <= s_consoleList[i].consoleLevel) {
        s_consoleList[i].func((uint8_t *)logBuf, length);
      }
    }

    if (level <= LOGGER_REMOTE_RECORD_LEVEL) {
      OsdkLogger_RemoteRecord((uint8_t *)logBuf, length);
    }
  }

  va_end(args);
  OsdkOsal_Free(logBuf);
  OsdkOsal_Free(formatBuf);
}

void OsdkLogger_PrintfHex(const uint8_t *data, uint16_t dataLen) {
  char *logBuf;
  int len = 0;

  if (dataLen >= LOGGER_HEX_MAX_NUM) {
    OSDK_LOG_ERROR(MODULE_NAME_LOGGER, "hex data length out of range");
    return;
  }

  logBuf = OsdkOsal_Malloc(LOGGER_BUF_MAX_SIZE);
  if (logBuf == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_LOGGER, "logger malloc error");
    return;
  }

  for (int i = 0; i < dataLen; i++) {
    if ((i + 1) % LOGGER_HEX_LINEBREAK_NUM == 0 && i > 0) {
      len += sprintf(&logBuf[len], "0x%02X\r\n", data[i]);
    } else {
      len += sprintf(&logBuf[len], "0x%02X ", data[i]);
    }
  }
  OsdkLogger_Log(MODULE_NAME_LOGGER, OSDK_LOGGER_CONSOLE_LOG_LEVEL_DEBUG,
                 "PrintHex:\r\n%s", logBuf);

  OsdkOsal_Free(logBuf);
}

/* Private functions definition-----------------------------------------------*/
static void OsdkLogger_RemoteRecord(const uint8_t *data, uint16_t dataLen) {
  // TODO: add the remote record to UAV later
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
