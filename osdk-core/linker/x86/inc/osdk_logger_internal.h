/**
 ********************************************************************
 * @file    osdk_logger_internal.h
 * @version V1.0.0
 * @date    2019/09/15
 * @brief   This is the header file for "osdk_logger.c", defining the structure
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
#ifndef OSDK_LOGGER_INTERNAL_H
#define OSDK_LOGGER_INTERNAL_H

/* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#include "osdk_console_color.h"
#include "osdk_logger.h"
#include "osdk_module_name.h"
#include "osdk_osal.h"
#include "osdk_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported functions --------------------------------------------------------*/
void OsdkLogger_Log(const char *moduleName, uint8_t level, const char *fmt,
                    ...);
void OsdkLogger_PrintfHex(const uint8_t *data, uint16_t dataLen);
/* Exported constants --------------------------------------------------------*/
// debug config
#define OSDK_USE_LOG 1

/* Exported types ------------------------------------------------------------*/

// Define various grade log information print.
#if OSDK_USE_LOG
#define OSDK_LOG_DEBUG(module, fmt, ...)                              \
  OsdkLogger_Log(module, OSDK_LOGGER_CONSOLE_LOG_LEVEL_DEBUG,         \
                 LOG_COLOR_WHITE "[Debug]-[%s:%d)" fmt LOG_COLOR_END, \
                 __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define OSDK_LOG_INFO(module, fmt, ...)                              \
  OsdkLogger_Log(module, OSDK_LOGGER_CONSOLE_LOG_LEVEL_INFO,         \
                 LOG_COLOR_GREEN "[Info]-[%s:%d]" fmt LOG_COLOR_END, \
                 __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define OSDK_LOG_WARN(module, fmt, ...)                               \
  OsdkLogger_Log(module, OSDK_LOGGER_CONSOLE_LOG_LEVEL_WARNING,       \
                 LOG_COLOR_YELLOW "[Warn]-[%s:%d]" fmt LOG_COLOR_END, \
                 __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define OSDK_LOG_ERROR(module, fmt, ...)                            \
  OsdkLogger_Log(module, OSDK_LOGGER_CONSOLE_LOG_LEVEL_ERROR,       \
                 LOG_COLOR_RED "[Error]-[%s:%d]" fmt LOG_COLOR_END, \
                 __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define OSDK_LOG_DEBUG(fmt, ...)
#define OSDK_LOG_INFO(fmt, ...)
#define OSDK_LOG_WARN(fmt, ...)
#define OSDK_LOG_ERROR(fmt, ...)
#endif

#ifdef __cplusplus
}
#endif

#endif  // OSDK_LOGGER_INTERNAL_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
