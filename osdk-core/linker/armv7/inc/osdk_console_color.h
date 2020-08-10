/**
 ********************************************************************
 * @file    osdk_console_color.h
 * @version V1.0.0
 * @date    2019/09/15
 * @brief   This is the header file for "osdk_logger_color.c", defining the
 *structure and
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
#ifndef OSDK_CONSOLE_COLOR_H
#define OSDK_CONSOLE_COLOR_H

#define OSDK_CONSOLE_COLOR false
/* Includes ------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
// Define different level log print color here
#if OSDK_CONSOLE_COLOR
#define LOG_COLOR_END "\033[0m"
#define LOG_COLOR_BLACK "\033[30m"
#define LOG_COLOR_RED "\033[31m"
#define LOG_COLOR_GREEN "\033[32m"
#define LOG_COLOR_YELLOW "\033[33m"
#define LOG_COLOR_BLUE "\033[34m"
#define LOG_COLOR_PURPLE "\033[35m"
#define LOG_COLOR_CYAN "\033[36m"
#define LOG_COLOR_WHITE "\033[37m"
#else
#define LOG_COLOR_END
#define LOG_COLOR_BLACK
#define LOG_COLOR_RED
#define LOG_COLOR_GREEN
#define LOG_COLOR_YELLOW
#define LOG_COLOR_BLUE
#define LOG_COLOR_PURPLE
#define LOG_COLOR_CYAN
#define LOG_COLOR_WHITE
#endif
/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif  // OSDK_CONSOLE_COLOR_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
