/**
 ********************************************************************
 * @file    osdk_osal.h
 * @version V2.0.0
 * @date    2019/07/01
 * @brief   OSDK type define file.
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
#ifndef OSDK_TYPEDEF_H
#define OSDK_TYPEDEF_H

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
#define OSDK_EMPTY_STRUCT uint8_t empty;

// float type define
typedef double osdk_f64_t;
typedef float osdk_f32_t;

// function process state type define
typedef enum {
  OSDK_STAT_OK = 0,
  OSDK_STAT_ERR_ALLOC = 1,
  OSDK_STAT_ERR_TIMEOUT = 2,
  OSDK_STAT_ERR_NOT_FOUND = 3,
  OSDK_STAT_ERR_OUT_OF_RANGE = 4,
  OSDK_STAT_ERR_PARAM = 5,
  OSDK_STAT_NO_NEED_ACK = 6,
  OSDK_STAT_SYS_ERR = 7,
  OSDK_STAT_NOT_READY = 8,
  OSDK_STAT_ERR = 0xFF,
} E_OsdkStat;

/* Exported functions --------------------------------------------------------*/

// define compiler specific symbols
#if defined(__ICCARM__)
#elif defined(__CC_ARM)
#pragma anon_unions
#elif defined(__GNUC__)
#elif defined(__TASKING__)
#endif

#ifdef __cplusplus
}
#endif

#endif  // OSDK_TYPEDEF_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
