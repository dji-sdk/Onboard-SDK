/**
 ********************************************************************
 * @file    osdk_util.h
 * @version V2.0.0
 * @date    2019/8/2
 * @brief
 *
 * @copyright (c) 2017-2018 DJI. All rights reserved.
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
#ifndef OSDK_UTIL_H
#define OSDK_UTIL_H

/* Includes ------------------------------------------------------------------*/
#include "osdk_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
#define OSDK_UTIL_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define OSDK_UTIL_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define OSDK_UTIL_UNUSED(x) ((x) = (x))
#define OSDK_UTIL_WORK_RUN(step, workfreq, taskfreq) \
  (!((step) % (uint32_t)((taskfreq) / (workfreq))))
#define OSDK_UTIL_ARRAY_SIZE(array) \
  ((uint32_t)(sizeof(array) / sizeof((array)[0])))

/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif  // OSDK_UTIL_H

/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
