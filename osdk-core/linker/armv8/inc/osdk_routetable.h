/**
 ********************************************************************
 * @file    osdk_routetable.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OSDK_ROUTETABLE_H
#define OSDK_ROUTETABLE_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

#define ADDR_SDK_COMMAND_INDEX 0x0
#define ADDR_V1_COMMAND_INDEX 0x1
#define ADDR_BIGDATA_LIVEVIEW_INDEX 0x2
#define ADDR_BIGDATA_ADVANCED_SENSING_INDEX 0x3

#define GEN_ADDR(receiver, index) \
  (((receiver << 16) & 0x00FF0000) | ((index << 8) & 0x0000FF00))
#define GEN_ADDR_RECEIVER_ONLY(receiver) ((receiver << 16) & 0x00FF0000)
#define GEN_ADDR_INDEX_ONLY(index) ((index << 8) & 0x0000FF00)
#define ADDR_INDEX_ONLY_MASK (0x0000FF00)

/* Exported types ------------------------------------------------------------*/


/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif  // OSDK_ROUTETABLE_H
        /************************ (C) COPYRIGHT DJI Innovations *******END OF
         * FILE******/
