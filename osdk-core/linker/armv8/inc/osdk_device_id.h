/**
 ********************************************************************
 * @file    osdk_device_id.h
 * @version V2.0.0
 * @date    2019/07/01
 * @brief   This is the header file for "osdk_device_id.c", defining the
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
#ifndef OSDK_DEVICE_ID_H
#define OSDK_DEVICE_ID_H

/* Includes ------------------------------------------------------------------*/
#include "osdk_protocol_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
#define OSDK_COMMAND_DEVICE_ID(type, index) (((index) << 5) | (type))

/* Exported types ------------------------------------------------------------*/
typedef enum {
  OSDK_COMMAND_LIB_HOST_ID =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_CENTER, 7),
  OSDK_COMMAND_CAMERA_HOST_ID =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_CAMERA, 0),
  OSDK_COMMAND_GIMBAL_HOST_ID =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_GIMBAL, 0),
  OSDK_COMMAND_HMSSERVICE_ID =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_HMS_SERVICE, 7),
  OSDK_COMMAND_BATTERY_FIRST_DEVICE_ID =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_SMART_BATTERY,0),
  OSDK_COMMAND_BATTERY_SECOND_DEVICE_ID =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_SMART_BATTERY,1)
} E_OsdkCommandHostId;

typedef enum {
  OSDK_COMMAND_PC_DEVICE_ID =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_PC, 1),
  OSDK_COMMAND_OSDK_DEVICE_ID =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_PC, 6),
  OSDK_COMMAND_APP_DEVICE_ID =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_APP, 0),
  OSDK_COMMAND_SKYPORT_DEVICE_ID =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_CENTER, 3),
  OSDK_COMMAND_FC_DEVICE_ID =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_FC, 0),
  OSDK_COMMAND_FC_2_DEVICE_ID =
    OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_FC, 6),
  OSDK_COMMAND_FLIGHT_ID =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_MONOCULAR, 5),
} E_OsdkCommandDeviceId;

/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif  // OSDK_DEVICE_ID_H
 /************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
