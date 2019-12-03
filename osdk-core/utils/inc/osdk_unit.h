/**
 ********************************************************************
 * @file    osdk_unit.h
 * @version V2.0.0
 * @date    2019/8/2
 * @brief   This is the header file for "osdk_unit.c", defining the structure
 *and
 * (exported) function prototypes.
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
#ifndef OSDK_UNIT_H
#define OSDK_UNIT_H

/* Includes ------------------------------------------------------------------*/
#include "osdk_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum {
  OSDK_UNIT_DATA_LENGTH_UNIT_BYTE = 0,
  OSDK_UNIT_DATA_LENGTH_UNIT_BIT,
  OSDK_UNIT_DATA_LENGTH_UNIT_KBYTE,
  OSDK_UNIT_DATA_LENGTH_UNIT_KBIT,
  OSDK_UNIT_DATA_LENGTH_UNIT_MBYTE,
  OSDK_UNIT_DATA_LENGTH_UNIT_MBIT,
} E_OsdkUnitDataLengthUnit;

/* Exported functions --------------------------------------------------------*/
E_OsdkStat OsdkUnit_DataLengthUnitConversion(float inDataLength,
                                             E_OsdkUnitDataLengthUnit inUnit,
                                             float *outDataLength,
                                             E_OsdkUnitDataLengthUnit outUnit);

#ifdef __cplusplus
}
#endif

#endif  // OSDK_UNIT_H

/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
