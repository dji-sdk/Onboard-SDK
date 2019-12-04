/**
 ********************************************************************
 * @file    osdk_unit.c
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

/* Includes ------------------------------------------------------------------*/
#include "osdk_unit.h"
#include "osdk_logger_internal.h"

/* Private constants ---------------------------------------------------------*/
#define OSDK_UNIT_TIMES_OF_BYTE_BIT 8
#define OSDK_UNIT_TIMES_OF_KBYTE_BYTE 1024

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/
static E_OsdkStat OsdkUnit_CalculateDataLengthUnitRelativeTimes(
    E_OsdkUnitDataLengthUnit unit, uint32_t *relativeTimes);

/* Private variables ---------------------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
E_OsdkStat OsdkUnit_DataLengthUnitConversion(float inDataLength,
                                             E_OsdkUnitDataLengthUnit inUnit,
                                             float *outDataLength,
                                             E_OsdkUnitDataLengthUnit outUnit) {
  E_OsdkStat osdkStat;
  uint32_t inUnitRelativeTimes = 0;   // times of unit relative to bit
  uint32_t outUnitRelativeTimes = 0;  // times of unit relative to bit

  if (outDataLength == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "input pointer is null.");
    return OSDK_STAT_ERR_PARAM;
  }

  osdkStat = OsdkUnit_CalculateDataLengthUnitRelativeTimes(
      inUnit, &inUnitRelativeTimes);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL,
                   "calculate relative times of data length unit error: %d.",
                   osdkStat);
    return OSDK_STAT_SYS_ERR;
  }

  osdkStat = OsdkUnit_CalculateDataLengthUnitRelativeTimes(
      outUnit, &outUnitRelativeTimes);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL,
                   "calculate relative times of data length unit error: %d.",
                   osdkStat);
    return OSDK_STAT_SYS_ERR;
  }

  *outDataLength = inDataLength * inUnitRelativeTimes / outUnitRelativeTimes;

  return OSDK_STAT_OK;
}

/* Private functions definition-----------------------------------------------*/
static E_OsdkStat OsdkUnit_CalculateDataLengthUnitRelativeTimes(
    E_OsdkUnitDataLengthUnit unit, uint32_t *relativeTimes) {
  if (relativeTimes == NULL) {
    OSDK_LOG_ERROR(MODULE_NAME_UTIL, "input pointer is null.");
    return OSDK_STAT_ERR_PARAM;
  }

  switch (unit) {
    case OSDK_UNIT_DATA_LENGTH_UNIT_BYTE:
      *relativeTimes = OSDK_UNIT_TIMES_OF_BYTE_BIT;
      break;
    case OSDK_UNIT_DATA_LENGTH_UNIT_BIT:
      *relativeTimes = 1;
      break;
    case OSDK_UNIT_DATA_LENGTH_UNIT_KBYTE:
      *relativeTimes =
          OSDK_UNIT_TIMES_OF_BYTE_BIT * OSDK_UNIT_TIMES_OF_KBYTE_BYTE;
      break;
    case OSDK_UNIT_DATA_LENGTH_UNIT_KBIT:
      *relativeTimes = OSDK_UNIT_TIMES_OF_KBYTE_BYTE;
      break;
    case OSDK_UNIT_DATA_LENGTH_UNIT_MBYTE:
      *relativeTimes = OSDK_UNIT_TIMES_OF_BYTE_BIT *
                       OSDK_UNIT_TIMES_OF_KBYTE_BYTE *
                       OSDK_UNIT_TIMES_OF_KBYTE_BYTE;
      break;
    case OSDK_UNIT_DATA_LENGTH_UNIT_MBIT:
      *relativeTimes =
          OSDK_UNIT_TIMES_OF_KBYTE_BYTE * OSDK_UNIT_TIMES_OF_KBYTE_BYTE;
      break;
    default:
      OSDK_LOG_ERROR(MODULE_NAME_UTIL, "data length unit is incorrect: %d.",
                     unit);
      return OSDK_STAT_ERR_OUT_OF_RANGE;
  }

  return OSDK_STAT_OK;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/