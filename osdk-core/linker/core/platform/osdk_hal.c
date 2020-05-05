/**
 ********************************************************************
 * @file    osdk_hal.c
 * @version V2.0.0
 * @date    2019/07/01
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
#include <string.h>
#include "osdk_hal.h"
#include "osdk_logger_internal.h"

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
static T_OsdkHalUartHandler s_osdkHalUart;
#ifdef __linux__
static T_OsdkHalUSBBulkHandler s_osdkHalUSBBulk;
#endif

static T_HalOps halOps[] = {
    {"UART",  OsdkHal_UartSendData, OsdkHal_UartReadData, OsdkHal_UartClose},
#ifdef __linux__
    {"USB_BULK",  OsdkHal_USBBulkSendData, OsdkHal_USBBulkReadData, OsdkHal_USBBulkClose},
#endif
};
/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
E_OsdkStat OsdkPlatform_RegHalUartHandler(const T_OsdkHalUartHandler *halUartHandler)
{
    if (halUartHandler->UartInit == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (halUartHandler->UartWriteData == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (halUartHandler->UartReadData == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }
    
    if (halUartHandler->UartClose == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    memcpy(&s_osdkHalUart, halUartHandler, sizeof(T_OsdkHalUartHandler));

    return OSDK_STAT_OK;
}

/**
 * @brief Uart interface init function.
 * @param port: uart interface port.
 * @param baudrate:  uart interface baudrate.
 * @param obj: pointer to the hal object, which is used to store uart interface parameters.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkHal_UartInit(const char *port, const int baudrate, T_HalObj *obj)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkHalUart.UartInit(port, baudrate, obj);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "uart init error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

/**
 * @brief Uart interface send function.
 * @param obj: pointer to the hal object, which including uart interface parameters.
 * @param pBuf:  pointer to the buffer which is used to store send data.
 * @param bufLen:  send data length.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkHal_UartSendData(const T_HalObj *obj, const uint8_t *pBuf, uint32_t bufLen)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkHalUart.UartWriteData(obj, pBuf, bufLen);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "uart write data error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

/**
 * @brief Uart interface send function.
 * @param obj: pointer to the hal object, which including uart interface parameters.
 * @param pBuf:  pointer to the buffer which is used to store receive data.
 * @param bufLen:  receive data length.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkHal_UartReadData(const T_HalObj *obj, uint8_t *pBuf, uint32_t *bufLen)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkHalUart.UartReadData(obj, pBuf, bufLen);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "uart read data error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

/**
 * @brief Uart interface close function.
 * @param obj: pointer to the hal object, which including uart interface parameters.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkHal_UartClose(T_HalObj *obj)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkHalUart.UartClose(obj);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "uart close error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

#ifdef __linux__
E_OsdkStat OsdkPlatform_RegHalUSBBulkHandler(const T_OsdkHalUSBBulkHandler *halUSBBulkHandler)
{
  if (halUSBBulkHandler->USBBulkInit == NULL) {
    return OSDK_STAT_ERR_PARAM;
  }

  if (halUSBBulkHandler->USBBulkWriteData == NULL) {
    return OSDK_STAT_ERR_PARAM;
  }

  if (halUSBBulkHandler->USBBulkReadData == NULL) {
    return OSDK_STAT_ERR_PARAM;
  }
  
  if (halUSBBulkHandler->USBBulkClose == NULL) {
    return OSDK_STAT_ERR_PARAM;
  }

  memcpy(&s_osdkHalUSBBulk, halUSBBulkHandler, sizeof(T_OsdkHalUSBBulkHandler));

  return OSDK_STAT_OK;
}


/**
 * @brief USBBulk interface init function.
 * @param pid: USBBulk product id.
 * @param vid: USBBulk vendor id.
 * @param num: USBBulk interface num.
 * @param epIn: USBBulk input endpoint .
 * @param epOut: USBBulk output endpoint.
 * @param obj: pointer to the hal object, which is used to store USBBulk interface parameters.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkHal_USBBulkInit(uint16_t pid, uint16_t vid, uint16_t num, uint16_t epIn, uint16_t epOut, T_HalObj *obj)
{
  E_OsdkStat osdkStat;

  osdkStat = s_osdkHalUSBBulk.USBBulkInit(pid, vid, num, epIn, epOut, obj);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "USBBulk init error, stat:%lld", osdkStat);
    return osdkStat;
  }

  return osdkStat;
}

/**
 * @brief USBBulk interface send function.
 * @param obj: pointer to the hal object, which including USBBulk interface parameters.
 * @param pBuf:  pointer to the buffer which is used to store send data.
 * @param bufLen:  send data length.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkHal_USBBulkSendData(const T_HalObj *obj, const uint8_t *pBuf, uint32_t bufLen)
{
  E_OsdkStat osdkStat;

  osdkStat = s_osdkHalUSBBulk.USBBulkWriteData(obj, pBuf, bufLen);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "uart write data error, stat:%lld", osdkStat);
    return osdkStat;
  }

  return osdkStat;
}

/**
 * @brief USBBulk interface receive function.
 * @param obj: pointer to the hal object, which including USBBulk interface parameters.
 * @param pBuf:  pointer to the buffer which is used to store receive data.
 * @param bufLen:  receive data length.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkHal_USBBulkReadData(const T_HalObj *obj, uint8_t *pBuf, uint32_t *bufLen)
{
  E_OsdkStat osdkStat;

  osdkStat = s_osdkHalUSBBulk.USBBulkReadData(obj, pBuf, bufLen);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "USBBulk read data error, stat:%lld", osdkStat);
    return osdkStat;
  }

  return osdkStat;
}

/**
 * @brief USBBulk interface close function.
 * @param obj: pointer to the hal object, which including USBBulk interface parameters.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkHal_USBBulkClose(T_HalObj *obj)
{
  E_OsdkStat osdkStat;

  osdkStat = s_osdkHalUSBBulk.USBBulkClose(obj);
  if (osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "USBBulk close error, stat:%lld", osdkStat);
    return osdkStat;
  }

  return osdkStat;
}

#endif
/**
 * @brief Function used to get hal operations.
 * @param interface: hal interface name.
 * @param ops: pointer to the hal interface operations.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkHal_GetHalOps(const char *interface, T_HalOps *ops)
{
  int i = 0;
  for (; i < (sizeof(halOps) / sizeof(T_HalOps)); ++i) {
    if (strcmp(halOps[i].name, interface) == 0) {
      *ops = halOps[i];
      return OSDK_STAT_OK;
    }
  }
  OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "can't find related HAL interface");
  return OSDK_STAT_ERR;
}
/* Private functions definition-----------------------------------------------*/

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
