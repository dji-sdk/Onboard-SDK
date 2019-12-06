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
static T_OsdkHalUdpHandler s_osdkHalUdp;
#endif

static T_HalOps halOps[INTERFACE_MAX_NUM] = {
    {"UART",  OsdkHal_UartSendData, OsdkHal_UartReadData},
#ifdef __linux__
    {"UDP",  OsdkHal_UdpSendData, OsdkHal_UdpReadData},
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
E_OsdkStat OsdkHal_UartSendData(const T_HalObj *obj, const uint8_t *pBuf, uint16_t bufLen)
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
E_OsdkStat OsdkHal_UartReadData(const T_HalObj *obj, uint8_t *pBuf, uint16_t *bufLen)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkHalUart.UartReadData(obj, pBuf, bufLen);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "uart read data error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

#ifdef __linux__
E_OsdkStat OsdkPlatform_RegHalUdpHandler(const T_OsdkHalUdpHandler *halUdpHandler)
{
    if (halUdpHandler->UdpInit == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (halUdpHandler->UdpWriteData == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    if (halUdpHandler->UdpReadData == NULL) {
        return OSDK_STAT_ERR_PARAM;
    }

    memcpy(&s_osdkHalUdp, halUdpHandler, sizeof(T_OsdkHalUdpHandler));

    return OSDK_STAT_OK;
}


/**
 * @brief Udp interface init function.
 * @param addr: udp interface address.
 * @param port:  udp interface port number.
 * @param obj: pointer to the hal object, which is used to store udp interface parameters.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkHal_UdpInit(const char *addr, const uint16_t port, T_HalObj *obj)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkHalUdp.UdpInit(addr, port, obj);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "udp init error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

/**
 * @brief Udp interface send function.
 * @param obj: pointer to the hal object, which including udp interface parameters.
 * @param pBuf:  pointer to the buffer which is used to store send data.
 * @param bufLen:  send data length.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkHal_UdpSendData(const T_HalObj *obj, const uint8_t *pBuf, uint16_t bufLen)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkHalUdp.UdpWriteData(obj, pBuf, bufLen);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "uart write data error, stat:%lld", osdkStat);
        return osdkStat;
    }

    return osdkStat;
}

/**
 * @brief Udp interface receive function.
 * @param obj: pointer to the hal object, which including udp interface parameters.
 * @param pBuf:  pointer to the buffer which is used to store receive data.
 * @param bufLen:  receive data length.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkHal_UdpReadData(const T_HalObj *obj, uint8_t *pBuf, uint16_t *bufLen)
{
    E_OsdkStat osdkStat;

    osdkStat = s_osdkHalUdp.UdpReadData(obj, pBuf, bufLen);
    if (osdkStat != OSDK_STAT_OK) {
        OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "udp read data error, stat:%lld", osdkStat);
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
  for (; i < INTERFACE_MAX_NUM; ++i) {
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
