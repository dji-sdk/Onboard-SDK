/**
 ********************************************************************
 * @file    osdk_hal.c
 * @version V1.0.0
 * @date    2019/09/25
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
#include "osdkhal_stm32.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t UartDataRecvQueue;
extern QueueHandle_t ACMDataRecvQueue;
extern QueueHandle_t ACMDataSendQueue;

enum STM32_LINK_FD
{
  INVALID_FD = -1,
  UART_FD    = 0,
  ACM_FD     = 1,
};

/* Exported functions definition ---------------------------------------------*/
E_OsdkStat OsdkSTM32_UartSendData(const T_HalObj *obj, const uint8_t *pBuf, uint32_t bufLen) {
  while (bufLen--)
  {
    if (obj->uartObject.fd == UART_FD) {
      while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
      USART_SendData(USART3, *pBuf++);
    } else if (obj->uartObject.fd == ACM_FD) {
      xQueueSend(ACMDataSendQueue, pBuf++, 0);
    } else {
      return OSDK_STAT_ERR;
    }
  }
  return OSDK_STAT_OK;
}

E_OsdkStat OsdkSTM32_UartReadData(const T_HalObj *obj, uint8_t *pBuf, uint32_t *bufLen) {
  uint8_t recvByte;
  
  if ((!pBuf) || (!bufLen)) {
    return OSDK_STAT_ERR;
  }
  uint16_t targetLen = *bufLen;
  *bufLen = 0;

  if (obj->uartObject.fd == UART_FD) {
    for (int cnt = 0; cnt < HAL_ONCE_READ_LEN; cnt++) {
    if(xQueueReceive(UartDataRecvQueue, &recvByte, 0)) {
        pBuf[*bufLen] = recvByte;
        (*bufLen)++;
      } else {
        break;
      }
    }
  } else if (obj->uartObject.fd == ACM_FD) {
    for (int cnt = 0; cnt < HAL_ONCE_READ_LEN; cnt++) {
    if(xQueueReceive(ACMDataRecvQueue, &recvByte, 0)) {
        pBuf[*bufLen] = recvByte;
        (*bufLen)++;
      } else {
        break;
      }
    }
  } else {
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

/* Private functions definition-----------------------------------------------*/
E_OsdkStat OsdkSTM32_UartInit(const char *port, const int baudrate,
                            T_HalObj *obj) {
  if (strcmp(port, UART_PORT) == 0) {
    obj->uartObject.fd = UART_FD;
  } else if (strcmp(port, ACM_PORT) == 0) {
    obj->uartObject.fd = ACM_FD;
  } else {
    obj->uartObject.fd = INVALID_FD;
  }
  
  return OSDK_STAT_OK;
}

E_OsdkStat OsdkSTM32_UartClose(T_HalObj *obj) {
  return OSDK_STAT_OK;
}

E_OsdkStat OsdkSTM32_UdpInit(const char *addr, uint16_t port, T_HalObj *obj) {
  return OSDK_STAT_ERR;
}
/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
