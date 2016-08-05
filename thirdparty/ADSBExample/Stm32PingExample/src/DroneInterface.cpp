/*! @file DroneInterface.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Implement the HardDriver for STM32.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */
#include "DroneInterface.h"
extern uint32_t tick;

/*
* @brief Send package bytes to Drone (flight controller) through UART
*/
size_t DroneInterface::send(const uint8_t* buf, size_t len)
{
  char* p = (char*)buf;

  if (NULL == buf)
  {
    return 0;
  }

  while (len--)
  {
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
      ;
    USART_SendData(USART3, *p++);
  }
  return 1;
}

DJI::time_ms DroneInterface::getTimeStamp()
{
  return tick;
}
