/*! @file driver.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Implementation of HardDriver for the STM32F4Discovery board.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#include <driver.h>

size_t STM32F4::send(const uint8_t* buf, size_t len)
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

DJI::time_ms STM32F4::getTimeStamp()
{
  return tick;
}

