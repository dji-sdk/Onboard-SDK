/*! @file driver.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Implementation of HardDriver for the STM32F4Discovery board.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#include "DJI_HardDriver.h"
#include "stm32f4xx.h"
#include <time.h>
extern uint32_t tick;

class STM32F4 : public DJI::onboardSDK::HardDriver
{
  public:
  virtual size_t send(const uint8_t* buf, size_t len)
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

  virtual DJI::time_ms getTimeStamp() { return tick; }

  public:
  virtual void init() { ; }
  virtual size_t readall(uint8_t* buf, size_t maxlen)
  {
    return 8;
  }
  virtual void lockMemory() { ; }
  virtual void freeMemory() { ; }
  virtual void lockMSG() { ; }
  virtual void freeMSG() { ; }
};
