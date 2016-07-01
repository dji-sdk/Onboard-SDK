/*! @file main.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  An exmaple program of DJI-onboard-SDK portable for stm32
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 *******************************************************************************
 *                                                                             *
 *          --------               --------                 --------           *
 *         |        |   USART2    |        |    USART3     |        |          *
 *         |   PC   | <---------> | stm32  |  <----------> |  M100  |          *
 *         |        | (USB-TTL)   |        |               |        |          *
 *         |        |             |        |               |        |          *
 *          --------               --------                 --------           *
 *                                                                             *
 *                                                                             *
 *******************************************************************************
 * */

#include "main.h"

#undef USE_ENCRYPT
/*-----------------------DJI_LIB VARIABLE-----------------------------*/
using namespace DJI::onboardSDK;
DJI::onboardSDK::HardDriver* driver = new STM32F4;
CoreAPI defaultAPI = CoreAPI(driver);
CoreAPI *coreApi = &defaultAPI;
Flight flight = Flight(coreApi);
FlightData flightData;
VirtualRC virtualrc = VirtualRC(coreApi);
VirtualRCData myVRCdata =
{ 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
    1024, 1024 };

int main()
{
  BSPinit();
  delay_nms(30);
  printf("This is the example App to test DJI onboard SDK on STM32F4Discovery Board! \r\n");
  printf("Refer to \r\n");
  printf("https://developer.dji.com/onboard-sdk/documentation/github-platform-docs/STM32/README.html \r\n");
  printf("for supported commands!\r\n");
  printf("Initializing...\r\n");
  delay_nms(1000);

  //! Change the version string to your platform/version as defined in DJI_Version.h
  coreApi->setVersion(versionM100_31);
  delay_nms(1000);
  coreApi->getDroneVersion();
  delay_nms(20);
  printf("Done~!\r\n");

  while (1)
  {
    if (Rx_Handle_Flag == 1)
    {
      Rx_Handle_Flag = 0;
      Rx_buff_Handler();
    }
    delay_nms(50);
  }
}

void delay_nms(u16 time)
{
  u32 i = 0;
  while (time--)
  {
    i = 30000;
    while (i--)
      ;
  }
}

