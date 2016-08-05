/*! @file main.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
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

HardDriver* driver = new STM32F4;
CoreAPI defaultAPI = CoreAPI(driver);
CoreAPI *coreApi = &defaultAPI;

Flight flight = Flight(coreApi);
FlightData flightData;

VirtualRC virtualrc = VirtualRC(coreApi);
VirtualRCData myVRCdata =
{ 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
    1024, 1024 };

extern TerminalCommand myTerminal;
extern LocalNavigationStatus droneState;
extern uint8_t myFreq[16];

int main()
{
  BSPinit();
  delay_nms(30);
  printf("This is the example App to test DJI onboard SDK on STM32F4Discovery Board! \r\n");
  printf("Refer to \r\n");
  printf("https://developer.dji.com/onboard-sdk/documentation/github-platform-docs/STM32/README.html \r\n");
  printf("for supported commands!\r\n");
  printf("Board Initialization Done!\r\n");
  delay_nms(1000);

  //! Change the version string to your platform/version as defined in DJI_Version.h
  coreApi->setVersion(versionM100_31);
  delay_nms(200);
  printf("API Version Set Done!\r\n");

  uint32_t runOnce = 1;
  uint32_t next500MilTick;
  while (1)
  {
    // One time automatic activation
    if (runOnce)
    {
      runOnce = 0;
      coreApi->setBroadcastFreq(myFreq);
      delay_nms(50);

      // The Local navigation example will run in broadcast call back function,
      // immediate after GPS position is updated
      coreApi->setBroadcastCallback(myRecvCallback,(DJI::UserData)(&droneState));

      User_Activate();      
      delay_nms(50);

      next500MilTick = driver->getTimeStamp() + 500;
    }

    if (driver->getTimeStamp() >= next500MilTick)
    {
      next500MilTick = driver->getTimeStamp() + 500;

      // Handle user commands from mobile device
      mobileCommandHandler(coreApi, &flight);

      // Handle user commands from serial (USART2)
      myTerminal.terminalCommandHandler(coreApi, &flight);
    }

    coreApi->sendPoll();
  }
}

