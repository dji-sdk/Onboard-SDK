/*! @file main.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  An exmaple program to integrate Ping ADS-B receiver with
 *  STM32F4 Discovery board running DJI onboard SDK. It sends
 *  air traffic information to PC and also mobile device
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 ********************************************************************************
 *                                                                              *
 *    --------             --------             --------              --------  *
 *   |        |   USART2  |        |   USART3  |        |     RC     |        | *
 *   |   PC   | <-------- | STM32  | <-------> |  M100  |<---------->| Mobile | *
 *   |        | (USB-TTL) |        |           |        |            |        | *
 *    --------             --------             --------              --------  *
 *                            ^                                                 *
 *                            |                 --------                        *
 *                            |    UART4       |        |                       *
 *                            |----------------|  Ping  |                       *
 *                                             |        |                       *
 *                                              --------                        *
 ********************************************************************************
 * */

#include "bsp.h"
#include "AppKey.h"
#include "Ping.h"
#include "DroneInterface.h"

#ifdef USE_ENCRYPT
#undef USE_ENCRYPT
#endif

using namespace DJI::onboardSDK;

/*-----------------------Global API Object-----------------------------*/
DroneInterface* driver = new DroneInterface;
CoreAPI defaultAPI = CoreAPI(driver);
CoreAPI *coreApi = &defaultAPI;

int main()
{
  boardInit();
  printf("Example App to use Ping ADS-B Radio with DJI onboard SDK on STM32F4Discovery Board! \r\n");
  printf("Board Initialization Done!\r\n");
  delay_nms(200);

  coreApi->setVersion(userDroneVersion);
  printf("API Version Set Done!\r\n");

  Ping ping(coreApi);

  // simple task scheduling
  uint32_t nextSecTick = driver->getTimeStamp() + 1000;
  uint32_t next500MilTick = driver->getTimeStamp() + 500;

  uint32_t runOnce = 1;
  while (1)
  {
    if (runOnce)
    {
      runOnce = 0;

      coreApi->getDroneVersion();
      delay_nms(50);

      coreApi->setBroadcastFreqDefaults();
      delay_nms(50);

      coreApi->activate(&userActivationInfo);
      delay_nms(50);

      coreApi->setControl(1);
      delay_nms(50);
    }

    ping.processPing();

    if (driver->getTimeStamp() >= nextSecTick)
    {
      nextSecTick = driver->getTimeStamp() + 1000;
      ping.updateTrafficAge();
    }

    if (driver->getTimeStamp() >= next500MilTick)
    {
      next500MilTick = driver->getTimeStamp() + 500;

      PingTraffic traffic;
      if (ping.getNextAvailPingTraffic(traffic))
      {
        MessageToMobile msgToMobile;
        msgToMobile.icao          = traffic.icao;
        msgToMobile.lat           = traffic.lat;
        msgToMobile.lon           = traffic.lon;
        msgToMobile.alt           = traffic.altitude;
        msgToMobile.heading       = traffic.heading;
        msgToMobile.velHoriz      = traffic.hor_velocity;
        msgToMobile.velUp         = traffic.ver_velocity;
        msgToMobile.distanceToMe  = traffic.distanceToTraffic;
        msgToMobile.age           = traffic.age;

        // For initial test: print to serial
        printf("ICAO:%X,Lat:%03.6f,Lon:%03.6f,Alt:%05d,Hdg:%03d,Vel:%03d\r\n",
            msgToMobile.icao,
            ((double)msgToMobile.lat)/1e7,
            ((double)msgToMobile.lon)/1e7,
            msgToMobile.alt/1000,
            msgToMobile.heading/100,
            msgToMobile.velHoriz/1000);

        // For Mobile App, organize the data to send to mobile
        coreApi->sendToMobile((uint8_t *)&msgToMobile, sizeof(MessageToMobile));
      }
      else
      {
        printf("Empty Traffic List;\r\n");
      }
    }
    coreApi->sendPoll();
  }
}



