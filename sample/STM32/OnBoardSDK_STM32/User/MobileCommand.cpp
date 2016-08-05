/*! @file MobileCommand.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Receive command from mobile OSDK App.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#include "MobileCommand.h"
#include "timer.h"
#include "DJI_Flight.h"
#include "Activate.h"
#include "LocalNavigation.h"

using namespace DJI::onboardSDK;
extern LocalNavigationStatus droneState;

void mobileCommandHandler(CoreAPI* api, Flight* flight)
{
  ackReturnToMobile returnACKMobile;

  if (api->getObtainControlMobileCMD())
  {
    api->setObtainControlMobileCMD(false);
    api->setControl(1);
    delay_nms(100);
    returnACKMobile.cmdID = 2;
    returnACKMobile.ack = api->getSimpleACK();
    api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
  }

  if (api->getReleaseControlMobileCMD())
  {
    api->setReleaseControlMobileCMD(false);
    api->setControl(0);
    delay_nms(100);
    returnACKMobile.cmdID = 3;
    returnACKMobile.ack = api->getSimpleACK();
    api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
  }

  if (api->getActivateMobileCMD())
  {
    api->setActivateMobileCMD(false);
    User_Activate();
    delay_nms(100);
    returnACKMobile.cmdID = 4;
    returnACKMobile.ack = api->getSimpleACK();
    api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
  }

  if (api->getArmMobileCMD())
  {
    api->setArmMobileCMD(false);
    flight->setArm(1);
    delay_nms(100);
    returnACKMobile.cmdID = 5;
    returnACKMobile.ack = api->getSimpleACK();
    api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
  }

  if (api->getDisArmMobileCMD())
  {
    api->setDisArmMobileCMD(false);
    flight->setArm(0);
    delay_nms(100);
    returnACKMobile.cmdID = 6;
    returnACKMobile.ack = api->getSimpleACK();
    api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
  }

  if (api->getTakeOffMobileCMD())
  {
    api->setTakeOffMobileCMD(false);
    flight->task(Flight::TASK_TAKEOFF);
    delay_nms(100);
    returnACKMobile.cmdID = 7;
    returnACKMobile.ack = api->getSimpleACK();
    api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
  }

  if (api->getLandingMobileCMD())
  {
    api->setLandingMobileCMD(false);
    flight->task(Flight::TASK_LANDING);
    delay_nms(100);
    returnACKMobile.cmdID = 8;
    returnACKMobile.ack = api->getSimpleACK();
    api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
  }

  if (api->getGoHomeMobileCMD())
  {
    api->setGoHomeMobileCMD(false);
    flight->task(Flight::TASK_GOHOME);
    delay_nms(100);
    returnACKMobile.cmdID = 9;
    returnACKMobile.ack = api->getSimpleACK();
    api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
  }

  if (api->getLocalNavTestMobileCMD())
  {
    api->setLocalNavTestMobileCMD(false);
    returnACKMobile.cmdID = 66;
    if(droneState.localNavExampleRunningFlag == 1)
    {
      returnACKMobile.ack = stopLocalNavExample();
    }
    else
    {
      returnACKMobile.ack = startLocalNavExample();
    }
    api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
  }
}

