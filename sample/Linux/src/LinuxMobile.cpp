/*! @file LinuxMobile.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Framework for executing Onboard SDK commands based on 
 *  mobile commands sent using Data Transparent Transmission.
 *
 *  Calls functions from the new Linux example based on user input
 *  in the companion iOS app
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "LinuxMobile.h"

using namespace DJI;
using namespace DJI::onboardSDK;

/*! Poll at 10Hz, waiting for data from Mobile OSDK App. The Onboard SDK
    receives this data, parses it and sets appropriate member variables
    of the API object. This loop checks to see if any of the members are
    set, and if so then executes that command. 

    The spin exits after ~150 mins.
!*/
void mobileCommandSpin(CoreAPI* api, Flight* flight, WayPoint* waypointObj)
{
  int t = 0;
  ackReturnToMobile returnACKMobile;
  while (t < 100000)
  {
    if (api->getObtainControlMobileCMD())
    {
      api->setObtainControlMobileCMD(false);
      ackReturnData takeControlStatus = takeControl(api);
      returnACKMobile.cmdID = 2;
      returnACKMobile.ack = takeControlStatus.ack;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getReleaseControlMobileCMD())
    {
      api->setReleaseControlMobileCMD(false);
      ackReturnData releaseControlStatus = releaseControl(api);
      returnACKMobile.cmdID = 3;
      returnACKMobile.ack = releaseControlStatus.ack;      
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getActivateMobileCMD())
    {
      api->setActivateMobileCMD(false);
      ackReturnData activateStatus = activate(api);
      returnACKMobile.cmdID = 4;
      returnACKMobile.ack = activateStatus.ack;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getTakeOffMobileCMD())
    {
      api->setTakeOffMobileCMD(false);
      ackReturnData takeoffStatus = monitoredTakeoff(api, flight);
      returnACKMobile.cmdID = 7;
      returnACKMobile.ack = takeoffStatus.ack;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getLandingMobileCMD())
    {
      api->setLandingMobileCMD(false);
      ackReturnData landingStatus = landing(api, flight);
      returnACKMobile.cmdID = 8;
      returnACKMobile.ack = landingStatus.ack;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getArmMobileCMD())
    {
      api->setArmMobileCMD(false);
      ackReturnData armStatus = arm(flight);
      returnACKMobile.cmdID = 5;
      returnACKMobile.ack = armStatus.ack;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getDisArmMobileCMD())
    {
      api->setDisArmMobileCMD(false);
      ackReturnData disArmStatus = disArm(flight);
      returnACKMobile.cmdID = 6;
      returnACKMobile.ack = disArmStatus.ack;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getGoHomeMobileCMD())
    {
      api->setGoHomeMobileCMD(false);
      ackReturnData goHomeStatus = goHome(flight);
      returnACKMobile.cmdID = 9;
      returnACKMobile.ack = goHomeStatus.ack;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getDrawSqrMobileCMD())
    {
      api->setDrawSqrMobileCMD(false);
      int drawSqrPosCtrlStatus = drawSqrPosCtrlSample(api, flight);
      returnACKMobile.cmdID = 62;
      returnACKMobile.ack = drawSqrPosCtrlStatus;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getWayPointTestMobileCMD())
    {
      api->setWayPointTestMobileCMD(false);
      wayPointMissionExample(api, waypointObj,1);
      returnACKMobile.cmdID = 65;
      returnACKMobile.ack = 1; //Always true since example does not return an ACK
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    usleep(100000);
    t++;
  }
}
