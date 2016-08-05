/** @file DJI_Mission.cpp
 *  @version 3.1.7
 *  @date July 1st, 2016
 *
 *  @brief
 *  Mission Framework for DJI onboardSDK library
 *
 *  @copyright 2016 DJI. All right reserved.
 *
 */

#include "DJI_API.h"
#include "DJI_Mission.h"
#include <string.h>

namespace DJI
{
namespace onboardSDK
{

MissionACKMap missionACKMAP[] = {
  //! @note common ACK code
  { 0x00, " 0x00 Success" },
  { 0x01, " 0x01 Wrong WayPoint Index" },
  { 0xD0, " 0xD0 Not At Mode F" },
  { 0xD1, " 0xD1 Need obtain control" },
  { 0xD2, " 0xD2 Need close IOC mode" },
  { 0xD3, " 0xD3 Mission not inited" },
  { 0xD4, " 0xD4 Mission not running" },
  { 0xD5, " 0xD5 Mission already running" },
  { 0xD6, " 0xD6 Too consuming of time" },
  { 0xD7, " 0xD7 Other mission running" },
  { 0xD8, " 0xD8 Bad GPS" },
  { 0xD9, " 0xD9 Low battery" },
  { 0xDA, " 0xDA UAV did not take off" },
  { 0xDB, " 0xDB Wrong patameters" },
  { 0xDC, " 0xDC Conditions not satisfied" },
  { 0xDD, " 0xDD Crossing No-Fly zone" },
  { 0xDE, " 0xDE Unrecorded Home" },
  { 0xDF, " 0xDF Already at No-Fly zone" },
  { 0xC0, " 0xC0 Too High" },
  { 0xC1, " 0xC1 Too Low" },
  { 0xC7, " 0xC7 Too Far from Home" },
  { 0xC8, " 0xC8 Mission not supported" },
  { 0xC9, " 0xC9 Too far from current position" },
  { 0xCA, " 0xCA Novice Mode not support missions" },
  { 0xF0, " 0xF0 Taking off" },
  { 0xF1, " 0xF1 Landing" },
  { 0xF2, " 0xF2 Returning Home" },
  { 0xF3, " 0xF3 Starting motors" },
  { 0xF4, " 0xF4 Wrong command" },
  { 0xFF, " 0xFF Unknown Error" },
  //! @note Follow ACK code
  { 0xB0, " 0xB0 too far from your position, lack of Radio connection" },
  { 0xB1, " 0xB1 Cutoff time overflow" },
  { 0xB2, " 0xB2 Too Large Gimbal pitch angle" },
  //! @note HotPoint ACK code
  { 0xC2, " 0xC2 Invalid Radius" },
  { 0xC3, " 0xC3 Too large yawRate" },
  { 0xC4, " 0xC4 Invalid vision" },
  { 0xC5, " 0xC5 Invalid Yaw mode" },
  { 0xC6, " 0xC6 Too far from HotPoint" },
  { 0xA2, " 0xA2 Invalid HotPoint Parameter" },
  { 0xA3, " 0xA3 Invalid latitude or longtitude" },
  { 0xA6, " 0xA6 Invalid direction" },
  { 0xA9, " 0xA9 HotPoint already paused" },
  { 0xAA, " 0xAA HotPoint did not paused" },
  //! @note WayPoint ACK code
  { 0xE0, " 0xE0 Invalid waypoint mission data" },
  { 0xE1, " 0xE1 Invalid waypoint point data" },
  { 0xE2, " 0xE2 Too long waypoint distance" },
  { 0xE3, " 0xE3 Too long waypoint mission" },
  { 0xE4, " 0xE4 Too much points" },
  { 0xE5, " 0xE5 Points too close" },
  { 0xE6, " 0xE6 Points too far" },
  { 0xE7, " 0xE7 Check faild" },
  { 0xE8, " 0xE8 Invalid action" },
  { 0xE9, " 0xE9 Point data not enough" },
  { 0xEA, " 0xEA Waypoint Mission data not enough" },
  { 0xEB, " 0xEB WayPoints not enough" },
  { 0xEC, " 0xEC WayPoint already running" },
  { 0xED, " 0xED WayPoint not running" },
  { 0xEE, " 0xEE Invaild velocity" },
  //! @note IOC ACK code
  { 0xA0, " 0xA0 Too near from home" },
  { 0xA1, " 0xA1 Unknown IOC type" }
};

bool CoreAPI::decodeMissionStatus(uint8_t ack)
{
  for (uint8_t i = 0; i < sizeof(missionACKMAP); ++i)
    if (missionACKMAP[i].code == ack)
    {
      //! @todo Fix memory leak issue
      API_LOG(serialDevice, STATUS_LOG, "0x%X %s\n", missionACKMAP[i].code, missionACKMAP[i].meaning);
      return true;
    }
  return false;
}

void missionCallback(CoreAPI *api, Header *protocolHeader, UserData userdata __UNUSED)
{
  MissionACK ack;
  if (protocolHeader->length - EXC_DATA_SIZE <= sizeof(ack))
  {
    memcpy((unsigned char *)&ack, (unsigned char *)protocolHeader + sizeof(Header),
        (protocolHeader->length - EXC_DATA_SIZE));
    if (!api->decodeMissionStatus(ack))
      API_LOG(api->getDriver(), ERROR_LOG, "Decode ACK error 0x%X\n", ack);
  }
  else
  {
    API_LOG(api->getDriver(), ERROR_LOG, "ACK is exception,session id %d,sequence %d\n",
        protocolHeader->sessionID, protocolHeader->sequenceNumber);
  }
}

void CoreAPI::setWayPointEventCallback(CallBackHandler callback)
{
  wayPointEventCallback = callback;
}

void CoreAPI::setMisssionCallback(CallBack handler, UserData userData)
{
  missionCallback.callback = handler;
  missionCallback.userData = userData;
}

void CoreAPI::setHotPointCallback(CallBack handler, UserData userData)
{
  hotPointCallback.callback = handler;
  hotPointCallback.userData = userData;
}

void CoreAPI::setWayPointCallback(CallBack handler, UserData userData)
{
  wayPointCallback.callback = handler;
  wayPointCallback.userData = userData;
}

void CoreAPI::setFollowCallback(CallBack handler, UserData userData)
{
  followCallback.callback = handler;
  followCallback.userData = userData;
}

void CoreAPI::setWayPointEventCallback(CallBack handler, UserData userData)
{
  wayPointEventCallback.callback = handler;
  wayPointEventCallback.userData = userData;
}



} // namespace onboardSDK
} // namespace DJI
