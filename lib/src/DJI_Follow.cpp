/** @file DJI_Follow.cpp
 *  @version 3.1.7
 *  @date July 1st, 2016
 *
 *  @brief
 *  Follow API for DJI onboardSDK library
 *
 *  @copyright 2016 DJI. All right reserved.
 *
 */

#include "DJI_Follow.h"

using namespace DJI;
using namespace DJI::onboardSDK;

Follow::Follow(CoreAPI *ControlAPI)
{
  api = ControlAPI;
  resetData();
}

void Follow::resetData()
{
  followData.mode = MODE_RELATIVE;
  followData.yaw = YAW_TOTARGET;
  followData.target.latitude = api->getBroadcastData().pos.latitude;
  followData.target.longitude = api->getBroadcastData().pos.longitude;
  followData.target.height = api->getBroadcastData().pos.altitude;
  followData.target.angle = 0;
  followData.sensitivity = 1;
}

void Follow::start(FollowData *Data, CallBack callback, UserData userData)
{
  if (Data)
    followData = *Data;
  else
    resetData();
  api->send(2, encrypt, SET_MISSION, CODE_FOLLOW_START, &followData, sizeof(followData), 500, 2,
      callback ? callback : missionCallback, userData);
}

MissionACK Follow::start(FollowData *Data, int timeout)
{
  if (Data)
    followData = *Data;
  else
    resetData();
  api->send(2, encrypt, SET_MISSION, CODE_FOLLOW_START, &followData, sizeof(followData), 500, 2, 0,0);

  api->serialDevice->lockACK();
  api->serialDevice->wait(timeout);
  api->serialDevice->freeACK();

  return api->missionACKUnion.missionACK;
}

void Follow::stop(CallBack callback, UserData userData)
{
  uint8_t zero = 0;
  api->send(2, encrypt, SET_MISSION, CODE_FOLLOW_STOP, &zero, sizeof(zero), 500, 2,
      callback ? callback : missionCallback, userData);
}

MissionACK Follow::stop(int timeout)
{
  uint8_t zero = 0;
  api->send(2, encrypt, SET_MISSION, CODE_FOLLOW_STOP, &zero, sizeof(zero), 500, 2, 0, 0);

  api->serialDevice->lockACK();
  api->serialDevice->wait(timeout);
  api->serialDevice->freeACK();

  return api->missionACKUnion.missionACK;
}

void Follow::pause(bool isPause, CallBack callback, UserData userData)
{
  uint8_t followData = isPause ? 0 : 1;
  api->send(2, encrypt, SET_MISSION, CODE_FOLLOW_SETPAUSE, &followData, sizeof(followData), 500, 2,
      callback ? callback : missionCallback, userData);
}

MissionACK Follow::pause(bool isPause, int timeout)
{
  uint8_t followData = isPause ? 0 : 1;
  api->send(2, encrypt, SET_MISSION, CODE_FOLLOW_SETPAUSE, &followData, sizeof(followData), 500, 2, 0, 0);

  api->serialDevice->lockACK();
  api->serialDevice->wait(timeout);
  api->serialDevice->freeACK();

  return api->missionACKUnion.missionACK;
}

void Follow::updateTarget(FollowTarget target)
{
  followData.target = target;
  api->send(0, encrypt, SET_MISSION, CODE_FOLLOW_TARGET, &(followData.target), sizeof(FollowTarget));
}

void Follow::updateTarget(float64_t latitude, float64_t longitude, uint16_t height,
    uint16_t angle)
{
  followData.target.latitude = latitude;
  followData.target.longitude = longitude;
  followData.target.height = height;
  followData.target.angle = angle;
  api->send(0, encrypt, SET_MISSION, CODE_FOLLOW_TARGET, &(followData.target), sizeof(FollowTarget));
}

FollowData Follow::getData() const { return followData; }

void Follow::setData(const FollowData &value) { followData = value; }

void Follow::setMode(const Follow::MODE mode __UNUSED)
{
  API_LOG(api->getDriver(), STATUS_LOG, "no available mode but default");
  followData.mode = 0;
}

void Follow::setTarget(FollowTarget target) { followData.target = target; }

void Follow::setYawType(const Follow::YAW_TYPE type) { followData.yaw = type; }

void Follow::setSensitivity(const Follow::SENSITIVITY sense __UNUSED)
{
  API_LOG(api->getDriver(), STATUS_LOG, "no available mode but default");
  followData.sensitivity = 1;
}
