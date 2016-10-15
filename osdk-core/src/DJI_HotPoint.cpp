/** @file DJI_HotPoint.cpp
 *  @version 3.1.7
 *  @date July 1st, 2016
 *
 *  @brief
 *  Hotpoint (Point of Interest) flight Control API for DJI onboardSDK library
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */

#include "DJI_HotPoint.h"
#include <string.h>

using namespace DJI;
using namespace DJI::onboardSDK;

HotPoint::HotPoint(CoreAPI *ControlAPI)
{
  api = ControlAPI;

  initData();
}

void HotPoint::initData()
{
  hotPointData.version = 0;

  hotPointData.height = api->getBroadcastData().pos.altitude;
  hotPointData.longitude = api->getBroadcastData().pos.longitude;
  hotPointData.latitude = api->getBroadcastData().pos.latitude;

  hotPointData.radius = 10;
  hotPointData.yawRate = 15;
  hotPointData.clockwise = 1;
  hotPointData.startPoint = HotPoint::VIEW_NEARBY;
  hotPointData.yawMode = HotPoint::YAW_INSIDE;
}

void HotPoint::start(CallBack callback, UserData userData)
{
  api->send(2, encrypt, SET_MISSION, CODE_HOTPOINT_START, &hotPointData, sizeof(hotPointData), 500, 2,
      callback ? callback : startCallback, userData);
}

HotPointStartACK HotPoint::start(int timeout)
{
  api->send(2, encrypt, SET_MISSION, CODE_HOTPOINT_START, &hotPointData, sizeof(hotPointData), 500, 2, 0, 0);

  api->serialDevice->lockACK();
  api->serialDevice->wait(timeout);
  api->serialDevice->freeACK();

  return api->missionACKUnion.hotpointStartACK;
}

void HotPoint::stop(CallBack callback, UserData userData)
{
  uint8_t zero = 0;
  api->send(2, encrypt, SET_MISSION, CODE_HOTPOINT_STOP, &zero, sizeof(zero), 500, 2,
      callback ? callback : missionCallback, userData);
}

MissionACK HotPoint::stop(int timeout)
{
  uint8_t zero = 0;
  api->send(2, encrypt, SET_MISSION, CODE_HOTPOINT_STOP, &zero, sizeof(zero), 500, 2, 0, 0);

  api->serialDevice->lockACK();
  api->serialDevice->wait(timeout);
  api->serialDevice->freeACK();

  return api->missionACKUnion.missionACK;
}

void HotPoint::pause(bool isPause, CallBack callback, UserData userData)
{
  uint8_t data = isPause ? 0 : 1;
  api->send(2, encrypt, SET_MISSION, CODE_HOTPOINT_SETPAUSE, &data, sizeof(data), 500, 2,
      callback ? callback : missionCallback, userData);
}

MissionACK HotPoint::pause(bool isPause, int timeout)
{
  uint8_t data = isPause ? 0 : 1;
  api->send(2, encrypt, SET_MISSION, CODE_HOTPOINT_SETPAUSE, &data, sizeof(data), 500, 2, 0, 0);

  api->serialDevice->lockACK();
  api->serialDevice->wait(timeout);
  api->serialDevice->freeACK();

  return api->missionACKUnion.missionACK;
}

void HotPoint::updateYawRate(HotPoint::YawRate &Data, CallBack callback, UserData userData)
{
  hotPointData.yawRate = Data.yawRate;
  hotPointData.clockwise = Data.clockwise ? 1 : 0;
  api->send(2, encrypt, SET_MISSION, CODE_HOTPOINT_YAWRATE, &Data, sizeof(Data), 500, 2,
      callback ? callback : missionCallback, userData);
}

MissionACK HotPoint::updateYawRate(HotPoint::YawRate &Data, int timeout)
{
  hotPointData.yawRate = Data.yawRate;
  hotPointData.clockwise = Data.clockwise ? 1 : 0;
  api->send(2, encrypt, SET_MISSION, CODE_HOTPOINT_YAWRATE, &Data, sizeof(Data), 500, 2, 0, 0);

  api->serialDevice->lockACK();
  api->serialDevice->wait(timeout);
  api->serialDevice->freeACK();

  return api->missionACKUnion.missionACK;
}

void HotPoint::updateYawRate(float32_t yawRate, bool isClockwise, CallBack callback,
    UserData userData)
{
  YawRate p;
  p.yawRate = yawRate;
  p.clockwise = isClockwise ? 1 : 0;
  updateYawRate(p, callback, userData);
}

void HotPoint::updateRadius(float32_t meter, CallBack callback, UserData userData)
{
  api->send(2, encrypt, SET_MISSION, CODE_HOTPOINT_RADIUS, &meter, sizeof(meter), 500, 2,
      callback ? callback : missionCallback, userData);
}

MissionACK HotPoint::updateRadius(float32_t meter, int timeout)
{
  api->send(2, encrypt, SET_MISSION, CODE_HOTPOINT_RADIUS, &meter, sizeof(meter), 500, 2, 0, 0);

  api->serialDevice->lockACK();
  api->serialDevice->wait(timeout);
  api->serialDevice->freeACK();

  return api->missionACKUnion.missionACK;
}

void HotPoint::resetYaw(CallBack callback, UserData userData)
{
  uint8_t zero = 0;
  api->send(2, encrypt, SET_MISSION, CODE_HOTPOINT_SETYAW, &zero, sizeof(zero), 500, 2,
      callback ? callback : missionCallback, userData);
}

MissionACK HotPoint::resetYaw(int timeout)
{
  uint8_t zero = 0;
  api->send(2, encrypt, SET_MISSION, CODE_HOTPOINT_SETYAW, &zero, sizeof(zero), 500, 2, 0, 0);

  api->serialDevice->lockACK();
  api->serialDevice->wait(timeout);
  api->serialDevice->freeACK();

  return api->missionACKUnion.missionACK;
}

void HotPoint::readData(CallBack callback, UserData userData)
{
  uint8_t zero = 0;
  api->send(2, encrypt, SET_MISSION, CODE_HOTPOINT_LOAD, &zero, sizeof(zero), 500, 2,
      callback ? callback : missionCallback, userData);
}

MissionACK HotPoint::readData(int timeout)
{
  uint8_t zero = 0;
  api->send(2, encrypt, SET_MISSION, CODE_HOTPOINT_LOAD, &zero, sizeof(zero), 500, 2, 0, 0);

  api->serialDevice->lockACK();
  api->serialDevice->wait(timeout);
  api->serialDevice->freeACK();

  return api->missionACKUnion.missionACK;
}

void HotPoint::setData(const HotPointData &value)
{
  hotPointData = value;
  hotPointData.version = 0;
}

void HotPoint::setHotPoint(float64_t longtitude, float64_t latitude, float64_t altitude)
{
  hotPointData.longitude = longtitude;
  hotPointData.latitude = latitude;
  hotPointData.height = altitude;
}

void HotPoint::setHotPoint(GPSPositionData gps)
{
  hotPointData.longitude = gps.longitude;
  hotPointData.latitude = gps.latitude;
  hotPointData.height = gps.altitude;
}

void HotPoint::setRadius(float64_t meter) { hotPointData.radius = meter; }

void HotPoint::setYawRate(float32_t degree) { hotPointData.yawRate = degree; }

void HotPoint::setClockwise(bool isClockwise) { hotPointData.clockwise = isClockwise ? 1 : 0; }

void HotPoint::setCameraView(HotPoint::View view) { hotPointData.startPoint = view; }

void HotPoint::setYawMode(HotPoint::YawMode mode) { hotPointData.yawMode = mode; }

HotPointData HotPoint::getData() const { return hotPointData; }

void HotPoint::startCallback(CoreAPI *api, Header *protocolHeader, UserData userdata __UNUSED)
{
  HotPointStartACK ack;
  if (protocolHeader->length - EXC_DATA_SIZE <= sizeof(HotPointStartACK))
  {
    memcpy((unsigned char *)&ack, (unsigned char *)protocolHeader + sizeof(Header),
        (protocolHeader->length - EXC_DATA_SIZE));
    API_LOG(api->getDriver(), STATUS_LOG, "Start ACK has Max radius %f, ACK 0x%X",
        ack.maxRadius, ack.ack);
    if (!api->decodeMissionStatus(ack.ack))
      API_LOG(api->getDriver(), ERROR_LOG, "Decode ACK error 0x%X", ack.ack);
  }
  else
  {
    API_LOG(api->getDriver(), ERROR_LOG, "ACK is exception,session id %d,sequence %d\n",
        protocolHeader->sessionID, protocolHeader->sequenceNumber);
  }
}

void HotPoint::readCallback(CoreAPI *api, Header *protocolHeader, UserData userdata)
{
  HotPoint *hp = (HotPoint *)userdata;
  HotPointReadACK ack;
  if (protocolHeader->length - EXC_DATA_SIZE <= sizeof(HotPointReadACK))
  {
    memcpy((unsigned char *)&ack, (unsigned char *)protocolHeader + sizeof(Header),
        (protocolHeader->length - EXC_DATA_SIZE));
    if (!api->decodeMissionStatus(ack.ack))
      API_LOG(api->getDriver(), ERROR_LOG, "Decode ACK error 0x%X", ack.ack);
    hp->hotPointData = ack.data;
  }
  else
  {
    API_LOG(api->getDriver(), ERROR_LOG, "ACK is exception,session id %d,sequence %d\n",
        protocolHeader->sessionID, protocolHeader->sequenceNumber);
  }
}
