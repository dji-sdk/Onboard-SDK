/** @brief
*  @file DJI_WayPoint.cpp
*  @version 3.1.7
*  @date July 1st, 2016
*
*  @brief
*  Waypoint flight API for DJI onboardSDK library
*
*  @copyright 2016 DJI. All right reserved.
*
*/

#include "DJI_WayPoint.h"
#include <string.h>

using namespace DJI;
using namespace DJI::onboardSDK;

#ifndef STATIC_MEMORY
WayPoint::WayPoint(CoreAPI *ControlAPI)
{
  api = ControlAPI;
  index = 0;
}
#else
WayPoint::WayPoint(WayPointData *list, uint8_t len, CoreAPI *ControlAPI)
{
  api = ControlAPI;
  index = list;
  maxIndex = len;
}
#endif // STATIC_MEMORY

void WayPoint::init(WayPointInitData *Info, CallBack callback, UserData userData)
{
  if (Info)
    setInfo(*Info);

  api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_INIT, &info, sizeof(info), 500, 2,
    callback ? callback : missionCallback, userData);
}

MissionACK WayPoint::init(WayPointInitData *Info, int timeout)
{
  if (Info)
    setInfo(*Info);

  api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_INIT, &info, sizeof(info), 500, 2, 0, 0);

  api->serialDevice->lockACK();
  api->serialDevice->wait(timeout);
  api->serialDevice->freeACK();

  return api->missionACKUnion.missionACK;
}

void WayPoint::start(CallBack callback, UserData userData)
{
  uint8_t start = 0;

  api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_SETSTART, &start, sizeof(start), 500, 2,
    callback ? callback : missionCallback, userData);
}

MissionACK WayPoint::start(int timeout)
{
  uint8_t start = 0;

  api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_SETSTART, &start, sizeof(start), 500, 2, 0, 0);

  api->serialDevice->lockACK();
  api->serialDevice->wait(timeout);
  api->serialDevice->freeACK();

  return api->missionACKUnion.missionACK;
}

void WayPoint::stop(CallBack callback, UserData userData)
{
  uint8_t stop = 1;

  api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_SETSTART, &stop, sizeof(stop), 500, 2,
    callback ? callback : missionCallback, userData);
}

MissionACK WayPoint::stop(int timeout)
{
  uint8_t stop = 1;

  api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_SETSTART, &stop, sizeof(stop), 500, 2, 0,0);

  api->serialDevice->lockACK();
  api->serialDevice->wait(timeout);
  api->serialDevice->freeACK();

  return api->missionACKUnion.missionACK;
}

void WayPoint::pause(bool isPause, CallBack callback, UserData userData)
{
 uint8_t data = isPause ? 0 : 1;
 
 api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_SETPAUSE, &data, sizeof(data), 500, 2,
    callback ? callback : missionCallback, userData);
}

MissionACK WayPoint::pause(bool isPause, int timeout)
{
 uint8_t data = isPause ? 0 : 1;

 api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_SETPAUSE, &data, sizeof(data), 500, 2, 0, 0);

 api->serialDevice->lockACK();
 api->serialDevice->wait(timeout);
 api->serialDevice->freeACK();

  return api->missionACKUnion.missionACK;
}

void WayPoint::readIdleVelocity(CallBack callback, UserData userData)
{
  uint8_t zero = 0;
  
  api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_GETVELOCITY, &zero, sizeof(zero), 500, 2,
    callback ? callback : idleVelocityCallback, userData ? userData : this);
}

bool WayPoint::uploadIndexData(WayPointData *data, CallBack callback, UserData userData)
{
  setIndex(data, data->index);
  return uploadIndexData(data->index, callback ? callback : uploadIndexDataCallback, userData);
}

bool WayPoint::uploadIndexData(uint8_t pos, CallBack callback, UserData userData)
{
  WayPointData send;
  if (pos < info.indexNumber)
    send = index[pos];
  else
    return false; //! @note range error

  api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_ADDPOINT, &send, sizeof(send), 1000, 4,
    callback ? callback : uploadIndexDataCallback, userData);

  return true;
}

WayPointDataACK WayPoint::uploadIndexData(WayPointData *data, int timeout)
{
  WayPointData wpData;

  setIndex(data, data->index);

  if (data->index < info.indexNumber)
     wpData = index[data->index];
   else
#ifndef STM32
    throw std::runtime_error("Range error\n");
#endif

  api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_ADDPOINT, &wpData, sizeof(wpData), 1000, 4, 0, 0);

  api->serialDevice->lockACK();
  api->serialDevice->wait(timeout);
  api->serialDevice->freeACK();

  return api->missionACKUnion.waypointDataACK;
}

/**
 * @note Read Waypoint mission settings from the flight controller
 *
 * @return Information about uploaded Waypoint(s)
 *
 */
WayPointInitACK WayPoint::getWaypointSettings(int timeout)
{
  uint8_t arbNumber = 0;
  api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_INFO_READ, &arbNumber, sizeof(arbNumber), 1000, 4, 0, 0);

  api->serialDevice->lockACK();
  api->serialDevice->wait(timeout);
  api->serialDevice->freeACK();

  return api->missionACKUnion.waypointInitACK;
}

void WayPoint::getWaypointSettings(CallBack callback, UserData userData)
{
  uint8_t arbNumber = 0;
  api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_INFO_READ, &arbNumber, sizeof(arbNumber), 1000, 4,
    callback ? callback : getWaypointSettingsCallback, userData ? userData : this);
}

WayPointDataACK WayPoint::getIndex(uint8_t index, int timeout)
{
  api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_INDEX_READ, &index, sizeof(index), 1000, 4, 0, 0);

  api->serialDevice->lockACK();
  api->serialDevice->wait(timeout);
  api->serialDevice->freeACK();

  return api->missionACKUnion.waypointDataACK;
}

void WayPoint::getIndex(uint8_t index, CallBack callback, UserData userData)
{
  api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_INDEX_READ, &index, sizeof(index), 1000, 4,
    callback ? callback : uploadIndexDataCallback, userData ? userData : this);
}

void WayPoint::updateIdleVelocity(float32_t meterPreSecond, CallBack callback,
                                  UserData userData)
{
  api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_SETVELOCITY, &meterPreSecond,
    sizeof(meterPreSecond), 500, 2, callback ? callback : idleVelocityCallback,
    userData ? userData : this);
}

WayPointInitData WayPoint::getInfo() const
{ 
  return info; 
}

void WayPoint::setInfo(const WayPointInitData &value)
{
  //! @todo set information for way point
  info = value;
  for (int i = 0; i < 16; ++i) info.reserved[i] = 0;
#ifndef STATIC_MEMORY
  if (index != 0)
    delete index;
  index = 0;
#else
  if (maxIndex < info.indexNumber)
    index = 0;
#endif // STATIC_MEMORY
}

void WayPoint::idleVelocityCallback(CoreAPI *api, Header *protocolHeader, UserData wpapi)
{
  WayPoint *wp = (WayPoint *)wpapi;
  WayPointVelocityACK ack;

  if (protocolHeader->length - EXC_DATA_SIZE <= sizeof(ack))
    memcpy((unsigned char *)&ack, ((unsigned char *)protocolHeader) + sizeof(Header),
            (protocolHeader->length - EXC_DATA_SIZE));
  else
  {
    API_LOG(api->getDriver(), ERROR_LOG, "ACK is exception, session id %d,sequence %d\n",
            protocolHeader->sessionID, protocolHeader->sequenceNumber);
    return;
  }
  api->decodeMissionStatus(ack.ack);
  wp->info.idleVelocity = ack.idleVelocity;
  API_LOG(api->getDriver(), STATUS_LOG, "Current idle velocity: %f", wp->info.idleVelocity);
}

void WayPoint::getWaypointSettingsCallback(CoreAPI *api, Header *protocolHeader, UserData wpapi)
{
  WayPoint *wp = (WayPoint *)wpapi;
  WayPointInitACK ack;

  if (protocolHeader->length - EXC_DATA_SIZE <= sizeof(ack))
    memcpy((unsigned char *)&ack, ((unsigned char *)protocolHeader) + sizeof(Header),
            (protocolHeader->length - EXC_DATA_SIZE));
  else
  {
    API_LOG(api->getDriver(), ERROR_LOG, "ACK is exception, session id %d,sequence %d\n",
            protocolHeader->sessionID, protocolHeader->sequenceNumber);
    return;
  }

  api->decodeMissionStatus(ack.ack);
  wp->info = ack.data;
  API_LOG(api->getDriver(), STATUS_LOG, "Index number: %d\n", wp->info.indexNumber);
}

void WayPoint::uploadIndexDataCallback(CoreAPI *api, Header *protocolHeader, UserData wpapi __UNUSED)
{
  WayPointDataACK ack;

  if (protocolHeader->length - EXC_DATA_SIZE <= sizeof(ack))
    memcpy((unsigned char *)&ack, ((unsigned char *)protocolHeader) + sizeof(Header),
            (protocolHeader->length - EXC_DATA_SIZE));
  else
  {
    API_LOG(api->getDriver(), ERROR_LOG, "ACK is exception, session id %d,sequence %d\n",
            protocolHeader->sessionID, protocolHeader->sequenceNumber);
    return;
  }
  api->decodeMissionStatus(ack.ack);
  API_LOG(api->getDriver(), STATUS_LOG, "Index number: %d\n", ack.data.index);
  API_LOG(api->getDriver(), STATUS_LOG, "Action number: %d\n", ack.data.actionNumber);
  API_LOG(api->getDriver(), STATUS_LOG, "Latitude: %f\n", ack.data.latitude);
  API_LOG(api->getDriver(), STATUS_LOG, "Longitude: %f\n", ack.data.longitude);
  API_LOG(api->getDriver(), STATUS_LOG, "Altitude: %f\n", ack.data.altitude);
  API_LOG(api->getDriver(), STATUS_LOG, "Yaw: %d\n", ack.data.yaw);
  API_LOG(api->getDriver(), STATUS_LOG, "Gimbal pitch: %d\n", ack.data.gimbalPitch);
}

void WayPoint::setIndex(WayPointData *value, size_t pos)
{
  if (index == 0)
  {
    index = new WayPointData[info.indexNumber];
    if (index == NULL)
    {
      API_LOG(api->getDriver(), ERROR_LOG, "Lack of memory\n");
      return;
    }
  }
  index[pos] = *value;
  for (int i = 0; i < 8; ++i) index[pos].reserved[i] = 0;
}
