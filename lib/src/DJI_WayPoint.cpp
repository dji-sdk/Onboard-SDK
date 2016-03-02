#include "DJI_WayPoint.h"
#include <string.h>

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

void WayPoint::init(const WayPointInitData *Info, CallBack callback, UserData userData)
{
    if (Info)
        setInfo(*Info);
    api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_INIT, &info, sizeof(info), 500, 2,
              callback ? callback : missionCallback, userData);
}

void WayPoint::start(CallBack callback, UserData userData)
{
    uint8_t start = 0;
    api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_SETSTART, &start, sizeof(start), 500, 2,
              callback ? callback : missionCallback, userData);
}

void WayPoint::stop(CallBack callback, UserData userData)
{
    uint8_t stop = 1;
    api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_SETSTART, &stop, sizeof(stop), 500, 2,
              callback ? callback : missionCallback, userData);
}

void WayPoint::pause(bool isPause, CallBack callback, UserData userData)
{
    uint8_t data = isPause ? 0 : 1;
    api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_SETPAUSE, &data, sizeof(data), 500, 2,
              callback ? callback : missionCallback, userData);
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
    return uploadIndexData(data->index,callback,userData);
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

void WayPoint::updateIdleVelocity(float32_t meterPreSecond, CallBack callback,
                                  UserData userData)
{
    api->send(2, encrypt, SET_MISSION, CODE_WAYPOINT_SETVELOCITY, &meterPreSecond,
              sizeof(meterPreSecond), 500, 2, callback ? callback : idleVelocityCallback,
              userData ? userData : this);
}
WayPointInitData WayPoint::getInfo() const { return info; }

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
WayPointData *WayPoint::getIndex() const { return index; }

WayPointData *WayPoint::getIndex(size_t pos) const { return &(index[pos]); }

void WayPoint::idleVelocityCallback(CoreAPI *This, Header *header, UserData wpThis)
{
    WayPoint *wp = (WayPoint *)wpThis;
    WayPointVelocityACK ack;

    if (header->length - EXC_DATA_SIZE <= sizeof(ack))
        memcpy((unsigned char *)&ack, ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
    else
    {
        API_LOG(This->getDriver(), ERROR_LOG, "ACK is exception,seesion id %d,sequence %d\n",
                header->sessionID, header->sequenceNumber);
        return;
    }
    This->decodeMissionStatus(ack.ack);
    wp->info.idleVelocity = ack.idleVelocity;
    API_LOG(This->getDriver(), STATUS_LOG, "Current idle velocity: %f", wp->info.idleVelocity);
}

void WayPoint::readInitDataCallback(CoreAPI *This, Header *header, UserData wpThis)
{
    WayPoint *wp = (WayPoint *)wpThis;
    WayPointInitACK ack;

    if (header->length - EXC_DATA_SIZE <= sizeof(ack))
        memcpy((unsigned char *)&ack, ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
    else
    {
        API_LOG(This->getDriver(), ERROR_LOG, "ACK is exception,seesion id %d,sequence %d\n",
                header->sessionID, header->sequenceNumber);
        return;
    }
    This->decodeMissionStatus(ack.ack);
    wp->info = ack.data;
    API_LOG(This->getDriver(), STATUS_LOG, "Index number: %d", wp->info.indexNumber);
}

void WayPoint::uploadIndexDataCallback(CoreAPI *This, Header *header, UserData wpThis __UNUSED)
{
    WayPointDataACK ack;

    if (header->length - EXC_DATA_SIZE <= sizeof(ack))
        memcpy((unsigned char *)&ack, ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
    else
    {
        API_LOG(This->getDriver(), ERROR_LOG, "ACK is exception,seesion id %d,sequence %d\n",
                header->sessionID, header->sequenceNumber);
        return;
    }
    This->decodeMissionStatus(ack.ack);
    API_LOG(This->getDriver(), STATUS_LOG, "Index number: %d", ack.index);
}

void WayPoint::setIndex(WayPointData *value, size_t pos)
{
    if (index == 0)
    {
        index = new WayPointData[info.indexNumber];
        if (index == NULL)
        {
            API_LOG(api->getDriver(), ERROR_LOG, "Lack of memory");
            return ;
        }
    }
    index[pos] = *value;
    for (int i = 0; i < 8; ++i) index[pos].reserved[i] = 0;
}
