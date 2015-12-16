#include "DJI_HotPoint.h"
#include <string.h>

using namespace DJI::onboardSDK;

HotPoint::HotPoint(CoreAPI *ControlAPI)
{
    api = ControlAPI;

    initData();
}

void HotPoint::initData()
{
    data.version = 0;

    data.altitude = api->getBroadcastData().pos.altitude;
    data.longtitude = api->getBroadcastData().pos.longtitude;
    data.latitude = api->getBroadcastData().pos.latitude;

    data.radius = 10;
    data.palstance = 15;
    data.clockwise = 1;
    data.startPoint = HotPoint::VIEW_NEARBY;
    data.yawMode = HotPoint::YAW_INSIDE;
}

void HotPoint::start(CallBack callback)
{
    api->send(2, 1, SET_MISSION, CODE_HOTPOINT_START, &data, sizeof(data),
              callback ? callback : startCallback, 500, 2);
}

void HotPoint::stop(CallBack callback)
{
    uint8_t zero = 0;
    api->send(2, 1, SET_MISSION, CODE_HOTPOINT_STOP, &zero, sizeof(zero),
              callback ? callback : commonCallback, 500, 2);
}

void HotPoint::resetPalstance(HotPoint::Palstance &Data, CallBack callback)
{
    data.palstance = Data.palstance;
    data.clockwise = Data.clockwise ? 1 : 0;
    api->send(2, 1, SET_MISSION, CODE_HOTPOINT_PALSTANCE, &Data, sizeof(Data),
              callback ? callback : commonCallback, 500, 2);
}

void HotPoint::resetPalstance(float32_t palstance, bool isClockwise,
                              CallBack callback)
{
    Palstance p;
    p.palstance = palstance;
    p.clockwise = isClockwise ? 1 : 0;
    resetPalstance(p, callback);
}

void HotPoint::resetRadius(float32_t degree, CallBack callback,UserData userData)
{
    api->send(2, 1, SET_MISSION, CODE_HOTPOINT_RADIUS, &degree, sizeof(degree), 500, 2,
              callback ? callback : commonCallback,userData);
}

void HotPoint::resetYaw(CallBack callback, UserData userData)
{
    uint8_t zero = 0;
    api->send(2, 1, SET_MISSION, CODE_HOTPOINT_SETYAW, &zero, sizeof(zero), 500,
              2, callback ? callback : commonCallback, userData);
}

void HotPoint::readData(CallBack callback, UserData userData)
{
    uint8_t zero = 0;
    api->send(2, 1, SET_MISSION, CODE_HOTPOINT_LOAD, &zero, sizeof(zero), 500,
              2, callback ? callback : commonCallback, userData);
}

void HotPoint::setData(const HotPointData &value) { data = value; }

void HotPoint::setHotPoint(float64_t longtitude, float64_t latitude,
                           float64_t altitude)
{
    data.longtitude = longtitude;
    data.latitude = latitude;
    data.altitude = altitude;
}

void HotPoint::setHotPoint(GPSData gps)
{
    data.longtitude = gps.longtitude;
    data.latitude = gps.latitude;
    data.altitude = gps.altitude;
}

void HotPoint::setRadius(float64_t meter) { data.radius = meter; }

void HotPoint::setPalstance(float32_t defree) { data.palstance = defree; }

void HotPoint::setClockwise(bool isClockwise)
{
    data.clockwise = isClockwise ? 1 : 0;
}

void HotPoint::setCameraView(HotPoint::View view) { data.startPoint = view; }

void HotPoint::setYawMode(HotPoint::YawMode mode) { data.yawMode = mode; }

HotPointData HotPoint::getData() const { return data; }

void HotPoint::startCallback(CoreAPI *This, Header *header,
                             UserData userdata __UNUSED)
{
    StartACK ack;
    if (header->length - EXC_DATA_SIZE <= sizeof(StartACK))
    {
        memcpy((unsigned char *)&ack, (unsigned char *)header + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        API_LOG(This->getDriver(), STATUS_LOG,
                "Start ACK has Max radius %f, ack 0x%X", ack.maxRadius,
                ack.ack);
        if (!This->decodeMissionStatus(ack.ack))
            API_LOG(This->getDriver(), ERROR_LOG, "decode ack error 0x%X",
                    ack.ack);
    }
    else
    {
        API_LOG(This->getDriver(), ERROR_LOG,
                "ACK is exception,seesion id %d,sequence %d\n",
                header->sessionID, header->sequence_number);
    }
}

void HotPoint::commonCallback(CoreAPI *This, Header *header,
                              UserData userdata __UNUSED)
{
    API_LOG(This->getDriver(), STATUS_LOG, "HotPoint common");
    CommonACK ack;
    if (header->length - EXC_DATA_SIZE <= sizeof(ack))
    {
        memcpy((unsigned char *)&ack, (unsigned char *)header + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        if (!This->decodeMissionStatus(ack))
            API_LOG(This->getDriver(), ERROR_LOG, "decode ack error 0x%X", ack);
    }
    else
    {
        API_LOG(This->getDriver(), ERROR_LOG,
                "ACK is exception,seesion id %d,sequence %d\n",
                header->sessionID, header->sequence_number);
    }
}

void HotPoint::readCallback(CoreAPI *This, Header *header,
                            UserData userdata __UNUSED)
{
    API_LOG(This->getDriver(), STATUS_LOG, "HotPoint common");
    CommonACK ack;
    if (header->length - EXC_DATA_SIZE <= sizeof(ack))
    {
        memcpy((unsigned char *)&ack, (unsigned char *)header + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        if (!This->decodeMissionStatus(ack))
            API_LOG(This->getDriver(), ERROR_LOG, "decode ack error 0x%X", ack);
    }
    else
    {
        API_LOG(This->getDriver(), ERROR_LOG,
                "ACK is exception,seesion id %d,sequence %d\n",
                header->sessionID, header->sequence_number);
    }
}
