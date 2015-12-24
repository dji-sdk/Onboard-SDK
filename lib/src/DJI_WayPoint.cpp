#include "DJI_WayPoint.h"

using namespace DJI::onboardSDK;

#ifndef STATIC_MEMORY
WayPoint::WayPoint(CoreAPI *ControlAPI)
{
    api = ControlAPI;
    index = 0;
}

void WayPoint::init(const WayPointInitData *info, CallBack callback, UserData userData) {}
#else
WayPoint::WayPoint(WayPointData *list, uint8_t len, CoreAPI *ControlAPI)
{
    api = ControlAPI;
    index = list;
    maxIndex = len;
}
#endif

void WayPoint::start(CallBack callback, UserData userData)
{
    uint8_t start = 0;
    api->send(2, 1, SET_MISSION, CODE_WAYPOINT_SETSTART, &start, sizeof(start), 500, 2,
              callback ? callback : missionCallback, userData);
}

void WayPoint::stop(CallBack callback, UserData userData)
{
    uint8_t stop = 1;
    api->send(2, 1, SET_MISSION, CODE_WAYPOINT_SETSTART, &stop, sizeof(stop), 500, 2,
              callback ? callback : missionCallback, userData);
}

void WayPoint::pause(bool isPause, CallBack callback, UserData userData)
{
    uint8_t data = isPause ? 0 : 1;
    api->send(2, 1, SET_MISSION, CODE_WAYPOINT_SETPAUSE, &data, sizeof(data), 500, 2,
              callback ? callback : missionCallback, userData);
}
WayPointInitData WayPoint::getInfo() const { return info; }

void WayPoint::setInfo(const WayPointInitData &value)
{
    info = value;
#ifndef STATIC_MEMORY
    if (index != 0)
        delete index;
    index = 0; // new WayPointData[info.indexNumber];
// if(index == NULL)
//  API_LOG(api->getDriver(),ERROR_LOG,"Lack of memory");
#else
    if (maxIndex < info.indexNumber)
        index = 0;
#endif // STATIC_MEMORY
}
WayPointData *WayPoint::getIndex() const { return index; }

void WayPoint::setIndex(WayPointData *value) { index = value; }
