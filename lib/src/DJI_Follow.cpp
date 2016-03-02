#include "DJI_Follow.h"

using namespace DJI::onboardSDK;

Follow::Follow(CoreAPI *ControlAPI)
{
    api = ControlAPI;
    resetData();
}

void Follow::resetData()
{
    data.mode = MODE_RELATIVE;
    data.yaw = YAW_TOTARGET;
    data.target.latitude = api->getBroadcastData().pos.latitude;
    data.target.longitude = api->getBroadcastData().pos.longitude;
    data.target.height = api->getBroadcastData().pos.altitude;
    data.target.angle = 0;
    data.sensitivity = 1;
}

void Follow::start(FollowData *Data, CallBack callback, UserData userData)
{
    if (Data)
        data = *Data;
    else
        resetData();
    api->send(2, encrypt, SET_MISSION, CODE_FOLLOW_START, &data, sizeof(data), 500, 2,
              callback ? callback : missionCallback, userData);
}

void Follow::stop(CallBack callback, UserData userData)
{
    uint8_t zero = 0;
    api->send(2, encrypt, SET_MISSION, CODE_FOLLOW_STOP, &zero, sizeof(zero), 500, 2,
              callback ? callback : missionCallback, userData);
}

void Follow::pause(bool isPause, CallBack callback, UserData userData)
{
    uint8_t data = isPause ? 0 : 1;
    api->send(2, encrypt, SET_MISSION, CODE_FOLLOW_SETPAUSE, &data, sizeof(data), 500, 2,
              callback ? callback : missionCallback, userData);
}

void Follow::updateTarget(FollowTarget target)
{
    data.target = target;
    api->send(0, encrypt, SET_MISSION, CODE_FOLLOW_TARGET, &(data.target), sizeof(FollowTarget));
}

void Follow::updateTarget(float64_t latitude, float64_t longitude, uint16_t height,
                          uint16_t angle)
{
    FollowTarget target;
    target.latitude = latitude;
    target.longitude = longitude;
    target.height = height;
    target.angle = angle;
    updateTarget(target);
}

FollowData Follow::getData() const { return data; }

void Follow::setData(const FollowData &value) { data = value; }

void Follow::setMode(const Follow::MODE mode __UNUSED)
{
    API_LOG(api->getDriver(), STATUS_LOG, "no available mode but default");
    data.mode = 0;
}

void Follow::setTarget(FollowTarget target) { data.target = target; }

void Follow::setYawType(const Follow::YAW_TYPE type) { data.yaw = type; }

void Follow::setSensitivity(const Follow::SENSITIVITY sense __UNUSED)
{
    API_LOG(api->getDriver(), STATUS_LOG, "no available mode but default");
    data.sensitivity = 1;
}
