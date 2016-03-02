#include "DJI_VirtualRC.h"

using namespace DJI::onboardSDK;

DJI::onboardSDK::VirtualRC::VirtualRC(CoreAPI *ControlAPI)
{
    api = ControlAPI;
    resetData();
}

void VirtualRC::setControl(bool enable, VirtualRC::CutOff cutoffType)
{
    VirtualRCSetting setting;
    setting.cutoff = cutoffType;
    setting.enable = enable ? 1 : 0;
    api->send(0, encrypt, SET_VIRTUALRC, CODE_VIRTUALRC_SETTINGS, &setting, sizeof(setting));
}

void VirtualRC::sendData(VirtualRCData Data)
{
    data = Data;
    sendData();
}

void VirtualRC::sendData()
{
    api->send(0, encrypt, SET_VIRTUALRC, CODE_VIRTUALRC_DATA, &data, sizeof(data));
}

void VirtualRC::resetData()
{
    data.roll = 1024;
    data.pitch = 1024;
    data.throttle = 1024;
    data.yaw = 1024;
    data.gear = 1024;
    data.reserved = 1024;
    data.mode = 1024;
    data.Channel_07 = 1024;
    data.Channel_08 = 1024;
    data.Channel_09 = 1024;
    data.Channel_10 = 1024;
    data.Channel_11 = 1024;
    data.Channel_12 = 1024;
    data.Channel_13 = 1024;
    data.Channel_14 = 1024;
    data.Channel_15 = 1024;
}

RadioData VirtualRC::getVRCdata() const { return api->getBroadcastData().rc; }

CoreAPI *VirtualRC::getApi() const { return api; }

void VirtualRC::setApi(CoreAPI *value) { api = value; }
