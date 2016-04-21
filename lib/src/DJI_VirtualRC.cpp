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

void VirtualRC::sendSafeModeData()
{
    resetData();
    sendData();
}

RadioData VirtualRC::getRCData() const { return api->getBroadcastData().rc; }

CoreAPI *VirtualRC::getApi() const { return api; }
void VirtualRC::setApi(CoreAPI *value) { api = value; }

VirtualRCData VirtualRC::getVRCData() const { return data; }
void VirtualRC::setVRCData(const VirtualRCData &value) { data = value; }

bool VirtualRC::isVirtualRC() const
{
    return api->getBroadcastData().ctrlInfo.vrcStatus == 0 ? false : true;
}

RadioData VirtualRC::toRadioData(VirtualRCData &vData)
{
    RadioData rd;
    rd.gear = (vData.gear == 1324) ? -454 : -10000;
    rd.mode = (1024 - vData.mode ) * 10000 / 660;
    rd.pitch = (vData.pitch - 1024) * (10000) / 660;
    rd.roll = (vData.roll - 1024) * (10000) / 660;
    rd.yaw = (vData.yaw - 1024) * (10000) / 660;
    rd.throttle = (vData.throttle - 1024) * (10000) / 660;
    return rd;
}

VirtualRCData VirtualRC::toVirtualRCData(RadioData &rData)
{
    VirtualRCData vd;
    vd.yaw = rData.yaw * 660 / 10000 + 1024;
    vd.throttle = rData.throttle * 660 / 10000 + 1024;
    vd.pitch = rData.pitch * 660 / 10000 + 1024;
    vd.roll = rData.roll * 660 / 10000 + 1024;
    vd.gear = (rData.gear == -4545) ? 1324 : 1684;
    vd.reserved = 1024;
    vd.mode = rData.mode * 660 / (-10000) + 1024;
    vd.Channel_07 = 1024;
    vd.Channel_08 = 1024;
    vd.Channel_09 = 1024;
    vd.Channel_10 = 1024;
    vd.Channel_11 = 1024;
    vd.Channel_12 = 1024;
    vd.Channel_13 = 1024;
    vd.Channel_14 = 1024;
    vd.Channel_15 = 1024;
    return vd;
}
