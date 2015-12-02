#include <DJI_API.h>

using namespace DJI::onboardSDK;

DJI::onboardSDK::VirtualRC::VirtualRC(CoreAPI *ContorlAPI)
{
    api = ContorlAPI;
    resetData();
}

void VirtualRC::sentContorl(bool enable, VirtualRC::CutOff cutoffType)
{
    VirtualRCSetting setting;
    setting.cutoff = cutoffType;
    setting.enable = enable ? 1 : 0;
    api->send(0, 1, SET_VIRTUALRC, CODE_VIRTUALRC_SETTINGS, &setting,
              sizeof(setting));
}

void VirtualRC::sendData(VirtualRCData Data)
{
    data = Data;
    sendData();
}

void VirtualRC::sendData()
{
    api->send(0, 1, SET_VIRTUALRC, CODE_VIRTUALRC_DATA, &data, sizeof(data));
}

void VirtualRC::resetData()
{
    data = { 1024, 1024, 1024, 1024, 1024, 1684, 1024, 1024,
             1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024 };
}

CoreAPI *VirtualRC::getApi() const { return api; }

void VirtualRC::setApi(CoreAPI *value) { api = value; }
