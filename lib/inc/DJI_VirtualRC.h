#ifndef DJI_VIRTUALRC_H
#define DJI_VIRTUALRC_H

#include "DJI_API.h"

namespace DJI{
namespace onboardSDK {

class VirtualRC
{
  public:
    enum CutOff
    {
        CutOff_ToLogic = 0,
        CutOff_ToRealRC = 1
    };

  public:
    VirtualRC(CoreAPI *ContorlAPI = 0);

    void sentContorl(bool enable, CutOff cutoffType);
    void sendData(VirtualRCData Data);
    void sendData();
    void resetData();

    RadioData getVRCdata() const;

  public:
    CoreAPI *getApi() const;
    void setApi(CoreAPI *value);

  private:
    CoreAPI *api;
    VirtualRCData data;
};

}
}

#endif // DJI_VIRTUALRC_H
