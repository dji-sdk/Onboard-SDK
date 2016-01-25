/*! @brief
 *  @file DJI_Codec.h
 *  @version 3.0
 *  @date Dec 16, 2015
 *
 *  @abstract
 *  Virtual Radio Control API for DJI onboardSDK library
 *
 *  @attention
 *  Project configuration:
 *
 *  @version features:
 *  -* @version V3.0
 *  -* DJI-onboard-SDK for Windows,QT,STM32,ROS,Cmake
 *  -* @date Dec 16, 2015
 *  -* @author william.wu
 *
 * */

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
    VirtualRC(CoreAPI *ControlAPI = 0);

    void setControl(bool enable, CutOff cutoffType);
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

} // namespace onboardSDK
} // namespace DJI

#endif // DJI_VIRTUALRC_H
