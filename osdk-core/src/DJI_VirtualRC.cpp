/**@file DJI_VirtualRC.cpp
 *  @version 3.1.7
 *  @date July 1st, 2016
 *
 *  @brief
 *  Virtual Radio Control API for DJI onboardSDK library
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */


#include "DJI_VirtualRC.h"

using namespace DJI::onboardSDK;

VirtualRC::VirtualRC(CoreAPI *ControlAPI)
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
  vrcData = Data; 
  //!api->send command was moved to this function from sendData(). 
  api->send(0, encrypt, SET_VIRTUALRC, CODE_VIRTUALRC_DATA, &vrcData, sizeof(vrcData));
}

//!This function will be deprecated in a future release. Please use sendData(VirtualRCData Data) instead. 
void VirtualRC::sendData()
{
  api->send(0, encrypt, SET_VIRTUALRC, CODE_VIRTUALRC_DATA, &vrcData, sizeof(vrcData));
}

//!This function will not be maintained and will be deprecated in a future release. Please use resetVRCData() instead.
void VirtualRC::resetData()
{
  vrcData.roll = 1024;
  vrcData.pitch = 1024;
  vrcData.throttle = 1024;
  vrcData.yaw = 1024;
  vrcData.gear = 1024;
  vrcData.reserved = 1024;
  vrcData.mode = 1024;
  vrcData.Channel_07 = 1024;
  vrcData.Channel_08 = 1024;
  vrcData.Channel_09 = 1024;
  vrcData.Channel_10 = 1024;
  vrcData.Channel_11 = 1024;
  vrcData.Channel_12 = 1024;
  vrcData.Channel_13 = 1024;
  vrcData.Channel_14 = 1024;
  vrcData.Channel_15 = 1024;
}

//!This function will be deprecated in the future. Please use neutralVRCSticks() instead. 
void VirtualRC::sendSafeModeData()
{
  resetData();
  sendData();
}

void VirtualRC::neutralVRCSticks()
{
  resetData();
  sendData();
}

//! @warning The return type will change to RCData in a future release.
RadioData VirtualRC::getRCData() const { return api->getBroadcastData().rc; }

CoreAPI *VirtualRC::getApi() const { return api; }
void VirtualRC::setApi(CoreAPI *value) { api = value; }

VirtualRCData VirtualRC::getVRCData() const { return vrcData; }
void VirtualRC::setVRCData(const VirtualRCData &value) 
{ 
  vrcData = value;         
}

bool VirtualRC::isVirtualRC() const
{
  return api->getBroadcastData().ctrlInfo.vrcStatus == 0 ? false : true;
}

//! @warning This function will be deprecated in a future release. Please use toRCData instead. 
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

RCData VirtualRC::toRCData(VirtualRCData &vData)
{
  RCData rcData;
  rcData.gear = (vData.gear == 1324) ? -454 : -10000;
  rcData.mode = (1024 - vData.mode ) * 10000 / 660;
  rcData.pitch = (vData.pitch - 1024) * (10000) / 660;
  rcData.roll = (vData.roll - 1024) * (10000) / 660;
  rcData.yaw = (vData.yaw - 1024) * (10000) / 660;
  rcData.throttle = (vData.throttle - 1024) * (10000) / 660;
  return rcData;
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
