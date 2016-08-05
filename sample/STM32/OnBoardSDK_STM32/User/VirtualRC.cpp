/*! @file VirtualRC.cpp
 *
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Test Virtual RC functionality of the onboard SDK.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#include "VirtualRC.h"

extern VirtualRC virtualrc;
extern VirtualRCData myVRCdata;
VirtualRCData VRCdataTurnF =
{ 1024, 1024, 1024, 1024, 1024, 1024, 496, 1024, 1024, 1024, 1024, 1024, 1024,
    1024, 1024 };

void VRC_TakeControl()
{
  virtualrc.setControl(1, virtualrc.CutOff_ToLogic);
  myVRCdata = VRCdataTurnF;
}

void VRCResetData()
{
  virtualrc.setControl(1, virtualrc.CutOff_ToLogic);
  myVRCdata.roll = 1024;
  myVRCdata.pitch = 1024;
  myVRCdata.throttle = 1024;
  myVRCdata.yaw = 1024;
  myVRCdata.gear = 1024;
  myVRCdata.reserved = 1024;
  myVRCdata.mode = 1024;
  myVRCdata.Channel_07 = 1024;
  myVRCdata.Channel_08 = 1024;
  myVRCdata.Channel_09 = 1024;
  myVRCdata.Channel_10 = 1024;
  myVRCdata.Channel_11 = 1024;
  myVRCdata.Channel_12 = 1024;
  myVRCdata.Channel_13 = 1024;
  myVRCdata.Channel_14 = 1024;
  myVRCdata.Channel_15 = 1024;
}
