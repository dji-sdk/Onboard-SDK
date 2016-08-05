/*! @file VirtualRC.h
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

#ifndef VIRTUALRC_H
#define VIRTUALRC_H
#include "DJI_API.h"
#include "string.h"
#include "DJI_VirtualRC.h"
using namespace DJI::onboardSDK;
void VRC_TakeControl();
void VRCResetData();
#endif //VIRTUALRC_H
