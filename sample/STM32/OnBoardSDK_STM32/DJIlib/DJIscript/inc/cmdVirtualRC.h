/*! @brief
 *  @file cmdVirtualRC.h
 *  @version 3.0
 *  @date Jan 20, 2016
 *
 *  @abstract
 *
 *
 *  @attention
 *  Project configuration:
 *  None
 *
 *  @version features:
 *  -* @version V3.0
 *  -* DJI-onboard-SDK for Windows,QT,STM32,ROS,Cmake
 *  -* @date Jan 20, 2016
 *  -* @author william.wu
 *
 * */

#ifndef CMDVIRTUALRC_H
#define CMDVIRTUALRC_H

#include "DJI_Script.h"

/*! @note It is not necessary to know the meaning of each function's name.
 *  Just use it please. Maybe these names will change somehow.
 * */

bool VC         (DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool startVC    (DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool stopVC     (DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool ctVC       (DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);

#endif // CMDVIRTUALRC_H
