/*! @brief
 *  @file cmdCoreAPI.h
 *  @version 3.0
 *  @date Jan 6, 2016
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
 *  -* @date Jan 6, 2016
 *  -* @author william.wu
 *
 * */

#ifndef CMDCAMERA_H
#define CMDCAMERA_H

#include "DJI_Script.h"

/*! @note It is not necessary to know the meaning of each function's name.
 *  Just use it please. Maybe these names will change somehow.
 * */

bool CC  (DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool cmCC(DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool agCC(DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool sgCC(DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);


#endif // CMDCAMERA_H
