/*! @brief
 *  @file cmdHotPoint.h
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

#ifndef CMDHOTPOINT_H
#define CMDHOTPOINT_H

#include "DJI_Script.h"

/*! @note It is not necessary to know the meaning of each function's name.
 *  Just use it please. Maybe these names will change somehow.
 * */

bool HP         (DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool startHP    (DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool stopHP     (DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool pauseHP    (DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool restartHP  (DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);


#endif // CMDHOTPOINT_H
