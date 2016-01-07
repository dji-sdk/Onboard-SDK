/*! @brief
 *  @file cmdCoreAPI.h
 *  @version 3.0
 *  @date Jan 7, 2016
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
 *  -* @date Jan 7, 2016
 *  -* @author william.wu
 *
 * */
#ifndef CMDSETTINGS_H
#define CMDSETTINGS_H

#include "DJI_Script.h"

/*! @note It is not necessary to know the meaning of each function's name.
 *  Just use it please. Maybe these names will change somehow.
 * */

bool SS    (DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool idSS  (DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool keySS (DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool saveSS(DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool loadSS(DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool spSS  (DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);

#endif // CMDSETTINGS_H
