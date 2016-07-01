/*! @file cmdCamera.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Camera Command Header for DJI Onboard SDK Command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef CMDCAMERA_H
#define CMDCAMERA_H

#include "DJI_Script.h"

/*! @note The function names follow this format - 
*         2-letter uppercase - Acronym for the command set (e.g. CC = Camera Control)
*         2-letter lowercase - Acronym for the specific function (e.g. ag = Angle Gimbal)
* */

bool CC  (DJI::onboardSDK::Script* script, DJI::UserData data);
bool cmCC(DJI::onboardSDK::Script* script, DJI::UserData data);
bool agCC(DJI::onboardSDK::Script* script, DJI::UserData data);
bool sgCC(DJI::onboardSDK::Script* script, DJI::UserData data);


#endif // CMDCAMERA_H
