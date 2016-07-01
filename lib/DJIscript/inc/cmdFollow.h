/*! @file cmdFollow.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Follow Me interface for DJI Onboard SDK command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef CMDFOLLOW_H
#define CMDFOLLOW_H

#include "DJI_Script.h"

/*! @note The function names follow this format - 
*         2-letter uppercase - Acronym for the command set (e.g. FM = Follow Me)
*         lowercase - The specific function
* */

bool FM       (DJI::onboardSDK::Script* script, DJI::UserData data);
bool startFM  (DJI::onboardSDK::Script* script, DJI::UserData data);
bool stopFM   (DJI::onboardSDK::Script* script, DJI::UserData data);
bool pauseFM  (DJI::onboardSDK::Script* script, DJI::UserData data);
bool restartFM(DJI::onboardSDK::Script* script, DJI::UserData data);
bool updateFM (DJI::onboardSDK::Script* script, DJI::UserData data);

#endif // CMDFOLLOW_H
