/*! @file cmdSettings.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Script Settings command for DJI Onboard SDK Command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef CMDSETTINGS_H
#define CMDSETTINGS_H

#include "DJI_Script.h"

/*! @note The function names follow this format - 
*         2-letter uppercase - Acronym for the command set (e.g. SS = Script Settings)
*         lowercase - The specific function
* */

bool SS    (DJI::onboardSDK::Script* script, DJI::UserData data);
bool idSS  (DJI::onboardSDK::Script* script, DJI::UserData data);
bool keySS (DJI::onboardSDK::Script* script, DJI::UserData data);
bool saveSS(DJI::onboardSDK::Script* script, DJI::UserData data);
bool loadSS(DJI::onboardSDK::Script* script, DJI::UserData data);
bool spSS  (DJI::onboardSDK::Script* script, DJI::UserData data);

#endif // CMDSETTINGS_H
