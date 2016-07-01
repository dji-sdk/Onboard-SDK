/*! @file cmdHotPoint.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  HotPoint commands for DJI Onboard SDK command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef CMDHOTPOINT_H
#define CMDHOTPOINT_H

#include "DJI_Script.h"

/*! @note The function names follow this format - 
*         2-letter uppercase - Acronym for the command set (e.g. HP = Hotpoint)
*         lowercase - The specific function
* */

bool HP       (DJI::onboardSDK::Script* script, DJI::UserData data);
bool startHP  (DJI::onboardSDK::Script* script, DJI::UserData data);
bool stopHP   (DJI::onboardSDK::Script* script, DJI::UserData data);
bool pauseHP  (DJI::onboardSDK::Script* script, DJI::UserData data);
bool restartHP(DJI::onboardSDK::Script* script, DJI::UserData data);


#endif // CMDHOTPOINT_H
