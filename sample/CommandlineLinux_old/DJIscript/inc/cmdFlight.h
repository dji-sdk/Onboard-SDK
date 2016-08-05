/*! @file cmdFlight.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Flight Control Commands for DJI Onboard SDK Command line example
 *  Execute tasks like takeoff/landing/RTH. Also execute movement control
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */


#ifndef CMDFLIGHT_H
#define CMDFLIGHT_H

#include "DJI_Script.h"

/*! @note The function names follow this format - 
*         2-letter uppercase - Acronym for the command set (e.g. FC = Flight Control)
*         2-letter lowercase - Acronym for the specific function (e.g. tk = Task)
* */

bool FC  (DJI::onboardSDK::Script* script, DJI::UserData data);
bool tkFC(DJI::onboardSDK::Script* script, DJI::UserData data);
bool mcFC(DJI::onboardSDK::Script* script, DJI::UserData data);
bool flFC(DJI::onboardSDK::Script* script, DJI::UserData data);

#endif // CMDFLIGHT_H
