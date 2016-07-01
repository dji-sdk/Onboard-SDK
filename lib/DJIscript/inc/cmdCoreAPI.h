/*! @file cmdCoreAPI.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Core API Commands for DJI Onboard SDK Command line example. 
 *  Use for getting/setting data, activation, taking control etc.
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */


#ifndef CMDCOREAPI
#define CMDCOREAPI

#include "DJI_Script.h"

/*! @note The function names follow this format - 
*         2-letter uppercase - Acronym for the command set (e.g. CA = CoreAPI)
*         2-letter lowercase - Acronym for the specific function (e.g. ac = Activation)
* */

bool CA  (DJI::onboardSDK::Script* script, DJI::UserData data);
bool acCA(DJI::onboardSDK::Script* script, DJI::UserData data);
bool vsCA(DJI::onboardSDK::Script* script, DJI::UserData data);
bool bfCA(DJI::onboardSDK::Script* script, DJI::UserData data);
bool bdCA(DJI::onboardSDK::Script* script, DJI::UserData data);
bool ctCA(DJI::onboardSDK::Script* script, DJI::UserData data);
bool syCA(DJI::onboardSDK::Script* script, DJI::UserData data);

#endif // CMDCOREAPI
