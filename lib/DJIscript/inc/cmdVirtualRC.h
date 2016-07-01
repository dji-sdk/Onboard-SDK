/*! @file cmdVirtualRC.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Virtual RC interface for DJI Onboard SDK command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef CMDVIRTUALRC_H
#define CMDVIRTUALRC_H

#include "DJI_Script.h"

/*! @note The function names follow this format - 
*         2-letter uppercase - Acronym for the command set (e.g. VC = Virtual RC)
*         lowercase - The specific function (or acronym, e.g. ct = Control)
* */

bool VC       (DJI::onboardSDK::Script* script, DJI::UserData data);
bool startVC  (DJI::onboardSDK::Script* script, DJI::UserData data);
bool stopVC   (DJI::onboardSDK::Script* script, DJI::UserData data);
bool ctVC     (DJI::onboardSDK::Script* script, DJI::UserData data);

#endif // CMDVIRTUALRC_H
