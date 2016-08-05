/*! @file cmdWayPoint.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Waypoint commands for DJI Onboard SDK command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef CMDWAYPOINT_H
#define CMDWAYPOINT_H

#include "DJI_Script.h"

/*! @note The function names follow this format - 
*         2-letter uppercase - Acronym for the command set (e.g. WP = Waypoint)
*         lowercase - The specific function (or acronym, e.g. ap = Add Point)
* */

bool WP(DJI::onboardSDK::Script* script, DJI::UserData data);
bool initWP(DJI::onboardSDK::Script* script, DJI::UserData data);
bool startWP(DJI::onboardSDK::Script* script, DJI::UserData data);
bool stopWP(DJI::onboardSDK::Script* script, DJI::UserData data);
bool pauseWP(DJI::onboardSDK::Script* script, DJI::UserData data);
bool restartWP(DJI::onboardSDK::Script* script, DJI::UserData data);
bool apWP(DJI::onboardSDK::Script* script, DJI::UserData data);

#endif // CMDWAYPOINT_H
