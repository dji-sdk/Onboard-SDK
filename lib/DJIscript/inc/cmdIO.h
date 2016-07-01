/*! @file cmdIO.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  I/O core interface for DJI Onboard SDK command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef CMDIO_H
#define CMDIO_H

#include "DJI_Script.h"

bool addTask(DJI::onboardSDK::Script* script, DJI::UserData data __UNUSED);
bool waitInput(DJI::onboardSDK::Script* script, DJI::UserData data __UNUSED);
bool help(DJI::onboardSDK::Script* script, DJI::UserData data);

#endif // CMDIO_H
