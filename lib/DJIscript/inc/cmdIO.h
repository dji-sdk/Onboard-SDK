#ifndef CMDIO_H
#define CMDIO_H

#include "DJI_Script.h"

bool addTask(DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data __UNUSED);
bool waitInput(DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data __UNUSED);
bool help(DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);

#endif // CMDIO_H
