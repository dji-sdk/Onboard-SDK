#ifndef CMDCOREAPI
#define CMDCOREAPI

#include "DJI_Script.h"

/*! @note It is not necessary to know the meaning of each function's name.
 *  Just use it please. Maybe these names will change somehow.
 * */

bool CA(DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool acCA(DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool vsCA(DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool bfCA(DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);
bool bdCA(DJI::onboardSDK::Script* script, DJI::onboardSDK::UserData data);

#endif // CMDCOREAPI

