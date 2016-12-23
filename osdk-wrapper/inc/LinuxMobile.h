/*! @file LinuxMobile.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Framework for executing Onboard SDK commands based on 
 *  mobile commands sent using Data Transparent Transmission.
 *
 *  Calls functions from the new Linux example based on user input
 *  in the companion iOS app
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef LINUXMOBILE_H
#define LINUXMOBILE_H

#include <DJI_Type.h>
#include <DJICommonType.h>
#include <DJI_API.h>
#include <DJI_Flight.h>
#include "LinuxSetup.h"
#include "LinuxCleanup.h"
#include "LinuxFlight.h"
#include "LinuxWaypoint.h"
#include "LinuxCamera.h"
//Local Mission Planning Suite Headers
#ifdef USE_PRECISION_MISSIONS
  #include <MissionplanHeaders.h>
#endif

using namespace DJI;
using namespace DJI::onboardSDK;

/*! Poll at 10Hz, waiting for data from Mobile OSDK App. The Onboard SDK
    receives this data, parses it and sets appropriate member variables
    of the API object. This loop checks to see if any of the members are
    set, and if so then executes that command. 

    The spin exits after ~15 mins.
!*/
void mobileCommandSpin(CoreAPI *api, Flight *flight, WayPoint *waypointObj, Camera *camera, char **trajFiles, int argc);
void mobileCommandSpinNonBlocking(CoreAPI* api, Flight* flight, WayPoint* waypointObj);

#endif //LINUXMOBILE_H