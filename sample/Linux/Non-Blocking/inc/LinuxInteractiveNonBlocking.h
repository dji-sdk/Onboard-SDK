/*! @file LinuxInteractive.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Barebones interactive UI for executing Onboard SDK commands.
 *  Calls functions from the new Linux example based on user input.
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef LINUXINTERACTIVE_NONBLOCKING_H
#define LINUXINTERACTIVE_NONBLOCKING_H

#include <DJI_Type.h>
#include <DJICommonType.h>
#include <DJI_API.h>
#include <DJI_Flight.h>
#include "LinuxSetup.h"
#include "LinuxCleanup.h"
#include "LinuxFlight.h"
#include "LinuxWaypoint.h"

/*! Poll at 10Hz, waiting for commands from the user.
    The spin runs indefinitely until the user sends the exit command.
!*/
void interactiveSpin(CoreAPI* api, Flight* flight, WayPoint* waypointObj);


#endif // LINUXINTERACTIVE_NONBLOCKING_H