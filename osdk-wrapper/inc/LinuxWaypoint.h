/*! @file LinuxWaypoint.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Provides Waypoint functionality in thenew Linux sample 
 *  and a small example to show usage.
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef LINUXWAYPOINT_H
#define LINUXWAYPOINT_H

#include <DJI_WayPoint.h> 
#include <DJI_API.h>
#include "LinuxSetup.h"
#include <iostream>

using namespace DJI::onboardSDK;

ackReturnData initWaypointMission(CoreAPI* api, WayPoint *waypoint, int numWaypoints, int timeout);
ackReturnData addWaypoint(CoreAPI* api, WayPoint *waypoint, PositionData* wpCoordinates, uint8_t index, int timeout);
ackReturnData startWaypointMission(CoreAPI* api, WayPoint *waypoint, int timeout);
ackReturnData stopWaypointMission(CoreAPI* api, WayPoint *waypoint, int timeout);
ackReturnData pauseWaypointMission(CoreAPI* api, WayPoint *waypoint, int timeout);
ackReturnData resumeWaypointMission(CoreAPI* api, WayPoint *waypoint, int timeout);

int wayPointMissionExample(CoreAPI* api, WayPoint* waypointObj, int timeout);


#endif