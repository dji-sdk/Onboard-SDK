/*! @file LinuxWaypoint.cpp
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

#include "LinuxWaypoint.h"

ackReturnData initWaypointMission(CoreAPI* api, WayPoint *waypoint, int numWaypoints, int timeout)
{
  ackReturnData wpInitAck;
  WayPointInitData wpInitData;
  
  wpInitData.maxVelocity = 10;
  wpInitData.idleVelocity = 5;
  wpInitData.finishAction = 0;
  wpInitData.executiveTimes = 1;
  wpInitData.yawMode = 0;
  wpInitData.traceMode = 0;
  wpInitData.RCLostAction = 0;
  wpInitData.gimbalPitch = 0;
  wpInitData.latitude = 0;
  wpInitData.longitude = 0;
  wpInitData.altitude = 0;
  wpInitData.indexNumber = numWaypoints;

  wpInitAck.ack = waypoint->init(&wpInitData, timeout);
  api->decodeMissionStatus(wpInitAck.ack);

  return wpInitAck;
}

ackReturnData addWaypoint(CoreAPI* api, WayPoint *waypoint, PositionData* wpCoordinates, uint8_t wpIndex, int timeout)
{
  ackReturnData wpAddAck;
  WayPointData wpData;

  wpData.damping = 0;
  wpData.yaw = 0;
  wpData.gimbalPitch = 0;
  wpData.turnMode = 0;
  wpData.hasAction = 0;
  wpData.actionTimeLimit = 100;
  wpData.actionNumber = 0;
  wpData.actionRepeat = 0;
  //! Set the passed parameters
  wpData.index = wpIndex;
  wpData.longitude = wpCoordinates->longitude;
  wpData.latitude = wpCoordinates->latitude;
  wpData.altitude = wpCoordinates->altitude;
  for (int i = 0; i < 16; ++i)
  {
    wpData.commandList[i] = 0;
    wpData.commandParameter[i] = 0;
  }

  wpAddAck.ack = (waypoint->uploadIndexData(&wpData, timeout)).ack;
  api->decodeMissionStatus(wpAddAck.ack);

  return wpAddAck;
}

ackReturnData startWaypointMission(CoreAPI* api, WayPoint *waypoint, int timeout)
{
  ackReturnData wpStartAck;
  wpStartAck.ack = waypoint->start(timeout); 
  api->decodeMissionStatus(wpStartAck.ack);

  return wpStartAck;
}

ackReturnData stopWaypointMission(CoreAPI* api, WayPoint *waypoint, int timeout)
{
  ackReturnData wpStopAck;
  wpStopAck.ack = waypoint->stop(timeout); 
  api->decodeMissionStatus(wpStopAck.ack);

  return wpStopAck;
}

ackReturnData pauseWaypointMission(CoreAPI* api, WayPoint *waypoint, int timeout)
{
  ackReturnData wpPauseAck;
  wpPauseAck.ack = waypoint->pause(true, timeout); 
  api->decodeMissionStatus(wpPauseAck.ack);

  return wpPauseAck;
}

ackReturnData resumeWaypointMission(CoreAPI* api, WayPoint *waypoint, int timeout)
{
  ackReturnData wpResumeAck;
  wpResumeAck.ack = waypoint->pause(false, timeout); 
  api->decodeMissionStatus(wpResumeAck.ack);

  return wpResumeAck;
}


int wayPointMissionExample(CoreAPI* api, WayPoint* waypointObj, int timeout)
{
  //! Create an ackReturnData struct
  ackReturnData wpAck;
  //! Create four waypoints near your current position
  int numWaypoints = 4;
  //! Init the mission
  std::cout << "Init Waypoint mission\n";
  wpAck = initWaypointMission(api, waypointObj, numWaypoints, timeout);

  //! Get current GPS location
  PositionData curPosition = api->getBroadcastData().pos;
  //! Create an offset in radians for the waypoint creation loop to access and permute.
  float64_t offsetInRadians = 0.000003;
  //! Waypoint creation and upload
  for (int index = 0; index < numWaypoints; index++)
  {
    PositionData wpPosition;
    switch(index)
    {
      case 0:
        wpPosition = curPosition;
        wpPosition.latitude += offsetInRadians;
        wpPosition.altitude = 20;
        break;
      case 1:
        wpPosition = curPosition;
        wpPosition.latitude += offsetInRadians;
        wpPosition.longitude += offsetInRadians;
        wpPosition.altitude = 20;
        break;
      case 2:
        wpPosition = curPosition;
        wpPosition.longitude += offsetInRadians;
        wpPosition.altitude = 20;
        break;
      case 3:
        wpPosition = curPosition;
        wpPosition.altitude = 20;
        break;
    }
    std::cout << "Adding waypoint " << index << "\n";
    wpAck = addWaypoint(api, waypointObj, &wpPosition, (uint8_t)index, timeout);    
  }
  //! Start mission
  std::cout << "Starting Waypoint mission\n";
  wpAck = startWaypointMission(api, waypointObj, timeout);

  //! Waiting for mission to finish. Increase this sleep if you increase the waypoint offset or number of waypoints.
  usleep(50000000);

  return 1; //More intelligent return values will be added with Ack parsing in a future release.
}