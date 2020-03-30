/*! @file missions/waypoint_v2_main.cpp
 *  @version 3.8
 *  @date Mar 07 2019
 *
 *  @brief
 *  main for Waypoint Missions V2 API usage in a Linux environment.
 *  Shows example usage of the Waypoint Missions through
 *  the Mission Manager API.
 *
 *  @Copyright (c) 2019 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "waypoint_v2_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int
main(int argc, char** argv)
{
  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  int responseTimeout = 1;

  // Obtain Control Authority
  vehicle->control->obtainCtrlAuthority(functionTimeout);

  WaypointV2MissionSample *sample = new WaypointV2MissionSample(vehicle);

  // Set up telemetry subscription
  if (!sample->setUpSubscription(responseTimeout))
  {
    std::cout << "Failed to set up Subscription!" << std::endl;
    return -1;
  }
  // wait for the subscription
  sleep(responseTimeout);

  ErrorCode::ErrorCodeType ret;
  ret = sample->initMissionSetting(responseTimeout);
  DSTATUS("initMissionSetting ErrorCode:%x\n", ret);

  auto *missionPtr = vehicle->missionManager->wpMissionV2;

  //missionPtr->getCurrentState();
  sleep(2);
  ret = sample->uploadWaypointMission(responseTimeout);
  DSTATUS("uploadWaypointMission ErrorCode:%x\n", ret);

  sleep(2);
  std::vector<DJIWaypointV2> mission;
  ret = sample->dowloadWaypointMission(mission,responseTimeout);

//  for (int i = 0; i<mission.size();i ++)
//  {
//    DSTATUS("waypoint [%d] positionX: %f",i, mission[i].positionX);
//    DSTATUS("waypoint [%d] positionY: %f",i, mission[i].positionY);
//    DSTATUS("waypoint [%d] positionZ: %f",i, mission[i].positionZ);
//  }

  DSTATUS("downloadWapointActions ErrorCode:%x\n", ret);


 // missionPtr->getMissionState();
//  sleep(2);
  ret = sample->uploadWapointActions(responseTimeout);
  DSTATUS("uploadWapointActions ErrorCode:%x\n", ret);
  sleep(2);
 // missionPtr->getMissionState();
  //ErrorCode::ErrorCodeType retCode2 = missionPtr->getActionRemainMemory(1);
 // DSTATUS("getActionRemainMemory%d\n",retCode2);


//  DSTATUS("getWaypointIndexInList ErrorCode:%x\n", retCode3);
 // DSTATUS("getWaypointIndexInList start index:%d, end index:%d\n", ack.startIndex, ack.endIndex);

  ret = sample->startWaypointMission(vehicle);

  DSTATUS("startWaypointMission ErrorCode:%x\n", ret);
  sleep(100);
  if(!sample->teardownSubscription(DEFAULT_PACKAGE_INDEX, responseTimeout))
  {
    std::cout << "Failed to tear down Subscription!" << std::endl;
    return -1;
  }

  // Mission will continue when we exit here

  return 0;
}