/*! @file waypoint_v2_sample.cpp
 *  @version 4.0
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
#include "dji_waypoint_v2_action.hpp"
#include "memory"
#include <ctime>
#include <stdlib.h>
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

//std::mutex m;
//std::condition_variable condVar;
//std::string funcName;
//
//void wait_execute() {
//  std::unique_lock<std::mutex> lk(m);
//  condVar.wait_for(lk, std::chrono::seconds(1));
//}



WaypointV2MissionSample::WaypointV2MissionSample(Vehicle *vehicle):vehiclePtr(vehicle){}

WaypointV2MissionSample::~WaypointV2MissionSample() {
  delete (vehiclePtr);
}

bool WaypointV2MissionSample::setUpSubscription(int responseTimeout) {
  // Telemetry: Verify the subscription
  ACK::ErrorCode subscribeStatus;

  subscribeStatus = vehiclePtr->subscribe->verify(responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }

  // Telemetry: Subscribe to flight status and mode at freq 10 Hz
  int freq = 10;
  TopicName topicList10Hz[] = {TOPIC_GPS_FUSED};
  int numTopic = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  bool enableTimestamp = false;

  bool pkgStatus = vehiclePtr->subscribe->initPackageFromTopicList(
      DEFAULT_PACKAGE_INDEX, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus)) {
    return pkgStatus;
  }

  // Start listening to the telemetry data
  subscribeStatus =
    vehiclePtr->subscribe->startPackage(DEFAULT_PACKAGE_INDEX, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup
    ACK::ErrorCode ack = vehiclePtr->subscribe->removePackage(
        DEFAULT_PACKAGE_INDEX, responseTimeout);
    if (ACK::getError(ack)) {
      DSTATUS(
          "Error unsubscribing; please restart the drone/FC to get "
          "back to a clean state.\n");
    }
    return false;
  }
  return true;
}

bool WaypointV2MissionSample::teardownSubscription(const int pkgIndex,
                          int responseTimeout) {
  ACK::ErrorCode ack =
      vehiclePtr->subscribe->removePackage(pkgIndex, responseTimeout);
  if (ACK::getError(ack)) {
    DSTATUS(
        "Error unsubscribing; please restart the drone/FC to get back "
        "to a clean state.\n");
    return false;
  }
  return true;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::uploadWaypointMission(int responseTimeout) {

  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->uploadMission(this->mission,responseTimeout);
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::dowloadWaypointMission(std::vector<DJIWaypointV2> &mission,int responseTimeout)
{
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->downloadMission(mission, responseTimeout);
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::uploadWapointActions(int responseTimeout)
{
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->uploadActionV2(actions,responseTimeout);
    return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::startWaypointMission(int responseTimeout) {
  if (!vehiclePtr->isM300()) {
    DSTATUS("This sample only supports M300 V3!\n");
    return false;
  }
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->start(responseTimeout);
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::stopWaypointMission(int responseTimeout) {
  if (!vehiclePtr->isM300()) {
    DSTATUS("This sample only supports M300 V3!\n");
    return false;
  }
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->stop(responseTimeout);
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::pauseWaypointMission(int responseTimeout) {
  if (!vehiclePtr->isM300()) {
    DSTATUS("This sample only supports M300 V3!\n");
    return false;
  }
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->pause(responseTimeout);
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::resumeWaypointMission(int responseTimeout) {
  if (!vehiclePtr->isM300()) {
    DSTATUS("This sample only supports M300 V3!\n");
    return false;
  }
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->resume(responseTimeout);
  return ret;
}

std::vector<DJIWaypointV2> WaypointV2MissionSample::generatePolygonWaypoints(float32_t radius, uint16_t polygonNum) {
  // Let's create a vector to store our waypoints in.
  std::vector<DJIWaypointV2> waypointList;
  DJIWaypointV2 startPoint;
  DJIWaypointV2 waypointV2;

  startPoint.positionX = 0;
  startPoint.positionY = 0;
  startPoint.positionZ = 10;
  setWaypointV2Defaults(&startPoint);
  waypointList.push_back(startPoint);

  // Iterative algorithm
  for (int i = 0; i < polygonNum; i++) {

    float32_t angle = i * 2 * M_PI / polygonNum;
    setWaypointV2Defaults(&waypointV2);
    waypointV2.positionX = radius * cos(angle);
    waypointV2.positionY = radius * sin(angle);
    waypointV2.positionZ = startPoint.positionZ ;
    waypointList.push_back(waypointV2);
  }
  waypointList.push_back(startPoint);
  return waypointList;
}

std::vector<DJIWaypointV2Action> WaypointV2MissionSample::generateWaypointActions(uint16_t actionNum)
{
  std::vector<DJIWaypointV2Action> actions;

  for(uint32_t i = 0; i < actionNum; i++)
  {
    DJIWaypointV2SampleReachPointTriggerParam reachPointTriggerParam;
    reachPointTriggerParam.Index = 0;
    reachPointTriggerParam.terminateNum = 0;

    auto *trigger = new DJIWaypointV2Trigger(DJIWaypointV2ActionTriggerTypeSampleReachPoint,(void *)&reachPointTriggerParam);

    auto *cameraActuatorParam = new DJIWaypointV2CameraActuatorParam(DJIWaypointV2ActionActuatorCameraOperationTypeTakePhoto, nullptr);

    auto *actuator = new DJIWaypointV2Actuator(DJIWaypointV2ActionActuatorTypeCamera, 0, cameraActuatorParam);

    auto *action = new DJIWaypointV2Action(i, *trigger,*actuator);

    actions.push_back(*action);
  }
  return actions;
}

void WaypointV2MissionSample::setWaypointV2Defaults(DJIWaypointV2* waypointV2) {

  waypointV2->waypointType = DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop;
  waypointV2->headingMode = DJIWaypointV2HeadingFixed;
  waypointV2->config.useLocalCruiseVel = 0;
  waypointV2->config.useLocalMaxVel = 0;

  waypointV2->dampingDistance = 40;
  waypointV2->heading = 0;
  waypointV2->turnMode = DJIWaypointV2TurnModeClockwise;

  waypointV2->pointOfInterest.longitude = 0;
  waypointV2->pointOfInterest.latitude = 0;
  waypointV2->maxFlightSpeed = 900;
  waypointV2->autoFlightSpeed = 400;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::initMissionSetting(int timeout) {

  /*generate waypoints*/
  float32_t radius = 10;
  uint32_t polygonNum = 6;
  uint16_t actionNum = 2;
  srand(int(time(0)));

  this->mission = generatePolygonWaypoints(radius, polygonNum);
  this->actions = generateWaypointActions(actionNum);

  WayPointV2InitSettings missionInitSettings;
  missionInitSettings.missionID = rand();
  missionInitSettings.missTotalLen = mission.size();
  missionInitSettings.repeatTimes  = 1;
  missionInitSettings.finishedAction = DJIWaypointV2MissionFinishedGoHome;
  missionInitSettings.maxFlightSpeed = 1000;
  missionInitSettings.autoFlightSpeed = 500;
  missionInitSettings.startIndex = 0;
  missionInitSettings.exitMissionOnRCSignalLost = 1;
  missionInitSettings.gotoFirstWaypointMode = DJIWaypointV2MissionGotoFirstWaypointModePointToPoint;
  // Global position retrieved via subscription
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;
  subscribeGPosition = vehiclePtr->subscribe->getValue<TOPIC_GPS_FUSED>();
  missionInitSettings.refLati = subscribeGPosition.latitude;
  missionInitSettings.refLong = subscribeGPosition.longitude;
  missionInitSettings.refAlti = 0;
  ErrorCode::ErrorCodeType retCode = vehiclePtr->waypointV2Mission->init(&missionInitSettings,timeout);
  return retCode;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::runWaypointV2Mission()
{
  int responseTimeout = 1;
  ErrorCode::ErrorCodeType ret;

  if (!setUpSubscription(responseTimeout))
  {
    DERROR("Failed to set up subscription!\n");
    return -1;
  }
  else
  {
    DSTATUS("Set up subscription successfully!\n");
  }

  /*! wait for subscription data come*/
  sleep(responseTimeout);
  ret = initMissionSetting(responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Init mission setting ErrorCode:%x\n", ret);
    return ret;
  }
  else
  {
    DSTATUS("Init mission setting successfully!\n");
  }
  sleep(responseTimeout);

  ret = uploadWaypointMission(responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Upload waypoint v2 mission ErrorCode:%x\n", ret);
    return ret;
  }
  else
  {
    DSTATUS("Upload waypoint v2 mission successfully!\n");
  }
  sleep(responseTimeout);

  std::vector<DJIWaypointV2> mission;
  ret = dowloadWaypointMission(mission,responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Download waypoint v2 mission ErrorCode:%x\n", ret);
    return ret;
  }
  else
  {
    DSTATUS("Download waypoint v2 mission successfully!\n");
  }
  sleep(responseTimeout);

  ret = uploadWapointActions(responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Upload waypoint v2 actions ErrorCode:%x\n", ret);
    return ret;
  }
  else
  {
    DSTATUS("Upload waypoint v2 actions successfully!\n");
  }
  sleep(responseTimeout);

  ret = startWaypointMission(responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Start waypoint v2 mission ErrorCode:%x\n", ret);
    return ret;
  }
  else
  {
    DSTATUS("Start waypoint v2 mission successfully!\n");
  }
  sleep(20);

  ret = pauseWaypointMission(responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Pause waypoint v2 mission ErrorCode:%x\n", ret);
    return ret;
  }
  else
  {
    DSTATUS("Pause waypoint v2 mission successfully!\n");
  }
  sleep(responseTimeout);

  ret = resumeWaypointMission(responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Resume Waypoint v2 mission ErrorCode:%x\n", ret);;
    return ret;
  }
  else
  {
    DSTATUS("Resume Waypoint v2 mission successfully!\n");
  }
  sleep(responseTimeout);

  /*! Set up telemetry subscription*/
  if(!teardownSubscription(DEFAULT_PACKAGE_INDEX, responseTimeout))
  {
    std::cout << "Failed to tear down Subscription!" << std::endl;
    return -1;
  }
}

