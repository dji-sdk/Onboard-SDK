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
  TopicName topicList10Hz[] = {TOPIC_GPS_FUSED, TOPIC_ALTITUDE_FUSIONED};
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
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Upload waypoint v2 mission ErrorCode:0x%lX\n", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Upload waypoint v2 mission successfully!\n");
  }
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::dowloadWaypointMission(std::vector<DJIWaypointV2> &mission,int responseTimeout)
{
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->downloadMission(mission, responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Download waypoint v2 mission ErrorCode:0x%lX\n", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Download waypoint v2 mission successfully!\n");
  }
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::uploadWapointActions(int responseTimeout)
{
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->uploadAction(actions,responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Upload waypoint v2 actions ErrorCode:0x%lX\n", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Upload waypoint v2 actions successfully!\n");
  }
    return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::startWaypointMission(int responseTimeout) {
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->start(responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Start waypoint v2 mission ErrorCode:0x%lX\n", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Start waypoint v2 mission successfully!\n");
  }
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::stopWaypointMission(int responseTimeout) {
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->stop(responseTimeout);
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::pauseWaypointMission(int responseTimeout) {
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->pause(responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Pause waypoint v2 mission ErrorCode:0x%lX\n", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Pause waypoint v2 mission successfully!\n");
  }
  sleep(5);
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::resumeWaypointMission(int responseTimeout) {

  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->resume(responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Resume Waypoint v2 mission ErrorCode:0x%lX\n", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Resume Waypoint v2 mission successfully!\n");
  }
  return ret;
}

void  WaypointV2MissionSample::getGlogalCruiseSpeed(int responseTimeout)
{
  GlobalCruiseSpeed cruiseSpeed;
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->getGlobalCruiseSpeed(cruiseSpeed, responseTimeout);
  if(ret !=  ErrorCode::SysCommonErr::Success)
  {
    DERROR("Get glogal cruise speed failed ErrorCode:0x%lX\n", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return;
  }
  DSTATUS("Current cruise speed is: %f m/s\n",cruiseSpeed);
}

void WaypointV2MissionSample::setGlogalCruiseSpeed(const GlobalCruiseSpeed &cruiseSpeed, int responseTimeout)
{
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->setGlobalCruiseSpeed(cruiseSpeed, responseTimeout);
  if(ret !=  ErrorCode::SysCommonErr::Success)
  {
    DERROR("Set glogal cruise speed %f m/s failed ErrorCode:0x%lX\n", cruiseSpeed, ret);
    ErrorCode::printErrorCodeMsg(ret);
    return;
  }
  DSTATUS("Current cruise speed is: %f m/s\n", cruiseSpeed);
}

std::vector<DJIWaypointV2> WaypointV2MissionSample::generatePolygonWaypoints(float32_t radius, uint16_t polygonNum) {
  // Let's create a vector to store our waypoints in.
  std::vector<DJIWaypointV2> waypointList;
  DJIWaypointV2 startPoint;
  DJIWaypointV2 waypointV2;

  startPoint.positionX = 0;
  startPoint.positionY = 0;
  startPoint.positionZ = 5;
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
  std::vector<DJIWaypointV2Action> actionVector;

  for(uint16_t i = 0; i < actionNum; i++)
  {
    DJIWaypointV2SampleReachPointTriggerParam sampleReachPointTriggerParam;
    sampleReachPointTriggerParam.waypointIndex = i;
    sampleReachPointTriggerParam.terminateNum = 0;

    auto *trigger = new DJIWaypointV2Trigger(DJIWaypointV2ActionTriggerTypeSampleReachPoint,&sampleReachPointTriggerParam);
    auto *cameraActuatorParam = new DJIWaypointV2CameraActuatorParam(DJIWaypointV2ActionActuatorCameraOperationTypeTakePhoto, nullptr);
    auto *actuator = new DJIWaypointV2Actuator(DJIWaypointV2ActionActuatorTypeCamera, 0, cameraActuatorParam);
    auto *action = new DJIWaypointV2Action(i, *trigger,*actuator);
    actionVector.push_back(*action);
  }
  return actionVector;
}

void WaypointV2MissionSample::setWaypointV2Defaults(DJIWaypointV2* waypointV2) {

  waypointV2->waypointType = DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop;
  waypointV2->headingMode = DJIWaypointV2HeadingFixed;
  waypointV2->config.useLocalCruiseVel = 0;
  waypointV2->config.useLocalMaxVel = 0;

  waypointV2->dampingDistance = 40;
  waypointV2->heading = 0;
  waypointV2->turnMode = DJIWaypointV2TurnModeClockwise;

  waypointV2->pointOfInterest.positionX = 0;
  waypointV2->pointOfInterest.positionY = 0;
  waypointV2->pointOfInterest.positionZ = 0;
  waypointV2->maxFlightSpeed= 9;
  waypointV2->autoFlightSpeed = 2;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::initMissionSetting(int timeout) {

  /*generate waypoints*/
  uint16_t polygonNum = 6;
  float32_t radius = 6;

  uint16_t actionNum = 5;
  srand(int(time(0)));

  this->mission = generatePolygonWaypoints(radius, polygonNum);
  this->actions = generateWaypointActions(actionNum);
  WayPointV2InitSettings missionInitSettings;
  missionInitSettings.missionID = rand();
  missionInitSettings.missTotalLen = mission.size();
  /*! set mission's repeat times is 2*/
  missionInitSettings.repeatTimes  = 1;
  missionInitSettings.finishedAction = DJIWaypointV2MissionFinishedGoHome;
  missionInitSettings.maxFlightSpeed = 10;
  missionInitSettings.autoFlightSpeed = 3;
  missionInitSettings.exitMissionOnRCSignalLost = 1;
  missionInitSettings.gotoFirstWaypointMode = DJIWaypointV2MissionGotoFirstWaypointModePointToPoint;

  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;
  Telemetry::TypeMap<TOPIC_ALTITUDE_FUSIONED>::type subscribetakeoffAltitude;

  subscribetakeoffAltitude =  vehiclePtr->subscribe->getValue<TOPIC_ALTITUDE_FUSIONED>();
  subscribeGPosition = vehiclePtr->subscribe->getValue<TOPIC_GPS_FUSED>();
  missionInitSettings.refLati = subscribeGPosition.latitude;
  missionInitSettings.refLong = subscribeGPosition.longitude;
  missionInitSettings.refAlti = subscribetakeoffAltitude;
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->init(&missionInitSettings,timeout);

  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Init mission setting ErrorCode:0x%lX\n", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Init mission setting successfully!\n");
  }
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::runWaypointV2Mission()
{
  if (!vehiclePtr->isM300()) {
    DSTATUS("This sample only supports M300 V3!\n");
    return false;
  }

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
    return ret;
  sleep(responseTimeout);

  uploadWaypointMission(responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(responseTimeout);

  std::vector<DJIWaypointV2> mission;
  dowloadWaypointMission(mission,responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(responseTimeout);

  uploadWapointActions(responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(responseTimeout);

  startWaypointMission(responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(40);

  setGlogalCruiseSpeed(1.5,responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(responseTimeout);

  getGlogalCruiseSpeed(responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(responseTimeout);

  pauseWaypointMission(responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(5);

  resumeWaypointMission(responseTimeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(100);
  /*! Set up telemetry subscription*/
  if(!teardownSubscription(DEFAULT_PACKAGE_INDEX, responseTimeout))
  {
    std::cout << "Failed to tear down Subscription!" << std::endl;
    return -1;
  }
}

