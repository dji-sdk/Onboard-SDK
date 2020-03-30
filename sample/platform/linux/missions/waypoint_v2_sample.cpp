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

std::mutex m;
std::condition_variable condVar;
std::string funcName;

void wait_execute() {
  std::unique_lock<std::mutex> lk(m);
  condVar.wait_for(lk, std::chrono::seconds(1));
}

//DJIWaypointV2MissionOperator::CommonErrorCallback defaultErrorCB =
//    [](DJIWaypointV2MissionOperator::CommonErrorCode errorCode) {
//      DSTATUS("%s return error code: %d\n", funcName.c_str(), errorCode);
//      condVar.notify_one();
//    };

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

  ErrorCode::ErrorCodeType ret = vehiclePtr->missionManager->wpMissionV2->uploadMission(this->mission,responseTimeout);
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::dowloadWaypointMission(std::vector<DJIWaypointV2> &mission,int responseTimeout)
{
  ErrorCode::ErrorCodeType ret = vehiclePtr->missionManager->wpMissionV2->downloadMission(mission, responseTimeout);
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::uploadWapointActions(int responseTimeout)
{
  ErrorCode::ErrorCodeType ret = vehiclePtr->missionManager->wpMissionV2->uploadActionV2(actions,responseTimeout);
    return ret;
}

std::vector<DJIWaypointV2Action> WaypointV2MissionSample::generateWaypointActions(uint16_t actionNum)
{
  std::vector<DJIWaypointV2Action> actions;

  for(uint32_t i = 0; i<actionNum; i++)
  {
    DJIWaypointV2SampleReachPointTriggerParam reachPointTriggerParam;
    reachPointTriggerParam.Index = 0;
    reachPointTriggerParam.terminateNum = 0;


    auto *trigger = new DJIWaypointV2Trigger(DJIWaypointV2ActionTriggerTypeSampleReachPoint,(void *)&reachPointTriggerParam);

    auto *cameraActuatorParam = new DJIWaypointV2CameraActuatorParam(DJIWaypointV2ActionActuatorCameraOperationTypeTakePhoto,nullptr);

    auto *actuator = new DJIWaypointV2Actuator(DJIWaypointV2ActionActuatorTypeCamera,0,cameraActuatorParam);

    auto *action = new DJIWaypointV2Action(i, *trigger,*actuator);

    actions.push_back(*action);
  }
  return actions;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::startWaypointMission(DJI::OSDK::Vehicle* vehicle) {
  if (!vehicle->isM300()) {
    DSTATUS("This sample only supports M210 V3!\n");
    return false;
  }

  ErrorCode::ErrorCodeType ret;
  ret = vehicle->missionManager->wpMissionV2->startV2(1);
  return ret;
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

ErrorCode::ErrorCodeType WaypointV2MissionSample::initMissionSetting(int timeout) {

  /*generate waypoints*/
  float32_t radius = 43;
  uint32_t polygonNum = 43;
  uint16_t actionNum = 2;
  srand(int(time(0)));

  this->mission = generatePolygonWaypoints(radius, polygonNum);
  this->actions = generateWaypointActions(actionNum);

  WayPointV2InitSettings missionInitSettings;
  missionInitSettings.version = 25856;
  missionInitSettings.saveFile = 0;
  missionInitSettings.reserved = 0;

  missionInitSettings.missionID = rand();
  missionInitSettings.missTotalLen = mission.size();
  /*!TODO: 这个地方横等于２就可以了，这个参数没有意义*/
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
  missionInitSettings.refAlti = subscribeGPosition.altitude;


 // DSTATUS("subscribeGPosition.altitude%f\n",subscribeGPosition.altitude);
 // DSTATUS("sizeof(generatedWaypts)%d\n",sizeof(generatedWaypts));
 // DSTATUS("generatedWaypts.size()%d\n",generatedWaypts.size());

  /*TODO: 这个地方要想办法去掉*/
  vehiclePtr->missionManager->init(DJI_MISSION_TYPE::WAYPOINT, timeout);

  auto missionPtr = vehiclePtr->missionManager->wpMissionV2;
  ErrorCode::ErrorCodeType retCode = missionPtr->init(&missionInitSettings,timeout);

//  ErrorCode::ErrorCodeType retCode2 = missionPtr->getActionRemainMemory(timeout);
//
//  DSTATUS("getActionRemainMemory%d\n",retCode2);
//  DSTATUS("sizeofWaypointV2()%d\n",sizeof(WaypointV2));
//  DSTATUS("Init mission waypoint v2 result is %x\n",retCode);
  return retCode;
}

//void createCameraAction() {
//
//}
//void createGimbalAction() {
//
//}
//void createAircraftAction() {
//
//}
//
//void createReachPointTrigger()
//{
//
//}
//void createAssociateTrigger() {
//  DJIWaypointV2AssociateTriggerParam param;
//  param.actionIdAssociated = 1;
//  param.actionAssociatedType = DJIWaypointV2TriggerAssociatedTimingTypeSimultaneously;
//  param.waitingTime = 0;
//}
//void createTrajectoryTrigger()
//{
//  DJIWaypointV2TrajectoryTriggerParam param;
//  param.endIndex =1;
//  param.startIndex =0;
//
//}
//void createIntervalTrigger()
//{DJIWaypointV2IntervalTriggerParam param;
//param.startIndex =0;
//param.actionIntervalType = DJIWaypointV2ActionIntervalTypeDistance;
//param.interval  = 10;
//}
//DJIWaypointV2Action createWaypoingV2Actions()
//{
//
//}
