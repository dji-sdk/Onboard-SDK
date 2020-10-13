/*! @file waypoint_v2_sample.cpp
 *  @version 4.0.0
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

//10HZ push ;1HZ print
E_OsdkStat updateMissionState(T_CmdHandle *cmdHandle, const T_CmdInfo *cmdInfo,
                              const uint8_t *cmdData, void *userData) {

  if (cmdInfo) {
    if (userData) {
      auto *wp2Ptr = (WaypointV2MissionOperator *)userData;
      auto *missionStatePushAck =
        (DJI::OSDK::MissionStatePushAck *)cmdData;

      wp2Ptr->setCurrentState(wp2Ptr->getCurrentState());
      wp2Ptr->setCurrentState((DJI::OSDK::DJIWaypointV2MissionState)missionStatePushAck->data.state);
      static uint32_t curMs = 0;
      static uint32_t preMs = 0;
      OsdkOsal_GetTimeMs(&curMs);
      if (curMs - preMs >= 1000)
      {
        preMs = curMs;
        DSTATUS("missionStatePushAck->commonDataVersion:%d",missionStatePushAck->commonDataVersion);
        DSTATUS("missionStatePushAck->commonDataLen:%d",missionStatePushAck->commonDataLen);
        DSTATUS("missionStatePushAck->data.state:0x%x",missionStatePushAck->data.state);
        DSTATUS("missionStatePushAck->data.curWaypointIndex:%d",missionStatePushAck->data.curWaypointIndex);
        DSTATUS("missionStatePushAck->data.velocity:%d",missionStatePushAck->data.velocity);
      }
    } else {
      DERROR("cmdInfo is a null value");
    }
    return OSDK_STAT_OK;
  }
  return OSDK_STAT_ERR_ALLOC;
}

/*! only push 0x00,0x10,0x11 event*/
E_OsdkStat updateMissionEvent(T_CmdHandle *cmdHandle, const T_CmdInfo *cmdInfo,
                              const uint8_t *cmdData, void *userData) {

  if (cmdInfo) {
    if (userData) {
      auto *MissionEventPushAck =
        (DJI::OSDK::MissionEventPushAck *)cmdData;

      DSTATUS("MissionEventPushAck->event ID :0x%x", MissionEventPushAck->event);

      if(MissionEventPushAck->event == 0x01)
        DSTATUS("interruptReason:0x%x",MissionEventPushAck->data.interruptReason);
      if(MissionEventPushAck->event == 0x02)
        DSTATUS("recoverProcess:0x%x",MissionEventPushAck->data.recoverProcess);
      if(MissionEventPushAck->event == 0x03)
        DSTATUS("finishReason:0x%x",MissionEventPushAck->data.finishReason);

      if(MissionEventPushAck->event == 0x10)
        DSTATUS("current waypointIndex:%d",MissionEventPushAck->data.waypointIndex);

      if(MissionEventPushAck->event == 0x11)
      {
        DSTATUS("currentMissionExecNum:%d",MissionEventPushAck->data.MissionExecEvent.currentMissionExecNum);
      }

      return OSDK_STAT_OK;
    }
  }
  return OSDK_STAT_SYS_ERR;
}

WaypointV2MissionSample::WaypointV2MissionSample(Vehicle *vehicle):vehiclePtr(vehicle){
  vehiclePtr->waypointV2Mission->RegisterMissionEventCallback(vehicle->waypointV2Mission, updateMissionEvent);
  vehiclePtr->waypointV2Mission->RegisterMissionStateCallback(vehicle->waypointV2Mission, updateMissionState);
}

WaypointV2MissionSample::~WaypointV2MissionSample() {
}

bool WaypointV2MissionSample::setUpSubscription(int timeout) {
  // Telemetry: Verify the subscription
  ACK::ErrorCode subscribeStatus;

  subscribeStatus = vehiclePtr->subscribe->verify(timeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }
  //TOPIC_ALTITUDE_OF_HOMEPOINT, TOPIC_GPS_POSITION
  // Telemetry: Subscribe to flight status and mode at freq 10 Hz
  int freq = 1;
  TopicName topicList10Hz[] = {TOPIC_GPS_FUSED };
  int numTopic = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  bool enableTimestamp = false;

  bool pkgStatus = vehiclePtr->subscribe->initPackageFromTopicList(
      DEFAULT_PACKAGE_INDEX, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus)) {
    return pkgStatus;
  }
  usleep(5000);
  // Start listening to the telemetry data
  subscribeStatus =
    vehiclePtr->subscribe->startPackage(DEFAULT_PACKAGE_INDEX, timeout);
  usleep(5000);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup
    ACK::ErrorCode ack = vehiclePtr->subscribe->removePackage(
        DEFAULT_PACKAGE_INDEX, timeout);
    if (ACK::getError(ack)) {
      DSTATUS(
          "Error unsubscribing; please restart the drone/FC to get "
          "back to a clean state.");
    }
    return false;
  }
  return true;
}

bool WaypointV2MissionSample::teardownSubscription(const int pkgIndex,
                          int timeout) {
  ACK::ErrorCode ack =
      vehiclePtr->subscribe->removePackage(pkgIndex, timeout);
  if (ACK::getError(ack)) {
    DSTATUS(
        "Error unsubscribing; please restart the drone/FC to get back "
        "to a clean state.");
    return false;
  }
  return true;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::runWaypointV2Mission()
{
  if (!vehiclePtr->isM300()) {
    DSTATUS("This sample only supports M300!");
    return false;
  }

  int timeout = 1;
  ErrorCode::ErrorCodeType ret;

  if (!setUpSubscription(timeout))
  {
    DERROR("Failed to set up subscription!");
    return -1;
  }
  else
  {
    DSTATUS("Set up subscription successfully!");
  }
  /*! wait for subscription data come*/
  sleep(timeout);

  /*! init mission */
  ret = initMissionSetting(timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(timeout);

  /*! upload mission */
  uploadWaypointMission(timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(timeout);

 /*! download mission */
  std::vector<WaypointV2> mission;
  downloadWaypointMission(mission, timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(timeout);

  /*! upload  actions */
  uploadWapointActions(timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(timeout);

  /*! start mission */
  startWaypointMission(timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(20);

  /*! set global cruise speed */
  setGlobalCruiseSpeed(1.5, timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(timeout);

  /*! get global cruise speed */
  getGlobalCruiseSpeed(timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(timeout);

  /*! pause the mission*/
  pauseWaypointMission(timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(5);

  /*! resume the mission*/
  resumeWaypointMission(timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(50);
  /*! Set up telemetry subscription*/
  if(!teardownSubscription(DEFAULT_PACKAGE_INDEX, timeout))
  {
    std::cout << "Failed to tear down Subscription!" << std::endl;
    return ErrorCode::SysCommonErr::UndefinedError;
  }

  return ErrorCode::SysCommonErr::Success;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::initMissionSetting(int timeout) {

  uint16_t polygonNum = 6;
  float32_t radius = 6;

  uint16_t actionNum = 5;
  srand(int(time(0)));

  /*! Generate waypoints*/


  /*! Generate actions*/
  this->actions = generateWaypointActions(actionNum);

  /*! Init waypoint settings*/
  WayPointV2InitSettings missionInitSettings;
  missionInitSettings.missionID = rand();
  missionInitSettings.repeatTimes  = 1;
  missionInitSettings.finishedAction = DJIWaypointV2MissionFinishedGoHome;
  missionInitSettings.maxFlightSpeed = 10;
  missionInitSettings.autoFlightSpeed = 2;
  missionInitSettings.exitMissionOnRCSignalLost = 1;
  missionInitSettings.gotoFirstWaypointMode = DJIWaypointV2MissionGotoFirstWaypointModePointToPoint;
  missionInitSettings.mission =  generatePolygonWaypoints(radius, polygonNum);
  missionInitSettings.missTotalLen = missionInitSettings.mission.size();

  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->init(&missionInitSettings,timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Init mission setting ErrorCode:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Init mission setting successfully!");
  }
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::uploadWaypointMission(int timeout) {

//  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->uploadMission(this->mission,timeout);
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->uploadMission(timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Upload waypoint v2 mission ErrorCode:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Upload waypoint v2 mission successfully!");
  }
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::downloadWaypointMission(std::vector<WaypointV2> &mission,int timeout)
{
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->downloadMission(mission, timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Download waypoint v2 mission ErrorCode:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Download waypoint v2 mission successfully!");
  }
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::uploadWapointActions(int timeout)
{
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->uploadAction(actions,timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Upload waypoint v2 actions ErrorCode:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Upload waypoint v2 actions successfully!");
  }
    return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::startWaypointMission(int timeout) {
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->start(timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Start waypoint v2 mission ErrorCode:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Start waypoint v2 mission successfully!");
  }
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::stopWaypointMission(int timeout) {
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->stop(timeout);
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::pauseWaypointMission(int timeout) {
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->pause(timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Pause waypoint v2 mission ErrorCode:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Pause waypoint v2 mission successfully!");
  }
  sleep(5);
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionSample::resumeWaypointMission(int timeout) {

  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->resume(timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Resume Waypoint v2 mission ErrorCode:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Resume Waypoint v2 mission successfully!");
  }
  return ret;
}

void  WaypointV2MissionSample::getGlobalCruiseSpeed(int timeout)
{
  GlobalCruiseSpeed cruiseSpeed = 0;
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->getGlobalCruiseSpeed(cruiseSpeed, timeout);
  if(ret !=  ErrorCode::SysCommonErr::Success)
  {
    DERROR("Get glogal cruise speed failed ErrorCode:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return;
  }
  DSTATUS("Current cruise speed is: %f m/s",cruiseSpeed);
}

void WaypointV2MissionSample::setGlobalCruiseSpeed(const GlobalCruiseSpeed &cruiseSpeed, int timeout)
{
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->setGlobalCruiseSpeed(cruiseSpeed, timeout);
  if(ret !=  ErrorCode::SysCommonErr::Success)
  {
    DERROR("Set glogal cruise speed %f m/s failed ErrorCode:0x%lX", cruiseSpeed, ret);
    ErrorCode::printErrorCodeMsg(ret);
    return;
  }
  DSTATUS("Current cruise speed is: %f m/s", cruiseSpeed);
}

std::vector<WaypointV2> WaypointV2MissionSample::generatePolygonWaypoints(float32_t radius, uint16_t polygonNum) {
  // Let's create a vector to store our waypoints in.
  std::vector<WaypointV2> waypointList;
  WaypointV2 startPoint;
  WaypointV2 waypointV2;

  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition = vehiclePtr->subscribe->getValue<TOPIC_GPS_FUSED>();
  startPoint.latitude  = subscribeGPosition.latitude;
  startPoint.longitude = subscribeGPosition.longitude;
  startPoint.relativeHeight = 15;
  setWaypointV2Defaults(startPoint);
  waypointList.push_back(startPoint);

  // Iterative algorithm
  for (int i = 0; i < polygonNum; i++) {
    float32_t angle = i * 2 * M_PI / polygonNum;
    setWaypointV2Defaults(waypointV2);
    float32_t X = radius * cos(angle);
    float32_t Y = radius * sin(angle);
    waypointV2.latitude = X/EARTH_RADIUS + startPoint.latitude;
    waypointV2.longitude = Y/(EARTH_RADIUS * cos(startPoint.latitude)) + startPoint.longitude;
    waypointV2.relativeHeight = startPoint.relativeHeight ;
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

void WaypointV2MissionSample::setWaypointV2Defaults(WaypointV2& waypointV2) {

  waypointV2.waypointType = DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop;
  waypointV2.headingMode = DJIWaypointV2HeadingModeAuto;
  waypointV2.config.useLocalCruiseVel = 0;
  waypointV2.config.useLocalMaxVel = 0;

  waypointV2.dampingDistance = 40;
  waypointV2.heading = 0;
  waypointV2.turnMode = DJIWaypointV2TurnModeClockwise;

  waypointV2.pointOfInterest.positionX = 0;
  waypointV2.pointOfInterest.positionY = 0;
  waypointV2.pointOfInterest.positionZ = 0;
  waypointV2.maxFlightSpeed= 9;
  waypointV2.autoFlightSpeed = 2;
}

