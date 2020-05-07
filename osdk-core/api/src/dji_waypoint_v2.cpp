/** @file dji_waypoint.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Implementation of GPS Waypoint Missions for DJI OSDK
 *
 *  @Copyright (c) 2016-2017 DJI
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

#include "dji_waypoint_v2.hpp"
#include <dji_waypoint_v2_action.hpp>
#include "dji_linker.hpp"
#include "dji_vehicle.hpp"
#include "memory.h"
#include "dji_internal_command.hpp"
#include <math.h>
using namespace DJI;
using namespace DJI::OSDK;


ErrorCode::ErrorCodeType getWP2LinkerErrorCode(E_OsdkStat cb_type) {
  switch (cb_type) {
    case OSDK_STAT_OK:
      return ErrorCode::SysCommonErr::Success;
    case OSDK_STAT_ERR_ALLOC:
      return ErrorCode::SysCommonErr::AllocMemoryFailed;
    case OSDK_STAT_ERR_TIMEOUT:
      return ErrorCode::SysCommonErr::ReqTimeout;
    default:
      return ErrorCode::SysCommonErr::UndefinedError;
  }
}

template <typename Type>
void elementEncode(const Type &data, uint16_t &tempTotalLen, uint8_t *&tempPtr) {
  if(tempPtr)
  {
    uint16_t dataLen = sizeof(Type);
    memcpy(tempPtr, (uint8_t *)&data, dataLen);
    tempTotalLen += dataLen;
    tempPtr += dataLen;
  }
  else
  {
    DERROR("Input tempPtr pointer is NULL !!!!");
    return;
  }

}

template <typename Type>
void elementDecode(Type &data, uint8_t *&tempPtr) {
  data = *(Type *)tempPtr;
  tempPtr += sizeof(Type);
}

bool missionEncode(const std::vector<WaypointV2Internal> &mission, uint8_t *pushPtr,
                   uint16_t &len) {
  if (mission.empty()) {
    pushPtr = nullptr;
    len = 0;
    return true;
  }

  bool finished = false;
  uint16_t tempTotalLen = 0;
  static uint16_t startIndex = 0;
  uint16_t endIndex = 0;
  uint8_t *tempPtr = pushPtr;

  /*! {{startIndex, endIndex, waypoint1, waypoint2,...},{startIndex, endIndex,
   * waypoint1, waypoint2,...}, ....*/
  elementEncode<uint16_t>(startIndex, tempTotalLen, tempPtr);
  uint8_t *tempTempPtr = tempPtr;

  elementEncode<uint16_t>(endIndex, tempTotalLen, tempPtr);
  uint16_t i = 0;
  for (i = startIndex; (tempTotalLen < 200) && i < mission.size(); ++i) {
    WaypointV2Internal wp = mission[i];
    elementEncode<float32_t>(wp.positionX, tempTotalLen, tempPtr);
    elementEncode<float32_t>(wp.positionY, tempTotalLen, tempPtr);
    elementEncode<float32_t>(wp.positionZ, tempTotalLen, tempPtr);
    elementEncode<DJIWaypointV2FlightPathMode>(wp.waypointType, tempTotalLen,
                                               tempPtr);
    elementEncode<DJIWaypointV2HeadingMode>(wp.headingMode, tempTotalLen,
                                            tempPtr);
    elementEncode<WaypointV2Config>(wp.config, tempTotalLen, tempPtr);

    if ((wp.waypointType == DJIWaypointV2FlightPathModeCoordinateTurn) ||
        (wp.waypointType ==
         DJIWaypointV2FlightPathModeGoToFirstPointAlongAStraightLine) ||
        (wp.waypointType == DJIWaypointV2FlightPathModeStraightOut)) {
      elementEncode<uint16_t>(wp.dampingDistance, tempTotalLen, tempPtr);
    }
    if (wp.headingMode == DJIWaypointV2HeadingWaypointCustom) {
      elementEncode<float32_t>(wp.heading, tempTotalLen, tempPtr);
      elementEncode<DJIWaypointV2TurnMode>(wp.turnMode, tempTotalLen, tempPtr);
    }
    if (wp.headingMode == DJIWaypointV2HeadingTowardPointOfInterest) {
      elementEncode<RelativePosition>(wp.pointOfInterest, tempTotalLen,
                                            tempPtr);
    }
    if (wp.config.useLocalCruiseVel == 1) {
      elementEncode<uint16_t>(wp.autoFlightSpeed , tempTotalLen, tempPtr);
    }
    if (wp.config.useLocalMaxVel == 1) {
      elementEncode<uint16_t>(wp.maxFlightSpeed, tempTotalLen, tempPtr);
    }
  }
  len = tempTotalLen;
  endIndex = i - 1;
  startIndex = endIndex + 1;
  memcpy(tempTempPtr, &endIndex, sizeof(endIndex));
  if (endIndex >= mission.size() - 1) {
    finished = true;
  }
  return finished;
}

bool missionDecode(std::vector<WaypointV2Internal> &mission, uint8_t *const pullPtr,
                   const uint16_t len) {
  bool finished = false;
  uint8_t *tempPtr = pullPtr;
  uint32_t result = 1;
  uint16_t startIndex = 0;
  uint16_t endIndex = 0;

  elementDecode<uint32_t>(result, tempPtr);
  elementDecode<uint16_t>(startIndex, tempPtr);
  elementDecode<uint16_t>(endIndex, tempPtr);

  for (int i = startIndex; i <= endIndex; i++) {
    WaypointV2Internal wp = {0};
    elementDecode<float32_t>(wp.positionX, tempPtr);
    elementDecode<float32_t>(wp.positionY, tempPtr);
    elementDecode<float32_t>(wp.positionZ, tempPtr);
    elementDecode<DJIWaypointV2FlightPathMode>(wp.waypointType, tempPtr);
    elementDecode<DJIWaypointV2HeadingMode>(wp.headingMode, tempPtr);
    elementDecode<WaypointV2Config>(wp.config, tempPtr);

    if ((wp.waypointType == DJIWaypointV2FlightPathModeCoordinateTurn) ||
        (wp.waypointType ==
         DJIWaypointV2FlightPathModeGoToFirstPointAlongAStraightLine) ||
        (wp.waypointType == DJIWaypointV2FlightPathModeStraightOut)) {
      elementDecode<uint16_t>(wp.dampingDistance, tempPtr);
    }
    if (wp.headingMode == DJIWaypointV2HeadingWaypointCustom) {
      elementDecode<float32_t>(wp.heading, tempPtr);
      elementDecode<DJIWaypointV2TurnMode>(wp.turnMode, tempPtr);
    }
    if (wp.headingMode == DJIWaypointV2HeadingTowardPointOfInterest) {
      elementDecode<RelativePosition>(wp.pointOfInterest, tempPtr);
    }
    if (wp.config.useLocalCruiseVel == 1) {
      elementDecode<uint16_t>(wp.maxFlightSpeed, tempPtr);
    }
    if (wp.config.useLocalMaxVel == 1) {
      elementDecode<uint16_t>(wp.maxFlightSpeed, tempPtr);
    }
    mission.push_back(wp);
  }
  if (pullPtr + len != tempPtr) {
    DERROR("DecompressMission error!");
  }

  return true;
}

std::vector<WaypointV2> transformMisssionInternal2Mission(std::vector<WaypointV2Internal> &missionInternal, float64_t refLatitude, float64_t refLongitude)
{
  std::vector<WaypointV2> mission;
  for(auto waypointV2Internal:missionInternal)
  {
    WaypointV2 waypointV2;
    waypointV2.longitude = float64_t (waypointV2Internal.positionX) / (EARTH_RADIUS *cos(refLatitude))+ refLongitude;
    waypointV2.latitude  = float64_t (waypointV2Internal.positionY) / EARTH_RADIUS + refLatitude ;
    waypointV2.relativeHeight    = waypointV2Internal.positionZ;
    waypointV2.waypointType      = waypointV2Internal.waypointType;
    waypointV2.headingMode       = waypointV2Internal.headingMode;
    waypointV2.config            = waypointV2Internal.config;
    waypointV2.dampingDistance   = waypointV2Internal.dampingDistance;
    waypointV2.heading           = waypointV2Internal.heading;
    waypointV2.turnMode          = waypointV2Internal.turnMode;
    waypointV2.pointOfInterest   = waypointV2Internal.pointOfInterest;
    waypointV2.maxFlightSpeed    = float32_t (waypointV2Internal.maxFlightSpeed) / 100;
    waypointV2.autoFlightSpeed   = float32_t (waypointV2Internal.autoFlightSpeed) / 100;
    mission.push_back(waypointV2);
  }
  return mission;
}

std::vector<WaypointV2Internal> transformMission2MisssionInternal(std::vector<WaypointV2> &mission)
{
  std::vector<WaypointV2Internal> missionInternal;
  for (auto waypointV2:mission)
  {
    WaypointV2Internal waypointV2Internal;
    waypointV2Internal.positionX = (waypointV2.longitude - mission[0].longitude) * EARTH_RADIUS *cos(mission[0].latitude);
    waypointV2Internal.positionY = (waypointV2.latitude - mission[0].latitude) * EARTH_RADIUS;
    waypointV2Internal.positionZ = waypointV2.relativeHeight;
    waypointV2Internal.waypointType    = waypointV2.waypointType ;
    waypointV2Internal.headingMode     = waypointV2.headingMode;
    waypointV2Internal.config          = waypointV2.config;
    waypointV2Internal.dampingDistance = waypointV2.dampingDistance;
    waypointV2Internal.heading         = waypointV2.heading;
    waypointV2Internal.turnMode        = waypointV2.turnMode;
    waypointV2Internal.pointOfInterest = waypointV2.pointOfInterest;
    waypointV2Internal.maxFlightSpeed  = uint16_t (waypointV2.maxFlightSpeed *100);
    waypointV2Internal.autoFlightSpeed = uint16_t (waypointV2.autoFlightSpeed *100);
    missionInternal.push_back(waypointV2Internal);
  }
  return missionInternal;

}

void actuatorTypeCameraEncode(const DJIWaypointV2CameraActuatorParam &actuatorCameraPtr, uint16_t &tempTotalLen, uint8_t *&tempPtr)
{
  /*! function id*/
  uint8_t retryTimes = 3;
  elementEncode<DJIWaypointV2ActionActuatorCameraOperationType>(actuatorCameraPtr.operationType, tempTotalLen, tempPtr);
  switch (actuatorCameraPtr.operationType) {
    case DJIWaypointV2ActionActuatorCameraOperationTypeTakePhoto:
    case DJIWaypointV2ActionActuatorCameraOperationTypeStartRecordVideo:
    case DJIWaypointV2ActionActuatorCameraOperationTypeStopRecordVideo: {
      elementEncode<uint8_t>(retryTimes, tempTotalLen, tempPtr);
      break;
    }
    case DJIWaypointV2ActionActuatorCameraOperationTypeFocus: {
      elementEncode<DJIWaypointV2CameraFocusParam>(actuatorCameraPtr.focusParam, tempTotalLen, tempPtr);
      break;
    }
    case DJIWaypointV2ActionActuatorCameraOperationTypeFocalLength: {
      elementEncode<DJIWaypointV2CameraFocalLengthParam>(actuatorCameraPtr.zoomParam, tempTotalLen, tempPtr);
      break;
    }
    default:
      break;
  }
}

void actuatorTypeGimbalEncode(const DJIWaypointV2GimbalActuatorParam &actuatorGimbalPtr, uint16_t &tempTotalLen, uint8_t *&tempPtr)
{
  elementEncode<DJIWaypointV2ActionActuatorGimbalOperationType>(actuatorGimbalPtr.operationType, tempTotalLen, tempPtr);
  switch (actuatorGimbalPtr.operationType) {
    case DJIWaypointV2ActionActuatorGimbalOperationTypeRotateGimbal:
      elementEncode<DJIGimbalRotation>(actuatorGimbalPtr.rotation, tempTotalLen, tempPtr);
      break;

      /*TODO  暂时不开放航点间均匀转动云台功能*/
//    case DJIWaypointV2ActionActuatorGimbalOperationTypeAircraftControlGimbal:
//      break;
    default:
      break;
  }
}

void actuatorTypeAircraftControlEncode(const DJIWaypointV2AircraftControlParam &aircraftControlPtr, uint16_t &tempTotalLen, uint8_t *&tempPtr)
{
  elementEncode<DJIWaypointV2ActionActuatorAircraftControlOperationType>(aircraftControlPtr.operationType, tempTotalLen, tempPtr);
  switch (aircraftControlPtr.operationType) {
    case DJIWaypointV2ActionActuatorAircraftControlOperationTypeRotateYaw: {
      elementEncode<DJIWaypointV2AircraftControlRotateHeadingParam>(aircraftControlPtr.yawRotatingParam, tempTotalLen, tempPtr);
      break;
    }
    case DJIWaypointV2ActionActuatorAircraftControlOperationTypeFlyingControl: {
      elementEncode<DJIWaypointV2AircraftControlFlyingParam>(aircraftControlPtr.flyControlParam, tempTotalLen, tempPtr);
      break;
    }
    default:
      break;
  }
}

void actuatorEncode(const DJIWaypointV2Actuator &actuator, uint16_t &tempTotalLen,
                    uint8_t *&tempPtr) {

  elementEncode<DJIWaypointV2ActionActuatorType>(actuator.actuatorType, tempTotalLen, tempPtr);
  elementEncode<uint8_t>(actuator.actuatorIndex, tempTotalLen, tempPtr);

  switch (actuator.actuatorType) {
    case DJIWaypointV2ActionActuatorTypeCamera: {
      actuatorTypeCameraEncode(actuator.cameraActuatorParam, tempTotalLen, tempPtr);
      break;
    }

    case DJIWaypointV2ActionActuatorTypeGimbal: {
      actuatorTypeGimbalEncode(actuator.gimbalActuatorParam, tempTotalLen, tempPtr);
      break;
    }

    case DJIWaypointV2ActionActuatorTypeAircraftControl: {
      actuatorTypeAircraftControlEncode(actuator.aircraftControlActuatorParam, tempTotalLen, tempPtr);
      break;
    }

    default:
      break;
  }
}

void triggerEncode(const DJIWaypointV2Trigger &trigger, uint16_t &tempTotalLen,
                   uint8_t *&tempPtr) {
  elementEncode<uint8_t >(trigger.actionTriggerType, tempTotalLen, *&tempPtr);
  switch (trigger.actionTriggerType) {
    case DJIWaypointV2ActionTriggerTypeReachPoint:
    {
      elementEncode<DJIWaypointV2ReachPointTriggerParam>(trigger.reachPointTriggerParam, tempTotalLen, *&tempPtr);
    }

    case DJIWaypointV2ActionTriggerTypeActionAssociated: {
      elementEncode<DJIWaypointV2AssociateTriggerParam>(trigger.associateTriggerParam, tempTotalLen, *&tempPtr);
      break;
    }

    case DJIWaypointV2ActionTriggerTypeTrajectory: {
      elementEncode<DJIWaypointV2TrajectoryTriggerParam>(trigger.trajectoryTriggerParam, tempTotalLen, *&tempPtr);
      break;
    }

    case DJIWaypointV2ActionTriggerTypeInterval: {
      elementEncode<DJIWaypointV2IntervalTriggerParam>(trigger.intervalTriggerParam, tempTotalLen, *&tempPtr);
      break;
    }
    case DJIWaypointV2ActionTriggerTypeSampleReachPoint:{
      elementEncode<DJIWaypointV2SampleReachPointTriggerParam>(trigger.sampleReachPointTriggerParam, tempTotalLen, *&tempPtr);
      break;
    }
    default:
      DERROR("Invalid trigger type%d\n", trigger.actionTriggerType);
      break;
  }
}

bool ActionsEncode(std::vector<DJIWaypointV2Action> &actions,
                   uint8_t *pushPtr, uint16_t &len) {
  uint16_t i;
  bool finished = false;
  uint16_t tempTotalLen = 0;
  uint8_t *tempPtr = pushPtr;

  static uint16_t startIndex = 0;
  for (i = startIndex; (i < actions.size()) && (tempTotalLen < 100); ++i) {
    DJIWaypointV2Action action = actions[i];

    /*! actionId*/
    elementEncode<uint16_t>(action.actionId, tempTotalLen, tempPtr);

    /*! trigger*/
    triggerEncode(action.trigger, tempTotalLen, tempPtr);

    /*! actuator*/
    actuatorEncode(action.actuator, tempTotalLen, tempPtr);

    len = tempTotalLen;
  }
  startIndex = i;
  if (startIndex >= actions.size() - 1) {
    finished = true;
    startIndex = 0;
  }
  return finished;
}

T_CmdInfo setCmdInfoDefault(Vehicle *vehicle, const uint8_t cmd[],
                            uint16_t len) {
  T_CmdInfo cmdInfo = {0};
  cmdInfo.receiver = OSDK_COMMAND_FLIGHT_ID;
  cmdInfo.sender = vehicle->linker->getLocalSenderId();
  cmdInfo.cmdSet = cmd[0];
  cmdInfo.cmdId = cmd[1];
  cmdInfo.dataLen = len;
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.encType = vehicle->getEncryption();
  cmdInfo.channelId = 0;
  return cmdInfo;
}

E_OsdkStat updateMissionState(T_CmdHandle *cmdHandle, const T_CmdInfo *cmdInfo,
                              const uint8_t *cmdData, void *userData) {

  if (cmdInfo) {
    if (userData) {
      auto *wp2Ptr = (WaypointV2MissionOperator *)userData;
      auto *missionStatePushAck =
        (DJI::OSDK::MissionStatePushAck *)cmdData;

      wp2Ptr->setCurrentState(wp2Ptr->getCurrentState());
      wp2Ptr->setCurrentState(
        (DJI::OSDK::DJIWaypointV2MissionState)
          missionStatePushAck->data.state);
      //     DSTATUS("missionStatePushAck->commonDataVersion%d\n",missionStatePushAck->commonDataVersion);
      //     DSTATUS("missionStatePushAck->commonDataLen%d\n",missionStatePushAck->commonDataLen);
      //     DSTATUS("missionStatePushAck->data%d\n",missionStatePushAck->data);
           DSTATUS("missionStatePushAck->data.state%x\n",missionStatePushAck->data.state);
      //     DSTATUS("missionStatePushAck->data.stateDetail%x\n",missionStatePushAck->data.stateDetail);
      //     DSTATUS("missionStatePushAck->data.curWaypointIndex%d\n",missionStatePushAck->data.curWaypointIndex);
    } else {
      DERROR("cmdInfo is a null value");
    }
    return OSDK_STAT_OK;
  }
  return OSDK_STAT_ERR_ALLOC;
}

E_OsdkStat updateOSDbrodcast(T_CmdHandle *cmdHandle, const T_CmdInfo *cmdInfo,
                             const uint8_t *cmdData, void *userData)
{
  if (cmdInfo) {
    if (userData) {
      auto *wp2Ptr = (WaypointV2MissionOperator *) userData;
      auto *gsStationData = (DJI::OSDK::FCGroundStationDataPush *) cmdData;
      //wp2Ptr->setTakeoffAltitude(gsStationData->takeoff_alti);
      wp2Ptr->setTakeoffAltitude(gsStationData->takeoff_alti);
    } else {
      DERROR("cmdInfo is a null value");
    }
    return OSDK_STAT_OK;
  }
  return OSDK_STAT_SYS_ERR;
}
/*! 仅仅推送了0x00,0x10,0x11这几个类别的事件*/
E_OsdkStat updateMissionEvent(T_CmdHandle *cmdHandle, const T_CmdInfo *cmdInfo,
                              const uint8_t *cmdData, void *userData) {

  if (cmdInfo) {
    if (userData) {

      auto *wp2Ptr = (WaypointV2MissionOperator *)userData;
      auto *MissionEventPushAck =
        (DJI::OSDK::MissionEventPushAck *)cmdData;

      DSTATUS("MissionEventPushAck->event ID :0x%x\n", MissionEventPushAck->event);

      if(MissionEventPushAck->event == 0x01)
        DSTATUS("interruptReason:0x%x\n",MissionEventPushAck->data.interruptReason);
      if(MissionEventPushAck->event == 0x02)
        DSTATUS("recoverProcess:0x%x\n",MissionEventPushAck->data.recoverProcess);
      if(MissionEventPushAck->event == 0x03)
        DSTATUS("finishReason:0x%x\n",MissionEventPushAck->data.finishReason);

      if(MissionEventPushAck->event == 0x10)
        DSTATUS("current waypointIndex:%d\n",MissionEventPushAck->data.waypointIndex);

      if(MissionEventPushAck->event == 0x11)
      {
        DSTATUS("currentMissionExecNum:%d\n",MissionEventPushAck->data.MissionExecEvent.currentMissionExecNum);
      }
      if(MissionEventPushAck->event == 0x30)
      {
        DSTATUS("currentActionId:0x%x\n",MissionEventPushAck->data.ActionExecEvent.actionId);
        DSTATUS("preActuatorState:0x%x\n",MissionEventPushAck->data.ActionExecEvent.preActuatorState);
        DSTATUS("curActuatorState:0x%x\n",MissionEventPushAck->data.ActionExecEvent.curActuatorState);
        DSTATUS("result:%x\n",MissionEventPushAck->data.ActionExecEvent.result);
      }
      return OSDK_STAT_OK;
    }
  }
  return OSDK_STAT_SYS_ERR;
}
//void WaypointV2MissionOperator::RegisterMissionEventCallback() {
//  static T_RecvCmdHandle handle = {0};
//  static T_RecvCmdItem item = {0};
//  handle.protoType = PROTOCOL_V1;
//  handle.cmdCount = 1;
//  handle.cmdList = &item;
//  item.device = OSDK_COMMAND_FC_2_DEVICE_ID;
//  item.cmdSet = V1ProtocolCMD::waypointV2::waypointGetEventPushDataV2[0];
//  item.cmdId = V1ProtocolCMD::waypointV2::waypointGetEventPushDataV2[1];
//  item.mask = MASK_HOST_DEVICE_SET_ID;
//  item.host = 0;
//  item.device = 0;
//  item.pFunc = updateMissionEvent;
//  item.userData = this;
//  bool registerRet = vehiclePtr->linker->registerCmdHandler(&handle);
//  DSTATUS("register result of geting mission event pushing : %d\n",
//          registerRet);
//}


//void RegisterMissionStateCallback(Vehicle *vehiclePtr, WaypointV2MissionOperator *missionOperator) {
//  static T_RecvCmdHandle handle = {0};
//  static T_RecvCmdItem item = {0};
//  handle.protoType = PROTOCOL_V1;
//  handle.cmdCount = 1;
//  handle.cmdList = &item;
//  item.device = OSDK_COMMAND_FC_2_DEVICE_ID;
//  item.cmdSet = V1ProtocolCMD::waypointV2::waypointGetStatePushDataV2[0];
//  item.cmdId = V1ProtocolCMD::waypointV2::waypointGetStatePushDataV2[1];
//  item.mask = MASK_HOST_DEVICE_SET_ID;
//  item.host = 0;
//  item.device = 0;
//  item.pFunc = updateMissionState;
//  item.userData = &missionOperator;
//  bool registerRet = vehiclePtr->linker->registerCmdHandler(&handle);
//  DSTATUS("register result of geting mission state pushing : %d\n",
//          registerRet);
//}

void WaypointV2MissionOperator::RegisterOSDInfoCallback(Vehicle *vehiclePtr) {
  static T_RecvCmdHandle handle = {0};
  static T_RecvCmdItem item = {0};
  handle.protoType = PROTOCOL_V1;
  handle.cmdCount = 1;
  handle.cmdList = &item;
  item.device = OSDK_COMMAND_FC_2_DEVICE_ID;
  item.cmdSet =0x03;
  item.cmdId = 0x6c;
  item.mask = MASK_HOST_DEVICE_SET_ID;
  item.host = 0;
  item.device = 0;
  item.pFunc = updateOSDbrodcast;
  item.userData = this;
  bool registerRet = vehiclePtr->linker->registerCmdHandler(&handle);
  DSTATUS("register result of geting　FC ground station status pushing : %d\n",
          registerRet);
}

WaypointV2MissionOperator::WaypointV2MissionOperator(Vehicle *vehiclePtr) {
  this->vehiclePtr = vehiclePtr;
  currentState = DJIWaypointV2MissionStateUnWaypointActionActuatorknown;
  prevState = DJIWaypointV2MissionStateUnWaypointActionActuatorknown;
//  RegisterMissionStateCallback(vehiclePtr, this);
//  RegisterMissionEventCallback();
  RegisterOSDInfoCallback(vehiclePtr);
}

WaypointV2MissionOperator::~WaypointV2MissionOperator() {
}

ErrorCode::ErrorCodeType WaypointV2MissionOperator::init(WayPointV2InitSettings *info, int timeout)
{
  T_CmdInfo ackInfo = {0};
  RetCodeType ackData[1024];

  WayPointV2InitSettingsInternal initSettingsInternal;
  initSettingsInternal.missionID    = info->missionID;
  initSettingsInternal.missTotalLen = info->missTotalLen;
  initSettingsInternal.waypointCount = 2;
  initSettingsInternal.repeatTimes = info->repeatTimes;
  initSettingsInternal.finishedAction = info->finishedAction;
  /*! Unit transform from m/s to cm/s*/
  initSettingsInternal.maxFlightSpeed =  uint16_t (info->maxFlightSpeed * 100);
  initSettingsInternal.autoFlightSpeed = uint16_t (info->autoFlightSpeed * 100);

  initSettingsInternal.startIndex     = 0;
  initSettingsInternal.exitMissionOnRCSignalLost = info->exitMissionOnRCSignalLost;
  initSettingsInternal.gotoFirstWaypointMode = info->gotoFirstWaypointMode;
  if(info->mission.size() >= 2)
  {
    initSettingsInternal.refLati = info->mission.front().latitude;
    initSettingsInternal.refLong = info->mission.front().longitude;
   // DSTATUS("initSettingsInternal.refLati %f\n",initSettingsInternal.refLati );
    missionV2 = info->mission;
  }
  else
  {
    DERROR("Mission's waypoint number must larger than 2 ");
    return ErrorCode::SysCommonErr::ReqNotSupported;
  }
  /*!Set reference altitude is takeoff altitude*/
  initSettingsInternal.refAlti = this->getTakeoffAltitude();
  //DSTATUS("initSettingsInternal.refAlti %f\n",initSettingsInternal.refAlti );
  T_CmdInfo cmdInfo =
    setCmdInfoDefault(vehiclePtr, V1ProtocolCMD::waypointV2::waypointInitV2,
                      sizeof(WayPointV2InitSettingsInternal));
  E_OsdkStat linkAck = vehiclePtr->linker->sendSync(&cmdInfo, (uint8_t *)&initSettingsInternal, &ackInfo,
                                                    ackData, timeout * 1000 / 4, 4);

  ErrorCode::ErrorCodeType ret = getWP2LinkerErrorCode(linkAck);

  if (ret != ErrorCode::SysCommonErr::Success) return ret;
  if (ackInfo.dataLen >= sizeof(RetCodeType)) {
    auto *ackCode = (WaypointV2CommonAck *)ackData;
    return ErrorCode::getErrorCode(ErrorCode::MissionV2Module,
                                   ErrorCode::MissionV2Common, *ackCode);
  } else {
    return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
}

ErrorCode::ErrorCodeType WaypointV2MissionOperator::downloadInitSetting(WayPointV2InitSettingsInternal &info, int timeout)
{
  T_CmdInfo ackInfo = {0};
  RetCodeType ackData[1024];
  WayPointV2InitSettingsInternal initSettingsInternal;

  T_CmdInfo cmdInfo =
    setCmdInfoDefault(vehiclePtr, V1ProtocolCMD::waypointV2::waypointDownloadInitV2,
                      sizeof(WayPointV2InitSettingsInternal));
  E_OsdkStat linkAck = vehiclePtr->linker->sendSync(&cmdInfo, (uint8_t *)&initSettingsInternal, &ackInfo,
                                                    ackData, timeout * 1000 / 4, 4);

  ErrorCode::ErrorCodeType ret = getWP2LinkerErrorCode(linkAck);

  if (ret != ErrorCode::SysCommonErr::Success) return ret;
  if (ackInfo.dataLen >= sizeof(RetCodeType)) {
    auto *ackCode = (DownloadInitSettingRawAck *)ackData;
    info = ackCode->initSettingsInternal;
    return ErrorCode::getErrorCode(ErrorCode::MissionV2Module,
                                   ErrorCode::MissionV2Common, ackCode->result);
  } else {
    return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
}

ErrorCode::ErrorCodeType WaypointV2MissionOperator::uploadMission(
  int timeout) {
  std::vector<WaypointV2Internal> mission = transformMission2MisssionInternal(this->missionV2);

  uint16_t dataLengthSinglePush = 0;
  bool finished = false;
  while (!finished) {
    void *waypointPushPtr = malloc(400);
    finished = missionEncode(mission, (uint8_t *) waypointPushPtr,
                             dataLengthSinglePush);
    T_CmdInfo ackInfo = {0};
    RetCodeType ackData[1024] = {0};

    T_CmdInfo cmdInfo =
      setCmdInfoDefault(vehiclePtr, V1ProtocolCMD::waypointV2::waypointUploadV2,
                        dataLengthSinglePush);

    E_OsdkStat linkAck =
      vehiclePtr->linker->sendSync(&cmdInfo, (uint8_t *)waypointPushPtr, &ackInfo,
                                   ackData, timeout * 1000 / 4, 4);
    ErrorCode::ErrorCodeType ret = getWP2LinkerErrorCode(linkAck);
    free(waypointPushPtr);

    if (ret != ErrorCode::SysCommonErr::Success) {
      return ret;
    }
    if (ackInfo.dataLen >= sizeof(RetCodeType)) {
      auto *ackCode = (UploadMissionRawAck *)ackData;
      if (ackCode->result != 0)
        return ErrorCode::getErrorCode(ErrorCode::MissionV2Module,
                                       ErrorCode::MissionV2Common,
                                       ackCode->result);
    } else {
      return ErrorCode::SysCommonErr::UnpackDataMismatch;
    }
  }
  return ErrorCode::SysCommonErr::Success;
}

ErrorCode::ErrorCodeType WaypointV2MissionOperator::downloadMission(
    std::vector<WaypointV2> &mission, int timeout) {

  bool finished = false;
  const uint8_t maxDownLoadNum = 10;
  uint16_t StartIndex = 0;
  uint16_t EndIndex = 0;
  std::vector<WaypointV2Internal> missionInternal;
  static uint16_t startIndex = 0;
  uint16_t endIndex = 0;

  static DownloadMissionRsp downloadMissionRsp = {0};
  T_CmdInfo ackInfo = {0};
  RetCodeType ackData[1024];

  GetWaypontStartEndIndexAck startEndIndexAck = {0};
  ErrorCode::ErrorCodeType ret =
      getWaypointIndexInList(startEndIndexAck, timeout);
  if (ret != ErrorCode::SysCommonErr::Success)
    return ret;
  else {
    StartIndex = startEndIndexAck.startIndex;
    EndIndex = startEndIndexAck.endIndex;
  }

  T_CmdInfo cmdInfo = setCmdInfoDefault(
      vehiclePtr, V1ProtocolCMD::waypointV2::waypointDownloadPtV2,
      sizeof(downloadMissionRsp));

  startIndex = StartIndex;
  endIndex = EndIndex - startIndex + 1 >= maxDownLoadNum? startIndex + maxDownLoadNum -1: EndIndex;

  while (!finished) {
    downloadMissionRsp ={startIndex,endIndex};
    E_OsdkStat linkAck =
        vehiclePtr->linker->sendSync(&cmdInfo, (uint8_t *)&downloadMissionRsp, &ackInfo,
                         ackData, timeout * 1000 / 4, 4);
    startIndex = endIndex +1;
    if(startIndex > EndIndex)
    {
      startIndex = 0;
      finished = true;
    } else{
      uint16_t remains = (EndIndex - startIndex +1);
      if(remains >= maxDownLoadNum)
        endIndex = startIndex + maxDownLoadNum - 1;
      else
        endIndex = EndIndex;
    }

    ret = getWP2LinkerErrorCode(linkAck);
    if (ret != ErrorCode::SysCommonErr::Success) {
      return ret;
    }
    if (ackInfo.dataLen >= sizeof(RetCodeType)) {
      auto *ackCode = (WaypointV2CommonAck *)ackData;
      if (*ackCode != 0)
        return ErrorCode::getErrorCode(
            ErrorCode::MissionV2Module, ErrorCode::MissionV2Common, *ackCode);
      else {
        missionDecode(missionInternal, (uint8_t *) ackData, ackInfo.dataLen);
      }
    } else {
      return ErrorCode::SysCommonErr::UnpackDataMismatch;
    }
  }
  WayPointV2InitSettingsInternal info;
  ret = downloadInitSetting(info,timeout);
  if (ret != ErrorCode::SysCommonErr::Success) {
    return ret;
  }
  else
  {
    mission = transformMisssionInternal2Mission(missionInternal, info.refLati, info.refLong);
  }
  return ErrorCode::SysCommonErr::Success;
}


ErrorCode::ErrorCodeType WaypointV2MissionOperator::uploadAction(
  std::vector<DJIWaypointV2Action> &actions, int timeout) {
  if (actions.size() == 0) {
    DERROR("Action number is zero, please reset actions vector");
  } else {
    bool finished = false;
    E_OsdkStat linkAck;
    while (!finished) {
      uint16_t dataLen = 0;
      void *actionsPushPtr = malloc(400);
      T_CmdInfo ackInfo = {0};
      RetCodeType ackData[1024];

      finished = ActionsEncode(actions, (uint8_t *) actionsPushPtr, dataLen);
      T_CmdInfo cmdInfo = setCmdInfoDefault(
          vehiclePtr, V1ProtocolCMD::waypointV2::waypointUploadActionV2, dataLen);

      linkAck = vehiclePtr->linker->sendSync(&cmdInfo, (uint8_t *)actionsPushPtr, &ackInfo,
                                 ackData, timeout * 1000 / 4, 4);
      free(actionsPushPtr);
      ErrorCode::ErrorCodeType ret = getWP2LinkerErrorCode(linkAck);

      if (ret != ErrorCode::SysCommonErr::Success) {
        return ret;
      }
      if (ackInfo.dataLen >= sizeof(RetCodeType)) {
        auto *ackCode = (UploadActionsRawAck *)ackData;
        if (ackCode->result != 0)
          return ErrorCode::getErrorCode(ErrorCode::MissionV2Module,
                                         ErrorCode::MissionV2Common,
                                         ackCode->result);
      } else {
        return ErrorCode::SysCommonErr::UnpackDataMismatch;
      }
    }
  }
  return ErrorCode::SysCommonErr::Success;
}

ErrorCode::ErrorCodeType WaypointV2MissionOperator::getActionRemainMemory(
    GetRemainRamAck &remainRamAck, int timeout) {
  bool finished = false;
  E_OsdkStat linkAck;
  uint16_t dataLen = 0;
  T_CmdInfo ackInfo = {0};
  RetCodeType ackData[1024];

  T_CmdInfo cmdInfo = setCmdInfoDefault(
      vehiclePtr, V1ProtocolCMD::waypointV2::waypointGetRemainSpaceV2, dataLen);

  linkAck = vehiclePtr->linker->sendSync(&cmdInfo, (uint8_t *)&dataLen, &ackInfo, ackData,
                             timeout * 1000 / 4, 4);

  auto *ack = (GetRemainRamAck *)ackData;
  DSTATUS("Total memory is:%d\n", ack->totalMemory);
  DSTATUS("Remain memory is:%d\n", ack->remainMemory);
  ErrorCode::ErrorCodeType ret = getWP2LinkerErrorCode(linkAck);
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionOperator::getWaypointIndexInList(
    GetWaypontStartEndIndexAck &startEndIndexAck, int timeout) {
  bool finished = false;
  E_OsdkStat linkAck;
  uint16_t dataLen = 0;
  T_CmdInfo ackInfo = {0};
  RetCodeType ackData[1024];

  T_CmdInfo cmdInfo = setCmdInfoDefault(
      vehiclePtr, V1ProtocolCMD::waypointV2::waypointGetWayptIdxInListV2, dataLen);

  linkAck = vehiclePtr->linker->sendSync(&cmdInfo, (uint8_t *)&dataLen, &ackInfo, ackData,
                             timeout * 1000 / 4, 4);

  ErrorCode::ErrorCodeType ret = getWP2LinkerErrorCode(linkAck);

  if (ret != ErrorCode::SysCommonErr::Success) {
    return ret;
  }
  if (ackInfo.dataLen >= sizeof(RetCodeType)) {
    auto *ackCode = (GetWaypontStartEndIndexAck *)ackData;
    startEndIndexAck = *ackCode;
    if (ackCode->result != 0)
      return ErrorCode::getErrorCode(
          ErrorCode::MissionV2Module, ErrorCode::MissionV2Common, ackCode->result);
  } else {
    return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
  return ret;
}

ErrorCode::ErrorCodeType WaypointV2MissionOperator::getGlobalCruiseSpeed(
  GlobalCruiseSpeed &cruiseSpeed, int timeout) {
  T_CmdInfo ackInfo = {0};
  RetCodeType ackData[1024];
  T_CmdInfo cmdInfo = setCmdInfoDefault(
      vehiclePtr, V1ProtocolCMD::waypointV2::waypointGetGlobVelocityV2,
      sizeof(GlobalCruiseSpeed));

  E_OsdkStat linkAck =
      vehiclePtr->linker->sendSync(&cmdInfo, (uint8_t *)&cruiseSpeed, &ackInfo, ackData,
                       timeout * 1000 / 4, 4);
  ErrorCode::ErrorCodeType ret = getWP2LinkerErrorCode(linkAck);
  if (ret != ErrorCode::SysCommonErr::Success) return ret;
  if (ackInfo.dataLen >= sizeof(RetCodeType)) {
    auto *ackCode = (GetGlobalCruiseVelAck *)ackData;
    cruiseSpeed =  ((float32_t)ackCode->globalCruiseVel)/100;
    return ErrorCode::getErrorCode(ErrorCode::MissionV2Module,
                                   ErrorCode::MissionV2Common, ackCode->result);
  } else {
    return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
}

ErrorCode::ErrorCodeType WaypointV2MissionOperator::setGlobalCruiseSpeed(
  const GlobalCruiseSpeed &cruiseSpeed, int timeout) {
  uint16_t cruiseSpeedCmperSec =  uint16_t (cruiseSpeed * 100);
  T_CmdInfo cmdInfo = setCmdInfoDefault(
      vehiclePtr, V1ProtocolCMD::waypointV2::waypointSetGlobVelocityV2,
      sizeof(cruiseSpeedCmperSec));
  T_CmdInfo ackInfo = {0};
  RetCodeType ackData[1024];

  E_OsdkStat linkAck =
      vehiclePtr->linker->sendSync(&cmdInfo, (uint8_t *)&cruiseSpeedCmperSec, &ackInfo, ackData,
                       timeout * 1000 / 4, 4);
  ErrorCode::ErrorCodeType ret = getWP2LinkerErrorCode(linkAck);
  if (ret != ErrorCode::SysCommonErr::Success) return ret;
  if (ackInfo.dataLen >= sizeof(RetCodeType)) {
    auto *ackCode = (WaypointV2CommonAck *)ackData;
    return ErrorCode::getErrorCode(ErrorCode::MissionV2Module,
                                   ErrorCode::MissionV2Common, *ackCode);
  } else {
    return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
}

ErrorCode::ErrorCodeType WaypointV2MissionOperator::start(int timeout) {
  /*start = 1; stop = 2*/

  uint8_t start = 1;
  T_CmdInfo ackInfo = {0};
  RetCodeType ackData[1024];

  T_CmdInfo cmdInfo = setCmdInfoDefault(
      vehiclePtr, V1ProtocolCMD::waypointV2::waypointStartStopV2, sizeof(start));
  E_OsdkStat linkAck = vehiclePtr->linker->sendSync(&cmdInfo, (uint8_t *)&start, &ackInfo,
                                        ackData, timeout * 1000 / 4, 4);
  ErrorCode::ErrorCodeType ret = getWP2LinkerErrorCode(linkAck);

  if (ret != ErrorCode::SysCommonErr::Success) return ret;
  if (ackInfo.dataLen >= sizeof(RetCodeType)) {
    auto *ackCode = (WaypointV2CommonAck *)ackData;
    return ErrorCode::getErrorCode(ErrorCode::MissionV2Module,
                                   ErrorCode::MissionV2Common, *ackCode);
  } else {
    return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
}

ErrorCode::ErrorCodeType WaypointV2MissionOperator::stop(int timeout) {
  /*start = 1; stop = 2*/
  uint8_t stop = 2;
  T_CmdInfo ackInfo = {0};
  RetCodeType ackData[1024];
  T_CmdInfo cmdInfo = setCmdInfoDefault(
      vehiclePtr, V1ProtocolCMD::waypointV2::waypointStartStopV2, sizeof(stop));
  E_OsdkStat linkAck = vehiclePtr->linker->sendSync(&cmdInfo, (uint8_t *)&stop, &ackInfo,
                                        ackData, timeout * 1000 / 4, 4);
  ErrorCode::ErrorCodeType ret = getWP2LinkerErrorCode(linkAck);

  if (ret != ErrorCode::SysCommonErr::Success) return ret;
  if (ackInfo.dataLen >= sizeof(RetCodeType)) {
    auto *ackCode = (WaypointV2CommonAck *)ackData;
    return ErrorCode::getErrorCode(ErrorCode::MissionV2Module,
                                   ErrorCode::MissionV2Common, *ackCode);
  } else {
    return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
}

ErrorCode::ErrorCodeType WaypointV2MissionOperator::pause(int timeout) {
  /*pause = 1; resume = 2*/
  uint8_t pause = 1;
  T_CmdInfo ackInfo = {0};
  RetCodeType ackData[1024];

  T_CmdInfo cmdInfo = setCmdInfoDefault(
      vehiclePtr, V1ProtocolCMD::waypointV2::waypointBreakRestoreV2, sizeof(pause));
  E_OsdkStat linkAck = vehiclePtr->linker->sendSync(&cmdInfo, (uint8_t *)&pause, &ackInfo,
                                        ackData, timeout * 1000 / 4, 4);
  ErrorCode::ErrorCodeType ret = getWP2LinkerErrorCode(linkAck);

  if (ret != ErrorCode::SysCommonErr::Success) return ret;
  if (ackInfo.dataLen >= sizeof(RetCodeType)) {
    auto *ackCode = (WaypointV2CommonAck *)ackData;
    return ErrorCode::getErrorCode(ErrorCode::MissionV2Module,
                                   ErrorCode::MissionV2Common, *ackCode);
  } else {
    return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
}

ErrorCode::ErrorCodeType WaypointV2MissionOperator::resume(int timeout) {
  /*pause = 1; resume = 2*/
  uint8_t resume = 2;
  T_CmdInfo ackInfo = {0};
  RetCodeType ackData[1024];

  T_CmdInfo cmdInfo = setCmdInfoDefault(
      vehiclePtr, V1ProtocolCMD::waypointV2::waypointBreakRestoreV2,
      sizeof(resume));
  E_OsdkStat linkAck = vehiclePtr->linker->sendSync(&cmdInfo, (uint8_t *)&resume, &ackInfo,
                                        ackData, timeout * 1000 / 4, 4);
  ErrorCode::ErrorCodeType ret = getWP2LinkerErrorCode(linkAck);

  if (ret != ErrorCode::SysCommonErr::Success) return ret;
  if (ackInfo.dataLen >= sizeof(RetCodeType)) {
    auto *ackCode = (WaypointV2CommonAck *)ackData;
    return ErrorCode::getErrorCode(ErrorCode::MissionV2Module,
                                   ErrorCode::MissionV2Common, *ackCode);
  } else {
    return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
}






