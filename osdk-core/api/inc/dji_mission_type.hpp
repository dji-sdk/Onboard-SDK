/** @file dji_mission_type.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Mission related data struct for DJI OSDK library
 *
 *  @Copyright (c) 2017 DJI
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

#ifndef ONBOARDSDK_DJI_MISSION_TYPE_H
#define ONBOARDSDK_DJI_MISSION_TYPE_H
#include <vector>
#include "dji_type.hpp"

namespace DJI
{

namespace OSDK
{
  const float32_t EARTH_RADIUS =  6378137.0;

// clang-format off
/**********Mission structs/Enums***********/

#pragma pack(1)

/**
 * @brief HotPoint Mission Initialization settings
 * @details This is one of the few structs in the OSDK codebase that
 * is used in both a sending and a receiving API.
 */
typedef struct HotPointSettings
{
  uint8_t   version;   /*!< Reserved, kept as 0 */
  float64_t latitude;  /*!< Latitude (radian) */
  float64_t longitude; /*!< Longitude (radian) */
  float64_t height;    /*!< Altitude (relative altitude from takeoff point */
  float64_t radius;    /*!< Radius (5m~500m) */
  float32_t yawRate;   /*!< Angle rate (0~30°/s) */
  uint8_t   clockwise; /*!< 0->fly in counter-clockwise direction, 1->clockwise direction */
  uint8_t startPoint;  /*!< Start point position <br>*/
  /*!< 0: north to the hot point <br>*/
  /*!< 1: south to the hot point <br>*/
  /*!< 2: west to the hot point <br>*/
  /*!< 3: east to the hot point <br>*/
  /*!< 4: from current position to nearest point on the hot point */
  uint8_t yawMode; /*!< Yaw mode <br>*/
  /*!< 0: point to velocity direction <br>*/
  /*!< 1: face inside <br>*/
  /*!< 2: face ouside <br>*/
  /*!< 3: controlled by RC <br>*/
  /*!< 4: same as the starting yaw<br> */
  uint8_t reserved[11]; /*!< Reserved */
} HotPointSettings;     // pack(1)

/**
 * @brief Waypoint Mission Initialization settings
 * @details This is one of the few structs in the OSDK codebase that
 * is used in both a sending and a receiving API.
 */
typedef struct WayPointInitSettings
{
  uint8_t   indexNumber; /*!< Total number of waypoints <br>*/
  float32_t maxVelocity; /*!< Maximum speed joystick input(2~15m) <br>*/
  float32_t idleVelocity; /*!< Cruising Speed */
  /*!< (without joystick input, no more than vel_cmd_range) */
  uint8_t finishAction; /*!< Action on finish <br>*/
  /*!< 0: no action <br>*/
  /*!< 1: return to home <br>*/
  /*!< 2: auto landing <br>*/
  /*!< 3: return to point 0 <br>*/
  /*!< 4: infinite mode， no exit <br>*/
  uint8_t executiveTimes; /*!< Function execution times <br>*/
  /*!< 1: once <br>*/
  /*!< 2: twice <br>*/
  uint8_t yawMode; /*!< Yaw mode <br>*/
  /*!< 0: auto mode(point to next waypoint) <br>*/
  /*!< 1: lock as an initial value <br>*/
  /*!< 2: controlled by RC <br>*/
  /*!< 3: use waypoint's yaw(tgt_yaw) */
  uint8_t traceMode; /*!< Trace mode <br>*/
  /*!< 0: point to point, after reaching the target waypoint hover, 
   * complete waypoints action (if any), 
   * then fly to the next waypoint <br>
   * 1: Coordinated turn mode, smooth transition between waypoints,
   * no waypoints task <br>
   */
  uint8_t RCLostAction; /*!< Action on rc lost <br>*/
  /*!< 0: exit waypoint and failsafe <br>*/
  /*!< 1: continue the waypoint <br>*/
  uint8_t gimbalPitch; /*!< Gimbal pitch mode <br>*/
  /*!< 0: free mode, no control on gimbal <br>*/
  /*!< 1: auto mode, Smooth transition between waypoints <br>*/
  float64_t latitude;     /*!< Focus latitude (radian) */
  float64_t longitude;    /*!< Focus longitude (radian) */
  float32_t altitude;     /*!< Focus altitude (relative takeoff point height) */
  uint8_t   reserved[16]; /*!< Reserved, must be set to 0 */

} WayPointInitSettings; // pack(1)

/**
 * @brief Waypoint settings for individual waypoints being added to the mission
 * @details This is one of the few structs in the OSDK codebase that
 * is used in both a sending and a receiving API.
 */
typedef struct WayPointSettings
{
  uint8_t   index;     /*!< Index to be uploaded */
  float64_t latitude;  /*!< Latitude (radian) */
  float64_t longitude; /*!< Longitude (radian) */
  float32_t altitude;  /*!< Altitude (relative altitude from takeoff point) */
  float32_t damping; /*!< Bend length (effective coordinated turn mode only) */
  int16_t   yaw;     /*!< Yaw (degree) */
  int16_t   gimbalPitch; /*!< Gimbal pitch */
  uint8_t   turnMode;    /*!< Turn mode <br> */
  /*!< 0: clockwise <br>*/
  /*!< 1: counter-clockwise <br>*/
  uint8_t reserved[8]; /*!< Reserved */
  uint8_t hasAction;   /*!< Action flag <br>*/
  /*!< 0: no action <br>*/
  /*!< 1: has action <br>*/
  uint16_t actionTimeLimit;      /*!< Action time limit */
  uint8_t  actionNumber : 4;     /*!< Total number of actions */
  uint8_t  actionRepeat : 4;     /*!< Total running times */
  uint8_t  commandList[16];      /*!< Command list */
  uint16_t commandParameter[16]; /*!< Command parameters */
} WayPointSettings;              // pack(1)

/**
 * @brief WayPoint Reached Data Incident Type enumerator
 */
//! @note can be separated by the first bytes of data
typedef enum WayPointIncidentType
{
  NAVI_UPLOAD_FINISH,
  NAVI_MISSION_FINISH,
  NAVI_MISSION_WP_REACH_POINT
} WayPointIncidentType;

/**
 * @brief WayPoint Push Data Incident Type enumerator
 */
typedef enum WayPointPushDataIncidentType
{
  NAVI_MODE_ATTI,
  NAVI_MISSION_WAYPOINT,
  NAVI_MISSION_HOTPOINT,
  NAVI_MISSION_FOLLOWME,
  NAVI_MISSION_IOC,
} WayPointPushDataIncidentType;

/**
 * @brief Waypoint Mission Finish Event Push Data
 */
typedef struct WayPointFinishData
{
  uint8_t  incident_type; /*! see WayPointIncidentType */
  uint8_t  repeat;
  uint16_t reserved_1;
  uint16_t reserved_2;
} WayPointFinishData; // pack(1)


/**********Mission waypoint V2 structs/Enums***********/

/**
 *  The action of waypoint mission that will be executed when the remote controller signal lost.
 */
enum DJIWaypointV2MissionV2RCLostAction:uint8_t {


  /**
   *  Stops current mission when the remote controller signal is lost.
   */
    DJIWaypointV2MissionV2RCLostActionStopMission,

  /**
   *  Continues the mission when the remote controller signal is lost.
   */
    DJIWaypointV2MissionV2RCLostActionContinue,
};


/**
*  Actions will be taken when the waypoint mission is finished.
*/
enum DJIWaypointV2MissionFinishedAction:uint8_t {

  /**
   *  No further action will be taken.
   *  The aircraft can be controlled by the remote controller.
   */
    DJIWaypointV2MissionFinishedNoAction,

  /**
   *  Goes home when the mission is finished. The aircraft will
   *  land directly if it is within 20 meters away from the home point.
   */
    DJIWaypointV2MissionFinishedGoHome,

  /**
   *  The aircraft will land automatically at the last waypoint.
   */
    DJIWaypointV2MissionFinishedAutoLanding,

  /**
   *  The aircraft will go back to the first waypoint and hover.
   */
    DJIWaypointV2MissionFinishedGoToFirstWaypoint,

  /**
   *  When the aircraft reaches its final waypoint, it will hover without ending the
   *  mission. The joystick  can still be used to pull the aircraft back along its
   *  previous waypoints. The only way this mission  can end is if stopMission is
   *  called.
   */
    DJIWaypointV2MissionFinishedContinueUntilStop
};


/**
*  Possible flight mode to executes the mission.
*/
enum  DJIWaypointV2MissionGotoFirstWaypointMode:uint8_t {

  /**
   *  Go to the waypoint safely. The aircraft will rise to the same altitude of the
   *  waypoint if the current  altitude is lower then the waypoint altitude. It then
   *  goes to the waypoint coordinate from the current  altitude, and proceeds to the
   *  altitude of the waypoint.
   */
    DJIWaypointV2MissionGotoFirstWaypointModeSafely,

  /**
   *  Go to the waypoint from the current aircraft point to the waypoint directly.
   */
    DJIWaypointV2MissionGotoFirstWaypointModePointToPoint,
};


/**
*  The type of ``DJIWaypointV2Action_DJIWaypointV2AssociateTriggerParam``,
*  Determines the time to execute the trigger associated with another one.
*/
enum DJIWaypointV2TriggerAssociatedTimingType:uint8_t {

  /**
   *  The trigger starts simultaneously with the trigger that is associated.
   */
    DJIWaypointV2TriggerAssociatedTimingTypeSimultaneously = 1,

  /**
   *  The trigger starts after the trigger associated has finished.
   */
    DJIWaypointV2TriggerAssociatedTimingTypeAfterFinised,

  /**
   *  Unkown timing type.
   */
    DJIWaypointV2TriggerAssociatedTimingTypeUnknown = 0xFF,
};


/**
*  The type of ``DJIWaypointV2Action_DJIWaypointV2IntervalTriggerParam``,
*  Determines the interval type of how action repeats.
*/
enum DJIWaypointV2ActionIntervalType:uint8_t {
  /**
   *  The action will be repeated after a particular period of time.
   */
    DJIWaypointV2ActionIntervalTypeTime  = 1,

  /**
   *  The action will be repeated after a particular distance.
   */
    DJIWaypointV2ActionIntervalTypeDistance,



  /**
   *  Unknown action trigger type.
   */
    DJIWaypointV2ActionIntervalTypeUnknown = 0xFF,
};

/**
*  Possible types of action trigger.
*/
enum  DJIWaypointV2ActionTriggerType:uint8_t {

  /**
   *  The action will be trigger when the aircraft reach the waypoint point.
   *  The parameters should be setting by ``DJIWaypointV2Action_DJIWaypointV2ReachPointTriggerParam``.
   */
    DJIWaypointV2ActionTriggerTypeReachPoint = 1,

  /**
   *  The action will be triggered when action associated executes.
   *  The parameters should be defined by ``DJIWaypointV2Action_DJIWaypointV2AssociateTriggerParam``.
   */
    DJIWaypointV2ActionTriggerTypeActionAssociated = 2,

  /**
   *  The action will be triggered when the aircraft flies from one waypoint to the next.
   *  The parameters should be defined by ``DJIWaypointV2Action_DJIWaypointV2TrajectoryTriggerParam``.
   */
    DJIWaypointV2ActionTriggerTypeTrajectory,

  /**
   *  The action will be triggered when the aircraft flies between two waypoints
   *  The parameters should be defined by ``DJIWaypointV2Action_DJIWaypointV2IntervalTriggerParam``.
   */
    DJIWaypointV2ActionTriggerTypeInterval,

    DJIWaypointV2ActionTriggerTypeSampleReachPoint,

  /**
   *  Unknown
   */
    DJIWaypointV2ActionTriggerTypeUnknown = 0xFF
};


/**
*  Possible types of action actuator.
*/
enum  DJIWaypointV2ActionActuatorType:uint8_t {

  /**
   *  The action will be executed by the camera.
   *  The parameters should be defined by ``DJIWaypointV2Action_DJIWaypointV2CameraActuatorParam``.
   */
    DJIWaypointV2ActionActuatorTypeCamera = 1,

  /**
   *  The action will be executed by the gimbal.
   *  The parameters should be defined by ``DJIWaypointV2Action_DJIWaypointV2GimbalActuatorParam``.
   */
    DJIWaypointV2ActionActuatorTypeGimbal,

  /**
   *  The action will executes by control aircraft.
   *  The parameters should be setting by ``DJIWaypointV2Action_DJIWaypointV2CameraActuatorParam``.
   */
    DJIWaypointV2ActionActuatorTypeAircraftControl,

  /**
   *  Unknown actuator type.
   */
    DJIWaypointV2ActionActuatorTypeUnknown = 0xFF
};


/**
*  The type of gimbal actuator operation.
*/
enum  DJIWaypointV2ActionActuatorGimbalOperationType:uint8_t {

  /**
   *  Rotates the gimbal. Only valid when the trigger type is
   *  ``DJIWaypointV2MissionV2_DJIWaypointV2TriggerAssociatedTimingType_ReachPoint``.
   */
    DJIWaypointV2ActionActuatorGimbalOperationTypeRotateGimbal = 1,

  /**
   *  Rotates the gimbal. Only valid when the trigger type is
   *  ``DJIWaypointV2MissionV2_DJIWaypointV2TriggerAssociatedTimingType_Trajectory``.
   */
  // DJIWaypointV2ActionActuatorGimbalOperationTypeAircraftControlGimbal,

  /**
   *  Unknown
   */
    DJIWaypointV2ActionActuatorGimbalOperationTypeUnknown = 0xFF,
};


/**
*  Possible types of camera actuator operation.
*/
enum DJIWaypointV2ActionActuatorCameraOperationType:uint16_t {

  /**
   *  Starts to shoot a photo.
   */
    DJIWaypointV2ActionActuatorCameraOperationTypeTakePhoto = 1,

  /**
   *  Starts to record a video.
   */
    DJIWaypointV2ActionActuatorCameraOperationTypeStartRecordVideo,

  /**
   *  Stops to record a video.
   */
    DJIWaypointV2ActionActuatorCameraOperationTypeStopRecordVideo,

  /**
   *  Starts focus.
   */
    DJIWaypointV2ActionActuatorCameraOperationTypeFocus,

  /**
   *  Starts focal lenth. Only support those support flocal lenth cameras.
   */
    DJIWaypointV2ActionActuatorCameraOperationTypeFocalLength,

  /**
   *  Unknown.
   */
  //  DJIWaypointV2ActionActuatorCameraOperationTypeUnknown = 0xFF,
};


/**
* Possible types of aircraft control actuator operation.
*/
enum  DJIWaypointV2ActionActuatorAircraftControlOperationType:uint8_t {

  /**
   *  Rotates the aircraft's yaw.
   */
    DJIWaypointV2ActionActuatorAircraftControlOperationTypeRotateYaw = 1,

  /**
   *  Keeps the aircraft stop flying or start flying.
   */
    DJIWaypointV2ActionActuatorAircraftControlOperationTypeFlyingControl,

  /**
   *  Unknown
   */
    DJIWaypointV2ActionActuatorAircraftControlOperationTypeUnknown = 0xFF,
};


/**
*  Current waypoint mission executing state.
*/
enum  DJIWaypointV2MissionExecutionState:uint8_t{

  /**
   *  Waypoint mission is initializing, which means the mission has started.
   */
    DJIWaypointV2MissionExecutionStateInitializing,

  /**
   *  Aircraft is currently is going to the first waypoint.
   */
    DJIWaypointV2MissionExecutionStateGoToFirstWaypoint,

  /**
   *  Aircraft is currently moving.
   */
    DJIWaypointV2MissionExecutionStateMoving,

  /**
   *  Aircraft is currently interrupted by user.
   */
    DJIWaypointV2MissionExecutionStateInterrupted,

  /**
   *  The aircraft finishes the waypoint mission and is leaving from the last waypoint.
   */
    DJIWaypointV2MissionExecutionStateFinished,

  /**
   *  The aircraft is going back to the home point. This happens when the
   *  ``DJIMutableWaypointV2Mission_finishedAction`` is set to
   *  ``DJIWaypointV2MissionV2_DJIWaypointV2MissionFinishedAction_GoHome``.
   */
    DJIWaypointV2MissionExecutionStateGoHome,

  /**
   *  The aircraft is landing. This happens when the
   *  ``DJIMutableWaypointV2Mission_finishedAction`` is set to
   *  ``DJIWaypointV2MissionV2_DJIWaypointV2MissionFinishedAction_AutoLanding``.
   */
    DJIWaypointV2MissionExecutionStateLanding,

  /**
   *  Aircraft start to fly back to the first waypoint. This happens when the
   *  ``DJIMutableWaypointV2Mission_finishedAction`` is set to
   *  ``DJIWaypointV2MissionV2_DJIWaypointV2MissionFinishedAction_GoToFirstWaypoint``.
   */
    DJIWaypointV2MissionExecutionStateReturnToFirstWaypoint,

  /**
   *  Unknown execution state.
   */
    DJIWaypointV2MissionExecutionStateUnknown = 0xFF,
};


/**
*  Current waypoint mission action executing event.
*/
enum DJIWaypointV2ActionExecutionEvent:uint8_t{
  /**
   *  Action begin execute event.
   */
    DJIWaypointV2ActionExecutionEventBeginAction,

  /**
   *  Action finished execute event.
   */
    DJIWaypointV2ActionExecutionEventFinishedAction,

  /**
   *  Unknown action event.
   */
    DJIWaypointV2ActionExecutionEventUnknown = 0xFF,
};

/**
*  Waypoint flight path mode.
*/
typedef enum  DJIWaypointV2FlightPathMode:uint8_t {

  /**
   *  In the mission, the aircraft will go to the waypoint along a curve and fly past the waypoint.
   */
    DJIWaypointV2FlightPathModeGoToPointAlongACurve,

  /**
   *  In the mission, the aircraft will go to the waypoint along a curve and stop at the waypoint.
   */
    DJIWaypointV2FlightPathModeGoToPointAlongACurveAndStop,

  /**
   *  In the mission, the aircraft will go to the waypoint along a straight line and stop at the waypoint.
   */
    DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop,

  /**
   *  In the mission, the aircraft will fly from the previous waypoint to the next waypoint along a smooth curve without stopping at this waypoint.
   *  the next in a curved motion,  adhering to the ``DJIWaypointV2_dampingDistance``, which is
   *  set in ``DJIWaypointV2``.
   */
    DJIWaypointV2FlightPathModeCoordinateTurn,

  /**
   *  In the mission, the aircraft will go to the first waypoint along a straight line.
   *  This is only valid for the first waypoint.
   */
    DJIWaypointV2FlightPathModeGoToFirstPointAlongAStraightLine,

  /**
   *  Straight exit the Last waypoint, Only valid for last waypoint.
   */
    DJIWaypointV2FlightPathModeStraightOut,

  /**
   *  Unknown
   */
    DJIWaypointV2FlightPathModeUnknown = 0xFF,
}DJIWaypointV2FlightPathMode;

/**
*  Represents current aircraft's heading mode on current waypoint.
*/
typedef enum  DJIWaypointV2HeadingMode:uint8_t {

  /**
   *  Aircraft's heading will always be in the direction of flight.
   */
    DJIWaypointV2HeadingModeAuto,

  /**
   * Aircraft's heading will be set to the heading when reaching the first waypoint.
   * Before reaching the first waypoint, the aircraft's heading can be controlled by
   * the remote controller. When the aircraft reaches the first waypoint, its
   * heading will be fixed.
   */
    DJIWaypointV2HeadingFixed,

  /**
   *  The aircraft's heading in the mission can be controlled by the remote controller.
   */
    DJIWaypointV2HeadingManual,

  /**
   * In the mission, the aircraft's heading will change dynamically and adapt to the heading set at the next waypoint.
   * See ``DJIWaypointV2_heading`` to preset the heading.
   */
    DJIWaypointV2HeadingWaypointCustom,

  /**
   *  Aircraft's heading will always toward point of interest.
   *  using ``DJIWaypointV2_pointOfInterest`` setting point of interest coordiate and ``DJIWaypointV2_pointOfInterestAltitude``
   *  setting point of interset altitute.
   */
    DJIWaypointV2HeadingTowardPointOfInterest,

  /**
   *  The aircraft's heading rotate simultaneously with its gimbal's yaw.
   */
    DJIWaypointV2HeadingGimbalYawFollow,

  /**
   *  Unknown.
   */
    DJIWaypointV2HeadingUnknown = 0xFF,
}DJIWaypointV2HeadingMode;

/**
*  The direction when the aircraft changes its heading to adapt to the heading at the waypoint.
*/
typedef enum  DJIWaypointV2TurnMode:uint8_t {

  /**
   *  The aircraft's heading rotates clockwise.
   */
    DJIWaypointV2TurnModeClockwise,

  /**
   *  The aircraft's heading rotates counterclockwise.
   */
    DJIWaypointV2TurnModeCounterClockwise,

  /**
   *  Changes the heading of the aircraft by rotating the aircraft anti-clockwise.
   */
    DJIWaypointV2TurnModeUnknown = 0xFF,
}DJIWaypointV2TurnMode;


/**
 *  All the possible state of ``WaypointV2MissionOperator``.
 */
enum DJIWaypointV2MissionState{
  /**
   *  The state of the operator is unknown. It is the initial state when the operator
   *  is just created.
   */
    DJIWaypointV2MissionStateUnWaypointActionActuatorknown = -1,

  /**
   *  The connection OSDK device, remote controller and aircraft is
   *  broken.
   */
    DJIWaypointV2MissionStateDisconnected = 0,

  /**
   *  Raed to execute the mission.
   */
    DJIWaypointV2MissionStateReadyToExecute = 1,

  /**
   *  The execution is started successfully.
   */
    DJIWaypointV2MissionStateExecuting = 2,

  /**
   *  Waypoint mission is paused successfully.
   */
    DJIWaypointV2MissionStateInterrupted = 3,

  /**
   *  Waypoint mission is restarted after interrupted.
   */
    DJIWaypointV2MissionStateResumeAfterInterrupted = 4,

  /**
   *  Waypoint mission is exited.
   */
    DJIWaypointV2MissionStateExitMission = 5,

  /**
   *  Waypoint mission is finished.
   */
    DJIWaypointV2MissionStateFinishedMission = 6,
};

typedef uint8_t  RetCodeType;

typedef uint32_t WaypointV2CommonAck;

/*! Global cruise speed of mission
 *  unit: m/s
 *  range:[0,WayPointV2InitSettings::maxFlightSpeed]
 */
typedef float32_t GlobalCruiseSpeed;

/*! Upload mission raw ack data
 */
typedef struct UploadMissionRawAck
{
  /*! 0: success; other:failed*/
  uint32_t result;

  /*! mission's first waypoint index*/
  uint16_t startIndex;

  /*! mission's final waypoint index*/
  uint16_t endIndex;
}UploadMissionRawAck;

/*! Upload actions raw ack data
 */
typedef struct UploadActionsRawAck
{
  /*! upload result 0: success; other:failed*/
  uint32_t result;

  /*! When upload failed, return the error action id*/
  uint16_t errorActionId;
}UploadActionsRawAck;

/*! Download mission raw request data
 */
typedef struct DownloadMissionRsp
{
  uint16_t startIndex;
  uint16_t endIndex;
}DownloadMissionRsp;

/*! Download mission raw ack data
 */
typedef struct DownloadMissionAck
{
  /*! download result 0: success; other:failed*/
  uint32_t result;

  /*! mission's first waypoint index*/
  uint16_t startIndex;

  /*! mission's first waypoint index*/
  uint16_t endIndex;
}DownloadMissionAck;

/*! Get the  mission global cruise speed raw ack data
 */
typedef struct GetGlobalCruiseVelAck{

  /*! get result 0: success; other:failed*/
  uint32_t result;

  /*!Unit: cm/s*/
  uint16_t globalCruiseVel;
}GetGlobalCruiseVelAck;

/*! Get the remain memory  ack data
 */
typedef struct GetRemainRamAck
{
  uint16_t totalMemory;
  uint16_t remainMemory;
}getRemainRamAck;

/*! Get the mission's start and stop index ack data
 */
typedef struct GetWaypontStartEndIndexAck
{
  uint32_t result;
  uint16_t startIndex;
  uint16_t endIndex;
}GetWaypontStartEndIndexAck;

/*! Mission's state  data
 */
typedef struct MissionStateCommanData
{

  uint16_t curWaypointIndex;
  uint8_t  stateDetail:4;
  uint8_t  state:4;
  uint16_t velocity;
  uint8_t  config;
}MissionStateCommanData;

/*! Mission's state push ack data
 */
typedef struct MissionStatePushAck
{
  uint8_t commonDataVersion = 1;
  uint16_t commonDataLen;
  MissionStateCommanData data;
}MissionStatePushAck;

/*! Mission's event data
 */
typedef union Eventdata
{
  /*ID:0x01*/
  uint8_t interruptReason;

  /*ID:0x02*/
  uint8_t recoverProcess;

  /*ID:0x03*/
  uint8_t finishReason;

  /*ID:0x10*/
  uint16_t waypointIndex;

  /*ID:0x11*/
  struct MissionExecEvent{
    uint8_t currentMissionExecNum;
    uint8_t finishedAllExecNum:1;
    uint8_t reserved:7;
  }MissionExecEvent;

  /*ID:0x12*/
  uint8_t avoidState;

  /*ID:0x20*/
  struct MissionValidityEvent {
    uint8_t misValidityFlag;
    float32_t estimateRunTime;
  }MissionValidityEvent;

  /*ID:0x30*/
  struct ActionExecEvent{
    uint16_t actionId;
    uint8_t preActuatorState;
    uint8_t curActuatorState;
    uint32_t result;
  }ActionExecEvent;
}Eventdata;

/*! Mission's event push ack data
 */
typedef struct MissionEventPushAck
{
  uint8_t event ;
  uint32_t FCTimestamp;
  Eventdata data;
}MissionEventPushAck;

/**
 * @brief Waypoint V2 Mission Initialization Settings Internal
 * User have no need to use it
 */
typedef struct WayPointV2InitSettingsInternal{

  uint16_t version = 0x6500;

  uint8_t  reserved = 0;

  uint32_t missionID;

  uint16_t missTotalLen;

  uint16_t waypointCount = 2;

  uint8_t repeatTimes;

  DJIWaypointV2MissionFinishedAction finishedAction;

  uint16_t maxFlightSpeed;

  uint16_t autoFlightSpeed;

  uint16_t startIndex;

  uint8_t exitMissionOnRCSignalLost;

  DJIWaypointV2MissionGotoFirstWaypointMode gotoFirstWaypointMode;

  float64_t refLati;

  float64_t refLong;

  float32_t refAlti;

}WayPointV2InitSettingsInternal;

typedef struct DownloadInitSettingRawAck
{
  /*! 0: success; other:failed*/
  uint32_t result;
  WayPointV2InitSettingsInternal initSettingsInternal;

}DownloadInitSettingRawAck;

/*! waypoint position relative to WayPointV2InitSettings's reference point
 * unit: m
 */
typedef struct RelativePosition{
  float32_t positionX; /*! X distance to reference point, North is positive*/
  float32_t positionY; /*! Y distance to reference point, East is positive*/
  float32_t positionZ; /*! Z distance to reference point, UP is positive*/
} RelativePosition;

/**
 *  Represents current waypoint's speed config.
 */
typedef struct WaypointV2Config
  {
    /*! 0: set local waypoint's cruise speed,
     *  1: unset global waypoint's cruise speed*/
    uint16_t  useLocalCruiseVel:1;

    /*! 0: set local waypoint's max speed,
     *  1: unset global waypoint's max speed*/
    uint16_t  useLocalMaxVel:1;

    uint16_t  reserved :14;

  }WaypointV2Config;

/**
*  The struct represents a target point in the waypoint mission. For a waypoint
*  mission, a flight route  consists of multiple `WaypointV2` objects.
*/
typedef struct WaypointV2Internal
{
  /*! waypoint position relative to WayPointV2InitSettings's reference point
   * unit: m
   */
  float32_t positionX; /*! X distance to reference point, North is positive*/
  float32_t positionY; /*! Y distance to reference point, East is positive*/
  float32_t positionZ; /*! Z distance to reference point, UP is positive*/

 /**
  *  Waypoint flight path mode.
  */
  DJIWaypointV2FlightPathMode   waypointType;

 /**
  *  Represents current aircraft's heading mode on current waypoint.
  */
  DJIWaypointV2HeadingMode      headingMode;

  /**
   *  Represents current waypoint's speed config.
   */
  WaypointV2Config config;

  uint16_t dampingDistance;

  /**
  *  The heading to which the aircraft will rotate by the time it reaches the
  *  waypoint. The aircraft heading  will gradually change between two waypoints with
  *  different headings if the waypoint  mission's `headingMode` is set  to
  *  `DJIWaypointV2_DJIWaypointV2HeadingMode_WaypointCustom`. A heading has a range of
  *  [-180, 180] degrees, where 0 represents True North.
  */
  float32_t heading;

  /**
  *  Determines whether the aircraft will turn clockwise or anticlockwise when
  *  changing its heading.
  */
  DJIWaypointV2TurnMode turnMode;

  /**
  *  Property is used when ``DJIWaypointV2_headingMode`` is
  *  ``DJIWaypointV2_DJIWaypointV2HeadingMode_TowardPointOfInterest``.
  *  Aircraft will always be heading to point while executing mission. Default is
  *  "kCLLocationCoordinate2DInvalid".
  */
  RelativePosition pointOfInterest;

  /**
  *  While the aircraft is travelling between waypoints, you can offset its speed by
  *  using the throttle joystick on the remote controller. "maxFlightSpeed" is this
  *  offset when the joystick is pushed to maximum deflection. For example, If
  *  maxFlightSpeed is 10 m/s, then pushing the throttle joystick all the way up will
  *  add 10 m/s to the aircraft speed, while pushing down will subtract 10 m/s from
  *  the aircraft speed. If the remote controller stick is not at maximum deflection,
  *  then the offset speed will be interpolated between "[0, maxFlightSpeed]"" with a
  *  resolution of 1000 steps. If the offset speed is negative, then the aircraft
  *  will fly backwards to previous waypoints. When it reaches the first waypoint, it
  *  will then hover in place until a positive speed is applied. "maxFlightSpeed" has
  *  a range of [2,15] m/s.
   *  unit:cm/s
  */
  uint16_t maxFlightSpeed;

  /**
  *  The base automatic speed of the aircraft as it moves between waypoints with
  *  range [-15, 15] m/s. The aircraft's actual speed is a combination of the base
  *  automatic speed, and the speed control given by the throttle joystick on the
  *  remote controller. If "autoFlightSpeed >0": Actual speed is "autoFlightSpeed" +
  *  Joystick Speed (with combined max of "maxFlightSpeed") If "autoFlightSpeed =0":
  *  Actual speed is controlled only by the remote controller joystick. If
  *  "autoFlightSpeed <0" and the aircraft is at the first waypoint, the aircraft
  *  will hover in place until the speed is made positive by the remote controller
  *  joystick. In flight controller firmware 3.2.10.0 or above, different speeds
  *  between individual waypoints can also be set in waypoint objects which will
  *  overwrite "autoFlightSpeed".
   * unit :cm/s
  */
  uint16_t autoFlightSpeed;
}WaypointV2Internal;

/**
*  The struct represents a target point in the waypoint mission. For a waypoint
*  mission, a flight route  consists of multiple `WaypointV2` objects.
*/
typedef struct WaypointV2
{
  /*! waypoint position relative to WayPointV2InitSettings's reference point
   * unit: m
   */
  float64_t longitude;
  float64_t latitude;
  float32_t relativeHeight; /*! relative to takeoff height*/

  /**
   *  Waypoint flight path mode.
   */
  DJIWaypointV2FlightPathMode   waypointType;

  /**
   *  Represents current aircraft's heading mode on current waypoint.
   */
  DJIWaypointV2HeadingMode      headingMode;

  /**
   *  Represents current waypoint's speed config.
   */
  WaypointV2Config config;

  uint16_t dampingDistance;

  /**
  *  The heading to which the aircraft will rotate by the time it reaches the
  *  waypoint. The aircraft heading  will gradually change between two waypoints with
  *  different headings if the waypoint  mission's `headingMode` is set  to
  *  `DJIWaypointV2_DJIWaypointV2HeadingMode_WaypointCustom`. A heading has a range of
  *  [-180, 180] degrees, where 0 represents True North.
  */
  float32_t heading;

  /**
  *  Determines whether the aircraft will turn clockwise or anticlockwise when
  *  changing its heading.
  */
  DJIWaypointV2TurnMode turnMode;

  /**
  *  Property is used when ``DJIWaypointV2_headingMode`` is
  *  ``DJIWaypointV2_DJIWaypointV2HeadingMode_TowardPointOfInterest``.
  *  Aircraft will always be heading to point while executing mission. Default is
  *  "kCLLocationCoordinate2DInvalid".
  */
  RelativePosition pointOfInterest;

  /**
  *  While the aircraft is travelling between waypoints, you can offset its speed by
  *  using the throttle joystick on the remote controller. "maxFlightSpeed" is this
  *  offset when the joystick is pushed to maximum deflection. For example, If
  *  maxFlightSpeed is 10 m/s, then pushing the throttle joystick all the way up will
  *  add 10 m/s to the aircraft speed, while pushing down will subtract 10 m/s from
  *  the aircraft speed. If the remote controller stick is not at maximum deflection,
  *  then the offset speed will be interpolated between "[0, maxFlightSpeed]"" with a
  *  resolution of 1000 steps. If the offset speed is negative, then the aircraft
  *  will fly backwards to previous waypoints. When it reaches the first waypoint, it
  *  will then hover in place until a positive speed is applied. "maxFlightSpeed" has
  *  a range of [2,15] m/s.
  */
  float32_t maxFlightSpeed;

  /**
  *  The base automatic speed of the aircraft as it moves between waypoints with
  *  range [-15, 15] m/s. The aircraft's actual speed is a combination of the base
  *  automatic speed, and the speed control given by the throttle joystick on the
  *  remote controller. If "autoFlightSpeed >0": Actual speed is "autoFlightSpeed" +
  *  Joystick Speed (with combined max of "maxFlightSpeed") If "autoFlightSpeed =0":
  *  Actual speed is controlled only by the remote controller joystick. If
  *  "autoFlightSpeed <0" and the aircraft is at the first waypoint, the aircraft
  *  will hover in place until the speed is made positive by the remote controller
  *  joystick. In flight controller firmware 3.2.10.0 or above, different speeds
  *  between individual waypoints can also be set in waypoint objects which will
  *  overwrite "autoFlightSpeed".
  */
  float32_t autoFlightSpeed;
}WaypointV2;

/**
 * @brief Waypoint V2 Mission Initialization settings
 * @details This is one of the few structs in the OSDK codebase that
 * is used in both a sending and a receiving API.
 */
typedef struct WayPointV2InitSettings{

  /**
   *  The Mission ID. Use to distinguish　different mission
   */
  uint32_t missionID;

  /**
   *  The Mission waypoint total length, could not exceed 65535
   */
  uint16_t missTotalLen;

  /**
   *  Mission execution can be repeated more than once. A value of 0 means the mission
   *  only executes once, and does not repeat. A value of 1 means the mission will
   *  execute a total of two times.
   */
  uint8_t repeatTimes;

  /**
   *  Action the aircraft will take when the waypoint mission is complete.
   */
  DJIWaypointV2MissionFinishedAction finishedAction;

  /**
   *  While the aircraft is travelling between waypoints, you can offset its speed by
   *  using the throttle joystick  on the remote controller. `maxFlightSpeed` is this
   *  offset when the joystick is pushed  to maximum deflection. For example, If
   *  maxFlightSpeed is 10 m/s, then pushing the throttle joystick all the  way up
   *  will add 10 m/s to the aircraft speed, while pushing down will subtract 10 m/s
   *  from the aircraft  speed. If the remote controller stick is not at maximum
   *  deflection, then the offset speed will be  interpolated between [0,
   *  `maxFlightSpeed`] with a resolution of 1000 steps. If the  offset speed is
   *  negative, then the aircraft will fly backwards to previous waypoints. When it
   *  reaches the  first waypoint, it will then hover in place until a positive speed
   *  is  applied. `maxFlightSpeed` has a range of [2,15] m/s.
   *  unit: m/s
   */
  float32_t maxFlightSpeed;

  /**
   *  The base automatic speed of the aircraft as it moves between waypoints with
   *  range [-15, 15] m/s. The  aircraft's actual speed is a combination of the base
   *  automatic speed, and the speed control given by  the throttle joystick on the
   *  remote controller. If `autoFlightSpeed` >0: Actual  speed is `autoFlightSpeed` +
   *  Joystick Speed (with combined max  of `maxFlightSpeed`) If `autoFlightSpeed` =0:
   *  Actual speed is  controlled only by the remote controller joystick. If
   *  `autoFlightSpeed` <0 and the  aircraft is at the first waypoint, the aircraft
   *  will hover in place until the speed is made positive by  the remote controller
   *  joystick.
   *  unit: m/s
   */
  float32_t autoFlightSpeed;

  /**
   *  Determines whether the mission should stop when connection between the  aircraft
   *  and remote controller is lost. Default is `NO`.
   */
  uint8_t exitMissionOnRCSignalLost;

  /**
   *  Defines how the aircraft will go to the first waypoint from its current
   *  position. Default  is ``DJIWaypointV2MissionV2_DJIWaypointV2MissionGotoWaypointMode_Safely``.
   */
  DJIWaypointV2MissionGotoFirstWaypointMode gotoFirstWaypointMode;

  std::vector<WaypointV2> mission;

}WayPointV2InitSettings;

typedef struct  FCGroundStationDataPush
{
  // If (use_rtk_flag && in_rtk_mode)=1 , longitude is rtk lon, otherwise it's (gps_lon - gps2rtk_lon_offset).
  // (The use_rtk_flag and in_rtk_mode are the members in the later section. )
  // Unit: rad
  double longitude;
  // If (use_rtk_flag && in_rtk_mode)=1 , latitude is rtk lat, otherwise it's (gps_lat - gps2rtk_lat_offset).
  // Unit: rad
  double latitude;
  // If (use_rtk_flag && in_rtk_mode && set_rtk_takeoff_alti_flag)=1 , relative_height is rtk_type, otherwise it's press_type.
  // Unit: m
  float relative_height;
  // Unit: 0.1 degree
  int16_t heading;
  // use rtk or not 0: rtk switch is off or rtk device is not connect， 1: us rtk position。
  uint8_t use_rtk_flag;
  // 0: rtk not healthy, 1: rtk 3d healthy
  uint8_t in_rtk_mode;
  // utc time's seconds
  uint32_t sec;
  //  utc time's nano seconds
  uint32_t nano_sec;
  // home type。0=NA，1=gps_type，2=rtk_type
  uint8_t home_type                :2;
  // use rtk height：0=no，1=yes
  uint8_t set_rtk_takeoff_alti_flag:1;
  // distance to home type。0=NA，1=gps_type，2=rtk_type
  uint8_t d2h_type                 :2;
  // reserve
  uint8_t resv                     :3;
  // RTK or GPS's svn
  uint8_t rtk_or_gps_svn;
  // Unit: rad
  double home_lon;
  // Unit: rad
  double home_lat;
  // take off altitude。
  // Unit: m
  float takeoff_alti;
  // aircraft's distance to home.
  // Unit: m
  float d2h;
  float abs_height;
  float gps_height;
} FCGroundStationDataPush;
#pragma pack()


// clang-format on
} // OSDK

} // DJI

#endif // ONBOARDSDK_DJI_MISSION_TYPE_H
