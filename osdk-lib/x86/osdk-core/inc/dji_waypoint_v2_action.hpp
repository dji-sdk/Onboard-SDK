//
// Created by dji on 20/03/20.
//

#ifndef DJIOSDK_CORE_DJI_WAYPOINT_V2_ACTION_HPP
#define DJIOSDK_CORE_DJI_WAYPOINT_V2_ACTION_HPP

#include "dji_mission_type.hpp"
namespace DJI {
  namespace OSDK {
#pragma pack(1)
// TODO 目前不知道这个结构提的定义,仅仅为了编译通过
    typedef struct CGPoint {
      uint16_t x;
      uint16_t y;
    } CGPoint;

// TODO 目前不知道这个结构提的定义,仅仅为了编译通过
    typedef struct DJIGimbalRotation {
      int16_t x;
      int16_t y;
      int16_t z;
      uint8_t rollCmdIgnore:1;
      uint8_t pitchCmdIgnore:1;
      uint8_t yawCmdIgnore:1;
      uint8_t absYawModeRef:1;
      uint8_t reserved:4;
      uint8_t duationTime;
    } DJIGimbalRotation;

/**
 *  This class defines
 * ``DJIWaypointV2MissionV2_DJIWaypointV2ActionTriggerType_ReachPoint``.
 *  It describes an action will be triggered when the aircraft reach the certain
 * waypoint.
 */
    typedef struct DJIWaypointV2ReachPointTriggerParam {
      /**
       *  It determines the index of the waypoint at which the action will be
       * triggered.
       */
      uint16_t startIndex;

      uint16_t endIndex;

      uint16_t intervalWPNum;

      /**
       *  It determines the waypoint count till the action triggered stops.
       */
      uint16_t waypointCountToTerminate;

    } DJIWaypointV2ReachPointTriggerParam;

    typedef struct DJIWaypointV2SampleReachPointTriggerParam {
      /**
       *  It determines the index of the waypoint at which the action will be
       * triggered.
       */
      uint16_t Index;

      uint16_t terminateNum;

    } DJIWaypointV2SampleReachPointTriggerParam;


/**
 *  This class defines
 * ``DJIWaypointV2MissionV2_DJIWaypointV2ActionTriggerType_ActionAssociated``.
 */
    typedef struct DJIWaypointV2AssociateTriggerParam {
      /**
       *  The type of assciate trigger.
       */
      DJIWaypointV2TriggerAssociatedTimingType actionAssociatedType;

      /**
       *  Waiting time in seconds after ActionTrigger starts.
       */
      uint8_t waitingTime;

      /**
       *  Associated action ID.
       */
      uint16_t actionIdAssociated;
    } DJIWaypointV2AssociateTriggerParam;

/**
 *  This class represents a trajectory trigger action when should be trigger.
 */
    typedef struct DJIWaypointV2TrajectoryTriggerParam {
      /**
       *  It determines the index of the waypoint at which the trigger starts.
       */
      uint16_t startIndex;

      /**
       *  It determines the waypoint when the trigger stops.
       */
      uint16_t endIndex;
    } DJIWaypointV2TrajectoryTriggerParam;

/**
 *  This class defines
 * ``DJIWaypointV2MissionV2_DJIWaypointV2ActionTriggerType_Trajectory``.
 */
    typedef struct DJIWaypointV2IntervalTriggerParam {
      /**
       *  It determines the index of the waypoint at which the trigger starts.
       */
      uint16_t startIndex;


      /*TODO 顺序以及含义变了*/

      /**
       *  If the
       * ``DJIWaypointV2Action_DJIWaypointV2IntervalTriggerParam_actionIntervalType``
       * is ``DJIWaypointV2MissionV2_DJIWaypointV2TriggerAssociatedTimingType_Time``
       *  The time interval in seconds when two action are executed as the aircraft
       * moves
       *  from the current waypoint to the next waypoint.
       *  If the
       * ``DJIWaypointV2Action_DJIWaypointV2IntervalTriggerParam_actionIntervalType``
       * is
       * ``DJIWaypointV2MissionV2_DJIWaypointV2TriggerAssociatedTimingType_Distance``
       *  The distance interval in meters when two action are executed as the
       * aircraft
       * moves
       *  from the current waypoint to the next waypoint.
        */
      uint16_t interval;
      /**
       *  The type of interval trigger.
       *  See ``DJIWaypointV2MissionV2_DJIWaypointV2ActionIntervalType``.
       */
      DJIWaypointV2ActionIntervalType actionIntervalType;

    } DJIWaypointV2IntervalTriggerParam;

/**
 *  This class defines a camera focus operation for
 * ``DJIWaypointV2Action_DJIWaypointV2CameraActuatorParam``.
 */
    typedef struct DJIWaypointV2CameraFocusParam {
      /**
       *  The lens focus target point. When the focus mode is auto, the target point
       *  is the focal point. When the focus mode is manual, the target point is the
       * zoom
       *  out area if the focus assistant is enabled for the manual mode.
       *  The range for x and y is from 0.0 to 1.0. The point [0.0, 0.0] represents
       * the top-left angle of the screen.
       */
      CGPoint focusTarget;

      uint8_t retryTimes = 1;

      uint8_t focusDelayTime = 0;

    } DJIWaypointV2CameraFocusParam;

/**
 *  This class defines a camera focal length operation for
 * ``DJIWaypointV2Action_DJIWaypointV2CameraActuatorParam``.
 */
    typedef struct DJIWaypointV2CameraFocalLengthParam {
      /**
       *  Focal length of zoom lens. Valid range is
       * [``DJICamera_DJICameraOpticalZoomSpec_minFocalLength``,
       * ``DJICamera_DJICameraOpticalZoomSpec_minFocalLength``]
       *  and must be a multiple of
       * ``DJICamera_DJICameraOpticalZoomSpec_focalLengthStep``.
       *  Only support by those camera
       * ``DJICamera_CameraSettings_isOpticalZoomSupported`` return ``TRUE``.
       */
      uint16_t focalLength;

      uint8_t retryTimes = 1;

    } DJIWaypointV2CameraFocalLengthParam;

/**
 *  This class defines how the aircraft rotates on the yaw axis.
 */
    typedef struct DJIWaypointV2AircraftControlRotateHeadingParam {
      /**
       *  Determines the aircraft rotate heading relative.
       *  if ``TRUE``, when the aircraft is rotating, relative to the current angle.
       */
      uint8_t isRelative:1;

      uint8_t reserved:8;

      /**
       *  Determines the direction how aircraft changes its heading.
       */
      float32_t yaw;

    } DJIWaypointV2AircraftControlRotateHeadingParam;

/**
 *  This class defines if the aircraft starts or stops the flight.
 */
    typedef struct DJIWaypointV2AircraftControlFlyingParam {
      /**
       *  Determines the aircraft start flying or stop flying.
       *  ``TRUE`` for the aircraft to start flying.
       */
      uint8_t  isStartFlying:1;
      uint8_t  reserved:7;
    } DJIWaypointV2AircraftControlFlyingParam;

#pragma pack()

typedef struct DJIWaypointV2Trigger
{
  DJIWaypointV2Trigger(DJIWaypointV2ActionTriggerType type, void *param){actionTriggerType = type; actionTriggerParam = param;}
  DJIWaypointV2ActionTriggerType actionTriggerType;
  void * actionTriggerParam;
}DJIWaypointV2Trigger;

/**
*  This class defines the parameters for
* ``DJIWaypointV2Action_DJIWaypointV2Actuator``.
*  This determines how the camera will be performed when a waypoint mission is
* executing.
*/
typedef struct DJIWaypointV2CameraActuatorParam {


  DJIWaypointV2CameraActuatorParam(DJIWaypointV2ActionActuatorCameraOperationType type , void *param){operationType =type; cameraActuatorParam =param;};

  /**
   *  The operation type of camera actuator.
   *  See
   * ``DJIWaypointV2MissionV2_DJIWaypointV2ActionActuatorCameraOperationType``.
   */
  DJIWaypointV2ActionActuatorCameraOperationType operationType;

  void *cameraActuatorParam;

//  /**
//   *  The parameters for camera focus operation. It is valid only when
//   * ``DJIWaypointV2Action_DJIWaypointV2CameraActuatorParam_operationType``
//   *  is
//   * ``DJIWaypointV2MissionV2_DJIWaypointV2ActionActuatorCameraOperationType_Focus``
//   */
//  DJIWaypointV2CameraFocusParam *focusParam;
//
//  /**
//   *  The parameters for camera focus length operation. It is valid only when
//   * ``DJIWaypointV2Action_DJIWaypointV2CameraActuatorParam_operationType``
//   *  is
//   * ``DJIWaypointV2MissionV2_DJIWaypointV2ActionActuatorCameraOperationType_FocalLength``
//   */
//  DJIWaypointV2CameraFocalLengthParam *zoomParam;
}DJIWaypointV2CameraActuatorParam;

/**
*  This class defines the parameters for
* ``DJIWaypointV2Action_DJIWaypointV2Actuator``. It determines how the
*  gimbal actuator will be performed when a waypoint mission is executed.
*/
typedef struct DJIWaypointV2GimbalActuatorParam {
  /**
   *  The operation type of gimbal actuator.
   *  See
   * ``DJIWaypointV2MissionV2_DJIWaypointV2ActionActuatorGimbalOperationType``.
   */

  DJIWaypointV2ActionActuatorGimbalOperationType operationType;

  /**
   *  The rotation of gimbal actuator.
   */
  DJIGimbalRotation *rotation;
}DJIWaypointV2GimbalActuatorParam;

/**
*  This class defines the parameters for
* ``DJIWaypointV2Action_DJIWaypointV2Actuator``. It determines how the
*  aircraft control actuator will be performed when a waypoint mission is
* executed.
*/
typedef struct DJIWaypointV2AircraftControlParam {
  /**
   *  The operation type of aircraft control actuator.
   *  See
   * ``DJIWaypointV2MissionV2_DJIWaypointV2ActionActuatorAircraftControlOperationType``.
   */

  DJIWaypointV2ActionActuatorAircraftControlOperationType operationType;

  void *aircraftControlParam;

//  /**
//   *  The parameter for rotating the aircraft's heading. It's valid only when
//   * ``DJIWaypointV2Action_DJIWaypointV2AircraftControlParam_operationType``
//   *  is
//   * ``DJIWaypointV2MissionV2_DJIWaypointV2ActionActuatorAircraftControlOperationType_RotateYaw``
//   */
//
//  DJIWaypointV2AircraftControlRotateHeadingParam *yawRotatingParam;
//
//  /**
//   *  The parameters to control flying behavior.  It's valid only when
//   * ``DJIWaypointV2Action_DJIWaypointV2AircraftControlParam_operationType``
//   *  is
//   * ``DJIWaypointV2MissionV2_DJIWaypointV2ActionActuatorAircraftControlOperationType_FlyingControl``
//   */
//
//  DJIWaypointV2AircraftControlFlyingParam *flyControlParam;

}DJIWaypointV2AircraftControlParam;

/**
*  This class defines an actuator for ``DJIWaypointV2Action``. It determines
* how the
*  action is performed when a waypoint mission is executed.
*/
typedef struct DJIWaypointV2Actuator {

  DJIWaypointV2Actuator(DJIWaypointV2ActionActuatorType type, uint8_t index, void *param){
    actuatorType = type;
    actuatorIndex = index;
    actuatorParam = param;
  };

  /**
   *  The type of Actuator.
   */
  DJIWaypointV2ActionActuatorType actuatorType;

  /**
   *  The index of actuator. It is valid when the diagnostics is related
   *  to camera or gimbal and the connected product has multiple gimbals and
   * cameras.
   */
  uint8_t actuatorIndex;

  void *actuatorParam;


//  /**
//   *  The camera actuator param, It is valid only when the
//   * ``DJIWaypointV2Action_DJIWaypointV2Actuator_type``
//   *  is ``DJIWaypointV2MissionV2_DJIWaypointV2ActionActuatorType_Camera``.
//   */
//  DJIWaypointV2CameraActuatorParam *cameraActuatorParam;
//
//  /**
//   *  Parameters for gimbal actuator. It is valid only when the
//   * ``DJIWaypointV2Action_DJIWaypointV2Actuator_type``
//   *  is ``DJIWaypointV2MissionV2_DJIWaypointV2ActionActuatorType_Gimbal``.
//   */
//  DJIWaypointV2GimbalActuatorParam *gimbalActuatorParam;
//
//  /**
//   *  Parameters for aircraft control actuator. It is valid only when the
//   * ``DJIWaypointV2Action_DJIWaypointV2Actuator_type``
//   *  is
//   * ``DJIWaypointV2MissionV2_DJIWaypointV2ActionActuatorType_AircraftControl``.
//   */
//  DJIWaypointV2AircraftControlParam *aircraftControlActuatorParam;
}DJIWaypointV2Actuator;

/**
*  This class represents an action for ``DJIWaypointV2Mission``. It
* determines how
*  action is performed when a waypoint mission is executed.
*/
class DJIWaypointV2Action {

public:
  DJIWaypointV2Action() = delete;

  DJIWaypointV2Action(uint32_t id,const   DJIWaypointV2Trigger &trigger,const   DJIWaypointV2Actuator &actuator);

  ~DJIWaypointV2Action();

public:
  /**
   *  The ID of Action.
   */
  uint16_t actionId;

  /**
   *  The trigger of action.
   */
  const  DJIWaypointV2Trigger *trigger;

  /**
   *  The actuator of action.
   */
  const  DJIWaypointV2Actuator *actuator;
};
  }
}
#endif //DJIOSDK_CORE_DJI_WAYPOINT_V2_ACTION_HPP
