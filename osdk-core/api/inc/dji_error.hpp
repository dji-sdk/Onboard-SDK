/** @file dji_error.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief All DJI OSDK OpenProtocol ACK Error Codes
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

#ifndef DJI_ERROR_H
#define DJI_ERROR_H

#ifdef STM32
#include <stdint.h>
#else
#include <cstdint>
#endif
#include <map>
#include "osdk_typedef.h"

namespace DJI {
namespace OSDK {

/*! @class ErrorCode contains all the acknowledgments sent by the aircraft
 * @details Each component of the SDK has its own subclass defined within the
 * ErrorCode class.
 */
class ErrorCode {
 public:
  /*! @brief Module ID type used in OnboardSDK Unified error
  */

  typedef uint8_t ModuleIDType;

  /*! @brief Function ID type used in OnboardSDK Unified error
   */
  typedef uint8_t FunctionIDType;

  /*! @brief RawRetCode ID type used in OnboardSDK Unified error
   */
  typedef uint32_t RawRetCodeType;

  /*! @brief Function max count of each module
   */
  static const uint8_t functionMaxCnt = 20;
  /*! @brief Module ID used in OnboardSDK Unified error
   */
  enum ModuleID {
    SysModule = 0,
    RESERVE_1 = 1,
    RESERVE_2 = 2,
    RESERVE_3 = 3,
    RESERVE_4 = 4,
    RESERVE_5 = 5,
    RESERVE_6 = 6,
    RESERVE_7 = 7,
    RESERVE_8 = 8,
    RESERVE_9 = 9,
    RESERVE_10 = 10,
    FCModule = 11,
    GimbalModule = 12,
    CameraModule = 13,
    PSDKModule = 14,
    RCModule = 15,
    BatteryModule = 16,
    MissionV2Module = 17,
    ModuleMaxCnt,
  };

  /*! @brief Function ID used with SystemModule in error codes
   */
  enum SystemFunctionID {
    SystemCommon = 0,
  };
  /*! @brief Function ID used with FCModule in error codes
   */
  enum FCFunctionID {
    FCControlTask       = 0,
    FCSubscribe         = 1,
   // FCWaypointMissionV1 = 2,
   // FCWaypointMissionV2 = 3,
    FCParameterTable    = 4,
    FCSetHomeLocation   = 5,
    FCAvoidObstacle     = 6,
    FCEmergencyBrake    = 7,
  //  FCHotpointMission   = 8,
  };

  /*! @brief Function ID used with CameraModule in error codes
   */
  enum CameraFunctionID {
    CameraCommon = 0,
  };

  /*! @brief Function ID used with GimbalModule in error codes
   */
  enum GimbalFunctionID {
    GimbalCommon = 0,
  };

  /*! @brief Function ID used with PSDKModule in error codes
   */
  enum PSDKFunctionID {
    PSDKCommon = 0,
  };

  enum MissionV2FunctionId
  {
    MissionV2Common = 0,
  };

  enum SYSTEM_ERROR_RAW_CODE {
    Success              = 0x00000000, /*!< Execute successfully */
    AllocMemoryFailed    = 0x00000001, /*!< Out of memory */
    ReqNotSupported      = 0x00000002, /*!< Request not supported */
    Timeout              = 0x00000003, /*!< Execute timeout */
    UnpackDataMismatch   = 0x00000004, /*!< Pack unpack failed */
    InstInitParamInvalid = 0x00000005, /*!< Instance init parameter invalid */
    UserCallbackInvalid  = 0x00000006, /*!< User Callback is a invalid value */
    UndefinedError       = 0xFFFFFFFF, /*!< Undefined error */
  };

  /*! @brief Unified error type
   */
  typedef int64_t ErrorCodeType;

  /*! @brief Releated messages about error codes
   */
  typedef struct ErrorCodeMsg {
    ErrorCodeMsg(const char* moduleMsg, const char* errorMsg,
                 const char* solutionMsg)
        : moduleMsg(moduleMsg), errorMsg(errorMsg), solutionMsg(solutionMsg){};
    const char* moduleMsg;
    const char* errorMsg;
    const char* solutionMsg;
  } ErrorCodeMsg;

  /*! @brief Map container type of errCode ID and msg
   */
  typedef std::map<const RawRetCodeType, ErrorCodeMsg> ErrorCodeMapType;

  typedef struct FunctionDataType
  {
    const char* FunctionName;
    const ErrorCodeMapType (*getMap)();
  } FunctionDataType;

  typedef struct ModuleDataType
  {
    const char* ModuleName;
    const FunctionDataType *data;
  } ModuleDataType;

  /*! @brief Build error code
   *  @param moduleID module ID used in errCode ref to
   * DJI::OSDK::ErrorCode::ModuleID
   *  @param functionID function ID used in errCode ref to
   * DJI::OSDK::ErrorCode::xxxxFunctionID
   *  @param rawRetCode raw return code from the ack data
   *  @return Unified error type
   */
  static constexpr ErrorCodeType getErrorCode(ModuleIDType moduleID,
                                          FunctionIDType functionID,
                                          RawRetCodeType rawRetCode) {
    return (!rawRetCode) ? (
      ((ErrorCodeType) ErrorCode::SysModule << moduleIDLeftMove)
        | ((ErrorCodeType) ErrorCode::SystemCommon << functionIDLeftMove) |
        (ErrorCodeType) 0x00000000) :
           (((ErrorCodeType) moduleID << moduleIDLeftMove) |
             ((ErrorCodeType) functionID << functionIDLeftMove) |
             (ErrorCodeType) rawRetCode);
  }

  /*! @brief Get the module ID from errCode
   *  @param errCode Unified error type
   *  @return Module ID ref to DJI::OSDK::ErrorCode::ModuleID
   */
  static ErrorCode::ModuleIDType getModuleID(ErrorCodeType errCode);

  /*! @brief Get the module name from errCode
   *  @param errCode Unified error type
   *  @return Module name
   */
  static const char* getModuleName(ErrorCodeType errCode);

  /*! @brief Get the function name from errCode
   *  @param errCode Unified error type
   *  @return Function name
   */
  static ErrorCode::FunctionIDType getFunctionID(ErrorCodeType errCode);

  /*! @brief Get the raw reture code from errCode
   *  @param errCode Unified error type
   *  @return Function name
   */
  static ErrorCode::RawRetCodeType getRawRetCode(ErrorCodeType errCode);

  static ErrorCode::ErrorCodeType getLinkerErrorCode(E_OsdkStat cb_type);

  /*! @brief Get error code messages from errCode
   *  @param errCode Unified error type
   *  @return Releated error code messages
   */
  static ErrorCodeMsg getErrorCodeMsg(int64_t errCode);

  /*! @brief Print error code messages to console
   *  @param errCode Unified error type
   */
  static void printErrorCodeMsg(int64_t errCode);

  class FlightControllerErr
  {
  public:
    class ParamReadWriteErr
    {
    public:
      static const ErrorCodeType Fail;
      static const ErrorCodeType InvalidParameter;
    };

    class SetHomeLocationErr
    {
    public:
      static const ErrorCodeType Fail;
    };

  };
  /*! @brief camera api error code
   */
  class CameraCommonErr {
   public:
    static const ErrorCodeType InvalidCMD;
    static const ErrorCodeType Timeout;
    static const ErrorCodeType OutOfMemory;
    static const ErrorCodeType InvalidParam;
    static const ErrorCodeType InvalidState;
    static const ErrorCodeType TimeNotSync;
    static const ErrorCodeType ParamSetFailed;
    static const ErrorCodeType ParamGetFailed;
    static const ErrorCodeType SDCardMISSING;
    static const ErrorCodeType SDCardFull;
    static const ErrorCodeType SDCardError;
    static const ErrorCodeType SensorError;
    static const ErrorCodeType SystemError;
    static const ErrorCodeType ParamLenTooLong;
    static const ErrorCodeType ModuleInactivated;
    static const ErrorCodeType FWSeqNumNotInOrder;
    static const ErrorCodeType FWCheckErr;
    static const ErrorCodeType FlashWriteError;
    static const ErrorCodeType FWInvalidType;
    static const ErrorCodeType RCDisconnect;
    static const ErrorCodeType HardwareErr;
    static const ErrorCodeType UAVDisconnect;
    static const ErrorCodeType UpgradeErrorNow;
    static const ErrorCodeType UndefineError;
  };

  /*! @brief gimbal api error code
   */
  class GimbalCommonErr {
   public:
    static const ErrorCodeType InvalidCMD;
    static const ErrorCodeType Timeout;
    static const ErrorCodeType OutOfMemory;
    static const ErrorCodeType InvalidParam;
    static const ErrorCodeType InvalidState;
    static const ErrorCodeType TimeNotSync;
    static const ErrorCodeType ParamSetFailed;
    static const ErrorCodeType ParamGetFailed;
    static const ErrorCodeType SDCardMISSING;
    static const ErrorCodeType SDCardFull;
    static const ErrorCodeType SDCardError;
    static const ErrorCodeType SensorError;
    static const ErrorCodeType SystemError;
    static const ErrorCodeType ParamLenTooLong;
    static const ErrorCodeType ModuleInactivated;
    static const ErrorCodeType FWSeqNumNotInOrder;
    static const ErrorCodeType FWCheckErr;
    static const ErrorCodeType FlashWriteError;
    static const ErrorCodeType FWInvalidType;
    static const ErrorCodeType RCDisconnect;
    static const ErrorCodeType HardwareErr;
    static const ErrorCodeType UAVDisconnect;
    static const ErrorCodeType UpgradeErrorNow;
    static const ErrorCodeType UndefineError;
  };

  /*! @brief camera api error code
 */
  class PSDKCommonErr {
   public:
    static const ErrorCodeType InvalidCMD;
    static const ErrorCodeType Timeout;
    static const ErrorCodeType OutOfMemory;
    static const ErrorCodeType InvalidParam;
    static const ErrorCodeType InvalidState;
    static const ErrorCodeType TimeNotSync;
    static const ErrorCodeType ParamSetFailed;
    static const ErrorCodeType ParamGetFailed;
    static const ErrorCodeType SDCardMISSING;
    static const ErrorCodeType SDCardFull;
    static const ErrorCodeType SDCardError;
    static const ErrorCodeType SensorError;
    static const ErrorCodeType SystemError;
    static const ErrorCodeType ParamLenTooLong;
    static const ErrorCodeType ModuleInactivated;
    static const ErrorCodeType FWSeqNumNotInOrder;
    static const ErrorCodeType FWCheckErr;
    static const ErrorCodeType FlashWriteError;
    static const ErrorCodeType FWInvalidType;
    static const ErrorCodeType RCDisconnect;
    static const ErrorCodeType HardwareErr;
    static const ErrorCodeType UAVDisconnect;
    static const ErrorCodeType UpgradeErrorNow;
    static const ErrorCodeType UndefineError;
  };

  /*! @brief system releated error code
   */
  class SysCommonErr {
   public:
    static const ErrorCodeType Success;
    static const ErrorCodeType AllocMemoryFailed;
    static const ErrorCodeType ReqNotSupported;
    static const ErrorCodeType ReqTimeout;
    static const ErrorCodeType UnpackDataMismatch;
    static const ErrorCodeType InstInitParamInvalid;
    static const ErrorCodeType UserCallbackInvalid;
    static const ErrorCodeType UndefinedError;
  };

  class WaypointV2MissionErr{
  public:

    static const ErrorCodeType TRAJ_RESV;
    static const ErrorCodeType TRAJ_INIT_WP_NUM_TOO_MANY ;
    static const ErrorCodeType TRAJ_INIT_WP_NUM_TOO_FEW   ;
    static const ErrorCodeType TRAJ_INIT_INVALID_END_INDEX      ;
    static const ErrorCodeType TRAJ_UPLOAD_START_ID_GT_END_ID ;
    static const ErrorCodeType TRAJ_UPLOAD_END_ID_GT_TOTAL_NUM ;
    static const ErrorCodeType TRAJ_DOWNLOAD_WPS_NOT_IN_STORED_RAGNE ;
    static const ErrorCodeType TRAJ_CUR_POS_IS_FAR_AWAY_FROM_FIRST_WP ;
    static const ErrorCodeType TRAJ_ADJ_WPS_TOO_CLOSE  ;
    static const ErrorCodeType TRAJ_ADJ_WPS_TOO_FAR   ;
    static const ErrorCodeType TRAJ_UPLOAD_MAX_VEL_GT_GLOBAL  ;
    static const ErrorCodeType TRAJ_UPLOAD_LOCAL_CRUISE_VEL_GT_LOCAL_MAX ;
    static const ErrorCodeType TRAJ_UPLOAD_LOCAL_CRUISE_VEL_GT_GLOBAL_MAX ;
    static const ErrorCodeType TRAJ_INIT_INVALID_GLOBAL_MAX_VEL   ;
    static const ErrorCodeType TRAJ_GLOBAL_CRUISE_VEL_GT_MAX_VEL  ;
    static const ErrorCodeType TRAJ_INIT_INVALID_GOTO_FIRST_FLAG  ;
    static const ErrorCodeType TRAJ_INIT_INVALID_FINISHED_ACTION  ;
    static const ErrorCodeType TRAJ_INIT_INVALID_RC_LOST_ACTION;
    static const ErrorCodeType TRAJ_UPLOAD_YAW_MODE_INVALID;
    static const ErrorCodeType TRAJ_UPLOAD_YAW_CMD_NOT_IN_RANGE ;
    static const ErrorCodeType TRAJ_UPLOAD_YAW_TURN_DIRECTION_INVALID;
    static const ErrorCodeType TRAJ_UPLOAD_WP_TYPE_INVALID ;
    static const ErrorCodeType TRAJ_GO_STOP_CMD_INVALID ;
    static const ErrorCodeType TRAJ_INVALID_PAUSE_RECOVERY_CMD;
    static const ErrorCodeType TRAJ_INVALID_BREAK_RESTORE_CMD ;
    static const ErrorCodeType TRAJ_INIT_INVALID_REF_POINT ;
    static const ErrorCodeType TRAJ_UPLOAD_CANNT_SET_WP_LINE_EXIT_TYPE;
    static const ErrorCodeType TRAJ_DAMPING_DIS_GE_DIS_OF_ADJ_POINTS ;
    static const ErrorCodeType TRAJ_INIT_INFO_NOT_UPLOADED ;
    static const ErrorCodeType TRAJ_WP_HAS_NOT_UPLOADED ;
    static const ErrorCodeType TRAJ_UPLOADED_WP_NOT_ENOUGH ;
    static const ErrorCodeType TRAJ_GS_HAS_STARTED ;
    static const ErrorCodeType TRAJ_GS_NOT_RUNNING ;
    static const ErrorCodeType TRAJ_GS_NOT_RUNNING_FOR_PAUSE_RECOVERY;
    static const ErrorCodeType TRAJ_GS_NOT_RUNNING_FOR_BREAK_RESTORE;
    static const ErrorCodeType TRAJ_NOT_IN_WP_MIS  ;
    static const ErrorCodeType TRAJ_MIS_HAS_BEEN_PAUSED ;
    static const ErrorCodeType TRAJ_MIS_NOT_PAUSED ;
    static const ErrorCodeType TRAJ_MIS_HAS_BEEN_BROKEN ;
    static const ErrorCodeType TRAJ_MIS_NOT_BROKEN ;
    static const ErrorCodeType TRAJ_PAUSE_RECOVERY_NOT_SUPPORTED;
    static const ErrorCodeType TRAJ_BREAK_RESTORE_NOT_SUPPORTED ;
    static const ErrorCodeType TRAJ_NO_BREAK_POINT  ;
    static const ErrorCodeType TRAJ_NO_CUR_TRAJ_PROJECT ;
    static const ErrorCodeType TRAJ_NO_NXT_TRAJ_PROJECT ;
    static const ErrorCodeType TRAJ_NO_NNT_TRAJ_PROJECT ;
    static const ErrorCodeType TRAJ_UPLOAD_WP_ID_NOT_CONTINUE ;
    static const ErrorCodeType TRAJ_WP_LINE_ENTER_NOT_SET_TO_START_WP  ;
    static const ErrorCodeType TRAJ_INIT_WP_WHEN_PLAN_HAS_STARTED  ;
    static const ErrorCodeType TRAJ_DAMPING_DIS_EXCEED_RANGE ;
    static const ErrorCodeType TRAJ_WAYPOINT_COOR_EXCEED_RANGE ;
    static const ErrorCodeType TRAJ_FIRST_WP_TYPE_IS_WP_TURN_NO;
    static const ErrorCodeType TRAJ_WP_EXCEED_RADIUS_LIMIT  ;
    static const ErrorCodeType TRAJ_WP_EXCEED_HEIGHT_LIMIT  ;


    static const ErrorCodeType COMMON_SUCCESS;
    static const ErrorCodeType COMMON_INVALID_DATA_LENGTH;
    static const ErrorCodeType COMMON_INVALD_FLOAT_NUM;
    static const ErrorCodeType TRAJ_WP_VERSION_NO_MATCH;
    static const ErrorCodeType COMMON_UNKNOWN;

    static const ErrorCodeType STATUS_RESV;
    static const ErrorCodeType STATUS_WP_MIS_CHECK_FAIL;
    static const ErrorCodeType STATUS_HOME_NOT_RECORDED;
    static const ErrorCodeType STATUS_LOW_LOCATION_ACCURACY;
    static const ErrorCodeType STATUS_RTK_CONDITION_IS_NOT_READY;

    static const ErrorCodeType SECURE_RESV;
    static const ErrorCodeType SECURE_CROSS_NFZ;
    static const ErrorCodeType SECURE_BAT_LOW;

    static const ErrorCodeType ACTION_COMMON_RESV;
    static const ErrorCodeType ACTION_COMMON_ACTION_ID_DUPLICATED;
    static const ErrorCodeType ACTION_COMMON_ACTION_ITEMS_SPACE_NOT_ENOUGH;
    static const ErrorCodeType ACTION_COMMON_ACTION_SIZE_GT_BUF_SIZE;
    static const ErrorCodeType ACTION_COMMON_ACTION_ID_NOT_FOUND;
    static const ErrorCodeType ACTION_COMMON_DOWNLOAD_ACTION_ID_RANGE_ERROR;
    static const ErrorCodeType ACTION_COMMON_NO_ACTION_ITEMS_STORED;


    static const ErrorCodeType TRIGGER_RESV;
    static const ErrorCodeType TRIGGER_TYPE_INVALID;
    static const ErrorCodeType TRIGGER_REACH_WP_END_INDEX_LT_START_INDEX;
    static const ErrorCodeType TRIGGER_REACH_WP_INVALID_INTERVAL_WP_NUM;
    static const ErrorCodeType TRIGGER_REACH_WP_INVALID_AUTO_TERMINATE_WP_NUM;
    static const ErrorCodeType TRIGGER_ASSOCIATE_INVALID_TYPE;
    static const ErrorCodeType TRIGGER_SIMPLE_INTERVAL_INVALID_TYPE;

    static const ErrorCodeType ACTUATOR_COMMON_RESV;
    static const ErrorCodeType ACTUATOR_COMMON_ACTUATOR_EXEC_NON_SUPPORTED;
    static const ErrorCodeType ACTUATOR_COMMON_ACTUATOR_TYPE_INVALID;
    static const ErrorCodeType ACTUATOR_COMMON_ACTUATOR_FUNC_INVALID;

    static const ErrorCodeType ACTUATOR_CAMERA_RESV;
    static const ErrorCodeType ACTUATOR_CAMERA_SEND_SINGLE_SHOT_CMD_TO_CAMERA_FAIL;
    static const ErrorCodeType ACTUATOR_CAMERA_SEND_VIDEO_START_CMD_TO_CAMERA_FAIL;
    static const ErrorCodeType ACTUATOR_CAMERA_SEND_VIDEO_STOP_CMD_TO_CAMERA_FAIL;
    static const ErrorCodeType ACTUATOR_CAMERA_FOCUS_PARAM_XY_INVALID;
    static const ErrorCodeType ACTUATOR_CAMERA_SEND_FOCUS_CMD_TO_CAMERA_FAIL;
    static const ErrorCodeType ACTUATOR_CAMERA_SEND_FOCALIZE_CMD_TO_CAMERA_FAIL;
    static const ErrorCodeType ACTUATOR_CAMERA_FOCAL_DISTANCE_INVALID;
    static const ErrorCodeType ACTUATOR_CAMERA_EXEC_FAIL;

    static const ErrorCodeType ACTUATOR_GIMBAL_RESV;
    static const ErrorCodeType ACTUATOR_GIMBAL_INVALID_RPY_ANGLE_CTRL_CMD;
    static const ErrorCodeType ACTUATOR_GIMBAL_INVALID_DURATION_CMD;
    static const ErrorCodeType ACTUATOR_GIMBAL_FAIL_TO_ARRIVE_TGT_ANGLE;
    static const ErrorCodeType ACTUATOR_GIMBAL_FAIL_TO_SEND_CMD_TO_GIMBAL;
    static const ErrorCodeType ACTUATOR_GIMBAL_THIS_INDEX_OF_GIMBAL_NOT_DOING_UNIFORM_CTRL;

    static const ErrorCodeType ACTUATOR_FLIGHT_RESV;
    static const ErrorCodeType ACTUATOR_FLIGHT_YAW_INVALID_YAW_ANGLE;
    static const ErrorCodeType ACTUATOR_FLIGHT_YAW_TO_TGT_ANGLE_TIMEOUT;
    static const ErrorCodeType ACTUATOR_FLIGHT_ACTION_YAW_OCCUPIED;
    static const ErrorCodeType ACTUATOR_FLIGHT_CUR_AND_TGT_VEL_CLE_STATUE_EQUAL;

    static const ErrorCodeType ACTUATOR_PAYLOAD_RESV;
    static const ErrorCodeType ACTUATOR_PAYLOAD_FAIL_TO_SEND_CMD_TO_PAYLOAD;
    static const ErrorCodeType ACTUATOR_PAYLOAD_EXEC_FAILED;

  };

  /*!
   * @brief Common ACK Error Codes
   * @deprecated This error code type is deprecated and will replaced
   * by new errorcode in the future.
   */
  class CommonACK {
   public:
    const static uint16_t SUCCESS;
    const static uint16_t KEY_ERROR;
    const static uint16_t NO_AUTHORIZATION_ERROR;
    const static uint16_t NO_RIGHTS_ERROR;
    const static uint16_t SYSTEM_ERROR;
    const static uint16_t NO_RESPONSE_ERROR;

    // These error codes would return either from
    // CMDSet Control::Task or from Missions
    const static uint8_t MOTOR_FAIL_NONE;
    /*! The compass being used appears as follows: <br>
     * (1) The compass data has too much noise. <br>
     * (2) The compass data is stuck. <br>
     * (3) The compass is disconnected. <br>
     * (4) Compass user compilation error. <br>
     * For the flight control of N3, A3 and M600, there are
     * more situations:  <br>
     * (5) The compass is disturbed.  <br>
     * (6) Multiple compasses point different directions. <br>
     * (7) Compass calibration failed. <br>
     * (8) The compass is not calibrated. */
    const static uint8_t MOTOR_FAIL_COMPASS_ABNORMAL;
    /*! The aircraft is connected to the software for debugging parameters via
       the USB cable. */
    const static uint8_t MOTOR_FAIL_ASSISTANT_PROTECTED;
    /*! The structure of the parameter list has changed after the FW upgrade.*/
    const static uint8_t MOTOR_FAIL_DEVICE_LOCKED;
    /*! The IMU being used appears as follows: <br>
     * (1) The accelerometer output exceeds its range. <br>
     * (2) The accelerometer is stuck. <br>
     * (3) The accelerometer data has too much noise. <br>
     * (4) The accelerometer outputs illegal floating numbers. <br>
     * (5) The factory data of IMU has exception. <br>
     * (6) Multiple accelerometers output differently. <br>
     * (7) The temperature of the IMU is too high. <br>
     * (8) The temperature of the IMU is very high. <br>
     * (9) The gyro output exceeds its range. <br>
     * (10) The gyro is stuck. <br>
     * (11) The gyro data has too much noise. <br>
     * (12) The gyro outputs illegal floating numbers. <br>
     * (13) Multiple accelerometers output differently. <br>
     * (14) The temperature control of gyro is abnormal. <br>
     * For the flight control of Inspire 2, there are more situations: <br>
     * (15)The default IMU exception causes the switch to backup IMU.*/
    const static uint8_t MOTOR_FAIL_IMU_NEED_ADV_CALIBRATION;
    /*! The SN status is wrong. */
    const static uint8_t MOTOR_FAIL_IMU_SN_ERROR;
    /*! The IMU being used is preheated and current temperature is not wihin the
     * calibration range. */
    const static uint8_t MOTOR_FAIL_IMU_PREHEATING;
    /*! Compass is being calibrated. */
    const static uint8_t MOTOR_FAIL_COMPASS_CALIBRATING;
    /*! The attitude data output by navigation system being used is zero.*/
    const static uint8_t MOTOR_FAIL_IMU_NO_ATTITUDE;
    /*! The aircraft is in Novice Mode without gps. */
    const static uint8_t MOTOR_FAIL_NO_GPS_IN_NOVICE_MODE;
    /*! Error occured in the battery cell. */
    const static uint8_t MOTOR_FAIL_BATTERY_CELL_ERROR;
    /*! Battery communication is abnormal. */
    const static uint8_t MOTOR_FAIL_BATTERY_COMMUNICATION_ERROR;
    /*! Battery voltage is below the minimum allowable value. */
    const static uint8_t MOTOR_FAIL_BATTERY_VOLTAGE_TOO_LOW;
    /*! The volume (%) is below the second-level power set by user. */
    const static uint8_t MOTOR_FAIL_BATTERY_USER_LOW_LAND;
    /*! The voltage is below the second-level power set by user. */
    const static uint8_t MOTOR_FAIL_BATTERY_MAIN_VOL_LOW;
    const static uint8_t MOTOR_FAIL_BATTERY_TEMP_VOL_LOW;
    /*! Flight contol calculates that current power is only adequate to land.*/
    const static uint8_t MOTOR_FAIL_BATTERY_SMART_LOW_LAND;
    /*! This error occurs whin 7s after power up.
     * Also, it occurs if the battery certification hasn't passed yet.*/
    const static uint8_t MOTOR_FAIL_BATTERY_NOT_READY;
    /*! The simulator is running. */
    const static uint8_t MOTOR_FAIL_RUNNING_SIMULATOR;
    /*! The aircraft (Inspire series) is setting itself to packing config.*/
    const static uint8_t MOTOR_FAIL_PACK_MODE;
    /*! This error is caused by attitude limit of IMU if horizontal
     * attitude output by navigation system is over 55 degrees when the
     * system powered up for the first time. */
    const static uint8_t MOTOR_FAIL_IMU_ATTI_LIMIT;
    /*! The device is not activated. */
    const static uint8_t MOTOR_FAIL_NOT_ACTIVATED;
    /*! The drone is in the restricted take-off area. */
    const static uint8_t MOTOR_FAIL_IN_FLYLIMIT_AREA;
    /*! The IMU is too biased if the gyro's bias is over 0.03rad/s
     * or the accelerometer's bias is over 50 mg when first started up.*/
    const static uint8_t MOTOR_FAIL_IMU_BIAS_LIMIT;
    /*! The status output by any esc is unhealthy. */
    const static uint8_t MOTOR_FAIL_ESC_ERROR;
    /*! The IMU is initializing.The attitude data of the current
     * navigation system has not converged yet and the height
     * data of the current navigation system is not ready.*/
    const static uint8_t MOTOR_FAIL_IMU_INITING;
    /*! The system is being upgraded. */
    const static uint8_t MOTOR_FAIL_UPGRADING;
    /*! The simulator has already been run.*/
    const static uint8_t MOTOR_FAIL_HAVE_RUN_SIM;
    /*! The IMU is in calibration or the aircraft should reset after IMU
       calibration.*/
    const static uint8_t MOTOR_FAIL_IMU_CALIBRATING;
    /*! The aircraft's horizontal attitude angle exceeds the limit
     * angle when requesting automatic takeoff. */
    const static uint8_t MOTOR_FAIL_TAKEOFF_TILT_TOO_LARGE;
    const static uint8_t MOTOR_FAIL_RESERVED_31;
    const static uint8_t MOTOR_FAIL_RESERVED_32;
    const static uint8_t MOTOR_FAIL_RESERVED_33;
    const static uint8_t MOTOR_FAIL_RESERVED_34;
    const static uint8_t MOTOR_FAIL_RESERVED_35;
    const static uint8_t MOTOR_FAIL_RESERVED_36;
    const static uint8_t MOTOR_FAIL_RESERVED_37;
    const static uint8_t MOTOR_FAIL_RESERVED_38;
    const static uint8_t MOTOR_FAIL_RESERVED_39;
    const static uint8_t MOTOR_FAIL_RESERVED_40;
    /*! invalid serial number */
    const static uint8_t MOTOR_FAIL_INVALID_SN;
    const static uint8_t MOTOR_FAIL_RESERVED_42;
    const static uint8_t MOTOR_FAIL_RESERVED_43;
    /*! accessing flash data, MCU is blocked */
    const static uint8_t MOTOR_FAIL_FLASH_OPERATING;
    /* The GPS is disconnected. */
    const static uint8_t MOTOR_FAIL_GPS_DISCONNECT;
    const static uint8_t MOTOR_FAIL_INTERNAL_46;
    /*! SD card has an exception. Please repair SD card if repeats after
     * reset.*/
    const static uint8_t MOTOR_FAIL_RECORDER_ERROR;
    /*! The firmware is unmatched with configured type.*/
    const static uint8_t MOTOR_FAIL_INVALID_PRODUCT;
    const static uint8_t MOTOR_FAIL_RESERVED_49;
    const static uint8_t MOTOR_FAIL_RESERVED_50;
    const static uint8_t MOTOR_FAIL_RESERVED_51;
    const static uint8_t MOTOR_FAIL_RESERVED_52;
    const static uint8_t MOTOR_FAIL_RESERVED_53;
    const static uint8_t MOTOR_FAIL_RESERVED_54;
    const static uint8_t MOTOR_FAIL_RESERVED_55;
    const static uint8_t MOTOR_FAIL_RESERVED_56;
    const static uint8_t MOTOR_FAIL_RESERVED_57;
    const static uint8_t MOTOR_FAIL_RESERVED_58;
    const static uint8_t MOTOR_FAIL_RESERVED_59;
    const static uint8_t MOTOR_FAIL_RESERVED_60;
    /*! IMU is disconnected. Please ask technical assistance
     * for help if repeats after reset. */
    const static uint8_t MOTOR_FAIL_IMU_DISCONNECTED;
    /*! RC is in calibration. Please finish rc calibration. */
    const static uint8_t MOTOR_FAIL_RC_CALIBRATING;
    /*! RC calibration has an exception. Please calibrate the RC. */
    const static uint8_t MOTOR_FAIL_RC_CALI_DATA_OUT_RANGE;
    /* RC calibration is unfinished. Please calibrate the RC. */
    const static uint8_t MOTOR_FAIL_RC_QUIT_CALI;
    /* The center value of RC is out of range. */
    const static uint8_t MOTOR_FAIL_RC_CENTER_OUT_RANGE;
    /* RC mapping has an exception. Please check RC channel mapping. */
    const static uint8_t MOTOR_FAIL_RC_MAP_ERROR;
    /*! The aircraft type in flash is unmatched with the type in firmware. <br>
     * Please check the aircraft type.*/
    const static uint8_t MOTOR_FAIL_WRONG_AIRCRAFT_TYPE;
    /*! Some modules that must be configured  have not been configured yet. */
    const static uint8_t MOTOR_FAIL_SOME_MODULE_NOT_CONFIGURED;
    const static uint8_t MOTOR_FAIL_RESERVED_69;
    const static uint8_t MOTOR_FAIL_RESERVED_70;
    const static uint8_t MOTOR_FAIL_RESERVED_71;
    const static uint8_t MOTOR_FAIL_RESERVED_72;
    const static uint8_t MOTOR_FAIL_RESERVED_73;
    /*! navigation system abnormal */
    const static uint8_t MOTOR_FAIL_NS_ABNORMAL;
    /*! Each craft has a set of devices to register. <br>
     * It won't take off if a class of device is missing. Please reset and check
     * the connection.*/
    const static uint8_t MOTOR_FAIL_TOPOLOGY_ABNORMAL;
    /*! The RC needs calibration. Please calibrate the RC. */
    const static uint8_t MOTOR_FAIL_RC_NEED_CALI;
    /*! The system detects illegal data. */
    const static uint8_t MOTOR_FAIL_INVALID_FLOAT;
    /*! This error will happen only in M600 if the aircraft
     * detects the battery number is not engouh. <br>
     * Please insert more battery. */
    const static uint8_t MOTOR_FAIL_M600_BAT_TOO_FEW;
    /*! Battery certification failed. <br>
     * Please ask technical assistance for help if repeats after reset. */
    const static uint8_t MOTOR_FAIL_M600_BAT_AUTH_ERR;
    /*! Battery communication is abnormal. <br>
     * Please check the battery connection. */
    const static uint8_t MOTOR_FAIL_M600_BAT_COMM_ERR;
    /*! Battery voltage difference is too large. Please check the battery
       status.*/
    const static uint8_t MOTOR_FAIL_M600_BAT_DIF_VOLT_LARGE_1;
    const static uint8_t MOTOR_FAIL_BATTERY_BOLTAHGE_DIFF_82;
    /*! Version mismatch. Please check the firmware version. */
    const static uint8_t MOTOR_FAIL_INVALID_VERSION;
    /*! There is an gimbal attitude error which happens only in M600.*/
    const static uint8_t MOTOR_FAIL_GIMBAL_GYRO_ABNORMAL;
    /*! There is an gimbal pitch motor error which happens only in M600. */
    const static uint8_t MOTOR_FAIL_GIMBAL_ESC_PITCH_NO_DATA;
    /*! There is an gimbal roll motor error which happens only in M600. */
    const static uint8_t MOTOR_FAIL_GIMBAL_ESC_ROLL_NO_DATA;
    /*! There is an gimbal yaw motor error which happens only in M600. */
    const static uint8_t MOTOR_FAIL_GIMBAL_ESC_YAW_NO_DATA;
    /*! The gimbal is updating. Please wait for the upgrade. <br>
     * This happens only in M600. */
    const static uint8_t MOTOR_FAIL_GIMBAL_FIRM_IS_UPDATING;
    /* The gimbal is out of control. This happens only in M600. */
    const static uint8_t MOTOR_FAIL_GIMBAL_OUT_OF_CONTROL;
    /*! The gimbal has self-oscillation in the pitch direction. <br>
     * Please lock the camera or reduce the gimbal sensitivity.*/
    const static uint8_t MOTOR_FAIL_GIMBAL_PITCH_SHOCK;
    /*! The gimbal has self-oscillation in the roll direction. <br>
     * Please lock the camera or reduce the gimbal sensitivity.*/
    const static uint8_t MOTOR_FAIL_GIMBAL_ROLL_SHOCK;
    /*! The gimbal has self-oscillation in the yaw direction. <br>
     * Please lock the camera or reduce the gimbal sensitivity.*/
    const static uint8_t MOTOR_FAIL_GIMBAL_YAW_SHOCK;
    /*! IMU calibration finished. Please reset aircraft.*/
    const static uint8_t MOTOR_FAIL_IMU_CALI_SUCCESS;
    /*! The aircraft has rollover when taking off. <br>
     * Please check the status of the IMU and motors. */
    const static uint8_t MOTOR_FAIL_TAKEOFF_EXCEPTION;
    /*! The motor is locked. Please check the status of the motors and blades.*/
    const static uint8_t MOTOR_FAIL_ESC_STALL_NEAR_GOUND;
    /*! The feedback speed of motor is different with the input command.*/
    const static uint8_t MOTOR_FAIL_ESC_UNBALANCE_ON_GRD;
    /*! There are some no-load motors. Please check the status of the motors and
       blades.*/
    const static uint8_t MOTOR_FAIL_ESC_PART_EMPTY_ON_GRD;
    /*! During starting, the speed of any motor is less than the minimum
     * starting
     * speed. <br>
     * For N3 and A3, the minimum starting speed is 100rpm. <br>
     * For M600, the minimum starting speed is 700rpm. <br>
     * For other aircrafts, the minimum starting speed is 1100rpm.*/
    const static uint8_t MOTOR_FAIL_ENGINE_START_FAILED;
    /*! During automatic take-off, the status of aircraft doesn't
     * change from "on the ground" to "in the air" in 5s.*/
    const static uint8_t MOTOR_FAIL_AUTO_TAKEOFF_LAUNCH_FAILED;
    /*! The aircraft is on a rollover on the ground or
     * the attitude control fails near ground. */
    const static uint8_t MOTOR_FAIL_ROLL_OVER_ON_GRD;
    /*! Battery version error. Please check the battery version. */
    const static uint8_t MOTOR_FAIL_BAT_VERSION_ERR;
    /*! RTK initialization error. */
    const static uint8_t MOTOR_FAIL_RTK_INITING;
    /*! rtk yaw and magnetometer yaw misaligned */
    const static uint8_t MOTOR_FAIL_RTK_FAIL_TO_INIT;
    const static uint8_t MOTOR_FAIL_RESERVED_104;
    const static uint8_t MOTOR_FAIL_RESERVED_105;
    const static uint8_t MOTOR_FAIL_RESERVED_106;
    const static uint8_t MOTOR_FAIL_RESERVED_107;
    const static uint8_t MOTOR_FAIL_RESERVED_108;
    const static uint8_t MOTOR_FAIL_RESERVED_109;
    /*! The motor status shows the motor has been started.*/
    const static uint8_t START_MOTOR_FAIL_MOTOR_STARTED;
    const static uint8_t MOTOR_FAIL_INTERNAL_111;
    /*! The esc is in calibration. */
    const static uint8_t MOTOR_FAIL_ESC_CALIBRATING;
    /* GPS signature is invalid because the GPS module
     * has not received valid signature information for 2s. */
    const static uint8_t MOTOR_FAIL_GPS_SIGNATURE_INVALID;
    /* The gimbal is in calibration. */
    const static uint8_t MOTOR_FAIL_GIMBAL_CALIBRATING;
    /*! The aircraft is force locked by APP.*/
    const static uint8_t MOTOR_FAIL_FORCE_DISABLE;
    /*! The height of the takeoff is abnormal. <br>
     * This error happens when the takeoff height relative to ground is up to
     * 100m.*/
    const static uint8_t TAKEOFF_HEIGHT_EXCEPTION;
    /*! ESC needs upgrade. */
    const static uint8_t MOTOR_FAIL_ESC_NEED_UPGRADE;
    /*! IMU direction is misaligned.*/
    const static uint8_t MOTOR_FAIL_GYRO_DATA_NOT_MATCH;
    /*! APP stops the takeoff.*/
    const static uint8_t MOTOR_FAIL_APP_NOT_ALLOW;
    /*! Compass direction is not the same with IMU. */
    const static uint8_t MOTOR_FAIL_COMPASS_IMU_MISALIGN;
    /*! The flash is unlocked. */
    const static uint8_t MOTOR_FAIL_FLASH_UNLOCK;
    /*! The ESC is in the buzzing mode.*/
    const static uint8_t MOTOR_FAIL_ESC_SCREAMING;
    /*! The temperature of ESC is too high. */
    const static uint8_t MOTOR_FAIL_ESC_TEMP_HIGH;
    /*! The battery is not in place. */
    const static uint8_t MOTOR_FAIL_BAT_ERR;
    /*! The aircraft detects an impact if the measured value of
     * accelerometer exceeds 8g near ground.*/
    const static uint8_t IMPACT_IS_DETECTED;
    /*! Under the P stall, the aircraft mode degenerates to the Attitude mode.*/
    const static uint8_t MOTOR_FAIL_MODE_FAILURE;
    /*! The aircraft recently had an error of NO. 125.*/
    const static uint8_t MOTOR_FAIL_CRAFT_FAIL_LATELY;
    /*! The aircraft kill switch is on, please disable it*/
    const static uint8_t KILL_SWITCH_ON;
    /*! The code logic is illegal.*/
    const static uint8_t MOTOR_FAIL_MOTOR_CODE_ERROR;
  };

  /*!
   * @brief CMDSet: Activation ACK Error Codes
   * @deprecated This error code type is deprecated and will replaced
   * by new errorcode in the future.
   */
  class ActivationACK {
   public:
    const static uint16_t SUCCESS;
    const static uint16_t PARAMETER_ERROR;
    const static uint16_t ENCODE_ERROR;
    const static uint16_t NEW_DEVICE_ERROR;
    const static uint16_t DJIGO_APP_NOT_CONNECTED;
    const static uint16_t NETWORK_ERROR;
    const static uint16_t SERVER_ACCESS_REFUSED;
    const static uint16_t ACCESS_LEVEL_ERROR;
    const static uint16_t OSDK_VERSION_ERROR;
  };

  /*!
   * @brief CMDSet: Control ACK Error Codes
   * @deprecated This error code type is deprecated and will replaced
   * by new errorcode in the future.
   */
  class ControlACK {
   public:
    /*!
     * @brief CMDID: SetControl
     */
    typedef struct SetControl {
      const static uint16_t RC_MODE_ERROR;
      const static uint16_t RELEASE_CONTROL_SUCCESS;
      const static uint16_t OBTAIN_CONTROL_SUCCESS;
      const static uint16_t OBTAIN_CONTROL_IN_PROGRESS;
      const static uint16_t RELEASE_CONTROL_IN_PROGRESS;
      const static uint16_t RC_NEED_MODE_F;
      const static uint16_t RC_NEED_MODE_P;
      const static uint16_t IOC_OBTAIN_CONTROL_ERROR;
    } SetControl;

    /*!
     * @note New 3.3 release
     *
     * @brief CMDID: Task
     */
    typedef struct Task {
      const static uint16_t SUCCESS;
      const static uint16_t MOTOR_ON;
      const static uint16_t MOTOR_OFF;
      const static uint16_t IN_AIR;
      const static uint16_t NOT_IN_AIR;
      const static uint16_t NO_HOMEPOINT;
      const static uint16_t BAD_GPS;
      // Do not consider as error?
      const static uint16_t IN_SIMULATOR_MODE;
      const static uint16_t ALREADY_RUNNING;
      const static uint16_t NOT_RUNNING;
      const static uint16_t INVAILD_COMMAND;
      const static uint16_t NO_LANDING_GEAR;
      // Do not consider as error?
      const static uint16_t GIMBAL_MOUNTED;
      const static uint16_t BAD_SENSOR;
      const static uint16_t ALREADY_PACKED;
      const static uint16_t NO_PACKED;
      const static uint16_t PACKED_MODE_NOT_SUPPORTED;
    } Task;

    /*
     * Task ACKs Supported in firmware version < 3.3
     */
    typedef struct LegacyTask {
      const static uint16_t SUCCESS;
      const static uint16_t FAIL;
    } LegacyTask;

    /*!
     * @brief CMDID: SetArm supported in products with
     * firmware version < 3.3
     */
    typedef struct SetArm {
      const static uint16_t SUCCESS;
      const static uint16_t OBTAIN_CONTROL_NEEDED_ERROR;
      const static uint16_t ALREADY_ARMED_ERROR;
      const static uint16_t AIRCRAFT_IN_AIR_ERROR;
    } SetArm;

    typedef struct KillSwitch {
      const static uint16_t SUCCESS;
    } KillSwitch;

    enum ParamReadWrite : RawRetCodeType
    {
      PARAM_READ_WRITE_SUCCESS = 0,
      PARAM_READ_WRITE_FAIL = 1,
      PARAM_READ_WRITE_INVALID_PARAMETER = 2,
    };

    enum SetHomeLocation : RawRetCodeType
    {
      SET_HOME_LOCATION_SUCCESS = 0,
      SET_HOME_LOCATION_FAIL = 1,
    };

  }; // Control class
  
  /*!
   * @note New in 3.3 release
   *
   * @brief CMDSet: Subscribe
   * @deprecated This error code type is deprecated and will replaced
   * by new errorcode in the future.
   */
  class SubscribeACK {
   public:
    const static uint8_t SUCCESS;
    const static uint8_t ILLEGAL_DATA_LENGTH;
    const static uint8_t VERSION_DOES_NOT_MATCH;
    const static uint8_t PACKAGE_OUT_OF_RANGE;
    const static uint8_t PACKAGE_ALREADY_EXISTS;
    const static uint8_t PACKAGE_DOES_NOT_EXIST;
    const static uint8_t ILLEGAL_FREQUENCY;
    const static uint8_t PACKAGE_TOO_LARGE;
    const static uint8_t PIPELINE_OVERFLOW;
    const static uint8_t INTERNAL_ERROR_0X09;
    const static uint8_t PACKAGE_EMPTY;
    const static uint8_t INCORRECT_NUM_OF_TOPICS;
    const static uint8_t ILLEGAL_UID;
    const static uint8_t PERMISSION_DENY;
    const static uint8_t MULTIPLE_SUBSCRIBE;
    const static uint8_t SOURCE_DEVICE_OFFLINE;
    const static uint8_t PAUSED;
    const static uint8_t RESUMED;
    const static uint8_t INTERNAL_ERROR_0X4A;
    const static uint8_t FAILED_AUTHENTICATION;
    const static uint8_t VERSION_VERSION_TOO_FAR;
    const static uint8_t VERSION_UNKNOWN_ERROR;
    const static uint8_t INTERNAL_ERROR_0XFF;
  };

  /*!
   * @brief Mission ACK Error Codes
   * @deprecated This error code type is deprecated and will replaced
   * by new errorcode in the future.
   */
  class MissionACK {
   public:
    /*! @brief Common Mission ACK codes
     *
     */
    typedef struct Common {
      const static uint8_t SUCCESS;
      const static uint8_t WRONG_WAYPOINT_INDEX;
      const static uint8_t RC_NOT_IN_MODE_F;
      const static uint8_t OBTAIN_CONTROL_REQUIRED;
      const static uint8_t CLOSE_IOC_REQUIRED;
      const static uint8_t NOT_INITIALIZED;
      const static uint8_t NOT_RUNNING;
      const static uint8_t IN_PROGRESS;
      /*!Estimated time needed to perform a task is greater
       * than the flight time left*/
      const static uint8_t TASK_TIMEOUT;
      const static uint8_t OTHER_MISSION_RUNNING;
      /*!GPS signal GPS_LEVEL < 3*/
      const static uint8_t BAD_GPS;
      const static uint8_t RTK_NOT_READY;
      /*!Battery beyond first-stage voltage for non-smart battery
       * OR first-stage volume for smart battery*/
      const static uint8_t LOW_BATTERY;
      const static uint8_t VEHICLE_DID_NOT_TAKE_OFF;
      const static uint8_t INVALID_PARAMETER;
      /*!Execution condition is not satisfied
       * @note Aircraft not in one of the following modes:
       * Assist Takeoff
       * Auto Takeoff
       * Auto Landing
       * Go Home
       */
      const static uint8_t CONDITIONS_NOT_SATISFIED;
      const static uint8_t CROSSING_NO_FLY_ZONE;
      /*!HomePoint not recorded*/
      const static uint8_t UNRECORDED_HOME;
      const static uint8_t AT_NO_FLY_ZONE;
      /*!Height is too high (higher than MAX flying height
       * set by user (default: 120m))*/
      const static uint8_t TOO_HIGH;
      /*!Height is too low (lower than 5m)*/
      const static uint8_t TOO_LOW;
      const static uint8_t TOO_FAR_FROM_HOME;
      /*!Mission not supported*/
      const static uint8_t NOT_SUPPORTED;
      /*!Current position of aircraft is too far from the HotPoint
       * or first point*/
      const static uint8_t TOO_FAR_FROM_CURRENT_POSITION;
      const static uint8_t BEGGINER_MODE_NOT_SUPPORTED;
      const static uint8_t TAKE_OFF_IN_PROGRESS;
      const static uint8_t LANDING_IN_PROGRESS;
      const static uint8_t RRETURN_HOME_IN_PROGRESS;
      const static uint8_t START_MOTORS_IN_PROGRESS;
      const static uint8_t INVALID_COMMAND;
      const static uint8_t UNKNOWN_ERROR;
    } Common;

    //! @brief Follow Mission ACK Error Code
    typedef struct Follow {
      const static uint8_t TOO_FAR_FROM_YOUR_POSITION_LACK_OF_RADIO_CONNECTION;
      const static uint8_t CUTOFF_TIME_OVERFLOW;
      const static uint8_t GIMBAL_PITCH_ANGLE_OVERFLOW;
    } Follow;

    //! @brief HotPoint Mission ACK Error Code
    typedef struct HotPoint {
      const static uint8_t INVALID_RADIUS;
      const static uint8_t YAW_RATE_OVERFLOW;
      /*
       * Start point given by user during HotPoint mission initialization is
       * invalid.
       * Available options are :
       * 0 - North to the HP
       * 1 - South
       * 2 - West
       * 3 - East
       * 4 - Nearest Point
       */
      const static uint8_t INVALID_START_POINT;
      const static uint8_t INVALID_YAW_MODE;
      const static uint8_t TOO_FAR_FROM_HOTPOINT;
      const static uint8_t INVALID_PAREMETER;
      const static uint8_t INVALID_LATITUDE_OR_LONGITUTE;
      const static uint8_t INVALID_DIRECTION;
      const static uint8_t IN_PAUSED_MODE;
      const static uint8_t FAILED_TO_PAUSE;
    } HotPoint;

    //! @brief WayPoint Mission ACK Error Code
    typedef struct WayPoint {
      const static uint8_t INVALID_DATA;
      const static uint8_t INVALID_POINT_DATA;
      const static uint8_t TRACE_TOO_LONG;
      const static uint8_t TOTAL_DISTANCE_TOO_LONG;
      const static uint8_t POINT_OVERFLOW;
      const static uint8_t POINTS_TOO_CLOSE;
      const static uint8_t POINTS_TOO_FAR;
      const static uint8_t CHECK_FAILED;
      const static uint8_t INVALID_ACTION;
      const static uint8_t POINT_DATA_NOT_ENOUGH;
      const static uint8_t DATA_NOT_ENOUGH;
      const static uint8_t POINTS_NOT_ENOUGH;
      const static uint8_t IN_PROGRESS;
      const static uint8_t NOT_IN_PROGRESS;
      const static uint8_t INVALID_VELOCITY;
      // Backward Compatibility
      const static uint8_t& DISTANCE_OVERFLOW __attribute__((deprecated));
      const static uint8_t& TIMEOUT __attribute__((deprecated));
    } WayPoint;

    //! @brief IOC ACK Mission Error Code
    typedef struct IOC {
      const static uint8_t TOO_CLOSE_TO_HOME;
      const static uint8_t UNKNOWN_TYPE;
    } IOC;

  };  // Class Mission

  class WaypointV2ACK{
  public:
/* The enum that defines the detail error code of common module */
    enum WaypointV2ErrorCodeCommon
    {
      GS_ERR_CODE_COMMON_SUCCESS                                  = 0x00000,
      GS_ERR_CODE_COMMON_INVALID_DATA_LENGTH                      = 0x00001,   /*!< the length of the data is illegal based on the protocol */
      GS_ERR_CODE_COMMON_INVALD_FLOAT_NUM                         = 0x00002,   /*!< invalid float number (NAN or INF) */
      GS_ERR_CODE_TRAJ_WP_VERSION_NO_MATCH                        = 0x00003,   /*!< waypoint mission version can't match with firmware*/
      GS_ERR_CODE_COMMON_UNKNOWN                                  = 0x0ffff,   /*!< Fatal error! Unexpected result! */
    } ;

/* The enum that defines the detail error code of flight-trajectory module */
    enum WaypointV2ErrorCodeTraj
    {
      GS_ERR_CODE_TRAJ_RESV                                       = 0x10000,
      GS_ERR_CODE_TRAJ_INIT_WP_NUM_TOO_MANY                       = 0x10001,   /*!< min_initial_waypoint_num is large than permitted_max_waypoint_num */
      GS_ERR_CODE_TRAJ_INIT_WP_NUM_TOO_FEW                        = 0x10002,   /*!< min_initial_waypoint_num is less than permitted_min_waypoint_num */
      GS_ERR_CODE_TRAJ_INIT_INVALID_END_INDEX                     = 0x10003,   /*!< waypoint_end_index is equal or large than total_waypoint_num */
      GS_ERR_CODE_TRAJ_UPLOAD_START_ID_GT_END_ID                  = 0x10004,   /*!< the start index is greater than end index of upload wps */
      GS_ERR_CODE_TRAJ_UPLOAD_END_ID_GT_TOTAL_NUM                 = 0x10005,   /*!< the end index of uplod wps is greater than inited total numbers */
      GS_ERR_CODE_TRAJ_DOWNLOAD_WPS_NOT_IN_STORED_RAGNE           = 0x10006,   /*!< the index of first and end waypoint expected to download is not in range of stored in FC */
      GS_ERR_CODE_TRAJ_CUR_POS_IS_FAR_AWAY_FROM_FIRST_WP          = 0x10008,   /*!< current position is far away from the first waypoint. */
      GS_ERR_CODE_TRAJ_ADJ_WPS_TOO_CLOSE                          = 0x1000a,   /*!< it is too close from two adjacent waypoints, the value of which might be distinguish from products */
      GS_ERR_CODE_TRAJ_ADJ_WPS_TOO_FAR                            = 0x1000b,   /*!< the distance betwween two adjacent waypoints is not in[0.5m, 5000m] the value of which might be distinguish from products */
      GS_ERR_CODE_TRAJ_UPLOAD_MAX_VEL_GT_GLOBAL                   = 0x1000c,   /*!< the max vel of uplod wp is greater than global max vel */
      GS_ERR_CODE_TRAJ_UPLOAD_LOCAL_CRUISE_VEL_GT_LOCAL_MAX       = 0x1000d,   /*!< the local cruise vel of upload wp is greater than local max vel */
      GS_ERR_CODE_TRAJ_UPLOAD_LOCAL_CRUISE_VEL_GT_GLOBAL_MAX      = 0x1000e,   /*!< the local cruise vel of upload wp is greater than global max vel */
      GS_ERR_CODE_TRAJ_INIT_INVALID_GLOBAL_MAX_VEL                = 0x1000f,   /*!< global_max_vel is greater than permitted_max_vel or less than permitted_min_vel */
      GS_ERR_CODE_TRAJ_GLOBAL_CRUISE_VEL_GT_MAX_VEL               = 0x10010,   /*!< global_cruise_vel is greater than global_max_vel */
      GS_ERR_CODE_TRAJ_INIT_INVALID_GOTO_FIRST_FLAG               = 0x10011,   /*!< goto_first_point_mode is out of range of waypoint_goto_first_flag_t_enum */
      GS_ERR_CODE_TRAJ_INIT_INVALID_FINISHED_ACTION               = 0x10012,   /*!< finished_action is out of range of wp_plan_finish_action_t_enum */
      GS_ERR_CODE_TRAJ_INIT_INVALID_RC_LOST_ACTION                = 0x10013,   /*!< rc_lost_action is out of range of wp_plan_rc_lost_action_t_enum */
      GS_ERR_CODE_TRAJ_UPLOAD_YAW_MODE_INVALID                    = 0x10014,   /*!< the yaw mode of upload wp is invalid. reference to waypoint2_yaw_mode_t defined in math_waypoint_planner.h */
      GS_ERR_CODE_TRAJ_UPLOAD_YAW_CMD_NOT_IN_RANGE                = 0x10015,   /*!< the yaw command of upload wp is not in range. the range for MR:[-180 180],unit:degree */
      GS_ERR_CODE_TRAJ_UPLOAD_YAW_TURN_DIRECTION_INVALID          = 0x10016,   /*!< the yaw turn direction of upload wp is invalid. it should be 0:clockwise or 1:anti-clockwise */
      GS_ERR_CODE_TRAJ_UPLOAD_WP_TYPE_INVALID                     = 0x10017,   /*!< the wp type of upload wp is invalid. reference to waypoint_type_t defined in math_waypoint_planner.h */
      GS_ERR_CODE_TRAJ_GO_STOP_CMD_INVALID                        = 0x10018,   /*!< go/stop command is invalid. */
      GS_ERR_CODE_TRAJ_INVALID_PAUSE_RECOVERY_CMD                 = 0x10019,   /*!< the command of pause/recovery is not equal to any of the command enum */
      GS_ERR_CODE_TRAJ_INVALID_BREAK_RESTORE_CMD                  = 0x1001a,   /*!< the command of break/restore is not equal to any of the command enum */
      GS_ERR_CODE_TRAJ_INIT_INVALID_REF_POINT                     = 0x1001b,   /*!< initial reference point position coordinate exceed set range */
      GS_ERR_CODE_TRAJ_UPLOAD_CANNT_SET_WP_LINE_EXIT_TYPE         = 0x1001d,   /*!< cann't set wp_line_exit type to wp */
      GS_ERR_CODE_TRAJ_DAMPING_DIS_GE_DIS_OF_ADJ_POINTS           = 0x1001c,    /*!< the damping dis is greater than or equal the distance of adjacent point */
      GS_ERR_CODE_TRAJ_INIT_INFO_NOT_UPLOADED                     = 0x1001e,   /*!< the init info of Ground Station is not uploaded yet */
      GS_ERR_CODE_TRAJ_WP_HAS_NOT_UPLOADED                        = 0x1001f,   /*!< the wp has not uploaded yet */
      GS_ERR_CODE_TRAJ_UPLOADED_WP_NOT_ENOUGH                     = 0x10020,   /*!< min_initial_waypoint_num is not uploaded. */
      GS_ERR_CODE_TRAJ_GS_HAS_STARTED                             = 0x10021,   /*!< waypoint plan has started when received go command. */
      GS_ERR_CODE_TRAJ_GS_NOT_RUNNING                             = 0x10022,   /*!< waypoint plan not running when received stop command. */
      GS_ERR_CODE_TRAJ_GS_NOT_RUNNING_FOR_PAUSE_RECOVERY          = 0x10023,   /*!< ground station(GS) is not started(used by pause/recovery) */
      GS_ERR_CODE_TRAJ_GS_NOT_RUNNING_FOR_BREAK_RESTORE           = 0x10024,   /*!< ground station(GS) is not started(used by break/restore) */
      GS_ERR_CODE_TRAJ_NOT_IN_WP_MIS                              = 0x10025,   /*!< not in the waypoint mission(MIS)(cannot pause/recovery or break/restore) */
      GS_ERR_CODE_TRAJ_MIS_HAS_BEEN_PAUSED                        = 0x10026,   /*!< the current status is paused, cannot command pause again */
      GS_ERR_CODE_TRAJ_MIS_NOT_PAUSED                             = 0x10027,   /*!< not in paused status, cannot command recovery */
      GS_ERR_CODE_TRAJ_MIS_HAS_BEEN_BROKEN                        = 0x10028,   /*!< the current status is broken, cannot command break again */
      GS_ERR_CODE_TRAJ_MIS_NOT_BROKEN                             = 0x10029,   /*!< not in break status, cannot command restore */
      GS_ERR_CODE_TRAJ_PAUSE_RECOVERY_NOT_SUPPORTED               = 0x1002a,   /*!< the configuration forbid using pause/recovery API */
      GS_ERR_CODE_TRAJ_BREAK_RESTORE_NOT_SUPPORTED                = 0x1002b,   /*!< the configuration forbid using break/restore API */
      GS_ERR_CODE_TRAJ_NO_BREAK_POINT                             = 0x1002c,   /*!< no break point is recorded for restore */
      GS_ERR_CODE_TRAJ_NO_CUR_TRAJ_PROJECT                        = 0x1002d,   /*!< no current trajectory project point is recorded for restore */
      GS_ERR_CODE_TRAJ_NO_NXT_TRAJ_PROJECT                        = 0x1002e,   /*!< no next trajectory project point is recorded for restore */
      GS_ERR_CODE_TRAJ_NO_NNT_TRAJ_PROJECT                        = 0x1002f,   /*!< no next the next trajectory project point is recorded for restore */
      GS_ERR_CODE_TRAJ_UPLOAD_WP_ID_NOT_CONTINUE                  = 0x10030,   /*!< the index of upload wp is not continue after the store wp */
      GS_ERR_CODE_TRAJ_WP_LINE_ENTER_NOT_SET_TO_START_WP          = 0x10031,   /*!< the WP_LINE_ENTER wp_type set to a wp which is not the init start waypoint */
      GS_ERR_CODE_TRAJ_INIT_WP_WHEN_PLAN_HAS_STARTED              = 0x10032,   /*!< the waypoint plan has started when initializing waypoint */
      GS_ERR_CODE_TRAJ_DAMPING_DIS_EXCEED_RANGE                   = 0x10033,   /*!< waypoint damping distance exceed set range */
      GS_ERR_CODE_TRAJ_WAYPOINT_COOR_EXCEED_RANGE                 = 0x10034,   /*!< waypoint position coordinate exceed rational range */
      GS_ERR_CODE_TRAJ_FIRST_WP_TYPE_IS_WP_TURN_NO                = 0x10035,   /*!< first waypoint type error, it can not be WP_TURN_NO */
      GS_ERR_CODE_TRAJ_WP_EXCEED_RADIUS_LIMIT                     = 0x10038,   /*!< waypoint position exceed radius limit */
      GS_ERR_CODE_TRAJ_WP_EXCEED_HEIGHT_LIMIT                     = 0x10039,   /*!< waypoint position exceed height limit */
    } ;

/* The enum that defines the detail error code of flight-status module */
    enum WaypointV2ErrorCodeStatus
    {
      GS_ERR_CODE_STATUS_RESV                                     = 0x20000,
      GS_ERR_CODE_STATUS_WP_MIS_CHECK_FAIL                        = 0x20001,   /*!< head_node is null or atti_not_healthy or gyro_not_healthy or horiz_vel_not healthy or horiz_abs_pos_not_healthy. */
      GS_ERR_CODE_STATUS_HOME_NOT_RECORDED                        = 0x20002,   /*!< the home point is no recorded yet, which will be executed at the first time of GPS level > 3(MR/FW). */
      GS_ERR_CODE_STATUS_LOW_LOCATION_ACCURACY                    = 0x20003,   /*!< current location accuracy is low for bad GPS signal. */
      GS_ERR_CODE_STATUS_RTK_CONDITION_IS_NOT_READY               = 0x20005,   /*!< use rtk_data, but rtk is not connected or rtk_data is invalid */
    } ;

/* The enum that defines the detail error code of flight-secure module */
    enum WaypointV2ErrorCodeSecure
    {
      GS_ERR_CODE_SECURE_RESV                                     = 0x30000,
      GS_ERR_CODE_SECURE_CROSS_NFZ                                = 0x30001,   /*!< the trajectory cross the NFZ */
      GS_ERR_CODE_SECURE_BAT_LOW                                  = 0x30002,   /*!< current capacity of smart battery or voltage of non-smart battery is lower than level 1 or level 2 threshold */
    } ;

/* The enum that defines the detail error code of action-common module */
    enum WaypointV2ErrorCodeActionCommon
    {
      GS_ERR_CODE_ACTION_COMMON_RESV                                 = 0x400000,
      GS_ERR_CODE_ACTION_COMMON_ACTION_ID_DUPLICATED                 = 0x400001,   /*!< the ID of Action is duplicated. */
      GS_ERR_CODE_ACTION_COMMON_ACTION_ITEMS_SPACE_NOT_ENOUGH        = 0x400002,   /*!< there is no enough memory space for new Action Item */
      GS_ERR_CODE_ACTION_COMMON_ACTION_SIZE_GT_BUF_SIZE              = 0x400003,   /*!< the size of buffer used to get the info of Action is less than the size of Action. Normally users can not get this. */
      GS_ERR_CODE_ACTION_COMMON_ACTION_ID_NOT_FOUND                  = 0x400004,   /*!< the ID of Action is not found. */
      GS_ERR_CODE_ACTION_COMMON_DOWNLOAD_ACTION_ID_RANGE_ERROR       = 0x400005,   /*!< the download action start id is bigger than the action end id */
      GS_ERR_CODE_ACTION_COMMON_NO_ACTION_ITEMS_STORED               = 0x400006,   /*!< can not download or get min-max action ID for no action items stored in action kernel */
    } ;

/* The enum that defines the detail error code of trigger module */
    enum WaypointV2ErrorCodeTriggerModule
    {
      GS_ERR_CODE_TRIGGER_RESV                                   = 0x410000,
      GS_ERR_CODE_TRIGGER_TYPE_INVALID                           = 0x410001,   /*!< the type ID of Trigger is invalid. It might not defined or the information is empty. */
      GS_ERR_CODE_TRIGGER_REACH_WP_END_INDEX_LT_START_INDEX      = 0x410021,   /*!< wp_end_index is less than wp_start_index in reach_waypoint_trigger. */
      GS_ERR_CODE_TRIGGER_REACH_WP_INVALID_INTERVAL_WP_NUM       = 0x410022,   /*!< interval_wp_num is large than the difference of wp_start_index and wp_end_index in reach_waypoint_trigger. */
      GS_ERR_CODE_TRIGGER_REACH_WP_INVALID_AUTO_TERMINATE_WP_NUM = 0x410023,   /*!< auto_terminate_wp_num is large than interval_wp_num in reach_waypoint_trigger. */
      GS_ERR_CODE_TRIGGER_ASSOCIATE_INVALID_TYPE                 = 0x410041,   /*!< the associate_type is greater than the maximum value.  */
      GS_ERR_CODE_TRIGGER_SIMPLE_INTERVAL_INVALID_TYPE           = 0x410081,   /*!< the interval type is greater than the maximum value. */
    } ;

/* The enum that defines the detail error code of actuator-common module */
    enum WaypointV2ErrorCodeActuatorCommon
    {
      GS_ERR_CODE_ACTUATOR_COMMON_RESV                                 = 0x420000,
      GS_ERR_CODE_ACTUATOR_COMMON_ACTUATOR_EXEC_NON_SUPPORTED          = 0x420001,   /*!< the execution of Actuator is not supported, e.g., try to stop camera shooting. */
      GS_ERR_CODE_ACTUATOR_COMMON_ACTUATOR_TYPE_INVALID                = 0x420002,   /*!< the type ID of Actuator is invalid. It might not defined or the information is empty. */
      GS_ERR_CODE_ACTUATOR_COMMON_ACTUATOR_FUNC_INVALID                = 0x420003,   /*!< the Function ID of Actuator is invalid. It might not defined or the information is empty. */
    } ;

/* The enum that defines the detail error code of action-camera module */
    enum WaypointV2ErrorCodeActuatorCamera
    {
      GS_ERR_CODE_ACTUATOR_CAMERA_RESV                                 = 0x430000,
      GS_ERR_CODE_ACTUATOR_CAMERA_SEND_SINGLE_SHOT_CMD_TO_CAMERA_FAIL  = 0x430001,   /*!< fail to send shot cmd to camera for no camera or camera is busy. */
      GS_ERR_CODE_ACTUATOR_CAMERA_SEND_VIDEO_START_CMD_TO_CAMERA_FAIL  = 0x430002,    /*!< fail to send video start cmd to camera for no camera or camera is busy. */
      GS_ERR_CODE_ACTUATOR_CAMERA_SEND_VIDEO_STOP_CMD_TO_CAMERA_FAIL   = 0x430003,    /*!< fail to send video stop cmd to camera for no camera or camera is not busy. */
      GS_ERR_CODE_ACTUATOR_CAMERA_FOCUS_PARAM_XY_INVALID               = 0x430004,   /*!< camera focus param xy exceed valid range (0, 1). */
      GS_ERR_CODE_ACTUATOR_CAMERA_SEND_FOCUS_CMD_TO_CAMERA_FAIL        = 0x430005,   /*!< fail to send focus cmd to camera for no camera or camera is busy. */
      GS_ERR_CODE_ACTUATOR_CAMERA_SEND_FOCALIZE_CMD_TO_CAMERA_FAIL     = 0x430006,   /*!< fail to send focalize cmd to camera for no camera or camera is busy. */
      GS_ERR_CODE_ACTUATOR_CAMERA_FOCAL_DISTANCE_INVALID               = 0x430007,   /*!< focal distance of camera focalize function exceed valid range. */
      GS_ERR_CODE_ACTUATOR_CAMERA_EXEC_FAIL                            = 0x430100,   /*!< this err code indicate camera fail to exec coressponding cmd, and the low 8 bit
                                                                                  will be replaced by the return code from camera, for example: 0x01E0 means current cmd
                                                                                  is not supported, 0x01E8 means SD card is not inserted and so on, the detailed camera
                                                                                  return code could be found in camera protocal. */
    } ;

/* The enum that defines the detail error code of action-gimbal module */
    enum WaypointV2ErrorCodeActuatorGimbal
    {
      GS_ERR_CODE_ACTUATOR_GIMBAL_RESV                                 = 0x440000,
      GS_ERR_CODE_ACTUATOR_GIMBAL_INVALID_RPY_ANGLE_CTRL_CMD           = 0x440001,   /*!< gimbal roll/pitch/yaw angle ctrl cmd param invalid, unable to exec. */
      GS_ERR_CODE_ACTUATOR_GIMBAL_INVALID_DURATION_CMD                 = 0x440002,   /*!< gimbal duration param invalid, unable to exec. */
      GS_ERR_CODE_ACTUATOR_GIMBAL_FAIL_TO_ARRIVE_TGT_ANGLE             = 0x440003,   /*!< gimbal fail to arrive target angle . */
      GS_ERR_CODE_ACTUATOR_GIMBAL_FAIL_TO_SEND_CMD_TO_GIMBAL           = 0x440004,   /*!< fail to send cmd to gimbal for gimbal is busy or no gimbal. */
      GS_ERR_CODE_ACTUATOR_GIMBAL_THIS_INDEX_OF_GIMBAL_NOT_DOING_UNIFORM_CTRL =  0x440005, /*!< fail to stop gimbal uniform ctrl because index error.*/
    } ;

/* The enum that defines the detail error code of action-flight module */
    enum WaypointV2ErrorCodeActuatorFlight
    {
      GS_ERR_CODE_ACTUATOR_FLIGHT_RESV                                 = 0x460000,
      GS_ERR_CODE_ACTUATOR_FLIGHT_YAW_INVALID_YAW_ANGLE                = 0x460001,   /*!< yaw angle is lager max yaw angle. */
      GS_ERR_CODE_ACTUATOR_FLIGHT_YAW_TO_TGT_ANGLE_TIMEOUT             = 0x460002,   /*!< faile to target yaw angle, because of timeout.*/
      GS_ERR_CODE_ACTUATOR_FLIGHT_ACTION_YAW_OCCUPIED                  = 0x460003,
      GS_ERR_CODE_ACTUATOR_FLIGHT_CUR_AND_TGT_VEL_CLE_STATUE_EQUAL     = 0x460004,   /*!<*/
    } ;

    enum WaypointV2ErrorCodeActuatorPayload
    {
      GS_ERR_CODE_ACTUATOR_PAYLOAD_RESV                                 = 0x470000,
      GS_ERR_CODE_ACTUATOR_PAYLOAD_FAIL_TO_SEND_CMD_TO_PAYLOAD          = 0x470001,   /*! */
      GS_ERR_CODE_ACTUATOR_PAYLOAD_EXEC_FAILED                          = 0x470002,   /*!*/
    } ;

  };

  /*!
   * @brief CMDSet: MFIO
   * @note New in 3.3 release
   * @deprecated This error code type is deprecated and will replaced
   * by new errorcode in the future.
   */
  class MFIOACK {
   public:
    /*!
     * @brief CMDID: init
     */
    typedef struct init {
      const static uint8_t SUCCESS;
      const static uint8_t UNKNOWN_ERROR;
      const static uint8_t PORT_NUMBER_ERROR;
      const static uint8_t PORT_MODE_ERROR;
      const static uint8_t PORT_DATA_ERROR;
    } init;

    /*!
     * @brief CMDID: set
     */
    typedef struct set {
      const static uint8_t SUCCESS;
      /*!Port not exit or not an output configuration*/
      const static uint8_t CHANNEL_ERROR;
      /*! Port not map to f channel*/
      const static uint8_t PORT_NOT_MAPPED_ERROR;
    } set;

    /*!
     * @brief CMDID: get
     */
    typedef struct get {
      const static uint8_t SUCCESS;  //! @note Anything else is failure
    } get;

  };  // Class MFIO


  enum DJI_CMD_RETURN_CODE {
    SUCCESS                          = 0x00, /*!< Execute successfully */
    FILE_TRANSFER_BUSY               = 0xC0, /*!< Busy transfering file */
    NAVIGATION_IS_FORBIDDEN          = 0xD0, /*!< RC mode is not F mode or FC disable navigation mode */
    NAVIGATION_IS_OFF                = 0xD1, /*!< FC disable navigation mode */
    INVALID_TASK_INFORMATION         = 0xD2, /*!< No valid task information */
    TASK_UPLOAD_ERROR                = 0xD3, /*!< Task up load error */
    INVALID_REQUEST_PARAMETER        = 0xD4, /*!< The request parameter is invalid */
    MAY_CROSS_RESTRICTED_AREA        = 0xD5, /*!< The task may across the restricted area */
    EXCEEDED_INPUT_TIME_LIMIT        = 0xD6, /*!< Task time may exceed input time limit */
    EXECUTING_HIGHER_PRIORITY_TASK   = 0xD7, /*!< A task of higher priority is being excuting */
    CANNOT_START_TASK_WEAK_GPS       = 0xD8, /*!< Weak GPS, cannot start the task*/
    CANNOT_START_TASK_VLOTAGE_ALARM  = 0xD9, /*!< Level 1 volt warning, cannot start the task */
    UNSUPPORTED_COMMAND              = 0xE0, /*!< Do not support this command */
    TIMEOUT                          = 0xE1, /*!< Execution timeout */
    RAM_ALLOCATION_FAILED            = 0xE2, /*!< Memory alloc failed */
    INVALID_COMMAND_PARAMETER        = 0xE3, /*!< Invalid parameter for the command */
    UNSUPPORTED_COMMAND_IN_CUR_STATE = 0xE4, /*!< Do not support this command in the current state */
    CAMERA_TIME_NOT_SYNCHRONIZED     = 0xE5, /*!< Timestamp of camera is not synchronized */
    PARAMETER_SET_FAILED             = 0xE6, /*!< Setting parameter failed */
    PARAMETER_GET_FAILED             = 0xE7, /*!< Getting parameter failed */
    SD_CARD_MISSING                  = 0xE8, /*!< SD card is not installed */
    SD_CARD_FULL                     = 0xE9, /*!< SD card is full */
    SD_CARD_ERROR                    = 0xEA, /*!< Error accessing the SD Card */
    SENSOR_ERROR                     = 0xEB, /*!< Sensor go wrong */
    SYSTEM_ERROR                     = 0xEC, /*!< System error */
    PARAMETER_TOTAL_TOO_LONG         = 0xED, /*!< Length of the parameter is too long */
    MODULE_INACTIVATED               = 0xEE, /*!< Module is too not activated yet */
    USER_UNBOND                      = 0xEF, /*!< User is not bond yet */
    FIRMWARE_DATA_NUM_DISCONTINUOUS  = 0xF0, /*!< Fireware data number is a discontinuous number */
    FIRMWARE_DATA_OVERLOAD_FLASH     = 0xF1, /*!< Fireware data number overload flash */
    FIRMWARE_VERIFICATION_ERROR      = 0xF2, /*!< Error verifying fireware */
    FLASH_ERASE_ERROR                = 0xF3, /*!< Error erasing flash */
    FLASH_WRITE_ERROR                = 0xF4, /*!< Error writing flash */
    UPGRADE_STATUS_ERROR             = 0xF5, /*!< Error status of upgrading */
    FIRMWARE_TYPE_MISMATCH           = 0xF6, /*!< Firmware type don't match */
    WAITING_CLIENT_UPGRADE_STATUS    = 0xF7, /*!< Upgrade host need the upgrade status pushing from Client (CmdID = 0x41) */
    REMOTE_CONTROL_UNCONNECTED       = 0xF8, /*!< Not connect remote control yet */
    MOTOR_NOT_STOPPED                = 0xF9, /*!< Motor is not stopped yet */
    HARDWARE_ERROR                   = 0xFA, /*!< Hardware fault */
    INSUFFICIENT_ELECTRICITY         = 0xFB, /*!< Device is of insufficient electricity */
    AIRCRAFT_UNCONNECTED             = 0xFC, /*!< Aircraft is not connected yet */
    FLASH_IS_ERASING                 = 0xFD, /*!< Flash is erasing (Avoid APP waiting timeout) */
    CANNOT_UPGRADE_IN_CUR_STATE      = 0xFE, /*!< Cannot upgrade in current status (Please reboot or contact with DJI support */
    UNDEFINE_ERROR                   = 0xFF, /*!< Undefined error */
  };
 private:
  static const uint8_t moduleIDLeftMove = 40;
  static const uint8_t functionIDLeftMove = 32;

  /*! @brief The err code message data of the PSDKCommonErr error code messages.
   */
  static const std::pair<const ErrorCode::ErrorCodeType, ErrorCode::ErrorCodeMsg> PSDKCommonErrData[];

  /*! @brief Get the map container of the PSDKCommonErr error code messages.
   */
  static const ErrorCodeMapType getPSDKCommonErrorMap();

  /*! @brief The err code message data of the CameraCommonErr error code messages.
   */
  static const std::pair<const ErrorCode::ErrorCodeType, ErrorCode::ErrorCodeMsg> CameraCommonErrData[];

  /*! @brief Get the map container of the CameraCommonErr error code messages.
   */
  static const ErrorCodeMapType getCameraCommonErrorMap();

  /*! @brief The err code message data of the GimbalCommonErr error code messages.
   */
  static const std::pair<const ErrorCode::ErrorCodeType, ErrorCode::ErrorCodeMsg> GimbalCommonErrData[];

  /*! @brief Get the map container of the GimbalCommonErr error code messages.
   */
  static const ErrorCodeMapType getGimbalCommonErrorMap();

  /*! @brief The err code message data of the SystemCommonErr error code messages.
   */
  static const std::pair<const ErrorCode::ErrorCodeType, ErrorCode::ErrorCodeMsg> SystemCommonErrData[];

  /*! @brief The err code message data of the WaypointV2 error code messages.
 */
  static const ErrorCodeMapType getWaypointV2CommonErrorMap();

  static const std::pair<const ErrorCode::ErrorCodeType, ErrorCode::ErrorCodeMsg> WaypointV2CommonErrData[];


  /*! @brief Get the map container of the SystemCommonErr error code messages.
   */
  static const ErrorCodeMapType getSystemCommonErrorMap();

  /*! @brief The array to contain all the function maps of gimbal.
   */
  static const FunctionDataType GimbalFunction[functionMaxCnt];

  /*! @brief The array to contain all the function maps of camera.
   */
  static const FunctionDataType CameraFunction[functionMaxCnt];

  /*! @brief The array to contain all the function maps of psdk.
   */
  static const FunctionDataType PSDKFunction[functionMaxCnt];

  /*! @brief The array to contain all the function maps of System.
   */
  static const FunctionDataType SystemFunction[functionMaxCnt];

  static const FunctionDataType WaypointV2Function[functionMaxCnt];
  /*! @brief The array to contain all the modules' function data.
   */
  static const ModuleDataType module[ModuleMaxCnt];


};  // Class ErrorCode

}  // namespace OSDK
}  // namespace DJI

#endif /* DJI_ERROR_H */
