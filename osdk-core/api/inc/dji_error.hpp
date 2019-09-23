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
    FCControlTask    = 0,
    FCSubscribe      = 1,
    FCMission        = 2,
    FCParameterTable = 4,
    FCSetHomeLocation= 5,
    FCAvoidObstacle  = 6,
  };

  /*! @brief Function ID used with CameraModule in error codes
   */
  enum CameraFunctionID {
    CameraCommon = 0,
  };

  /*! @brief Function ID used with PSDKModule in error codes
   */
  enum PSDKFunctionID {
    PSDKCommon = 0,
  };

  enum SYSTEM_ERROR_RAW_CODE {
    Success              = 0x00000000, /*!< Execute successfully */
    AllocMemoryFailed    = 0x00000001, /*!< Out of memory */
    ReqNotSupported      = 0x00000002, /*!< Request not supported */
    Timeout              = 0x00000003, /*!< Execute timeout */
    UnpackDataMismatch   = 0x00000004, /*!< Pack unpack failed */
    InstInitParamInvalid = 0x00000005, /*!< Instance init parameter invalid */
    UserCallbackInvalid  = 0x00000006, /*!< User Callback is a invalid value */
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

  /*! @brief The err code message data of the SystemCommonErr error code messages.
   */
  static const std::pair<const ErrorCode::ErrorCodeType, ErrorCode::ErrorCodeMsg> SystemCommonErrData[];

  /*! @brief Get the map container of the SystemCommonErr error code messages.
   */
  static const ErrorCodeMapType getSystemCommonErrorMap();

  /*! @brief The array to contain all the function maps of camera.
   */
  static const FunctionDataType CameraFunction[functionMaxCnt];

  /*! @brief The array to contain all the function maps of psdk.
   */
  static const FunctionDataType PSDKFunction[functionMaxCnt];

  /*! @brief The array to contain all the function maps of System.
   */
  static const FunctionDataType SystemFunction[functionMaxCnt];

  /*! @brief The array to contain all the modules' function data.
   */
  static const ModuleDataType module[ModuleMaxCnt];


};  // Class ErrorCode

}  // namespace OSDK
}  // namespace DJI

#endif /* DJI_ERROR_H */
