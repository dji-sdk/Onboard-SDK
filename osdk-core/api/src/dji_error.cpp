/** @file dji_error.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief All DJI OSDK OpenProtocol ACK Error Codes
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

#ifdef STM32
#include <stdio.h>
#else
#include <cstdio>
#endif
#include "dji_error.hpp"
#include "dji_log.hpp"
#include "dji_type.hpp"

using namespace DJI::OSDK;

// clang-format off
const uint16_t DJI::OSDK::ErrorCode::CommonACK::SUCCESS                = 0x0000;
const uint16_t DJI::OSDK::ErrorCode::CommonACK::KEY_ERROR              = 0xFF00;
const uint16_t DJI::OSDK::ErrorCode::CommonACK::NO_AUTHORIZATION_ERROR = 0xFF01;
const uint16_t DJI::OSDK::ErrorCode::CommonACK::NO_RIGHTS_ERROR        = 0xFF02;
const uint16_t DJI::OSDK::ErrorCode::CommonACK::SYSTEM_ERROR           = 0xFF03;
const uint16_t DJI::OSDK::ErrorCode::CommonACK::NO_RESPONSE_ERROR      = 0xFFFF;
const uint8_t  DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_NONE        = 0;
const uint8_t  DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_COMPASS_ABNORMAL = 1;
const uint8_t  DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_ASSISTANT_PROTECTED = 2;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_DEVICE_LOCKED = 3;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_IMU_NEED_ADV_CALIBRATION = 5;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_IMU_SN_ERROR   = 6;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_IMU_PREHEATING = 7;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_COMPASS_CALIBRATING = 8;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_IMU_NO_ATTITUDE = 9;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_NO_GPS_IN_NOVICE_MODE = 10;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_BATTERY_CELL_ERROR = 11;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_BATTERY_COMMUNICATION_ERROR = 12;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_BATTERY_VOLTAGE_TOO_LOW = 13;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_BATTERY_USER_LOW_LAND = 14;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_BATTERY_MAIN_VOL_LOW = 15;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_BATTERY_TEMP_VOL_LOW = 16;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_BATTERY_SMART_LOW_LAND = 17;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_BATTERY_NOT_READY = 18;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RUNNING_SIMULATOR = 19;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_PACK_MODE        = 20;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_IMU_ATTI_LIMIT   = 21;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_NOT_ACTIVATED    = 22;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_IN_FLYLIMIT_AREA = 23;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_IMU_BIAS_LIMIT   = 24;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_ESC_ERROR        = 25;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_IMU_INITING      = 26;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_UPGRADING        = 27;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_HAVE_RUN_SIM     = 28;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_IMU_CALIBRATING  = 29;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_TAKEOFF_TILT_TOO_LARGE = 30;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_31      = 31;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_32      = 32;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_33      = 33;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_34      = 34;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_35      = 35;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_36      = 36;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_37      = 37;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_38      = 38;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_39      = 39;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_40      = 40;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_INVALID_SN       = 41;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_42      = 42;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_43      = 43;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_FLASH_OPERATING  = 44;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_GPS_DISCONNECT   = 45;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_INTERNAL_46      = 46;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RECORDER_ERROR   = 47;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_INVALID_PRODUCT  = 48;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_49      = 49;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_50      = 50;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_51      = 51;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_52      = 52;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_53      = 53;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_54      = 54;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_55      = 55;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_56      = 56;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_57      = 57;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_58      = 58;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_59      = 59;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_60      = 60;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_IMU_DISCONNECTED = 61;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RC_CALIBRATING   = 62;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RC_CALI_DATA_OUT_RANGE = 63;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RC_QUIT_CALI = 64;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RC_CENTER_OUT_RANGE = 65;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RC_MAP_ERROR = 66;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_WRONG_AIRCRAFT_TYPE = 67;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_SOME_MODULE_NOT_CONFIGURED = 68;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_69 = 69;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_70 = 70;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_71 = 71;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_72 = 72;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_73 = 73;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_NS_ABNORMAL = 74;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_TOPOLOGY_ABNORMAL = 75;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RC_NEED_CALI     = 76;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_INVALID_FLOAT    = 77;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_M600_BAT_TOO_FEW = 78;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_M600_BAT_AUTH_ERR = 79;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_M600_BAT_COMM_ERR = 80;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_M600_BAT_DIF_VOLT_LARGE_1 = 81;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_BATTERY_BOLTAHGE_DIFF_82 = 82;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_INVALID_VERSION = 83;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_GYRO_ABNORMAL = 84;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_ESC_PITCH_NO_DATA = 85;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_ESC_ROLL_NO_DATA = 86;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_ESC_YAW_NO_DATA = 87;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_FIRM_IS_UPDATING = 88;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_OUT_OF_CONTROL = 89;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_PITCH_SHOCK = 90;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_ROLL_SHOCK = 91;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_YAW_SHOCK = 92;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_IMU_CALI_SUCCESS = 93;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_TAKEOFF_EXCEPTION = 94;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_ESC_STALL_NEAR_GOUND = 95;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_ESC_UNBALANCE_ON_GRD = 96;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_ESC_PART_EMPTY_ON_GRD = 97;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_ENGINE_START_FAILED = 98;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_AUTO_TAKEOFF_LAUNCH_FAILED = 99;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_ROLL_OVER_ON_GRD = 100;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_BAT_VERSION_ERR = 101;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RTK_INITING     = 102;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RTK_FAIL_TO_INIT = 103;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_104 = 104;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_105 = 105;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_106 = 106;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_107 = 107;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_108 = 108;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_RESERVED_109 = 109;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::START_MOTOR_FAIL_MOTOR_STARTED = 110;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_INTERNAL_111    = 111;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_ESC_CALIBRATING = 112;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_GPS_SIGNATURE_INVALID = 113;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_GIMBAL_CALIBRATING = 114;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_FORCE_DISABLE = 115;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::TAKEOFF_HEIGHT_EXCEPTION = 116;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_ESC_NEED_UPGRADE = 117;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_GYRO_DATA_NOT_MATCH = 118;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_APP_NOT_ALLOW = 119;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_COMPASS_IMU_MISALIGN = 120;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_FLASH_UNLOCK  = 121;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_ESC_SCREAMING = 122;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_ESC_TEMP_HIGH = 123;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_BAT_ERR       = 124;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::IMPACT_IS_DETECTED       = 125;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_MODE_FAILURE  = 126;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_CRAFT_FAIL_LATELY = 127;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::KILL_SWITCH_ON           = 135;
const uint8_t DJI::OSDK::ErrorCode::CommonACK::MOTOR_FAIL_MOTOR_CODE_ERROR = 255;
const uint16_t DJI::OSDK::ErrorCode::ActivationACK::SUCCESS          = 0x0000;
const uint16_t DJI::OSDK::ErrorCode::ActivationACK::PARAMETER_ERROR  = 0x0001;
const uint16_t DJI::OSDK::ErrorCode::ActivationACK::ENCODE_ERROR     = 0x0002;
const uint16_t DJI::OSDK::ErrorCode::ActivationACK::NEW_DEVICE_ERROR = 0x0003;
const uint16_t DJI::OSDK::ErrorCode::ActivationACK::DJIGO_APP_NOT_CONNECTED = 0x0004;
const uint16_t DJI::OSDK::ErrorCode::ActivationACK::NETWORK_ERROR = 0x0005;
const uint16_t DJI::OSDK::ErrorCode::ActivationACK::SERVER_ACCESS_REFUSED = 0x0006;
const uint16_t DJI::OSDK::ErrorCode::ActivationACK::ACCESS_LEVEL_ERROR = 0x0007;
const uint16_t DJI::OSDK::ErrorCode::ActivationACK::OSDK_VERSION_ERROR = 0x0008;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::SetControl::RC_MODE_ERROR = 0x0000;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::SetControl::RELEASE_CONTROL_SUCCESS = 0x0001;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::SetControl::OBTAIN_CONTROL_SUCCESS = 0x0002;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::SetControl::OBTAIN_CONTROL_IN_PROGRESS = 0x0003;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::SetControl::RELEASE_CONTROL_IN_PROGRESS = 0x0004;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::SetControl::RC_NEED_MODE_F = 0x0006;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::SetControl::RC_NEED_MODE_P = 0x0005;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::SetControl::IOC_OBTAIN_CONTROL_ERROR = 0x00C9;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::SUCCESS      = 0;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::MOTOR_ON     = 1;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::MOTOR_OFF    = 2;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::IN_AIR       = 3;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::NOT_IN_AIR   = 4;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::NO_HOMEPOINT = 5;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::BAD_GPS      = 6;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::IN_SIMULATOR_MODE = 7;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::ALREADY_RUNNING   = 8;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::NOT_RUNNING       = 9;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::INVAILD_COMMAND   = 10;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::NO_LANDING_GEAR   = 11;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::GIMBAL_MOUNTED    = 12;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::BAD_SENSOR        = 13;/*PACK_US_NOT_HEALTH*/
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::ALREADY_PACKED    = 14;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::NO_PACKED         = 15;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::LegacyTask::SUCCESS  = 0x0002;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::LegacyTask::FAIL     = 0x0001;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::Task::PACKED_MODE_NOT_SUPPORTED = 0x0016;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::SetArm::AIRCRAFT_IN_AIR_ERROR       = 0x0003;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::SetArm::ALREADY_ARMED_ERROR 	     = 0x0002;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::SetArm::OBTAIN_CONTROL_NEEDED_ERROR = 0x0001;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::SetArm::SUCCESS 		     = 0x0000;
const uint16_t DJI::OSDK::ErrorCode::ControlACK::KillSwitch::SUCCESS 		 = 0x00;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::SUCCESS                = 0x00;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::ILLEGAL_DATA_LENGTH    = 0x01;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::VERSION_DOES_NOT_MATCH = 0x02;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::PACKAGE_OUT_OF_RANGE   = 0x03;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::PACKAGE_ALREADY_EXISTS = 0x04;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::PACKAGE_DOES_NOT_EXIST = 0x05;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::ILLEGAL_FREQUENCY      = 0x06;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::PACKAGE_TOO_LARGE      = 0x07;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::PIPELINE_OVERFLOW      = 0x08;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::INTERNAL_ERROR_0X09    = 0x09;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::PACKAGE_EMPTY          = 0x20;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::INCORRECT_NUM_OF_TOPICS = 0x21;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::ILLEGAL_UID          = 0x22;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::PERMISSION_DENY      = 0x23;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::MULTIPLE_SUBSCRIBE   = 0x24;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::SOURCE_DEVICE_OFFLINE = 0x25;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::PAUSED               = 0x48;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::RESUMED              = 0x49;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::INTERNAL_ERROR_0X4A  = 0x4A;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::FAILED_AUTHENTICATION  = 0x50;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::VERSION_VERSION_TOO_FAR = 0x51;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::VERSION_UNKNOWN_ERROR = 0x59;
const uint8_t DJI::OSDK::ErrorCode::SubscribeACK::INTERNAL_ERROR_0XFF   = 0xFF;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::SUCCESS         = 0x00;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::WRONG_WAYPOINT_INDEX = 0x01;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::RC_NOT_IN_MODE_F = 0xD0;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::OBTAIN_CONTROL_REQUIRED = 0xD1;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::CLOSE_IOC_REQUIRED = 0xD2;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::NOT_INITIALIZED = 0xD3;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::NOT_RUNNING     = 0xD4;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::IN_PROGRESS     = 0xD5;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::TASK_TIMEOUT    = 0xD6;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::OTHER_MISSION_RUNNING = 0xD7;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::BAD_GPS     = 0xD8;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::RTK_NOT_READY     = 0xCD;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::LOW_BATTERY = 0xD9;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::VEHICLE_DID_NOT_TAKE_OFF = 0xDA;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::INVALID_PARAMETER = 0xDB;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::CONDITIONS_NOT_SATISFIED = 0xDC;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::CROSSING_NO_FLY_ZONE = 0xDD;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::UNRECORDED_HOME = 0xDE;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::AT_NO_FLY_ZONE  = 0xDF;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::TOO_HIGH        = 0xC0;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::TOO_LOW         = 0xC1;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::TOO_FAR_FROM_HOME = 0xC7;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::NOT_SUPPORTED = 0xC8;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::TOO_FAR_FROM_CURRENT_POSITION = 0xC9;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::BEGGINER_MODE_NOT_SUPPORTED = 0xCA;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::TAKE_OFF_IN_PROGRESS = 0xF0;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::LANDING_IN_PROGRESS = 0xF1;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::RRETURN_HOME_IN_PROGRESS = 0xF2;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::START_MOTORS_IN_PROGRESS = 0xF3;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::INVALID_COMMAND = 0xF4;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Common::UNKNOWN_ERROR   = 0xFF;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Follow::TOO_FAR_FROM_YOUR_POSITION_LACK_OF_RADIO_CONNECTION = 0xB0;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Follow::CUTOFF_TIME_OVERFLOW = 0xB1;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::Follow::GIMBAL_PITCH_ANGLE_OVERFLOW = 0xB2;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::HotPoint::INVALID_RADIUS = 0xC2;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::HotPoint::YAW_RATE_OVERFLOW = 0xC3;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::HotPoint::INVALID_START_POINT = 0xC4;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::HotPoint::INVALID_YAW_MODE = 0xC5;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::HotPoint::TOO_FAR_FROM_HOTPOINT = 0xC6;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::HotPoint::INVALID_PAREMETER = 0xA2;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::HotPoint::INVALID_LATITUDE_OR_LONGITUTE = 0xA3;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::HotPoint::INVALID_DIRECTION = 0xA6;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::HotPoint::IN_PAUSED_MODE = 0xA9;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::HotPoint::FAILED_TO_PAUSE = 0xAA;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::WayPoint::INVALID_DATA = 0xE0;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::WayPoint::INVALID_POINT_DATA = 0xE1;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::WayPoint::TRACE_TOO_LONG = 0xE2; /*!< This is the distance from WP1 to WPn. Newer A3/N3 FW > 60km; all others > 30km >*/
const uint8_t DJI::OSDK::ErrorCode::MissionACK::WayPoint::TOTAL_DISTANCE_TOO_LONG        = 0xE3; /*!< This is the distance from WP1 to WPn, plus distance from homepoint to WP1 and the distance from WPn back to the homepoint. Newer A3/N3 FW > 100km; all others > 60km >*/
const uint8_t DJI::OSDK::ErrorCode::MissionACK::WayPoint::POINT_OVERFLOW = 0xE4;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::WayPoint::POINTS_TOO_CLOSE = 0xE5;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::WayPoint::POINTS_TOO_FAR = 0xE6;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::WayPoint::CHECK_FAILED   = 0xE7;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::WayPoint::INVALID_ACTION = 0xE8;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::WayPoint::POINT_DATA_NOT_ENOUGH = 0xE9;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::WayPoint::DATA_NOT_ENOUGH = 0xEA;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::WayPoint::POINTS_NOT_ENOUGH = 0xEB;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::WayPoint::IN_PROGRESS = 0xEC;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::WayPoint::NOT_IN_PROGRESS = 0xED;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::WayPoint::INVALID_VELOCITY = 0xEE;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::IOC::TOO_CLOSE_TO_HOME  = 0xA0;
const uint8_t DJI::OSDK::ErrorCode::MissionACK::IOC::UNKNOWN_TYPE       = 0xA1;
const uint8_t DJI::OSDK::ErrorCode::MFIOACK::init::SUCCESS              = 0x00;
const uint8_t DJI::OSDK::ErrorCode::MFIOACK::init::UNKNOWN_ERROR        = 0x01;
const uint8_t DJI::OSDK::ErrorCode::MFIOACK::init::PORT_NUMBER_ERROR    = 0x02;
const uint8_t DJI::OSDK::ErrorCode::MFIOACK::init::PORT_MODE_ERROR      = 0x03;
const uint8_t DJI::OSDK::ErrorCode::MFIOACK::init::PORT_DATA_ERROR      = 0x04;
const uint8_t DJI::OSDK::ErrorCode::MFIOACK::set::SUCCESS               = 0x00;
const uint8_t DJI::OSDK::ErrorCode::MFIOACK::set::CHANNEL_ERROR         = 0x01;
const uint8_t DJI::OSDK::ErrorCode::MFIOACK::set::PORT_NOT_MAPPED_ERROR = 0x02;
const uint8_t DJI::OSDK::ErrorCode::MFIOACK::get::SUCCESS               = 0x00;
// clang-format on

// Backward compatibility

const uint8_t& DJI::OSDK::ErrorCode::MissionACK::WayPoint::DISTANCE_OVERFLOW = DJI::OSDK::ErrorCode::MissionACK::WayPoint::TRACE_TOO_LONG;
const uint8_t& DJI::OSDK::ErrorCode::MissionACK::WayPoint::TIMEOUT = DJI::OSDK::ErrorCode::MissionACK::WayPoint::TOTAL_DISTANCE_TOO_LONG;



// clang-format off
/*! flight controller parameter table read and write error code*/
const ErrorCode::ErrorCodeType ErrorCode::FlightControllerErr::ParamReadWriteErr::Fail = ErrorCode::getErrorCode(FCModule,  FCParameterTable, ControlACK::ParamReadWrite::PARAM_READ_WRITE_FAIL);
const ErrorCode::ErrorCodeType ErrorCode::FlightControllerErr::ParamReadWriteErr::InvalidParameter = ErrorCode::getErrorCode(FCModule,  FCParameterTable, ControlACK::ParamReadWrite::PARAM_READ_WRITE_INVALID_PARAMETER);
const ErrorCode::ErrorCodeType ErrorCode::FlightControllerErr::SetHomeLocationErr::Fail   = ErrorCode::getErrorCode(FCModule,  FCSetHomeLocation, ErrorCode::ControlACK::SetHomeLocation::SET_HOME_LOCATION_FAIL);

const ErrorCode::ModuleDataType ErrorCode::module[ModuleMaxCnt] = {
    {"System",     SystemFunction}, /*!< SysModule */
    {"Reserve_1",  NULL},           /*!< RESERVE_1 */
    {"Reserve_2",  NULL},           /*!< RESERVE_2 */
    {"Reserve_3",  NULL},           /*!< RESERVE_3 */
    {"Reserve_4",  NULL},           /*!< RESERVE_4 */
    {"Reserve_5",  NULL},           /*!< RESERVE_5 */
    {"Reserve_6",  NULL},           /*!< RESERVE_6 */
    {"Reserve_7",  NULL},           /*!< RESERVE_7 */
    {"Reserve_8",  NULL},           /*!< RESERVE_8 */
    {"Reserve_9",  NULL},           /*!< RESERVE_9 */
    {"Reserve_10", NULL},           /*!< RESERVE_10 */
    {"FC",         NULL},           /*!< FCModule */
    {"Gimbal",     GimbalFunction}, /*!< GimbalModule */
    {"Camera",     CameraFunction}, /*!< CameraModule */
    {"PSDK",       PSDKFunction},   /*!< PSDKModule */
    {"RC",         NULL},           /*!< RCModule */
    {"Battery",    NULL},           /*!< BatteryModule */
    {"WaypointV2", WaypointV2Function},/*! WaypointV2Module*/
};

/*! camera api error code */
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::InvalidCMD         = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::UNSUPPORTED_COMMAND);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::Timeout            = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::TIMEOUT);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::OutOfMemory        = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::RAM_ALLOCATION_FAILED);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::InvalidParam       = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::INVALID_COMMAND_PARAMETER);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::InvalidState       = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::UNSUPPORTED_COMMAND_IN_CUR_STATE);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::TimeNotSync        = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::CAMERA_TIME_NOT_SYNCHRONIZED);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::ParamSetFailed     = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::PARAMETER_SET_FAILED);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::ParamGetFailed     = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::PARAMETER_GET_FAILED);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::SDCardMISSING      = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::SD_CARD_MISSING);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::SDCardFull         = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::SD_CARD_FULL);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::SDCardError        = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::SD_CARD_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::SensorError        = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::SENSOR_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::SystemError        = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::SYSTEM_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::ParamLenTooLong    = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::PARAMETER_TOTAL_TOO_LONG);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::ModuleInactivated  = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::MODULE_INACTIVATED);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::FWSeqNumNotInOrder = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::FIRMWARE_DATA_NUM_DISCONTINUOUS);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::FWCheckErr         = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::FIRMWARE_VERIFICATION_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::FlashWriteError    = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::FLASH_WRITE_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::FWInvalidType      = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::FIRMWARE_TYPE_MISMATCH);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::RCDisconnect       = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::REMOTE_CONTROL_UNCONNECTED);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::HardwareErr        = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::HARDWARE_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::UAVDisconnect      = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::AIRCRAFT_UNCONNECTED);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::UpgradeErrorNow    = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::CANNOT_UPGRADE_IN_CUR_STATE);
const ErrorCode::ErrorCodeType ErrorCode::GimbalCommonErr::UndefineError      = ErrorCode::getErrorCode(GimbalModule, GimbalCommon, DJI_CMD_RETURN_CODE::UNDEFINE_ERROR);

/*! gimbal api error code */
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::InvalidCMD         = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::UNSUPPORTED_COMMAND);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::Timeout            = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::TIMEOUT);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::OutOfMemory        = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::RAM_ALLOCATION_FAILED);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::InvalidParam       = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::INVALID_COMMAND_PARAMETER);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::InvalidState       = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::UNSUPPORTED_COMMAND_IN_CUR_STATE);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::TimeNotSync        = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::CAMERA_TIME_NOT_SYNCHRONIZED);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::ParamSetFailed     = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::PARAMETER_SET_FAILED);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::ParamGetFailed     = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::PARAMETER_GET_FAILED);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::SDCardMISSING      = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::SD_CARD_MISSING);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::SDCardFull         = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::SD_CARD_FULL);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::SDCardError        = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::SD_CARD_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::SensorError        = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::SENSOR_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::SystemError        = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::SYSTEM_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::ParamLenTooLong    = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::PARAMETER_TOTAL_TOO_LONG);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::ModuleInactivated  = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::MODULE_INACTIVATED);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::FWSeqNumNotInOrder = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::FIRMWARE_DATA_NUM_DISCONTINUOUS);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::FWCheckErr         = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::FIRMWARE_VERIFICATION_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::FlashWriteError    = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::FLASH_WRITE_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::FWInvalidType      = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::FIRMWARE_TYPE_MISMATCH);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::RCDisconnect       = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::REMOTE_CONTROL_UNCONNECTED);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::HardwareErr        = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::HARDWARE_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::UAVDisconnect      = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::AIRCRAFT_UNCONNECTED);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::UpgradeErrorNow    = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::CANNOT_UPGRADE_IN_CUR_STATE);
const ErrorCode::ErrorCodeType ErrorCode::CameraCommonErr::UndefineError      = ErrorCode::getErrorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::UNDEFINE_ERROR);

/*! PSDK api error code */
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::InvalidCMD         = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::UNSUPPORTED_COMMAND);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::Timeout            = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::TIMEOUT);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::OutOfMemory        = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::RAM_ALLOCATION_FAILED);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::InvalidParam       = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::INVALID_COMMAND_PARAMETER);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::InvalidState       = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::UNSUPPORTED_COMMAND_IN_CUR_STATE);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::TimeNotSync        = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::CAMERA_TIME_NOT_SYNCHRONIZED);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::ParamSetFailed     = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::PARAMETER_SET_FAILED);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::ParamGetFailed     = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::PARAMETER_GET_FAILED);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::SDCardMISSING      = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::SD_CARD_MISSING);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::SDCardFull         = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::SD_CARD_FULL);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::SDCardError        = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::SD_CARD_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::SensorError        = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::SENSOR_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::SystemError        = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::SYSTEM_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::ParamLenTooLong    = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::PARAMETER_TOTAL_TOO_LONG);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::ModuleInactivated  = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::MODULE_INACTIVATED);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::FWSeqNumNotInOrder = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::FIRMWARE_DATA_NUM_DISCONTINUOUS);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::FWCheckErr         = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::FIRMWARE_VERIFICATION_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::FlashWriteError    = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::FLASH_WRITE_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::FWInvalidType      = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::FIRMWARE_TYPE_MISMATCH);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::RCDisconnect       = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::REMOTE_CONTROL_UNCONNECTED);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::HardwareErr        = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::HARDWARE_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::UAVDisconnect      = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::AIRCRAFT_UNCONNECTED);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::UpgradeErrorNow    = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::CANNOT_UPGRADE_IN_CUR_STATE);
const ErrorCode::ErrorCodeType ErrorCode::PSDKCommonErr::UndefineError      = ErrorCode::getErrorCode(PSDKModule, PSDKCommon, DJI_CMD_RETURN_CODE::UNDEFINE_ERROR);

/*! WaypointV2 Mission api error code */
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::COMMON_SUCCESS                            = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeCommon::GS_ERR_CODE_COMMON_SUCCESS);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::COMMON_INVALID_DATA_LENGTH                = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeCommon::GS_ERR_CODE_COMMON_INVALID_DATA_LENGTH);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::COMMON_INVALD_FLOAT_NUM                   = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeCommon::GS_ERR_CODE_COMMON_INVALD_FLOAT_NUM);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_WP_VERSION_NO_MATCH                  = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeCommon::GS_ERR_CODE_TRAJ_WP_VERSION_NO_MATCH);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::COMMON_UNKNOWN                            = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeCommon::GS_ERR_CODE_COMMON_UNKNOWN);

const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_RESV                                   = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_RESV);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_INIT_WP_NUM_TOO_MANY                   = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_INIT_WP_NUM_TOO_MANY);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_INIT_WP_NUM_TOO_FEW                    = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_INIT_WP_NUM_TOO_FEW  );
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_INIT_INVALID_END_INDEX                 = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_INIT_INVALID_END_INDEX );
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_UPLOAD_START_ID_GT_END_ID              = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_UPLOAD_START_ID_GT_END_ID);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_UPLOAD_END_ID_GT_TOTAL_NUM             = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_UPLOAD_END_ID_GT_TOTAL_NUM);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_DOWNLOAD_WPS_NOT_IN_STORED_RAGNE       = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_DOWNLOAD_WPS_NOT_IN_STORED_RAGNE);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_CUR_POS_IS_FAR_AWAY_FROM_FIRST_WP      = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_CUR_POS_IS_FAR_AWAY_FROM_FIRST_WP);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_ADJ_WPS_TOO_CLOSE                      = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_ADJ_WPS_TOO_CLOSE );
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_ADJ_WPS_TOO_FAR                        = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_ADJ_WPS_TOO_FAR  );
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_UPLOAD_MAX_VEL_GT_GLOBAL               = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_UPLOAD_MAX_VEL_GT_GLOBAL );
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_UPLOAD_LOCAL_CRUISE_VEL_GT_LOCAL_MAX   = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_UPLOAD_LOCAL_CRUISE_VEL_GT_LOCAL_MAX);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_UPLOAD_LOCAL_CRUISE_VEL_GT_GLOBAL_MAX  = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_UPLOAD_LOCAL_CRUISE_VEL_GT_GLOBAL_MAX);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_INIT_INVALID_GLOBAL_MAX_VEL            = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_INIT_INVALID_GLOBAL_MAX_VEL  );
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_GLOBAL_CRUISE_VEL_GT_MAX_VEL           = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_GLOBAL_CRUISE_VEL_GT_MAX_VEL );
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_INIT_INVALID_GOTO_FIRST_FLAG           = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_INIT_INVALID_GOTO_FIRST_FLAG );
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_INIT_INVALID_FINISHED_ACTION           = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_INIT_INVALID_FINISHED_ACTION );
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_INIT_INVALID_RC_LOST_ACTION            = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_INIT_INVALID_RC_LOST_ACTION);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_UPLOAD_YAW_MODE_INVALID                = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_UPLOAD_YAW_MODE_INVALID);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_UPLOAD_YAW_CMD_NOT_IN_RANGE            = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_UPLOAD_YAW_CMD_NOT_IN_RANGE);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_UPLOAD_YAW_TURN_DIRECTION_INVALID      = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_UPLOAD_YAW_TURN_DIRECTION_INVALID);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_UPLOAD_WP_TYPE_INVALID                 = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_UPLOAD_WP_TYPE_INVALID);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_GO_STOP_CMD_INVALID                    = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_GO_STOP_CMD_INVALID);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_INVALID_PAUSE_RECOVERY_CMD             = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_INVALID_PAUSE_RECOVERY_CMD);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_INVALID_BREAK_RESTORE_CMD              = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_INVALID_BREAK_RESTORE_CMD);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_INIT_INVALID_REF_POINT                 = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_INIT_INVALID_REF_POINT);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_UPLOAD_CANNT_SET_WP_LINE_EXIT_TYPE     = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_UPLOAD_CANNT_SET_WP_LINE_EXIT_TYPE);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_DAMPING_DIS_GE_DIS_OF_ADJ_POINTS       = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_DAMPING_DIS_GE_DIS_OF_ADJ_POINTS);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_INIT_INFO_NOT_UPLOADED                 = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_INIT_INFO_NOT_UPLOADED);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_WP_HAS_NOT_UPLOADED                    = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_WP_HAS_NOT_UPLOADED);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_UPLOADED_WP_NOT_ENOUGH                 = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_UPLOADED_WP_NOT_ENOUGH);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_GS_HAS_STARTED                         = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_GS_HAS_STARTED);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_GS_NOT_RUNNING                         = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_GS_NOT_RUNNING);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_GS_NOT_RUNNING_FOR_PAUSE_RECOVERY      = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_GS_NOT_RUNNING_FOR_PAUSE_RECOVERY);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_GS_NOT_RUNNING_FOR_BREAK_RESTORE       = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_GS_NOT_RUNNING_FOR_BREAK_RESTORE);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_NOT_IN_WP_MIS                          = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_NOT_IN_WP_MIS );
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_MIS_HAS_BEEN_PAUSED                    = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_MIS_HAS_BEEN_PAUSED);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_MIS_NOT_PAUSED                         = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_MIS_NOT_PAUSED);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_MIS_HAS_BEEN_BROKEN                    = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_MIS_HAS_BEEN_BROKEN);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_MIS_NOT_BROKEN                         = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_MIS_NOT_BROKEN);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_PAUSE_RECOVERY_NOT_SUPPORTED           = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_PAUSE_RECOVERY_NOT_SUPPORTED);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_BREAK_RESTORE_NOT_SUPPORTED            = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_BREAK_RESTORE_NOT_SUPPORTED);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_NO_BREAK_POINT                         = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_NO_BREAK_POINT );
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_NO_CUR_TRAJ_PROJECT                    = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_NO_CUR_TRAJ_PROJECT);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_NO_NXT_TRAJ_PROJECT                    = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_NO_NXT_TRAJ_PROJECT);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_NO_NNT_TRAJ_PROJECT                    = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_NO_NNT_TRAJ_PROJECT);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_UPLOAD_WP_ID_NOT_CONTINUE              = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_UPLOAD_WP_ID_NOT_CONTINUE);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_WP_LINE_ENTER_NOT_SET_TO_START_WP      = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_WP_LINE_ENTER_NOT_SET_TO_START_WP );
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_INIT_WP_WHEN_PLAN_HAS_STARTED          = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_INIT_WP_WHEN_PLAN_HAS_STARTED );
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_DAMPING_DIS_EXCEED_RANGE               = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_DAMPING_DIS_EXCEED_RANGE);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_WAYPOINT_COOR_EXCEED_RANGE             = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_WAYPOINT_COOR_EXCEED_RANGE);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_FIRST_WP_TYPE_IS_WP_TURN_NO            = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_FIRST_WP_TYPE_IS_WP_TURN_NO);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_WP_EXCEED_RADIUS_LIMIT                 = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_WP_EXCEED_RADIUS_LIMIT );
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRAJ_WP_EXCEED_HEIGHT_LIMIT                 = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTraj::GS_ERR_CODE_TRAJ_WP_EXCEED_HEIGHT_LIMIT );

const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::STATUS_RESV                                 = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeStatus::GS_ERR_CODE_STATUS_RESV);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::STATUS_WP_MIS_CHECK_FAIL                    = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeStatus::GS_ERR_CODE_STATUS_WP_MIS_CHECK_FAIL);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::STATUS_HOME_NOT_RECORDED                    = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeStatus::GS_ERR_CODE_STATUS_HOME_NOT_RECORDED);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::STATUS_LOW_LOCATION_ACCURACY                = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeStatus::GS_ERR_CODE_STATUS_LOW_LOCATION_ACCURACY);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::STATUS_RTK_CONDITION_IS_NOT_READY           = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeStatus::GS_ERR_CODE_STATUS_RTK_CONDITION_IS_NOT_READY);

const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::SECURE_RESV                                 = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeSecure::GS_ERR_CODE_SECURE_RESV);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::SECURE_CROSS_NFZ                            = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeSecure::GS_ERR_CODE_SECURE_CROSS_NFZ);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::SECURE_BAT_LOW                              = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeSecure::GS_ERR_CODE_SECURE_BAT_LOW);

const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTION_COMMON_RESV                           = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActionCommon::GS_ERR_CODE_ACTION_COMMON_RESV);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTION_COMMON_ACTION_ID_DUPLICATED           = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActionCommon::GS_ERR_CODE_ACTION_COMMON_ACTION_ID_DUPLICATED);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTION_COMMON_ACTION_ITEMS_SPACE_NOT_ENOUGH  = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActionCommon::GS_ERR_CODE_ACTION_COMMON_ACTION_ITEMS_SPACE_NOT_ENOUGH);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTION_COMMON_ACTION_SIZE_GT_BUF_SIZE        = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActionCommon::GS_ERR_CODE_ACTION_COMMON_ACTION_SIZE_GT_BUF_SIZE);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTION_COMMON_ACTION_ID_NOT_FOUND            = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActionCommon::GS_ERR_CODE_ACTION_COMMON_ACTION_ID_NOT_FOUND);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTION_COMMON_DOWNLOAD_ACTION_ID_RANGE_ERROR = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActionCommon::GS_ERR_CODE_ACTION_COMMON_DOWNLOAD_ACTION_ID_RANGE_ERROR);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTION_COMMON_NO_ACTION_ITEMS_STORED         = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActionCommon::GS_ERR_CODE_ACTION_COMMON_NO_ACTION_ITEMS_STORED);


const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRIGGER_RESV                                  = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTriggerModule::GS_ERR_CODE_TRIGGER_RESV);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRIGGER_TYPE_INVALID                          = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTriggerModule::GS_ERR_CODE_TRIGGER_TYPE_INVALID);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRIGGER_REACH_WP_END_INDEX_LT_START_INDEX     = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTriggerModule::GS_ERR_CODE_TRIGGER_REACH_WP_END_INDEX_LT_START_INDEX);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRIGGER_REACH_WP_INVALID_INTERVAL_WP_NUM      = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTriggerModule::GS_ERR_CODE_TRIGGER_REACH_WP_INVALID_INTERVAL_WP_NUM);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRIGGER_REACH_WP_INVALID_AUTO_TERMINATE_WP_NUM = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTriggerModule::GS_ERR_CODE_TRIGGER_REACH_WP_INVALID_AUTO_TERMINATE_WP_NUM);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRIGGER_ASSOCIATE_INVALID_TYPE                   = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTriggerModule::GS_ERR_CODE_TRIGGER_ASSOCIATE_INVALID_TYPE);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::TRIGGER_SIMPLE_INTERVAL_INVALID_TYPE          = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeTriggerModule::GS_ERR_CODE_TRIGGER_SIMPLE_INTERVAL_INVALID_TYPE);

const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_COMMON_RESV                         = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorCommon::GS_ERR_CODE_ACTUATOR_COMMON_RESV);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_COMMON_ACTUATOR_EXEC_NON_SUPPORTED  = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorCommon::GS_ERR_CODE_ACTUATOR_COMMON_ACTUATOR_EXEC_NON_SUPPORTED);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_COMMON_ACTUATOR_TYPE_INVALID        = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorCommon::GS_ERR_CODE_ACTUATOR_COMMON_ACTUATOR_TYPE_INVALID);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_COMMON_ACTUATOR_FUNC_INVALID        = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorCommon::GS_ERR_CODE_ACTUATOR_COMMON_ACTUATOR_FUNC_INVALID);

const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_CAMERA_RESV                               = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorCamera::GS_ERR_CODE_ACTUATOR_CAMERA_RESV);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_CAMERA_SEND_SINGLE_SHOT_CMD_TO_CAMERA_FAIL = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorCamera::GS_ERR_CODE_ACTUATOR_CAMERA_SEND_SINGLE_SHOT_CMD_TO_CAMERA_FAIL);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_CAMERA_SEND_VIDEO_START_CMD_TO_CAMERA_FAIL = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorCamera::GS_ERR_CODE_ACTUATOR_CAMERA_SEND_VIDEO_START_CMD_TO_CAMERA_FAIL);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_CAMERA_SEND_VIDEO_STOP_CMD_TO_CAMERA_FAIL  = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorCamera::GS_ERR_CODE_ACTUATOR_CAMERA_SEND_VIDEO_STOP_CMD_TO_CAMERA_FAIL);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_CAMERA_FOCUS_PARAM_XY_INVALID              = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorCamera::GS_ERR_CODE_ACTUATOR_CAMERA_FOCUS_PARAM_XY_INVALID);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_CAMERA_SEND_FOCUS_CMD_TO_CAMERA_FAIL       = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorCamera::GS_ERR_CODE_ACTUATOR_CAMERA_SEND_FOCUS_CMD_TO_CAMERA_FAIL);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_CAMERA_SEND_FOCALIZE_CMD_TO_CAMERA_FAIL    = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorCamera::GS_ERR_CODE_ACTUATOR_CAMERA_SEND_FOCALIZE_CMD_TO_CAMERA_FAIL);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_CAMERA_FOCAL_DISTANCE_INVALID              = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorCamera::GS_ERR_CODE_ACTUATOR_CAMERA_FOCAL_DISTANCE_INVALID);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_CAMERA_EXEC_FAIL                           = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorCamera::GS_ERR_CODE_ACTUATOR_CAMERA_EXEC_FAIL);

const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_GIMBAL_RESV                                        = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorGimbal::GS_ERR_CODE_ACTUATOR_GIMBAL_RESV);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_GIMBAL_INVALID_RPY_ANGLE_CTRL_CMD                  = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorGimbal::GS_ERR_CODE_ACTUATOR_GIMBAL_INVALID_RPY_ANGLE_CTRL_CMD);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_GIMBAL_INVALID_DURATION_CMD                        = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorGimbal::GS_ERR_CODE_ACTUATOR_GIMBAL_INVALID_DURATION_CMD);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_GIMBAL_FAIL_TO_ARRIVE_TGT_ANGLE                    = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorGimbal::GS_ERR_CODE_ACTUATOR_GIMBAL_FAIL_TO_ARRIVE_TGT_ANGLE);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_GIMBAL_FAIL_TO_SEND_CMD_TO_GIMBAL                  = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorGimbal::GS_ERR_CODE_ACTUATOR_GIMBAL_FAIL_TO_SEND_CMD_TO_GIMBAL);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_GIMBAL_THIS_INDEX_OF_GIMBAL_NOT_DOING_UNIFORM_CTRL = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorGimbal::GS_ERR_CODE_ACTUATOR_GIMBAL_THIS_INDEX_OF_GIMBAL_NOT_DOING_UNIFORM_CTRL);

const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_FLIGHT_RESV                                        = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorFlight::GS_ERR_CODE_ACTUATOR_FLIGHT_RESV);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_FLIGHT_YAW_INVALID_YAW_ANGLE                       = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorFlight::GS_ERR_CODE_ACTUATOR_FLIGHT_YAW_INVALID_YAW_ANGLE);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_FLIGHT_YAW_TO_TGT_ANGLE_TIMEOUT                    = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorFlight::GS_ERR_CODE_ACTUATOR_FLIGHT_YAW_TO_TGT_ANGLE_TIMEOUT);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_FLIGHT_ACTION_YAW_OCCUPIED                         = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorFlight::GS_ERR_CODE_ACTUATOR_FLIGHT_ACTION_YAW_OCCUPIED);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_FLIGHT_CUR_AND_TGT_VEL_CLE_STATUE_EQUAL            = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorFlight::GS_ERR_CODE_ACTUATOR_FLIGHT_CUR_AND_TGT_VEL_CLE_STATUE_EQUAL);

const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_PAYLOAD_RESV                        = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorPayload::GS_ERR_CODE_ACTUATOR_PAYLOAD_RESV);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_PAYLOAD_FAIL_TO_SEND_CMD_TO_PAYLOAD = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorPayload::GS_ERR_CODE_ACTUATOR_PAYLOAD_FAIL_TO_SEND_CMD_TO_PAYLOAD);
const ErrorCode::ErrorCodeType ErrorCode::WaypointV2MissionErr::ACTUATOR_PAYLOAD_EXEC_FAILED                 = ErrorCode::getErrorCode(MissionV2Module, MissionV2Common, WaypointV2ACK::WaypointV2ErrorCodeActuatorPayload::GS_ERR_CODE_ACTUATOR_PAYLOAD_EXEC_FAILED);


const std::pair<const ErrorCode::ErrorCodeType, ErrorCode::ErrorCodeMsg> ErrorCode::WaypointV2CommonErrData[] = {
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_RESV),                               ErrorCodeMsg(module[MissionV2Module].ModuleName,   "reserved", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_INIT_WP_NUM_TOO_MANY ),              ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"min_initial_waypoint_num is large than permitted_max_waypoint_num ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_INIT_WP_NUM_TOO_FEW   ),             ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"min_initial_waypoint_num is less than permitted_min_waypoint_num ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_INIT_INVALID_END_INDEX  ),           ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"waypoint_end_index is equal or large than total_waypoint_num ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_UPLOAD_START_ID_GT_END_ID ),         ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the start index is greater than end index of upload wps ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_UPLOAD_END_ID_GT_TOTAL_NUM ),        ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the end index of uplod wps is greater than inited total numbers ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_DOWNLOAD_WPS_NOT_IN_STORED_RAGNE ),  ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the index of first and end waypoint expected to download is not in range of stored in FC ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_CUR_POS_IS_FAR_AWAY_FROM_FIRST_WP ), ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"current position is far away from the first waypoint. ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_ADJ_WPS_TOO_CLOSE  ),                ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"it is too close from two adjacent waypoints","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_ADJ_WPS_TOO_FAR   ),                 ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the distance betwween two adjacent waypoints is not in[0.5m, 5000m]","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_UPLOAD_MAX_VEL_GT_GLOBAL  ),         ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the max vel of uplod wp is greater than global max vel ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_UPLOAD_LOCAL_CRUISE_VEL_GT_LOCAL_MAX ), ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the local cruise vel of upload wp is greater than local max vel ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_UPLOAD_LOCAL_CRUISE_VEL_GT_GLOBAL_MAX), ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the local cruise vel of upload wp is greater than global max vel ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_INIT_INVALID_GLOBAL_MAX_VEL   ),     ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"global_max_vel is greater than permitted_max_vel or less than permitted_min_vel ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_GLOBAL_CRUISE_VEL_GT_MAX_VEL  ),     ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"global_cruise_vel is greater than global_max_vel ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_INIT_INVALID_GOTO_FIRST_FLAG  ),     ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"goto_first_point_mode is out of range of waypoint_goto_first_flag_t_enum ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_INIT_INVALID_FINISHED_ACTION  ),     ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"finished_action is out of range of wp_plan_finish_action_t_enum ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_INIT_INVALID_RC_LOST_ACTION),        ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"rc_lost_action is out of range of wp_plan_rc_lost_action_t_enum ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_UPLOAD_YAW_MODE_INVALID),            ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the yaw mode of upload wp is invalid. reference to waypoint2_yaw_mode_t defined in math_waypoint_planner.h ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_UPLOAD_YAW_CMD_NOT_IN_RANGE ),       ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the yaw command of upload wp is not in range. the range for MR:[-180 180]","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_UPLOAD_YAW_TURN_DIRECTION_INVALID),  ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the yaw turn direction of upload wp is invalid. it should be 0:clockwise or 1:anti-clockwise ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_UPLOAD_WP_TYPE_INVALID ),            ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the wp type of upload wp is invalid. reference to waypoint_type_t defined in math_waypoint_planner.h ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_GO_STOP_CMD_INVALID ),               ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"go/stop command is invalid. ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_INVALID_PAUSE_RECOVERY_CMD),         ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the command of pause/recovery is not equal to any of the command enum ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_INVALID_BREAK_RESTORE_CMD ),         ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the command of break/restore is not equal to any of the command enum ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_INIT_INVALID_REF_POINT ),            ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"initial reference point position coordinate exceed set range ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_UPLOAD_CANNT_SET_WP_LINE_EXIT_TYPE), ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"cann't set wp_line_exit type to wp ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_DAMPING_DIS_GE_DIS_OF_ADJ_POINTS ),  ErrorCodeMsg(module[MissionV2Module].ModuleName, 	 "the damping dis is greater than or equal the distance of adjacent point ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_INIT_INFO_NOT_UPLOADED ),            ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the init info of Ground Station is not uploaded yet ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_WP_HAS_NOT_UPLOADED ),               ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the wp has not uploaded yet ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_UPLOADED_WP_NOT_ENOUGH ),            ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"min_initial_waypoint_num is not uploaded. ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_GS_HAS_STARTED ),                    ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"waypoint plan has started when received go command. ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_GS_NOT_RUNNING ),                    ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"waypoint plan not running when received stop command. ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_GS_NOT_RUNNING_FOR_PAUSE_RECOVERY),  ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"ground station(GS) is not started(used by pause/recovery) ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_GS_NOT_RUNNING_FOR_BREAK_RESTORE),   ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"ground station(GS) is not started(used by break/restore) ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_NOT_IN_WP_MIS  ),                    ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"not in the waypoint mission(MIS)(cannot pause/recovery or break/restore) ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_MIS_HAS_BEEN_PAUSED ),               ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the current status is paused","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_MIS_NOT_PAUSED ),                    ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"not in paused status","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_MIS_HAS_BEEN_BROKEN ),               ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the current status is broken","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_MIS_NOT_BROKEN ),                    ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"not in break status","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_PAUSE_RECOVERY_NOT_SUPPORTED),       ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the configuration forbid using pause/recovery API ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_BREAK_RESTORE_NOT_SUPPORTED ),       ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the configuration forbid using break/restore API ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_NO_BREAK_POINT  ),                   ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"no break point is recorded for restore ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_NO_CUR_TRAJ_PROJECT ),               ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"no current trajectory project point is recorded for restore ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_NO_NXT_TRAJ_PROJECT ),               ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"no next trajectory project point is recorded for restore ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_NO_NNT_TRAJ_PROJECT ),               ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"no next the next trajectory project point is recorded for restore ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_UPLOAD_WP_ID_NOT_CONTINUE ),         ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the index of upload wp is not continue after the store wp ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_WP_LINE_ENTER_NOT_SET_TO_START_WP ), ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the WP_LINE_ENTER wp_type set to a wp which is not the init start waypoint ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_INIT_WP_WHEN_PLAN_HAS_STARTED  ),    ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"the waypoint plan has started when initializing waypoint ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_DAMPING_DIS_EXCEED_RANGE ),          ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"waypoint damping distance exceed set range ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_WAYPOINT_COOR_EXCEED_RANGE ),        ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"waypoint position coordinate exceed rational range ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_FIRST_WP_TYPE_IS_WP_TURN_NO),        ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"first waypoint type error","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_WP_EXCEED_RADIUS_LIMIT  ),           ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"waypoint position exceed radius limit ","none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_WP_EXCEED_HEIGHT_LIMIT  ),           ErrorCodeMsg(module[MissionV2Module].ModuleName, 	"waypoint position exceed height limit ","none")),

  std::make_pair(getRawRetCode(WaypointV2MissionErr::COMMON_SUCCESS                            ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "				", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::COMMON_INVALID_DATA_LENGTH                ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "the length of the data is illegal based on the protocol ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::COMMON_INVALD_FLOAT_NUM                   ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "invalid float number (NAN or INF) ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRAJ_WP_VERSION_NO_MATCH                  ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "waypoint mission version can't match with firmware			", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::COMMON_UNKNOWN                            ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "Fatal error	 Unexpected result	 	", "none")),

  std::make_pair(getRawRetCode(WaypointV2MissionErr::STATUS_RESV                               ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "				", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::STATUS_WP_MIS_CHECK_FAIL                  ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "head_node is null or atti_not_healthy or gyro_not_healthy or horiz_vel_not healthy or horiz_abs_pos_not_healthy. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::STATUS_HOME_NOT_RECORDED                  ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "the home point is no recorded yet	 which will be executed at the first time of GPS level > 3(MR	FW). 	", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::STATUS_LOW_LOCATION_ACCURACY              ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "current location accuracy is low for bad GPS signal. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::STATUS_RTK_CONDITION_IS_NOT_READY         ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "use rtk_data	 but rtk is not connected or rtk_data is invalid 		", "none")),

  std::make_pair(getRawRetCode(WaypointV2MissionErr::SECURE_RESV                               ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "				", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::SECURE_CROSS_NFZ                          ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "the trajectory cross the NFZ ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::SECURE_BAT_LOW                            ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "current capacity of smart battery or voltage of non-smart battery is lower than level 1 or level 2 threshold ", "none")),

  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTION_COMMON_RESV                           ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "				", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTION_COMMON_ACTION_ID_DUPLICATED           ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "the ID of Action is duplicated. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTION_COMMON_ACTION_ITEMS_SPACE_NOT_ENOUGH  ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "there is no enough memory space for new Action Item ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTION_COMMON_ACTION_SIZE_GT_BUF_SIZE        ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "the size of buffer used to get the info of Action is less than the size of Action. Normally users can not get this. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTION_COMMON_ACTION_ID_NOT_FOUND            ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "the ID of Action is not found. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTION_COMMON_DOWNLOAD_ACTION_ID_RANGE_ERROR ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "the download action start id is bigger than the action end id ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTION_COMMON_NO_ACTION_ITEMS_STORED         ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "can not download or get min-max action ID for no action items stored in action kernel ", "none")),


  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRIGGER_RESV                                     ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "				", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRIGGER_TYPE_INVALID                             ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "the type ID of Trigger is invalid. It might not defined or the information is empty. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRIGGER_REACH_WP_END_INDEX_LT_START_INDEX        ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "wp_end_index is less than wp_start_index in reach_waypoint_trigger. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRIGGER_REACH_WP_INVALID_INTERVAL_WP_NUM         ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "interval_wp_num is large than the difference of wp_start_index and wp_end_index in reach_waypoint_trigger. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRIGGER_REACH_WP_INVALID_AUTO_TERMINATE_WP_NUM 	), ErrorCodeMsg(module[MissionV2Module].ModuleName, "auto_terminate_wp_num is large than interval_wp_num in reach_waypoint_trigger. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRIGGER_ASSOCIATE_INVALID_TYPE                   ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "the associate_type is greater than the maximum value. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::TRIGGER_SIMPLE_INTERVAL_INVALID_TYPE             ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "the interval type is greater than the maximum value. ", "none")),

  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_COMMON_RESV                             ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "				", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_COMMON_ACTUATOR_EXEC_NON_SUPPORTED      ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "the execution of Actuator is not supported	 e.g.	 try to stop camera shooting. 	", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_COMMON_ACTUATOR_TYPE_INVALID            ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "the type ID of Actuator is invalid. It might not defined or the information is empty. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_COMMON_ACTUATOR_FUNC_INVALID            ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "the Function ID of Actuator is invalid. It might not defined or the information is empty. ", "none")),

  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_CAMERA_RESV                               ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "				", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_CAMERA_SEND_SINGLE_SHOT_CMD_TO_CAMERA_FAIL), ErrorCodeMsg(module[MissionV2Module].ModuleName, "fail to send shot cmd to camera for no camera or camera is busy. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_CAMERA_SEND_VIDEO_START_CMD_TO_CAMERA_FAIL), ErrorCodeMsg(module[MissionV2Module].ModuleName, "fail to send video start cmd to camera for no camera or camera is busy. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_CAMERA_SEND_VIDEO_STOP_CMD_TO_CAMERA_FAIL  ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "fail to send video stop cmd to camera for no camera or camera is not busy. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_CAMERA_FOCUS_PARAM_XY_INVALID              ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "camera focus param xy exceed valid range (0	 1). 		", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_CAMERA_SEND_FOCUS_CMD_TO_CAMERA_FAIL       ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "fail to send focus cmd to camera for no camera or camera is busy. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_CAMERA_SEND_FOCALIZE_CMD_TO_CAMERA_FAIL    ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "fail to send focalize cmd to camera for no camera or camera is busy. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_CAMERA_FOCAL_DISTANCE_INVALID              ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "focal distance of camera focalize function exceed valid range. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_CAMERA_EXEC_FAIL                           ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "this err code indicate camera fail to exec coressponding cmd	 and the low 8 bit", "none")),

  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_GIMBAL_RESV                           ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "				", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_GIMBAL_INVALID_RPY_ANGLE_CTRL_CMD     ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "gimbal roll	pitch	yaw angle ctrl cmd param invalid	", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_GIMBAL_INVALID_DURATION_CMD           ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "gimbal duration param invalid	 unable to exec. 		", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_GIMBAL_FAIL_TO_ARRIVE_TGT_ANGLE       ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "gimbal fail to arrive target angle . ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_GIMBAL_FAIL_TO_SEND_CMD_TO_GIMBAL     ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "fail to send cmd to gimbal for gimbal is busy or no gimbal. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_GIMBAL_THIS_INDEX_OF_GIMBAL_NOT_DOING_UNIFORM_CTRL 	), ErrorCodeMsg(module[MissionV2Module].ModuleName, "fail to stop gimbal uniform ctrl because index error.			", "none")),

  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_FLIGHT_RESV                           ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "				", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_FLIGHT_YAW_INVALID_YAW_ANGLE          ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "yaw angle is lager max yaw angle. ", "none")),
  std::make_pair(getRawRetCode(WaypointV2MissionErr::ACTUATOR_FLIGHT_YAW_TO_TGT_ANGLE_TIMEOUT       ), ErrorCodeMsg(module[MissionV2Module].ModuleName, "faile to target yaw angle	 because of timeout.		", "none")),
  };

const ErrorCode::ErrorCodeMapType ErrorCode::getWaypointV2CommonErrorMap() {
  const ErrorCodeMapType WaypointV2CommonErrorMap(WaypointV2CommonErrData,
                                              WaypointV2CommonErrData
                                              + sizeof WaypointV2CommonErrData
                                                / sizeof WaypointV2CommonErrData[0]);
  return WaypointV2CommonErrorMap;
}
/*! system releated error code */
const ErrorCode::ErrorCodeType ErrorCode::SysCommonErr::Success              = ErrorCode::getErrorCode(SysModule, SystemCommon, SYSTEM_ERROR_RAW_CODE::Success);
const ErrorCode::ErrorCodeType ErrorCode::SysCommonErr::AllocMemoryFailed    = ErrorCode::getErrorCode(SysModule, SystemCommon, SYSTEM_ERROR_RAW_CODE::AllocMemoryFailed);
const ErrorCode::ErrorCodeType ErrorCode::SysCommonErr::ReqNotSupported      = ErrorCode::getErrorCode(SysModule, SystemCommon, SYSTEM_ERROR_RAW_CODE::ReqNotSupported);
const ErrorCode::ErrorCodeType ErrorCode::SysCommonErr::ReqTimeout           = ErrorCode::getErrorCode(SysModule, SystemCommon, SYSTEM_ERROR_RAW_CODE::Timeout);
const ErrorCode::ErrorCodeType ErrorCode::SysCommonErr::UnpackDataMismatch   = ErrorCode::getErrorCode(SysModule, SystemCommon, SYSTEM_ERROR_RAW_CODE::UnpackDataMismatch);
const ErrorCode::ErrorCodeType ErrorCode::SysCommonErr::InstInitParamInvalid = ErrorCode::getErrorCode(SysModule, SystemCommon, SYSTEM_ERROR_RAW_CODE::InstInitParamInvalid);
const ErrorCode::ErrorCodeType ErrorCode::SysCommonErr::UserCallbackInvalid  = ErrorCode::getErrorCode(SysModule, SystemCommon, SYSTEM_ERROR_RAW_CODE::UserCallbackInvalid);
const ErrorCode::ErrorCodeType ErrorCode::SysCommonErr::UndefinedError       = ErrorCode::getErrorCode(SysModule, SystemCommon, SYSTEM_ERROR_RAW_CODE::UndefinedError);

const std::pair<const ErrorCode::ErrorCodeType, ErrorCode::ErrorCodeMsg> ErrorCode::CameraCommonErrData[] = {
    std::make_pair(getRawRetCode(CameraCommonErr::InvalidCMD),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Command not supported", "Check the firmware or command validity")),
    std::make_pair(getRawRetCode(CameraCommonErr::Timeout),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Camera's execution of this action has timed out", "Try again or check the firmware or command")),
    std::make_pair(getRawRetCode(CameraCommonErr::OutOfMemory),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Camera's execution of this action is out of memory", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(CameraCommonErr::InvalidParam),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Camera received invalid parameters", "Check the validity of the parameter")),
    std::make_pair(getRawRetCode(CameraCommonErr::InvalidState),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Camera is busy or the command is not supported in the Camera's current state", "Check current camera state is if appropriate fot the CMD")),
    std::make_pair(getRawRetCode(CameraCommonErr::TimeNotSync),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "The time stamp of the camera is not sync", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(CameraCommonErr::ParamSetFailed),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Camera failed to set the parameters it received", "Please check the parameter to set is if supported in your devices.")),
    std::make_pair(getRawRetCode(CameraCommonErr::ParamGetFailed),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Camera param get failed", "Please check the parameter to get is if supported in your devices.")),
    std::make_pair(getRawRetCode(CameraCommonErr::SDCardMISSING),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Camera has no SD Card", "Please install SD card.")),
    std::make_pair(getRawRetCode(CameraCommonErr::SDCardFull),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "The Camera's SD Card is full", "Please make sure the SD card has enough space.")),
    std::make_pair(getRawRetCode(CameraCommonErr::SDCardError),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Error accessing the SD Card", "Please check the validity of the SD card.")),
    std::make_pair(getRawRetCode(CameraCommonErr::SensorError),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Camera sensor error", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(CameraCommonErr::SystemError),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Camera system error", "Please recheck all the running conditions or contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(CameraCommonErr::ParamLenTooLong),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Camera param get failed", "Please check the validity of the parameter length")),
    std::make_pair(getRawRetCode(CameraCommonErr::ModuleInactivated),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Camera module is not activated", "Please activate the module first.")),
    std::make_pair(getRawRetCode(CameraCommonErr::FWSeqNumNotInOrder),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "The seq number of Firmware data is invalid", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(CameraCommonErr::FWCheckErr),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Firmware check error", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(CameraCommonErr::FlashWriteError),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Camera flash write error", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(CameraCommonErr::FWInvalidType),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Firmware type is invalid", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(CameraCommonErr::RCDisconnect),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Remote Control is disconnected now", "Please check the connection with remote control is if OK.")),
    std::make_pair(getRawRetCode(CameraCommonErr::HardwareErr),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Camera hardware error", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(CameraCommonErr::UAVDisconnect),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Disconnect with aircraft", "Please check the connection with aircraft is if OK.")),
    std::make_pair(getRawRetCode(CameraCommonErr::UpgradeErrorNow),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Camera cannot not upgrade in current status", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(CameraCommonErr::UndefineError),
                   ErrorCodeMsg(module[CameraModule].ModuleName, "Undefined error", "Please contact <dev@dji.com> for help.")),
};

const ErrorCode::ErrorCodeMapType ErrorCode::getCameraCommonErrorMap() {
  const ErrorCodeMapType CameraCommonErrorMap(CameraCommonErrData,
                                              CameraCommonErrData
                                                + sizeof CameraCommonErrData
                                                  / sizeof CameraCommonErrData[0]);
  return CameraCommonErrorMap;
}


const std::pair<const ErrorCode::ErrorCodeType, ErrorCode::ErrorCodeMsg> ErrorCode::GimbalCommonErrData[] = {
    std::make_pair(getRawRetCode(GimbalCommonErr::InvalidCMD),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Command not supported", "Check the firmware or command validity")),
    std::make_pair(getRawRetCode(GimbalCommonErr::Timeout),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Gimbal's execution of this action has timed out", "Try again or check the firmware or command")),
    std::make_pair(getRawRetCode(GimbalCommonErr::OutOfMemory),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Gimbal's execution of this action is out of memory", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::InvalidParam),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Gimbal received invalid parameters", "Check the validity of the parameter")),
    std::make_pair(getRawRetCode(GimbalCommonErr::InvalidState),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Gimbal is busy or the command is not supported in the Gimbal's current state", "Check current Gimbal state is if appropriate fot the CMD")),
    std::make_pair(getRawRetCode(GimbalCommonErr::TimeNotSync),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "The time stamp of the Gimbal is not sync", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::ParamSetFailed),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Gimbal failed to set the parameters it received", "Please check the parameter to set is if supported in your devices.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::ParamGetFailed),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Gimbal param get failed", "Please check the parameter to get is if supported in your devices.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::SDCardMISSING),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Gimbal has no SD Card", "Please install SD card.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::SDCardFull),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "The Gimbal's SD Card is full", "Please make sure the SD card has enough space.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::SDCardError),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Error accessing the SD Card", "Please check the validity of the SD card.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::SensorError),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Gimbal sensor error", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::SystemError),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Gimbal system error", "Please recheck all the running conditions or contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::ParamLenTooLong),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Gimbal param get failed", "Please check the validity of the parameter length")),
    std::make_pair(getRawRetCode(GimbalCommonErr::ModuleInactivated),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Gimbal module is not activated", "Please activate the module first.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::FWSeqNumNotInOrder),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "The seq number of Firmware data is invalid", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::FWCheckErr),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Firmware check error", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::FlashWriteError),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Gimbal flash write error", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::FWInvalidType),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Firmware type is invalid", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::RCDisconnect),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Remote Control is disconnected now", "Please check the connection with remote control is if OK.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::HardwareErr),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Gimbal hardware error", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::UAVDisconnect),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Disconnect with aircraft", "Please check the connection with aircraft is if OK.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::UpgradeErrorNow),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Gimbal cannot not upgrade in current status", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(GimbalCommonErr::UndefineError),
                   ErrorCodeMsg(module[GimbalModule].ModuleName, "Undefined error", "Please contact <dev@dji.com> for help.")),
};

const ErrorCode::ErrorCodeMapType ErrorCode::getGimbalCommonErrorMap() {
  const ErrorCodeMapType GimbalCommonErrorMap(GimbalCommonErrData,
                                              GimbalCommonErrData
                                                  + sizeof GimbalCommonErrData
                                                      / sizeof GimbalCommonErrData[0]);
  return GimbalCommonErrorMap;
}

const std::pair<const ErrorCode::ErrorCodeType, ErrorCode::ErrorCodeMsg> ErrorCode::PSDKCommonErrData[] = {
    std::make_pair(getRawRetCode(PSDKCommonErr::InvalidCMD),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "Command not supported", "Check the firmware or command validity")),
    std::make_pair(getRawRetCode(PSDKCommonErr::Timeout),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "PSDK device's execution of this action has timed out", "Try again or check the firmware or command")),
    std::make_pair(getRawRetCode(PSDKCommonErr::OutOfMemory),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "PSDK device's execution of this action is out of memory", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::InvalidParam),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "PSDK device received invalid parameters", "Check the validity of the parameter")),
    std::make_pair(getRawRetCode(PSDKCommonErr::InvalidState),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "PSDK device is busy or the command is not supported in the PSDK device's current state", "Check current camera state is if appropriate fot the CMD")),
    std::make_pair(getRawRetCode(PSDKCommonErr::TimeNotSync),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "The time stamp of the camera is not sync", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::ParamSetFailed),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "PSDK device failed to set the parameters it received", "Please check the parameter to set is if supported in your devices.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::ParamGetFailed),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "PSDK device param get failed", "Please check the parameter to get is if supported in your devices.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::SDCardMISSING),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "PSDK device has no SD Card", "Please install SD card.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::SDCardFull),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "The PSDK device's SD Card is full", "Please make sure the SD card has enough space.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::SDCardError),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "Error accessing the SD Card", "Please check the validity of the SD card.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::SensorError),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "PSDK device sensor error", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::SystemError),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "PSDK device system error", "Please recheck all the running conditions or contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::ParamLenTooLong),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "PSDK device param get failed", "Please check the validity of the parameter length")),
    std::make_pair(getRawRetCode(PSDKCommonErr::ModuleInactivated),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "PSDK device module is not activated", "Please activate the module first.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::FWSeqNumNotInOrder),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "The seq number of Firmware data is invalid", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::FWCheckErr),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "Firmware check error", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::FlashWriteError),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "PSDK device flash write error", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::FWInvalidType),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "Firmware type is invalid", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::RCDisconnect),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "Remote Control is disconnected now", "Please check the connection with remote control is if OK.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::HardwareErr),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "PSDK device hardware error", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::UAVDisconnect),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "Disconnect with aircraft", "Please check the connection with aircraft is if OK.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::UpgradeErrorNow),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "PSDK device cannot not upgrade in current status", "Please contact <dev@dji.com> for help.")),
    std::make_pair(getRawRetCode(PSDKCommonErr::UndefineError),
                   ErrorCodeMsg(module[PSDKModule].ModuleName, "Undefined error", "Please contact <dev@dji.com> for help.")),
};

const ErrorCode::ErrorCodeMapType ErrorCode::getPSDKCommonErrorMap() {
  const ErrorCodeMapType PSDKCommonErrorMap(PSDKCommonErrData,
                                            PSDKCommonErrData
                                              + sizeof PSDKCommonErrData
                                                / sizeof PSDKCommonErrData[0]);
  return PSDKCommonErrorMap;
}

const std::pair<const ErrorCode::ErrorCodeType, ErrorCode::ErrorCodeMsg> ErrorCode::SystemCommonErrData[] = {
    std::make_pair(getRawRetCode(SysCommonErr::Success),
                   ErrorCodeMsg(module[SysModule].ModuleName, "Execute successfully", "None")),
    std::make_pair(getRawRetCode(SysCommonErr::AllocMemoryFailed),
                   ErrorCodeMsg(module[SysModule].ModuleName, "Alloc memory failed", "Please make sure there is enough memory space to support the code running.")),
    std::make_pair(getRawRetCode(SysCommonErr::ReqNotSupported),
                   ErrorCodeMsg(module[SysModule].ModuleName, "This request is not supported to the handler", "Please make sure this request is already supported to the handler.")),
    std::make_pair(getRawRetCode(SysCommonErr::ReqTimeout),
                   ErrorCodeMsg(module[SysModule].ModuleName, "Request time out", "Try again or check the status of the target object.")),
    std::make_pair(getRawRetCode(SysCommonErr::UnpackDataMismatch),
                   ErrorCodeMsg(module[SysModule].ModuleName, "The respond unpacking mismatch", "Please make sure the firmware is matching this OSDK version.")),
    std::make_pair(getRawRetCode(SysCommonErr::InstInitParamInvalid),
                   ErrorCodeMsg(module[SysModule].ModuleName, "Instance init parameter invalid", "Please make sure the parameter used in instance initializing is valid.")),
    std::make_pair(getRawRetCode(SysCommonErr::UserCallbackInvalid),
                   ErrorCodeMsg(module[SysModule].ModuleName, "The callback set by user is a invalid", "Please make sure the validity of the callback you requesting.")),
    std::make_pair(getRawRetCode(SysCommonErr::UndefinedError),
                   ErrorCodeMsg(module[SysModule].ModuleName, "Undefined error", "Unknown error code : 0X%lX, please contact <dev@dji.com> for help.")),
};

const ErrorCode::ErrorCodeMapType ErrorCode::getSystemCommonErrorMap() {
  const ErrorCodeMapType SystemCommonErrorMap(SystemCommonErrData,
                                              SystemCommonErrData
                                                + sizeof SystemCommonErrData
                                                  / sizeof SystemCommonErrData[0]);
  return SystemCommonErrorMap;
}

const ErrorCode::FunctionDataType ErrorCode::SystemFunction[functionMaxCnt] = {
    {"SystemCommon", getSystemCommonErrorMap},   /*!< SystemCommon */
};

const ErrorCode::FunctionDataType ErrorCode::GimbalFunction[functionMaxCnt] = {
    {"GimbalCommon", getGimbalCommonErrorMap},   /*!< GimbalCommon */
};


const ErrorCode::FunctionDataType ErrorCode::CameraFunction[functionMaxCnt] = {
    {"CameraCommon", getCameraCommonErrorMap},   /*!< CameraCommon */
};

const ErrorCode::FunctionDataType ErrorCode::PSDKFunction[functionMaxCnt] = {
    {"PSDKCommon", getPSDKCommonErrorMap},   /*!< PSDKCommon */
};

const ErrorCode::FunctionDataType ErrorCode::WaypointV2Function[functionMaxCnt] = {
  {"WaypointV2Common", getWaypointV2CommonErrorMap},   /*!< WaypointV2Common */
};
// clang-format on

ErrorCode::ErrorCodeMsg ErrorCode::getErrorCodeMsg(int64_t errCode) {
  ModuleIDType moduleID = getModuleID(errCode);
  FunctionIDType functionID = getFunctionID(errCode);
  RawRetCodeType rawRetCode = getRawRetCode(errCode);
  char defaultResolutionMsg[100] = {0};
  snprintf(defaultResolutionMsg, sizeof(defaultResolutionMsg),
           "Unknown error code : 0X%lX, please contact <dev@dji.com> for help.",
           errCode);
  ErrorCodeMsg retMsg(getModuleName(errCode), "Unknown", defaultResolutionMsg);

  if ((moduleID < ModuleMaxCnt) && (functionID < functionMaxCnt) &&
      (module[moduleID].data)) {
    auto msg = module[moduleID].data[functionID].getMap();
    if (msg.find(rawRetCode) != msg.end()) {
      retMsg = msg.find(rawRetCode)->second;
    }
  }
  return retMsg;
}

void ErrorCode::printErrorCodeMsg(int64_t errCode) {
  ErrorCodeMsg errMsg = getErrorCodeMsg(errCode);
  if (errCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Execute successfully.");
  } else {
    DERROR(">>>>Error module   : %s", errMsg.moduleMsg);
    DERROR(">>>>Error message  : %s", errMsg.errorMsg);
    DERROR(">>>>Error solution : %s", errMsg.solutionMsg);
  }
}

ErrorCode::ModuleIDType ErrorCode::getModuleID(ErrorCodeType errCode) {
  return (ModuleIDType)((errCode >> moduleIDLeftMove) & 0xFF);
}

const char* ErrorCode::getModuleName(ErrorCodeType errCode) {
  ModuleIDType moduleID = getModuleID(errCode);
  if (moduleID < ModuleMaxCnt) {
    return module[moduleID].ModuleName;
  } else {
    return "Unknown";
  }
}

ErrorCode::FunctionIDType ErrorCode::getFunctionID(ErrorCodeType errCode) {
  return (FunctionIDType)((errCode >> functionIDLeftMove) & 0xFF);
}

ErrorCode::RawRetCodeType ErrorCode::getRawRetCode(ErrorCodeType errCode) {
  return (RawRetCodeType)(errCode & 0xFFFFFFFF);
}

ErrorCode::ErrorCodeType ErrorCode::getLinkerErrorCode(E_OsdkStat cb_type) {
  switch (cb_type)
  {
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