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
    {"Gimbal",     NULL},           /*!< GimbalModule */
    {"Camera",     CameraFunction}, /*!< CameraModule */
    {"PSDK",       PSDKFunction},   /*!< PSDKModule */
    {"RC",         NULL},           /*!< RCModule */
    {"Battery",    NULL},           /*!< BatteryModule */
};

/*! camera api error code */
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


/*! system releated error code */
const ErrorCode::ErrorCodeType ErrorCode::SysCommonErr::Success              = ErrorCode::getErrorCode(SysModule, SystemCommon, SYSTEM_ERROR_RAW_CODE::Success);
const ErrorCode::ErrorCodeType ErrorCode::SysCommonErr::AllocMemoryFailed    = ErrorCode::getErrorCode(SysModule, SystemCommon, SYSTEM_ERROR_RAW_CODE::AllocMemoryFailed);
const ErrorCode::ErrorCodeType ErrorCode::SysCommonErr::ReqNotSupported      = ErrorCode::getErrorCode(SysModule, SystemCommon, SYSTEM_ERROR_RAW_CODE::ReqNotSupported);
const ErrorCode::ErrorCodeType ErrorCode::SysCommonErr::ReqTimeout           = ErrorCode::getErrorCode(SysModule, SystemCommon, SYSTEM_ERROR_RAW_CODE::Timeout);
const ErrorCode::ErrorCodeType ErrorCode::SysCommonErr::UnpackDataMismatch   = ErrorCode::getErrorCode(SysModule, SystemCommon, SYSTEM_ERROR_RAW_CODE::UnpackDataMismatch);
const ErrorCode::ErrorCodeType ErrorCode::SysCommonErr::InstInitParamInvalid = ErrorCode::getErrorCode(SysModule, SystemCommon, SYSTEM_ERROR_RAW_CODE::InstInitParamInvalid);
const ErrorCode::ErrorCodeType ErrorCode::SysCommonErr::UserCallbackInvalid  = ErrorCode::getErrorCode(SysModule, SystemCommon, SYSTEM_ERROR_RAW_CODE::UserCallbackInvalid);

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

const ErrorCode::FunctionDataType ErrorCode::CameraFunction[functionMaxCnt] = {
    {"CameraCommon", getCameraCommonErrorMap},   /*!< CameraCommon */
};

const ErrorCode::FunctionDataType ErrorCode::PSDKFunction[functionMaxCnt] = {
    {"PSDKCommon", getPSDKCommonErrorMap},   /*!< PSDKCommon */
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