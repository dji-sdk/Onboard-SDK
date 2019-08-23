/** @file dji_unified_error_code.cpp
 *  @version 3.9
 *  @date August 2019
 *
 *  @brief DJI new unified error codes.
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

#include "dji_unified_error_code.hpp"
#include "dji_type.hpp"
#include "dji_log.hpp"

using namespace DJI;
using namespace DJI::OSDK;

// clang-format off

/*! camera api error code */
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::InvalidCMD         = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::UNSUPPORTED_COMMAND);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::Timeout            = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::TIMEOUT);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::OutOfMemory        = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::RAM_ALLOCATION_FAILED);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::InvalidParam       = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::INVALID_COMMAND_PARAMETER);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::InvalidState       = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::UNSUPPORTED_COMMAND_IN_CUR_STATE);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::TimeNotSync        = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::CAMERA_TIME_NOT_SYNCHRONIZED);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::ParamSetFailed     = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::PARAMETER_SET_FAILED);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::ParamGetFailed     = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::PARAMETER_GET_FAILED);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::SDCardMISSING      = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::SD_CARD_MISSING);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::SDCardFull         = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::SD_CARD_FULL);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::SDCardError        = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::SD_CARD_ERROR);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::SensorError        = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::SENSOR_ERROR);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::SystemError        = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::SYSTEM_ERROR);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::ParamLenTooLong    = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::PARAMETER_TOTAL_TOO_LONG);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::ModuleInactivated  = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::MODULE_INACTIVATED);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::FWSeqNumNotInOrder = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::FIRMWARE_DATA_NUM_DISCONTINUOUS);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::FWCheckErr         = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::FIRMWARE_VERIFICATION_ERROR);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::FlashWriteError    = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::FLASH_WRITE_ERROR);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::FWInvalidType      = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::FIRMWARE_TYPE_MISMATCH);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::RCDisconnect       = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::REMOTE_CONTROL_UNCONNECTED);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::HardwareErr        = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::HARDWARE_ERROR);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::UAVDisconnect      = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::AIRCRAFT_UNCONNECTED);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::UpgradeErrNow      = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::CANNOT_UPGRADE_IN_CUR_STATE);
const ErrCode::ErrCodeType ErrCode::CameraCommonErr::UndefineError      = ErrCode::errorCode(CameraModule, CameraCommon, DJI_CMD_RETURN_CODE::UNDEFINE_ERROR);

/*! system releated error code */
const ErrCode::ErrCodeType ErrCode::SysCommonErr::Success                  = ErrCode::errorCode(SysModule, SystemCommon, 0x00000000);
const ErrCode::ErrCodeType ErrCode::SysCommonErr::ReqHandlerNotFound       = ErrCode::errorCode(SysModule, SystemCommon, 0x00000001);
const ErrCode::ErrCodeType ErrCode::SysCommonErr::ReqNotSupportedByHandler = ErrCode::errorCode(SysModule, SystemCommon, 0x00000002);
const ErrCode::ErrCodeType ErrCode::SysCommonErr::ReqTimeout               = ErrCode::errorCode(SysModule, SystemCommon, 0x00000003);
const ErrCode::ErrCodeType ErrCode::SysCommonErr::SendPackFailure          = ErrCode::errorCode(SysModule, SystemCommon, 0x00000004);
const ErrCode::ErrCodeType ErrCode::SysCommonErr::Disconnected             = ErrCode::errorCode(SysModule, SystemCommon, 0x00000005);
const ErrCode::ErrCodeType ErrCode::SysCommonErr::InvalidParam             = ErrCode::errorCode(SysModule, SystemCommon, 0x00000006);
const ErrCode::ErrCodeType ErrCode::SysCommonErr::SystemError              = ErrCode::errorCode(SysModule, SystemCommon, 0x00000007);
const ErrCode::ErrCodeType ErrCode::SysCommonErr::CommandInterrupted       = ErrCode::errorCode(SysModule, SystemCommon, 0x00000008);
const ErrCode::ErrCodeType ErrCode::SysCommonErr::ParametersGetError       = ErrCode::errorCode(SysModule, SystemCommon, 0x00000009);
const ErrCode::ErrCodeType ErrCode::SysCommonErr::ParametersSetError       = ErrCode::errorCode(SysModule, SystemCommon, 0x0000000A);
const ErrCode::ErrCodeType ErrCode::SysCommonErr::InvalidRespond           = ErrCode::errorCode(SysModule, SystemCommon, 0x0000000B);
const ErrCode::ErrCodeType ErrCode::SysCommonErr::ParamOutOfRange          = ErrCode::errorCode(SysModule, SystemCommon, 0x0000000C);
const ErrCode::ErrCodeType ErrCode::SysCommonErr::InvalidReqInCurState     = ErrCode::errorCode(SysModule, SystemCommon, 0x0000000D);
const ErrCode::ErrCodeType ErrCode::SysCommonErr::ExecutionFailed          = ErrCode::errorCode(SysModule, SystemCommon, 0x0000000E);

const std::map<const ErrCode::ErrCodeType, ErrCode::ErrCodeMsg> ErrCode::CameraCommonErrMap = {
    {ErrCode::CameraCommonErr::InvalidCMD,
     ErrCode::ErrCodeMsg("Camera", "Command not supported", "Check the firmware or command validity")},
    {ErrCode::CameraCommonErr::Timeout,
     ErrCode::ErrCodeMsg("Camera", "Camera's execution of this action has timed out", "Try again or check the firmware or command")},
    {ErrCode::CameraCommonErr::OutOfMemory,
     ErrCode::ErrCodeMsg("Camera", "Camera's execution of this action is out of memory", "Please contact <dev@dji.com> for help.")},
    {ErrCode::CameraCommonErr::InvalidParam,
     ErrCode::ErrCodeMsg("Camera", "Camera received invalid parameters", "Check the validity of the parameter")},
    {ErrCode::CameraCommonErr::InvalidState,
     ErrCode::ErrCodeMsg("Camera", "Camera is busy or the command is not supported in the Camera's current state", "Check current camera state is if appropriate fot the CMD")},
    {ErrCode::CameraCommonErr::TimeNotSync,
     ErrCode::ErrCodeMsg("Camera", "The time stamp of the camera is not sync", "Please contact <dev@dji.com> for help.")},
    {ErrCode::CameraCommonErr::ParamSetFailed,
     ErrCode::ErrCodeMsg("Camera", "Camera failed to set the parameters it received", "Please check the parameter to set is if supported in your devices.")},
    {ErrCode::CameraCommonErr::ParamGetFailed,
     ErrCode::ErrCodeMsg("Camera", "Camera param get failed", "Please check the parameter to get is if supported in your devices.")},
    {ErrCode::CameraCommonErr::SDCardMISSING,
     ErrCode::ErrCodeMsg("Camera", "Camera has no SD Card", "Please install SD card.")},
    {ErrCode::CameraCommonErr::SDCardFull,
     ErrCode::ErrCodeMsg("Camera", "The Camera's SD Card is full", "Please make sure the SD card has enough space.")},
    {ErrCode::CameraCommonErr::SDCardError,
     ErrCode::ErrCodeMsg("Camera", "Error accessing the SD Card", "Please check the validity of the SD card.")},
    {ErrCode::CameraCommonErr::SensorError,
     ErrCode::ErrCodeMsg("Camera", "Camera sensor error", "Please contact <dev@dji.com> for help.")},
    {ErrCode::CameraCommonErr::SystemError,
     ErrCode::ErrCodeMsg("Camera", "Camera system error", "Please recheck all the running conditions or contact <dev@dji.com> for help.")},
    {ErrCode::CameraCommonErr::ParamLenTooLong,
     ErrCode::ErrCodeMsg("Camera", "Camera param get failed", "Please check the validity of the parameter length")},
    {ErrCode::CameraCommonErr::ModuleInactivated,
     ErrCode::ErrCodeMsg("Camera", "Camera module is not activated", "Please activate the module first.")},
    {ErrCode::CameraCommonErr::FWSeqNumNotInOrder,
     ErrCode::ErrCodeMsg("Camera", "The seq number of Firmware data is invalid", "Please contact <dev@dji.com> for help.")},
    {ErrCode::CameraCommonErr::FWCheckErr,
     ErrCode::ErrCodeMsg("Camera", "Firmware check error", "Please contact <dev@dji.com> for help.")},
    {ErrCode::CameraCommonErr::FlashWriteError,
     ErrCode::ErrCodeMsg("Camera", "Camera flash write error", "Please contact <dev@dji.com> for help.")},
    {ErrCode::CameraCommonErr::FWInvalidType,
     ErrCode::ErrCodeMsg("Camera", "Firmware type is invalid", "Please contact <dev@dji.com> for help.")},
    {ErrCode::CameraCommonErr::RCDisconnect,
     ErrCode::ErrCodeMsg("Camera", "Remote Control is disconnected now", "Please check the connection with remote control is if OK.")},
    {ErrCode::CameraCommonErr::HardwareErr,
     ErrCode::ErrCodeMsg("Camera", "Camera hardware error", "Please contact <dev@dji.com> for help.")},
    {ErrCode::CameraCommonErr::UAVDisconnect,
     ErrCode::ErrCodeMsg("Camera", "Disconnect with aircraft", "Please check the connection with aircraft is if OK.")},
    {ErrCode::CameraCommonErr::UpgradeErrNow,
     ErrCode::ErrCodeMsg("Camera", "Camera cannot not upgrade in current status", "Please contact <dev@dji.com> for help.")},
    {ErrCode::CameraCommonErr::UndefineError,
     ErrCode::ErrCodeMsg("Camera", "Undefined error", "Please contact <dev@dji.com> for help.")},
};

const std::map<const ErrCode::ErrCodeType, ErrCode::ErrCodeMsg> ErrCode::SystemCommonErrMap = {
    {ErrCode::SysCommonErr::Success,
     ErrCode::ErrCodeMsg("System", "Execute successfully", "None")},
    {ErrCode::SysCommonErr::ReqHandlerNotFound,
     ErrCode::ErrCodeMsg("System", "The handler to execute this request is not found yet", "Please make sure the handler to execute this request is already ready.")},
    {ErrCode::SysCommonErr::ReqNotSupportedByHandler,
     ErrCode::ErrCodeMsg("System", "This request is not supported to the handler", "Please make sure this request is already supported to the handler.")},
    {ErrCode::SysCommonErr::ReqTimeout,
     ErrCode::ErrCodeMsg("System", "Request time out", "Try again or check the status of the target object.")},
    {ErrCode::SysCommonErr::SendPackFailure,
     ErrCode::ErrCodeMsg("System", "Send package failed", "Please check the link layer is if ready to send.")},
    {ErrCode::SysCommonErr::Disconnected,
     ErrCode::ErrCodeMsg("System", "The connection to the target object is disconnected", "Please check the connection to target object.")},
    {ErrCode::SysCommonErr::InvalidParam,
     ErrCode::ErrCodeMsg("System", "Parameter is invalid", "Please check the validity of the parameter.")},
    {ErrCode::SysCommonErr::SystemError,
     ErrCode::ErrCodeMsg("System", "System error", "Please check the calling of the API make sense or not.")},
    {ErrCode::SysCommonErr::CommandInterrupted,
     ErrCode::ErrCodeMsg("System", "The execution of the command was interrupted", "Please check the interrupting reason for this calling.")},
    {ErrCode::SysCommonErr::ParametersGetError,
     ErrCode::ErrCodeMsg("System", "Get parameter error", "Please check the reason for getting parameter failed.")},
    {ErrCode::SysCommonErr::ParametersSetError,
     ErrCode::ErrCodeMsg("System", "Set parameter error", "Please check the reason for setting parameter failed.")},
    {ErrCode::SysCommonErr::InvalidRespond,
     ErrCode::ErrCodeMsg("System", "The respond is invalid", "Please check the version of the device and the code.")},
    {ErrCode::SysCommonErr::ParamOutOfRange,
     ErrCode::ErrCodeMsg("System", "Parameter is out of range", "Please check the validity of the request parameter.")},
    {ErrCode::SysCommonErr::InvalidReqInCurState,
     ErrCode::ErrCodeMsg("System", "Do not support this request in current state.", "Please make sure the current state is suitable for this calling.")},
    {ErrCode::SysCommonErr::ExecutionFailed,
     ErrCode::ErrCodeMsg("System", "Execute failed", "Please check the failed reason.")},
};

// clang-format on

ErrCode::ErrCodeMsg ErrCode::getErrCodeMsg(ErrCode::ErrCodeType errCode) {
  ModuleIDType ModuleID = getModuleID(errCode);
  FunctionIDType FunctionID = getFunctionID(errCode);
  char defaultResolutionMsg[100] = {0};
  snprintf(defaultResolutionMsg, sizeof(defaultResolutionMsg),
           "Unknown error code : 0X%lX, please contact <dev@dji.com> for help.",
           errCode);
  ErrCodeMsg retMsg(getModuleName(errCode), "Unknown", defaultResolutionMsg);
  switch (ModuleID) {
    case SysModule:
      if (FunctionID == SystemCommon) {
        auto msg = SystemCommonErrMap.find(errCode);
        if (msg != SystemCommonErrMap.end()) {
          retMsg = msg->second;
        }
      }
      break;
    case FCModule:
      break;
    case GimbalModule:
      break;
    case CameraModule:
      if (FunctionID == CameraCommon) {
        auto msg = CameraCommonErrMap.find(errCode);
        if (msg != CameraCommonErrMap.end()) {
          retMsg = msg->second;
        }
      }
      break;
    case PSDKModule:
      break;
    case RCModule:
      break;
    case BatteryModule:
      break;
    default:
      break;
  }
  return retMsg;
}

void ErrCode::printErrCodeMsg(ErrCode::ErrCodeType errCode) {
  ErrCodeMsg errMsg = getErrCodeMsg(errCode);
  DERROR("\033[;31m Error module   : %s\033[0m", errMsg.moduleMsg.c_str());
  DERROR("\033[;31m Error message  : %s\033[0m", errMsg.errorMsg.c_str());
  DERROR("\033[;31m Error solution : %s\033[0m", errMsg.solutionMsg.c_str());
}

const ErrCode::ErrCodeType ErrCode::errorCode(ErrCode::ModuleIDType moduleID,
                                              ErrCode::FunctionIDType functionID,
                                              uint32_t rawRetCode) {
  ErrCodeType retErrCode = 0;
  /*! If the rawRetCode = 0, then the ErrCode should be
   * ErrCode::SysCommonErr::Success */
  if (!rawRetCode) {
    retErrCode = (((ErrCodeType) ErrCode::SysModule << moduleIDLeftMove) |
        ((ErrCodeType) ErrCode::SystemCommon << functionIDLeftMove) |
        (ErrCodeType) 0x00000000);
  } else {
    retErrCode = (((ErrCodeType) moduleID << moduleIDLeftMove) |
        ((ErrCodeType) functionID << functionIDLeftMove) |
        (ErrCodeType) rawRetCode);
  }
  return retErrCode;
}

ErrCode::ModuleIDType ErrCode::getModuleID(ErrCodeType errCode) {
  return (ModuleIDType)((errCode >> moduleIDLeftMove) & 0xFF);
}

std::string ErrCode::getModuleName(ErrCodeType errCode) {
  ModuleIDType moduleID = getModuleID(errCode);
  switch (moduleID) {
    case ModuleID::SysModule:
      return "System";
    case ModuleID::FCModule:
      return "FC";
    case ModuleID::GimbalModule:
      return "Gimbal";
    case ModuleID::CameraModule:
      return "Camera";
    case ModuleID::PSDKModule:
      return "PSDK";
    case ModuleID::RCModule:
      return "RC";
    case ModuleID::BatteryModule:
      return "Battery";
    default:
      return "Unknown";
  }
}

ErrCode::FunctionIDType ErrCode::getFunctionID(ErrCodeType errCode) {
  return (FunctionIDType)((errCode >> functionIDLeftMove) & 0xFF);
}
