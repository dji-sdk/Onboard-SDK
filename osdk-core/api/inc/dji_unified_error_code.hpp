/** @file dji_unified_error_code.hpp
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

#ifndef ONBOARDSDK_INTERNAL_DJI_UNIFIED_ERROR_CODE_HPP
#define ONBOARDSDK_INTERNAL_DJI_UNIFIED_ERROR_CODE_HPP

#include <stdint.h>
#include <map>
#include <string>

namespace DJI {
namespace OSDK {
/*! @class ErrCode is a new error code system for OSDK. It will be used
 * in the new APIs from OSDK 3.9 and more old error code will be replaced by
 * this in the future.
 */
class ErrCode {
 public:
  /*! @brief Module ID used in OnboardSDK Unified error
   */
  enum ModuleID {
    SysModule     = 0,
    FCModule      = 11,
    GimbalModule  = 12,
    CameraModule  = 13,
    PSDKModule    = 14,
    RCModule      = 15,
    BatteryModule = 16,
  };

  /*! @brief Function ID used with SystemModule in error codes
   */
  enum SystemFunctionID {
    SystemCommon = 0,
  };
  /*! @brief Function ID used with FCModule in error codes
   */
  enum FCFunctionID {
    FCControl   = 0,
    FCSubscribe = 1,
    FCMission   = 2,
  };

  /*! @brief Function ID used with CameraModule in error codes
   */
  enum CameraFunctionID {
    CameraCommon = 0,
  };

 public:
  ErrCode();
  ~ErrCode();

  /*! @brief Unified error type
   */
  typedef int64_t ErrCodeType;

  /*! @brief Releated messages about error codes
   */
  typedef struct ErrCodeMsg {
    ErrCodeMsg(std::string s1, std::string s2, std::string s3)
        : moduleMsg(s1), errorMsg(s2), resolutionMsg(s3){};
    std::string moduleMsg;
    std::string errorMsg;
    std::string resolutionMsg;
  } ErrCodeMsg;

  /*! @brief Build error code
   *  @param moduleID module ID used in errCode ref to
   * DJI::OSDK::ErrCode::ModuleID
   *  @param functionID function ID used in errCode ref to
   * DJI::OSDK::ErrCode::xxxxFunctionID
   *  @param rawRetCode raw return code from the ack data
   *  @return Unified error type
   */
  static constexpr ErrCodeType errorCode(uint8_t moduleID, uint8_t functionID,
                                         uint32_t rawRetCode) {
    return (((ErrCodeType)moduleID << moduleIDLeftMove) |
            ((ErrCodeType)functionID << functionIDLeftMove) |
            (ErrCodeType)rawRetCode);
  }

  /*! @brief Get the module ID from errCode
   *  @param errCode Unified error type
   *  @return Module ID ref to DJI::OSDK::ErrCode::ModuleID
   */
  static uint8_t getModuleID(ErrCodeType errCode);

  /*! @brief Get the module name from errCode
   *  @param errCode Unified error type
   *  @return Module name
   */
  static std::string getModuleName(ErrCodeType errCode);

  /*! @brief Get the function name from errCode
   *  @param errCode Unified error type
   *  @return Function name
   */
  static uint8_t getFunctionID(ErrCodeType errCode);

  /*! @brief Get error code messages from errCode
   *  @param errCode Unified error type
   *  @return Releated error code messages
   */
  static ErrCodeMsg getErrCodeMsg(ErrCodeType errCode);

  /*! @brief camera api error code
   */
  class CameraCommonErr {
   public:
    static const ErrCodeType Success;
    static const ErrCodeType InvalidCMD;
    static const ErrCodeType Timeout;
    static const ErrCodeType OutOfMemory;
    static const ErrCodeType InvalidParam;
    static const ErrCodeType InvalidState;
    static const ErrCodeType TimeNotSync;
    static const ErrCodeType ParamSetFailed;
    static const ErrCodeType ParamGetFailed;
    static const ErrCodeType SDCardMISSING;
    static const ErrCodeType SDCardFull;
    static const ErrCodeType SDCardError;
    static const ErrCodeType SensorError;
    static const ErrCodeType SystemError;
    static const ErrCodeType ParamLenTooLong;
    static const ErrCodeType ModuleInactivated;
    static const ErrCodeType FWSeqNumNotInOrder;
    static const ErrCodeType FWCheckErr;
    static const ErrCodeType FlashWriteError;
    static const ErrCodeType FWInvalidType;
    static const ErrCodeType RCDisconnect;
    static const ErrCodeType HardwareErr;
    static const ErrCodeType UAVDisconnect;
    static const ErrCodeType UpgradeErrNow;
    static const ErrCodeType UndefineError;
  };

  /*! @brief system releated error code
   */
  class SysCommonErr {
   public:
    static const ErrCodeType Success;
    static const ErrCodeType ReqHandlerNotFound;
    static const ErrCodeType ReqNotSupportedByHandler;
    static const ErrCodeType ReqTimeout;
    static const ErrCodeType SendPackFailure;
    static const ErrCodeType Disconnected;
    static const ErrCodeType InvalidParam;
    static const ErrCodeType SystemError;
    static const ErrCodeType CommandInterrupted;
    static const ErrCodeType ParametersGetError;
    static const ErrCodeType ParametersSetError;
    static const ErrCodeType InvalidRespond;
    static const ErrCodeType ParamOutOfRange;
    static const ErrCodeType InvalidReqInCurState;
    static const ErrCodeType ExecutionFailed;
  };

 private:
  static const uint8_t moduleIDLeftMove = 40;
  static const uint8_t functionIDLeftMove = 32;

  /*! @brief The map container of the CameraCommonErr error code messages.
   */
  static const std::map<const ErrCodeType, ErrCodeMsg> CameraCommonErrMap;

  /*! @brief The map container of the SystemCommonErr error code messages.
   */
  static const std::map<const ErrCodeType, ErrCodeMsg> SystemCommonErrMap;
};

}  // namespace OSDK
}  // namespace DJI
#endif  // ONBOARDSDK_INTERNAL_DJI_UNIFIED_ERROR_CODE_HPP
