/** @file dji_camera_module.cpp
 *  @version 3.9
 *  @date July 2019
 *
 *  @brief Implementation of camera module for payload node
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

#include <dji_linker.hpp>
#include "dji_camera_module.hpp"
#include "dji_vehicle_callback.hpp"
#include "dji_legacy_linker.hpp"
#include "dji_camera_module.hpp"
#include "dji_internal_command.hpp"

using namespace DJI;
using namespace DJI::OSDK;

CameraModule::ShutterSpeedType createShutterSpeedStruct(int reciprocal,
                                                        int integer_part,
                                                        int decimal_part);

CameraModule::ShutterSpeedType ShutterSpeedEnumToShutterSpeedType(
    CameraModule::ShutterSpeed shutterSpeed);

CameraModule::ShutterSpeed ShutterSpeedTypeToShutterSpeedEnum(int reciprocal,
                                                              int integer_part,
                                                              int decimal_part);


CameraModule::CameraModule(Linker* linker,
                           PayloadIndexType payloadIndex, std::string name,
                           bool enable)
    : PayloadBase(linker, payloadIndex, name, enable) {}

CameraModule::~CameraModule() {}

typedef struct handlerType {
  void * cb;
  void *udata;
} handlerType;

void retAckCB(const T_CmdInfo *cmdInfo,
              const uint8_t *cmdData,
              void *userData, E_OsdkStat cb_type) {
  auto *handler = (handlerType *) userData;
  if (handler && handler->cb) {
    ErrorCode::ErrorCodeType ret;

    if ((cb_type == OSDK_STAT_OK) && (cmdData) && (cmdInfo->dataLen > 0)) {
      ret = ErrorCode::getErrorCode(ErrorCode::CameraModule,
                                    ErrorCode::CameraCommon,
                                    cmdData[0]);
    } else {
      ret = (cb_type == OSDK_STAT_ERR_TIMEOUT)
            ? ErrorCode::SysCommonErr::ReqTimeout
            : ErrorCode::SysCommonErr::UndefinedError;
    }

    auto cb = (void (*)(ErrorCode::ErrorCodeType retCode,
                        UserData userData)) handler->cb;
    cb(ret, handler->udata);
  }

  if (handler)free(handler);
}


template<typename AckT>
ErrorCode::ErrorCodeType dataParser(const uint8_t *pdata, uint32_t len,
                                    AckT &ack) {
  if (len >= sizeof(AckT) && pdata) {
    ack = *(AckT *) pdata;
    return ErrorCode::getErrorCode(ErrorCode::CameraModule,
                                   ErrorCode::CameraCommon, ack.ret_code);
  } else {
    DERROR("ACK is exception, data len %d (expect >= %d)\n", len, sizeof(AckT));
    return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
}

void paramAckCB(const T_CmdInfo *cmdInfo,
                const uint8_t *cmdData,
                void *userData, E_OsdkStat cb_type) {
  auto *handler = (handlerType *) userData;
  if (handler && handler->cb) {
    ErrorCode::ErrorCodeType ret = ErrorCode::SysCommonErr::Success;

    if (!cmdInfo) {
      DERROR("Cannot get the link info, system error.");
      return;
    }

    if ((cb_type == OSDK_STAT_OK) && cmdData && cmdInfo
        && (cmdInfo->dataLen > 0)) {
      ret = ErrorCode::getErrorCode(ErrorCode::CameraModule,
                                    ErrorCode::CameraCommon,
                                    cmdData[0]);
    } else {
      ret = (cb_type == OSDK_STAT_ERR_TIMEOUT)
            ? ErrorCode::SysCommonErr::ReqTimeout
            : ErrorCode::SysCommonErr::UndefinedError;
    }

//clang-format off
    /*! here cannot use switch, or compile fail */
    if (cmdInfo->cmdSet == V1ProtocolCMD::CMDSet::camera) {
      if (cmdInfo->cmdId == V1ProtocolCMD::Camera::getMode[1]) {
        /*! 0x0211 */
        auto cb = (void (*)(ErrorCode::ErrorCodeType, CameraModule::WorkMode, UserData)) handler->cb;
        CameraModule::WorkModeAck ack = {0};
        ack.workingMode = CameraModule::WORK_MODE_UNKNOWN;
        if (ret == ErrorCode::SysCommonErr::Success)
          ret = dataParser<CameraModule::WorkModeAck>(cmdData, cmdInfo->dataLen, ack);
        cb(ret, (CameraModule::WorkMode) ack.workingMode, handler->udata);
      } else if (cmdInfo->cmdId == V1ProtocolCMD::Camera::getExposureMode[1]) {
        /*! 0x021f */
        auto cb = (void (*)(ErrorCode::ErrorCodeType, CameraModule::ExposureMode, UserData)) handler->cb;
        CameraModule::ExposureModeAck ack = {0};
        ack.exposureMode = CameraModule::EXPOSURE_UNKNOWN;
        if (ret == ErrorCode::SysCommonErr::Success)
          ret = dataParser<CameraModule::ExposureModeAck>(cmdData, cmdInfo->dataLen, ack);
        cb(ret, (CameraModule::ExposureMode) ack.exposureMode, handler->udata);
      } else if (cmdInfo->cmdId == V1ProtocolCMD::Camera::getFocusMode[1]) {
        /*! 0x0225 */
        auto cb = (void (*)(ErrorCode::ErrorCodeType, CameraModule::FocusMode, UserData)) handler->cb;
        CameraModule::FocusModeAck ack = {0};
        ack.focusMode = (CameraModule::FocusModeData) CameraModule::FOCUS_MODE_UNKNOWN;
        if (ret == ErrorCode::SysCommonErr::Success)
          ret = dataParser<CameraModule::FocusModeAck>(cmdData, cmdInfo->dataLen, ack);
        cb(ret, (CameraModule::FocusMode) ack.focusMode, handler->udata);
      } else if (cmdInfo->cmdId == V1ProtocolCMD::Camera::getEvParameter[1]) {
        /*! 0x022f*/
        auto cb = (void (*)(ErrorCode::ErrorCodeType, CameraModule::ExposureCompensation, UserData)) handler->cb;
        CameraModule::ExposureCompensationAck ack = {0};
        ack.ev_param = (CameraModule::ExposureCompensationData) CameraModule::UNKNOWN;
        if (ret == ErrorCode::SysCommonErr::Success)
          ret = dataParser<CameraModule::ExposureCompensationAck>(cmdData, cmdInfo->dataLen, ack);
        cb(ret, (CameraModule::ExposureCompensation) ack.ev_param, handler->udata);
      } else if (cmdInfo->cmdId == V1ProtocolCMD::Camera::getSpotFocusAera[1]) {
        /*! 0x0231 */
        auto cb = (void (*)(ErrorCode::ErrorCodeType, CameraModule::TapFocusPosData, UserData)) handler->cb;
        CameraModule::TapFocusPosAck ack = {0};
        if (ret == ErrorCode::SysCommonErr::Success)
          ret = dataParser<CameraModule::TapFocusPosAck>(cmdData, cmdInfo->dataLen, ack);
        cb(ret, (CameraModule::TapFocusPosData) ack.p, handler->udata);
      } else if (cmdInfo->cmdId == V1ProtocolCMD::Camera::getPointZoomMode[1]) {
        /*! 0x02c5 */
        /*! 注意,这里把tapzoom的enable和multipler同时获取了 */
        auto cb = (void (*)(ErrorCode::ErrorCodeType, CameraModule::TapZoomEnableAck, UserData)) handler->cb;
        CameraModule::TapZoomEnableAck ack = {0};
        if (ret == ErrorCode::SysCommonErr::Success)
          ret = dataParser<CameraModule::TapZoomEnableAck>(cmdData, cmdInfo->dataLen, ack);
        cb(ret, ack, handler->udata);
      } else if (cmdInfo->cmdId == V1ProtocolCMD::Camera::getIsoParameter[1]) {
        /*! 0x022b */
        auto cb = (void (*)(ErrorCode::ErrorCodeType, CameraModule::ISO, UserData)) handler->cb;
        CameraModule::ISOParamAck ack = {0};
        ack.iso = (CameraModule::ISOParamData)CameraModule::ISO_UNKNOWN;
        if (ret == ErrorCode::SysCommonErr::Success)
          ret = dataParser<CameraModule::ISOParamAck>(cmdData, cmdInfo->dataLen, ack);
        cb(ret, (CameraModule::ISO) ack.iso, handler->udata);
      } else if (cmdInfo->cmdId == V1ProtocolCMD::Camera::getShutterSpeed[1]) {
        /*! 0x0229 */
        auto cb = (void (*)(ErrorCode::ErrorCodeType, CameraModule::ShutterSpeed, UserData)) handler->cb;
        CameraModule::ShutterAck ack = {0};
        if (ret == ErrorCode::SysCommonErr::Success)
          ret = dataParser<CameraModule::ShutterAck>(cmdData, cmdInfo->dataLen, ack);
        auto shutterSpeed = ShutterSpeedTypeToShutterSpeedEnum(
            ack.shutter.reciprocal,
            ack.shutter.integer_part,
            ack.shutter.decimal_part);
        cb(ret, (CameraModule::ShutterSpeed) shutterSpeed, handler->udata);
      } else if (cmdInfo->cmdId == V1ProtocolCMD::Camera::getApertureSize[1]) {
        /*! 0x0227 */
        auto cb = (void (*)(ErrorCode::ErrorCodeType, CameraModule::Aperture, UserData)) handler->cb;
        CameraModule::ApertureAck ack = {0};
        ack.size = (CameraModule::ApertureData)CameraModule::F_UNKNOWN;
        if (ret == ErrorCode::SysCommonErr::Success)
          ret = dataParser<CameraModule::ApertureAck>(cmdData, cmdInfo->dataLen, ack);
        cb(ret, (CameraModule::Aperture) ack.size, handler->udata);
      } else if (cmdInfo->cmdId == V1ProtocolCMD::Camera::getMeteringMode[1]) {
        /*! 0x0223 未实现 */
        DERROR("Not implement the unpack callback yet, system error.");
      } else if (cmdInfo->cmdId == V1ProtocolCMD::Camera::getSpotFocusAera[1]) {
        /*! 0x0231 未实现 */
        DERROR("Not implement the unpack callback yet, system error.");
      } else if (cmdInfo->cmdId == V1ProtocolCMD::Camera::getShotMode[1]) {
        /*! 0x026b */
        auto cb = (void (*)(ErrorCode::ErrorCodeType, CameraModule::CaptureParamData, UserData)) handler->cb;
        CameraModule::CaptureParamAck ack = {0};
        if (ret == ErrorCode::SysCommonErr::Success)
          ret = dataParser<CameraModule::CaptureParamAck>(cmdData, cmdInfo->dataLen, ack);
        cb(ret, ack.captureParam, handler->udata);
      } else {
        DERROR("Unsupport command : [0x%02X, 0x%02X]", cmdInfo->cmdSet,
               cmdInfo->cmdId);
      }
    } else {
      DERROR("Unsupport command : [0x%02X, 0x%02X]", cmdInfo->cmdSet,
             cmdInfo->cmdId);
    }
//clang-format on
    if (handler)free(handler);
  }
}

template<typename DataT>
void CameraModule::getInterfaceAsync(
    const uint8_t cmd[2],
    void (*userCB)(ErrorCode::ErrorCodeType, DataT data, UserData),
    UserData userData, int timeout,
    int retry_time) {

  if (!getEnable() && userCB) {
    DataT data;
    userCB(ErrorCode::SysCommonErr::ReqNotSupported, data, userData);
  }

  T_CmdInfo cmdInfo = {0};
  cmdInfo.cmdSet = cmd[0];
  cmdInfo.cmdId = cmd[1];
  cmdInfo.dataLen = 0;
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.receiver = OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_CAMERA,
                                            getIndex() * 2);
  cmdInfo.sender = getLinker()->getLocalSenderId();

  auto *handler = (handlerType *) malloc(sizeof(handlerType));
  handler->cb = (void *) userCB;
  handler->udata = userData;
  uint8_t temp = 0; // @TODO:fix the linker send data len = 0 issue

  getLinker()->sendAsync(&cmdInfo, &temp, paramAckCB, handler, timeout,
                         retry_time);
}

ErrorCode::ErrorCodeType CameraModule::getInterfaceSync(const uint8_t cmd[2],
                                                        uint8_t *outData,
                                                        uint32_t &outDataLen,
                                                        int timeout,
                                                        int rtyTimes) {
  if (!getEnable()) return ErrorCode::SysCommonErr::ReqNotSupported;

  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t ackData[1024] = {0};
  cmdInfo.receiver = OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_CAMERA,
                                            getIndex() * 2);
  cmdInfo.sender = getLinker()->getLocalSenderId();
  cmdInfo.cmdSet = cmd[0];
  cmdInfo.cmdId = cmd[1];
  cmdInfo.dataLen = 0;
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.encType = 0;
  uint8_t temp = 0; // @TODO:fix the linker send data len = 0 issue
  E_OsdkStat ret =
      getLinker()->sendSync(&cmdInfo, &temp, &ackInfo, ackData,
                            timeout, 3);

  if ((ret == OSDK_STAT_OK) && (outData)) {
    outDataLen = (ackInfo.dataLen < outDataLen) ? ackInfo.dataLen : outDataLen;
    memcpy(outData, ackData, outDataLen); //防止传入的buffer不够大导致内存溢出
    return ErrorCode::getErrorCode(ErrorCode::CameraModule,
                                   ErrorCode::CameraCommon,
                                   outData[0]);
  } else {
    return (ret == OSDK_STAT_ERR_TIMEOUT) ? ErrorCode::SysCommonErr::ReqTimeout
                                          : ErrorCode::SysCommonErr::UndefinedError;
  }
}

void CameraModule::setInterfaceAsync(const uint8_t cmd[2], const uint8_t *pdata,
                                     uint32_t dataLen,
                                     void (*userCB)(ErrorCode::ErrorCodeType,
                                                    UserData),
                                     UserData userData, int timeout,
                                     int retry_time) {
  if (!getEnable() && userCB)
    userCB(ErrorCode::SysCommonErr::ReqNotSupported, userData);

  T_CmdInfo cmdInfo = {0};
  cmdInfo.cmdSet = cmd[0];
  cmdInfo.cmdId = cmd[1];
  cmdInfo.dataLen = dataLen;
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.receiver = OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_CAMERA,
                                            getIndex() * 2);
  cmdInfo.sender = getLinker()->getLocalSenderId();

  auto *handler = (handlerType *) malloc(sizeof(handlerType));
  handler->cb = (void *) userCB;
  handler->udata = userData;

  getLinker()->sendAsync(&cmdInfo, pdata, retAckCB, handler, timeout,
                         retry_time);
}

ErrorCode::ErrorCodeType CameraModule::setInterfaceSync(const uint8_t cmd[2],
                                                        const uint8_t *pdata,
                                                        uint32_t dataLen,
                                                        int timeout,
                                                        uint8_t rtyTimes) {
  if (!getEnable()) return ErrorCode::SysCommonErr::ReqNotSupported;
  uint8_t outData[1024] = {0};

  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  cmdInfo.receiver = OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_CAMERA,
                                            getIndex() * 2);
  cmdInfo.sender = getLinker()->getLocalSenderId();
  cmdInfo.cmdSet = cmd[0];
  cmdInfo.cmdId = cmd[1];
  cmdInfo.dataLen = dataLen;
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.encType = 0;
  E_OsdkStat ret =
      getLinker()->sendSync(&cmdInfo, pdata, &ackInfo, outData,
                            timeout, 3);
  if ((ret == OSDK_STAT_OK) && (outData) && (ackInfo.dataLen > 0)) {
    return ErrorCode::getErrorCode(ErrorCode::CameraModule,
                                   ErrorCode::CameraCommon,
                                   outData[0]);
  } else {
    return (ret == OSDK_STAT_ERR_TIMEOUT) ? ErrorCode::SysCommonErr::ReqTimeout
                                          : ErrorCode::SysCommonErr::UndefinedError;
  }
}

void CameraModule::actionInterfaceAsync(
    const uint8_t cmd[2], const uint8_t *pdata, uint32_t dataLen,
    void (*userCB)(ErrorCode::ErrorCodeType, UserData),
    UserData userData, int timeout, int retry_time) {
  setInterfaceAsync(cmd, pdata, dataLen, userCB, userData, timeout, retry_time);
}

ErrorCode::ErrorCodeType CameraModule::actionInterfaceSync(const uint8_t cmd[2],
                                                           const uint8_t *pdata,
                                                           uint32_t dataLen,
                                                           int timeout,
                                                           uint8_t rtyTimes) {
  return setInterfaceSync(cmd, pdata, dataLen, timeout, rtyTimes);
}

void CameraModule::setExposureModeAsync(
    ExposureMode mode,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {

  if (!getEnable()) {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::ReqNotSupported, userData);
    return;
  }

  ExposureModeReq req = {(ExposureModeData) mode, 0};
  T_CmdInfo cmdInfo = {0};

  cmdInfo.cmdSet = V1ProtocolCMD::Camera::setExposureMode[0];
  cmdInfo.cmdId = V1ProtocolCMD::Camera::setExposureMode[1];
  cmdInfo.dataLen = sizeof(req);
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.receiver = OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_CAMERA,
                                            getIndex() * 2);
  cmdInfo.sender = getLinker()->getLocalSenderId();

  auto *handler = (handlerType *) malloc(sizeof(handlerType));
  handler->cb = (void *)UserCallBack;
  handler->udata = userData;

  getLinker()->sendAsync(&cmdInfo, (uint8_t *) &req, retAckCB, handler, 1000, 3);
}

ErrorCode::ErrorCodeType CameraModule::setExposureModeSync(ExposureMode mode,
                                                           int timeout) {

  ExposureModeReq req = {(ExposureModeData) mode, 0};
  return setInterfaceSync(V1ProtocolCMD::Camera::setExposureMode,
                          (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
}

void CameraModule::getExposureModeAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, ExposureMode mode,
                         UserData userData),
    UserData userData) {
  getInterfaceAsync<ExposureMode>(V1ProtocolCMD::Camera::getExposureMode,
                                  UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::getExposureModeSync(ExposureMode& mode,
                                                           int timeout) {
  uint8_t outData[1024] = {0};
  uint32_t outDataLen = sizeof(outData);
  ErrorCode::ErrorCodeType ret =
      getInterfaceSync(V1ProtocolCMD::Camera::getExposureMode, outData,
                       outDataLen, timeout * 1000 / 3, 3);
  if (ret == ErrorCode::SysCommonErr::Success) {
    mode = (ExposureMode)(((ExposureModeAck *)outData)->exposureMode);
  }
  return ret;
}

void CameraModule::setISOAsync(
    ISO iso,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  ISOParamReq req = {};
  req.iso = iso;
  setInterfaceAsync(V1ProtocolCMD::Camera::setIsoParameter, (uint8_t *) &req,
                    sizeof(req), UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::setISOSync(ISO iso, int timeout) {
  ISOParamReq req = {(ISOParamData)iso};
  return setInterfaceSync(V1ProtocolCMD::Camera::setIsoParameter,
                          (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
}

void CameraModule::getISOAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType,
                                                    ISO iso, UserData userData),
                               UserData userData) {
  getInterfaceAsync<ISO>(V1ProtocolCMD::Camera::getIsoParameter, UserCallBack,
                         userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::getISOSync(ISO& iso, int timeout) {
  uint8_t outData[1024] = {0};
  uint32_t outDataLen = sizeof(outData);
  ErrorCode::ErrorCodeType ret =
      getInterfaceSync(V1ProtocolCMD::Camera::getIsoParameter, outData,
                       outDataLen, timeout * 1000 / 3, 3);
  if (ret == ErrorCode::SysCommonErr::Success) {
    iso = (ISO)(((ISOParamAck *)outData)->iso);
  }
  return ret;
}

void CameraModule::startShootPhotoAsync(
    ShootPhotoMode mode,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  ShootPhotoReq req = {};
  req.takePhotoType = mode;
  actionInterfaceAsync(V1ProtocolCMD::Camera::takePhoto, (uint8_t *) &req,
                    sizeof(req), UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::startShootPhotoSync(ShootPhotoMode mode,
                                                           int timeout) {
  ShootPhotoReq req = {};
  req.takePhotoType = mode;
  return actionInterfaceSync(V1ProtocolCMD::Camera::takePhoto, (uint8_t *) &req,
                             sizeof(req), timeout * 1000 / 3, 3);
}

void CameraModule::stopShootPhotoAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  ShootPhotoReq req = {};
  req.takePhotoType = DJI_CAMERA_TAKE_PHOTO_TYPE_STOP;
  actionInterfaceAsync(V1ProtocolCMD::Camera::takePhoto, (uint8_t *) &req,
                       sizeof(req), UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::stopShootPhotoSync(int timeout) {
  ShootPhotoReq req = {};
  req.takePhotoType = DJI_CAMERA_TAKE_PHOTO_TYPE_STOP;
  return actionInterfaceSync(V1ProtocolCMD::Camera::takePhoto, (uint8_t *) &req,
                             sizeof(req), timeout * 1000 / 3, 3);
}

void CameraModule::startRecordVideoAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  RecordVideoReq req = {};
  req.recording_control = DJI_CAMERA_RECORDING_CONTROL_BEGIN;
  req.recording_type = DJI_CAMERA_RECORDING_TYPE_COMMON;
  actionInterfaceAsync(V1ProtocolCMD::Camera::takeVideo, (uint8_t *) &req,
                       sizeof(req), UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::startRecordVideoSync(int timeout) {
  RecordVideoReq req = {};
  req.recording_control = DJI_CAMERA_RECORDING_CONTROL_BEGIN;
  req.recording_type = DJI_CAMERA_RECORDING_TYPE_COMMON;
  return actionInterfaceSync(V1ProtocolCMD::Camera::takeVideo, (uint8_t *) &req,
                             sizeof(req), timeout * 1000 / 3, 3);
}

void CameraModule::stopRecordVideoAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  RecordVideoReq req = {};
  req.recording_control = DJI_CAMERA_RECORDING_CONTROL_STOP;
  req.recording_type = DJI_CAMERA_RECORDING_TYPE_COMMON;
  actionInterfaceAsync(V1ProtocolCMD::Camera::takeVideo, (uint8_t *) &req,
                       sizeof(req), UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::stopRecordVideoSync(int timeout) {
  RecordVideoReq req = {};
  req.recording_control = DJI_CAMERA_RECORDING_CONTROL_STOP;
  req.recording_type = DJI_CAMERA_RECORDING_TYPE_COMMON;
  return actionInterfaceSync(V1ProtocolCMD::Camera::takeVideo, (uint8_t *) &req,
                             sizeof(req), timeout * 1000 / 3, 3);
}

void CameraModule::setModeAsync(
    WorkMode mode,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  WorkModeReq req = {};
  req.workingMode = mode;
  setInterfaceAsync(V1ProtocolCMD::Camera::setMode, (uint8_t *) &req,
                    sizeof(req), UserCallBack, userData, 3000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::setModeSync(WorkMode mode, int timeout) {
  WorkModeReq req = {(WorkModeData)mode};
  return setInterfaceSync(V1ProtocolCMD::Camera::setMode,
                          (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
}

void CameraModule::getModeAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, WorkMode workingMode,
                         UserData userData),
    UserData userData) {
  getInterfaceAsync<WorkMode>(V1ProtocolCMD::Camera::getMode, UserCallBack,
                              userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::getModeSync(WorkMode& workingMode,
                                                   int timeout) {
  uint8_t outData[1024] = {0};
  uint32_t outDataLen = sizeof(outData);
  ErrorCode::ErrorCodeType ret =
      getInterfaceSync(V1ProtocolCMD::Camera::getMode, outData, outDataLen,
                       timeout * 1000 / 3, 3);
  if (ret == ErrorCode::SysCommonErr::Success) {
    workingMode = (WorkMode)(((WorkModeAck *) outData)->workingMode);
  }
  return ret;
}

void CameraModule::setFocusModeAsync(
    FocusMode mode,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  FocusModeReq req = {};
  req.focusMode = mode;
  setInterfaceAsync(V1ProtocolCMD::Camera::setFocusMode, (uint8_t *) &req,
                    sizeof(req), UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::setFocusModeSync(FocusMode mode,
                                                        int timeout) {
  FocusModeReq req = {(FocusModeData)mode};
  return setInterfaceSync(V1ProtocolCMD::Camera::setFocusMode,
                          (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
}

void CameraModule::getFocusModeAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, FocusMode focusMode,
                         UserData userData),
    UserData userData) {
  getInterfaceAsync<FocusMode>(V1ProtocolCMD::Camera::getFocusMode, UserCallBack,
                               userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::getFocusModeSync(FocusMode& focusMode,
                                                        int timeout) {
  uint8_t outData[1024] = {0};
  uint32_t outDataLen = sizeof(outData);
  ErrorCode::ErrorCodeType ret =
      getInterfaceSync(V1ProtocolCMD::Camera::getFocusMode, outData, outDataLen,
                       timeout * 1000 / 3, 3);
  if (ret == ErrorCode::SysCommonErr::Success) {
    focusMode = (FocusMode)(((FocusModeAck *) outData)->focusMode);
  }
  return ret;
}

void CameraModule::setFocusTargetAsync(
    TapFocusPosData tapFocusPos,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  TapFocusPosReq req = {};
  req.p = tapFocusPos;
  setInterfaceAsync(V1ProtocolCMD::Camera::setSpotFocusAera, (uint8_t *) &req,
                    sizeof(req), UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::setFocusTargetSync(
    TapFocusPosData tapFocusPos, int timeout) {
  TapFocusPosReq req = {tapFocusPos};
  return setInterfaceSync(V1ProtocolCMD::Camera::setSpotFocusAera,
                          (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
}

void CameraModule::tapZoomAtTargetAsync(
    TapZoomPosData tapZoomPos,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  TapZoomPosReq req = {};
  req.p = tapZoomPos;
  setInterfaceAsync(V1ProtocolCMD::Camera::pointZoomCtrl, (uint8_t *) &req,
                    sizeof(req), UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::tapZoomAtTargetSync(
    TapZoomPosData tapZoomPos, int timeout) {
  TapZoomPosReq req = {tapZoomPos};
  return setInterfaceSync(V1ProtocolCMD::Camera::pointZoomCtrl,
                          (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
}

void CameraModule::getFocusTargetAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType, TapFocusPosData tapFocusPos,
                         UserData userData),
    UserData userData) {
  getInterfaceAsync<TapFocusPosData>(V1ProtocolCMD::Camera::getSpotFocusAera,
                                     UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::getFocusTargetSync(
    TapFocusPosData& tapFocusPos, int timeout) {
  uint8_t outData[1024] = {0};
  uint32_t outDataLen = sizeof(outData);
  ErrorCode::ErrorCodeType ret =
      getInterfaceSync(V1ProtocolCMD::Camera::getSpotFocusAera, outData,
                       outDataLen, timeout * 1000 / 3, 3);
  if (ret == ErrorCode::SysCommonErr::Success) {
    tapFocusPos = ((TapFocusPosAck *) outData)->p;
  }
  return ret;
}

void CameraModule::setApertureAsync(
    Aperture size,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  ApertureReq req = {};
  req.size = size;
  setInterfaceAsync(V1ProtocolCMD::Camera::setApertureSize, (uint8_t *) &req,
                    sizeof(req), UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::setApertureSync(Aperture size,
                                                       int timeout) {
  ApertureReq req = {(ApertureData)size};
  return setInterfaceSync(V1ProtocolCMD::Camera::setApertureSize,
                          (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
}

void CameraModule::getApertureAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType, Aperture size,
                         UserData userData),
    UserData userData) {
  getInterfaceAsync<Aperture>(V1ProtocolCMD::Camera::getApertureSize,
                              UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::getApertureSync(Aperture& size,
                                                       int timeout) {
  uint8_t outData[1024] = {0};
  uint32_t outDataLen = sizeof(outData);
  ErrorCode::ErrorCodeType ret =
      getInterfaceSync(V1ProtocolCMD::Camera::getApertureSize, outData,
                       outDataLen, timeout * 1000 / 3, 3);
  if (ret == ErrorCode::SysCommonErr::Success) {
    size = (Aperture)((ApertureAck *) outData)->size;
  }
  return ret;
}

void CameraModule::setShutterSpeedAsync(
    ShutterSpeed shutterSpeed,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  ShutterReq req = {};
  req.shutter_mode = SHUTTER_MANUAL_MODE;
  req.shutterSpeed =
      ShutterSpeedEnumToShutterSpeedType((ShutterSpeed)shutterSpeed);
  setInterfaceAsync(V1ProtocolCMD::Camera::setShutterSpeed, (uint8_t *) &req,
                    sizeof(req), UserCallBack, userData, 1000 / 3, 3);
}

void CameraModule::getShutterSpeedAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         ShutterSpeed shutterSpeed, UserData userData),
    UserData userData) {
  getInterfaceAsync<ShutterSpeed>(V1ProtocolCMD::Camera::getShutterSpeed,
                                  UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::setShutterSpeedSync(
    ShutterSpeed shutterSpeed, int timeout) {
  ShutterReq req = {};
  req.shutter_mode = SHUTTER_MANUAL_MODE;
  req.shutterSpeed =
      ShutterSpeedEnumToShutterSpeedType((ShutterSpeed)shutterSpeed);
  return setInterfaceSync(V1ProtocolCMD::Camera::setShutterSpeed,
                          (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::getShutterSpeedSync(
    ShutterSpeed& shutterSpeed, int timeout) {

  uint8_t outData[1024] = {0};
  uint32_t outDataLen = sizeof(outData);
  ErrorCode::ErrorCodeType ret =
      getInterfaceSync(V1ProtocolCMD::Camera::getShutterSpeed, outData,
                       outDataLen, timeout * 1000 / 3, 3);
  if (ret == ErrorCode::SysCommonErr::Success) {
    ShutterAck ack = *(ShutterAck *)outData;
    shutterSpeed = ShutterSpeedTypeToShutterSpeedEnum(ack.shutter.reciprocal,
                                                      ack.shutter.integer_part,
                                                      ack.shutter.decimal_part);
  }
  return ret;
}

void CameraModule::setExposureCompensationAsync(
    ExposureCompensation ev,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  ExposureCompensationReq req = {};
  req.ev = ev;
  setInterfaceAsync(V1ProtocolCMD::Camera::setEvParameter, (uint8_t *) &req,
                    sizeof(req), UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::setExposureCompensationSync(
    ExposureCompensation ev, int timeout) {
  ExposureCompensationReq req = {(ExposureCompensationData)ev};
  return setInterfaceSync(V1ProtocolCMD::Camera::setEvParameter,
                          (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
}

void CameraModule::getExposureCompensationAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         ExposureCompensation ev, UserData userData),
    UserData userData) {
  getInterfaceAsync<ExposureCompensation>(V1ProtocolCMD::Camera::getEvParameter,
                                          UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::getExposureCompensationSync(
    ExposureCompensation& ev, int timeout) {
  uint8_t outData[1024] = {0};
  uint32_t outDataLen = sizeof(outData);
  ErrorCode::ErrorCodeType ret =
      getInterfaceSync(V1ProtocolCMD::Camera::getEvParameter, outData,
                       outDataLen, timeout * 1000 / 3, 3);
  if (ret == ErrorCode::SysCommonErr::Success) {
    ev = (ExposureCompensation)((ExposureCompensationAck *) outData)->ev_param;
  }
  return ret;
}

void CameraModule::startContinuousOpticalZoomAsync(
    zoomDirectionData zoomDirection, zoomSpeedData zoomSpeed,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  zoomOptiParamReq req = {0};
  req.zoomOptiParam.zoomType = 1;
  req.zoomOptiParam.zoomSpeed = zoomSpeed;
  req.zoomOptiParam.zoomParam.zoomContiParam.zoomDirection = zoomDirection;
  req.zoomOptiParam.zoomParam.zoomContiParam.padding = 0;
  setInterfaceAsync(V1ProtocolCMD::Camera::setZoomParameter, (uint8_t *) &req,
                    sizeof(req), UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::startContinuousOpticalZoomSync(
    zoomDirectionData zoomDirection, zoomSpeedData zoomSpeed, int timeout) {
  zoomOptiParamReq req = {0};
  req.zoomOptiParam.zoomType = 1;
  req.zoomOptiParam.zoomSpeed = zoomSpeed;
  req.zoomOptiParam.zoomParam.zoomContiParam.zoomDirection = zoomDirection;
  req.zoomOptiParam.zoomParam.zoomContiParam.padding = 0;
  return setInterfaceSync(V1ProtocolCMD::Camera::setZoomParameter,
                          (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::setOpticalZoomFactorSync(float factor, int timeout) {
  camera_zoom_data_type req = {0};
  req.zoom_config.optical_zoom_mode = 1;
  req.zoom_config.optical_zoom_enable = 1;
  /*! factor in command struct is starting from 0, do adapting */
  factor = factor - 1;
  if (factor < 0) factor = 0;
  req.optical_zoom_param.pos_param.zoom_pos_level = (uint16_t)(factor * 100);
  return setInterfaceSync(V1ProtocolCMD::Camera::setCommonZoomPara,
                          (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::getOpticalZoomFactorSync(float &factor, int timeout) {
  uint8_t outData[1024] = {0};
  uint32_t outDataLen = sizeof(outData);
  ErrorCode::ErrorCodeType ret = getInterfaceSync(V1ProtocolCMD::Camera::getCommonZoomPara,
      outData, outDataLen, timeout * 1000 / 3, 3);
  if ((ret == ErrorCode::SysCommonErr::Success) && (outData[0] == 0x00)) {
    factor = *(uint16_t *)(outData + 1) / 100.0f;
  }

  return ret;
}

void CameraModule::stopContinuousOpticalZoomAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  zoomOptiParamReq req = {0};
  req.zoomOptiParam.zoomType = 255;
  setInterfaceAsync(V1ProtocolCMD::Camera::setZoomParameter, (uint8_t *) &req,
                    sizeof(req), UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::stopContinuousOpticalZoomSync(
    int timeout) {
  zoomOptiParamReq req = {0};
  req.zoomOptiParam.zoomType = 255;
  return setInterfaceSync(V1ProtocolCMD::Camera::setZoomParameter,
                          (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
}

typedef struct TapZoomEnabledHandler {
  CameraModule* cameraModule;
  CameraModule::TapZoomEnableData enable;
  CameraModule::TapZoomMultiplierData multiplier;
  void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData);
  UserData userData;
} TapZoomEnabledHandler;

void CameraModule::setTapZoomEnabledAsync(
    bool param,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  auto handler = (TapZoomEnabledHandler *)malloc(sizeof(TapZoomEnabledHandler));
  handler->cameraModule = this;
  handler->enable = param;
  handler->UserCallBack = UserCallBack;
  handler->userData = userData;
  getTapZoomMultiplierAsync(callbackToSetTapZoomEnabled, handler);
}

ErrorCode::ErrorCodeType CameraModule::setTapZoomEnabledSync(bool param,
                                                             int timeout) {
  TapZoomMultiplierData multiplier;
  ErrorCode::ErrorCodeType errCode = getTapZoomMultiplierSync(multiplier, 1);
  if (errCode != ErrorCode::SysCommonErr::Success) {
    return errCode;
  } else {
    TapZoomEnableReq req = {};
    req.tapZoomEnable = param;
    req.multiplier = multiplier;
    return setInterfaceSync(V1ProtocolCMD::Camera::setPointZoomMode,
                            (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
  }
}

void CameraModule::callbackToSetTapZoomEnabled(ErrorCode::ErrorCodeType retCode,
                                               TapZoomMultiplierData multiplier,
                                               UserData userData) {
  if (!userData) return;
  TapZoomEnabledHandler handler = *(TapZoomEnabledHandler*)userData;
  if (retCode != ErrorCode::SysCommonErr::Success) {
    if (handler.UserCallBack) handler.UserCallBack(retCode, handler.userData);
  } else {
    TapZoomEnableReq req = {};
    req.tapZoomEnable = handler.enable;
    req.multiplier = multiplier;
    handler.cameraModule->setInterfaceAsync(
        V1ProtocolCMD::Camera::setPointZoomMode, (uint8_t *) &req,
        sizeof(req), handler.UserCallBack, handler.userData, 1000 / 3, 3);
  }
  if (userData) free(userData);
}

void CameraModule::getTapZoomDataAckAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         TapZoomEnableAck data, UserData userData),
    UserData userData) {
  getInterfaceAsync<TapZoomEnableAck>(V1ProtocolCMD::Camera::getPointZoomMode,
                                      UserCallBack, userData, 1000 / 3, 3);
}

void CameraModule::getTapZoomEnabledAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, bool param,
                         UserData userData),
    UserData userData) {
  auto *handler = (handlerType *) malloc(sizeof(handlerType));
  handler->cb = (void *) UserCallBack;
  handler->udata = userData;
  getTapZoomDataAckAsync(getTapZoomEnabledDecoder, handler);
}

ErrorCode::ErrorCodeType CameraModule::getTapZoomEnabledSync(bool& param,
                                                             int timeout) {
  uint8_t outData[1024] = {0};
  uint32_t outDataLen = sizeof(outData);
  ErrorCode::ErrorCodeType ret =
      getInterfaceSync(V1ProtocolCMD::Camera::getPointZoomMode, outData,
                       outDataLen, timeout * 1000 / 3, 3);
  if (ret == ErrorCode::SysCommonErr::Success) {
    param = (bool)((TapZoomEnableAck *) outData)->tapZoomEnable;
  }
  return ret;
}

void CameraModule::setTapZoomMultiplierAsync(
    TapZoomMultiplierData param,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  auto handler = (TapZoomEnabledHandler *)malloc(sizeof(TapZoomEnabledHandler));
  handler->cameraModule = this;
  handler->enable = false;
  handler->multiplier = param;
  handler->UserCallBack = UserCallBack;
  handler->userData = userData;
  getTapZoomEnabledAsync(callbackToSetTapZoomMultiplier, handler);
}

ErrorCode::ErrorCodeType CameraModule::setTapZoomMultiplierSync(
    TapZoomMultiplierData param, int timeout) {
  bool enableData;
  ErrorCode::ErrorCodeType errCode = getTapZoomEnabledSync(enableData, 1);
  if (errCode != ErrorCode::SysCommonErr::Success) {
    return errCode;
  } else {
    TapZoomEnableReq req = {};
    req.tapZoomEnable = enableData;
    req.multiplier = param;
    return setInterfaceSync(V1ProtocolCMD::Camera::setPointZoomMode,
                            (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
  }
}

void CameraModule::callbackToSetTapZoomMultiplier(
    ErrorCode::ErrorCodeType retCode, bool enable, UserData userData) {
  if (!userData) return;
  TapZoomEnabledHandler handler = *(TapZoomEnabledHandler*)userData;
  if (retCode != ErrorCode::SysCommonErr::Success) {
    if (handler.UserCallBack) handler.UserCallBack(retCode, handler.userData);
  } else {
    TapZoomEnableReq req = {};
    req.tapZoomEnable = enable;
    req.multiplier = handler.multiplier;
    handler.cameraModule->setInterfaceAsync(
        V1ProtocolCMD::Camera::setPointZoomMode, (uint8_t *) &req,
        sizeof(req), handler.UserCallBack, handler.userData, 1000 / 3, 3);
  }
  if (userData) free(userData);
}

void CameraModule::getTapZoomMultiplierAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         TapZoomMultiplierData param, UserData userData),
    UserData userData) {
  auto *handler = (handlerType *) malloc(sizeof(handlerType));
  handler->cb = (void *) UserCallBack;
  handler->udata = userData;
  getTapZoomDataAckAsync(getTapZoomMultiplierDecoder, handler);
}

ErrorCode::ErrorCodeType CameraModule::getTapZoomMultiplierSync(
    TapZoomMultiplierData& param, int timeout) {
  uint8_t outData[1024] = {0};
  uint32_t outDataLen = sizeof(outData);
  ErrorCode::ErrorCodeType ret =
      getInterfaceSync(V1ProtocolCMD::Camera::getPointZoomMode, outData,
                       outDataLen, timeout * 1000 / 3, 3);
  if (ret == ErrorCode::SysCommonErr::Success) {
    param = (TapZoomMultiplierData)((TapZoomEnableAck *) outData)->multiplier;
  }
  return ret;
}

void CameraModule::getShootPhotoModeDataDecoder(ErrorCode::ErrorCodeType retCode,
                                                CaptureParamData captureParam,
                                                UserData userData) {
  if (!userData) return;
  auto *handler = (handlerType *) userData;
  auto cb = (void (*)(ErrorCode::ErrorCodeType, CameraModule::ShootPhotoMode, UserData)) handler->cb;
  if(cb) cb(retCode, (CameraModule::ShootPhotoMode)captureParam.captureMode, handler->udata);
  free(userData);
}

void CameraModule::getPhotoAEBCountDecoder(ErrorCode::ErrorCodeType retCode,
                                           CaptureParamData captureParam,
                                           UserData userData) {
  if (!userData) return;
  auto *handler = (handlerType *) userData;
  auto cb = (void (*)(ErrorCode::ErrorCodeType, CameraModule::PhotoAEBCount, UserData)) handler->cb;
  if(cb) cb(retCode, (CameraModule::PhotoAEBCount)captureParam.photoNumBurst, handler->udata);
  free(userData);
}

void CameraModule::getPhotoBurstCountDecoder(ErrorCode::ErrorCodeType retCode,
                                             CaptureParamData captureParam,
                                             UserData userData) {
  if (!userData) return;
  auto *handler = (handlerType *) userData;
  auto cb = (void (*)(ErrorCode::ErrorCodeType, CameraModule::PhotoBurstCount, UserData)) handler->cb;
  if(cb) cb(retCode, (CameraModule::PhotoBurstCount)captureParam.photoNumBurst, handler->udata);
  free(userData);
}

void CameraModule::getPhotoIntervalDatasDecoder(ErrorCode::ErrorCodeType retCode,
                                                CaptureParamData captureParam,
                                                UserData userData) {
  if (!userData) return;
  auto *handler = (handlerType *) userData;
  auto cb = (void (*)(ErrorCode::ErrorCodeType, PhotoIntervalData, UserData)) handler->cb;
  if(cb) cb(retCode, captureParam.intervalSetting, handler->udata);
  free(userData);
}

void CameraModule::getTapZoomEnabledDecoder(ErrorCode::ErrorCodeType retCode,
                                            TapZoomEnableAck data,
                                            UserData userData) {
  if (!userData) return;
  auto *handler = (handlerType *) userData;
  auto cb = (void (*)(ErrorCode::ErrorCodeType, bool, UserData)) handler->cb;
  if(cb) cb(retCode, data.tapZoomEnable, handler->udata);
  free(userData);
}

void CameraModule::getTapZoomMultiplierDecoder(ErrorCode::ErrorCodeType retCode,
                                               TapZoomEnableAck data,
                                               UserData userData) {
  if (!userData) return;
  auto *handler = (handlerType *) userData;
  auto cb = (void (*)(ErrorCode::ErrorCodeType, TapZoomMultiplierData, UserData)) handler->cb;
  if(cb) cb(retCode, data.multiplier, handler->udata);
  free(userData);
}

CameraModule::ShutterSpeedType createShutterSpeedStruct(
    int reciprocal, int integer_part, int decimal_part) {
  CameraModule::ShutterSpeedType speed;
  speed.reciprocal = reciprocal;
  speed.integer_part = integer_part;
  speed.decimal_part = decimal_part;
  return speed;
}

CameraModule::ShutterSpeedType ShutterSpeedEnumToShutterSpeedType(
    CameraModule::ShutterSpeed shutterSpeed) {
  CameraModule::ShutterSpeedType type = {0};
  switch (shutterSpeed) {
    case CameraModule::CameraModule::ShutterSpeed::SHUTTER_SPEED_1_8000:
      type = createShutterSpeedStruct(1, 8000, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_6400:
      type = createShutterSpeedStruct(1, 6400, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_6000:
      type = createShutterSpeedStruct(1, 6000, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_5000:
      type = createShutterSpeedStruct(1, 5000, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_4000:
      type = createShutterSpeedStruct(1, 4000, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_3200:
      type = createShutterSpeedStruct(1, 3200, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_3000:
      type = createShutterSpeedStruct(1, 3000, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_2500:
      type = createShutterSpeedStruct(1, 2500, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_2000:
      type = createShutterSpeedStruct(1, 2000, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_1500:
      type = createShutterSpeedStruct(1, 1500, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_1600:
      type = createShutterSpeedStruct(1, 1600, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_1250:
      type = createShutterSpeedStruct(1, 1250, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_1000:
      type = createShutterSpeedStruct(1, 1000, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_800:
      type = createShutterSpeedStruct(1, 800, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_725:
      type = createShutterSpeedStruct(1, 725, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_640:
      type = createShutterSpeedStruct(1, 640, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_500:
      type = createShutterSpeedStruct(1, 500, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_400:
      type = createShutterSpeedStruct(1, 400, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_350:
      type = createShutterSpeedStruct(1, 350, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_320:
      type = createShutterSpeedStruct(1, 320, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_250:
      type = createShutterSpeedStruct(1, 250, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_240:
      type = createShutterSpeedStruct(1, 240, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_200:
      type = createShutterSpeedStruct(1, 200, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_180:
      type = createShutterSpeedStruct(1, 180, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_160:
      type = createShutterSpeedStruct(1, 160, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_125:
      type = createShutterSpeedStruct(1, 125, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_120:
      type = createShutterSpeedStruct(1, 120, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_100:
      type = createShutterSpeedStruct(1, 100, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_90:
      type = createShutterSpeedStruct(1, 90, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_80:
      type = createShutterSpeedStruct(1, 80, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_60:
      type = createShutterSpeedStruct(1, 60, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_50:
      type = createShutterSpeedStruct(1, 50, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_40:
      type = createShutterSpeedStruct(1, 40, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_30:
      type = createShutterSpeedStruct(1, 30, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_25:
      type = createShutterSpeedStruct(1, 25, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_20:
      type = createShutterSpeedStruct(1, 20, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_15:
      type = createShutterSpeedStruct(1, 15, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_12DOT5:
      type = createShutterSpeedStruct(1, 12, 5);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_10:
      type = createShutterSpeedStruct(1, 10, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_8:
      type = createShutterSpeedStruct(1, 8, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_6DOT25:
      type = createShutterSpeedStruct(1, 6, 25);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_5:
      type = createShutterSpeedStruct(1, 5, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_4:
      type = createShutterSpeedStruct(1, 4, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_3:
      type = createShutterSpeedStruct(1, 3, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_2DOT5:
      type = createShutterSpeedStruct(1, 2, 5);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_2:
      type = createShutterSpeedStruct(1, 2, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_1DOT67:
      type = createShutterSpeedStruct(1, 1, 67);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1_1DOT25:
      type = createShutterSpeedStruct(1, 1, 25);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1:
      type = createShutterSpeedStruct(0, 1, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1DOT3:
      type = createShutterSpeedStruct(0, 1, 3);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_1DOT6:
      type = createShutterSpeedStruct(0, 1, 6);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_2:
      type = createShutterSpeedStruct(0, 2, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_2DOT5:
      type = createShutterSpeedStruct(0, 2, 5);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_3:
      type = createShutterSpeedStruct(0, 3, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_3DOT2:
      type = createShutterSpeedStruct(0, 3, 2);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_4:
      type = createShutterSpeedStruct(0, 4, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_5:
      type = createShutterSpeedStruct(0, 5, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_6:
      type = createShutterSpeedStruct(0, 6, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_7:
      type = createShutterSpeedStruct(0, 7, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_8:
      type = createShutterSpeedStruct(0, 8, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_9:
      type = createShutterSpeedStruct(0, 9, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_10:
      type = createShutterSpeedStruct(0, 10, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_13:
      type = createShutterSpeedStruct(0, 13, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_15:
      type = createShutterSpeedStruct(0, 15, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_20:
      type = createShutterSpeedStruct(0, 20, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_25:
      type = createShutterSpeedStruct(0, 25, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_30:
      type = createShutterSpeedStruct(0, 30, 0);
      break;
    case CameraModule::ShutterSpeed::SHUTTER_SPEED_UNKNOWN:
      break;
  }

  return type;
}

CameraModule::ShutterSpeed ShutterSpeedTypeToShutterSpeedEnum(
    int reciprocal, int integer_part, int decimal_part) {
  CameraModule::ShutterSpeed type =
      CameraModule::ShutterSpeed::SHUTTER_SPEED_UNKNOWN;

  if (reciprocal == 1 && integer_part == 8000 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_8000;
  if (reciprocal == 1 && integer_part == 6400 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_6400;
  if (reciprocal == 1 && integer_part == 6000 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_6000;
  if (reciprocal == 1 && integer_part == 5000 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_5000;
  if (reciprocal == 1 && integer_part == 4000 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_4000;
  if (reciprocal == 1 && integer_part == 3200 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_3200;
  if (reciprocal == 1 && integer_part == 3000 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_3000;
  if (reciprocal == 1 && integer_part == 2500 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_2500;
  if (reciprocal == 1 && integer_part == 2000 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_2000;
  if (reciprocal == 1 && integer_part == 1500 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_1500;
  if (reciprocal == 1 && integer_part == 1600 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_1600;
  if (reciprocal == 1 && integer_part == 1250 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_1250;
  if (reciprocal == 1 && integer_part == 1000 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_1000;
  if (reciprocal == 1 && integer_part == 800 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_800;
  if (reciprocal == 1 && integer_part == 725 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_725;
  if (reciprocal == 1 && integer_part == 640 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_640;
  if (reciprocal == 1 && integer_part == 500 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_500;
  if (reciprocal == 1 && integer_part == 400 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_400;
  if (reciprocal == 1 && integer_part == 350 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_350;
  if (reciprocal == 1 && integer_part == 320 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_320;
  if (reciprocal == 1 && integer_part == 250 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_250;
  if (reciprocal == 1 && integer_part == 240 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_240;
  if (reciprocal == 1 && integer_part == 200 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_200;
  if (reciprocal == 1 && integer_part == 180 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_180;
  if (reciprocal == 1 && integer_part == 160 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_160;
  if (reciprocal == 1 && integer_part == 125 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_125;
  if (reciprocal == 1 && integer_part == 120 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_120;
  if (reciprocal == 1 && integer_part == 100 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_100;
  if (reciprocal == 1 && integer_part == 90 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_90;
  if (reciprocal == 1 && integer_part == 80 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_80;
  if (reciprocal == 1 && integer_part == 60 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_60;
  if (reciprocal == 1 && integer_part == 50 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_50;
  if (reciprocal == 1 && integer_part == 40 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_40;
  if (reciprocal == 1 && integer_part == 30 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_30;
  if (reciprocal == 1 && integer_part == 25 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_25;
  if (reciprocal == 1 && integer_part == 20 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_20;
  if (reciprocal == 1 && integer_part == 15 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_15;
  if (reciprocal == 1 && integer_part == 12 && decimal_part == 5)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_12DOT5;
  if (reciprocal == 1 && integer_part == 10 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_10;
  if (reciprocal == 1 && integer_part == 8 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_8;
  if (reciprocal == 1 && integer_part == 6 && decimal_part == 25)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_6DOT25;
  if (reciprocal == 1 && integer_part == 5 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_5;
  if (reciprocal == 1 && integer_part == 4 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_4;
  if (reciprocal == 1 && integer_part == 3 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_3;
  if (reciprocal == 1 && integer_part == 2 && decimal_part == 5)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_2DOT5;
  if (reciprocal == 1 && integer_part == 2 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_2;
  if (reciprocal == 1 && integer_part == 1 && decimal_part == 67)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_1DOT67;
  if (reciprocal == 1 && integer_part == 1 && decimal_part == 25)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1_1DOT25;
  if (reciprocal == 0 && integer_part == 1 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1;
  if (reciprocal == 0 && integer_part == 1 && decimal_part == 3)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1DOT3;
  if (reciprocal == 0 && integer_part == 1 && decimal_part == 6)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_1DOT6;
  if (reciprocal == 0 && integer_part == 2 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_2;
  if (reciprocal == 0 && integer_part == 2 && decimal_part == 5)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_2DOT5;
  if (reciprocal == 0 && integer_part == 3 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_3;
  if (reciprocal == 0 && integer_part == 3 && decimal_part == 2)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_3DOT2;
  if (reciprocal == 0 && integer_part == 4 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_4;
  if (reciprocal == 0 && integer_part == 5 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_5;
  if (reciprocal == 0 && integer_part == 6 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_6;
  if (reciprocal == 0 && integer_part == 7 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_7;
  if (reciprocal == 0 && integer_part == 8 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_8;
  if (reciprocal == 0 && integer_part == 9 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_9;
  if (reciprocal == 0 && integer_part == 10 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_10;
  if (reciprocal == 0 && integer_part == 13 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_13;
  if (reciprocal == 0 && integer_part == 15 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_15;
  if (reciprocal == 0 && integer_part == 20 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_20;
  if (reciprocal == 0 && integer_part == 25 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_25;
  if (reciprocal == 0 && integer_part == 30 && decimal_part == 0)
    type = CameraModule::ShutterSpeed::SHUTTER_SPEED_30;

  return type;
}

void CameraModule::getCaptureParamDataAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         CaptureParamData captureParam, UserData userData),
    UserData userData) {
  getInterfaceAsync<CaptureParamData>(V1ProtocolCMD::Camera::getShotMode,
                                      UserCallBack, userData, 1000 / 3, 3);
}

ErrorCode::ErrorCodeType CameraModule::getCaptureParamDataSync(
    CaptureParamData& captureParam, int timeout) {
  uint8_t outData[1024] = {0};
  uint32_t outDataLen = sizeof(outData);
  ErrorCode::ErrorCodeType ret =
      getInterfaceSync(V1ProtocolCMD::Camera::getShotMode, outData,
                       outDataLen, timeout * 1000 / 3, 3);
  if (ret == ErrorCode::SysCommonErr::Success) {
    captureParam = ((CaptureParamAck *) outData)->captureParam;
  }
  return ret;
}

typedef struct shootPhotoParamHandler {
  CameraModule* cameraModule;
  CameraModule::CaptureParamData paramData;
  void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData);
  UserData userData;
} shootPhotoParamHandler;

void CameraModule::callbackToSetShootPhotoMode(ErrorCode::ErrorCodeType retCode,
                                               CaptureParamData captureParam,
                                               UserData userData) {
  if (!userData) return;
  shootPhotoParamHandler handler = *(shootPhotoParamHandler*)userData;
  if (retCode != ErrorCode::SysCommonErr::Success) {
    if (handler.UserCallBack) handler.UserCallBack(retCode, handler.userData);
  } else {
    CaptureParamReq req = {};
    req.captureParam = captureParam;
    req.captureParam.captureMode = handler.paramData.captureMode;

    handler.cameraModule->setInterfaceAsync(
        V1ProtocolCMD::Camera::setShotMode, (uint8_t *) &req,
        sizeof(req), handler.UserCallBack, handler.userData, 1000 / 3, 3);
  }
}

void CameraModule::setShootPhotoModeAsync(
    ShootPhotoMode takePhotoMode,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  auto handler = (shootPhotoParamHandler*)malloc(sizeof(shootPhotoParamHandler));
  handler->cameraModule = this;
  handler->paramData.captureMode = takePhotoMode;
  handler->UserCallBack = UserCallBack;
  handler->userData = userData;
  getCaptureParamDataAsync(callbackToSetShootPhotoMode, handler);
}

ErrorCode::ErrorCodeType CameraModule::setShootPhotoModeSync(
    ShootPhotoMode takePhotoMode, int timeout) {
  CaptureParamData captureParamData;
  ErrorCode::ErrorCodeType errCode =
      getCaptureParamDataSync(captureParamData, 1);
  if (errCode != ErrorCode::SysCommonErr::Success) {
    return errCode;
  } else {
    CaptureParamReq req = {};
    /*! @TODO Here strange code is to deal with issue that the AEB and BURST
     * share the same setting and getting CMD and the same byte in the
     * protocol. After the camera info pushing is supported by OSDK, here will
     * be fixed. */
    if (takePhotoMode == ShootPhotoMode::AEB) {
      if (captureParamData.photoNumBurst > AEB_COUNT_5)
      captureParamData.photoNumBurst = AEB_COUNT_5;
      else if (captureParamData.photoNumBurst < AEB_COUNT_3)
        captureParamData.photoNumBurst = AEB_COUNT_3;
    }
    req.captureParam = captureParamData;
    req.captureParam.captureMode = takePhotoMode;
    return setInterfaceSync(V1ProtocolCMD::Camera::setShotMode,
                            (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
  }
}

void CameraModule::getShootPhotoModeAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         ShootPhotoMode takePhotoMode, UserData userData),
    UserData userData) {
  auto *handler = (handlerType *) malloc(sizeof(handlerType));
  handler->cb = (void *) UserCallBack;
  handler->udata = userData;
  getCaptureParamDataAsync(getShootPhotoModeDataDecoder, handler);
}

ErrorCode::ErrorCodeType CameraModule::getShootPhotoModeSync(
    ShootPhotoMode& takePhotoMode, int timeout) {
  CaptureParamData captureParam;
  ErrorCode::ErrorCodeType errCode =
      getCaptureParamDataSync(captureParam, timeout);
  if (errCode == ErrorCode::SysCommonErr::Success)
    takePhotoMode = (ShootPhotoMode)captureParam.captureMode;
  return errCode;
}

void CameraModule::callbackToSetPhotoBurstCount(
    ErrorCode::ErrorCodeType retCode, CaptureParamData captureParam,
    UserData userData) {
  if (!userData) return;
  shootPhotoParamHandler handler = *(shootPhotoParamHandler*)userData;
  if (retCode != ErrorCode::SysCommonErr::Success) {
    if (handler.UserCallBack) handler.UserCallBack(retCode, handler.userData);
  } else {
    CaptureParamReq req = {};
    req.captureParam = captureParam;
    req.captureParam.photoNumBurst = handler.paramData.photoNumBurst;

    handler.cameraModule->setInterfaceAsync(
        V1ProtocolCMD::Camera::setShotMode, (uint8_t *) &req,
        sizeof(req), handler.UserCallBack, handler.userData, 1000 / 3, 3);
  }
}

void CameraModule::setPhotoBurstCountAsync(
    PhotoBurstCount count,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  auto handler = (shootPhotoParamHandler*)malloc(sizeof(shootPhotoParamHandler));
  handler->cameraModule = this;
  handler->paramData.photoNumBurst = count;
  handler->UserCallBack = UserCallBack;
  handler->userData = userData;
  getCaptureParamDataAsync(callbackToSetPhotoBurstCount, handler);
}

ErrorCode::ErrorCodeType CameraModule::setPhotoBurstCountSync(
    PhotoBurstCount count, int timeout) {
  CaptureParamData captureParamData;
  ErrorCode::ErrorCodeType errCode =
      getCaptureParamDataSync(captureParamData, 1);
  if (errCode != ErrorCode::SysCommonErr::Success) {
    return errCode;
  } else {
    CaptureParamReq req = {};
    req.captureParam = captureParamData;
    req.captureParam.photoNumBurst = count;
    return setInterfaceSync(V1ProtocolCMD::Camera::setShotMode,
                            (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
  }
}

void CameraModule::getPhotoBurstCountAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         PhotoBurstCount count, UserData userData),
    UserData userData) {
  auto *handler = (handlerType *) malloc(sizeof(handlerType));
  handler->cb = (void *) UserCallBack;
  handler->udata = userData;
  getCaptureParamDataAsync(getPhotoBurstCountDecoder, handler);
}

ErrorCode::ErrorCodeType CameraModule::getPhotoBurstCountSync(
    PhotoBurstCount& count, int timeout) {
  CaptureParamData captureParam;
  ErrorCode::ErrorCodeType errCode =
      getCaptureParamDataSync(captureParam, timeout);
  if (errCode == ErrorCode::SysCommonErr::Success)
    count = (PhotoBurstCount)captureParam.photoNumBurst;
  return errCode;
}

void CameraModule::setPhotoAEBCountAsync(
    PhotoAEBCount count,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  setPhotoBurstCountAsync((PhotoBurstCount)count, UserCallBack, userData);
}

void CameraModule::getPhotoAEBCountAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, PhotoAEBCount count,
                         UserData userData),
    UserData userData) {
  auto *handler = (handlerType *) malloc(sizeof(handlerType));
  handler->cb = (void *) UserCallBack;
  handler->udata = userData;
  getCaptureParamDataAsync(getPhotoAEBCountDecoder, handler);
}

ErrorCode::ErrorCodeType CameraModule::setPhotoAEBCountSync(PhotoAEBCount count,
                                                            int timeout) {
  return setPhotoBurstCountSync((PhotoBurstCount)count, timeout);
}

ErrorCode::ErrorCodeType CameraModule::getPhotoAEBCountSync(
    PhotoAEBCount& count, int timeout) {
  CaptureParamData captureParam;
  ErrorCode::ErrorCodeType errCode =
      getCaptureParamDataSync(captureParam, timeout);
  if (errCode == ErrorCode::SysCommonErr::Success)
    count = (PhotoAEBCount)captureParam.photoNumBurst;
  return errCode;
}

void CameraModule::callbackToSetPhotoTimeIntervalSettings(
    ErrorCode::ErrorCodeType retCode, CaptureParamData captureParam,
    UserData userData) {
  if (!userData) return;
  shootPhotoParamHandler handler = *(shootPhotoParamHandler*)userData;
  if (retCode != ErrorCode::SysCommonErr::Success) {
    if (handler.UserCallBack) handler.UserCallBack(retCode, handler.userData);
  } else {
    CaptureParamReq req = {};
    req.captureParam = captureParam;
    req.captureParam.intervalSetting = handler.paramData.intervalSetting;

    handler.cameraModule->setInterfaceAsync(
        V1ProtocolCMD::Camera::setShotMode, (uint8_t *) &req,
        sizeof(req), handler.UserCallBack, handler.userData, 1000 / 3, 3);
  }
}

void CameraModule::setPhotoTimeIntervalSettingsAsync(
    PhotoIntervalData intervalSetting,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  auto handler = (shootPhotoParamHandler*)malloc(sizeof(shootPhotoParamHandler));
  handler->cameraModule = this;
  handler->paramData.intervalSetting = intervalSetting;
  handler->UserCallBack = UserCallBack;
  handler->userData = userData;
  getCaptureParamDataAsync(callbackToSetPhotoTimeIntervalSettings, handler);
}

ErrorCode::ErrorCodeType CameraModule::setPhotoTimeIntervalSettingsSync(
    PhotoIntervalData intervalSetting, int timeout) {
  CaptureParamData captureParamData;
  ErrorCode::ErrorCodeType errCode =
      getCaptureParamDataSync(captureParamData, 1);
  if (errCode != ErrorCode::SysCommonErr::Success) {
    return errCode;
  } else {
    CaptureParamReq req = {};
    req.captureParam = captureParamData;
    req.captureParam.intervalSetting = intervalSetting;
    return setInterfaceSync(V1ProtocolCMD::Camera::setShotMode,
                            (uint8_t *) &req, sizeof(req), timeout * 1000 / 3, 3);
  }
}

void CameraModule::getPhotoIntervalDatasAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         PhotoIntervalData intervalSetting, UserData userData),
    UserData userData) {
  auto *handler = (handlerType *) malloc(sizeof(handlerType));
  handler->cb = (void *) UserCallBack;
  handler->udata = userData;
  getCaptureParamDataAsync(getPhotoIntervalDatasDecoder, handler);
}

ErrorCode::ErrorCodeType CameraModule::getPhotoIntervalDatasSync(
    PhotoIntervalData& intervalSetting, int timeout) {
  CaptureParamData captureParam;
  ErrorCode::ErrorCodeType errCode =
      getCaptureParamDataSync(captureParam, timeout);
  if (errCode == ErrorCode::SysCommonErr::Success)
    intervalSetting = captureParam.intervalSetting;
  return errCode;
}

ErrorCode::ErrorCodeType CameraModule::obtainDownloadRightSync(bool enable,
                                                           int timeout) {
#pragma pack(1)
  /*! Obtain the download right from liveview (0x49,0x20) , 该协议保密 */
  typedef struct download_right_req {
    //设备主类型,相机(0),雷达(1),PSDK(2) refer to MAJOR_TYPE
    uint8_t major_type;
    //camera(0), PSDK(2) refer to MINOR_TYPE
    uint8_t minor_type;
    //设备位置信息,1号云台(0),2号云台(1),上置云台(2),
    uint8_t dev_pos;
    //默认为0,低2位用作相机下载数据标示,其余6位预留
    uint8_t cam_data_idex;
    //1:抢夺控制权 0:释放控制权
    uint8_t data_download_ctrl_right;
  } download_right_req;
#pragma pack()

  download_right_req data = {0};
  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t ackData[1024];

  data.major_type = 0; //camera
  data.minor_type = 0; //camera
  data.dev_pos = this->getIndex();
  data.cam_data_idex = 0;
  data.data_download_ctrl_right = (enable == true) ? 1 : 0;

  cmdInfo.cmdSet = V1ProtocolCMD::SDK::obtainDownloadRight[0];
  cmdInfo.cmdId = V1ProtocolCMD::SDK::obtainDownloadRight[1];
  cmdInfo.dataLen = sizeof(data);
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.receiver =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_AB_DM368, 0);
  cmdInfo.sender = getLinker()->getLocalSenderId();

  E_OsdkStat linkAck =
      getLinker()->sendSync(&cmdInfo, (uint8_t *) &data, &ackInfo, ackData,
                            timeout * 1000 / 4, 4);

  ErrorCode::ErrorCodeType ret = ErrorCode::getLinkerErrorCode(linkAck);
  if (ret != ErrorCode::SysCommonErr::Success) return ret;
  return ErrorCode::getErrorCode(ErrorCode::CameraModule,
                                 ErrorCode::CameraCommon, ackData[0]);
}
