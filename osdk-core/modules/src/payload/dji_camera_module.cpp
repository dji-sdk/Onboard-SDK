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

#include "dji_camera_module.hpp"
#include "dji_vehicle_callback.hpp"

using namespace DJI;
using namespace DJI::OSDK;

CameraModule::CameraModule(PayloadLink* payloadLink,
                           PayloadIndexType payloadIndex, std::string name,
                           bool enable)
    : PayloadBase(payloadIndex, name, enable), payloadLink(payloadLink) {}

CameraModule::~CameraModule() {}

template <typename AckT>
ErrCode::ErrCodeType CameraModule::commonDataUnpacker(RecvContainer recvFrame,
                                                      AckT& ack) {
  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin >= sizeof(AckT)) {
    ack = *(AckT*)(recvFrame.recvData.raw_ack_array);
    return ErrCode::errorCode(ErrCode::CameraModule, ErrCode::CameraCommon,
                              ack.ret_code);
  } else {
    DERROR("ACK is exception, data len %d (expect >= %d)\n",
           recvFrame.recvInfo.len - OpenProtocol::PackageMin, sizeof(AckT));
    return ErrCode::SysCommonErr::InvalidRespond;
  }
}

template <typename DataT>
void CameraModule::getInterfaceAsync(
    FuncParam req,
    void (*ackDecoderCB)(Vehicle* vehicle, RecvContainer recvFrame,
                         UCBRetParamHandler<DataT>* ucb),
    void (*userCB)(ErrCode::ErrCodeType, DataT data, UserData userData),
    UserData userData, int timeout, int retry_time) {
  if (getEnable()) {
    payloadLink->sendAsync(OpenProtocolCMD::CMDSet::Control::extendedFunction,
                           &req, sizeof(req), (void*)ackDecoderCB,
                           allocUCBHandler((void*)userCB, userData), timeout,
                           retry_time);
  } else {
    DataT data;
    if (userCB)
      userCB(ErrCode::SysCommonErr::ReqHandlerNotFound, data, userData);
  }
}

template <typename AckT>
ErrCode::ErrCodeType CameraModule::getInterfaceSync(FuncParam req, AckT& ack,
                                                    int timeout) {
  if (getEnable()) {
    ACK::ExtendedFunctionRsp* rsp = payloadLink->sendSync(
        OpenProtocolCMD::CMDSet::Control::extendedFunction, &req, sizeof(req),
        timeout);
    if (rsp->updated && rsp->info.buf &&
        (rsp->info.len - OpenProtocol::PackageMin >= sizeof(AckT))) {
      ack = (*(AckT*)rsp->info.buf);
      return ErrCode::errorCode(ErrCode::CameraModule, ErrCode::CameraCommon,
                                ack.ret_code);
    } else {
      if (!rsp->updated)
        return ErrCode::SysCommonErr::ReqTimeout;
      else
        return ErrCode::SysCommonErr::InvalidRespond;
    }
  }
  return ErrCode::SysCommonErr::ReqHandlerNotFound;
}

template <typename ReqT>
void CameraModule::setInterfaceAsync(
    ReqT req,
    void (*ackDecoderCB)(Vehicle* vehicle, RecvContainer recvFrame,
                         UCBRetCodeHandler* ucb),
    void (*userCB)(ErrCode::ErrCodeType, UserData userData), UserData userData,
    int timeout, int retry_time) {
  if (getEnable()) {
    payloadLink->sendAsync(OpenProtocolCMD::CMDSet::Control::extendedFunction,
                           &req, sizeof(req), (void*)ackDecoderCB,
                           allocUCBHandler((void*)userCB, userData), timeout,
                           retry_time);
  } else {
    if (userCB) userCB(ErrCode::SysCommonErr::ReqHandlerNotFound, userData);
  }
}

template <typename ReqT>
ErrCode::ErrCodeType CameraModule::setInterfaceSync(ReqT req, int timeout) {
  if (getEnable()) {
    ACK::ExtendedFunctionRsp* rsp = payloadLink->sendSync(
        OpenProtocolCMD::CMDSet::Control::extendedFunction, &req, sizeof(ReqT),
        timeout);
    if (rsp->updated && rsp->info.buf &&
        (rsp->info.len - OpenProtocol::PackageMin >= sizeof(CommonAck))) {
      return ErrCode::errorCode(ErrCode::CameraModule, ErrCode::CameraCommon,
                                ((CommonAck*)rsp->info.buf)->ret_code);
    } else {
      if (!rsp->updated)
        return ErrCode::SysCommonErr::ReqTimeout;
      else
        return ErrCode::SysCommonErr::InvalidRespond;
    }
  }
  return ErrCode::SysCommonErr::ReqHandlerNotFound;
}

template <typename ReqT>
void CameraModule::actionInterfaceAsync(
    ReqT req,
    void (*ackDecoderCB)(Vehicle* vehicle, RecvContainer recvFrame,
                         UCBRetCodeHandler* ucb),
    void (*userCB)(ErrCode::ErrCodeType, UserData userData), UserData userData,
    int timeout, int retry_time) {
  setInterfaceAsync<ReqT>(req, ackDecoderCB, userCB, userData, timeout,
                          retry_time);
}

template <typename ReqT>
ErrCode::ErrCodeType CameraModule::actionInterfaceSync(ReqT req, int timeout) {
  setInterfaceSync<ReqT>(req, timeout);
}

/*! @TODO Here is only the temporary way to alloc memory for the asynchronous
 * interface to stash the user data. This method will be optimized in the
 * future.
 * @Note There are 32 memory units for this method. So if the API calling
 * this method too fast, the memory units of this API may overflow. So the
 * calling frequency of releated APIs should less than 10Hz */
CameraModule::UCBRetCodeHandler* CameraModule::allocUCBHandler(
    void* callback, UserData userData) {
  static int ucbHandlerIndex = 0;

  ucbHandlerIndex++;
  if (ucbHandlerIndex >= maxSize) {
    ucbHandlerIndex = 0;
  }
  ucbHandler[ucbHandlerIndex].UserCallBack =
      (void (*)(ErrCode::ErrCodeType errCode, UserData userData))callback;
  ucbHandler[ucbHandlerIndex].userData = userData;
  return &(ucbHandler[ucbHandlerIndex]);
}

void CameraModule::setExposureModeAsync(
    ExposureMode mode,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  ExposureModeReq req;
  req.funcParam.funcIndex = FUNCTION_SET_EXPOSURE_MODE;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.exposureModeMode = mode;
  req.reserve = 0;
  setInterfaceAsync<ExposureModeReq>(req, commonAckDecoder, UserCallBack,
                                     userData);
}

ErrCode::ErrCodeType CameraModule::setExposureModeSync(ExposureMode mode,
                                                       int timeout) {
  ExposureModeReq req;
  req.funcParam.funcIndex = FUNCTION_SET_EXPOSURE_MODE;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.exposureModeMode = mode;
  req.reserve = 0;
  return setInterfaceSync<ExposureModeReq>(req, timeout);
}

void CameraModule::getExposureModeAsync(
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, ExposureMode mode,
                         UserData userData),
    UserData userData) {
  FuncParam req;
  req.funcIndex = FUNCTION_GET_EXPOSURE_MODE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  getInterfaceAsync<ExposureMode>(req, getExposureModeDecoder, UserCallBack,
                                  userData);
}

ErrCode::ErrCodeType CameraModule::getExposureModeSync(ExposureMode& mode,
                                                       int timeout) {
  FuncParam req;
  ExposureModeAck ack;
  req.funcIndex = FUNCTION_GET_EXPOSURE_MODE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  ErrCode::ErrCodeType ret =
      getInterfaceSync<ExposureModeAck>(req, ack, timeout);
  if (ret == ErrCode::SysCommonErr::Success)
    mode = (ExposureMode)ack.exposureMode;
  return ret;
}

void CameraModule::setISOAsync(
    ISO iso,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  ISOParamReq req;
  req.funcParam.funcIndex = FUNCTION_SET_ISO_PARAMETER;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.iso = iso;
  setInterfaceAsync<ISOParamReq>(req, commonAckDecoder, UserCallBack, userData);
}

ErrCode::ErrCodeType CameraModule::setISOSync(ISO iso, int timeout) {
  ISOParamReq req;
  req.funcParam.funcIndex = FUNCTION_SET_ISO_PARAMETER;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.iso = iso;
  return setInterfaceSync<ISOParamReq>(req, timeout);
}

void CameraModule::getISOAsync(void (*UserCallBack)(ErrCode::ErrCodeType,
                                                    ISO iso, UserData userData),
                               UserData userData) {
  FuncParam req;
  req.funcIndex = FUNCTION_GET_ISO_PARAMETER;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  getInterfaceAsync<ISO>(req, getISODecoder, UserCallBack, userData);
}

ErrCode::ErrCodeType CameraModule::getISOSync(ISO& iso, int timeout) {
  FuncParam req;
  ISOParamAck ack;
  req.funcIndex = FUNCTION_GET_ISO_PARAMETER;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  ErrCode::ErrCodeType ret = getInterfaceSync<ISOParamAck>(req, ack, timeout);
  if (ret == ErrCode::SysCommonErr::Success) iso = (ISO)ack.iso;
  return ret;
}

void CameraModule::startShootPhotoAsync(
    ShootPhotoMode mode,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  ShootPhotoReq req;
  req.funcParam.funcIndex = FUNCTION_SIMPLE_SHOT;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.takePhotoType = mode;
  actionInterfaceAsync<ShootPhotoReq>(req, commonAckDecoder, UserCallBack,
                                      userData, 2000, 1);
}

ErrCode::ErrCodeType CameraModule::startShootPhotoSync(ShootPhotoMode mode,
                                                       int timeout) {
  ShootPhotoReq req;
  req.funcParam.funcIndex = FUNCTION_SIMPLE_SHOT;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.takePhotoType = mode;
  return actionInterfaceSync<ShootPhotoReq>(req, timeout);
}

void CameraModule::stopShootPhotoAsync(
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  ShootPhotoReq req;
  req.funcParam.funcIndex = FUNCTION_SIMPLE_SHOT;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.takePhotoType = DJI_CAMERA_TAKE_PHOTO_TYPE_STOP;
  actionInterfaceAsync<ShootPhotoReq>(req, commonAckDecoder, UserCallBack,
                                      userData);
}

ErrCode::ErrCodeType CameraModule::stopShootPhotoSync(int timeout) {
  ShootPhotoReq req;
  req.funcParam.funcIndex = FUNCTION_SIMPLE_SHOT;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.takePhotoType = DJI_CAMERA_TAKE_PHOTO_TYPE_STOP;
  return actionInterfaceSync<ShootPhotoReq>(req, timeout);
}

void CameraModule::startRecordVideoAsync(
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  RecordVideoReq req;
  req.funcParam.funcIndex = FUNCTION_RECORD_VIDEO;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.recording_control = DJI_CAMERA_RECORDING_CONTROL_BEGIN;
  req.recording_type = DJI_CAMERA_RECORDING_TYPE_COMMON;
  actionInterfaceAsync<RecordVideoReq>(req, commonAckDecoder, UserCallBack,
                                       userData);
}

ErrCode::ErrCodeType CameraModule::startRecordVideoSync(int timeout) {
  RecordVideoReq req;
  req.funcParam.funcIndex = FUNCTION_RECORD_VIDEO;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.recording_control = DJI_CAMERA_RECORDING_CONTROL_BEGIN;
  req.recording_type = DJI_CAMERA_RECORDING_TYPE_COMMON;
  return actionInterfaceSync<RecordVideoReq>(req, timeout);
}

void CameraModule::stopRecordVideoAsync(
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  RecordVideoReq req;
  req.funcParam.funcIndex = FUNCTION_RECORD_VIDEO;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.recording_control = DJI_CAMERA_RECORDING_CONTROL_STOP;
  req.recording_type = DJI_CAMERA_RECORDING_TYPE_COMMON;
  actionInterfaceAsync<RecordVideoReq>(req, commonAckDecoder, UserCallBack,
                                       userData);
}

ErrCode::ErrCodeType CameraModule::stopRecordVideoSync(int timeout) {
  RecordVideoReq req;
  req.funcParam.funcIndex = FUNCTION_RECORD_VIDEO;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.recording_control = DJI_CAMERA_RECORDING_CONTROL_STOP;
  req.recording_type = DJI_CAMERA_RECORDING_TYPE_COMMON;
  return actionInterfaceSync<RecordVideoReq>(req, timeout);
}

void CameraModule::setModeAsync(
    WorkMode mode,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  WorkModeReq req;
  req.funcParam.funcIndex = FUNCTION_SET_WORKING_MODE;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.workingMode = mode;
  setInterfaceAsync<WorkModeReq>(req, commonAckDecoder, UserCallBack, userData,
                                 2000, 1);
}

ErrCode::ErrCodeType CameraModule::setModeSync(WorkMode mode, int timeout) {
  WorkModeReq req;
  req.funcParam.funcIndex = FUNCTION_SET_WORKING_MODE;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.workingMode = mode;
  return setInterfaceSync<WorkModeReq>(req, timeout);
}

void CameraModule::getModeAsync(
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, WorkMode workingMode,
                         UserData userData),
    UserData userData) {
  FuncParam req;
  req.funcIndex = FUNCTION_GET_WORKING_MODE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  getInterfaceAsync<WorkMode>(req, getModeDecoder, UserCallBack, userData);
}

ErrCode::ErrCodeType CameraModule::getModeSync(WorkMode& workingMode,
                                               int timeout) {
  FuncParam req;
  WorkModeAck ack;
  req.funcIndex = FUNCTION_GET_WORKING_MODE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  ErrCode::ErrCodeType ret = getInterfaceSync<WorkModeAck>(req, ack, timeout);
  if (ret == ErrCode::SysCommonErr::Success)
    workingMode = (WorkMode)ack.workingMode;
  return ret;
}

void CameraModule::setFocusModeAsync(
    FocusMode mode,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  FocusModeReq req;
  req.funcParam.funcIndex = FUNCTION_SET_FOCUS_MODE;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.focusMode = mode;
  setInterfaceAsync<FocusModeReq>(req, commonAckDecoder, UserCallBack,
                                  userData);
}

ErrCode::ErrCodeType CameraModule::setFocusModeSync(FocusMode mode,
                                                    int timeout) {
  FocusModeReq req;
  req.funcParam.funcIndex = FUNCTION_SET_FOCUS_MODE;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.focusMode = mode;
  return setInterfaceSync<FocusModeReq>(req, timeout);
}

void CameraModule::getFocusModeAsync(
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, FocusMode focusMode,
                         UserData userData),
    UserData userData) {
  FuncParam req;
  req.funcIndex = FUNCTION_GET_FOCUS_MODE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  getInterfaceAsync<FocusMode>(req, getFocusModeDecoder, UserCallBack,
                               userData);
}

ErrCode::ErrCodeType CameraModule::getFocusModeSync(FocusMode& focusMode,
                                                    int timeout) {
  FuncParam req;
  FocusModeAck ack;
  req.funcIndex = FUNCTION_GET_FOCUS_MODE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  ErrCode::ErrCodeType ret = getInterfaceSync<FocusModeAck>(req, ack, timeout);
  if (ret == ErrCode::SysCommonErr::Success)
    focusMode = (FocusMode)ack.focusMode;
  return ret;
}

void CameraModule::setFocusTargetAsync(
    TapFocusPosData tapFocusPos,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  TapFocusPosReq req;
  req.funcParam.funcIndex = FUNCTION_SET_FOCUS_PARAMETER;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.p = tapFocusPos;
  setInterfaceAsync<TapFocusPosReq>(req, commonAckDecoder, UserCallBack,
                                    userData);
}

ErrCode::ErrCodeType CameraModule::setFocusTargetSync(
    TapFocusPosData tapFocusPos, int timeout) {
  TapFocusPosReq req;
  req.funcParam.funcIndex = FUNCTION_SET_FOCUS_PARAMETER;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.p = tapFocusPos;
  return setInterfaceSync<TapFocusPosReq>(req, timeout);
}

void CameraModule::tapZoomAtTargetAsync(
    TapZoomPosData tapZoomPos,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  TapZoomPosReq req;
  req.funcParam.funcIndex = FUNCTION_POINT_ZOOM_CTRL;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.p = tapZoomPos;
  setInterfaceAsync<TapZoomPosReq>(req, commonAckDecoder, UserCallBack,
                                   userData);
}

ErrCode::ErrCodeType CameraModule::tapZoomAtTargetSync(
    TapZoomPosData tapZoomPos, int timeout) {
  TapZoomPosReq req;
  req.funcParam.funcIndex = FUNCTION_POINT_ZOOM_CTRL;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.p = tapZoomPos;
  return setInterfaceSync<TapZoomPosReq>(req, timeout);
}

void CameraModule::getFocusTargetAsync(
    void (*UserCallBack)(ErrCode::ErrCodeType, TapFocusPosData tapFocusPos,
                         UserData userData),
    UserData userData) {
  FuncParam req;
  req.funcIndex = FUNCTION_GET_FOCUS_PARAMETER;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  getInterfaceAsync<TapFocusPosData>(req, getFocusTargetDecoder, UserCallBack,
                                     userData);
}

ErrCode::ErrCodeType CameraModule::getFocusTargetSync(
    TapFocusPosData& tapFocusPos, int timeout) {
  FuncParam req;
  TapFocusPosAck ack;
  req.funcIndex = FUNCTION_GET_FOCUS_PARAMETER;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  ErrCode::ErrCodeType ret =
      getInterfaceSync<TapFocusPosAck>(req, ack, timeout);
  if (ret == ErrCode::SysCommonErr::Success) tapFocusPos = ack.p;
  return ret;
}

void CameraModule::setApertureAsync(
    Aperture size,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  ApertureReq req;
  req.funcParam.funcIndex = FUNCTION_SET_APERTURE_SIZE;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.size = size;
  setInterfaceAsync<ApertureReq>(req, commonAckDecoder, UserCallBack, userData);
}

ErrCode::ErrCodeType CameraModule::setApertureSync(Aperture size, int timeout) {
  ApertureReq req;
  req.funcParam.funcIndex = FUNCTION_SET_APERTURE_SIZE;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.size = size;
  return setInterfaceSync<ApertureReq>(req, timeout);
}

void CameraModule::getApertureAsync(void (*UserCallBack)(ErrCode::ErrCodeType,
                                                         Aperture size,
                                                         UserData userData),
                                    UserData userData) {
  FuncParam req;
  req.funcIndex = FUNCTION_GET_APERTURE_SIZE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  getInterfaceAsync<Aperture>(req, getApertureDecoder, UserCallBack, userData);
}

ErrCode::ErrCodeType CameraModule::getApertureSync(Aperture& size,
                                                   int timeout) {
  FuncParam req;
  ApertureAck ack;
  req.funcIndex = FUNCTION_GET_APERTURE_SIZE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  ErrCode::ErrCodeType ret = getInterfaceSync<ApertureAck>(req, ack, timeout);
  if (ret == ErrCode::SysCommonErr::Success) size = (Aperture)ack.size;
  return ret;
}

void CameraModule::setShutterSpeedAsync(
    ShutterSpeed shutterSpeed,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  ShutterReq req;
  req.funcParam.funcIndex = FunctionID::FUNCTION_SET_SHUTTER_SPEED;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.shutter_mode = SHUTTER_MANUAL_MODE;
  req.shutterSpeed =
      ShutterSpeedEnumToShutterSpeedType((ShutterSpeed)shutterSpeed);
  setInterfaceAsync<ShutterReq>(req, commonAckDecoder, UserCallBack, userData);
}

void CameraModule::getShutterSpeedAsync(
    void (*UserCallBack)(ErrCode::ErrCodeType retCode,
                         ShutterSpeed shutterSpeed, UserData userData),
    UserData userData) {
  FuncParam req;
  req.funcIndex = FUNCTION_GET_SHUTTER_SPEED;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  getInterfaceAsync<ShutterSpeed>(req, getShutterSpeedDecoder, UserCallBack,
                                  userData);
}

ErrCode::ErrCodeType CameraModule::setShutterSpeedSync(
    ShutterSpeed shutterSpeed, int timeout) {
  ShutterReq req;
  req.funcParam.funcIndex = FUNCTION_SET_SHUTTER_SPEED;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.shutter_mode = SHUTTER_MANUAL_MODE;
  req.shutterSpeed =
      ShutterSpeedEnumToShutterSpeedType((ShutterSpeed)shutterSpeed);
  return setInterfaceSync<ShutterReq>(req, timeout);
}

ErrCode::ErrCodeType CameraModule::getShutterSpeedSync(
    ShutterSpeed& shutterSpeed, int timeout) {
  FuncParam req;
  ShutterAck ack;
  req.funcIndex = FUNCTION_GET_SHUTTER_SPEED;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  ErrCode::ErrCodeType ret = getInterfaceSync<ShutterAck>(req, ack, timeout);
  if (ret == ErrCode::SysCommonErr::Success)
    shutterSpeed = ShutterSpeedTypeToShutterSpeedEnum(ack.shutter.reciprocal,
                                                      ack.shutter.integer_part,
                                                      ack.shutter.decimal_part);
  return ret;
}

void CameraModule::setExposureCompensationAsync(
    ExposureCompensation ev,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  ExposureCompensationReq req;
  req.funcParam.funcIndex = FUNCTION_SET_EV_PARAMETER;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.ev = ev;
  setInterfaceAsync<ExposureCompensationReq>(req, commonAckDecoder,
                                             UserCallBack, userData);
}

ErrCode::ErrCodeType CameraModule::setExposureCompensationSync(
    ExposureCompensation ev, int timeout) {
  ExposureCompensationReq req;
  req.funcParam.funcIndex = FUNCTION_SET_EV_PARAMETER;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.ev = ev;
  return setInterfaceSync<ExposureCompensationReq>(req, timeout);
}

void CameraModule::getExposureCompensationAsync(
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, ExposureCompensation ev,
                         UserData userData),
    UserData userData) {
  FuncParam req;
  req.funcIndex = FUNCTION_GET_EV_PARAMETER;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  getInterfaceAsync<ExposureCompensation>(req, getEVDecoder, UserCallBack,
                                          userData);
}

ErrCode::ErrCodeType CameraModule::getExposureCompensationSync(
    ExposureCompensation& ev, int timeout) {
  FuncParam req;
  ExposureCompensationAck ack;
  req.funcIndex = FUNCTION_GET_EV_PARAMETER;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  ErrCode::ErrCodeType ret =
      getInterfaceSync<ExposureCompensationAck>(req, ack, timeout);
  if (ret == ErrCode::SysCommonErr::Success)
    ev = (ExposureCompensation)ack.ev_param;
  return ret;
}

void CameraModule::startContinuousOpticalZoomAsync(
    zoomDirectionData zoomDirection, zoomSpeedData zoomSpeed,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  zoomOptiParamReq req = {0};
  req.funcParam.funcIndex = FUNCTION_CONTROL_OPTIZOOM;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.zoomOptiParam.zoomType = 1;
  req.zoomOptiParam.zoomSpeed = zoomSpeed;
  req.zoomOptiParam.zoomParam.zoomContiParam.zoomDirection = zoomDirection;
  req.zoomOptiParam.zoomParam.zoomContiParam.padding = 0;
  setInterfaceAsync<zoomOptiParamReq>(req, commonAckDecoder, UserCallBack,
                                      userData);
}

ErrCode::ErrCodeType CameraModule::startContinuousOpticalZoomSync(
    zoomDirectionData zoomDirection, zoomSpeedData zoomSpeed, int timeout) {
  zoomOptiParamReq req = {0};
  req.funcParam.funcIndex = FUNCTION_CONTROL_OPTIZOOM;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.zoomOptiParam.zoomType = 1;
  req.zoomOptiParam.zoomSpeed = zoomSpeed;
  req.zoomOptiParam.zoomParam.zoomContiParam.zoomDirection = zoomDirection;
  req.zoomOptiParam.zoomParam.zoomContiParam.padding = 0;
  return setInterfaceSync<zoomOptiParamReq>(req, timeout);
}

void CameraModule::stopContinuousOpticalZoomAsync(
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  zoomOptiParamReq req = {0};
  req.funcParam.funcIndex = FUNCTION_CONTROL_OPTIZOOM;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.zoomOptiParam.zoomType = 255;
  setInterfaceAsync<zoomOptiParamReq>(req, commonAckDecoder, UserCallBack,
                                      userData);
}

ErrCode::ErrCodeType CameraModule::stopContinuousOpticalZoomSync(int timeout) {
  zoomOptiParamReq req = {0};
  req.funcParam.funcIndex = FUNCTION_CONTROL_OPTIZOOM;
  req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.zoomOptiParam.zoomType = 255;
  return setInterfaceSync<zoomOptiParamReq>(req, timeout);
}

typedef struct TapZoomEnabledHandler {
  CameraModule* cameraModule;
  CameraModule::TapZoomEnableData enable;
  CameraModule::TapZoomMultiplierData multiplier;
  void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData);
  UserData userData;
} TapZoomEnabledHandler;

/*! @TODO Here is only the temporary way to alloc memory for the asynchronous
 * interface to stash the user data. This method will be optimized in the
 * future.
 * @Note There are 32 memory units for this method. So if the API calling
 * this method too fast, the memory units of this API may overflow. So the
 * calling frequency of releated APIs should less than 10Hz */
TapZoomEnabledHandler* allocTapZoomEnabledHandlerMemory() {
  const uint8_t maxNum = 32;
  static TapZoomEnabledHandler handler[maxNum] = {0};
  static uint8_t index = 0;

  index++;
  if (index >= maxNum) {
    index = 0;
  }
  return &(handler[index]);
}

void CameraModule::setTapZoomEnabledAsync(
    bool param,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  TapZoomEnabledHandler* handler = allocTapZoomEnabledHandlerMemory();
  handler->cameraModule = this;
  handler->enable = param;
  handler->UserCallBack = UserCallBack;
  handler->userData = userData;
  getTapZoomMultiplierAsync(callbackToSetTapZoomEnabled, handler);
}

ErrCode::ErrCodeType CameraModule::setTapZoomEnabledSync(bool param,
                                                         int timeout) {
  TapZoomMultiplierData multiplier;
  ErrCode::ErrCodeType errCode = getTapZoomMultiplierSync(multiplier, 1);
  if (errCode != ErrCode::SysCommonErr::Success) {
    return errCode;
  } else {
    TapZoomEnableReq req;
    req.funcParam.funcIndex = FUNCTION_SET_POINT_ZOOM_MODE;
    req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
    req.tapZoomEnable = param;
    req.multiplier = multiplier;
    return setInterfaceSync<TapZoomEnableReq>(req, timeout);
  }
}

void CameraModule::callbackToSetTapZoomEnabled(ErrCode::ErrCodeType retCode,
                                               TapZoomMultiplierData multiplier,
                                               UserData userData) {
  if (!userData) return;
  TapZoomEnabledHandler handler = *(TapZoomEnabledHandler*)userData;
  if (retCode != ErrCode::SysCommonErr::Success) {
    if (handler.UserCallBack) handler.UserCallBack(retCode, userData);
  } else {
    TapZoomEnableReq req;
    req.funcParam.funcIndex = FUNCTION_SET_POINT_ZOOM_MODE;
    req.funcParam.payloadNodeIndex =
        (uint8_t)(handler.cameraModule->getIndex() + 1);
    req.tapZoomEnable = handler.enable;
    req.multiplier = multiplier;
    handler.cameraModule->setInterfaceAsync<TapZoomEnableReq>(
        req, commonAckDecoder, handler.UserCallBack, handler.userData);
  }
}

void CameraModule::getTapZoomEnabledAsync(
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, bool param,
                         UserData userData),
    UserData userData) {
  FuncParam req;
  req.funcIndex = FUNCTION_GET_POINT_ZOOM_MODE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  getInterfaceAsync<bool>(req, getTapZoomEnabledDecoder, UserCallBack,
                          userData);
}

ErrCode::ErrCodeType CameraModule::getTapZoomEnabledSync(bool& param,
                                                         int timeout) {
  FuncParam req;
  TapZoomEnableAck ack;
  req.funcIndex = FUNCTION_GET_POINT_ZOOM_MODE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  ErrCode::ErrCodeType ret =
      getInterfaceSync<TapZoomEnableAck>(req, ack, timeout);
  if (ret == ErrCode::SysCommonErr::Success) param = (bool)ack.tapZoomEnable;
  return ret;
}

void CameraModule::setTapZoomMultiplierAsync(
    TapZoomMultiplierData param,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  TapZoomEnabledHandler* handler = allocTapZoomEnabledHandlerMemory();
  handler->cameraModule = this;
  handler->enable = false;
  handler->multiplier = param;
  handler->UserCallBack = UserCallBack;
  handler->userData = userData;
  getTapZoomEnabledAsync(callbackToSetTapZoomMultiplier, handler);
}

ErrCode::ErrCodeType CameraModule::setTapZoomMultiplierSync(
    TapZoomMultiplierData param, int timeout) {
  bool enableData;
  ErrCode::ErrCodeType errCode = getTapZoomEnabledSync(enableData, 1);
  if (errCode != ErrCode::SysCommonErr::Success) {
    return errCode;
  } else {
    TapZoomEnableReq req;
    req.funcParam.funcIndex = FUNCTION_SET_POINT_ZOOM_MODE;
    req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
    req.tapZoomEnable = enableData;
    req.multiplier = param;
    return setInterfaceSync<TapZoomEnableReq>(req, timeout);
  }
}

void CameraModule::callbackToSetTapZoomMultiplier(ErrCode::ErrCodeType retCode,
                                                  bool enable,
                                                  UserData userData) {
  if (!userData) return;
  TapZoomEnabledHandler handler = *(TapZoomEnabledHandler*)userData;
  if (retCode != ErrCode::SysCommonErr::Success) {
    if (handler.UserCallBack) handler.UserCallBack(retCode, userData);
  } else {
    TapZoomEnableReq req;
    req.funcParam.funcIndex = FUNCTION_SET_POINT_ZOOM_MODE;
    req.funcParam.payloadNodeIndex =
        (uint8_t)(handler.cameraModule->getIndex() + 1);
    req.tapZoomEnable = enable;
    req.multiplier = handler.multiplier;
    handler.cameraModule->setInterfaceAsync<TapZoomEnableReq>(
        req, commonAckDecoder, handler.UserCallBack, handler.userData);
  }
}

void CameraModule::getTapZoomMultiplierAsync(
    void (*UserCallBack)(ErrCode::ErrCodeType retCode,
                         TapZoomMultiplierData param, UserData userData),
    UserData userData) {
  FuncParam req;
  req.funcIndex = FUNCTION_GET_POINT_ZOOM_MODE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  getInterfaceAsync<TapZoomMultiplierData>(req, getTapZoomMultiplierDecoder,
                                           UserCallBack, userData);
}

ErrCode::ErrCodeType CameraModule::getTapZoomMultiplierSync(
    TapZoomMultiplierData& param, int timeout) {
  FuncParam req;
  TapZoomEnableAck ack;
  req.funcIndex = FUNCTION_GET_POINT_ZOOM_MODE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  ErrCode::ErrCodeType ret =
      getInterfaceSync<TapZoomEnableAck>(req, ack, timeout);
  if (ret == ErrCode::SysCommonErr::Success) param = ack.multiplier;
  return ret;
}

void CameraModule::commonAckDecoder(Vehicle* vehicle, RecvContainer recvFrame,
                                    UCBRetCodeHandler* ucb) {
  if (ucb && ucb->UserCallBack) {
    CommonAck ack = {0};
    ErrCode::ErrCodeType ret = 0;
    if (recvFrame.recvInfo.len - OpenProtocol::PackageMin >=
        sizeof(CommonAck)) {
      ack = *(CommonAck*)(recvFrame.recvData.raw_ack_array);
      ret = ErrCode::errorCode(ErrCode::CameraModule, ErrCode::CameraCommon,
                               ack.ret_code);
    } else {
      DERROR("ACK is exception, data len %d (expect >= %d)\n",
             recvFrame.recvInfo.len - OpenProtocol::PackageMin,
             sizeof(CommonAck));
      ret = ErrCode::SysCommonErr::InvalidRespond;
    }
    ucb->UserCallBack(ret, ucb->userData);
  }
}

void CameraModule::getISODecoder(Vehicle* vehicle, RecvContainer recvFrame,
                                 UCBRetParamHandler<ISO>* ucb) {
  if (ucb && ucb->UserCallBack) {
    ISOParamAck ack = {0};
    ErrCode::ErrCodeType retCode =
        commonDataUnpacker<ISOParamAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, (ISO)ack.iso, ucb->userData);
  }
}

void CameraModule::getExposureModeDecoder(
    Vehicle* vehicle, RecvContainer recvFrame,
    UCBRetParamHandler<ExposureMode>* ucb) {
  if (ucb && ucb->UserCallBack) {
    ExposureModeAck ack = {0};
    ErrCode::ErrCodeType retCode =
        commonDataUnpacker<ExposureModeAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, (ExposureMode)ack.exposureMode, ucb->userData);
  }
}

void CameraModule::getModeDecoder(Vehicle* vehicle, RecvContainer recvFrame,
                                  UCBRetParamHandler<WorkMode>* ucb) {
  if (ucb && ucb->UserCallBack) {
    WorkModeAck ack = {0};
    ErrCode::ErrCodeType retCode =
        commonDataUnpacker<WorkModeAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, (WorkMode)ack.workingMode, ucb->userData);
  }
}

void CameraModule::getFocusModeDecoder(Vehicle* vehicle,
                                       RecvContainer recvFrame,
                                       UCBRetParamHandler<FocusMode>* ucb) {
  if (ucb && ucb->UserCallBack) {
    FocusModeAck ack = {0};
    ErrCode::ErrCodeType retCode =
        commonDataUnpacker<FocusModeAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, (FocusMode)ack.focusMode, ucb->userData);
  }
}

void CameraModule::getFocusTargetDecoder(
    Vehicle* vehicle, RecvContainer recvFrame,
    UCBRetParamHandler<TapFocusPosData>* ucb) {
  if (ucb && ucb->UserCallBack) {
    TapFocusPosAck ack = {0};
    ErrCode::ErrCodeType retCode =
        commonDataUnpacker<TapFocusPosAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, ack.p, ucb->userData);
  }
}

void CameraModule::getApertureDecoder(Vehicle* vehicle, RecvContainer recvFrame,
                                      UCBRetParamHandler<Aperture>* ucb) {
  if (ucb && ucb->UserCallBack) {
    ApertureAck ack = {0};
    ErrCode::ErrCodeType retCode =
        commonDataUnpacker<ApertureAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, (Aperture)ack.size, ucb->userData);
  }
}

void CameraModule::getCaptureParamDataDecoder(
    Vehicle* vehicle, RecvContainer recvFrame,
    UCBRetParamHandler<CaptureParamData>* ucb) {
  if (ucb && ucb->UserCallBack) {
    CaptureParamAck ack = {0};
    ErrCode::ErrCodeType retCode =
        commonDataUnpacker<CaptureParamAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, ack.captureParam, ucb->userData);
  }
}

void CameraModule::getShootPhotoModeDataDecoder(
    Vehicle* vehicle, RecvContainer recvFrame,
    UCBRetParamHandler<ShootPhotoMode>* ucb) {
  if (ucb && ucb->UserCallBack) {
    CaptureParamAck ack = {0};
    ErrCode::ErrCodeType retCode =
        commonDataUnpacker<CaptureParamAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, (ShootPhotoMode)ack.captureParam.captureMode,
                      ucb->userData);
  }
}

void CameraModule::getPhotoAEBCountDecoder(
    Vehicle* vehicle, RecvContainer recvFrame,
    UCBRetParamHandler<PhotoAEBCount>* ucb) {
  if (ucb && ucb->UserCallBack) {
    CaptureParamAck ack = {0};
    ErrCode::ErrCodeType retCode =
        commonDataUnpacker<CaptureParamAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, (PhotoAEBCount)ack.captureParam.photoNumBurst,
                      ucb->userData);
  }
}

void CameraModule::getPhotoBurstCountDecoder(
    Vehicle* vehicle, RecvContainer recvFrame,
    UCBRetParamHandler<PhotoBurstCount>* ucb) {
  if (ucb && ucb->UserCallBack) {
    CaptureParamAck ack = {0};
    ErrCode::ErrCodeType retCode =
        commonDataUnpacker<CaptureParamAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, (PhotoBurstCount)ack.captureParam.photoNumBurst,
                      ucb->userData);
  }
}

void CameraModule::getPhotoIntervalDatasDecoder(
    Vehicle* vehicle, RecvContainer recvFrame,
    UCBRetParamHandler<PhotoIntervalData>* ucb) {
  if (ucb && ucb->UserCallBack) {
    CaptureParamAck ack = {0};
    ErrCode::ErrCodeType retCode =
        commonDataUnpacker<CaptureParamAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, ack.captureParam.intervalSetting, ucb->userData);
  }
}

void CameraModule::getShutterSpeedDecoder(
    Vehicle* vehicle, RecvContainer recvFrame,
    UCBRetParamHandler<ShutterSpeed>* ucb) {
  ShutterAck ack = {0};
  if (ucb && ucb->UserCallBack) {
    ErrCode::ErrCodeType retCode =
        commonDataUnpacker<ShutterAck>(recvFrame, ack);
    ShutterSpeed shutterSpeed = ShutterSpeedTypeToShutterSpeedEnum(
        ack.shutter.reciprocal, ack.shutter.integer_part,
        ack.shutter.decimal_part);
    ucb->UserCallBack(retCode, shutterSpeed, ucb->userData);
  }
}

void CameraModule::getEVDecoder(Vehicle* vehicle, RecvContainer recvFrame,
                                UCBRetParamHandler<ExposureCompensation>* ucb) {
  if (ucb && ucb->UserCallBack) {
    ExposureCompensationAck ack = {0};
    ErrCode::ErrCodeType retCode =
        commonDataUnpacker<ExposureCompensationAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, (ExposureCompensation)ack.ev_param,
                      ucb->userData);
  }
}

void CameraModule::getTapZoomEnabledDecoder(Vehicle* vehicle,
                                            RecvContainer recvFrame,
                                            UCBRetParamHandler<bool>* ucb) {
  if (ucb && ucb->UserCallBack) {
    TapZoomEnableAck ack = {0};
    ErrCode::ErrCodeType retCode =
        commonDataUnpacker<TapZoomEnableAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, (bool)ack.tapZoomEnable, ucb->userData);
  }
}

void CameraModule::getTapZoomMultiplierDecoder(
    Vehicle* vehicle, RecvContainer recvFrame,
    UCBRetParamHandler<TapZoomMultiplierData>* ucb) {
  if (ucb && ucb->UserCallBack) {
    TapZoomEnableAck ack = {0};
    ErrCode::ErrCodeType retCode =
        commonDataUnpacker<TapZoomEnableAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, ack.multiplier, ucb->userData);
  }
}

CameraModule::ShutterSpeedType CameraModule::createShutterSpeedStruct(
    int reciprocal, int integer_part, int decimal_part) {
  ShutterSpeedType speed;
  speed.reciprocal = reciprocal;
  speed.integer_part = integer_part;
  speed.decimal_part = decimal_part;
  return speed;
}

CameraModule::ShutterSpeedType CameraModule::ShutterSpeedEnumToShutterSpeedType(
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

CameraModule::ShutterSpeed CameraModule::ShutterSpeedTypeToShutterSpeedEnum(
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
    void (*UserCallBack)(ErrCode::ErrCodeType retCode,
                         CaptureParamData captureParam, UserData userData),
    UserData userData) {
  FuncParam req;
  req.funcIndex = FUNCTION_GET_SHOT_MODE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  getInterfaceAsync<CaptureParamData>(req, getCaptureParamDataDecoder,
                                      UserCallBack, userData);
}

ErrCode::ErrCodeType CameraModule::getCaptureParamDataSync(
    CaptureParamData& captureParam, int timeout) {
  FuncParam req;
  CaptureParamAck ack;
  req.funcIndex = FUNCTION_GET_SHOT_MODE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  ErrCode::ErrCodeType ret =
      getInterfaceSync<CaptureParamAck>(req, ack, timeout);
  if (ret == ErrCode::SysCommonErr::Success) captureParam = ack.captureParam;
  return ret;
}

typedef struct shootPhotoParamHandler {
  CameraModule* cameraModule;
  CameraModule::CaptureParamData paramData;
  void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData);
  UserData userData;
} shootPhotoParamHandler;

/*! @TODO Here is only the temporary way to alloc memory for the asynchronous
 * interface to stash the user data. This method will be optimized in the
 * future.
 * @Note There are 32 memory units for this method. So if the API calling
 * this method too fast, the memory units of this API may overflow. So the
 * calling frequency of releated APIs should less than 10Hz */
shootPhotoParamHandler* allocShootPhotoParamHandlerMemory() {
  const uint8_t maxNum = 32;
  static shootPhotoParamHandler handler[maxNum] = {0};
  static uint8_t index = 0;

  index++;
  if (index >= maxNum) {
    index = 0;
  }
  return &(handler[index]);
}

void CameraModule::callbackToSetShootPhotoMode(ErrCode::ErrCodeType retCode,
                                               CaptureParamData captureParam,
                                               UserData userData) {
  if (!userData) return;
  shootPhotoParamHandler handler = *(shootPhotoParamHandler*)userData;
  if (retCode != ErrCode::SysCommonErr::Success) {
    if (handler.UserCallBack) handler.UserCallBack(retCode, userData);
  } else {
    CaptureParamReq req;
    req.funcParam.funcIndex = FUNCTION_SET_SHOT_MODE;
    req.funcParam.payloadNodeIndex =
        (uint8_t)(handler.cameraModule->getIndex() + 1);
    req.captureParam = captureParam;
    req.captureParam.captureMode = handler.paramData.captureMode;

    handler.cameraModule->setInterfaceAsync<CaptureParamReq>(
        req, commonAckDecoder, handler.UserCallBack, handler.userData);
  }
}

void CameraModule::setShootPhotoModeAsync(
    ShootPhotoMode takePhotoMode,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  shootPhotoParamHandler* handler = allocShootPhotoParamHandlerMemory();
  handler->cameraModule = this;
  handler->paramData.captureMode = takePhotoMode;
  handler->UserCallBack = UserCallBack;
  handler->userData = userData;
  getCaptureParamDataAsync(callbackToSetShootPhotoMode, handler);
}

ErrCode::ErrCodeType CameraModule::setShootPhotoModeSync(
    ShootPhotoMode takePhotoMode, int timeout) {
  CaptureParamData captureParamData;
  ErrCode::ErrCodeType errCode = getCaptureParamDataSync(captureParamData, 1);
  if (errCode != ErrCode::SysCommonErr::Success) {
    return errCode;
  } else {
    CaptureParamReq req;
    req.funcParam.funcIndex = FUNCTION_SET_SHOT_MODE;
    req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
    req.captureParam = captureParamData;
    req.captureParam.captureMode = takePhotoMode;
    return setInterfaceSync<CaptureParamReq>(req, timeout);
  }
}

void CameraModule::getShootPhotoModeAsync(
    void (*UserCallBack)(ErrCode::ErrCodeType retCode,
                         ShootPhotoMode takePhotoMode, UserData userData),
    UserData userData) {
  FuncParam req;
  req.funcIndex = FUNCTION_GET_SHOT_MODE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  getInterfaceAsync<ShootPhotoMode>(req, getShootPhotoModeDataDecoder,
                                    UserCallBack, userData);
}

ErrCode::ErrCodeType CameraModule::getShootPhotoModeSync(
    ShootPhotoMode& takePhotoMode, int timeout) {
  CaptureParamData captureParam;
  ErrCode::ErrCodeType errCode = getCaptureParamDataSync(captureParam, timeout);
  if (errCode == ErrCode::SysCommonErr::Success)
    takePhotoMode = (ShootPhotoMode)captureParam.captureMode;
  return errCode;
}

void CameraModule::callbackToSetPhotoBurstCount(ErrCode::ErrCodeType retCode,
                                                CaptureParamData captureParam,
                                                UserData userData) {
  if (!userData) return;
  shootPhotoParamHandler handler = *(shootPhotoParamHandler*)userData;
  if (retCode != ErrCode::SysCommonErr::Success) {
    if (handler.UserCallBack) handler.UserCallBack(retCode, userData);
  } else {
    CaptureParamReq req;
    req.funcParam.funcIndex = FUNCTION_SET_SHOT_MODE;
    req.funcParam.payloadNodeIndex =
        (uint8_t)(handler.cameraModule->getIndex() + 1);
    req.captureParam = captureParam;
    req.captureParam.photoNumBurst = handler.paramData.photoNumBurst;

    handler.cameraModule->setInterfaceAsync<CaptureParamReq>(
        req, commonAckDecoder, handler.UserCallBack, handler.userData);
  }
}

void CameraModule::setPhotoBurstCountAsync(
    PhotoBurstCount count,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  shootPhotoParamHandler* handler = allocShootPhotoParamHandlerMemory();
  handler->cameraModule = this;
  handler->paramData.photoNumBurst = count;
  handler->UserCallBack = UserCallBack;
  handler->userData = userData;
  getCaptureParamDataAsync(callbackToSetPhotoBurstCount, handler);
}

ErrCode::ErrCodeType CameraModule::setPhotoBurstCountSync(PhotoBurstCount count,
                                                          int timeout) {
  CaptureParamData captureParamData;
  ErrCode::ErrCodeType errCode = getCaptureParamDataSync(captureParamData, 1);
  if (errCode != ErrCode::SysCommonErr::Success) {
    return errCode;
  } else {
    CaptureParamReq req;
    req.funcParam.funcIndex = FUNCTION_SET_SHOT_MODE;
    req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
    req.captureParam = captureParamData;
    req.captureParam.photoNumBurst = count;
    return setInterfaceSync<CaptureParamReq>(req, timeout);
  }
}

void CameraModule::getPhotoBurstCountAsync(
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, PhotoBurstCount count,
                         UserData userData),
    UserData userData) {
  FuncParam req;
  req.funcIndex = FUNCTION_GET_SHOT_MODE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  getInterfaceAsync<PhotoBurstCount>(req, getPhotoBurstCountDecoder,
                                     UserCallBack, userData);
}

ErrCode::ErrCodeType CameraModule::getPhotoBurstCountSync(
    PhotoBurstCount& count, int timeout) {
  CaptureParamData captureParam;
  ErrCode::ErrCodeType errCode = getCaptureParamDataSync(captureParam, timeout);
  if (errCode == ErrCode::SysCommonErr::Success)
    count = (PhotoBurstCount)captureParam.photoNumBurst;
  return errCode;
}

void CameraModule::setPhotoAEBCountAsync(
    PhotoAEBCount count,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  setPhotoBurstCountAsync((PhotoBurstCount)count, UserCallBack, userData);
}

void CameraModule::getPhotoAEBCountAsync(
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, PhotoAEBCount count,
                         UserData userData),
    UserData userData) {
  FuncParam req;
  req.funcIndex = FUNCTION_GET_SHOT_MODE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  getInterfaceAsync<PhotoAEBCount>(req, getPhotoAEBCountDecoder, UserCallBack,
                                   userData);
}

ErrCode::ErrCodeType CameraModule::setPhotoAEBCountSync(PhotoAEBCount count,
                                                        int timeout) {
  return setPhotoBurstCountSync((PhotoBurstCount)count, timeout);
}

ErrCode::ErrCodeType CameraModule::getPhotoAEBCountSync(PhotoAEBCount& count,
                                                        int timeout) {
  CaptureParamData captureParam;
  ErrCode::ErrCodeType errCode = getCaptureParamDataSync(captureParam, timeout);
  if (errCode == ErrCode::SysCommonErr::Success)
    count = (PhotoAEBCount)captureParam.photoNumBurst;
  return errCode;
}

void CameraModule::callbackToSetPhotoTimeIntervalSettings(
    ErrCode::ErrCodeType retCode, CaptureParamData captureParam,
    UserData userData) {
  if (!userData) return;
  shootPhotoParamHandler handler = *(shootPhotoParamHandler*)userData;
  if (retCode != ErrCode::SysCommonErr::Success) {
    if (handler.UserCallBack) handler.UserCallBack(retCode, userData);
  } else {
    CaptureParamReq req;
    req.funcParam.funcIndex = FUNCTION_SET_SHOT_MODE;
    req.funcParam.payloadNodeIndex =
        (uint8_t)(handler.cameraModule->getIndex() + 1);
    req.captureParam = captureParam;
    req.captureParam.intervalSetting = handler.paramData.intervalSetting;

    handler.cameraModule->setInterfaceAsync<CaptureParamReq>(
        req, commonAckDecoder, handler.UserCallBack, handler.userData);
  }
}

void CameraModule::setPhotoTimeIntervalSettingsAsync(
    PhotoIntervalData intervalSetting,
    void (*UserCallBack)(ErrCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  shootPhotoParamHandler* handler = allocShootPhotoParamHandlerMemory();
  handler->cameraModule = this;
  handler->paramData.intervalSetting = intervalSetting;
  handler->UserCallBack = UserCallBack;
  handler->userData = userData;
  getCaptureParamDataAsync(callbackToSetPhotoTimeIntervalSettings, handler);
}

ErrCode::ErrCodeType CameraModule::setPhotoTimeIntervalSettingsSync(
    PhotoIntervalData intervalSetting, int timeout) {
  CaptureParamData captureParamData;
  ErrCode::ErrCodeType errCode = getCaptureParamDataSync(captureParamData, 1);
  if (errCode != ErrCode::SysCommonErr::Success) {
    return errCode;
  } else {
    CaptureParamReq req;
    req.funcParam.funcIndex = FUNCTION_SET_SHOT_MODE;
    req.funcParam.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
    req.captureParam = captureParamData;
    req.captureParam.intervalSetting = intervalSetting;
    return setInterfaceSync<CaptureParamReq>(req, timeout);
  }
}

void CameraModule::getPhotoIntervalDatasAsync(
    void (*UserCallBack)(ErrCode::ErrCodeType retCode,
                         PhotoIntervalData intervalSetting, UserData userData),
    UserData userData) {
  FuncParam req;
  req.funcIndex = FUNCTION_GET_SHOT_MODE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  getInterfaceAsync<PhotoIntervalData>(req, getPhotoIntervalDatasDecoder,
                                       UserCallBack, userData);
}

ErrCode::ErrCodeType CameraModule::getPhotoIntervalDatasSync(
    PhotoIntervalData& intervalSetting, int timeout) {
  CaptureParamData captureParam;
  ErrCode::ErrCodeType errCode = getCaptureParamDataSync(captureParam, timeout);
  if (errCode == ErrCode::SysCommonErr::Success)
    intervalSetting = captureParam.intervalSetting;
  return errCode;
}