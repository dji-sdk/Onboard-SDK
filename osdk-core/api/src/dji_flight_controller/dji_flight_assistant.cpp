/** @file dji_flight_assistant.cpp
 *  @version 3.9
 *  @date April 2019SSS
 *
 *  @brief
 *  Flight Assistant API for DJI OSDK library
 *
 *  @Copyright (c) 2016-2019 DJI
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

#include "dji_flight_assistant.hpp"

using namespace DJI;
using namespace DJI::OSDK;

FlightAssistant::FlightAssistant(ControlLink* controlLink) {
  this->controlLink = controlLink;
}
FlightAssistant::~FlightAssistant() { delete this->controlLink; }

ErrorCode::ErrCodeType FlightAssistant::writeParameterByHashSync(
    uint32_t hashValue, void* data, uint8_t len, int timeout) {
  ParameterData param = {0};
  param.hashValue = hashValue;
  memcpy(param.paramValue, data, len);

  ACK::ParamAck rsp = *(ACK::ParamAck*)controlLink->sendSync(
      OpenProtocolCMD::CMDSet::Control::parameterWrite, &param,
      sizeof(param.hashValue) + len, timeout);
  if (rsp.updated && hashValue == rsp.data.hashValue &&
      (rsp.ack.info.len - OpenProtocol::PackageMin <=
       sizeof(ACK::ParamAckInternal))) {
    return rsp.data.retCode;
  } else {
    if (!rsp.updated)
      return ErrorCode::UnifiedErrCode::kErrorRequestTimeout;
    else if (hashValue != rsp.data.hashValue)
      return ErrorCode::UnifiedErrCode::kErrorInvalidParam;
    else
      return ErrorCode::UnifiedErrCode::kErrorInvalidRespond;
  }
}

ErrorCode::ErrCodeType FlightAssistant::readParameterByHashSync(
    ParamHashValue hashValue, void* param, int timeout) {
  ACK::ParamAck rsp = *(ACK::ParamAck*)controlLink->sendSync(
      OpenProtocolCMD::CMDSet::Control::parameterRead, &hashValue,
      sizeof(hashValue), timeout);
  if (rsp.updated && hashValue == rsp.data.hashValue &&
      (rsp.ack.info.len - OpenProtocol::PackageMin <=
       sizeof(ACK::ParamAckInternal))) {
    memcpy(param, rsp.data.paramValue, MAX_PARAMETER_VALUE_LENGTH);
    return rsp.data.retCode;
  } else {
    if (!rsp.updated)
      return ErrorCode::UnifiedErrCode::kErrorRequestTimeout;
    else if (hashValue != rsp.data.hashValue)
      return ErrorCode::UnifiedErrCode::kErrorInvalidParam;
    else
      return ErrorCode::UnifiedErrCode::kErrorInvalidRespond;
  }
}

void FlightAssistant::writeParameterByHashAsync(
    uint32_t hashValue, void* data, uint8_t len,
    void (*ackDecoderCB)(Vehicle* vehicle, RecvContainer recvFrame,
                         UCBRetCodeHandler* ucb),
    void (*userCB)(ErrorCode::ErrCodeType, UserData userData),
    UserData userData, int timeout, int retry_time) {
  if (controlLink) {
    ParameterData param = {0};
    param.hashValue = hashValue;
    memcpy(param.paramValue, data, len);
    controlLink->sendAsync(OpenProtocolCMD::CMDSet::Control::parameterWrite,
                           &param, sizeof(hashValue) + len, (void*)ackDecoderCB,
                           allocUCBHandler((void*)userCB, userData), timeout,
                           retry_time);
  } else {
    if (userCB) userCB(ErrorCode::UnifiedErrCode::kErrorSystemError, userData);
  }
}

template <typename DataT>
void FlightAssistant::readParameterByHashAsync(
    ParamHashValue hashValue,
    void (*ackDecoderCB)(Vehicle* vehicle, RecvContainer recvFrame,
                         UCBRetParamHandler<DataT>* ucb),
    void (*userCB)(ErrorCode::ErrCodeType, DataT data, UserData userData),
    UserData userData, int timeout, int retry_time) {
  if (controlLink) {
    controlLink->sendAsync(OpenProtocolCMD::CMDSet::Control::parameterRead,
                           &hashValue, sizeof(hashValue), (void*)ackDecoderCB,
                           allocUCBHandler((void*)userCB, userData), timeout,
                           retry_time);
  } else {
    DataT data;
    if (userCB)
      userCB(ErrorCode::UnifiedErrCode::kErrorSystemError, data, userData);
  }
}
template <typename AckT>
ErrorCode::ErrCodeType FlightAssistant::commonDataUnpacker(
    RecvContainer recvFrame, AckT& ack) {
  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin >= sizeof(AckT)) {
    ack = *(AckT*)(recvFrame.recvData.raw_ack_array);
    return ack.retCode;
  } else {
    DERROR("ACK is exception, data len %d (expect >= %d)\n",
           recvFrame.recvInfo.len - OpenProtocol::PackageMin, sizeof(AckT));
    return ErrorCode::UnifiedErrCode::kErrorInvalidRespond;
  }
}

void FlightAssistant::commonAckDecoder(Vehicle* vehicle,
                                       RecvContainer recvFrame,
                                       UCBRetCodeHandler* ucb) {
  if (ucb && ucb->UserCallBack) {
    CommonAck ack = {0};
    ErrorCode::ErrCodeType ret = 0;
    if (recvFrame.recvInfo.len - OpenProtocol::PackageMin >=
        sizeof(CommonAck)) {
      ack = *(CommonAck*)(recvFrame.recvData.raw_ack_array);
      ret = ErrorCode::UnifiedErrCode::kNoError;
    } else {
      DERROR("ACK is exception, data len %d (expect >= %d)\n",
             recvFrame.recvInfo.len - OpenProtocol::PackageMin,
             sizeof(CommonAck));
      ret = ErrorCode::UnifiedErrCode::kErrorInvalidRespond;
    }
    ucb->UserCallBack(
        (ret != ErrorCode::UnifiedErrCode::kNoError) ? ret : ack.ret_code,
        ucb->userData);
  }
}

FlightAssistant::UCBRetCodeHandler* FlightAssistant::allocUCBHandler(
    void* callback, UserData userData) {
  static int ucbHandlerIndex = 0;
  ucbHandlerIndex++;
  if (ucbHandlerIndex >= maxSize) {
    ucbHandlerIndex = 0;
  }
  ucbHandler[ucbHandlerIndex].UserCallBack =
      (void (*)(ErrorCode::ErrCodeType errCode, UserData userData))callback;
  ucbHandler[ucbHandlerIndex].userData = userData;
  return &(ucbHandler[ucbHandlerIndex]);
}

void FlightAssistant::getRTKEnableDecoder(
    Vehicle* vehicle, RecvContainer recvFrame,
    UCBRetParamHandler<rtkEnableData>* ucb) {
  if (ucb && ucb->UserCallBack) {
    rtkEnableAck ack = {0};
    ErrorCode::ErrCodeType retCode =
        commonDataUnpacker<rtkEnableAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, (rtkEnableData)ack.rtkEnable, ucb->userData);
  }
}

void FlightAssistant::getGoHomeAltitudeDecoder(
    Vehicle* vehicle, RecvContainer recvFrame,
    UCBRetParamHandler<goHomeAltitude>* ucb) {
  if (ucb && ucb->UserCallBack) {
    goHomeAltitudeAck ack = {0};
    ErrorCode::ErrCodeType retCode =
        commonDataUnpacker<goHomeAltitudeAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, (goHomeAltitude)ack.altitude, ucb->userData);
  }
}

ErrorCode::ErrCodeType FlightAssistant::setRtkEnableSync(
    rtkEnableData rtkEnable, int timeout) {
  return writeParameterByHashSync(ParamHashValue::USE_RTK_DATA,
                                  (void*)&rtkEnable, sizeof(rtkEnable),
                                  timeout);
}

void FlightAssistant::setRTKEnableAsync(
    rtkEnableData rtkEnable,
    void (*UserCallBack)(ErrorCode::ErrCodeType, UserData userData),
    UserData userData) {
  writeParameterByHashAsync(ParamHashValue::USE_RTK_DATA, (void*)&rtkEnable,
                            sizeof(rtkEnable), commonAckDecoder, UserCallBack,
                            userData);
}

ErrorCode::ErrCodeType FlightAssistant::getRtkEnableSync(
    rtkEnableData& rtkEnable, int timeout) {
  uint8_t param[MAX_PARAMETER_VALUE_LENGTH];
  ErrorCode::ErrCodeType ret =
      readParameterByHashSync(ParamHashValue::USE_RTK_DATA, param, timeout);
  if (ret == ErrorCode::UnifiedErrCode::kNoError) {
    rtkEnable = *(rtkEnableData*)param;
  }
  return ret;
}

void FlightAssistant::getRTKEnableAsync(
    void (*UserCallBack)(ErrorCode::ErrCodeType, rtkEnableData rtkEnable,
                         UserData userData),
    UserData userData) {
  readParameterByHashAsync<rtkEnableData>(ParamHashValue::USE_RTK_DATA,
                                          getRTKEnableDecoder, UserCallBack,
                                          userData);
}

ErrorCode::ErrCodeType FlightAssistant::setGoHomeAltitudeSync(
    goHomeAltitude altitude, int timeout) {
  if (!goHomeAltitudeValidCheck(altitude)) {
    return ErrorCode::UnifiedErrCode::kErrorInvalidParam;
  }
  ErrorCode::ErrCodeType ret =
      writeParameterByHashSync(ParamHashValue::GO_HOME_ALTITUDE,
                               (void*)&altitude, sizeof(altitude), timeout);
  return ret;
}

void FlightAssistant::setGoHomeAltitudeAsync(
    goHomeAltitude altitude,
    void (*UserCallBack)(ErrorCode::ErrCodeType, UserData userData),
    UserData userData) {
  if (goHomeAltitudeValidCheck(altitude)) {
    writeParameterByHashAsync(ParamHashValue::GO_HOME_ALTITUDE,
                              (void*)&altitude, sizeof(altitude),
                              commonAckDecoder, UserCallBack, userData);
  } else {
    if (UserCallBack) {
      UserCallBack(ErrorCode::UnifiedErrCode::kErrorInvalidParam, userData);
    }
  }
}

ErrorCode::ErrCodeType FlightAssistant::getGoHomeAltitudeSync(
    goHomeAltitude& altitude, int timeout) {
  uint8_t param[MAX_PARAMETER_VALUE_LENGTH];
  ErrorCode::ErrCodeType ret =
      readParameterByHashSync(ParamHashValue::GO_HOME_ALTITUDE, param, timeout);
  if (ret == ErrorCode::UnifiedErrCode::kNoError) {
    altitude = *(rtkEnableData*)param;
  }
  return ret;
}

void FlightAssistant::getGoHomeAltitudeAsync(
    void (*UserCallBack)(ErrorCode::ErrCodeType, goHomeAltitude altitude,
                         UserData userData),
    UserData userData) {
  readParameterByHashAsync(ParamHashValue::GO_HOME_ALTITUDE,
                           getGoHomeAltitudeDecoder, UserCallBack, userData);
}

ErrorCode::ErrCodeType FlightAssistant::setAvoidObstacleSwitchSync(
    AvoidObstacleData avoidObstacle, int timeout) {
  if (controlLink) {
    ACK::ErrorCode rsp = *(ACK::ErrorCode*)controlLink->sendSync(
        OpenProtocolCMD::CMDSet::Intelligent::setAvoidObstacleEnable,
        &avoidObstacle, sizeof(avoidObstacle), timeout);
    if ((rsp.info.len - OpenProtocol::PackageMin <=
         sizeof(ACK::ParamAckInternal))) {
      return rsp.data;
    } else {
      return ErrorCode::UnifiedErrCode::kErrorInvalidRespond;
    }
  } else {
    return ErrorCode::UnifiedErrCode::kErrorSystemError;
  }
}

void FlightAssistant::setAvoidObstacleSwitchAsync(
    AvoidObstacleData avoidObstacle,
    void (*UserCallBack)(ErrorCode::ErrCodeType, UserData userData),
    UserData userData) {
  if (controlLink) {
    controlLink->sendAsync(
        OpenProtocolCMD::CMDSet::Intelligent::setAvoidObstacleEnable,
        &avoidObstacle, sizeof(avoidObstacle), (void*)commonAckDecoder,
        allocUCBHandler((void*)UserCallBack, userData));
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::UnifiedErrCode::kErrorSystemError, userData);
  }
}

ErrorCode::ErrCodeType FlightAssistant::setHomePointSync(
    SetHomepointData homePoint, int timeout) {
  if (controlLink) {
    ACK::ErrorCode rsp = *(ACK::ErrorCode*)controlLink->sendSync(
        OpenProtocolCMD::CMDSet::Control::setHomePoint, &homePoint,
        sizeof(homePoint), timeout);
    if ((rsp.info.len - OpenProtocol::PackageMin <=
         sizeof(ACK::ParamAckInternal))) {
      return rsp.data;
    } else {
      return ErrorCode::UnifiedErrCode::kErrorInvalidRespond;
    }
  } else {
    return ErrorCode::UnifiedErrCode::kErrorSystemError;
  }
}

void FlightAssistant::setHomePointAsync(
    SetHomepointData homePoint,
    void (*UserCallBack)(ErrorCode::ErrCodeType, UserData userData),
    UserData userData) {
  if (controlLink) {
    controlLink->sendAsync(OpenProtocolCMD::CMDSet::Control::setHomePoint,
                           &homePoint, sizeof(homePoint),
                           (void*)commonAckDecoder,
                           allocUCBHandler((void*)UserCallBack, userData));
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::UnifiedErrCode::kErrorSystemError, userData);
  }
}

bool FlightAssistant::goHomeAltitudeValidCheck(goHomeAltitude altitude) {
  if (altitude > MAX_FLIGHT_HEIGHT) {
    DERROR("Go home altitude is larger than MAX_FLIGHT_HEIGHT:%d\n", altitude);
    return false;
  } else
    return true;
}