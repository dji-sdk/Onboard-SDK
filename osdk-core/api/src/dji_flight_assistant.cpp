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
#include "dji_control_link.hpp"
using namespace DJI;
using namespace DJI::OSDK;

FlightAssistant::FlightAssistant(Vehicle* vehicle) {
  this->controlLink = new ControlLink(vehicle);
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

  if (rsp.updated && (hashValue == rsp.data.hashValue) &&
      (!memcmp((void*)rsp.data.paramValue, data, len)) &&
      (rsp.info.len - OpenProtocol::PackageMin <=
       sizeof(ACK::ParamAckInternal))) {
    return ErrorCode::SysCommonErr::Success;
  } else {
    if (!rsp.updated)
      return ErrorCode::SysCommonErr::ReqTimeout;
    else if (hashValue != rsp.data.hashValue ||
             !memcmp((void*)rsp.data.paramValue, data, len))
      return ErrorCode::FlightControllerErr::ParamReadWirteErr::
          InvalidParameter;
    else
      return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
}

ErrorCode::ErrCodeType FlightAssistant::readParameterByHashSync(
    ParamHashValue hashValue, void* param, int timeout) {
  ACK::ParamAck rsp = *(ACK::ParamAck*)controlLink->sendSync(
      OpenProtocolCMD::CMDSet::Control::parameterRead, &hashValue,
      sizeof(hashValue), timeout);
  if (rsp.updated && hashValue == rsp.data.hashValue &&
      (rsp.info.len - OpenProtocol::PackageMin <=
       sizeof(ACK::ParamAckInternal))) {
    memcpy(param, rsp.data.paramValue, MAX_PARAMETER_VALUE_LENGTH);
    return ErrorCode::SysCommonErr::Success;
  } else {
    if (!rsp.updated)
      return ErrorCode::SysCommonErr::ReqTimeout;
    else if (hashValue != rsp.data.hashValue)
      return ErrorCode::FlightControllerErr::ParamReadWirteErr::
          InvalidParameter;
    else
      return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
}

void FlightAssistant::writeParameterByHashAsync(
    uint32_t hashValue, void* data, uint8_t len,
    void (*ackDecoderCB)(Vehicle* vehicle, RecvContainer recvFrame,
                         UCBRetCodeHandler* ucb),
    void (*userCB)(ErrorCode::ErrCodeType retCode, UserData userData),
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
    if (userCB) userCB(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

template <typename DataT>
void FlightAssistant::readParameterByHashAsync(
    ParamHashValue hashValue,
    void (*ackDecoderCB)(Vehicle* vehicle, RecvContainer recvFrame,
                         UCBRetParamHandler<DataT>* ucb),
    void (*userCB)(ErrorCode::ErrCodeType retCode, DataT data,
                   UserData userData),
    UserData userData, int timeout, int retry_time) {
  if (controlLink) {
    controlLink->sendAsync(OpenProtocolCMD::CMDSet::Control::parameterRead,
                           &hashValue, sizeof(hashValue), (void*)ackDecoderCB,
                           allocUCBHandler((void*)userCB, userData), timeout,
                           retry_time);
  } else {
    DataT data;
    if (userCB)
      userCB(ErrorCode::SysCommonErr::AllocMemoryFailed, data, userData);
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

template <typename AckT>
ErrorCode::ErrCodeType FlightAssistant::commonDataUnpacker(
    RecvContainer recvFrame, AckT& ack) {
  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin >= sizeof(AckT)) {
    ack = *(AckT*)(recvFrame.recvData.raw_ack_array);
    return ack.retCode;
  } else {
    DERROR("ACK is exception, data len %d (expect >= %d)\n",
           recvFrame.recvInfo.len - OpenProtocol::PackageMin, sizeof(AckT));
    return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
}

void FlightAssistant::setParameterDecoder(Vehicle* vehicle,
                                          RecvContainer recvFrame,
                                          UCBRetCodeHandler* ucb) {
  if (ucb && ucb->UserCallBack) {
    ACK::ParamAckInternal ack = {0};
    ErrorCode::ErrCodeType ret = 0;
    if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <=
        sizeof(ACK::ParamAckInternal)) {
      ack = *(ACK::ParamAckInternal*)(recvFrame.recvData.raw_ack_array);
      ret = ErrorCode::getErrorCode(ErrorCode::FCModule,
                                    ErrorCode::FCParameterTable, ack.retCode);
    } else {
      DERROR("ACK is exception, data len %d (expect >= %d)\n",
             recvFrame.recvInfo.len - OpenProtocol::PackageMin,
             sizeof(ACK::ParamAckInternal));
      ret = ErrorCode::SysCommonErr::UnpackDataMismatch;
    }
    ucb->UserCallBack(ret, ucb->userData);
  }
}

void FlightAssistant::getRtkEnableDecoder(
    Vehicle* vehicle, RecvContainer recvFrame,
    UCBRetParamHandler<RtkEnableData>* ucb) {
  if (ucb && ucb->UserCallBack) {
    RtkEnableAck ack = {0};
    ErrorCode::ErrCodeType retCode =
        commonDataUnpacker<RtkEnableAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, (RtkEnableData)ack.rtkEnable, ucb->userData);
  }
}

void FlightAssistant::getGoHomeAltitudeDecoder(
    Vehicle* vehicle, RecvContainer recvFrame,
    UCBRetParamHandler<GoHomeAltitude>* ucb) {
  if (ucb && ucb->UserCallBack) {
    GoHomeAltitudeAck ack = {0};
    ErrorCode::ErrCodeType retCode =
        commonDataUnpacker<GoHomeAltitudeAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, (GoHomeAltitude)ack.altitude, ucb->userData);
  }
}

void FlightAssistant::setHomePointAckDecoder(Vehicle* vehicle,
                                             RecvContainer recvFrame,
                                             UCBRetCodeHandler* ucb) {
  if (ucb && ucb->UserCallBack) {
    ACK::ErrorCode ack = {0};
    ErrorCode::ErrCodeType ret = 0;
    if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <= sizeof(ack.data)) {
      ack.data = *(uint32_t*)(recvFrame.recvData.raw_ack_array);
      ret = ErrorCode::getErrorCode(ErrorCode::FCModule,
                                    ErrorCode::FCSetHomePoint, ack.data);
    } else {
      DERROR("ACK is exception, data len %d (expect >= %d)\n",
             recvFrame.recvInfo.len - OpenProtocol::PackageMin,
             sizeof(ack.data));
      ret = ErrorCode::SysCommonErr::UnpackDataMismatch;
    }
    ucb->UserCallBack(ret, ucb->userData);
  }
}

void FlightAssistant::avoidObstacleAckDecoder(Vehicle* vehicle,
                                              RecvContainer recvFrame,
                                              UCBRetCodeHandler* ucb) {
  if (ucb && ucb->UserCallBack) {
    ACK::ErrorCode ack = {0};
    ErrorCode::ErrCodeType ret = 0;
    ack.data = *(uint32_t*)(recvFrame.recvData.raw_ack_array);
    if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <= sizeof(ack.data)) {
      /*! NOTE:There is no ret code for this function , so ack mean's successful */
      ret == ErrorCode::SysCommonErr::Success;
    } else {
      DERROR("ACK is exception, data len %d (expect >= %d)\n",
             recvFrame.recvInfo.len - OpenProtocol::PackageMin,
             sizeof(ack.data));
      ret = ErrorCode::SysCommonErr::UnpackDataMismatch;
    }
    ucb->UserCallBack(ret, ucb->userData);
  }
}

ErrorCode::ErrCodeType FlightAssistant::setRtkEnableSync(
    RtkEnableData rtkEnable, int timeout) {
  return writeParameterByHashSync(ParamHashValue::USE_RTK_DATA,
                                  (void*)&rtkEnable, sizeof(rtkEnable),
                                  timeout);
}

void FlightAssistant::setRtkEnableAsync(
    RtkEnableData rtkEnable,
    void (*UserCallBack)(ErrorCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  writeParameterByHashAsync(ParamHashValue::USE_RTK_DATA, (void*)&rtkEnable,
                            sizeof(rtkEnable), setParameterDecoder,
                            UserCallBack, userData);
}

ErrorCode::ErrCodeType FlightAssistant::getRtkEnableSync(
    RtkEnableData& rtkEnable, int timeout) {
  uint8_t param[MAX_PARAMETER_VALUE_LENGTH];
  ErrorCode::ErrCodeType ret =
      readParameterByHashSync(ParamHashValue::USE_RTK_DATA, param, timeout);
  if (ret == ErrorCode::SysCommonErr::Success) {
    rtkEnable = *(RtkEnableData*)param;
  }
  return ret;
}

void FlightAssistant::getRtkEnableAsync(
    void (*UserCallBack)(ErrorCode::ErrCodeType retCode,
                         RtkEnableData rtkEnable, UserData userData),
    UserData userData) {
  readParameterByHashAsync<RtkEnableData>(ParamHashValue::USE_RTK_DATA,
                                          getRtkEnableDecoder, UserCallBack,
                                          userData);
}

ErrorCode::ErrCodeType FlightAssistant::setGoHomeAltitudeSync(
    GoHomeAltitude altitude, int timeout) {
  if (!goHomeAltitudeValidCheck(altitude)) {
    return ErrorCode::FlightControllerErr::ParamReadWirteErr::InvalidParameter;
  }
  ErrorCode::ErrCodeType ret =
      writeParameterByHashSync(ParamHashValue::GO_HOME_ALTITUDE,
                               (void*)&altitude, sizeof(altitude), timeout);
  return ret;
}

void FlightAssistant::setGoHomeAltitudeAsync(
    GoHomeAltitude altitude,
    void (*UserCallBack)(ErrorCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  if (goHomeAltitudeValidCheck(altitude)) {
    writeParameterByHashAsync(ParamHashValue::GO_HOME_ALTITUDE,
                              (void*)&altitude, sizeof(altitude),
                              setParameterDecoder, UserCallBack, userData);
  } else
    UserCallBack(
        ErrorCode::FlightControllerErr::ParamReadWirteErr::InvalidParameter,
        userData);
}

ErrorCode::ErrCodeType FlightAssistant::getGoHomeAltitudeSync(
    GoHomeAltitude& altitude, int timeout) {
  uint8_t param[MAX_PARAMETER_VALUE_LENGTH];
  ErrorCode::ErrCodeType ret =
      readParameterByHashSync(ParamHashValue::GO_HOME_ALTITUDE, param, timeout);
  if (ret == ErrorCode::SysCommonErr::Success) {
    altitude = *(RtkEnableData*)param;
  }
  return ret;
}

void FlightAssistant::getGoHomeAltitudeAsync(
    void (*UserCallBack)(ErrorCode::ErrCodeType retCode,
                         GoHomeAltitude altitude, UserData userData),
    UserData userData) {
  readParameterByHashAsync(ParamHashValue::GO_HOME_ALTITUDE,
                           getGoHomeAltitudeDecoder, UserCallBack, userData);
}

ErrorCode::ErrCodeType FlightAssistant::setAvoidObstacleSwitchSync(
    AvoidObstacleData avoidObstacle, int timeout) {
  if (controlLink) {
    ACK::ErrorCode ack = *(ACK::ErrorCode*)controlLink->sendSync(
        OpenProtocolCMD::CMDSet::Intelligent::setAvoidObstacleEnable,
        &avoidObstacle, sizeof(avoidObstacle), timeout);

    uint8_t ackData = (uint8_t)ack.data;
    if ((ack.info.len - OpenProtocol::PackageMin <=
         sizeof(ack.data)) &&
        ((ackData & 0x01) == avoidObstacle.frontBrakeFLag) &&
        (((ackData & 0x02) >> 1) == avoidObstacle.rightBrakeFlag) &&
        (((ackData & 0x04) >> 2) == avoidObstacle.backBrakeFlag) &&
        (((ackData & 0x08) >> 3) == avoidObstacle.leftBrakeFlag) &&
        (((ackData & 0x10) >> 4) == avoidObstacle.activeAvoidFlag)) {
      return ErrorCode::SysCommonErr::Success;
    } else {
      return ErrorCode::SysCommonErr::UnpackDataMismatch;
    }
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void FlightAssistant::setAvoidObstacleSwitchAsync(
    AvoidObstacleData avoidObstacle,
    void (*UserCallBack)(ErrorCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  if (controlLink) {
    controlLink->sendAsync(
        OpenProtocolCMD::CMDSet::Intelligent::setAvoidObstacleEnable,
        &avoidObstacle, sizeof(avoidObstacle), (void*)avoidObstacleAckDecoder,
        allocUCBHandler((void*)UserCallBack, userData));
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
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
      return ErrorCode::getErrorCode(ErrorCode::FCModule,
                                     ErrorCode::FCSetHomePoint, rsp.data);
    } else {
      return ErrorCode::SysCommonErr::UnpackDataMismatch;
    }
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void FlightAssistant::setHomePointAsync(
    SetHomepointData homePoint,
    void (*UserCallBack)(ErrorCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  if (controlLink) {
    controlLink->sendAsync(OpenProtocolCMD::CMDSet::Control::setHomePoint,
                           &homePoint, sizeof(homePoint),
                           (void*)setHomePointAckDecoder,
                           allocUCBHandler((void*)UserCallBack, userData));
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

bool FlightAssistant::goHomeAltitudeValidCheck(GoHomeAltitude altitude) {
  if (altitude < MIN_GO_HOME_HEIGHT || altitude > MAX_FLIGHT_HEIGHT) {
    DERROR(
        "Go home altitude is not in between MIN_GO_HOME_HEIGHT and  "
        "MAX_FLIGHT_HEIGHT:%d\n",
        altitude);
    return false;
  } else
    return true;
}
