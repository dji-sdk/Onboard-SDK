/** @file dji_flight_assistant_module.cpp
 *  @version 3.9
 *  @date July 2019
 *
 *  @brief Implementation of flight module
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

#include "dji_flight_assistant_module.hpp"
#include <dji_vehicle.hpp>
#include "dji_flight_link.hpp"

using namespace DJI;
using namespace DJI::OSDK;

FlightAssistant::FlightAssistant(Vehicle* vehicle) {
  flightLink = new FlightLink(vehicle);
}

FlightAssistant::~FlightAssistant() { delete this->flightLink; }

ErrorCode::ErrorCodeType FlightAssistant::writeParameterByHashSync(
    uint32_t hashValue, void* data, uint8_t len, int timeout) {
  ParameterData param = {0};
  param.hashValue = hashValue;
  memcpy(param.paramValue, data, len);

  ACK::ParamAck rsp = *(ACK::ParamAck*)flightLink->sendSync(
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
      return ErrorCode::FlightControllerErr::ParamReadWriteErr::
          InvalidParameter;
    else
      return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
}

void FlightAssistant::writeParameterByHashAsync(
    uint32_t hashValue, void* data, uint8_t len,
    void (*ackDecoderCB)(Vehicle* vehicle, RecvContainer recvFrame,
                         UCBRetCodeHandler* ucb),
    void (*userCB)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData, int timeout, int retryTime) {
  if (flightLink) {
    ParameterData param = {0};
    param.hashValue = hashValue;
    memcpy(param.paramValue, data, len);
    flightLink->sendAsync(
        OpenProtocolCMD::CMDSet::Control::parameterWrite, &param,
        sizeof(hashValue) + len, (void*)ackDecoderCB,
        allocUCBHandler((void*)userCB, userData), timeout, retryTime);
  } else {
    if (userCB) userCB(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType FlightAssistant::readParameterByHashSync(
    ParamHashValue hashValue, void* param, int timeout) {
  ACK::ParamAck rsp = *(ACK::ParamAck*)flightLink->sendSync(
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
      return ErrorCode::FlightControllerErr::ParamReadWriteErr::
          InvalidParameter;
    else
      return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
}

template <typename DataT>
void FlightAssistant::readParameterByHashAsync(
    ParamHashValue hashValue,
    void (*ackDecoderCB)(Vehicle* vehicle, RecvContainer recvFrame,
                         UCBRetParamHandler<DataT>* ucb),
    void (*userCB)(ErrorCode::ErrorCodeType retCode, DataT data,
                   UserData userData),
    UserData userData, int timeout, int retryTime) {
  if (flightLink) {
    flightLink->sendAsync(
        OpenProtocolCMD::CMDSet::Control::parameterRead, &hashValue,
        sizeof(hashValue), (void*)ackDecoderCB,
        allocUCBHandler((void*)userCB, userData), timeout, retryTime);
  } else {
    DataT data  = {};
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
      (void (*)(ErrorCode::ErrorCodeType errCode, UserData userData))callback;
  ucbHandler[ucbHandlerIndex].userData = userData;
  return &(ucbHandler[ucbHandlerIndex]);
}

template <typename AckT>
ErrorCode::ErrorCodeType FlightAssistant::commonDataUnpacker(
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
    ErrorCode::ErrorCodeType ret = 0;
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
    ErrorCode::ErrorCodeType retCode =
        commonDataUnpacker<RtkEnableAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, (RtkEnableData)ack.rtkEnable, ucb->userData);
  }
}

void FlightAssistant::getGoHomeAltitudeDecoder(
    Vehicle* vehicle, RecvContainer recvFrame,
    UCBRetParamHandler<GoHomeAltitude>* ucb) {
  if (ucb && ucb->UserCallBack) {
    GoHomeAltitudeAck ack = {0};
    ErrorCode::ErrorCodeType retCode =
        commonDataUnpacker<GoHomeAltitudeAck>(recvFrame, ack);
    ucb->UserCallBack(retCode, (GoHomeAltitude)ack.altitude, ucb->userData);
  }
}

void FlightAssistant::setHomePointAckDecoder(Vehicle* vehicle,
                                              RecvContainer recvFrame,
                                              UCBRetCodeHandler* ucb) {
  if (ucb && ucb->UserCallBack) {
    ACK::ErrorCode ack = {0};
    ErrorCode::ErrorCodeType ret = 0;
    if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <= sizeof(ack.data)) {
      ack.data = *(uint32_t*)(recvFrame.recvData.raw_ack_array);
      ret = ErrorCode::getErrorCode(ErrorCode::FCModule,
                                    ErrorCode::FCSetHomeLocation, ack.data);
    } else {
      DERROR("ACK is exception, data len %d (expect >= %d)\n",
             recvFrame.recvInfo.len - OpenProtocol::PackageMin,
             sizeof(ack.data));
      ret = ErrorCode::SysCommonErr::UnpackDataMismatch;
    }
    ucb->UserCallBack(ret, ucb->userData);
  }
}

ErrorCode::ErrorCodeType FlightAssistant::setRtkEnableSync(
    RtkEnableData rtkEnable, int timeout) {
  return writeParameterByHashSync(ParamHashValue::USE_RTK_DATA,
                                  (void*)&rtkEnable, sizeof(rtkEnable),
                                  timeout);
}

void FlightAssistant::setRtkEnableAsync(
    RtkEnableData rtkEnable,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  writeParameterByHashAsync(ParamHashValue::USE_RTK_DATA, (void*)&rtkEnable,
                            sizeof(rtkEnable), setParameterDecoder,
                            UserCallBack, userData);
}

ErrorCode::ErrorCodeType FlightAssistant::getRtkEnableSync(
    RtkEnableData& rtkEnable, int timeout) {
  uint8_t param[MAX_PARAMETER_VALUE_LENGTH];
  ErrorCode::ErrorCodeType ret =
      readParameterByHashSync(ParamHashValue::USE_RTK_DATA, param, timeout);
  if (ret == ErrorCode::SysCommonErr::Success) {
    rtkEnable = *(RtkEnableData*)param;
  }
  return ret;
}

void FlightAssistant::getRtkEnableAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         RtkEnableData rtkEnable, UserData userData),
    UserData userData) {
  readParameterByHashAsync<RtkEnableData>(ParamHashValue::USE_RTK_DATA,
                                          getRtkEnableDecoder, UserCallBack,
                                          userData);
}

ErrorCode::ErrorCodeType FlightAssistant::setGoHomeAltitudeSync(
    GoHomeAltitude altitude, int timeout) {
  if (!goHomeAltitudeValidCheck(altitude)) {
    return ErrorCode::FlightControllerErr::ParamReadWriteErr::InvalidParameter;
  }
  ErrorCode::ErrorCodeType ret =
      writeParameterByHashSync(ParamHashValue::GO_HOME_ALTITUDE,
                               (void*)&altitude, sizeof(altitude), timeout);
  return ret;
}

void FlightAssistant::setGoHomeAltitudeAsync(
    GoHomeAltitude altitude,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (goHomeAltitudeValidCheck(altitude)) {
    writeParameterByHashAsync(ParamHashValue::GO_HOME_ALTITUDE,
                              (void*)&altitude, sizeof(altitude),
                              setParameterDecoder, UserCallBack, userData);
  } else
    UserCallBack(
        ErrorCode::FlightControllerErr::ParamReadWriteErr::InvalidParameter,
        userData);
}

ErrorCode::ErrorCodeType FlightAssistant::getGoHomeAltitudeSync(
    GoHomeAltitude& altitude, int timeout) {
  uint8_t param[MAX_PARAMETER_VALUE_LENGTH];
  ErrorCode::ErrorCodeType ret =
      readParameterByHashSync(ParamHashValue::GO_HOME_ALTITUDE, param, timeout);
  if (ret == ErrorCode::SysCommonErr::Success) {
    altitude = *(GoHomeAltitude*)param;
  }
  return ret;
}

void FlightAssistant::getGoHomeAltitudeAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         GoHomeAltitude altitude, UserData userData),
    UserData userData) {
  readParameterByHashAsync<GoHomeAltitude>(ParamHashValue::GO_HOME_ALTITUDE,
                                           getGoHomeAltitudeDecoder,
                                           UserCallBack, userData);
}

ErrorCode::ErrorCodeType FlightAssistant::setHomeLocationSync(
  SetHomeLocationData homeLocation, int timeout) {
  if (flightLink) {
    ACK::ErrorCode rsp = *(ACK::ErrorCode*)flightLink->sendSync(
        OpenProtocolCMD::CMDSet::Control::setHomeLocation, &homeLocation,
        sizeof(homeLocation), timeout);
    if ((rsp.info.len - OpenProtocol::PackageMin <=
         sizeof(ACK::ParamAckInternal))) {
      return ErrorCode::getErrorCode(ErrorCode::FCModule,
                                     ErrorCode::FCSetHomeLocation, rsp.data);
    } else {
      return ErrorCode::SysCommonErr::UnpackDataMismatch;
    }
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void FlightAssistant::setHomeLocationAsync(
  SetHomeLocationData homeLocation,
  void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
  UserData userData) {
  if (flightLink) {
    flightLink->sendAsync(OpenProtocolCMD::CMDSet::Control::setHomeLocation,
                              &homeLocation, sizeof(homeLocation),
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
