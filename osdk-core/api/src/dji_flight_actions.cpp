/** @file dji_flight_actions.cpp
 *  @version 3.9
 *  @date April 2019
 *
 *  @brief
 *  FlightActions API for DJI OSDK library
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
#include "dji_flight_actions.hpp"
#include "dji_control_link.hpp"

using namespace DJI;
using namespace DJI::OSDK;

FlightActions::FlightActions(Vehicle *vehicle) {
  controlLink = new ControlLink(vehicle);
}

FlightActions::~FlightActions() { delete this->controlLink; }
/* TODO move this part's code to independent class*/
void FlightActions::commonAckDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                                     UCBRetCodeHandler *ucb) {
  if (ucb && ucb->UserCallBack) {
    CommonAck ack = {0};
    ErrorCode::ErrCodeType ret = 0;
    if (recvFrame.recvInfo.len - OpenProtocol::PackageMin >=
        sizeof(CommonAck)) {
      ack = *(CommonAck *)(recvFrame.recvData.raw_ack_array);
      ret = ErrorCode::getErrorCode(ErrorCode::FCModule,
                                    ErrorCode::FCControlTask, ack.retCode);
    } else {
      DERROR("ACK is exception, data len %d (expect >= %d)\n",
             recvFrame.recvInfo.len - OpenProtocol::PackageMin,
             sizeof(CommonAck));
      ret = ErrorCode::SysCommonErr::UnpackDataMismatch;
    }
    ucb->UserCallBack(ret, ucb->userData);
  }
}

FlightActions::UCBRetCodeHandler *FlightActions::allocUCBHandler(
    void *callback, UserData userData) {
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

template <typename ReqT>
ErrorCode::ErrCodeType FlightActions::actionSync(ReqT req, int timeout) {
  if (controlLink) {
    ACK::ErrorCode *rsp = (ACK::ErrorCode *)controlLink->sendSync(
        OpenProtocolCMD::CMDSet::Control::task, (void *)&req, sizeof(req),
        timeout);
    if (rsp->info.buf &&
        (rsp->info.len - OpenProtocol::PackageMin >= sizeof(CommonAck))) {
      return rsp->data;
    } else {
      return ErrorCode::SysCommonErr::UnpackDataMismatch;
    }
  } else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}


template <typename ReqT>
void FlightActions::actionAsync(
    ReqT req, void (*ackDecoderCB)(Vehicle *vehicle, RecvContainer recvFrame,
                                   UCBRetCodeHandler *ucb),
    void (*userCB)(ErrorCode::ErrCodeType, UserData userData),
    UserData userData, int timeout, int retry_time) {
  if (controlLink) {
    controlLink->sendAsync(OpenProtocolCMD::CMDSet::Control::task, &req,
                           sizeof(req), (void *)ackDecoderCB,
                           allocUCBHandler((void *)userCB, userData), timeout,
                           retry_time);
  } else {
    if (userCB) userCB(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrCodeType FlightActions::startTakeoffSync(int timeout) {
  return actionSync(FlightCommand::TAKE_OFF, timeout);
}

void FlightActions::startTakeoffAsync(
    void (*UserCallBack)(ErrorCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  this->actionAsync(FlightCommand::TAKE_OFF, commonAckDecoder, UserCallBack,
                    userData);
}

ErrorCode::ErrCodeType FlightActions::startForceLandingSync(int timeout) {
  return actionSync(FlightCommand::FORCE_LANDING, timeout);
}

void FlightActions::startForceLandingAsync(
    void (*UserCallBack)(ErrorCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  this->actionAsync(FlightCommand::FORCE_LANDING, commonAckDecoder,
                    UserCallBack, userData);
}

ErrorCode::ErrCodeType FlightActions::startForceLandingAvoidGroundSync(
    int timeout) {
  return actionSync(FlightCommand::FORCE_LANDING_AVOID_GROUND, timeout);
}

void FlightActions::startForceLandingAvoidGroundAsync(
    void (*UserCallBack)(ErrorCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  this->actionAsync(FlightCommand::FORCE_LANDING_AVOID_GROUND, commonAckDecoder,
                    UserCallBack, userData);
}

ErrorCode::ErrCodeType FlightActions::startGoHomeSync(int timeout) {
  return actionSync(FlightCommand::GO_HOME, timeout);
}

void FlightActions::startGoHomeAsync(
    void (*UserCallBack)(ErrorCode::ErrCodeType retCode, UserData userData),
    UserData userData) {
  this->actionAsync(FlightCommand::GO_HOME, commonAckDecoder, UserCallBack,
                    userData);
}