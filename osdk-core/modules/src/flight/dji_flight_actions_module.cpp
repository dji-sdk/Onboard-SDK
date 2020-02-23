/** @file dji_flight_actions_module.cpp
 *  @version 3.9
 *  @date August 2019
 *
 *  @brief Implementation of flight actions module
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

#include "dji_flight_actions_module.hpp"
#include <dji_vehicle.hpp>
#include "dji_flight_link.hpp"
#include "osdk_device_id.h"
#include "dji_linker.hpp"

using namespace DJI;
using namespace DJI::OSDK;

FlightActions::FlightActions(Vehicle* vehicle) {
  flightLink = new FlightLink(vehicle);
  setUSBFlightOn(vehicle, true);
}

FlightActions::~FlightActions() { delete this->flightLink; }

bool FlightActions::setUSBFlightOn(Vehicle* v, bool en) {
  uint8_t retryTimes = 4;
  uint16_t l = 1;
  uint16_t h = 1;
  uint16_t c = (h << 8) | (l & 0xff);
  uint8_t lBit = c & 1;
  uint8_t hBit = (uint16_t)c >> 15;
  c |= 1 << 8;
  c |= 1 << 15;

  USBCtrlData data;
  memset(&data, 0, sizeof(data));
  data.version = c;
  data.cmd = (en) ? 1 : 0;

  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t cbData[1024] = {0};

  cmdInfo.cmdSet = V1ProtocolCMD::fc::usbFlightMode[0];
  cmdInfo.cmdId = V1ProtocolCMD::fc::usbFlightMode[1];
  cmdInfo.dataLen = sizeof(data);
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.receiver = OSDK_COMMAND_FC_DEVICE_ID;
  cmdInfo.sender = v->linker->getLocalSenderId();
retryUSBFlight:
  DSTATUS("Trying to set usb-connected-flight as [%s]", en ? "enable" : "disable");
  E_OsdkStat ret =
      v->linker->sendSync(&cmdInfo, (uint8_t*)&data, &ackInfo, cbData, 1000, 3);
  if ((ret != OSDK_STAT_OK) || (cbData[0] == 0x00)) {
    retryTimes--;
    if (retryTimes > 0) {
      OsdkOsal_TaskSleepMs(1000);
      goto retryUSBFlight;
    } else {
      DERROR("Configure usb-connected-flight failed! Cannot take off !");
      return false;
    }
  } else {
    DSTATUS("Configure usb-connected-flight successfully.");
    return true;
  }
}

FlightActions::UCBRetCodeHandler* FlightActions::allocUCBHandler(
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

void FlightActions::commonAckDecoder(Vehicle* vehicle, RecvContainer recvFrame,
                                     UCBRetCodeHandler* ucb) {
  if (ucb && ucb->UserCallBack) {
    CommonAck ack = {0};
    ErrorCode::ErrorCodeType ret = 0;
    if (recvFrame.recvInfo.len - OpenProtocol::PackageMin >=
        sizeof(CommonAck)) {
      ack = *(CommonAck*)(recvFrame.recvData.raw_ack_array);
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

ErrorCode::ErrorCodeType FlightActions::actionSync(uint8_t req, int timeout) {
  if (flightLink) {
    ACK::ErrorCode* rsp = (ACK::ErrorCode*)flightLink->sendSync(
        OpenProtocolCMD::CMDSet::Control::task, (void*)&req, sizeof(req),
        timeout);
    if (rsp->info.buf &&
        (rsp->info.len - OpenProtocol::PackageMin >= sizeof(CommonAck))) {
      return ErrorCode::getErrorCode(ErrorCode::FCModule,
                                     ErrorCode::FCControlTask, rsp->data);

    } else {
      return ErrorCode::SysCommonErr::UnpackDataMismatch;
    }
  } else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightActions::actionAsync(FlightCommand req,
                                void (*ackDecoderCB)(Vehicle* vehicle,
                                                     RecvContainer recvFrame,
                                                     UCBRetCodeHandler* ucb),
                                void (*userCB)(ErrorCode::ErrorCodeType,
                                               UserData userData),
                                UserData userData, int timeout, int retryTime) {
  if (flightLink) {
    flightLink->sendAsync(OpenProtocolCMD::CMDSet::Control::task, &req,
                          sizeof(req), (void*)ackDecoderCB,
                          allocUCBHandler((void*)userCB, userData), timeout,
                          retryTime);
  } else {
    if (userCB) userCB(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType FlightActions::EmergencyBrakeActionSync(uint8_t req,
                                                                 int timeout) {
  if (flightLink) {
    ACK::ErrorCode* rsp = (ACK::ErrorCode*)flightLink->sendSync(
        OpenProtocolCMD::CMDSet::Control::emergencyBrake, (void*)&req,
        sizeof(req), timeout);
    if (rsp->info.buf &&
        (rsp->info.len - OpenProtocol::PackageMin >= sizeof(CommonAck))) {
      return ErrorCode::getErrorCode(ErrorCode::FCModule,
                                     ErrorCode::FCEmergencyBrake, rsp->data);

    } else {
      return ErrorCode::SysCommonErr::UnpackDataMismatch;
    }
  } else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

ErrorCode::ErrorCodeType FlightActions::killSwitch(KillSwitch cmd,
                                                   int wait_timeout,
                                                   char debugMsg[10]) {
  if (flightLink) {
    ACK::ErrorCode ack;
    KillSwitchData data;
    data.high_version = 0x01;
    data.low_version = 0x01;
    memcpy(data.debug_description, debugMsg, 10);
    data.cmd = cmd;
    data.reserved = 0;
    ACK::ErrorCode* rsp = (ACK::ErrorCode*)flightLink->sendSync(
        OpenProtocolCMD::CMDSet::Control::killSwitch, (void*)&data,
        sizeof(data), wait_timeout);

    if (rsp->info.buf &&
        (rsp->info.len - OpenProtocol::PackageMin >= sizeof(CommonAck))) {
      return ErrorCode::getErrorCode(ErrorCode::FCModule,
                                     ErrorCode::FCEmergencyBrake, rsp->data);
    } else {
      return ErrorCode::SysCommonErr::UnpackDataMismatch;
    }
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}