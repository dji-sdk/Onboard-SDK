/** @file dji_flight_link.cpp
 *  @version 4.0.0
 *  @date July 2019
 *
 *  @brief Implementation of flight link
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

#include "dji_flight_link.hpp"
#include <dji_vehicle.hpp>
#include "dji_linker.hpp"

using namespace DJI;
using namespace DJI::OSDK;

FlightLink::FlightLink(Vehicle *vehicle) : vehicle(vehicle) {}

FlightLink::~FlightLink() {}

void FlightLink::sendAsync(const uint8_t cmd[], void *pdata, size_t len,
                            void *callBack, UserData userData, int timeout,
                            int retryTime) {
  vehicle->legacyLinker->sendAsync(cmd, (uint8_t *) pdata, len, timeout,
                                   retryTime, (VehicleCallBack) callBack,
                                   userData);
}

void *FlightLink::sendSync(const uint8_t cmd[], void *pdata, size_t len,
                            int timeout) {
  return vehicle->legacyLinker->sendSync(cmd, (void *) pdata, len,
                                         timeout * 1000 / 2, 2);
}

void FlightLink::linkSendFCAsync(const uint8_t cmd[], const uint8_t *cmdData, size_t len,
                                 Command_SendCallback func,
                                 void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
                                 void *userData, uint32_t timeOut, uint16_t retryTimes)
{
   T_CmdInfo cmdInfo  = {0};

   cmdInfo.cmdSet     = cmd[0];
   cmdInfo.cmdId      = cmd[1];
   cmdInfo.dataLen    = len;
   cmdInfo.needAck    = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
   cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
   cmdInfo.protoType  = PROTOCOL_SDK;
   cmdInfo.receiver   = OSDK_COMMAND_FC_2_DEVICE_ID;
   cmdInfo.addr       = GEN_ADDR(0, ADDR_SDK_COMMAND_INDEX);

   callbackWarpperHandler *handler = (callbackWarpperHandler *)OsdkOsal_Malloc(sizeof(callbackWarpperHandler));
   handler->cb    = UserCallBack;
   handler->udata = userData;

   vehicle->linker->sendAsync(&cmdInfo, cmdData, func, handler, timeOut, retryTimes);
}

E_OsdkStat FlightLink::linkSendFCSync(const uint8_t cmd[], const uint8_t *cmdData, size_t req_len, uint8_t *ackData,
                                      uint32_t *ack_len, uint32_t timeOut, uint16_t retryTimes)
{
   T_CmdInfo cmdInfo = {0};
   T_CmdInfo ackInfo = {0};

   cmdInfo.cmdSet     = cmd[0];
   cmdInfo.cmdId      = cmd[1];
   cmdInfo.dataLen    = req_len;
   cmdInfo.needAck    = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
   cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
   cmdInfo.protoType  = PROTOCOL_SDK;
   cmdInfo.receiver   = OSDK_COMMAND_FC_2_DEVICE_ID;
   cmdInfo.addr       = GEN_ADDR(0, ADDR_SDK_COMMAND_INDEX);

   E_OsdkStat linkAck = vehicle->linker->sendSync(&cmdInfo, cmdData, &ackInfo, ackData, timeOut, retryTimes);

   return linkAck;
}

void FlightLink::sendDirectly(const uint8_t cmd[], void *pdata, size_t len){
   vehicle->legacyLinker->send(cmd,pdata, len);

}
