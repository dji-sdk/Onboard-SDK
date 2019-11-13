/** @file dji_legacy_linker.cpp
 *  @version 4.0
 *  @date November 2019
 *
 *  @brief
 *  Legacy adapting in linker for OSDK 3.9
 *
 *  @Copyright (c) 2016-2017 DJI
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

#include "dji_legacy_linker.hpp"
#include "dji_vehicle.hpp"

#ifdef STM32
#include <stdio.h>
#endif

using namespace DJI;
using namespace DJI::OSDK;

LegacyLinker::LegacyLinker(Vehicle *vehicle)
    : vehicle(vehicle) {
}

LegacyLinker::~LegacyLinker() {
}

void LegacyLinker::send(const uint8_t cmd[], void *pdata, size_t len) {
  T_CmdInfo cmdInfo = {0};

  cmdInfo.cmdSet = cmd[0];
  cmdInfo.cmdId = cmd[1];
  cmdInfo.dataLen = len;
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_NO_NEED;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_SDK_COMMAND_INDEX);
  cmdInfo.encType = vehicle->getEncryption();
  cmdInfo.channelId = 0;
  vehicle->linker->send(&cmdInfo, (uint8_t *) pdata);
}

typedef struct legacyAdaptingData {
  VehicleCallBack cb;
  UserData udata;
  Vehicle *vehicle;
} legacyAdaptingData;

void LegacyLinker::legacyAdaptingAsyncCB(const T_CmdInfo *cmdInfo,
                                         const uint8_t *cmdData,
                                         void *userData, E_OsdkStat cb_type) {
  if (cb_type == OSDK_STAT_OK) {
    if ((!cmdInfo) && (!userData) && (!((legacyAdaptingData *) (userData))->cb)
        && (!((legacyAdaptingData *) (userData))->vehicle)) {
      legacyAdaptingData para = *(legacyAdaptingData *) userData;
      RecvContainer recvFrame = {0};

      recvFrame.dispatchInfo.isAck = true;
      recvFrame.recvInfo.cmd_set = cmdInfo->cmdSet;
      recvFrame.recvInfo.cmd_id = cmdInfo->cmdId;
      memcpy(recvFrame.recvData.raw_ack_array, cmdData, cmdInfo->dataLen);
      recvFrame.dispatchInfo.isCallback = true;
      recvFrame.dispatchInfo.callbackID = cmdInfo->sessionId;
      recvFrame.recvInfo.buf = (uint8_t *) cmdData;
      recvFrame.recvInfo.seqNumber = cmdInfo->seqNum;
      recvFrame.recvInfo.len = cmdInfo->dataLen + OpenProtocol::PackageMin;
      para.cb(para.vehicle, recvFrame, para.udata);
    } else {
      DERROR("Parameter invalid.");
    }
  } else if (cb_type == OSDK_STAT_ERR_TIMEOUT) {
    DERROR("wait for callback time out.");
  } else {
    DERROR("wait for callback error.");
  }

  free(userData);
}

void LegacyLinker::sendAsync(const uint8_t cmd[], void *pdata, size_t len,
                             int timeout, int retry_time,
                             VehicleCallBack callback, UserData userData) {
  T_CmdInfo cmdInfo = {0};

  cmdInfo.cmdSet = cmd[0];
  cmdInfo.cmdId = cmd[1];
  cmdInfo.dataLen = len;
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_SDK_COMMAND_INDEX);
  cmdInfo.encType = vehicle->getEncryption();
  cmdInfo.channelId = 0;
  legacyAdaptingData
      *udata = (legacyAdaptingData *) malloc(sizeof(legacyAdaptingData));
  *udata = {callback, userData, vehicle};

  vehicle->linker->sendAsync(&cmdInfo, (uint8_t *) pdata, legacyAdaptingAsyncCB,
                             udata, timeout, retry_time);
}

ACK::ErrorCode LegacyLinker::sendSync(const uint8_t cmd[], void *pdata,
                                      size_t len, int timeout, int retry_time) {
  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t ackData[1024];
  ACK::ErrorCode ack = {0};

  cmdInfo.cmdSet = cmd[0];
  cmdInfo.cmdId = cmd[1];
  cmdInfo.dataLen = len;
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_SDK_COMMAND_INDEX);
  cmdInfo.encType = vehicle->getEncryption();
  cmdInfo.channelId = 0;

  E_OsdkStat ret =
      vehicle->linker->sendSync(&cmdInfo, (uint8_t *) pdata, &ackInfo, ackData,
                                timeout, retry_time);
  ack.info.buf = ackData;
  ack.info.cmd_set = cmdInfo.cmdSet;
  ack.info.cmd_id = cmdInfo.cmdId;
  ack.info.seqNumber = cmdInfo.seqNum;
  ack.info.len = ackInfo.dataLen;
  ack.info.version = vehicle->getFwVersion();
  ack.data = ackData[0];

  return ack;
}

E_OsdkStat LegacyLinker::legacyAdaptingRegisterCB(
    struct _CommandHandle *cmdHandle,
    const T_CmdInfo *cmdInfo,
    const uint8_t *cmdData, void *userData) {
  legacyAdaptingData *legacyData = (legacyAdaptingData *)userData;
  if (cmdInfo && legacyData && legacyData->cb && legacyData->vehicle) {
    legacyAdaptingData para = *(legacyAdaptingData *) userData;
    RecvContainer recvFrame = {0};

    recvFrame.dispatchInfo.isAck = true;
    recvFrame.recvInfo.cmd_set = cmdInfo->cmdSet;
    recvFrame.recvInfo.cmd_id = cmdInfo->cmdId;
    memcpy(recvFrame.recvData.raw_ack_array, cmdData, cmdInfo->dataLen);
    recvFrame.dispatchInfo.isCallback = true;
    recvFrame.dispatchInfo.callbackID = cmdInfo->sessionId;
    recvFrame.recvInfo.buf = (uint8_t *) cmdData;
    recvFrame.recvInfo.seqNumber = cmdInfo->seqNum;
    recvFrame.recvInfo.len = cmdInfo->dataLen + OpenProtocol::PackageMin;
    para.cb(para.vehicle, recvFrame, para.udata);
    return OSDK_STAT_OK;
  } else {
    DERROR("Parameter invalid.");
    return OSDK_STAT_ERR_PARAM;
  }
}

bool LegacyLinker::registerCMDCallback(uint8_t cmdSet, uint8_t cmdID,
                                       VehicleCallBack callback,
                                       UserData userData) {
  T_RecvCmdHandle
      *recvCmdHandle = (T_RecvCmdHandle *) malloc(sizeof(T_RecvCmdHandle));
  T_RecvCmdItem *item = (T_RecvCmdItem *) malloc(sizeof(T_RecvCmdItem));
  legacyAdaptingData
      *cbParam = (legacyAdaptingData *) malloc(sizeof(legacyAdaptingData));
  *cbParam = {callback, userData, vehicle};

  item->cmdSet = cmdSet;
  item->cmdId = cmdID;
  item->pFunc = legacyAdaptingRegisterCB;
  item->userData = cbParam;
  item->mask = MASK_HOST_XXXXXX_SET_ID;

  recvCmdHandle->cmdCount = 1;
  recvCmdHandle->protoType = PROTOCOL_SDK;
  recvCmdHandle->cmdList = item;
  if (vehicle->linker->registerCmdHandler(recvCmdHandle)) {

    return true;
  } else {
    free(recvCmdHandle->cmdList->userData);
    free(recvCmdHandle->cmdList);
    free(recvCmdHandle);

    return false;
  }
}
