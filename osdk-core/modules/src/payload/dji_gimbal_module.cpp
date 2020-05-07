/** @file dji_gimbal_module.cpp
 *  @version 4.0
 *  @date November 2019
 *
 *  @brief Implementation of gimbal module for payload node
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

#include "dji_gimbal_module.hpp"
#include "dji_linker.hpp"
#include "dji_legacy_linker.hpp"
#include "dji_internal_command.hpp"

#include <vector>
#include "osdk_device_id.h"
using namespace DJI;
using namespace DJI::OSDK;

GimbalModule::GimbalModule(Linker *linker, PayloadIndexType payloadIndex,
                           std::string name, bool enable) :
                           PayloadBase(linker, payloadIndex, name, enable) {}

GimbalModule::~GimbalModule(){
}

void callbackWrapperFunc(const T_CmdInfo *cmdInfo,
                                       const uint8_t *cmdData,
                                       void *userData, E_OsdkStat cb_type) {
  if(!userData)
    return;

  GimbalModule::callbackWarpperHandler
      *handler = (GimbalModule::callbackWarpperHandler *) userData;
  if ((cmdInfo) && (cmdInfo->dataLen >= sizeof(uint8_t))) {
    if (handler->cb) {
      ErrorCode::ErrorCodeType ret = ErrorCode::getLinkerErrorCode(cb_type);
      if(ret != ErrorCode::SysCommonErr::Success) {
        handler->cb(ret, handler->udata);
      } else {
        ret = ErrorCode::getErrorCode(ErrorCode::GimbalModule,
                                      ErrorCode::GimbalCommon, cmdData[0]);
        handler->cb(ret, handler->udata);
      }
    }
  } else {
    handler->cb(ErrorCode::getLinkerErrorCode(cb_type), handler->udata);
  }

  free(userData);
}

void GimbalModule::resetAsync(
                void (*userCB)(ErrorCode::ErrorCodeType retCode,
                                     UserData userData),
                UserData userData) {
    if (getEnable()) {
      gimbalWorkModeAndReturnCenterSetting setting;
      setting.workMode = WORK_MODE_DONT_CHANGE;
      setting.returnCenterCmd = PITCH_AND_YAW;
      T_CmdInfo cmdInfo = {0};

      cmdInfo.cmdSet = V1ProtocolCMD::Gimbal::resetAngle[0];
      cmdInfo.cmdId = V1ProtocolCMD::Gimbal::resetAngle[1];
      cmdInfo.dataLen = sizeof(setting);
      cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
      cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
      cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
      uint8_t V1GimbalIndex =
          getIndex() == PAYLOAD_INDEX_0 ? getIndex() : getIndex() + 1;
      cmdInfo.receiver = OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_GIMBAL,
                                                V1GimbalIndex);
      cmdInfo.sender = getLinker()->getLocalSenderId();

      callbackWarpperHandler *handler = (callbackWarpperHandler *)malloc(sizeof(callbackWarpperHandler));
      handler->cb = userCB;
      handler->udata = userData;

      getLinker()->sendAsync(&cmdInfo, (uint8_t *) &setting, callbackWrapperFunc,
                        handler, 500, 4);
    } else {
      if (userCB) userCB(ErrorCode::SysCommonErr::ReqNotSupported, userData);
    }
}

ErrorCode::ErrorCodeType GimbalModule::resetSync(int timeout) {
  if (!getEnable()) return ErrorCode::SysCommonErr::ReqNotSupported;

  gimbalWorkModeAndReturnCenterSetting setting;
  setting.workMode = WORK_MODE_DONT_CHANGE;
  setting.returnCenterCmd = PITCH_AND_YAW;
  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t ackData[1024];

  cmdInfo.cmdSet = V1ProtocolCMD::Gimbal::resetAngle[0];
  cmdInfo.cmdId = V1ProtocolCMD::Gimbal::resetAngle[1];
  cmdInfo.dataLen = sizeof(setting);
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  uint8_t V1GimbalIndex =
      getIndex() == PAYLOAD_INDEX_0 ? getIndex() : getIndex() + 1;
  cmdInfo.receiver =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_GIMBAL, V1GimbalIndex);
  cmdInfo.sender = getLinker()->getLocalSenderId();
  E_OsdkStat linkAck =
      getLinker()->sendSync(&cmdInfo, (uint8_t *) &setting, &ackInfo, ackData,
                       timeout * 1000 / 4, 4);
  ErrorCode::ErrorCodeType ret = ErrorCode::getLinkerErrorCode(linkAck);
  if (ret != ErrorCode::SysCommonErr::Success) return ret;
  if (ackInfo.dataLen >= sizeof(retCodeType)) {
    return ErrorCode::getErrorCode(ErrorCode::GimbalModule,
                                   ErrorCode::GimbalCommon, ackData[0]);
  } else {
    return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
}

void GimbalModule::rotateAsync(Rotation rotation,
                               void (*userCB)(ErrorCode::ErrorCodeType retCode,
                                              UserData userData),
                               UserData userData) {
  if (getEnable()) {
    gimbalAngleSetting setting = {0};
    setting.yaw_angle = rotation.yaw * 10;
    setting.pitch_angle = rotation.pitch * 10;
    setting.roll_angle = rotation.roll * 10;
    setting.allowance = 10;  /*!< 0.1 degree allowance */
    setting.reference = 0; /*!< default reference (initial point) */
    setting.coordinate = rotation.rotationMode;
    setting.is_control = 1; /*!< take control */
    setting.timeout = 0; /*!< default 2s timeout */
    setting.time_for_action = rotation.time;

    T_CmdInfo cmdInfo = {0};
    cmdInfo.cmdSet = V1ProtocolCMD::Gimbal::rotateAngle[0];
    cmdInfo.cmdId = V1ProtocolCMD::Gimbal::rotateAngle[1];
    cmdInfo.dataLen = sizeof(setting);
    cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
    cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
    cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
    uint8_t V1GimbalIndex =
        getIndex() == PAYLOAD_INDEX_0 ? getIndex() : getIndex() + 1;
    cmdInfo.receiver =
        OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_GIMBAL, V1GimbalIndex);
    cmdInfo.sender = getLinker()->getLocalSenderId();

    callbackWarpperHandler *handler = (callbackWarpperHandler *)malloc(sizeof(callbackWarpperHandler));
    handler->cb = userCB;
    handler->udata = userData;

    getLinker()->sendAsync(&cmdInfo, (uint8_t *) &setting, callbackWrapperFunc,
                      handler, 500, 4);
  } else {
    if (userCB) userCB(ErrorCode::SysCommonErr::ReqNotSupported, userData);
  }
}

ErrorCode::ErrorCodeType GimbalModule::rotateSync(Rotation rotation,
                                                  int timeout) {
  if (!getEnable()) return ErrorCode::SysCommonErr::ReqNotSupported;

  gimbalAngleSetting setting = {0};
  setting.yaw_angle = (int16_t)(rotation.yaw * 10);
  setting.pitch_angle = (int16_t)(rotation.pitch * 10);
  setting.roll_angle = (int16_t)(rotation.roll * 10);
  setting.allowance = 10;  /*!< 0.1 degree allowance */
  setting.reference = 0; /*!< default reference (initial point) */
  setting.coordinate = rotation.rotationMode;
  setting.is_control = 1; /*!< take control */
  setting.timeout = 0; /*!< default 2s timeout */
  setting.time_for_action = rotation.time * 100;

  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t ackData[1024];

  cmdInfo.cmdSet = V1ProtocolCMD::Gimbal::rotateAngle[0];
  cmdInfo.cmdId = V1ProtocolCMD::Gimbal::rotateAngle[1];
  cmdInfo.dataLen = sizeof(setting);
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  uint8_t V1GimbalIndex =
      getIndex() == PAYLOAD_INDEX_0 ? getIndex() : getIndex() + 1;
  cmdInfo.receiver =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_GIMBAL, V1GimbalIndex);
  cmdInfo.sender = getLinker()->getLocalSenderId();
  E_OsdkStat linkAck =
      getLinker()->sendSync(&cmdInfo, (uint8_t *) &setting, &ackInfo, ackData,
                       timeout * 1000 / 4, 4);
  ErrorCode::ErrorCodeType ret = ErrorCode::getLinkerErrorCode(linkAck);
  if (ret != ErrorCode::SysCommonErr::Success) return ret;
  if (ackInfo.dataLen >= sizeof(retCodeType)) {
    return ErrorCode::getErrorCode(ErrorCode::GimbalModule,
                                   ErrorCode::GimbalCommon, ackData[0]);
  } else {
    return ErrorCode::SysCommonErr::UnpackDataMismatch;
  }
}
