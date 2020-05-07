/** @file dji_perception_impl.cpp
 *  @version 4.0
 *  @date Jan 2020
 *
 *  @brief DJI perception API implementation of OSDK
 *
 *  @Copyright (c) 2020 DJI
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


#include <dji_vehicle.hpp>
#include "dji_perception_impl.hpp"
#include "osdk_osal.h"

using namespace DJI;
using namespace DJI::OSDK;

const char PerceptionImpl::rectifyDownLeft[11] = "rectify_dl";
const char PerceptionImpl::rectifyDownRight[11] = "rectify_dr";
const char PerceptionImpl::rectifyFrontLeft[11] = "rectify_fl";
const char PerceptionImpl::rectifyFrontRight[11] = "rectify_fr";
const char PerceptionImpl::rectifyRearLeft[11] = "rectify_bl";
const char PerceptionImpl::rectifyRearRight[11] = "rectify_br";
const char PerceptionImpl::rectifyUpLeft[11] = "rectify_ul";
const char PerceptionImpl::rectifyUpRight[11] = "rectify_ur";
const char PerceptionImpl::rectifyLeftLeft[11] = "rectify_ll";
const char PerceptionImpl::rectifyLeftRight[11] = "rectify_lr";
const char PerceptionImpl::rectifyRightLeft[11] = "rectify_rl";
const char PerceptionImpl::rectifyRightRight[11] = "rectify_rr";

uint32_t PerceptionImpl::updateJudgingInMs = 100;
uint32_t PerceptionImpl::imageUpdateSysMs[] = {0};

PerceptionImpl::PerceptionImageHandler PerceptionImpl::imageHandler = {NULL, NULL};
PerceptionImpl::PerceptionCamParamHandler PerceptionImpl::camParamHandler = {NULL, NULL};

T_RecvCmdItem s_bulkCmdList[] = {
    PROT_CMD_ITEM(0, 0, 0x24, 0x13, MASK_HOST_DEVICE_SET_ID, &PerceptionImpl::imageHandler,
                  PerceptionImpl::cameraImageHandler),
};

T_RecvCmdItem s_v1CmdList[] = {
    PROT_CMD_ITEM(0, 0, 0x24, 0x33, MASK_HOST_DEVICE_SET_ID, &PerceptionImpl::camParamHandler,
                  PerceptionImpl::cameraParamHandler),
};

PerceptionImpl::PerceptionImpl(Vehicle* vehiclePtr) : vehicle(vehiclePtr) {
  T_RecvCmdHandle recvCmdHandle1;
  T_RecvCmdHandle recvCmdHandle2;

  recvCmdHandle1.cmdList = s_bulkCmdList;
  recvCmdHandle1.cmdCount = sizeof(s_bulkCmdList) / sizeof(T_RecvCmdItem);
  recvCmdHandle1.protoType = PROTOCOL_USBMC;

  recvCmdHandle2.cmdList = s_v1CmdList;
  recvCmdHandle2.cmdCount = sizeof(s_v1CmdList) / sizeof(T_RecvCmdItem);
  recvCmdHandle2.protoType = PROTOCOL_V1;

  if(!vehicle->linker->registerCmdHandler(&recvCmdHandle1)) {
    DERROR("Register perception image callback failed!");
  }
  if(!vehicle->linker->registerCmdHandler(&recvCmdHandle2)) {
    DERROR("Register perception parameters callback failed!");
  }
}

PerceptionImpl::~PerceptionImpl()
{
}

vector<Perception::DirectionType> PerceptionImpl::getUpdatingDiretcion() {
  vector<Perception::DirectionType> v;
  v.clear();
  for (int i = 0; i < IMAGE_MAX_DIRECTION_NUM; i++) {
    uint32_t curMs = 0;
    OsdkOsal_GetTimeMs(&curMs);
    if ((curMs - imageUpdateSysMs[i]) < updateJudgingInMs
        || (curMs - imageUpdateSysMs[i]) > -updateJudgingInMs) {
      v.push_back((Perception::DirectionType) i);
    }
  }

  return v;
}

E_OsdkStat PerceptionImpl::cameraImageHandler(struct _CommandHandle *cmdHandle,
                                              const T_CmdInfo *cmdInfo,
                                              const uint8_t *cmdData,
                                              void *userData) {
  Perception::ImageInfoType *header;
  //DSTATUS("### got camera image");
  if (!userData) {
    DERROR("userData is a null way");
    return OSDK_STAT_ERR_PARAM;
  }

  if (cmdInfo && cmdData
      && (cmdInfo->dataLen >= sizeof(Perception::ImageInfoType))) {
    header = (Perception::ImageInfoType *) cmdData;
  } else {
    DERROR("invalid received data");
    return OSDK_STAT_SYS_ERR;
  }

  PerceptionImageHandler *handler = (PerceptionImageHandler *) userData;
  if (header->rawInfo.direction < IMAGE_MAX_DIRECTION_NUM)
    OsdkOsal_GetTimeMs(&imageUpdateSysMs[header->rawInfo.direction]);

  if (handler->cb)
    handler->cb(*header,
                (uint8_t *) (cmdData + sizeof(Perception::ImageInfoType)),
                cmdInfo->dataLen - sizeof(Perception::ImageInfoType),
                handler->userData);
  else {
//    DERROR("Callback is a null value");
  }
#if 0
  if(writePictureData(cmdData + IMAGE_INFO_LEN, cmdInfo->dataLen - IMAGE_INFO_LEN) != 0) {
     printf("write image failed!\n");
   }
#endif
  return OSDK_STAT_OK;
}

E_OsdkStat PerceptionImpl::cameraParamHandler(struct _CommandHandle *cmdHandle,
                                     const T_CmdInfo *cmdInfo, const uint8_t *cmdData, void *userData) {
  Perception::CamParamPacketType pack = {0};
  DSTATUS("### got camera params");
  if (!userData) {
    DERROR("userData is a null way");
    return OSDK_STAT_ERR_PARAM;
  }

  if (cmdInfo && cmdData
      && (cmdInfo->dataLen <= sizeof(Perception::CamParamPacketType))) {
    /*! cmdData length might be not enough */
    memcpy(&pack, cmdData, cmdInfo->dataLen);
  } else {
    DERROR("invalid received data");
    return OSDK_STAT_SYS_ERR;
  }

  PerceptionCamParamHandler *handler = (PerceptionCamParamHandler *) userData;

  if (handler->cb)
    handler->cb(pack, handler->userData);

  return OSDK_STAT_OK;
}

E_OsdkStat PerceptionImpl::subscribePerceptionImage(const char camChoice[11]) {
  vector<Perception::DirectionType> updatingDirection = getUpdatingDiretcion();
  if (updatingDirection.size()) {
    DERROR("Now the direction [(DirectionType)%d] images are updating.", updatingDirection[0]);
    DERROR("Only the SINGLE direction subscription is allowed in current OSDK version. ");
    DERROR("Please do unsubscription firstly before do new subscribing");
    return OSDK_STAT_NOT_READY;
  }
  T_CmdInfo info;
  T_CmdInfo ackInfo;
  uint8_t ackData[1024];
  E_OsdkStat result;
  uint16_t nameLen = 10;
  uint8_t data[12];
  memcpy(data, (uint8_t *)(&nameLen), 2);
  memcpy(&data[2], camChoice, 10);

  info.cmdSet = 0x24;
  info.cmdId = 0x11;
  info.dataLen = 12;
  info.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  info.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  info.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  info.encType = 0;
  info.sender = vehicle->linker->getLocalSenderId();
  info.receiver = OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_MONOCULAR, 4); //0x91;
  result = OsdkCommand_SendSync(OsdkCommand_GetInstance(), &info, data,
                                &ackInfo, ackData, 2000, 3);

  if(result != OSDK_STAT_OK) {
    if(result == OSDK_STAT_ERR_TIMEOUT) {
      printf("timeout, no ack receive!\n");
    }
    printf("command test failed!\n");
    return result;
  }

  if(ackData[0] == 0)
    return OSDK_STAT_OK;
  else
    return OSDK_STAT_SYS_ERR;
}

string PerceptionImpl::getSubscribeString(Perception::CamPositionType camChoice) {
  switch (camChoice) {
    case Perception::RECTIFY_DOWN_LEFT:return rectifyDownLeft;
    case Perception::RECTIFY_DOWN_RIGHT:return rectifyDownRight;
    case Perception::RECTIFY_FRONT_LEFT:return rectifyFrontLeft;
    case Perception::RECTIFY_FRONT_RIGHT:return rectifyFrontRight;
    case Perception::RECTIFY_REAR_LEFT:return rectifyRearLeft;
    case Perception::RECTIFY_REAR_RIGHT:return rectifyRearRight;
    case Perception::RECTIFY_UP_LEFT:return rectifyUpLeft;
    case Perception::RECTIFY_UP_RIGHT:return rectifyUpRight;
    case Perception::RECTIFY_LEFT_LEFT:return rectifyLeftLeft;
    case Perception::RECTIFY_LEFT_RIGHT:return rectifyLeftRight;
    case Perception::RECTIFY_RIGHT_LEFT:return rectifyRightLeft;
    case Perception::RECTIFY_RIGHT_RIGHT:return rectifyRightRight;
    default: return "";
  }
}

E_OsdkStat PerceptionImpl::unsubscribePerceptionImage(const char camChoice[11]) {
  T_CmdInfo info;
  T_CmdInfo ackInfo;
  uint8_t ackData[1024];
  E_OsdkStat result;
  uint16_t nameLen = 10;
  uint8_t data[12];
  memcpy(data, (uint8_t *)(&nameLen), 2);
  memcpy(&data[2], camChoice, 10);

  info.cmdSet = 0x24;
  info.cmdId = 0x12;
  info.dataLen = 12;
  info.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  info.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  info.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  info.encType = 0;
  info.sender = vehicle->linker->getLocalSenderId();
  info.receiver = OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_MONOCULAR, 4); //0x91;
  result = OsdkCommand_SendSync(OsdkCommand_GetInstance(), &info, data,
                                &ackInfo, ackData, 2000, 3);

  if(result != OSDK_STAT_OK) {
    if(result == OSDK_STAT_ERR_TIMEOUT) {
      printf("timeout, no ack receive!\n");
    }
    printf("command test failed!\n");
    return result;
  }

  if (ackData[0] == 0)
    return OSDK_STAT_OK;
  else
    return OSDK_STAT_SYS_ERR;
}

E_OsdkStat PerceptionImpl::subscribePerceptionImage(Perception::CamPositionType camChoice) {
  string choiceCmd = getSubscribeString(camChoice);
  if (choiceCmd.size())
    return subscribePerceptionImage(choiceCmd.c_str());
  else
    return OSDK_STAT_ERR_PARAM;
}

E_OsdkStat PerceptionImpl::unsubscribePerceptionImage(Perception::CamPositionType camChoice) {
  string choiceCmd = getSubscribeString(camChoice);
  if (choiceCmd.size())
    return unsubscribePerceptionImage(choiceCmd.c_str());
  else
    return OSDK_STAT_ERR_PARAM;
}

E_OsdkStat PerceptionImpl::subscribePerceptionImage(Perception::DirectionType directionChoice) {
  E_OsdkStat retLeft = OSDK_STAT_OK;
  E_OsdkStat retRight = OSDK_STAT_OK;
  switch (directionChoice) {
    case Perception::RECTIFY_DOWN:
      retLeft = subscribePerceptionImage(Perception::RECTIFY_DOWN_LEFT);
      retRight = subscribePerceptionImage(Perception::RECTIFY_DOWN_RIGHT);
      break;
    case Perception::RECTIFY_UP:
      retLeft = subscribePerceptionImage(Perception::RECTIFY_UP_LEFT);
      retRight = subscribePerceptionImage(Perception::RECTIFY_UP_RIGHT);
      break;
    case Perception::RECTIFY_FRONT:
      retLeft = subscribePerceptionImage(Perception::RECTIFY_FRONT_LEFT);
      retRight = subscribePerceptionImage(Perception::RECTIFY_FRONT_RIGHT);
      break;
    case Perception::RECTIFY_REAR:
      retLeft = subscribePerceptionImage(Perception::RECTIFY_REAR_LEFT);
      retRight = subscribePerceptionImage(Perception::RECTIFY_REAR_RIGHT);
      break;
    case Perception::RECTIFY_LEFT:
      retLeft = subscribePerceptionImage(Perception::RECTIFY_LEFT_LEFT);
      retRight = subscribePerceptionImage(Perception::RECTIFY_LEFT_RIGHT);
      break;
    case Perception::RECTIFY_RIGHT:
      retLeft = subscribePerceptionImage(Perception::RECTIFY_RIGHT_LEFT);
      retRight = subscribePerceptionImage(Perception::RECTIFY_RIGHT_RIGHT);
      break;
  }
  if (retLeft != OSDK_STAT_OK) return retLeft;
  else if (retRight != OSDK_STAT_OK) return retRight;
  else return OSDK_STAT_OK;
}


E_OsdkStat PerceptionImpl::unsubscribePerceptionImage(Perception::DirectionType directionChoice) {
  E_OsdkStat retLeft = OSDK_STAT_OK;
  E_OsdkStat retRight = OSDK_STAT_OK;
  switch (directionChoice) {
    case Perception::RECTIFY_DOWN:
      retLeft = unsubscribePerceptionImage(Perception::RECTIFY_DOWN_LEFT);
      retRight = unsubscribePerceptionImage(Perception::RECTIFY_DOWN_RIGHT);
      break;
    case Perception::RECTIFY_UP:
      retLeft = unsubscribePerceptionImage(Perception::RECTIFY_UP_LEFT);
      retRight = unsubscribePerceptionImage(Perception::RECTIFY_UP_RIGHT);
      break;
    case Perception::RECTIFY_FRONT:
      retLeft = unsubscribePerceptionImage(Perception::RECTIFY_FRONT_LEFT);
      retRight = unsubscribePerceptionImage(Perception::RECTIFY_FRONT_RIGHT);
      break;
    case Perception::RECTIFY_REAR:
      retLeft = unsubscribePerceptionImage(Perception::RECTIFY_REAR_LEFT);
      retRight = unsubscribePerceptionImage(Perception::RECTIFY_REAR_RIGHT);
      break;
    case Perception::RECTIFY_LEFT:
      retLeft = unsubscribePerceptionImage(Perception::RECTIFY_LEFT_LEFT);
      retRight = unsubscribePerceptionImage(Perception::RECTIFY_LEFT_RIGHT);
      break;
    case Perception::RECTIFY_RIGHT:
      retLeft = unsubscribePerceptionImage(Perception::RECTIFY_RIGHT_LEFT);
      retRight = unsubscribePerceptionImage(Perception::RECTIFY_RIGHT_RIGHT);
      break;
  }
  if (retLeft != OSDK_STAT_OK) return retLeft;
  else if (retRight != OSDK_STAT_OK) return retRight;
  else return OSDK_STAT_OK;
}

void PerceptionImpl::cancelAllSubsciptions() {
  auto updatingMsg = getUpdatingDiretcion();
  if (updatingMsg.size()) {
    for (auto dir : updatingMsg) {
      DSTATUS("Unsubscribing stereo camera images (DirectionType : %d)", dir);
      unsubscribePerceptionImage(dir);
    }
  }
}

E_OsdkStat PerceptionImpl::subscribeCameraParam() {
  T_CmdInfo info;
  T_CmdInfo ackInfo;
  uint8_t ackData[1024];
  E_OsdkStat result;
  uint8_t data = 0;

  info.cmdSet = 0x24;
  info.cmdId = 0x32;
  info.dataLen = 1;
  info.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  info.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  info.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  info.encType = 0;
  info.sender = vehicle->linker->getLocalSenderId();
  info.receiver = OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_BINOCULAR, 5);
  result = OsdkCommand_SendSync(OsdkCommand_GetInstance(), &info, &data,
                                &ackInfo, ackData, 2000, 3);

  if(result != OSDK_STAT_OK) {
    if(result == OSDK_STAT_ERR_TIMEOUT) {
      DERROR("timeout, no ack receive!\n");
    }
    DERROR("command test failed!\n");
  }

  info.receiver = OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_BINOCULAR, 4);
  result = OsdkCommand_SendSync(OsdkCommand_GetInstance(), &info, &data,
                                &ackInfo, ackData, 2000, 3);
  if(result != OSDK_STAT_OK) {
    if(result == OSDK_STAT_ERR_TIMEOUT) {
      DERROR("timeout, no ack receive!\n");
    }
    DERROR("command test failed!\n");
    return result;
  }

  if(ackData[0] == 0)
    return OSDK_STAT_OK;
  else
    return OSDK_STAT_SYS_ERR;
}
