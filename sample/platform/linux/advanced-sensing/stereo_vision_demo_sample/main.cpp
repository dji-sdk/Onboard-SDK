/*! @file advanced-sensing/main.cpp
 *  @version 3.4
 *  @date Sep 15 2017
 *
 *  @brief
 *  Logging API usage in a Linux environment.
 *  Shows example usage of various advanced sensing APIs and controls.
 *
 *  @Copyright (c) 2017 DJI
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

#include "dji_linux_helpers.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

using namespace DJI::OSDK;

#define IMAGE_FILE_PATH                "./image"
#define IMAGE_FILE_PATH_LEN            (64)
#define IMAGE_INFO_LEN                 (sizeof(DUSS_MSG_OSDK_IMAGE_INFO_t))
#define IMAGE_MAX_DIRECTION_NUM        (6)

typedef enum {
    RECTIFY_DOWN_LEFT = 1,
    RECTIFY_DOWN_RIGHT = 2,
    RECTIFY_FRONT_LEFT = 3,
    RECTIFY_FRONT_RIGHT = 4,
    RECTIFY_REAR_LEFT = 5,
    RECTIFY_REAR_RIGHT = 6,
    RECTIFY_UP_LEFT = 21,
    RECTIFY_UP_RIGHT = 22,
    RECTIFY_LEFT_LEFT = 23,
    RECTIFY_LEFT_RIGHT = 24,
    RECTIFY_RIGHT_LEFT = 25,
    RECTIFY_RIGHT_RIGHT = 26
} RECTIFY_TYPE_ID_ENUM;

typedef enum {
    RECTIFY_DOWN = 0,
    RECTIFY_FRONT = 1,
    RECTIFY_REAR = 2,
    RECTIFY_UP = 3,
    RECTIFY_LEFT = 4,
    RECTIFY_RIGHT = 5
} RECTIFY_DIRECTION_ENUM;

typedef struct {
  uint32_t index;
  uint8_t direction;
  uint8_t bpp;
  uint32_t width;
  uint32_t height;
} __attribute__((packed)) DUSS_MSG_VISION_RAW_IMAGE_INFO_t;

typedef struct {
  DUSS_MSG_VISION_RAW_IMAGE_INFO_t rawInfo;
  uint16_t dataId;
  uint16_t sequence;
  uint32_t dataType;
  uint64_t timeStamp;
} __attribute__((packed)) DUSS_MSG_OSDK_IMAGE_INFO_t;

typedef struct {
  int8_t direction;
  float leftIntrinsics[9];
  float rightIntrinsic[9];
  float rotaionLeftInRight[9];
  float translationLeftInRight[3];
} __attribute__((packed)) DUSS_MSG_CAMERA_PARAM_t;

typedef struct {
  uint32_t timeStamp;//ms
  uint32_t directionNum;
  DUSS_MSG_CAMERA_PARAM_t cameraParam[IMAGE_MAX_DIRECTION_NUM];
} __attribute__((packed)) DUSS_MSG_CAMERA_PARAM_PACK_t;

static E_OsdkStat cameraImageHandler(struct _CommandHandle *cmdHandle,
                      const T_CmdInfo *cmdInfo, const uint8_t *cmdData, void *userData);
static E_OsdkStat cameraParamHandler(struct _CommandHandle *cmdHandle,
                      const T_CmdInfo *cmdInfo, const uint8_t *cmdData, void *userData);

static int writePictureData(const uint8_t *data, uint32_t len);

T_RecvCmdItem s_bulkCmdList[] = {
    PROT_CMD_ITEM(0, 0, 0x24, 0x13, MASK_HOST_DEVICE_SET_ID, NULL,
                  cameraImageHandler),
};

T_RecvCmdItem s_v1CmdList[] = {
    PROT_CMD_ITEM(0, 0, 0x24, 0x33, MASK_HOST_DEVICE_SET_ID, NULL,
                  cameraParamHandler),
};


/********************************************************************************/

E_OsdkStat cameraImageHandler(struct _CommandHandle *cmdHandle,
                      const T_CmdInfo *cmdInfo, const uint8_t *cmdData, void *userData) {
  DUSS_MSG_OSDK_IMAGE_INFO_t *header = (DUSS_MSG_OSDK_IMAGE_INFO_t *)cmdData;
  static uint32_t count = 0;
  count++;
  if(count % 1024 == 0) {
    printf("count: %u\n", count);
  }

  //dataType define see RECTIFY_TYPE_ID_ENUM

  if(header->rawInfo.width == 640 &&
     header->rawInfo.height == 480 &&
     (cmdInfo->dataLen - IMAGE_INFO_LEN) == 307200) {
     printf("dataType = %d, dataId = %d\n", header->dataType, header->dataId);
     printf("sequence = %d\n", header->sequence);
     printf("direction: %d, bpp: %d\n", header->rawInfo.direction, header->rawInfo.bpp);
     printf("width: %d, height: %d\n", header->rawInfo.width, header->rawInfo.height);
     printf("dataLen = %u\n", cmdInfo->dataLen - IMAGE_INFO_LEN);
     printf("timeStamp = %lu\n", header->timeStamp);
#if 0
     if(writePictureData(cmdData + IMAGE_INFO_LEN, cmdInfo->dataLen - IMAGE_INFO_LEN) != 0) {
       printf("write image failed!\n");
     }
#endif
     return OSDK_STAT_OK;
  } else {
     printf("data error! count = %u\n", count);
     return OSDK_STAT_ERR;
  }
}

static E_OsdkStat cameraParamHandler(struct _CommandHandle *cmdHandle,
                      const T_CmdInfo *cmdInfo, const uint8_t *cmdData, void *userData) {
  DUSS_MSG_CAMERA_PARAM_PACK_t *pack = (DUSS_MSG_CAMERA_PARAM_PACK_t *)cmdData;

  if(cmdInfo->dataLen < sizeof(DUSS_MSG_CAMERA_PARAM_t)) {
    printf("invalid dataLen!\n");
    return OSDK_STAT_ERR;
  }
  //direction define see
  for(int i = 0; i < pack->directionNum; i++) {
    printf("direction = %d\n", pack->cameraParam[i].direction);
    // fr and fl use direction 1
    //@TODO do what you want
  }
  return OSDK_STAT_OK;
}

int writePictureData(const uint8_t *data, uint32_t len) {
  DIR *dirp = NULL;
  FILE *fp = NULL;
  char fileName[IMAGE_FILE_PATH_LEN] = {0};
  size_t size = 0;
  static uint32_t index = 0;

  dirp = opendir(IMAGE_FILE_PATH);
  if(dirp == NULL && mkdir(IMAGE_FILE_PATH, S_IRWXU | S_IRGRP | S_IROTH | S_IXOTH)) {
    return -1;
  }
  closedir(dirp);
  
  snprintf(fileName, IMAGE_FILE_PATH_LEN, "%s/%d.raw", IMAGE_FILE_PATH, index);
  
  fp = fopen(fileName, "w+");
  if(fp == NULL) {
    return -1;
  }
  size = fwrite(data, 1, len, fp);
  if(size != len) {
    return -1;
  }
  
  if(fp) {
    fclose(fp);
  }
  index++;
  return 0;
}

/*****************************************************************************************************/

int subscribeLeftTopic(Vehicle *vehicle) {
  T_CmdInfo info;
  T_CmdInfo ackInfo;
  uint8_t ackData[1024];
  int result;
  uint16_t nameLen = 10;
  const uint8_t name[11] = "rectify_fl";
  uint8_t data[12];
  memcpy(data, (uint8_t *)(&nameLen), 2);
  memcpy(&data[2], name, 10);
  
  info.cmdSet = 0x24;
  info.cmdId = 0x11;
  info.dataLen = 12;
  info.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  info.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  info.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  info.encType = 0;
  info.sender = 0xca;
  info.receiver = 0x91;
  result = OsdkCommand_SendSync(OsdkCommand_GetInstance(), &info, data,
                                 &ackInfo, ackData, 2000, 3);

  if(result != OSDK_STAT_OK) {
    if(result == OSDK_STAT_ERR_TIMEOUT) {
      printf("timeout, no ack receive!\n");
    }
    printf("command test failed!\n");
    return -1;
  }

  printf("ack sender: %x receiver: %x, data: \n", ackInfo.sender, ackInfo.receiver);
  for(int i = 0; i < ackInfo.dataLen; i++) {
    printf("%x", ackData[i]);
  }
  printf("\nend\n");
  if(ackData[0] == 0 || ackData[0] == 3)
    return 0;
  else
    return -1;
}

int subscribeRightTopic(Vehicle *vehicle) {
  T_CmdInfo info;
  T_CmdInfo ackInfo;
  uint8_t ackData[1024];
  int result;
  uint16_t nameLen = 10;
  const uint8_t name[11] = "rectify_fr";
  uint8_t data[12];
  memcpy(data, (uint8_t *)(&nameLen), 2);
  memcpy(&data[2], name, 10);
  
  info.cmdSet = 0x24;
  info.cmdId = 0x11;
  info.dataLen = 12;
  info.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  info.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  info.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  info.encType = 0;
  info.sender = 0xca;
  info.receiver = 0x91;
  result = OsdkCommand_SendSync(OsdkCommand_GetInstance(), &info, data,
                                 &ackInfo, ackData, 2000, 3);

  if(result != OSDK_STAT_OK) {
    if(result == OSDK_STAT_ERR_TIMEOUT) {
      printf("timeout, no ack receive!\n");
    }
    printf("command test failed!\n");
    return -1;
  }

  printf("ack sender: %x receiver: %x, data: \n", ackInfo.sender, ackInfo.receiver);
  for(int i = 0; i < ackInfo.dataLen; i++) {
    printf("%x", ackData[i]);
  }
  printf("\nend\n");
  if(ackData[0] == 0 || ackData[0] == 3)
    return 0;
  else
    return -1;
}

int unsubscribeLeftTopic(Vehicle *vehicle) {
  T_CmdInfo info;
  T_CmdInfo ackInfo;
  uint8_t ackData[1024];
  int result;
  uint16_t nameLen = 10;
  const uint8_t name[11] = "rectify_fl";
  uint8_t data[12];
  memcpy(data, (uint8_t *)(&nameLen), 2);
  memcpy(&data[2], name, 10);

  info.cmdSet = 0x24;
  info.cmdId = 0x12;
  info.dataLen = 12;
  info.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  info.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  info.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  info.encType = 0;
  info.sender = 0xca;
  info.receiver = 0x91;
  result = OsdkCommand_SendSync(OsdkCommand_GetInstance(), &info, data,
                                 &ackInfo, ackData, 2000, 3);

  if(result != OSDK_STAT_OK) {
    if(result == OSDK_STAT_ERR_TIMEOUT) {
      printf("timeout, no ack receive!\n");
    }
    printf("command test failed!\n");
    return -1;
  }

  printf("ack sender: %x receiver: %x, data: \n", ackInfo.sender, ackInfo.receiver);
  for(int i = 0; i < ackInfo.dataLen; i++) {
    printf("%x", ackData[i]);
  }
  printf("\nend\n");
  if(ackData[0] == 0 || ackData[0] == 2)
    return 0;
  else
    return -1;
}

int unsubscribeRightTopic(Vehicle *vehicle) {
  T_CmdInfo info;
  T_CmdInfo ackInfo;
  uint8_t ackData[1024];
  int result;
  uint16_t nameLen = 10;
  const uint8_t name[11] = "rectify_fr";
  uint8_t data[12];
  memcpy(data, (uint8_t *)(&nameLen), 2);
  memcpy(&data[2], name, 10);

  info.cmdSet = 0x24;
  info.cmdId = 0x12;
  info.dataLen = 12;
  info.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  info.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  info.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  info.encType = 0;
  info.sender = 0xca;
  info.receiver = 0x91;
  result = OsdkCommand_SendSync(OsdkCommand_GetInstance(), &info, data,
                                 &ackInfo, ackData, 2000, 3);

  if(result != OSDK_STAT_OK) {
    if(result == OSDK_STAT_ERR_TIMEOUT) {
      printf("timeout, no ack receive!\n");
    }
    printf("command test failed!\n");
    return -1;
  }

  printf("ack sender: %x receiver: %x, data: \n", ackInfo.sender, ackInfo.receiver);
  for(int i = 0; i < ackInfo.dataLen; i++) {
    printf("%x", ackData[i]);
  }
  printf("\nend\n");
  if(ackData[0] == 0 || ackData[0] == 2)
    return 0;
  else
    return -1;
}

int subscribeCameraParam(Vehicle *vehicle) {
  T_CmdInfo info;
  T_CmdInfo ackInfo;
  uint8_t ackData[1024];
  int result;
  uint8_t data = 0;

  info.cmdSet = 0x24;
  info.cmdId = 0x32;
  info.dataLen = 1;
  info.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  info.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  info.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  info.encType = 0;
  info.sender = 0xca;
  info.receiver = 0x91;
  result = OsdkCommand_SendSync(OsdkCommand_GetInstance(), &info, &data,
                                 &ackInfo, ackData, 2000, 3);

  if(result != OSDK_STAT_OK) {
    if(result == OSDK_STAT_ERR_TIMEOUT) {
      printf("timeout, no ack receive!\n");
    }
    printf("command test failed!\n");
    return -1;
  }

  printf("ack sender: %x receiver: %x, data: \n", ackInfo.sender, ackInfo.receiver);
  for(int i = 0; i < ackInfo.dataLen; i++) {
    printf("%x", ackData[i]);
  }
  printf("\nend\n");
  if(ackData[0] == 0)
    return 0;
  else
    return -1;
}


int startAdvancedSensingTest(Vehicle *vehicle) {
  subscribeCameraParam(vehicle);
  vehicle->linker->createAdvancedSensingTask();
  if(subscribeLeftTopic(vehicle) == -1) {
    printf("subscribe topic failed\n");
    vehicle->linker->destroyAdvancedSensingTask();
    return -1;
  }
  if(subscribeRightTopic(vehicle) == -1) {
    printf("subscribe topic failed\n");
    unsubscribeLeftTopic(vehicle);
    vehicle->linker->destroyAdvancedSensingTask();
    return -1;
  }
}

int stopAdvencedSensingTest(Vehicle *vehicle) {
  unsubscribeLeftTopic(vehicle);
  unsubscribeRightTopic(vehicle);
  vehicle->linker->destroyAdvancedSensingTask();
}

int
main(int argc, char** argv)
{

  T_RecvCmdHandle recvCmdHandle1;
  T_RecvCmdHandle recvCmdHandle2;

  recvCmdHandle1.cmdList = s_bulkCmdList;
  recvCmdHandle1.cmdCount = sizeof(s_bulkCmdList) / sizeof(T_RecvCmdItem);
  recvCmdHandle1.protoType = PROTOCOL_USBMC;

  recvCmdHandle2.cmdList = s_v1CmdList;
  recvCmdHandle2.cmdCount = sizeof(s_v1CmdList) / sizeof(T_RecvCmdItem);
  recvCmdHandle2.protoType = PROTOCOL_V1;


  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle *vehicle = linuxEnvironment.getVehicle();

  if (vehicle == NULL) {
      std::cout << "Vehicle not initialized, exiting. \n";
      return -1;
  }
  if(!vehicle->linker->registerCmdHandler(&recvCmdHandle1)) {
      std::cout << "register cmd handler failed, exiting. \n";
      return -1;
  }
  if(!vehicle->linker->registerCmdHandler(&recvCmdHandle2)) {
      std::cout << "register cmd handler failed, exiting. \n";
      return -1;
  }

  DSTATUS("Logging is completely independent of DJI::OSDK::Vehicle.");
  DSTATUS("In this example, we don't instantiate a Vehicle at all.\n");
  char inputChar;

  while(1) {
	  // Display interactive prompt
    std::cout
      << "| Available commands:                                            |"
      << std::endl;
    std::cout
      << "| [a] start advanced sensing test                                |"
      << std::endl;
    std::cout
      << "| [b] stop advanced sensing test                                 |"
      << std::endl;
    std::cout
      << "| [q] quit                                                       |"
      << std::endl;

    std::cin >> inputChar;
    switch (inputChar)
    {
      case 'a': {
        startAdvancedSensingTest(vehicle);
      }
        break;
      case 'b': {
        stopAdvencedSensingTest(vehicle);
      }
        break;
      case 'q': {
        return 0;
      }
        break;
      default:
        break;
    }
  }

  return 0;
}
