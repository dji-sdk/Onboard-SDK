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
#include <sys/time.h>
#include <dirent.h>
#include "osdk_logger_internal.h"
#include "osdk_device_id.h"

#ifdef OPEN_CV_INSTALLED
#include "opencv2/opencv.hpp"
  #include "opencv2/highgui/highgui.hpp"
#endif

using namespace DJI::OSDK;
using namespace cv;
using namespace std;

void show_rgb(CameraRGBImage img, char* name)
{
  OSDK_LOG_INFO("####","Got image from:%s", name);
#ifdef OPEN_CV_INSTALLED
  Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width*3);
  cvtColor(mat, mat, COLOR_RGB2BGR);
  imshow(name,mat);
  cv::waitKey(1);
#endif
}

using namespace DJI::OSDK;

#define IMAGE_FILE_PATH_LEN                     (64)

#define OSDK_CMDSET_LIVEVIEW                    (0x08)
#define OSDK_CMDID_SUBSCRIBE_LIVEVIEW_STATUS    (0x65)
#define OSDK_CMDID_PUSH_LIVEVIEW_STATUS         (0x66)
#define OSDK_CMDID_LIVEVIEW_HEARTBEAT           (0x67)
#define OSDK_CMDID_REQ_SDR_LIVEVIEW             (0x68)

#define OSDK_CAMERA_INDEX_LIVEVIEW              (0)

#define OSDK_FPV_LIVEVIEW_CHANNEL               (81)
#define OSDK_MAIN_CAMERA_LIVEVIEW_CHANNEL       (82)
#define OSDK_VICE_CAMERA_LIVEVIEW_CHANNEL       (85)
#define OSDK_TOP_CAMERA_LIVEVIEW_CHANNEL        (90)



typedef enum {
  OSDK_CAMERA_TYPE_FC350 = 0,
  OSDK_CAMERA_TYPE_FC550 = 1,
  OSDK_CAMERA_TYPE_FC260 = 2,
  OSDK_CAMERA_TYPE_FC300S = 3,
  OSDK_CAMERA_TYPE_FC300X = 4,
  OSDK_CAMERA_TYPE_FC550RAW = 5,
  OSDK_CAMERA_TYPE_FC330 = 6,
  OSDK_CAMERA_TYPE_TAU640 = 7,   //XT_640
  OSDK_CAMERA_TYPE_TAU336 = 8,   //XT_336
  OSDK_CAMERA_TYPE_FC220 = 9,
  OSDK_CAMERA_TYPE_FC300XW = 10,
  OSDK_CAMERA_TYPE_CV600 = 11,  //3.5X
  OSDK_CAMERA_TYPE_FC65XX = 12,
  OSDK_CAMERA_TYPE_FC6310 = 13,
  OSDK_CAMERA_TYPE_FC6510 = 14,
  OSDK_CAMERA_TYPE_FC6520 = 15,
  OSDK_CAMERA_TYPE_FC6532 = 16,
  OSDK_CAMERA_TYPE_FC6540 = 17,
  OSDK_CAMERA_TYPE_FC220L = 18,
  OSDK_CAMERA_TYPE_FC1102 = 19,
  OSDK_CAMERA_TYPE_GD600 = 20,  //30X, Z30
  OSDK_CAMERA_TYPE_FC6310A = 21,
  OSDK_CAMERA_TYPE_FC300SE = 22,
  OSDK_CAMERA_TYPE_WM230 = 23,

  OSDK_CAMERA_TYPE_FC1705 = 26,  //XT2
  OSDK_CAMERA_TYPE_PSDK = 31,
  OSDK_CAMERA_TYPE_FPV = 39,  //Matrice FPV
  OSDK_CAMERA_TYPE_TP1810 = 41,  //XTS
  OSDK_CAMERA_TYPE_GD610_DOUBLE_CAM = 42,
  OSDK_CAMERA_TYPE_GD610_TIRPLE_CAM = 43, //IR
  OSDK_CAMERA_TYPE_UNKNOWN = 0xFF
} E_OSDKCameraType;

typedef enum {
  OSDK_CAMERA_POSITION_NO_1 = 0,
  OSDK_CAMERA_POSITION_NO_2 = 1,
  OSDK_CAMERA_POSITION_NO_3 = 2,
  OSDK_CAMERA_POSITION_FPV = 7
} E_OSDK_CameraPosition;

enum {
  UUID_MAJOR_TYPE_CAMERA = 0,
  UUID_MAJOR_TYPE_RADAR = 1,
  UUID_MAJOR_TYPE_PSDK = 2,
  UUID_MAJOR_TYPE_UNKNOW = 255
};

typedef struct {
  // 2:OSDK
  uint8_t role;
  // 0:subscribe 1:unsubscribe
  uint8_t action;
  // always 0
  uint8_t type;
} __attribute__((packed)) T_SubscribeData;

typedef struct {
  uint8_t role;
} __attribute__((packed)) T_HeartBeatData;

typedef struct {
  uint8_t role;
  uint8_t chnNum;
} __attribute__((packed)) T_LiveViewHeader;

typedef struct {
  uint16_t cmdSize;
  uint8_t mode;
  uint8_t action;
  uint8_t channelId;
  uint8_t priority;
} __attribute__((packed)) T_LiveViewMiniChannelItem;

typedef struct {
  uint16_t cmdSize;
  uint8_t mode;
  uint8_t action;
  uint8_t channelId;
  uint8_t priority;
  uint8_t fps;
  uint16_t width;
  uint16_t height;
  uint8_t codecStrategy;
  uint8_t codecFormat;
  uint8_t adaptiveResolutionEnable;
  uint8_t sourceNum;
  uint8_t size;
} __attribute__((packed)) T_LiveViewChannelItem;

typedef struct {
  uint8_t version;
  uint8_t major;
  uint8_t minor;
  uint8_t dataIdx : 3;
  uint8_t devicePos : 3;
  uint8_t reserved : 2;
} __attribute__((packed)) T_LiveViewUuid;

typedef struct {
  T_LiveViewUuid uuid;
  uint8_t cropEnable;
  float cropOffsetX;
  float cropOffsetY;
  float cropWidth;
  float cropHeight;
} __attribute__((packed)) T_LiveViewSingleSourceItem;

typedef struct {
  T_LiveViewUuid uuid;
  uint8_t cropEnable;
  float cropOffsetX;
  float cropOffsetY;
  float cropWidth;
  float cropHeight;
  uint8_t order;
  float blendingOffsetX;
  float blendingOffsetY;
  float blendingWidth;
  float blendingHeight;
} __attribute__((packed)) T_LiveViewMultiSourceItem;

typedef struct {
  T_LiveViewHeader header;
  T_LiveViewChannelItem channel;
  T_LiveViewSingleSourceItem source;
} __attribute__((packed)) T_LiveViewSubscribeItem;

typedef struct {
  T_LiveViewHeader header;
  T_LiveViewMiniChannelItem channel;
} __attribute__((packed)) T_LiveViewUnsubscribeItem;

static E_OsdkStat RecordStreamHandler(struct _CommandHandle *cmdHandle,
                      const T_CmdInfo *cmdInfo, const uint8_t *cmdData, void *userData);
static E_OsdkStat liveViewInfoHandler(struct _CommandHandle *cmdHandle,
                               const T_CmdInfo *cmdInfo, const uint8_t *cmdData, void *userData);
static int writeStreamData(const char *fileName, const uint8_t *data, uint32_t len);

static pthread_t taskHandle;

T_RecvCmdItem s_bulkCmdList[] = {
    PROT_CMD_ITEM(0, 0, 0x65, 0x54, MASK_HOST_DEVICE_SET_ID, (void *)"FPV.h264",
                  RecordStreamHandler),
    PROT_CMD_ITEM(0, 0, 0x65, 0x55, MASK_HOST_DEVICE_SET_ID, (void *)"MainCam.h264",
                  RecordStreamHandler),
    PROT_CMD_ITEM(0, 0, 0x65, 0x56, MASK_HOST_DEVICE_SET_ID, (void *)"ViceCam.h264",
                  RecordStreamHandler),
    PROT_CMD_ITEM(0, 0, 0x65, 0x57, MASK_HOST_DEVICE_SET_ID, (void *)"TopCam.h264",
                  RecordStreamHandler),
};

T_RecvCmdItem s_v1CmdList[] = {
    PROT_CMD_ITEM(0, 0, OSDK_CMDSET_LIVEVIEW, OSDK_CMDID_PUSH_LIVEVIEW_STATUS,
                   MASK_HOST_DEVICE_SET_ID, NULL, liveViewInfoHandler),
};

E_OsdkStat RecordStreamHandler(struct _CommandHandle *cmdHandle,
                    const T_CmdInfo *cmdInfo, const uint8_t *cmdData, void *userData) {
  const char* fileName = "loggingFile";
  if (userData) fileName = (const char *)userData;

  if(writeStreamData(fileName, cmdData, cmdInfo->dataLen) != 0) {
    printf("write data failed!\n");
  } else {
    OSDK_LOG_INFO("LiveView Writing", "writing data %d to file:%s\n", cmdInfo->dataLen, fileName);
  }

  return OSDK_STAT_OK;
}

E_OsdkStat liveViewInfoHandler(struct _CommandHandle *cmdHandle,
                      const T_CmdInfo *cmdInfo, const uint8_t *cmdData, void *userData) {
  //DSTATUS("liveview info len = %d!\n", cmdInfo->dataLen);
  if (cmdInfo->dataLen >= 6) {
    T_LiveViewUuid uuid = *(T_LiveViewUuid *)(cmdData + 2);
    //DSTATUS("uuid.version=%d, uuid.major=%d, uuid.minor=%d, uuid.dataIdx=%d, uuid.devicePos=%d\n", uuid.version, uuid.major, uuid.minor, uuid.dataIdx, uuid.devicePos);
  }

  //@TODO:parse liveview info
  return OSDK_STAT_OK;
}

int writeStreamData(const char *fileName, const uint8_t *data, uint32_t len) {
  FILE *fp = NULL;
  size_t size = 0;
  
  fp = fopen(fileName, "a+");
  if(fp == NULL) {
    printf("fopen failed!\n");
    return -1;
  }
  size = fwrite(data, 1, len, fp);
  if(size != len) {
    return -1;
  }
  
  fflush(fp);
  if(fp) {
    fclose(fp);
  }
  return 0;
}

typedef struct CameraListType
{
    E_OSDKCameraType cameraType[3];
    bool isMounted[3];
} CameraListType;

E_OsdkStat getCameraPushing(struct _CommandHandle *cmdHandle,
                            const T_CmdInfo *cmdInfo,
                            const uint8_t *cmdData, void *userData)
{
  if (cmdInfo) {
    /*OSDK_LOG_ERROR("test pushing", "0x80 puhsing sender=0x%02X receiver:0x%02X len=%d", cmdInfo->sender,
                   cmdInfo->receiver, cmdInfo->dataLen);*/
    if (userData && (cmdInfo->dataLen >= 34)) {
      CameraListType *camList = (CameraListType *) userData;
      switch (cmdInfo->sender) {
        case 0x01:
        case 0x21:
          camList->isMounted[0] = true;
          camList->cameraType[0] = (E_OSDKCameraType) cmdData[33];
          break;
        case 0x41:
        case 0x61:
          camList->isMounted[1] = true;
          camList->cameraType[1] = (E_OSDKCameraType) cmdData[33];
          break;
        case 0x81:
        case 0xA1:
          camList->isMounted[2] = true;
          camList->cameraType[2] = (E_OSDKCameraType) cmdData[33];
          break;
      }
    } else {
      DERROR("cmdInfo is a null value");
    }
    return OSDK_STAT_OK;
  }
}

CameraListType getCameraList(Vehicle *vehicle) {
  static T_RecvCmdHandle handle = {0};
  static T_RecvCmdItem item = {0};
  static CameraListType typeList;

  for (int i = 0; i < 3; i++) {
    typeList.isMounted[i] = false;
  }

  handle.protoType = PROTOCOL_V1;
  handle.cmdCount = 1;
  handle.cmdList = &item;
  item.cmdSet = 0x02;
  item.cmdId = 0x80;
  item.mask = MASK_HOST_XXXXXX_SET_ID;
  item.host = 0;
  item.device = 0;
  item.pFunc = getCameraPushing;
  item.userData = &typeList;

  bool registerRet = vehicle->linker->registerCmdHandler(&(handle));
  DSTATUS("register result of geting camera pushing : %d\n", registerRet);

  uint8_t reqStartData[] = {0x01, 0x00, 0x02, 0x80};
  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t ackData[1024];

  cmdInfo.cmdSet = 0x05;
  cmdInfo.cmdId = 0x0b;
  cmdInfo.dataLen = sizeof(reqStartData);
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.receiver =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_CENTER, 0);
  cmdInfo.sender = vehicle->linker->getLocalSenderId();
  E_OsdkStat linkAck =
      vehicle->linker->sendSync(&cmdInfo, (uint8_t *) reqStartData, &ackInfo, ackData,
                                1000 / 4, 4);
  DSTATUS("Request start pushing camera info ack = %d\n", linkAck);

  sleep(1);

  uint8_t reqEndData[] = {0x00, 0x00, 0x02, 0x80};
  cmdInfo.cmdSet = 0x05;
  cmdInfo.cmdId = 0x0b;
  cmdInfo.dataLen = sizeof(reqEndData);
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.receiver =
      OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_CENTER, 0);
  cmdInfo.sender = vehicle->linker->getLocalSenderId();
  linkAck =
      vehicle->linker->sendSync(&cmdInfo, (uint8_t *) reqEndData, &ackInfo, ackData,
                                1000 / 4, 4);
  DSTATUS("Request end pushing camera info ack = %d\n", linkAck);
  DSTATUS("mounted----------------->cam0:%d cam2:%d cam3:%d", typeList.isMounted[0], typeList.isMounted[1], typeList.isMounted[2]);
  DSTATUS("type-------------------->cam0:%d cam2:%d cam3:%d", typeList.cameraType[0], typeList.cameraType[1], typeList.cameraType[2]);
  return typeList;
}

int subscribeLiveViewInfo(Vehicle *vehicle) {
  T_CmdInfo sendInfo;
  T_CmdInfo ackInfo;
  T_SubscribeData data;
  uint8_t ackData[1024];
  int result;
  data.role = 2;
  data.action = 0;
  data.type = 0;
  
  sendInfo.cmdSet = OSDK_CMDSET_LIVEVIEW;
  sendInfo.cmdId = OSDK_CMDID_SUBSCRIBE_LIVEVIEW_STATUS;
  sendInfo.dataLen = sizeof(T_SubscribeData);
  sendInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  sendInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  sendInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  sendInfo.encType = 0;
  sendInfo.sender = 0xca;
  sendInfo.receiver = 0x08;
  result = vehicle->linker->sendSync(&sendInfo, (uint8_t *)&data, &ackInfo, ackData, 2000, 3);
  if(result != OSDK_STAT_OK) {
    if(result == OSDK_STAT_ERR_TIMEOUT) {
      printf("timeout, no ack receive!\n");
    }
    printf("command test failed!\n");
    return -1;
  }
  if(ackInfo.dataLen == 1 && ackData[0] == 0) {
    printf("subsrcibe info success!\n");
  } else {
    printf("subscribe info failed!\n");
    printf("ackData:");
    for(int i = 0; i < ackInfo.dataLen; i++) {
      printf("%x", ackData[i]);
    }
    printf("\nend\n");
    return -1;
  }
  return 0;
}

int unsubscribeLiveViewInfo(Vehicle *vehicle) {
  T_CmdInfo sendInfo;
  T_CmdInfo ackInfo;
  T_SubscribeData data;
  uint8_t ackData[1024];
  int result;
  data.role = 2;
  data.action = 1;
  data.type = 0;
  
  sendInfo.cmdSet = OSDK_CMDSET_LIVEVIEW;
  sendInfo.cmdId = OSDK_CMDID_SUBSCRIBE_LIVEVIEW_STATUS;
  sendInfo.dataLen = sizeof(T_SubscribeData);
  sendInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  sendInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  sendInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  sendInfo.encType = 0;
  sendInfo.sender = 0xca;
  sendInfo.receiver = 0x08;
  result = vehicle->linker->sendSync(&sendInfo, (uint8_t *)&data, &ackInfo, ackData, 2000, 3);
  if(result != OSDK_STAT_OK) {
    if(result == OSDK_STAT_ERR_TIMEOUT) {
      printf("timeout, no ack receive!\n");
    }
    printf("command test failed!\n");
    return -1;
  }
  if(ackInfo.dataLen == 1 && ackData[0] == 0) {
    printf("unsubsrcibe info success!\n");
  } else {
    printf("unsubscribe info failed!\n");
    printf("ackData:");
    for(int i = 0; i < ackInfo.dataLen; i++) {
      printf("%x", ackData[i]);
    }
    printf("\nend\n");
    return -1;
  }
  return 0;
}

int subscribeLiveViewData(Vehicle *vehicle, E_OSDKCameraType type, E_OSDK_CameraPosition pos) {
  T_CmdInfo sendInfo;
  T_CmdInfo ackInfo;
  uint8_t ackData[1024];
  int result;

  T_LiveViewSubscribeItem *subCtx;
  subCtx = (T_LiveViewSubscribeItem *)malloc(sizeof(T_LiveViewSubscribeItem));
  if(subCtx == NULL) {
    printf("malloc failed!\n");
    goto malloc_fail;
  }

  subCtx->header.role = 2;
  subCtx->header.chnNum = 1;
  subCtx->channel.cmdSize = sizeof(T_LiveViewChannelItem)
                            + sizeof(T_LiveViewSingleSourceItem);
  subCtx->channel.mode = 0;
  subCtx->channel.action = 0; //open

  switch (pos) {
    case OSDK_CAMERA_POSITION_FPV:
      subCtx->channel.channelId = OSDK_FPV_LIVEVIEW_CHANNEL - 1;
      break;
    case OSDK_CAMERA_POSITION_NO_1:
      subCtx->channel.channelId = OSDK_MAIN_CAMERA_LIVEVIEW_CHANNEL - 1;
      break;
    case OSDK_CAMERA_POSITION_NO_2:
      subCtx->channel.channelId = OSDK_VICE_CAMERA_LIVEVIEW_CHANNEL - 1;
      break;
    case OSDK_CAMERA_POSITION_NO_3:
      subCtx->channel.channelId = OSDK_TOP_CAMERA_LIVEVIEW_CHANNEL - 1;
      break;
  }

  subCtx->channel.priority = 0;
  subCtx->channel.fps = 30;
  subCtx->channel.width = 0;
  subCtx->channel.height = 0;
  subCtx->channel.codecStrategy = 0;
  subCtx->channel.codecFormat = 0;
  subCtx->channel.adaptiveResolutionEnable = 0;
  subCtx->channel.sourceNum = 1;
  subCtx->channel.size = sizeof(T_LiveViewSingleSourceItem);
  subCtx->source.uuid.version = 1;
  if(type == OSDK_CAMERA_TYPE_PSDK)   subCtx->source.uuid.major = UUID_MAJOR_TYPE_PSDK;
  else subCtx->source.uuid.major = UUID_MAJOR_TYPE_CAMERA;
  subCtx->source.uuid.minor = type; 
  subCtx->source.uuid.reserved = 0;
  subCtx->source.uuid.dataIdx = OSDK_CAMERA_INDEX_LIVEVIEW;
  subCtx->source.cropEnable = 0;
  subCtx->source.cropOffsetX = 0;
  subCtx->source.cropOffsetY = 0;
  subCtx->source.cropWidth = 0;
  subCtx->source.cropHeight = 0;
  subCtx->source.uuid.devicePos = pos;

  sendInfo.cmdSet = OSDK_CMDSET_LIVEVIEW;
  sendInfo.cmdId = OSDK_CMDID_REQ_SDR_LIVEVIEW;
  sendInfo.dataLen = sizeof(T_LiveViewSubscribeItem);
  sendInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  sendInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  sendInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  sendInfo.encType = 0;
  sendInfo.sender = 0xca;
  sendInfo.receiver = 0x08;
  result = vehicle->linker->sendSync(&sendInfo, (uint8_t *)subCtx, &ackInfo, ackData, 2000, 3);
  if(result != OSDK_STAT_OK) {
    if(result == OSDK_STAT_ERR_TIMEOUT) {
      printf("sendSync timeout, no ack receive!\n");
    }
    printf("sendSync failed!\n");
    goto send_fail;
  }
  if(ackInfo.dataLen == 1 && ackData[0] == 0) {
    printf("subsrcibe data success!\n");
  } else {
    printf("subscribe data failed!\n");
    printf("ackData:");
    for(int i = 0; i < ackInfo.dataLen; i++) {
      printf("%x", ackData[i]);
    }
    printf("\nend\n");
    goto subscribe_fail;
  }
  free(subCtx);
  return 0;

subscribe_fail:
send_fail:
  free(subCtx);
malloc_fail:
  return -1;
}

int unsubscribeLiveViewData(Vehicle *vehicle, E_OSDK_CameraPosition pos) {
T_CmdInfo sendInfo;
  T_CmdInfo ackInfo;
  uint8_t ackData[1024];
  int result;

  T_LiveViewUnsubscribeItem *unsubCtx;
  unsubCtx = (T_LiveViewUnsubscribeItem *)malloc(sizeof(T_LiveViewUnsubscribeItem));
  if(unsubCtx == NULL) {
    printf("malloc failed!\n");
    goto malloc_fail;
  }

  unsubCtx->header.role = 2;
  unsubCtx->header.chnNum = 1;
  unsubCtx->channel.cmdSize = sizeof(T_LiveViewMiniChannelItem);
  unsubCtx->channel.mode = 0;
  unsubCtx->channel.action = 1; //close

  switch (pos) {
    case OSDK_CAMERA_POSITION_FPV:
      unsubCtx->channel.channelId = OSDK_FPV_LIVEVIEW_CHANNEL - 1;
      break;
    case OSDK_CAMERA_POSITION_NO_1:
      unsubCtx->channel.channelId = OSDK_MAIN_CAMERA_LIVEVIEW_CHANNEL - 1;
      break;
    case OSDK_CAMERA_POSITION_NO_2:
      unsubCtx->channel.channelId = OSDK_VICE_CAMERA_LIVEVIEW_CHANNEL - 1;
      break;
    case OSDK_CAMERA_POSITION_NO_3:
      unsubCtx->channel.channelId = OSDK_TOP_CAMERA_LIVEVIEW_CHANNEL - 1;
      break;
  }
  unsubCtx->channel.priority = 0;

  sendInfo.cmdSet = OSDK_CMDSET_LIVEVIEW;
  sendInfo.cmdId = OSDK_CMDID_REQ_SDR_LIVEVIEW;
  sendInfo.dataLen = sizeof(T_LiveViewUnsubscribeItem);
  sendInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  sendInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  sendInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  sendInfo.encType = 0;
  sendInfo.sender = 0xca;
  sendInfo.receiver = 0x08;
  result = vehicle->linker->sendSync(&sendInfo, (uint8_t *)unsubCtx, &ackInfo, ackData, 2000, 3);
  if(result != OSDK_STAT_OK) {
    if(result == OSDK_STAT_ERR_TIMEOUT) {
      printf("sendSync timeout, no ack receive!\n");
    }
    printf("sendSync failed!\n");
    goto send_fail;
  }
  if(ackInfo.dataLen == 1 && ackData[0] == 0) {
    printf("unsubsrcibe data success!\n");
  } else {
    printf("unsubscribe data failed!\n");
    printf("ackData:");
    for(int i = 0; i < ackInfo.dataLen; i++) {
      printf("%x", ackData[i]);
    }
    printf("\nend\n");
    goto unsubscribe_fail;
  }
  free(unsubCtx);
  return 0;

unsubscribe_fail:
send_fail:
  free(unsubCtx);
malloc_fail:
  return -1;
}

void* heartBeatTask(void *p) {
  Vehicle *vehicle = (Vehicle *)p;
  struct timeval start;
  struct timeval end;
  uint32_t timeMs;
  T_CmdInfo sendInfo;
  uint8_t data = 2;
  int result;
  
  sendInfo.cmdSet = OSDK_CMDSET_LIVEVIEW;
  sendInfo.cmdId = OSDK_CMDID_LIVEVIEW_HEARTBEAT;
  sendInfo.dataLen = 1;
  sendInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  sendInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  sendInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  sendInfo.encType = 0;
  sendInfo.sender = 0xca;
  sendInfo.receiver = 0x08;
  
  gettimeofday(&start, NULL);
  while(1) {
    gettimeofday(&end, NULL);
    timeMs = (end.tv_sec*1000 + end.tv_usec/1000) - (start.tv_sec*1000 + start.tv_usec/1000);
    if(timeMs < 1000) {
      usleep(5000);
      continue;
    } else {
      result = vehicle->linker->send(&sendInfo, &data);
      if(result != true) {
        printf("heart beat task send failed!\n");
      }
      gettimeofday(&start, NULL);
    }
  }
}

int startHeartBeatTask(Vehicle *vehicle) {
  return pthread_create(&taskHandle, NULL, heartBeatTask, (void *)vehicle);
}

int stopHeartBeatTask() {
  return pthread_cancel(taskHandle);
}

int startLiveViewTest(Vehicle *vehicle, E_OSDK_CameraPosition pos) {
  E_OSDKCameraType targetCamType;
  if ((pos <= OSDK_CAMERA_POSITION_NO_3) && (pos >= OSDK_CAMERA_POSITION_NO_1)) {
    CameraListType cameraList = getCameraList(vehicle);
    if (cameraList.isMounted[pos]) {
      DSTATUS("camera[%d] is mounted\n", pos);
      targetCamType = cameraList.cameraType[pos];
    } else {
      DERROR("camera[%d] is not mounted\n", pos);
      return -1;
    }
  } else if (pos == OSDK_CAMERA_POSITION_FPV){
    DSTATUS("Getting FPV Test");
    targetCamType = OSDK_CAMERA_TYPE_FPV;
  } else {
    DERROR("camera[%d] is not mounted\n", pos);
    return -1;
  }

  if(subscribeLiveViewInfo(vehicle) == -1)
    return -1;
  static uint8_t liveviewTaskRunning = 0;
  if (liveviewTaskRunning == 0) {
    liveviewTaskRunning = 1;
    vehicle->linker->createLiveViewTask();
  }
  if(subscribeLiveViewData(vehicle, targetCamType, pos) == -1) {
     vehicle->linker->destroyLiveViewTask();
     return -1;
  }
  if(startHeartBeatTask(vehicle) != 0) {
     vehicle->linker->destroyLiveViewTask();
     return -1;
  }
}

int stopLiveViewTest(Vehicle *vehicle, E_OSDK_CameraPosition pos) {
  unsubscribeLiveViewInfo(vehicle);
  unsubscribeLiveViewData(vehicle, pos);
  stopHeartBeatTask();
  //vehicle->linker->destroyLiveViewTask();
}

int
main(int argc, char** argv)
{

  T_RecvCmdHandle recvCmdHandle1;

  recvCmdHandle1.cmdList = s_bulkCmdList;
  recvCmdHandle1.cmdCount = sizeof(s_bulkCmdList) / sizeof(T_RecvCmdItem);
  recvCmdHandle1.protoType = PROTOCOL_USBMC;

  T_RecvCmdHandle recvCmdHandle2;

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
      std::cout << "register cmd handler1 failed, exiting. \n";
      return -1;
  }
  if(!vehicle->linker->registerCmdHandler(&recvCmdHandle2)) {
      std::cout << "register cmd handler2 failed, exiting. \n";
      return -1;
  }

  // Display interactive prompt
  while(1) {
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] start fpv camera liveview test                             |"
    << std::endl;
  std::cout
    << "| [b] start main camera liveview test                            |"
    << std::endl;
  std::cout
    << "| [c] start vice camera liveview test                            |"
    << std::endl;
  std::cout
    << "| [d] start top camera liveview test                             |"
    << std::endl;
  std::cout
    << "| [q] quit                                                       |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;

  switch (inputChar) {
    case 'a':
      startLiveViewTest(vehicle, OSDK_CAMERA_POSITION_FPV);
      break;
    case 'b':
      startLiveViewTest(vehicle, OSDK_CAMERA_POSITION_NO_1);
      break;
    case 'c':
      startLiveViewTest(vehicle, OSDK_CAMERA_POSITION_NO_2);
      break;
    case 'd':
      startLiveViewTest(vehicle, OSDK_CAMERA_POSITION_NO_3);
      break;
      case 'q': {
        return 0;
      }
      break;
      default:
        break;
    }

    sleep(10);

    switch (inputChar) {
      case 'a':
        stopLiveViewTest(vehicle, OSDK_CAMERA_POSITION_FPV);
        break;
      case 'b':
        stopLiveViewTest(vehicle, OSDK_CAMERA_POSITION_NO_1);
        break;
      case 'c':
        stopLiveViewTest(vehicle, OSDK_CAMERA_POSITION_NO_2);
        break;
      case 'd':
        stopLiveViewTest(vehicle, OSDK_CAMERA_POSITION_NO_3);
        break;
      case 'q': {
        return 0;
      }
        break;
      default:
        break;
    }
    sleep(2);
  }
}

