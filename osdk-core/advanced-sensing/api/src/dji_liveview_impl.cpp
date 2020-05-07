/** @file dji_liveview_impl.cpp
 *  @version 4.0
 *  @date Jan 2020
 *
 *  @brief Camera liveview API code implement
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
#include "dji_liveview_impl.hpp"
#include "osdk_osal.h"

using namespace DJI;
using namespace DJI::OSDK;

#define IMAGE_FILE_PATH_LEN                     (64)

#define OSDK_CMDSET_LIVEVIEW                    (0x08)
#define OSDK_CMDID_SUBSCRIBE_LIVEVIEW_STATUS    (0x65)
#define OSDK_CMDID_PUSH_LIVEVIEW_STATUS         (0x66)
#define OSDK_CMDID_LIVEVIEW_HEARTBEAT           (0x67)
#define OSDK_CMDID_REQ_SDR_LIVEVIEW             (0x68)

#define OSDK_CAMERA_INDEX_LIVEVIEW              (0)
#define OSDK_GD610_INDEX_LIVEVIEW               (1)

#define OSDK_FPV_LIVEVIEW_CHANNEL               (81)
#define OSDK_MAIN_CAMERA_LIVEVIEW_CHANNEL       (82)
#define OSDK_VICE_CAMERA_LIVEVIEW_CHANNEL       (85)
#define OSDK_TOP_CAMERA_LIVEVIEW_CHANNEL        (90)

#define LIVEVIEW_TEMP_CMD_SET                   (0x65)
#define LIVEVIEW_FPV_CAM_TEMP_CMD_ID            (0x54)
#define LIVEVIEW_MAIN_CAM_TEMP_CMD_ID           (0x55)
#define LIVEVIEW_VICE_CAM_TEMP_CMD_ID           (0x56)
#define LIVEVIEW_TOP_CAM_TEMP_CMD_ID            (0x57)

std::map<LiveView::LiveViewCameraPosition, LiveViewImpl::H264CallbackHandler> LiveViewImpl::h264CbHandlerMap = {
        {LiveView::OSDK_CAMERA_POSITION_NO_1, {NULL, NULL}},
        {LiveView::OSDK_CAMERA_POSITION_NO_2, {NULL, NULL}},
        {LiveView::OSDK_CAMERA_POSITION_NO_3, {NULL, NULL}},
        {LiveView::OSDK_CAMERA_POSITION_FPV,  {NULL, NULL}},
    };

T_RecvCmdItem LiveViewImpl::bulkCmdList[] = {
    PROT_CMD_ITEM(0, 0, LIVEVIEW_TEMP_CMD_SET, LIVEVIEW_FPV_CAM_TEMP_CMD_ID,  MASK_HOST_DEVICE_SET_ID, (void *)&h264CbHandlerMap, RecordStreamHandler),
    PROT_CMD_ITEM(0, 0, LIVEVIEW_TEMP_CMD_SET, LIVEVIEW_MAIN_CAM_TEMP_CMD_ID, MASK_HOST_DEVICE_SET_ID, (void *)&h264CbHandlerMap, RecordStreamHandler),
    PROT_CMD_ITEM(0, 0, LIVEVIEW_TEMP_CMD_SET, LIVEVIEW_VICE_CAM_TEMP_CMD_ID, MASK_HOST_DEVICE_SET_ID, (void *)&h264CbHandlerMap, RecordStreamHandler),
    PROT_CMD_ITEM(0, 0, LIVEVIEW_TEMP_CMD_SET, LIVEVIEW_TOP_CAM_TEMP_CMD_ID,  MASK_HOST_DEVICE_SET_ID, (void *)&h264CbHandlerMap, RecordStreamHandler),
};

LiveViewImpl::LiveViewImpl(Vehicle* vehiclePtr) :
    vehicle(vehiclePtr)
{
  T_RecvCmdHandle recvCmdHandle;
  recvCmdHandle.cmdList = bulkCmdList;
  recvCmdHandle.cmdCount = sizeof(bulkCmdList) / sizeof(T_RecvCmdItem);
  recvCmdHandle.protoType = PROTOCOL_USBMC;

  if(!vehicle->linker->registerCmdHandler(&recvCmdHandle)) {
    DERROR("register h264 cmd callback handler failed, exiting.");
  }
}

LiveViewImpl::~LiveViewImpl()
{
}

E_OsdkStat LiveViewImpl::RecordStreamHandler(struct _CommandHandle *cmdHandle,
                                             const T_CmdInfo *cmdInfo,
                                             const uint8_t *cmdData,
                                             void *userData) {
  if ((!cmdInfo) || (!userData)) {
    DERROR("Recv Info is a null value");
    return OSDK_STAT_ERR;
  }

  std::map<LiveView::LiveViewCameraPosition, H264CallbackHandler> handlerMap =
      *(std::map<LiveView::LiveViewCameraPosition, H264CallbackHandler> *)userData;

  LiveView::LiveViewCameraPosition pos;
  switch (cmdInfo->cmdId) {
    case LIVEVIEW_FPV_CAM_TEMP_CMD_ID:
      pos = LiveView::OSDK_CAMERA_POSITION_FPV;
      break;
    case LIVEVIEW_MAIN_CAM_TEMP_CMD_ID:
      pos = LiveView::OSDK_CAMERA_POSITION_NO_1;
      break;
    case LIVEVIEW_VICE_CAM_TEMP_CMD_ID:
      pos = LiveView::OSDK_CAMERA_POSITION_NO_2;
      break;
    case LIVEVIEW_TOP_CAM_TEMP_CMD_ID:
      pos = LiveView::OSDK_CAMERA_POSITION_NO_3;
      break;
    default:
      return OSDK_STAT_ERR_OUT_OF_RANGE;
  }

  if((handlerMap.find(pos) != handlerMap.end()) && (handlerMap[pos].cb != NULL)) {
    handlerMap[pos].cb((uint8_t *)cmdData, cmdInfo->dataLen, handlerMap[pos].userData);
  } else {
    DERROR("Can't find valid cb in handlerMap");
  }

  return OSDK_STAT_OK;
}

E_OsdkStat LiveViewImpl::getCameraPushing(struct _CommandHandle *cmdHandle,
                                          const T_CmdInfo *cmdInfo,
                                          const uint8_t *cmdData,
                                          void *userData) {
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
  } else {
    return OSDK_STAT_SYS_ERR;
  }
}

LiveViewImpl::CameraListType LiveViewImpl::getCameraList() {
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

  OsdkOsal_TaskSleepMs(1000);

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
  DSTATUS("mounted\tcam0:%d cam2:%d cam3:%d", typeList.isMounted[0], typeList.isMounted[1], typeList.isMounted[2]);
  DSTATUS("type   \tcam0:%d cam2:%d cam3:%d", typeList.cameraType[0], typeList.cameraType[1], typeList.cameraType[2]);
  return typeList;
}

int LiveViewImpl::subscribeLiveViewData(E_OSDKCameraType type, LiveView::LiveViewCameraPosition pos) {
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
  memset(subCtx, 0, sizeof(T_LiveViewSubscribeItem));
  subCtx->header.role = 2;
  subCtx->header.chnNum = 1;
  subCtx->channel.cmdSize = sizeof(T_LiveViewChannelItem)
      + sizeof(T_LiveViewSingleSourceItem);
  subCtx->channel.mode = 0;
  subCtx->channel.action = 0; //open

  switch (pos) {
    case LiveView::OSDK_CAMERA_POSITION_FPV:
      subCtx->channel.channelId = OSDK_FPV_LIVEVIEW_CHANNEL - 1;
      break;
    case LiveView::OSDK_CAMERA_POSITION_NO_1:
      subCtx->channel.channelId = OSDK_MAIN_CAMERA_LIVEVIEW_CHANNEL - 1;
      break;
    case LiveView::OSDK_CAMERA_POSITION_NO_2:
      subCtx->channel.channelId = OSDK_VICE_CAMERA_LIVEVIEW_CHANNEL - 1;
      break;
    case LiveView::OSDK_CAMERA_POSITION_NO_3:
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
  subCtx->source.uuid.minor = ((type == OSDK_CAMERA_TYPE_PSDK) ? 0 : type);
  subCtx->source.uuid.reserved = 0;
  if ((type == OSDK_CAMERA_TYPE_GD610_DOUBLE_CAM)
      || (type == OSDK_CAMERA_TYPE_GD610_TIRPLE_CAM))
    subCtx->source.uuid.dataIdx = OSDK_GD610_INDEX_LIVEVIEW;
  else
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
  sendInfo.sender = vehicle->linker->getLocalSenderId();
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

int LiveViewImpl::unsubscribeLiveViewData(LiveView::LiveViewCameraPosition pos) {
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
    case LiveView::OSDK_CAMERA_POSITION_FPV:
      unsubCtx->channel.channelId = OSDK_FPV_LIVEVIEW_CHANNEL - 1;
      break;
    case LiveView::OSDK_CAMERA_POSITION_NO_1:
      unsubCtx->channel.channelId = OSDK_MAIN_CAMERA_LIVEVIEW_CHANNEL - 1;
      break;
    case LiveView::OSDK_CAMERA_POSITION_NO_2:
      unsubCtx->channel.channelId = OSDK_VICE_CAMERA_LIVEVIEW_CHANNEL - 1;
      break;
    case LiveView::OSDK_CAMERA_POSITION_NO_3:
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
  sendInfo.sender = vehicle->linker->getLocalSenderId();
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

void *LiveViewImpl::heartBeatTask(void *p) {
  Vehicle *vehicle = (Vehicle *) p;
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
  sendInfo.sender = vehicle->linker->getLocalSenderId();
  sendInfo.receiver = 0x08;

  while (1) {
    OsdkOsal_TaskSleepMs(1000);
    result = vehicle->linker->send(&sendInfo, &data);
    if (result != OSDK_STAT_OK) {
      DERROR("heart beat task send failed!\n");
    }
  }
}

E_OsdkStat LiveViewImpl::startHeartBeatTask() {
  return OsdkOsal_TaskCreate(&h264TaskHandle, heartBeatTask,
                             OSDK_TASK_STACK_SIZE_DEFAULT,
                             vehicle);
}

E_OsdkStat LiveViewImpl::stopHeartBeatTask() {
  return OsdkOsal_TaskDestroy(h264TaskHandle);
}

LiveView::LiveViewErrCode LiveViewImpl::startH264Stream(LiveView::LiveViewCameraPosition pos, H264Callback cb, void *userData) {
  E_OSDKCameraType targetCamType;
  if ((pos <= LiveView::OSDK_CAMERA_POSITION_NO_3) && (pos >= LiveView::OSDK_CAMERA_POSITION_NO_1)) {
    CameraListType cameraList = getCameraList();
    if (cameraList.isMounted[pos]) {
      DSTATUS("camera[%d] is mounted\n", pos);
      targetCamType = cameraList.cameraType[pos];
    } else {
      DERROR("camera[%d] is not mounted\n", pos);
      return LiveView::OSDK_LIVEVIEW_CAM_NOT_MOUNTED;
    }
  } else if (pos == LiveView::OSDK_CAMERA_POSITION_FPV){
    DSTATUS("Getting FPV Test");
    targetCamType = OSDK_CAMERA_TYPE_FPV;
  } else {
    DERROR("camera[%d] is not mounted\n", pos);
    return LiveView::OSDK_LIVEVIEW_CAM_NOT_MOUNTED;
  }

  if (h264CbHandlerMap.find(pos) != h264CbHandlerMap.end()) {
    h264CbHandlerMap[pos].cb = cb;
    h264CbHandlerMap[pos].userData = userData;
  }

  if(subscribeLiveViewData(targetCamType, pos) == -1) {
    //vehicle->linker->destroyLiveViewTask();
    return LiveView::OSDK_LIVEVIEW_SUBSCRIBE_FAIL;
  }

  if (startHeartBeatTask() != OSDK_STAT_OK) {
    return LiveView::OSDK_LIVEVIEW_HEART_BEAT_START_FAIL;
  }

  return LiveView::OSDK_LIVEVIEW_PASS;
}

LiveView::LiveViewErrCode LiveViewImpl::stopH264Stream(LiveView::LiveViewCameraPosition pos) {
  unsubscribeLiveViewData(pos);
  stopHeartBeatTask();
  return LiveView::OSDK_LIVEVIEW_PASS;
  //vehicle->linker->destroyLiveViewTask();
}
