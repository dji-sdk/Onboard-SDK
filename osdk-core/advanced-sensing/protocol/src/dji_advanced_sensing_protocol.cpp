/*
 * DJI Onboard SDK Advanced Sensing APIs
 *
 * Copyright (c) 2017-2018 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 */

#include "../inc/dji_advanced_sensing_protocol.hpp"

using namespace DJI;
using namespace DJI::OSDK;

namespace DJI {
namespace OSDK {

#pragma pack(1)
typedef struct VGADescription {
  uint32_t index;
  uint32_t time_stamp;
  uint32_t direction;
  uint32_t type;
  uint32_t width;
  uint32_t height;
  uint32_t format;
  uint32_t reserved[57];
} VGADescription;

typedef struct AdvancedSensingHeader {
  uint8_t header[2];
  uint8_t cmd_id;
  uint8_t check_sum;
  uint32_t length;
  uint32_t reserved;
} AdvancedSensingHeader;

typedef struct AdvancedSensingCmdContainer {
  AdvancedSensingHeader *header;
  void *data_payload;
  uint16_t data_size;
  bool hasCallback;
  int callbackID;
} AdvancedSensingCmdContainer;
#pragma pack()

} // namespace OSDK
} // namespace DJI

AdvancedSensingProtocol::AdvancedSensingProtocol()
  : stereoImgData(NULL)
  , stereoVGAImgData(NULL)
{
  //! Step 1: Initialize Hardware Driver
  this->deviceDriver = new LinuxUSBDevice();
  this->threadHandle = new PosixThreadManager();

  //! Step 1.2: Initialize the hardware driver
  this->deviceDriver->init();
  this->threadHandle->init();

  //! Step 2: Initialize the Protocol
  this->setHeaderLength(sizeof(AdvancedSensingHeader));

  this->setMaxRecvLength(USB_MAXRECV);

  init(NULL, NULL);
}

AdvancedSensingProtocol::~AdvancedSensingProtocol()
{
  if (this->stereoImgData)
    delete this->stereoImgData;

  if (this->stereoVGAImgData)
    delete this->stereoVGAImgData;

  if (this->p_filter)
    delete this->p_filter;
}

void
AdvancedSensingProtocol::init(HardDriver* Driver, MMU* mmuPtr, bool userCallbackThread)
{
  p_recvContainer       = new RecvContainer();

  stereoImgData         = new ACK::StereoImgData();
  stereoVGAImgData      = new ACK::StereoVGAImgData();

  p_filter              = new SDKFilter();
  p_filter->recvIndex   = 0;
  p_filter->reuseCount  = 0;
  p_filter->reuseIndex  = 0;
  p_filter->encode      = 0;
  p_filter->recvBuf     = new uint8_t[MAX_RECV_LEN];

  BUFFER_SIZE = 1024*600;

  buf = new uint8_t[BUFFER_SIZE];

  buf_read_pos = 0;
  read_len     = 0;

  reuse_buffer = false;

  is_large_data_protocol = true;
}

/******************** Send Pipeline **********************/

int
AdvancedSensingProtocol::send(void *cmd, void *data, uint16_t data_len)
{
  AdvancedSensingCmdContainer cmdContainer;

  //! Command
  cmdContainer.header  = (AdvancedSensingHeader*)cmd;

  //! Data
  cmdContainer.data_payload = data;
  cmdContainer.data_size    = data_len;

  return sendInterface((void*)&cmdContainer);
}

int
AdvancedSensingProtocol::sendInterface(void *in_cmd_container)
{
  int ret = 0;

  // @todo ignore the cmd session first, implement the minimal version first
  threadHandle->lockRecvContainer();

  AdvancedSensingCmdContainer *cmd_container = (AdvancedSensingCmdContainer*)(in_cmd_container);

  // @note AdvancedSensing protocol sends cmd in two steps:
  // 1. send a header
  ret = sendData((uint8_t*)(cmd_container->header));
  // 2. send the cmd payload
  ret = sendWithLen(cmd_container->data_payload, cmd_container->data_size);

  threadHandle->freeRecvContainer();

  return ret;
}

int
AdvancedSensingProtocol::sendData(uint8_t *buf)
{
  int ans;
  // @note AdvancedSensing protocol sends cmd in two steps:
  // 1. send a header
  // 2. send the cmd payload
  // this is step 1
  ans = this->deviceDriver->send(buf, sizeof(AdvancedSensingHeader));

  if (ans == -116)
    DSTATUS("Port send time out");
  else if (ans == -5)
    DERROR("Port closed");
  else if (ans == -1)
    DERROR("Send error");

  return (int)ans;
}

int
AdvancedSensingProtocol::sendWithLen(void *data, int data_len)
{
  size_t ret = 0;

  // @note AdvancedSensing protocol sends cmd in two steps:
  // 1. send a header
  // 2. send the cmd payload
  // this is step 2
  if( data != NULL && data_len > 0)
    ret = this->deviceDriver->send((uint8_t*)data, data_len);

  if (ret == -116)
    DSTATUS("Port time out");
  else if (ret == -5)
    DERROR("Port closed");

  return (int)ret;
}

int
AdvancedSensingProtocol::formatProtocol(uint8_t *dst,
                                 uint8_t cmd_id,
                                 uint8_t *data,
                                 uint32_t data_size)
{
  AdvancedSensingHeader *header = (AdvancedSensingHeader*)dst;
  header->header[0] = SOF1;
  header->header[1] = SOF2;
  header->cmd_id = cmd_id;
  header->check_sum = 0;
  header->length = data_size;
  header->reserved = 0;
  return 0;
}

/******************** Receive Pipeline **********************/
//! step 0 - 4 are in base class

//! step 5
bool
AdvancedSensingProtocol::checkStream()
{
  AdvancedSensingHeader* p_head = (AdvancedSensingHeader*)(p_filter->recvBuf);

  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = false;
  if (p_filter->recvIndex < sizeof(AdvancedSensingHeader))
  {
    // Continue receive data, nothing to do
    return false;
  }
  else if (p_filter->recvIndex == sizeof(AdvancedSensingHeader))
  {
    // recv a full-head
    isFrame = verifyHead();
    return false;
  }
  else if (p_filter->recvIndex == (p_head->length + sizeof(AdvancedSensingHeader)))
  {
    isFrame = verifyData();
  }
  return isFrame;
}

//! step 6
bool
AdvancedSensingProtocol::verifyHead()
{
  AdvancedSensingHeader* p_head = (AdvancedSensingHeader*)(p_filter->recvBuf);
  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = false;

  if ((p_head->header[0] == AdvancedSensingProtocol::SOF1) &&
      (p_head->header[1] == AdvancedSensingProtocol::SOF2))
  {
    ;
  }
  else
  {
    shiftDataStream();
  }
  return isFrame;
}

//! step 7
bool
AdvancedSensingProtocol::verifyData()
{
  AdvancedSensingHeader* p_head = (AdvancedSensingHeader*)(p_filter->recvBuf);

  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = false;

  if ((p_head->header[0] == AdvancedSensingProtocol::SOF1) &&
      (p_head->header[1] == AdvancedSensingProtocol::SOF2))
  {
    isFrame = callApp();
  }
  return isFrame;
}

//! step 8
bool
AdvancedSensingProtocol::callApp()
{

  bool isFrame = appHandler((void *) p_filter->recvBuf);
  prepareDataStream();


  return isFrame;
}

//! step 9
bool
AdvancedSensingProtocol::appHandler(void *protocolHeader)
{
//! @todo Filter replacement
#ifdef API_TRACE_DATA
  printFrame(serialDevice, protocolHeader, false);
#endif

  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = false;

  threadHandle->lockRecvContainer();

  uint8_t *data_buf = (uint8_t*)protocolHeader;

  AdvancedSensingHeader header;
  memcpy(&header, &data_buf[0], sizeof(AdvancedSensingHeader));

  if ( header.cmd_id == AdvancedSensingProtocol::PROCESS_IMG_CMD_ID )
  {
    //! this is complicated, plz refer to protocol documentation
    uint32_t img_desc[CAMERA_PAIR_NUM][IMAGE_TYPE_NUM];
    memcpy(&img_desc,
           &data_buf[0+sizeof(AdvancedSensingHeader)+header.length-CAMERA_PAIR_NUM*IMAGE_TYPE_NUM*sizeof(uint32_t)],
           CAMERA_PAIR_NUM*IMAGE_TYPE_NUM*sizeof(uint32_t));

    //! we are going to use a 8-byte uint64_t to represent a 200-byte long array
    //! 200-byte array uses 50 uint32_t, each of them is boolean
    stereoImgData->img_desc = 0;
    stereoImgData->num_imgs = 0;

    int mem_location_offset = sizeof(AdvancedSensingHeader);
    for (int pair_idx = 0; pair_idx < CAMERA_PAIR_NUM; ++pair_idx) {
      for (int dir_idx = 0; dir_idx < IMAGE_TYPE_NUM; ++dir_idx) {
        if (img_desc[pair_idx][dir_idx])
        {

          if (pair_idx == AdvancedSensingProtocol::FRONT){
            if (dir_idx == AdvancedSensingProtocol::LEFT)
              memcpy(stereoImgData->img_vec[stereoImgData->num_imgs].name,
                     "front_left\0", 12);
            if (dir_idx == AdvancedSensingProtocol::RIGHT)
              memcpy(stereoImgData->img_vec[stereoImgData->num_imgs].name,
                     "front_right\0", 12);
            if (dir_idx == AdvancedSensingProtocol::DISPARITY)
              memcpy(stereoImgData->img_vec[stereoImgData->num_imgs].name,
                     "front_depth\0", 12);
          }else if (pair_idx == AdvancedSensingProtocol::DOWN){
            if (dir_idx == AdvancedSensingProtocol::LEFT)
              memcpy(stereoImgData->img_vec[stereoImgData->num_imgs].name,
                     "down_back\0", 10);
            if (dir_idx == AdvancedSensingProtocol::RIGHT)
              memcpy(stereoImgData->img_vec[stereoImgData->num_imgs].name,
                     "down_front\0", 11);
          }

          memcpy(&stereoImgData->img_vec[stereoImgData->num_imgs++].image,
                 &data_buf[mem_location_offset], ACK::IMG_240P_SIZE);

          stereoImgData->img_desc |= 1 << (pair_idx*IMAGE_TYPE_NUM + dir_idx);

          mem_location_offset += ACK::IMG_240P_SIZE;
        }
      }
    }

    memcpy(&stereoImgData->frame_index, data_buf+mem_location_offset, sizeof(int));
    memcpy(&stereoImgData->time_stamp, data_buf+mem_location_offset+sizeof(int), sizeof(int));

    p_recvContainer->recvData.stereoImgData = stereoImgData;
    p_recvContainer->recvInfo.cmd_id = header.cmd_id;

    isFrame = true;
  }else if ( header.cmd_id == AdvancedSensingProtocol::PROCESS_VGA_CMD_ID )
  {

    VGADescription desc;
    memcpy(&desc, &data_buf[0+sizeof(AdvancedSensingHeader)+header.length-sizeof(VGADescription)], sizeof(VGADescription));

    stereoVGAImgData->direction = desc.direction;
    stereoVGAImgData->frame_index = desc.index;
    stereoVGAImgData->time_stamp = desc.time_stamp;

    memcpy(&stereoVGAImgData->img_vec[0], &data_buf[0+sizeof(AdvancedSensingHeader)],
           ACK::IMG_VGA_SIZE);
    memcpy(&stereoVGAImgData->img_vec[1], &data_buf[0+sizeof(AdvancedSensingHeader)+ACK::IMG_VGA_SIZE],
           ACK::IMG_VGA_SIZE);

    p_recvContainer->recvData.stereoVGAImgData = stereoVGAImgData;
    p_recvContainer->recvInfo.cmd_id = header.cmd_id;

    isFrame = true;
  }

  threadHandle->freeRecvContainer();

  return isFrame;
}

int
AdvancedSensingProtocol::crcHeadCheck(uint8_t* pMsg, size_t nLen)
{
  return 0;
}

int
AdvancedSensingProtocol::crcTailCheck(uint8_t* pMsg, size_t nLen)
{
  return 0;
}
