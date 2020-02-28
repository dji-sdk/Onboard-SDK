/*! @file dji_advanced_sensing_protocol.hpp
 *  @version 3.4.0
 *  @date Dec 2017
 *
 *
 *  @Copyright (c) 2016-17 DJI.
 *  NOTE THAT this file is part of the advanced sensing
 *  closed-source library. For licensing information,
 *  please visit https://developer.dji.com/policies/eula/
 * */

#ifndef ONBOARDSDK_DJI_ADVANCED_SENSING_PROTOCOL_H
#define ONBOARDSDK_DJI_ADVANCED_SENSING_PROTOCOL_H

#include "dji_hard_driver.hpp"
#include "dji_thread_manager.hpp"
#include "dji_protocol_base.hpp"
#include "linux_usb_device.hpp"
#include "dji_ack.hpp"

/*! Platform includes:
 *  This set of macros figures out which files to include based on your
 *  platform.
 */

//! handle array of characters
#include "posix_thread_manager.hpp"
#include <cstring>

namespace DJI
{
namespace OSDK
{

class AdvancedSensingProtocol : public ProtocolBase
{
public:
  enum CMDID
  {
    START_CMD_ID            = 0x09,
    SELECT_IMG_CMD_ID       = 0x00,
    PROCESS_IMG_CMD_ID      = 0x01,
    SELECT_VGA_IMG_CMD_ID   = 0x31,
    PROCESS_VGA_CMD_ID      = 0x2F,
  };

  enum VisionSensorDirection
  {
    DOWN  = 0x00,
    FRONT = 0x01,
    BACK  = 0x03,
    LEFT  = 0x00,
    RIGHT = 0x01,
    DISPARITY = 0x05,
  };

  enum FREQ
  {
    FREQ_10HZ = 0x01,
    FREQ_20HZ = 0x00,
  };

public:
  //! Constructor
  AdvancedSensingProtocol();

  //! Destructor
  ~AdvancedSensingProtocol();

  /************************** Init ******************************************/
public:
  void init(HardDriver* Driver, MMU* mmuPtr, bool userCallbackThread = false);

  /********************* Useful protocol related constants ******************/
  static const uint8_t SOF1 = 0x55;
  static const uint8_t SOF2 = 0xaa;
  // max 5 pairs of stereo camera (2*BW_imgs + 1*depth) 240p
  // plus header_len: 12, metadata: 8, img_desc: 200
  static const int     USB_MAXRECV  = 5*(2+1)*240*320 + 12 + 8 + 200;


  /*******************************Send Pipeline*****************************/
public:
  int formatProtocol(uint8_t *dst, uint8_t cmd_id,
                     uint8_t *data, uint32_t data_size);

  int send(void *cmd, void *data, uint16_t data_len);

private:
  int sendInterface(void *in_cmd_container);

  int sendData(uint8_t *buf);

  int sendWithLen(void *data, int data_len);

  /************************** Receive Pipeline ******************************/
private:
  //! step 0 - 4 are in base class

  //! step 5
  //! determine which part of the stream to verify
  bool checkStream();

  //! step 6
  //! verify cmd header
  bool verifyHead();

  //! step 7
  //! verify data payload
  bool verifyData();

  //! step 8
  //! Once checks are done, dispatch them to receive pipelines
  bool callApp();

  //! step 9
  //! A lot of ACK parsing logic
  bool appHandler(void *protocolHeader);

  /********************************** CRC **********************************/
private:
  int crcHeadCheck(uint8_t* pMsg, size_t nLen);

  int crcTailCheck(uint8_t* pMsg, size_t nLen);

/******************************* Member variables ************************/
private:
  ACK::StereoImgData      *stereoImgData;
  ACK::StereoVGAImgData   *stereoVGAImgData;

}; // class AdvancedSensingProtocol

} // namespace OSDK
} // namespace DJI

#endif //ONBOARDSDK_DJI_ADVANCED_SENSING_PROTOCOL_H
