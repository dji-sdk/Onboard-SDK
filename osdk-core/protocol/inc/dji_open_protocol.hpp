/** @file dji_open_protocol.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  OPEN Protocol implementation for DJI Onboard SDK library
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

#ifndef ONBOARDSDK_INTERNAL_DJI_PROTOCOLLAYER_H
#define ONBOARDSDK_INTERNAL_DJI_PROTOCOLLAYER_H

#include "dji_ack.hpp"
#include "dji_aes.hpp"
#include "dji_crc.hpp"
#include "dji_hard_driver.hpp"
#include "dji_log.hpp"
#include "dji_platform_manager.hpp"
#include "dji_protocol_base.hpp"
#include "dji_thread_manager.hpp"
#include "dji_type.hpp"

/*! Platform includes:
 *  This set of macros figures out which files to include based on your
 *  platform.
 */
#ifdef QT
#include "qt_serial_device.hpp"
#include "qt_thread.hpp"
#elif defined(__linux__)
//! handle array of characters
#include "linux_serial_device.hpp"
#include "posix_thread_manager.hpp"
#include <cstring>
#elif STM32
//! handle array of characters
#include <stdlib.h>
#include <string.h>
#endif

namespace DJI
{
namespace OSDK
{

/****************************Globals**************************************/

#define MSG_ENABLE_FLAG_LEN 2

//----------------------------------------------------------------------
// App layer function
//----------------------------------------------------------------------

typedef struct
{
  uint16_t sequence_number;
  uint8_t  session_id : 5;
  uint8_t  need_encrypt : 1;
  uint8_t  reserve : 2;
} req_id_t;

#define SET_CMD_SIZE (2u)

//----------------------------------------------------------------------
// Session Management
//----------------------------------------------------------------------

#define ACK_SESSION_IDLE 0
#define ACK_SESSION_PROCESS 1
#define ACK_SESSION_USING 2
#define CMD_SESSION_0 0
#define CMD_SESSION_1 1
#define CMD_SESSION_AUTO 32

#define POLL_TICK 20 // unit is ms

//----------------------------------------------------------------------
// Codec Management
//----------------------------------------------------------------------

// @todo replace this
#define _SDK_U32_SET(_addr, _val) (*((uint32_t*)(_addr)) = (_val))
#define _SDK_U16_SET(_addr, _val) (*((uint16_t*)(_addr)) = (_val))

class OpenProtocol : public ProtocolBase
{
public:
  //! Constructor
  OpenProtocol(PlatformManager* platformManager_ptr, const char* device,
               uint32_t baudRate);

  //! Destructor
  ~OpenProtocol();

  /************************** Init ******************************************/
public:
  void init(HardDriver* Driver, MMU* mmuPtr, bool userCallbackThread = false);

  /********************* Useful protocol related constants ******************/
  static const int     ACK_SIZE   = 10;
  static const uint8_t SOF        = 0xAA;
  static const int     CRCHead    = sizeof(uint16_t);
  static const int     CRCData    = sizeof(uint32_t);
  static const int     CRCHeadLen = sizeof(OpenHeader) - CRCHead;
  static const int     PackageMin = sizeof(OpenHeader) + CRCData;

  /*************************** Session Management **************************/
private:
  ACKSession* allocACK(uint16_t session_id, uint16_t size);

  //! high level setup
  void setup();

  //! init all values
  void setupSession();

  //! dispatch according to send info
  CMDSession* allocSession(uint16_t session_id, uint16_t size);

  void freeSession(CMDSession* session);

  void freeACK(ACKSession* session);

  /*******************************Send Pipeline*****************************/
public:
  void send(uint8_t session_mode, bool is_enc, const uint8_t cmd[], void* pdata,
            size_t len, int timeout = 0, int retry_time = 1,
            bool hasCallback = false, int callbackID = 0);

  void send(Command* parameter);

  void sendPoll();

private:
  int sendInterface(void* cmdContainer);

  int sendData(uint8_t* buf);

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
  bool appHandler(void* protocolHeader);

  //! For CMD-Frame data (push data) handling
  bool recvReqData(OpenHeader* protocolHeader);

  //! CMD receive
  uint8_t getCmdCode(OpenHeader* protocolHeader);

  uint8_t getCmdSet(OpenHeader* protocolHeader);

  /********************************** CRC **********************************/
private:
  int crcHeadCheck(uint8_t* pMsg, size_t nLen);

  int crcTailCheck(uint8_t* pMsg, size_t nLen);

  void calculateCRC(void* p_data);

  uint16_t crc16Update(uint16_t crc, uint8_t ch);

  uint32_t crc32Update(uint32_t crc, uint8_t ch);

  uint16_t crc16Calc(const uint8_t* pMsg, size_t nLen);

  uint32_t crc32Calc(const uint8_t* pMsg, size_t nLen);

  /****************************** Utility Functions ***********************/
private:
  uint16_t calculateLength(uint16_t size, uint16_t encrypt_flag);

  void transformTwoByte(const char* pstr, uint8_t* pdata);

  void setRawFrame(uint8_t* p_header);

  /****************************** Encryption ******************************/
private:
  uint16_t encrypt(uint8_t* pdest, const uint8_t* psrc, uint16_t w_len,
                   uint8_t is_ack, uint8_t is_enc, uint8_t session_id,
                   uint16_t seq_num);
  void encodeData(OpenHeader* p_head, ptr_aes256_codec codec_func);

  /*************************** Multithreading support **********************/
private:
  //! Thread sync for ACK
  ACK::TypeUnion allocateACK(OpenHeader* protocolHeader);

  void setACKFrameStatus(uint32_t usageFlag);

  /**********************************Fitlered******************************/
public:
  void setKey(const char* key);

  /*
   * Get raw frame before encoding
   */
  uint8_t* getRawFrame();

  /********************************Member variables*************************/
private:
  //! Memory management
  MMU* mmu;

  //! Session Management
  CMDSession CMDSessionTab[SESSION_TABLE_NUM];
  ACKSession ACKSessionTab[SESSION_TABLE_NUM - 1];

  //! Encode buffers
  uint8_t*  encodeSendData;
  uint8_t   encodeACK[ACK_SIZE];

  //! Frame-related.
  uint32_t ackFrameStatus;
  bool     broadcastFrameStatus;
  uint8_t* rawFrame;

}; // class OpenProtocol

} // namespace OSDK
} // namespace DJI

#endif // ONBOARDSDK_INTERNAL_DJI_PROTOCOLLAYER_H
