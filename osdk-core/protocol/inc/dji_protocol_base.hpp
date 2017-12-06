/** @file dji_protocol_base.hpp
 *  @version 3.4
 *  @date Dec 2017
 *
 *  @brief
 *  Abstract protocol implementation for DJI OSDK
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

#ifndef ONBOARDSDK_INTERNAL_DJI_PROTOCOL_BASE_H
#define ONBOARDSDK_INTERNAL_DJI_PROTOCOL_BASE_H

#include "dji_ack.hpp"
#include "dji_aes.hpp"
#include "dji_hard_driver.hpp"
#include "dji_log.hpp"
#include "dji_thread_manager.hpp"
#include "dji_type.hpp"

#ifdef STM32
#include <string.h>
#else
#include <cstring>
#endif

namespace DJI
{
namespace OSDK
{

class Vehicle;

/*! @brief Dispatch info
 *  @details This struct has booleans that get populated in the protocol layer
 *           and help the dispatcher in the Vehicle layer decide what to do
 *           with the received packet.
 */
typedef struct DispatchInfo
{
  bool    isAck;
  bool    isCallback;
  uint8_t callbackID;
} DispatchInfo;

/*! @brief Received info
 *  @details This struct contains the ack or data struct return from
 *           the vehicle with sending info
 */
typedef struct RecvContainer
{
  DJI::OSDK::ACK::Entry     recvInfo;
  DJI::OSDK::ACK::TypeUnion recvData;
  DJI::OSDK::DispatchInfo   dispatchInfo;
} RecvContainer;

class ProtocolBase
{
public:
  ProtocolBase();
  virtual ~ProtocolBase();

  /************************** Init ******************************************/
public:
  virtual void init(HardDriver* Driver, MMU* mmuPtr,
                    bool userCallbackThread = false) = 0;

  /************************** Send Pipeline *********************************/
  // @todo is there any benefit to have an unified send function?
  // public:
  //  //! highest level interface
  //  virtual void send(cmdContainer *cmd_container);

protected:
  //! some session management here
  virtual int sendInterface(void* cmdContainer) = 0;

  //! call the device driver
  virtual int sendData(uint8_t* buf) = 0;

  /************************** Receive Pipeline ******************************/
public:
  //! step 0
  //! highest level of receive function
  virtual RecvContainer* receive();

protected:
  typedef struct SDKFilter
  {
    uint32_t reuseIndex;
    uint32_t reuseCount;
    uint32_t recvIndex;
    uint8_t* recvBuf;
    uint8_t  sdkKey[32]; // for encryption
    uint8_t  encode;
  } SDKFilter;

  //! step 1
  //! Lowest-level function interfaces with SerialDevice
  virtual bool readPoll();

  //! step 2
public:
  virtual bool byteHandler(const uint8_t in_data);

protected:
  //! step 3
  //! Integrity checks for incoming data.
  virtual bool streamHandler(uint8_t in_data);

  //! step 4
  //! store received data into a buffer with idx management
  virtual void storeData(uint8_t in_data);

  //! step 5
  //! determine which part of the stream to verify
  virtual bool checkStream() = 0;

  //! step 6
  //! verify cmd header
  virtual bool verifyHead() = 0;

  //! step 7
  //! verify data payload
  virtual bool verifyData() = 0;

  //! step 8
  //! Once checks are done, dispatch them to receive pipelines
  virtual bool callApp() = 0;

  //! step 9
  //! A lot of ACK parsing logic
  virtual bool appHandler(void* protocolHeader) = 0;

  //! helper function for buffer management
  void prepareDataStream();

  //! helper function for buffer management
  void shiftDataStream();

  //! helper function for buffer management
  void reuseDataStream();

  /********************************** CRC **********************************/
protected:
  virtual int crcHeadCheck(uint8_t* pMsg, size_t nLen) = 0;

  virtual int crcTailCheck(uint8_t* pMsg, size_t nLen) = 0;

  /*********************** Getters and setters ******************************/
public:
  //! Get serial device handler.
  HardDriver* getDriver() const;

  //! Get handler to thread data.
  ThreadAbstract* getThreadHandle() const;

  void setDriver(HardDriver* hardDriver_ptr);

  void setThreadHandle(ThreadAbstract* threadAbstract_ptr);

  //! Set header length in different protocols
  void setHeaderLength(uint8_t length);

  //! Set max receive size
  void setMaxRecvLength(int length);

  int getBufReadPos();

  int getReadLen();

  RecvContainer* getReceivedFrame();

  /******************************* Member variables ************************/
protected:
  //! Serial driver pointer
  HardDriver* deviceDriver;

  //! Thread data
  bool            stopCond;
  ThreadAbstract* threadHandle;

  //! Frame-related.
  uint16_t seq_num;

  //! Buffer management
  int            buf_read_pos;
  int            read_len;
  int            BUFFER_SIZE; // this should not be changed, init this in constructor
  uint8_t*       buf;
  uint8_t        HEADER_LEN;
  int            MAX_RECV_LEN;
  RecvContainer* p_recvContainer;
  SDKFilter*     p_filter;

  //! A flag to enable buffer reuse mechanism
  bool reuse_buffer;

  //! A flag for large data protocol to avoid checking byte by byte
  bool is_large_data_protocol;

}; // class ProtocolBase

} // OSDK

} // DJI

#endif // ONBOARDSDK_INTERNAL_DJI_PROTOCOL_BASE_H
