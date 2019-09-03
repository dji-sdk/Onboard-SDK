/** @file dji_open_protocol.cpp
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

#include "dji_open_protocol.hpp"
#include <dji_vehicle.hpp>

#ifdef STM32
#include <stdio.h>
#endif

using namespace DJI;
using namespace DJI::OSDK;

//! Constructor
OpenProtocol::OpenProtocol(PlatformManager* platformManager_ptr,
                           const char* device, uint32_t baudrate)
{
  //! Step 1: Initialize Hardware Driver
  this->deviceDriver = platformManager_ptr->addHardDriver(
    PlatformManager::SERIAL_DEVICE, device, baudrate);
  this->threadHandle = platformManager_ptr->addThreadHandle();

  //! Step 1.2: Initialize the hardware driver
  this->deviceDriver->init();
  this->threadHandle->init();

  this->setHeaderLength(sizeof(OpenHeader));

  this->setMaxRecvLength(BUFFER_SIZE);

  //! Step 2: Initialize the Protocol
  init(this->deviceDriver, this->deviceDriver->getMmu());
}

OpenProtocol::~OpenProtocol()
{
  delete[] p_filter->recvBuf;
  delete p_filter;
  delete[](buf);
  delete[](encodeSendData);
  delete(p_recvContainer);
}

/******************* Init ******************************/
void
OpenProtocol::init(HardDriver* sDevice, MMU* mmuPtr, bool userCallbackThread)
{

  deviceDriver = sDevice;

  p_recvContainer = new RecvContainer();

  seq_num              = 0;
  ackFrameStatus       = 11;
  broadcastFrameStatus = false;

  p_filter             = new SDKFilter();
  p_filter->recvIndex  = 0;
  p_filter->reuseCount = 0;
  p_filter->reuseIndex = 0;
  p_filter->encode     = 0;
  p_filter->recvBuf    = new uint8_t[MAX_RECV_LEN];

  buf             = new uint8_t[BUFFER_SIZE];
  encodeSendData  = new uint8_t[BUFFER_SIZE];

  mmu          = mmuPtr;
  buf_read_pos = 0;
  read_len     = 0;

  setup();
}

/******************* Session memory **************************/

void
OpenProtocol::setup()
{
  mmu->setupMMU();
  setupSession();
}

void
OpenProtocol::setupSession()
{
  uint32_t i;
  for (i = 0; i < SESSION_TABLE_NUM; i++)
  {
    CMDSessionTab[i].sessionID = i;
    CMDSessionTab[i].usageFlag = 0;
    CMDSessionTab[i].mmu       = (MMU_Tab*)NULL;
  }

  for (i = 0; i < (SESSION_TABLE_NUM - 1); i++)
  {
    ACKSessionTab[i].sessionID     = i + 1;
    ACKSessionTab[i].sessionStatus = ACK_SESSION_IDLE;
    ACKSessionTab[i].mmu           = (MMU_Tab*)NULL;
  }
}

CMDSession*
OpenProtocol::allocSession(uint16_t session_id, uint16_t size)
{
  uint32_t i;
  DDEBUG("Allocation size %d", size);
  MMU_Tab* memoryTab = NULL;

  if (session_id == 0 || session_id == 1)
  {
    if (this->CMDSessionTab[session_id].usageFlag == 0)
      i = session_id;
    else
    {
      /* session is busy */
      DERROR("session %d is busy\n", session_id);
      return NULL;
    }
  }
  else
  {
    for (i = 2; i < SESSION_TABLE_NUM; i++)
      if (CMDSessionTab[i].usageFlag == 0)
        break;
  }
  if (i < 32 && CMDSessionTab[i].usageFlag == 0)
  {
    CMDSessionTab[i].usageFlag = 1;
    memoryTab                  = mmu->allocMemory(size);
    if (memoryTab == NULL)
      CMDSessionTab[i].usageFlag = 0;
    else
    {
      CMDSessionTab[i].mmu = memoryTab;
      return &CMDSessionTab[i];
    }
  }
  return NULL;
}

void
OpenProtocol::freeSession(CMDSession* session)
{
  if (session->usageFlag == 1)
  {
    DDEBUG("session id %d\n", session->sessionID);
    mmu->freeMemory(session->mmu);
    session->usageFlag = 0;
  }
}

ACKSession*
OpenProtocol::allocACK(uint16_t session_id, uint16_t size)
{
  MMU_Tab* memoryTab = NULL;
  if (session_id > 0 && session_id < 32)
  {
    if (ACKSessionTab[session_id - 1].mmu)
      freeACK(&ACKSessionTab[session_id - 1]);
    memoryTab = mmu->allocMemory(size);
    if (memoryTab == NULL)
    {
      DERROR("there is not enough memory\n");
      return NULL;
    }
    else
    {
      ACKSessionTab[session_id - 1].mmu = memoryTab;
      return &ACKSessionTab[session_id - 1];
    }
  }
  DERROR("wrong Ack session ID: 0x%X\n", session_id);
  return NULL;
}

void
OpenProtocol::freeACK(ACKSession* session)
{
  mmu->freeMemory(session->mmu);
}

/******************** Send Pipeline **********************/

void
OpenProtocol::send(uint8_t session_mode, bool is_enc, const uint8_t cmd[],
                   void* pdata, size_t len, int timeout, int retry_time,
                   bool hasCallback, int callbackID)
{
  Command  cmdContainer;
  uint8_t* ptemp = (uint8_t*)encodeSendData;
  *ptemp++       = cmd[0];
  *ptemp++       = cmd[1];

  memcpy(encodeSendData + SET_CMD_SIZE, pdata, len);

  cmdContainer.sessionMode = session_mode;
  cmdContainer.length      = len + SET_CMD_SIZE;
  cmdContainer.buf         = encodeSendData;
  cmdContainer.cmd_set     = cmd[0]; // cmd set
  cmdContainer.cmd_id      = cmd[1]; // cmd id
  cmdContainer.retry       = retry_time;

  cmdContainer.timeout = timeout;
  cmdContainer.encrypt = is_enc ? 1 : 0;

  //! Callback
  cmdContainer.isCallback = hasCallback;
  cmdContainer.callbackID = callbackID;

  sendInterface((void*)&cmdContainer);
}

void
OpenProtocol::send(Command* cmdContainer)
{
  sendInterface((void*)cmdContainer);
}

int
OpenProtocol::sendInterface(void* cmd_container)
{
  uint16_t    ret          = 0;
  CMDSession* cmdSession   = (CMDSession*)NULL;
  Command*    cmdContainer = (Command*)cmd_container;
  if (cmdContainer->length > PRO_PURE_DATA_MAX_SIZE)
  {
    DERROR("ERROR,length=%lu is over-sized\n", cmdContainer->length);
    return -1;
  }
  /*! Switch on session to decide whether the command is requesting an ACK and
   * whether it is requesting
   *  guarantees on transmission
   */

  switch (cmdContainer->sessionMode)
  {
    case 0:
      //! No ACK required and no retries
      threadHandle->lockRecvContainer();
      cmdSession =
        allocSession(CMD_SESSION_0, calculateLength(cmdContainer->length,
                                                    cmdContainer->encrypt));

      if (cmdSession == (CMDSession*)NULL)
      {
        threadHandle->freeRecvContainer();
        DERROR("ERROR,there is not enough memory\n");
        return -1;
      }
      //! Encrypt the data being sent
      ret =
        encrypt(cmdSession->mmu->pmem, cmdContainer->buf, cmdContainer->length,
                0, cmdContainer->encrypt, cmdSession->sessionID, seq_num);
      if (ret == 0)
      {
        DERROR("encrypt ERROR\n");
        freeSession(cmdSession);
        threadHandle->freeRecvContainer();
        return -1;
      }

      DDEBUG("send data in session mode 0\n");

      //! Actually send the data
      sendData(cmdSession->mmu->pmem);
      seq_num++;
      freeSession(cmdSession);
      threadHandle->freeRecvContainer();
      break;

    case 1:
      //! ACK required; Session 1; will retry until failure
      threadHandle->lockRecvContainer();
      cmdSession =
        allocSession(CMD_SESSION_1, calculateLength(cmdContainer->length,
                                                    cmdContainer->encrypt));
      /*! @Detail If the sessions are full, clear the timeout session and try again
       *  @TODO This whole layer will be improved in the future. */
      if(!cmdSession) {
        clearTimeoutSession();
        cmdSession =
            allocSession(CMD_SESSION_1, calculateLength(cmdContainer->length,
                                                        cmdContainer->encrypt));
      }
      if (cmdSession == (CMDSession*)NULL)
      {
        threadHandle->freeRecvContainer();
        DERROR("ERROR,there is not enough memory\n");
        return -1;
      }
      if (seq_num == cmdSession->preSeqNum)
      {
        seq_num++;
      }
      ret =
        encrypt(cmdSession->mmu->pmem, cmdContainer->buf, cmdContainer->length,
                0, cmdContainer->encrypt, cmdSession->sessionID, seq_num);
      if (ret == 0)
      {
        DERROR("encrypt ERROR\n");
        freeSession(cmdSession);
        threadHandle->freeRecvContainer();
        return -1;
      }
      cmdSession->preSeqNum = seq_num++;

      //@todo replace with a bool
      cmdSession->isCallback = cmdContainer->isCallback;
      cmdSession->callbackID = cmdContainer->callbackID;
      cmdSession->timeout =
        (cmdContainer->timeout > POLL_TICK) ? cmdContainer->timeout : POLL_TICK;
      cmdSession->preTimestamp = deviceDriver->getTimeStamp();
      cmdSession->sent         = 1;
      cmdSession->retry        = 1;
      DDEBUG("sending session %d\n", cmdSession->sessionID);
      sendData(cmdSession->mmu->pmem);
      threadHandle->freeRecvContainer();
      break;

    case 2:
      //! ACK required, Sessions 2 - END; no guarantees and no retries.
      threadHandle->lockRecvContainer();
      cmdSession =
        allocSession(CMD_SESSION_AUTO, calculateLength(cmdContainer->length,
                                                       cmdContainer->encrypt));
      /*! @Detail If the sessions are full, clear the timeout session and try again
       *  @TODO This whole layer will be improved in the future. */
      if(!cmdSession) {
        clearTimeoutSession();
        cmdSession =
            allocSession(CMD_SESSION_AUTO, calculateLength(cmdContainer->length,
                                                        cmdContainer->encrypt));
      }
      if (cmdSession == (CMDSession*)NULL)
      {
        threadHandle->freeRecvContainer();
        DERROR("ERROR,there is not enough memory\n");
        return -1;
      }
      if (seq_num == cmdSession->preSeqNum)
      {
        seq_num++;
      }
      ret =
        encrypt(cmdSession->mmu->pmem, cmdContainer->buf, cmdContainer->length,
                0, cmdContainer->encrypt, cmdSession->sessionID, seq_num);

      if (ret == 0)
      {
        DERROR("encrypt ERROR");
        freeSession(cmdSession);
        threadHandle->freeRecvContainer();
        return -1;
      }

      // To use in ErrorCode manager
      cmdSession->cmd_set = cmdContainer->cmd_set;
      cmdSession->cmd_id  = cmdContainer->cmd_id;
      // Will carry information: obtain/release control
      cmdSession->buf = cmdContainer->buf;

      cmdSession->preSeqNum = seq_num++;
      //@todo replace with a bool
      cmdSession->isCallback = cmdContainer->isCallback;
      cmdSession->callbackID = cmdContainer->callbackID;
      cmdSession->timeout =
        (cmdContainer->timeout > POLL_TICK) ? cmdContainer->timeout : POLL_TICK;
      cmdSession->preTimestamp = deviceDriver->getTimeStamp();
      cmdSession->sent         = 1;
      cmdSession->retry        = cmdContainer->retry;
      DDEBUG("Sending session %d\n", cmdSession->sessionID);
      sendData(cmdSession->mmu->pmem);
      threadHandle->freeRecvContainer();
      break;
    default:
      DERROR("Unknown mode:%d\n", cmdContainer->sessionMode);
      break;
  }
  return 0;
}

int
OpenProtocol::sendData(uint8_t* buf)
{
  size_t      ans;
  OpenHeader* pHeader = (OpenHeader*)buf;

#ifdef API_TRACE_DATA
  printFrame(serialDevice, pHeader, true);
#endif

  //! Serial Device call: last link in the send pipeline
  ans = deviceDriver->send(buf, pHeader->length);
  if (ans == 0)
    DSTATUS("Port did not send");
  if (ans == (size_t)-1)
    DERROR("Port closed.");

  if (ans != pHeader->length)
  {
    DERROR("Open Protocol cmd send failed, send_len: %d packet_len: %d\n", ans,
           pHeader->length);
  }
  else
  {
    DDEBUG("Open Protocol cmd send success\n");
  }

  return (int)ans;
}

//! Session management for the send pipeline: Poll
void
OpenProtocol::sendPoll()
{
  uint8_t i;
  time_ms curTimestamp;
  for (i = 1; i < SESSION_TABLE_NUM; i++)
  {
    if (CMDSessionTab[i].usageFlag == 1)
    {
      curTimestamp = deviceDriver->getTimeStamp();
      if ((curTimestamp - CMDSessionTab[i].preTimestamp) >
          CMDSessionTab[i].timeout)
      {
        threadHandle->lockRecvContainer();
        if (CMDSessionTab[i].retry > 0)
        {
          if (CMDSessionTab[i].sent >= CMDSessionTab[i].retry)
          {
            DSTATUS("Sending timeout, Free session %d\n",
                    CMDSessionTab[i].sessionID);
            freeSession(&CMDSessionTab[i]);
          }
          else
          {
            DDEBUG("Retry session %d\n", CMDSessionTab[i].sessionID);
            sendData(CMDSessionTab[i].mmu->pmem);
            CMDSessionTab[i].preTimestamp = curTimestamp;
            CMDSessionTab[i].sent++;
          }
        }
        else
        {
          DDEBUG("Send once %d\n", i);
          sendData(CMDSessionTab[i].mmu->pmem);
          CMDSessionTab[i].preTimestamp = curTimestamp;
        }
        threadHandle->freeRecvContainer();
      }
      else
      {
        DDEBUG("Wait for timeout Session: %d \n", i);
      }
    }
  }
  //! @note Add auto resendpoll
}

/*! @note Clear all the total-timeout sessions in OpenProtocol. Fix the issue that
 * timeout more than 32 times than cannot send any packet in 2~32 sessions.
 * It will be called when the sessions are full, than all the timeout sessions will
 * be checkout. Whole this layer will be refactored in the future */
void
OpenProtocol::clearTimeoutSession()
{
  uint8_t i;
  time_ms curTimestamp;
  DSTATUS("[%s]Now clearing the timeout sessions", __FUNCTION__);
  for (i = 1; i < SESSION_TABLE_NUM; i++)
  {
    if (CMDSessionTab[i].usageFlag == 1)
    {
      curTimestamp = deviceDriver->getTimeStamp();
      if ((curTimestamp - CMDSessionTab[i].preTimestamp) >
          ((CMDSessionTab[i].retry - CMDSessionTab[i].sent) *
              CMDSessionTab[i].timeout))
      {
        DSTATUS("[%s]Sending total time timeout, Free session %d",
                __FUNCTION__, CMDSessionTab[i].sessionID);
        freeSession(&CMDSessionTab[i]);
      }
    }
  }
}

/******************** Receive Pipeline **********************/
//! step 0 - 4 are in base class

//! Step 5
bool
OpenProtocol::checkStream()
{
  OpenHeader* p_head = (OpenHeader*)(p_filter->recvBuf);
  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = false;
  if (p_filter->recvIndex < sizeof(OpenHeader))
  {
    // Continue receive data, nothing to do
    return false;
  }
  else if (p_filter->recvIndex == sizeof(OpenHeader))
  {
    // recv a full-head
    isFrame = verifyHead();
  }
  else if (p_filter->recvIndex == p_head->length)
  {
    isFrame = verifyData();
  }
  return isFrame;
}

//! Step 6
bool
OpenProtocol::verifyHead()
{
  OpenHeader* p_head = (OpenHeader*)(p_filter->recvBuf);

  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = false;

  if ((p_head->sof == OpenProtocol::SOF) && (p_head->version == 0) &&
      (p_head->length < OpenProtocol::MAX_RECV_LEN) &&
      (p_head->reserved0 == 0) && (p_head->reserved1 == 0) &&
      (crcHeadCheck((uint8_t*)p_head, sizeof(OpenHeader)) == 0))
  {
    // check if this head is a ack or simple package
    if (p_head->length == sizeof(OpenHeader))
    {
      isFrame = callApp();
    }
  }
  else
  {
    shiftDataStream();
  }
  return isFrame;
}

//! Step 7
bool
OpenProtocol::verifyData()
{
  OpenHeader* p_head = (OpenHeader*)(p_filter->recvBuf);

  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = false;

  if (crcTailCheck((uint8_t*)p_head, p_head->length) == 0)
  {
    isFrame = callApp();
  }
  else
  {
    //! @note data crc fail, re-use the data part
    reuseDataStream();
  }
  return isFrame;
}

//! Step 8
bool
OpenProtocol::callApp()
{
  // pass current data to handler
  OpenHeader* p_head = (OpenHeader*)p_filter->recvBuf;

  encodeData(p_head, aes256_decrypt_ecb);
  bool isFrame = appHandler((OpenHeader*)p_filter->recvBuf);
  prepareDataStream();

  return isFrame;
}

//! Step 9
bool
OpenProtocol::appHandler(void* protocolHeader)
{
//! @todo Filter replacement
#ifdef API_TRACE_DATA
  printFrame(serialDevice, openHeader, false);
#endif

  OpenHeader* p2protocolHeader;
  OpenHeader* openHeader = (OpenHeader*)protocolHeader;
  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = false;

  if (openHeader->isAck == 1)
  {
    //! Case 0: This is an ACK frame that came in.
    if (openHeader->sessionID > 1 && openHeader->sessionID < 32)
    {
      //! Session is valid
      if (CMDSessionTab[openHeader->sessionID].usageFlag == 1)
      {
        //! Session in use
        threadHandle->lockRecvContainer();
        p2protocolHeader =
          (OpenHeader*)CMDSessionTab[openHeader->sessionID].mmu->pmem;
        if (p2protocolHeader->sessionID == openHeader->sessionID &&
            p2protocolHeader->sequenceNumber == openHeader->sequenceNumber)
        {
          DDEBUG("Recv Session %d ACK\n", p2protocolHeader->sessionID);

          //! Create receive container for error code management
          p_recvContainer->dispatchInfo.isAck = true;
          p_recvContainer->recvInfo.cmd_set =
            CMDSessionTab[openHeader->sessionID].cmd_set;
          p_recvContainer->recvInfo.cmd_id =
            CMDSessionTab[openHeader->sessionID].cmd_id;
          p_recvContainer->recvData = allocateACK(openHeader);
          p_recvContainer->dispatchInfo.isCallback =
            CMDSessionTab[openHeader->sessionID].isCallback;
          p_recvContainer->dispatchInfo.callbackID =
            CMDSessionTab[openHeader->sessionID].callbackID;
          p_recvContainer->recvInfo.buf =
            CMDSessionTab[openHeader->sessionID].buf;
          p_recvContainer->recvInfo.seqNumber = openHeader->sequenceNumber;
          p_recvContainer->recvInfo.len       = openHeader->length;
          //! Set bool
          isFrame = true;

          //! Finish the session
          freeSession(&CMDSessionTab[openHeader->sessionID]);
          threadHandle->freeRecvContainer();
          /**
           * Set end of ACK frame
           * @todo Implement proper notification mechanism
           */
          setACKFrameStatus((&CMDSessionTab[openHeader->sessionID])->usageFlag);
        }
        else
        {
          threadHandle->freeRecvContainer();
        }
      }
    }
  }
  else
  {
    //! Not an ACK frame
    switch (openHeader->sessionID)
    {
      case 0:
        isFrame = recvReqData(openHeader);
        break;
      case 1:
      //! @todo unnecessary ack in case 1. Maybe add code later
      //! @todo check algorithm,
      //! @attention here real have a bug about self-looping issue.
      //! @bug not affect OSDK currerently. 2017-1-18
      default: //! @note session id is 2
        DSTATUS("ACK %d", openHeader->sessionID);

        if (ACKSessionTab[openHeader->sessionID - 1].sessionStatus ==
            ACK_SESSION_PROCESS)
        {
          DDEBUG("This session is waiting for App ACK:"
                 "session id=%d,seq_num=%d\n",
                 openHeader->sessionID, openHeader->sequenceNumber);
        }
        else if (ACKSessionTab[openHeader->sessionID - 1].sessionStatus ==
                 ACK_SESSION_IDLE)
        {
          if (openHeader->sessionID > 1)
            ACKSessionTab[openHeader->sessionID - 1].sessionStatus =
              ACK_SESSION_PROCESS;
          isFrame = recvReqData(openHeader);
        }
        else if (ACKSessionTab[openHeader->sessionID - 1].sessionStatus ==
                 ACK_SESSION_USING)
        {
          threadHandle->lockRecvContainer();
          p2protocolHeader =
            (OpenHeader*)ACKSessionTab[openHeader->sessionID - 1].mmu->pmem;
          if (p2protocolHeader->sequenceNumber == openHeader->sequenceNumber)
          {
            DDEBUG("Repeat ACK to remote,session "
                   "id=%d,seq_num=%d\n",
                   openHeader->sessionID, openHeader->sequenceNumber);
            sendData(ACKSessionTab[openHeader->sessionID - 1].mmu->pmem);
            threadHandle->freeRecvContainer();
          }
          else
          {
            DDEBUG("Same session,but new seq_num pkg,session id=%d,"
                   "pre seq_num=%d,cur seq_num=%d\n",
                   openHeader->sessionID, p2protocolHeader->sequenceNumber,
                   openHeader->sequenceNumber);
            ACKSessionTab[openHeader->sessionID - 1].sessionStatus =
              ACK_SESSION_PROCESS;
            threadHandle->freeRecvContainer();
            isFrame = recvReqData(openHeader);
          }
        }
        break;
    }
  }
  return isFrame;
}

ACK::TypeUnion
OpenProtocol::allocateACK(OpenHeader* protocolHeader)
{

  ACK::TypeUnion recvData;

  if (protocolHeader->length <= MAX_ACK_SIZE)
  {
    memcpy(recvData.raw_ack_array,
           ((uint8_t*)protocolHeader) + sizeof(OpenHeader),
           (protocolHeader->length - OpenProtocol::PackageMin));
  }
  else
  {
    //! @note throw not supported in STM32
    // throw std::runtime_error("Unknown ACK");
  }

  return recvData;
}

void
OpenProtocol::setACKFrameStatus(uint32_t usageFlag)
{
  ackFrameStatus = usageFlag;
}

/***************** Receive CMD functions *******************/

uint8_t
OpenProtocol::getCmdSet(OpenHeader* protocolHeader)
{
  uint8_t* ptemp = ((uint8_t*)protocolHeader) + sizeof(OpenHeader);
  return *ptemp;
}

uint8_t
OpenProtocol::getCmdCode(OpenHeader* protocolHeader)
{
  uint8_t* ptemp = ((uint8_t*)protocolHeader) + sizeof(OpenHeader);
  ptemp++;
  return *ptemp;
}

//! Step 10: In case we received a CMD frame and not an ACK frame
bool
OpenProtocol::recvReqData(OpenHeader* protocolHeader)
{
  uint8_t buf[100] = { 0, 0 };

  //@todo: Please monitor lengths to see whether we need to change the max size
  // of RecvContainer.recvData
  p_recvContainer->dispatchInfo.isAck = false;
  uint8_t* payload = (uint8_t*)protocolHeader + sizeof(OpenHeader) + 2;
  p_recvContainer->recvInfo.cmd_set = getCmdSet(protocolHeader);
  p_recvContainer->recvInfo.cmd_id  = getCmdCode(protocolHeader);
  p_recvContainer->recvInfo.len     = protocolHeader->length;
  //@todo: Please monitor to make sure the length is correct
  memcpy(p_recvContainer->recvData.raw_ack_array, payload,
         (protocolHeader->length - (OpenProtocol::PackageMin + 2)));

  p_recvContainer->dispatchInfo.isCallback = false;
  p_recvContainer->dispatchInfo.callbackID = 0;

  //! isFrame = true
  return true;
}

/******************* Utility Functions *********************/

uint16_t
OpenProtocol::calculateLength(uint16_t size, uint16_t encrypt_flag)
{
  uint16_t len;
  if (encrypt_flag)
    len = size + sizeof(OpenHeader) + 4 + (16 - size % 16);
  else
    len = size + sizeof(OpenHeader) + 4;
  return len;
}

void
OpenProtocol::transformTwoByte(const char* pstr, uint8_t* pdata)
{
  int      i;
  char     temp_area[3];
  uint32_t temp8;
  temp_area[0] = temp_area[1] = temp_area[2] = 0;

  for (i = 0; i < 32; i++)
  {
    temp_area[0] = pstr[0];
    temp_area[1] = pstr[1];
    sscanf(temp_area, "%x", &temp8);
    pdata[i] = temp8;
    pstr += 2;
  }
}

/******************* CRC Calculationns *********************/

void
OpenProtocol::calculateCRC(void* p_data)
{
  OpenHeader* p_head = (OpenHeader*)p_data;
  uint8_t*    p_byte = (uint8_t*)p_data;
  uint32_t    index_of_crc32;

  if (p_head->sof != OpenProtocol::SOF)
    return;
  if (p_head->version != 0)
    return;
  if (p_head->length > OpenProtocol::MAX_RECV_LEN)
    return;
  if (p_head->length > sizeof(OpenHeader) &&
      p_head->length < OpenProtocol::PackageMin)
    return;

  p_head->crc = crc16Calc(p_byte, OpenProtocol::CRCHeadLen);

  if (p_head->length >= OpenProtocol::PackageMin)
  {
    index_of_crc32 = p_head->length - OpenProtocol::CRCData;
    _SDK_U32_SET(p_byte + index_of_crc32, crc32Calc(p_byte, index_of_crc32));
  }
}

uint16_t
OpenProtocol::crc16Update(uint16_t crc, uint8_t ch)
{
  uint16_t tmp;
  uint16_t msg;

  msg = 0x00ff & static_cast<uint16_t>(ch);
  tmp = crc ^ msg;
  crc = (crc >> 8) ^ crc_tab16[tmp & 0xff];

  return crc;
}

uint32_t
OpenProtocol::crc32Update(uint32_t crc, uint8_t ch)
{
  uint32_t tmp;
  uint32_t msg;

  msg = 0x000000ffL & static_cast<uint32_t>(ch);
  tmp = crc ^ msg;
  crc = (crc >> 8) ^ crc_tab32[tmp & 0xff];
  return crc;
}

uint16_t
OpenProtocol::crc16Calc(const uint8_t* pMsg, size_t nLen)
{
  size_t   i;
  uint16_t wCRC = CRC_INIT;

  for (i = 0; i < nLen; i++)
  {
    wCRC = crc16Update(wCRC, pMsg[i]);
  }

  return wCRC;
}

uint32_t
OpenProtocol::crc32Calc(const uint8_t* pMsg, size_t nLen)
{
  size_t   i;
  uint32_t wCRC = CRC_INIT;

  for (i = 0; i < nLen; i++)
  {
    wCRC = crc32Update(wCRC, pMsg[i]);
  }

  return wCRC;
}

/******************* Encryption *********************/

void
OpenProtocol::encodeData(OpenHeader* p_head, ptr_aes256_codec codec_func)
{
  aes256_context ctx;
  uint32_t       buf_i;
  uint32_t       loop_blk;
  uint32_t       data_len;
  uint32_t       data_idx;
  uint8_t*       data_ptr;

  if (p_head->enc == 0)
    return;
  if (p_head->length <= OpenProtocol::PackageMin)
    return;

  data_ptr = (uint8_t*)p_head + sizeof(OpenHeader);
  data_len = p_head->length - OpenProtocol::PackageMin;

  loop_blk = data_len / 16;
  data_idx = 0;

  aes256_init(&ctx, p_filter->sdkKey);
  for (buf_i = 0; buf_i < loop_blk; buf_i++)
  {
    codec_func(&ctx, data_ptr + data_idx);
    data_idx += 16;
  }
  aes256_done(&ctx);

  if (codec_func == aes256_decrypt_ecb)
    p_head->length = p_head->length - p_head->padding; // minus padding length;

  if(data_len == 32)
  {
    setRawFrame(data_ptr);
  }
}

uint16_t
OpenProtocol::encrypt(uint8_t* pdest, const uint8_t* psrc, uint16_t w_len,
                      uint8_t is_ack, uint8_t is_enc, uint8_t session_id,
                      uint16_t seq_num)
{
  uint16_t data_len;

  OpenHeader* p_head = (OpenHeader*)pdest;

  if (w_len > 1024)
    return 0;

  if (p_filter->encode == 0 && is_enc)
  {
    DERROR("Can not send encode data, Please activate your device to get an "
           "available key.\n");
    return 0;
  }
  if (w_len == 0 || psrc == 0)
    data_len = static_cast<uint16_t>(sizeof(OpenHeader));
  else
    data_len =
      static_cast<uint16_t>(sizeof(OpenHeader) + OpenProtocol::CRCData + w_len);

  if (is_enc)
    data_len = data_len + (16 - w_len % 16);

  DDEBUG("data len: %d\n", data_len);

  p_head->sof       = OpenProtocol::SOF;
  p_head->length    = data_len;
  p_head->version   = 0;
  p_head->sessionID = session_id;
  p_head->isAck     = is_ack ? 1 : 0;
  p_head->reserved0 = 0;

  p_head->padding   = is_enc ? (16 - w_len % 16) : 0;
  p_head->enc       = is_enc ? 1 : 0;
  p_head->reserved1 = 0;

  p_head->sequenceNumber = seq_num;
  p_head->crc            = 0;

  if (psrc && w_len)
    memcpy(pdest + sizeof(OpenHeader), psrc, w_len);
  encodeData(p_head, aes256_encrypt_ecb);

  calculateCRC(pdest);

  return data_len;
}

/******************* Filter *********************/

void
OpenProtocol::setKey(const char* key)
{
  transformTwoByte(key, p_filter->sdkKey);
  p_filter->encode = 1;
}

int
OpenProtocol::crcHeadCheck(uint8_t* pMsg, size_t nLen)
{
  return crc16Calc(pMsg, nLen);
}

int
OpenProtocol::crcTailCheck(uint8_t* pMsg, size_t nLen)
{
  return crc32Calc(pMsg, nLen);
}

void
OpenProtocol::setRawFrame(uint8_t* p_header)
{
  this->rawFrame = p_header;
}

uint8_t*
OpenProtocol::getRawFrame()
{
  return this->rawFrame;
}
