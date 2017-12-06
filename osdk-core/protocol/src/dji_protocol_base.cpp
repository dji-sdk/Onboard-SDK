/** @file dji_protocol_base.cpp
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

#include "dji_protocol_base.hpp"

using namespace DJI;
using namespace DJI::OSDK;

ProtocolBase::ProtocolBase()
  : reuse_buffer(true)
  , is_large_data_protocol(false)
  , BUFFER_SIZE(1024)
{
}

ProtocolBase::~ProtocolBase()
{
  if (this->deviceDriver)
    delete this->deviceDriver;

  if (this->threadHandle)
    delete this->threadHandle;
}

//! Step 0: Call this in a loop.
RecvContainer*
ProtocolBase::receive()
{
  //! Create a local container that will be used for storing data lower down in
  //! the stack
  p_recvContainer->recvInfo.cmd_id = 0xFF;

  //! Run the readPoll see if we got a frame
  readPoll();

  //! When we receive a true, return a copy of container to the caller: this is
  //! the 'receive' interface

  return p_recvContainer;
}

//! Step 1
bool
ProtocolBase::readPoll()
{
  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = false;

  //! Step 1: Check if the buffer has been consumed
  if (buf_read_pos >= read_len)
  {
    this->buf_read_pos = 0;
    this->read_len     = deviceDriver->readall(this->buf, BUFFER_SIZE);
  }

#ifdef API_BUFFER_DATA
  onceRead = read_len;
  totalRead += onceRead;
#endif // API_BUFFER_DATA

  //! Step 2:
  //! For large data protocol, store the value and only verify the header
  //! For small data protocol, Go through the buffer and return when you
  //! see a full frame. buf_read_pos will maintain state about how much
  //! buffer data we have already read
  if (is_large_data_protocol && this->read_len == BUFFER_SIZE)
  {
    memcpy(p_filter->recvBuf + (p_filter->recvIndex), this->buf,
           sizeof(uint8_t) * BUFFER_SIZE);
    p_filter->recvIndex += BUFFER_SIZE;
    this->buf_read_pos = BUFFER_SIZE;
  }
  else
  {
    for (this->buf_read_pos; this->buf_read_pos < this->read_len;
         this->buf_read_pos++)
    {
      isFrame = byteHandler(buf[this->buf_read_pos]);

      if (isFrame)
      {
        return isFrame;
      }
    }
  }

  //! Step 3: If we don't find a full frame by this time, return false.
  //! The receive function calls readPoll in a loop, so if it returns false
  //! it'll just be called again
  return isFrame;
}

//! Step 2
bool
ProtocolBase::byteHandler(const uint8_t in_data)
{
  p_filter->reuseCount = 0;
  p_filter->reuseIndex = MAX_RECV_LEN;

  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = streamHandler(in_data);

  /*! @note Just think a command as below
    *
    * [123456HHD1234567===HHHH------------------] --- is buf un-used part
    *
    * if after recv full of above, but crc failed, we throw all data?
    * NO!
    * Just throw ONE BYTE, we move like below
    *
    * [123456HH------------------D1234567===HHHH]
    *
    * Use the buffer high part to re-loop, try to find a new command
    *
    * if new cmd also fail, and buf like below
    *
    * [56HHD1234567----------------------===HHHH]
    *
    * throw one byte, buf looks like
    *
    * [6HHD123-----------------------4567===HHHH]
    *
    * the command tail part move to buffer right
    * */
  if (reuse_buffer)
  {
    if (p_filter->reuseCount != 0)
    {
      while (p_filter->reuseIndex < MAX_RECV_LEN)
      {
        /*! @note because reuse_index maybe re-located, so reuse_index must
         * be
         *  always point to un-used index
         *  re-loop the buffered data
         *  */
        isFrame = streamHandler(p_filter->recvBuf[p_filter->reuseIndex++]);
      }
      p_filter->reuseCount = 0;
    }
  }
  return isFrame;
}

//! Step 3
bool
ProtocolBase::streamHandler(uint8_t in_data)
{
  storeData(in_data);
  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = checkStream();
  return isFrame;
}

//! Step 4
//! @note push data to filter buffer.
//! SDKFilter is just a buffer.
void
ProtocolBase::storeData(uint8_t in_data)
{
  if (p_filter->recvIndex < MAX_RECV_LEN)
  {
    p_filter->recvBuf[p_filter->recvIndex] = in_data;
    p_filter->recvIndex++;
  }
  else
  {
    DERROR("buffer overflow");
    memset(p_filter->recvBuf, 0, p_filter->recvIndex);
    p_filter->recvIndex = 0;
  }
}

void
ProtocolBase::prepareDataStream()
{
  uint32_t bytes_to_move = HEADER_LEN - 1;
  uint32_t index_of_move = p_filter->recvIndex - bytes_to_move;

  memmove(p_filter->recvBuf, p_filter->recvBuf + index_of_move, bytes_to_move);
  memset(p_filter->recvBuf + bytes_to_move, 0, index_of_move);
  p_filter->recvIndex = bytes_to_move;
}

void
ProtocolBase::shiftDataStream()
{
  if (p_filter->recvIndex)
  {
    p_filter->recvIndex--;
    if (p_filter->recvIndex)
    {
      memmove(p_filter->recvBuf, p_filter->recvBuf + 1, p_filter->recvIndex);
    }
  }
}

// this function will move the data part to buffer end,
// head part will move left
//
//  1. there no re-use data
//  |------------------------------------------| <= cache
//                       ^
//                       reuse_index
//  [12345678][ data Part ]--------------------| 1. p_filter
//  [12345678]---------------------[ data Part ] 2. move data to end
//  [2345678]----------------------[ data Part ] 3. forward head
//  [2345678]------------[ data need to re-use ] 4. final mem layout
//
//  2. already has re-use data
//  |---------------------------------[rev data] <= cache
//                  ^
//                  reuse_index, the data already used
//  [12345678][ data Part ]-----------[rev data] 1. p_filter
//  [12345678]-----------[ data Part ][rev data] 2. move data to end
//  [2345678]------------[ data Part ][rev data] 3. forward head
//  [2345678]------------[ data need to re-use ] 4. final mem layout
//
// the re-use data will loop later

void
ProtocolBase::reuseDataStream()
{
  uint8_t* p_buf         = p_filter->recvBuf;
  uint16_t bytes_to_move = p_filter->recvIndex - HEADER_LEN;
  uint8_t* p_src         = p_buf + HEADER_LEN;

  uint16_t n_dest_index = p_filter->reuseIndex - bytes_to_move;
  uint8_t* p_dest       = p_buf + n_dest_index;

  memmove(p_dest, p_src, bytes_to_move);

  p_filter->recvIndex = HEADER_LEN;
  shiftDataStream();

  p_filter->reuseIndex = n_dest_index;
  p_filter->reuseCount++;
}

HardDriver*
ProtocolBase::getDriver() const
{
  return this->deviceDriver;
}

ThreadAbstract*
ProtocolBase::getThreadHandle() const
{
  return this->threadHandle;
}

void
ProtocolBase::setDriver(HardDriver* hardDriver_ptr)
{
  this->deviceDriver = hardDriver_ptr;
}

void
ProtocolBase::setThreadHandle(ThreadAbstract* threadAbstract_ptr)
{
  this->threadHandle = threadAbstract_ptr;
}

void
ProtocolBase::setHeaderLength(uint8_t length)
{
  HEADER_LEN = length;
}

void
ProtocolBase::setMaxRecvLength(int length)
{
  MAX_RECV_LEN = length;
}

int
ProtocolBase::getBufReadPos()
{
  return buf_read_pos;
}

int
ProtocolBase::getReadLen()
{
  return read_len;
}

RecvContainer*
ProtocolBase::getReceivedFrame()
{
  return p_recvContainer;
}