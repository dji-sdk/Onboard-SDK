/** @file dji_circular_buffer.cpp
 *  @version 3.3
 *  @date Jun 2017
 *
 *  @brief Circular buffer class for the DJI OSDK
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

#include "dji_circular_buffer.hpp"

using namespace DJI;
using namespace DJI::OSDK;

CircularBuffer::CircularBuffer()
  : maxLen(5000)
{
  buffer =
    (VehicleCallBackHandler*)malloc(5000 * sizeof(VehicleCallBackHandler));
  if(buffer == NULL){
    DERROR("buffer memory alloc failed\n");
  }

  buffer2 = (RecvContainer*)malloc(5000 * sizeof(RecvContainer));
  if(buffer2 == NULL){
    DERROR("buffer2 memory alloc failed\n");
  }

  head    = 0;
  tail    = 0;
}

CircularBuffer::~CircularBuffer()
{
  if (buffer)
  {
    free(buffer);
  }
  if (buffer2)
  {
    free(buffer2);
  }
}

int
CircularBuffer::cbPush(CircularBuffer*                   CBuffer,
                       DJI::OSDK::VehicleCallBackHandler cbData,
                       RecvContainer                     recvData)
{
  int next = head + 1;
  if (next >= maxLen)
  {
    next = 0;
  }
  //! Circular buffer is full, pop the old value and discard.
  if (next == tail)
  {
    CBuffer->cbPop(CBuffer, &cbData, &recvData);
    DSTATUS("Warning: Circular Buffer Full. Discarded Callback from Tail \n");
  }
  buffer2[head] = recvData;
  buffer[head]  = cbData;
  head          = next;
  return 0;
}

int
CircularBuffer::cbPop(CircularBuffer*                    CBuffer,
                      DJI::OSDK::VehicleCallBackHandler* cbData,
                      RecvContainer*                     recvData)
{
  if (head == tail)
  {
    DSTATUS("Circular Buffer empty \n");
    return -1;
  }
  *cbData   = buffer[tail];
  *recvData = buffer2[tail];

  //! Clear data
  memset(&buffer[tail], 0, sizeof(VehicleCallBackHandler));
  memset(&buffer2[tail], 0, sizeof(RecvContainer));

  int next = tail + 1;
  if (next >= maxLen)
    next = 0;
  tail   = next;
  return 0;
}
