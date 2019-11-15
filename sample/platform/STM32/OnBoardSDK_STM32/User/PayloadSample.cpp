/*! @file PayloadSample.cpp
 *  @version 3.8.1
 *  @date May 2019
 *
 *  @brief
 *  PSDK Communication STM32 example.
 *
 *  @Copyright (c) 2016-2019 DJI
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

#include "PayloadSample.h"

#define TEST_DATA_0 0x22
#define TEST_DATA_1 0x33

using namespace DJI;
using namespace DJI::OSDK;

extern Vehicle  vehicle;
extern Vehicle* v;

void parseFromPayloadCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData)
{
  uint8_t *   payload_data;
  uint16_t    payload_data_len;

  payload_data_len = recvFrame.recvInfo.len;
  payload_data  = recvFrame.recvData.raw_ack_array;

  DSTATUS("Received the payload data length is : %d\r\n", payload_data_len);
  DSTATUS("Received the payload data is :\n%s\r\n", payload_data);
}

void PayloadSendingTest(uint8_t TestSecond)
{
	uint8_t data[] = {TEST_DATA_0, TEST_DATA_1};

	DSTATUS("Test start, TestSecond:%ds\n", TestSecond);
	while (TestSecond--)
	{
	  v->payloadDevice->sendDataToPSDK(data, sizeof(data));
		delay_nms(1000);
	}
}
