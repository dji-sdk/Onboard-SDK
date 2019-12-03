/*! @file payload-control-sample.cpp
 *  @version 3.8.1
 *  @date May 05 2019
 *
 *  @brief
 *  Payload SDK Communication API usage in a Linux environment.
 *  Shows example usage of the payload<-->onboard SDK communication API.
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
#include "payload-control-sample.hpp"



void sendDataToPSDK(Vehicle* vehicle, uint8_t* data, uint16_t len)
{
  vehicle->payloadDevice->sendDataToPSDK(data, len);
}

void parseFromPayloadCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData)
{
  uint8_t *   payload_data;
  uint16_t    payload_data_len;

  payload_data_len = recvFrame.recvInfo.len;
  payload_data  = recvFrame.recvData.raw_ack_array;

  std::cout << "Received the payload data length is :"   << payload_data_len<< std::endl;
  std::cout << "Received the payload data is :"   << payload_data << std::endl;
}

void setFromPSDKCallback(Vehicle* vehicle, LinuxSetup* linuxEnvironment)
{
  vehicle->payloadDevice->setFromPSDKCallback(parseFromPayloadCallback, linuxEnvironment);
}
