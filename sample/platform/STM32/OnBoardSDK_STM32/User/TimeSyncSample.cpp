/*! @file TimeSyncSample.cpp
 *  @version 3.8.1
 *  @date May 2019
 *
 *  @brief
 *  TimeSync STM32 example.
 *
 *  @Copyright (c) 2019 DJI
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
#include <vector>
#include "TimeSyncSample.h"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
extern Vehicle  vehicle;
extern Vehicle* v;

void NMEACallback(Vehicle* vehiclePtr,
                  RecvContainer recvFrame,
                  UserData userData)
{
  int length = recvFrame.recvInfo.len-OpenProtocol::PackageMin-4;
  char* rawBuf = (char*)malloc(length);
  memcpy(rawBuf, recvFrame.recvData.raw_ack_array, length);
  DSTATUS("%s\n", std::string((char*)rawBuf, length).c_str());
  free(rawBuf);
}

void UTCTimeCallback(Vehicle* vehiclePtr,
                     RecvContainer recvFrame,
                     UserData userData)
{
  int length = recvFrame.recvInfo.len-OpenProtocol::PackageMin-4;
  char* rawBuf = (char*)malloc(length);
  memcpy(rawBuf, recvFrame.recvData.raw_ack_array, length);
  DSTATUS("The UTC time for the next PPS pulse (ard 500ms) is...");
  DSTATUS("%s\n", std::string((char*)rawBuf, length).c_str());
  free(rawBuf);
}

void FCTimeInUTCCallback(Vehicle* vehiclePtr,
                       RecvContainer recvFrame,
                       UserData userData)
{
  DSTATUS("Received Flight controller time in UTC reference...");
  DSTATUS("FC: %u, UTC time: %u, UTC date: %u\n",
          recvFrame.recvData.fcTimeInUTC.fc_timestamp_us,
          recvFrame.recvData.fcTimeInUTC.utc_hhmmss,
          recvFrame.recvData.fcTimeInUTC.utc_yymmdd);
}

void PPSSourceCallback(Vehicle* vehiclePtr,
                         RecvContainer recvFrame,
                         UserData userData)
{
  std::string temp[] = {"0", "INTERNAL_GPS", "EXTERNAL_GPS", "RTK"};
  std::vector<std::string> stringVec(temp, temp + 4);
  DSTATUS("PPS pulse is coming from %s\n", stringVec[recvFrame.recvData.ppsSourceType].c_str());
}

int
time_sync_callback_test()
{
  // Initialize variables
  int functionTimeout = 1;

  // Note that these CBs share the same thread with serial reading
  v->hardSync->subscribeNMEAMsgs(NMEACallback, NULL);
  v->hardSync->subscribeUTCTime(UTCTimeCallback, NULL);
  v->hardSync->subscribeFCTimeInUTCRef(FCTimeInUTCCallback, NULL);
  v->hardSync->subscribePPSSource(PPSSourceCallback, NULL);

  delay_nms(20*1000);

  v->hardSync->unsubscribeNMEAMsgs();
  v->hardSync->unsubscribeUTCTime();
  v->hardSync->unsubscribeFCTimeInUTCRef();
  v->hardSync->unsubscribePPSSource();

  return 0;
}

int
time_sync_poll_test()
{
  // Initialize variables
  int functionTimeout = 1;

  const int waitTimeMs = 100;
  int timeSoFar = 0;
  int totalTimeMs = 30*1000; // 30 secs
  while(timeSoFar < totalTimeMs)
  {
    for (int i = 0; i < HardwareSync::NMEAType::TYPENUM; ++i) {
      DJI::OSDK::HardwareSync::NMEAData nmea;
      if(v->hardSync->getNMEAMsg((HardwareSync::NMEAType) i, nmea))
      {
        DSTATUS("%s\n", nmea.sentence.c_str());
      }
      else
      {
        DSTATUS("Did not get msg\n");
      }
      delay_nms(waitTimeMs);
      timeSoFar += waitTimeMs;
    }
  }

  return 0;
}