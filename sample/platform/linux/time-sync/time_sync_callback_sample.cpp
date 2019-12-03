/*! @file time-sync/time_sync_callback_sample.cpp
 *  @version 3.8
 *  @date Mar 11 2019
 *
 *  @brief
 *  M210 V2 time sync APIs in a Linux environment.
 *  Shows example usage of getting NMEA data and time information 
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
// Helpers
#include <dji_linux_helpers.hpp>
#include <vector>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

void NMEACallback(Vehicle* vehiclePtr,
                  RecvContainer recvFrame,
                  UserData userData)
{
  int length = recvFrame.recvInfo.len-OpenProtocol::PackageMin-4;
  uint8_t rawBuf[length];
  memcpy(rawBuf, recvFrame.recvData.raw_ack_array, length);
  DSTATUS("%s\n", std::string((char*)rawBuf, length).c_str());
}

void UTCTimeCallback(Vehicle* vehiclePtr,
                     RecvContainer recvFrame,
                     UserData userData)
{
  int length = recvFrame.recvInfo.len-OpenProtocol::PackageMin-4;
  uint8_t rawBuf[length];
  memcpy(rawBuf, recvFrame.recvData.raw_ack_array, length);
  DSTATUS("The UTC time for the next PPS pulse (ard 500ms) is...");
  DSTATUS("%s\n", std::string((char*)rawBuf, length).c_str());
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
  std::vector<std::string> stringVec = {"0", "INTERNAL_GPS", "EXTERNAL_GPS", "RTK"};
  DSTATUS("PPS pulse is coming from %s\n", stringVec[recvFrame.recvData.ppsSourceType].c_str());
}

int
main(int argc, char** argv)
{
  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);


  // Note that these CBs share the same thread with serial reading
  vehicle->hardSync->subscribeNMEAMsgs(NMEACallback, nullptr);
  vehicle->hardSync->subscribeUTCTime(UTCTimeCallback, nullptr);
  vehicle->hardSync->subscribeFCTimeInUTCRef(FCTimeInUTCCallback, nullptr);
  vehicle->hardSync->subscribePPSSource(PPSSourceCallback, nullptr);

  sleep(20);

  vehicle->hardSync->unsubscribeNMEAMsgs();
  vehicle->hardSync->unsubscribeUTCTime();
  vehicle->hardSync->unsubscribeFCTimeInUTCRef();
  vehicle->hardSync->unsubscribePPSSource();

  return 0;
}