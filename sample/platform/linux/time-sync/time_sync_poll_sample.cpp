/*! @file time-sync/time_sync_poll_sample.cpp
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
#include <dji_linux_helpers.hpp>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int main(int argc, char** argv) {
  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle* vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL) {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Obtain Control Authority
  vehicle->control->obtainCtrlAuthority(functionTimeout);
  const int waitTimeMs = 200;
  int timeSoFar = 0;
  int totalTimeMs = 30 * 1000;  // 30 secs

  DJI::OSDK::HardwareSync::GNGSAPackage GNGSA;
  DJI::OSDK::HardwareSync::NMEAData GPRMC;

  while (timeSoFar < totalTimeMs) {
    if (vehicle->hardSync->getGNGSAMsg(GNGSA)) {
      for (int j = 0; j < HardwareSync::SatelliteIndex::MAX_INDEX_CNT; j++) {
        DSTATUS("%s\n", GNGSA.Satellite[j].sentence.c_str());
      }
    } else {
      DSTATUS("Did not get GNGSA msg\n");
    }
    if (vehicle->hardSync->getGNRMCMsg(GPRMC)) {
      DSTATUS("%s\n", GPRMC.sentence.c_str());
    } else {
      DSTATUS("Did not get GNRMC msg\n");
    }
    usleep(waitTimeMs * 1000);
    timeSoFar += waitTimeMs;
  }
  return 0;
}
