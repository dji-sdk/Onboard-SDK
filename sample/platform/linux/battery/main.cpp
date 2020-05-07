/** @file main.cpp
 *  @version 4.0
 *  @date April 2020
 *
 *  @brief
 *  Battery API usage in a Linux environment
 *  show example usage of getting flight's battery information
 *
 *  @Copyright (c) 2020 DJI
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

#include "dji_vehicle.hpp"
#include "dji_linux_helpers.hpp"
#include "osdkosal_linux.h"


using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int main(int argc, char** argv) {
    // Setup OSDK.
    LinuxSetup linuxEnvironment(argc, argv);
    Vehicle *vehicle = linuxEnvironment.getVehicle();

    if (vehicle == NULL) {
        std::cout << "Vehicle not initialized, exiting.\n";
        return -1;
    }
    bool enableSubscribeBatteryWholeInfo = true;
    BatteryWholeInfo batteryWholeInfo;
    SmartBatteryDynamicInfo firstBatteryDynamicInfo;
    SmartBatteryDynamicInfo secondBatteryDynamicInfo;
    const int waitTimeMs = 500;

    vehicle->djiBattery->subscribeBatteryWholeInfo(enableSubscribeBatteryWholeInfo);

    while(true)
    {
      vehicle->djiBattery->getBatteryWholeInfo(batteryWholeInfo);
      DSTATUS("batteryCapacityPercentage is %ld\n",batteryWholeInfo.batteryCapacityPercentage);
      vehicle->djiBattery->getSingleBatteryDynamicInfo(DJIBattery::RequestSmartBatteryIndex::FIRST_SMART_BATTERY, firstBatteryDynamicInfo);
      DSTATUS("battery index %d batteryCapacityPercent is %ld\n",firstBatteryDynamicInfo.batteryIndex, firstBatteryDynamicInfo.batteryCapacityPercent);
      vehicle->djiBattery->getSingleBatteryDynamicInfo(DJIBattery::RequestSmartBatteryIndex::SECOND_SMART_BATTERY, secondBatteryDynamicInfo);
      DSTATUS("battery index %d batteryCapacityPercent is %ld\n",secondBatteryDynamicInfo.batteryIndex, secondBatteryDynamicInfo.batteryCapacityPercent);

      OsdkLinux_TaskSleepMs(waitTimeMs);
    }

    return 0;
}
