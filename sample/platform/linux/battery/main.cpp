/** @file hms_poll_sample.cpp
 *  @version 4.0
 *  @date Dec 2019
 *
 *  @brief
 *  HMS(Health Management System) API usage in a Linux environment
 *  show example usage of getting flight's health information by normal subscription
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

#include "dji_vehicle.hpp"
#include "dji_linux_helpers.hpp"
#include "dji_hms.hpp"
#include "hms_sample.hpp"
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
    bool enableSubscribeSingleBatteryDynamicInfoCmd = true;
    uint8_t batteryIndex = 1;
    BatteryWholeInfo batteryWholeInfo;
    SmartBatteryDynamicInfo firstBatteryDynamicInfo;
    SmartBatteryDynamicInfo secondBatteryDynamicInfo;

    vehicle->djiBattery->subscribeBatteryWholeInfo(enableSubscribeBatteryWholeInfo);
    vehicle->djiBattery->subscribeSingleBatteryDynamicInfo(enableSubscribeSingleBatteryDynamicInfoCmd, batteryIndex);
    batteryIndex = 2;
    vehicle->djiBattery->subscribeSingleBatteryDynamicInfo(enableSubscribeSingleBatteryDynamicInfoCmd, batteryIndex);
    while(true)
    {
        vehicle->djiBattery->getBatteryWholeInfo(batteryWholeInfo);
        vehicle->djiBattery->getFirstBatteryDynamicInfo(firstBatteryDynamicInfo);
        vehicle->djiBattery->getSecondBatteryDynamicInfo(secondBatteryDynamicInfo);
    }

    return 0;
}
