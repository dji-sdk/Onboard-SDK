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

    HMSSample* hmsSample = new HMSSample(vehicle);
    uint32_t timeOutMs = 500;

    //! get hms version
    std::string hmsVerison = hmsSample->getHMSVersion();
    DSTATUS("HMS Version: %s\n",hmsVerison.c_str());

    //! 1.subscribe flight'status.(or default Error Information is about on the ground)
    int pkgIndex = 0;
    if (!hmsSample->subscribeFlightStatus(pkgIndex))
    {
        return -1;
    }

    //! 2.subscribe HMS's pushing.(Then it will print all error information with 5HZ in English)
    if (!hmsSample->subscribeHMSInf(true, timeOutMs))
    {
        hmsSample->unsubscribeFlightStatus((pkgIndex));
        return -1;
    }

    //! 2.1 print all pushed raw error data with 5HZ in 30 senconds
    const int waitTimeMs = 200;
    int timeSoFar = 0;
    int totalTimeMs = 30*1000; // 30 secs
    while(timeSoFar < totalTimeMs)
    {
        OsdkLinux_TaskSleepMs(waitTimeMs);
        hmsSample->printAllError();
        DSTATUS("now flight status is %d\n", hmsSample->getFlightStatus());
        timeSoFar += waitTimeMs;
    }

    //! 3. reset osdk's subscription for HMS's pushing
    hmsSample->subscribeHMSInf(false, timeOutMs);
    //! 4. unsubscribe flight status
    hmsSample->unsubscribeFlightStatus(pkgIndex);

    delete hmsSample;

    return 0;
}
