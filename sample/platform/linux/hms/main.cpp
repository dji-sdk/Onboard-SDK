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
#include "../../../core/inc/hms_sample.hpp"
#include "osdkosal_linux.h"


using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int main(int argc, char** argv) {
    // Initialize variables
    int functionTimeout = 1;

    // Setup OSDK.
    LinuxSetup linuxEnvironment(argc, argv);
    Vehicle *vehicle = linuxEnvironment.getVehicle();

    if (vehicle == NULL) {
        std::cout << "Vehicle not initialized, exiting.\n";
        return -1;
    }

    // Obtain Control Authority
    vehicle->control->obtainCtrlAuthority(functionTimeout);

    HMSSample* hmsSample = new HMSSample(vehicle);
    uint32_t timeOut = 500;

    //! get hms version
    std::string hmsVerison = hmsSample->getHMSVersion();
    DSTATUS("HMS Version: %s\n",hmsVerison.c_str());

    //! 1. subscribe HMS's pushing.
    bool ret = hmsSample->subscribeHMSInf(true, timeOut);
    int pkgIndex = 0;
    int controlFreqInHz = 10;  // Hz
    int responseTimeout = 1;
    if (true == ret)
    {
        //! 1.1  when subscription is succcess, subscribe flight status.
        //!      different prompt information will be displayed according to different flight status
        //!      If this step is not performed, the prompt message of flight status on the ground will be displayed by default.

        TopicName topicList[] = { TOPIC_STATUS_FLIGHT };
        int numTopic = sizeof(topicList) / sizeof(topicList[0]);
        if (!hmsSample->setUpSubscription(pkgIndex, controlFreqInHz, topicList, numTopic,
                                          responseTimeout)) {
            return -1;
        }
    }
    else
    {
        return -1;
    }

    //5HZ.
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

    //! 2. reset osdk's subscription for HMS's pushing
    hmsSample->subscribeHMSInf(false, timeOut);
    //! 3. tear down flight status's subscription
    hmsSample->teardownSubscription(pkgIndex, timeOut);

    return 0;
}
