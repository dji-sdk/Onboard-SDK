/** @file dji_hms.hpp
 *  @version 4.0
 *  @date Dec 2019
 *
 *  @brief
 *  HMS sample use HMS API in a Linux environment..
 *  Provides a number of helpful additions to core API calls,
 *  like API of subscribing flight status.
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
#include "hms_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

HMSSample::HMSSample(Vehicle *vehicle)
        :vehicle(vehicle)
{

};

HMSSample::~HMSSample()
{

};

std::string HMSSample::getHMSVersion()
{
    return vehicle->djiHms->getHMSVersion();
}

bool HMSSample::subscribeHMSInf(bool enable, uint32_t timeOutMs)
{
    return vehicle->djiHms->subscribeHMSInf(enable, timeOutMs);
}

bool HMSSample::subscribeFlightStatus(const int pkgIndex)
{
    if (vehicle) {
        /*! Telemetry: Verify the subscription*/
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = vehicle->subscribe->verify(1);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, __func__);
            return false;
        }

        bool enableTimestamp = false;
        TopicName topicList[] = { TOPIC_STATUS_FLIGHT };
        int numTopic = sizeof(topicList) / sizeof(topicList[0]);
        int freq = 10;  // Hz
        int responseTimeout = 1;

        bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
                pkgIndex, numTopic, topicList, enableTimestamp, freq);
        if (!(pkgStatus))
        {
            return pkgStatus;
        }

        /*! Start listening to the telemetry data */
        subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, __func__);
            /*! Cleanup*/
            ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
            if (ACK::getError(ack))
            {
                DERROR(
                        "Error unsubscription; please restart the drone/FC to get "
                        "back to a clean state");
            }
            return false;
        }
        return true;
    }
    else
    {
        DERROR("vehicle haven't been initialized", __func__);
        return false;
    }
}

bool HMSSample::unsubscribeFlightStatus(const int pkgIndex) {
    int responseTimeout = 1;
    ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    if (ACK::getError(ack)) {
        DERROR(
                "Error unsubscription; please restart the drone/FC to get back "
                "to a clean state.");
        return false;
    }
    return true;
}


uint8_t HMSSample::getFlightStatus(void)
{
    return vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>();
}

void HMSSample::printAllError(void)
{
    HMSPushPacket hmsPushPacket = vehicle->djiHms->getHMSPushPacket();
    uint8_t deviceIndex = vehicle->djiHms->getDeviceIndex();
    DSTATUS("TimeStamp: %ld, msgversion: %d, globalIndex: %d, msgEnd: %d, msgIndex: %d",
            hmsPushPacket.timeStamp,
            hmsPushPacket.hmsPushData.msgVersion,
            hmsPushPacket.hmsPushData.globalIndex,
            hmsPushPacket.hmsPushData.msgEnd,
            hmsPushPacket.hmsPushData.msgIndex);
    for (size_t i = 0; i < hmsPushPacket.hmsPushData.errList.size(); i++)
    {
        DSTATUS("hmsErrListNum: %d, alarm_id: 0x%08x, sensorIndex: %d, level: %d",
                i, hmsPushPacket.hmsPushData.errList[i].alarmID,
                hmsPushPacket.hmsPushData.errList[i].sensorIndex,
                hmsPushPacket.hmsPushData.errList[i].reportLevel);
        if (deviceIndex != 0xff)
        {
            DSTATUS("%d Camera/Gimbal has trouble!", deviceIndex);
        }
    }
}

