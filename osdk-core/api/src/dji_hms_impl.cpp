/** @file dji_hms.cpp
 *  @version 4.0
 *  @date Dec 2019
 *
 *  @brief HMS(Health Management System) API for DJI OSDK implement
 *  @Details Provide API to subscribe Flight's Health State.
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

#include <unistd.h>
#include "dji_vehicle.hpp"
#include "dji_hms_impl.hpp"
#include "osdk_device_id.h"

using namespace std;
using namespace DJI;
using namespace DJI::OSDK;
namespace DJI{
    namespace OSDK{
        extern void encodeSender(const uint8_t sender,uint8_t & deviceType, uint8_t & deviceIndex);
    }
}

#ifndef DJIOSDK_HMS_MAJOR_VERSION
#define DJIOSDK_HMS_MAJOR_VERSION 1
#endif

#ifndef DJIOSDK_HMS_MINOR_VERSION
#define DJIOSDK_HMS_MINOR_VERSION 0
#endif

#ifndef DJIOSDK_HMS_PATCH_VERSION
#define DJIOSDK_HMS_PATCH_VERSION 1
#endif

DJIHMSImpl::DJIHMSImpl(Vehicle *vehicle):vehicle(vehicle)
{
    this->createHMSInfoLock();
}

DJIHMSImpl::~DJIHMSImpl()
{
    this->destroyHMSInfoLock();
}

string DJIHMSImpl::getHMSVersion()
{
    string hmsVersion;
    sprintf((char *)hmsVersion.data(),"HMS%d.%d.%d",DJIOSDK_HMS_MAJOR_VERSION,
            DJIOSDK_HMS_MINOR_VERSION, DJIOSDK_HMS_PATCH_VERSION);

    return hmsVersion.data();
}

HMSPushPacket& DJIHMSImpl::getHMSPushPacket()
{
    return this->hmsPushPacket;
}

uint8_t DJIHMSImpl::getDeviceIndex()
{

    return deviceIndex;
}

void DJIHMSImpl::setHMSPushData(const uint8_t *hmsPushData, uint16_t dataLen)
{
    memcpy(&(this->hmsPushPacket.hmsPushData), hmsPushData, 3 * sizeof(uint8_t));
    this->hmsPushPacket.hmsPushData.errList.clear();
    this->hmsPushPacket.hmsPushData.errList.resize((dataLen - 3 * sizeof(uint8_t)) / sizeof(ErrList));
    memcpy(&(this->hmsPushPacket.hmsPushData.errList[0]), hmsPushData + 3 * sizeof(uint8_t), dataLen - 3 * sizeof(uint8_t));
}

void DJIHMSImpl::setHMSTimeStamp()
{
    OsdkOsal_GetTimeMs(&this->hmsPushPacket.timeStamp);
}

void DJIHMSImpl::setDeviceIndex(uint8_t sender)
{
    this->deviceIndex = getComponentIndex(sender);
}

uint8_t DJIHMSImpl::getComponentIndex(uint8_t sender)
{
    uint8_t deviceType = 0;
    uint8_t deviceIndex = 0;
    DJI::OSDK::encodeSender(sender, deviceType, deviceIndex);
    if (deviceType == OSDK_COMMAND_DEVICE_TYPE_CAMERA ||
        deviceType == OSDK_COMMAND_DEVICE_TYPE_CENTER)
    {
        switch (deviceIndex)
        {
            case 0x00:
            case 0x01:
                /*! index of camera 1*/
                return CameraIndex1;
            case 0x02:
            case 0x03:
                /*! index of camera 2*/
                return CameraIndex2;
            case 0x04:
            case 0x05:
                /*! index of camera 3*/
                return CameraIndex3;
        }
    } else if (deviceType == OSDK_COMMAND_DEVICE_TYPE_GIMBAL){
        switch (deviceIndex)
        {
            case 0x00:
                /*! index of gimbal 1*/
                return GimbalIndex1;
            case 0x02:
                /*! index of gimbal 2*/
                return GimbalIndex2;
            case 0x04:
                /*! index of gimbal 3*/
                return GimbalIndex3;
        }
    }

    return InvalidIndex;
}

bool DJIHMSImpl::createHMSInfoLock()
{
    E_OsdkStat errCode;
    errCode = OsdkOsal_MutexCreate(&m_hmsLock);

    return errCode == OSDK_STAT_OK;
}

bool DJIHMSImpl::lockHMSInfo()
{
    E_OsdkStat errCode;
    errCode = OsdkOsal_MutexLock(m_hmsLock);

    return errCode == OSDK_STAT_OK;
}

bool DJIHMSImpl::freeHMSInfo()
{
    E_OsdkStat errCode;
    errCode = OsdkOsal_MutexUnlock(m_hmsLock);

    return errCode == OSDK_STAT_OK;
}

bool DJIHMSImpl::destroyHMSInfoLock()
{
    E_OsdkStat errCode;
    errCode = OsdkOsal_MutexDestroy(&m_hmsLock);

    return errCode == OSDK_STAT_OK;
}

