/** @file dji_hms_impl.hpp
 *  @version 4.0
 *  @date APRIL 2020
 *
 *  @brief Battery API for DJI OSDK implement
 *  @Details Provide API to get battery's Dynamic data.
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

#include "dji_battery_impl.hpp"
#include "dji_vehicle.hpp"

DJIBatteryImpl::DJIBatteryImpl(Vehicle* vehicle)
:vehicle(vehicle)
{
    this->createBatteryInfoLock();
}

DJIBatteryImpl::~DJIBatteryImpl()
{
    this->destroyBatteryInfoLock();
}

void DJIBatteryImpl::setBatteryWholeInfo(const uint8_t *batteryData, const uint16_t dataLen)
{
    memcpy(&batteryWholeInfo, batteryData ,dataLen);
}

void DJIBatteryImpl::getBatteryWholeInfo(BatteryWholeInfoImpl& batteryWholeInfo)
{
    batteryWholeInfo = this->batteryWholeInfo;
}

bool DJIBatteryImpl::createBatteryInfoLock(void)
{
    E_OsdkStat errCode;
    errCode = OsdkOsal_MutexCreate(&m_batteryLock);

    return errCode == OSDK_STAT_OK;
}

bool DJIBatteryImpl::lockBatteryInfo()
{
    E_OsdkStat errCode;
    errCode = OsdkOsal_MutexLock(m_batteryLock);

    return errCode == OSDK_STAT_OK;
}

bool DJIBatteryImpl::freeBatteryInfo()
{
    E_OsdkStat errCode;
    errCode = OsdkOsal_MutexUnlock(m_batteryLock);

    return errCode == OSDK_STAT_OK;
}

bool DJIBatteryImpl::destroyBatteryInfoLock()
{
    E_OsdkStat errCode;
    errCode = OsdkOsal_MutexDestroy(&m_batteryLock);

    return errCode == OSDK_STAT_OK;
}
