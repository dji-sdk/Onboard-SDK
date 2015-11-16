/*
 * DJI_Pro_Hw.cpp
 *
 *  Created on: Aug 24, 2015
 *  Author: wuyuwei
 */
#include "DJI_HardDriver.h"
void DJI::onboardSDK::HardDriver::lockMemory()
{
    mmu_lock->lock();
}

void DJI::onboardSDK::HardDriver::freeMemory()
{
    mmu_lock->unlock();
}

void DJI::onboardSDK::HardDriver::lockMSG()
{
    std_msg_lock->lock();
}

void DJI::onboardSDK::HardDriver::freeMSG()
{
    std_msg_lock->unlock();
}
DJI::onboardSDK::port_t DJI::onboardSDK::HardDriver::getPort() const
{
    return port;
}

void DJI::onboardSDK::HardDriver::setPort(const DJI::onboardSDK::port_t &value)
{
    port = value;
}


DJI::onboardSDK::HardDriver::HardDriver()
{
    mmu_lock = new QMutex();
    std_msg_lock = new QMutex();
}
