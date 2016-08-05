/*! @file DroneInterface.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Implement the HardDriver for STM32.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */


#ifndef DRONEINTERFACE_H
#define	DRONEINTERFACE_H

#include "DJI_HardDriver.h"
#include "stm32f4xx.h"
#include <time.h>


class DroneInterface : public DJI::onboardSDK::HardDriver
{
  public:
    virtual size_t send(const uint8_t* buf, size_t len);
    virtual DJI::time_ms getTimeStamp();
  
  public:
    virtual void init() { ; }
    virtual void lockMemory() { ; }
    virtual void freeMemory() { ; }

    virtual void lockMSG() { ; }
    virtual void freeMSG() { ; }

    virtual void lockACK() { ; }
    virtual void freeACK() { ; }

    virtual void notify() { ; }
    virtual void wait(int timeout) { ; }
    virtual size_t readall(uint8_t* buf, size_t maxlen) {return 8;}
};

#endif
