/*! @file main.cpp
*  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  An exmaple program of DJI-onboard-SDK portable for stm32
 *
 *  Copyright 2016 DJI. All right reserved.
 *  */

#ifndef MAIN_H
#define MAIN_H

#include "string.h"
#include "stdio.h"
#include "DJI_API.h"
#include "driver.h"
#include "DJI_Flight.h"
#include "BspUsart.h"
#include "stm32f4xx_conf.h"
#include "cppforstm32.h"
#include "stm32f4xx.h"
#include "Activate.h"
#include "DJI_VirtualRC.h"
#include "timer.h"
#include <stdlib.h>
#include "VirtualRC.h"
#include "DJI_Flight.h"
#include "DJI_HotPoint.h"
#include "HotPoint.h"
#include "Receive.h"
#include "bsp.h"
#include "LocalNavigation.h"
#include "MobileCommand.h"

extern uint32_t tick; //tick is the time stamp,which record how many ms since u initialize the system.
//warnning: after 49 days of non-reset running, tick will RESET to ZERO.

#endif //MAIN_H

