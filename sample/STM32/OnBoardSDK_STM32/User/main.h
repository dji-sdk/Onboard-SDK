#ifndef MAIN_H
#define MAIN_H

#include "string.h"
#include "stdio.h"
#include "DJI_API.h"
#include "DJI_HardDriver.h"
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

extern uint32_t tick;    //tick is the time stamp,which record how many ms since u initialize the system.
												 //warnning: after 49 days of non-reset running, tick will RESET to ZERO.												 
extern	int Rx_Handle_Flag;	

void delay_nms(u16 time);

#endif //MAIN_H

