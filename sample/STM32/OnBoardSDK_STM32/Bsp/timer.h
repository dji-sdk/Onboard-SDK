/*! @file timer.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Timer helper functions and ISR for board STM32F4Discovery
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#ifndef  TIMER_H
#define  TIMER_H
#include "DJI_API.h"
#include "string.h"
#include <stm32f4xx.h>
using namespace DJI::onboardSDK;
void SystickConfig();
void Timer1Config();
void Timer2Config();
void delay_nms(uint16_t time);

#endif //TIMER_H
