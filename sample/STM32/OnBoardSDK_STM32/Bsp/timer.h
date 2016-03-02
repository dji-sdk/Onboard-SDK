#ifndef  TIMER_H
#define  TIMER_H
#include "DJI_API.h"
#include "string.h"
#include <stm32f4xx.h>
using namespace DJI::onboardSDK;
void SystickConfig();
void Timer1Config();
void Timer2Config();
#endif //TIMER_H
