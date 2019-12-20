/*! @file timer.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Timer helper functions and ISR for board STM32F4Discovery
 *
 *  @Copyright (c) 2017 DJI
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

#include "stm32f4xx.h"
#include "timer.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

uint32_t tick = 0; // tick is the time stamp,which record how many ms since u
                   // initialize the system.
uint64_t timer1Tick;

void
Timer1Config()
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_Period =
    (100 - 1); // t is the time between each Timer irq.
  TIM_TimeBaseInitStructure.TIM_Prescaler =
    (168 - 1); // t = (1+TIM_Prescaler/SystemCoreClock)*(1+TIM_Period)
  TIM_TimeBaseInitStructure.TIM_RepetitionCounter =
    0x00; // here configure TIM1 in 50Hz
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

  NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_UP_TIM10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  timer1Tick = 0;
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  //TIM_Cmd(TIM1, DISABLE);
  TIM_Cmd(TIM1, ENABLE);
}

void
SystickConfig()
{
  if (SysTick_Config(SystemCoreClock / 1000)) // 1000 ticks per second.
  {
    while (1)
      ; // run here when error.
  }
}

void
delay_nms(uint16_t time)
{
  vTaskDelay(time * portTICK_RATE_MS);
	/*
  uint32_t startTick = tick;
  if ((startTick + time) < 4233600000ll)
  {
    while ((tick - startTick) < time)
      ;
  }
  else
  {
    while (true)
    {
      if ((tick < startTick) && (tick >= (startTick + time - 4233600000ll)))
        break;
    }
  }*/
}

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

//void
//SysTick_Handler(void)
void vApplicationTickHook(void)
{
  if (tick > 4233600000ll) // 49 days non-reset would cost a tick reset.
  {
    tick = 0;
  }
  tick++;
}
void
TIM1_UP_TIM10_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
  {
    //    virtualrc.sendData(myVRCdata);
    timer1Tick++;
  }
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);
}

void
TIM2_IRQHandler()
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
  {
    TIM_Cmd(TIM2, DISABLE);
  }
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
}
#ifdef __cplusplus
}
#endif //__cplusplus
