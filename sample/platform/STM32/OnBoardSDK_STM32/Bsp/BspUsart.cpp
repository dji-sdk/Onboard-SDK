/*! @file BspUsart.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Usart helper functions and ISR for board STM32F4Discovery
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
#include "BspUsart.h"
#include "timer.h"
#include "FreeRTOS.h"
#include "queue.h"

#define USART_3_BUFFER_SIZE 1024

using namespace DJI::OSDK;

QueueHandle_t UartDataRecvQueue;

void
USART2_Gpio_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // tx
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // rx
}

void
USART3_Gpio_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); // tx
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); // rx
}

/*
 * USART2 is used for receiving commands from PC and
 * printing debug information to PC
 */
void
USART2_Config(void)
{
  USART2_Gpio_Config();

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate   = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits   = USART_StopBits_1;
  USART_InitStructure.USART_Parity     = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  USART_Cmd(USART2, ENABLE);

  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) != SET)
    ;
}

/*
 * USART3 is used for communicating with the DJI flight controller
 * The Baud rate needs to match the Baud rate used by the flight controller
 */
void
USART3_Config(void)
{
  UartDataRecvQueue = xQueueCreate(USART_3_BUFFER_SIZE, sizeof(uint8_t));
  USART3_Gpio_Config();

  USART_InitTypeDef USART_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  USART_InitStructure.USART_BaudRate   = 921600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits   = USART_StopBits_1;
  USART_InitStructure.USART_Parity     = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART3, &USART_InitStructure);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  USART_Cmd(USART3, ENABLE);
  while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) != SET)
    ;
}

void
USARTxNVIC_Config()
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  NVIC_InitTypeDef NVIC_InitStructure_USART3;
  NVIC_InitStructure_USART3.NVIC_IRQChannelPreemptionPriority = 0x05;
  NVIC_InitStructure_USART3.NVIC_IRQChannel                   = USART3_IRQn;
  NVIC_InitStructure_USART3.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure_USART3);

  NVIC_InitTypeDef NVIC_InitStructure_USART2;
  NVIC_InitStructure_USART2.NVIC_IRQChannelPreemptionPriority = 0x06;
  NVIC_InitStructure_USART2.NVIC_IRQChannel                   = USART2_IRQn;
  NVIC_InitStructure_USART2.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure_USART2);
}

void
UsartConfig()
{
  USART2_Config();
  USART3_Config();
  USARTxNVIC_Config();
}

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus
    
  
int u3IrqCnt = 0;

void
USART3_IRQHandler(void)
{
  if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
  {
    BaseType_t xHigherPriorityTaskWoken = false;
    uint8_t data = USART_ReceiveData(USART3);
    u3IrqCnt++;
    xQueueSendFromISR(UartDataRecvQueue, &data, &xHigherPriorityTaskWoken);
  }
}

#ifdef __cplusplus
}
#endif //__cplusplus
