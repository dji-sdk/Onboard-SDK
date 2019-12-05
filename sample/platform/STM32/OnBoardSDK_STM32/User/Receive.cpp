/*! @file Receive.cpp
 *
 *  @version 3.3
 *  @date Jun 2017
 *
 *  @brief
 *  This function parses Rx buffer and execute commands sent from computer.
 *
 *  @Copyright (c) 2016-2017 DJI
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
#include "Receive.h"
#include "main.h"

using namespace DJI::OSDK;

/*
 * @brief Helper function to assemble two bytes into a float number
 */
static float32_t
hex2Float(uint8_t HighByte, uint8_t LowByte)
{
  float32_t high = (float32_t)(HighByte & 0x7f);
  float32_t low  = (float32_t)LowByte;
  if (HighByte & 0x80) // MSB is 1 means a negative number
  {
    return -(high * 256.0f + low) / 100.0f;
  }
  else
  {
    return (high * 256.0f + low) / 100.0f;
  }
}

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus
void
USART2_IRQHandler(void)
{
  if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET)
  {
    uint8_t oneByte = USART_ReceiveData(USART2);
  }
}
#ifdef __cplusplus
}
#endif //__cplusplus
