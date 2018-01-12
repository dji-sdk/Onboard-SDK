/*! @file STM32F4SerialDriver.cpp
 *  @version 3.3
 *  @date Jun 2017
 *
 *  @brief
 *  Implementation of HardDriver for the STM32F4Discovery board.
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
#include <STM32F4SerialDriver.h>
#include <time.h>

void STM32F4RAA::delay_nms(uint16_t time) {
	HAL_Delay(time);
}

size_t STM32F4RAA::send(const uint8_t* buf, size_t len) {
//-> Map this through to the USART wrapper on uart 1
	sdkuart.transmit((uint8_t*)buf, len);
}

DJI::OSDK::time_ms STM32F4RAA::getTimeStamp() {
	return HAL_GetTick();
}

