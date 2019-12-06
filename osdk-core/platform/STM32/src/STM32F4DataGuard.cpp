/*! @file STM32F4DataGuard.cpp
 *  @version 3.3
 *  @date Apr 12th, 2017
 *
 *  @brief
 *  Data protection for thread access for DJI Onboard SDK Linux/*NIX platforms
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
 */

#include "STM32F4DataGuard.h"

using namespace DJI::OSDK;

STM32F4DataGuard::STM32F4DataGuard()
{
}

STM32F4DataGuard::~STM32F4DataGuard()
{
  vSemaphoreDelete(this->m_memLock);
  vSemaphoreDelete(this->m_msgLock);
  vSemaphoreDelete(this->m_ackLock);
  vSemaphoreDelete(this->m_headerLock);
  vSemaphoreDelete(this->m_nbAckLock);
  vSemaphoreDelete(this->m_frameLock);
  vSemaphoreDelete(this->m_ackRecvSem);
}

void
STM32F4DataGuard::init()
{
  this->m_memLock = xSemaphoreCreateMutex();
  this->m_msgLock = xSemaphoreCreateMutex();
  this->m_ackLock = xSemaphoreCreateMutex();
  this->m_headerLock = xSemaphoreCreateMutex();
  this->m_nbAckLock = xSemaphoreCreateMutex();
  this->m_frameLock = xSemaphoreCreateMutex();
  this->m_ackRecvSem = xSemaphoreCreateBinary();
}

void
STM32F4DataGuard::lockRecvContainer()
{
  xSemaphoreTake(this->m_memLock, portMAX_DELAY);
}

void
STM32F4DataGuard::freeRecvContainer()
{
  xSemaphoreGive(this->m_memLock);
}

void
STM32F4DataGuard::lockMSG()
{
  xSemaphoreTake(this->m_msgLock, portMAX_DELAY);
}

void
STM32F4DataGuard::freeMSG()
{
  xSemaphoreGive(this->m_msgLock);
}

void
STM32F4DataGuard::lockACK()
{
  xSemaphoreTake(this->m_ackLock, portMAX_DELAY);
}

void
STM32F4DataGuard::freeACK()
{
  xSemaphoreGive(this->m_ackLock);
}

void
STM32F4DataGuard::lockProtocolHeader()
{
  xSemaphoreTake(this->m_headerLock, portMAX_DELAY);
}

void
STM32F4DataGuard::freeProtocolHeader()
{
  xSemaphoreGive(this->m_headerLock);
}

void
STM32F4DataGuard::lockNonBlockCBAck()
{
  xSemaphoreTake(this->m_nbAckLock, portMAX_DELAY);
}

void
STM32F4DataGuard::freeNonBlockCBAck()
{
  xSemaphoreGive(this->m_nbAckLock);
}

void
STM32F4DataGuard::lockStopCond()
{
}

void
STM32F4DataGuard::freeStopCond()
{
}

void
STM32F4DataGuard::lockFrame()
{
  xSemaphoreTake(this->m_frameLock, portMAX_DELAY);
}

void
STM32F4DataGuard::freeFrame()
{
  xSemaphoreGive(this->m_frameLock);
}

void
STM32F4DataGuard::notify()
{
  xSemaphoreGive(this->m_ackRecvSem);
}

void
STM32F4DataGuard::notifyNonBlockCBAckRecv()
{
}

void
STM32F4DataGuard::nonBlockWait()
{
}

void
STM32F4DataGuard::wait(int timeoutInSeconds)
{
  xSemaphoreTake(this->m_ackRecvSem, timeoutInSeconds * 1000 * portTICK_PERIOD_MS);
}
