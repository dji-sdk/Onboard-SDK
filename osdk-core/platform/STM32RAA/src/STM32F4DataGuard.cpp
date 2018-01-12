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
}

void
STM32F4DataGuard::init()
{
}

void
STM32F4DataGuard::lockRecvContainer()
{
}

void
STM32F4DataGuard::freeRecvContainer()
{
}

void
STM32F4DataGuard::lockMSG()
{
}

void
STM32F4DataGuard::freeMSG()
{
}

void
STM32F4DataGuard::lockACK()
{
}

void
STM32F4DataGuard::freeACK()
{
}

void
STM32F4DataGuard::lockProtocolHeader()
{
}

void
STM32F4DataGuard::freeProtocolHeader()
{
}

void
STM32F4DataGuard::lockNonBlockCBAck()
{
}

void
STM32F4DataGuard::freeNonBlockCBAck()
{
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
}

void
STM32F4DataGuard::freeFrame()
{
}

void
STM32F4DataGuard::notify()
{
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
}
