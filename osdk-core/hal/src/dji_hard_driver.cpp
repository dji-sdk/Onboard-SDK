/*! @file dji_hard_driver.cpp
 *  @version 3.3
 *  @date Jun 15 2017
 *
 *  @brief
 *  Serial device driver abstraction. Provided as an abstract class. Please
 *  inherit and implement for individual platforms.
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

#include "dji_hard_driver.hpp"

using namespace DJI::OSDK;

//! @todo change to dji_logging method
char DJI::OSDK::buffer[DJI::OSDK::HardDriver::bufsize];

HardDriver::HardDriver()
{
}

HardDriver::~HardDriver()
{
}

void
HardDriver::displayLog(const char* buf)
{
  if (buf)
    DDEBUG("%s", buf);
  else
    DDEBUG("%s", DJI::OSDK::buffer);
}