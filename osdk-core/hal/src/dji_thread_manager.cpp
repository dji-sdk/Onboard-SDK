/*! @file dji_thread_manager.cpp
 *  @version 3.3
 *  @date Jun 2017
 *
 *  @brief Data protection and thread management abstract classes.
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

#include "dji_thread_manager.hpp"

using namespace DJI;
using namespace DJI::OSDK;

Thread::Thread()
{
}

Thread::~Thread()
{
}

bool
Thread::getStopCondition()
{
  return this->stop_condition;
}

void
Thread::setStopCondition(bool condition)
{
  this->stop_condition = condition;
}

ThreadAbstract::ThreadAbstract()
{
}

ThreadAbstract::~ThreadAbstract()
{
}

void
ThreadAbstract::lockProtocolHeader()
{
  ;
}

void
ThreadAbstract::freeProtocolHeader()
{
  ;
}

void
ThreadAbstract::lockNonBlockCBAck()
{
  ;
}

void
ThreadAbstract::freeNonBlockCBAck()
{
  ;
}

void
ThreadAbstract::notifyNonBlockCBAckRecv()
{
  ;
}

void
ThreadAbstract::nonBlockWait()
{
  ;
}

void
ThreadAbstract::lockStopCond()
{
  ;
}

void
ThreadAbstract::freeStopCond()
{
  ;
}

void
ThreadAbstract::lockFrame()
{
  ;
}

void
ThreadAbstract::freeFrame()
{
  ;
}

Mutex::Mutex()
{
}

Mutex::~Mutex()
{
}
