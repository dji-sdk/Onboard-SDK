/*! @file dji_thread_manager.cpp
 *  @version 3.3
 *  @date Jun 2017
 *
 *  @brief Data protection and thread management abstract classes.
 *
 *  @copyright 2017 DJI. All rights reserved.
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
