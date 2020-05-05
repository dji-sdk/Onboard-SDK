/*! @file dji_thread_manager.hpp
 *  @version 3.3
 *  @date Jun 2017
 *
 *  @brief Data protection and thread management abstract classes.
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

#ifndef ONBOARDSDK_THREADMANAGER_H
#define ONBOARDSDK_THREADMANAGER_H

namespace DJI
{
namespace OSDK
{

//! Forward Declaration of vehicle
class Vehicle;

//! @todo start use this class
class Mutex
{
public:
  Mutex();
  virtual ~Mutex();

public:
  virtual void lock()   = 0;
  virtual void unlock() = 0;
}; // class Mutex

class ThreadAbstract
{
public:
  ThreadAbstract();
  virtual ~ThreadAbstract();

  //! Mutex operations
public:
  virtual void lockRecvContainer() = 0;
  virtual void freeRecvContainer() = 0;

  virtual void lockMSG() = 0;
  virtual void freeMSG() = 0;

  virtual void lockACK() = 0;
  virtual void freeACK() = 0;

  virtual void lockProtocolHeader();
  virtual void freeProtocolHeader();

  virtual void lockNonBlockCBAck();
  virtual void freeNonBlockCBAck();

  virtual void notifyNonBlockCBAckRecv();
  virtual void nonBlockWait();

  virtual void lockStopCond();
  virtual void freeStopCond();

  virtual void lockFrame();
  virtual void freeFrame();

  //! Thread comm/sync
public:
  virtual void notify()          = 0;
  virtual void wait(int timeout) = 0;

public:
  virtual void init() = 0;
};

class Thread
{
public:
  Thread();
  virtual ~Thread();

  virtual bool createThread() = 0;
  virtual int  stopThread()   = 0;

  bool getStopCondition();

  void setStopCondition(bool condition);

protected:
  Vehicle* vehicle;
  int      type;
  bool     stop_condition;
};

} // namespace DJI
} // namespace OSDK

#endif // ONBOARDSDK_THREADMANAGER_H
