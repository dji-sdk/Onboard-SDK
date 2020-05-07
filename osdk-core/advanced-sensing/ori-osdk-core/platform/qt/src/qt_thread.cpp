/** @file qt_thread.cpp
 *  @version 3.4
 *  @date Dec 2017
 *
 *  @brief Threading implementation using Qt constructs
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

#include <dji_vehicle.hpp>
#include <qt_thread.hpp>

using namespace DJI::OSDK;

OSDKThread::OSDKThread(DJI::OSDK::Vehicle* vehicle, int Type, QObject* parent)
{
  this->vehicle = vehicle;
  type          = Type;
  callTimes     = 0;
  this->setStopCondition(false);
  numReceiveSignals = 0;
}

void
OSDKThread::run()
{
  RecvContainer *recvContainer_copy = new RecvContainer();
  while (!(this->getStopCondition()))
  {
    callTimes++;
    if (type == 1)
      vehicle->protocolLayer->sendPoll();
    else if (type == 2)
    {
      callReceivePipeline(recvContainer_copy);
    }
    else if (type == 3)
    {
      vehicle->callbackPoll();
      QThread::msleep(1);
    }
    QThread::usleep(100);
  }
  delete recvContainer_copy;

}

void
OSDKThread::setStopCondition(bool stopCond)
{
  this->stop_condition = stopCond;
}

void
OSDKThread::callReceivePipeline(RecvContainer* recvContainer_copy)
{
  if (this->getStopCondition())
    return;
  RecvContainer* recvFrame = vehicle->protocolLayer->receive();
  if (recvFrame->recvInfo.cmd_id != 0xFF)
  {
    memcpy(recvContainer_copy, recvFrame, sizeof(RecvContainer));
    vehicle->processReceivedData(recvContainer_copy);
  }
}

bool
OSDKThread::createThread()
{
  //  this->start();
  //  return this->isRunning();
  return true;
}

int
OSDKThread::stopThread()
{
  this->setStopCondition(true);
  qThreadPtr->quit();
  return 0;
}

size_t
OSDKThread::getCallTimes() const
{
  return callTimes;
}

void
OSDKThread::setCallTimes(const size_t& value)
{
  callTimes = value;
}

OSDKThread::OSDKThread()
{
  vehicle = 0;
  type    = 0;
}

QThreadManager::~QThreadManager()
{
}

void
QThreadManager::init()
{
}

void
QThreadManager::lockRecvContainer()
{
  m_memLock.lock();
}

void
QThreadManager::freeRecvContainer()
{
  m_memLock.unlock();
}

void
QThreadManager::lockMSG()
{
  m_msgLock.lock();
}

void
QThreadManager::freeMSG()
{
  m_msgLock.unlock();
}

void
QThreadManager::lockACK()
{
  m_ackLock.lock();
}

void
QThreadManager::freeACK()
{
  m_ackLock.unlock();
}

void
QThreadManager::lockProtocolHeader()
{
  m_headerLock.lock();
}

void
QThreadManager::freeProtocolHeader()
{
  m_headerLock.unlock();
}

void
QThreadManager::lockNonBlockCBAck()
{
  m_nbAckLock.lock();
}

void
QThreadManager::freeNonBlockCBAck()
{
  m_nbAckLock.unlock();
}

void
QThreadManager::lockStopCond()
{
  m_stopCondLock.lock();
}

void
QThreadManager::freeStopCond()
{
  m_stopCondLock.unlock();
}

void
QThreadManager::lockFrame()
{
  m_frameLock.lock();
}

void
QThreadManager::freeFrame()
{
  m_frameLock.unlock();
}

void
QThreadManager::notify()
{
  lockACK();
  m_ackRecvCv.wakeAll();
  freeACK();
}

void
QThreadManager::notifyNonBlockCBAckRecv()
{
  m_nbAckRecv.wakeAll();
}

void
QThreadManager::nonBlockWait()
{
  m_nbAckRecv.wait(&m_nbAckLock);
}

void
QThreadManager::wait(int timeoutInSeconds)
{
  unsigned long timeout_ms = 1000 * timeoutInSeconds;
  m_ackRecvCv.wait(&m_ackLock, timeout_ms);
}
