/** @file qt_thread.hpp
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

#ifndef QT_THREAD
#define QT_THREAD

#include "dji_thread_manager.hpp"
#include <QMutex>
#include <QSerialPort>
#include <QThread>
#include <QWaitCondition>

// Forward Declaration
namespace DJI
{
namespace OSDK
{
  struct RecvContainer;
}
}


// Let's not subclass OSDKThread from Qthread; this violates event loop
// constructs of Qt. Instead, let's moveToThread(OSDKThread).
class OSDKThread : public QObject, public DJI::OSDK::Thread
{
  Q_OBJECT

public:
  OSDKThread();
  OSDKThread(DJI::OSDK::Vehicle* vehicle, int Type, QObject* parent = 0);

  void callReceivePipeline(DJI::OSDK::RecvContainer* recvContainer_copy);
  bool createThread();
  int  stopThread();

  size_t getCallTimes() const;
  void setCallTimes(const size_t& value);
  void setQThreadPtr(QThread* threadPtr)
  {
    this->qThreadPtr = threadPtr;
  }

private slots:
  void run();

private:
  QThread* qThreadPtr;
  int      numReceiveSignals;
  size_t   callTimes;
  void setStopCondition(bool stopCond);
};

/*! @brief Qt-style Data Protection and Condition Variables for
 * Windows/unix/linux
 * platforms
 *
 */
class QThreadManager : public DJI::OSDK::ThreadAbstract
{
public:
  QThreadManager()
  {
  }
  ~QThreadManager();

  void init();

  //! Implementing virtual functions from ThreadManager
public:
  void lockRecvContainer();
  void freeRecvContainer();

  void lockMSG();
  void freeMSG();

  void lockACK();
  void freeACK();

  void lockProtocolHeader();
  void freeProtocolHeader();

  void lockNonBlockCBAck();
  void freeNonBlockCBAck();

  void lockStopCond();
  void freeStopCond();

  void lockFrame();
  void freeFrame();

  void notify();
  void notifyNonBlockCBAckRecv();
  void wait(int timeoutInSeconds);
  void nonBlockWait();

private:
  QMutex         m_memLock;
  QMutex         m_msgLock;
  QMutex         m_ackLock;
  QWaitCondition m_ackRecvCv;

  QMutex         m_headerLock;
  QMutex         m_nbAckLock;
  QWaitCondition m_nbAckRecv;

  //! Thread protection for setting stop condition for threads
  QMutex m_stopCondLock;

  //! Thread protection for last received frame storage
  QMutex m_frameLock;
};

#endif // QT_THREAD
