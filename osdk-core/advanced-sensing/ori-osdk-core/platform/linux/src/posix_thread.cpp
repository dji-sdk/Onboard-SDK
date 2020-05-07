/*! @file posix_thread.cpp
 *  @version 3.3
 *  @date Jun 15 2017
 *
 *  @brief
 *  Pthread-based threading for DJI Onboard SDK on linux platforms
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

#include "posix_thread.hpp"
#include "dji_vehicle.hpp"

using namespace DJI::OSDK;

PosixThread::PosixThread()
{
  this->vehicle        = 0;
  this->type           = 0;
  this->stop_condition = false;
}

PosixThread::PosixThread(Vehicle* vehicle, int Type)
{
  this->vehicle        = vehicle;
  this->type           = Type;
  this->stop_condition = false;
}

bool
PosixThread::createThread()
{
  int         ret = -1;
  std::string infoStr;

  /* Initialize and set thread detached attribute */
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

  if (1 == type)
  {
    ret     = pthread_create(&threadID, NULL, send_call, (void*)vehicle);
    infoStr = "sendPoll";
  }
  else if (2 == type)
  {
    ret =
      pthread_create(&threadID, NULL, uart_serial_read_call, (void*)vehicle);
    infoStr = "readPoll";
  }
  else if (3 == type)
  {
    ret     = pthread_create(&threadID, NULL, callback_call, (void*)vehicle);
    infoStr = "callback";
  }
  else if (5 == type)
  {
    ret     = pthread_create(&threadID, NULL, USB_read_call, (void*)vehicle);
    infoStr = "USBReadPoll";
  }
  else
  {
    infoStr = "error type number";
  }

  if (0 != ret)
  {
    DERROR("fail to create thread for %s!\n", infoStr.c_str());
    return false;
  }

  ret = pthread_setname_np(threadID, infoStr.c_str());
  if (0 != ret)
  {
    DERROR("fail to set thread name for %s!\n", infoStr.c_str());
    return false;
  }
  return true;
}

int
PosixThread::stopThread()
{
  int   ret = -1;
  void* status;
  this->stop_condition = true;

  /* Free attribute and wait for the other threads */
  if (int i = pthread_attr_destroy(&attr))
  {
    DERROR("Failed to destroy thread %d\n", i);
  }
  else
  {
    DDEBUG("Succeeded to destroy thread\n");
  }

  ret = pthread_join(threadID, &status);

  if (ret)
  {
    DDEBUG("Join thread error: %d\n", ret);
    return ret;
  }

  return 0;
}

void*
PosixThread::send_call(void* param)
{
  Vehicle* vehiclePtr = (Vehicle*)param;
  while (true)
  {
    vehiclePtr->protocolLayer->sendPoll();
    usleep(10); //! @note CPU optimization, reduce the CPU usage a lot
  }
}

void*
PosixThread::uart_serial_read_call(void* param)
{
  RecvContainer* recvContainer_copy = new RecvContainer();
  Vehicle*       vehiclePtr         = (Vehicle*)param;
  while (!(vehiclePtr->getSerialReadThread()->getStopCondition()))
  {
    // receive() implemented on the OpenProtocolCMD side
    // do a copy here to prevent low level changes to the ptr
    RecvContainer* recvContainer = vehiclePtr->protocolLayer->receive();

    if (recvContainer->recvInfo.cmd_id != 0xFF)
    {
      vehiclePtr->protocolLayer->getThreadHandle()->lockRecvContainer();
      memcpy(recvContainer_copy, recvContainer, sizeof(RecvContainer));
      vehiclePtr->protocolLayer->getThreadHandle()->freeRecvContainer();
      vehiclePtr->processReceivedData(recvContainer_copy);
    }
    usleep(10); //! @note CPU optimization, reduce the CPU usage a lot
  }

  delete recvContainer_copy;
  DDEBUG("Quit read function\n");
}

void*
PosixThread::USB_read_call(void* param)
{
  RecvContainer* recvContainer = new RecvContainer();
  Vehicle*       vehiclePtr    = (Vehicle*)param;
#ifdef ADVANCED_SENSING
  while (!vehiclePtr->getUSBReadThread()->getStopCondition())
  {
    if (vehiclePtr->isUSBThreadReady())
    {
      recvContainer = vehiclePtr->advancedSensing->getAdvancedSensingProtocol()->receive();

      if (recvContainer->recvInfo.cmd_id != 0xFF)
      {
        vehiclePtr->processAdvancedSensingImgs(recvContainer);
      }
    }
    usleep(10);
  }
#endif

  delete recvContainer;
  DDEBUG("Quit USB read function\n");
}

void*
PosixThread::callback_call(void* param)
{
  Vehicle* vehiclePtr = (Vehicle*)param;
  while (!(vehiclePtr->getCallbackThread()->getStopCondition()))
  {
    vehiclePtr->callbackPoll();
    usleep(10); //! @note CPU optimization, reduce the CPU usage a lot
  }
  DDEBUG("Quit callback function\n");
}
