/*! @file dji_platform_manager.cpp
 *  @version 3.4
 *  @date Dec 2017
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

#include "dji_platform_manager.hpp"

using namespace DJI::OSDK;

PlatformManager::PlatformManager()
{
}

PlatformManager::~PlatformManager()
{
}

Thread*
PlatformManager::addThread(Vehicle* vehicle_ptr, uint8_t thread_type)
{
#ifdef QT
  if (thread_type == UART_SERIAL_READ_THREAD)
  {
    OSDKThread* readThreadPtr =
      new (std::nothrow) OSDKThread(vehicle_ptr, UART_SERIAL_READ_THREAD);
    if (readThreadPtr == NULL)
    {
      DERROR("Failed to initialize read thread!\n");
    }
    else
    {
      QThread* qReadThread = new QThread;
      readThreadPtr->setQThreadPtr(qReadThread);
      readThreadPtr->moveToThread(qReadThread);
      QObject::connect(qReadThread, SIGNAL(started()), readThreadPtr,
                       SLOT(run()));
      QObject::connect(qReadThread, SIGNAL(finished()), qReadThread,
                       SLOT(deleteLater()));
      qReadThread->start();
      return readThreadPtr;
    }
  }

  if (thread_type == CALLBACK_THREAD)
  {
    OSDKThread* cbThreadPtr =
      new (std::nothrow) OSDKThread(vehicle_ptr, CALLBACK_THREAD);
    if (cbThreadPtr == NULL)
    {
      DERROR("Failed to initialize callback thread!\n");
    }
    else
    {
      QThread* qCbThread = new QThread;
      cbThreadPtr->setQThreadPtr(qCbThread);
      cbThreadPtr->moveToThread(qCbThread);
      QObject::connect(qCbThread, SIGNAL(started()), cbThreadPtr, SLOT(run()));
      QObject::connect(qCbThread, SIGNAL(finished()), qCbThread,
                       SLOT(deleteLater()));
      qCbThread->start();
      return cbThreadPtr;
    }
  }

  return NULL;
#elif STM32
  //! Threads not supported by default
  return NULL;
#elif defined(__linux__)
  if (thread_type == CALLBACK_THREAD)
  {
    Thread* callbackThread =
      new (std::nothrow) PosixThread(vehicle_ptr, CALLBACK_THREAD);
    if (callbackThread == NULL)
    {
      DERROR("Failed to initialize read callback thread!\n");
    }
    else
    {
      return callbackThread;
    }
  }

  if (thread_type == UART_SERIAL_READ_THREAD)
  {
    Thread* readThread =
      new (std::nothrow) PosixThread(vehicle_ptr, UART_SERIAL_READ_THREAD);
    if (readThread == NULL)
    {
      DERROR("Failed to initialize read thread!\n");
    }
    else
    {
      return readThread;
    }
  }

  if (thread_type == USB_READ_THREAD)
  {
    Thread* USBreadThread =
      new (std::nothrow) PosixThread(vehicle_ptr, USB_READ_THREAD);
    if (USBreadThread == NULL)
    {
      DERROR("Failed to initialize read thread!\n");
    }
    else
    {
      return USBreadThread;
    }
  }

  return NULL;
#endif
}

HardDriver*
PlatformManager::addHardDriver(uint8_t driver_type, const char* device_port,
                               uint32_t baudrate)
{
#ifdef QT
  if (driver_type == PlatformManager::SERIAL_DEVICE)
  {
    QThread*     serialEventThread = new QThread;
    QHardDriver* driver            = new QHardDriver(0, device_port, baudrate);
    driver->moveToThread(serialEventThread);
    QObject::connect(serialEventThread, SIGNAL(started()), driver,
                     SLOT(init()));
    QObject::connect(driver, SIGNAL(finished()), driver, SLOT(deleteLater()));

    QObject::connect(serialEventThread, SIGNAL(finished()), serialEventThread,
                     SLOT(deleteLater()));
    serialEventThread->start();
    QThread::msleep(100);
    return driver;
  }
  else
  {
    return NULL;
  }
//! Add correct Qt serial device constructor here
#elif STM32
  if (driver_type == PlatformManager::SERIAL_DEVICE)
  {
    STM32F4* stm32f4 = new STM32F4;
    return stm32f4;
  }
  else
  {
    return NULL;
  }
#elif defined(__linux__)
  if (driver_type == PlatformManager::SERIAL_DEVICE)
  {
    LinuxSerialDevice* serialDevice =
      new LinuxSerialDevice(device_port, baudrate);
    return serialDevice;
  }
  else
  {
    return NULL;
  }
#endif
}

ThreadAbstract*
PlatformManager::addThreadHandle()
{
#ifdef QT
  QThreadManager* qThreadManager = new QThreadManager();
  return qThreadManager;
#elif STM32
  STM32F4DataGuard* stm32f4DataGuard = new STM32F4DataGuard;
  return stm32f4DataGuard;
#elif defined(__linux__)
  PosixThreadManager* posixThreadManager = new PosixThreadManager();
  return posixThreadManager;
#endif
}

void
PlatformManager::millisecSleep(int milliseconds)
{
#if STM32
  STM32F4::delay_nms(milliseconds);
#elif defined(QT)
  QThread::msleep(milliseconds);
#elif defined(__linux__)
  usleep(milliseconds * 1000);
#endif
}