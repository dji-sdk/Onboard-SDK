/*! @file dji_platform_manager.hpp
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

#ifndef ONBOARDSDK_DJI_PLATFORM_MANAGER_H
#define ONBOARDSDK_DJI_PLATFORM_MANAGER_H

#include "dji_hard_driver.hpp"

#include "dji_protocol_base.hpp"
#include "dji_thread_manager.hpp"
#include <new>

#ifdef QT
#include <qt_serial_device.hpp>
#include <qt_thread.hpp>
#elif defined(__linux__)
#include "linux_serial_device.hpp"
#include "posix_thread.hpp"
#include "posix_thread_manager.hpp"
#elif STM32
#include "STM32F4DataGuard.h"
#include "STM32F4SerialDriver.h"
#endif

namespace DJI
{
namespace OSDK
{

//! Forward declaration
class Vehicle;

class PlatformManager
{
public:
  enum ThreadType
  {
    SEND_THREAD             = 1,
    UART_SERIAL_READ_THREAD = 2,
    CALLBACK_THREAD         = 3,
    USB_SERIAL_READ_THREAD  = 4,
    USB_READ_THREAD         = 5,
  };

  enum HardDriverType
  {
    SERIAL_DEVICE = 1,
    USB_DEVICE    = 2,
  };

public:
  PlatformManager();
  ~PlatformManager();

public:
  Thread* addThread(Vehicle* vehicle_ptr, uint8_t thread_type);

  HardDriver* addHardDriver(uint8_t driver_type, const char* device_port = NULL,
                            uint32_t baudrate = 0);

  ThreadAbstract* addThreadHandle();

  void millisecSleep(int milliseconds);
};

} // OSDK
} // DJI

#endif // ONBOARDSDK_DJI_PLATFORM_MANAGER_H
