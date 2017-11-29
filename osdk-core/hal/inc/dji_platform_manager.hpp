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
