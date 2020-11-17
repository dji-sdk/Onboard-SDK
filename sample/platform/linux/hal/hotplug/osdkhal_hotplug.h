//
// Created by dji on 9/25/20.
//

#ifndef ONBOARDSDK_SAMPLE_PLATFORM_LINUX_HAL_OSDKHAL_HOTPLUG_H_
#define ONBOARDSDK_SAMPLE_PLATFORM_LINUX_HAL_OSDKHAL_HOTPLUG_H_

#include <libudev.h>
#include "string.h"
#include "stdbool.h"
#include "stdint.h"
#include "osdk_platform.h"

typedef struct DevFilter {
  bool valid;
  union {
    struct {
      char devPathHeader[100];
    } uartFilter;
    struct {
      uint16_t pid;
      uint16_t vid;
      uint16_t num;
    } usbBulkFilter;
  };
} DevFilter;

typedef struct DeviceInitParam
{
  T_HalObj* obj;
  union {
    struct {
      char port[100];
      int  baudrate;
    } uartInitParam;
    struct {
      uint16_t pid;
      uint16_t vid;
      uint16_t num;
      uint16_t epIn;
      uint16_t epOut;
    } bulkInitParam;
  };
} DeviceInitParam;

typedef struct HotplugHandler {
  const char* subsystem;
  DeviceInitParam param;
  DevFilter filter;
  void (*callback)(struct udev_device *dev, struct HotplugHandler *handler);
} HotplugHandler;

DevFilter getDevFilter(const char *devName);
void hotplugMonitor(HotplugHandler handler);
void* hotplugThread(void *arg);

#endif // ONBOARDSDK_SAMPLE_PLATFORM_LINUX_HAL_OSDKHAL_HOTPLUG_H_
