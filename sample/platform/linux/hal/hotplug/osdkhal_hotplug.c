/* udev_example1.c
 *
 * Copyright (C) 2014 Robert Milasan <rmilasan@suse.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This example will get basic information about a specified network
 * device using libudev API.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include "sys/stat.h"
#include "osdkhal_hotplug.h"

#define REMOVE_ACTION "remove"
#define ADD_ACTION    "add"

DevFilter getDevFilter(const char *devName) {
  DevFilter filter = {false, {0}};
  struct udev *udev;
  struct udev_device *dev;
  /* create udev object */
  udev = udev_new();
  if (!udev) {
    fprintf(stderr, "Cannot create udev context.\n");
    return filter;
  }

  /* get device based on path */
  struct stat statBuf;
  int ret = stat(devName, &statBuf);
  //printf("ret = %d\n", ret);
  //printf("statBuf.st_rdev = %ld\n", statBuf.st_rdev);
  dev = udev_device_new_from_devnum(udev, 'c', statBuf.st_rdev);
  if (!dev) {
    fprintf(stderr, "Failed to get device.\n");
    return filter;
  }

  //printf("I: DEVNUM=%lu\n", udev_device_get_devnum(dev));
  //printf("I: DEVNODE=%s\n", udev_device_get_devnode(dev));
  //printf("I: DEVNAME=%s\n", udev_device_get_sysname(dev));
  //printf("I: DEVPATH=%s\n", udev_device_get_devpath(dev));
  const char *devPath = udev_device_get_devpath(dev);
  if (devPath) {
    char *addr = strstr(devPath, "tty");
    if (addr && (addr < (devPath + strlen(devPath)))) {
      strncpy(filter.uartFilter.devPathHeader,
              devPath,
              (addr - devPath) > sizeof(filter.uartFilter.devPathHeader) ? sizeof(filter.uartFilter.devPathHeader) : (addr - devPath));
      filter.valid = true;
      //printf("---> devPathHeader : %s\n", filter.uartFilter.devPathHeader);
    }
  }

  /* free dev */
  udev_device_unref(dev);
  /* free udev */
  udev_unref(udev);

  return filter;
}

void* hotplugThread(void *arg) {
  if (!arg) return NULL;
  HotplugHandler handler = *(HotplugHandler *)arg;
  if (!handler.filter.valid || !handler.callback) {
    printf("hotplugThread : error handler");
    return NULL;
  } else {
    HotPlug_Log("Hotplug thread for subSystem[%s] created.\n", handler.subsystem);
  }

  struct udev *udev;
  struct udev_device *dev;
  /* create udev object */
  udev = udev_new();
  if (!udev) {
    fprintf(stderr, "Cannot create udev context.\n");
    return NULL;
  }
  struct udev_monitor *mon = udev_monitor_new_from_netlink(udev, "udev");
  udev_monitor_filter_add_match_subsystem_devtype(mon, handler.subsystem, NULL);
  udev_monitor_enable_receiving(mon);
  int fd = udev_monitor_get_fd(mon);

  fd_set fds;
  struct timeval tv;
  int ret;

  for(;;) {
    uint32_t pollingMs = 100;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    tv.tv_sec = 0;
    tv.tv_usec = pollingMs * 1000;

    ret = select(fd + 1, &fds, NULL, NULL, &tv);
    if (ret > 0 && FD_ISSET(fd, &fds)) {
      dev = udev_monitor_receive_device(mon);
      if (dev) {
        if (handler.callback)
          handler.callback(dev, &handler);
        /* free dev */
        udev_device_unref(dev);
      }
    }
  }
  /* free dev */
  udev_device_unref(dev);
  /* free udev */
  udev_unref(udev);

  free(arg);
  return NULL;
}

