/*
 * DJI Onboard SDK Advanced Sensing APIs
 *
 * Copyright (c) 2017-2018 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 */

#include "linux_usb_device.hpp"
#include <algorithm>
#include <iterator>

#include "iostream"

using namespace DJI::OSDK;

LinuxUSBDevice::LinuxUSBDevice() :
  foundDJIDevice(false)
{
  DJI_usb_dev_filter[0].pid = 0x001F;  DJI_usb_dev_filter[0].vid = 0xFFF0;
  DJI_usb_dev_filter[1].pid = 0x0020;  DJI_usb_dev_filter[1].vid = 0xFFF0;
  DJI_usb_dev_filter[2].pid = 0xd008;  DJI_usb_dev_filter[2].vid = 0x18d1;
  DJI_usb_dev_filter[3].pid = 0xd009;  DJI_usb_dev_filter[3].vid = 0x18d1;
  DJI_usb_dev_filter[4].pid = 0x001F;  DJI_usb_dev_filter[4].vid = 0x2CA3;
  DJI_usb_dev_filter[5].pid = 0x0020;  DJI_usb_dev_filter[5].vid = 0x2CA3;
}

LinuxUSBDevice::~LinuxUSBDevice()
{
  // @todo maybe there's more steps
  libusb_close(DJI_dev_handle);
}

void
LinuxUSBDevice::init()
{
  DSTATUS("Looking for USB device...\n");

  libusb_context *ctx = NULL;
  int ret = libusb_init(&ctx);
  if(ret < 0) {
    DERROR("Failed to Initialized libusb session...\n");
    return;
  }

  libusb_device **devs;
  ret = libusb_get_device_list(NULL, &devs);
  if(ret < 0) {
    DERROR("....Failed to get any USB Device\n");
    deviceStatus = false;
    return;
  }else{
    DSTATUS("Found %d USB devices, identifying DJI device...\n", ret);
  }

  for (int i = 0; NULL != devs[i]; ++i)
  {
    libusb_device *dev = devs[i];
    struct libusb_device_descriptor desc;
    ret = libusb_get_device_descriptor(dev, &desc);
    if (ret < 0)
    {
      DDEBUG("Failed to get device descriptor\n");
      continue;
    }

    if (isDJIUSBDevice(desc.idVendor, desc.idProduct))
    {
      DJI_device = dev;
      DJI_device_product_id = desc.idProduct;
      DSTATUS("Found a DJI device...\n");
      this->foundDJIDevice = true;
    }
  }

  if (!this->foundDJIDevice){
    DERROR("Did not find any DJI USB device, "
             "please double-check your connection.\n");
    DERROR("Some helpful commands are 'lsusb' and 'dmesg'.\n");
    deviceStatus = false;
    return;
  }

  DSTATUS("Attempting to open DJI USB device...\n");

  ret = libusb_open(DJI_device, &DJI_dev_handle);
  if (ret < 0)
  {
    DERROR("Failed to open DJI USB device...\n");
    DERROR("Error code: %d", ret);
    if (ret == LIBUSB_ERROR_ACCESS)
    {
      DERROR("Please make sure you provide a udev file for your system and reboot the computer");
    }
    DJI_dev_handle = NULL;
    deviceStatus = false;
    return;
  }
  libusb_free_device_list(devs, 1);

  int interface_num;
  if(0xd008 == DJI_device_product_id)
    interface_num = 1;
  else if(0x001f == DJI_device_product_id || 0x0020 == DJI_device_product_id)
    interface_num = 3;
  else
    interface_num = 0;

  ret = libusb_claim_interface(DJI_dev_handle, interface_num);
  if(ret != LIBUSB_SUCCESS){
    libusb_detach_kernel_driver(DJI_dev_handle, 0);
    ret = libusb_claim_interface(DJI_dev_handle, interface_num);
    if(ret != LIBUSB_SUCCESS){
      DERROR("Failed to claim DJI USB device interface... %d\n", ret);
      libusb_detach_kernel_driver(DJI_dev_handle, 0);
      libusb_close(DJI_dev_handle);
      deviceStatus = false;
      return;
    }
  }

  DSTATUS("...DJI USB device started successfully.\n");
  deviceStatus = true;
}

bool
LinuxUSBDevice::isDJIUSBDevice(uint16_t idVendor, uint16_t idProduct)
{
  for (int i = 0; i < POSSIBLE_DJI_DEVICE_NUM; ++i)
  {
    if (idVendor == DJI_usb_dev_filter[i].vid &&
      idProduct == DJI_usb_dev_filter[i].pid)
      return true;
  }
  return false;
}

// @note libusb_bulk_transfer return sending result instead of sent length
size_t
LinuxUSBDevice::send(const uint8_t* buf, size_t len)
{
  static int retry_count = 0;
  int sent_len = 0, ret;
  ret = libusb_bulk_transfer(DJI_dev_handle, OUT_END_PT,
                             const_cast<uint8_t*>(buf), (int)len,
                             &sent_len, TIMEOUT);
  if (0 == ret){
    retry_count = 0;
    return (size_t)sent_len;
  }
  else{
    if(retry_count > 3){
      return -1;
    }
    DERROR("LIBUSB send error, retry %d times", ++retry_count);
    send(buf, len);
  }
  return (size_t)-1;
}

size_t
LinuxUSBDevice::readall(uint8_t* buf, size_t maxlen)
{
  int read_len = 0, ret;
  ret = libusb_bulk_transfer(DJI_dev_handle, IN_END_PT,
                             buf, maxlen, &read_len, TIMEOUT);
  if (0 == ret)
    return (size_t)read_len;

  return (size_t)-1;
}

time_ms
LinuxUSBDevice::getTimeStamp()
{
  return (uint32_t)time(NULL);
}