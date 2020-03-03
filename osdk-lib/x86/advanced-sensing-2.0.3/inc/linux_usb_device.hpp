/*! @file linux_usb_device.hpp
 *  @version 3.4.0
 *  @date Dec 2017
 *
 *
 *  @Copyright (c) 2016-17 DJI.
 *  NOTE THAT this file is part of the advanced sensing
 *  closed-source library. For licensing information,
 *  please visit https://developer.dji.com/policies/eula/
 * */

#ifndef ONBOARDSDK_LINUX_USB_DEVICE_H
#define ONBOARDSDK_LINUX_USB_DEVICE_H

#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <libusb.h>

#include "dji_hard_driver.hpp"


namespace DJI
{

namespace OSDK
{

/*! @brief POSIX-Compatible USB Driver for *NIX platforms
 *
 */
class LinuxUSBDevice : public HardDriver
{
  static const int POSSIBLE_DJI_DEVICE_NUM  = 6;
  static const int TIMEOUT                  = 50;
  static const int OUT_END_PT               = 0x0A;
  static const int IN_END_PT                = 0x84;

  typedef struct USBFilter
  {
    uint16_t  vid;
    uint16_t  pid;
  } USBFilter;


public:
//  static const int BUFFER_SIZE = 2048;

public:
//  LinuxUSBDevice(const char* device, uint32_t baudrate);
  LinuxUSBDevice();
  ~LinuxUSBDevice();

  void init();
  bool isDJIUSBDevice(uint16_t idVendor, uint16_t idProduct);

  //! Start of DJI_HardDriver virtual function implementations
  size_t send(const uint8_t* buf, size_t len);
  size_t readall(uint8_t* buf, size_t maxlen);

  time_ms getTimeStamp();
private:
  libusb_device*        DJI_device;
  libusb_device_handle* DJI_dev_handle;
  int                   DJI_device_product_id;
  USBFilter             DJI_usb_dev_filter[POSSIBLE_DJI_DEVICE_NUM];

  bool                  deviceStatus;
  bool                  foundDJIDevice;
};
}
}


#endif //ONBOARDSDK_LINUX_USB_DEVICE_H
