/**
 ********************************************************************
 * @file    osdkhal_linux.c
 * @version V1.0.0
 * @date    2019/09/25
 * @brief
 *
 * @copyright (c) 2018-2019 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "osdkhal_linux.h"

/* Exported functions definition ---------------------------------------------*/

/**
 * @brief Uart interface send function.
 * @param obj: pointer to the hal object, which including uart interface parameters.
 * @param pBuf:  pointer to the buffer which is used to store send data.
 * @param bufLen:  send data length.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_UartSendData(const T_HalObj *obj, const uint8_t *pBuf,
                                  uint32_t bufLen) {
  int32_t realLen;

  if ((obj == NULL) || (obj->uartObject.fd == -1)) {
    return OSDK_STAT_ERR;
  }

  realLen = write(obj->uartObject.fd, pBuf, bufLen);
  if (realLen == bufLen) {
    return OSDK_STAT_OK;
  } else {
    return OSDK_STAT_ERR;
  }
}

/**
 * @brief Uart interface send function.
 * @param obj: pointer to the hal object, which including uart interface parameters.
 * @param pBuf:  pointer to the buffer which is used to store receive data.
 * @param bufLen:  receive data length.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_UartReadData(const T_HalObj *obj, uint8_t *pBuf,
                                  uint32_t *bufLen) {
  if ((obj == NULL) || (obj->uartObject.fd == -1)) {
    return OSDK_STAT_ERR;
  }
  *bufLen = read(obj->uartObject.fd, pBuf, 1024);

  return OSDK_STAT_OK;
}

/**
 * @brief Uart interface close function.
 * @param obj: pointer to the hal object, which including uart interface parameters.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_UartClose(T_HalObj *obj) {
  if ((obj == NULL) || (obj->uartObject.fd == -1)) {
    return OSDK_STAT_ERR;
  }
  close(obj->uartObject.fd);

  return OSDK_STAT_OK;
}

/**
 * @brief Uart interface init function.
 * @param port: uart interface port.
 * @param baudrate:  uart interface baudrate.
 * @param obj: pointer to the hal object, which is used to store uart interface parameters.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_UartInit(const char *port, const int baudrate,
                              T_HalObj *obj) {
  int st_baud[] = {B4800,   B9600,   B19200,  B38400,   B57600,   B115200,
                   B230400, B460800, B921600, B1000000, B1152000, B3000000};
  int std_rate[] = {4800,   9600,   19200,  38400,   57600,   115200,
                    230400, 460800, 921600, 1000000, 1152000, 3000000};

  struct termios options;
  E_OsdkStat OsdkStat = OSDK_STAT_OK;
  int i = 0;

  if (!port) {
    return OSDK_STAT_ERR_PARAM;
  }

  obj->uartObject.fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  if (obj->uartObject.fd == -1) {
    OsdkStat = OSDK_STAT_ERR;
    goto out;
  }

  if (tcgetattr(obj->uartObject.fd, &options) != 0) {
    close(obj->uartObject.fd);
    OsdkStat = OSDK_STAT_ERR;

    goto out;
  }

  for (i = 0; i < sizeof(std_rate) / sizeof(int); ++i) {
    if (std_rate[i] == baudrate) {
      /* set standard baudrate */
      cfsetispeed(&options, st_baud[i]);
      cfsetospeed(&options, st_baud[i]);
      break;
    }
  }
  if (i == sizeof(std_rate) / sizeof(int)) {
    close(obj->uartObject.fd);
    OsdkStat = OSDK_STAT_ERR;

    goto out;
  }

  options.c_cflag |= CLOCAL;
  options.c_cflag |= CREAD;
  options.c_cflag &= ~CRTSCTS;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~PARENB;
  options.c_iflag &= ~INPCK;
  options.c_cflag &= ~CSTOPB;
  options.c_oflag &= ~OPOST;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  options.c_cc[VTIME] = 0;
  options.c_cc[VMIN] = 0;

  tcflush(obj->uartObject.fd, TCIFLUSH);

  if (tcsetattr(obj->uartObject.fd, TCSANOW, &options) != 0) {
    close(obj->uartObject.fd);
    OsdkStat = OSDK_STAT_ERR;

    goto out;
  }

  out:

  return OsdkStat;
}

#ifdef ADVANCED_SENSING

/**
 * @brief USBBulk interface init function.
 * @param pid: USBBulk product id.
 * @param vid: USBBulk vendor id.
 * @param num: USBBulk interface num.
 * @param epIn: USBBulk input endpoint .
 * @param epOut: USBBulk output endpoint.
 * @param obj: pointer to the hal object, which is used to store USBBulk interface parameters.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_USBBulkInit(uint16_t pid, uint16_t vid, uint16_t num, uint16_t epIn,
                                 uint16_t epOut, T_HalObj *obj) {
  struct libusb_device_handle *handle = NULL;

  int ret = libusb_init(NULL);
  if(ret < 0) {
    return OSDK_STAT_ERR;
  }

  handle = libusb_open_device_with_vid_pid(NULL, vid, pid);
  if (!handle)
  {
    return OSDK_STAT_ERR;
  }

  ret = libusb_claim_interface(handle, num);
  if(ret != LIBUSB_SUCCESS){
    libusb_close(handle);
    return OSDK_STAT_ERR;
  }

  obj->bulkObject.handle = (void *)handle;
  obj->bulkObject.epIn = epIn;
  obj->bulkObject.epOut = epOut;

  return OSDK_STAT_OK;
}


E_OsdkStat OsdkLinux_USBBulkSendData(const T_HalObj *obj, const uint8_t *pBuf,
                                     uint32_t bufLen) {
  struct libusb_device_handle *handle = NULL;
  int sent_len = 0, ret;
  
  if((obj == NULL) || (obj->bulkObject.handle == NULL)) {
    return OSDK_STAT_ERR; 
  }

  handle = (struct libusb_device_handle *)obj->bulkObject.handle;
  ret = libusb_bulk_transfer(handle, obj->bulkObject.epOut,
                             (uint8_t *)pBuf, bufLen,
                             &sent_len, 50);
  if (0 != ret){
    OsdkLinux_USBBulkSendData(obj, pBuf, bufLen);
    return OSDK_STAT_ERR;
  }
  return OSDK_STAT_OK;
}


E_OsdkStat OsdkLinux_USBBulkReadData(const T_HalObj *obj, uint8_t *pBuf,
                                     uint32_t *bufLen) {
  struct libusb_device_handle *handle = NULL;
  int ret;
  
  if((obj == NULL) || (obj->bulkObject.handle == NULL)) {
    return OSDK_STAT_ERR; 
  }

  handle = (struct libusb_device_handle *)obj->bulkObject.handle;
  ret = libusb_bulk_transfer(handle, obj->bulkObject.epIn,
                             pBuf, *bufLen, bufLen, (unsigned int)(-1));
  if (ret != 0) {
    if (-7 == ret)
      return OSDK_STAT_ERR_TIMEOUT;
    else
      return OSDK_STAT_ERR;
  }

  if(*bufLen == 0) {
    return OSDK_STAT_ERR;
  }

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkLinux_USBBulkClose(T_HalObj *obj) {
  struct libusb_device_handle *handle = NULL;
  
  if((obj == NULL) || (obj->bulkObject.handle == NULL)) {
    return OSDK_STAT_ERR; 
  }

  handle = (struct libusb_device_handle *)obj->bulkObject.handle;
  libusb_close(handle);
  return OSDK_STAT_OK;
}

#endif

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
