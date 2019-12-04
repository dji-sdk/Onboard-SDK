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
                                uint16_t bufLen) {
  int32_t realLen;

  if (obj->uartObject.fd == -1) {
    OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "uart fd error");
    return OSDK_STAT_ERR;
  }

  realLen = write(obj->uartObject.fd, pBuf, bufLen);
  if (realLen == bufLen) {
    return OSDK_STAT_OK;
  } else {
    OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "uart send error");
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
                                uint16_t *bufLen) {
  if (obj->uartObject.fd == -1) {
    OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "uart fd error");
    return OSDK_STAT_ERR;
  }

  *bufLen = read(obj->uartObject.fd, pBuf, 1024);

  return OSDK_STAT_OK;
}

/**
 * @brief Udp interface send function.
 * @param obj: pointer to the hal object, which including udp interface parameters.
 * @param pBuf:  pointer to the buffer which is used to store send data.
 * @param bufLen:  send data length.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_UdpSendData(const T_HalObj *obj, const uint8_t *pBuf,
                               uint16_t bufLen) {
  int32_t realLen;

  if (obj->udpObject.fd == -1) {
    OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "udp fd error");
    return OSDK_STAT_ERR;
  }

  realLen = sendto(obj->udpObject.fd, pBuf, bufLen, 0,
                   (struct sockaddr *)&obj->udpObject.socketAddr,
                   sizeof(struct sockaddr_in));

  if (realLen == bufLen) {
    return OSDK_STAT_OK;
  } else {
    return OSDK_STAT_ERR;
  }
}

/**
 * @brief Udp interface receive function.
 * @param obj: pointer to the hal object, which including udp interface parameters.
 * @param pBuf:  pointer to the buffer which is used to store receive data.
 * @param bufLen:  receive data length.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_UdpReadData(const T_HalObj *obj, uint8_t *pBuf,
                               uint16_t *bufLen) {
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
    OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "uart device %s open error", port);
    OsdkStat = OSDK_STAT_ERR;
    goto out;
  }

  if (tcgetattr(obj->uartObject.fd, &options) != 0) {
    close(obj->uartObject.fd);
    OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "uart device getattr error");
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
    OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "invalid baud param");
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
    OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "uart device setattr error");
    OsdkStat = OSDK_STAT_ERR;

    goto out;
  }

out:

  return OsdkStat;
}

/**
 * @brief Udp interface init function.
 * @param addr: udp interface address.
 * @param port:  udp interface port number.
 * @param obj: pointer to the hal object, which is used to store udp interface parameters.
 * @return an enum that represents a status of OSDK
 */
E_OsdkStat OsdkLinux_UdpInit(const char *addr, uint16_t port, T_HalObj *obj) {
  if ((obj->udpObject.fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    OSDK_LOG_ERROR(MODULE_NAME_PLATFORM, "Osdk udp socket init error!");
    return OSDK_STAT_SYS_ERR;
  }

  bzero(&obj->udpObject.socketAddr, sizeof(struct sockaddr_in));
  obj->udpObject.socketAddr.sin_family = AF_INET;
  obj->udpObject.socketAddr.sin_addr.s_addr = inet_addr(addr);
  obj->udpObject.socketAddr.sin_port = htons(port);

  return OSDK_STAT_OK;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
