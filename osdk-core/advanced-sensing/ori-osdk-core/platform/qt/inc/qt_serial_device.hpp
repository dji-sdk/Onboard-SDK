/*! @file DJI_HardDriver_Qt.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Serial device hardware abstraction for DJI Onboard SDK Qt example.
 *
 *  @note New Qt sample coming soon!
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

#ifndef QONBOARDSDK_H
#define QONBOARDSDK_H

#include "dji_hard_driver.hpp"

#include <QComboBox>
#include <QItemDelegate>
#include <QMutex>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTextBrowser>

class QHardDriver : public QObject, public DJI::OSDK::HardDriver
{
  Q_OBJECT

public:
  explicit QHardDriver(QObject* parent, const char* portName = 0,
                       int baudrate = 230400);
  QHardDriver(QObject* parent = 0, QSerialPort* extSerialPort = 0);
  ~QHardDriver()
  {
  }

  DJI::OSDK::time_ms getTimeStamp();
  size_t send(const uint8_t* buf, size_t len);
  size_t readall(uint8_t* buf, size_t maxlen);
  bool getDeviceStatus();
  void setDeviceStatus(bool status);

  void setBaudrate(int value);

  QTextBrowser* getDisplay() const;
  void setDisplay(QTextBrowser* value);

  QSerialPort* port;


private:
  QHardDriver();

public slots:
  void init();

private:
  bool          initStatus;
  int           baudrate;
  QMutex        statusLock;
  QMutex        initLock;
  QMutex        sendLock;
  QMutex        bufferLock;
  QTextBrowser* display;
  QString       portName;
};

#endif // QONBOARDSDK_H
