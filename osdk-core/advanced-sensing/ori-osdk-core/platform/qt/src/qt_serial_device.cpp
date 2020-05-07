/** @file qt_serial_device.cpp
 *  @version 3.4
 *  @date Dec 2017
 *
 *  @brief Serial driver implementation using Qt constructs
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

#include <QDateTime>
#include <QDebug>
#include <QScrollBar>
#include <QThread>
#include <qt_serial_device.hpp>

using namespace DJI::OSDK;

QHardDriver::QHardDriver()
{
  port     = 0;
  baudrate = 230400;
}
QTextBrowser*
QHardDriver::getDisplay() const
{
  return display;
}

void
QHardDriver::setDisplay(QTextBrowser* value)
{
  display = value;
}

QHardDriver::QHardDriver(QObject* parent, const char* portName, int baudrate)
  : QObject(parent)
{
  this->portName = QString(portName);
  this->baudrate = baudrate;
  this->initStatus = false;
  display        = 0;
}

QHardDriver::QHardDriver(QObject* parent, QSerialPort* extSerialPort)
{
  this->port     = extSerialPort;
  this->baudrate = extSerialPort->baudRate();
  this->initStatus = false;
  display        = 0;
}

bool
QHardDriver::getDeviceStatus()
{
  bool curStatus = false;
  statusLock.lock();
  curStatus = this->initStatus;
  statusLock.unlock();

  return curStatus;
}

void
QHardDriver::setDeviceStatus(bool status)
{
  statusLock.lock();
  this->initStatus = status;
  statusLock.unlock();
}
void
QHardDriver::init()
{
  initLock.lock();
  if (this->getDeviceStatus())
  {
    initLock.unlock();
    return;
  }

  port = new QSerialPort(QString(portName));
  if (port != 0)
  {
    if (port->isOpen())
      port->close();
    port->setBaudRate(baudrate);
    port->setParity(QSerialPort::NoParity);
    port->setDataBits(QSerialPort::Data8);
    port->setStopBits(QSerialPort::OneStop);
    port->setFlowControl(QSerialPort::NoFlowControl);
    if (port->open(QIODevice::ReadWrite))
    {
      DSTATUS("port %s open success", port->portName().toLocal8Bit().data());
      setDeviceStatus(true);
      DSTATUS("Read buf size: %d", port->readBufferSize());
    }
    else
    {
      DERROR("fail to open port %s", port->portName().toLocal8Bit().data());
      setDeviceStatus(false);
    }
    DSTATUS("BaudRate: %d", port->baudRate());
  }
  initLock.unlock();
}

time_ms
QHardDriver::getTimeStamp()
{
  return QDateTime::currentMSecsSinceEpoch();
}

size_t
QHardDriver::send(const uint8_t* buf, size_t len)
{
  sendLock.lock();
  size_t sent = 0;
  if (port != 0)
  {
    //    if (port->isOpen())
    while (sent != len)
    {
      sent += port->write(reinterpret_cast<const char*>(buf + sent), len);
      port->waitForBytesWritten(2);
    }
    sendLock.unlock();
    return sent;
  }
  else
  {
    sendLock.unlock();
    return 0;
  }
  sendLock.unlock();
  return sent;
}

size_t
QHardDriver::readall(uint8_t* buf, size_t maxlen)
{
  size_t ans = 0;
  if (port != 0)
  {
    if (port->isOpen())
    {
      if (port->bytesAvailable() > 0)
      {
        QThread::usleep(100);
        ans = port->read(reinterpret_cast<char*>(buf), maxlen);
        // bufferLock.unlock();
      }
    }
  }
  return ans;
}

// void
// QHardDriver::displayLog(const char* buf)
//{
//  if (buf)
//    qDebug("%s", buf);
//  else
//  {
//    if (display)
//    {
//      bufferLock.lock();
//      QString data = QString(DJI::OSDK::buffer);
//      size_t  len  = data.length();
//      if (len < DJI::OSDK::bufsize)
//        display->append(data);
//      bufferLock.unlock();
//      display->verticalScrollBar()->setValue(
//        display->verticalScrollBar()->maximum());
//    }
//    else
//    {
//      bufferLock.lock();
//      qDebug("%s", DJI::OSDK::buffer);
//      bufferLock.unlock();
//    }
//  }
//}

void
QHardDriver::setBaudrate(int value)
{
  baudrate = value;
}
