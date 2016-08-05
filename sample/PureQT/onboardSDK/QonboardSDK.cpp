#include "QonboardSDK.h"
#include <QDateTime>
#include <QScrollBar>
#include <QDebug>

QtOnboardsdkPortDriver::QtOnboardsdkPortDriver()
{
    port = 0;
    baudrate = 9600;
}
QTextBrowser *QtOnboardsdkPortDriver::getDisplay() const { return display; }

void QtOnboardsdkPortDriver::setDisplay(QTextBrowser *value) { display = value; }

QtOnboardsdkPortDriver::QtOnboardsdkPortDriver(QSerialPort *Port)
{
    port = Port;
    baudrate = 9600;
    display = 0;
}

void QtOnboardsdkPortDriver::init()
{
    if (port != 0)
    {
        if (port->isOpen())
            port->close();
        port->setBaudRate(baudrate);
        port->setParity(QSerialPort::NoParity);
        port->setDataBits(QSerialPort::Data8);
        port->setStopBits(QSerialPort::OneStop);
        if (port->open(QIODevice::ReadWrite))
        {
            API_LOG(this, STATUS_LOG, "port %s open success",
                    port->portName().toLocal8Bit().data());
        }
        else
        {
            API_LOG(this, ERROR_LOG, "fail to open port %s",
                    port->portName().toLocal8Bit().data());
        }
        API_LOG(this, STATUS_LOG, "BaudRate: %d", port->baudRate());
    }
}

DJI::time_ms QtOnboardsdkPortDriver::getTimeStamp() { return QDateTime::currentMSecsSinceEpoch(); }

size_t QtOnboardsdkPortDriver::send(const uint8_t *buf, size_t len)
{
    sendLock.lock();
    size_t sent = 0;
    if (port != 0)
    {
        if (port->isOpen())
            while (sent != len)
            {
                sent += port->write((char *)(buf + sent), len);
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

size_t QtOnboardsdkPortDriver::readall(uint8_t *buf, size_t maxlen)
{
    size_t ans = 0;
    if (port != 0)
    {
        if (port->isOpen())
            if (port->bytesAvailable() > 0)
                ans = port->read((char *)buf, maxlen);
    }
    return ans;
}

void QtOnboardsdkPortDriver::lockMemory() { memory.lock(); }

void QtOnboardsdkPortDriver::freeMemory() { memory.unlock(); }

void QtOnboardsdkPortDriver::lockMSG() { msg.lock(); }

void QtOnboardsdkPortDriver::freeMSG() { msg.unlock(); }

void QtOnboardsdkPortDriver::lockACK() {ack.lock();}
void QtOnboardsdkPortDriver::freeACK() {ack.unlock();}

void QtOnboardsdkPortDriver::notify() {};
void QtOnboardsdkPortDriver::wait(int timeout) {};

void QtOnboardsdkPortDriver::displayLog(const char *buf)
{
    if (buf)
        qDebug("%s", buf);
    else
    {
        if (display)
        {
            bufferLock.lock();
            QString data = QString(DJI::onboardSDK::buffer);
            size_t len = data.length();
            if (len < DJI::onboardSDK::bufsize)
                display->append(data);
            bufferLock.unlock();
            display->verticalScrollBar()->setValue(display->verticalScrollBar()->maximum());
        }
        else
        {
            bufferLock.lock();
            qDebug("%s", DJI::onboardSDK::buffer);
            bufferLock.unlock();
        }
    }
}

void QtOnboardsdkPortDriver::setBaudrate(int value) { baudrate = value; }

APIThread::APIThread(CoreAPI *API, int Type, QObject *parent) : QThread(parent)
{
    api = API;
    type = Type;
}

void APIThread::run()
{
    while (1)
    {
        callTimes++;
        if (type == 1)
            api->sendPoll();
        else if (type == 2)
            api->readPoll();
        msleep(1);
    }
}
size_t APIThread::getCallTimes() const { return callTimes; }

void APIThread::setCallTimes(const size_t &value) { callTimes = value; }

APIThread::APIThread()
{
    api = 0;
    type = 0;
}
