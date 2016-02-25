#include "QonboardSDK.h"
#include <QDateTime>

QHardDriver::QHardDriver()
{
    port = 0;
    baudrate = 9600;
}

QHardDriver::QHardDriver(QSerialPort *Port)
{
    port = Port;
    baudrate = 9600;
}

void QHardDriver::init()
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

DJI::time_ms QHardDriver::getTimeStamp() { return QDateTime::currentMSecsSinceEpoch(); }

size_t QHardDriver::send(const uint8_t *buf, size_t len)
{
    sendlock.lock();
    size_t sent = 0;
    if (port != 0)
    {
        if (port->isOpen())
            while (sent != len)
            {
                sent += port->write((char *)(buf + sent), len);
                port->waitForBytesWritten(2);
            }
        sendlock.unlock();
        return sent;
    }
    else
    {
        sendlock.unlock();
        return 0;
    }
    sendlock.unlock();
    return sent;
}

size_t QHardDriver::readall(uint8_t *buf, size_t maxlen)
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

void QHardDriver::lockMemory() { memory.lock(); }

void QHardDriver::freeMemory() { memory.unlock(); }

void QHardDriver::lockMSG() { msg.lock(); }

void QHardDriver::freeMSG() { msg.unlock(); }

void QHardDriver::setBaudrate(int value) { baudrate = value; }

APIThread::APIThread(CoreAPI *API, int Type, QObject *parent) : QThread(parent)
{
    api = API;
    type = Type;
}

void APIThread::run()
{
    while (1)
    {
        if (type == 1)
            api->sendPoll();
        else if (type == 2)
            api->readPoll();
        msleep(1);
    }
}

APIThread::APIThread()
{
    api = 0;
    type = 0;
}
