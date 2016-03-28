#include "QHardDriver.h"
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
            qDebug("port %s open success",
                   port->portName().toLocal8Bit().data());
        }
        else
        {
            qDebug( "fail to open port %s",
                    port->portName().toLocal8Bit().data());
        }
        qDebug("BaudRate: %d", port->baudRate());
    }
}

size_t QHardDriver::send(char *buf, size_t len)
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
