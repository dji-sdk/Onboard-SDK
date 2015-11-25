#ifndef QONBOARDSDK_H
#define QONBOARDSDK_H

#include <DJI_HardDriver.h>
#include <DJI_API.h>
#include <QSerialPort>
#include <QSerialPortInfo>

#include <QMutex>

#include <QThread>

using namespace DJI::onboardSDK;

class QHardDriver : public HardDriver
{
  public:
    QHardDriver(QSerialPort *Port);

    void init();
    unsigned int getTimeStamp();
    size_t send(const uint8_t *buf, size_t len);
    size_t readall(uint8_t *buf, size_t maxlen);

    void lockMemory();
    void freeMemory();

    void lockMSG();
    void freeMSG();

    void setBaudrate(int value);

  private:
    QHardDriver();

  private:
    int baudrate;
    QSerialPort *port;
    QMutex memory;
    QMutex msg;
};

class APIThread : public QThread
{
    Q_OBJECT

  public:
    APIThread();
    APIThread(CoreAPI *API, int Type);

    void run();

  private:
    CoreAPI *api;
    int type;
};

#endif // QONBOARDSDK_H
