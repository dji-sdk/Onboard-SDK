#ifndef QONBOARDSDK_H
#define QONBOARDSDK_H

#include <DJI_HardDriver.h>
#include <DJI_Camera.h>
#include <DJI_Flight.h>
#include <DJI_HotPoint.h>
#include <DJI_Follow.h>
#include <DJI_WayPoint.h>
#include <DJI_VirtualRC.h>
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
    DJI::time_ms getTimeStamp();
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
    QMutex sendlock;
};

class APIThread : public QThread
{
    Q_OBJECT

  public:
    APIThread();
    APIThread(CoreAPI *API, int Type, QObject *parent = 0);

    void run();

  private:
    CoreAPI *api;
    int type;
};

#endif // QONBOARDSDK_H
