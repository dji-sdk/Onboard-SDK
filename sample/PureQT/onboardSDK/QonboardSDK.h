#ifndef QONBOARDSDK_H
#define QONBOARDSDK_H

#include <DJI_HardDriver.h>
#include <DJI_Camera.h>
#include <DJI_Flight.h>
#include <DJI_VirtualRC.h>
#include <DJI_API.h>
#include <QSerialPort>
#include <QSerialPortInfo>

#include <QMutex>
#include <QThread>

#include <QTextBrowser>

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

    void displayLog(char *buf = 0);

    void setBaudrate(int value);

    QTextBrowser *getDisplay() const;
    void setDisplay(QTextBrowser *value);

private:
    QHardDriver();

private:
    int baudrate;
    QSerialPort *port;
    QMutex memory;
    QMutex msg;
    QMutex sendlock;
    QTextBrowser *display;
};

class APIThread : public QThread
{
    Q_OBJECT

  public:
    APIThread();
    APIThread(CoreAPI *API, int Type,QObject *parent = 0);

    void run();

  private:
    CoreAPI *api;
    int type;
};

#endif // QONBOARDSDK_H
