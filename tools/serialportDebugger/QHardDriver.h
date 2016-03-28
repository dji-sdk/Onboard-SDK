#include "stdint.h"
#include <QSerialPort>
#include <QMutex>
#include <QDebug>

class QHardDriver
{
  public:
    QHardDriver(QSerialPort *Port);
    void init();
    size_t send(char *buf, size_t len);
    size_t readall(uint8_t *buf, size_t maxlen);
    void setBaudrate(int value) { baudrate = value; }

  private:
    QHardDriver();

  private:
    int baudrate;
    QSerialPort *port;
    QMutex sendlock;
};
