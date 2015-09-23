#ifndef __DJI_PRO_HW_H__
#define __DJI_PRO_HW_H__

#if defined(PLATFORM_LINUX) || defined(__linux)
int Pro_Hw_Send(unsigned char *buf, int len);
int Pro_Hw_Recv(unsigned char *buf, int len);
int Pro_Hw_Setup(const char *device,int baudrate);
#endif


#ifdef PLATFORM_QT

#include <QMainWindow>
#include <QString>
#include <QObject>
#include <QThread>
#include "qextserialport.h"

#define DJI_PRO_HW_BUFFER_SIZE      1024

class DJI_Pro_Hw : public QThread
{
    Q_OBJECT
public:
    explicit DJI_Pro_Hw(QObject *parent = 0);
    explicit DJI_Pro_Hw(QString name,int baudrate);
    ~DJI_Pro_Hw();

signals:

public slots:
private slots:
    void Pro_Hw_Recv();
private:
    QextSerialPort *port;
    unsigned char *pbuffer;
    int findindex(int);
public:
    void Pro_Hw_Close();
    void Pro_Hw_Flush();
    int Pro_Hw_Send(unsigned char *buf, int len);
    bool Pro_Hw_Setup(QString port_name,int baudrate);
    int load_con;
protected:
    void run();
public:
    static DJI_Pro_Hw *serial_object;
    static DJI_Pro_Hw *Pro_Hw_Create_Instance(void);
    static DJI_Pro_Hw *Pro_Hw_Get_Instance(void);
};

#endif

#endif // DJI_PRO_HW_H
