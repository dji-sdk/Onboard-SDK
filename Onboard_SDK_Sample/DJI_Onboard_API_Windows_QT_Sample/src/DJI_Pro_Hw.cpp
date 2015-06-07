#include "DJI_Pro_Hw.h"
#include "DJI_Pro_Codec.h"
#include <QThread>
#include <QByteArray>
#include <qextserialport.h>
#include <pthread.h>
#include <string.h>
#include <sys/time.h>

#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>

#include "DJI_Pro_Link.h"

#include "DJI_Pro_Codec.h"
DJI_Pro_Hw Pro_Hw;

typedef struct
 {
    int baudrate;
    int index;
 }Baud_index;

 Baud_index baud_index_t[] = {
     {4800,11},
     {9600,12},
     {14400,13},
     {19200,14},
     {38400,15},
     {56000,16},
     {57600,17},
     {115200,19},
     {256000,21}
 };

DJI_Pro_Hw::DJI_Pro_Hw(QObject *parent) :
    QThread(parent),
    pbuffer(NULL)
{

    port = new QextSerialPort();

    pbuffer = new unsigned char[DJI_PRO_HW_BUFFER_SIZE];

   // connect(port,SIGNAL(readyRead()),this,SLOT(Pro_Hw_Recv()));

    load_con = 0;

}

DJI_Pro_Hw::~DJI_Pro_Hw()
{
    if(port)
    {
        delete port;
    }

    if(pbuffer)
    {
        delete [] pbuffer;
    }
}

void DJI_Pro_Hw::Pro_Hw_Close()
{
    if(port->isOpen())
        port->close();
}

void DJI_Pro_Hw::Pro_Hw_Flush()
{
    if(port->isOpen())
    {
        port->flush();
    }
}

int DJI_Pro_Hw::Pro_Hw_Send(unsigned char *buf, int len)
{
    int sent = 0;
    int ret;
    //bool retval;
    if(port->isOpen())
    {
        for(;sent < len;)
        {
            ret = port->write((char *)buf + sent,len - sent);
            /*retval = */port->waitForBytesWritten(-1);
            /*if(retval == false)
            {
                printf("%s,%d,Time out,or error occurred\n",
                       __func__,__LINE__);
                if(port->isOpen() == false)
                {
                    break;
                }
            }*/

            if(port->isOpen() == false)
            {
                break;
            }

            if(ret > 0)
            {
              sent += ret;
            }

        }
    }
    return sent;
}

void DJI_Pro_Hw::Pro_Hw_Recv()
{
  /*  int depth;
    int limit_len,read_len;
    depth = port->bytesAvailable();
 static int load_con = 0;
    limit_len = depth > DJI_PRO_HW_BUFFER_SIZE ? DJI_PRO_HW_BUFFER_SIZE \
                : depth;

    read_len = port->read((char *)pbuffer,limit_len);


    for(int i = 0; i < read_len; i ++)
    {
        sdk_serial_byte_handle(pbuffer[i]);
    }

    load_con += read_len;
    printf("\r len:%d",load_con);

*/
}

int DJI_Pro_Hw::findindex(int baudrate)
{
    int index_len;
    index_len = sizeof(baud_index_t)/sizeof(baud_index_t[0]);
    for(int i=0;i<index_len;i++)
    {
        if (baudrate==baud_index_t[i].baudrate)
        {
            return baud_index_t[i].index;
        }
    }
    return -1;
}


bool DJI_Pro_Hw::Pro_Hw_Setup(QString port_name,int baudrate)
{
    QString str = "\\\\.\\";
    int index=findindex(baudrate);
      if (-1 == index)
    {
        index=baudrate;
    }
    bool ret = false;
    str += port_name;
    port->setPortName(str);
    ret = port->open(QIODevice::ReadWrite|QIODevice::Unbuffered);
    port->setBaudRate((BaudRateType)index);
    port->setFlowControl(FLOW_OFF);
    port->setParity(PAR_NONE);
    port->setDataBits(DATA_8);
    port->setStopBits(STOP_1);
    //set timeouts to 20 ms
    port->setTimeout(20);
   // port->setQueryMode(QextSerialBase::EventDriven);

    if(ret == true)
    {
        printf("OPEN SUCCESS\n");
        port->flush();
    }
    return ret;
}

void DJI_Pro_Hw::run()
{
    int depth;
    int limit_len,read_len;
    while(1)
    {
        if(port->isOpen())
        {

            depth = port->bytesAvailable();
            if(depth > 0)
            {
                limit_len = depth > DJI_PRO_HW_BUFFER_SIZE ? DJI_PRO_HW_BUFFER_SIZE \
                    : depth;

                read_len = port->read((char *)pbuffer,limit_len);


                for(int i = 0; i < read_len; i ++)
                {
                    sdk_serial_byte_handle(pbuffer[i]);
                }

                load_con += read_len;
                //printf("\rReceived :%d",load_con);
            }
            else
            {
                msleep(2);
            }

        }
        else
        {
            msleep(2);
        }

    }
}

