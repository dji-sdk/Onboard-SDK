/*
 * DJI_Pro_Hw.cpp
 *
 *  Created on: Aug 24, 2015
 *  Author: wuyuwei
 */
#if defined(PLATFORM_LINUX) || defined(__linux)
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <string.h>
#include <pthread.h>
#include "DJI_Pro_Codec.h"
#include "DJI_Pro_Hw.h"

static int serial_fd = -1;
static fd_set serial_fd_set;

int SerialOpen(const char *port_str)
{
	serial_fd = open(port_str, O_RDWR | O_NOCTTY);  //block mode
	if(serial_fd < 0)
	{
		printf("%s,%d:ERROR\n",__func__,__LINE__);
		return -1;
	}
	return 0;
}

int SerialClose()
{
	close(serial_fd);
	serial_fd = -1;
	return 0;
}

int SerialFlush()
{
	if(serial_fd < 0)
	{
		printf("%s,%d:ERROR\n",__func__,__LINE__);
	}
	else
	{
		tcflush(serial_fd,TCIFLUSH);
	}
	return 0;
}

int SerialConfig(int baudrate,char data_bits,char parity_bits,char stop_bits)
{
	int st_baud[]=
	{
		B4800,
		B9600,
		B19200,
		B38400,
		B57600,
		B115200,
		B230400,
		B1000000,
		B1152000,
		B3000000,
	};
	int std_rate[]=
	{
		4800,
		9600,
		19200,
		38400,
		57600,
		115200,
		230400,
		1000000,
		1152000,
		3000000,
	};

	int i,j;
	struct termios newtio, oldtio;
	/* save current port parameter */
	if (tcgetattr(serial_fd, &oldtio) != 0)
	{
		printf("%s,%d:ERROR\n",__func__,__LINE__);
		return -1;
	}
	bzero(&newtio, sizeof(newtio));

	/* config the size of char */
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	/* config data bit */
	switch (data_bits)
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}
	/* config the parity bit */
	switch (parity_bits)
	{
		/* odd */
	case 'O':
	case 'o':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		break;
		/* even */
	case 'E':
	case 'e':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
		/* none */
	case 'N':
	case 'n':
		newtio.c_cflag &= ~PARENB;
		break;
	}
	/* config baudrate */
	j = sizeof(std_rate)/4;
	for(i = 0;i < j;i ++)
	{
		if(std_rate[i] == baudrate)
		{
			/* set standard baudrate */
			cfsetispeed(&newtio, st_baud[i]);
			cfsetospeed(&newtio, st_baud[i]);
			break;
		}
	}
	/* config stop bit */
    if( stop_bits == 1 )
       newtio.c_cflag &=  ~CSTOPB;
    else if ( stop_bits == 2 )
       newtio.c_cflag |=  CSTOPB;

    /* config waiting time & min number of char */
    newtio.c_cc[VTIME]  = 1;
    newtio.c_cc[VMIN] = 1;

    /* using the raw data mode */
    newtio.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);
    newtio.c_oflag  &= ~OPOST;

    /* flush the hardware fifo */
    tcflush(serial_fd,TCIFLUSH);

    /* activite the configuration */
    if((tcsetattr(serial_fd,TCSANOW,&newtio))!=0)
    {
    	printf("%s,%d:ERROR\n",__func__,__LINE__);
    	return -1;
    }
    return 0;
}

static int SerialStart(const char *dev_name,int baud_rate)
{
	const char *ptemp;
	if(dev_name == NULL)
	{
		ptemp = "/dev/ttyUSB0";
	}
	else
	{
		ptemp = dev_name;
	}
	if(SerialOpen(ptemp) == 0
			&& SerialConfig(baud_rate,8,'N',1) == 0)
	{
		FD_ZERO(&serial_fd_set);
		FD_SET(serial_fd, &serial_fd_set);
		return serial_fd;
	}
	return -1;
}

static int SerialWrite(unsigned char *buf,int len)
{
	return write(serial_fd,buf,len);
}

static int SerialRead(unsigned char *buf,int len)
{
	int saved = 0;
	int ret = -1;

	if( buf == NULL)
		return -1;
	else
	{
		for(; saved < len ;)
		{
			ret = read(serial_fd,buf + saved,len - saved);
			if(ret > 0)
			{
				saved += ret;
			}
			else
			{
				break;
			}
		}
		return saved;
	}
}

static void * SerialRecvThread(void * arg)
{
	int ret;
	unsigned int depth,len;
	unsigned char buf[64];
	int i;
	while(1)
	{
        ret = select(FD_SETSIZE, &serial_fd_set, (fd_set *)0, (fd_set *)0,(struct timeval *) 0);
		if (ret < 1)
		{
			printf("%s,%d,ERROR\n", __func__, __LINE__);
			FD_ZERO(&serial_fd_set);
			FD_SET(serial_fd, &serial_fd_set);
			continue;
		}

		ioctl(serial_fd, FIONREAD, &depth);
		if(depth > 0)
		{
			len = depth > sizeof(buf) ? sizeof(buf) : depth;
			ret = read(serial_fd,buf,len);
			for(i = 0; i < ret; i ++)
			{
				//TODO...call protocol decode function
				sdk_serial_byte_handle(buf[i]);
			}
		}
	}
	return NULL;
}

int SerialStartThread(void)
{
	int ret;
	pthread_t A_ARR;
	ret = pthread_create(&A_ARR, 0,SerialRecvThread,NULL);
	if(ret != 0)
	{
		return -1;
	}
	return 0;
}

int Pro_Hw_Send(unsigned char *buf, int len)
{
	return SerialWrite(buf,len);
}

int Pro_Hw_Recv(unsigned char *buf, int len)
{
	return SerialRead(buf,len);
}

int Pro_Hw_Setup(const char *device,int baudrate)
{
	if(SerialStart(device,baudrate) < 0)
	{
		SerialClose();
		return -1;
	}

	if(SerialStartThread() < 0)
	{
		return -1;
	}

	return 0;
}

#endif

#ifdef PLATFORM_QT
#include <QThread>
#include <QByteArray>
#include <qextserialport.h>
#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include "DJI_Pro_Link.h"
#include "DJI_Pro_Codec.h"
#include "DJI_Pro_Hw.h"
#include "DJI_Pro_Codec.h"

DJI_Pro_Hw *DJI_Pro_Hw::serial_object = (DJI_Pro_Hw *)NULL;

typedef struct
{
    int baudrate;
    int index;
}Baud_index;

Baud_index baud_index_t[] =
{
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
    {
        port->close();
    }
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
    if(port->isOpen())
    {
        for(;sent < len;)
        {
            ret = port->write((char *)buf + sent,len - sent);
            port->waitForBytesWritten(-1);
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
}

int DJI_Pro_Hw::findindex(int baudrate)
{
    int index_len;
    index_len = sizeof(baud_index_t) / sizeof(baud_index_t[0]);
    for(int i=0;i<index_len;i++)
    {
        if (baudrate == baud_index_t[i].baudrate)
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
    if(ret == true)
    {
        printf("%s,line %d,OPEN SUCCESS\n",__func__,__LINE__);
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

DJI_Pro_Hw * DJI_Pro_Hw::Pro_Hw_Create_Instance(void)
{
    if(serial_object == (DJI_Pro_Hw*)NULL)
    {
        serial_object = new DJI_Pro_Hw;
    }
    return serial_object;
}

DJI_Pro_Hw * DJI_Pro_Hw::Pro_Hw_Get_Instance(void)
{
    return Pro_Hw_Create_Instance();
}

#endif
