#include "DJI_Pro_Hw.h"

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include<sys/ioctl.h>

#include <termios.h>
#include <string.h>
#include <pthread.h>

#include "DJI_Pro_Codec.h"

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
			//TODO...Call protocol decode function
			len = depth > sizeof(buf) ? sizeof(buf) : depth;
			ret = read(serial_fd,buf,len);
			for(i = 0; i < ret; i ++)
			{
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
