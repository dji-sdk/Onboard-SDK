/*! @file LinuxSerialDevice.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Serial device hardware implementation for Linux machines.
 *  This is a generic Linux serial device implementation along with basic memory lock implementations.
 *
 *  Use this in your own Linux-based DJI Onboard SDK implementations.  
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "LinuxSerialDevice.h"

using namespace DJI::onboardSDK;

/*! Implementing inherited functions from abstract class DJI_HardDriver */

LinuxSerialDevice::LinuxSerialDevice(std::string device, unsigned int baudrate) {
  m_device = device;
  m_baudrate = baudrate;
  m_memLock = PTHREAD_MUTEX_INITIALIZER;
  m_msgLock = PTHREAD_MUTEX_INITIALIZER;
  m_ackLock = PTHREAD_MUTEX_INITIALIZER;
  pthread_cond_init(&m_ackRecvCv, NULL);

/*! These mutexes are used for the non blocking callback ACK mechanism */
  m_nbAckLock = PTHREAD_MUTEX_INITIALIZER;
  m_headerLock = PTHREAD_MUTEX_INITIALIZER;
  pthread_cond_init(&m_nbAckRecv, NULL);
}


LinuxSerialDevice::~LinuxSerialDevice() {
  _serialClose();
  pthread_mutex_destroy(&m_memLock);
  pthread_mutex_destroy(&m_msgLock);
  pthread_mutex_destroy(&m_ackLock);

  pthread_mutex_destroy(&m_nbAckLock);
  pthread_mutex_destroy(&m_headerLock);

  pthread_cond_destroy(&m_nbAckRecv);
  pthread_cond_destroy(&m_ackRecvCv);
}

void LinuxSerialDevice::init() {
  API_LOG(this, STATUS_LOG, "Attempting to open device %s with baudrate %u...\n", 
      m_device.c_str(), m_baudrate);
  if( _serialStart(m_device.c_str(), m_baudrate) < 0 ) 
  {
    _serialClose();
    API_LOG(this, ERROR_LOG, "...Failed to start serial\n");
    deviceStatus = false;
  }
  else 
  {
    API_LOG(this, STATUS_LOG, "...Serial started successfully.\n");
    deviceStatus = true;
  }
}

bool LinuxSerialDevice::getDeviceStatus()
{
  return deviceStatus;
}

DJI::time_ms LinuxSerialDevice::getTimeStamp() {
  return (unsigned int)time(NULL);
}


size_t LinuxSerialDevice::send(const uint8_t *buf, size_t len) {
  return _serialWrite(buf, len);
}


size_t LinuxSerialDevice::readall(uint8_t *buf, size_t maxlen) {
  return _serialRead(buf, maxlen);
}


void LinuxSerialDevice::lockMemory() {
  pthread_mutex_lock(&m_memLock);
}


void LinuxSerialDevice::freeMemory() {
  pthread_mutex_unlock(&m_memLock);
}


void LinuxSerialDevice::lockMSG() {
  pthread_mutex_lock(&m_msgLock);
}


void LinuxSerialDevice::freeMSG() {
  pthread_mutex_unlock(&m_msgLock);
}

void LinuxSerialDevice::lockACK(){
  pthread_mutex_lock(&m_ackLock);
}

void LinuxSerialDevice::freeACK(){
  pthread_mutex_unlock(&m_ackLock);
}

void LinuxSerialDevice::notify(){
  pthread_cond_signal(&m_ackRecvCv);
}

void LinuxSerialDevice::lockProtocolHeader(){
    pthread_mutex_lock(&m_headerLock);
}

void LinuxSerialDevice::freeProtocolHeader(){
    pthread_mutex_unlock(&m_headerLock);
}

void LinuxSerialDevice::lockNonBlockCBAck(){
    pthread_mutex_lock(&m_nbAckLock);
}

void LinuxSerialDevice::freeNonBlockCBAck(){
    pthread_mutex_unlock(&m_nbAckLock);
}

void LinuxSerialDevice::notifyNonBlockCBAckRecv(){
    pthread_cond_signal(&m_nbAckRecv);
}

void LinuxSerialDevice::nonBlockWait() {

    pthread_cond_wait(&m_nbAckRecv, &m_nbAckLock);
}

void LinuxSerialDevice::wait(int timeoutInSeconds){
  struct timespec curTime, absTimeout;
  //Use clock_gettime instead of getttimeofday for compatibility with POSIX APIs
  clock_gettime(CLOCK_REALTIME, &curTime);
  //absTimeout = curTime;
  absTimeout.tv_sec = curTime.tv_sec + timeoutInSeconds;
  absTimeout.tv_nsec = curTime.tv_nsec; 
  pthread_cond_timedwait(&m_ackRecvCv, &m_ackLock, &absTimeout);
}

/*! Implement functions specific to this hardware driver */


/**** 
  The next few functions set serial port I/O parameters and implement serial R/W functions. 
  Implement termios-based serial i/o.
****/
void LinuxSerialDevice::setBaudrate(unsigned int baudrate) {
  m_baudrate = baudrate;
}

void LinuxSerialDevice::setDevice(std::string device) {
  m_device = device;
}

int LinuxSerialDevice::_checkBaudRate(uint8_t (&buf)[BUFFER_SIZE])
{
  int lengthForCheck = 200;
  int timeoutInSeconds = 2;

  struct timespec curTime, absTimeout;
  //Use clock_gettime instead of getttimeofday for compatibility with POSIX APIs
  clock_gettime(CLOCK_REALTIME, &curTime);
  absTimeout.tv_sec = curTime.tv_sec + timeoutInSeconds;
  absTimeout.tv_nsec = curTime.tv_nsec; 

  int receivedBytes = _serialRead(buf, lengthForCheck);

  while (curTime.tv_sec < absTimeout.tv_sec)
  {
    if(receivedBytes < lengthForCheck)
      receivedBytes += _serialRead(buf + receivedBytes, lengthForCheck - receivedBytes);
    else
      break;
    
    clock_gettime(CLOCK_REALTIME, &curTime);    
  }
  if (curTime.tv_sec >= absTimeout.tv_sec)
    return -1;
  if (std::end(buf) == std::find(std::begin(buf), std::end(buf), 0xAA))
    return -2;

  return 1;
}

bool LinuxSerialDevice::_serialOpen(const char* dev) {
#ifdef __arm__  
  m_serial_fd = open(dev, O_RDWR | O_NONBLOCK );
#elif __x86_64__
   m_serial_fd = open(dev, O_RDWR | O_NOCTTY );
#endif
  if(m_serial_fd < 0) {
    API_LOG(this, ERROR_LOG, "cannot open device %s\n", dev);
    return false;
  }
  return true;
}


bool LinuxSerialDevice::_serialClose() {
  close(m_serial_fd);
  m_serial_fd = -1;
  return true;
}


bool LinuxSerialDevice::_serialFlush() {
  if(m_serial_fd < 0) {
    API_LOG(this, ERROR_LOG, "flushing fail because no device is opened\n");
    return false;
  } else {
    tcflush(m_serial_fd, TCIFLUSH);
    return true;
  }
}


bool LinuxSerialDevice::_serialConfig(int baudrate, char data_bits, char parity_bits, char stop_bits, bool testForData) {
  int st_baud[] = {
    B4800,
    B9600,
    B19200,
    B38400,
    B57600,
    B115200,
    B230400
  };
  int std_rate[] = {
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
  if (tcgetattr(m_serial_fd, &oldtio) != 0) {
    API_LOG(this, ERROR_LOG, "fail to save current port\n");
    return false;
  }
  memset(&newtio, 0, sizeof(newtio));

  /* config the size of char */
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;

  /* config data bit */
  switch (data_bits) {
    case 7:
      newtio.c_cflag |= CS7;
      break;
    case 8:
      newtio.c_cflag |= CS8;
      break;
  }
  /* config the parity bit */
  switch (parity_bits) {
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
  j = sizeof(std_rate) / 4;
  for(i = 0; i < j; ++i) {
    if(std_rate[i] == baudrate) {
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
  //! If you just want to see if there is data on the line, put the serial config in an unconditional timeout state
#if __x86_64__
  if (testForData)
  {
    newtio.c_cc[VTIME]  = 8;
    newtio.c_cc[VMIN] = 0;  
  }
  else
  {
    newtio.c_cc[VTIME]  = 1;
    newtio.c_cc[VMIN] = 18;
  }
#endif
  /* using the raw data mode */
  newtio.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);
  newtio.c_oflag  &= ~OPOST;

  /* flush the hardware fifo */
  tcflush(m_serial_fd,TCIFLUSH);

  /* activite the configuration */
  if((tcsetattr(m_serial_fd,TCSANOW,&newtio))!=0) {
    API_LOG(this, ERROR_LOG, "failed to activate serial configuration\n");
    return false;
  }
  return true;
}


int LinuxSerialDevice::_serialStart(const char *dev_name, int baud_rate) {
  const char *ptemp;
  if(dev_name == NULL) {
    ptemp = "/dev/ttyUSB0";
  } else {
    ptemp = dev_name;
  }
  if(true == _serialOpen(ptemp) 
      && true == _serialConfig(baud_rate,8,'N',1)) {

    FD_ZERO(&m_serial_fd_set);
    FD_SET(m_serial_fd, &m_serial_fd_set);
    return m_serial_fd;

  }
  return -1;
}


int LinuxSerialDevice::_serialWrite(const unsigned char *buf, int len) {
  return write(m_serial_fd, buf, len);
}

//! Current _serialRead behavior: Wait for 500 ms between characters till 18 char, read 18 characters if data available & return
//! 500 ms: long timeout to make sure that if we query the input buffer in the middle of a packet we still get the full packet
//! 18 char: len of most ACK packets.
int LinuxSerialDevice::_serialRead(unsigned char *buf, int len) {
  int ret = -1;

  if(NULL == buf)
  {
    return -1;
  }
  else 
  {
  ret = read(m_serial_fd,buf,len);

  return ret;
  }
}


