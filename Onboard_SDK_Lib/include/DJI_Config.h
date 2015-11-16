#ifndef __DJI_PRO_CONFIG_H__
#define __DJI_PRO_CONFIG_H__


#include <QMutex>
#include <QSerialPort>
namespace DJI
{
namespace onboardSDK
{
typedef QMutex* mutex_t;
typedef QSerialPort* port_t;
}
}

#define MEMORY_SIZE     1000   //unit is byte
#define BUFFER_SIZE     1024

//#define API_DEBUG_DATA
//#define API_ERROR_DATA
#define API_STATUS_DATA


#endif//__DJI_PRO_CONFIG_H__
