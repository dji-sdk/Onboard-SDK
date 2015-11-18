#ifndef __DJI_PRO_HW_H__
#define __DJI_PRO_HW_H__

#include <stdint.h>
#include <QMutex>
#include "DJI_Type.h"



namespace DJI
{
namespace onboardSDK
{



class HardDriver
{
public:
    HardDriver();
public:
    virtual void init() = 0;
    virtual unsigned int getTimeStamp() = 0;
    virtual size_t send(const uint8_t *buf,size_t len) = 0;
    virtual size_t readall(uint8_t *buf, size_t maxlen) = 0;
public:
    virtual void lockMemory();
    virtual void freeMemory();

    virtual void lockMSG();
    virtual void freeMSG();
};
}//namespace onboardSDK
}//namespace DJI

#ifdef V2_0
extern void Get_Memory_Lock(void);
extern void Free_Memory_Lock(void);
#endif




#endif // DJI_PRO_HW_H
