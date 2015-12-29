/*! @brief
 *  @file DJI_WayPoint.h
 *  @version 3.0
 *  @date Dec 22, 2015
 *
 *  @abstract
 *  WayPoint API for DJI onboardSDK library
 *
 *  @attention
 *  Project configuration:
 *
 *  @version features:
 *  -* @version V3.0
 *  -* DJI-onboard-SDK for Windows,QT,STM32,ROS,Cmake
 *  -* @date Dec 16, 2015
 *  -* @author william.wu
 *
 * */
#include "DJI_Mission.h"

#ifndef DJI_WAYPOINT_H
#define DJI_WAYPOINT_H

namespace DJI
{
namespace onboardSDK
{

class WayPoint;

#pragma pack(1)
typedef struct WayPointInitData
{
    uint8_t indexNumber;
    float32_t maxVelocity;
    float32_t idleVelocity;

    uint8_t finishAction;
    uint8_t executiveTimes;
    uint8_t yawMode;
    uint8_t traceMode;
    uint8_t RCLostAction;
    uint8_t gimbalPitch;
    float64_t latitude;  //! @note For Camera to recording
    float64_t longitude; //! not supported yet
    float32_t altitude;

    uint8_t reserved[16];
} WayPointInitData;

typedef struct WayPointData
{
    uint8_t index;

    float64_t latitude;
    float64_t longitude;
    float32_t altitude;
    float32_t damping;

    int16_t yaw;
    int16_t gimbalPitch;
    uint8_t turnMode;

    uint8_t reserved[8];
    uint8_t hasAction;
    uint16_t actionTimeLimit;

    uint8_t actionNumber : 4;
    uint8_t actionRepeat : 4;

    uint8_t commandList[16];//! @note issues here list number is 15
    int16_t commandParameter[16];
} WayPointData;

typedef struct WayPointVelocityACK
{
    uint8_t ack;
    float32_t idleVelocity;
} WayPointVelocityACK;

typedef struct WayPointInitACK
{
    uint8_t ack;
    WayPointInitData data;
} WayPointInitACK;

typedef struct WayPointDataACK
{
    uint8_t ack;
    uint8_t index;
} WayPointDataACK;

#pragma pack()

class WayPoint
{
  public:
#ifndef STATIC_MEMORY
    WayPoint(CoreAPI *ControlAPI = 0);
#else
    WayPoint(WayPointData *list, uint8_t len, CoreAPI *ControlAPI = 0);
#endif // STATIC_MEMORY
    void init(const WayPointInitData *Info = 0, CallBack callback = 0, UserData userData = 0);
    void start(CallBack callback = 0, UserData userData = 0);
    void stop(CallBack callback = 0, UserData userData = 0);
    //! @note true for pause, false for resume
    void pause(bool isPause, CallBack callback = 0, UserData userData = 0);
    void readInitData(CallBack callback = 0, UserData userData = 0);
    void readIndexData(uint8_t index, CallBack callback = 0, UserData userData = 0);
    void readIdleVelocity(CallBack callback = 0, UserData userData = 0);
    //! @todo uploadAll
    //void uploadAll(CallBack callback = 0, UserData userData = 0);
    bool uploadIndexData(WayPointData *data, CallBack callback = 0, UserData userData = 0);
    bool uploadIndexData(uint8_t pos, CallBack callback = 0, UserData userData = 0);
    void updateIdleVelocity(float32_t meterPreSecond, CallBack callback = 0,
                            UserData userData = 0);

    void setInfo(const WayPointInitData &value);
    void setIndex(WayPointData *value, size_t pos);
    WayPointInitData getInfo() const;
    WayPointData *getIndex() const;
    WayPointData *getIndex(size_t pos) const;

  public:
    static void idleVelocityCallback(CoreAPI *This, Header *header, UserData wpThis);
    static void readInitDataCallback(CoreAPI *This, Header *header, UserData wpThis);
    static void uploadIndexDataCallback(CoreAPI *This, Header *header, UserData wpThis);

  private:
    CoreAPI *api;
    WayPointInitData info;
    WayPointData *index;
#ifdef STATIC_MEMORY
    uint8_t maxIndex;
#endif // STATIC_MEMORY
};

} // namespace onboardSDK
} // namespace DJI

#endif // DJI_WAYPOINT_H
