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

#pragma pack(1)
typedef struct WayPointInitData
{
    uint8_t indexNumber;
    float32_t maxVelocity;
    float32_t idleVelocity;

    uint8_t finishAction;
    uint8_t executiveMode;
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

    uint8_t commandList[15];
    int16_t commandParameter[15];
} WayPointData;

#pragma pack()

class WayPoint
{
  public:
#ifndef STATIC_MEMORY
    WayPoint(CoreAPI *ControlAPI = 0);
#else
    WayPoint(WayPointData *list, uint8_t len, CoreAPI *ControlAPI = 0);
#endif // STATIC_MEMORY
    void init(const WayPointInitData *info = 0,CallBack callback = 0, UserData userData = 0);
    void start(CallBack callback = 0, UserData userData = 0);
    void stop(CallBack callback = 0, UserData userData = 0);
    //! @note true for pause, false for resume
    void pause(bool isPause, CallBack callback = 0, UserData userData = 0);
    void readInitData(CallBack callback = 0, UserData userData = 0);
    void readIndexData(uint8_t index, CallBack callback = 0, UserData userData = 0);

    void setInfo(const WayPointInitData &value);
    void setIndex(WayPointData *value);
    WayPointInitData getInfo() const;
    WayPointData *getIndex() const;
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
