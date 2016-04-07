/*! @brief
 *  @file DJI_HotPoint.h
 *  @version 3.0
 *  @date Dec 16, 2015
 *
 *  @abstract
 *  HotPoint API for DJI onboardSDK library
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

#ifndef DJI_HOTPOINT_H
#define DJI_HOTPOINT_H

#include "DJI_Mission.h"

namespace DJI
{
namespace onboardSDK
{

#pragma pack(1)

typedef struct HotPointData
{
    uint8_t version;

    float64_t latitude;
    float64_t longitude;
    float64_t height;

    float64_t radius;
    float32_t palstance; // degree

    uint8_t clockwise;
    uint8_t startPoint;
    uint8_t yawMode;
    uint8_t reserved[11];
} HotPointData;

#pragma pack()

class HotPoint
{
  public:
#pragma pack(1)
    typedef struct StartACK
    {
        uint8_t ack;
        float32_t maxRadius;
    } StartACK;

    typedef struct Palstance
    {
        uint8_t clockwise;
        float32_t palstance;
    } Palstance;

    typedef struct ReadACK
    {
        MissionACK ack;
        HotPointData data;
    } ReadACK;
#pragma pack()

    enum View
    {
        VIEW_NORTH = 0,
        VIEW_SOUTH = 1,
        VIEW_WEST = 2,
        VIEW_EAST = 3,
        VIEW_NEARBY = 4
    };

    enum YawMode
    {
        YAW_AUTO = 0,
        YAW_INSIDE = 1,
        YAW_OUTSIDE = 2,
        YAW_CUSTOM = 3,
        YAW_STATIC = 4,
    };

  public:
    HotPoint(CoreAPI *ControlAPI = 0);
    void initData();

    /*! @note API functions
     *  @attention difference between set and update
     *  Set functions only change the HotPoint data in this class,
     *  Update functions will change the Mission status.
     *  In other words: drone will response update functions immediately.
     * */

    void start(CallBack callback = 0, UserData userData = 0);
    void stop(CallBack callback = 0, UserData userData = 0);
    void pause(bool isPause, CallBack callback = 0, UserData userData = 0);

    void updatePalstance(Palstance &Data, CallBack callback = 0, UserData userData = 0);
    void updatePalstance(float32_t palstance, bool isClockwise, CallBack callback = 0,
                        UserData userData = 0);
    void updateRadius(float32_t meter, CallBack callback = 0, UserData userData = 0);
    void resetYaw(CallBack callback = 0, UserData userData = 0);

    void readData(CallBack callback = 0, UserData userData = 0);

  public:
    //! @note data access functions
    void setData(const HotPointData &value);
    void setHotPoint(float64_t longtitude, float64_t latitude, float64_t altitude);
    void setHotPoint(GPSData gps);
    void setRadius(float64_t meter);
    void setPalstance(float32_t defree);
    void setClockwise(bool isClockwise);
    void setCameraView(View view);
    void setYawMode(YawMode mode);

    HotPointData getData() const;

  public:
    static void startCallback(CoreAPI *This, Header *header, UserData userdata = 0);
    static void readCallback(CoreAPI *This, Header *header, UserData userdata);

  private:
    CoreAPI *api;
    HotPointData data;
};

} // namespace onboardSDK
} // namespace DJI

#endif // DJI_HOTPOINT_H
