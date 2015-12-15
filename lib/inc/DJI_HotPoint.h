#ifndef DJI_HOTPOINT_H
#define DJI_HOTPOINT_H

#include "DJI_API.h"

namespace DJI
{
namespace onboardSDK
{

//! @todo check complier
//! #pragma anon_unions //! @attention maybe not available for alll compilers
#pragma pack(1)

typedef struct HotPointData
{
    uint8_t version;

    float64_t latitude;
    float64_t longtitude;
    float64_t altitude;

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
#pragma pack(1)
    typedef uint8_t CommonACK;
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

    void start(CallBack callback = 0);
    void stop(CallBack callback = 0);
    void startPalstance(Palstance &data, CallBack callback = 0);
    void startPalstance(float32_t palstance, bool isClockwise,
                        CallBack callback = 0);

  public:
    //! @note data access functions
    void setData(const HotPointData &value);
    void setHotPoint(float64_t longtitude, float64_t latitude,
                     float64_t altitude);
    void setHotPoint(GPSData gps);

    void setRadius(float64_t meter);
    void setPalstance(float32_t radps);
    void setClockwise(bool isClockwise);
    void setCameraView(View view);
    void setYawMode(YawMode mode);

    HotPointData getData() const;

  public:
    static void startCallback(CoreAPI *This, Header *header);
    static void commonCallback(CoreAPI *This, Header *header);

  private:
    CoreAPI *api;
    HotPointData data;
};

} // namespace onboardSDK
} // namespace DJI

#endif // DJI_HOTPOINT_H
