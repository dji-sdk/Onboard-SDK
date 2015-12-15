#ifndef DJI_FLIGHT_H
#define DJI_FLIGHT_H

#include "DJI_API.h"

namespace DJI
{
namespace onboardSDK
{
#pragma pack(1)

typedef struct FlightData
{
    uint8_t ctrl_flag;
    float32_t roll_or_x;
    float32_t pitch_or_y;
    float32_t thr_z;
    float32_t yaw;
} FlightData;

#pragma pack()



class Flight
{
  public:
    enum TASK
    {
        TASK_GOHOME = 1,
        TASK_TAKEOFF = 4,
        TASK_LANDING = 6
    };

    enum VerticalLogic
    {
        VERTICAL_VELOCITY = 0x00,
        VERTICAL_POSSITION = 0x10,
        VERTICAL_THRUST = 0x20,
    };

    enum HorizontalLogic
    {
        HORIZONTAL_ANGLE = 0x00,
        HORIZONTAL_VELOCITY = 0x40,
        HORIZONTAL_POSSITION = 0X80,
    };

    enum YawLogic
    {
        YAW_ANGLE = 0x00,
        YAW_PALSTANCE = 0x08
    };

    enum HorizontalCoordinate
    {
        HORIZONTAL_GROUND = 0x00,
        HORIZONTAL_BODY = 0x02
    };

#ifdef SDK_VERSION_2_3
    enum YawCoordinate
    {
        YAW_GROUND = 0x00,
        YAW_BODY = 0X01
    };
#endif // SDK_VERSION_2_3

#ifndef SDK_VERSION_2_3
    enum SmoothMode
    {
        SMOOTH_DISABLE = 0x00,
        SMOOTH_ENABLE = 0x01
    };
#endif // SDK_VERSION_3_0

  public:
    Flight(CoreAPI *ContorlAPI = 0);

    void task(TASK taskname, CallBack TaskCallback = 0);
    void setArm(bool enable, CallBack ArmCallback = 0);
    void setFlight(FlightData *data);
    void setFlight(uint8_t ControlFlag = 0x00);

    QuaternionData getQuaternion() const;
    PossitionData getPossition() const;
    VelocityData getVelocity() const;
    CommonData getAcceleration() const;
    CommonData getPalstance() const;
    MagnetData getMagnet() const;

  public: //! @note callbacks
    static void armCallback(CoreAPI *This, Header *header);
    static void taskCallback(CoreAPI *This, Header *header);

  public: //! @note Access method
    CoreAPI *getApi() const;
    void setApi(CoreAPI *value);

  private:
    CoreAPI *api;
    TaskData taskData;
};

} // namespace onboardSDK
} // namespace DJI

#endif // DJI_FLIGHT_H
