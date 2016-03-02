/*! @brief
 *  @file DJI_Flight.h
 *  @version 3.0
 *  @date Dec 16, 2015
 *
 *  @abstract
 *  Flight Control API for DJI onboardSDK library
 *
 *  @attention
 *  Project configuration:
 *
 *  @version features:
 *  -* @version V3.0
 *  -* DJI-onboard-SDK for Windows,QT,STM32,ROS,Cmake
 *  -* @date Nov 15, 2015
 *  -* @author william.wu
 *
 * */
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
    uint8_t flag;
    float32_t x;
    float32_t y;
    float32_t z;
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

//! @version 2.3
    enum YawCoordinate
    {
        YAW_GROUND = 0x00,
        YAW_BODY = 0X01
    };
//! @version 3.1
    enum SmoothMode
    {
        SMOOTH_DISABLE = 0x00,
        SMOOTH_ENABLE = 0x01
    };

    /*! @note
     *  In order to keep your drone safe,
     *  you must keep sending FlightData to flight controller.
     *  Or your drone will hover.
     *
     *  Possion control is a open-looped control.
     *  That means it accept incremental data, not absolute possition data.
     * */
  public:
    Flight(CoreAPI *ControlAPI = 0);

    void task(TASK taskname, CallBack TaskCallback = 0, UserData userData = 0);
    void setArm(bool enable, CallBack ArmCallback = 0, UserData userData = 0);
    void setFlight(FlightData *data);
    void setFlight(uint8_t ControlFlag = 0x00);

    QuaternionData getQuaternion() const;
    PossitionData getPossition() const;
    VelocityData getVelocity() const;
    CommonData getAcceleration() const;
    CommonData getPalstance() const;
    MagnetData getMagnet() const;

  public: //! @note callbacks
    static void armCallback(CoreAPI *This, Header *header,UserData userData = 0);
    static void taskCallback(CoreAPI *This, Header *header,UserData userData = 0);

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
