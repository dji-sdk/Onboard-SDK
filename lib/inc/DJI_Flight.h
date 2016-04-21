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
        VERTICAL_POSITION = 0x10,
        VERTICAL_THRUST = 0x20,
    };

    enum HorizontalLogic
    {
        HORIZONTAL_ANGLE = 0x00,
        HORIZONTAL_VELOCITY = 0x40,
        HORIZONTAL_POSITION = 0X80,
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

    enum Status
    {
        STATUS_GROUND_STANDBY = 1,
        STATUS_TAKE_OFF = 2,
        STATUS_SKY_STANDBY = 3,
        STATUS_LANDING = 4,
        STATUS_FINISHING_LANDING = 5,
    };

    enum Device
    {
        DEVICE_RC = 0,
        DEVICE_APP = 1,
        DEVICE_SDK = 2,
    };

    //! @todo rename
    enum Mode
    {
        ATTI_STOP = 0,
        HORIZ_ANG_VERT_VEL_YAW_ANG = 1,
        HORIZ_ANG_VERT_VEL_YAW_RATE = 2,
        HORIZ_VEL_VERT_VEL_YAW_ANG = 3,
        HORIZ_VEL_VERT_VEL_YAW_RATE = 4,
        HORIZ_POS_VERT_VEL_YAW_ANG = 5,
        HORIZ_POS_VERT_VEL_YAW_RATE = 6,
        HORIZ_ANG_VERT_POS_YAW_ANG = 7,
        HORIZ_ANG_VERT_POS_YAW_RATE = 8,
        HORIZ_VEL_VERT_POS_YAW_ANG = 9,
        HORIZ_VEL_VERT_POS_YAW_RATE = 10,
        HORIZ_POS_VERT_POS_YAW_ANG = 11,
        HORIZ_POS_VERT_POS_YAW_RATE = 12,
        HORIZ_ANG_VERT_THR_YAW_ANG = 13,
        HORIZ_ANG_VERT_THR_YAW_RATE = 14,
        HORIZ_VEL_VERT_THR_YAW_ANG = 15,
        HORIZ_VEL_VERT_THR_YAW_RATE = 16,
        HORIZ_POS_VERT_THR_YAW_ANG = 17,
        HORIZ_POS_VERT_THR_YAW_RATE = 18,
        GPS_ATII_CTRL_CL_YAW_RATE = 97,
        GPS_ATTI_CTRL_YAW_RATE = 98,
        ATTI_CTRL_YAW_RATE = 99,
        ATTI_CTRL_STOP = 100,
        MODE_NOT_SUPPORTED = 0xFF
    };

    /*! @note
     *  In order to keep your drone safe,
     *  you must keep sending FlightData to flight controller.
     *  Or your drone will hover.
     *
     *  Position control is a open-looped control.
     *  That means it accept incremental data, not absolute position data.
     * */
  public:
    Flight(CoreAPI *ControlAPI = 0);

    void task(TASK taskname, CallBack TaskCallback = 0, UserData userData = 0);
    void setArm(bool enable, CallBack ArmCallback = 0, UserData userData = 0);
    void control(uint8_t flag, float32_t x, float32_t y, float32_t z, float32_t yaw);
    void setFlight(FlightData *data); //! @note old interface

    QuaternionData getQuaternion() const;
    EulerianAngle getEulerianAngle() const;
    PositionData getPosition() const;
    VelocityData getVelocity() const;
    CommonData getAcceleration() const;
    CommonData getPalstance() const;
    MagnetData getMagnet() const;
    Device getControlDevice() const;
    Status getStatus() const;
    Mode getControlMode() const;

    Angle getYaw() const;
    Angle getRoll() const;
    Angle getPitch() const;

  public: //! @note callbacks
    static void armCallback(CoreAPI *This, Header *header, UserData userData = 0);
    static void taskCallback(CoreAPI *This, Header *header, UserData userData = 0);

  public: //! @note mathematical method
    static EulerianAngle toEulerianAngle(QuaternionData data);
    static QuaternionData toQuaternion(EulerianAngle data);

  public: //! @note Access method
    CoreAPI *getApi() const;
    void setApi(CoreAPI *value);

  private:
    CoreAPI *api;
    TaskData taskData;

/*! @note simulator API
 *
 * It could not respond like a real M100,
 * just a simple simulator
 *
 * */
#ifdef USE_SIMULATION
  public:
    bool isSimulating() const;
    void setSimulating(bool value);

  private:
    bool simulating;
    SpaceVector position;
    SpaceVector speed;
    EulerianAngle AngularSim;
#endif // USE_SIMULATION
};

class FlightUnitTest
{
    //! @todo implement
  public:
    FlightUnitTest();

  private:
    bool mathematicalMethod();
};

} // namespace onboardSDK
} // namespace DJI

#endif // DJI_FLIGHT_H
