/** @file dji_control.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Control API for DJI OSDK library
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef DJI_CONTROL_H
#define DJI_CONTROL_H

#include "dji_ack.hpp"
#include "dji_open_protocol.hpp"
#include "dji_type.hpp"
#include "dji_vehicle_callback.hpp"

namespace DJI
{
namespace OSDK
{

// Forward Declarations
class Vehicle;

/*! @brief Flight control API: high-level actions and low-level control modes
 *
 */
class Control
{

public:
  /*! @brief Flight control commands
   */
  class FlightCommand
  {
  public:
    /*
     * @note Matrice 100 flight commands
     */
    typedef struct LegacyCMD
    {
      const static int goHome  = 1;
      const static int takeOff = 4;
      const static int landing = 6;
    } LegacyCMD;

    /*
     * @note OSDK release 3.3
     */
    const static int takeOff = 1; /*!< vehicle takeoff*/
    const static int landing = 2; /*!< vehicle landing*/
    //! @note independent mode courseLock,
    //! cannot be controlled through SDK
    const static int courseLock       = 5;
    const static int goHome           = 6; /*!< vehicle return home position*/
    const static int startMotor       = 7;
    const static int stopMotor        = 8;
    const static int calibrateCompass = 9;
    const static int exitGoHome       = 12;
    const static int exitTakeOff      = 13;
    const static int exitLanding      = 14;
    const static int exitCalibrateCompass = 21;
    const static int landingGearDown      = 28;
    const static int landingGearUp        = 29;
  };
  // clang-format off
  /*! @brief bit 5:4 of the 8-bit (7:0) CtrlData.flag
   *
   *  We suggest developers do not use VERTICAL_POSITION control mode indoor
   * when your UAV
   *  flight height is larger than 3 meters.
   *  This is because in indoor environments, barometer can be inaccurate, and
   * the
   *  vertical controller may fail to keep the height of the UAV.
   */
  enum VerticalLogic
  {
    /*!
     - Set the control-mode to control the vertical
       speed of UAV, upward is positive
     - Limit: -5 to 5 m/s
     */
    VERTICAL_VELOCITY = 0x00,
    /*!
     - Set the control-mode to control the height of UAV
     - Limit: 0 to 120 m
     */
    VERTICAL_POSITION = 0x10,
    /*!
     - Set the control-mode to directly control the thrust
     - Range: 0% to 100%
     */
    VERTICAL_THRUST = 0x20,
  };

  /*! @brief bit 7:6 of the 8-bit (7:0) CtrlData.flag
   *
   *  @note
   *        - Only when the GPS signal is good (health_flag >=3)，horizontal
   * position control (HORIZONTAL_POSITION) related control modes can be used.
   *        - Only when GPS signal is good (health_flag >=3)，or when AdvancedSensing
   * system is working properly with Autopilot，
   *          horizontal velocity control（HORIZONTAL_VELOCITY）related control
   * modes can be used.
   */
  enum HorizontalLogic
  {
    /*!
     - Set the control-mode to control pitch & roll
     angle of the vehicle.
     - Need to be referenced to either the ground or
     body frame by HorizontalCoordinate setting.
     - Limit: 35 degree
     */
    HORIZONTAL_ANGLE = 0x00,
    /*!
     - Set the control-mode to control horizontal
     vehicle velocities.
     - Need to be referenced to either the ground
     or body frame by HorizontalCoordinate setting.
     - Limit: 30 m/s
     */
    HORIZONTAL_VELOCITY = 0x40,
    /*!
     - Set the control-mode to control position
     offsets of pitch & roll directions
     - Need to be referenced to either the ground
     or body frame by HorizontalCoordinate setting.
     - Limit: N/A
     */
    HORIZONTAL_POSITION = 0x80,
    /*!
     - Set the control-mode to control rate of
     change of the vehicle's attitude
     - Need to be referenced to either the ground
     or body frame by HorizontalCoordinate setting.
     - Limit: 150.0 deg/s
     */
    HORIZONTAL_ANGULAR_RATE = 0xC0
  };
  /*! @brief bit 3 of the 8-bit (7:0) CtrlData.flag
   */
  enum YawLogic
  {
    /*!
     - Set the control-mode to control yaw angle.
     - Yaw angle is referenced to the ground frame.
     - In this control mode, Ground frame is enforeced in Autopilot.
     */
    YAW_ANGLE = 0x00,
    /*!
     - Set the control-mode to control yaw angular velocity.
     - Same reference frame as YAW_ANGLE.
     - Limite: 150 deg/s
     */
    YAW_RATE = 0x08
  };

  /*! @brief bit 2:1 of the 8-bit (7:0) CtrlData.flag
   */
  enum HorizontalCoordinate
  {
    /*! Set the x-y of ground frame as the horizontal frame (NEU) */
    HORIZONTAL_GROUND = 0x00,
    /*! Set the x-y of body frame as the horizontal frame (FRU) */
    HORIZONTAL_BODY = 0x02
  };

  /*!
   * @brief bit 0 of the 8-bit (7:0) CtrlData.flag.
   *
   * Only works in Horizontal velocity control mode
   * In velocity stable mode, drone will brake and hover at one position once the input command is zero.
   * Drone will try to stay in position once in hover state.
   *
   * In velocity non-stable mode, drone will follow the velocity command and doesn’t hover when the command is zero.
   * That’s to say drone will drift with the wind.
   */
  enum StableMode
  {
    STABLE_DISABLE = 0x00, /*!< Disable the stable mode */
    STABLE_ENABLE  = 0x01  /*!< Enable the stable mode */
  };

  /*!
   * @brief turn on or off the motors for emergency reasons
   */
  enum KillSwitch
  {
    ENABLE  = 0x01, /*!< Enable the killswitch */
    DISABLE = 0x02  /*!< Disable the killswitch */
  };
// clang-format on

/*! The struct for CtrlData

 */
#pragma pack(1)
  /*! @brief CtrlData used for flight control.
    *
    */
  typedef struct CtrlData
  {
    uint8_t flag;  /*!< control data flag consists of 8 bits.
 
                      - CtrlData.flag = ( DJI::OSDK::Control::HorizontalLogic |
                      DJI::OSDK::Control::VerticalLogic |
                      DJI::OSDK::Control::YawLogic |
                      DJI::OSDK::Control::HorizontalCoordinate |
                      DJI::OSDK::Control::StableMode)
                   */
    float32_t x;   /*!< Control with respect to the x axis of the
                      DJI::OSDK::Control::HorizontalCoordinate.*/
    float32_t y;   /*!< Control with respect to the y axis of the
                      DJI::OSDK::Control::HorizontalCoordinate.*/
    float32_t z;   /*!< Control with respect to the z axis, up is positive. */
    float32_t yaw; /*!< Yaw position/velocity control w.r.t. the ground frame.*/

    /*!
     * \brief CtrlData initialize the CtrlData variable.
     * \param in_flag   See CtrlData.flag
     * \param in_x      See CtrlData.x
     * \param in_y      See CtrlData.y
     * \param in_z      See CtrlData.z
     * \param in_yaw    See CtrlData.yaw
     */
    CtrlData(uint8_t in_flag, float32_t in_x, float32_t in_y, float32_t in_z,
             float32_t in_yaw);
  } CtrlData; // pack(1)

  /*! @brief AdvancedCtrlData
   *
   *  @note for flag, x, y, z, yaw definition see CtrlData.
   */
  typedef struct AdvancedCtrlData
  {
    uint8_t   flag;
    uint8_t   advFlag;
    float32_t x;
    float32_t y;
    float32_t z;
    float32_t yaw;
    float32_t xFeedforward;
    float32_t yFeedforward;

    AdvancedCtrlData(uint8_t in_flag, float32_t in_x, float32_t in_y,
                     float32_t in_z, float32_t in_yaw, float32_t x_forw,
                     float32_t y_forw);
  } AdvancedCtrlData; // pack(1)

  // CMD data supported in Matrice 100
  typedef struct LegacyCMDData
  {
    uint8_t sequence;
    uint8_t cmd;
  } LegacyCMDData; // pack (1)

  typedef struct KillSwitchData
  {
    uint8_t high_version;    /*!< version # for FW version match */
    uint8_t low_version;     /*!< version # for protocol compatibility */
    uint8_t debug_description[10]; /*!< insert reasons for emergency stop for debugging purpose */
    uint8_t cmd : 2;  /*!< 0: no action, 1: motor stop with protection, 2: disarm protection */
    uint8_t reserved : 6;
  } KillSwitchData; // pack(1)


#pragma pack()

  /*! @note
   *
   *  Basically control class provide two functions:
   *  1. action() which implements CMD_ID_TASK
   *  2. modeCtrl() which implements CMD_ID_CONTROL
   *
   *  The rest of the functions are just wrapper functions
   *  that provide commonly used control functions
   *  EX: takeoff, landing, and position control mode
   */
private:
  const int wait_timeout;

public:
  Control(Vehicle* vehicle = 0);
  ~Control();

  Vehicle* vehicle;

  /*! @brief Basic action command for the vehicle, see FlightCommand for cmd
   * choices
   *
   *  @param cmd action command from FlightCommand
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void action(const int cmd, VehicleCallBack callback = 0,
              UserData userData = 0);
  /*! @brief Control the vehicle using user-specified mode, see FlightCommand
   * for cmd choices
   *
   *  @param cmd action command from FlightCommand
   *  @param timeout timeout to wait for ACK
   *  @return ErrorCode
   */
  ACK::ErrorCode action(const int cmd, int timeout);

  /*! @brief Wrapper function for arming the motors
   *
   *  @return ACK::ErrorCode struct with the acknowledgement from the FC
   */
  ACK::ErrorCode armMotors(int wait_timeout);
  /*! @brief Wrapper function for arming the motors
   *
   *  @note If user does not provide his/her own callback, default callback
   *  will be executed
   */
  void armMotors(VehicleCallBack callback = 0, UserData userData = 0);

  /*! @brief Wrapper function for disarming the motors
   *
   *  @return ACK::ErrorCode struct with the acknowledgement from the FC
   */
  ACK::ErrorCode disArmMotors(int wait_timeout);
  /*! @brief Wrapper function for disarming the motors
   *
   *  @note If user does not provide his/her own callback, default callback
   *  will be executed
   */
  void disArmMotors(VehicleCallBack callback = 0, UserData userData = 0);

  /*! @brief Wrapper function for take off
   *
   *  @return ACK::ErrorCode struct with the acknowledgement from the FC
   */
  ACK::ErrorCode takeoff(int wait_timeout);
  /*! @brief Wrapper function for take off
   *
   *  @note If user does not provide his/her own callback, default callback
   *  will be executed
   */
  void takeoff(VehicleCallBack callback = 0, UserData userData = 0);

  /*! @brief Wrapper function for go Home
   *
   *  @return ACK::ErrorCode struct with the acknowledgement from the FC
   */
  ACK::ErrorCode goHome(int wait_timeout);
  /*! @brief Wrapper function for go Home
   *
   *  @note If user does not provide his/her own callback, default callback
   *  will be executed
   */
  void goHome(VehicleCallBack callback = 0, UserData userData = 0);

  /*! @brief Wrapper function for landing
   *
   *  @return ACK::ErrorCode struct with the acknowledgement from the FC
   */
  ACK::ErrorCode land(int wait_timeout);
  /*! @brief Wrapper function for landing
   *
   *  @note If user does not provide his/her own callback, default callback
   *  will be executed
   */
  void land(VehicleCallBack callback = 0, UserData userData = 0);

  /*! @brief Control the vehicle using user-specified mode
   *
   *  @param data control set-points and flags
   *
   * @details Control Mode Byte
   *
   *  bit 7:6                  | bit 5:4           | bit 3          | bit 2:1            | bit 0                |
   *  ----------------         | ----------------- | -------------- | -----------------  | -----------------    |
   *  0b00: HORI_ATTI_TILT_ANG | 0b00: VERT_VEL    | 0b00: YAW_ANG  | 0b00: ground frame | 0b0: non-stable mode |
   *  0b01: HORI_VEL           | 0b01: VERT_POS    | 0b01: YAW_RATE | 0b01: body frame   | 0b1: stable mode     |
   *  0b10: HORI_POS           | 0b10: VERT_THRUST |                | (horizontal frame) |                      |
   *  0b11: HORI_ANG_VEL       |                   |                |                    |                      |
   *
   * @details Command Value
   *
   * ctrl_flag | ctrl_byte | roll_or_x | pitch_or_y | thr_z   | yaw     | feedforward_x | feedforward_y
   * --------  | --------  | --------- | ---------- | ------- | ------- | -----------   | -------------
   * uint8_t   | uint8_t   | float32   | float32    | float32 | float32 | float32       | float32
   *
   *
   * | Mode        | Input Limit |
   * |-------------|-------------|
   * | vert_thrust | 0 to 100 %  |
   * | vert_vel    | -5 to 5 m/s |
   * | vert_pos    | 0 to 120m   |
   * | hori_ang    | 35 degrees  |
   * | hori_vel    | 30 m/s      |
   * | hori_acc    | 7 m/s^2     |
   * | yaw_rate    | 150 deg/s   |
   *
   */
  void flightCtrl(CtrlData data);

  /*! @brief Control the vehicle using user-specified mode (overloaded)
   *
   *  @note this mode only works in HORIZONTAL_VELOCITY and the unit of
   *  feedforward term is m/s^2
   *
   *  @param data control set-points and flags
   *
   *
   * @details Control Advanced Byte
   *
   * bit 7                 | bit 6                 | bit 5         | bit 4:3                 | bit 2:0     |
   * -----------------     | -----------------     | ------------- | -----------------       | ----------- |
   * adv flag              | brake_flag            | yaw_ctrl_flag | avoid_atti_limit_enable | reserved    |
   * (1 for ON, 0 for OFF) | (1 for ON, 0 for OFF) |               | (1 for ON, 2 for OFF)   |             |
   *
   * @details If adv_flag is ON, feedforward of x axis and y axis will work in horiz_vel
   * @details If brake_flag is ON, emergency brake will work when you use joystick.
   * @details If avoid_atti_limit_enable is ON, copter's atti limit will use avoid_atti_limit, that means the camera could not see the propeller.
   * @details If avoid_atti_limit_enable is OFF, copter's atti limit won't use avoid_atti_limit, that means the camera may see the propeller.
   */
  void flightCtrl(AdvancedCtrlData data);

  /*! @brief Control the position and yaw angle of the vehicle.
   *  The reference frame is the DJI::OSDK::Control::HORIZONTAL_GROUND (NEU).
   *
   *  @param x position set-point in x axis of ground frame (m)
   *  @param y position set-point in y axis of ground frame (m)
   *  @param z position set-point in z axis of ground frame (m), input limit see
   * DJI::OSDK::Control::VERTICAL_POSITION
   *  @param yaw yaw set-point (deg)
   *
   */
  void positionAndYawCtrl(float32_t x, float32_t y, float32_t z, float32_t yaw);

  /*! @brief Control the velocity and yaw rate of the vehicle.
   *  The reference frame is the DJI::OSDK::Control::HORIZONTAL_GROUND (NEU).
   *
   *  @param Vx velocity set-point in x axis of ground frame (m/s), input limit
   * see DJI::OSDK::Control::HORIZONTAL_VELOCITY
   *  @param Vy velocity set-point in y axis of ground frame (m/s), input limit
   * see DJI::OSDK::Control::HORIZONTAL_VELOCITY
   *  @param Vz velocity set-point in z axis of ground frame (m/s), input limit
   * see DJI::OSDK::Control::VERTICAL_VELOCITY
   *  @param yawRate yawRate set-point (deg/s)
   */
  void velocityAndYawRateCtrl(float32_t Vx, float32_t Vy, float32_t Vz,
                              float32_t yawRate);

  /*! @brief Control the attitude and vertical position of the vehicle
   *
   *  @param roll   attitude set-point in x axis of body frame (FRU) (deg),
   * input limit see DJI::OSDK::Control::HORIZONTAL_ANGLE
   *  @param pitch  attitude set-point in y axis of body frame (FRU) (deg),
   * input limit see DJI::OSDK::Control::HORIZONTAL_ANGLE
   *  @param z      z position set-point in z axis of ground frame (NED) (m),
   * input limit see DJI::OSDK::Control::VERTICAL_POSITION
   *  @param yaw    attitude set-point in z axis of ground frame (NED) (deg)
   */
  void attitudeAndVertPosCtrl(float32_t roll, float32_t pitch, float32_t yaw,
                              float32_t z);

  /*! @brief Control the attitude rate and vertical position of the vehicle
   *
   *  @param rollRate   attitude rate set-point in x axis of body frame (FRU)
   * (deg/s)
   *  @param pitchRate  attitude rate set-point in y axis of body frame (FRU)
   * (deg/s)
   *  @param yawRate    attitude rate set-point in z axis of body frame (FRU)
   * (deg/s), input limit see DJI::OSDK::Control::YAW_RATE
   *  @param z          z position set-point in z axis of ground frame (NED)
   * (m), input limit see DJI::OSDK::Control::VERTICAL_POSITION
   */
  void angularRateAndVertPosCtrl(float32_t rollRate, float32_t pitchRate,
                                 float32_t yawRate, float32_t z);

  /*! @brief Stop the vehicle in horiz velocity, vert velocity, yaw rate mode
   * (body frame)
   *
   */
  void emergencyBrake();

  /*! @brief A callback function for action non-blocking calls
   *
   *  @param recvFrame the data comes with the callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   *
   */
  static void actionCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                             UserData userData);

  /*! @brief Turn on or off the kill switch
   *
   *  @param cmd enable or disable the kill switch
   *  @param wait_timeout timeout for blocking call
   *  @param debugMsg inject debug message to flight control FW for logging, size limit: 10 bytes
   *
   *  @return ACK::ErrorCode struct with the acknowledgement from the FC
   */
  ACK::ErrorCode killSwitch(KillSwitch cmd, int wait_timeout = 10, char debugMsg[10] = (char *)"OSDK_API");
  /*! @brief Turn on or off the kill switch
   *
   *  @param cmd enable or disable the kill switch
   *  @param debugMsg inject debug message to flight control FW for logging, size limit: 10 bytes
   *  @param callback callback function you want called upon ACK
   *  @param userData additional data you want the callback function to have access to
   *
   *  @note If user does not provide his/her own callback, default callback
   *  will be executed
   */
  void killSwitch(KillSwitch cmd, char debugMsg[10] = (char *)"OSDK_API", VehicleCallBack callback = 0, UserData userData = 0);

private:
  /*! @brief Wrapper function for arming/disarming the motors
   *  @note Supported in Matrice 100
   *  @return ACK::ErrorCode struct with the acknowledgment from the FC
   */
  ACK::ErrorCode setArm(bool armSetting, int timeout);
  /*! @brief Wrapper function for arming/disarming the motors
   *  @note Supported on Matrice 100. If user does not provide his/her
   *  own callback, default callback will be executed.
   */
  void setArm(bool armSetting, VehicleCallBack callback = 0,
              UserData userData = 0);

  /*
   * Task CMD data to send to the flight controller (supported in Matrice 100)
   */
  LegacyCMDData legacyCMDData;
}; // class Control

} // OSDK
} // DJI

#endif // DJI_CONTROL_H
