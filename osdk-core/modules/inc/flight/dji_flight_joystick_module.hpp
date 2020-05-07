/** @file dji_flight_joystick_module.hpp
 *  @version 4.0
 *  @date August 2019
 *
 *  @brief Implementation of flight actions module
 *
 *  @Copyright (c) 2019 DJI
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
#ifndef DJI_FLIGHT_JOYSTICK_MODULE_HPP
#define DJI_FLIGHT_JOYSTICK_MODULE_HPP

#include "dji_vehicle_callback.hpp"
namespace DJI {
namespace OSDK {
class Vehicle;
class FlightLink;
class FlightJoystick {
 public:

  /*! @brief bit 7:6 of the 8-bit (7:0) CtrlData.flag
   *
   *  @note
   *        - Only when the GPS signal is good (health_flag >=3)，horizontal
   * position control (HORIZONTAL_POSITION) related control modes can be used.
   *        - Only when GPS signal is good (health_flag >=3)，or when
   * AdvancedSensing
   * system is working properly with Autopilot，
   *          horizontal velocity control（HORIZONTAL_VELOCITY）related control
   * modes can be used.
   */
  enum HorizontalLogic {
    /*!
     - Set the control-mode to control pitch & roll angle of the vehicle.
     - Need to be referenced to either the ground or
     body frame by HorizontalCoordinate setting.
     - Limit: 35 degree
     */
    HORIZONTAL_ANGLE = 0,
    /*!
     - Set the control-mode to control horizontal vehicle velocities.
     - Need to be referenced to either the ground or body frame by
     HorizontalCoordinate setting.
     - Limit: 30 m/s
     */
    HORIZONTAL_VELOCITY = 1,
    /*!
     - Set the control-mode to control position offsets of pitch & roll directions
     - Need to be referenced to either the ground r body frame by HorizontalCoordinate setting.
     - Limit: N/A
     */
    HORIZONTAL_POSITION = 2,
    /*!
     - Set the control-mode to control rate of change of the vehicle's attitude
     - Need to be referenced to either the ground or body frame by HorizontalCoordinate setting.
     - Limit: 150.0 deg/s
     */
    HORIZONTAL_ANGULAR_RATE = 3
  };

  /*! @brief bit 5:4 of the 8-bit (7:0) CtrlData.flag
   *
   *  We suggest developers do not use VERTICAL_POSITION control mode indoor
   * when your UAV flight height is larger than 3 meters.
   *  This is because in indoor environments, barometer can be inaccurate, and
   * the vertical controller may fail to keep the height of the UAV.
   */
  enum VerticalLogic {
    /*!
     - Set the control-mode to control the vertical
       speed of UAV, upward is positive
     - Limit: -5 to 5 m/s
     */
    VERTICAL_VELOCITY = 0,
    /*!
     - Set the control-mode to control the height of UAV
     - Limit: 0 to 120 m
     */
    VERTICAL_POSITION = 1,
    /*!
     - Set the control-mode to directly control the thrust
     - Range: 0% to 100%
     */
    VERTICAL_THRUST = 2,
  };

  /*! @brief bit 3 of the 8-bit (7:0) CtrlData.flag
   */
  enum YawLogic {
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
    YAW_RATE = 1
  };

  /*! @brief bit 2:1 of the 8-bit (7:0) CtrlData.flag
   */
  enum HorizontalCoordinate {
    /*! Set the x-y of ground frame as the horizontal frame (NEU) */
    HORIZONTAL_GROUND = 0,
    /*! Set the x-y of body frame as the horizontal frame (FRU) */
    HORIZONTAL_BODY = 1
  };

  /*!
   * @brief bit 0 of the 8-bit (7:0) CtrlData.flag.
   *
   * Only works in Horizontal velocity control mode
   * In velocity stable mode, drone will brake and hover at one position once
   * the input command is zero.
   * Drone will try to stay in position once in hover state.
   *
   * In velocity non-stable mode, drone will follow the velocity command and
   * doesn’t hover when the command is zero.
   * That’s to say drone will drift with the wind.
   */
  enum StableMode {
    STABLE_DISABLE = 0, /*!< Disable the stable mode */
    STABLE_ENABLE = 1   /*!< Enable the stable mode */
  };
#pragma pack(1)

  /*! @brief CtrlData used for flight control.
    *
    */
  typedef struct ControlCommand {
    float32_t x;   /*!< Control with respect to the x axis of the
                        DJI::OSDK::Control::HorizontalCoordinate.*/
    float32_t y;   /*!< Control with respect to the y axis of the
                        DJI::OSDK::Control::HorizontalCoordinate.*/
    float32_t z;   /*!< Control with respect to the z axis, up is positive. */
    float32_t yaw; /*!< Yaw position/velocity control w.r.t. the ground frame.*/
  } ControlCommand;// pack(1)

  typedef struct ControlMode {
    uint8_t stableMode : 1;
    uint8_t horizFrame : 2;
    uint8_t yawMode    : 1;
    uint8_t vertiMode  : 2;
    uint8_t horizMode  : 2;
  } ControlMode;// pack(1)

  typedef struct CtrlData {
    ControlMode controlMode;
    ControlCommand controlCommand;
  } CtrlData;     // pack(1)

  /*! @brief AdvancedCtrlData
   *
   *  @note for flag, x, y, z, yaw definition see CtrlData.
   */
  typedef struct AdvancedCtrlData {
    ControlMode controlMode;
    uint8_t advFlag;
    ControlCommand controlCommand;
    float32_t xFeedforward;
    float32_t yFeedforward;
  } AdvancedCtrlData;  // pack(1)

  typedef struct KillSwitchData {
    uint8_t high_version;          /*!< version # for FW version match */
    uint8_t low_version;           /*!< version # for protocol compatibility */
    uint8_t debug_description[10]; /*!< insert reasons for emergency stop for
                                  debugging purpose */
    uint8_t cmd : 2;   /*!< 0: no action, 1: motor stop with protection,
                        * 2: disarm protection */
    uint8_t reserved : 6;
  } KillSwitchData;  // pack(1)
#pragma pack()

 public:
  FlightJoystick(Vehicle *vehicle);
  ~FlightJoystick();

  void setHorizontalLogic(HorizontalLogic horizontalLogic);
  void setVerticalLogic(VerticalLogic verticalLogic);
  void setYawLogic(YawLogic yawLogic);
  void setHorizontalCoordinate(HorizontalCoordinate horizontalCoordinate);
  void setStableMode(StableMode stableMode);

  void setControlCommand(const ControlCommand &controlCommand);
  void joystickAction();

  void getControlCommand(ControlCommand &controlCommand);
  void getControlMode(ControlMode &controlCommand);

 private:
  CtrlData ctrlData;
  FlightLink *flightLink;
};
}
}

#endif  // DJI_FLIGHT_JOYSTICK_MODULE_HPP
