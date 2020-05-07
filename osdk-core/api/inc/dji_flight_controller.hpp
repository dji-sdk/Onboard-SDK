/** @file dji_flight_controller.hpp
 *  @version 3.9
 *  @date August 2019
 *
 *  @brief Implementation of flight controller
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

#ifndef DJI_FLIGHT_CONTROLLER_HPP
#define DJI_FLIGHT_CONTROLLER_HPP

#include "dji_flight_actions_module.hpp"
#include "dji_flight_assistant_module.hpp"
#include "dji_flight_joystick_module.hpp"
#include "dji_telemetry.hpp"
namespace DJI {
namespace OSDK {

/*! @brief Flight controller API: set or get parameter, execute flight actions.
 *
 */
class FlightController {
 public:
  FlightController(Vehicle *vehicle);
  ~FlightController();

  typedef Telemetry::HomeLocationData HomeLocation;
  typedef FlightAssistant::RtkEnableData
      RtkEnabled; /*!< 0: disable, 1: enable*/
  typedef FlightAssistant::AvoidEnable AvoidEnable;
  typedef FlightAssistant::UpwardsAvoidEnable UpwardsAvoidEnable;
  typedef FlightAssistant::GoHomeAltitude
      GoHomeHeight; /*!< unit:meter, range 20~500*/
  typedef FlightJoystick::ControlCommand JoystickCommand;
  typedef enum FlightJoystick::HorizontalLogic HorizontalLogic;
  typedef enum FlightJoystick::VerticalLogic VerticalLogic;
  typedef enum FlightJoystick::YawLogic YawLogic;
  typedef enum FlightJoystick::HorizontalCoordinate HorizontalCoordinate;
  typedef enum FlightJoystick::StableMode StableMode;
  typedef enum FlightActions::KillSwitch KillSwitch;

  typedef struct JoystickMode {
    HorizontalLogic horizontalLogic;
    VerticalLogic verticalLogic;
    YawLogic yawLogic;
    HorizontalCoordinate horizontalCoordinate;
    StableMode stableMode;
  } JoystickMode;

  /*! @brief Set RTK enable or disable, blocking calls
   *
   *  @param rtkEnable reference in RtkEnabled, RTK_DISABLE: disable,
   * RTK_ENABLE: enable
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setRtkEnableSync(RtkEnabled rtkEnable, int timeout);

  /*! @brief Set RTK enable or disable, non-blocking calls.
   *
   *  @param rtkEnable reference in RtkEnabled, RTK_DISABLE: disable,
   * RTK_ENABLE: enable
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode  ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   *  is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setRtkEnableAsync(RtkEnabled rtkEnable,
                         void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                              UserData userData),
                         UserData userData);

  /*! @brief Get rtk enable or disable, blocking calls.
   *
   *  @param rtkEnable reference in RtkEnabled, RTK_DISABLE: disable,
   * RTK_ENABLE: enable
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getRtkEnableSync(RtkEnabled &rtkEnable, int timeout);

  /*! @brief Get RTK enable or disable, non-blocking calls.
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode ErrorCode::ErrorCodeType error code
   *  @arg @b rtkEnable reference in RtkEnabled, RTK_DISABLE: disable,
   * RTK_ENABLE: enable
   *  @arg @b userData the interface to transfer userData in when the callback
   * is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getRtkEnableAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                              RtkEnabled rtkEnable,
                                              UserData userData),
                         UserData userData);

  /*! @brief Set go home altitude, blocking calls.
   *
   *  @note If aircraft's current altitude is higher than the setting value of
   * go home altitude, aircraft will go home using current altitude. Otherwise,
   * it will climb to setting of go home altitude ,and then execute go home
   * action. The details could be find in the documentation.
   * Go home altitude setting is between MIN_GO_HOME_HEIGHT to
   * MAX_FLIGHT_HEIGHT.
   * @param altitude go home altitude settings, unit: meter
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setGoHomeAltitudeSync(GoHomeHeight altitude,
                                                 int timeout);

  /*! @brief Set go home altitude, non-blocking calls.
   *
   *  @note If aircraft's current altitude is higher than the setting value of
   * go home altitude, aircraft will go home using current altitude. Otherwise,
   * it will climb to setting of go home altitude ,and then execute go home
   * action. The details could be find in the documentation.
   * Go home altitude setting is between MIN_GO_HOME_HEIGHT to
   * MAX_FLIGHT_HEIGHT.
   * @param altitude go home altitude settings, unit: meter
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   * is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setGoHomeAltitudeAsync(
      GoHomeHeight altitude,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Get go home altitude, blocking calls.
   *
   *  @param altitude go home altitude
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getGoHomeAltitudeSync(GoHomeHeight &altitude,
                                                 int timeout);

  /*! @brief Get go home altitude, non-blocking calls.
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode ErrorCode::ErrorCodeType error code
   *  @arg @b altitude go home altitude
   *  @arg @b userData the interface to transfer userData in when the callback
   * is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getGoHomeAltitudeAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           GoHomeHeight altitude, UserData userData),
      UserData userData);

  /*! @brief Set collision avoidance enable or disable, blocking calls
   *
   *  @param avoidEnable AvoidEnable, AVOID_DISABLE: disable, AVOID_ENABLE: enable
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setCollisionAvoidanceEnabledSync(
      AvoidEnable avoidEnable, int timeout);

  /*! @brief Set collision avoidance enable or disable, non-blocking calls
   *
   *  @param avoidEnable AvoidEnable, AVOID_DISABLE: disable, AVOID_ENABLE: enable
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode  OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   *  is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setCollisionAvoidanceEnabledAsync(
    AvoidEnable avoidEnable,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData);

  /*! @brief Get collision avoidance enable or disable, blocking calls
   *
   *  @param avoidEnable AvoidEnable, AVOID_DISABLE: disable, AVOID_ENABLE: enable
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getCollisionAvoidanceEnabledSync(
    AvoidEnable &avoidEnable, int timeout);

  /*! @brief Get collision avoidance enable or disable, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode the OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b avoidEnable AvoidEnable, AVOID_DISABLE: disable, AVOID_ENABLE: enable
   *  @arg @b userData the interface to pass userData in when the callback is
   *  called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getCollisionAvoidanceEnabledAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         AvoidEnable avoidEnable, UserData userData),
    UserData userData);


  /*! @brief Set upwards avoidance enable or disable, blocking calls
   *
   *  @param UpwardsAvoidEnable UpwardsAvoidEnable  UPWARDS_AVOID_DISABLE: disable,
   *  UPWARDS_AVOID_ENABLE: enable
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setUpwardsAvoidanceEnabledSync(UpwardsAvoidEnable upwardsAvoidEnable,
                                                          int timeout);

  /*! @brief Set upwards avoidance enable or disable, non-blocking calls
   *
   *  @param upwardsAvoidEnable UpwardsAvoidEnable  UPWARDS_AVOID_DISABLE: disable,
   *  UPWARDS_AVOID_ENABLE: enable
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode  OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   *  is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setUpwardsAvoidanceEnabledAsync(UpwardsAvoidEnable upwardsAvoidEnable,
                                       void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                                            UserData userData),
                                       UserData userData);

  /*! @brief Get upwards avoidance enable or disable, blocking calls
   *
   *  @param upwardsAvoidEnable UpwardsAvoidEnable  UPWARDS_AVOID_DISABLE: disable,
   *  UPWARDS_AVOID_ENABLE: enable
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getUpwardsAvoidanceEnabledSync(UpwardsAvoidEnable &upwardsAvoidEnable,
                                                          int timeout);

  /*! @brief Get upwards avoidance enable or disable, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode the OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b upwardsEnable UpwardsAvoidEnable, UPWARDS_AVOID_DISABLE: disable,
   *  UPWARDS_AVOID_ENABLE: enable
   *  @arg @b userData the interface to pass userData in when the callback is
   *  called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getUpwardsAvoidanceEnabledAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                                            UpwardsAvoidEnable upwardsEnable,
                                                            UserData userData),
                                       UserData userData);


  /*! @brief Set customized GPS(not RTK) home location, blocking calls.
   *
   *  @note  Set customized home location failed reason may as follows:
   *  1. The distance between new home location and last home location is larger
   * than MAX_FLY_RADIUS(20km).
   *  2. Set init home location failed after start aircraft.
   *  @param homeLocation HomeLocation include latitude and longitude
   *  @param timeout blocking timeout in seconds
   *  @return  ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setHomeLocationSync(HomeLocation homeLocation,
                                               int timeout);

  /*! @brief Set customized GPS(not RTK) home location, non-blocking calls.
   *
   *  @note  Set customized home location failed reasons may as follows:
   *  1. The distance between new home location and last home location is larger
   * than MAX_FLY_RADIUS(20km)
   *  2. Set init home location failed after start aircraft.
   *  @param homeLocation  HomeLocation include latitude and longitude
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   * is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setHomeLocationAsync(
      HomeLocation homeLocation,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Set home location using current aircraft GPS(not RTK) location,
   * blocking calls.
   *
   *  @note  Set home location failed reasons may as follows:
   *  1. Aircraft's gps level can't reach the condition of recording home
   * location.
   *  2. Set init home location failed after start aircraft.
   *  @param timeout blocking timeout in seconds
   */
  ErrorCode::ErrorCodeType setHomeLocationUsingCurrentAircraftLocationSync(
      int timeout);

  /*! @brief Set home location using current aircraft GPS(not RTK) location,
   * non-blocking calls.
   *
   *  @note  Set home location failed reasons may as follows:
   *  1. Aircraft's gps level can't reach the condition of recording home
   * location.
   *  2. There's no init home location before you set.
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   * is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setHomeLocationUsingCurrentAircraftLocationAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Wrapper function for turn on motors, blocking calls.
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType turnOnMotorsSync(int timeout);

  /*! @brief Wrapper function for turn on motors, non-blocking calls.
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   *  is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void turnOnMotorsAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                              UserData userData),
                         UserData userData);

  /*! @brief Wrapper function for turn off motors, blocking calls.
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType turnOffMotorsSync(int timeout);

  /*! @brief Wrapper function for turn off motors, non-blocking calls.
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   *  is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void turnOffMotorsAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                              UserData userData),
                         UserData userData);


  /*! @brief Wrapper function for aircraft takeoff, blocking calls.
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startTakeoffSync(int timeout);

  /*! @brief Wrapper function for aircraft takeoff, non-blocking calls.
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   *  is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startTakeoffAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                              UserData userData),
                         UserData userData);

  /*! @brief Wrapper function for aircraft landing, blocking calls.
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startLandingSync(int timeout);

  /*! @brief Wrapper function for aircraft landing, non-blocking calls.
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   *  is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startLandingAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                              UserData userData),
                         UserData userData);

  /*! @brief Wrapper function for cancel aircraft landing, blocking calls.
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType cancelLandingSync(int timeout);

  /*! @brief Wrapper function for cancel aircraft landing, non-blocking calls.
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   *  is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void cancelLandingAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                              UserData userData),
                         UserData userData);

  /*! @brief Wrapper function for aircraft force landing, blocking calls.
   *
   *  @note This api will ignore the smart landing function, when use this
   *  api, it will landing directly (would not stop at 0.7m and wait
   * user's  command),it may make the aircraft crash.
   *  @param timeout blocking timeout in seconds
   *  @return  ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startForceLandingSync(int timeout);

  /*! @brief Wrapper function for aircraft force landing, non-blocking calls.
   *
   *  @note This api will ignore the smart landing function, when use this
   * api, it will landing directly (would not stop at 0.7m and wait
   * user's  command),it may make the aircraft crash.
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   *  is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startForceLandingAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Wrapper function for  aircraft confirm landing and avoid ground,
   * blocking calls.
   *
   *  @note When the clearance between the aircraft and the ground is less than
   * 0.7m, the aircraft will pause landing and wait for user's confirmation.This
   * api use for confirm landing. If the ground is not suitable for landing
   *  ,user must use RC to control it landing manually or force landing.
   *  @param timeout blocking timeout in seconds
   *  @return  ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startConfirmLandingSync(int timeout);

  /*! @brief Wrapper function for  aircraft confirm landing and avoid ground,
   * non-blocking calls.
   *
   *
   *  @note When the clearance between the aircraft and the ground is less than
   * 0.7m, the aircraft will pause landing and wait for user's confirmation.This
   * api use for confirm landing. If the ground is not suitable for landing
   *  ,user must use RC to control it landing manually or force landing.
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   * is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startConfirmLandingAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Wrapper function for  go home action, blocking calls.
   *
   *  @param timeout blocking timeout in seconds
   *  @return  ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startGoHomeSync(int timeout);

  /*! @brief Wrapper function for  go home action, non-blocking calls.
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   * is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startGoHomeAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                             UserData userData),
                        UserData userData);

  /*! @brief Wrapper function for stop go home action, blocking calls.
   *
   *  @param timeout blocking timeout in seconds
   *  @return  ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType cancelGoHomeSync(int timeout);

  /*! @brief Wrapper function for stop go home action, non-blocking calls.
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   * is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void cancelGoHomeAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                             UserData userData),
                        UserData userData);

  /*! @brief Wrapper function for set joystick mode, non-blocking calls.
   *
   *  @param JoystickMode FlightController::JoystickMode include  horizontal
   *  logic,vertical logic,yaw logic, horizontal coordinate, stable mode.
   */
  void setJoystickMode(const JoystickMode &joystickMode);

  /*! @brief Wrapper function for set joystick command, non-blocking calls.
   *
   *  @param JoystickCommand FlightController::JoystickCommand  include
   *  x, y, z and yaw's command.
   */
  void setJoystickCommand(const JoystickCommand &JoystickCommand);

  /*! @brief Wrapper function for set joystick action, non-blocking calls.
   *
   *  @note User must set the joystick mode and command before using
   *  this function to execute the command.
   */
  void joystickAction();

  /*! @brief Wrapper function for get joystick mode, non-blocking calls.
   *
   *  @param JoystickMode FlightController::JoystickMode include  horizontal
   *  logic,vertical logic,yaw logic, horizontal coordinate, stable mode.
   */
  void getJoystickMode(FlightJoystick::ControlMode &joystickMode);

  /*! @brief Wrapper function for get joystick command, non-blocking calls.
   *
   *  @param JoystickCommand FlightController::JoystickCommand  include
   *  x, y, z and yaw's command.
   */
  void getJoystickCommand(JoystickCommand &joystickCommand);


  ErrorCode::ErrorCodeType killSwitch(KillSwitch cmd,int wait_timeout,
                                                     char debugMsg[10]);

 private:
  FlightAssistant *flightAssistant;
  FlightActions *flightActions;
  FlightJoystick *flightJoystick;
};
}  // namespace OSDK
}  // namespace DJI
#endif  // DJI_FLIGHT_CONTROLLER_HPP
