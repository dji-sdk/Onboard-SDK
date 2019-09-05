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
namespace DJI {
namespace OSDK {

/*! @brief Flight controller API: set or get parameter, execute flight actions
 *
 */
class FlightController {
 public:
  FlightController(Vehicle *vehicle);
  ~FlightController();

  /*! @brief Set RTK enable or disable, blocking calls
   *
   *  @param rtkEnable RtkEnableData  RTK_DISABLE: disable, RTK_ENABLE: enable
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setRtkEnableSync(
      FlightAssistant::RtkEnableData rtkEnable, int timeout);

  /*! @brief Set RTK enable or disable, non-blocking calls
   *
   *  @param rtkEnable rtkEnableData, RTK_DISABLE: disable, RTK_ENABLE: enable
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode  OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   * is
   *  called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setRtkEnableAsync(FlightAssistant::RtkEnableData rtkEnable,
                         void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                              UserData userData),
                         UserData userData);

  /*! @brief Get rtk enable or disable, blocking calls
   *
   *  @param rtkEnable rtkEnableData, RTK_DISABLE: disable, RTK_ENABLE: enable
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getRtkEnableSync(
      FlightAssistant::RtkEnableData &rtkEnable, int timeout);

  /*! @brief get RTK enable or disable, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode the OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b rtkEnable rtkEnableData, RTK_DISABLE: disable, RTK_ENABLE: enable
   *  @arg @b userData the interface to trans userData in when the callback is
   *  called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getRtkEnableAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           FlightAssistant::RtkEnableData rtkEnable,
                           UserData userData),
      UserData userData);

  /*! @brief Set go home altitude, blocking calls
   *
   *  @note If current altitude is higher than settings, aircraft will go home
   *  by current altitude. the altitude setting is between 20m to 500m, if
   *  setting exceed this range, for example setting is 10m or 510m, it will
   * remind you
   * ErrorCode::FlightControllerErr::ParamReadWirteErr::InvalidParameter.
   *  @param altitude go home altitude settings must between MIN_GO_HOME_HEIGHT
   * and MAX_FLIGHT_HEIGHT
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setGoHomeAltitudeSync(
      FlightAssistant::GoHomeAltitude altitude, int timeout);

  /*! @brief Set go home altitude, non-blocking calls
   *
   *  @note If current altitude is higher than settings, aircraft will go home
   *  by current altitude. the altitude setting is between 20m to 500m, if
   *  setting exceed this range, for example setting is 10m or 510m, it will
   * remind you
   * ErrorCode::FlightControllerErr::ParamReadWirteErr::InvalidParameter.
   *  @param altitude go home altitude
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode the OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to trans userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setGoHomeAltitudeAsync(
      FlightAssistant::GoHomeAltitude altitude,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Get go home altitude, blocking calls
   *
   *  @param altitude go home altitude
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getGoHomeAltitudeSync(
      FlightAssistant::GoHomeAltitude &altitude, int timeout);

  /*! @brief Get go home altitude, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode the OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b altitude go home altitude
   *  @arg @b userData the interface to trans userData in when the callback is
   *  called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getGoHomeAltitudeAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           FlightAssistant::GoHomeAltitude altitude,
                           UserData userData),
      UserData userData);

  /*! @brief Set home location, blocking calls
   *
   *  @note  Set home location failed reason may as follows:
   *  1 Use the type DJI_HOMEPOINT_AIRCRAFT_LOACTON, but aircraft's gps level
   *  can't reach the status of record home location.
   *  2 The distance between new home location and init home location is larger
   * than
   *  MAX_FLY_RADIUS(20km)
   *  @param homeLocation SetHomeLocationData include latitude and longitude
   *  @param timeout blocking timeout in seconds
   *  @return  OSDK ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setHomeLocationSync(
      FlightAssistant::SetHomeLocationData homeLocation, int timeout);

  /*! @brief Set home location, non-blocking calls
   *
   *  @note  Set home location failed reason may as follows:
   *  1. Use the type DJI_HOMEPOINT_AIRCRAFT_LOACTON, but aircraft's gps level
   *  can't reach the status of record home location.
   *  2. The distance between new home location and init home location is larger
   * than
   *  MAX_FLY_RADIUS(20km)
   *  @param homeLocation  SetHomeLocationData include latitude and longitude
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode the OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   * is called
   *  @param when UserCallBack is called, used in UserCallBack
   */
  void setHomeLocationAsync(
      FlightAssistant::SetHomeLocationData homeLocation,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Set avoid obstacle switch enable or disable, blocking calls
   *
   *  @param avoidObstacle reference in FlightAssistant::AvoidObstacleData
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setAvoidObstacleSwitchSync(
      FlightAssistant::AvoidObstacleData avoidObstacle, int timeout);

  /*! @brief Set set avoid obstacle switch enable or disable, non-blocking calls
   *
   *  @param avoidObstacle reference in FlightAssistant::AvoidObstacleData
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode  OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to trans userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setAvoidObstacleSwitchAsync(
      FlightAssistant::AvoidObstacleData avoidObstacle,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Wrapper function for aircraft take off, blocking calls
   *
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startTakeoffSync(int timeout);

  /*! @brief Wrapper function for aircraft take off, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   *  is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startTakeoffAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                              UserData userData),
                         UserData userData);

  /*! @brief Wrapper function for aircraft force landing, blocking calls
   *
   *  @note this api will ignore the smart landing function, when use this
   * function api, it will landing directly (would not stop at 0.7m and wait
   * user's  command),it may make the aircraft crash.
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startForceLandingSync(int timeout);

  /*! @brief Wrapper function for aircraft force landing, non-blocking calls
   *
   *  @note This api will ignore the smart landing function, when use this
   * api landing, it will landing directly (would not stop at 0.7m and wait
   * user's  command),it may make the aircraft crash.
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   *  is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startForceLandingAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Wrapper function for  aircraft confirm landing and avoid ground,
   * blocking calls
   *
   *  @note When the clearance between the aircraft and the ground is less than
   * 0.7m, the aircraft will pause landing and wait for user's confirmation.This
   * api use for confirm landing. If the ground is not suitable for landing
   *  ,user must use RC to control it landing manually or force landing.
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startConfirmLandingSync(int timeout);

  /*! @brief Wrapper function for  aircraft confirm landing and avoid ground,
   * non-blocking calls
   *
   *
   *  @note When the clearance between the aircraft and the ground is less than
   * 0.7m, the aircraft will pause landing and wait for user's confirmation.This
   * api use for confirm landing. If the ground is not suitable for landing
   *  ,user must use RC to control it landing manually or force landing.
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   * is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startConfirmLandingAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Wrapper function for  go home action, blocking calls
   *
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startGoHomeSync(int timeout);

  /*! @brief Wrapper function for  go home action, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   * is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startGoHomeAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                             UserData userData),
                        UserData userData);

 private:
  FlightAssistant *flightAssistant;
  FlightActions *flightActions;
};
}  // namespace OSDK
}  // namespace DJI
#endif  // DJI_FLIGHT_CONTROLLER_HPP
