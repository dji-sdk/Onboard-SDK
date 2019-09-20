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

  typedef struct Telemetry::HomeLocationData HomeLocation;
  typedef FlightAssistant::RtkEnableData
      RtkEnabled; /*!< 0: disable, 1: enable*/
  typedef FlightAssistant::GoHomeAltitude
      GoHomeHeight; /*!< unit:meter, range 20~500*/

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
   *  @arg @b userData the interface to transfer userData in when the callback is
   *  called
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
   *  @arg @b userData the interface to transfer userData in when the callback is
   * called
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
   *  @arg @b userData the interface to transfer userData in when the callback is
   *  called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getGoHomeAltitudeAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           GoHomeHeight altitude, UserData userData),
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

 private:
  FlightAssistant *flightAssistant;
  FlightActions *flightActions;
};
}  // namespace OSDK
}  // namespace DJI
#endif  // DJI_FLIGHT_CONTROLLER_HPP
