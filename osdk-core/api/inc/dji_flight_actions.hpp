/** @file dji_flight_actions.hpp
 *  @version 3.9
 *  @date July 2019
 *
 *  @brief Implementation of flight actions
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

#ifndef DJI_FLIGHT_ACTIONS_HPP
#define DJI_FLIGHT_ACTIONS_HPP
#include "dji_open_protocol.hpp"

namespace DJI {
namespace OSDK {
class ControlLink;
class FlightActions {
 public:
  FlightActions(Vehicle *vehicle);
  ~FlightActions();

 public:
  /*! @brief Basic flight control commands
   */
  enum FlightCommand : uint8_t {
    TAKE_OFF = 1,                    /*!< vehicle takeoff*/
    GO_HOME = 6,                     /*!< vehicle return home position*/
    FORCE_LANDING_AVOID_GROUND = 30, /*!< force landing and avoid ground*/
    FORCE_LANDING = 31,              /*!< force landing */
  };

  typedef struct CommonAck {
    uint8_t ret_code; /*!< original return code from vehicle */
  } CommonAck;        // pack(1)

  /*TODO: move this part code to an new class*/
  typedef struct UCBRetCodeHandler {
    void (*UserCallBack)(ErrorCode::ErrCodeType errCode, UserData userData);
    UserData userData;
  } UCBRetCodeHandler;

  static const int maxSize = 32;
  UCBRetCodeHandler ucbHandler[maxSize];

  /*! @brief Wrapper function for aircraft take off, blocking calls
   *
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrCodeType error code
   */
  ErrorCode::ErrCodeType startTakeoffSync(int timeout);

  /*! @brief Wrapper function for aircraft take off, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the OSDK ErrorCode::ErrCodeType error code
   *  @arg @b userData the interface to trans userData in when the callback
   *  is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startTakeoffAsync(void (*UserCallBack)(ErrorCode::ErrCodeType retCode,
                                              UserData userData),
                         UserData userData);

  /*! @brief Wrapper function for aircraft force landing, blocking calls
   *
   *  @note this api will ignore the smart landing function, when use this
   * function api, it will landing directly (would not stop at 0.7m and wait
   * user's  command),it may make the aircraft crash.
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrCodeType error code
   */
  ErrorCode::ErrCodeType startForceLandingSync(int timeout);

  /*! @brief Wrapper function for aircraft force landing, non-blocking calls
   *
   *  @note this api will ignore the smart landing function, when use this
   * api landing, it will landing directly (would not stop at 0.7m and wait
   * user's  command),it may make the aircraft crash.
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the OSDK ErrorCode::ErrCodeType error code
   *  @arg @b userData the interface to trans userData in when the callback
   *  is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startForceLandingAsync(
      void (*UserCallBack)(ErrorCode::ErrCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Wrapper function for  aircraft force landing and avoid ground,
   * blocking calls
   *
   *  @note this api must be used after the aircraft is close to ground and
   *  stop in the air (about 0.7m), if the ground is not suitable for landing
   *  ,user must use RC to control it landing manually or force landing.
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrCodeType error code
   */
  ErrorCode::ErrCodeType startForceLandingAvoidGroundSync(int timeout);

  /*! @brief Wrapper function for  aircraft force landing and avoid ground,
   * non-blocking calls
   *
   *  @note this api must be used after the aircraft is close to ground and
   *  stop in the air (about 0.7m), if the ground is not suitable for landing
   *  ,user must use RC to control it landing manually or force landing.
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the OSDK ErrorCode::ErrCodeType error code
   *  @arg @b userData the interface to trans userData in when the callback is
   *  called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startForceLandingAvoidGroundAsync(
      void (*UserCallBack)(ErrorCode::ErrCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Wrapper function for  go home action, blocking calls
   *
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrCodeType error code
   */
  ErrorCode::ErrCodeType startGoHomeSync(int timeout);

  /*! @brief Wrapper function for  go home action, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the OSDK ErrorCode::ErrCodeType error code
   *  @arg @b userData the interface to trans userData in when the callback is
   *  called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startGoHomeAsync(void (*UserCallBack)(ErrorCode::ErrCodeType retCode,
                                             UserData userData),
                        UserData userData);

 private:
  ControlLink *controlLink;

  static void commonAckDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                               UCBRetCodeHandler *ucb);
  FlightActions::UCBRetCodeHandler *allocUCBHandler(void *callback,
                                                    UserData userData);

  template <typename ReqT>
  ErrorCode::ErrCodeType actionSync(ReqT req, int timeout);

  template <typename ReqT>
  void actionAsync(ReqT req, void (*ackDecoderCB)(Vehicle *vehicle,
                                                  RecvContainer recvFrame,
                                                  UCBRetCodeHandler *ucb),
                   void (*userCB)(ErrorCode::ErrCodeType, UserData userData),
                   UserData userData, int timeout = 2000, int retry_time = 1);
};
}  // namespace OSDK
}  // namespace DJI
#endif  // DJI_FLIGHT_ACTIONS_HPP
