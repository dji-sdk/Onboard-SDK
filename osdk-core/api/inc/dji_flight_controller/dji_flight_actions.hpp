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

#include "dji_ack.hpp"
#include "dji_control_link.hpp"
#include "dji_open_protocol.hpp"
#include "dji_type.hpp"
#include "dji_vehicle_callback.hpp"

namespace DJI {
namespace OSDK {
class FlightActions {
 public:
  FlightActions(ControlLink *controlLink);
  ~FlightActions();

 public:
  /*! @brief Basic flight control commands
   */
  enum FlightCommand {
    TAKE_OFF = 1,                    /*!< vehicle takeoff*/
    FORCE_LANDING_AVOID_GROUND = 30, /*!< force landing and avoid ground*/
    FORCE_LANDING = 31,              /*!< force landing */

  };

  typedef struct CommonAck {
    uint8_t ret_code; /*!< original return code from vehicle */
  } CommonAck;        // pack(1)

  /*TODO: move this part code to an new class(down)*/
  typedef struct UCBRetCodeHandler {
    void (*UserCallBack)(ErrorCode::ErrCodeType errCode, UserData userData);
    UserData userData;
  } UCBRetCodeHandler;

  static const int maxSize = 32;
  UCBRetCodeHandler ucbHandler[maxSize];
  /*TODO: move this part code to an new class(up)*/

  /*! @brief Wrapper function for aircraft take off, blocking calls
   *
   *  @param timeout blocking timeout in seconds
   *  @return ACK::ErrorCode struct with the acknowledgement from the FC
   */
  ErrorCode::ErrCodeType takeoffSync(int timeout);

  /*! @brief Wrapper function for aircraft take off, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void takeoffAsync(void (*UserCallBack)(ErrorCode::ErrCodeType retCode,
                                         UserData userData),
                    UserData userData);

  /*! @brief Wrapper function for aircraft force landing, blocking calls
   *
   *  @param timeout blocking timeout in seconds
   *  @return ACK::ErrorCode struct with the acknowledgement from the FC
   */
  ErrorCode::ErrCodeType forceLandingSync(int timeout);

  /*! @brief Wrapper function for aircraft force landing, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void forceLandingAsync(void (*UserCallBack)(ErrorCode::ErrCodeType retCode,
                                              UserData userData),
                         UserData userData);

  /*! @brief Wrapper function for aircraft force landing and avoid ground,
   * blocking calls
   *
   *  @param timeout blocking timeout in seconds
   *  @return ACK::ErrorCode struct with the acknowledgement from the FC
   */
  ErrorCode::ErrCodeType forceLandingAvoidGroundSync(int wait_timeout);

  /*! @brief Wrapper function for aircraft force landing and avoid ground,
   * non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void forceLandingAvoidGroundAsync(
      void (*UserCallBack)(ErrorCode::ErrCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Control the position and yaw angle of the vehicle.
   *  @param xOffsetDesired position set-point in x axis of ground frame (m)
   *  @param yOffsetDesired position set-point in y axis of ground frame (m)
   *  @param zOffsetDesired position set-point in z axis of ground frame (m),
   *  input limit see DJI::OSDK::Control::VERTICAL_POSITION
   *  @param yawDesired yaw set-point (deg)
   *  @param posThreshold position threshold (m)
   *  @param yawThreshold yaw threshold (deg)
   */
  void moveByPositionReference(ControlLink *controlLink, float xOffsetDesired,
                               float yOffsetDesired, float zOffsetDesired,
                               float yawDesired, float posThreshold,
                               float yawThreshold);

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
  void moveByVelocityReference(ControlLink *controlLink, float xVelocity,
                               float yVelocity, float zVelocity, float yawRate);

  /*! @brief Control the attitude and yaw rate of the vehicle
   *
   *  @param roll  attitude set-point in x axis of body frame (FRD) (deg),
   * input limit see DJI::OSDK::Control::HORIZONTAL_ANGLE
   *  @param pitch  attitude set-point in y axis of body frame (FRD) (deg),
   * input limit see DJI::OSDK::Control::HORIZONTAL_ANGLE
   *  @param zVelocity  z velocity set-point in z axis of ground frame (NED)
   * (m/s)
   *  @param yawRate  set-point in z axis of ground frame (NED) (deg/s)
   */
  void moveByAttitudeReference(ControlLink *controlLink, float roll,
                               float pitch, float zVelocity, float yawRate);

 private:
  ControlLink *controlLink;

  static void commonAckDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                               UCBRetCodeHandler *ucb);
  FlightActions::UCBRetCodeHandler *allocUCBHandler(void *callback,
                                                    UserData userData);

  /*! @brief Control the vehicle using user-specified mode, see FlightCommand
   * for cmd choices
   *
   *  @param cmd action command from FlightCommand
   *  @param timeout blocking timeout in seconds
   *  @return ACK::ErrorCode struct with the acknowledgement from the FC
   */
  template <typename ReqT>
  ErrorCode::ErrCodeType actionSync(ReqT req, int timeout);

  /*! @brief Execute basic command for the vehicle, see FlightCommand for cmd
   * choices
   *
   *  @param cmd action command in FlightCommand
   *  @param callback user callback function
   *  @param userData user data (void ptr)
   */
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
