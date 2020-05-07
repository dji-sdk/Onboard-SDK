/** @file dji_flight_actions_module.hpp
 *  @version 3.9
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

#ifndef DJI_FLIGHT_ACTIONS_MODULE_HPP
#define DJI_FLIGHT_ACTIONS_MODULE_HPP

#include "dji_vehicle_callback.hpp"

namespace DJI {
namespace OSDK {

class FlightLink;
class FlightActions {
 public:
  FlightActions(Vehicle *vehicle);
  ~FlightActions();

 public:
  /*! @brief Basic flight control commands
   */
  enum FlightCommand : uint8_t {
    TAKE_OFF                   = 1, /*!< vehicle takeoff */
    LANDING                    = 2, /*!< vehicle landing*/
    COURSE_LOCK                = 5,
    GO_HOME                    = 6, /*!< vehicle return home position */
    START_MOTOR                = 7,
    STOP_MOTOR                 = 8,
    CALIBRATE_COMPASS          = 9,
    EXIT_GOHOME                = 12,
    EXIT_TAKEOFF               = 13,
    EXIT_LANDING               = 14,
    EXIT_CALIBRATE_COMPASS     = 21,
    LANDING_GEAR_DOWN          = 28,
    LANDING_GEAR_UP            = 29,
    FORCE_LANDING_AVOID_GROUND = 30, /*!< confirm landing */
    FORCE_LANDING              = 31, /*!< force landing */
  };
  enum KillSwitch
  {
    ENABLE  = 0x01, /*!< Enable the killswitch */
    DISABLE = 0x02  /*!< Disable the killswitch */
  };
#pragma pack(1)
  typedef struct CommonAck {
    uint8_t retCode; /*!< original return code from vehicle */
  } CommonAck;       // pack(1)

  typedef struct KillSwitchData
  {
    uint8_t high_version;    /*!< version # for FW version match */
    uint8_t low_version;     /*!< version # for protocol compatibility */
    uint8_t debug_description[10]; /*!< insert reasons for emergency stop for debugging purpose */
    uint8_t cmd : 2;         /*!< 0: no action, 1: motor stop with protection, 2: disarm protection */
    uint8_t reserved : 6;
  } KillSwitchData; // pack(1)
#pragma pack()

  /*! @brief type of callback only deal the retCode for user
   */
  typedef struct UCBRetCodeHandler {
    void (*UserCallBack)(ErrorCode::ErrorCodeType errCode, UserData userData);
    UserData userData;
  } UCBRetCodeHandler;

  static const int maxSize = 32;
  UCBRetCodeHandler ucbHandler[maxSize];

  UCBRetCodeHandler *allocUCBHandler(void *callback, UserData userData);

  static void commonAckDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                               UCBRetCodeHandler *ucb);

  ErrorCode::ErrorCodeType actionSync(uint8_t req, int timeout);

  void actionAsync(FlightCommand req,
                   void (*ackDecoderCB)(Vehicle *vehicle,
                                        RecvContainer recvFrame,
                                        UCBRetCodeHandler *ucb),
                   void (*userCB)(ErrorCode::ErrorCodeType, UserData userData),
                   UserData userData, int timeout = 2000, int retryTime = 1);

  ErrorCode::ErrorCodeType  EmergencyBrakeActionSync(uint8_t req, int timeout);

  ErrorCode::ErrorCodeType killSwitch(KillSwitch cmd, int wait_timeout, char debugMsg[10]);
 private:
  FlightLink *flightLink;

};
}
}

#endif  // DJI_FLIGHT_ACTIONS_MODULE_HPP
