/** @file dji_flight_states.hpp
 *  @version 3.9
 *  @date July 2019
 *
 *  @brief Implementation of flight states
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

#ifndef DJI_FLIGHT_STATES_HPP
#define DJI_FLIGHT_STATES_HPP
#include "dji_ack.hpp"
#include "dji_telemetry.hpp"
namespace DJI {
namespace OSDK {
class Vehicle;
class FlightStatus {
 public:
  FlightStatus();
  ~FlightStatus();

 public:
  /*! TODO　后续完善该类中的这些接口*/
  /*! @brief Get the motor start error
   *  @return error code of disarm failed
   */
  ACK::ErrorCode getMotorStartError();

  /*! @brief Get the battery information
   *  @return Battery info, details in  DJI::OSDK::Telemetry::Battery
   */
  Telemetry::Battery getBatteryInfo();

  /*! @brief Get the status of flight
   *  @return
   */
  uint8_t getFlightStatus();

  /*! @brief Get the motor start error
   *  @return Flight mode info, details in DJI::OSDK::VehicleStatus::DisplayMode
   */
  uint8_t
  getFlightMode(); /*TODO
                      这些接口需要broadcast或者subscribe功能配合,如何表示获取数据无效的情况*/

  /*! @brief Get quaternion of aircraft
   *  @return Quaternion info, details in  DJI::OSDK::Telemetry::Quaternion
   */
  Telemetry::Quaternion getQuaternion();

  /*! @brief Get the velocity in ground coordinate
   *  @return vector3f info, details in  DJI::OSDK::Telemetry::vector3f
   */
  Telemetry::Vector3f getVelocity();

  /*! @brief Get the fused gps position
   *  @return GPSFused info, details in  DJI::OSDK::Telemetry::GPSFused
   */
  Telemetry::GPSFused getGPSFused();

  /*TODO　增加角速度速度获取接口*/
};
}  // namespace OSDK
}  // namespace DJI
#endif  // DJI_FLIGHT_STATES_HPP
