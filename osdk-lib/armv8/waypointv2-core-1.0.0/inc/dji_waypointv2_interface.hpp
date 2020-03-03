/** @file dji_waypointv2_interface.hpp
 *  @version 3.8
 *  @date April 2019
 *
 *  @brief Implementation of GPS Waypoint Missions V2 for DJI OSDK
 *
 *  @Copyright (c) 2016-2019 DJI
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

#ifndef WAYPOINTV2_CORE_DJI_WAYPOINTV2_INTERFACE_HPP
#define WAYPOINTV2_CORE_DJI_WAYPOINTV2_INTERFACE_HPP

#include <functional>
#include <unordered_map>
#include <vector>

#include "dji_log.hpp"
#include "dji_command.hpp"
#include "dji_ack.hpp"
#include "dji_vehicle_callback.hpp"
#include "WaypointV2.pb.h"

namespace DJI
{
namespace OSDK
{

class Vehicle;

class WaypointV2Interface
{
public:
  typedef int CommonErrorCode;
  typedef std::function<void(CommonErrorCode)> CommonErrorCallback;

  const double RAD_2_DEGREE = 57.2957795;

  WaypointV2Interface(Vehicle *vehicle);
  ~WaypointV2Interface();

  /*! @brief
   *
   *  start the waypt mission
   *
   *  @param register a callback function for error code
   */
  void start(CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  stop the waypt mission
   *
   *  @param register a callback function for error code
   */
  void stop(CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  pause the waypt mission
   *
   *  @param register a callback function for error code
   */
  void pause(CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  resume the waypt mission
   *
   *  @param register a callback function for error code
   */
  void resume(CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  get current speed of the waypt mission
   *
   *  @param register a callback function for error code
   */
  void getCurrentSpeed(std::function<void(float cruise_speed, CommonErrorCode error_code)> callback);
  /*! @brief
   *
   *  set current speed of the waypt mission
   *
   *  @param register a callback function for error code
   */
  void setCurrentSpeed(float speed, CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  update push data from the drone to the internal waypt core library
   *
   *  @param cmd_id
   *  @param cmd seq_num
   *  @param raw data
   *  @param length of the raw data
   */
  void updatePushData(uint8_t cmd_id, uint16_t seq_num, const void *data, int data_length);
  /*! @brief
   *
   *  upload a waypt mission with old data strucutre
   *
   *  @param mission settings
   *  @param a vector of GPS waypoints
   *  @param register a callback function for error code
   */
  void uploadMission(const WayPointInitSettings &initSettings,
                     const std::vector<WayPointSettings> &waypts,
                     CommonErrorCallback &errorCallback);
  /*! @brief
   *
   *  upload a waypt mission with new data strucutre
   *
   *  @param mission settings
   *  @param register a callback function for error code
   */
  void uploadMission(const dji::waypointv2::WaypointMission &waypointMission,
                     CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  download a waypt mission with new data strucutre
   *
   *  @param mission setting data struct to be written
   *  @param register a callback function for error code
   */
  bool DownloadMission(dji::waypointv2::WaypointMission &outMission,
                       CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  upload a action which work in parallel with waypt mission
   *
   *  @param action settings
   *  @param register a callback function for error code
   */
  void uploadAction(const std::vector<dji::waypointv2::WaypointActionConfig> &actions,
                    CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  get current mission state
   *
   */
  inline dji::waypointv2::AbstractionState getCurrentState() { return currentState; }
  /*! @brief
   *
   *  get previous mission state
   *
   */
  inline dji::waypointv2::AbstractionState getPrevState() { return prevState; }
  /*! @brief
   *
   *  get current action state
   *
   */
  inline dji::waypointv2::ActionState getCurrentActionState() { return currentActionState; }
  /*! @brief
   *
   *  get previous action state
   *
   */
  inline dji::waypointv2::ActionState getPrevActionState() { return prevActionState; }

protected:
  void setUpCMDMap();

  void setUpStateMsgMap();

  void setUpActionStateMsgMap();

  static void sendCmdCallback(Vehicle *vehiclePtr, RecvContainer recvFrame,
                              UserData userData);

protected:
  Vehicle* vehiclePtr;

  int kDeviceID;

  WayPointInitSettings info;

  std::unordered_map<int, const uint8_t*> cmdMap;
  std::unordered_map<int, const char*> stateMsgMap;
  std::unordered_map<int, const char*> actionStateMsgMap;
  dji::waypointv2::AbstractionState prevState;
  dji::waypointv2::AbstractionState currentState;
  dji::waypointv2::ActionState prevActionState;
  dji::waypointv2::ActionState currentActionState;
};
} // namespace OSDK
} // namespace DJI

#endif //WAYPOINTV2_CORE_DJI_WAYPOINTV2_INTERFACE_HPP
