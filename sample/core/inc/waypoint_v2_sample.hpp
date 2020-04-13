/*! @file waypoint_v2_sample.hpp
 *  @version 4.0
 *  @date Mar 07 2019
 *
 *  @brief
 *  main for Waypoint Missions V2 API usage in a Linux environment.
 *  Shows example usage of the Waypoint Missions through
 *  the Mission Manager API.
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

#ifndef DJIOSDK_WAYPOINT_V2_SAMPLE_HPP
#define DJIOSDK_WAYPOINT_V2_SAMPLE_HPP

// System Includes
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <vector>
// DJI OSDK includes
#include <dji_vehicle.hpp>
#include "dji_waypoint_v2.hpp"
// Helpers
#include <dji_linux_helpers.hpp>
const int DEFAULT_PACKAGE_INDEX = 0;
class WaypointV2MissionSample {

public:
  WaypointV2MissionSample(Vehicle *vehicle);

  ~WaypointV2MissionSample();

public:
  /*! @brief Sample to run a complete mission, include init mission,
   * upload mission and action, start, pause, resume mission and so on
   *
   *  @note If any one of the steps fails, it will return the failed error code
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType runWaypointV2Mission();

  /*! @brief Sample to init mission settings,
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType initMissionSetting(int timeout);

  /*! @brief Sample to upload mission
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType uploadWaypointMission(int timeout);

  /*! @brief Sample to upload actions
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType uploadWapointActions(int timeout);

  /*! @brief Sample to download mission
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType downloadWaypointMission(std::vector<WaypointV2> &mission,int timeout);

  /*! @brief Sample to start mission
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startWaypointMission(int timeout);

  /*! @brief Sample to stop mission
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType stopWaypointMission(int timeout);

  /*! @brief Sample to pause mission
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType pauseWaypointMission(int timeout);

  /*! @brief Sample to resume mission
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType resumeWaypointMission(int timeout);

  /*! @brief Sample to get global cruise speed
   *
   *  @param timeout blocking timeout in seconds
   */
  void getGlobalCruiseSpeed(int timeout);

  /*! @brief Sample to set global cruise speed
   *
   *  @param cruiseSpeed global cruise speed,unit: m/s
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  void setGlobalCruiseSpeed(const GlobalCruiseSpeed &cruiseSpeed, int timeout);

  /*! @brief Sample to set up subscription
   *
   *  @param timeout blocking timeout in seconds
   *  @return bool result, 0:fail, 1:success
   */
  bool setUpSubscription(int timeout);

  /*! @brief Sample to tear down subscription
   *
   *  @param timeout blocking timeout in seconds
   *  @return bool result, 0:fail, 1:success
   */
  bool teardownSubscription(const int pkgIndex,
                            int timeout);
  /*! @brief Sample to set single waypoint default value
   *
   *  @param waypointV2 struct of DJIWaypointV2
   */
//  void setWaypointV2Defaults(DJIWaypointV2& waypointV2);

  void setWaypointV2Defaults(WaypointV2& waypointV2);
  /*! @brief Sample generate polygon waypoints
   *
   *  @param radius radius of polygon,unit: meter
   *  @param polygonNum number of polygon sides
   *  @return vector of DJIWaypointV2
   */
//  std::vector<DJIWaypointV2> generatePolygonWaypoints(float32_t radius, uint16_t polygonNum);

  std::vector<WaypointV2> generatePolygonWaypoints(float32_t radius, uint16_t polygonNum);


  /*! @brief Sample generate polygon waypoints
   *
   *  @param actionNum number of actions
   *  @param vector of DJIWaypointV2Action
   */
  std::vector<DJIWaypointV2Action> generateWaypointActions(uint16_t actionNum);

private:
  Vehicle *vehiclePtr;
  std::vector<DJIWaypointV2Action> actions;
};

#endif  // DJIOSDK_WAYPOINT_V2_SAMPLE_HPP
