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

  ErrorCode::ErrorCodeType initMissionSetting(int timeout);

  bool setUpSubscription(int responseTimeout);

  bool teardownSubscription( const int pkgIndex,
                            int responseTimeout);

  ErrorCode::ErrorCodeType uploadWaypointMission(int responseTimeout);

  ErrorCode::ErrorCodeType uploadWapointActions(int responseTimeout);

  ErrorCode::ErrorCodeType dowloadWaypointMission(std::vector<DJIWaypointV2> &mission,int responseTimeout);

  ErrorCode::ErrorCodeType startWaypointMission(DJI::OSDK::Vehicle* vehicle);

  // void setWaypointDefaults(DJI::OSDK::WayPointSettings* wp);

  void setWaypointInitDefaults(DJI::OSDK::WayPointInitSettings* fdata);

  std::vector<DJI::OSDK::WayPointSettings> createWaypoints(
      DJI::OSDK::Vehicle* vehicle, int numWaypoints,
      DJI::OSDK::float64_t distanceIncrement, DJI::OSDK::float32_t start_alt);



  void setWaypointV2Defaults(DJIWaypointV2* waypointV2);

  std::vector<DJIWaypointV2> generatePolygonWaypoints(float32_t radius, uint16_t polygonNum);

  std::vector<DJIWaypointV2Action> generateWaypointActions(uint16_t actionNum);


private:
  Vehicle *vehiclePtr;
  std::vector<DJIWaypointV2> mission;
  std::vector<DJIWaypointV2Action> actions;
};

#endif  // DJIOSDK_WAYPOINT_V2_SAMPLE_HPP
