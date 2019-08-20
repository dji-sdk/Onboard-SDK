/*! @file MissionSample.h
 *  @version 3.3
 *  @date May 2017
 *
 *  @brief
 *  Missions STM32 example.
 *
 *  @Copyright (c) 2016-2017 DJI
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

#ifndef MISSIONSAMPLE_H
#define MISSIONSAMPLE_H

#include "BspUsart.h"
#include "FlightControlSample.h"
#include "dji_vehicle.hpp"
#include "timer.h"
#include <math.h>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool setUpSubscription(DJI::OSDK::Vehicle* vehicle);

bool runWaypointMission(uint8_t numWaypoints = 6);

void setWaypointDefaults(DJI::OSDK::WayPointSettings* wp);
void setWaypointInitDefaults(DJI::OSDK::WayPointInitSettings* fdata);

std::vector<DJI::OSDK::WayPointSettings> createWaypoints(
  int numWaypoints, DJI::OSDK::float64_t distanceIncrement,
  DJI::OSDK::float32_t start_alt);

std::vector<DJI::OSDK::WayPointSettings> generateWaypointsPolygon(
  DJI::OSDK::WayPointSettings* start_data, DJI::OSDK::float64_t increment,
  int num_wp);

void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wp_list);

bool runHotpointMission(int initialRadius = 10);

const int DEFAULT_PACKAGE_INDEX = 1;

#endif // MISSIONSAMPLE_H
