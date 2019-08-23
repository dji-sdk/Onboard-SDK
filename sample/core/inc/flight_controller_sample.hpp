/*! @file flight_controller_sample.hpp
 *  @version 3.9
 *  @date August 05 2019
 *
 *  @brief
 *  Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
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

#ifndef DJIOSDK_FLIGHT_CONTROLLER_SAMPLE_HPP
#define DJIOSDK_FLIGHT_CONTROLLER_SAMPLE_HPP

// System Includes
#include <cmath>
// DJI OSDK includes
#include <dji_vehicle.hpp>
#include "dji_status.hpp"

#define PI 3.1415926535898

/*write by sandy.yu*/
bool setUpSubscription(DJI::OSDK::Vehicle* vehicle, int pkgIndex, int freq,
                       Telemetry::TopicName topicList[], uint8_t topicSize,
                       int timeout);
bool teardownSubscription(Vehicle* vehicle, const int pkgIndex, int timeout);
bool checkActionStarted(Vehicle* vehicle, uint8_t mode);

bool setGoHomeAltitude(Vehicle* vehicle,
                       FlightAssistant::goHomeAltitude altitude, int timeout=1);
bool setNewHomePoint(Vehicle* vehicle, int timeout=1);
bool openAvoidObstacle(Vehicle* vehicle, int timeout=1);
bool closeAvoidObstacle(Vehicle* vehicle, int timeout=1);
bool goHomeAndForceLanding(Vehicle* vehicle, int timeout=1);

#endif  // DJIOSDK_FLIGHT_CONTROLLER_SAMPLE_HPP
