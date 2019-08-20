/*! @file FlightControlSample.h
 *  @version 3.3
 *  @date May 2017
 *
 *  @brief
 *  Flight control STM32 example.
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

#ifndef FLIGHTCONTROLSAMPLE_H
#define FLIGHTCONTROLSAMPLE_H

#include "BspUsart.h"
#include "dji_vehicle.hpp"
#include "timer.h"
#include <math.h>

using namespace DJI::OSDK;

#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252

bool monitoredTakeOff();
bool monitoredLanding();
int moveByPositionOffset(float xOffsetDesired, float yOffsetDesired,
                         float zOffsetDesired, float yawDesired,
                         float posThresholdInM   = 0.2,
                         float yawThresholdInDeg = 1.0);

//! Helper functions
void localOffsetFromGpsOffset(Vehicle* vehicle, Telemetry::Vector3f& deltaNed,
                              void* target, void* origin);
Telemetry::Vector3f toEulerAngle(void* quaternionData);
void startGlobalPositionBroadcast(Vehicle* vehicle);

#endif // FLIGHTCONTROLSAMPLE_H