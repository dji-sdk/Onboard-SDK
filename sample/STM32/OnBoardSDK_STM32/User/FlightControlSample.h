/*! @file FlightControlSample.h
 *  @version 3.3
 *  @date May 2017
 *
 *  @brief
 *  Flight control STM32 example.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#ifndef FLIGHTCONTROLSAMPLE_H
#define FLIGHTCONTROLSAMPLE_H

#include "BspUsart.h"
#include "timer.h"
#include "dji_vehicle.hpp"
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
void localOffsetFromGpsOffset(Telemetry::Vector3f&     deltaNed,
                              Telemetry::GPSFused& target,
                              Telemetry::GPSFused& origin);
Telemetry::Vector3f toEulerAngle(
  Telemetry::TypeMap<Telemetry::TOPIC_QUATERNION>::type &quaternionData);

#endif // FLIGHTCONTROLSAMPLE_H