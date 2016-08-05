/*! @file LinuxFlight.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Flight-related commands for new Onboard SDK Linux sample. 
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef LINUXFLIGHT_H
#define LINUXFLIGHT_H

#include <DJI_Type.h>
#include <DJI_API.h>
#include <DJI_Flight.h>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include "unistd.h"
#include "LinuxSetup.h"


#define C_EARTH (double) 6378137.0
#define DEG2RAD 0.01745329252

using namespace DJI::onboardSDK;

//!@note: All the default timeout parameters are for acknowledgement packets from the aircraft. 

//! Pre-flight Functions

//! Arm the aircraft (Blocking API call) and return status as well as ack
ackReturnData arm(Flight* flight, int timeout = 1);

/*! Takeoff (Blocking API call) and return status as well as ack. 
    The ack is only for starting the takeoff. If you want an ack
    after the takeoff is complete, use monitoredTakeoff.
!*/
ackReturnData takeoff(Flight* flight, int timeout = 1);

/*! Monitored Takeoff (Blocking API call). Return status as well as ack. 
    This version of takeoff makes sure your aircraft actually took off 
    and only returns when takeoff is complete. 
    Use unless you want to do other stuff during takeoff - this will block 
    the main thread.
!*/ 
ackReturnData monitoredTakeoff(CoreAPI* api, Flight* flight, int timeout = 1);

//! Check if takeoff completed and stabilized - used in monitoredTakeoff.
bool takeoffConverged(CoreAPI* api, int settlingTimeout);


//! Commonly used Flight Mode APIs 

/*! Attitude Control allows you to set a desired attitude and returns 
    when it reaches that attitude. It does not hold that attitude and
    will return to nominal attitude after 50 ms of this function returning.
    Typical use would be as a building block in an outer loop that calls 
    this function in succession for different desired attitudes 
!*/
int attitudeControl(CoreAPI* api, Flight* flight, float32_t rollDesired, float32_t pitchDesired, float32_t yawDesired, int timeoutInMs = 2000, float thresholdInDeg = 1); //Holds current height

/*! Attitude/Altitude Control allows you to set a desired attitude and Z height 
 *  and returns when it reaches that attitude+altitude. It does not hold
 *  that attitude (but does hold altitude) and will return to nominal
 *  attitude after 50 ms of this function returning. Typical use would be
 *  as a building block in an outer loop that calls this function in
 *  succession for different desired attitudes. 
!*/
int attitudeAltitudeControl(CoreAPI* api, Flight* flight, float32_t rollDesired, float32_t pitchDesired, float32_t yawDesired, float32_t zDesired, int timeoutInMs = 2000, float attiThresholdInDeg = 1, float zThresholdInCm = 1);

/*! Position Control. Allows you to set an offset from your current
    location. The aircraft will move to that position and stay there. 
    Typical use would be as a building block in an outer loop that does not
    require many fast changes, perhaps a few-waypoint trajectory. For smoother
    transition and response you should convert your trajectory to attitude 
    setpoints and use attitude control or convert to velocity setpoints
    and use velocity control. 
!*/
int moveByPositionOffset(CoreAPI* api, Flight* flight, float32_t xOffsetDesired, float32_t yOffsetDesired, float32_t zOffsetDesired, float32_t yawDesired ,  int timeoutInMs = 10000, float yawThresholdInDeg = 1, float posThresholdInCm = 5);

/*! Velocity Control. Allows you to set a body-frame velocity setpoint 
    of the form (x_dot, y_dot, z_dot, yaw_dot). 
    The aircraft will reach that velocity and this function will return.

    Typical use would be as a building block in an outer loop that calculates
    velocity setpoints and calls this function repeatedly. 
!*/
int moveWithVelocity(CoreAPI* api, Flight* flight, float32_t xVelocityDesired, float32_t yVelocityDesired, float32_t zVelocityDesired, float32_t yawRateDesired ,  int timeoutInMs = 2000, float yawRateThresholdInDegS = 0.5, float velThresholdInMs = 0.5);

/*  Note - If you want a control mode other than the three above,
    implement it through the movementControl call defined in the 
    DJI Onboard SDK Library Flight API - similar to how the above
    functions are implemented. 
*/

//! Post-flight functions

// Execute Return to Home command
ackReturnData goHome(Flight* flight, int timeout = 1);

/*! Land. If it does not work the first time, this function waits 
    for 4 seconds and tries to land again.
!*/
ackReturnData landing(CoreAPI* api, Flight* flight, int timeout = 1);

//! Disarm the aircraft (Blocking API call) and return status as well as ack
ackReturnData disArm(Flight* flight, int timeout = 1);

//! Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS coordinates.
    Accurate when distances are small.
!*/
void localOffsetFromGpsOffset(DJI::Vector3dData& deltaNed,
    PositionData* target,
    PositionData* origin);

// Position Control Sample -  Draw a 10m Square with two diagonals
int drawSqrPosCtrlSample(CoreAPI* api, Flight* flight); 

#endif //LINUXFLIGHT_H