/*! @file FlightControlSample.cpp
 *  @version 3.3
 *  @date May 2017
 *
 *  @brief
 *  Flight control STM32 example.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#include "FlightControlSample.h"

extern Vehicle  vehicle;
extern Vehicle* v;

bool
monitoredTakeOff()
{
  //@todo: remove this once the getErrorCode function signature changes
  char           func[50];
  ACK::ErrorCode ack;

  // Telemetry: Subscribe to flight status and mode at freq 10 Hz
  int                  pkgIndex        = 0;
  int                  freq            = 10;
  Telemetry::TopicName topicList10Hz[] = {
    Telemetry::TOPIC_STATUS_FLIGHT, Telemetry::TOPIC_STATUS_DISPLAYMODE
  };
  int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  bool enableTimestamp = false;

  bool pkgStatus = v->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  //! Start listening to subscribed packages
  v->subscribe->startPackage(pkgIndex);
  delay_nms(500);
  /*ack = waitForACK();
  if(ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, func);

    // Cleanup
    v->subscribe->removePackage(pkgIndex);
    ack = waitForACK();
    if(ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }

    return false;
  }*/

  // Start takeoff
  v->control->takeoff();
  delay_nms(500);
  /*ack = waitForACK();
  if(ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, func);

    // Cleanup
    v->subscribe->removePackage(pkgIndex);
    ack = waitForACK();
    if(ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }

    return false;
  }*/

  // First check: Motors started
  uint32_t CONTROL_TIMEOUT = 15000; // milliseconds
  uint32_t RETRY_TICK      = 500;   // milliseconds
  uint32_t nextRetryTick   = 0;     // millisesonds
  uint32_t timeoutTick;
  bool     isTakeOffState = false;
  bool     isInAirState   = false;
  bool     isHoverState   = false;

  timeoutTick = v->protocolLayer->getDriver()->getTimeStamp() + CONTROL_TIMEOUT;
  do
  {
    //! Two seconds delay
    delay_nms(2000);

    if (v->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>() ==
          OpenProtocol::ErrorCode::CommonACK::FlightStatus::ON_GROUND &&
        v->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() ==
          VehicleStatus::MODE_ENGINE_START)
    {
      isTakeOffState = true;
      break;
    }

    nextRetryTick = v->protocolLayer->getDriver()->getTimeStamp() + RETRY_TICK;
  } while (nextRetryTick < timeoutTick);

  if (!isTakeOffState)
  {
    printf("Takeoff failed. Motors failed to start!\n");

    //! Clean up
    v->subscribe->removePackage(pkgIndex);
    delay_nms(50);
    /*ack = waitForACK();
    if (ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }*/

    return false;
  }
  else
  {
    printf("\nSuccessful takeof!\n");
  }

  timeoutTick = v->protocolLayer->getDriver()->getTimeStamp() + CONTROL_TIMEOUT;
  do
  {
    //! Two seconds delay
    delay_nms(2000);

    if (v->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>() ==
          OpenProtocol::ErrorCode::CommonACK::FlightStatus::IN_AIR &&
        (v->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() ==
           VehicleStatus::MODE_ASSISTED_TAKEOFF ||
         v->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() ==
           VehicleStatus::MODE_AUTO_TAKEOFF))
    {
      isInAirState = true;
      break;
    }

    nextRetryTick = v->protocolLayer->getDriver()->getTimeStamp() + RETRY_TICK;
  } while (nextRetryTick < timeoutTick);

  if (!isInAirState)
  {
    printf("Takeoff failed. Aircraft is still on the ground, but the "
           "motors are spinning.\n");

    //! Clean up
    v->subscribe->removePackage(pkgIndex);
    delay_nms(500);
    /*ack = waitForACK();
    if (ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }
    */
    return false;
  }
  else
  {
    printf("Vehicle ascending...\n");
  }

  timeoutTick = v->protocolLayer->getDriver()->getTimeStamp() + CONTROL_TIMEOUT;
  do
  {
    //! Two seconds delay
    delay_nms(2000);

    if (v->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::MODE_ASSISTED_TAKEOFF ||
        v->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::MODE_AUTO_TAKEOFF)
    {
      isHoverState = true;
      break;
    }

    nextRetryTick = v->protocolLayer->getDriver()->getTimeStamp() + RETRY_TICK;
  } while (nextRetryTick < timeoutTick);

  if (v->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
        VehicleStatus::MODE_P_GPS ||
      v->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
        VehicleStatus::MODE_ATTITUDE)
  {
    printf("Vehicle is hovering\n");
  }
  else
  {
    printf("Takeoff finished, but the aircraft is in an unexpected mode. "
           "Please connect DJI GO.\n");
    // Cleanup
    v->subscribe->removePackage(pkgIndex);
    delay_nms(500);
    /*ack = waitForACK();
    if (ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }*/

    return false;
  }

  // Cleanup
  v->subscribe->removePackage(pkgIndex);
  delay_nms(3000);
  /*ack = waitForACK();
  if(ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, func);
  }*/

  return true;
}

/*! Monitored Takeoff (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/
bool
monitoredLanding()
{
  //@todo: remove this once the getErrorCode function signature changes
  char           func[50];
  ACK::ErrorCode ack;
  uint32_t       SUBSCRIBE_TIMOUT = 30000; // milliseconds
  uint32_t       RETRY_TICK       = 500;   // milliseconds
  uint32_t       nextRetryTick    = 0;     // millisesonds
  uint32_t       timeoutTick;
  bool           isLandingState         = false;
  bool           isFinishedLandingState = false;
  bool           isHoverState           = false;

  // Telemetry: Subscribe to flight status and mode at freq 10 Hz
  int                  pkgIndex        = 0;
  int                  freq            = 10;
  Telemetry::TopicName topicList10Hz[] = {
    Telemetry::TOPIC_STATUS_FLIGHT, Telemetry::TOPIC_STATUS_DISPLAYMODE
  };
  int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  bool enableTimestamp = false;

  bool pkgStatus = v->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  v->subscribe->startPackage(pkgIndex);
  delay_nms(500);
  /*ack = waitForACK();
  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, func);
    // Cleanup
    v->subscribe->removePackage(pkgIndex);
    delay_nms(500);
    return false;
  }*/

  // Start landing
  v->control->land();
  delay_nms(500);
  /*ack = waitForACK();
  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, func);
    return false;
  }*/

  // First check: Landing started

  timeoutTick =
    v->protocolLayer->getDriver()->getTimeStamp() + SUBSCRIBE_TIMOUT;
  do
  {
    //! Two seconds delay
    delay_nms(2000);

    if (v->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() ==
        VehicleStatus::MODE_AUTO_LANDING)
    {
      isLandingState = true;
      break;
    }

    nextRetryTick = v->protocolLayer->getDriver()->getTimeStamp() + RETRY_TICK;
  } while (nextRetryTick < timeoutTick);

  if (!isLandingState)
  {
    printf("Landing failed. Aircraft is still in the air.\n");
    // Cleanup before return
    v->subscribe->removePackage(pkgIndex);
    delay_nms(500);
    return false;
  }
  else
  {
    printf("Vehicle landing...\n");
  }

  // Second check: Finished landing

  timeoutTick =
    v->protocolLayer->getDriver()->getTimeStamp() + SUBSCRIBE_TIMOUT;
  do
  {
    //! Two seconds delay
    delay_nms(2000);

    if (v->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::MODE_AUTO_LANDING &&
        v->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>() !=
          OpenProtocol::ErrorCode::CommonACK::FlightStatus::IN_AIR)
    {
      if (v->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::MODE_P_GPS ||
          v->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::MODE_ATTITUDE)
      {
        isFinishedLandingState = true;
        break;
      }
    }

    nextRetryTick = v->protocolLayer->getDriver()->getTimeStamp() + RETRY_TICK;
  } while (nextRetryTick < timeoutTick);

  if (isFinishedLandingState)
  {
    printf("Successful landing!\n");
  }
  else
  {
    printf("Landing finished, but the aircraft is in an unexpected mode. "
           "Please connect DJI GO.\n");
    v->subscribe->removePackage(pkgIndex);
    delay_nms(500);
    return false;
  }

  // Cleanup before return
  v->subscribe->removePackage(pkgIndex);
  delay_nms(3000);
  return true;
}

/*! Position Control. Allows you to set an offset from your current
    location. The aircraft will move to that position and stay there.
    Typical use would be as a building block in an outer loop that does not
    require many fast changes, perhaps a few-waypoint trajectory. For smoother
    transition and response you should convert your trajectory to attitude
    setpoints and use attitude control or convert to velocity setpoints
    and use velocity control.
!*/
int
moveByPositionOffset(float xOffsetDesired, float yOffsetDesired,
                     float zOffsetDesired, float yawDesired,
                     float posThresholdInM, float yawThresholdInDeg)
{
  // Set timeout: this timeout is the time you allow the drone to take to finish
  // the
  // mission
  int timeoutInMilSec              = 100000;
  int controlFreqInHz              = 50; // Hz
  int cycleTimeInMs                = 1000 / controlFreqInHz;
  int outOfControlBoundsTimeLimit  = 10 * cycleTimeInMs; // 10 cycles
  int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles

  //@todo: remove this once the getErrorCode function signature changes
  char           func[50];
  ACK::ErrorCode ack;

  // Telemetry: Subscribe to quaternion, fused lat/lon and altitude at freq 50
  // Hz
  int                  pkgIndex        = 0;
  int                  freq            = 50;
  Telemetry::TopicName topicList50Hz[] = { Telemetry::TOPIC_QUATERNION,
                                           Telemetry::TOPIC_GPS_FUSED };
  int  numTopic        = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
  bool enableTimestamp = false;

  bool pkgStatus = v->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  v->subscribe->startPackage(pkgIndex);
  delay_nms(500);
  /*ack = waitForACK();
  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, func);

    // Cleanup
    v->subscribe->removePackage(pkgIndex);
    delay_nms(500);
    return false;
  }*/

  // Wait for data to come in
  delay_nms(8000);

  // Get data
  Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type currentGPS =
    v->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
  Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type originGPS = currentGPS;

  // Convert position offset from first position to local coordinates
  Telemetry::Vector3f localOffset;
  localOffsetFromGpsOffset(localOffset, currentGPS, originGPS);

  // Get initial offset. We will update this in a loop later.
  double xOffsetRemaining = xOffsetDesired - localOffset.x;
  double yOffsetRemaining = yOffsetDesired - localOffset.y;
  double zOffsetRemaining = zOffsetDesired - (-localOffset.z);

  // Conversions
  double yawDesiredRad     = DEG2RAD * yawDesired;
  double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

  //! Get Euler angle
  Telemetry::TypeMap<Telemetry::TOPIC_QUATERNION>::type q =
    v->subscribe->getValue<Telemetry::TOPIC_QUATERNION>();
  double yawInRad = toEulerAngle(q).z / DEG2RAD;

  int   elapsedTimeInMs     = 0;
  int   withinBoundsCounter = 0;
  int   outOfBounds         = 0;
  int   brakeCounter        = 0;
  int   speedFactor         = 2;
  float xCmd, yCmd, zCmd;
  // There is a deadband in position control
  // the z cmd is absolute height
  // while x and y are in relative
  float zDeadband = 0.12;

  /*! Calculate the inputs to send the position controller. We implement basic
   *  receding setpoint position control and the setpoint is always 1 m away
   *  from the current position - until we get within a threshold of the goal.
   *  From that point on, we send the remaining distance as the setpoint.
   */
  if (xOffsetDesired > 0)
    xCmd = (xOffsetDesired < speedFactor) ? xOffsetDesired : speedFactor;
  else if (xOffsetDesired < 0)
    xCmd =
      (xOffsetDesired > -1 * speedFactor) ? xOffsetDesired : -1 * speedFactor;
  else
    xCmd = 0;

  if (yOffsetDesired > 0)
    yCmd = (yOffsetDesired < speedFactor) ? yOffsetDesired : speedFactor;
  else if (yOffsetDesired < 0)
    yCmd =
      (yOffsetDesired > -1 * speedFactor) ? yOffsetDesired : -1 * speedFactor;
  else
    yCmd = 0;

  zCmd = currentGPS.altitude + zOffsetDesired;

  //! Main closed-loop receding setpoint position control
  while (elapsedTimeInMs < timeoutInMilSec)
  {
    v->control->positionAndYawCtrl(xCmd, yCmd, zCmd, yawDesiredRad / DEG2RAD);

    delay_nms(cycleTimeInMs);
    elapsedTimeInMs += cycleTimeInMs;

    //! Get current position in required coordinates and units
    q          = v->subscribe->getValue<Telemetry::TOPIC_QUATERNION>();
    yawInRad   = toEulerAngle(q).z;
    currentGPS = v->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
    localOffsetFromGpsOffset(localOffset, currentGPS, originGPS);

    //! See how much farther we have to go
    xOffsetRemaining = xOffsetDesired - localOffset.x;
    yOffsetRemaining = yOffsetDesired - localOffset.y;
    zOffsetRemaining = zOffsetDesired - (-localOffset.z);

    //! See if we need to modify the setpoint
    if (std::abs(xOffsetRemaining) < speedFactor)
      xCmd = xOffsetRemaining;
    if (std::abs(yOffsetRemaining) < speedFactor)
      yCmd = yOffsetRemaining;

    if (std::abs(xOffsetRemaining) < posThresholdInM &&
        std::abs(yOffsetRemaining) < posThresholdInM &&
        std::abs(zOffsetRemaining) < zDeadband &&
        std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
    {
      //! 1. We are within bounds; start incrementing our in-bound counter
      withinBoundsCounter += cycleTimeInMs;
    }
    else
    {
      if (withinBoundsCounter != 0)
      {
        //! 2. Start incrementing an out-of-bounds counter
        outOfBounds += cycleTimeInMs;
      }
    }
    //! 3. Reset withinBoundsCounter if necessary
    if (outOfBounds > outOfControlBoundsTimeLimit)
    {
      withinBoundsCounter = 0;
      outOfBounds         = 0;
    }
    //! 4. If within bounds, set flag and break
    if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
    {
      break;
    }
  }

  //! Set velocity to zero, to prevent any residual velocity from position
  //! command

  while (brakeCounter < withinControlBoundsTimeReqmt)
  {
    v->control->emergencyBrake();
    delay_nms(cycleTimeInMs);
    brakeCounter += cycleTimeInMs;
  }

  if (elapsedTimeInMs >= timeoutInMilSec)
  {
    printf("Task timeout!\n");

    // Cleanup
    v->subscribe->removePackage(pkgIndex);
    delay_nms(500);

    return ACK::FAIL;
  }

  v->subscribe->removePackage(pkgIndex);
  delay_nms(3000);
  return ACK::SUCCESS;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
 *  coordinates (accurate when distances are small).
 */
void
localOffsetFromGpsOffset(Telemetry::Vector3f& deltaNed,
                         Telemetry::GPSFused& target,
                         Telemetry::GPSFused& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;
  deltaNed.x      = deltaLat * C_EARTH;
  deltaNed.y      = deltaLon * C_EARTH * cos(target.latitude);
  deltaNed.z      = target.altitude - origin.altitude;
}

Telemetry::Vector3f
toEulerAngle(
  Telemetry::TypeMap<Telemetry::TOPIC_QUATERNION>::type& quaternionData)
{
  Telemetry::Vector3f ans;

  double q2sqr = quaternionData.q2 * quaternionData.q2;
  double t0    = -2.0 * (q2sqr + quaternionData.q3 * quaternionData.q3) + 1.0;
  double t1    = +2.0 * (quaternionData.q1 * quaternionData.q2 +
                      quaternionData.q0 * quaternionData.q3);
  double t2 = -2.0 * (quaternionData.q1 * quaternionData.q3 -
                      quaternionData.q0 * quaternionData.q2);
  double t3 = +2.0 * (quaternionData.q2 * quaternionData.q3 +
                      quaternionData.q0 * quaternionData.q1);
  double t4 = -2.0 * (quaternionData.q1 * quaternionData.q1 + q2sqr) + 1.0;

  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;

  ans.x = asin(t2);
  ans.y = atan2(t3, t4);
  ans.z = atan2(t1, t0);

  return ans;
}
