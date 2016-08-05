/*! @file LinuxFlight.cpp
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

#include "LinuxFlight.h"

//! Pre-Flight Functions

//! Arm the aircraft (Blocking API call) and return status as well as ack
ackReturnData arm(Flight* flight, int timeout)
{
  ackReturnData armAck;
  armAck.ack = flight->setArm(true, timeout);
  switch (armAck.ack)
  {
    case ACK_ARM_SUCCESS:
      std::cout << "Armed successfully." << std::endl;
      armAck.status = 1;
      return armAck;
      break;
    case ACK_ARM_NEED_CONTROL:
      std::cout << "Need to obtain control before arming." << std::endl;
      break;
    case ACK_ARM_ALREADY_ARMED:
      std::cout <<  "Already armed" << std::endl;
      break;
    case ACK_ARM_IN_AIR:
      std::cout << "Cannot arm while in air" << std::endl;
      break; 
  }
  armAck.status = -1;
  return armAck;
}

//! Takeoff (Blocking API call) and return status as well as ack. The ack is only for starting takeoff
ackReturnData takeoff(Flight* flight, int timeout)
{
  //Call takeoff task
  ackReturnData takeoffAck;
  takeoffAck.ack = flight->task(Flight::TASK_TAKEOFF, timeout);
  //Get takeoff ACK
  if (takeoffAck.ack == 0x0001)
  {
    //Execution failed
    std::cout << "Takeoff failed. This might happen due to a number of reasons, including the following:\n(1) If the aircraft is armed, auto takeoff does not work. Disarm and try again.\n(2) If you have not obtained control, auto takeoff does not work.\n(3) If you just landed, wait for 2s before trying auto takeoff." << std::endl;
    takeoffAck.status = -1;
    return takeoffAck;
  }
  else if (takeoffAck.ack != 0x0002)
  {
    //Execution failed - error in FC's ACK.
    std::cout << "Takeoff failed. The flight controller returned an unknown response. \nTry again, if that fails then reboot everything and try again." << std::endl;
    takeoffAck.status = -1;
    return takeoffAck;
  }
  else
  {
    //Takeoff started successfully.
    std::cout << "Takeoff started successfully." << std::endl;
  }
  takeoffAck.status = 1;
  return takeoffAck;
}

/*! Monitored Takeoff (Blocking API call). Return status as well as ack. 
    This version of takeoff makes sure your aircraft actually took off 
    and only returns when takeoff is complete. 
    Use unless you want to do other stuff during takeoff - this will block 
    the main thread.
!*/ 
ackReturnData monitoredTakeoff(CoreAPI* api, Flight* flight, int timeout)
{
  //Start takeoff
  ackReturnData takeoffStatus = takeoff(flight, timeout);
  if (takeoffStatus.status == -1) {return takeoffStatus;}

  //Start broadcasting flight status as well as position info 
  uint8_t freq[16];

  freq[0] = BROADCAST_FREQ_0HZ;
  freq[1] = BROADCAST_FREQ_0HZ;
  freq[2] = BROADCAST_FREQ_0HZ;
  freq[3] = BROADCAST_FREQ_50HZ;
  freq[4] = BROADCAST_FREQ_0HZ;
  freq[5] = BROADCAST_FREQ_50HZ;
  freq[6] = BROADCAST_FREQ_0HZ;
  freq[7] = BROADCAST_FREQ_0HZ;
  freq[8] = BROADCAST_FREQ_0HZ;
  freq[9] = BROADCAST_FREQ_10HZ;
  freq[10] = BROADCAST_FREQ_0HZ;
  freq[11] = BROADCAST_FREQ_0HZ; 

  unsigned short broadcastAck = api->setBroadcastFreq(freq, timeout);
  //If ACK = true, then start monitoring broadcastData. 

  //! First check: See if the aircraft actually got off the ground
  int stillOnGround = 0;
  int timeoutCycles = 20;
  while (api->getBroadcastData().status != 3 && stillOnGround < timeoutCycles)
  {
    stillOnGround++;
    usleep (100000);
  }
  if (stillOnGround == timeoutCycles)
  {
    std::cout << "Takeoff failed. Aircraft is still on the ground." << std::endl;
    std::cout << "Trying to takeoff again.." << std::endl;
    //Try again
    takeoffStatus = takeoff(flight, timeout);
    stillOnGround = 0;
    while (api->getBroadcastData().status != 3 && stillOnGround < timeoutCycles)
    {
      stillOnGround++;
      usleep (100000);
    }
    if (stillOnGround == timeoutCycles)
    {
      std::cout << "Takeoff failed twice. Aircraft is still on the ground." << std::endl;
      takeoffStatus.status = -1;
      return takeoffStatus;
    }
  }
  //! Second check: See if takeoff completed successfully
  int takeoffInProgress = 0;
  timeoutCycles = 40;
  float currentZ = 0.1, prevZ = 0;

  if(takeoffConverged(api, timeoutCycles))
  {
    takeoffStatus.status = 1;
    return takeoffStatus;
  }
  else
  {
    takeoffStatus.status = -2;
    return takeoffStatus;
  }
}

//! Check if takeoff completed and stabilized.
bool takeoffConverged(CoreAPI* api, int settlingTimeout)
{
  //! Velocity check algorithm
  double currentVelZ = api->getBroadcastData().v.z;
  //! Still not started to ascend
  int liftoffTimeout = 500; //Hundredths of a second
  int cycles = 0;
  while (currentVelZ < 0.25 && cycles < liftoffTimeout)
  {
    usleep(10000);
    currentVelZ = api->getBroadcastData().v.z;
    cycles++;
  }
  if (cycles == liftoffTimeout)
  {
    std::cout << "Did not lift off. Are your props correctly connected? Did you call takeoff after arming? See DJI GO for more info." << std::endl;
    return false;
  }
  else
  {
    //! Has started ascending
    std::cout << "Ascending" << std::endl;
  }
  //! Wait until we stop ascending
  while (currentVelZ > 0.25)
  {
    usleep(10000);
    currentVelZ = api->getBroadcastData().v.z;
  }
  std::cout << "Reached Takeoff altitude. Stabilizing.." << std::endl;
  //! Wait for stabilization
  int t = 0; //Tenths of a second
  int settled = 0; //Tenths of a second
  while(t < settlingTimeout)
  {
    if (abs(currentVelZ) < 0.25)
    {
      settled++;
    }
    else
    {
      settled = 0;
    }
    usleep(100000);
    currentVelZ = api->getBroadcastData().v.z;
    t++;
  }
  if (settled > 0.75*t)
  {
    std::cout << "Stable takeoff" << std::endl;
  }
  else if (settled > 0.25*t)
  {
    std::cout << "Acceptable takeoff stability" << std::endl;
  }
  else 
  {
    std::cout << "Unstable takeoff. Please use caution when flying." << std::endl; 
    return false;
  }
  return true;
}


/*! Attitude Control allows you to set a desired attitude and returns 
    when it reaches that attitude. It does not hold that attitude and
    will return to nominal attitude after 50 ms of this function returning.
    Typical use would be as a building block in an outer loop that calls 
    this function in succession for different desired attitudes 
!*/

int attitudeControl(CoreAPI* api, Flight* flight, float32_t rollDesired, float32_t pitchDesired, float32_t yawDesired, int timeoutInMs, float thresholdInDeg) //Holds current height
{
  //!Convert to set Control modes
  uint8_t flag = 0x10; //Attitude + z-position control; referenced to ground frame; non-stable mode
  
  float32_t curZ = api->getBroadcastData().pos.height;

  QuaternionData curQuaternion = api->getBroadcastData().q;
  DJI::EulerAngle curEuler = Flight::toEulerAngle(curQuaternion);
  
  double thresholdInRad = DEG2RAD*thresholdInDeg;
  double rollDesiredRad = DEG2RAD*rollDesired;
  double pitchDesiredRad = DEG2RAD*pitchDesired;
  double yawDesiredRad = DEG2RAD*yawDesired;
  
  int elapsedTime = 0;
  //! 50Hz Attitude control loop
  while(std::abs(curEuler.roll - rollDesiredRad) > thresholdInRad || std::abs(curEuler.pitch - pitchDesiredRad) > thresholdInRad || std::abs(curEuler.yaw - yawDesiredRad) > thresholdInRad)
  {
    if (elapsedTime >= timeoutInMs)
      break;
    flight->setMovementControl(flag,rollDesired, pitchDesired, curZ, yawDesired);
    usleep(20000);
    elapsedTime += 20;
    curEuler = Flight::toEulerAngle(api->getBroadcastData().q);
  }
  if (elapsedTime >= timeoutInMs)
    std::cout << "Timed out \n";

  return 1;
}

/*! Attitude/Altitude Control allows you to set a desired attitude and Z height 
 *  and returns when it reaches that attitude+altitude. It does not hold
 *  that attitude (but does hold altitude) and will return to nominal
 *  attitude after 50 ms of this function returning. Typical use would be
 *  as a building block in an outer loop that calls this function in
 *  succession for different desired attitudes. 
!*/

int attitudeAltitudeControl(CoreAPI* api, Flight* flight, float32_t rollDesired, float32_t pitchDesired, float32_t yawDesired, float32_t zDesired, int timeoutInMs, float attiThresholdInDeg, float zThresholdInCm)
{
  //!Convert to set Control modes
  uint8_t flag = 0x10; //Attitude + z-position control; referenced to ground frame; non-stable mode
  
  double attiThresholdInRad = DEG2RAD*attiThresholdInDeg;
  double rollDesiredRad = DEG2RAD*rollDesired;
  double pitchDesiredRad = DEG2RAD*pitchDesired;
  double yawDesiredRad = DEG2RAD*yawDesired;

  float32_t curZ = api->getBroadcastData().pos.height;
  QuaternionData curQuaternion = api->getBroadcastData().q;
  DJI::EulerAngle curEuler = Flight::toEulerAngle(curQuaternion);
  
  int elapsedTime = 0;
  //! 50Hz Attitude control loop
  while(std::abs(curEuler.roll - rollDesiredRad) > attiThresholdInRad || std::abs(curEuler.pitch - pitchDesiredRad) > attiThresholdInRad || std::abs(curEuler.yaw - yawDesiredRad) > attiThresholdInRad || std::abs(curZ - zDesired) > zThresholdInCm)
  {
    if (elapsedTime >= timeoutInMs)
      break;
    flight->setMovementControl(flag,rollDesired, pitchDesired, zDesired, yawDesired);
    usleep(20000);
    elapsedTime += 20;
    curEuler = Flight::toEulerAngle(api->getBroadcastData().q);
    curZ = api->getBroadcastData().pos.height;
  }
  if (elapsedTime >= timeoutInMs)
    std::cout << "Timed out \n";

  return 1;
}

/*! Position Control. Allows you to set an offset from your current
    location. The aircraft will move to that position and stay there. 
    Typical use would be as a building block in an outer loop that does not
    require many fast changes, perhaps a few-waypoint trajectory. For smoother
    transition and response you should convert your trajectory to attitude 
    setpoints and use attitude control or convert to velocity setpoints
    and use velocity control. 
!*/
int moveByPositionOffset(CoreAPI* api, Flight* flight, float32_t xOffsetDesired, float32_t yOffsetDesired, float32_t zOffsetDesired, float32_t yawDesired ,  int timeoutInMs, float yawThresholdInDeg, float posThresholdInCm)
{
  uint8_t flag = 0x91; //Position Control

  // Get current poition
  PositionData curPosition = api->getBroadcastData().pos;
  PositionData originPosition = curPosition;
  DJI::Vector3dData curLocalOffset; 
  
  DJI::EulerAngle curEuler = Flight::toEulerAngle(api->getBroadcastData().q);

  //Convert position offset from first position to local coordinates
  localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
  
  //See how much farther we have to go
  float32_t xOffsetRemaining = xOffsetDesired - curLocalOffset.x;
  float32_t yOffsetRemaining = yOffsetDesired - curLocalOffset.y;
  float32_t zOffsetRemaining = zOffsetDesired - curLocalOffset.z;

  //Conversions
  double yawDesiredRad = DEG2RAD*yawDesired;
  double yawThresholdInRad = DEG2RAD*yawThresholdInDeg;
  float32_t posThresholdInM = posThresholdInCm/100;

  int elapsedTime = 0;
  int speedFactor = 2;
  float xCmd, yCmd, zCmd;

  /*! Calculate the inputs to send the position controller. We implement basic
      receding setpoint position control and the setpoint is always 1 m away 
      from the current position - until we get within a threshold of the goal.
      From that point on, we send the remaining distance as the setpoint.
  !*/ 
  if (xOffsetDesired > 0)
    xCmd = xOffsetDesired < speedFactor ? xOffsetDesired : speedFactor; 
  else if (xOffsetDesired < 0)
    xCmd = xOffsetDesired > -1*speedFactor ? xOffsetDesired : -1*speedFactor; 
  else 
    xCmd = 0;

  if (yOffsetDesired > 0)
    yCmd = yOffsetDesired < speedFactor ? yOffsetDesired : speedFactor;
  else if (yOffsetDesired < 0)
    yCmd = yOffsetDesired > -1*speedFactor ? yOffsetDesired : -1*speedFactor; 
  else
    yCmd = 0;

  zCmd = curPosition.height + zOffsetDesired;


  //! Main closed-loop receding setpoint position control
  while(std::abs(xOffsetRemaining) > posThresholdInM || std::abs(yOffsetRemaining) > posThresholdInM || std::abs(zOffsetRemaining) > posThresholdInM || std::abs(curEuler.yaw - yawDesiredRad) > yawThresholdInRad)
  {
    // Check timeout
    if (elapsedTime >= timeoutInMs)
    {
      break;
    }
    
    //MovementControl API call
    flight->setMovementControl(flag,xCmd, yCmd, zCmd, yawDesired);
    usleep(20000);
    elapsedTime += 20;

    //Get current position in required coordinates and units 
    curEuler = Flight::toEulerAngle(api->getBroadcastData().q);
    curPosition = api->getBroadcastData().pos;
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
    
    //See how much farther we have to go
    xOffsetRemaining = xOffsetDesired - curLocalOffset.x;
    yOffsetRemaining = yOffsetDesired - curLocalOffset.y;
    zOffsetRemaining = zOffsetDesired - curLocalOffset.z;
    //See if we need to modify the setpoint
    if (std::abs(xOffsetRemaining) < speedFactor)
      xCmd = xOffsetRemaining;
    if (std::abs(yOffsetRemaining) < speedFactor)
      yCmd = yOffsetRemaining;

  }
  return 1;
}

/*! Velocity Control. Allows you to set a body-frame velocity setpoint 
    of the form (x_dot, y_dot, z_dot, yaw_dot). 
    The aircraft will reach that velocity and this functio      n will return.

    Typical use would be as a building block in an outer loop that calculates
    velocity setpoints and calls this function repeatedly. 
!*/
int moveWithVelocity(CoreAPI* api, Flight* flight, float32_t xVelocityDesired, float32_t yVelocityDesired, float32_t zVelocityDesired, float32_t yawRateDesired ,  int timeoutInMs, float yawRateThresholdInDegS, float velThresholdInMs)
{
  uint8_t flag = 0x49; //Velocity Control

  VelocityData curVelocity = api->getBroadcastData().v;
  CommonData curW = api->getBroadcastData().w;

  double yawRateDesiredRad = DEG2RAD*yawRateDesired;
  double yawRateThresholdInRadS = DEG2RAD*yawRateThresholdInDegS;

  int elapsedTime = 0;

  while(std::abs(curVelocity.x - xVelocityDesired) > velThresholdInMs || std::abs(curVelocity.y - yVelocityDesired) > velThresholdInMs || std::abs(curVelocity.z - zVelocityDesired) > velThresholdInMs || std::abs(curW.z - yawRateDesiredRad) > yawRateThresholdInRadS)
  {
    if (elapsedTime >= timeoutInMs)
      break;
    
    flight->setMovementControl(flag, xVelocityDesired, yVelocityDesired, zVelocityDesired, yawRateDesired);
    usleep(20000);
    elapsedTime += 20;
    
    curW = api->getBroadcastData().w;
    curVelocity = api->getBroadcastData().v;
  }
  
  if (elapsedTime >= timeoutInMs)
    std::cout << "Timed out \n";

  return 1;
}


//! Post-Flight functions

//! Disarm the aircraft (Blocking API call) and return status as well as ack
ackReturnData disArm(Flight* flight, int timeout)
{
  ackReturnData disArmAck;
  disArmAck.ack = flight->setArm(false, timeout);
  switch (disArmAck.ack)
  {
    case ACK_ARM_SUCCESS:
      std::cout << "Disarmed successfully." << std::endl;
      disArmAck.status = 1;
      return disArmAck;
      break;
    case ACK_ARM_NEED_CONTROL:
      std::cout << "Need to obtain control before disarming." << std::endl;
      break;
    case ACK_ARM_ALREADY_ARMED:
      std::cout <<  "Already disarmed" << std::endl;
      break;
    case ACK_ARM_IN_AIR:
      std::cout << "Cannot disarm while in air. Try to land instead." << std::endl;
      break;
  }
  disArmAck.status = -1;
  return disArmAck;
}

/*! Land. If it does not work the first time, this function waits 
    for 4 seconds and tries to land again. Wait until landing finishes to return to caller.
!*/
ackReturnData landing(CoreAPI* api, Flight* flight, int timeout)
{
  //Call landing task
  ackReturnData landingAck;
  landingAck.ack = flight->task(Flight::TASK_LANDING, timeout);
  //Get landing ACK
  if (landingAck.ack == 0x0001)
  {
    //Execution failed
    std::cout << "Landing failed. This might happen due to a number of reasons, including the following:\n(1) If the aircraft is armed but not in air, landing does not work. Disarm instead.\n(2) If you have not obtained control, auto landing does not work." << std::endl;
    std::cout << "Trying to land again..\n";
    usleep(4000000);
    landingAck.ack = flight->task(Flight::TASK_LANDING, timeout);
    if (landingAck.ack == 0x0001) 
    {
      landingAck.status = -1;
      return landingAck;
    }
    else
      std::cout << "Landing started successfully.\n";
  }
  else if (landingAck.ack != 0x0002)
  {
    //Execution failed - error in FC's ACK.
    std::cout << "Landing failed. The flight controller returned an unknown response. \nTry disarming or landing manually." << std::endl;
    landingAck.status = -1;
    return landingAck;
  }
  else
  {
    //Landing started successfully.
    std::cout << "Landing started successfully." << std::endl;

    //Set a timeout to make sure the system doesn't freeze in case landing does not kick in
    float landingStartTimeout = 3; //seconds 
    float landingTimer = 0;
    while (api->getBroadcastData().status != 4 && landingTimer < landingStartTimeout)
    {
      usleep(50000);
      landingTimer += 0.05;
    }
    if (landingTimer >= landingStartTimeout)
    {
      landingTimer = 0;
      std::cout << "Did not enter landing mode. Please land manually.\n";
      landingAck.status = -1;
      return landingAck;
    }

    std::cout << "Descending..\n";
    while (api->getBroadcastData().status == 4)
    {
      usleep(500000);
    }
    std::cout << "Landed. Wait until system is ready to accept new commands..\n";
    
    //Set a timeout to make sure the system doesn't freeze in case landing cleanup does not finish
    float landingFinishTimeout = 4; //Seconds
    while (api->getBroadcastData().status == 5 && landingTimer < landingFinishTimeout)
    {
      usleep(500000);
      landingFinishTimeout += 0.5;
    }
    if (landingTimer >= landingFinishTimeout)
    {
      landingTimer = 0;
      std::cout << "Did not clean up after landing. Your next command might need to be sent twice.\n" ;
      landingAck.status = -2;
      return landingAck;
    }
    std::cout << "Landing completed successfully.\n";
  }
  landingAck.status = 1;
  return landingAck;
}

// Execute Return to Home command
ackReturnData goHome(Flight* flight, int timeout)
{
  ackReturnData goHomeAck;
  goHomeAck.ack = flight->task(Flight::TASK_GOHOME, timeout);
  //Get goHome ACK
  if (goHomeAck.ack == 0x0001)
  {
    //Execution failed
    std::cout << "Go home failed. This might happen due to a number of reasons, including the following:\n(1) If the aircraft is armed but not in air, go home does not work. Disarm instead.\n(2) If you have not obtained control, auto landing does not work." << std::endl;
    goHomeAck.status = -1;
    return goHomeAck;
  }
  else if (goHomeAck.ack != 0x0002)
  {
    //Execution failed - error in FC's ACK.
    std::cout << "Go home failed. The flight controller returned an unknown response. \nTry disarming or landing manually." << std::endl;
    goHomeAck.status = -1;
    return goHomeAck;
  }
  else
  {
    //Takeoff started successfully.
    std::cout << "Go home started successfully." << std::endl;
  }
  goHomeAck.status = 1;
  return goHomeAck;
}

//! Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS coordinates.
    Accurate when distances are small.
!*/
void localOffsetFromGpsOffset(DJI::Vector3dData& deltaNed,
    PositionData* target,
    PositionData* origin)
{
  double deltaLon = target->longitude - origin->longitude;
  double deltaLat = target->latitude - origin->latitude;
  deltaNed.x =  deltaLat * C_EARTH;
  deltaNed.y = deltaLon * C_EARTH * cos(target->latitude);
  deltaNed.z = target->height - origin->height;
}

int drawSqrPosCtrlSample(CoreAPI* api, Flight* flight)
{
  //Check if in air
  if (api->getBroadcastData().status != 3)
  {
    std::cout << "Please takeoff and then try this sample.\n";
    return -1;
  }
  int positionControlStatus = moveByPositionOffset(api, flight, 0, 0, 10, 0);
  positionControlStatus = moveByPositionOffset(api, flight, 0, 10, 0, 0);
  positionControlStatus = moveByPositionOffset(api, flight, 10, 0, 0, 0);
  positionControlStatus = moveByPositionOffset(api, flight, 0, -10, 0, 0);
  positionControlStatus = moveByPositionOffset(api, flight, -10, 0, 0, 0);
  positionControlStatus = moveByPositionOffset(api, flight, 10, 10, 0, 0);
  positionControlStatus = moveByPositionOffset(api, flight, 0, -10, 0, 0);
  positionControlStatus = moveByPositionOffset(api, flight, -10, 10, 0, 0);
  //Stabilize at endpoint
  positionControlStatus = moveWithVelocity(api, flight, 0, 0, 0, 0, 2000, 0,0);
  return positionControlStatus;
}
