/*! @file flight_sample.cpp
 *  @version 4.0
 *  @date  Dec 2019
 *
 *  @brief
 *  Flight sample us FlightController API  in a Linux environment..
 *  Provides a number of helpful additions to core API calls,
 *  especially for joystick, go home ,landing, set rtk switch.
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

#include "flight_sample.hpp"
#include <cmath>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

static const double EarthCenter = 6378137.0;
static const double DEG2RAD = 0.01745329252;

FlightSample::FlightSample(Vehicle* vehicle) { this->vehicle = vehicle; }
FlightSample::~FlightSample() { delete (this->vehicle); }

bool FlightSample::monitoredTakeoff(int timeout) {
  int pkgIndex = 0;
  int freq = 10;
  TopicName topicList10Hz[] = {TOPIC_STATUS_FLIGHT, TOPIC_STATUS_DISPLAYMODE};
  int topicSize = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  setUpSubscription(pkgIndex, freq, topicList10Hz, topicSize, timeout);

  //! Start takeoff
  //ErrorCode::ErrorCodeType takeoffStatus =
      vehicle->flightController->startTakeoffSync(timeout);

  //! Motors start check
  if (!motorStartedCheck()) {
    std::cout << "Takeoff failed. Motors are not spinning." << std::endl;
    teardownSubscription(pkgIndex, timeout);
    return false;
  } else {
    std::cout << "Motors spinning...\n";
  }
  //! In air check
  if (!takeOffInAirCheck()) {
    std::cout << "Takeoff failed. Aircraft is still on the ground, but the "
                 "motors are spinning."
              << std::endl;
    teardownSubscription(pkgIndex, timeout);
    return false;
  } else {
    std::cout << "Ascending...\n";
  }

  //! Finished takeoff check
  if (takeoffFinishedCheck()) {
    std::cout << "Successful takeoff!\n";
  } else {
    std::cout << "Takeoff finished, but the aircraft is in an unexpected mode. "
                 "Please connect DJI GO.\n";
    teardownSubscription(pkgIndex, timeout);
    return false;
  }
  teardownSubscription(pkgIndex, timeout);
  return true;
}

bool FlightSample::moveByPositionOffset(const Vector3f& offsetDesired,
                                        float yawDesiredInDeg,
                                        float posThresholdInM,
                                        float yawThresholdInDeg) {
  int responseTimeout = 1;
  int timeoutInMilSec = 40000;
  int controlFreqInHz = 50;  // Hz
  int cycleTimeInMs = 1000 / controlFreqInHz;
  int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs;    // 10 cycles
  int withinControlBoundsTimeReqmt = 100 * cycleTimeInMs;  // 100 cycles
  int elapsedTimeInMs = 0;
  int withinBoundsCounter = 0;
  int outOfBounds = 0;
  int brakeCounter = 0;
  int speedFactor = 2;

  int pkgIndex = 0;
  TopicName topicList[] = {TOPIC_QUATERNION, TOPIC_GPS_FUSED,
                           TOPIC_HEIGHT_FUSION};
  int numTopic = sizeof(topicList) / sizeof(topicList[0]);
  if (!setUpSubscription(pkgIndex, controlFreqInHz, topicList, numTopic,
                         responseTimeout)) {
    return false;
  }
  sleep(1);
  //! get origin position and relative height(from home point)of aircraft.
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originGPSPosition =
      vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
  /*! TODO: TOPIC_HEIGHT_FUSION is abnormal in real world but normal in simulator */
  Telemetry::GlobalPosition currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
  float32_t originHeightBaseHomepoint = currentBroadcastGP.height;
  FlightController::JoystickMode joystickMode = {
    FlightController::HorizontalLogic::HORIZONTAL_POSITION,
    FlightController::VerticalLogic::VERTICAL_POSITION,
    FlightController::YawLogic::YAW_ANGLE,
    FlightController::HorizontalCoordinate::HORIZONTAL_GROUND,
    FlightController::StableMode::STABLE_ENABLE,
  };
  vehicle->flightController->setJoystickMode(joystickMode);

  while (elapsedTimeInMs < timeoutInMilSec) {
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentGPSPosition =
        vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    Telemetry::TypeMap<TOPIC_QUATERNION>::type currentQuaternion =
        vehicle->subscribe->getValue<TOPIC_QUATERNION>();
    float yawInRad = quaternionToEulerAngle(currentQuaternion).z;
    //! get the vector between aircraft and origin point.
    Vector3f localOffset =
        localOffsetFromGpsOffset(currentGPSPosition, originGPSPosition);
    //! get the vector between aircraft and target point.
    Vector3f offsetRemaining = vector3FSub(offsetDesired, localOffset);

    Vector3f positionCommand = offsetRemaining;
    horizCommandLimit(speedFactor, positionCommand.x, positionCommand.y);

    FlightController::JoystickCommand joystickCommand = {
        positionCommand.x, positionCommand.y,
        offsetDesired.z + originHeightBaseHomepoint, yawDesiredInDeg};

    vehicle->flightController->setJoystickCommand(joystickCommand);

    vehicle->flightController->joystickAction();

    if (vectorNorm(offsetRemaining) < posThresholdInM &&
        std::fabs(yawInRad / DEG2RAD - yawDesiredInDeg) < yawThresholdInDeg) {
      //! 1. We are within bounds; start incrementing our in-bound counter
      withinBoundsCounter += cycleTimeInMs;
    } else {
      if (withinBoundsCounter != 0) {
        //! 2. Start incrementing an out-of-bounds counter
        outOfBounds += cycleTimeInMs;
      }
    }
    //! 3. Reset withinBoundsCounter if necessary
    if (outOfBounds > outOfControlBoundsTimeLimit) {
      withinBoundsCounter = 0;
      outOfBounds = 0;
    }
    //! 4. If within bounds, set flag and break
    if (withinBoundsCounter >= withinControlBoundsTimeReqmt) {
      break;
    }
    usleep(cycleTimeInMs * 1000);
    elapsedTimeInMs += cycleTimeInMs;
  }

  while (brakeCounter < withinControlBoundsTimeReqmt) {
    //! TODO: remove emergencyBrake
    vehicle->control->emergencyBrake();
    usleep(cycleTimeInMs * 1000);
    brakeCounter += cycleTimeInMs;
  }

  if (elapsedTimeInMs >= timeoutInMilSec) {
    std::cout << "Task timeout!\n";
    teardownSubscription(pkgIndex);
    return false;
  }
  teardownSubscription(pkgIndex);
  return true;
}

bool FlightSample::goHomeAndConfirmLanding(int timeout) {
  /*! Step 1: Verify and setup the subscription */
  const int pkgIndex = 0;
  int freq = 10;
  TopicName topicList[] = {TOPIC_STATUS_FLIGHT, TOPIC_STATUS_DISPLAYMODE,
                           TOPIC_AVOID_DATA, TOPIC_VELOCITY};
  int topicSize = sizeof(topicList) / sizeof(topicList[0]);
  setUpSubscription(pkgIndex, freq, topicList, topicSize, timeout);

  /*! Step 2: Start go home */
  DSTATUS("Start go home action");
  ErrorCode::ErrorCodeType goHomeAck =
      vehicle->flightController->startGoHomeSync(timeout);
  if (goHomeAck != ErrorCode::SysCommonErr::Success) {
    DERROR("Fail to execute go home action!  Error code: %llx\n", goHomeAck);
    return false;
  }
  if (!checkActionStarted(VehicleStatus::DisplayMode::MODE_NAVI_GO_HOME)) {
    return false;
  } else {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
               VehicleStatus::DisplayMode::MODE_NAVI_GO_HOME &&
           vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
               VehicleStatus::FlightStatus::IN_AIR) {
      Platform::instance().taskSleepMs(
          1000);  // waiting for this action finished
    }
  }
  DSTATUS("Finished go home action");

  /*! Step 3: Start landing */
  DSTATUS("Start landing action");
  if (!checkActionStarted(VehicleStatus::DisplayMode::MODE_AUTO_LANDING)) {
    DERROR("Fail to execute Landing action!");
    return false;
  } else {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
               VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
               VehicleStatus::FlightStatus::IN_AIR) {
      Telemetry::TypeMap<TOPIC_AVOID_DATA>::type avoidData =
          vehicle->subscribe->getValue<TOPIC_AVOID_DATA>();
      Platform::instance().taskSleepMs(1000);
      if ((0.65 < avoidData.down && avoidData.down < 0.75) &&
          (avoidData.downHealth == 1)) {
        break;
      }
    }
  }
  DSTATUS("Finished landing action");

  /*! Step 4: Confirm Landing */
  DSTATUS("Start confirm Landing and avoid ground action");
  ErrorCode::ErrorCodeType forceLandingAvoidGroundAck =
      vehicle->flightController->startConfirmLandingSync(timeout);
  if (forceLandingAvoidGroundAck != ErrorCode::SysCommonErr::Success) {
    DERROR(
        "Fail to execute confirm landing avoid ground action! Error code: "
        "%llx\n ",
        forceLandingAvoidGroundAck);
    return false;
  }
  if (!checkActionStarted(VehicleStatus::DisplayMode::MODE_AUTO_LANDING)) {
    return false;
  } else {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
               VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
               VehicleStatus::FlightStatus::IN_AIR) {
      Platform::instance().taskSleepMs(1000);
    }
  }
  DSTATUS("Finished force Landing and avoid ground action");

  /*! Step 5: Landing finished check*/
  if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::DisplayMode::MODE_P_GPS ||
      vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::DisplayMode::MODE_ATTITUDE) {
    DSTATUS("Successful landing!");
  } else {
    DERROR(
        "Landing finished, but the aircraft is in an unexpected mode. "
        "Please connect DJI Assistant.");
    teardownSubscription(pkgIndex, timeout);
    return false;
  }
  /*! Step 6: Cleanup */
  teardownSubscription(pkgIndex, timeout);
  return true;
}

bool FlightSample::setUpSubscription(int pkgIndex, int freq,
                                     TopicName topicList[], uint8_t topicSize,
                                     int timeout) {
  if (vehicle) {
    /*! Telemetry: Verify the subscription*/
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
      ACK::getErrorCodeMessage(subscribeStatus, __func__);
      return false;
    }

    bool enableTimestamp = false;
    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
        pkgIndex, topicSize, topicList, enableTimestamp, freq);
    if (!(pkgStatus)) {
      return pkgStatus;
    }

    /*! Start listening to the telemetry data */
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
      ACK::getErrorCodeMessage(subscribeStatus, __func__);
      /*! Cleanup*/
      ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
      if (ACK::getError(ack)) {
        DERROR(
            "Error unsubscription; please restart the drone/FC to get "
            "back to a clean state");
      }
      return false;
    }
    return true;
  } else {
    DERROR("vehicle haven't been initialized", __func__);
    return false;
  }
}

bool FlightSample::teardownSubscription(const int pkgIndex, int timeout) {
  ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
  if (ACK::getError(ack)) {
    DERROR(
        "Error unsubscription; please restart the drone/FC to get back "
        "to a clean state.");
    return false;
  }
  return true;
}

bool FlightSample::checkActionStarted(uint8_t mode) {
  int actionNotStarted = 0;
  int timeoutCycles = 20;
  while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != mode &&
         actionNotStarted < timeoutCycles) {
    actionNotStarted++;
    Platform::instance().taskSleepMs(100);
  }
  if (actionNotStarted == timeoutCycles) {
    DERROR("Start actions mode %d failed, current DISPLAYMODE is: %d ...", mode,
           vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>());
    return false;
  } else {
    DSTATUS("DISPLAYMODE: %d ...", mode);
    return true;
  }
}

bool FlightSample::getHomeLocation(HomeLocationSetStatus& homeLocationSetStatus,
                                   HomeLocationData& homeLocationInfo,
                                   int responseTimeout) {
  ACK::ErrorCode subscribeStatus;
  subscribeStatus = vehicle->subscribe->verify(responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }
  int pkgIndex = 0;
  int freq = 1;
  TopicName topicList[] = {TOPIC_HOME_POINT_SET_STATUS, TOPIC_HOME_POINT_INFO};

  int numTopic = sizeof(topicList) / sizeof(topicList[0]);
  bool enableTimestamp = false;

  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList, enableTimestamp, freq);

  if (!(pkgStatus)) {
    return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    /*! Cleanup before return */
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }
  /*! Wait for the data to start coming in.*/
  Platform::instance().taskSleepMs(2000);
  homeLocationSetStatus =
      vehicle->subscribe->getValue<TOPIC_HOME_POINT_SET_STATUS>();
  homeLocationInfo = vehicle->subscribe->getValue<TOPIC_HOME_POINT_INFO>();
  ACK::ErrorCode ack =
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
  if (ACK::getError(ack)) {
    DERROR(
        "Error unsubscription; please restart the drone/FC to get back "
        "to a clean state.");
  }
  return true;
}

ErrorCode::ErrorCodeType FlightSample::setGoHomeAltitude(
    FlightController::GoHomeHeight altitude, int timeout) {
  ErrorCode::ErrorCodeType ret =
      vehicle->flightController->setGoHomeAltitudeSync(altitude, timeout);
  if (ret != ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set go home altitude failed, ErrorCode is:%8x", ret);
  } else {
    DSTATUS("Set go home altitude successfully,altitude is: %d", altitude);
  }
  return ret;
}

ErrorCode::ErrorCodeType FlightSample::setNewHomeLocation(int timeout) {
  HomeLocationSetStatus homeLocationSetStatus;
  HomeLocationData originHomeLocation;
  ErrorCode::ErrorCodeType ret =
      ErrorCode::FlightControllerErr::SetHomeLocationErr::Fail;
  bool retCode =
      getHomeLocation(homeLocationSetStatus, originHomeLocation, timeout);
  if (retCode && (homeLocationSetStatus.status == 1)) {
    ret = vehicle->flightController
              ->setHomeLocationUsingCurrentAircraftLocationSync(timeout);
    if (ret != ErrorCode::SysCommonErr::Success) {
      DSTATUS("Set new home location failed, ErrorCode is:%8x", ret);
    } else {
      DSTATUS("Set new home location successfully");
    }
  }
  return ret;
}

void FlightSample::horizCommandLimit(float speedFactor, float& commandX,
                                     float& commandY) {
  if (fabs(commandX) > speedFactor)
    commandX = signOfData<float>(commandX) * speedFactor;
  if (fabs(commandY) > speedFactor)
    commandY = signOfData<float>(commandY) * speedFactor;
}

Vector3f FlightSample::quaternionToEulerAngle(
    const Telemetry::Quaternion& quat) {
  Telemetry::Vector3f eulerAngle;
  double q2sqr = quat.q2 * quat.q2;
  double t0 = -2.0 * (q2sqr + quat.q3 * quat.q3) + 1.0;
  double t1 = 2.0 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);
  double t2 = -2.0 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);
  double t3 = 2.0 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);
  double t4 = -2.0 * (quat.q1 * quat.q1 + q2sqr) + 1.0;
  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;
  eulerAngle.x = asin(t2);
  eulerAngle.y = atan2(t3, t4);
  eulerAngle.z = atan2(t1, t0);
  return eulerAngle;
}

template <typename Type>
int FlightSample::signOfData(Type type) {
  return type < 0 ? -1 : 1;
}

float32_t FlightSample::vectorNorm(Vector3f v) {
  return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

Vector3f FlightSample::vector3FSub(const Vector3f& vectorA,
                                   const Vector3f& vectorB) {
  Telemetry::Vector3f result;
  result.x = vectorA.x - vectorB.x;
  result.y = vectorA.y - vectorB.y;
  result.z = vectorA.z - vectorB.z;
  return result;
}

Vector3f FlightSample::localOffsetFromGpsOffset(
    const Telemetry::GPSFused& target, const Telemetry::GPSFused& origin) {
  Telemetry::Vector3f deltaNed;
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;
  deltaNed.x = deltaLat * EarthCenter;
  deltaNed.y = deltaLon * EarthCenter * cos(target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
  return deltaNed;
}

bool FlightSample::motorStartedCheck() {
  int motorsNotStarted = 0;
  int timeoutCycles = 20;
  while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
             VehicleStatus::FlightStatus::ON_GROUND &&
         vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
             VehicleStatus::DisplayMode::MODE_ENGINE_START &&
         motorsNotStarted < timeoutCycles) {
    motorsNotStarted++;
    usleep(100000);
  }
  return motorsNotStarted != timeoutCycles ? true : false;
}

bool FlightSample::takeOffInAirCheck() {
  int stillOnGround = 0;
  int timeoutCycles = 110;
  while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
             VehicleStatus::FlightStatus::IN_AIR &&
         (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
              VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
          vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
              VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
         stillOnGround < timeoutCycles) {
    stillOnGround++;
    usleep(100000);
  }

  return stillOnGround != timeoutCycles ? true : false;
}

bool FlightSample::takeoffFinishedCheck() {
  while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
         vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) {
    sleep(1);
  }
  return ((vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
           VehicleStatus::DisplayMode::MODE_P_GPS) ||
          (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
           VehicleStatus::DisplayMode::MODE_ATTITUDE))
             ? true
             : false;
}
