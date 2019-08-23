/*! @file flight_controller_sample.cpp
 *  @version 3.9
 *  @date  August 05 2019
 *
 *  @brief
 *  Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2019 DJI
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
/*modify the path of include*/
#include "../inc/flight_controller_sample.hpp"
#include "dji_flight_assistant.hpp"
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

/*bool getLLAfromOffset(HomePointData origin, float northOffset, float
eastOffset,
                      HomePointData& target) {
  if ((-PI / 2 <= origin.latitude) && (origin.latitude <= PI / 2) &&
      (-PI <= origin.longitude) && (origin.longitude <= PI)) {
    target.latitude = origin.latitude + northOffset / C_EARTH;
    target.longitude =
      origin.longitude + eastOffset / C_EARTH / cos(origin.latitude);
    return true;
  } else
    return false;
}*/
bool setUpSubscription(Vehicle* vehicle, int pkgIndex, int freq,
                       TopicName topicList[], uint8_t topicSize, int timeout) {
  // Telemetry: Verify the subscription
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

  // Start listening to the telemetry data
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup
    ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
    if (ACK::getError(ack)) {
      DERROR(
          "Error unsubscription; please restart the drone/FC to get "
          "back to a clean state");
    }
    return false;
  }
  return true;
}

bool teardownSubscription(Vehicle* vehicle, const int pkgIndex, int timeout) {
  ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
  if (ACK::getError(ack)) {
    DERROR(
        "Error unsubscription; please restart the drone/FC to get back "
        "to a clean state.");
    return false;
  }
  return true;
}

bool checkActionStarted(Vehicle* vehicle, uint8_t mode) {
  int actionNotStarted = 0;
  int timeoutCycles = 20;
  while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != mode &&
         actionNotStarted < timeoutCycles) {
    actionNotStarted++;
    DSTATUS("vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>():%d",
            vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>());
    usleep(100000);
  }
  if (actionNotStarted == timeoutCycles) {
    return false;
  } else {
    DSTATUS("DISPLAYMODE: %d ...", mode);
    return true;
  }
}

bool getHomePoint(Vehicle* vehicle, HomePointStatus& homePointSetStatus,
                  HomePointData& homePointInfo, int responseTimeout) {
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
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }
  // Wait for the data to start coming in.
  sleep(2);
  homePointSetStatus =
      vehicle->subscribe->getValue<TOPIC_HOME_POINT_SET_STATUS>();
  homePointInfo = vehicle->subscribe->getValue<TOPIC_HOME_POINT_INFO>();
  ACK::ErrorCode ack =
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
  if (ACK::getError(ack)) {
    DERROR(
        "Error unsubscribing; please restart the drone/FC to get back "

        "to a clean state.");
  }
  return true;
}
bool setGoHomeAltitude(Vehicle* vehicle,
                       FlightAssistant::goHomeAltitude altitude, int timeout) {
  return (vehicle->flightAssistant->setGoHomeAltitudeSync(altitude, timeout) ==
          ErrorCode::UnifiedErrCode::kNoError)
             ? true
             : false;
}

bool setNewHomePoint(Vehicle* vehicle, int timeout) {
  HomePointStatus homePointSetStatus;
  HomePointData originHomePoint;
  FlightAssistant::SetHomepointData homepoint = {
      FlightAssistant::HomePointType::DJI_HOMEPOINT_AIRCRAFT_LOACTON, 0, 0, 0};
  bool retCode =
      getHomePoint(vehicle, homePointSetStatus, originHomePoint, timeout);
  if (retCode && (homePointSetStatus.status == 1)) {
    if (vehicle->flightAssistant->setHomePointSync(homepoint, timeout) ==
        ErrorCode::UnifiedErrCode::kNoError) {
      return true;
    }
  }
  return false;
}

bool openAvoidObstacle(Vehicle* vehicle, int timeout) {
  FlightAssistant::AvoidObstacleData data = {0};
  data.frontBrakeFLag = 1;
  ErrorCode::ErrCodeType ret =
      vehicle->flightAssistant->setAvoidObstacleSwitchSync(data, timeout);
  if (ret == ErrorCode::UnifiedErrCode::kNoError)
    return true;
  else
    return false;
}

bool closeAvoidObstacle(Vehicle* vehicle, int timeout) {
  FlightAssistant::AvoidObstacleData data = {0};
  data.frontBrakeFLag = 0;
  ErrorCode::ErrCodeType ret =
      vehicle->flightAssistant->setAvoidObstacleSwitchSync(data, timeout);
  if (ret == ErrorCode::UnifiedErrCode::kNoError)
    return true;
  else
    return false;
}

bool goHomeAndForceLanding(Vehicle* vehicle, int timeout) {
  const int pkgIndex = 0;
  int freq = 10;
  TopicName topicList[] = {TOPIC_STATUS_FLIGHT, TOPIC_STATUS_DISPLAYMODE,
                           TOPIC_AVOID_DATA};
  int topicSize = sizeof(topicList) / sizeof(topicList[0]);
  // Step 1: Verify the subscription
  setUpSubscription(vehicle, pkgIndex, freq, topicList, topicSize, timeout);

  // Step 2: Start go home
  ErrorCode::ErrCodeType goHomeAck =
      vehicle->flightActions->startGoHomeSync(timeout);
  if (goHomeAck != ErrorCode::UnifiedErrCode::kNoError) {
    DERROR("Fail to execute go home action! ");
    return false;
  }

  // Step 3: Check go home started or not
  if (!checkActionStarted(vehicle,
                          VehicleStatus::DisplayMode::MODE_NAVI_GO_HOME)) {
    return false;
  } else {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
               VehicleStatus::DisplayMode::MODE_NAVI_GO_HOME &&
           vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
               VehicleStatus::FlightStatus::IN_AIR) {
      sleep(1);  // waiting for this action finished
    }
  }
  // Step 4: start force landing avoid ground
  ErrorCode::ErrCodeType forceLandingAvoidGroundAck =
      vehicle->flightActions->startForceLandingAvoidGroundSync(timeout);
  if (forceLandingAvoidGroundAck != ErrorCode::UnifiedErrCode::kNoError) {
    DERROR("Fail to execute force landing avoid ground action! ");
    return false;
  }
  // Step 5: check force landing avoid ground action started or not
  if (!checkActionStarted(vehicle,
                          VehicleStatus::DisplayMode::MODE_AUTO_LANDING)) {
    return false;
  } else {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
               VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
               VehicleStatus::FlightStatus::IN_AIR) {
      DSTATUS("Current display mode:%d",
              vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>());
      Telemetry::TypeMap<TOPIC_AVOID_DATA>::type avoidData;
      avoidData = vehicle->subscribe->getValue<TOPIC_AVOID_DATA>();
      DSTATUS("Down distance:%d", avoidData.down);
      sleep(1);
    }
  }
  /* 这个地方有问题*/
  if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::DisplayMode::MODE_P_GPS ||
      vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::DisplayMode::MODE_ATTITUDE) {
    DSTATUS("Successful landing!");
  } else {
    DERROR(
        "Landing finished, but the aircraft is in an unexpected mode. "
        "Please connect DJI Assistant.");
    teardownSubscription(vehicle, pkgIndex, timeout);
    return false;
  }
  // Cleanup
  teardownSubscription(vehicle, pkgIndex, timeout);
  return true;
}
