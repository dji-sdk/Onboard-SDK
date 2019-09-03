/*! @file flight_controller_sample.cpp
 *  @version 3.9
 *  @date  August 05 2019
 *
 *  @brief
 *  Flight Controller API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for go home ,landing, set rtk and avoid obstacle switch.
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

#include "flight_controller_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

bool setUpSubscription(Vehicle* vehicle, int pkgIndex, int freq,
                       TopicName topicList[], uint8_t topicSize, int timeout) {
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
    vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>();
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
    /*! Cleanup before return */
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }
  /*! Wait for the data to start coming in.*/
  sleep(2);
  homePointSetStatus =
      vehicle->subscribe->getValue<TOPIC_HOME_POINT_SET_STATUS>();
  homePointInfo = vehicle->subscribe->getValue<TOPIC_HOME_POINT_INFO>();
  ACK::ErrorCode ack =
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
  if (ACK::getError(ack)) {
    DERROR(
        "Error unsubscription; please restart the drone/FC to get back "
        "to a clean state.");
  }
  return true;
}

ErrorCode::ErrCodeType setGoHomeAltitude(
    Vehicle* vehicle, FlightAssistant::GoHomeAltitude altitude, int timeout) {
  ErrorCode::ErrCodeType ret =
      vehicle->flightAssistant->setGoHomeAltitudeSync(altitude, timeout);
  if (ret != ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set go home altitude failed, ErrorCode is:%8x", ret);
  } else {
    DSTATUS("Set go home altitude successfully,altitude is: %d", altitude);
  }
  return ret;
}

ErrorCode::ErrCodeType setNewHomePoint(Vehicle* vehicle, int timeout) {
  HomePointStatus homePointSetStatus;
  HomePointData originHomePoint;
  ErrorCode::ErrCodeType ret =
      ErrorCode::FlightControllerErr::SetHomePointErr::Fail;
  FlightAssistant::SetHomepointData homepoint = {
      FlightAssistant::HomePointType::DJI_HOMEPOINT_AIRCRAFT_LOACTON, 0, 0, 0};
  bool retCode =
      getHomePoint(vehicle, homePointSetStatus, originHomePoint, timeout);
  DSTATUS("retCode:%d , homePointSetStatus.status%d", retCode,
          homePointSetStatus.status);
  if (retCode && (homePointSetStatus.status == 1)) {
    ret = vehicle->flightAssistant->setHomePointSync(homepoint, timeout);
    if (ret != ErrorCode::SysCommonErr::Success) {
      DSTATUS("Set new home point failed, ErrorCode is:%8x", ret);
    } else {
      DSTATUS("Set new home point successfully");
    }
  }
  return ret;
}

ErrorCode::ErrCodeType openAvoidObstacle(Vehicle* vehicle, int timeout) {
  FlightAssistant::AvoidObstacleData data = {0};
  data.frontBrakeFLag = 1;
  ErrorCode::ErrCodeType ret =
      vehicle->flightAssistant->setAvoidObstacleSwitchSync(data, timeout);
  if (ret != ErrorCode::SysCommonErr::Success) {
    DSTATUS("Open avoid obstacle switch failed, ErrorCode is:%8x", ret);
  } else {
    DSTATUS("Open avoid obstacle switch successfully");
  }
  return ret;
}

ErrorCode::ErrCodeType closeAvoidObstacle(Vehicle* vehicle, int timeout) {
  FlightAssistant::AvoidObstacleData data = {0};
  data.frontBrakeFLag = 0;
  ErrorCode::ErrCodeType ret =
      vehicle->flightAssistant->setAvoidObstacleSwitchSync(data, timeout);
  if (ret != ErrorCode::SysCommonErr::Success) {
    DSTATUS("Close avoid obstacle switch failed, ErrorCode is:%8x", ret);
  } else {
    DSTATUS("Close avoid obstacle switch successfully");
  }
  return ret;
}

ErrorCode::ErrCodeType openRtkSwtich(Vehicle* vehicle, int timeout) {
  ErrorCode::ErrCodeType ret = vehicle->flightAssistant->setRtkEnableSync(
      FlightAssistant::RtkEnableData::RTK_ENABLE, timeout);
  if (ret != ErrorCode::SysCommonErr::Success) {
    DSTATUS("Open rtk switch failed, ErrorCode is:%8x", ret);
  } else {
    DSTATUS("Open rtk switch successfully");
  }
  return ret;
}

ErrorCode::ErrCodeType closeRtkSwtich(Vehicle* vehicle, int timeout) {
  ErrorCode::ErrCodeType ret = vehicle->flightAssistant->setRtkEnableSync(
      FlightAssistant::RtkEnableData::RTK_DISABLE, timeout);
  if (ret != ErrorCode::SysCommonErr::Success) {
    DSTATUS("Close rtk switch failed, ErrorCode is:%8x", ret);
  } else {
    DSTATUS("Close rtk switch successfully");
  }
  return ret;
}

bool goHomeAndForceLanding(Vehicle* vehicle, int timeout) {
  const int pkgIndex = 0;
  int freq = 10;
  TopicName topicList[] = {TOPIC_STATUS_FLIGHT, TOPIC_STATUS_DISPLAYMODE,
                           TOPIC_AVOID_DATA, TOPIC_VELOCITY};
  int topicSize = sizeof(topicList) / sizeof(topicList[0]);

  /*! Step 1: Verify and setup the subscription */
  setUpSubscription(vehicle, pkgIndex, freq, topicList, topicSize, timeout);

  /*! Step 2: Start go home */
  DSTATUS("Start go home action");
  ErrorCode::ErrCodeType goHomeAck =
      vehicle->flightActions->startGoHomeSync(timeout);
  if (goHomeAck != ErrorCode::SysCommonErr::Success) {
    DERROR("Fail to execute go home action! ");
    return false;
  }
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
  DSTATUS("Finished go home action");

  /*! Step 3: Start landing */
  DSTATUS("Start landing action");
  if (!checkActionStarted(vehicle,
                          VehicleStatus::DisplayMode::MODE_AUTO_LANDING)) {
    DERROR("Fail to execute Landing action! ");
    return false;
  } else {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
               VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
               VehicleStatus::FlightStatus::IN_AIR) {
      Telemetry::TypeMap<TOPIC_AVOID_DATA>::type avoidData =
          vehicle->subscribe->getValue<TOPIC_AVOID_DATA>();
      sleep(1);
      if ((0.65 < avoidData.down && avoidData.down < 0.75) &&
          (avoidData.downHealth == 1)) {
        break;
      }
    }
  }
  DSTATUS("Finished landing action");

  /*! Step 4: Force landing and avoid ground */
  DSTATUS("Start force Landing and avoid ground action");
  ErrorCode::ErrCodeType forceLandingAvoidGroundAck =
      vehicle->flightActions->startForceLandingAvoidGroundSync(timeout);
  if (forceLandingAvoidGroundAck != ErrorCode::SysCommonErr::Success) {
    DERROR("Fail to execute force landing avoid ground action! ");
    return false;
  }
  if (!checkActionStarted(vehicle,
                          VehicleStatus::DisplayMode::MODE_AUTO_LANDING)) {
    return false;
  } else {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
               VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
               VehicleStatus::FlightStatus::IN_AIR) {
      sleep(1);
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
    teardownSubscription(vehicle, pkgIndex, timeout);
    return false;
  }
  /*! Step 6: Cleanup */
  teardownSubscription(vehicle, pkgIndex, timeout);
  return true;
}
