/** @file dji_flight_controller.cpp
 *  @version 3.9
 *  @date August 2019
 *
 *  @brief
 *  Flight Controller API for DJI OSDK library
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

#include "dji_flight_controller.hpp"
#include <dji_telemetry.hpp>
using namespace DJI;
using namespace DJI::OSDK;

FlightController::FlightController(Vehicle* vehicle) {
  flightAssistant = new FlightAssistant(vehicle);
  flightActions = new FlightActions(vehicle);
}
FlightController::~FlightController() {
  delete this->flightAssistant;
  delete this->flightActions;
}

ErrorCode::ErrorCodeType FlightController::setRtkEnableSync(
    RtkEnabled rtkEnable, int timeout) {
  if (flightAssistant)
    return flightAssistant->setRtkEnableSync(rtkEnable, timeout);
  else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightController::setRtkEnableAsync(
    RtkEnabled rtkEnable,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (flightAssistant) {
    flightAssistant->setRtkEnableAsync(rtkEnable, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType FlightController::getRtkEnableSync(
    RtkEnabled& rtkEnable, int timeout) {
  if (flightAssistant)
    return flightAssistant->getRtkEnableSync(rtkEnable, timeout);
  else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightController::getRtkEnableAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, RtkEnabled rtkEnable,
                         UserData userData),
    UserData userData) {
  if (flightAssistant) {
    flightAssistant->getRtkEnableAsync(UserCallBack, userData);
  } else {
    RtkEnabled rtkEnable = FlightAssistant::RtkEnableData::RTK_DISABLE;
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, rtkEnable,
                   userData);
  }
}

ErrorCode::ErrorCodeType FlightController::setGoHomeAltitudeSync(
    GoHomeHeight altitude, int timeout) {
  if (flightAssistant)
    return flightAssistant->setGoHomeAltitudeSync(altitude, timeout);
  else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightController::setGoHomeAltitudeAsync(
    GoHomeHeight altitude,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (flightAssistant) {
    flightAssistant->setGoHomeAltitudeAsync(altitude, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType FlightController::getGoHomeAltitudeSync(
    GoHomeHeight& altitude, int timeout) {
  if (flightAssistant)
    return flightAssistant->getGoHomeAltitudeSync(altitude, timeout);
  else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightController::getGoHomeAltitudeAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         GoHomeHeight altitude, UserData userData),
    UserData userData) {
  if (flightAssistant)
    flightAssistant->getGoHomeAltitudeAsync(UserCallBack, userData);
  else {
    GoHomeHeight altitude = 0;
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, altitude,
                   userData);
  }
}

ErrorCode::ErrorCodeType FlightController::setHomeLocationSync(
    HomeLocation homeLocation, int timeout) {
  if (flightAssistant) {
    FlightAssistant::SetHomeLocationData setHomeLocation = {
        FlightAssistant::DJI_HOMEPOINT_SDK_SET_LOCAIION, homeLocation.latitude,
        homeLocation.longitude, 0};
    return flightAssistant->setHomeLocationSync(setHomeLocation, timeout);
  } else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightController::setHomeLocationAsync(
    HomeLocation homeLocation,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (flightAssistant) {
    FlightAssistant::SetHomeLocationData setHomeLocation = {
        FlightAssistant::DJI_HOMEPOINT_SDK_SET_LOCAIION, homeLocation.latitude,
        homeLocation.longitude, 0};
    flightAssistant->setHomeLocationAsync(setHomeLocation, UserCallBack,
                                          userData);
  }

  else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType
FlightController::setHomeLocationUsingCurrentAircraftLocationSync(int timeout) {
  FlightAssistant::SetHomeLocationData homeLocation = {};
  homeLocation.homeType = FlightAssistant::DJI_HOMEPOINT_AIRCRAFT_LOACTON;
  if (flightAssistant)
    return flightAssistant->setHomeLocationSync(homeLocation, timeout);
  else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightController::setHomeLocationUsingCurrentAircraftLocationAsync(
  void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
  UserData userData) {
  FlightAssistant::SetHomeLocationData homeLocation = {};
  homeLocation.homeType = FlightAssistant::DJI_HOMEPOINT_AIRCRAFT_LOACTON;
  if (flightAssistant)
    flightAssistant->setHomeLocationAsync(homeLocation, UserCallBack, userData);
  else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType FlightController::startTakeoffSync(int timeout) {
  if (flightActions)
    return flightActions->actionSync(FlightActions::FlightCommand::TAKE_OFF,
                                     timeout);
  else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightController::startTakeoffAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (flightAssistant)
    flightActions->actionAsync(FlightActions::FlightCommand::TAKE_OFF,
                               FlightActions::commonAckDecoder, UserCallBack,
                               userData);
  else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType FlightController::startForceLandingSync(int timeout) {
  if (flightActions)
    return flightActions->actionSync(
        FlightActions::FlightCommand::FORCE_LANDING, timeout);
  else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightController::startForceLandingAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (flightAssistant)
    flightActions->actionAsync(FlightActions::FlightCommand::FORCE_LANDING,
                               FlightActions::commonAckDecoder, UserCallBack,
                               userData);
  else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType FlightController::startConfirmLandingSync(
    int timeout) {
  if (flightActions)
    return flightActions->actionSync(
        FlightActions::FlightCommand::FORCE_LANDING_AVOID_GROUND, timeout);
  else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightController::startConfirmLandingAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (flightAssistant)
    flightActions->actionAsync(
        FlightActions::FlightCommand::FORCE_LANDING_AVOID_GROUND,
        FlightActions::commonAckDecoder, UserCallBack, userData);
  else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType FlightController::startGoHomeSync(int timeout) {
  if (flightActions)
    return flightActions->actionSync(FlightActions::FlightCommand::GO_HOME,
                                     timeout);
  else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightController::startGoHomeAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (flightAssistant)
    flightActions->actionAsync(FlightActions::FlightCommand::GO_HOME,
                               FlightActions::commonAckDecoder, UserCallBack,
                               userData);
  else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}