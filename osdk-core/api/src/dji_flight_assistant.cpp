/** @file dji_flight_assistant.cpp
 *  @version 3.9
 *  @date April 2019SSS
 *
 *  @brief
 *  Flight Assistant API for DJI OSDK library
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

#include "dji_flight_assistant.hpp"
#include "dji_flight_module.hpp"

using namespace DJI;
using namespace DJI::OSDK;

FlightAssistant::FlightAssistant(Vehicle* vehicle) {
  controllerModule = new FlightModule(vehicle);
}
FlightAssistant::~FlightAssistant() { delete this->controllerModule; }

ErrorCode::ErrorCodeType FlightAssistant::setRtkEnableSync(
    FlightModule::RtkEnableData rtkEnable, int timeout) {
  if (controllerModule)
    return controllerModule->setRtkEnableSync(rtkEnable, timeout);
  else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightAssistant::setRtkEnableAsync(
    FlightModule::RtkEnableData rtkEnable,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (controllerModule) {
    controllerModule->setRtkEnableAsync(rtkEnable, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType FlightAssistant::getRtkEnableSync(
    FlightModule::RtkEnableData& rtkEnable, int timeout) {
  if (controllerModule)
    return controllerModule->getRtkEnableSync(rtkEnable, timeout);
  else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightAssistant::getRtkEnableAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         FlightModule::RtkEnableData rtkEnable,
                         UserData userData),
    UserData userData) {
  if (controllerModule) {
    controllerModule->getRtkEnableAsync(UserCallBack, userData);
  } else {
    FlightModule::RtkEnableData rtkEnable;
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, rtkEnable,
                   userData);
  }
}

ErrorCode::ErrorCodeType FlightAssistant::setGoHomeAltitudeSync(
    FlightModule::GoHomeAltitude altitude, int timeout) {
  if (controllerModule)
    return controllerModule->setGoHomeAltitudeSync(altitude, timeout);
  else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightAssistant::setGoHomeAltitudeAsync(
    FlightModule::GoHomeAltitude altitude,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (controllerModule) {
    controllerModule->setGoHomeAltitudeAsync(altitude, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType FlightAssistant::getGoHomeAltitudeSync(
    FlightModule::GoHomeAltitude& altitude, int timeout) {
  if (controllerModule)
    return controllerModule->getGoHomeAltitudeSync(altitude, timeout);
  else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightAssistant::getGoHomeAltitudeAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         FlightModule::GoHomeAltitude altitude,
                         UserData userData),
    UserData userData) {
  if (controllerModule)
    controllerModule->getGoHomeAltitudeAsync(UserCallBack, userData);
  else {
    FlightModule::GoHomeAltitude altitude;
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, altitude,
                   userData);
  }
}

ErrorCode::ErrorCodeType FlightAssistant::setAvoidObstacleSwitchSync(
    FlightModule::AvoidObstacleData avoidObstacle, int timeout) {
  if (controllerModule)
    return controllerModule->setAvoidObstacleSwitchSync(avoidObstacle, timeout);
  else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightAssistant::setAvoidObstacleSwitchAsync(
    FlightModule::AvoidObstacleData avoidObstacle,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (controllerModule)
    controllerModule->setAvoidObstacleSwitchAsync(avoidObstacle, UserCallBack,
                                                  userData);
  else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType FlightAssistant::setHomePointSync(
    FlightModule::SetHomepointData homePoint, int timeout) {
  if (controllerModule)
    return controllerModule->setHomePointSync(homePoint, timeout);
  else
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
}

void FlightAssistant::setHomePointAsync(
    FlightModule::SetHomepointData homePoint,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (controllerModule)
    controllerModule->setHomePointAsync(homePoint, UserCallBack, userData);
  else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}
