/** @file dji_flight_actions.cpp
 *  @version 3.9
 *  @date April 2019
 *
 *  @brief
 *  FlightActions API for DJI OSDK library
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
#include "dji_flight_actions.hpp"
#include "dji_flight_module.hpp"

using namespace DJI;
using namespace DJI::OSDK;

FlightActions::FlightActions(Vehicle *vehicle) {
  flightModule = new FlightModule(vehicle);
}

FlightActions::~FlightActions() { delete this->flightModule; }

ErrorCode::ErrorCodeType FlightActions::startTakeoffSync(int timeout) {
  return flightModule->actionSync(FlightModule::FlightCommand::TAKE_OFF,
                                      timeout);
}

void FlightActions::startTakeoffAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  flightModule->actionAsync(FlightModule::FlightCommand::TAKE_OFF,
                                FlightModule::commonAckDecoder,
                                UserCallBack, userData);
}

ErrorCode::ErrorCodeType FlightActions::startForceLandingSync(int timeout) {
  return flightModule->actionSync(
      FlightModule::FlightCommand::FORCE_LANDING, timeout);
}

void FlightActions::startForceLandingAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  flightModule->actionAsync(FlightModule::FlightCommand::FORCE_LANDING,
                                FlightModule::commonAckDecoder,
                                UserCallBack, userData);
}

ErrorCode::ErrorCodeType FlightActions::startForceLandingAvoidGroundSync(
    int timeout) {
  return flightModule->actionSync(
      FlightModule::FlightCommand::FORCE_LANDING_AVOID_GROUND, timeout);
}

void FlightActions::startForceLandingAvoidGroundAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  flightModule->actionAsync(
      FlightModule::FlightCommand::FORCE_LANDING_AVOID_GROUND,
      FlightModule::commonAckDecoder, UserCallBack, userData);
}

ErrorCode::ErrorCodeType FlightActions::startGoHomeSync(int timeout) {
  return flightModule->actionSync(FlightModule::FlightCommand::GO_HOME,
                                      timeout);
}

void FlightActions::startGoHomeAsync(
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  flightModule->actionAsync(FlightModule::FlightCommand::GO_HOME,
                                FlightModule::commonAckDecoder,
                                UserCallBack, userData);
}
