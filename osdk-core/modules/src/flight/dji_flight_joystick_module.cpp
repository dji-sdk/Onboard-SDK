/** @file dji_flight_joystick_module.cpp
 *  @version 3.9
 *  @date July 2019
 *
 *  @brief Implementation of flight module
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

#include "dji_flight_joystick_module.hpp"
#include <dji_vehicle.hpp>
#include "dji_flight_link.hpp"

using namespace DJI;
using namespace DJI::OSDK;

FlightJoystick::FlightJoystick(Vehicle *vehicle) {
  flightLink = new FlightLink(vehicle);
  setHorizontalLogic(HORIZONTAL_POSITION);
  setVerticalLogic(VERTICAL_POSITION);
  setYawLogic(YAW_ANGLE);
  setHorizontalCoordinate(HORIZONTAL_GROUND);
  setStableMode(STABLE_ENABLE);
  setControlCommand({0,0,0,0});
}

FlightJoystick::~FlightJoystick() { delete (flightLink); }

void FlightJoystick::setHorizontalLogic( HorizontalLogic horizontalLogic) {
  ctrlData.controlMode.horizMode = horizontalLogic;
}

void FlightJoystick::setVerticalLogic( VerticalLogic verticalLogic) {
  ctrlData.controlMode.vertiMode = verticalLogic;
}

void FlightJoystick::setYawLogic(YawLogic yawLogic) {
  ctrlData.controlMode.yawMode = yawLogic;
}

void FlightJoystick::setHorizontalCoordinate(HorizontalCoordinate horizontalCoordinate) {
  ctrlData.controlMode.horizFrame = horizontalCoordinate;
}

void FlightJoystick::setStableMode(StableMode stableMode) {
  ctrlData.controlMode.stableMode = stableMode;
}

void FlightJoystick::setControlCommand(const ControlCommand &controlCommand) {
  ctrlData.controlCommand = controlCommand;
}

void FlightJoystick::getControlCommand(ControlCommand &controlCommand) {
  controlCommand = ctrlData.controlCommand;
}
void FlightJoystick::getControlMode(ControlMode &controlMode) {
  controlMode = ctrlData.controlMode;
}

void FlightJoystick::joystickAction() {
 if(flightLink)
  flightLink->sendDirectly(OpenProtocolCMD::CMDSet::Control::control,
                           (void *)(&this->ctrlData), sizeof(CtrlData));
 else
   DERROR(" flight Link is NULL");

}
