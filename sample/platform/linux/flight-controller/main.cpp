/*! @file flight-control-4.0/main.cpp
 *  @version 3.9
 *  @date JULY 24 2019
 *
 *  @brief
 *  main for Flight Control API usage in a Linux environment.
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
#include <dji_linux_helpers.hpp>
#include <dji_vehicle.hpp>
#include "dji_flight_assistant.hpp"
#include "flight_controller_sample.hpp"
#include "../flight-control/flight_control_sample.hpp"
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int main(int argc, char **argv) {
  // Initialize variables
  int functionTimeout = 1;
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle *vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL) {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }
  vehicle->obtainCtrlAuthority(functionTimeout);

  /*Set rtk enable*/
  vehicle->flightAssistant->setRtkEnableSync(
      FlightAssistant::rtkEnableData::RTK_DISABLE, 1);

  /*Take off*/
  monitoredTakeoff(vehicle);

  /*Open avoid obstacle switch*/
  // openAvoidObstacle(vehicle, 1);

  /*Move to high altitude*/
  moveByPositionOffset(vehicle, 0, 0, 30, 0);

  /*Move to anther position*/
  moveByPositionOffset(vehicle, 0, 10, 0, -30);

  /*Set aircraft current position as home point */
  setNewHomePoint(vehicle);

  /*Set go home altitude*/
  setGoHomeAltitude(vehicle, 50);

  /*Move to anther position*/
  moveByPositionOffset(vehicle, 0, 40, 0, -30);

  /*Close avoid obstacle switch*/
  // closeAvoidObstacle(vehicle, 1);

  /*go home*/
  goHomeAndForceLanding(vehicle, 1);

  /*force landing*/
}
