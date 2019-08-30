/*! @file flight-controller/main.cpp
 *  @version 3.9
 *  @date August 05 2019
 *
 *  @brief
 *  main for flight actions and flight assistant usage in a Linux environment.
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
/*! use some flight_control_sample api */
#include "../flight-control/flight_control_sample.hpp"

#include <dji_linux_helpers.hpp>
#include <dji_vehicle.hpp>
#include "flight_controller_sample.hpp"
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int main(int argc, char **argv) {
  int functionTimeout = 1;
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle *vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL) {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }
  vehicle->obtainCtrlAuthority(functionTimeout);

  /*! Open rtk switch */
  openRtkSwtich(vehicle);

  /*! Open avoid obstacle switch */
  openAvoidObstacle(vehicle, 1);

  /*!  Take off */
  monitoredTakeoff(vehicle);

  /*! Move to higher altitude */
  moveByPositionOffset(vehicle, 0, 0, 30, 0);

  /*! Move a short distance*/
  moveByPositionOffset(vehicle, 10, 0, 0, -30);

  /*! Set aircraft current position as new home point */
  setNewHomePoint(vehicle);

  /*! Set new go home altitude */
  setGoHomeAltitude(vehicle, 50);

  /*! Move to another position */
  moveByPositionOffset(vehicle, 40, 0, 0, 0);

  /*! Close avoid obstacle switch */
  closeAvoidObstacle(vehicle, 1);

  /*! go home and force landing avoid ground */
  goHomeAndForceLanding(vehicle, 1);
}
