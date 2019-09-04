/** @file dji_flight_link.cpp
 *  @version 3.9
 *  @date July 2019
 *
 *  @brief Implementation of flight link
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

#include "dji_flight_link.hpp"
#include <dji_vehicle.hpp>

using namespace DJI;
using namespace DJI::OSDK;

FlightLink::FlightLink(Vehicle *vehicle) : vehicle(vehicle) {}

FlightLink::~FlightLink() {}

void FlightLink::sendAsync(const uint8_t cmd[], void *pdata, size_t len,
                            void *callBack, UserData userData, int timeout,
                            int retryTime) {
  int cbIndex = setCallback(callBack, userData);
  vehicle->protocolLayer->send(2, vehicle->getEncryption(), cmd,
                               (uint8_t *)pdata, len, timeout, retryTime, true,
                               cbIndex);
}

void *FlightLink::sendSync(const uint8_t cmd[], void *pdata, size_t len,
                            int timeout) {
  vehicle->protocolLayer->send(2, vehicle->getEncryption(), cmd,
                               (uint8_t *)pdata, len, 500, 2, false, 2);
  return vehicle->waitForACK({cmd[0],cmd[1]}, timeout);
}

/*! @TODO The callback and userdata recording work should be implemented in
 * the protocol layer. It will be improved in OSDK 4.0. So here is the
 * temporary way.
 */
int FlightLink::setCallback(void *callBack, UserData userData) {
  int cbIndex = vehicle->callbackIdIndex();
  vehicle->nbCallbackFunctions[cbIndex] = (void *)callBack;
  vehicle->nbUserData[cbIndex] = userData;
  return cbIndex;
}
