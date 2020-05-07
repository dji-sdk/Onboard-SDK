/** @file dji_payload_link.cpp
 *  @version 3.9
 *  @date July 2019
 *
 *  @brief Implementation of payload node
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

#include "dji_payload_link.hpp"
#include <dji_vehicle.hpp>

using namespace DJI;
using namespace DJI::OSDK;

PayloadLink::PayloadLink(Vehicle *vehicle) : vehicle(vehicle) {}

PayloadLink::~PayloadLink() {}

void PayloadLink::sendAsync(const uint8_t cmd[], void *pdata, size_t len,
                            void *callBack, UserData userData, int timeout,
                            int retry_time) {
  vehicle->legacyLinker->sendAsync(cmd, (uint8_t *) pdata, len, timeout,
                                   retry_time, (VehicleCallBack) callBack,
                                   userData);
}

ACK::ExtendedFunctionRsp *PayloadLink::sendSync(const uint8_t cmd[],
                                                void *pdata, size_t len,
                                                int timeout) {
  return (ACK::ExtendedFunctionRsp *) vehicle->legacyLinker->sendSync(
      cmd, (uint8_t *)pdata, len, timeout * 1000 / 2, 2);
}

void PayloadLink::sendToPSDK(uint8_t *data, uint16_t len) {
  if (!vehicle->getActivationStatus()) {
    DERROR("The drone has not been activated");
    return;
  }
  vehicle->legacyLinker->send(OpenProtocolCMD::CMDSet::Activation::toPayload,
                              data, len);
}
