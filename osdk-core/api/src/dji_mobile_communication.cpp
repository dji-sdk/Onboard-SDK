/** @file dji_mobile_communication.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Implementation of DJI Mobile-Onboard SDK Communication (MOC)
 *
 *  @Copyright (c) 2016-2017 DJI
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

#include "dji_mobile_communication.hpp"
#include "dji_vehicle.hpp"

using namespace DJI;
using namespace DJI::OSDK;

MobileCommunication::MobileCommunication(Vehicle* vehicle)
  : vehicle(vehicle)
{
  this->fromMSDKHandler.callback = getDataFromMSDKCallback;
  this->fromMSDKHandler.userData = 0;
}

MobileCommunication::~MobileCommunication()
{
  this->fromMSDKHandler.callback = 0;
  this->fromMSDKHandler.userData = 0;
}

Vehicle*
MobileCommunication::getVehicle() const
{
  return vehicle;
}

void
MobileCommunication::setVehicle(Vehicle* value)
{
  vehicle = value;
}

void
MobileCommunication::sendDataToMSDK(uint8_t* data, uint8_t len)
{
  if (len > 100)
  {
    DERROR("Too much data to send");
    return;
  }
  vehicle->protocolLayer->send(0, vehicle->getEncryption(),
                               OpenProtocolCMD::CMDSet::Activation::toMobile,
                               data, len, 500, 1, NULL, 0);
}

void
MobileCommunication::getDataFromMSDKCallback(Vehicle*      vehiclePtr,
                                             RecvContainer recvFrame,
                                             UserData      userData)
{
  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <= 100)
  {
    DSTATUS("The len of received mobile communication data: %d\n", recvFrame.recvInfo.len);
  }
}

void
MobileCommunication::setFromMSDKCallback(VehicleCallBack callback,
                                         UserData        userData)
{
  this->fromMSDKHandler.callback = callback;
  this->fromMSDKHandler.userData = userData;
}
