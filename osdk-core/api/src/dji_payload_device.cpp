/** @file dji_payload_device.cpp
 *  @version 3.8.1
 *  @date  May 2019
 *
 *  @brief Implementation of DJI Payload-3-thirty Device Abstraction
 *
 *  @copyright 2017-19 DJI. All rights reserved.
 *
 */

#include "dji_payload_device.hpp"
#include "dji_vehicle.hpp"

PayloadDevice::PayloadDevice(Vehicle *vehicle)
    : vehicle(vehicle)
{
  setFromPSDKCallback(getDataFromPSDKCallback, NULL);
}
PayloadDevice::~PayloadDevice()
{
  setFromPSDKCallback(NULL, NULL);
}
Vehicle* PayloadDevice::getVehicle() const
{
  return vehicle;
}
void PayloadDevice::setVehicle(Vehicle *value)
{
  vehicle = value;
}

void PayloadDevice::sendDataToPSDK(uint8_t *data, uint16_t len)
{
  if (len > MAX_SIZE_OF_PACKAGE)
  {
    DERROR("Too much data to send");
    return;
  }
  if (!vehicle->getActivationStatus())
  {
    DERROR("The drone has not been activated");
    return;
  }
  vehicle->legacyLinker->send(OpenProtocolCMD::CMDSet::Activation::toPayload,
                              data, len);
}
void PayloadDevice::getDataFromPSDKCallback(Vehicle *vehiclePtr,
                                            RecvContainer recvFrame,
                                            UserData userData)
{

  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <= 100)
  {
    DDEBUG("The len of received payload device data: %d\n", recvFrame.recvInfo.len);
  }
}
void PayloadDevice::setFromPSDKCallback(VehicleCallBack callback, UserData userData)
{
  this->fromPSDKHandler.callback = callback;
  this->fromPSDKHandler.userData =  userData;
  vehicle->legacyLinker->registerCMDCallback(
      OpenProtocolCMD::CMDSet::Broadcast::fromPayload[0],
      OpenProtocolCMD::CMDSet::Broadcast::fromPayload[1],
      fromPSDKHandler.callback, fromPSDKHandler.userData);
}

