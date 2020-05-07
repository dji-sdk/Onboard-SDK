/** @file dji_mobile_device.cpp
 *  @version 3.7
 *  @date July 2018
 *
 *  @brief Implementation of DJI Mobile Device Abstraction
 *
 *  @copyright 2017-18 DJI. All rights reserved.
 *
 */

#include "dji_mobile_device.hpp"
#include "dji_vehicle.hpp"

MobileDevice::MobileDevice(Vehicle* vehicle)
  : vehicle(vehicle)
{
  setFromMSDKCallback(getDataFromMSDKCallback, NULL);
}

MobileDevice::~MobileDevice()
{
  setFromMSDKCallback(NULL, NULL);
}

Vehicle*
MobileDevice::getVehicle() const
{
  return vehicle;
}

void
MobileDevice::setVehicle(Vehicle* value)
{
  vehicle = value;
}

void
MobileDevice::sendDataToMSDK(uint8_t* data, uint8_t len)
{
  if (len > 100)
  {
    DERROR("Too much data to send");
    return;
  }
  if (!vehicle->getActivationStatus())
  {
    DERROR("The drone has not been activated");
    return;
  }
  vehicle->legacyLinker->send(OpenProtocolCMD::CMDSet::Activation::toMobile,
                              data, len);
}

void
MobileDevice::getDataFromMSDKCallback(Vehicle*      vehiclePtr,
                                      RecvContainer recvFrame,
                                      UserData      userData)
{
  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <= 100)
  {
    DSTATUS("The len of received mobile device data: %d\n", recvFrame.recvInfo.len);
  }
}

void
MobileDevice::setFromMSDKCallback(VehicleCallBack callback, UserData userData)
{
  this->fromMSDKHandler.callback = callback;
  this->fromMSDKHandler.userData = userData;
  vehicle->legacyLinker->registerCMDCallback(
      OpenProtocolCMD::CMDSet::Broadcast::fromMobile[0],
      OpenProtocolCMD::CMDSet::Broadcast::fromMobile[1],
      fromMSDKHandler.callback, fromMSDKHandler.userData);
}
