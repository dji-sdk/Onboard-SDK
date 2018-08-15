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

  this->fromMSDKHandler.callback = getDataFromMSDKCallback;
  this->fromMSDKHandler.userData = 0;
}

MobileDevice::~MobileDevice()
{
  this->fromMSDKHandler.callback = 0;
  this->fromMSDKHandler.userData = 0;
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

  vehicle->protocolLayer->send(0, vehicle->getEncryption(),
                               OpenProtocolCMD::CMDSet::Activation::toMobile,
                               data, len, 500, 1, NULL, 0);
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
}
