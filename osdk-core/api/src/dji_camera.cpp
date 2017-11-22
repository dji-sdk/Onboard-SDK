/** @file dji_camera.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Camera/Gimbal API for DJI onboardSDK library
 *
 *  @copyright 2016-17 DJI. All rights reserved.
 *
 */

#include "dji_camera.hpp"
#include "dji_vehicle.hpp"

using namespace DJI;
using namespace DJI::OSDK;

Camera::Camera(Vehicle* vehicle)
  : vehicle(vehicle)
{
}

Camera::~Camera()
{
}

void
Camera::shootPhoto()
{
  action(OpenProtocolCMD::CMDSet::Control::cameraShot);
}

void
Camera::videoStart()
{
  action(OpenProtocolCMD::CMDSet::Control::cameraVideoStart);
}

void
Camera::videoStop()
{
  action(OpenProtocolCMD::CMDSet::Control::cameraVideoStop);
}

void
Camera::action(const uint8_t cmd[])
{
  uint8_t sendData = 0;
  vehicle->protocolLayer->send(0, vehicle->getEncryption(), cmd, &sendData, 1);
}
