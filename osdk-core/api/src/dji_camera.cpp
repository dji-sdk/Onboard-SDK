/** @file dji_camera.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Camera/Gimbal API for DJI onboardSDK library
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
