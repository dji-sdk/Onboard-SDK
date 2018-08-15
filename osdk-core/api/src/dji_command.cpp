/** @file dji_command.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief All DJI OSDK OpenProtocol Command IDs
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

#include "dji_command.hpp"

const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Activation::getVersion[] = {
  OpenProtocolCMD::CMDSet::activation, 0x00
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Activation::activate[] = {
  OpenProtocolCMD::CMDSet::activation, 0x01
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Activation::frequency[] = {
  OpenProtocolCMD::CMDSet::activation, 0x10
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Activation::toMobile[] = {
  OpenProtocolCMD::CMDSet::activation, 0xFE
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Broadcast::broadcast[] = {
  OpenProtocolCMD::CMDSet::broadcast, 0x00
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Broadcast::lostCTRL[] = {
  OpenProtocolCMD::CMDSet::broadcast, 0x01
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Broadcast::fromMobile[] = {
  OpenProtocolCMD::CMDSet::broadcast, 0x02
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Broadcast::mission[] = {
  OpenProtocolCMD::CMDSet::broadcast, 0x03
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Broadcast::waypoint[] = {
  OpenProtocolCMD::CMDSet::broadcast, 0x04
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Broadcast::subscribe[] = {
  OpenProtocolCMD::CMDSet::broadcast, 0x05
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Broadcast::test[] = {
  OpenProtocolCMD::CMDSet::broadcast, 0xEF
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Control::setControl[] = {
  OpenProtocolCMD::CMDSet::control, 0x00
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Control::task[] = {
  OpenProtocolCMD::CMDSet::control, 0x01
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Control::status[] = {
  OpenProtocolCMD::CMDSet::control, 0x02
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Control::control[] = {
  OpenProtocolCMD::CMDSet::control, 0x03
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Control::setArm[] = {
  OpenProtocolCMD::CMDSet::control, 0x05
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Control::killSwitch[] = {
  OpenProtocolCMD::CMDSet::control, 0x06
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Control::cameraShot[] = {
  OpenProtocolCMD::CMDSet::control, 0x20
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Control::cameraVideoStart[] =
  { OpenProtocolCMD::CMDSet::control, 0x21 };
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Control::cameraVideoStop[] = {
  OpenProtocolCMD::CMDSet::control, 0x22
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Control::gimbalSpeed[] = {
  OpenProtocolCMD::CMDSet::control, 0x1A
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Control::gimbalAngle[] = {
  OpenProtocolCMD::CMDSet::control, 0x1B
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::waypointInit[] = {
  OpenProtocolCMD::CMDSet::mission, 0x10
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::waypointAddPoint[] =
  { OpenProtocolCMD::CMDSet::mission, 0x11 };
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::waypointSetStart[] =
  { OpenProtocolCMD::CMDSet::mission, 0x12 };
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::waypointSetPause[] =
  { OpenProtocolCMD::CMDSet::mission, 0x13 };
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::waypointDownload[] =
  { OpenProtocolCMD::CMDSet::mission, 0x14 };
const uint8_t
  DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::waypointIndexDownload[] = {
    OpenProtocolCMD::CMDSet::mission, 0x15
  };
const uint8_t
  DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::waypointSetVelocity[] = {
    OpenProtocolCMD::CMDSet::mission, 0x16
  };
const uint8_t
  DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::waypointGetVelocity[] = {
    OpenProtocolCMD::CMDSet::mission, 0x17
  };
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::hotpointStart[] = {
  OpenProtocolCMD::CMDSet::mission, 0x20
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::hotpointStop[] = {
  OpenProtocolCMD::CMDSet::mission, 0x21
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::hotpointSetPause[] =
  { OpenProtocolCMD::CMDSet::mission, 0x22 };
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::hotpointYawRate[] = {
  OpenProtocolCMD::CMDSet::mission, 0x23
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::hotpointRadius[] = {
  OpenProtocolCMD::CMDSet::mission, 0x24
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::hotpointSetYaw[] = {
  OpenProtocolCMD::CMDSet::mission, 0x25
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::hotpointDownload[] =
  { OpenProtocolCMD::CMDSet::mission, 0x26 };
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::followStart[] = {
  OpenProtocolCMD::CMDSet::mission, 0x30
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::followStop[] = {
  OpenProtocolCMD::CMDSet::mission, 0x31
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::followSetPause[] = {
  OpenProtocolCMD::CMDSet::mission, 0x32
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Mission::followTarget[] = {
  OpenProtocolCMD::CMDSet::mission, 0x33
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::HardwareSync::broadcast[] = {
  OpenProtocolCMD::CMDSet::hardwareSync, 0x00
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::VirtualRC::settings[] = {
  OpenProtocolCMD::CMDSet::virtualRC
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::VirtualRC::data[] = {
  OpenProtocolCMD::CMDSet::virtualRC
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::MFIO::init[] = {
  OpenProtocolCMD::CMDSet::mfio, 0x02
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::MFIO::set[] = {
  OpenProtocolCMD::CMDSet::mfio, 0x03
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::MFIO::get[] = {
  OpenProtocolCMD::CMDSet::mfio, 0x04
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Subscribe::versionMatch[] = {
  OpenProtocolCMD::CMDSet::subscribe, 0x00
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Subscribe::addPackage[] = {
  OpenProtocolCMD::CMDSet::subscribe, 0x01
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Subscribe::reset[] = {
  OpenProtocolCMD::CMDSet::subscribe, 0x02
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Subscribe::removePackage[] = {
  OpenProtocolCMD::CMDSet::subscribe, 0x03
};
const uint8_t
  DJI::OSDK::OpenProtocolCMD::CMDSet::Subscribe::updatePackageFreq[] = {
    OpenProtocolCMD::CMDSet::subscribe, 0x04
  };
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Subscribe::pauseResume[] = {
  OpenProtocolCMD::CMDSet::subscribe, 0x05
};
const uint8_t DJI::OSDK::OpenProtocolCMD::CMDSet::Subscribe::getConfig[] = {
  OpenProtocolCMD::CMDSet::subscribe, 0x06
};
