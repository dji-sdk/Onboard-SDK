/** @file dji_control.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Control API for DJI OSDK library
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

#include "dji_control.hpp"
#include "dji_vehicle.hpp"

using namespace DJI;
using namespace DJI::OSDK;

Control::Control(Vehicle* vehicle)
  : vehicle(vehicle)
  , wait_timeout(10)
{
}

Control::~Control()
{
}

void
Control::action(const int cmd, VehicleCallBack callback, UserData userData)
{
  int cbIndex = vehicle->callbackIdIndex();
  if (callback)
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
    vehicle->nbUserData[cbIndex]          = userData;
  }
  else
  {
    // Support for default callbacks
    vehicle->nbCallbackFunctions[cbIndex] = (void*)actionCallback;
    vehicle->nbUserData[cbIndex]          = NULL;
  }

  // Check which version of firmware we are dealing with
  if (vehicle->isLegacyM600())
  {
    legacyCMDData.cmd = cmd;
    legacyCMDData.sequence++;
    vehicle->protocolLayer->send(
        2, vehicle->getEncryption(), OpenProtocolCMD::CMDSet::Control::task,
        (uint8_t*)&legacyCMDData, sizeof(legacyCMDData), 500, 2, true, cbIndex);
  }
  else if (vehicle->isM100())
  {
    legacyCMDData.cmd = cmd;
    legacyCMDData.sequence++;
    vehicle->protocolLayer->send(
        2,vehicle->getEncryption(), OpenProtocolCMD::CMDSet::Control::task,
        (uint8_t*)&legacyCMDData, sizeof(legacyCMDData), 500, 2, true, cbIndex);
  }
  else
  {
    uint8_t data = cmd;
    vehicle->protocolLayer->send(2, vehicle->getEncryption(),
                                 OpenProtocolCMD::CMDSet::Control::task, &data,
                                 sizeof(data), 500, 2, true, cbIndex);
  }
}

ACK::ErrorCode
Control::action(const int cmd, int timeout)
{
  ACK::ErrorCode ack;

  if (vehicle->isLegacyM600())
  {
    legacyCMDData.cmd = cmd;
    legacyCMDData.sequence++;
    vehicle->protocolLayer->send(
        2,vehicle->getEncryption(), OpenProtocolCMD::CMDSet::Control::task,
        (uint8_t*)&legacyCMDData, sizeof(legacyCMDData), 500, 2, false, 2);
  }
  else if (vehicle->isM100())
  {
    legacyCMDData.cmd = cmd;
    legacyCMDData.sequence++;
    vehicle->protocolLayer->send(
        2, vehicle->getEncryption(), OpenProtocolCMD::CMDSet::Control::task,
        (uint8_t*)&legacyCMDData, sizeof(legacyCMDData), 100, 3, false, 2);
  }
  else
  {
    uint8_t data = cmd;
    vehicle->protocolLayer->send(2, vehicle->getEncryption(),
                                 OpenProtocolCMD::CMDSet::Control::task, &data,
                                 sizeof(data), 500, 2, false, 2);
  }

  ack = *((ACK::ErrorCode*)vehicle->waitForACK(
    OpenProtocolCMD::CMDSet::Control::task, timeout));

  return ack;
}

void
Control::setArm(bool armSetting, VehicleCallBack callback, UserData userData)
{
  uint8_t data    = armSetting ? 1 : 0;
  int     cbIndex = vehicle->callbackIdIndex();
  if (callback)
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
    vehicle->nbUserData[cbIndex]          = userData;
  }
  else
  {
    // Support for default callbacks
    vehicle->nbCallbackFunctions[cbIndex] = (void*)actionCallback;
    vehicle->nbUserData[cbIndex]          = NULL;
  }

  vehicle->protocolLayer->send(2, vehicle->getEncryption(),
                               OpenProtocolCMD::CMDSet::Control::setArm, &data,
                               sizeof(data), 10, 10, true, cbIndex);
}

ACK::ErrorCode
Control::setArm(bool armSetting, int timeout)
{
  ACK::ErrorCode ack;
  uint8_t        data = armSetting ? 1 : 0;

  vehicle->protocolLayer->send(2, vehicle->getEncryption(),
                               OpenProtocolCMD::CMDSet::Control::setArm, &data,
                               sizeof(data), 10, 10, false, 2);

  ack = *((ACK::ErrorCode*)vehicle->waitForACK(
    OpenProtocolCMD::CMDSet::Control::setArm, timeout));

  return ack;
}

ACK::ErrorCode
Control::armMotors(int wait_timeout)
{
  if (vehicle->isLegacyM600())
  {
    return this->setArm(true, wait_timeout);
  }
  else if (vehicle->isM100())
  {
    return this->setArm(true, wait_timeout);
  }
  else
  {
    return this->action(FlightCommand::startMotor, wait_timeout);
  }
}

void
Control::armMotors(VehicleCallBack callback, UserData userData)
{
  if (vehicle->isLegacyM600())
  {
    this->setArm(true, callback, userData);
  }
  else if (vehicle->isM100())
  {
    this->setArm(true, callback, userData);
  }
  else
  {
    this->action(FlightCommand::startMotor, callback, userData);
  }
}

ACK::ErrorCode
Control::disArmMotors(int wait_timeout)
{
  if (vehicle->isLegacyM600())
  {
    return this->setArm(false, wait_timeout);
  }
  else if (vehicle->isM100())
  {
    return this->setArm(false, wait_timeout);
  }
  else
  {
    return this->action(FlightCommand::stopMotor, wait_timeout);
  }
}

void
Control::disArmMotors(VehicleCallBack callback, UserData userData)
{
  if (vehicle->isLegacyM600())
  {
    this->setArm(false, callback, userData);
  }
  else if (vehicle->isM100())
  {
    this->setArm(false, callback, userData);
  }
  else
  {
    this->action(FlightCommand::stopMotor, callback, userData);
  }
}

ACK::ErrorCode
Control::takeoff(int wait_timeout)
{
  if (vehicle->isLegacyM600())
  {
    return this->action(FlightCommand::LegacyCMD::takeOff, wait_timeout);
  }
  else if (vehicle->isM100())
  {
    return this->action(FlightCommand::LegacyCMD::takeOff, wait_timeout);
  }
  else
  {
    return this->action(FlightCommand::takeOff, wait_timeout);
  }
}

void
Control::takeoff(VehicleCallBack callback, UserData userData)
{
  if (vehicle->isLegacyM600())
  {
    this->action(FlightCommand::LegacyCMD::takeOff, callback, userData);
  }
  else if (vehicle->isM100())
  {
    this->action(FlightCommand::LegacyCMD::takeOff, callback, userData);
  }
  else
  {
    this->action(FlightCommand::takeOff, callback, userData);
  }
}

ACK::ErrorCode
Control::goHome(int wait_timeout)
{
  if (vehicle->isLegacyM600())
  {
    return this->action(FlightCommand::LegacyCMD::goHome, wait_timeout);
  }
  else if (vehicle->isM100())
  {
    return this->action(FlightCommand::LegacyCMD::goHome, wait_timeout);
  }
  else
  {
    return this->action(FlightCommand::goHome, wait_timeout);
  }
}

void
Control::goHome(VehicleCallBack callback, UserData userData)
{
  if (vehicle->isLegacyM600())
  {
    this->action(FlightCommand::LegacyCMD::goHome, callback, userData);
  }
  else if (vehicle->isM100())
  {
    this->action(FlightCommand::LegacyCMD::goHome, callback, userData);
  }
  else
  {
    this->action(FlightCommand::goHome, callback, userData);
  }
}

ACK::ErrorCode
Control::land(int wait_timeout)
{
  if (vehicle->isLegacyM600())
  {
    return this->action(FlightCommand::LegacyCMD::landing, wait_timeout);
  }
  else if (vehicle->isM100())
  {
    return this->action(FlightCommand::LegacyCMD::landing, wait_timeout);
  }
  else
  {
    return this->action(FlightCommand::landing, wait_timeout);
  }
}

void
Control::land(VehicleCallBack callback, UserData userData)
{
  if (vehicle->isLegacyM600())
  {
    this->action(FlightCommand::LegacyCMD::landing, callback, userData);
  }
  else if (vehicle->isM100())
  {
    this->action(FlightCommand::LegacyCMD::landing, callback, userData);
  }
  else
  {
    this->action(FlightCommand::landing, callback, userData);
  }
}

void
Control::flightCtrl(CtrlData data)
{
  vehicle->protocolLayer->send(
    0, vehicle->getEncryption(), OpenProtocolCMD::CMDSet::Control::control,
    static_cast<void*>(&data), sizeof(CtrlData), 500, 2, false, 1);
}

void
Control::flightCtrl(AdvancedCtrlData data)
{
  if (vehicle->getFwVersion() > extendedVersionBase)
  {
    vehicle->protocolLayer->send(
      0, vehicle->getEncryption(), OpenProtocolCMD::CMDSet::Control::control,
      static_cast<void*>(&data), sizeof(AdvancedCtrlData), 500, 2, false, 1);
  }
  else
  {
    if (strcmp(vehicle->getHwVersion(), "M100") == 0)
    {
      DERROR("Advanced flight control not supported on Matrice 100!\n");
    }
    else
    {
      DERROR("This advanced Flight Control feature is only supported on newer version of firmware.\n");
    }
  }
}

void
Control::positionAndYawCtrl(float32_t x, float32_t y, float32_t z,
                            float32_t yaw)
{
  //! @note 145 is the flag value of this mode
  uint8_t ctrl_flag = (VERTICAL_POSITION | HORIZONTAL_POSITION | YAW_ANGLE |
                       HORIZONTAL_GROUND | STABLE_ENABLE);
  CtrlData data(ctrl_flag, x, y, z, yaw);

  this->flightCtrl(data);
}

void
Control::velocityAndYawRateCtrl(float32_t Vx, float32_t Vy, float32_t Vz,
                                float32_t yawRate)
{
  //! @note 72 is the flag value of this mode
  uint8_t ctrl_flag =
    (VERTICAL_VELOCITY | HORIZONTAL_VELOCITY | YAW_RATE | HORIZONTAL_GROUND);
  CtrlData data(ctrl_flag, Vx, Vy, Vz, yawRate);

  this->flightCtrl(data);
}

void
Control::attitudeAndVertPosCtrl(float32_t roll, float32_t pitch, float32_t yaw,
                                float32_t z)
{
  //! @note 18 is the flag value of this mode
  uint8_t ctrl_flag =
    (VERTICAL_POSITION | HORIZONTAL_ANGLE | YAW_ANGLE | HORIZONTAL_BODY);
  CtrlData data(ctrl_flag, roll, pitch, z, yaw);

  this->flightCtrl(data);
}

void
Control::angularRateAndVertPosCtrl(float32_t rollRate, float32_t pitchRate,
                                   float32_t yawRate, float32_t z)
{
  if (vehicle->getFwVersion() > extendedVersionBase)
  {
    //! @note 218 is the flag value of this mode
    uint8_t ctrl_flag = (VERTICAL_POSITION | HORIZONTAL_ANGULAR_RATE |
                         YAW_RATE | HORIZONTAL_BODY);
    CtrlData data(ctrl_flag, rollRate, pitchRate, z, yawRate);

    this->flightCtrl(data);
  }
  else
  {
    if (strcmp(vehicle->getHwVersion(), "M100") == 0)
    {
      DERROR("angularRateAndVertPosCtrl not supported on Matrice 100!\n");
    }
    else
    {
      DERROR("angularRateAndVertPosCtrl is only supported on newer firmware.\n");
    }
  }
}

void
Control::emergencyBrake()
{
  if (vehicle->getFwVersion() > extendedVersionBase)
  {
    //! @note 75 is the flag value of this mode
    AdvancedCtrlData data(72, 0, 0, 0, 0, 0, 0);

    this->flightCtrl(data);
  }
  else
  {
    if (strcmp(vehicle->getHwVersion(), "M100") == 0)
    {
      DERROR("emergencyBrake not supported on Matrice 100!\n");
    }
    else
    {
      DERROR(
        "emergencyBrake is only supported on newer firmware for your product.\n");
    }
  }
}

void
Control::actionCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                        UserData userData)
{
  ACK::ErrorCode ack;
  Control*       controlPtr = vehiclePtr->control;

  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <= sizeof(uint16_t))
  {

    ack.info = recvFrame.recvInfo;
    ack.data = recvFrame.recvData.commandACK;

    if (ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, __func__);
    }
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
  }
}
Control::CtrlData::CtrlData(uint8_t in_flag, float32_t in_x, float32_t in_y,
                            float32_t in_z, float32_t in_yaw)
  : flag(in_flag)
  , x(in_x)
  , y(in_y)
  , z(in_z)
  , yaw(in_yaw)
{
}

Control::AdvancedCtrlData::AdvancedCtrlData(uint8_t in_flag, float32_t in_x,
                                            float32_t in_y, float32_t in_z,
                                            float32_t in_yaw, float32_t x_forw,
                                            float32_t y_forw)
  : flag(in_flag)
  , x(in_x)
  , y(in_y)
  , z(in_z)
  , yaw(in_yaw)
  , xFeedforward(x_forw)
  , yFeedforward(y_forw)
  , advFlag(0x01)
{
}
