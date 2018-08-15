/*! @file mobile_sample.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Mobile SDK Communication API usage in a Linux environment.
 *  Shows example usage of the mobile<-->onboard SDK communication API.
 *
 *  @Copyright (c) 2017 DJI
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

#include "mobile_sample.hpp"

#define TEST_MOC 1
#define TEST_MOBILE_DEVICE 2
#define TEST_BOTH_MOC_MD 3

using namespace DJI;
using namespace DJI::OSDK;

// GLOBAL: mobileDataID for keeping track of command from mobile
uint16_t mobileDataID_glob = 0;
// GLOBAL: keepLoopRunning to maintain state of when to stop checking mobile
// command state
bool keepLoopRunning = true;

// GLOBAL: In 3.7 release, to compatible with users existing code, we have moc
// and mobielDevice both object exists in vehicle class.
// 1 means testing moc, 2 means testing mobileDevice, and 3 means testing both
// of them.
int test_moc_mobileDevice = 0;

void
setTestSuite(int testSuiteNumber)
{
  test_moc_mobileDevice = testSuiteNumber;
}

void
parseFromMobileCallback(Vehicle* vehicle, RecvContainer recvFrame,
                        UserData userData)
{
  // First, lets cast the userData to LinuxSetup*
  LinuxSetup* linuxEnvironment = (LinuxSetup*)userData;
  uint16_t    mobile_data_id;
  mobile_data_id =
    *(reinterpret_cast<uint16_t*>(&recvFrame.recvData.raw_ack_array));

  // Now, let's set up some variables that don't cross case initializations
  bool    coreMissionStatus;
  uint8_t wayptPolygonSides;
  int     hotptInitRadius;
  int     responseTimeout = 1;

  switch (mobile_data_id)
  {
    case 1:
      sendDroneVersionFromCache(vehicle);
      break;
    case 2:
      vehicle->obtainCtrlAuthority(controlAuthorityMobileCallback);
      break;
    case 3:
      vehicle->releaseCtrlAuthority(controlAuthorityMobileCallback);
      break;
    case 4:
      vehicle->activate(linuxEnvironment->getActivateData(),
                        activateMobileCallback);
      break;
    case 5:
      vehicle->control->armMotors(actionMobileCallback);
      break;
    case 6:
      vehicle->control->disArmMotors(actionMobileCallback);
      break;
    case 7:
      vehicle->control->takeoff(actionMobileCallback);
      break;
    case 8:
      vehicle->control->land(actionMobileCallback);
      break;
    case 9:
      vehicle->control->goHome(actionMobileCallback);
      break;
    case 10:
      vehicle->camera->shootPhoto();
      sendAckToMobile(vehicle, mobile_data_id);
      break;
    case 11:
      vehicle->camera->videoStart();
      sendAckToMobile(vehicle, mobile_data_id);
      break;
    case 12:
      vehicle->camera->videoStop();
      sendAckToMobile(vehicle, mobile_data_id);
      break;
    case 0x3E:
      //! Since we have a long function call involving blocking API calls here,
      //! we'll be setting a global and servicing it in a different thread.
      mobileDataID_glob = mobile_data_id;
      break;
    case 0x40:
      //! Since we have a long function call involving blocking API calls here,
      //! we'll be setting a global and servicing it in a different thread.
      mobileDataID_glob = mobile_data_id;
      break;
    case 0x41:
      //! Since we have a long function call involving blocking API calls here,
      //! we'll be setting a global and servicing it in a different thread.
      mobileDataID_glob = mobile_data_id;
      break;
    case 0x42:
      //! Since we have a long function call involving blocking API calls here,
      //! we'll be setting a global and servicing it in a different thread.
      mobileDataID_glob = mobile_data_id;
      break;
    default:
      break;
  }
}

/**
 * Testing purpose method to just receive the callback data, which will not take
 * any action
 *
 */
void
parseFromMobileCallback2(Vehicle* vehicle, RecvContainer recvFrame,
                         UserData userData)
{
  // First, lets cast the userData to LinuxSetup*
  LinuxSetup* linuxEnvironment = (LinuxSetup*)userData;
  uint16_t    mobile_data_id;
  mobile_data_id =
    *(reinterpret_cast<uint16_t*>(&recvFrame.recvData.raw_ack_array));

  std::cout << "Received the mobile data ID is " << mobile_data_id << std::endl;
}
/**
 *  Test helper method on the tested method.
 * */
void
setFromMSDKCallback(Vehicle* vehicle, LinuxSetup* linuxEnvironment)
{
  if (test_moc_mobileDevice == TEST_MOC)
  {
    vehicle->moc->setFromMSDKCallback(parseFromMobileCallback,
                                      linuxEnvironment);
  }
  else if (test_moc_mobileDevice == TEST_MOBILE_DEVICE)
  {
    vehicle->mobileDevice->setFromMSDKCallback(parseFromMobileCallback,
                                               linuxEnvironment);
  }
  else if (test_moc_mobileDevice == TEST_BOTH_MOC_MD)
  {
    vehicle->mobileDevice->setFromMSDKCallback(parseFromMobileCallback,
                                               linuxEnvironment);
    vehicle->moc->setFromMSDKCallback(parseFromMobileCallback2,
                                      linuxEnvironment);
  }
}

void
sendDataToMSDK(Vehicle* vehicle, uint8_t* data, uint8_t len)
{
  if (test_moc_mobileDevice == TEST_MOC)
  {
    vehicle->moc->sendDataToMSDK(data, len);
  }
  else if (test_moc_mobileDevice == TEST_MOBILE_DEVICE)
  {
    vehicle->mobileDevice->sendDataToMSDK(data, len);
  }
  else if (test_moc_mobileDevice == TEST_BOTH_MOC_MD)
  {
    vehicle->mobileDevice->sendDataToMSDK(data, len);
    sleep(0.5);
    vehicle->moc->sendDataToMSDK(data, len);
  }
}

bool
setupMSDKParsing(Vehicle* vehicle, LinuxSetup* linuxEnvironment)
{
  // First, register the callback for parsing mobile data
  setFromMSDKCallback(vehicle, linuxEnvironment);

  // Then, setup a thread to poll the incoming data, for large functions
  pthread_t threadID = setupSamplePollingThread(vehicle);

  // User input
  std::cout << "Listening to mobile commands. Press any key to exit.\n";
  char a;
  std::cin >> a;

  // Now that we're exiting, Let's shut off the polling thread
  keepLoopRunning = false;
  void* status;
  pthread_join(threadID, &status);

  return true;
}

void
controlAuthorityMobileCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                               UserData userData)
{
  ACK::ErrorCode ack;
  ack.data = OpenProtocolCMD::ErrorCode::CommonACK::NO_RESPONSE_ERROR;

  unsigned char data    = 0x1;
  int           cbIndex = vehiclePtr->callbackIdIndex();

  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <= sizeof(uint16_t))
  {
    ack.data = recvFrame.recvData.ack;
    ack.info = recvFrame.recvInfo;
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
  }

  if (ACK::getError(ack))
  {
    if (ack.data ==
        OpenProtocolCMD::ControlACK::SetControl::OBTAIN_CONTROL_IN_PROGRESS)
    {
      ACK::getErrorCodeMessage(ack, __func__);
      vehiclePtr->obtainCtrlAuthority(controlAuthorityMobileCallback);
    }
    else if (ack.data == OpenProtocolCMD::ControlACK::SetControl::
                           RELEASE_CONTROL_IN_PROGRESS)
    {
      ACK::getErrorCodeMessage(ack, __func__);
      vehiclePtr->releaseCtrlAuthority(controlAuthorityMobileCallback);
    }
    else
    {
      ACK::getErrorCodeMessage(ack, __func__);
    }
  }
  else
  {
    // We have a success case.
    // Send this data to mobile
    AckReturnToMobile mobileAck;
    // Find out which was called: obtain or release
    if (recvFrame.recvInfo.buf[2] == ACK::OBTAIN_CONTROL)
    {
      mobileAck.cmdID = 0x02;
    }
    else if (recvFrame.recvInfo.buf[2] == ACK::RELEASE_CONTROL)
    {
      mobileAck.cmdID = 0x03;
    }
    mobileAck.ack = static_cast<uint16_t>(ack.data);
    sendDataToMSDK(vehiclePtr, reinterpret_cast<uint8_t*>(&mobileAck),
                   sizeof(mobileAck));
    // Display ACK message
    ACK::getErrorCodeMessage(ack, __func__);
  }
}

void
actionMobileCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                     UserData userData)
{
  ACK::ErrorCode ack;

  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <= sizeof(uint16_t))
  {

    ack.info = recvFrame.recvInfo;

    if (vehiclePtr->isLegacyM600())
      ack.data = recvFrame.recvData.ack;
    else if (vehiclePtr->isM100())
      ack.data = recvFrame.recvData.ack;
    else
      ack.data = recvFrame.recvData.commandACK;

    // Display ACK message
    ACK::getErrorCodeMessage(ack, __func__);

    AckReturnToMobile mobileAck;
    const uint8_t     cmd[] = { recvFrame.recvInfo.cmd_set,
                            recvFrame.recvInfo.cmd_id };

    // startMotor supported in FW version >= 3.3
    // setArm supported only on Matrice 100 && M600 old FW
    if (vehiclePtr->isM100() || vehiclePtr->isLegacyM600())
    {
      if ((memcmp(cmd, OpenProtocolCMD::CMDSet::Control::setArm, sizeof(cmd)) &&
           recvFrame.recvInfo.buf[2] == true))
      {
        mobileAck.cmdID = 0x05;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        sendDataToMSDK(vehiclePtr, reinterpret_cast<uint8_t*>(&mobileAck),
                       sizeof(mobileAck));
      }
      else if ((memcmp(cmd, OpenProtocolCMD::CMDSet::Control::setArm,
                       sizeof(cmd)) &&
                recvFrame.recvInfo.buf[2] == false))
      {
        mobileAck.cmdID = 0x06;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        sendDataToMSDK(vehiclePtr, reinterpret_cast<uint8_t*>(&mobileAck),
                       sizeof(mobileAck));
      }
      else if (recvFrame.recvInfo.buf[2] ==
               Control::FlightCommand::LegacyCMD::takeOff)
      {
        mobileAck.cmdID = 0x07;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        sendDataToMSDK(vehiclePtr, reinterpret_cast<uint8_t*>(&mobileAck),
                       sizeof(mobileAck));
      }
      else if (recvFrame.recvInfo.buf[2] ==
               Control::FlightCommand::LegacyCMD::landing)
      {
        mobileAck.cmdID = 0x08;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        sendDataToMSDK(vehiclePtr, reinterpret_cast<uint8_t*>(&mobileAck),
                       sizeof(mobileAck));
      }
      else if (recvFrame.recvInfo.buf[2] ==
               Control::FlightCommand::LegacyCMD::goHome)
      {
        mobileAck.cmdID = 0x09;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        sendDataToMSDK(vehiclePtr, reinterpret_cast<uint8_t*>(&mobileAck),
                       sizeof(mobileAck));
      }
    }
    else // Newer firmware
    {
      if (recvFrame.recvInfo.buf[2] == Control::FlightCommand::startMotor)
      {
        mobileAck.cmdID = 0x05;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        sendDataToMSDK(vehiclePtr, reinterpret_cast<uint8_t*>(&mobileAck),
                       sizeof(mobileAck));
      }
      else if (recvFrame.recvInfo.buf[2] == Control::FlightCommand::stopMotor)
      {
        mobileAck.cmdID = 0x06;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        sendDataToMSDK(vehiclePtr, reinterpret_cast<uint8_t*>(&mobileAck),
                       sizeof(mobileAck));
      }
      else if (recvFrame.recvInfo.buf[2] == Control::FlightCommand::takeOff)
      {
        mobileAck.cmdID = 0x07;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        sendDataToMSDK(vehiclePtr, reinterpret_cast<uint8_t*>(&mobileAck),
                       sizeof(mobileAck));
      }
      else if (recvFrame.recvInfo.buf[2] == Control::FlightCommand::landing)
      {
        mobileAck.cmdID = 0x08;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        sendDataToMSDK(vehiclePtr, reinterpret_cast<uint8_t*>(&mobileAck),
                       sizeof(mobileAck));
      }
      else if (recvFrame.recvInfo.buf[2] == Control::FlightCommand::goHome)
      {
        mobileAck.cmdID = 0x09;
        mobileAck.ack   = static_cast<uint16_t>(ack.data);
        sendDataToMSDK(vehiclePtr, reinterpret_cast<uint8_t*>(&mobileAck),
                       sizeof(mobileAck));
      }
    }
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
  }
}

void
sendDroneVersionFromCache(DJI::OSDK::Vehicle* vehiclePtr)
{
  // We will reformat the cached drone version and send it back.
  std::string hwVersionString(vehiclePtr->getHwVersion());
  std::string fwVersionString   = std::to_string(vehiclePtr->getFwVersion());
  std::string versionString     = hwVersionString + ' ' + fwVersionString;
  char        tempRawString[38] = { 0 };
  memcpy(&tempRawString, versionString.c_str(), versionString.length());
  // Now pack this up into a mobile packet
  VersionMobilePacket versionPack(0x01, &tempRawString[0]);

  // And send it
  sendDataToMSDK(vehiclePtr, reinterpret_cast<uint8_t*>(&versionPack),
                 sizeof(versionPack));
}

VersionMobilePacket::VersionMobilePacket(uint16_t cmdID, char* versionPack)
  : version{ 0 }
{
  this->cmdID = cmdID;
  memcpy(this->version, versionPack, sizeof(version) / sizeof(version[0]));
}

void
activateMobileCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                       UserData userData)
{

  ACK::ErrorCode ack;

  // First, let's go through the same steps the default Vehicle callback does
  // for activation
  // since activation status is necessary for  the rest of the system to
  // function properly.

  uint16_t ack_data;
  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <= 2)
  {
    ack_data = recvFrame.recvData.ack;

    ack.data = ack_data;
    ack.info = recvFrame.recvInfo;

    if (ACK::getError(ack) &&
        ack_data ==
          OpenProtocolCMD::ErrorCode::ActivationACK::OSDK_VERSION_ERROR)
    {
      DERROR("SDK version did not match\n");
      vehiclePtr->getDroneVersion();
    }

    //! Let user know about other errors if any
    ACK::getErrorCodeMessage(ack, __func__);
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
  }

  if (ack_data == OpenProtocolCMD::ErrorCode::ActivationACK::SUCCESS &&
      vehiclePtr->getAccountData().encKey)
  {
    vehiclePtr->protocolLayer->setKey(vehiclePtr->getAccountData().encKey);
  }

  // Now, we have to send the ACK back to mobile
  AckReturnToMobile mobileAck;

  mobileAck.cmdID = 0x04;
  mobileAck.ack   = (uint16_t)ack.data;

  sendDataToMSDK(vehiclePtr, reinterpret_cast<uint8_t*>(&mobileAck),
                 sizeof(mobileAck));
}

void
sendAckToMobile(DJI::OSDK::Vehicle* vehiclePtr, uint16_t cmdID, uint16_t ack)
{
  // Generate a local ACK to send the ACK back to mobile
  AckReturnToMobile mobileAck;
  mobileAck.cmdID = cmdID;
  mobileAck.ack   = ack;
  sendDataToMSDK(vehiclePtr, reinterpret_cast<uint8_t*>(&mobileAck),
                 sizeof(mobileAck));
}

bool
runPositionControlSample(Vehicle* vehicle)
{
  bool positionControlError = false;
  positionControlError      = monitoredTakeoff(vehicle);
  positionControlError &= moveByPositionOffset(vehicle, 0, 6, 6, 30);
  positionControlError &= moveByPositionOffset(vehicle, 6, 0, -3, -30);
  positionControlError &= moveByPositionOffset(vehicle, -6, -6, 0, 0);
  positionControlError &= monitoredLanding(vehicle);

  return (!positionControlError); // We want to return success status, not error
                                  // status
}

pthread_t
setupSamplePollingThread(Vehicle* vehicle)
{
  int         ret = -1;
  std::string infoStr;

  pthread_t      threadID;
  pthread_attr_t attr;

  /* Initialize and set thread detached attribute */
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

  ret     = pthread_create(&threadID, NULL, mobileSamplePoll, (void*)vehicle);
  infoStr = "mobilePoll";

  if (0 != ret)
  {
    DERROR("fail to create thread for %s!\n", infoStr.c_str());
  }

  ret = pthread_setname_np(threadID, infoStr.c_str());
  if (0 != ret)
  {
    DERROR("fail to set thread name for %s!\n", infoStr.c_str());
  }
  return threadID;
}

void*
mobileSamplePoll(void* vehiclePtr)
{
  Vehicle* vehicle = (Vehicle*)vehiclePtr;

  // Initialize variables so as to not cross case statements
  bool coreMissionStatus = false;
  int  wayptPolygonSides, hotptInitRadius;
  int  responseTimeout = 1;

  // Run polling loop until we're told to stop
  while (keepLoopRunning)
  {
    // Check global variable to find out if we need to execute an action
    switch (mobileDataID_glob)
    {
      case 0x3E:
        coreMissionStatus = runPositionControlSample(vehicle);
        sendAckToMobile(vehicle, 0x3E, coreMissionStatus);
        mobileDataID_glob = 0;
        break;
      case 0x40:
        coreMissionStatus = gimbalCameraControl(vehicle);
        sendAckToMobile(vehicle, 0x40, coreMissionStatus);
        mobileDataID_glob = 0;
        break;
      case 0x41:
        wayptPolygonSides = 6;
        coreMissionStatus =
          runWaypointMission(vehicle, wayptPolygonSides, responseTimeout);
        sendAckToMobile(vehicle, 0x41, coreMissionStatus);
        mobileDataID_glob = 0;
        break;
      case 0x42:
        hotptInitRadius = 10;
        coreMissionStatus =
          runHotpointMission(vehicle, hotptInitRadius, responseTimeout);
        sendAckToMobile(vehicle, 0x42, coreMissionStatus);
        mobileDataID_glob = 0;
        break;
      default:
        break;
    }
    usleep(500000);
  }
}