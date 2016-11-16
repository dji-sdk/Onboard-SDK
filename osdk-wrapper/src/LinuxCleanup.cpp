/*! @file LinuxCleanup.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Executes some cleanup commands that are required by the Onboard SDK
 *  but are not pertinent to the user's work. Also cleans up threads and memory.    
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "LinuxCleanup.h"

int cleanup(LinuxSerialDevice* serialDevice, CoreAPI* api, Flight* flight, LinuxThread* read)
{
  //! Set the drone broadcast frequencies back to defaults 
  int broadcastStatus = api->setBroadcastFreqDefaults(1);
  if (broadcastStatus != 0)
  {
    std::cout << "Unable to set broadcast frequencies to defaults.\nPlease go to DJI Assistant 2 and change frequencies." << std::endl;
  }

  //! Release control
  ackReturnData controlStatus = releaseControl(api);
  if (controlStatus.status == -1)
  {
   std::cout << "Unable to release control. \nYou should manually switch control by toggling the RC mode to P and back to F." << std::endl; 
  }

  //! Stop the read thread
  read->stopThread();

  //! Delete the serialDevice and api objects
  delete serialDevice;
  delete api;
  delete flight;

  return 0;

}

int cleanupNonBlocking(LinuxSerialDevice* serialDevice, CoreAPI* api, Flight* flight, LinuxThread* read, LinuxThread* callback)
{
    //! Set the drone broadcast frequencies back to defaults
    int broadcastStatus = api->setBroadcastFreqDefaults(1);
    if (broadcastStatus != 0)
    {
        std::cout << "Unable to set broadcast frequencies to defaults.\nPlease go to DJI Assistant 2 and change frequencies." << std::endl;
    }

    //! Release control
    releaseControlNonBlocking(api);
    usleep(1000000);
    //! Stop the read thread
    read->stopThread();

    //! Delete the serialDevice and api objects
    delete serialDevice;
    delete api;
    delete flight;

    return 0;

}

ackReturnData releaseControl(CoreAPI* api)
{
  ackReturnData controlAck;
  //! Release code 0x0
  unsigned char data = 0x0;

  controlAck.ack = api->setControl(false, 1);
  switch (controlAck.ack)
  {
    case ACK_SETCONTROL_NEED_MODE_F:
      std::cout << "Failed to release control.\nYour RC mode switch is not in mode F. (Is the RC still connected and paired?)" << std::endl;
      controlAck.status = -1;
      return controlAck;
      break;
    case ACK_SETCONTROL_NEED_MODE_P:
      std::cout << "Failed to release control.\nFor A3 v3.2, your RC needs to be in P mode. (Is the RC connected and paired?)" << std::endl;
      controlAck.status = -1;
      return controlAck;
      break;
    case ACK_SETCONTROL_RELEASE_SUCCESS:
      std::cout << "Released control successfully."<< std::endl;
      break;
    case ACK_SETCONTROL_RELEASE_RUNNING:
      std::cout << "Release control running.."<< std::endl;
      releaseControl(api);
      break;
    case ACK_SETCONTROL_IOC:
      std::cout << "The aircraft is in IOC mode. Cannot release control.\nGo to DJI GO and stop all intelligent flight modes before trying this." << std::endl;
      controlAck.status = -1;
      return controlAck;
      break;
    default:
      std::cout << "Error in setControl API function." << std::endl;
      break;
  }
  controlAck.status = 1;
  return controlAck;
}

void releaseControlCallback(CoreAPI* api, Header *protHeader, DJI::UserData data) {
{

  uint16_t simpAck = api->missionACKUnion.simpleACK;
  switch (simpAck)
  {
    case ACK_SETCONTROL_ERROR_MODE:
      if(api->getSDKVersion() != versionA3_32)
        std::cout << "Failed to release control.\nYour RC mode switch is not in mode F. (Is the RC still connected and paired?)" << std::endl;
      else
        std::cout << "Failed to release control.\nYour RC mode switch is not in mode P. (Is the RC still connected and paired?)" << std::endl;
      break;
    case ACK_SETCONTROL_RELEASE_SUCCESS:
      std::cout << "Released control successfully."<< std::endl;
      break;
    case ACK_SETCONTROL_RELEASE_RUNNING:
      std::cout << "Release control running.."<< std::endl;
      releaseControlNonBlocking(api);
      break;
    case ACK_SETCONTROL_IOC:
      std::cout << "The aircraft is in IOC mode. Cannot release control.\nGo to DJI GO and stop all intelligent flight modes before trying this." << std::endl;
      break;
    default:
      std::cout << "Error in setControl API function." << std::endl;
      break;
  }
}

}
void releaseControlNonBlocking(CoreAPI* api)
{
  void (*pointerreleaseControlCallback)(CoreAPI*, Header*, DJI::UserData);
  pointerreleaseControlCallback = releaseControlCallback;
  api->setControl(false, pointerreleaseControlCallback, NULL);
}

