/*! @file main.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Command line example for DJI Onboard SDK. Based on DJI Script. 
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include <iostream>
#include <string>
#include <cstring>

#include "DJIHardDriverManifold.h"
#include "conboardsdktask.h"
#include "APIThread.h"
#include "User_Config.h"
#include "unistd.h"

#include <DJI_Version.h>

using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;

//! Example user-defined callback for overriding the DJI API callbacks.
/*!
    This callback sets a flag passed to it by the main function. The main funtion can then
    do further processing based on the flag set by the user. In this case, the further processing is simply 
    descriptive error messages specific to the command line setup.
*/ 
void activateCallbackUser(DJI::onboardSDK::CoreAPI *, Header *, void * );

//! Main function for the command line sample. Makes heavy use of DJI_Script. 
int main(int argc, char *argv[])
{
  std::string droneVerStr, SdkVerStr; 

  //! Introductory messages
  int initialConfirmation;

  switch (targetVersion){
    case DJI::onboardSDK::versionA3_31:
      droneVerStr = "A3";
      SdkVerStr = "3.1";
      break;
    case DJI::onboardSDK::versionM100_31:
      droneVerStr = "M100";
      SdkVerStr = "3.1";
      break;
    case DJI::onboardSDK::versionM100_23:
      droneVerStr = "M100";
      SdkVerStr = "2.3";
      break;
    default:
      droneVerStr = "Error. Please check your User_Config file.";
      SdkVerStr = "Error. Please check your User_Config file.";
      break;
  }
  std::cout << "Please take a moment to confirm your settings.\n"
      "Serial port = " << deviceName << "\n"
      "Baudrate = "<< baudRate << "\n"
      "Drone/FC Version: " << droneVerStr << "\n"
      "SDK Version: " << SdkVerStr << "\n"
      "\nDoes everything look correct? If not, navigate to inc/User_Config.h and make changes."
      "\nIf everything looks okay, type 1. Else type 0.\n";
  
  std::cin >> initialConfirmation;
  
  if (initialConfirmation != 1)
    return(0);

  //! @note replace these two lines below to change to another serial device driver. 
  HardDriverManifold driver(deviceName,baudRate);
  driver.init();

  //! @note Instantiate a CoreAPI with this hardware driver and a Script with this CoreAPI.
  CoreAPI api(&driver);
  ConboardSDKScript sdkScript(&api);

  //! @note Tell the SDK our hardware and software versions
  api.setVersion(targetVersion);

  // @note Instantiate a Script thread. Beyond this point, accept user commands for various tasks.
  ScriptThread st(&sdkScript);

  //! @note replace these four lines below to change to another serial device driver. Currently, the manifold driver works for all linux setups.
  APIThread send(&api, 1);
  APIThread read(&api, 2);
  send.createThread();
  read.createThread();

  //! @note Sequence for automatic activation. Reuse the --SS load fuctionality
  std::cout << "\nAttempting automatic activation..\n";
  std::string ssLoadStr = "--SS load " + keyLocation;
  char * ssLoadStr_cstr = new char [ssLoadStr.length()+1];
  std::strcpy (ssLoadStr_cstr, ssLoadStr.c_str());

  //! If ID/Key is loaded, proceed to attempt activation
  if(!loadSS(&sdkScript, ssLoadStr_cstr))
    std::cout << "Could not load key or problem in format. Make sure your key is in location" << keyLocation << "and in the right format. Retry activation.\n";
  else
  {
    //! Define a variable that we will pass by reference to the activation callback.
    //  Init with default number that will never be set by the activation callback
    uint8_t activationStatus = 99; 
    
    //! Call the activate function. Note that we are passing two optional arguments (..,callback, userData).
    //  If you do not want to implement your own callbacks, you can simply omit those. 
    sdkScript.getApi()->activate(&sdkScript.adata, activateCallbackUser, &activationStatus);
    //! Give the callback time to return
    usleep(500000);

    //! See what value the callback set 
    switch (activationStatus)
    {
      case ACK_ACTIVE_SUCCESS:
        std::cout << "Automatic activation successful." << std::endl;
        usleep(3000000);
        break;
      case ACK_ACTIVE_NEW_DEVICE:
        std::cout << "Your activation did not go through. \nThis is a new device. \nMake sure DJI GO is turned on and connected to the internet \nso you can contact the server for activation. \nActivate with '--SS load ../key.txt' followed by '--CA ac'." << std::endl;
        usleep(3000000);
        break;
      case ACK_ACTIVE_PARAMETER_ERROR:
        std::cout << "Your activation did not go through. \nThere was a parameter error. \nPlease check your setup and retry." << std::endl;
        usleep(3000000);
        break;
      case ACK_ACTIVE_ENCODE_ERROR:
        std::cout << "Your activation did not go through. \nThere was an encoding error. \nPlease check your setup and retry." << std::endl;
        usleep(3000000);
        break;
      case ACK_ACTIVE_APP_NOT_CONNECTED:
        std::cout << "Your activation did not go through. \nDJI Go does not seem to be connected. \nPlease check your mobile device and activate with '--SS load ../key.txt' followed by '--CA ac'." << std::endl;
        usleep(3000000);
        break;
      case ACK_ACTIVE_NO_INTERNET:
        std::cout << "Your activation did not go through. \nYour mobile device doesn't seem to be connected to the internet. \nPlease check your connection and activate with '--SS load ../key.txt' followed by '--CA ac'." << std::endl;
        usleep(3000000);
        break;
      case ACK_ACTIVE_SERVER_REFUSED:
        std::cout << "Your activation did not go through. \nThe server refused your credentials. \nPlease check your DJI developer account details in ../key.txt." << std::endl;
        usleep(3000000);
        break;
      case ACK_ACTIVE_ACCESS_LEVEL_ERROR:
        std::cout << "Your activation did not go through. \nYou don't seem to have the right DJI SDK permissions. \nPlease check your DJI developer account details." << std::endl;
        usleep(3000000);
        break;
      case ACK_ACTIVE_VERSION_ERROR:
        std::cout << "Your activation did not go through. \nYour SDK version in User_config.h does not match the one reported by the drone. \nPlease correct that, rebuild and activate." << std::endl;
        usleep(3000000);
        break;
      default:
        std::cout << "There was an error with the activation command. Perhaps your drone is not powered on? \nPlease check your setup and retry." << std::endl;
        usleep(3000000);
        break;
    }
  }

  //! @note Run the interactive script to allow further commands from the user 
  st.run();
  return 0;
}

void activateCallbackUser(DJI::onboardSDK::CoreAPI *api, Header *protocolHeader, void * activationStatus)
{
  uint8_t ack_data;
  if (protocolHeader->length - EXC_DATA_SIZE <= 2)
  {
    memcpy((unsigned char *)&ack_data, ((unsigned char *)protocolHeader) + sizeof(Header),
        (protocolHeader->length - EXC_DATA_SIZE));
    if (ack_data == ACK_ACTIVE_SUCCESS)
    {
      if (api->getAccountData().encKey)
          api->setKey(api->getAccountData().encKey);
    }
    uint8_t *acStatPtr = (uint8_t *)activationStatus;
    *acStatPtr = ack_data;
  }
  else
  {
    std::cout << "Automatic Activation Unsuccessful. Please try with '--SS load' and '--CA ac' " << std::endl;
  }
}
