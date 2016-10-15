/*! @file main.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  New Linux App for DJI Onboard SDK. 
 *  Provides a number of convenient abstractions/wrappers around core API calls.
 *
 *  Calls are blocking; the calling thread will sleep until the
 *  call returns. Use Core API calls or another sample if you 
 *  absolutely need non-blocking calls. 
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

//System Headers
#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>

//DJI Linux Application Headers
#include "LinuxSerialDevice.h"
#include "LinuxThread.h"
#include "LinuxSetup.h"
#include "LinuxCleanup.h"
#include "ReadUserConfig.h"
#include "LinuxMobile.h"
#include "LinuxFlight.h"
#include "LinuxInteractiveNonBlocking.h"
#include "LinuxWaypoint.h"

//DJI OSDK Library Headers
#include <DJI_Follow.h>
#include <DJI_Flight.h>
#include <DJI_Version.h>
#include <DJI_WayPoint.h>

using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;

//! Main function for the Linux sample. Lightweight. Users can call their own API calls inside the Programmatic Mode else on Line 68. 
int main(int argc, char *argv[])
{
  //! Instantiate a serialDevice, an API object, flight and waypoint objects and a read thread.
  LinuxSerialDevice* serialDevice = new LinuxSerialDevice(UserConfig::deviceName, UserConfig::baudRate);
  CoreAPI* api = new CoreAPI(serialDevice);
  Flight* flight = new Flight(api);
  WayPoint* waypointObj = new WayPoint(api);

  //! Enable non-blocking callback thread mechanism
  api->nonBlockingCBThreadEnable = true;

  //! Initializes the read thread and the call back thread.
  LinuxThread readThread(api,2);
  LinuxThread CallbackThread(api,3);


  //! Setup
  int setupStatus = setupNonBlocking(serialDevice, api, &readThread, &CallbackThread);
  if (setupStatus == -1)
  {
    std::cout << "This program will exit now. \n";
    return 0;
  }
  //! Set broadcast Freq Defaults
  unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);

  //! Mobile Mode
  if (argv[1] && !strcmp(argv[1],"-mobile"))
  {
    std::cout << "Listening to Mobile Commands\n";
    mobileCommandSpinNonBlocking(api, flight, waypointObj);
  }
  //! Interactive Mode
  else if (argv[1] && !strcmp(argv[1], "-interactive"))
  {
    interactiveSpin(api, flight, waypointObj);
  }

  else if (argv[1] && !strcmp(argv[1], "-programmatic"))
  {
    std::cout << "Add your programmatic code here.\n";
  }
  //! Programmatic Mode - execute everything here without any interactivity. Useful for automation.

  else
    std::cout << "\nYou need to call the OSDK Linux sample with one of\n"
                 "the following arguments: \n\n"
                 "-mobile      : Run in Mobile Data Transparent Transmission mode\n"
                 "-interactive : Run in a Terminal-based UI mode\n"
                 "-programmatic: Run without user input, use if you have put automated\n"
                 "               calls in the designated space in the main function. \n"
                 "               By default this mode will execute an automated waypoint\n"
                 "               mission example, so be careful.\n\n";
  //! Cleanup
  int cleanupStatus = cleanupNonBlocking(serialDevice, api, flight, &readThread , &CallbackThread);
  if (cleanupStatus == -1)
  {
    std::cout << "Unable to cleanly destroy OSDK infrastructure. There may be residual objects in the system memory.\n";
    return 0;
  }
  std::cout << "Program exited successfully." << std::endl;

  return 0;
}

