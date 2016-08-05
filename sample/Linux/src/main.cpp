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
#include "../../platform/LinuxSerialDevice.h"
#include "LinuxThread.h"
#include "LinuxSetup.h"
#include "LinuxCleanup.h"
#include "ReadUserConfig.h"
#include "LinuxMobile.h"
#include "LinuxFlight.h"
#include "LinuxInteractive.h"
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
  LinuxThread read(api, 2);

  //! Setup
  int setupStatus = setup(serialDevice, api, &read);
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
    mobileCommandSpin(api, flight, waypointObj);
  }
  //! Interactive Mode
  else if (argv[1] && !strcmp(argv[1], "-interactive"))
  {
    interactiveSpin(api, flight, waypointObj);
  }
  //! Programmatic Mode - execute everything here without any interactivity. Useful for automation.
  else if (argv[1] && !strcmp(argv[1], "-programmatic"))
  {
    /*! Set a blocking timeout - this is the timeout for blocking API calls
        to wait for acknowledgements from the aircraft. Do not set to 0.
    !*/
    int blockingTimeout = 1; //Seconds
    
    //! Monitored Takeoff
    ackReturnData takeoffStatus = monitoredTakeoff(api, flight, blockingTimeout);

    //! Set broadcast Freq Defaults
    unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);
    
    //! If the aircraft took off, continue to do flight control tasks 
    if (takeoffStatus.status == 1)
    {

      /*! This is where you can add your own flight functionality.
          Check out LinuxWaypoint and LinuxFlight for available APIs. 
          You can always execute direct Onboard SDK Library API calls
          through the api object available in this example.
      !*/ 

      //! Do aircraft control - Waypoint example. 
      wayPointMissionExample(api, waypointObj,blockingTimeout);

      //! Land
      ackReturnData landingStatus = landing(api, flight,blockingTimeout);
    }
    else 
    {
      //Try to land directly
      ackReturnData landingStatus = landing(api, flight,blockingTimeout);
    }
  }
  //! No mode specified or invalid mode specified" 
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
  int cleanupStatus = cleanup(serialDevice, api, flight, &read);
  if (cleanupStatus == -1)
  {
    std::cout << "Unable to cleanly destroy OSDK infrastructure. There may be residual objects in the system memory.\n";
    return 0;
  }
  std::cout << "Program exited successfully." << std::endl;

  return 0;
}

