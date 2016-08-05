/*! @file LinuxInteractive.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Barebones interactive UI for executing Onboard SDK commands.
 *  Calls functions from the new Linux example based on user input.
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "LinuxInteractive.h"

using namespace std;

char userInput()
{
  cout << endl;
  cout << "|------------------DJI Onboard SDK Interactive Sample------------|" << endl;
  cout << "|                                                                |" << endl;
  cout << "| The interactive mode allows you to execute a few commands      |" << endl;
  cout << "| to help you get a feel of the DJI Onboard SDK. Try them out!   |" << endl;
  cout << "|                                                                |" << endl;
  cout << "| Standard DJI Onboard SDK Usage Flow:                           |" << endl;
  cout << "| 1. Activate (The sample has already done this for you)         |" << endl;
  cout << "| 2. Obtain Control (The sample has already done this for you)   |" << endl;
  cout << "| 3. Takeoff                                                     |" << endl;
  cout << "| 4. Execute Aircraft control (Movement control/Missions/Camera) |" << endl;
  cout << "| 5. Return to Home/Land                                         |" << endl;
  cout << "| 6. Release Control (The sample will do this for you on exit)   |" << endl;
  cout << "|                                                                |" << endl;
  cout << "| Available commands:                                            |" << endl;
  cout << "| [a] Request Control                                            |" << endl;
  cout << "| [b] Release Control                                            |" << endl;
  cout << "| [c] Arm the Drone                                              |" << endl;
  cout << "| [d] Disarm the Drone                                           |" << endl;
  cout << "| [e] Takeoff                                                    |" << endl;
  cout << "| [f] Waypoint Sample                                            |" << endl;
  cout << "| [g] Position Control Sample: Draw Square                       |" << endl;
  cout << "| [h] Landing                                                    |" << endl;
  cout << "| [i] Go Home                                                    |" << endl;
  cout << "| [j] Exit this sample                                           |" << endl;
  cout << "|                                                                |" << endl;
  cout << "| Type one of these letters and then press the enter key.        |" << endl;
  cout << "|                                                                |" << endl;
  cout << "| If you're new here, try following the usage flow shown above.  |" << endl;
  cout << "|                                                                |" << endl;
  cout << "| Visit developer.dji.com/onboard-sdk/documentation for more.    |" << endl;
  cout << "|                                                                |" << endl;
  cout << "|------------------DJI Onboard SDK Interactive Sample------------|" << endl;

  char inputChar;
  cin >> inputChar;
  return inputChar;
}

void interactiveSpin(CoreAPI* api, Flight* flight, WayPoint* waypointObj)
{
  bool userExitCommand = false;

  ackReturnData takeControlStatus;
  ackReturnData releaseControlStatus;
  ackReturnData armStatus;
  ackReturnData disArmStatus;
  ackReturnData takeoffStatus;
  ackReturnData landingStatus;
  ackReturnData goHomeStatus;
  int drawSqrPosCtrlStatus;

  while (!userExitCommand)
  {
    char getUserInput = userInput();
    switch (getUserInput)
    {
      case 'a':
        takeControlStatus = takeControl(api);
        break;
      case 'b':
        releaseControlStatus = releaseControl(api);
        break;
      case 'c':
        armStatus = arm(flight);
        break;
      case 'd':
        disArmStatus = disArm(flight);
        break;
      case 'e': 
        takeoffStatus = monitoredTakeoff(api, flight);
        break;
      case 'f':
        wayPointMissionExample(api, waypointObj,1);
        break;
      case 'g':
        drawSqrPosCtrlStatus = drawSqrPosCtrlSample(api, flight);
        break;
      case 'h':
        landingStatus = landing(api,flight);
        break;
      case 'i':
        goHomeStatus = goHome(flight);
        break;
      case 'j':
        userExitCommand = true;
        break;
    }
    usleep(1000000);
  }
  return;
}