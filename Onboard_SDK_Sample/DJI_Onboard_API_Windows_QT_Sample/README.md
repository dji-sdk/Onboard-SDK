#DJI Onboard SDK Windows QT Sample
##Intro
This example aims to help you understand and play with the basic flight procedures including:

* The activation
* The flight control obtainment
* The flight control release
* The take off procedure
* The landing procedure
* The go home procedure
* The attitude control
* The gimbal control

<!-- * The flight control info obtainment -->  

Developers can play with this example via the GUI interface in Windows.

##Directory Structure
* exe: executables, dependent files are also included
* src: source code
* src/DJI_LIB: the DJI Onboard SDK API Library
* README.md: this file

##Compile & Run Environment
The below environment has been tested.
* Operating System: Windows 7 64Bit  
* Integrated Development Environment (IDE): Qt Creator 3.3.1  
* Qt Ver: Qt 5.3.2 MinGW 32bit  

## Hardware Installation
* In order to communicate with the N1 Autopilot via the DJI OPEN protocal, a physical connection between your Onboard Device and the N1 Autopilot is required with a USB to TTL serial cable (SOLD Seperately).
* In order to monitor & control the flight, a remote controller connects to your Mobile Device (with the DJI GO APP running) is needed.

##Configs
1. Open *DJI_Onboard_API_Windows_QT_Sample.pro* locates at *src* sub-directory in the DJI_Onboard_API_Windows_QT_Sample*.

2. Select the *Desktop Qt 5.3 MinGW 32bit* kit and click *Configure Project* to proceed.

3. During the initialization, enter the following info into the GUI interface including:

* APP ID
* App Level
* Communication Key
* uart device name (e.g. COM1, COM2)
* baudrate

>Note: the 'baudrate' needs to be consistent with the setting in the DJI N1 PC assistant.

##Compile
Click the hammer icon on the bottom left in the Qt Creator IDE to start compiling.

##Run
Click the green triangle icon on the bottom left corner to run the sample.
We recommend you first run this example in the simulator then carefully move to the real flight test.

##What You Can Expect
* You can see the flight control simulations on screen if you are using the DJI PC simulator. Otherwise, real flight happens.
* You can see the actual 'gimbal and camera' movement.
* You can see the image/video you capture from you Mobile Device.

ENJOY your flight!
