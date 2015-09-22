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
* The flight control info obtainment  

Developers can play with this example via the GUI interaction in Windows.

##Directory Structure
* doc: (this section will be decrypted SHORTLY)
* exe: executables. Running time dependent files are also included.
* src: source code files
* src/DJI_LIB: the DJI Onboard SDK API Library (Experimental version included)
* README.md: this file

##Compile & Run Environment
* Operating System: Windows 7 64Bit  
* Integrated Development Environment(IDE): Qt Creator 3.3.1  
* Qt Ver: Qt 5.3.2 MinGW 32bit  

## Hardware Installation
* In order to communicate with the N1 Autopilot via the DJI OPEN protocal, a physical connection between your Onboard Device and the N1 Autopilot is required with a USB to TTL Serial cable(SOLD Seperately).
* In order to monitor & control the flight, a remote controller connects to the mobile device(with the DJI GO APP running) is needed.

##Configs
Open *DJI_Onboard_API_Windows_QT_Sample.pro* locates at *src* sub-directory in the *DJI_Onboard_API_Windows_QT_Sample*;
Select the *Desktop Qt 5.3 MinGW 32bit* kit and click *Configure Project* to proceed;
During the initialization, config the following info into the GUI interface including:
* APP ID
* App Level
* Communication Key
* uart device name(e.g. COM1,COM2)
* baudrate

>Note: the 'baudrate' set in the 'config.xml' needs to be consistent with the setting in the DJI N1 PC assistant.

##Compile
Click the hammer icon on the bottom left corner in the Qt Creator 3.3.1 IDE to start compiling.

##Run
Click the green triangle icon on the bottom left corner to run the sample.
We recommend you first run this example in the simulator then carefully move to the real flight test.

##What You Can Expect
* You can see the flight control simulations on screen if you are using the DJI PC simulator. Otherwise, real flight happens.
* You can see the actual 'gimbal and camera' movement
* You can see the image/video you capture from you mobile device

ENJOY your flight!
