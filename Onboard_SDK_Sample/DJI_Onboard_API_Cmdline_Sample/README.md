#DJI Onboard SDK Command Line Example
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

Developers can play with this example via the command line interaction in Linux.

##Directory Structure
* cmake: makefile and temporary files
* output: executables
* src: source code
* src/DJI_LIB: the DJI Onboard SDK API Library
* README.md: this file

##Compile & Run Environment
The below environment has been tested.
* Operating System: Ubuntu 12.04
* g++ version: 4.6.3

## Hardware Installation
* In order to communicate with the N1 Autopilot via the DJI OPEN protocal, a physical connection between your Onboard Device and the N1 Autopilot is required with a USB to TTL Serial cable (SOLD Seperately).
* In order to monitor & control the flight, a remote controller connects to the mobile device(with the DJI GO APP running) is needed.

##Configs
Enter the following info into *./output/config.xml.*

* APP ID
* App Level
* Communication Key
* uart device name (e.g. /dev/ttyUSB0)
* baudrate

>Note: the 'baudrate' set in the 'config.xml' needs to be consistent with the setting in the DJI N1 PC assistant.

## Compile
~~~bash
cd cmake
make
~~~

If the compilation is completed, an Linux executable called 'DJI_Onboard_API_Cmdline_Test' will be sitted in the *output* dir.

##Run
We recommend you first run this example in the simulator then move to the real flight test. Also, please be aware that you will need sudo privilege to manipulate the Linux serial port. You may need to enter the following command to gain the access privilege.

Please make sure that 'config.xml' file is sitted in the *output* folder.

Run the example by entering the following command
~~~bash
sudo ./DJI_Onboard_API_Cmdline_Test
~~~

##What You Can Expect
* You can see the flight control simulations on screen if you are using the DJI PC simulator. Otherwise, real flight happens.
* You can see the actual 'gimbal and camera' movement.
* You can see the image/video you capture from you mobile device.

ENJOY your flight!
