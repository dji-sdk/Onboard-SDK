#DJI_Onboard_API_Cmdline_Sample
##Intro
This example aims to help you understand the following basic flight procedures including:
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
* doc: the section will be decrypted SHORTLY
* output: executables
* src: source code
* src/DJI_LIB: the core DJI Onboard API Library (Experimental)
* README.md: this file

##Compile & Run Environment
Operating System: Ubuntu 12.04
g++ version: 4.6.3

## Hardware Installation
In order to communicate with the N1 Autopilot via the DJI OPEN protocal, a physical connection between your computer and N1 Autopilot is required with a USB to TTL Serial cable(SOLD Seperately).

##Configs
Enter the following info into *./output/config.xml.*
* APP ID
* App Level
* Communication Key
* uart device name
* baudrate

>Note: the 'baudrate' set in the 'config.xml' needs to be consistent with the setting in the DJI N1 PC assistant.

##Compile
~~~bash
cd cmake
make
~~~

If the compilation is completed, an Linux executable called 'DJI_Onboard_API_Cmdline_Test' will be sitted in the *output* dir.

##Run
We recommand running this demo along with the DJI PC Simulator and Mobile Device for better user experience. Also, please be aware that you will need sudo privilege to manipulate the Linux serial port. Assume your linux serial communication device of this program is */dev/ttyUSB0*, enter the following command to gain access privilege.

~~~bash
sudo chmod 777 /dev/ttyUSB0
~~~

Run the example by entering the following
~~~bash
./DJI_Onboard_API_Cmdline_Test
~~~

##What you will expect
* You can see the flight control simulations on screen. Otherwise, real flight happens:)
* You can see the actual 'gimbal and camera' movement
* You can see the image/video you capture from you mobile device

Have FUN!
