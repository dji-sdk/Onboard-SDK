#DJI_Onboard_API_Cmdline_Sample
##Directory Structure
* src: source code
* src/DJI_LIB: DJI Onboard API Library
* cmake: makefile and temporary files
* output: executable files

##Development Environment
Operating System: Ubuntu 12.04
g++ version: 4.6.3


## Hardware Installation
In order to communicate with the N1 Autopilot, a physical connection between your computer and N1 Autopilot is required with a USB to TTL Serial cable(SOLD Seperately).

##Configuration
* Enter the following info *APP ID*,*App Level* and *Communication Key* into *./output/config.xml.*  
* Enter your *uart device name?* and *baudrate* in *./output/config.xml*.

>Note:Developers need tp ensure that the baudrate set in "config.xml" is consistent with the one of UAV set in DJI N1 PC assistant software.

##Compile
~~~bash
cd cmake
make
~~~

Now, if the compilation is successful, you can locate the Linuc executable file in the *output* directory.

##Run

For beginners, we recommand running this demo in DJI PC Simulator.

Ensure that the current account has acess privilege to the serial device. Assume that the serial device is named as */dev/ttyUSB0*, use the following command to gain access privilege for the serial device.

~~~bash
sudo chmod 777 /dev/ttyUSB0??
~~~


Using the following command to launch the testing program
~~~bash
./DJI_Onboard_API_Cmdline_Test
~~~
Then there should be a menu display on screen.
Follow tips on screen and control the UAV.