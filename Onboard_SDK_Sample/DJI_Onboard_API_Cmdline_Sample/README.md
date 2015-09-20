#DJI_Onboard_API_Cmdline_Sample
##Directory Structure
* cmake: makefile and temporary files
* src: source code
* src/DJI_LIB: DJI Onboard API Library (Experimental)
* output: executables

##Development & Running Environment
Operating System: Ubuntu 12.04
g++ version: 4.6.3

## Hardware Installation
In order to communicate with the N1 Autopilot via the DJI OPEN protocal, a physical connection between your computer and N1 Autopilot is required with a USB to TTL Serial cable(SOLD Seperately).

##Configs
* Enter the following info into *./output/config.xml.*
*APP ID*
*App Level*
*Communication Key*
*uart device name?*
*baudrate*.

>Note: the 'baudrate' set in 'config.xml' needs to be consistent with the one set in the DJI N1 PC assistant.

##Compile
~~~bash
cd cmake
make
~~~

If the compilation is completed, an Linux executable called 'DJI_Onboard_API_Cmdline_Test' will be sitted in the *output* directory.

##Run

For beginners, we recommand running this demo along with the DJI PC Simulator.

Note: Please make sure that you have READ/WRITE/ACCESS(?) privilege to the Linux serial ports(?). Assume your serial device is called */dev/ttyUSB0*, enter the following command to gain access privilege(?).

~~~bash
sudo chmod 777 /dev/ttyUSB0??
~~~

Run the program by entering the following command
~~~bash
./DJI_Onboard_API_Cmdline_Test
~~~

ENJOY the demo!
