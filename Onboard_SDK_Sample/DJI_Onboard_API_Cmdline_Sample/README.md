#DJI_Onboard_API_Cmdline_Sample(? rename)
##Intro
* (Needed to be filled including purpose, features and what you can learn)

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
* Enter the following info into *./output/config.xml.*
*APP ID*
*App Level*
*Communication Key*
*uart device name?*
*baudrate*.

>Note: the 'baudrate' set in the 'config.xml' needs to be consistent with the setting in the DJI N1 PC assistant.

##Compile
~~~bash
cd cmake
make
~~~

If the compilation is completed, an Linux executable called 'DJI_Onboard_API_Cmdline_Test' will be sitted in the *output* dir.

##Run
For beginners, we recommand running this demo along with the DJI PC Simulator for better user experience.

Note: Please be aware that you will need READ/WRITE/ACCESS(?) rights to the program specifc Linux serial port(?). Assume your linux serial communication device for this program is */dev/ttyUSB0*, enter the following command to gain access privilege(?).

~~~bash
sudo chmod 777 /dev/ttyUSB0(? Or we do NOT need to tell them how to do it and they will google it)
~~~

Run the example by entering the following
~~~bash
./DJI_Onboard_API_Cmdline_Test
~~~

##What you will expect
(needs to be filled)
