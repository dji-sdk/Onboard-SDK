#DJI_Onboard_API_Windows_QT_Sample
##Directory Structure
* src: source code files
* src/DJI_LIB: DJI Onboard API Library
* output: compiled output files
* exe: executable file. dependent files are also included.

##Development Environment
Operation System: Windows 7 64bit  
Integrated Development Environment(IDE): Qt Creator 3.3.1  
Qt Version: Qt 5.4.1 MinGV 32bit  

## Hardware Installation
In order to communicate with the N1 Autopilot, a physical connection between your computer and N1 Autopilot is required with a USB to TTL Serial cable(SOLD Seperately).

##Configuration
Open *DJI_Onboard_API_Windows_QT_Sample.pro* locates at *src* sub-directory in the *DJI_Onboard_API_Windows_QT_Sample*.

Select the *Desktop Qt 5.3 MinGW 32bit* kit and click *Configure Project* to proceed.

##Compile
This section will guide you to compile the sample code by uing the Qt Creator 3.3.1 IDE. Click the hammer icon on the bottom left corner to start compiling.

##Run

Click the green triangle icon on the bottom left corner to run the sample.

Now, the main GUI of the sample is shown.

Serial Port Settings

* Set the avaiable serial port and its baudrate value in the Serial Port section.  
* Developers must ensure the baudrate set is consistent with the one of the aircraft  
* Click *Refresh* button to check the information of seial devices connected with windows host in the windows console.