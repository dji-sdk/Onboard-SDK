#SerialportDebugger

## Intro
As finding a serialport debugger which support baudrate 921600 and 230400 is difficult,
we provide a simple debugger. 

## Compile & Run Environment
The below environment are required.
* Qt Ver: QT 5.4 or later
* Compiler Ver: MinGW 4.9.2 or later; MSVC2012 or later

##Config and compile
1. Open *V1Viewer.pro* located at *tools/serialportDebugger*.

2. Select your compiler and click *Configure Project* to proceed.

3. During the init process, enter the following info into the GUI interface including:

* uart device name (e.g. COM1, COM2)
* baudrate

>Note: the 'baudrate' needs to be consistent with the setting in the DJI N1 PC assistant.
