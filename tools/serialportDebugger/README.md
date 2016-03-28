#SerialportDebugger
##Intro
As finding a serialport debugger which support baudrate 921600 and 230400 is a difficult work,
we provide a simple debugger. 

##Compile & Run Environment
The below environment are required.
* Qt Ver: QT 5.4 or later
* Compiler Ver: MinGW 4.9.2 or later; MSVC2012 or later

##Config and compile
1. Open *V1Viewer.pro* locates at *tools/serialportDebugger*.

2. Select a proper compiler and click *Configure Project* to proceed.

3. During the init process, enter the following info into the GUI interface including:

* uart device name (e.g. COM1, COM2)
* baudrate

>Note: the 'baudrate' needs to be consistent with the setting in the DJI N1 PC assistant.

#串口调试助手

##简介

没什么好简介的，网上的串口助手都太挫了，不支持230400和921600的波特率，为了方便大家，特地提供一个。

##调试与运行环境
我们成功测试了如下运行环境：
* QT 版本： QT 5.4 或更新。
* 编译器版本： MinGW 4.9.2 或更新；MSVC2012或更新。

##配置和编译

1. 打开  `V1Viewer.pro` 载入 QT 例程。
2. 选择合适的编译器并编译项目。
3. 在程序中选择好正确的设备和波特率。

##运行

就是用来测试硬件链路的玩具代码，将就着用吧，以后没准会改的正常一些。