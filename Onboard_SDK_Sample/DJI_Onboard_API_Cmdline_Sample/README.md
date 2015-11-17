#DJI Onboard SDK Command Line Example

##Introduction

This example aims to help you understand and play with the basic flight procedures including:

* The activation
* The flight control obtainment
* The flight control release
* The take off procedure
* The landing procedure
* The go home procedure
* The attitude control
* The gimbal movement control
* The camera control
* The status query

Developers can play with this example via the command line interaction interface in linux.

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
* In order to monitor & control the flight, a remote controller connects to the mobile device (with the DJI GO APP running) is needed.

##Configs

Enter the following info into `./output/config.xml`

* APP ID
* App Level
* Communication Key
* uart device name (e.g. /dev/ttyUSB0)
* baudrate

>Note: the 'baudrate' set in the `config.xml` needs to be consistent with the setting in the DJI N1 PC assistant.

## Compile

~~~bash
cd cmake
make
~~~

If the compilation is completed, a linux executable file called 'DJI_Onboard_API_Cmdline_Test' will be generated in the `output` dir.

##Run

We recommend you first run this example in the simulator then move to the real flight test. Also, please be aware that you will need sudo privilege to manipulate the linux serial port. You may need to gain the sudo access privilege.

Please make sure that `config.xml` file is sitted in the `output` folder before running. Then run the example by entering the following command:

~~~bash
sudo ./DJI_Onboard_API_Cmdline_Test
~~~

##What You Can Expect

* You can see the flight simulations on screen if you are using the DJI PC simulator. Otherwise, real flight happens.
* You can see the actual 'gimbal and camera' movement.
* You can see the image/video you capture from your Mobile Device.

ENJOY your flight!

---

#DJI Onboard SDK Linux 终端例程

##简介

这是一份运行在Linux终端下的实例程序。开发者可以参考此程序来理解和实现对飞机的控制指令，例如：

* 激活 Matrice100（以下简称 M100）
* 获取 M100 的控制权
* 释放 M100 的控制权
* 向 M100 发送起飞指令
* 向 M100 发送降落指令
* 向 M100 发送返航指令
* 对 M100 进行姿态控制
* 对 M100 进行云台角度控制
* 向 M100 发送相机控制指令
* 查询 M100 当前状态信息

开发者可以在 Linux 终端中运行程序，来测试上述功能。

##目录架构

* cmake: Makefile文件与编译器生成文件
* output：编译后得到的可执行文件
* src：代码源文件
* src/DJI_LIB：供用户参考的 Onboard SDK 库
* README.md：此说明文档

##编译和运行环境

此例程在以下系统环境中进行了测试：

* Ubuntu 12.04 操作系统
* G++ 版本 4.6.3

##硬件安装

* 开发者需要通过 USB 转 TTL 连接线（需另行购买）连接机载设备与 M100 来进行数据通信。
* 为了更好的监控 M100 当前状态，开发者可以在移动设备上运行 DJI GO 来查看 M100 当前信息。

##程序配置

将以下信息输入至 `./output/config.xml`

	* APP ID （在官网注册key后得到）
	* API Level （key对应的 API 权限等级）
	* Communication Key（在官网注册key后得到）
	* Uart Device Name（串口设备名称）
	* Baudrate（比特率）

	> 注意：比特率需要同 N1 调参中设置的比特率保持一致。

##编译

~~~bash
cd cmake 
make
~~~

编译完成后，一个 Linux 系统下的可执行文件 `DJI_Onboard_API_Cmdline_Test`将会出现在 `output`文件夹下。

##运行

确保 `config.xml` 文件中的配置信息无误后，在`output`文件夹中运行如下命令即可运行程序：

~~~bash
sudo ./DJI_Onboard_API_Cmdline_Test
~~~

我们建议用户首先在模拟器中尝试我们的程序再进行实际飞行测试。

##运行结果：

* 开发者可以通过程序实现对飞机的动作控制。
* 开发者可以通过程序实现对飞机云台的姿态控制。
* 开发者可以在移动设备上用 DJI GO 查看通过程序拍摄的照片和视频。