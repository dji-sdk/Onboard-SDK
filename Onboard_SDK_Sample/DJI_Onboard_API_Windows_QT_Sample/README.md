#DJI Onboard SDK Windows QT Sample
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
* The data transparent transmission
* The status query

Developers can play with this example via the GUI interaction interface in Windows.

##Directory Structure
* exe: executables, dependent files are also included
* src: source code
* src/DJI_LIB: the DJI Onboard SDK API Library
* README.md: this file

##Compile & Run Environment
The below environment has been tested.
* Operating System: Windows 7 64Bit  
* Integrated Development Environment (IDE): Qt Creator 3.3.1  
* Qt Ver: Qt 5.3.2 MinGW 32bit  

## Hardware Installation
* In order to communicate with the N1 Autopilot via the DJI OPEN protocal, a physical connection between your Onboard Device and the N1 Autopilot is required with a USB to TTL serial cable (SOLD Seperately).
* In order to monitor & control the flight, a remote controller connects to your Mobile Device (with the DJI GO APP running) is needed.

##Configs
1. Open `DJI_Onboard_API_Windows_QT_Sample.pro` locates at `src` sub-directory in the DJI_Onboard_API_Windows_QT_Sample*.

2. Select the `Desktop Qt 5.3 MinGW 32bit` kit and click `Configure Project` to proceed.

3. During the init process, enter the following info into the GUI interface including:

* APP ID
* API Level
* Communication Key
* Uart Device Name (e.g. COM1, COM2)
* Baudrate

>Note: the 'baudrate' needs to be consistent with the setting in the DJI N1 PC assistant.

##Compile
Click the hammer icon on the bottom left in the Qt Creator IDE to start compiling.

##Run
Click the green triangle icon on the bottom left corner to run the sample.
We recommend you first run this example in the simulator then carefully move to the real flight test.

##What You Can Expect
* You can see the flight control simulations on screen if you are using the DJI PC simulator. Otherwise, real flight happens.
* You can see the actual 'gimbal and camera' movement.
* You can see the image/video you capture from you Mobile Device.

ENJOY your flight!

---

#DJI Onboard SDK WIndows QT 例程
##简介
这是一份利用 QT 实现的运行在 Windows 系统下的示例程序。开发者可以参考此程序来理解和实现对飞机的控制指令，例如：

* 激活 Matrice100 （以下简称M100）
* 获取 M100 的控制权
* 释放 M100 的控制权
* 向 M100 发送起飞指令
* 向 M100 发送降落指令
* 向 M100 发送返航指令
* 对 M100 进行姿态控制
* 对 M100 进行云台角度控制
* 向 M100 发送相机控制指令
* 使用 M100 的数据透传功能
* 查询 M100 当前状态信息

用户可以通过 QT 的 GUI 界面与程序进行交互，来测试上述功能。

##目录架构
* exe: 可执行文件与 dll 依赖文件
* src：代码源文件
* src/DJI_LIB：供用户参考的 Onboard SDK 库
* README.md：此说明文档

##编译和运行环境
此例程在以下系统环境中进行过测试：
* Windows 7 64位操作系统
* IDE：QT Creator 3.3.1
* Qt版本：QT 5.3.2 MinGW 32位

##硬件安装
* 开发者需要通过 USB 转 TTL 连接线（需另行购买）连接机载设备与 M100 来进行数据通信。
* 为了更好的监控 M100 当前状态，开发者可以通过移动设备运行 DJI GO 来查看 M100 当前信息。

##程序配置
1. 在QT中打开位于`src`文件夹下的`DJI_Onbard_API_Windows_QT_Sample.pro` 工程文件。
2. 选择 `Desktop Qt 5.3 MinGW 32bit` 并且单击 `Configure Project` 来进入下一步。
3. 在初始化过程中，用户需要将下列内容输入 GUI 界面信息中：

* APP ID （在官网注册key后得到）
* API Level （key对应的 API 权限等级）
* Communication Key（在官网注册key后得到）
* Uart Device Name（串口设备名称）
* Baudrate（比特率）

> 注意：比特率需要同 N1 调参中设置的比特率保持一致。

##编译

点击 QT IDE 中的编译按钮来编译项目。

##运行

点击 QT IDE 中的运行按钮来运行程序。

我们建议用户首先在模拟器中尝试我们的程序再进行实际飞行测试。

##运行结果

* 开发者可以通过程序实现对飞机的动作控制。
* 开发者可以通过程序实现对飞机云台的姿态控制。
* 开发者可以在移动设备上用 DJI GO 查看通过程序拍摄的照片和视频。