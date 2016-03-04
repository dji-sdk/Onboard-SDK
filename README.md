# DJI Onboard SDK Main Page | 主页

[![Join the chat at https://gitter.im/dji-sdk/Onboard-SDK](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/dji-sdk/Onboard-SDK?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

If you are not new here and worked on our 2.3 firmware before, you are suggested to read [the release notes](ReleaseNotes.md)

如果你是从2.3固件升级至3.1固件的开发者，我们建议你首先阅读[版本发布记录](ReleaseNotes.md)

[English](#welcome-onboard-sdk-developers) | [中文](#欢迎onboard-sdk开发者)

## Welcome, Onboard SDK developers! 

### Via this document, you can:
- Get a general understanding of the Onboard SDK in a short time
- Understand the Onboard SDK APPs development paradigm
- Get some best practice tips while getting your hands dirty

### However, we assume you having the following basic requirements already:
- You are passionated to be an UAV APP developer
- Some basic programming experience in C/C++ 
- Some basic flight control knowledge like the concepts of Pitch, Roll and Yaw
- One complete set of DJI developer flight platform* and one USB-TTL serial cable (SOLD seperately)


## Introduction

The Onboard SDK allows developers to communicate with the Autopilot from any Onboard Devices through the serial port interface. Via the communication between the Onboard Device (SOLD seperately) and the Autopilot, developers can easily

- Write their own UAV APPs on top of the Onboard Device in his/her favourite programming languages
- Dispatch the 'flight' job to the Autopilot following the DJI Onboard OPEN protocal


### Key Features

- Flexible Flight Control

  Different flight control modes such as position, velocity and attitude.
  
- Diverse Monitoring Data

  Flight data are diverse and can be obtained easily.
  
- Programmable Design

  Flight mode control and flight data are designed to aid autonomous flight control & navigation.

- Data Trasparent Transmission

  The data of Onboard Device can be transmitted to your Mobile Device in real time and vice versa.

### System Architecture & Recommended Development Paradigm
Two core components of the system architecture are the Autopilot and an Onboard Device. They are physically connected via the serial port interface. 

Since the Onboard OPEN protocal is designed to be opened thoroughly, experienced developers can directly:

1. Write some low-level logics inside their APPs to contruct the underlying communication bit sequence;
2. Handle the package loss & resending mechanisms with the benifit of enjoying complete freedom.

While for beginners, they can just comfortably use our Onboard SDK API Library to communicate with the Autopilot, all they need to do is to call our APIs.


## Environment Setup

### Software Installation

#### Step1:

If you want to use our Onboard SDK API Library which have been used in our examples, you can download it from Github.

#### Step2: 

In order to develop your Apps via the Onboard SDK, you need to download the following DJI specific latest tools: 

1. the DJI PC assistant software and the related Driver
2. the DJI PC Simulator
3. the DJI GO APP
     
>Note:   
>For 1 & 2, please download them from: https://developer.dji.com/onboard-sdk/downloads/    
>For 3, download & install them from the iOS/Android APP STORE

### Hardware Installation

#### Step1:

In order to use the DJI PC Simulator & DJI PC assistant, a physical connection between your computer and Autopilot is required with a USB to Micro-USB cable.

#### Step2: 

In order to communicate with the Autopilot via the Onboard OPEN protocal, a physical connection between your computer and Autopilot is required with a USB to TTL full-duplex serial cable (SOLD Seperately).

>Note:   
*  The serial level of Autopilot should be 3.3 V.
*  Serial communication requires the use of full-duplex mode.
   
### Onboard SDK APP Registration & Flight Platform Activation

Since the Onboard SDK allows developers to develop programmable UAV APPs, a more serious Onboard SDK APP registration and Flight Platform Activation has been introduced.

#### Step1: 

For Onboard SDK APP Registration, please go to https://developer.dji.com/register/

#### Step2: 

For Flight Platform Activation, please go to [Activation Guide][Activation]


### The prioritization sequence for different control sources

For now, the UAV can be controlled by (1) Remote Controller (2) Mobile Device and (3) Onboard Device. The prioritization sequence is set to be (1) > (2) > (3).

The remote controller always enjoys the top priority for the UAV control. The Autopilot can enter the API Control Mode (Programmable Mode) if the following 2 conditions are met:

* The 'enable API control' box is checked in the assistant software.
* The mode selection bar of the remote controller is placed at the F position.

Once the above conditions are met, developers can call the related 'flight control request function' to request the flight control of UAV.


## Closing Remarks

### Dear Onboard SDK developers, you are all set now!

Here, we list all the avaliable document pointers for your reference and we recommend you the following development steps.

1. Try some examples in our 'Compile and Run Example Section'
2. Learn from those examples and start building your own UAV APPs

If you encounter any questions during the development, take a look at our [FAQ][FAQ]. After that, feel free to contact us.

### Safety Warnings:

Please comply with the local regulations during the development process and the flight. Please refer to http://flysafe.dji.com/ for more.

### Compile & Run Example Section:

- [QT example, both Windows and Linux](sample/PureQT)

- [Command Line example, both Windows and Linux](sample/commandline)

- [Linux ROS example](https://github.com/dji-sdk/Onboard-SDK-ROS)

- [STM32 example](sample/STM32)

### References Document:

- [OPEN Protocol][0] & [Appendix][1] 

- [Data Transparent Transmission][2]  

- [Programming Guide][3]

- [Ground Station Protocol][4]

- [Virtual RC][5]
 
- [Ground Station Programming Guide][6]

- [Activation Guide][Activation]

- [FAQ][FAQ]


[0]:doc/en/OPENProtocol.md  
[1]:doc/en/Appendix.md  
[2]:doc/en/DataTransparentTransmission.md   
[3]:doc/en/ProgrammingGuide.md  
[4]:doc/en/GroundStationProtocol.md  
[5]:doc/en/VirtualRCProtocol.md 
[6]:doc/en/GroundStationProgrammingGuide.md  

[FAQ]:doc/en/FAQ_en.md   
[Activation]:doc/en/ActivationGuide.md

---

## 欢迎，Onboard SDK开发者！

### 通过阅读本文档，您将能够：

* 在短时间内对Onboard SDK有整体了解
* 明白Onboard SDK应用开发流程
* 在进行Onboard SDK开发的同时获取有用的编程提示

### 在这里，我们认为您：
* 对无人机应用开发富有热情
* 具有一些基础的C/C++编程经验
* 了解基础的飞行控制知识如横滚 (Roll)、俯仰 (Pitch)和偏航 (Yaw)
* 拥有一套飞行平台*和一条USB-TTL串口线（需另购）




## 简介

Onboard SDK能够允许开发者选用任意合适的机载设备（需另购）与飞控通过串行端口进行通讯。开发者能够容易的：

- 在机载设备上使用任意编程语言设计并实现自己的飞行应用
- 将底层飞行控制交由飞控完成，让开发者只需关注应用本身的逻辑



### Onboard SDK 特性

- 灵活的飞行控制  
    包括姿态、速度及位置的多种飞行控制模式
- 丰富的推送数据  
    包括姿态、速度等十余种飞行数据
- 可编程的设计  
    通过编程方式实现自主飞行、导航等
- 透明的数据传输（下简称数据透传）
    机载设备及机载外部设备的数据可通过某特定链路回传至移动设备，反之亦然



### 系统架构及推荐开发流程

两个系统的重要组成部分是飞控和机载设备，它们通过串口进行物理连接并遵循DJI OPEN Protocal以进行通讯。 

因为飞控与机载设备间的通讯协议是完全开放的，有经验的开发者在享受完全自由的同时，也需要在飞行应用中加入底层逻辑以实现通讯数据比特流的构建，丢包重发机制的实现等。而对于普通开发者，DJI则提供了Onboard SDK API库，开发者仅需要调用相应API函数，即可完成与飞控之间的通讯，藉由飞控完成对无人机的飞行控制，以简化开发流程和步骤。

##环境设置

### 软件安装

#### 步骤1： 

如果您想使用我们的Onboard SDK API库（现在的库只包含于例子中），您可以从我们的官方Github中下载

#### 步骤2：

为了简化Onboard SDK应用开发流程，我们推荐下载以下DJI最新的专属工具软件

1. DJI 调参软件及驱动程序
2. DJI PC模拟器
3. DJI GO APP

>备注:  
>对于1 & 2, 可以在https://developer.dji.com/onboard-sdk/downloads/ 下载并安装。  
>对于3, 可以在iOS/Android 应用程序商店中下载并安装。

### 硬件安装

#### 步骤1： 

用一条USB-Micro USB线缆连接您的电脑与飞控，以使用DJI PC模拟器及DJI PC调参软件。

#### 步骤2：

用一条USB-TTL全双工串口线缆（需另购）连接您的电脑与飞控，以通过Onboard SDK开放协议进行通讯。

>备注：  
* 飞控的串口电平为3.3 V。  
* 串口需采用全双工方式通讯。  

### 注册Onboard SDK APP和激活飞行平台
因为Onboard SDK允许开发者开发无人机飞行应用，因此DJI对注册Onboard SDK APP和激活飞行平台有更为严格的规定。

#### 步骤1： 

关于APP注册的更多内容，请访问 https://developer.dji.com/register/

#### 步骤2： 

关于激活飞行平台的更多内容，请访问[激活指南][cnActivation]

### 三种控制信号源的优先级

到目前为止，无人机可以被 (1)遥控器 (2)移动设备和 (3)机载设备所控制。控制优先权由内部系统设定为：(1) > (2) > (3)。

遥控器被定义为控制权限最高的设备。当以下3个条件均满足时，飞行控制可以切换到API控制模式（即可编程模式）：

* 在PC assistant调参软件中，“启用API控制”勾选框已被勾选
* 在DJI GO App中，IOC模式已被关闭
* 遥控器的模式选择开关置于F档

满足以上三个条件后，开发者即可用“获取控制权”的相关命令获取无人机的飞行控制权。

## 结语

### 尊敬的Onboard SDK开发者，您已经准备好并可以投入到飞行应用的实际开发中了！
在这里，我们列出所有可供您参考的文档。通过阅读这些文档，能使您的开发之路更为顺畅。以下是我们推荐您的开发步骤：

1. 编译并运行Onboard SDK提供的一些示例
2. 把示例作为您学习的例子，继而创建您自己专属的飞行应用

如您在开发过程中有任何疑问，请首先参阅我们的[FAQ][cnFAQ]部分，然后再与我们取得联系。

###安全声明:

真实无人机飞行请遵循当地法律法规，更多信息请访问 http://flysafe.dji.com/cn

### 编译与运行示例:

- [QT example, both Windows and Linux](sample/PureQT)

- [Command Line example, both Windows and Linux](sample/commandline)

- [Linux ROS example](https://github.com/dji-sdk/Onboard-SDK-ROS)

- [STM32 example](sample/STM32)

### 参考文档:

- [开放协议][cn0] & [附录][cn1]  
  
- [数据透传][cn2]   
  
- [编程指南][cn3]  

- [地面站功能][cn4]   

- [地面站编程指南][cn6]   

- [虚拟遥控][cn5]   

- [激活指南][cnActivation]
  
- [FAQ][cnFAQ]


[cn0]:doc/cn/OPENProtocol_cn.md  
[cn1]:doc/cn/Appendix_cn.md  
[cn2]:doc/cn/DataTransparentTransmission_cn.md
[cn3]:doc/cn/ProgrammingGuide_cn.md
[cn4]:doc/cn/GroundStationProtocol_cn.md
[cn5]:doc/cn/VirtualRCProtocol_cn.md
[cn6]:doc/cn/GroundStationProgrammingGuide_cn.md
[cnActivation]:doc/cn/ActivationGuide_cn.md
[cnFAQ]:doc/cn/FAQ_cn.md
