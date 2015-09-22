# DJI Onboard   

[English Version](README.md)   

---

## 欢迎,Onboard SDK开发者！

### 通过阅读本文档，您将能够：

* 在短时间内对Onboard SDK有整体了解
* 明白Onboard SDK应用开发流程
* 在进行Onboard SDK开发的同时获取有用的编程提示

### 在这里，我们认为您：
* 对无人机应用开发富有热情
* 具有一些基础的C/C++编程经验
* 了解基础的飞行控制知识如横滚(Roll)、俯仰(Pitch)和偏航(Yaw)
* 拥有一套M100飞行平台和一条USB-TTL串口线（需另购）

> 注意: Onboard SDK目前只支持M100飞行平台。

---

## 简介

Onboard SDK能够允许开发者选用任意合适的机载设备（需另购）与N1飞控*通过异步串行端口（UART）进行通讯。开发者能够容易的：

- 在机载设备上使用任意编程语言设计并实现自己的飞行应用
- 将底层飞行控制交由飞控完成，使开发者只需关注应用本身的逻辑

>Note:*N1飞控目前仅支持M100飞行平台*

### Onboard SDK 特性

- 灵活的飞行控制  
    包括姿态、速度及位置的多种飞行控制模式
- 丰富的推送数据  
    包括姿态、速度等十余种飞行数据
- 可编程的设计  
    通过编程方式实现自主飞行、导航等
- 透明的数据传输*  
    机载设备及外部设备的数据可通过某特定链路回传至移动设备

>Note:*下简称数据透传*

### 系统架构及推荐开发流程

两个系统的重要组成部分是N1飞控和机载设备，它们通过串口进行物理连接并遵循DJI OPEN Protocal以通讯。 

因为N1飞控与机载设备间的通讯协议是完全开放的，有经验的开发者在享受完全自由的同时，也需要在飞行应用中加入底层逻辑以实现(1)通讯数据比特流的构建 (2)丢包重发机制的实现等。而对于普通开发者，DJI则提供了功能强大的Onboard SDK库*，开发者仅需要调用相应API函数，即可完成与飞控之间的通讯，藉由飞控完成对无人机的飞行控制，以此简化开发流程和步骤。

>Note: *目前Onboard SDK库尚为实验版本*

---

##环境设置

### 软件安装

#### 步骤1： 

如果您想使用我们的Onboard SDK API库，您可以从我们的官方Github频道进行下载

#### 步骤2：

为了简化Onboard SDK应用开发流程，我们推荐下载以下DJI专属工具软件

1. DJI N1调参软件及驱动程序
2. DJI PC模拟器
3. DJI GO APP

>Note: 
>对于1 & 2, 请到https://developer.dji.com/onboard-sdk/downloads/下载并安装
>对于3, 可以在iOS/Android APP STORE中下载并安装.*

### 硬件安装

#### 步骤1： 

用一条USB-Micro USB线缆连接您的电脑与N1飞控，以使用DJI PC模拟器及DJI N1 PC调参软件。

#### 步骤2：

用一条USB-TTL串口线缆（需另购）连接您的电脑与N1飞控，以使用Onboard开放协议进行通讯。

### 注册和激活APP
因为Onboard SDK允许开发者开发超视距无人机应用，所以DJI对注册APP及飞行器激活有更为严格的规定。

#### Step1: 

关于注册APP的更多内容，请关注(https://developer.dji.com/register/)

#### Step2: 

关于激活飞行器的更多内容，请关注(http://bbs.dji.com/thread-21892-1-1.html)

### 三种控制信号源的优先级

目前为止，无人机可以被（1）遥控器（2）移动设备和（3）机载设备所控制。控制优先权由内部系统设定为：(1) > (2) > (3)。

>遥控器被定义为控制权限最高的设备。当以下3个条件均满足时，飞行控制可以切换到API控制模式（即可编程模式）：

>* 在PC N1 assistant调参软件中，“启用API控制”勾选框已被勾选
* 在DJI GO App中，IOC模式已被关闭
* 遥控器的模式选择开关已置于F档

满足以上三个条件后，开发者即可用“获取控制权”的相关命令以获取无人机的飞行控制权。

---

## 结语

### 再次欢迎您，Onboard SDK开发者！您已经准备好并可以投入到应用的实际开发中了。
在这里，我们列出所有我们建议可供您参考的文档。通过阅读这些文档，能使您的开发之路更为顺畅。以下是我们推荐您的开发步骤：

1. 编译并运行Onboard SDK提供的一些示例
2. 认真阅读我们的步骤式教程
3. 创建您自己专属的飞行应用

如您在开发过程中有任何疑问，请首先参阅我们的Onboard SDK FAQ部分，然后再与我们取得联系。

###安全声明:

飞行过程中请遵循当地法律法规。更多信息请访问 [http://flysafe.dji.com/cn](http://flysafe.dji.com/cn)

### 编译及运行部分:

[Linux + ROS example](Onboard_SDK_Sample/DJI_Onboard_API_ROS_Sample/)(中文)

[Windows QT example](Onboard_SDK_Sample/DJI_Onboard_API_Windows_QT_Sample/)(中文)

[Linux Command Line](Onboard_SDK_Sample/DJI_Onboard_API_Cmdline_Sample/)(中文)

### 步骤式教程部分:

### API参考部分:

[开放协议说明][0]  
[飞行控制附加说明][1]  
[数据透传说明文档][2]  

### 由外部开发者提供的非官方文档:

[Onboard Device Selection Tips](http://bbs.dji.com/forum.php?mod=viewthread&tid=21106&extra=page%3D1%26filter%3Ddigest%26digest%3D1)(中文)

[Use STM32 to take control of the M100](http://bbs.dji.com/forum.php?mod=viewthread&tid=19754&extra=page%3D1%26filter%3Ddigest%26digest%3D1)(中文)



[0]:Onboard_API_Doc/cn/开放协议说明.md  
[1]:Onboard_API_Doc/cn/飞行控制附加说明.md  
[2]:Onboard_API_Doc/cn/数据透传说明文档.md   
