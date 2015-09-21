# DJI Onboard SDK

[Chinese Version](README_CN.md)  

[FAQ](Onboard_API_Doc/en/FAQ.md)  
[Glossary](Onboard_API_Doc/en/Glossary.md)  

---

## Welcome, Onboard SDK developers! 

### Via this document, you can:
- Get a general understanding of the Onboard SDK in a short time
- Understand the Onboard SDK application development paradigm
- Get some best practice tips while getting your hands dirty

### However, we assume you having the following basic requirements already:
- You are passionated to be an UAV APP developer
- Some basic programming experience in C/C++ 
- Some basic flight control knowledge like the concepts of Pitch, Roll and Yaw
- One complete set of UAV* flight platform and one USB-TTL serial cable (SOLD seperately)

> Note: For now, ONLY M100 flight platform is supported.

---

## Introduction

The Onboard SDK allows developers to communicate with the N1 Autopilot* from any Onboard Devices through the serial, specifically the UART interface. Via the communication between the Onboard Device*(SOLD seperately) and the N1 Autopilot, developers can easily

- Write their own Flight APPs on top of the Onboard Device in his/her favourite programming languages
- Dispatch the 'flight' job to the N1 Autopilot following the DJI Onboard OPEN protocal

>Note: *N1 Autopilot is the flight controller of the M100 filght platform*

### Key Features

- Flexible Flight Control
  Different flight control modes such as position, velocity and attitude are included.
  
- Diverse Monitoring Data
  Flight data are diverse and can be obtained easily.
  
- Programmable Design
  Flight mode control and flight data are designed to aid autonomous flight control & navigation.

- Data Trasparent Transmission
  The computing results of your Onboard Device can be transmitted to your Mobile Device in real time.

### System Architecture & Recommended Development Paradigm
Two core components of the system architecture are the N1 Autopilot and an Onboard Device. They are physically connected via the serial(UART) interface. 

Since the Onboard OPEN protocal is designed to be opened thoroughly, experienced developers can directly write some low-level logics inside their APPs to (1) Contruct the underlying communication bit sequence;(2) Handle the package loss & resending mechanisms with the benifit of enjoying complete freedom. While for beginners, they can just comfortably use our 'official but EXPERIMENTAL' Onboard SDK Library to communicate with the N1 Autopilot, all they need to do is to call our powerful APIs.


---

## Environment Setup

### Software Installation

#### Step1:

If you want to use our Onboard SDK API Library, you can download it from Github

#### Step2: 

In order to develop your Apps via the Onboard SDK, you need to download the following DJI specific tools: 

1. DJI N1 PC assistant software and the corresponding N1 Driver
2. DJI PC Simulator
3. DJI GO APP
     
>Note: 
>For 1 & 2, please download them from: https://developer.dji.com/onboard-sdk/downloads/
>
For 3, download & install them from the iOS/Android APP STORE.*

### Hardware Installation

#### Step1:

In order to use the DJI PC Simulator & DJI N1 PC assistant, a physical connection between your computer and N1 Autopilot is required with a USB to Micro-USB cable.

#### Step2: 

In order to communicate with the N1 Autopilot via the Onboard OPEN protocal, a physical connection between your computer and N1 Autopilot is required with a USB to TTL Serial cable(SOLD Seperately).

### Onboard APP Registration & Flight Platform Activation

Since the Onboard SDK allows developers to develop programmable UAV APPs beyond line-of-sight, a more serious Onboard APP registration and Flight Platform Activation has been introduced.

#### Step1: 

For Onboard APP Registration, please go to (https://developer.dji.com/register/)

#### Step2: 

For Flight Platform Activation, please go to (http://bbs.dji.com/thread-21892-1-1.html)


### The prioritization sequence for different control sources

For now, the UAV can be controlled by (1) Remote Controller (2) Mobile Device and (3) Onboard Device. The prioritization sequence is set to be (1) > (2) > (3).

>The remote controller(RC for short below) always enjoys the top priority for the control of the UAV. The N1 Autopilot can enter the API Control Mode(Programmable Mode) if the following 3 conditions are met:

>* The 'enable API control' box is checked in the N1 assistant software.
* The IOC mode inside the DJI GO App is off.
* The mode selection bar of the remote controller is placed at the F position.

Once the above conditions are met, developers can call the related 'flight control request function' to request the flight control of UAV.

---

## Closing Remarks

### Welcome again Onboard SDK developers! You are all set now.

Here, we list all the avaliable document pointers for your reference and we recommend you the following development steps.

1. Try some examples in our 'Compile and Run Section'
2. Follow some tutorials in our 'Step by Step Section'
3. Start building your own app

If you encounter any questions during the development, take a look at our FAQ section. After that, feel free to contact us.

### Safety Warnings:

Please comply with the local regulations during the development process and flight. Please refer to http://flysafe.dji.com/ for more.

### Compile & Run Sections:

[Linux + ROS example](https://github.com/dji-sdk/Onboard-SDK/tree/master/Onboard_SDK_Sample/DJI_Onboard_API_ROS_Sample/doc)(Chinese)

[Windows QT example](https://github.com/dji-sdk/Onboard-SDK/tree/master/Onboard_SDK_Sample/DJI_Onboard_API_Windows_QT_Sample/doc)(Chinese)

### Step by Step Tutorial Section:

[Linux Command Line](https://github.com/dji-sdk/Onboard-SDK/tree/master/Onboard_SDK_Sample/DJI_Onboard_API_Cmdline_Sample/doc)(Chinese)

### API References Section:
[Onboard API References](https://github.com/dji-sdk/Onboard-SDK/tree/master/Onboard_API_Doc/en/1)(English)

[Data Transparent Transmission](https://github.com/dji-sdk/Onboard-SDK/tree/master/Onboard_API_Doc/en/2)(English)

[Onboard SDK Flight Control Explain](https://github.com/dji-sdk/Onboard-SDK/tree/master/Onboard_API_Doc/en/3)(English)

### Articles contributed by Onboard SDK external developers:
[Onboard Device Selection Tips](http://bbs.dji.com/forum.php?mod=viewthread&tid=21106&extra=page%3D1%26filter%3Ddigest%26digest%3D1)(Chinese)

[Use STM32 to take control of M100](http://bbs.dji.com/forum.php?mod=viewthread&tid=19754&extra=page%3D1%26filter%3Ddigest%26digest%3D1)(Chinese)

(TODO: The M100 & Onboard SDK FAQ will be a seperate doc apart from the quickstart version in the gitlab. Thanks.)
(TODO: The activation article from the forum needs to be put back into the doc file)
(TODO: Communicate with Ben for two things:(1)the author of the activation article in the forum (2)The time which the new forum will be in place.)

