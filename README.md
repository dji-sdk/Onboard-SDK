# DJI Onboard SDK

[![Join the chat at https://gitter.im/dji-sdk/Onboard-SDK](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/dji-sdk/Onboard-SDK?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

## What is the DJI Onboard SDK?

DJI's Onboard SDK allows you to connect your own Onboard Embedded System (OES) to a supported DJI vehicle [Matrice 100](http://www.dji.com/product/matrice100) or [Matrice 600](http://www.dji.com/product/matrice600) or flight controller [A3](http://www.dji.com/product/a3) using a common serial port (TTL UART). This setup opens up an exciting opportunity to integrate your own hardware with DJI's flying platforms.  New applications and commercial uses for aerial robotics awaits and we can't wait to see what you build!

---
## New Major Release

A new major version of DJI Onboard SDK (v3.2.0) was released on 12/23. This version brings all the advanced features first unveiled at the [DJI Airworks](http://www.dji.com/newsroom/news/dji-enterprise-launches-airworks-conference) conference. Be sure to read the [release notes](https://developer.dji.com/onboard-sdk/documentation/appendix/releaseNotes.html)! If you're new here, please read the rest of this document.

---

## Prerequisites

This SDK is for developers with:

- programming experience in C and C++
- embedded systems knowledge
- a DJI [Matrice 100](http://www.dji.com/product/matrice100) vehicle, a DJI [Matrice 600](http://www.dji.com/product/matrice600) vehicle, or DJI [A3](http://www.dji.com/product/a3) flight controller integrated into your own vehicle
- your own Onboard Embedded System (OES) with an available com port (TTL UART)
- Windows PC to run the required software tools
- an iOS or Android mobile device to run DJI Go
- (optional) an iOS device to run the [DJI Mobile-Onboard SDK](https://github.com/dji-sdk/Mobile-OSDK-iOS-App) App

## Get Started Immediately

Developers can follow the [Get Started Guide](https://developer.dji.com/onboard-sdk/documentation/quick-start/index.html) to get basic understanding of Onboard SDK key features, hardware setup, registration process and run the examples to see how the DJI Onboard SDK can be used.

## Hardware Setup Guide

This guide will help you connect your onboard embedded system (OES) with the M100 vehicle, M600 vehicle, or A3 flight controller. 

For more details, please visit the [Hardware Setup Guide](https://developer.dji.com/onboard-sdk/documentation/hardware-setup/index.html) in DJI Developer Website.

## Application Development Guides

Please visit [Programming Guide](https://developer.dji.com/onboard-sdk/documentation/application-development-guides/programming-guide.html) and [Ground Station Programming Guide](https://developer.dji.com/onboard-sdk/documentation/application-development-guides/ground-station-programming-guide.html) for more details.

## Quick Reference

### Introduction

- [Architecture Guide](https://developer.dji.com/onboard-sdk/documentation/introduction/architecture-guide.html)

- [Things to Know](https://developer.dji.com/onboard-sdk/documentation/introduction/things-to-know.html)

### Protocol Documents

- [OPEN Protocol](https://developer.dji.com/onboard-sdk/documentation/introduction/index.html) 

- [Data Transparent Transmission](https://developer.dji.com/onboard-sdk/documentation/introduction/data-transparent-transmission.html)  

- [Ground Station Protocol](https://developer.dji.com/onboard-sdk/documentation/introduction/ground-station-protocol.html)  

- [Virtual RC Protocol](https://developer.dji.com/onboard-sdk/documentation/introduction/virtual-rc-protocol.html)  

### Platform Guides

- [QT Example](https://developer.dji.com/onboard-sdk/documentation/github-platform-docs/PureQT/README.html)

- [STM32 Example](https://developer.dji.com/onboard-sdk/documentation/github-platform-docs/STM32/README.html)

- [Linux Example](https://developer.dji.com/onboard-sdk/documentation/github-platform-docs/Linux/README.html)

- [ROS Example](https://developer.dji.com/onboard-sdk/documentation/github-platform-docs/ROS/README.html)

### Appendix

- [Release Notes for Onboard SDK 3.2.0](https://developer.dji.com/onboard-sdk/documentation/appendix/releaseNotes.html)

- [FAQ](https://developer.dji.com/onboard-sdk/documentation/appendix/FAQ.html) & [Appendix](https://developer.dji.com/onboard-sdk/documentation/appendix/index.html)

- [Doxygen](doc/doxygen-doc/html/index.html) is available (beta) for code documentation of the core library. 

## Support

You can get support from DJI with the following methods:

- Send email to dev@dji.com describing your problem and a clear description of your setup
- Post questions in [**Stackoverflow**](http://stackoverflow.com) using [**dji-sdk**](http://stackoverflow.com/questions/tagged/dji-sdk) tag
- Github Issues or [gitter.im](https://gitter.im/dji-sdk/Onboard-SDK)
- [**DJI Forum**](http://forum.dev.dji.com/en)