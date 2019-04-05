# DJI Onboard SDK (OSDK) 3.8

[![Join the chat at https://gitter.im/dji-sdk/Onboard-SDK](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/dji-sdk/Onboard-SDK?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

## What is the DJI Onboard SDK?

The DJI Onboard SDK allows you to connect your own Onboard Computer to a [supported](https://developer.dji.com/onboard-sdk/documentation/introduction/osdk-hardware-introduction.html#supported-products) DJI vehicle or flight controller using a serial port (TTL UART). For full documentation, please visit the [DJI Developer Site](https://developer.dji.com/onboard-sdk/documentation/). Documentation regarding the code can be found in the [OSDK API Reference](https://developer.dji.com/onboard-api-reference/index.html) section of the developer website.

## Latest Release
OSDK 3.8 was released on 3 April 2019. This release adds support of M210 V2, new time synchronization feature, and waypoint mission v2 (beta). Please see the [release notes](https://developer.dji.com/onboard-sdk/documentation/appendix/releaseNotes.html) for more information.


## Last Major Release
OSDK 3.7 was released on 14 Aug 2018. This release adds many new telemetry topics, an emergency stop API and more for the A3, N3 and M210/M210RTK platforms. 
## Firmware Compatibility

This chart shows the latest firmware that were available and are supported at the time of 3.8 release.

| Aircraft/FC           | Firmware Package Version | Flight Controller Version | OSDK Branch            | Notes                                                                 |
|-----------------------|--------------------------|---------------------------|------------------------|-----------------------------------------------------------------------|
| **M210/M210 RTK V2**  | **1.0.0300**             | **3.4.3.24**              | **OSDK 3.8**           |                                                                       |
|                       |                          |                           |                        |                                                                       |
| **M210/M210 RTK**     | **1.1.0913**             | **3.3.10.4**              | **OSDK 3.8**           |                                                                       |
|                       |                          |                           |                        |                                                                       |
| **M600/M600 Pro**     | **1.0.1.66**             | **3.2.41.13**             | **OSDK 3.8**           |                                                                       |
|                       |                          |                           |                        |                                                                       |
| **A3/A3 Pro**         | **1.7.6.0**              | **3.3.8.39**              | **OSDK 3.8**           |                                                                       |
|                       |                          |                           |                        |                                                                       |
| **N3**                | **1.7.6.0**              | **3.3.8.39**              | **OSDK 3.8**           |                                                                       |
|                       |                          |                           |                        |                                                                       |
| **M100**              | 1.3.1.82                 | **3.1.10.0**              | **OSDK 3.8**           |                                                                       |


## Support

You can get support from DJI and the community with the following methods:

- **Send email to dev@dji.com** describing your problem and a clear description of your setup
- Github Issues or [gitter.im](https://gitter.im/dji-sdk/Onboard-SDK)
- Post questions on [**Stackoverflow**](http://stackoverflow.com) using [**dji-sdk**](http://stackoverflow.com/questions/tagged/dji-sdk) tag
- [**DJI Forum**](http://forum.dev.dji.com/en)
