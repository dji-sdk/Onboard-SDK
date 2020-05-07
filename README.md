# DJI Onboard SDK (OSDK) 4.0

[![Join the chat at https://gitter.im/dji-sdk/Onboard-SDK](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/dji-sdk/Onboard-SDK?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

## What is the DJI Onboard SDK?

The DJI Onboard SDK allows you to connect your own Onboard Computer to a [supported](https://developer.dji.com/onboard-sdk/documentation/introduction/osdk-hardware-introduction.html#supported-products) DJI vehicle or flight controller using a serial port (TTL UART). For full documentation, please visit the [DJI Developer Site](https://developer.dji.com/onboard-sdk/documentation/). Documentation regarding the code can be found in the [OSDK API Reference](https://developer.dji.com/onboard-api-reference/index.html) section of the developer website.

## Latest Release
OSDK 4.0 was released on 7 May 2020. This version supports M300 and M210/M210 RTK V2, you can try OSDK functions at M300 now. Besides, STM32 platform runs on RTOS and linker layer is refactored. For camera stream, user can get camera H264 stream now by new API. Please see the [OSDK document](https://developer.dji.com/onboard-sdk/documentation/introduction/homepage.html) for more information.

## Last Major Release
OSDK 3.9 was released on 24 September 2019. New APIs about flight controller such as home point setting, confirm landing, some function switches are supported in the version. For the payload device, more APIs about camera functions and parameters are added in the version.  Please see the [release notes](https://developer.dji.com/onboard-sdk/documentation/appendix/releaseNotes.html) for more information.

## Firmware Compatibility

This chart shows the latest firmware that were available and are supported at the time of 4.0 release.

| Aircraft/FC           | Firmware Package Version | Flight Controller Version | OSDK Branch              | Notes                                                                 |
|-----------------------|--------------------------|---------------------------|--------------------------|-----------------------------------------------------------------------|
| **M300            **  | **01.00.01.07**          | **3.4.8.43**              | **4.0**                  |                                                                       |
| **M210/M210 RTK V2**  | **01.00.0670**           | **3.4.80.255**            | **4.0**                  |                                                                       |

## Support

You can get support from DJI and the community with the following methods:

- **Send email to dev@dji.com** describing your problem and a clear description of your setup+

- Github Issues or [gitter.im](https://gitter.im/dji-sdk/Onboard-SDK)
- Post questions on [**Stackoverflow**](http://stackoverflow.com) using [**dji-sdk**](http://stackoverflow.com/questions/tagged/dji-sdk) tag
- [**DJI Forum**](http://forum.dev.dji.com/en)
