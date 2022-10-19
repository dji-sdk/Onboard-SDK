# DJI Onboard SDK (OSDK) 4.1.0

[![Join the chat at https://gitter.im/dji-sdk/Onboard-SDK](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/dji-sdk/Onboard-SDK?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

## What is the DJI Onboard SDK?

The DJI Onboard SDK allows you to connect your own Onboard Computer to a [supported](https://developer.dji.com/onboard-sdk/documentation/purchaseguide/hardware.html) DJI vehicle or flight controller using a serial port (TTL UART). For full documentation, please visit the [DJI Developer Documentation](https://developer.dji.com/onboard-sdk/documentation/). Documentation regarding the code can be found in the [OSDK API Reference](https://developer.dji.com/onboard-api-reference/index.html) section of the developer's website.

## Latest Release
OSDK 4.1.0 was released on 2 February 2021.This version adds the USB reconnection function, provides some basic interface of flightcontroller and camera, and verifies that the battery module partially support the M300. Fixed some problems in waypoint V2, camera image decoding, camera file download and MOP functions. At the same time, optimized some implementations in flightcontroller and activation. For ROS, most of the interfaces included in OSDK lib but not included in ROS are added.

## Last Major Release
OSDK 4.0.1 was released on 21 August 2020. This version mainly fixes OSDK 4.0.0 issues, such as camera stream related problems, download function optimization, MOP optimization, waypoint 2.0 problems repair, etc. At the same time, we also add an automatic code porting script to assist the porting of FreeRTOS.

## Firmware Compatibility

This chart shows the latest firmware that were available and are supported at the time of 4.1.0 release.

<table>
<thead>
<tr><th>Drone</th>
<th>Drone's Firmware</th>
<th>Controller's Firmware</th>
</tr></thead>
<tbody>
<tr>
<td>M300 RTK </td>
<td>02.02.0102</td>
<td>3.4.8.69</td>
</tr>
<tr>
<td>M210 RTK V2</td>
<td>01.00.0710</td>
<td>3.4.3.44</td>
</tr>
<tr>
<td>M210 V2</td>
<td>01.00.0710</td>
<td>3.4.3.44</td>
</tr>

</tbody>
</table>

## Support

You can get support from DJI and the community with the following methods:

- Post questions on Developer Forum
  * [DJI SDK Developer Forum(Cn)](https://djisdksupport.zendesk.com/hc/zh-cn/community/topics)
  * [DJI SDK Developer Forum(En)](https://djisdksupport.zendesk.com/hc/en-us/community/topics)
- Submit a request describing your problem on Developer Support
  * [DJI SDK Developer Support(Cn)](https://djisdksupport.zendesk.com/hc/zh-cn/requests/new)
  * [DJI SDK Developer Support(En)](https://djisdksupport.zendesk.com/hc/en-us/requests/new)
- Github Issues or [gitter.im](https://gitter.im/dji-sdk/Onboard-SDK)
- Post questions on [**Stackoverflow**](http://stackoverflow.com) using [**dji-sdk**](http://stackoverflow.com/questions/tagged/dji-sdk) tag

