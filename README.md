# DJI Onboard SDK (OSDK) 4.0.1

[![Join the chat at https://gitter.im/dji-sdk/Onboard-SDK](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/dji-sdk/Onboard-SDK?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

## What is the DJI Onboard SDK?

The DJI Onboard SDK allows you to connect your own Onboard Computer to a [supported](https://developer.dji.com/onboard-sdk/documentation/purchaseguide/hardware.html) DJI vehicle or flight controller using a serial port (TTL UART). For full documentation, please visit the [DJI Developer Documentation](https://developer.dji.com/onboard-sdk/documentation/). Documentation regarding the code can be found in the [OSDK API Reference](https://developer.dji.com/onboard-api-reference/index.html) section of the developer's website.

## Latest Release
OSDK 4.0.1 was released on 21 August 2020. This version mainly fixes OSDK 4.0.0 issues, such as camera stream related problems, download function optimization, MOP optimization, waypoint 2.0 problems repair, etc. At the same time, we also add an automatic code porting script to assist the porting of FreeRTOS.

## Last Major Release
OSDK 4.0.0 was released on 8 May 2019. New APIs about flight controller such as home point setting, confirm landing, some function switches are supported in the version. For the payload device, more APIs about camera functions and parameters are added in the version.  Please see the [release notes](https://developer.dji.com/onboard-sdk/downloads/) for more information.

## Firmware Compatibility

This chart shows the latest firmware that were available and are supported at the time of 4.0.0 release.

<table>
<thead>
<tr><th>Drone</th>
<th>Drone's Firmware</th>
<th>Controller's Firmware</th>
</tr></thead>
<tbody>
<tr>
<td>M300 RTK </td>
<td>01.00.02.11</td>
<td>3.4.8.56</td>
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

- **Send E-mail to dev@dji.com** describing your problem and a clear description of your setup
- Post questions on Developer Forum
  * [DJI SDK Developer Forum(Cn)](https://bbs.dji.com/forum-79-1.html?from=developer)
  * [DJI SDK Developer Forum(En)](https://forum.dji.com/forum-139-1.html?from=developer)
- Github Issues or [gitter.im](https://gitter.im/dji-sdk/Onboard-SDK)
- Post questions on [**Stackoverflow**](http://stackoverflow.com) using [**dji-sdk**](http://stackoverflow.com/questions/tagged/dji-sdk) tag

