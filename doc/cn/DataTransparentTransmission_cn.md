#### 本文档已停止维护, 请移步到DJI开发者官网的 <https://developer.dji.com/onboard-sdk/documentation/introduction/data-transparent-transmission.html> 查看最新版本. 

# 数据透传

---

## 数据透传功能介绍

在本文档中，我们将对移动设备和机载设备之间的数据透传功能做一个整体介绍，包括开发这一功能的主要目的、数据透传的使用方法，以及相关的示例代码。

我们推荐开发者先阅读[Onboard SDK开放协议](OPENProtocol_cn.md)，了解数据透传功能Onboard SDK部分的指令发送方法，以及了解Moblie SDK的使用，然后在阅读本说明文档

开发者需注意移动设备透传数据给机载设备的通信带宽约 1KB/s，机载设备透传数据给移动设备的通信带宽约为 8KB/s。

### 数据透传功能的开发目的

DJI 为开发者提供了两种功能完善的 SDK 帮助开发飞行应用：Mobile SDK 和 Onboard SDK。Mobile SDK允许开发者基于iOS/Android编写相应的移动端应用以控制飞行器，而Onboard SDK则允许开发者基于 Windows/Linux 编写相应的 PC 端应用，或直接利用单片机等其他计算设备编写相应控制程序，并将相应的计算设备挂载到飞行器上，通过串口直接控制飞行器。

开发者可以从机载设备端向移动设备发送数据，以便将机载设备上的数据发至移动设备显示或开发者监控程序运行等;也可以从移动设备端向机载设备发送数据，用以控制机载设备端的程序运行等。

数据透传功能可以作为 Mobile SDK 与 Onboard SDK 之间的通信桥梁，方便开发者最大限度的实现自己的定制开发。

![streamFrame](Images/streamFrame.png)

## 机载设备透传数据至移动设备

### Onboard SDK相关命令

**命令集 0x00 命令码 0xFE 透传数据（机载设备至移动设备）**
<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>说明</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>1~100</td>
  <td>用户自定义数据，带宽约 8KB/s</td>
</tr>

<tr>
 <td >应答值</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>

</table>

### Mobile SDK相关命令

- iOS

请按照如下方式实现DJIFlightControllerDelegate:

~~~objc
- (void)flightController:(DJIFlightController *)fc didReceiveDataFromExternalDevice:(NSData *)data;
~~~

具体细节请参阅**DJIFlightController.h**文件

- Android

请按照如下方式实现回调函数 `FlightControllerReceivedDataFromExternalDeviceCallback`:

~~~java
DJIAircraft mAircraft = (DJIAircraft)DJISDKManager.getInstance().getDJIProduct();
DJIFlightController mFlightController = mAircraft.getFlightController();

mFlightController.setReceiveExternalDeviceDataCallback(new FlightControllerReceivedDataFromExternalDeviceCallback() {         
          @Override
          public void onResult(byte[] data) {
          }
        });
~~~

具体细节请参阅**FlightController**类

## 移动设备透传数据至机载设备

### Mobile SDK相关命令

- iOS

请按如下方式实现 DJIFlightController:

~~~objc
- (void)sendDataToOnboardSDKDevice:(NSData *)data withCompletion:(DJICompletionBlock)completion;
~~~

具体细节请参阅**DJIFlightController.h**文件

- Android

请按如下方式实现 DJIFlightController 中的`sendDataToOnboardSDKDevice`:

~~~java
DJIAircraft mAircraft = (DJIAircraft)DJISDKManager.getInstance().getDJIProduct();
DJIFlightController mFlightController = mAircraft.getFlightController();

mFlightController.sendDataToOnboardSDKDevice(data,
                new DJICompletionCallback() {
                    @Override
                    public void onResult(DJIError pError) {
                    }
                });
~~~

具体细节请参阅**FlightController**类

### Onboard SDK相关命令
**命令集 0x02 命令码 0x02 透传数据（移动设备至机载设备）**

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>说明</th>
</tr>

<tr>
  <td >命令值</td>
  <td>0</td>
  <td>1~100</td>
  <td>用户自定义数据，带宽约 1KB/s</td>
</tr>

<tr>
  <td >应答值</td>
  <td>---</td>
  <td>---</td>
  <td>无应答值</td>
</tr>

</table>
