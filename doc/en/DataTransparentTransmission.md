# Data Transparent Transmission

## Intro

The purpose of this document is to give an overview of the 'Data Transparent Transmission' between an Onboard Device and an Mobile Device. The rest of this document is organized as followed:
* Problem Motivation
* 2 Usage Scenarios (with sample code snippet)

Please be aware that the current upstream(Mobile to Onboard Device) bandwidth is around _1KB/s_ while the downstream (Onboard to Mobile Device) bandwidth is around _8KB/s_.

### Problem Motivation

DJI provides two types of APIs for developers to create their own applications: Mobile API and Onboard API. Mobile API allows developers to monitor and control the UAV from a mobile device running iOS or Android with the remote controller connected. Onboard API allows developers to monitor and control the UAV from any system directly connected to the UAV through a serial(UART) port interface.

Mobile API can be used without any other devices and allows developers to monitor the flight status easily. However, this configuration has some obvious limitations such as: a relatively low computing power, limited wireless bandwidth and unacceptable time latency for real-time or complex control operations.

Onboard API is implemented through the Onboard Device which is mounted on the UAV. Communication with the UAV is done directly through the serial port. It provides sufficient computing power and stability for developers to run complex and demanding applications. Since the Onboard Device is mounted directly on the UAV, developers are not been able to monitor the flight status from their programs easily. If program crashes, developers will have to manually control the UAV with the remote controller and the valuable debugging log data during the run is hard to retrive.

'Data Transparent Transmission' is developed to combine the benefits of these two APIs by establishing a connection between an Mobile Device and an Onboard Device. Via the 'Data Transparent Transmission', developers are able to send data from their Mobile Device to Onboard Device to control over the program while receiving computing results, log data, flight status and external sensor data etc.

In short, Data Transparent Transmission serves as a linkage between Mobile API and Onboard API, granting developers a better flexibility in creating their own flight APPs.

![streamFrame](Images/streamFrame.png)

## Usage Scenario 1 - downstream from Onboard Device to Mobile Device

The CMD set and ID used here is compatible with the Onboard SDK OPEN protocol as follows:

    CMD set: 0x00
    CMD ID: 0xFE

|Data Type|Offset(byte)|Size(byte)|Description|
|---------|------|----|-----------|
|CMD Val|0|0~100|User Data|
|ACK Val|---|---| N/A|

The following code snippet shows you how to receive the data on different mobile platforms including iOS and Android.

- iOS

~~~cSharp
//Setting Delegation
inspireMC.mcDelegate = self;
  
//The delegation function is called when the data comes
(void)mainController:(DJIMainController*)mc didReceivedDataFromExternalDevice:(NSData*)data {
//Here is the receiving data
NSLog(@"%@",data);
}
~~~
  
- Android

~~~java
//Init the data callback interface
DJIMainControllerExternalDeviceRecvDataCallBack mExtDevReceiveDataCallBack = null;
  
//Instantiate callback interface
mExtDevReceiveDataCallBack = new DJIMainControllerExternalDeviceRecvDataCallBack() {
@override
public void onResult(byte[] data) {
  //Here is the receiving data
}
};
  
//Setting callback interface
DJIDrone.getDjiMC().setExternalDeviceRecvDataCallBack(mExtDevReceiveDataCallBack);
~~~

## Usage Scenario 2 - upstream from Mobile Device to Onboard Device

The following code snippet shows you how to send User Data on different mobile platforms including iOS and Android.

#### iOS

  - Initialization
  
~~~cSharp
//Create DJI Drone object according to relative UAV type.
DJIDrone* drone = [DJIDrone droneWithType:DJIDrone_Inspire];
//Obtain Main controller object from DJI Drone object.
DJIInspireMainController* inspireMC = (DJIInspireMainController*)drone.mainController;
//Start data connection.
[drone connectToDrone];
~~~
  
  - Sending data.
  
~~~cSharp
  //Please note that data size should be no larger than 100 bytes.
  NSData* data = [NSData dataWithByte:"..."];
  //Sending data to peripheral and check the sending status through callback function.
  [inspireMC sendDataToExternalDevice:data withResult:(^(DJIError* error)){
    if(error.errorCode == ERR_Successed){
      //Data sent successfully.
    }
    else if(error.errorCode == ERR_InvalidParam) {
      //Data size is null or larger than 100 bytes.
    }
    else {
      //Data sent failed
    }
  }];
~~~
  
#### Android

~~~java
  //Data needs to be sent, please note that data size should be no larger than 100 bytes.
  byte[] data = {0};
  //Sending data to UAV
  DJIDrone.getDjiMC().sendDataToExternalDevice(data,new DJIExecuteResultCallback(){
    @override
    public void onResult(DJIError result) {
      //result is the callback status after sending data
      //1. result == DJIError.ERR_PARAM_IILEGAL,  Data size is null or larger than 100 bytes.
      //2. result == DJIError.ERR_TIMEOUT,        Data sent failed.
      //3. result == DJIError.RESULT_OK,          Data sent successfully.
    }
  });
~~~

The Onboard Device can receive the data sent from the Mobile Device by means of a CMD from Autopilot with the CMD Set, ID and Val to be:

    CMD Set: 0x02
    CMD ID: 0x02

|Data Type|Offset(byte)|Size(byte)|Description|
|---------|------|----|-----------|
|CMD Val|0|1~100|User Data|
|ACK Val|---|---|N/A|
