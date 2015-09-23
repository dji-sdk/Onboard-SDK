# Data Transparent Transmission

## Intro

The purpose of this document is to give an overview of the 'Data Transparent Transmission' between an Onboard Device and an Mobile Device. The rest of this document is organized as followed:
* Problem Motivation
* Usage Scenarios
* Sample Codes for implementation

Please be aware that the current upstream bandwidth (Mobile to Onboard Device) is around _1KB/s_ while the downstream bandwidth (Onboard to Mobile Device) is around _8KB/s_.

### Problem Motivation

DJI provides two types of APIs for developers to create their own applications: Mobile API and Onboard API. Mobile API allows developers to monitor and control the UAV from a mobile device running iOS or Android with the remote controller connected. Onboard API allows developers to monitor and control the UAV from any system directly connected to the UAV through a serial(UART) port interface.

Mobile API can be used without any other devices and allows developers to monitor the flight status easily. However, this configuration has some obvious limitations such as: a relatively low computing power, limited wireless bandwidth and unacceptable time latency for real-time or complex control operations.

Onboard API is implemented through the Onboard Device which is mounted on the UAV. Communication with the UAV is done directly through their serial ports. It provides sufficient computing power and stability for developers to run complex and demanding applications. Since the Onboard Device is mounted directly on the UAV, developers are not been able to monitor the flight status from their programs easily. If program crashes, developers will have to manually control the UAV with the remote controller and the valuable debugging log data during the run is hard to retrive.

'Data Transparent Transmission' was developed to combine the benefits of these two APIs by establishing a connection between an Mobile Device and an Onboard Device. Using the 'Data Transparent Transmission', developers are able to send data from their Mobile Device to Onboard Device to control over the program while receiving computing results, log data, flight status and external sensor data etc.

In short, Data Transparent Transmission serves as a linkage between Mobile API and Onboard API, granting developers a greater flexibility in creating their own flight APPs.

![streamFrame](Images/streamFrame.png)

## USAGE Scenario 1 - downstream from Onboard to Mobile Device

### Step1: Onboard Device to UAV

The following CMD set and CMD ID is compatible with the Onboard SDK OPEN protocol.

    CMD set: 0x00
    CMD ID: 0xFE

|Data Type|Offset|Size|Description|
|---------|------|----|-----------|
|CMD Val|0|0~100|Actual Data needed to be sent to the Mobile Device|
|ACK Val|0|2| Return code 0: Success|

The following code snippet shows you the construction of the CMD data(including CMD set, ID and Val) in C.
~~~c
char cmd_buf[10];
cmd_buf[0] = 0x00;
cmd_buf[1] = 0xFE;
memcpy(&cd_buf[2], "Hello!", 7);
Linklayer_Send(SESSION_MODE3,
                cmd_buf,
                9,
                0,
                200,
                3,
                0
);
~~~

### Step2: UAV to Mobile Device

The following code snippet shows you how to receive data sent from UAV on different mobile platforms including iOS and Android.

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

## USAGE Scenario 2 - upstream from Mobile to Onboard Device

### Mobile Device to UAV

The following code snippet shows you how to send data to UAV on different mobile platforms including iOS and Android.

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

### UAV to Onboard Device

The Onboard Device can receive the data sent from the Mobile Device via the CMD sent from the N1 Autopilot. The detail CMD Set, ID and Val are as follows:
    CMD Set: 0x02
    CMD ID: 0x02

|Data Type|Offset|Size|Description|
|---------|------|----|-----------|
|CMD Val|0|1~100|User Data|
|ACK Val|---|---|N/A|
