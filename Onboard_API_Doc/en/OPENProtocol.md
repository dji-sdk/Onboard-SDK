# DJI Onboard OPEN Protocol  
---

**This documention does not contain Transparent Transport with Mobile SDK part.**

## Protocol Frame Format

   ```
   |<--------------Protocol Frame Header---------------->|<--Protocol Frame Data-->|<--Protocol Frame Checksum-->|
   |SOF|LEN|VER|SESSION|ACK|RES0|PADDING|ENC|RES1|SEQ|CRC16|          DATA           |            CRC32            |
   ```

### Protocol Frame Explanation

<table>
<tr>
  <th>Field</th>
  <th>Byte Index</th>
  <th>Size(bit)</th>
  <th>Description</th>
</tr>

<tr>
  <td>SOF</td>
  <td>0</td>
  <td>8</td>
  <td>Frame start byte, fixed to be 0xAA</td>
</tr>

<tr>
  <td>LEN</td>
  <td rowspan="2">1</td>
  <td>10</td>
  <td>Frame length, maximum length is 1023 (12+1007+4) bytes</td>
</tr>

<tr>
  <td>VER</td>
  <td>6</td>
  <td>Version of the protocol</td>
</tr>

<tr>
  <td>SESSION</td>
  <td rowspan="3">3</td>
  <td>5</td>
  <td>The session ID used during communication</td>
</tr>

<tr>
  <td>ACK</td>
  <td>1</td>
  <td>Frame Type<ul>
    <li>0 ： data</li>
    <li>1 ： acknowlegement</li>
    </ul></td>
</tr>

<tr>
  <td>RES0</td>
  <td>2</td>
  <td>Reserved bits, fixed to be 0</td>
</tr>

<tr>
  <td>PADDING</td>
  <td rowspan="2">4</td>
  <td>5</td>
  <td>The length of additional data added in link layer. It comes from the encryption process</td>
</tr>

<tr>
  <td>ENC</td>
  <td>3</td>
  <td>Frame data encryption type<ul>
    <li>0 ： no encryption</li>
    <li>1 ： AES encryption</li>
    </ul></td>
</tr>

<tr>
  <td>RES1</td>
  <td>5</td>
  <td>24</td>
  <td>Reserved bits, fixed to be 0</td>
</tr>

<tr>
  <td>SEQ</td>
  <td>8</td>
  <td>16</td>
  <td>Frame sequence number</td>
</tr>

<tr>
  <td>CRC16</td>
  <td>10</td>
  <td>16</td>
  <td>Frame header CRC16 checksum</td>
</tr>

<tr>
  <td>DATA</td>
  <td>12</td>
  <td>---</td>
  <td>Frame data, maximum length 1007 bytes</td>
</tr>

<tr>
  <td>CRC32</td>
  <td>---</td>
  <td>32</td>
  <td>Frame CRC32 checksum</td>
</tr>
</table>



### Protocol Data Field

All serial packages exchanged between MATRICE 100 and the onboard device can be classified into three types

|Packages Types|Transmission direction|Transmission Content|
|------------|:----------:|---------------|
|command package|Onboard Device==>MATRICE 100|control commands|
|ACK package|MATRICE 100==>Onboard Device|control results|
|message package|MATRICE 100==>Onboard Device|status|

#### Command Package

```
|<-------Protocol Frame Data------->|
|COMMAND SET|COMMAND ID|COMMAND DATA|
```

|Data Field|Byte Index|Size(byte)|
|----|--------|-----------------|
|COMMAND SET|0|1|
|COMMAND ID|1|1|
|COMMAND DATA|2|depends on the exact command|

#### ACK package
*ACK in header of an ACK package is set*

```
|<-Protocol Frame Data->|
|COMMAND RETURN|ACK DATA|
```


|Data Field|Byte Index|Size(byte)|
|----|--------|-----------------|
|COMMAND RETURN|0|2|
|ACK DATA|2|depends on the exact command|return data|


#### Message Package

```
|<-------Protocol Frame Data------->|
|COMMAND SET|COMMAND ID|COMMAND DATA|
```

|Data Field|Byte Index|Size(byte)|
|----|--------|-----------------|
|COMMAND SET|0|1|
|COMMAND ID|1|1|
|COMMAND DATA*|2|depends on the exact command|

**COMMAND DATA in Message Packageis able to be configured in N1-Assistant*

---

## 协议通信机制


### Session

协议设计使用了会话机制, 以保证命令数据和Return Data不会因为丢包而出现通信双方异常。通信双方在向对方发起通信会话时, 可以根据需要通过设置协议的 SESSION 字段来选择会话方式。协议中设计了三种会话方式。

|Session Mode|SESSION|Description|
|------------|-------|-----------|
|Mode 1|0|Sender do not need acknowledgement.|
|Mode 2|1|Sender need acknowledgement, but can tolerate ACK package loss.|
|Mode 3|2-31|Sender wants to make sure the ACK is reliably sent.*|

**For these sessions, Receiver saves the sequence number in the command package and send an  ACK package upon receiving it. If ACK package loss happened, Sender may request Receiver again using the same command package with the same sequence number.*


*Note: Here a dummy link layer send interface is defined for demonstration purpose. Since Session Mode 3 is reliable, the communication function interface should contain parameters such as length of timeout and number of resending times.*

---

## Command Set and Command ID

### Command Set

**The DJI onboard API has three sets or categories of commands:**  

|Category|Command Set ID|Description|
|--------|-----------|--------------|
|Activation related|0x00|All commands used to activate API|
|Control related|0x01|Commands to control MATRICE 100|
|Monitoring related|0x02|Commands that contains autopilot status data|

### Command ID

Each Command Set contains some Command ID to achieve different functions

*All Command needs to be performed at an Authorization Level. A command will not be performed when this command needs a higher lever than the level which autopilot has, while an autopilot is able to receive a command with lower level.*

|API Levels|Brief Plan|
|:--------:|----------|
|0|API activation commands|
|1|Camera and gimbal control commands|
|2|Flight control commands|

*The Authorization Level of a autopilot can be changed by command 'Activate API'. The default of Authorization Level is 0.*



**Onboard API Function Index**
<table>
<tr>
  <th>Command Set</th>
  <th>Command ID</th>
  <th>Function</th>
  <th>Authorization Level</th>
</tr>
  <td rowspan="2">0x00<br>Activation Command Set</td>
  <td>0x00</td>
  <td>Get API version</td>
  <td>0</td>
</tr>

</tr>
  <td>0x01</td>
  <td>Activation</td>
  <td>0</td>
</tr>

<tr>
  <td rowspan="9">0x01<br>Control Command Set</td>
  <td>0x00</td>
  <td>Control Authority Request</td>
  <td>2</td>
</tr>

<tr>
  <td>0x01</td>
  <td>Switch Flight Mode</td>
  <td>2</td>
</tr>
<tr>
  <td>0x02</td>
  <td>Request Flight Mode</td>
  <td>2</td>
</tr>

<tr>
  <td>0x03</td>
  <td>Movement Control</td>
  <td>2</td>
</tr>

<tr>
  <td>0x0A</td>
  <td>Gimbal Control in Rate</td>
  <td>1</td>
</tr>

<tr>
  <td>0x0B</td>
  <td>Gimbal Control in Position</td>
  <td>1</td>
</tr>

<tr>
  <td>0x20</td>
  <td>Take Photo</td>
  <td>1</td>
</tr>

<tr>
  <td>0x21</td>
  <td>Start Record</td>
  <td>1</td>
</tr>

<tr>
  <td>0x22</td>
  <td>Stop Record</td>
  <td>1</td>
</tr>

<tr>
  <td rowspan="2">0x02<br>Monitor Command Set</td>
  <td>0x00</td>
  <td>Message Package</td>
  <td>0</td>
</tr>

<tr>
  <td>0x01</td>
  <td>Control Authority Change Notification</td>
  <td>0</td>
</tr>
</table>


## Command Data

### Activation Command Set: 0x00 

#### Command ID 0x00: Get API version

<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr> 
  <td>Request Data</td>
  <td>0</td>
  <td>1</td>
  <td>Arbitrary number</td>
</tr>

<tr>
  <td rowspan="3">Return Data</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code<ul>
    <li>0x0000：Autopilot Activation</li>
    <li>0xFF01：Autopilot No Activation</li>
</tr>

<tr>
  <td>2</td>
  <td>4</td>
  <td>The CRC code of API version string</td>
</tr>

<tr>
  <td>6</td>
  <td>32</td>
  <td>API version string</td>
</tr>
</table>


#### Command ID: 0x01 Activation

<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="4">Request Data</td>
  <td>0</td>
  <td>4</td>
  <td>app_id, a number obtained when user registers as a developer</td>
</tr>

<tr>
  <td>4</td>
  <td>4</td>
  <td>api_level, Authorization Level</td>
</tr>

<tr>
  <td>8</td>
  <td>4</td>
  <td>Fixed value, 0x02030A00</td>
</tr>

<tr>
  <td>12</td>
  <td>32</td>
  <td>Fixed string, "12345678901234567890123456789012"</td>
</tr>

<tr>
  <td>Return Data</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code：<ul>
    <li>0x0000：Success</li>
    <li>0x0001：Invalid parameters</li>
    <li>0x0002：Cannot recognize encrypted package</li>
    <li>0x0003：No activate, Attempt to activate</li>
    <li>0x0004：DJI GO APP no response </li>
    <li>0x0005：DJI GO APP no Internet</li>
    <li>0x0006：Server rejected activation attempt</li>
    <li>0x0007：Insufficient authority level</li>
    <li>0x0008：Wrong SDK version</li>
    </ul></td>
</tr>

</table>

### Command Set 0x01 Control Command 

#### Command ID 0x00: Control Authority Request

<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td >Request Data</td>
  <td>0</td>
  <td>1</td>
  <td>Request Code<ul>
    <li>0x01 ： request to get control authority</li>
    <li>0x00 ： request to release control authority</li>
    </ul></td>
</tr>

<tr>
 <td >Return Data</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code <ul>
    <li>0x0001：successfully released control authority</li>
    <li>0x0002：successfully obtained control authority</li>
    <li>0x0003：control authority failed to change</li>
    </ul></td>
</tr>

</table>

There are three types of control devices: 1. Remote Controller 2. Mobile Device 3. Onboard Device

The control priority is Remote Controller > Mobile Device > Onboard Device

#### Command ID 0x01 Switch Flight Mode

<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="2">Request Data</td>
  <td>0</td>
  <td>1</td>
  <td>Command Sequence Number</td>
</tr>

<tr>
  <td>1</td>
  <td>1</td>
  <td>Request mode<ul>
    <li>0x01 ： request return to home(RTH)</li>
    <li>0x04 ： request auto take off</li>
    <li>0x06 ： request auto landing</li>
    </ul></td>
</tr>

<tr>
  <td>Return Data</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code<ul>
    <li>0x0001：the command is received but rejected</li>
    <li>0x0002：start to execute the command</li>
    </ul></td>
</tr>

</table>

#### Command ID 0x02 Request Flight Mode
<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td >Request Data</td>
  <td>0</td>
  <td>2</td>
  <td>Command Sequence Number</td>
</tr>


<tr>
  <td>Return Data</td>
  <td>0</td>
  <td>1</td>
  <td>Return Code<ul>
    <li>0x0001：query failed, current command is not the query command</li>
    <li>0x0003：command is executing</li>
    <li>0x0004：command failed</li>
    <li>0x0005：command succeed</li>
    </ul></td>
</tr>

</table>

#### Command ID 0x03 Movement Control*
<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="5">Request Data</td>
  <td>0</td>
  <td>1</td>
  <td>Control mode byte</td>
</tr>

<tr>
  <td>1</td>
  <td>4</td>
  <td>Roll control value or X-axis control value</td>
</tr>

<tr>
  <td>5</td>
  <td>4</td>
  <td>Pitch control value or Y-axis control value</td>
</tr>

<tr>
  <td>9</td>
  <td>4</td>
  <td>Yaw control value</td>
</tr>

<tr>
  <td>13</td>
  <td>4</td>
  <td>Vertical control value or Z-axis control value</td>
</tr>

<tr>
  <td>Return Data</td>
  <td>---</td>
  <td>---</td>
  <td>NO ACK</td>
</tr>

</table>

**The amount charged for the translation or rotation is determined by the mode flag byte , the specific content related to movement control refer to [Additional Explanation for Flight Control][0]*  

#### Command ID 0x0A Gimbal Control in Rate
<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="4">Request Data</td>
  <td>0</td>
  <td>2</td>
  <td>Rate in Yaw</td>
</tr>
<tr>
  <td>2</td>
  <td>2</td>
  <td>Rate in Roll</td>
</tr>
<tr>
  <td>4</td>
  <td>2</td>
  <td>Rate in Pitch</td>
</tr>
<tr>
  <td>6</td>
  <td>1</td>
  <td>Fixed value, 0x80</td>
</tr>

<tr>
  <td>Return Data</td>
  <td>---</td>
  <td>---</td>
  <td>NO ACK</td>
</tr>

</table>


<table>
<tr>
  <th>Data Name</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td>rate in Yaw</td>
  <td>int16_t</td>
  <td>unit 0.1º/s, Input Range[-1800,+1800]</td>
</tr>

<tr>
  <td>rate in Roll</td>
  <td>int16_t</td>
  <td>unit 0.1º/s, Input Range[-1800,+1800]</td>
</tr>

<tr>
  <td>rate in Pitch</td>
  <td>int16_t</td>
  <td>unit 0.1º/s, Input Range[-1800,+1800]</td>
</tr>
</table>

#### Command ID 0x0B Gimbal Control in Position
<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="5">Request Data</td>
  <td>0</td>
  <td>2</td>
  <td>Yaw axis angle</td>
</tr>
<tr>
  <td>2</td>
  <td>2</td>
  <td>Roll axis angle</td>
</tr>
<tr>
  <td>4</td>
  <td>2</td>
  <td>Pitch axis angle</td>
</tr>
<tr>
  <td>6</td>
  <td>1</td>
  <td>Control flag byte<ul>
    <li>bit 0：Mode flag bit</li>
        <ul>0 ： Incremental control, the angle reference for the current Gimbal location</ul>
        <ul>1 ： Absolute control, the angle reference coordinate system to the Northeast</ul>
    <li>bit 1：Yaw invaild bit 
        <ul>0 ： Gimble will follow the command in Yaw </ul>
        <ul>1 ： Gimble will maintain position in Yaw  </ul>
    <li>bit 2：Roll invaild bit, the same as bit[1]</li>
    <li>bit 3：Pitch invaild bit, the same as bit[1]</li>
    <li>bit [4:7]：reserved, should be 0</li>
    </ul></td>

<tr>
  <td>7</td>
  <td>1</td>
  <td>Command completion time</td>
</tr>
</tr>

<tr>
  <td>Return Data</td>
  <td>---</td>
  <td>---</td>
  <td>NO ACK</td>
</tr>
</table>

<table>
<tr>
  <th>Data Name</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td>Yaw axis angle</td>
  <td>int16_t</td>
  <td>unit 0.1º, Input Range [-3200,+3200]</td>
</tr>

<tr>
  <td>Roll axis angle</td>
  <td>int16_t</td>
  <td>unit 0.1º, Input Range [-350,+350]</td>
</tr>

<tr>
  <td>Pitch axis angle</td>
  <td>int16_t</td>
  <td>unit 0.1º, Input Range [-900,+300]</td>
</tr>

<tr>
  <td>Command completion time</td>
  <td>uint8_t</td>
  <td>unit 0.1s, for example 20 means gimble will reach the pose given by command in 2 seconds<br>recommand do not let gambil rotate rate over 400º/s</td>
</tr>
</table>

#### Command ID 0x20 Take Photo
<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr> 
  <td>Request Data</td>
  <td>0</td>
  <td>1</td>
  <td>Arbitrary number</td>
</tr>

<tr>
  <td>Return Data</td>
  <td>---</td>
  <td>---</td>
  <td>NO ACK</td>
</tr>
</table>
#### Command ID 0x21 Start Record

<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr> 
  <td>Request Data</td>
  <td>0</td>
  <td>1</td>
  <td>Arbitrary number</td>
</tr>

<tr>
  <td>Return Data</td>
  <td>---</td>
  <td>---</td>
  <td>NO ACK</td>
</tr>

</table>
#### Command ID 0x22 Stop Record

<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr> 
  <td>Request Data</td>
  <td>0</td>
  <td>1</td>
  <td>Arbitrary number</td>
</tr>

<tr>
  <td>Return Data</td>
  <td>---</td>
  <td>---</td>
  <td>NO ACK</td>
</tr>
</table>
### Command Set 0x02 Monitor Command Set

#### Command ID 0x00 Message Package

Message Package content and frequency can be configured by DJI N1 assistant software. Each data item in the message package has individual frequency. The maximum message frequency is equal to the message's highest data item update frequency.

<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)*</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="13">Message Package</td>
  <td>0</td>
  <td>2</td>
  <td>Item presence byte, Bit with value 1 means the message package contains corresponding data item<ul>
    <li>bit 0：flag of time stamp</li>
    <li>bit 1：flag of attitude quaternion</li>
    <li>bit 2：flag of linear acceleration</li>
    <li>bit 3：flag of linear velocity</li>
    <li>bit 4：flag of angular velocity</li>
    <li>bit 5：flag of GPS location, altitude and healthiness</li>
    <li>bit 6：flag of magnetometer</li>
    <li>bit 7：flag of remote controller data</li>
    <li>bit 8：flag of gimbal yaw,pitch and roll</li>
    <li>bit 9：flag of flight status</li>
    <li>bit 10：flag of battery info</li>
    <li>bit 11：flag of control device</li>
    <li>bit [12:15]：reserved</li>
    </ul></td>
</tr>

<tr>
  <td>2</td>
  <td>4</td>
  <td>Time stamp</td>
</tr>

<tr>
  <td>6</td>
  <td>16</td>
  <td>Attitude quaternion</td>
</tr>

<tr>
  <td>22</td>
  <td>12</td>
  <td>Linear acceleration</td>
</tr>

<tr>
  <td>34</td>
  <td>13</td>
  <td>Linear velocity</td>
</tr>

<tr>
  <td>47</td>
  <td>12</td>
  <td>Angular velocity</td>
</tr>

<tr>
  <td>59</td>
  <td>24</td>
  <td>GPS position, altitude, height and healthiness</td>
</tr>

<tr>
  <td>83</td>
  <td>12</td>
  <td>Magnetometer data</td>
</tr>

<tr>
  <td>95</td>
  <td>10</td>
  <td>Remote controller channels</td>
</tr>

<tr>
  <td>105</td>
  <td>12</td>
  <td>Gimbal roll, pitch and yaw</td>
</tr>

<tr>
  <td>117</td>
  <td>1</td>
  <td>Flight status</td>
</tr>

<tr>
  <td>118</td>
  <td>1</td>
  <td>Battery percentage</td>
</tr>

<tr>
  <td>119</td>
  <td>1</td>
  <td>Control device</td>
</tr>
</table>

**Offset(byte)：Offset(byte) in this table is in the case that all items exist in one Message Package*

Actual Offset in Message Package is calculated by flags in 'item presence byte'.
 
**The meaning of each data item in the message package**

<table>
<tr>
  <td>Item Name</td>
  <td>Variables</td>
  <td>Type</td>
  <td>Description</td>
  <td>Unit</td>
  <td>Default Frequency</td>
</tr>

<tr>
  <td>Time Stamp</td>
  <td>time</td>
  <td>unsigned int</td>
  <td>time_stamp</td>
  <td>Time in tick (tick interval 1/600s)</td>
  <td>100Hz</td>
</tr>
<tr>
  <td rowspan="4">Quarternion</td>
  <td>q0</td>
  <td>float32</td>
  <td rowspan="4">Attitude quaternion<br>From ground to body frame*</td>
  <td rowspan="4">---</td>
  <td rowspan="4">100Hz</td>
</tr>
<tr>
  <td>q1</td>
  <td>float32</td>
</tr>
<tr>
  <td>q2</td>
  <td>float32</td>
</tr>
<tr>
  <td>q3</td>
  <td>float32</td>
</tr>

<tr>
  <td rowspan="3">Linear acceleration</td>
  <td>agx</td>
  <td>float32</td>
  <td rowspan="3">Linear acceleration</td>
  <td rowspan="3">m/s<sup>2</sup> </td>
  <td rowspan="3">100Hz</td>
</tr>
<tr>
  <td>agy</td>
  <td>float32</td>
</tr>
<tr>
  <td>agz</td>
  <td>float32</td>
</tr>

<tr>
  <td rowspan="4">Linear velocity</td>
  <td>vgx</td>
  <td>float32</td>
  <td rowspan="3">Linear velocity</td>
  <td rowspan="3">m/s</td>
  <td rowspan="4">100Hz</td>
</tr>
<tr>
  <td>vgy</td>
  <td>float32</td>
</tr>
<tr>
  <td>vgz</td>
  <td>float32</td>
</tr>

<tr>
  <td>vgstatus</td>
  <td>uint8_t</td>
  <td>Byte of linear velocity status<ul>
  <li>bit 0：data valid flag</li>
    <ul>0：invalid</ul>
    <ul>1：valid</ul>
  <li>bit 1:4 ：data source</li>
    <ul>0b011：GPS</ul>
    <ul>0b110：MVO (Mono Video Odometer)</ul>
    <ul>0b111：SVO (Stereo Video Odometer)</ul>
  <li>bit 5:7 ：reserved</li>
  </ul></td>
  <td>---</td>
</tr>

<tr>
  <td rowspan="3">Angular velocity</td>
  <td>wx</td>
  <td>float32</td>
  <td rowspan="3">Angular velocity </td>
  <td rowspan="3">rad/s</td>
  <td rowspan="3">100Hz</td>
</tr>
<tr>
  <td>wy</td>
  <td>float32</td>
</tr>
<tr>
  <td>wz</td>
  <td>float32</td>
</tr>

<tr>
  <td rowspan="5">GPS and altitude</td>
  <td>longti</td>
  <td>double</td>
  <td rowspan="2">GPS location</td>
  <td rowspan="2">rad</td>
  <td rowspan="5">100Hz</td>
</tr>
<tr>
  <td>lati</td>
  <td>double</td>
</tr>
<tr>
  <td>alti</td>
  <td>float32</td>
  <td>Altitude</td>
  <td>m</td>
</tr>
<tr>
  <td>height</td>
  <td>float32</td>
  <td>Height to ground**</td>
  <td>m</td>
</tr>
<tr>
  <td>health_flag</td>
  <td>uint8_t</td>
  <td>GPS healthiness </td>
  <td>0-5, 5 is the best condition</td>
</tr>

<tr>
  <td rowspan="3">Magnetometer</td>
  <td>mx</td>
  <td>int16_t</td>
  <td rowspan="3">Magnetometer data</td>
  <td rowspan="3">Magnetometer data</td>
  <td rowspan="3">0Hz</td>
</tr>
<tr>
  <td>my</td>
  <td>int16_t</td>
</tr>
<tr>
  <td>mz</td>
  <td>int16_t</td>
</tr>

<tr>
  <td rowspan="6">Remote controller*</td>
  <td>roll</td>
  <td>int16_t</td>
  <td>Remote controller roll channel</td>
  <td rowspan="6">---</td>
  <td rowspan="6">50Hz</td>
</tr>
<tr>
  <td>pitch</td>
  <td>int16_t</td>
  <td>Remote controller pitch channel</td>
</tr>
<tr>
  <td>yaw</td>
  <td>int16_t</td>
  <td>Remote controller wya channel</td>
</tr>
<tr>
  <td>throttle</td>
  <td>int16_t</td>
  <td>Remote controller throttle channel</td>
</tr>
<tr>
  <td>mode</td>
  <td>int16_t</td>
  <td>Remote controller mode channel</td>
</tr>
<tr>
  <td>gear</td>
  <td>int16_t</td>
  <td>Remote controller gear channel</td>
</tr>

<tr>
  <td rowspan="3">Gimbal</td>
  <td>roll</td>
  <td>float32</td>
  <td rowspan="3">Gimbal roll, pitch and yaw in ground frame</td>
  <td rowspan="3">º</td>
  <td rowspan="3">50Hz</td>
</tr>
<tr>
  <td>pitch</td>
  <td>float32</td>
</tr>
<tr>
  <td>yaw</td>
  <td>float32</td>
</tr>

<tr>
  <td>Flight status*</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>Flight status</td>
  <td>---</td>
  <td>10Hz</td>
</tr>

<tr>
  <td>Battery</td>
  <td>battery</td>
  <td>uint8_t</td>
  <td>Battery percentage</td>
  <td>Battery percentage</td>
  <td>1Hz</td>
</tr>

<tr>
  <td>Control device</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>Control device<ul>
     <li>bit 0:2 ：Control device</li>
     <ul>0b000 ：RC</ul>
     <ul>0b001 ：APP</ul>
     <ul>0b010 ：Onboard device</ul>
     <li>bit 3 ：Flag of Onboard Device control authority Request </li>
     <ul>0：No request</ul>
     <ul>1：Have requested</ul>
     <li>bit 4:7 ：reserved</li>
  </ul></td>
  <td>---</td>
  <td>0Hz</td>
</tr>
<tr>
  <td>Control device</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>Control device<ul>
     <li>0x00 ： RC</li>
     <li>0x01 ： APP</li>
     <li>0x02 ： Onboard device</li>
     </ul></td>
  <td>---</td>
  <td>0Hz</td>
</tr>
</table>
**More info about Ground frame、Body frame、Remote controller and Flight status please refer [Additional Explanation for Flight Control][0]*  
***alti is the result of barometer-IMU fusion in the unit of pressure. If the Matrice 100 has no ultrasonic sensor (no Gudiance installed), or it has ultrasonic sensor but its distance to the ground is larger than 3 meters (the measurement of ultrasonic sensor will be unstable at this distance), than height is supported by barometer and IMU only. Because barometer is very inaccurate indoor, alti is unreliable.*


#### Command ID 0x01 Control Authority Change Notification
Onboard device has the lowers control priority. Its control authority can be taken over by RC and mobile device at any time. Once the control authority changed, a notification message will be sent to the onboard device by the autopilot.

<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td >Message Package</td>
  <td>0</td>
  <td>1</td>
  <td>Fixed value, 0x04</td>
</tr>


</table>

[0]: FlightControlExplain.md

