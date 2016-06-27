#### This documentation is now deprecated, please refer to <https://developer.dji.com/onboard-sdk/documentation/introduction/index.html> in DJI Developer Website.

# Onboard SDK OPEN Protocol  

---

## Protocol Frame

The Protocal Frame is the smallest unit for transmission. It contains the Header, Data and the Tail as follows:
   ```
   |<---------------------Header-------------------------->|<--Data-->|<--Tail-->|
   |SOF|LEN|VER|SESSION|ACK|RES0|PADDING|ENC|RES1|SEQ|CRC16|   DATA   |  CRC32   |
   ```

### Frame Format

<table>
<tr>
  <th>Field</th>
  <th>Index (byte)</th>
  <th>Size (bit)</th>
  <th>Description</th>
</tr>

<tr>
  <td>SOF</td>
  <td>0</td>
  <td>8</td>
  <td>starting byte, fixed to be 0xAA</td>
</tr>

<tr>
  <td>LEN</td>
  <td rowspan="2">1</td>
  <td>10</td>
  <td>len of frame</td>
</tr>

<tr>
  <td>VER</td>
  <td>6</td>
  <td>version of the frame header, set to be 0</td>
</tr>

<tr>
  <td>SESSION</td>
  <td rowspan="3">3</td>
  <td>5</td>
  <td>session ID</td>
</tr>

<tr>
  <td>ACK</td>
  <td>1</td>
  <td>frame type<ul>
    <li>0: CMD</li>
    <li>1: ACK</li>
    </ul></td>
</tr>

<tr>
  <td>RES0</td>
  <td>2</td>
  <td>reserved, fixed to be 0</td>
</tr>

<tr>
  <td>PADDING</td>
  <td rowspan="2">4</td>
  <td>5</td>
  <td>len of padding data used by the Data encryption</td>
</tr>

<tr>
  <td>ENC</td>
  <td>3</td>
  <td>encryption type<ul>
    <li>0: no encryption</li>
    <li>1: AES encryption</li>
    </ul></td>
</tr>

<tr>
  <td>RES1</td>
  <td>5</td>
  <td>24</td>
  <td>reserved, fixed to be 0</td>
</tr>

<tr>
  <td>SEQ</td>
  <td>8</td>
  <td>16</td>
  <td>frame sequence num</td>
</tr>

<tr>
  <td>CRC16</td>
  <td>10</td>
  <td>16</td>
  <td>CRC16 frame header checksum</td>
</tr>

<tr>
  <td>DATA</td>
  <td>12</td>
  <td>variable size</td>
  <td>frame data</td>
</tr>

<tr>
  <td>CRC32</td>
  <td>variable size</td>
  <td>32</td>
  <td>CRC32 whole frame checksum</td>
</tr>
</table>

Note: After CRC16 encryption, developers should update the `PADDING` and `LEN` part in your frame header before doing CRC32 encryption.
<!-- >备注: 如果协议帧需要AES加密，应先对数据段进行加密，该过程会改变数据长度，需要更新帧头中*PADDING*字段及*LEN*字段。再对帧头部分进行CRC16校验，获得校验值后再对整个协议帧进行CRC32校验 -->
### Frame Type
There are two types of frames.

|Frame Type|Data Type|Transmission Direction|Content|
|------------|----------|:----------:|---------------|
|CMD frame|CMD frame data|Onboard Device <=>  Autopilot|flight control related CMDs|
|ACK frame|ACK frame data|Autopilot <=> Onboard Device|ACK related data|

#### CMD frame data
```
|<------CMD frame data------>|
| CMD SET | CMD ID | CMD VAL |
```

|Field|Index (byte)|Size (byte)|
|----|--------|-----------------|
|CMD SET|0|1|
|CMD ID|1|1|
|CMD VAL|2|vary by CMDs|

#### ACK frame data

```
|<-ACK frame data->|
|      ACK VAL     |
```

|Field|Index (byte)|Size (byte)|
|----|--------|-----------------|
|ACK VAL|0|vary by ACKs|

SEQ field in ACK frame is the same as the one in corresponding CMD frame. Developers can use the SEQ field of the ACK frame to match the corresponding CMD frame. 

---

## Protocal Transmission Mechanism

### Session

The session mechanism has been used in order to prevent the exceptions such as package loss. 3 kinds of session have been introduced:

|Type|SESSION|Description|
|------------|-------|-----------|
|0|0|Sender doesn't need ACKs.|
|1|1|Sender needs ACKs but can be tolerated.|
|2|2-31|Sender needs ACKs.*|

>Note: *Type 1 and Type 2 can ONLY be applied to the CMDs which have ACKs.*


*Since type 2 is designed to be reliable. Developers should implement the package loss & resending mechinism based on the current SEQ and SESSION fields. When package loss is found from the sender, the sender can send the CMD frame again with the same SEQ and SESSION lost previously to retrieve back the ACK frame.


### Encryption (optional)

DJI designs encryption interface in communication between Onboard Device and autopilot, considering it is possible that developer implement the communication with unsafe channel, for example various types of wireless transparent transmission module.

AES encrypting is designed only for data part of a frame. And developers can encrypt their own data with a key which is despatched when they register the APP ID, and encryption function in `DJI_LIB`.

Due to encryption process, data need to be aligned into blocks and each one contains 16 bytes. 'PADDING' in frame header is the length of additional data generated in this progress. Also 'LEN' need to be caclulated again, and 'ENC' need to be set to '1'. For more details about encryption algorithm, please refer to `sdk_encrypt_interface` in `DJI_Pro_Codec.cpp` .

On the other hand, there is no need to encrypt data, when developers use a serial cable to connect Onboard Device and autopilot. In this case, just set 'ENC' to '0'.

---

## CMD Set & CMD ID

### CMD Set & CMD ID

Each CMD Set contains some CMD IDs for different operations.

### Function Index
<table>
<tr>
  <th>CMD Set</th>
  <th>CMD ID</th>
  <th>Function</th>
</tr>
<tr>
  <td rowspan="4">0x00<br>Initialization CMD Set</td>
  <td>0x00</td>
  <td>Get Protocal Version</td>
</tr>

<tr>
  <td>0x01</td>
  <td>Activation</td>
</tr>

<tr>
  <td>0x10</td>
  <td>Set 'Flight Data' frequency</td>
</tr>
<tr>
  <td>0xFE</td>
  <td>Data Transparent Transmission<br>From Onboard SDK to Mobile SDK</td>
</tr>

<tr>
  <td rowspan="10">0x01<br>Control CMD Set</td>
  <td>0x00</td>
  <td>Obtain/Release Control Authorization</td>
</tr>

<tr>
  <td>0x01</td>
  <td>Switch Flight Mode</td>
</tr>
<tr>
  <td>0x02</td>
  <td>Request Switch Result</td>
</tr>

<tr>
  <td>0x03</td>
  <td>Movement Control</td>
</tr>

<tr>
  <td>0x05</td>
  <td>Arm/Disarm</td>
</tr>


<tr>
  <td>0x1A</td>
  <td>Gimbal Control in Rate</td>
</tr>

<tr>
  <td>0x1B</td>
  <td>Gimbal Control in Position</td>
</tr>

<tr>
  <td>0x20</td>
  <td>Take Photo</td>
</tr>

<tr>
  <td>0x21</td>
  <td>Start Recording Video</td>
</tr>

<tr>
  <td>0x22</td>
  <td>Stop Recording Video</td>
</tr>

<tr>
  <td rowspan="5">0x02<br>Push Data CMD Set</td>
  <td>0x00</td>
  <td>Flight Data</td>
</tr>

<tr>
  <td>0x01</td>
  <td>Lost of Flight Control</td>
</tr>
<tr>
  <td>0x02</td>
  <td>Data Transparent Transmission<br>From Mobile SDK to Onboard SDK</td>
</tr>

<tr>
  <td>0x03</td>
  <td>Ground Station State</td>
</tr>
<tr>
  <td>0x04</td>
  <td>Waypoint Event</td>
</tr>

<tr>
  <td rowspan="8">0x03<br>Ground Station CMD Set<br>Waypoint</td>
  <td>0x10</td>
  <td>Upload waypoint info</td>
</tr>
<tr>
  <td>0x11</td>
  <td>Upload waypoint</td>
</tr>
<tr>
  <td>0x12</td>
  <td>Start/Stop waypoint</td>
</tr>
<tr>
  <td>0x13</td>
  <td>Pause/Resume waypoint</td>
</tr>
<tr>
  <td>0x14</td>
  <td>Download waypoint info</td>
</tr>
<tr>
  <td>0x15</td>
  <td>Download index waypoint</td>
</tr>
<tr>
  <td>0x16</td>
  <td>Set idle speed</td>
</tr>
<tr>
  <td>0x17</td>
  <td>Get idle speed</td>
</tr>
<tr>
  <td rowspan="8">0x03<br>Ground Station CMD Set<br>Hotpoint</td>
  <td>0x20</td>
  <td>Start hotpoint</td>
</tr>
<tr>
  <td>0x21</td>
  <td>Stop hotpoint</td>
</tr>
<tr>
  <td>0x22</td>
  <td>Pause/Resume hotpoint</td>
</tr>
<tr>
  <td>0x23</td>
  <td>Set idle speed</td>
</tr>
<tr>
  <td>0x24</td>
  <td>Set radius</td>
</tr>
<tr>
  <td>0x25</td>
  <td>Reset yaw</td>
</tr>
<tr>
  <td>0x26</td>
  <td>Download hotpoint info</td>
</tr>
<tr>
   <td>0x27</td>
   <td>Set Auto Radius</td>
<tr>
  <td rowspan="4">0x03<br>Ground Station CMD Set<br>Follow Me</td>
  <td>0x30</td>
  <td>Start follow me</td>
</tr>
<tr>
  <td>0x31</td>
  <td>Stop follow me</td>
</tr>
<tr>
  <td>0x32</td>
  <td>Pause/Resume follow me</td>
</tr>
<tr>
  <td>0x33</td>
  <td>Set target pos info</td>
</tr>
<tr>
  <td>0x04<br>Sync signal CMD Set</td>
  <td>0x00</td>
  <td>Set Sync signal output frequency</td>

</tr>
<tr>
  <td rowspan="2">0x05<br>Virual RC CMD Set</td>
  <td>0x00</td>
  <td>Virual RC request</td>
</tr>
<tr>
  <td>0x01</td>
  <td>Virual RC data</td>
</tr>
</table>

## CMD Val & ACK Val

### Activation CMD Set: 0x00 

#### CMD ID 0x00: Get Protocal Version

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr> 
  <td>CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>arbitrary num</td>
</tr>

<tr>
  <td rowspan="3">ACK Val</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code<ul>
    <li>0x0000:  Autopilot is activated</li>
    <li>0xFF01:  Autopilot is NOT activated</li>
</tr>

<tr>
  <td>2</td>
  <td>4</td>
  <td>The CRC val of the protocal version</td>
</tr>

<tr>
  <td>6</td>
  <td>32</td>
  <td>protocal version</td>
</tr>
</table>

#### CMD ID 0x01: Activation

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="3">CMD Val</td>
  <td>0</td>
  <td>4</td>
  <td>app_id, app unique identifer</td>
</tr>

<tr>
  <td>8</td>
  <td>4</td>
  <td>Fixed value:<ul>
   M100: 0x03010A00<br>A3: 0x03016400
  </ul></td>
</tr>

<tr>
  <td>12</td>
  <td>32</td>
  <td>Fixed string, "12345678901234567890123456789012"</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code: <ul>
    <li>0x0000: Success</li>
    <li>0x0001: Invalid parameters</li>
    <li>0x0002: Cannot recognize the encrypted package</li>
    <li>0x0003: New APP ID, please connect DJI GO APP</li>
    <li>0x0004: No response from DJI GO APP </li>
    <li>0x0005: No Internet from DJI GO APP </li>
    <li>0x0006: Server rejected</li>
    <li>0x0007: Authorization level insufficient</li>
    <li>0x0008: Wrong SDK version</li>
    </ul></td>
</tr>

</table>
#### CMD ID 0x10: Set 'Flight Data' frequency

M100:

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="13">CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>time stamp</td>
</tr>
<tr>
  <td>1</td>
  <td>1</td>
  <td>attitude quaternion</td>
</tr>
<tr>
  <td>2</td>
  <td>1</td>
  <td>linear acceleration</td>
</tr>
<tr>
  <td>3</td>
  <td>1</td>
  <td>linear velocity</td>
</tr>
<tr>
  <td>4</td>
  <td>1</td>
  <td>angular velocity</td>
</tr>
<tr>
  <td>5</td>
  <td>1</td>
  <td>GPS location, altitude and height</td>
</tr>
<tr>
  <td>6</td>
  <td>1</td>
  <td>magnetometer</td>
</tr>
<tr>
  <td>7</td>
  <td>1</td>
  <td>remote controller data</td>
</tr>
<tr>
  <td>8</td>
  <td>1</td>
  <td>roll, pitch and yaw of gimbal </td>
</tr>
<tr>
  <td>9</td>
  <td>1</td>
  <td>flight status</td>
</tr>
<tr>
  <td>10</td>
  <td>1</td>
  <td>battery info</td>
</tr>
<tr>
  <td>11</td>
  <td>1</td>
  <td>control device</td>
</tr>
<tr>
  <td>12</td>
  <td>4</td>
  <td>reserved</td>
</tr>
<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code<ul>
    <li>0x0000:  success</li>
    <li>0x0001:  param error</li>
    </ul></td>
</tr>

</table>

A3:

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="15">CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>time stamp</td>
</tr>
<tr>
  <td>1</td>
  <td>1</td>
  <td>attitude quaternion</td>
</tr>
<tr>
  <td>2</td>
  <td>1</td>
  <td>linear acceleration</td>
</tr>
<tr>
  <td>3</td>
  <td>1</td>
  <td>linear velocity</td>
</tr>
<tr>
  <td>4</td>
  <td>1</td>
  <td>angular velocity</td>
</tr>
<tr>
  <td>5</td>
  <td>1</td>
  <td>GPS location, altitude and height</td>
</tr>
<tr>
  <td>6</td>
  <td>1</td>
  <td>GPS detailed information</td>
</tr>
<tr>
  <td>7</td>
  <td>1</td>
  <td>RTK detailed information</td>
</tr>
<tr>
  <td>8</td>
  <td>1</td>
  <td>magnetometer</td>
</tr>
<tr>
  <td>9</td>
  <td>1</td>
  <td>remote controller data</td>
</tr>
<tr>
  <td>10</td>
  <td>1</td>
  <td>roll, pitch and yaw of gimbal </td>
</tr>
<tr>
  <td>11</td>
  <td>1</td>
  <td>flight status</td>
</tr>
<tr>
  <td>12</td>
  <td>1</td>
  <td>battery info</td>
</tr>
<tr>
  <td>13</td>
  <td>1</td>
  <td>control device</td>
</tr>
<tr>
  <td>14</td>
  <td>2</td>
  <td>reserved</td>
</tr>
<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code<ul>
    <li>0x0000:  success</li>
    <li>0x0001:  param error</li>
    </ul></td>
</tr>

</table>

|Val|Freq|
|-----|------|
|0|0Hz|
|1|1Hz|
|2|10Hz|
|3|50Hz|
|4|100Hz|
|5|Last freq|

### CMD Set 0x01 Control CMDs

#### CMD ID 0x00: Obtain/Release Control Authorization

Please make sure the following conditions have been met:

* The 'enable API control' box is checked in the assistant software.
* The mode selection bar of the remote controller is placed at the F position.

Due to the consideration of safety, all requests of obtain control and release control need to be sent **twice** and the first ACK will be fail. In our official library, we have already implemented this feature.

**IMPORTANT! afterwase the release of firmware 3.0, the drone will enter F mode logic directly if the mode bar is placed at F when power on, developers do not need to turn away then back to enter F mode logic as the way in firmware 2.3; Please pay attention to this change if you are upgrading from 2.3**  

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>
    0x01: request to obtain control authorization</br>
    0x00: request to release control authorization</br>
  </td>
</tr>

<tr>
 <td >ACK Val</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code <ul>
    <li>0x0000: the mode selection bar of the remote controller is not placed at the F position</li>
    <li>0x0001: successfully released control authorization</li>
    <li>0x0002: successfully obtained control authorization</li>
    <li>0x0003: failed obtained control authorization</li>
    <li>0x0004: failed realeased control authorization</li>
    <li>0x00C9: in IOC mode</li>
    </ul></td>
</tr>

</table>


#### CMD ID 0x01 Switch Flight Mode

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="2">CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>CMD Sequence Number</td>
</tr>

<tr>
  <td>1</td>
  <td>1</td>
  <td>
    0x01 :  return to home(RTH)</br>
    0x04 :  auto take off</br>
    0x06 :  auto landing</br>
  </td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code<ul>
    <li>0x0001: execution fail</li>
    <li>0x0002: start executing</li>
    </ul></td>
</tr>

</table>
 
>Note: auto takeoff only works when motor isn't running 

#### CMD ID 0x02 Request Switch Result
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>CMD Sequence Number</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code<ul>
    <li>0x0001: wrong CMD Sequence Number</li>
    <li>0x0003: switching in progress</li>
    <li>0x0004: switching failed</li>
    <li>0x0005: switching succeed</li>
    </ul></td>
</tr>

</table>

#### CMD ID 0x03 Movement Control

For more info about Movement Control, please refer to [Control mode byte part in Appendix](Appendix.md#control-mode-byte).

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="5">CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>---</td>
  <td>Control mode byte</td>
</tr>

<tr>
  <td>1</td>
  <td>4</td>
  <td>float32</td>
  <td>Roll or X-axis control value</td>
</tr>

<tr>
  <td>5</td>
  <td>4</td>
  <td>float32</td>
  <td>Pitch or Y-axis control value</td>
</tr>

<tr>
  <td>9</td>
  <td>4</td>
  <td>float32</td>
  <td>Throttle or Z-axis control value</td>
</tr>

<tr>
  <td>13</td>
  <td>4</td>
  <td>float32</td>
  <td>Yaw control value</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>

</table>

#### CMD ID 0x05 Arm/Disarm
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>


<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>
    0x01 :  Arm<br>
    0x00 :  Disarm
  </td>
</tr>

<tr>
 <td >ACK Val</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code <ul>
    <li>0x0000: Arm/Disarm Successfully</li>
    <li>0x0001: Please obtain control at first</li>
    <li>0x0002: Already Armed/Disarmed</li>
    <li>0x0003: Cannot disarm when drone not in the air</li>
    </ul></td>
</tr>

</table>

#### CMD ID 0x1A Gimbal Control in Rate
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="4">CMD Val</td>
  <td>0</td>
  <td>2</td>
  <td>int16_t</td>
  <td>Yaw in rate<br>unit 0.1º/s, input range[-1800,1800]</td>
</tr>
<tr>
  <td>2</td>
  <td>2</td>
  <td>int16_t</td>
  <td>Roll in rate<br>unit 0.1º/s, input range[-1800,1800]</td>
</tr>
<tr>
  <td>4</td>
  <td>2</td>
  <td>int16_t</td>
  <td>Pitch in rate<br>unit 0.1º/s, input range[-1800,1800]</td>
</tr>
<tr>
  <td>6</td>
  <td>1</td>
  <td><ul>
    <li>7 bit</li>
    <li>1 bit</li>
    </ul></td>
  <td><ul>
    <li>Reserved</li>
    <li>Enable Control</li>
    </ul></td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>
</table>
>Note: M600 with A3 only supports Ronin-MX/Zenmuse X5 Series/Zenmuse X3/Zenmuse XT when operating gimbal control in rate.

#### CMD ID 0x1B Gimbal Control in Position
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="5">CMD Val</td>
  <td>0</td>
  <td>2</td>
  <td>int16_t</td>
  <td>Yaw angle<br>unit 0.1º, input range [-3200,3200]</td>
</tr>
<tr>
  <td>2</td>
  <td>2</td>
  <td>int16_t</td>
  <td>Roll angle<br>unit 0.1º, input range [-350,350]</td>
</tr>
<tr>
  <td>4</td>
  <td>2</td>
  <td>int16_t</td>
  <td>Pitch angle<br>unit 0.1º, input range [-900,300]</td>
</tr>
<tr>
  <td>6</td>
  <td>1</td>
  <td>---</td>
  <td>control flag byte<ul>
    <li>bit 0: mode flag bit</li>
        <ul>0 :  Incremental control, the angle reference is the current Gimbal location</ul>
        <ul>1 :  Absolute control, the angle reference is related to configuration in DJI Go App</ul>
    <li>bit 1: Yaw invaild bit 
        <ul>0 :  Gimbal will follow the command in Yaw </ul>
        <ul>1 :  Gimbal will maintain position in Yaw  </ul>
    <li>bit 2: Roll invaild bit, the same as bit[1]</li>
    <li>bit 3: Pitch invaild bit, the same as bit[1]</li>
    <li>bit [4:7]: reserved, set to be 0</li>
    </ul></td>
<tr>
  <td>7</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>Command completion time<br>unit 0.1s, for example 20 means gimbal will reach the commended postition in 2 seconds<br>rotate rate beyond 400º/s is not recommand</td>
</tr>
</tr>

<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>
</table>

**The relationship bewteen the angle reference in absolute control mode and gimbal mode configuration in DJI Go App**  

<table>
  <tr>
    <th>Gimbal Mode</th>
    <th>Roll</th>
    <th>Pitch</th>
    <th>Yaw</th>
    <th>Gimbal Follow UAV's Head</th>
  </tr>
  <tr>
    <td>Follow</td>
    <td>Ground</td>
    <td>Ground</td>
    <td>Body</td>
    <td align="center">Y</td>
  </tr>
  <tr>
    <td>FPV</td>
    <td>N/A</td>
    <td>Ground</td>
    <td>N/A</td>
    <td align="center">Y</td>
  </tr>
  <tr>
    <td>Free</td>
    <td>Ground</td>
    <td>Ground</td>
    <td>Ground</td>
    <td align="center">N</td>
  </tr>
</table>

**Note: Rotating 90 degree in `pitch` direction will cause gimbal lock problem, in which the value of `roll` and `yaw` are not reliable.**

>Note: M600 with A3 only supports Ronin-MX/Zenmuse X5 Series/Zenmuse X3/Zenmuse XT when operating gimbal control in angle.

#### CMD ID 0x20 Take Photo
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr> 
  <td>CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>arbitrary number</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>
</table>

>Note: M600 with A3 only supports Zenmuse Series when operating camera control.

#### CMD ID 0x21 Start Video Recording

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr> 
  <td>CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>arbitrary number</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>

</table>
>Note: M600 with A3 only supports Zenmuse Series when operating camera control.

#### CMD ID 0x22 Stop Video Recording

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr> 
  <td>CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>arbitrary number</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>
</table>
>Note: M600 with A3 only supports Zenmuse Series when operating camera control.

### CMD Set 0x02 Push Data CMD Set

#### CMD ID 0x00 Flight Data

The flight data from the  Autopilot can be configured by the DJI  assistant software.
More info about Flight Data, please refer to [Flight Data part in Appendix](Appendix.md#flight-data) .
<table>
<tr>
  <th>Data Type</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="13">CMD Val</td>
  <td>2</td>
  <td>item presence byte, Bit with value 1 means this flight data contains corresponding data item<br>M100:<ul>
    <li>bit 0: flag of time stamp</li>
    <li>bit 1: flag of attitude quaternion</li>
    <li>bit 2: flag of linear acceleration</li>
    <li>bit 3: flag of linear velocity</li>
    <li>bit 4: flag of angular velocity</li>
    <li>bit 5: flag of GPS location, altitude and healthiness</li>
    <li>bit 6: flag of magnetometer</li>
    <li>bit 7: flag of remote controller data</li>
    <li>bit 8: flag of roll, pitch and yaw of gimbal </li>
    <li>bit 9: flag of flight status</li>
    <li>bit 10: flag of battery info</li>
    <li>bit 11: flag of control device</li>
    <li>bit [12:15]: reserved</li>
    </ul>
    A3:<ul>
    <li>bit 0: flag of time stamp</li>
    <li>bit 1: flag of attitude quaternion</li>
    <li>bit 2: flag of linear acceleration</li>
    <li>bit 3: flag of linear velocity</li>
    <li>bit 4: flag of angular velocity</li>
    <li>bit 5: flag of GPS location, altitude and healthiness</li>
    <li>bit 6: flag of GPS detailed information</li>
    <li>bit 7: flag of RTK detailed information</li>
    <li>bit 8: flag of magnetometer</li>
    <li>bit 9: flag of remote controller data</li>
    <li>bit 10: flag of roll, pitch and yaw of gimbal </li>
    <li>bit 11: flag of flight status</li>
    <li>bit 12: flag of battery info</li>
    <li>bit 13: flag of control device</li>
    <li>bit [14:15]: reserved</li>
  </td>
</tr>

<tr>
  <td>9</td>
  <td>Time stamp</td>
</tr>

<tr>
  <td>16</td>
  <td>Attitude quaternion</td>
</tr>

<tr>
  <td>12</td>
  <td>Linear acceleration</td>
</tr>

<tr>
  <td>13</td>
  <td>Linear velocity</td>
</tr>

<tr>
  <td>12</td>
  <td>Angular velocity</td>
</tr>

<tr>
  <td>25</td>
  <td>GPS position, altitude, height and healthiness</td>
</tr>

<tr>
  <td>68</td>
  <td>GPS detailed information (A3 only)</td>
</tr>

<tr>
  <td>74</td>
  <td>RTK detailed information (A3 only)</td>
</tr>

<tr>
  <td>6</td>
  <td>Magnetometer data</td>
</tr>

<tr>
  <td>12</td>
  <td>Remote controller channels</td>
</tr>

<tr>
  <td>13</td>
  <td>Roll, pitch and yaw of Gimbal </td>
</tr>

<tr>
  <td>1</td>
  <td>Flight status</td>
</tr>

<tr>
  <td>1</td>
  <td>Battery percentage</td>
</tr>

<tr>
  <td>2</td>
  <td>Control device</td>
</tr>

</table>

>Note: The actual offset of data item in the flight data should be calculated by the flags of 'item presence byte'.
 
#### CMD ID 0x01 Lost of Flight Control
Onboard Device has the lowerest control priority. Its control authorization can be taken over by remote controller and Mobile Device. Once the flight control is lost from the Onboard Device, a push data will be sent by the  Autopilot.

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td>CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>Fixed value, set to be 0x04</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>

</table>

#### CMD ID 0x03 Ground Station State
For more info about Ground Station, please refer to [Ground Station Protocol](GroundStationProtocol.md).    
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>


<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>6</td>
  <td>Ground Station State package</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>
</table>
#### CMD ID 0x04 Waypoint Event
For more info about Ground Station, please refer to [Ground Station Protocol](GroundStationProtocol.md).    
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>


<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>6</td>
  <td>Waypoint Event package</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>
</table>
### CMD Set 0x03 Ground Station CMD Set

For more info about Ground Station, please refer to [Ground Station Protocol](GroundStationProtocol.md).  

#### CMD ID 0x10 Upload waypoint info
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>*</td>
  <td>waypoint_mission_info_comm_t</td>
  <td>waypoint mission information</td>
</tr>
<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>Return Code:<ul>
    <li>0x00: No error </li>
    <li>0xD1: need to obtain control authorization</li>
    <li>0xD4: uav has been in waypoint</li>
    <li>0xC8: not support waypoint</li>
    <li>0xE0: waypoint parameter is wrong, out of range</li>
  </ul></td>
</tr>
</table>

#### CMD ID 0x11 Upload waypoint data
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="2">CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>waypoint index</td>
</tr>
<tr>
  <td>1</td>
  <td>*</td>
  <td>waypoint_comm_t</td>
  <td>waypoint information</td>
</tr>
<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>Return Code: <ul>
    <li>0x00: No error </li>
    <li>0xD4: uav has been in waypoint</li>
    <li>0xDD: pass limit area</li>
    <li>0xE1: data invaild</li>
    <li>0xE2: trace too lang(15km)</li>
    <li>0xE4: index over range</li>
    <li>0xE5: waypoint too close</li>
    <li>0xE6: waypoint too far</li>
    <li>0xE7: 'damping_dis' check faild</li>
    <li>0xE8: waypoint action data invaild</li>
    <li>0xE9: missing remaining waypoint</li>
    <li>0xEA: waypoint info not upload </li>
  </ul></td>
  
</tr>
</table>
#### CMD ID 0x12 Start/Stop waypoint

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>
    0x01 :  Stop waypoint<br>
    0x00 :  Start waypoint
  </td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>1</td>
  <td>Return Code:<ul>
    <li>0x00: no error </li>
    <li>0xC0: waypoint altitude too high</li>
    <li>0xC7: waypoint out of radius limited</li>
    <li>0xD7: followme is running</li>
    <li>0xD8: bad GPS</li>
    <li>0xD9: low battery capacity</li>
    <li>0xDA: UAV is not in air</li>
    <li>0xDE: bad GPS, home position is not recorded</li>
    <li>0xE3: total waypoints too long(15km)</li>
    <li>0xEA: waypoint info not upload</li>
    <li>0xEB: waypoint data not upload</li>
    <li>0xEC: request is running</li>
    <li>0xF4: wrong CMD Val </li>
  </ul></td>
</tr>

</table>
#### CMD ID 0x13 Pause/Resume waypoint

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>
    0x01 :  Resume waypoint<br>
    0x00 :  Pause waypoint
  </td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>1</td>
  <td>Return Code:<ul>
    <li>0x00: no error</li>
    <li>0xED: waypoint is not running</li>
    <li>0xF4: wrong CMD Val </li>
  </ul></td>
</tr>
</table>

#### CMD ID 0x14 Download waypoint info
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>arbitrary num</td>
</tr>
<tr>
  <td rowspan="2">ACK Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>Return Code:<ul>
    <li>0x00: no error </li>
    <li>0xEA: waypoint info not upload</li>
  </ul></td>
</tr>
<tr>
  <td>1</td>
  <td>*</td>
  <td>waypoint_mission_info_comm_t</td>
  <td>waypoint mission information</td>
</tr>
</table>
#### CMD ID 0x15 Download index waypoint
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>waypoint index</td>
</tr>
<tr>
  <td rowspan="3">ACK Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>Return Code:<ul>
    <li>0x00: no error </li>
    <li>0xEB: waypoint data not upload</li>
  </ul></td>
</tr>
<tr>
  <td>2</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>waypoint index</td>
</tr>
<tr>
  <td>3</td>
  <td>*</td>
  <td>waypoint_commt_t</td>
  <td>waypoint information</td>
</tr>
</table>
#### CMD ID 0x16 Set idle speed

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>*</td>
  <td>float32</td>
  <td>idle speed</td>
</tr>
<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>Return Code:<ul>
    <li>0x00: no error </li>
    <li>0xD1: need to obtain control authorization</li>
    <li>0xEA: waypoint info not upload </li>
    <li>0xEE: idle speed invalid</li>
  </ul></td>
</tr>
</table>
#### CMD ID 0x17 Get idle speed

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>arbitrary num</td>
</tr>
<tr>
  <td rowspan="2">ACK Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>Return Code:<ul>
    <li>0x00: no error  </li>
    <li>0xD1: need to obtain control authorization</li>
    <li>0xEA: waypoint info not upload</li>
    </ul></td>
</tr>
<tr>
  <td>1</td>
  <td>2</td>
  <td>float32</td>
  <td>idle speed</td>
</tr>
</table>
#### CMD ID 0x20 Start hotpoint
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>*</td>
  <td>hotpoint_mission_setting_t</td>
  <td>hotpoint settings</td>
</tr>
<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>Return Code:<ul>
    <li>0x00: no error </li>
    <li>0xA2: invaild float value</li>
    <li>0xA3: invaild lati and lonti</li>
    <li>0xA6: unknown direction</li>
    <li>0xC0: altitude too high</li>
    <li>0xC1: altitude too low</li>
    <li>0xC2: radius out of range(5m~500m)</li>
    <li>0xC3: speed too large</li>
    <li>0xC4: invaild start position</li>
    <li>0xC5: invaild yaw mode</li>
    <li>0xC7: radius limited</li>
    <li>0xC8: not support</li>
    <li>0xC9: uav is too far from hotpoint</li>
    <li>0xD3: hotpoint is not inited</li>
    <li>0xD5: hotpoint is running</li>
    <li>0xD6: uav cannot get to hotpoint</li>
    <li>0xD7: waypoint or followme is running</li>
    <li>0xD8: bad GPS</li>
    <li>0xD9: low battery capacity</li>
    <li>0xDA: UAV is not in air</li>
    <li>0xDC: uav is taking off, landing or going home</li>
    <li>0xDD: pass limit area</li>
    <li>0xDE: bad GPS, home position is not recorded</li>
    </ul></td>
</tr>
</table>

#### CMD ID 0x21 Stop hotpoint

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>arbitrary num</td>
</tr>
<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>Return Code:<ul>
    <li>0x00: no error</li>
    <li>0xD1: need to obtain control authorization</li>
    <li>0xD5: hotpoint is not running</li>
  </ul></td>
</tr>
</table>
#### CMD ID 0x22 Pause/Resume hotpoint

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>
    0x01 :  Resume hotpoint<br>
    0x00 :  Pause hotpoint
  </td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>1</td>
  <td>Return Code:<ul>
    <li>0x00: no error</li>
    <li>0xD5: hotpoint is not running</li>
    <li>0xA9: hotpoint has pasued</li>
    <li>0xAA: hotpoint does not pasue </li>
  </ul></td>
</tr>
</table>

#### CMD ID 0x23 Set idle speed

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="2 ">CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>is clockwise</td>
</tr>
<tr>
  <td>1</td>
  <td>4</td>
  <td>float32</td>
  <td>idle speed</td>
</tr>
<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>
</table>
#### CMD ID 0x24 Set radius
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td>CMD Val</td>
  <td>0</td>
  <td>4</td>
  <td>float32</td>
  <td>radius</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>
</table>

#### CMD ID 0x25 Reset yaw
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td>CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>arbitrary num</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>
</table>
#### CMD ID 0x26 Download hotpoint task info

<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td>CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>arbitrary num</td>
</tr>

<tr>
  <td rowspan="2">ACK Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>Return Code:<ul>
    <li>0x00: no error</li>
    <li>0xD5: hotpoint is not running</li>
  </ul></td>
</tr>
<tr>
  <td>1</td>
  <td>*</td>
  <td>hotpoint_mission_setting_t</td>
  <td>hotpoint settings</td>
</tr>
</table>

#### CMD ID 0x27 Enable auto radiu mode

<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Length(byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="2">CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>enable(1:start, 0:stop)</td>
</tr>

<tr>
  <td>1</td>
  <td>1</td>
  <td>int8_t</td>
  <td>rate(rate of radiu change)</td>
</tr>
<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>Return Code:<ul>
    <li>0x00: Success</li>
    <li>other error code</li>
  </ul></td>
</tr>
</table>

#### CMD ID 0x30 Start follow me
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td>CMD Val</td>
  <td>0</td>
  <td>*</td>
  <td>follow_me_mission_setting_t</td>
  <td>follow me settings</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>Return Code:<ul>
    <li>0x00: no error </li>
    <li>0xB0: target is too far from uav (over 20000m)</li>
    <li>0xC1: uav is too low</li>
    <li>0xC7: distance between target and uav out of radius</li>
    <li>0xC8: not support follow me</li>
    <li>0xD1: need to obtain control authorization</li>
    <li>0xD3: follow me is not inited</li>
    <li>0xD5: follow me is running</li>
    <li>0xD8: bad GPS</li>
    <li>0xD9: low battery capacity</li>
    <li>0xDA: UAV is not inair</li>
    <li>0xDE: bad GPS, home position is not recorded</li>
  </ul></td>
</tr>
</table>
#### CMD ID 0x31 Stop follow me
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td>CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>arbitrary num</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>Return Code:<ul>
    <li>0x00: no error </li>
    <li>0xD1: need to obtain control authorization</li>
    <li>0xD4: follow me is not running</li>
  </ul></td>
</tr>
</table>
#### CMD ID 0x32 Pause/Resume follow me
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>
    0x01 :  Resume follow me<br>
    0x00 :  Pause follow me
  </td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>1</td>
  <td>Return Code:<ul>
    <li>0x00: no error </li>
    <li>0xD4: follow me is not running</li>
  </ul></td>
</tr>
</table>

#### CMD ID 0x33 Set target pos info
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td>CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>uint8_t</td>
  <td>arbitrary num</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>*</td>
  <td>cmd_mission_follow_target_info</td>
  <td>target information</td>
</tr>
</table>
### CMD Set 0x04 Sync signal CMD Set
#### CMD ID 0x00 Set synchronization signal frequency
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td>CMD Val</td>
  <td>0</td>
  <td>4</td>
  <td>uint32_t</td>
  <td>0 means that a single sync signal output, other value indicates the frequency of sync signal output, unit hz</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>
</table>
### Set 0x05 Virtual RC CMD set

For more info about virual RC, please refer to [Virtual RC Protocol](VirtualRCProtocol.md).  

#### CMD ID 0x00 Virual RC request
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td>CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>
    bit 0: <ul>1 Request open<br>0 Request close</ul>
    bit 1: <ul>1 When disconnected automatically switches to real remote control<br> 0 No switches，execute disconnect program</ul>
    bit 2:7 <ul> reversed</ul></td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>
</table>
#### CMD ID 0x01 Virual RC data
<table>
<tr>
  <th>Data Type</th>
  <th>Offset (byte)</th>
  <th>Size (byte)</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="1">CMD Val</td>
  <td>0</td>
  <td>4 * 16</td>
  <td>uint32_t[16]</td>
  <td>data of sixteen channels，range[1024-660， 1024+660]</td>
</tr>
<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>
</table>
[0]: Appendix.md
