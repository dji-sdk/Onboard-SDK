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
  <th>Byte Index</th>
  <th>Size(bit)</th>
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
    <li>0：CMD</li>
    <li>1：ACK</li>
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
  <td>len of padding data used by the whole frame encryption</td>
</tr>

<tr>
  <td>ENC</td>
  <td>3</td>
  <td>encryption type<ul>
    <li>0：no encryption</li>
    <li>1：AES encryption</li>
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
  <td>26</td>
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

### Frame Type
There are two types of frames.

|Frame Type|Data Type|Transmission Direction|Content|
|------------|:----------:|----------:|---------------|
|CMD frame|CMD frame data|Onboard Device <=> N1 Autopilot|flight control related CMDs|
|ACK frame|ACK frame data|N1 Autopilot <=> Onboard Device|ACK related data|

#### CMD frame data
```
|<-------CMD frame data------->|
|CMD SET|CMD ID|CMD VAL|
```

|Data Field|Byte Index|Size(byte)|
|----|--------|-----------------|
|CMD SET|0|1|
|CMD ID|1|1|
|CMD VAL|2|vary by CMDs|

#### ACK frame data

```
|<-ACK frame data->|
|ACK VAL|
```

|Data Field|Byte Index|Size(byte)|
|----|--------|-----------------|
|ACK VAL|0|vary by ACKs|

SEQ field in ACK frame is the same as the one in corresponding CMD frame. Developers can use the SEQ field of the ACK frame to match the corresponding CMD frame. 

---

## Protocal Transmission Mechanism

### Session

The session mechanism has been used in order to prevent the exceptions such as package loss. 3 kinds of session have been introduced:

Note: Type 1 and Type 2 can ONLY be applied to the CMDs which have ACKs.

|Type|SESSION|Description|
|------------|-------|-----------|
|0|0|Sender doesn't need ACKs.|
|1|1|Sender needs ACKs but can be tolerated.|
|2|2-31|Sender needs ACKs.*|

*Since type 2 is designed to be reliable. Developers should implement the package loss & resending mechinism based on the current SEQ and SESSION fields. When package loss is found from the sender, the sender can send the CMD frame again with the same SEQ and SESSION lost previously to retrieve back the ACK frame.

---

## CMD Set & CMD ID

### CMD Set

The CMDs have three different sets  

|CMD Set|CMD ID|Description|
|--------|-----------|--------------|
|Activation|0x00|activation related CMDs|
|Flight Control|0x01|flight control related CMDs|
|Push Data|0x02|flight data related CMDs|

### CMD ID

Each CMD Set contains some CMD IDs for different operations.

The execution of different CMDs needs an corresponding Authorization Level. A CMD will not be executed when the current Authorization Level granted is lower.

|Levels|Description|
|:--------:|----------|
|0|Activation related|
|1|Gimbal and Camera control related|
|2|Flight Control related|

*Note: The Authorization Level of the N1 Autopilot can be changed by the 'Activation' CMD. The init level is set to be 0.*

**Function Index**
<table>
<tr>
  <th>CMD Set</th>
  <th>CMD ID</th>
  <th>Function</th>
  <th>Level Required</th>
</tr>
  <td rowspan="2">0x00<br>Activation CMD Set</td>
  <td>0x00</td>
  <td>Get Protocal Version</td>
  <td>0</td>
</tr>

</tr>
  <td>0x01</td>
  <td>Activation</td>
  <td>0</td>
</tr>

<tr>
  <td rowspan="9">0x01<br>Control CMD Set</td>
  <td>0x00</td>
  <td>Request/Release Control Authorization</td>
  <td>2</td>
</tr>

<tr>
  <td>0x01</td>
  <td>Switch Flight Mode</td>
  <td>2</td>
</tr>
<tr>
  <td>0x02</td>
  <td>Request switch result</td>
  <td>2</td>
</tr>

<tr>
  <td>0x03</td>
  <td>Movement Control</td>
  <td>2</td>
</tr>

<tr>
  <td>0x1A</td>
  <td>Gimbal Control in Rate</td>
  <td>1</td>
</tr>

<tr>
  <td>0x1B</td>
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
  <td>Start Recording Video</td>
  <td>1</td>
</tr>

<tr>
  <td>0x22</td>
  <td>Stop Recording Video</td>
  <td>1</td>
</tr>

<tr>
  <td rowspan="2">0x02<br>Push Data CMD Set</td>
  <td>0x00</td>
  <td>Push Data</td>
  <td>0</td>
</tr>

<tr>
  <td>0x01</td>
  <td>Lost of Flight Control</td>
  <td>0</td>
</tr>
</table>

## CMD Val & ACK Val

### Activation CMD Set: 0x00 

#### CMD ID 0x00: Get Protocal Version

<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
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
    <li>0x0000：N1 Autopilot is activated</li>
    <li>0xFF01：N1 Autopilot is NOT activated</li>
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
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="4">CMD Val</td>
  <td>0</td>
  <td>4</td>
  <td>app_id, app unique identifer</td>
</tr>

<tr>
  <td>4</td>
  <td>4</td>
  <td>api_level, authorization level</td>
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
  <td>ACK Val</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code：<ul>
    <li>0x0000：Success</li>
    <li>0x0001：Invalid parameters</li>
    <li>0x0002：Cannot recognize the encrypted package</li>
    <li>0x0003：New APP ID, attempt to activate</li>
    <li>0x0004：No response from DJI GO APP </li>
    <li>0x0005：No Internet from DJI GO APP </li>
    <li>0x0006：Server rejected</li>
    <li>0x0007：Authorization level insufficient</li>
    <li>0x0008：Wrong SDK version</li>
    </ul></td>
</tr>

</table>

### CMD Set 0x01 Control CMDs

#### CMD ID 0x00: Request/Release Control Authorization

<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>
    0x01：request to obtain control authorization</br>
    0x00：request to release control authorization</br>
  </td>
</tr>

<tr>
 <td >ACK Val</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code <ul>
    <li>0x0001：successfully released control authorization</li>
    <li>0x0002：successfully obtained control authorization</li>
    <li>0x0003：in progress</li>
    </ul></td>
</tr>

</table>


#### CMD ID 0x01 Switch Flight Mode

<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
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
    0x01 ： return to home(RTH)</br>
    0x04 ： auto take off</br>
    0x06 ： auto landing</br>
  </td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code<ul>
    <li>0x0001：execution fail</li>
    <li>0x0002：start executing</li>
    </ul></td>
</tr>

</table>

#### CMD ID 0x02 Request Switch Result
<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
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
    <li>0x0001：wrong CMD Sequence Number</li>
    <li>0x0003：switching in progress</li>
    <li>0x0004：switching failed</li>
    <li>0x0005：switching succeed</li>
    </ul></td>
</tr>

</table>

#### CMD ID 0x03 Movement Control*
<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="5">CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>Control mode byte</td>
</tr>

<tr>
  <td>1</td>
  <td>4</td>
  <td>Roll or X-axis control value</td>
</tr>

<tr>
  <td>5</td>
  <td>4</td>
  <td>Pitch or Y-axis control value</td>
</tr>

<tr>
  <td>9</td>
  <td>4</td>
  <td>Throttle or Z-axis control value</td>
</tr>

<tr>
  <td>13</td>
  <td>4</td>
  <td>Yaw control value</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>

</table>

**For more please refer to [Additional Explanation for Flight Control][0]*  

#### CMD ID 0x1A Gimbal Control in Rate
<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="4">CMD Val</td>
  <td>0</td>
  <td>2</td>
  <td>Yaw in rate</td>
</tr>
<tr>
  <td>2</td>
  <td>2</td>
  <td>Roll in rate</td>
</tr>
<tr>
  <td>4</td>
  <td>2</td>
  <td>Pitch in rate</td>
</tr>
<tr>
  <td>6</td>
  <td>1</td>
  <td>Fixed value, 0x80</td>
</tr>

<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>

</table>

**Data Type Description**

<table>
<tr>
  <th>Data Name</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td>Yaw in rate</td>
  <td>int16_t</td>
  <td>unit 0.1º/s, input range[-1800,1800]</td>
</tr>

<tr>
  <td>Roll in rate</td>
  <td>int16_t</td>
  <td>unit 0.1º/s, input range[-1800,1800]</td>
</tr>

<tr>
  <td>Pitch in rate</td>
  <td>int16_t</td>
  <td>unit 0.1º/s, input range[-1800,1800]</td>
</tr>
</table>

#### CMD ID 0x1B Gimbal Control in Position
<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="5">CMD Val</td>
  <td>0</td>
  <td>2</td>
  <td>Yaw angle</td>
</tr>
<tr>
  <td>2</td>
  <td>2</td>
  <td>Roll angle</td>
</tr>
<tr>
  <td>4</td>
  <td>2</td>
  <td>Pitch angle</td>
</tr>
<tr>
  <td>6</td>
  <td>1</td>
  <td>control flag byte<ul>
    <li>bit 0：mode flag bit</li>
        <ul>0 ： Incremental control, the angle reference is the current Gimbal location</ul>
        <ul>1 ： Absolute control, the angle reference is coordinate system to the Northeast</ul>
    <li>bit 1：Yaw invaild bit 
        <ul>0 ： Gimble will follow the command in Yaw </ul>
        <ul>1 ： Gimble will maintain position in Yaw  </ul>
    <li>bit 2：Roll invaild bit, the same as bit[1]</li>
    <li>bit 3：Pitch invaild bit, the same as bit[1]</li>
    <li>bit [4:7]：reserved, set to be 0</li>
    </ul></td>
    
    TODO:Related to Gimble mode setting in APP

<tr>
  <td>7</td>
  <td>1</td>
  <td>Command completion time</td>
</tr>
</tr>

<tr>
  <td>ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>
</table>

**Data Type Description**

<table>
<tr>
  <th>Data Name</th>
  <th>Data Type</th>
  <th>Description</th>
</tr>

<tr>
  <td>Yaw angle</td>
  <td>int16_t</td>
  <td>unit 0.1º, input range [-3200,3200]</td>
</tr>

<tr>
  <td>Roll angle</td>
  <td>int16_t</td>
  <td>unit 0.1º, input range [-350,350]</td>
</tr>

<tr>
  <td>Pitch angle</td>
  <td>int16_t</td>
  <td>unit 0.1º, input range [-900,300]</td>
</tr>

<tr>
  <td>Command completion time</td>
  <td>uint8_t</td>
  <td>unit 0.1s, for example 20 means gimble will reach the commended postition in 2 seconds<br>rotate rate beyond 400º/s is not recommand</td>
</tr>
</table>

#### CMD ID 0x20 Take Photo
<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
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
#### CMD ID 0x21 Start Video Recording

<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
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
#### CMD ID 0x22 Stop Video Recording

<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
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
### CMD Set 0x02 Push Data CMD Set

#### CMD ID 0x00 Push Data

The push data from the N1 Autopilot can be configured by the DJI N1 assistant software.

<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)*</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="13">CMD Val</td>
  <td>0</td>
  <td>2</td>
  <td>item presence byte, Bit with value 1 means this push data contains corresponding data item<ul>
    <li>bit 0：flag of time stamp</li>
    <li>bit 1：flag of attitude quaternion</li>
    <li>bit 2：flag of linear acceleration</li>
    <li>bit 3：flag of linear velocity</li>
    <li>bit 4：flag of angular velocity</li>
    <li>bit 5：flag of GPS location, altitude and healthiness</li>
    <li>bit 6：flag of magnetometer</li>
    <li>bit 7：flag of remote controller data</li>
    <li>bit 8：flag of roll,pitch and yaw of gimbal </li>
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
  <td>Roll, pitch and yaw of Gimbal </td>
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

**The offset in the above table is with the assumption that all data items exist. The actual offset of data item in the push data should be calculated by the flags of 'item presence byte'. *
 
**Data Item Index**

<table>
<tr>
  <td>Item Name</td>
  <td>Variables</td>
  <td>Data Type</td>
  <td>Description</td>
  <td>Unit</td>
  <td>Default Frequency</td>
</tr>

<tr>
  <td>Time Stamp</td>
  <td>time</td>
  <td>uint32_t</td>
  <td>time stamp</td>
  <td>1/600s</td>
  <td>100Hz</td>
</tr>
<tr>
  <td rowspan="4">Quarternion</td>
  <td>q0</td>
  <td>float32</td>
  <td rowspan="4">Attitude quaternion<br>From ground frame to body frame*</td>
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
  <td>Status byte of linear velocity<ul>
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
  <td rowspan="3">º/s</td>
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
  <td>Height relatively to ground**</td>
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
  <td>roll channel</td>
  <td rowspan="6">---</td>
  <td rowspan="6">50Hz</td>
</tr>
<tr>
  <td>pitch</td>
  <td>int16_t</td>
  <td>pitch channel</td>
</tr>
<tr>
  <td>yaw</td>
  <td>int16_t</td>
  <td>yaw channel</td>
</tr>
<tr>
  <td>throttle</td>
  <td>int16_t</td>
  <td>throttle channel</td>
</tr>
<tr>
  <td>mode</td>
  <td>int16_t</td>
  <td>mode channel</td>
</tr>
<tr>
  <td>gear</td>
  <td>int16_t</td>
  <td>gear channel</td>
</tr>

<tr>
  <td rowspan="3">Gimbal</td>
  <td>roll</td>
  <td>float32</td>
  <td rowspan="3">roll, pitch and yaw of ground frame</td>
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
  <td>Flight status</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>Flight status</td>
  <td>---</td>
  <td>10Hz</td>
</tr>

<tr>
  <td>Battery</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>Battery percentage</td>
  <td>%</td>
  <td>1Hz</td>
</tr>

<tr>
  <td>Source of Control</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>Control device<ul>
     <li>bit 0:2 ：Control device</li>
     <ul>0b000 ：Remote Controller</ul>
     <ul>0b001 ：Mobile Device</ul>
     <ul>0b010 ：Onboard Device</ul>
     <li>bit 3 ：Flag of Onboard Device control authorization request signature </li>
     <ul>0：No request</ul>
     <ul>1：Been requested</ul>
     <li>bit 4:7 ：reserved</li>
  </ul></td>
  <td>---</td>
  <td>0Hz</td>
</tr>

</table>
**For more please refer to [Additional Explanation for Flight Control][0]*  
***Height is the fusion result of barometer, IMU and ultrasonic sensor. If the flight plantform has no ultrasonic sensor, or its distance to the ground is higher than 3 meters, the height is supported by barometer and IMU only. Since the barometer is inaccurate being used indoor, height is unreliable in this case.*

#### CMD ID 0x01 Lost of Flight Control
Onboard Device has the lowerest control priority. Its control authorization can be taken over by remote controller and Mobile Device. Once the flight control is lost from the Onboard Device, a push data will be sent by the N1 Autopilot.

<table>
<tr>
  <th>Data Type</th>
  <th>Offset(byte)</th>
  <th>Size(byte)</th>
  <th>Description</th>
</tr>

<tr>
  <td >CMD Val</td>
  <td>0</td>
  <td>1</td>
  <td>Fixed value, set to be 0x04</td>
</tr>

<tr>
  <td >ACK Val</td>
  <td>---</td>
  <td>---</td>
  <td>N/A</td>
</tr>

</table>

[0]: FlightControlExplain.md
Note: The 'Data Transparent Transmission' is NOT included in this document.
