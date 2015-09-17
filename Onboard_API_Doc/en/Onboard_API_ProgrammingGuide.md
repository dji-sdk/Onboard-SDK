# DJI Onboard API Programming Guide 

*If you come across any mistakes or bugs in this tutorial, please let us know using a Github issue, a post on the DJI forum, or email. Please feel free to send us Github pull request and help us fix any issues. However, all pull requests related to document must follow the [document style](https://github.com/dji-sdk/Onboard-SDK/issues/19)*

---

DJI offers two powerful APIs for developers to create custom applications: the Mobile Device API, and the UAV Onboard API. The Mobile Device API is part of the DJI Mobile SDK and lets developers monitor and control the UAV from a mobile device running iOS or Android that is connected to the remote controller. The UAV Onboard API allows developers to monitor and control the UAV from any wired connected system through the serial (UART) interface. 

This documentation discusses the protocol programming when communicating with MATRICE 100. We recommend developers follow quick start first to run our sample code before reading this programming guide.

**There is not any content about transparent transport.**
## Protocol Description

### Protocol Frame Format

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



#### Protocol Data Field Explanation

All serial packages exchanged between MATRICE 100 and the onboard device can be classified into three types

|Packages Types|Transmission direction|Transmission Content|
|------------|:----------:|---------------|
|command package|Onboard Device==>MATRICE 100|control commands|
|ACK package|MATRICE 100==>Onboard Device|control results|
|message package|MATRICE 100==>Onboard Device|status|

##### Command Package

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


##### Message Package

```
|<-------Protocol Frame Data------->|
|COMMAND SET|COMMAND ID|COMMAND DATA|
```

|Data Field|Byte Index|Size(byte)|
|----|--------|-----------------|
|COMMAND SET|0|1|
|COMMAND ID|1|1|
|COMMAND DATA*|2|depends on the exact command|

*\*COMMAND DATA in Message Packageis able to be configured in N1-Assistant*

---

### 协议通信机制


#### Session

协议设计使用了会话机制，以保证命令数据和Return Data不会因为丢包而出现通信双方异常。通信双方在向对方发起通信会话时，可以根据需要通过设置协议的 SESSION 字段来选择会话方式。协议中设计了三种会话方式。

|Session Mode|SESSION|Description|
|------------|-------|-----------|
|Mode 1|0|Sender do not need acknowledgement.|
|Mode 2|1|Sender need acknowledgement, but can tolerate ACK package loss.|
|Mode 3|2-31|Sender wants to make sure the ACK is reliably sent.*|

*\*For these sessions, Receiver saves the sequence number in the command package and send an  ACK package upon receiving it. If ACK package loss happened, Sender may request Receiver again using the same command package with the same sequence number.*


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
  <td>api_level，Authorization Level</td>
</tr>

<tr>
  <td>8</td>
  <td>4</td>
  <td>fixed value，0x02030A00</td>
</tr>

<tr>
  <td>12</td>
  <td>32</td>
  <td>fixed string，"12345678901234567890123456789012"</td>
</tr>

<tr>
  <td>Return Data</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code：<ul>
    <li>0x0000：Success</li>
    <li>0x0001：Invalid parameters</li>
    <li>0x0002：Cannot recognize encrypted package</li>
    <li>0x0003：No activate，Attempt to activate</li>
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

#### Command ID 0x03 Movement Control\*
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

*\*被控量为平动或转动由模式标志字节决定，有关姿态控制的具体内容请参阅[飞行控制附加说明][0]*  

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
  <td>rate in Yaw</td>
</tr>
<tr>
  <td>2</td>
  <td>2</td>
  <td>rate in Roll</td>
</tr>
<tr>
  <td>4</td>
  <td>2</td>
  <td>rate in Pitch</td>
</tr>
<tr>
  <td>6</td>
  <td>1</td>
  <td>fixed value，0x80</td>
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
  <td>unit 0.1度/s，输入范围（[-1800,+1800]）</td>
</tr>

<tr>
  <td>rate in Roll</td>
  <td>int16_t</td>
  <td>unit 0.1度/s，输入范围[-1800,+1800]</td>
</tr>

<tr>
  <td>rate in Pitch</td>
  <td>int16_t</td>
  <td>unit 0.1度/s，输入范围[-1800,+1800]</td>
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
  <td>Yaw轴角度</td>
</tr>
<tr>
  <td>2</td>
  <td>2</td>
  <td>Roll轴角度</td>
</tr>
<tr>
  <td>4</td>
  <td>2</td>
  <td>Pitch轴角度</td>
</tr>
<tr>
  <td>6</td>
  <td>1</td>
  <td>属性控制字节<ul>
    <li>bit 0：控制模式选择位</li>
        <ul>0 ： 增量控制，角度基准为当前云台所处位置</ul>
        <ul>1 ： 绝对控制，角度基准为东北地坐标系</ul>
    <li>bit 1：Yaw轴命令控制失效位 
        <ul>0 ： 云台Yaw角度运动到命令位置 </ul>
        <ul>1 ： 云台Yaw将维持上一时刻状态 </ul>
    <li>bit 2：Roll轴命令控制失效位，同bit[1]描述</li>
    <li>bit 3：Pitch轴命令控制失效位，同bit[1]描述</li>
    <li>bit [4:7]：保留，必须为0</li>
    </ul></td>

<tr>
  <td>7</td>
  <td>1</td>
  <td>命令完成时间</td>
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
  <td>Yaw轴角度</td>
  <td>int16_t</td>
  <td>unit 0.1度，输入范围 [-3200~+3200]</td>
</tr>

<tr>
  <td>Roll轴角度</td>
  <td>int16_t</td>
  <td>unit 0.1度，输入范围 [-350~+350]</td>
</tr>

<tr>
  <td>Pitch轴角度</td>
  <td>int16_t</td>
  <td>unit 0.1度，输入范围 [-900~+300]</td>
</tr>

<tr>
  <td>命令完成时间</td>
  <td>uint8_t</td>
  <td>unit 0.1s，例如20代表云台在2s内匀速转动至命令位置<br>建议用户控制速度不超过400度/秒</td>
</tr>
</table>

#### 命令码 0x20 Take Photo

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>说明</th>
</tr>

<tr> 
  <td>请求数据</td>
  <td>0</td>
  <td>1</td>
  <td>任意值</td>
</tr>

<tr>
  <td>应答数据</td>
  <td>---</td>
  <td>---</td>
  <td>无应答数据</td>
</tr>
</table>
#### 命令码 0x21 Start Record

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>说明</th>
</tr>

<tr> 
  <td>请求数据</td>
  <td>0</td>
  <td>1</td>
  <td>任意值</td>
</tr>

<tr>
  <td>应答数据</td>
  <td>---</td>
  <td>---</td>
  <td>无应答数据</td>
</tr>

</table>
#### 命令码 0x22 Stop Record

<table>
<tr>
  <th>数据类型</th>
  <th>偏移（字节）</th>
  <th>大小（字节）</th>
  <th>说明</th>
</tr>

<tr> 
  <td>请求数据</td>
  <td>0</td>
  <td>1</td>
  <td>任意值</td>
</tr>

<tr>
  <td>应答数据</td>
  <td>---</td>
  <td>---</td>
  <td>无应答数据</td>
</tr>
</table>
### Command Set 0x02 Monitor Command Set

#### Command ID 0x00 Message Package

飞控外发的状态数据包可以通过 DJI N1 PC 调参软件配置。可以配置状态包是否发送及发送的频率。  

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
  <td>状态包存在标志位，标志位为 1 表示标准数据包中存在该状态包<ul>
    <li>bit 0：时间戳包存在标</li>
    <li>bit 1：姿态四元素包存在标志</li>
    <li>bit 2：Ground 坐标系下的加速度包存在标志</li>
    <li>bit 3：Ground 坐标系下的速度包存在标志</li>
    <li>bit 4：Body 坐标系的角速度包存在标志</li>
    <li>bit 5：GPS 位置、海拔（气压计数值）、相对地面高度、健康度包存在标志</li>
    <li>bit 6：磁感计数值包存在标志</li>
    <li>bit 7：遥控器通道值包存在标志</li>
    <li>bit 8：云台 roll、pitch、yaw 数据包存在标志</li>
    <li>bit 9：飞行状态包存在标志</li>
    <li>bit 10：剩余电池百分比包存在标志</li>
    <li>bit 11：控制设备包存在标志</li>
    <li>bit [12:15]：保留不用</li>
    </td>></ul>
</tr>

<tr>
  <td>2</td>
  <td>4</td>
  <td>时间戳</td>
</tr>

<tr>
  <td>6</td>
  <td>16</td>
  <td>姿态四元数</td>
</tr>

<tr>
  <td>22</td>
  <td>12</td>
  <td>Ground 坐标系下的加速度</td>
</tr>

<tr>
  <td>34</td>
  <td>12</td>
  <td>Ground 坐标系下的速度</td>
</tr>

<tr>
  <td>46</td>
  <td>12</td>
  <td>Body 坐标系的角速度</td>
</tr>

<tr>
  <td>58</td>
  <td>24</td>
  <td>GPS 位置, 海拔（气压计数值）, 相对地面高度</td>
</tr>

<tr>
  <td>82</td>
  <td>12</td>
  <td>磁感计数值</td>
</tr>

<tr>
  <td>94</td>
  <td>10</td>
  <td>遥控器通道值</td>
</tr>

<tr>
  <td>104</td>
  <td>12</td>
  <td>云台姿态</td>
</tr>

<tr>
  <td>116</td>
  <td>1</td>
  <td>飞行状态</td>
</tr>

<tr>
  <td>117</td>
  <td>1</td>
  <td>剩余电池百分比</td>
</tr>

<tr>
  <td>118</td>
  <td>1</td>
  <td>控制设备</td>
</tr>
</table>

*\*Offset(byte)：表格中Offset(byte)为Message Package中存在所有状态包的情况。*

 实际数据在Message Package中的偏移需要根据状态包存在标志位确定存在的状态包，然后根据各状态包大小计算出状态包的实际偏移大小。
 
**数据包中各个状态包的数据段含义**

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
  <td rowspan="4">姿态四元数</td>
  <td>q0</td>
  <td>float32</td>
  <td rowspan="4">姿态四元数<br>从 Ground 坐标系转到 Body 坐标系变换*</td>
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
  <td rowspan="3">加速度</td>
  <td>agx</td>
  <td>float32</td>
  <td rowspan="3">飞行器在 Ground 坐标系下加速度</td>
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
  <td rowspan="3">速度</td>
  <td>vgx</td>
  <td>float32</td>
  <td rowspan="3">飞行器在 Ground 坐标系下速度</td>
  <td rowspan="3">m/s</td>
  <td rowspan="3">100Hz</td>
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
  <td rowspan="3">角速度</td>
  <td>wx</td>
  <td>float32</td>
  <td rowspan="3">飞行器在 Body 坐标系下角速度 </td>
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
  <td rowspan="5">GPS及高度</td>
  <td>longti</td>
  <td>double</td>
  <td rowspan="2">GPS 位置</td>
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
  <td>海拔</td>
  <td>气压值</td>
</tr>
<tr>
  <td>height</td>
  <td>float32</td>
  <td>相对地面高度**</td>
  <td>m</td>
</tr>
<tr>
  <td>health_flag</td>
  <td>uint8_t</td>
  <td>GPS 健康度 </td>
  <td>0-5, 5 为最好</td>
</tr>

<tr>
  <td rowspan="3">磁感计</td>
  <td>mx</td>
  <td>int16_t</td>
  <td rowspan="3">磁感计数值</td>
  <td rowspan="3">磁感计数值</td>
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
  <td rowspan="6">遥控器*</td>
  <td>roll</td>
  <td>int16_t</td>
  <td>遥控通道 roll 数据</td>
  <td rowspan="6">---</td>
  <td rowspan="6">50Hz</td>
</tr>
<tr>
  <td>pitch</td>
  <td>int16_t</td>
  <td>遥控通道 pitch 数据</td>
</tr>
<tr>
  <td>yaw</td>
  <td>int16_t</td>
  <td>遥控通道 yaw 数据</td>
</tr>
<tr>
  <td>throttle</td>
  <td>int16_t</td>
  <td>遥控通道 throttle 数据</td>
</tr>
<tr>
  <td>mode</td>
  <td>int16_t</td>
  <td>遥控通道 mode 数据（模式选择开关）</td>
</tr>
<tr>
  <td>gear</td>
  <td>int16_t</td>
  <td>遥控通道 gear 数据（正面的圆形拨杆）</td>
</tr>

<tr>
  <td rowspan="3">云台姿态</td>
  <td>roll</td>
  <td>float32</td>
  <td rowspan="3">云台在Ground 坐标系下姿态</td>
  <td rowspan="3">度</td>
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
  <td>飞行状态*</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>飞行状态</td>
  <td>---</td>
  <td>10Hz</td>
</tr>

<tr>
  <td>电量</td>
  <td>battery</td>
  <td>uint8_t</td>
  <td>剩余电量百分比</td>
  <td>剩余电量百分比</td>
  <td>1Hz</td>
</tr>

<tr>
  <td>控制信号源</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>控制设备<ul>
     <li>0x00 ： 遥控器</li>
     <li>0x01 ： 移动设备</li>
     <li>0x02 ： 机载设备</li>
     </ul></td>
  <td>---</td>
  <td>0Hz</td>
</tr>
</table>
*\*Ground 坐标系、Body 坐标系、遥控器及飞行状态相关详细Description请参阅XXXXXX*  
*\*\*相对地面高度是超声波、气压计和IMU融合的结果。如果飞行器上没有安装Guidance，或者安装Guidance但是相对地面的距离超过3米，相对地面高度则由气压计气压计计算得出。由于室内无法准确获取气压值，此数据将不可靠。*


#### Command ID 0x01 Control Authority Change Notification
Control Authority Change Notification数据包会在机载控制权被夺去的时由飞控主动推送。  
机载设备的控制权优先级最低，其控制权可能在任何时候被夺去。

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
  <td>固定值，0x04</td>
</tr>


</table>


##Onboard API

